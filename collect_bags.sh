#!/bin/bash
# collect_bags.sh - bring recorded bags back to the LAPTOP repo, with resume.
#
# Complements `./fleet_ctl collect`, which rsyncs experiment_logs/ but is a
# blocking `rsync -az` with a 300 s timeout - fine for CSVs, wrong for
# multi-GB bags. This one is resumable, reports size and rate, and verifies
# the laptop copy with `ros2 bag info` before it claims success.
#
#   ./collect_bags.sh <run> ghost delta      # fetch one run
#   ./collect_bags.sh --list ghost delta     # what is on the drones
#   ./collect_bags.sh --verify <run>         # re-verify the local copy only
#
# WHERE BAGS LIVE, AND WHY NOT IN GIT
# -----------------------------------
#   bags/<run>/<drone>/            the rosbag2 directory  (GITIGNORED)
#   bags/<run>/<drone>.manifest.json                      (TRACKED in git)
#   bags/<run>/COLLECTION.md       transfer + verification record (TRACKED)
#
# A single 10 min two-drone `figures` run is ~2.9 GB measured, which does not
# belong in git. The manifests do: they are small, they pin the git SHAs of
# every source repo, the profile, the topic list and the UTC window, so a bag
# that has been archived elsewhere is still fully described by the repo.
# Standing rule: anything that must survive leaving the lab ends up in the
# laptop repo - the manifest satisfies that for the bag's provenance, and the
# bag itself is copied here too, just not committed.
#
# The drone copy is NEVER deleted automatically. Delete it by hand once you
# have confirmed the laptop copy verifies.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/credentials.env" 2>/dev/null
: "${JETSON_PASS:?credentials.env missing - copy credentials.env.example and fill it in}"

WS=/mnt/nova_ssd/workspaces/isaac_ros-dev
BAGROOT="$WS/experiment_logs/bags"
DEST="$HERE/bags"

declare -A IPS=( [delta]=192.168.0.40 [buckshee]=192.168.0.60 [ghost]=192.168.0.50 [thunderstrike]=192.168.0.70 )

drones_from(){ local out=(); for a in "$@"; do case "$a" in
    all) out=(delta buckshee ghost thunderstrike) ;;
    delta|buckshee|ghost|thunderstrike) out+=("$a") ;;
  esac; done
  [ ${#out[@]} -eq 0 ] && out=(ghost delta); printf '%s\n' "${out[@]}"; }

ssh_d(){ local d="$1"; shift
  sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o LogLevel=ERROR \
    -o ConnectTimeout=10 "$d@${IPS[$d]}" "$@"; }

if [ "${1:-}" = "--list" ]; then shift
  mapfile -t DR < <(drones_from "$@")
  for d in "${DR[@]}"; do echo "=== $d ==="
    ssh_d "$d" "du -sh $BAGROOT/*/ 2>/dev/null | sed 's|$BAGROOT/||'" | sed 's/^/  /'
  done; exit 0
fi

verify_local(){ local run="$1" rc=0
  # ROS setup.bash dereferences unset variables; `set -u` would abort the whole
  # function silently and make verification look like it produced nothing.
  set +u; source /opt/ros/humble/setup.bash 2>/dev/null; set -u
  for bd in "$DEST/$run"/*/; do
    [ -d "$bd" ] || continue
    local d; d=$(basename "$bd")
    if [ ! -f "$bd/metadata.yaml" ]; then
      echo "  $d: FAILED - no metadata.yaml (recording was killed, not SIGINTed)"; rc=1; continue; fi
    local info; info=$(ros2 bag info "$bd" 2>&1)
    if grep -q "Messages:" <<<"$info"; then
      local msgs dur size
      msgs=$(grep -oP 'Messages:\s+\K[0-9]+' <<<"$info" | head -1)
      dur=$(grep -oP 'Duration:\s+\K[0-9.]+' <<<"$info" | head -1)
      size=$(du -sh "$bd" | cut -f1)
      # a bag with a header but no data is the failure mode that cost a run
      if [ "${msgs:-0}" -eq 0 ]; then
        echo "  $d: FAILED - bag opens but contains 0 messages"; rc=1
      else
        echo "  $d: OK - $msgs msgs, ${dur}s, $size"
      fi
    else
      echo "  $d: FAILED - ros2 bag info could not read it"
      sed 's/^/       /' <<<"$info" | head -3; rc=1; fi
  done
  return $rc; }

if [ "${1:-}" = "--verify" ]; then shift
  run="${1:?usage: --verify <run>}"; echo "=== verify $run ==="; verify_local "$run"; exit $?; fi

RUN="${1:?usage: ./collect_bags.sh <run> [drones...]}"; shift
mapfile -t DR < <(drones_from "$@")
mkdir -p "$DEST/$RUN"

echo "=== collecting run '$RUN' from ${DR[*]} ==="
TOTAL_START=$(date +%s)
for d in "${DR[@]}"; do
  src="$BAGROOT/$RUN/$d"
  remote_sz=$(ssh_d "$d" "du -sb '$src' 2>/dev/null | cut -f1")
  if [ -z "$remote_sz" ] || [ "$remote_sz" = 0 ]; then
    echo "$d: nothing at $src"; continue; fi
  echo "--- $d: $(numfmt --to=iec "$remote_sz") to fetch ---"
  t0=$(date +%s)
  # --partial --append-verify: a dropped link resumes instead of restarting.
  # -h --info=progress2: human-readable running progress on one line.
  sshpass -p "$JETSON_PASS" rsync -a --partial --inplace -h --info=progress2 \
    -e "ssh -o StrictHostKeyChecking=no -o LogLevel=ERROR" \
    "$d@${IPS[$d]}:$src" "$DEST/$RUN/" 2>&1 | tail -3
  # the manifest rides along separately (it sits one level up)
  sshpass -p "$JETSON_PASS" rsync -a --partial \
    -e "ssh -o StrictHostKeyChecking=no -o LogLevel=ERROR" \
    "$d@${IPS[$d]}:$BAGROOT/$RUN/$d.manifest.json" "$DEST/$RUN/" 2>/dev/null
  t1=$(date +%s); dt=$(( t1 - t0 )); [ "$dt" -lt 1 ] && dt=1
  echo "    $(numfmt --to=iec "$remote_sz") in ${dt}s = $(numfmt --to=iec $(( remote_sz / dt )))/s"
done
TOTAL_DT=$(( $(date +%s) - TOTAL_START ))

echo
echo "=== verifying laptop copy with ros2 bag info ==="
verify_local "$RUN"; VRC=$?

LOCAL_SZ=$(du -sh "$DEST/$RUN" 2>/dev/null | cut -f1)
{ echo "# Collection record: $RUN"
  echo
  echo "- collected_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "- drones: ${DR[*]}"
  echo "- transfer_seconds: $TOTAL_DT"
  echo "- local_size: $LOCAL_SZ"
  echo "- local_path: bags/$RUN/"
  echo "- verification: $([ $VRC -eq 0 ] && echo 'PASS (ros2 bag info)' || echo 'FAIL - see below')"
  echo
  echo '```'
  verify_local "$RUN"
  echo '```'
  echo
  echo "The drone copies were NOT deleted. Remove them by hand after confirming this."
} > "$DEST/$RUN/COLLECTION.md"

echo
echo "local: $DEST/$RUN ($LOCAL_SZ)   record: bags/$RUN/COLLECTION.md"
[ $VRC -eq 0 ] || echo "VERIFICATION FAILED - do not delete the drone copies."
exit $VRC
