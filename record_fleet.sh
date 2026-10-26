#!/bin/bash
# record_fleet.sh - per-drone ros2 bag recording for multi-drone exploration runs.
#
# Runs ALONGSIDE the normal flow (./launch_fleet_tmux.sh ghost delta). Purely
# additive: it starts no nodes, remaps nothing, and touches no network config.
# It only opens extra subscribers inside the container each drone already runs.
#
#   ./record_fleet.sh start  <run> [profile] ghost delta
#   ./record_fleet.sh status <run> ghost delta      # proves the bag is GROWING
#   ./record_fleet.sh stop   <run> ghost delta      # SIGINT -> metadata.yaml
#   ./record_fleet.sh list   ghost delta
#
#   profile = figures (default) | diagnostic | full
#
# Output on each drone (picked up by ./fleet_ctl collect, or ./collect_bags.sh):
#   experiment_logs/bags/<run>/<drone>/          rosbag2 dir, split into volumes
#   experiment_logs/bags/<run>/<drone>.manifest.json
#
# ---------------------------------------------------------------- tmux session
# The session is named 'bagrec' ON PURPOSE. `fleet_ctl stop` kills only sessions
# matching ^(fleet|zenoh|rviz|loratest), so a recording SURVIVES a stack
# teardown and must be stopped explicitly with `./record_fleet.sh stop`.
#
# That is the deliberate choice, and the reason matters: `ros2 bag record` only
# writes metadata.yaml when it receives SIGINT. If the sweep killed the session,
# the bag would be left as raw .mcap/.db3 volumes with NO metadata.yaml, which
# `ros2 bag info` and `ros2 bag play` both refuse to open. Letting teardown kill
# the recorder would silently corrupt the run. So: stop the recording first,
# then tear the stack down.
#
# ---------------------------------------------------------------- disk
# MEASURED 2026-08-28 on ghost and delta: /mnt/nova_ssd is NOT a separate mount.
# `findmnt /mnt/nova_ssd` -> not a mountpoint; it is a directory on the root
# filesystem (/dev/nvme0n1p1, 937G, mounted at /). There is no eMMC on either
# vehicle. The free-space guard therefore checks the filesystem that actually
# backs the bag directory, which is the root fs.
#
# ---------------------------------------------------------------- timebase
# Bags are stamped with each drone's CLOCK_REALTIME, the same clock
# metrics_recorder.py and rf_stats.sh use. Same-drone joins are exact.
# Cross-drone is bounded by NTP skew: roughly +/-25-50 ms late in a poll
# interval (see rf_stats.sh header for the measurement). Good enough to merge
# trajectories and maps for a figure; NOT good enough to claim per-message
# cross-drone synchrony.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/credentials.env" 2>/dev/null
: "${JETSON_PASS:?credentials.env missing - copy credentials.env.example and fill it in}"

WS=/mnt/nova_ssd/workspaces/isaac_ros-dev
CWS=/workspaces/isaac_ros-dev
CONTAINER=isaac_ros_realsense
BAGROOT="$WS/experiment_logs/bags"      # HOST path  (df, du, ls, manifest via ssh)
CBAGROOT="$CWS/experiment_logs/bags"    # CONTAINER path (ros2 bag record runs in here)
# These are the same tree: $WS on the host is bind-mounted at $CWS in the
# container. They are NOT interchangeable - using the host path inside the
# container fails with "No such file or directory" (see commit 0ce7d64).
SESS="${SESS:-bagrec}"
MIN_FREE_GB="${MIN_FREE_GB:-20}"     # refuse to start below this
MAX_BAG_MB="${MAX_BAG_MB:-2048}"     # split volumes at 2 GiB

declare -A IPS=( [delta]=192.168.0.40 [buckshee]=192.168.0.60 [ghost]=192.168.0.50 [thunderstrike]=192.168.0.70 )
declare -A IDS=( [delta]=1 [buckshee]=2 [ghost]=3 [thunderstrike]=4 )

ssh_d(){ local d="$1"; shift
  sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o LogLevel=ERROR \
    -o ConnectTimeout=10 "$d@${IPS[$d]}" "$@"; }

# run a command inside the drone's ROS container (base64 so quoting survives)
ros_d(){ local d="$1"; shift
  local pre="source /opt/ros/humble/setup.bash; source $CWS/install/setup.bash;"
  pre="$pre export ROS_DOMAIN_ID=${IDS[$d]} ROS_LOCALHOST_ONLY=0;"
  local b64; b64=$(printf '%s %s' "$pre" "$*" | base64 -w0)
  ssh_d "$d" "docker exec -u admin $CONTAINER bash -c 'echo $b64 | base64 -d | bash'"; }

# ------------------------------------------------------------------ profiles
# Bandwidth figures are MEASURED (2026-08-28, ghost+delta stationary on the
# bench, both stacks tabs 1-7 up). Per-topic worst case across the two drones.
# Map topics scale with mapped surface area, so these grow during a flight;
# budget with margin.
profile_topics(){
  local prof="$1" id="$2" peers="$3" t=()

  # --- figures: everything the paper visuals need. -------------------------
  # Deliberately biased to types stock ROS 2 Humble desktop can deserialise,
  # because the LAPTOP HAS NO nvblox_msgs / multi_drone_nvblox / px4_msgs
  # (verified 2026-08-28). See the replay script for how the rest degrades.
  t+=( /tf /tf_static )                                    # 6.7 KB/s + latched
  t+=( /nvblox_node/color_layer_marker )                   # 1.92 MB/s  Marker  <- the map
  t+=( /nvblox_node/static_occupancy_grid )                # 288 KB/s   OccupancyGrid
  t+=( /nvblox_node/esdf_slice_bounds )                    # 1.9 KB/s
  t+=( /nvblox_node/workspace_bounds )                     # 1.7 KB/s
  t+=( /nvblox_shared_node/color_layer_marker )            # peers' map (Marker)
  t+=( /nvblox_shared_node/static_occupancy_grid )         # 62 KB/s
  # NOT the two static_esdf_pointcloud topics: outdoors they are 4.3 MB per
  # message at ~2 Hz per mapper = 86 % of a 37 GB / 7 min flight bag
  # (thunderstrike, 2026-10-23). The occupancy grids carry the same slice for
  # replay; the ESDF clouds stay in the diagnostic profile.
  t+=( /visual_slam/tracking/odometry )                    # 21 KB/s  <- trajectory
  t+=( /fis/frontier_markers /fis/frontier_centroids )     # 5.2 KB/s / 31 B/s
  t+=( /fis/frontier_viewpoints /fis/frontier_gains )      # 30 B/s / 17 B/s
  t+=( /exploration/markers /exploration/state )           # 0 / 323 B/s
  t+=( /reactive_guard/status /reactive_guard/blocked )    # 488 B/s
  t+=( /loop_closure_alignment )                           # event-driven
  t+=( /swarm/peers )                                      # 572 B/s  custom type
  t+=( /coordination/claim_intent /coordination/mode_state ) # event-driven, custom
  t+=( "/d$id/alignment/T_world_self" "/d$id/alignment/status" )  # 418 B/s / 56 B/s
  t+=( "/d$id/swarm/link_stats" )                          # 38 B/s
  [ "$prof" = figures ] && { printf '%s\n' "${t[@]}"; return; }

  # --- diagnostic: + planner internals, keyframes, radio, PX4 --------------
  t+=( /nvblox_node/static_esdf_pointcloud )               # 4.3 MB/msg outdoors
  t+=( /nvblox_shared_node/static_esdf_pointcloud )        # 4.3 MB/msg outdoors
  t+=( /visual_slam/tracking/slam_path )                   # 435 KB/s cumulative Path
  t+=( /visual_slam/tracking/vo_path )                     # 473 KB/s cumulative Path
  t+=( /visual_slam/tracking/vo_pose /visual_slam/status )
  t+=( /nvblox_node/static_map_slice )                     # 1.15 MB/s DistanceMapSlice
  t+=( /nvblox_node/tsdf_layer_marker )                    # 4.24 MB/s Marker
  t+=( /nvblox_shared_node/static_map_slice )              # 1.11 MB/s
  t+=( /planning/trajectory_setpoint )                     # 1.37 KB/s px4_msgs
  t+=( "/d$id/keyframes/depth" )                           # 51 KB/s
  for p in $peers; do t+=( "/d$p/keyframes/depth" "/d$p/alignment/T_world_self" ); done
  t+=( "/d$id/fault/state" )
  t+=( "/d$id/lora/peer_pose" "/d$id/lora/peer_msg" "/d$id/lora/peer_event" )
  t+=( "/d$id/lora/mesh_status" "/d$id/lora/tx_msg" "/d$id/lora/tx_pose" )
  t+=( /fmu/out/vehicle_local_position /fmu/out/vehicle_attitude /fmu/out/vehicle_status )
  t+=( /camera0/depth/camera_info /camera0/color/camera_info )
  t+=( /diagnostics /rosout )
  [ "$prof" = diagnostic ] && { printf '%s\n' "${t[@]}"; return; }

  # --- full: + raw camera. BENCH ONLY. ~99 MB/s = 5.9 GB/min PER DRONE. ----
  # NOT FOR FLIGHT: this saturates the disk and steals CPU from cuVSLAM.
  t+=( /camera0/color/image_raw )                          # 9.26 MB/s
  t+=( /camera0/color/image_raw/compressed )               # 660 KB/s
  t+=( /camera0/depth/image_rect_raw )                     # 22.61 MB/s
  t+=( /camera0/depth/image_rect_raw/compressedDepth )     # 5.49 MB/s
  t+=( /camera0/infra1/image_rect_raw )                    # 18.48 MB/s
  t+=( /camera0/infra2/image_rect_raw )                    # 17.55 MB/s
  t+=( /camera0/imu /camera0/accel/sample /camera0/gyro/sample )  # 68 KB/s each
  printf '%s\n' "${t[@]}"; }

usage(){ sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }

drones_from(){ local out=(); for a in "$@"; do case "$a" in
    all) out=(delta buckshee ghost thunderstrike) ;;
    delta|buckshee|ghost|thunderstrike) out+=("$a") ;;
  esac; done
  [ ${#out[@]} -eq 0 ] && out=(ghost delta); printf '%s\n' "${out[@]}"; }

# ------------------------------------------------------------------ start
cmd_start(){
  local run="${1:?usage: start <run> [profile] <drones...>}"; shift
  local prof=figures
  case "${1:-}" in figures|diagnostic|full) prof="$1"; shift ;; esac
  local note="${NOTE:-}" transport="${TRANSPORT:-bench}"
  mapfile -t DR < <(drones_from "$@")

  if [ "$prof" = full ]; then
    echo "!! profile 'full' records raw camera: ~99 MB/s = 5.9 GB/min PER DRONE."
    echo "!! BENCH ONLY - it will not keep up in flight and steals CPU from cuVSLAM."
  fi

  local ids=(); for d in "${DR[@]}"; do ids+=("${IDS[$d]}"); done
  for d in "${DR[@]}"; do
    local id="${IDS[$d]}" peers=""
    for p in "${ids[@]}"; do [ "$p" = "$id" ] || peers+=" $p"; done
    local dir="$BAGROOT/$run/$d"

    # ---- refuse if the session is already recording this run
    if ssh_d "$d" "tmux has-session -t $SESS 2>/dev/null"; then
      echo "$(printf '%-8s' "$d") ALREADY RECORDING (tmux '$SESS') - stop it first"; continue; fi

    # ---- never clobber a finished flight: the recorder below rm -rf's its
    # output dir (ros2 bag record needs it absent), so a reused run name
    # would silently delete the earlier bag.
    if ssh_d "$d" "ls '$dir'/*.db3 >/dev/null 2>&1"; then
      echo "$(printf '%-8s' "$d") REFUSING to start: run '$run' already has a bag here - pick a new run name"; continue; fi

    # ---- free space guard, on the fs that actually backs the bag dir
    local freegb
    freegb=$(ssh_d "$d" "mkdir -p '$dir'; df -BG --output=avail '$dir' | tail -1 | tr -dc '0-9'")
    if [ -z "$freegb" ] || [ "$freegb" -lt "$MIN_FREE_GB" ]; then
      echo "$(printf '%-8s' "$d") REFUSING to start: ${freegb:-?} GB free < ${MIN_FREE_GB} GB threshold"; continue; fi

    # ---- topic list, and which of them actually have a publisher right now.
    # ROS 2 discovery is eventually-consistent: a single `ros2 topic list` can
    # under-report while the graph is still converging. Observed 2026-08-28 on
    # delta, which returned 4 nodes / a partial topic list while all seven tabs
    # were demonstrably healthy - which silently narrowed a recording to 13 of
    # 26 topics. So: sample until two consecutive reads agree, then use that.
    # (--no-daemon is NOT the fix; it returned FEWER topics, 128 vs 154.)
    mapfile -t TOPICS < <(profile_topics "$prof" "$id" "$peers")
    local live prev="" n=0
    for _ in 1 2 3 4 5 6; do
      live=$(ros_d "$d" "ros2 topic list 2>/dev/null" | tr -d '\r')
      n=$(grep -c . <<<"$live")
      [ -n "$prev" ] && [ "$n" = "$prev" ] && [ "$n" -gt 0 ] && break
      prev="$n"; sleep 4
    done
    local rec=() missing=()
    for t in "${TOPICS[@]}"; do
      if grep -qxF "$t" <<<"$live"; then rec+=("$t"); else missing+=("$t"); fi; done
    if [ ${#rec[@]} -eq 0 ]; then
      echo "$(printf '%-8s' "$d") REFUSING to start: none of the profile topics exist"; continue; fi

    # ---- manifest
    local shas; shas=$(ssh_d "$d" "for r in $WS/src/*/; do [ -d \"\$r/.git\" ] && \
        printf '    \"%s\": \"%s\",\n' \"\$(basename \$r)\" \"\$(git -C \$r rev-parse --short HEAD 2>/dev/null)\"; done")
    local mf="$BAGROOT/$run/$d.manifest.json"
    local start_utc; start_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
    ssh_d "$d" "mkdir -p '$BAGROOT/$run'; cat > '$mf' <<MEOF
{
  \"run\": \"$run\",
  \"drone\": \"$d\",
  \"drone_id\": $id,
  \"ros_domain_id\": $id,
  \"profile\": \"$prof\",
  \"transport\": \"$transport\",
  \"note\": \"$note\",
  \"start_utc\": \"$start_utc\",
  \"end_utc\": null,
  \"hostname\": \"\$(hostname)\",
  \"bag_dir\": \"$dir\",
  \"max_bag_size_mb\": $MAX_BAG_MB,
  \"storage_id\": \"sqlite3\",
  \"free_gb_at_start\": $freegb,
  \"git_shas\": {
$shas
    \"_\": \"\"
  },
  \"topics_recorded\": [$(printf '"%s",' "${rec[@]}" | sed 's/,$//')],
  \"topics_absent_at_start\": [$(printf '"%s",' "${missing[@]-}" | sed 's/,$//')]
}
MEOF"

    # ---- launch the recorder in its own tmux ON THE DRONE
    # -d <dir> is removed first: rosbag2 refuses to write into an existing dir.
    # --storage sqlite3, NOT mcap. The drones have librosbag2_storage_mcap.so
    # but the LAPTOP HAS ONLY rosbag2_storage_default_plugins (sqlite3),
    # verified 2026-08-28. An mcap bag would be unreadable exactly where the
    # figures get made, and installing the plugin is out of scope.
    local cdir="$CBAGROOT/$run/$d"
    local inner="cd $CWS/experiment_logs && rm -rf '$cdir' && ros2 bag record \
--max-bag-size $((MAX_BAG_MB*1024*1024)) --storage sqlite3 -o '$cdir' ${rec[*]}"
    local b64; b64=$(printf 'source /opt/ros/humble/setup.bash; source %s/install/setup.bash; export ROS_DOMAIN_ID=%d ROS_LOCALHOST_ONLY=0; %s' "$CWS" "$id" "$inner" | base64 -w0)
    ssh_d "$d" "tmux new-session -d -x 200 -y 50 -s $SESS \
      \"docker exec -i -u admin $CONTAINER bash -c 'echo $b64 | base64 -d | bash'; exec bash\" && \
      tmux pipe-pane -o -t $SESS 'cat >> $WS/experiment_logs/pane_${SESS}_\$(date +%s).log'" >/dev/null

    echo "$(printf '%-8s' "$d") recording '$prof' -> $dir  (${#rec[@]} topics, ${#missing[@]} absent, ${freegb} GB free)"
    [ ${#missing[@]} -gt 0 ] && printf '           absent: %s\n' "$(printf '%s ' "${missing[@]}")"
  done
  echo
  echo "Prove it is growing:  ./record_fleet.sh status $run ${DR[*]}"
  echo "Stop cleanly:         ./record_fleet.sh stop   $run ${DR[*]}"
}

# ------------------------------------------------------------------ status
# Proves GROWTH, not existence. A bag directory with only a header is the
# failure mode that has already cost this project a run, so `status` samples
# the on-disk byte count twice, 5 s apart, and reports the delta.
cmd_status(){
  local run="${1:?usage: status <run> <drones...>}"; shift
  mapfile -t DR < <(drones_from "$@")
  for d in "${DR[@]}"; do
    local dir="$BAGROOT/$run/$d"
    local alive; alive=$(ssh_d "$d" "tmux has-session -t $SESS 2>/dev/null && echo yes || echo no")
    local proc;  proc=$(ssh_d "$d" "docker exec -u admin $CONTAINER pgrep -cf '[b]ag record' 2>/dev/null; true" | head -1 | tr -dc '0-9')
    proc="${proc:-0}"
    read -r b1 b2 files < <(ssh_d "$d" "a=\$(du -sb '$dir' 2>/dev/null | cut -f1); sleep 5; \
        b=\$(du -sb '$dir' 2>/dev/null | cut -f1); n=\$(ls -1 '$dir' 2>/dev/null | wc -l); echo \"\${a:-0} \${b:-0} \${n:-0}\"")
    local grew=$(( ${b2:-0} - ${b1:-0} ))
    local verdict
    if   [ "$alive" != yes ];        then verdict="NOT RUNNING (no '$SESS' session)"
    elif [ "${proc:-0}" -lt 1 ];     then verdict="SESSION UP but NO 'bag record' PROCESS"
    elif [ "$grew" -gt 0 ];          then verdict="HEALTHY - grew $(numfmt --to=iec $grew 2>/dev/null || echo ${grew}B) in 5 s"
    elif [ "${b2:-0}" -eq 0 ];       then verdict="BROKEN - bag dir is EMPTY"
    else                                  verdict="STALLED - $(numfmt --to=iec ${b2:-0} 2>/dev/null) on disk but NO GROWTH in 5 s"
    fi
    printf '%-8s tmux=%-3s proc=%-2s files=%-3s size=%-9s %s\n' \
      "$d" "$alive" "${proc:-0}" "${files:-0}" "$(numfmt --to=iec ${b2:-0} 2>/dev/null || echo ${b2:-0})" "$verdict"
  done
}

# ------------------------------------------------------------------ stop
cmd_stop(){
  local run="${1:?usage: stop <run> <drones...>}"; shift
  mapfile -t DR < <(drones_from "$@")
  for d in "${DR[@]}"; do
    # SIGINT the recorder so rosbag2 finalises metadata.yaml, THEN drop the
    # session. Killing the session first would leave an unreadable bag.
    # Bracketed pattern: an unbracketed one matches the sweeping shell itself.
    ssh_d "$d" "docker exec -u admin $CONTAINER pkill -INT -f '[b]ag record' 2>/dev/null; true" >/dev/null
    sleep 3
    ssh_d "$d" "tmux kill-session -t $SESS 2>/dev/null; true" >/dev/null
    local mf="$BAGROOT/$run/$d.manifest.json"
    ssh_d "$d" "[ -f '$mf' ] && sed -i 's|\"end_utc\": null|\"end_utc\": \"$(date -u +%Y-%m-%dT%H:%M:%SZ)\"|' '$mf'; true" >/dev/null
    local info; info=$(ros_d "$d" "ros2 bag info '$CBAGROOT/$run/$d' 2>&1 | head -40")
    echo "=== $d ==="; echo "$info" | sed 's/^/  /'
  done
}

cmd_list(){ mapfile -t DR < <(drones_from "$@")
  for d in "${DR[@]}"; do echo "=== $d ==="
    ssh_d "$d" "ls -1 '$BAGROOT' 2>/dev/null | sed 's/^/  /' || echo '  (no bags)'"; done; }

case "${1:-}" in
  start)  shift; cmd_start  "$@" ;;
  status) shift; cmd_status "$@" ;;
  stop)   shift; cmd_stop   "$@" ;;
  list)   shift; cmd_list   "$@" ;;
  -h|--help|help|"") usage 0 ;;
  *) echo "unknown subcommand: $1"; usage 1 ;;
esac
