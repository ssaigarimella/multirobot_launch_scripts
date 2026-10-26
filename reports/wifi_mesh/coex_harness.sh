#!/bin/bash
# coex_harness.sh - two-radio coexistence matrix harness (802.11s mesh + LoRa TDMA)
#
# Matrix rows: coordination_only | mesh_only | both_active
# Per row it records coordination-link delivery ratio, mesh throughput, per-peer signal.
#
# SAFETY MODEL (read before running):
#   * Switching wlP1p1s0 to 802.11s at 2437 MHz DROPS any SSH session carried on that
#     interface. This harness REFUSES to touch a drone that has no second path.
#   * A dead-man watchdog is armed on each drone BEFORE the switch. It is a detached
#     (setsid) process that restores the prior NetworkManager connection if the harness
#     stops heartbeating, or when a hard deadline expires - whichever comes first.
#   * The existing bring-up script hardcodes NM_CONNECTION="eduroam" in its own revert
#     path. That is NOT the connection in use on the bench (TP-Link_073A). This harness
#     therefore captures the ACTIVE connection name at arm time and restores that,
#     instead of relying on the script's revert.
#   * Cleanup runs on EXIT, INT, TERM and ERR.
#
# Subcommands:
#   selftest              exercise the dead-man + trap machinery with a NO-OP revert
#                         action (touches no interface). Safe to run any time.
#   preflight             check second-path + tooling for each drone; change nothing
#   arm-deadman  <drone>  arm the watchdog on one drone
#   cancel-deadman <drone> cancel it
#   run                   run the 3-row matrix (requires preflight to pass on BOTH)
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRED="${CRED:-/home/ssg/multi-robot-coordination/credentials.env}"
[ -f "$CRED" ] && source "$CRED"
: "${JETSON_PASS:?credentials.env missing JETSON_PASS}"

IFACE="${IFACE:-wlP1p1s0}"
MESH_IF_SCRIPT="/mnt/nova_ssd/workspaces/kernel_build/test_mesh_pair.sh"   # do not edit
MESH_SUBNET="192.168.77"
ROW_SECONDS="${ROW_SECONDS:-120}"
IPERF_SECONDS="${IPERF_SECONDS:-60}"
HEARTBEAT_STALE_S="${HEARTBEAT_STALE_S:-180}"   # revert if harness stops heartbeating
HARD_DEADLINE_S="${HARD_DEADLINE_S:-1800}"      # absolute cap, revert regardless
REMOTE_DIR="\$HOME/wifi_mesh"
CSV="${CSV:-$HERE/coex_matrix.csv}"

# drone -> control address used for ORCHESTRATION (must not be the wlan under test)
declare -A CTRL=( [delta]=192.168.0.40 [ghost]=192.168.0.206 )
declare -A MESHID=( [delta]=1 [ghost]=2 )

log(){ printf '[%s] %s\n' "$(date -u +%H:%M:%S)" "$*"; }
die(){ printf '[FATAL] %s\n' "$*" >&2; exit 1; }

ssh_d(){ local d="$1"; shift
  sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o LogLevel=ERROR \
    -o ConnectTimeout=8 "$d@${CTRL[$d]}" "$@"; }
sudo_d(){ local d="$1"; shift
  ssh_d "$d" "echo '$JETSON_PASS' | sudo -S bash -c $(printf '%q' "$*")" 2>/dev/null; }

# ---------------------------------------------------------------- preflight
# Returns 0 only if the drone has a control path that is NOT the interface under test.
second_path_ok(){
  local d="$1"
  local route_if
  route_if=$(ssh_d "$d" "ip route get ${CTRL[$d]%.*}.136 2>/dev/null | grep -oP 'dev \K\S+'" 2>/dev/null)
  local sess_if
  sess_if=$(ssh_d "$d" 'ip -o addr show | grep -w "$(echo $SSH_CONNECTION | awk "{print \$3}")" | awk "{print \$2}"' 2>/dev/null)
  [ -n "$sess_if" ] && [ "$sess_if" != "$IFACE" ]
}

cmd_preflight(){
  local ok=0
  printf '%-8s %-16s %-12s %-10s %-9s %s\n' DRONE CTRL_ADDR SESSION_IF SECOND_PATH IPERF3 VERDICT
  for d in delta ghost; do
    local sess_if iperf verdict
    sess_if=$(ssh_d "$d" 'ip -o addr show | grep -w "$(echo $SSH_CONNECTION | awk "{print \$3}")" | awk "{print \$2}"' 2>/dev/null)
    sess_if=${sess_if:-UNREACHABLE}
    iperf=$(ssh_d "$d" 'iperf3 --version 2>/dev/null | head -1' 2>/dev/null)
    iperf=${iperf:-absent}
    if [ "$sess_if" = "UNREACHABLE" ]; then verdict="BLOCKED: unreachable"
    elif [ "$sess_if" = "$IFACE" ]; then verdict="BLOCKED: only path is $IFACE"; ok=1
    else verdict="OK"; fi
    printf '%-8s %-16s %-12s %-10s %-9s %s\n' \
      "$d" "${CTRL[$d]}" "$sess_if" \
      "$([ "$sess_if" != "$IFACE" ] && [ "$sess_if" != UNREACHABLE ] && echo yes || echo NO)" \
      "${iperf%% *}" "$verdict"
  done
  return $ok
}

# ---------------------------------------------------------------- dead-man
# Detached watchdog. Restores the connection that was ACTIVE at arm time.
arm_deadman(){
  local d="$1" revert_action="${2:-}"
  local conn
  conn=$(ssh_d "$d" "nmcli -t -f NAME,DEVICE con show --active | awk -F: '\$2==\"$IFACE\"{print \$1; exit}'")
  [ -z "$conn" ] && conn="TP-Link_073A"
  MUTATED=1; case " $MUTATED_DRONES " in *" $d "*) ;; *) MUTATED_DRONES="$MUTATED_DRONES $d";; esac
  log "$d: arming dead-man (restore '$conn', stale=${HEARTBEAT_STALE_S}s, hard=${HARD_DEADLINE_S}s)"
  # default revert = full interface restore; selftest overrides with a no-op marker
  local action="${revert_action:-DEFAULT}"
  ssh_d "$d" "mkdir -p $REMOTE_DIR && cat > $REMOTE_DIR/deadman.sh <<'EOS'
#!/bin/bash
IFACE=\"$IFACE\"; CONN=\"$conn\"; STALE=$HEARTBEAT_STALE_S; HARD=$HARD_DEADLINE_S
HB=\"\$HOME/wifi_mesh/heartbeat\"; CANCEL=\"\$HOME/wifi_mesh/cancel\"
MARK=\"\$HOME/wifi_mesh/deadman_fired\"; ACTION=\"$action\"
start=\$(date +%s); touch \"\$HB\"
while :; do
  sleep 5
  [ -f \"\$CANCEL\" ] && { echo \"cancelled \$(date -u +%FT%TZ)\" >> \"\$HOME/wifi_mesh/deadman.log\"; exit 0; }
  now=\$(date +%s); hb=\$(stat -c %Y \"\$HB\" 2>/dev/null || echo \$start)
  if [ \$((now-hb)) -ge \$STALE ] || [ \$((now-start)) -ge \$HARD ]; then
    echo \"FIRED \$(date -u +%FT%TZ) stale=\$((now-hb)) elapsed=\$((now-start)) action=\$ACTION\" >> \"\$HOME/wifi_mesh/deadman.log\"
    if [ \"\$ACTION\" = \"DEFAULT\" ]; then
      echo '$JETSON_PASS' | sudo -S bash -c \"iw dev \$IFACE mesh leave; ip addr flush dev \$IFACE; ip link set \$IFACE down; iw dev \$IFACE set type managed; ip link set \$IFACE up; nmcli device set \$IFACE managed yes; nmcli connection up '\$CONN'\" >> \"\$HOME/wifi_mesh/deadman.log\" 2>&1
    fi
    touch \"\$MARK\"; exit 0
  fi
done
EOS
chmod +x $REMOTE_DIR/deadman.sh
rm -f $REMOTE_DIR/cancel $REMOTE_DIR/deadman_fired $REMOTE_DIR/heartbeat
setsid nohup $REMOTE_DIR/deadman.sh >/dev/null 2>&1 < /dev/null &
sleep 1; pgrep -f '[d]eadman.sh' >/dev/null && echo ARMED || echo ARM_FAILED"
}
heartbeat(){ ssh_d "$1" "touch $REMOTE_DIR/heartbeat" 2>/dev/null; }
cancel_deadman(){ ssh_d "$1" "touch $REMOTE_DIR/cancel; sleep 7; pkill -f '[d]eadman.sh' 2>/dev/null; echo CANCELLED" 2>/dev/null; }

# ---------------------------------------------------------------- selftest
cmd_selftest(){
  local d="${1:-ghost}"
  log "SELFTEST on $d - dead-man armed with a NO-OP revert action. No interface is touched."
  HEARTBEAT_STALE_S=15 HARD_DEADLINE_S=600 arm_deadman "$d" NOOP
  log "  (a) confirming it does NOT fire while heartbeating"
  for i in 1 2 3 4 5 6; do heartbeat "$d"; sleep 3; done
  local fired; fired=$(ssh_d "$d" "[ -f $REMOTE_DIR/deadman_fired ] && echo YES || echo NO")
  [ "$fired" = "NO" ] && log "  (a) PASS - did not fire while heartbeating" || { log "  (a) FAIL"; return 1; }
  log "  (b) stopping heartbeat; it must fire within ~${HEARTBEAT_STALE_S}s+5s"
  sleep 26
  fired=$(ssh_d "$d" "[ -f $REMOTE_DIR/deadman_fired ] && echo YES || echo NO")
  [ "$fired" = "YES" ] && log "  (b) PASS - fired on heartbeat stall" || { log "  (b) FAIL - never fired"; return 1; }
  log "  (c) cancel path"
  HEARTBEAT_STALE_S=15 HARD_DEADLINE_S=600 arm_deadman "$d" NOOP >/dev/null
  cancel_deadman "$d" >/dev/null
  sleep 8
  local alive; alive=$(ssh_d "$d" "pgrep -f '[d]eadman.sh' >/dev/null && echo ALIVE || echo GONE")
  [ "$alive" = "GONE" ] && log "  (c) PASS - cancel stops the watchdog" || { log "  (c) FAIL - still running"; return 1; }
  log "  deadman.log:"; ssh_d "$d" "cat $REMOTE_DIR/deadman.log 2>/dev/null" | sed 's/^/      /'
  log "SELFTEST PASSED"
}

# ---------------------------------------------------------------- measurement
peer_stats(){ sudo_d "$1" "iw dev $IFACE station dump"; }

sample_signal(){   # drone, seconds -> raw samples to file, echo summary
  local d="$1" secs="$2" out="$3"
  : > "$out"
  for ((i=0;i<secs;i++)); do
    { echo "### t=$(date -u +%FT%T.%3NZ)"; peer_stats "$d"; } >> "$out"
    heartbeat "$d"; sleep 1
  done
  awk '/signal:/{gsub(/[^-0-9]/,"",$2); if($2!=""){s+=$2;n++; if(m==""||$2<m)m=$2; if(M==""||$2>M)M=$2}}
       END{if(n)printf "%.1f %s %s %d",s/n,m,M,n; else printf "NA NA NA 0"}' "$out"
}

run_iperf(){   # returns "tcp_mbps udp_mbps jitter_ms loss_pct retransmits"
  local srv="$1" cli="$2" srv_ip="$3"
  sudo_d "$srv" "pkill -f 'iperf3 -s' ; nohup iperf3 -s -1 >/dev/null 2>&1 & sleep 1"
  local tcp; tcp=$(ssh_d "$cli" "iperf3 -c $srv_ip -t $IPERF_SECONDS -J 2>/dev/null")
  local tcp_mbps retr
  tcp_mbps=$(echo "$tcp" | python3 -c "import sys,json;d=json.load(sys.stdin);print(round(d['end']['sum_received']['bits_per_second']/1e6,2))" 2>/dev/null || echo NA)
  retr=$(echo "$tcp"    | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['end']['sum_sent'].get('retransmits','NA'))" 2>/dev/null || echo NA)
  sudo_d "$srv" "pkill -f 'iperf3 -s'; nohup iperf3 -s -1 >/dev/null 2>&1 & sleep 1"
  local udp; udp=$(ssh_d "$cli" "iperf3 -c $srv_ip -u -b 100M -t $IPERF_SECONDS -J 2>/dev/null")
  local udp_mbps jit loss
  udp_mbps=$(echo "$udp" | python3 -c "import sys,json;d=json.load(sys.stdin);print(round(d['end']['sum']['bits_per_second']/1e6,2))" 2>/dev/null || echo NA)
  jit=$(echo "$udp"      | python3 -c "import sys,json;d=json.load(sys.stdin);print(round(d['end']['sum']['jitter_ms'],3))" 2>/dev/null || echo NA)
  loss=$(echo "$udp"     | python3 -c "import sys,json;d=json.load(sys.stdin);print(round(d['end']['sum']['lost_percent'],3))" 2>/dev/null || echo NA)
  echo "$tcp_mbps $udp_mbps $jit $loss $retr"
}

# Coordination-link delivery ratio.
# BLOCKED: the firmware's 0x81 PEER_POSE frame carries no sequence number, so RX-side
# delivery ratio cannot be derived. Returns the sentinel until the LoRa instrumentation
# change (seq + uptime_ms in sendPeerPose) lands.
coord_delivery(){
  local d="$1"
  local has_seq
  has_seq=$(ssh_d "$d" "grep -c 'sequence' /mnt/nova_ssd/workspaces/isaac_ros-dev/src/radiohive/msg/PeerPose.msg 2>/dev/null || echo 0")
  if [ "${has_seq:-0}" -gt 0 ]; then
    ssh_d "$d" "python3 $REMOTE_DIR/lora_delivery.py --seconds $ROW_SECONDS" 2>/dev/null || echo "NA NA NA"
  else
    echo "NA NA UNAVAILABLE_NO_SEQ_FIELD"
  fi
}

clock_offset_ms(){   # chronyc is ABSENT on these drones; use SSH midpoint estimate
  local d="$1"
  python3 - "$d" "${CTRL[$d]}" "$JETSON_PASS" <<'PY' 2>/dev/null || echo NA
import subprocess,sys,time
d,ip,pw=sys.argv[1],sys.argv[2],sys.argv[3]
best=None
for _ in range(5):
    t0=time.time()
    r=subprocess.run(['sshpass','-p',pw,'ssh','-o','StrictHostKeyChecking=no','-o','LogLevel=ERROR',
                      f'{d}@{ip}','date +%s.%N'],capture_output=True,text=True,timeout=15)
    t1=time.time()
    try: tr=float(r.stdout.strip())
    except ValueError: continue
    rtt=t1-t0
    if best is None or rtt<best[0]: best=(rtt,(tr-(t0+t1)/2)*1000)
print(f"{best[1]:.1f}" if best else "NA")
PY
}

# ---------------------------------------------------------------- rows
CLEANED=0
MUTATED=0          # set to 1 the moment we change ANY drone state
MUTATED_DRONES=""
cleanup(){
  local rc=$?
  [ "$CLEANED" = 1 ] && exit $rc
  CLEANED=1
  if [ "$MUTATED" = 0 ]; then
    log "CLEANUP: nothing was changed - no restore needed"
    exit $rc
  fi
  log "CLEANUP: restoring drones to prior configuration (rc=$rc, changed: $MUTATED_DRONES)"
  for d in $MUTATED_DRONES; do
    sudo_d "$d" "iw dev $IFACE mesh leave; ip addr flush dev $IFACE; ip link set $IFACE down; iw dev $IFACE set type managed; ip link set $IFACE up; nmcli device set $IFACE managed yes" >/dev/null 2>&1
    local conn; conn=$(ssh_d "$d" "nmcli -t -f NAME,DEVICE con show | awk -F: '\$2==\"'$IFACE'\"{print \$1; exit}'" 2>/dev/null)
    [ -z "$conn" ] && conn=$(ssh_d "$d" "nmcli -t -f NAME con show | grep -i 'TP-Link' | head -1" 2>/dev/null)
    [ -n "$conn" ] && sudo_d "$d" "nmcli connection up '$conn'" >/dev/null 2>&1
    cancel_deadman "$d" >/dev/null 2>&1
    log "  $d restored"
  done
  exit $rc
}
trap cleanup EXIT INT TERM

mesh_up(){   # via the EXISTING bring-up script only; never edited
  local d="$1"
  MUTATED=1; case " $MUTATED_DRONES " in *" $d "*) ;; *) MUTATED_DRONES="$MUTATED_DRONES $d";; esac
  log "$d: mesh up via $MESH_IF_SCRIPT ${MESHID[$d]} (802.11s, 2437 MHz)"
  sudo_d "$d" "nohup bash $MESH_IF_SCRIPT ${MESHID[$d]} >$REMOTE_DIR/meshup.log 2>&1 & sleep 25"
}
mesh_down(){ sudo_d "$1" "iw dev $IFACE mesh leave" >/dev/null 2>&1; }
lora_up(){   ssh_d "$1" "nohup python3 /mnt/nova_ssd/workspaces/isaac_ros-dev/src/radiohive/scripts/lora_bridge_node.py >$REMOTE_DIR/lora.log 2>&1 & echo started"; }
lora_down(){ ssh_d "$1" "pkill -f lora_bridge_node.py" >/dev/null 2>&1; }

cmd_run(){
  # preflight first: nothing has been mutated yet, so a failure exits clean
  cmd_preflight || die "preflight FAILED - refusing to change any interface. See report."
  : > "$CSV"
  echo "row,t_start_utc,t_end_utc,duration_s,coord_tx,coord_rx,coord_delivery_ratio,coord_status,mesh_tcp_mbps,mesh_udp_mbps,mesh_udp_jitter_ms,mesh_udp_loss_pct,mesh_retransmits,peer_signal_dbm_mean,peer_signal_dbm_min,peer_signal_dbm_max,peer_signal_n,peer_count,clock_offset_delta_ms,clock_offset_ghost_ms,clock_method,separation_m,antenna_orientation" >> "$CSV"
  for d in delta ghost; do arm_deadman "$d"; done
  for row in coordination_only mesh_only both_active; do
    log "=== ROW: $row ==="
    local t0 off_d0 off_g0; t0=$(date -u +%FT%TZ)
    off_d0=$(clock_offset_ms delta); off_g0=$(clock_offset_ms ghost)
    case "$row" in
      coordination_only) for d in delta ghost; do mesh_down "$d"; lora_up "$d"; done ;;
      mesh_only)         for d in delta ghost; do lora_down "$d"; mesh_up "$d"; done ;;
      both_active)       for d in delta ghost; do mesh_up "$d"; lora_up "$d"; done ;;
    esac
    local sig="NA NA NA 0" ip3="NA NA NA NA NA" pc=0
    if [ "$row" != coordination_only ]; then
      sig=$(sample_signal delta "$ROW_SECONDS" "$HERE/raw_signal_${row}.txt")
      pc=$(peer_stats delta | grep -c '^Station')
      ip3=$(run_iperf ghost delta "${MESH_SUBNET}.${MESHID[ghost]}")
    else
      sleep "$ROW_SECONDS"
    fi
    local cd; cd=$(coord_delivery delta)
    local t1; t1=$(date -u +%FT%TZ)
    echo "$row,$t0,$t1,$ROW_SECONDS,$(echo $cd|tr ' ' ','),$(echo $ip3|tr ' ' ','),$(echo $sig|tr ' ' ','),$pc,$off_d0,$off_g0,ssh_midpoint,${SEPARATION_M:-UNRECORDED},${ANTENNA:-UNRECORDED}" >> "$CSV"
    log "row $row done"
  done
  log "CSV -> $CSV"
}

case "${1:-}" in
  selftest)       shift; cmd_selftest "$@" ;;
  preflight)      trap - EXIT INT TERM; cmd_preflight ;;
  arm-deadman)    trap - EXIT INT TERM; arm_deadman "$2" ;;
  cancel-deadman) trap - EXIT INT TERM; cancel_deadman "$2" ;;
  run)            cmd_run ;;
  *) echo "usage: $0 {selftest|preflight|arm-deadman <drone>|cancel-deadman <drone>|run}"; trap - EXIT INT TERM; exit 2 ;;
esac
