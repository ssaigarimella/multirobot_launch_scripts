#!/bin/bash
# run01: A/B baseline — capture broken (wifi ON) and working (radio OFF) bundles
# in ONE offline trip, using an on-Jetson nohup script that survives rfkill.
#
# Run from the LAPTOP while on TP-Link_073A:   bash ghost_debug/run01/test.sh
# No internet needed. Takes ~12-15 min. SSH to ghost WILL drop mid-run (radio-off
# phase) — that is expected; the script keeps polling until ghost comes back.
#
# IF THIS SCRIPT ENDS WITH "WIFI RESTORE FAILED": power-cycle ghost, reconnect,
# then re-run ONLY the fetch step printed at the bottom.
set -u

GHOST="ghost@192.168.0.50"
PASS="abc123"
RUNDIR="$(cd "$(dirname "$0")" && pwd)"
REMOTE_OUT="/home/ghost/ghost_debug_run01"

SSHQ() { sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$GHOST" "$@"; }
SCPQ() { sshpass -p "$PASS" scp -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$@"; }

echo "[run01] 1/6 checking connectivity to ghost ..."
if ! SSHQ 'echo connected' ; then
    echo "[run01] FAIL: cannot reach $GHOST. Are you on TP-Link_073A?"
    exit 1
fi

echo "[run01] 2/6 cleaning state on ghost (tmux kill-server, stale runs, container procs) ..."
SSHQ 'pkill -f gdbg_run01_jetson 2>/dev/null; tmux kill-server 2>/dev/null; rm -rf /home/ghost/ghost_debug_run01 /home/ghost/gdbg_run01_nohup.log; docker ps --format "{{.Names}}" | grep -qx isaac_ros_realsense || docker start isaac_ros_realsense; docker exec -u admin isaac_ros_realsense bash -c "pkill -f MicroXRCEAgent; pkill -f vio_bridge; pkill -f component_container; pkill -f realsense; pkill -f \"ros2 launch\"; true" 2>/dev/null; true'

echo "[run01] 3/6 pushing on-Jetson A/B capture script ..."
TMPF="$(mktemp)"
# Quoted heredoc: NOTHING expands on the laptop; this exact text runs on ghost.
cat > "$TMPF" <<'JEOF'
#!/bin/bash
# gdbg_run01_jetson.sh — runs ON ghost under nohup/setsid. A/B capture.
set -u
OUT=/home/ghost/ghost_debug_run01
PASS=abc123
CONTAINER=isaac_ros_realsense
# Exact DOCKER_SOURCE from launch_ghost_jetson_eduroam_python_frontierexplore_tmux.sh
# (+ PYTHONUNBUFFERED so ros2 CLI output survives `timeout` when piped to files)
DSRC='export ROS_DOMAIN_ID=3 ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1 && source /opt/ros/humble/setup.bash && cd /workspaces/isaac_ros-dev && source install/setup.bash'

WIFI_IF=$(iw dev 2>/dev/null | awk '$1=="Interface"{print $2; exit}')
[ -z "$WIFI_IF" ] && WIFI_IF=wlP1p1s0

LOG(){ echo "[$(date '+%H:%M:%S')] $*"; }
SUDO(){ echo "$PASS" | sudo -S -p '' "$@"; }

ACTIVE_CONN=$(nmcli -t -f NAME,DEVICE connection show --active 2>/dev/null | awk -F: -v d="$WIFI_IF" '$2==d{print $1; exit}')

# ---- bulletproof wifi restore: runs on ANY exit path -----------------------
WIFI_RESTORED=0
restore_wifi(){
    [ "$WIFI_RESTORED" = 1 ] && return 0
    LOG "restoring wifi: rfkill unblock"
    SUDO rfkill unblock wifi
    sleep 8
    for i in $(seq 1 24); do
        if ping -c1 -W2 192.168.0.1 >/dev/null 2>&1; then
            WIFI_RESTORED=1; LOG "wifi restored (ping gateway OK)"; return 0
        fi
        sleep 5
    done
    LOG "no ping after 120s; fallback: nmcli connection up '$ACTIVE_CONN'"
    [ -n "$ACTIVE_CONN" ] && SUDO nmcli connection up "$ACTIVE_CONN"
    for i in $(seq 1 12); do
        if ping -c1 -W2 192.168.0.1 >/dev/null 2>&1; then
            WIFI_RESTORED=1; LOG "wifi restored after nmcli up"; return 0
        fi
        sleep 5
    done
    LOG "WIFI RESTORE FAILED — ghost needs a power-cycle"
    return 1
}
trap restore_wifi EXIT

# ---- stack control (mirrors the real launch script's tab1/tab2) ------------
cat > /home/ghost/gdbg_tab1.sh <<WEOF
docker exec -it -u admin $CONTAINER bash -c "$DSRC && ros2 launch nvblox_examples_bringup realsense_example.launch.py run_rviz:=False; ec=\\\$?; echo; echo ===== process exited code \\\$ec =====; exec bash"
WEOF
cat > /home/ghost/gdbg_tab2.sh <<WEOF
docker exec -it -u admin $CONTAINER bash -c "$DSRC && ros2 launch px4_offboard vio_bridge.launch.py; ec=\\\$?; echo; echo ===== process exited code \\\$ec =====; exec bash"
WEOF

launch_stack(){
    tmux new-session -d -s gdbg_tab1 'bash /home/ghost/gdbg_tab1.sh'
    sleep 20
    tmux new-session -d -s gdbg_tab2 'bash /home/ghost/gdbg_tab2.sh'
}
kill_stack(){
    tmux kill-session -t gdbg_tab1 2>/dev/null
    tmux kill-session -t gdbg_tab2 2>/dev/null
    docker exec -u admin $CONTAINER bash -c "pkill -f MicroXRCEAgent; pkill -f vio_bridge; pkill -f component_container; pkill -f realsense; pkill -f 'ros2 launch'; true" 2>/dev/null
    sleep 6
}

CDIAG(){ docker exec -u admin "$CONTAINER" bash -c "{ $DSRC ; } >/dev/null 2>&1; $1" 2>&1; }

capture(){
    local D="$1"
    mkdir -p "$D"
    LOG "capturing bundle -> $D"
    { date; uptime; uname -a; } > "$D/meta.txt"
    # --- Jetson host ---
    rfkill list                          > "$D/rfkill.txt" 2>&1
    ip -br addr                          > "$D/ip_addr.txt" 2>&1
    iw dev "$WIFI_IF" link               > "$D/iw_link.txt" 2>&1
    iw dev "$WIFI_IF" get power_save     > "$D/iw_power_save.txt" 2>&1
    iw reg get                           > "$D/iw_reg.txt" 2>&1
    nmcli dev show "$WIFI_IF"            > "$D/nmcli_dev.txt" 2>&1
    cat /proc/interrupts                 > "$D/interrupts_t0.txt"
    sleep 5
    cat /proc/interrupts                 > "$D/interrupts_t1.txt"
    top -bn1 | head -40                  > "$D/top.txt"
    lsusb -t                             > "$D/lsusb_t.txt" 2>&1
    SUDO dmesg --ctime 2>/dev/null | tail -150            > "$D/dmesg_tail.txt"
    SUDO journalctl -u NetworkManager --since '20 min ago' --no-pager 2>/dev/null | tail -200 > "$D/journal_nm.txt"
    SUDO ss -tulpn                       > "$D/ss_tulpn.txt" 2>&1
    ls -la /dev/shm                      > "$D/dev_shm.txt" 2>&1
    ( SUDO timeout 6 tegrastats --interval 1000 2>/dev/null | head -6 ) > "$D/tegrastats.txt"
    # --- container (env sourced exactly as the launch script does) ---
    CDIAG "env | grep -E 'ROS_|RMW_|FASTRTPS|CYCLONE'"                       > "$D/c_env.txt"
    CDIAG "timeout 10 ros2 node list"                                        > "$D/c_node_list.txt"
    CDIAG "timeout 10 ros2 topic list"                                       > "$D/c_topic_list.txt"
    CDIAG "stdbuf -oL timeout 12 ros2 topic hz /camera0/imu"                 > "$D/c_hz_camera_imu.txt"
    CDIAG "stdbuf -oL timeout 12 ros2 topic hz /visual_slam/tracking/odometry" > "$D/c_hz_vslam_odom.txt"
    CDIAG "stdbuf -oL timeout 12 ros2 topic hz /fmu/in/vehicle_visual_odometry" > "$D/c_hz_fmu_in_vvo.txt"
    CDIAG "timeout 8 ros2 topic echo --once /fmu/out/vehicle_status"         > "$D/c_fmu_vehicle_status.txt"
    CDIAG "timeout 8 ros2 topic echo --once /fmu/out/vehicle_local_position" > "$D/c_fmu_local_position.txt"
    CDIAG "pgrep -af 'MicroXRCEAgent|vio|component_container|realsense'"     > "$D/c_pgrep.txt"
    # --- full tmux scrollback (Lost IMU warnings, cuVSLAM errors, agent logs) ---
    tmux capture-pane -pS -5000 -t gdbg_tab1 > "$D/tab1_pane.txt" 2>&1
    tmux capture-pane -pS -5000 -t gdbg_tab2 > "$D/tab2_pane.txt" 2>&1
    LOG "bundle done: $D"
}

# ============================ MAIN ===========================================
rm -rf "$OUT"; mkdir -p "$OUT"
LOG "run01 starting. wifi_if=$WIFI_IF active_conn='$ACTIVE_CONN'"
echo "$ACTIVE_CONN" > "$OUT/active_conn.txt"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    docker start "$CONTAINER" || { LOG "FATAL: container missing"; exit 1; }
    sleep 5
fi

# ---------- PHASE A: wifi ON (broken state) ----------
LOG "PHASE A: wifi ON"
kill_stack
launch_stack
LOG "waiting 75s for stack startup"
sleep 75
capture "$OUT/A_wifi_on"
kill_stack

# ---------- PHASE B: radio OFF (working state) ----------
LOG "PHASE B: rfkill block wifi"
SUDO rfkill block wifi
sleep 10
rfkill list > "$OUT/B_rfkill_confirm.txt" 2>&1
CDIAG "ros2 daemon stop" >/dev/null 2>&1   # don't let CLI daemon cache stale ifaces
launch_stack
LOG "waiting 75s for stack startup"
sleep 75
capture "$OUT/B_radio_off"
kill_stack

# ---------- PHASE C: restore wifi, post-capture extras ----------
restore_wifi
# Router band survey for H-A3 (done now so the scan can't pollute A/B captures)
SUDO iw dev "$WIFI_IF" scan 2>/dev/null | grep -iE '^BSS|SSID:|freq:|signal:' > "$OUT/wifi_scan.txt"
iw dev "$WIFI_IF" link > "$OUT/post_iw_link.txt" 2>&1

rm -f /home/ghost/gdbg_tab1.sh /home/ghost/gdbg_tab2.sh
cp /home/ghost/gdbg_run01_nohup.log "$OUT/jetson_nohup.log" 2>/dev/null
echo "completed $(date)" > "$OUT/DONE"
LOG "ALL DONE"
JEOF

SCPQ "$TMPF" "$GHOST:/home/ghost/gdbg_run01_jetson.sh"
rm -f "$TMPF"

echo "[run01] 4/6 starting detached A/B capture on ghost (survives radio-off) ..."
SSHQ 'chmod +x /home/ghost/gdbg_run01_jetson.sh; nohup setsid bash /home/ghost/gdbg_run01_jetson.sh > /home/ghost/gdbg_run01_nohup.log 2>&1 < /dev/null & echo "started, pid $!"'

echo "[run01] 5/6 polling for completion (~12-15 min; SSH failures mid-run are NORMAL) ..."
DEADLINE=$((SECONDS + 1500))
DONE=0
while [ $SECONDS -lt $DEADLINE ]; do
    sleep 20
    if SSHQ 'test -f /home/ghost/ghost_debug_run01/DONE' 2>/dev/null; then
        DONE=1; break
    fi
    echo "  ... waiting ($(( (DEADLINE - SECONDS) / 60 )) min budget left)"
done

if [ "$DONE" != 1 ]; then
    echo "[run01] FAIL: timed out waiting for DONE marker."
    echo "  If ghost answers ping: ssh in and check /home/ghost/gdbg_run01_nohup.log"
    echo "  If ghost is unreachable: POWER-CYCLE ghost, then fetch with:"
    echo "  sshpass -p $PASS scp -r -o StrictHostKeyChecking=no $GHOST:$REMOTE_OUT $RUNDIR/data"
    exit 1
fi

echo "[run01] 6/6 fetching bundles ..."
rm -rf "$RUNDIR/data"
SCPQ -r "$GHOST:$REMOTE_OUT" "$RUNDIR/data"

echo ""
echo "=== QUICK LOOK ==="
for ph in A_wifi_on B_radio_off; do
    echo "--- $ph ---"
    for f in c_hz_camera_imu c_hz_vslam_odom c_hz_fmu_in_vvo; do
        echo "  [$f]: $(grep -m1 'average rate' "$RUNDIR/data/$ph/$f.txt" 2>/dev/null || echo 'NO RATE OUTPUT')"
    done
    echo "  [Lost IMU in tab1]: $(grep -c 'Lost IMU' "$RUNDIR/data/$ph/tab1_pane.txt" 2>/dev/null || echo 0) occurrences"
done
echo ""

if [ -s "$RUNDIR/data/A_wifi_on/c_topic_list.txt" ] && [ -s "$RUNDIR/data/B_radio_off/c_topic_list.txt" ]; then
    echo "[run01] PASS — both bundles captured. Saved to: $RUNDIR/data/"
else
    echo "[run01] FAIL — one or both bundles incomplete. Saved what exists to: $RUNDIR/data/"
    exit 1
fi
