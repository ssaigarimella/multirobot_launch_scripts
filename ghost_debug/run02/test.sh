#!/bin/bash
# run02: DDS domain-mismatch confirm + fix validation. WiFi stays ON — no rfkill,
# no risk of losing ghost. ~6 min total, live output.
# Run from laptop on TP-Link_073A:  bash ghost_debug/run02/test.sh
set -u

GHOST="ghost@192.168.0.50"
PASS="abc123"
RUNDIR="$(cd "$(dirname "$0")" && pwd)"
CONTAINER=isaac_ros_realsense
ROUT=/home/ghost/ghost_debug_run02
# Exact DOCKER_SOURCE from the tmux launch script (domain 3) and the fix candidate (no domain)
DSRC3='export ROS_DOMAIN_ID=3 ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1 && source /opt/ros/humble/setup.bash && cd /workspaces/isaac_ros-dev && source install/setup.bash'
DSRC0='export ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1 && source /opt/ros/humble/setup.bash && cd /workspaces/isaac_ros-dev && source install/setup.bash'

SSHQ() { sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$GHOST" "$@"; }

echo "[run02] connectivity check ..."
SSHQ 'echo connected' || { echo "FAIL: cannot reach ghost (on TP-Link?)"; exit 1; }

# Hard kill: bracket-trick patterns so pkill can't match its own shell; verify dead.
HARDKILL='tmux kill-server 2>/dev/null;
docker exec -u admin isaac_ros_realsense bash -c "
  pkill -9 -f \"MicroXRCEAgen[t]\"; pkill -9 -f \"vio_bridg[e]\";
  pkill -9 -f \"component_containe[r]\"; pkill -9 -f \"realsense_exampl[e]\";
  pkill -9 -f \"ros2 launc[h]\"; pkill -9 -f \"vio_bridge.launch.p[y]\"; true" 2>/dev/null
for i in 1 2 3 4 5 6 7 8 9 10; do
  N=$(docker exec -u admin isaac_ros_realsense bash -c "pgrep -cf \"MicroXRCEAgen[t]|vio_bridg[e]|component_containe[r]|ros2 launc[h]\"" 2>/dev/null)
  [ "${N:-0}" = "0" ] && break
  sleep 2
done
echo "leftover-procs:${N:-?}"'

echo "[run02] hard-killing any old stack ..."
SSHQ "$HARDKILL"
SSHQ "rm -rf $ROUT && mkdir -p $ROUT
docker ps --format '{{.Names}}' | grep -qx $CONTAINER || docker start $CONTAINER
cat > /home/ghost/gdbg2_tab1_d3.sh <<WEOF
docker exec -it -u admin $CONTAINER bash -c \"$DSRC3 && ros2 launch nvblox_examples_bringup realsense_example.launch.py run_rviz:=False; exec bash\"
WEOF
cat > /home/ghost/gdbg2_tab2_d3.sh <<WEOF
docker exec -it -u admin $CONTAINER bash -c \"$DSRC3 && ros2 launch px4_offboard vio_bridge.launch.py; exec bash\"
WEOF
cat > /home/ghost/gdbg2_tab1_d0.sh <<WEOF
docker exec -it -u admin $CONTAINER bash -c \"$DSRC0 && ros2 launch nvblox_examples_bringup realsense_example.launch.py run_rviz:=False; exec bash\"
WEOF
cat > /home/ghost/gdbg2_tab2_d0.sh <<WEOF
docker exec -it -u admin $CONTAINER bash -c \"$DSRC0 && ros2 launch px4_offboard vio_bridge.launch.py; exec bash\"
WEOF
echo wrappers-ready"

CDIAG3() { SSHQ "docker exec -u admin $CONTAINER bash -c '{ $DSRC3 ; } >/dev/null 2>&1; $1' 2>&1"; }
CDIAG0() { SSHQ "docker exec -u admin $CONTAINER bash -c '{ $DSRC0 ; } >/dev/null 2>&1; $1' 2>&1"; }

# ---------------- PHASE 1: stack on domain 3 (exact tmux-script env), peek from domain 0
echo "[run02] PHASE 1: launching stack with ROS_DOMAIN_ID=3 (as the real script does) ..."
SSHQ "tmux new-session -d -s gdbg2_tab1 'bash /home/ghost/gdbg2_tab1_d3.sh'; sleep 20; tmux new-session -d -s gdbg2_tab2 'bash /home/ghost/gdbg2_tab2_d3.sh'"
echo "[run02] waiting 75s for startup ..."
sleep 75

mkdir -p "$RUNDIR/data"
echo "[run02] domain-3 view of /fmu topics:"
CDIAG3 "ros2 daemon stop >/dev/null 2>&1; sleep 2; timeout 15 ros2 topic list | grep /fmu || echo NO_FMU_TOPICS_DOMAIN3" | tee "$RUNDIR/data/p1_domain3_fmu_topics.txt"
echo "[run02] domain-0 view of /fmu topics (PREDICTION: /fmu/out/* visible here):"
CDIAG0 "ros2 daemon stop >/dev/null 2>&1; sleep 2; timeout 15 ros2 topic list | grep /fmu || echo NO_FMU_TOPICS_DOMAIN0" | tee "$RUNDIR/data/p1_domain0_fmu_topics.txt"
CDIAG0 "timeout 8 ros2 topic echo --once /fmu/out/vehicle_status 2>&1 | head -15" | tee "$RUNDIR/data/p1_domain0_vehicle_status.txt"

# ---------------- PHASE 2: relaunch on domain 0 (fix candidate), full goal checks
echo "[run02] PHASE 2: hard-kill, relaunch WITHOUT ROS_DOMAIN_ID (domain 0) ..."
SSHQ "$HARDKILL"
SSHQ "tmux new-session -d -s gdbg2_tab1 'bash /home/ghost/gdbg2_tab1_d0.sh'; sleep 20; tmux new-session -d -s gdbg2_tab2 'bash /home/ghost/gdbg2_tab2_d0.sh'"
echo "[run02] waiting 75s for startup ..."
sleep 75

echo "[run02] goal checks on domain 0 (wifi is ON right now):"
CDIAG0 "ros2 daemon stop >/dev/null 2>&1; sleep 2; timeout 15 ros2 topic list | grep /fmu || echo NO_FMU_TOPICS" | tee "$RUNDIR/data/p2_fmu_topics.txt"
CDIAG0 "stdbuf -oL timeout 12 ros2 topic hz /fmu/in/vehicle_visual_odometry 2>&1 | tail -5" | tee "$RUNDIR/data/p2_hz_vvo.txt"
CDIAG0 "stdbuf -oL timeout 12 ros2 topic hz /camera0/imu 2>&1 | tail -3" | tee "$RUNDIR/data/p2_hz_imu.txt"
CDIAG0 "timeout 8 ros2 topic echo --once /fmu/out/vehicle_status 2>&1 | head -20" | tee "$RUNDIR/data/p2_vehicle_status.txt"
CDIAG0 "timeout 8 ros2 topic echo --once /fmu/out/vehicle_local_position 2>&1 | grep -E \"xy_valid|z_valid|v_xy_valid|v_z_valid|xy_global|z_global|^x:|^y:|^z:\" || echo NO_LOCAL_POSITION" | tee "$RUNDIR/data/p2_local_position_flags.txt"
SSHQ "tmux capture-pane -pS -2000 -t gdbg2_tab2" > "$RUNDIR/data/p2_tab2_pane.txt" 2>&1
SSHQ "tmux capture-pane -pS -2000 -t gdbg2_tab1" > "$RUNDIR/data/p2_tab1_pane.txt" 2>&1

# Leave the domain-0 stack RUNNING so the human can check POSITION mode in QGC right now.
SSHQ "rm -f /home/ghost/gdbg2_tab1_d3.sh /home/ghost/gdbg2_tab2_d3.sh"

echo ""
echo "================= VERDICT ================="
P1=$(grep -c '/fmu/out' "$RUNDIR/data/p1_domain0_fmu_topics.txt" 2>/dev/null || echo 0)
P2T=$(grep -c '/fmu/out' "$RUNDIR/data/p2_fmu_topics.txt" 2>/dev/null || echo 0)
P2V=$(grep -m1 'average rate' "$RUNDIR/data/p2_hz_vvo.txt" 2>/dev/null || echo none)
echo "Phase1 /fmu/out topics visible on domain 0 (stack on domain 3): $P1  (>0 => mismatch CONFIRMED)"
echo "Phase2 /fmu/out topics on domain 0 stack: $P2T"
echo "Phase2 VVO rate: $P2V"
echo "Phase2 local-position validity flags:"
cat "$RUNDIR/data/p2_local_position_flags.txt" 2>/dev/null
echo ""
if [ "$P2T" -gt 0 ]; then
    echo "[run02] PASS — domain fix validated. Stack is STILL RUNNING on ghost (domain 0):"
    echo "        check QGC NOW: can you select POSITION mode with valid local position?"
else
    echo "[run02] FAIL — see $RUNDIR/data/ ; stack left running for inspection."
fi
echo "Files: $RUNDIR/data/"
