#!/bin/bash
# run03: validate the permanent fix (PX4 UXRCE_DDS_DOM_ID=3, set via QGC + FMU reboot).
# Launches tab1+tab2 with the REAL script's exact env (ROS_DOMAIN_ID=3), checks goal
# metrics, and LEAVES THE STACK RUNNING so you can check POSITION mode in QGC.
# Run from laptop on TP-Link_073A:  bash ghost_debug/run03/test.sh   (~4 min)
set -u

GHOST="ghost@192.168.0.50"
PASS="abc123"
RUNDIR="$(cd "$(dirname "$0")" && pwd)"
CONTAINER=isaac_ros_realsense
# Exact DOCKER_SOURCE from launch_ghost_jetson_eduroam_python_frontierexplore_tmux.sh
DSRC3='export ROS_DOMAIN_ID=3 ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1 && source /opt/ros/humble/setup.bash && cd /workspaces/isaac_ros-dev && source install/setup.bash'

SSHQ() { sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$GHOST" "$@"; }
CDIAG3() { SSHQ "docker exec -u admin $CONTAINER bash -c '{ $DSRC3 ; } >/dev/null 2>&1; $1' 2>&1"; }

echo "[run03] connectivity check ..."
SSHQ 'echo connected' || { echo "FAIL: cannot reach ghost (on TP-Link?)"; exit 1; }

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

echo "[run03] hard-killing any old stack ..."
SSHQ "$HARDKILL"

echo "[run03] launching tab1+tab2 with the REAL script env (ROS_DOMAIN_ID=3) ..."
SSHQ "docker ps --format '{{.Names}}' | grep -qx $CONTAINER || docker start $CONTAINER
cat > /home/ghost/gdbg3_tab1.sh <<WEOF
docker exec -it -u admin $CONTAINER bash -c \"$DSRC3 && ros2 launch nvblox_examples_bringup realsense_example.launch.py run_rviz:=False; exec bash\"
WEOF
cat > /home/ghost/gdbg3_tab2.sh <<WEOF
docker exec -it -u admin $CONTAINER bash -c \"$DSRC3 && ros2 launch px4_offboard vio_bridge.launch.py; exec bash\"
WEOF
tmux new-session -d -s gdbg3_tab1 'bash /home/ghost/gdbg3_tab1.sh'
sleep 20
tmux new-session -d -s gdbg3_tab2 'bash /home/ghost/gdbg3_tab2.sh'"

echo "[run03] waiting 75s for startup ..."
sleep 75

mkdir -p "$RUNDIR/data"
echo "[run03] /fmu topics on domain 3 (PREDICTION: /fmu/out/* now present):"
CDIAG3 "ros2 daemon stop >/dev/null 2>&1; sleep 2; timeout 15 ros2 topic list | grep /fmu || echo NO_FMU_TOPICS" | tee "$RUNDIR/data/fmu_topics.txt"
echo "[run03] VVO rate:"
CDIAG3 "stdbuf -oL timeout 12 ros2 topic hz /fmu/in/vehicle_visual_odometry 2>&1 | tail -5" | tee "$RUNDIR/data/hz_vvo.txt"
echo "[run03] vehicle_status:"
CDIAG3 "timeout 8 ros2 topic echo --once /fmu/out/vehicle_status 2>&1 | head -20" | tee "$RUNDIR/data/vehicle_status.txt"
echo "[run03] local position validity flags:"
CDIAG3 "timeout 8 ros2 topic echo --once /fmu/out/vehicle_local_position 2>&1 | grep -E \"xy_valid|z_valid|v_xy_valid|v_z_valid|^x:|^y:|^z:\" || echo NO_LOCAL_POSITION" | tee "$RUNDIR/data/local_position_flags.txt"
SSHQ "tmux capture-pane -pS -2000 -t gdbg3_tab2" > "$RUNDIR/data/tab2_pane.txt" 2>&1
SSHQ "tmux capture-pane -pS -2000 -t gdbg3_tab1" > "$RUNDIR/data/tab1_pane.txt" 2>&1

echo ""
echo "================= VERDICT ================="
FOUT=$(grep -c '/fmu/out' "$RUNDIR/data/fmu_topics.txt" 2>/dev/null || echo 0)
VVO=$(grep -m1 'average rate' "$RUNDIR/data/hz_vvo.txt" 2>/dev/null || echo none)
XYV=$(grep -m1 'xy_valid' "$RUNDIR/data/local_position_flags.txt" 2>/dev/null || echo none)
ZV=$(grep -m1 '^z_valid' "$RUNDIR/data/local_position_flags.txt" 2>/dev/null || echo none)
echo "/fmu/out topics: $FOUT   VVO: $VVO"
echo "$XYV ; $ZV"
if [ "$FOUT" -gt 0 ] && echo "$XYV" | grep -qi true; then
    echo "[run03] PASS — fix validated with the UNMODIFIED launch script."
    echo "        Stack is STILL RUNNING: check POSITION mode in QGC now."
else
    echo "[run03] FAIL — verify UXRCE_DDS_DOM_ID=3 was SAVED and the FMU was REBOOTED."
    echo "        Stack left running for inspection. Files: $RUNDIR/data/"
fi
