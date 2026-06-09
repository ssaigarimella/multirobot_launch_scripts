#!/bin/bash
# run_delta.sh — Launch all delta tabs automatically, no Enter needed.
# Usage: ./run_delta.sh [--rviz]

CONTAINER="isaac_ros_realsense"
SETUP='export ROS_LOCALHOST_ONLY=0 && export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/src/multi_drone_nvblox/config/fastdds_loopback.xml && source /opt/ros/humble/setup.bash && cd /workspaces/isaac_ros-dev && source install/setup.bash'
RVIZ="False"
[ "$1" = "--rviz" ] && RVIZ="True"

log() { echo "[$(date +%H:%M:%S)] $*"; }

# Kill stale
log "Killing stale processes..."
docker exec -u admin $CONTAINER bash -c 'pkill -9 -f ros2; pkill -9 -f python3; true' 2>/dev/null
sleep 1

docker ps --format '{{.Names}}' | grep -qx $CONTAINER || docker start $CONTAINER

run() {
    local name="$1"; local cmd="$2"; local logfile="$3"
    log "  $name"
    docker exec -d -u admin $CONTAINER bash -c "$SETUP && $cmd 2>&1 | tee $logfile"
}

run "Tab 1: cuVSLAM + nvblox (rviz=$RVIZ)" \
    "ros2 launch nvblox_examples_bringup realsense_example.launch.py run_rviz:=$RVIZ" \
    /tmp/delta_tab1.log
sleep 3

run "Tab 2: VIO Bridge" \
    "sudo chmod 666 /dev/ttyUSB0 2>/dev/null; ros2 launch px4_offboard vio_bridge.launch.py" \
    /tmp/delta_tab2.log
sleep 1

run "Tab 3: FIS" \
    "ros2 launch active_exploration fis.launch.py flight_height:=1.0" \
    /tmp/delta_tab3.log
sleep 1

run "Tab 4: Depth Guard" \
    "ros2 launch active_exploration reactive_guard.launch.py" \
    /tmp/delta_tab4.log
sleep 1

run "Tab 5: Planner" \
    "ros2 launch active_exploration simple_planner_launch.py flight_height:=1.0 debug_skip_arm_check:=true" \
    /tmp/delta_tab5.log
sleep 1

run "Tab 6: Loop Closure" \
    "python3 src/multi_drone_nvblox/scripts/loop_closure_sp_node.py --ros-args -p robot_namespace:=drone1 -p vocabulary_file:=/workspaces/isaac_ros-dev/src/multi_drone_nvblox/models/sp_vocab_4096.pkl -p process_rate:=1.0 -p use_pcm:=True -p publish_tf:=True" \
    /tmp/delta_tab6.log
sleep 1

run "Tab 7: OctoMap Exchange" \
    "ros2 run multi_drone_nvblox octomap_exchange_node --ros-args -p robot_namespace:=drone1 -p drone_id:=1 -p resolution:=0.05 -p alignment_config:=/workspaces/isaac_ros-dev/src/multi_drone_nvblox/config/swarm_alignment.yaml -p use_sim_time:=False" \
    /tmp/delta_tab7.log

log "All Docker tabs launched."
log "Starting Zenoh bridge (mesh switch)..."
echo ""

# Tab 8: Zenoh — runs in foreground, Ctrl+C restores WiFi
sudo bash /mnt/nova_ssd/workspaces/isaac_ros-dev/launch_mesh_zenoh.sh 1 3
