#!/bin/bash
# launch_local.sh - Run a drone's exploration stack in a tmux session ON THE JETSON.
#
# Run this AFTER you SSH into the Jetson. It starts every component in its own
# tmux window inside a DETACHED session, so closing the SSH connection (or losing
# WiFi) does NOT kill the stack -- the tmux server keeps it running on the Jetson.
#
# Usage (on the Jetson):
#   ./launch_local.sh <drone_id>            # 1=delta 2=buckshee 3=ghost 4=thunderstrike
#   ./launch_local.sh 3 --rviz --debug      # rviz on, skip arm check
#   ./launch_local.sh 3 --vio               # VIO only (cuVSLAM + vio bridge)
#
# Then:
#   tmux attach -t swarm     # watch it (Ctrl-b n/p switch windows, Ctrl-b d detach)
#   tmux kill-session -t swarm   # stop the whole stack

CONTAINER="isaac_ros_realsense"
SESSION="swarm"
SETUP="export ROS_LOCALHOST_ONLY=0 && source /opt/ros/humble/setup.bash && cd /workspaces/isaac_ros-dev && source install/setup.bash"

ID="${1:?usage: ./launch_local.sh <drone_id> [--rviz] [--debug] [--vio]}"
RVIZ="False"; DEBUG="false"; VIO="0"
for a in "$@"; do
    case "$a" in
        --rviz)  RVIZ="True" ;;
        --debug) DEBUG="true" ;;
        --vio)   VIO="1" ;;
    esac
done

command -v tmux >/dev/null || { echo "tmux not installed. Run: sudo apt install -y tmux"; exit 1; }
docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || docker start "$CONTAINER" >/dev/null 2>&1 \
    || { echo "Container '$CONTAINER' is not running and could not be started."; exit 1; }

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session ':$SESSION' is already running. Attach: tmux attach -t $SESSION"
    echo "Or stop it first: tmux kill-session -t $SESSION"
    exit 0
fi

# Each component runs in its own tmux window; 'exec bash' keeps the window open
# after a crash so you can read the error / restart it by hand.
win() { tmux new-window -t "$SESSION" -d -n "$1" \
    "docker exec -it -u admin $CONTAINER bash -lc '$SETUP && $2; exec bash'"; }

tmux new-session -d -s "$SESSION" -n shell    # plain container-less host shell

# Staged startup: cuVSLAM/RealSense must come up before VIO consumes its odom.
win cuvslam "ros2 launch nvblox_examples_bringup realsense_example.launch.py run_rviz:=$RVIZ"; sleep 10
win vio     "ros2 launch px4_offboard vio_bridge.launch.py"; sleep 4
if [ "$VIO" != "1" ]; then
    win fis     "ros2 launch active_exploration fis.launch.py flight_height:=1.0"; sleep 3
    win guard   "ros2 launch active_exploration reactive_guard.launch.py"; sleep 3
    win planner "ros2 launch active_exploration simple_planner_launch.py flight_height:=1.0 debug_skip_arm_check:=$DEBUG"; sleep 3
    win loop    "python3 src/multi_drone_nvblox/scripts/loop_closure_sp_node.py --ros-args -p robot_namespace:=drone1 -p vocabulary_file:=/workspaces/isaac_ros-dev/src/multi_drone_nvblox/models/sp_vocab_4096.pkl -p process_rate:=1.0 -p use_pcm:=True -p publish_tf:=True"; sleep 3
    win octomap "ros2 run multi_drone_nvblox octomap_exchange_node --ros-args -p robot_namespace:=drone1 -p drone_id:=$ID -p resolution:=0.05 -p alignment_config:=/workspaces/isaac_ros-dev/src/multi_drone_nvblox/config/swarm_alignment.yaml -p use_sim_time:=False"
fi

echo "[OK] stack launched in tmux session ':$SESSION' on $(hostname)."
echo "     watch it : tmux attach -t $SESSION"
echo "     stop it  : tmux kill-session -t $SESSION"
