#!/bin/bash
# rotation_check.sh - Experiment E6 in one command (disarmed, never armed).
#
#   ./experiments/rotation_check.sh ghost
#
# PRECONDITION: tabs 1 and 2 of ./launch_fleet_tmux.sh <drone> are running
# (camera/cuVSLAM/nvblox incl. odom_correction, and vio_bridge). FMU powered
# is fine; the drone must NOT be armed.
# Then: start this, pick the drone up level, walk it FORWARD 2-3 m at a
# constant height, stop, press Ctrl-C. The verdict prints on exit.
HERE="$(dirname "$(readlink -f "$0")")"; ROOT="$(dirname "$HERE")"
source "$ROOT/credentials.env"
declare -A IPS=( [delta]=192.168.0.40 [buckshee]=192.168.0.60 \
                 [ghost]=192.168.0.50 [thunderstrike]=192.168.0.70 )
declare -A IDS=( [delta]=1 [buckshee]=2 [ghost]=3 [thunderstrike]=4 )
D="${1:?usage: $0 <drone>}"
LOG="$ROOT/experiments/results/rotation_check_${D}_$(date +%Y%m%d_%H%M%S).txt"
mkdir -p "$(dirname "$LOG")"
echo "Walk $D FORWARD 2-3 m, LEVEL, at CONSTANT HEIGHT, then Ctrl-C."
echo "(transcript -> $LOG)"
sshpass -p "$JETSON_PASS" ssh -t -o StrictHostKeyChecking=no "$D@${IPS[$D]}" \
  "docker exec -it -u admin isaac_ros_realsense bash -c 'source /opt/ros/humble/setup.bash \
   && source /workspaces/isaac_ros-dev/install/setup.bash \
   && export ROS_DOMAIN_ID=${IDS[$D]} ROS_LOCALHOST_ONLY=0 \
   && ros2 run multi_drone_nvblox rotation_check.py'" | tee "$LOG"
