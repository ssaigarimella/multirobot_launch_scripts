#!/bin/bash
# drone_build_sequence.sh — THE canonical colcon build sequence for the isaac_ros-dev
# workspace. Runs INSIDE the isaac_ros_realsense container (as admin).
# These exact commands/order are required — deviating breaks the ROS2 system.
# Deployed to /workspaces/isaac_ros-dev/ on each drone by build_all_drones.sh.

set -e
source /opt/ros/humble/setup.bash
cd /workspaces/isaac_ros-dev

step() { echo ""; echo "================ STEP $1: $2 ================"; }

step 1 "realsense packages"
colcon build --symlink-install \
--packages-up-to-regex 'realsense.*' \
--cmake-args -DBUILD_TESTING=OFF \
--allow-overriding isaac_ros_common

step 2 "isaac_ros_nvblox"
colcon build --symlink-install \
--packages-up-to isaac_ros_nvblox \
--cmake-args -DBUILD_TESTING=OFF \
--allow-overriding isaac_ros_common isaac_ros_test

step 3 "isaac_ros_visual_slam"
colcon build --symlink-install \
--packages-up-to isaac_ros_visual_slam \
--cmake-args -DBUILD_TESTING=OFF \
--allow-overriding isaac_ros_common isaac_ros_test

step 4 "multi_drone_nvblox"
colcon build --packages-select multi_drone_nvblox --symlink-install

step 5 "active_exploration"
colcon build --packages-select active_exploration --symlink-install

step 6 "px4_msgs"
colcon build --packages-select px4_msgs

step 7 "px4_offboard"
colcon build --packages-select px4_offboard

step 8 "radiohive"
colcon build --packages-select radiohive --symlink-install

echo ""
echo "================ ALL 8 STEPS SUCCEEDED ================"
