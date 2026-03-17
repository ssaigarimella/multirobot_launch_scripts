#!/bin/bash
# launch_delta_local.sh - Opens gnome-terminal tabs on the Jetson itself (not via SSH)
# Each tab enters the Docker container with ROS2 sourced, then waits for Enter.
#
# Usage: ./launch_delta_local.sh

CONTAINER="isaac_ros_realsense"
IMAGE="isaac_ros:dev-realsense"
SCRIPT="$(readlink -f "$0")"
SETUP="export ROS_LOCALHOST_ONLY=0 && source /opt/ros/humble/setup.bash && cd /workspaces/isaac_ros-dev && source install/setup.bash"

# Ensure Docker container is running
ensure_container() {
    if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
        echo "[OK] Container '$CONTAINER' is already running."
    elif docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
        echo "[INFO] Container '$CONTAINER' exists but stopped. Starting..."
        docker start "$CONTAINER"
    else
        echo "[INFO] Creating container '$CONTAINER'..."
        docker run -dit \
            --privileged \
            --network host \
            --ipc host \
            -v /mnt/nova_ssd/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
            -v /mnt/nova_ssd/workspaces/Micro-XRCE-DDS-Agent:/workspaces/Micro-XRCE-DDS-Agent \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v "$HOME/.Xauthority:/home/admin/.Xauthority:rw" \
            -e DISPLAY \
            -e NVIDIA_VISIBLE_DEVICES=all \
            -e NVIDIA_DRIVER_CAPABILITIES=all \
            --runtime nvidia \
            --name "$CONTAINER" \
            "$IMAGE" \
            /bin/bash
    fi

    if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
        echo "[ERROR] Container '$CONTAINER' is not running. Aborting."
        exit 1
    fi
}

# Run a command inside the container with ROS2 sourced, wait for Enter first
docker_tab() {
    local label="$1"
    local cmd="$2"
    echo "=== $label ==="
    echo "Press Enter to launch..."
    read
    docker exec -it -u admin "$CONTAINER" bash -c "$SETUP && $cmd"
    echo ""
    echo "[$label exited. Press Enter to close tab.]"
    read
}

case "$1" in
    --tab1)
        docker_tab "Tab 1: cuVSLAM + RealSense + nvblox" \
            "ros2 launch nvblox_examples_bringup realsense_example.launch.py run_rviz:=False"
        ;;
    --tab2)
        docker_tab "Tab 2: VIO Bridge + DDS Agent" \
            "ros2 launch px4_offboard vio_bridge.launch.py"
        ;;
    --tab3)
        docker_tab "Tab 3: Frontier Info Structure (FIS)" \
            "ros2 launch active_exploration fis.launch.py flight_height:=1.0"
        ;;
    --tab4)
        docker_tab "Tab 4: Reactive Depth Guard" \
            "ros2 launch active_exploration reactive_guard.launch.py"
        ;;
    --tab5)
        docker_tab "Tab 5: Exploration Manager" \
            "ros2 launch active_exploration exploration_manager.launch.py flight_height:=1.0"
        ;;
    --tab6)
        docker_tab "Tab 6: Loop Closure (SuperPoint + LightGlue)" \
            "python3 src/multi_drone_nvblox/scripts/loop_closure_sp_node.py --ros-args \
                -p robot_namespace:=drone1 \
                -p vocabulary_file:=/workspaces/isaac_ros-dev/src/multi_drone_nvblox/models/sp_vocab_4096.pkl \
                -p process_rate:=1.0 \
                -p use_pcm:=True \
                -p publish_tf:=True"
        ;;
    --tab7)
        docker_tab "Tab 7: OctoMap Exchange (incremental)" \
            "ros2 run multi_drone_nvblox octomap_exchange_node --ros-args \
                -p robot_namespace:=drone1 \
                -p resolution:=0.05 \
                -p use_sim_time:=False"
        ;;
    --tab8)
        # Tab 8 runs on the HOST (not Docker) — it manages WiFi + Zenoh
        echo "=== Tab 8: Mesh + Zenoh Bridge ==="
        echo ""
        echo "This switches WiFi from eduroam to 802.11s mesh,"
        echo "starts Zenoh to bridge SLAM topics, and restores"
        echo "eduroam when you press Ctrl+C."
        echo ""
        echo "Node IDs: 1=delta, 2=buckshee, 3=ghost, 4=thunderstrike"
        echo ""
        read -rp "This drone's ID: " NODE_ID
        read -rp "Peer drone IDs (space-separated, e.g. '3' or '2 3 4'): " PEER_IDS
        echo ""
        echo "Running: sudo bash /mnt/nova_ssd/workspaces/isaac_ros-dev/launch_mesh_zenoh.sh $NODE_ID $PEER_IDS"
        echo ""
        sudo bash /mnt/nova_ssd/workspaces/isaac_ros-dev/launch_mesh_zenoh.sh "$NODE_ID" $PEER_IDS
        echo ""
        echo "[Tab 8 exited. Press Enter to close tab.]"
        read
        ;;
    --tab9)
        docker_tab "Tab 9: Interactive Shell" \
            "bash"
        ;;
    -h|--help)
        echo "Usage: ./launch_delta_local.sh"
        echo ""
        echo "Opens 9 gnome-terminal tabs, each inside Docker with ROS2 sourced."
        echo "Press Enter in each tab to launch that component."
        echo ""
        echo "Tabs:"
        echo "  1: cuVSLAM + RealSense + nvblox"
        echo "  2: VIO Bridge + DDS Agent"
        echo "  3: FIS (frontier detection)"
        echo "  4: Reactive Depth Guard"
        echo "  5: Exploration Manager"
        echo "  6: Loop Closure (SuperPoint + LightGlue)"
        echo "  7: OctoMap Exchange (incremental)"
        echo "  8: Mesh + Zenoh Bridge (switches to 802.11s, runs on HOST)"
        echo "  9: Interactive Shell (Docker + ROS2 sourced)"
        exit 0
        ;;
    *)
        # Default: ensure container is running, then open all 9 tabs
        ensure_container
        echo "============================================================"
        echo " Delta Jetson - Local Launch (9 tabs)"
        echo "============================================================"
        echo " Tab 1: cuVSLAM + RealSense + nvblox"
        echo " Tab 2: VIO Bridge + DDS Agent"
        echo " Tab 3: FIS (frontier detection)"
        echo " Tab 4: Reactive Depth Guard"
        echo " Tab 5: Exploration Manager"
        echo " Tab 6: Loop Closure (SuperPoint + LightGlue)"
        echo " Tab 7: OctoMap Exchange (incremental)"
        echo " Tab 8: Mesh + Zenoh (HOST, requires sudo)"
        echo " Tab 9: Interactive Shell"
        echo "============================================================"
        echo "Press Enter in each tab to launch that component."
        echo ""

        gnome-terminal --tab --title="1: cuVSLAM"      -- bash "$SCRIPT" --tab1
        sleep 0.3
        gnome-terminal --tab --title="2: VIO + DDS"    -- bash "$SCRIPT" --tab2
        sleep 0.3
        gnome-terminal --tab --title="3: FIS"          -- bash "$SCRIPT" --tab3
        sleep 0.3
        gnome-terminal --tab --title="4: Depth Guard"  -- bash "$SCRIPT" --tab4
        sleep 0.3
        gnome-terminal --tab --title="5: Explorer"     -- bash "$SCRIPT" --tab5
        sleep 0.3
        gnome-terminal --tab --title="6: Loop Closure" -- bash "$SCRIPT" --tab6
        sleep 0.3
        gnome-terminal --tab --title="7: OctoMap"      -- bash "$SCRIPT" --tab7
        sleep 0.3
        gnome-terminal --tab --title="8: Zenoh"        -- bash "$SCRIPT" --tab8
        sleep 0.3
        gnome-terminal --tab --title="9: Shell"        -- bash "$SCRIPT" --tab9
        ;;
esac
