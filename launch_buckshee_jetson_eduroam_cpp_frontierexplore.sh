#!/bin/bash
# launch_buckshee_jetson.sh - Opens gnome-terminal tabs for the Buckshee Jetson exploration stack
#
# Run this from your LAPTOP (not the Jetson).
# It SSHes into the Jetson and runs the full stack inside the Isaac ROS container.
#
# Usage:
#   ./launch_buckshee_jetson.sh              # Full exploration stack (default)
#   ./launch_buckshee_jetson.sh explore      # Same as above
#   ./launch_buckshee_jetson.sh vio          # VIO only (no exploration)

JETSON_HOST="buckshee@10.90.130.212"
JETSON_PASS="abc123"
CONTAINER="isaac_ros_realsense"
IMAGE="isaac_ros:dev-realsense"
SCRIPT="$(readlink -f "$0")"
DOCKER_SOURCE="cd /workspaces/isaac_ros-dev && source install/setup.bash"

do_ssh() {
    sshpass -p "$JETSON_PASS" ssh -t -o StrictHostKeyChecking=no "$JETSON_HOST" "$1"
}

case "$1" in
    --tab1)
        # Tab 1: cuVSLAM + RealSense + nvblox (inside Docker)
        echo "=== Tab 1: cuVSLAM + RealSense + nvblox ==="
        echo "Press Enter to launch..."
        read
        do_ssh "
            docker exec -it -u admin $CONTAINER bash -c '$DOCKER_SOURCE && \
                ros2 launch nvblox_examples_bringup realsense_example.launch.py run_rviz:=False'
        "
        read -rp "Session ended. Press Enter to close."
        ;;
    --tab2)
        # Tab 2: VIO bridge + MicroXRCE-DDS Agent (inside Docker)
        echo "=== Tab 2: VIO Bridge + DDS Agent ==="
        echo "Waiting for cuVSLAM to start..."
        sleep 5
        echo "Press Enter to launch..."
        read
        do_ssh "
            docker exec -it -u admin $CONTAINER bash -c '$DOCKER_SOURCE && \
                ros2 launch px4_offboard vio_bridge.launch.py'
        "
        read -rp "Session ended. Press Enter to close."
        ;;
    --tab3)
        # Tab 3: FIS - Frontier Info Structure (inside Docker)
        echo "=== Tab 3: Frontier Info Structure (FIS) ==="
        echo "Waiting for nvblox ESDF to be available..."
        sleep 10
        echo "Press Enter to launch..."
        read
        do_ssh "
            docker exec -it -u admin $CONTAINER bash -c '$DOCKER_SOURCE && \
                ros2 launch active_exploration fis.launch.py'
        "
        read -rp "Session ended. Press Enter to close."
        ;;
    --tab4)
        # Tab 4: Exploration Manager (inside Docker)
        echo "=== Tab 4: Exploration Manager ==="
        echo "Waiting for FIS to start publishing frontiers..."
        sleep 15
        echo "Press Enter to launch..."
        read
        do_ssh "
            docker exec -it -u admin $CONTAINER bash -c '$DOCKER_SOURCE && \
                ros2 launch active_exploration exploration_manager.launch.py'
        "
        read -rp "Session ended. Press Enter to close."
        ;;
    -h|--help|help)
        echo "Usage:"
        echo "  ./launch_buckshee_jetson.sh              # Full exploration stack"
        echo "  ./launch_buckshee_jetson.sh explore      # Same as above"
        echo "  ./launch_buckshee_jetson.sh vio          # VIO only (no exploration)"
        echo ""
        echo "Tabs:"
        echo "  1: cuVSLAM + RealSense + nvblox"
        echo "  2: VIO Bridge + DDS Agent (odom -> PX4)"
        echo "  3: FIS (frontier detection + viewpoints)"
        echo "  4: Exploration Manager (planner + offboard control)"
        echo ""
        echo "Workflow:"
        echo "  1. Press Enter in each tab (in order) to launch"
        echo "  2. Take off in position mode"
        echo "  3. Switch to offboard mode -> exploration begins automatically"
        echo "  4. Switch back to position mode at any time to pause"
        exit 0
        ;;
    *)
        # Default: set up container, then open gnome-terminal tabs
        MISSION="${1:-explore}"

        if ! command -v sshpass &>/dev/null; then
            echo "sshpass required: sudo apt install sshpass"
            exit 1
        fi

        echo "============================================================"
        echo " Buckshee Jetson Exploration Stack (from laptop)"
        echo " Mode:   $MISSION"
        echo " Remote: $JETSON_HOST"
        echo "============================================================"
        echo ""
        echo "Setting up container on Jetson..."

        # Remove old container and create fresh one (heredoc so $HOME expands on Jetson)
        sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_HOST" bash -s <<SETUP_EOF
docker rm -f $CONTAINER 2>/dev/null
sleep 1
echo '[INFO] Creating fresh container...'
docker run -dit \
    --privileged \
    --network host \
    --ipc host \
    -v /mnt/nova_ssd/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
    -v /mnt/nova_ssd/workspaces/Micro-XRCE-DDS-Agent:/workspaces/Micro-XRCE-DDS-Agent \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v \$HOME/.Xauthority:/home/admin/.Xauthority:rw \
    -e DISPLAY \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    --runtime nvidia \
    --name $CONTAINER \
    $IMAGE \
    /bin/bash && echo '[OK] Container is running.' || echo '[ERROR] Container failed to start.'
SETUP_EOF

        # Verify container is running before opening tabs
        if ! sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_HOST" \
            "docker ps --format '{{.Names}}' | grep -qx $CONTAINER"; then
            echo "ERROR: Container '$CONTAINER' is not running. Aborting."
            exit 1
        fi

        echo ""
        echo "Tab 1: cuVSLAM + RealSense + nvblox"
        echo "Tab 2: VIO Bridge + DDS Agent"

        if [ "$MISSION" = "vio" ]; then
            echo ""
            echo "VIO-only mode: skipping exploration tabs."
            echo "Press Enter in each tab to launch that component."
            echo "============================================================"

            gnome-terminal --tab --title="1: cuVSLAM" -- "$SCRIPT" --tab1
            sleep 0.3
            gnome-terminal --tab --title="2: VIO + DDS" -- "$SCRIPT" --tab2
        else
            echo "Tab 3: FIS (Frontier Info Structure)"
            echo "Tab 4: Exploration Manager"
            echo ""
            echo "Press Enter in each tab to launch that component."
            echo "============================================================"

            gnome-terminal --tab --title="1: cuVSLAM" -- "$SCRIPT" --tab1
            sleep 0.3
            gnome-terminal --tab --title="2: VIO + DDS" -- "$SCRIPT" --tab2
            sleep 0.3
            gnome-terminal --tab --title="3: FIS" -- "$SCRIPT" --tab3
            sleep 0.3
            gnome-terminal --tab --title="4: Explorer" -- "$SCRIPT" --tab4
        fi
        ;;
esac
