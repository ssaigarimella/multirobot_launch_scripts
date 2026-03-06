#!/bin/bash
# launch_delta_jetson_TPLink.sh - Opens gnome-terminal tabs for the Delta Jetson VIO stack
#
# Run this from your LAPTOP (not the Jetson).
# It SSHes into the Jetson and runs cuVSLAM + VIO bridge inside the Isaac ROS container.
#
# Usage:
#   ./launch_delta_jetson_TPLink.sh              # VIO + simple mission (default)
#   ./launch_delta_jetson_TPLink.sh figure8      # VIO + figure-8 mission
#   ./launch_delta_jetson_TPLink.sh vio          # VIO only (no mission)

JETSON_HOST="delta@192.168.0.59"
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
        # Tab 2: VIO bridge (inside Docker)
        echo "=== Tab 2: VIO Bridge ==="
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
        MISSION="$2"
        if [ "$MISSION" = "vio" ]; then
            echo "=== VIO-only mode, no mission tab needed ==="
            read -rp "Press Enter to close."
            exit 0
        fi
        if [ "$MISSION" = "figure8" ]; then
            NODE="figure8_mission"
            echo "=== Tab 3: Figure-8 Mission ==="
        else
            NODE="simple_offboard_mission"
            echo "=== Tab 3: Simple Offboard Mission ==="
        fi
        echo "Waiting for VIO stack to stabilize..."
        sleep 8
        echo "Press Enter to start mission node..."
        read
        do_ssh "
            docker exec -it -u admin $CONTAINER bash -c '$DOCKER_SOURCE && \
                echo \"Starting $NODE...\" && \
                ros2 run px4_offboard $NODE'
        "
        read -rp "Session ended. Press Enter to close."
        ;;
    # --tab-relay)
    #     # Domain relay (on Jetson host, NOT in Docker)
    #     # Relays /visual_slam/tracking/odometry from domain 0 → domain 44
    #     echo "=== Tab: Domain Relay (0 → 44) ==="
    #     echo "Waiting for cuVSLAM to start..."
    #     sleep 8
    #     echo "Press Enter to start relay..."
    #     read
    #     do_ssh "
    #         source /opt/ros/humble/setup.bash && \
    #         python3 /tmp/odom_domain_relay.py
    #     "
    #     read -rp "Session ended. Press Enter to close."
    #     ;;
    -h|--help|help)
        echo "Usage:"
        echo "  ./launch_delta_jetson_TPLink.sh              # Simple mission (default)"
        echo "  ./launch_delta_jetson_TPLink.sh figure8      # Figure-8 mission"
        echo "  ./launch_delta_jetson_TPLink.sh vio          # VIO only (no mission node)"
        exit 0
        ;;
    *)
        # Default: set up container, then open gnome-terminal tabs
        MISSION="${1:-simple}"

        if ! command -v sshpass &>/dev/null; then
            echo "sshpass required: sudo apt install sshpass"
            exit 1
        fi

        case "$MISSION" in
            figure8)  T3_TITLE="3: Figure-8 Mission" ;;
            vio)      T3_TITLE="3: (VIO only)" ;;
            *)        T3_TITLE="3: Simple Mission" ;;
        esac

        echo "============================================================"
        echo " Delta Jetson VIO Launch (from laptop)"
        echo " Mission: $MISSION"
        echo " Remote:  $JETSON_HOST"
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
        echo "Tab 2: VIO Bridge"
        echo "Tab 3: $T3_TITLE"
        echo ""
        echo "Press Enter in each tab to launch that component."
        echo "============================================================"

        gnome-terminal --tab --title="1: cuVSLAM" -- "$SCRIPT" --tab1
        sleep 0.3
        gnome-terminal --tab --title="2: VIO Bridge" -- "$SCRIPT" --tab2
        sleep 0.3
        gnome-terminal --tab --title="$T3_TITLE" -- "$SCRIPT" --tab3 "$MISSION"
        ;;
esac
