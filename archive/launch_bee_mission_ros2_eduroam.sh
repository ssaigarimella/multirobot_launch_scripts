#!/bin/bash
# launch_mission_ros2.sh - Opens gnome-terminal tabs for the ROS2 offboard mission
#
# Run this from your LAPTOP (not the Khadas).
# It SSHes into the Khadas and runs the ROS2 Docker stack.
#
# Usage:
#   ./launch_mission_ros2.sh           # Tab 3 runs simple_offboard_mission
#   ./launch_mission_ros2.sh figure8   # Tab 3 runs figure8_mission
#   ./launch_mission_ros2.sh vio       # VIO stack only, no mission

REMOTE="khadas@10.90.178.137"
PASS="edge2_123"
AF="autonomous_flying"
ROS2_WS="$AF/ros2_ws"
IMAGE="ros2_vio_drone:latest"
SERIAL="/dev/ttyUSB0"
BAUD="921600"
SCRIPT="$(readlink -f "$0")"
CONTAINER="vio_drone_ros2"

# Docker run base command (shared across tabs)
DOCKER_BASE="docker run -d --rm --privileged --network=host -v /dev/bus/usb:/dev/bus/usb -v ~/$ROS2_WS:/ros2_ws --name $CONTAINER $IMAGE bash -c"
DOCKER_EXEC="docker exec -it $CONTAINER bash -c"
DOCKER_SOURCE="source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash"

do_ssh() {
    sshpass -p "$PASS" ssh -t -o StrictHostKeyChecking=no "$REMOTE" "$1"
}

case "$1" in
    --tab1)
        # Tab 1: Start Docker container + DDS agent + T265 + VIO bridge
        echo "=== Tab 1: VIO Stack (Docker) ==="
        echo "Starting Docker container and launching VIO stack..."
        echo "Press Enter to launch..."
        read
        do_ssh "
            cd ~/$AF && \
            docker stop $CONTAINER 2>/dev/null; \
            docker rm $CONTAINER 2>/dev/null; \
            docker run -it --rm \
                --privileged \
                --network=host \
                -v /dev/bus/usb:/dev/bus/usb \
                -v ~/$ROS2_WS:/ros2_ws \
                --name $CONTAINER \
                $IMAGE \
                bash -c '$DOCKER_SOURCE && \
                    ros2 launch vio_drone vio.launch.py serial_device:=$SERIAL baud:=$BAUD'
        "
        read -rp "Session ended. Press Enter to close."
        ;;
    --tab2)
        # Tab 2: Monitor — topic echo and diagnostics
        echo "=== Tab 2: Monitor ==="
        echo "Waiting for container to start..."
        sleep 5
        echo "Press Enter to start monitoring..."
        read
        do_ssh "
            docker exec -it $CONTAINER bash -c '$DOCKER_SOURCE && \
                echo \"=== VIO Topic Rate ===\" && \
                timeout 5 ros2 topic hz /fmu/in/vehicle_visual_odometry 2>&1; \
                echo \"\" && \
                echo \"=== Latest VIO Message ===\" && \
                timeout 3 ros2 topic echo /fmu/in/vehicle_visual_odometry --once 2>&1; \
                echo \"\" && \
                echo \"=== PX4 Vehicle Status ===\" && \
                timeout 3 ros2 topic echo /fmu/out/vehicle_status --once --field nav_state --field arming_state 2>&1; \
                echo \"\" && \
                echo \"--- Continuous VIO rate (Ctrl+C to stop) ---\" && \
                ros2 topic hz /fmu/in/vehicle_visual_odometry'
        "
        read -rp "Session ended. Press Enter to close."
        ;;
    --tab3)
        MISSION="$2"
        LOOPS="${3:-1}"
        SPEED="${4:-0.3}"
        if [ "$MISSION" = "vio" ]; then
            echo "=== VIO-only mode, no mission tab needed ==="
            read -rp "Press Enter to close."
            exit 0
        fi
        if [ "$MISSION" = "figure8" ]; then
            NODE="figure8_mission"
            ROS_ARGS=""
            echo "=== Tab 3: Figure-8 Mission ==="
        elif [ "$MISSION" = "lemniscate" ]; then
            NODE="lemniscate_mission"
            ROS_ARGS="--ros-args -p loops:=$LOOPS -p speed:=$SPEED"
            echo "=== Tab 3: Lemniscate Mission (loops=$LOOPS, speed=${SPEED} m/s) ==="
        else
            NODE="simple_offboard_mission"
            ROS_ARGS=""
            echo "=== Tab 3: Simple Offboard Mission ==="
        fi
        echo "Waiting for VIO stack to stabilize..."
        sleep 8
        echo "Press Enter to start mission node..."
        read
        do_ssh "
            docker exec -it $CONTAINER bash -c '$DOCKER_SOURCE && \
                echo \"Starting $NODE...\" && \
                ros2 run vio_drone $NODE $ROS_ARGS'
        "
        read -rp "Session ended. Press Enter to close."
        ;;
    -h|--help|help)
        echo "Usage:"
        echo "  ./launch_mission_ros2.sh                       # Simple mission (default)"
        echo "  ./launch_mission_ros2.sh figure8                # Figure-8 mission"
        echo "  ./launch_mission_ros2.sh lemniscate [loops] [speed]   # Smooth figure-8 (default: 1 loop, 0.3 m/s)"
        echo "  ./launch_mission_ros2.sh vio                    # VIO only (no mission node)"
        exit 0
        ;;
    *)
        # Default: open gnome-terminal tabs
        MISSION="${1:-simple}"
        LOOPS="${2:-1}"
        SPEED="${3:-0.3}"

        if ! command -v sshpass &>/dev/null; then
            echo "sshpass required: sudo apt install sshpass"
            exit 1
        fi

        case "$MISSION" in
            figure8)     T3_TITLE="3: Figure-8 Mission" ;;
            lemniscate)  T3_TITLE="3: Lemniscate Mission" ;;
            vio)         T3_TITLE="3: (VIO only)" ;;
            *)           T3_TITLE="3: Simple Mission" ;;
        esac

        echo "============================================================"
        echo " ROS2 Flight Launch (from laptop)"
        echo " Mission: $MISSION"
        echo " Remote:  $REMOTE"
        echo "============================================================"
        echo ""
        echo "Tab 1: VIO Stack (DDS + T265 + bridge)"
        echo "Tab 2: Monitor (topic rates + diagnostics)"
        echo "Tab 3: $T3_TITLE"
        echo ""
        echo "Press Enter in each tab to launch that component."
        echo "============================================================"

        gnome-terminal --tab --title="1: VIO Stack" -- "$SCRIPT" --tab1
        sleep 0.3
        gnome-terminal --tab --title="2: Monitor" -- "$SCRIPT" --tab2
        sleep 0.3
        gnome-terminal --tab --title="$T3_TITLE" -- "$SCRIPT" --tab3 "$MISSION" "$LOOPS" "$SPEED"
        ;;
esac
