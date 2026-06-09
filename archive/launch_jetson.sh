#!/bin/bash
# launch_jetson.sh - Opens 3 gnome-terminal tabs for PX4 offboard control (Jetson/Docker)
#
# Usage:
#   ./launch_jetson.sh            # All 3 terminals
#   ./launch_jetson.sh nomonitor  # Only terminals 1 & 2 (skip monitor)

REMOTE="delta@10.90.134.66"
PASS="abc123"
CONTAINER="isaac_ros_realsense"
DOCKER_EXEC="docker exec -it $CONTAINER bash -c"
SETUP="cd /workspaces/isaac_ros-dev && source install/setup.bash"
SCRIPT="$(readlink -f "$0")"

do_ssh() {
    sshpass -p "$PASS" ssh -t -o StrictHostKeyChecking=no "$REMOTE" "$1"
}

case "$1" in
    --tab1)
        do_ssh "$DOCKER_EXEC 'echo \">>> MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600\" && echo \"Press Enter to launch...\" && read && MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600'"
        read -rp "Session ended. Press Enter to close."
        ;;
    --tab2)
        do_ssh "$DOCKER_EXEC '$SETUP && echo \">>> ros2 run px4_offboard offboard_control.py\" && echo \"Press Enter to launch...\" && read && ros2 run px4_offboard offboard_control.py'"
        read -rp "Session ended. Press Enter to close."
        ;;
    --tab3)
        do_ssh "$DOCKER_EXEC '$SETUP && echo \">>> ros2 topic echo /fmu/out/vehicle_local_position\" && echo \"Press Enter to launch...\" && read && ros2 topic echo /fmu/out/vehicle_local_position'"
        read -rp "Session ended. Press Enter to close."
        ;;
    *)
        MODE="${1:-all}"

        if ! command -v sshpass &>/dev/null; then
            echo "sshpass required: sudo apt install sshpass"
            exit 1
        fi

        # Open all tabs in the current window
        gnome-terminal --tab --title="1: XRCE-DDS" -- "$SCRIPT" --tab1
        sleep 0.3
        gnome-terminal --tab --title="2: Offboard" -- "$SCRIPT" --tab2
        sleep 0.3
        if [ "$MODE" != "nomonitor" ]; then
            gnome-terminal --tab --title="3: Monitor" -- "$SCRIPT" --tab3
        fi
        ;;
esac
