#!/bin/bash
# launch_mission.sh - Opens 4 gnome-terminal tabs for the offboard mission
#
# Usage:
#   ./launch_mission.sh           # Terminal 4 runs simple_offboard_mission
#   ./launch_mission.sh figure8   # Terminal 4 runs figure8_mission

REMOTE="khadas@10.90.178.137"
PASS="edge2_123"
WS="autonomous_flying/catkin_ws"
SETUP="source ~/$WS/devel/setup.bash"
SCRIPT="$(readlink -f "$0")"

do_ssh() {
    sshpass -p "$PASS" ssh -t -o StrictHostKeyChecking=no "$REMOTE" "$1"
}

case "$1" in
    --tab1)
        do_ssh "cd ~/$WS && $SETUP && echo '>>> roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600' && echo 'Press Enter to launch...' && read && roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600"
        read -rp "Session ended. Press Enter to close."
        ;;
    --tab2)
        do_ssh "cd ~/$WS && $SETUP && echo '>>> roslaunch realsense2_camera rs_t265.launch' && echo 'Press Enter to launch...' && read && roslaunch realsense2_camera rs_t265.launch"
        read -rp "Session ended. Press Enter to close."
        ;;
    --tab3)
        do_ssh "cd ~/$WS && $SETUP && echo '>>> rosrun t265_vio_bridge vio_node.py' && echo 'Press Enter to launch...' && read && rosrun t265_vio_bridge vio_node.py"
        read -rp "Session ended. Press Enter to close."
        ;;
    --tab4)
        MISSION="$2"
        if [ "$MISSION" = "figure8" ]; then
            CMD="rosrun t265_vio_bridge figure8_mission"
        else
            CMD="rosrun t265_vio_bridge simple_offboard_mission"
        fi
        do_ssh "cd ~/$WS && $SETUP && echo '>>> $CMD' && echo 'Press Enter to launch...' && read && $CMD"
        read -rp "Session ended. Press Enter to close."
        ;;
    *)
        MISSION="${1:-offboard}"
        if [ "$MISSION" = "figure8" ]; then
            T4_TITLE="4: Figure-8"
        else
            T4_TITLE="4: Offboard"
        fi

        if ! command -v sshpass &>/dev/null; then
            echo "sshpass required: sudo apt install sshpass"
            exit 1
        fi

        # Open all tabs in the current window
        gnome-terminal --tab --title="1: MAVROS" -- "$SCRIPT" --tab1
        sleep 0.3
        gnome-terminal --tab --title="2: T265" -- "$SCRIPT" --tab2
        sleep 0.3
        gnome-terminal --tab --title="3: VIO" -- "$SCRIPT" --tab3
        sleep 0.3
        gnome-terminal --tab --title="$T4_TITLE" -- "$SCRIPT" --tab4 "$MISSION"
        ;;
esac
