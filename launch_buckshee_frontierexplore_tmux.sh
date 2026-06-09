#!/bin/bash
# launch_buckshee_jetson.sh - Opens gnome-terminal tabs for the Buckshee Jetson exploration stack
#
# Run this from your LAPTOP (not the Jetson).
# It SSHes into the Jetson and runs the full stack inside the Isaac ROS container.
#
# tmux-hardened variant: every remote command runs inside a NAMED tmux session on
# the Jetson. If the SSH link drops (e.g. iPhone hotspot sleeping/throttling), the
# ROS nodes keep running on the Jetson instead of getting SIGHUP'd. Re-running a tab
# re-attaches to the live session. Requires tmux on the Jetson (sudo apt install tmux).
#
# Usage:
#   ./launch_buckshee_jetson.sh              # Full exploration stack (default)
#   ./launch_buckshee_jetson.sh explore      # Same as above
#   ./launch_buckshee_jetson.sh vio          # VIO only (no exploration)
#   ./launch_buckshee_jetson.sh kill         # Kill all tmux sessions on the Jetson

# eduroam
# JETSON_HOST="buckshee@10.90.179.174"

# IFL TPLINK router
JETSON_HOST="buckshee@192.168.0.60"

# iPhone
# JETSON_HOST="buckshee@172.20.10.10"


JETSON_PASS="abc123"
CONTAINER="isaac_ros_realsense"
IMAGE="isaac_ros:dev-realsense"
SCRIPT="$(readlink -f "$0")"
# DOCKER_SOURCE="export ROS_LOCALHOST_ONLY=0 && export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/src/multi_drone_nvblox/config/fastdds_loopback.xml && source /opt/ros/humble/setup.bash && cd /workspaces/isaac_ros-dev && source install/setup.bash"
# NOTE: PX4 param UXRCE_DDS_DOM_ID must EQUAL this ROS_DOMAIN_ID (set via QGC +
# FMU reboot; ghost=3 was done 2026-06-09 — VERIFY/SET buckshee=2 in QGC before flying!). If they differ, PX4's /fmu/* topics
# land on a different DDS domain and EKF2 silently gets no VIO (see ghost_debug/POSTMORTEM.md).
DOCKER_SOURCE="export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=0 && source /opt/ros/humble/setup.bash && cd /workspaces/isaac_ros-dev && source install/setup.bash"


# CLI flags: --rviz (enable RViz2), --debug (skip arm check)
RVIZ_FLAG="False"
DEBUG_ARM="false"
export DEBUG_ARM
for arg in "$@"; do
    if [ "$arg" = "--rviz" ]; then
        RVIZ_FLAG="True"
    elif [ "$arg" = "--debug" ]; then
        DEBUG_ARM="true"
        export DEBUG_ARM
    fi
done

# SSH with keepalives so brief stalls on a flaky link don't trigger a disconnect.
do_ssh() {
    sshpass -p "$JETSON_PASS" ssh -t \
        -o StrictHostKeyChecking=no \
        -o ServerAliveInterval=15 \
        -o ServerAliveCountMax=4 \
        "$JETSON_HOST" "$1"
}

# Helper for SSH docker tabs.
# Runs the command inside a named tmux session on the Jetson:
#   - SSH drops  -> tmux detaches, the node keeps running on the Jetson.
#   - re-run tab -> "new-session -A" re-attaches to the live session.
ssh_docker_tab() {
    local label="$1"
    local cmd="$2"
    local sess="${label//[^a-zA-Z0-9]/_}"   # tmux-safe session name (no spaces/colons)
    echo "=== $label ==="
    echo "(tmux session '$sess' on Jetson — survives SSH drops; re-run this tab to re-attach)"
    echo "Press Enter to launch..."
    read
    # Append '; exec bash' so the tmux pane STAYS OPEN if the command crashes —
    # otherwise tmux destroys the window on exit and you never see the error.
    do_ssh "tmux new-session -A -s $sess \"docker exec -it -u admin $CONTAINER bash -c '$DOCKER_SOURCE && $cmd; ec=\$?; echo; echo ===== process exited code \$ec - pane kept open, Ctrl-b d to detach =====; exec bash'\""
    echo ""
    echo "[$label detached or exited. Press Enter to close tab.]"
    read
}

case "$1" in
    --tab1)
        ssh_docker_tab "Tab 1: cuVSLAM + RealSense + nvblox" \
            "ros2 launch nvblox_examples_bringup realsense_example.launch.py run_rviz:=${RVIZ_FLAG}"
        ;;
    --tab2)
        ssh_docker_tab "Tab 2: VIO Bridge + DDS Agent" \
            "ros2 launch px4_offboard vio_bridge.launch.py"
        ;;
    --tab3)
        ssh_docker_tab "Tab 3: Frontier Info Structure (FIS)" \
            "ros2 launch active_exploration fis.launch.py flight_height:=1.0"
        ;;
    --tab4)
        ssh_docker_tab "Tab 4: Reactive Depth Guard" \
            "ros2 launch active_exploration reactive_guard.launch.py"
        ;;
    --tab5)
        # Check if --debug was passed as a subsequent arg
        DEBUG_VAL="false"
        for a in "$@"; do [ "$a" = "--debug" ] && DEBUG_VAL="true"; done
        ssh_docker_tab "Tab 5: Simple Exploration Planner" \
            "ros2 launch active_exploration simple_planner_launch.py flight_height:=1.0 debug_skip_arm_check:=${DEBUG_VAL}"
        ;;
    --tab6)
        ssh_docker_tab "Tab 6: Loop Closure (SuperPoint + LightGlue)" \
            "python3 src/multi_drone_nvblox/scripts/loop_closure_sp_node.py --ros-args \
                -p robot_namespace:=drone2 \
                -p vocabulary_file:=/workspaces/isaac_ros-dev/src/multi_drone_nvblox/models/sp_vocab_4096.pkl \
                -p process_rate:=1.0 \
                -p use_pcm:=True \
                -p publish_tf:=True"
        ;;
    --tab7)
        ssh_docker_tab "Tab 7: OctoMap Exchange (incremental)" \
            "ros2 run multi_drone_nvblox octomap_exchange_node --ros-args \
                -p robot_namespace:=drone2 \
                -p drone_id:=2 \
                -p resolution:=0.05 \
                -p alignment_config:=/workspaces/isaac_ros-dev/src/multi_drone_nvblox/config/swarm_alignment.yaml \
                -p use_sim_time:=False"
        ;;
    --tab8)
        # Tab 8 runs on the Jetson HOST (not Docker) — it manages WiFi + Zenoh.
        # Wrapped in tmux because switching WiFi to the mesh DROPS this SSH link by design.
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
        echo "Running mesh+zenoh on Jetson via SSH (tmux session 'bk_mesh_zenoh')..."
        do_ssh "tmux new-session -A -s bk_mesh_zenoh \"sudo bash /mnt/nova_ssd/workspaces/isaac_ros-dev/launch_mesh_zenoh.sh $NODE_ID $PEER_IDS\""
        echo ""
        echo "[Tab 8 detached or exited. Press Enter to close tab.]"
        read
        ;;
    --tab9)
        ssh_docker_tab "Tab 9: Interactive Shell" \
            "bash"
        ;;
    kill|--kill)
        echo "Killing all tmux sessions on $JETSON_HOST ..."
        do_ssh "tmux kill-server 2>/dev/null; echo '[OK] tmux sessions cleared.'"
        exit 0
        ;;
    -h|--help|help)
        echo "Usage:"
        echo "  ./launch_buckshee_jetson.sh [--rviz] [--debug]"
        echo "  ./launch_buckshee_jetson.sh explore      # Full exploration stack (default)"
        echo "  ./launch_buckshee_jetson.sh vio          # VIO only (no exploration)"
        echo "  ./launch_buckshee_jetson.sh kill         # Kill all tmux sessions on the Jetson"
        echo ""
        echo "Options:"
        echo "  --rviz    Launch RViz2 with the cuVSLAM/nvblox pipeline (Tab 1)"
        echo "  --debug   Skip arm check in exploration manager (for hand-carry testing)"
        echo ""
        echo "Tabs:"
        echo "  1: cuVSLAM + RealSense + nvblox"
        echo "  2: VIO Bridge + DDS Agent (odom -> PX4)"
        echo "  3: FIS (frontier detection + viewpoints)"
        echo "  4: Reactive Depth Guard"
        echo "  5: Exploration Manager (planner + offboard control)"
        echo "  6: Loop Closure (SuperPoint + LightGlue)"
        echo "  7: OctoMap Exchange (incremental)"
        echo "  8: Mesh + Zenoh Bridge (switches to 802.11s, runs on Jetson HOST)"
        echo "  9: Interactive Shell (Docker + ROS2 sourced)"
        echo ""
        echo "tmux: every tab runs in a named tmux session on the Jetson, so an SSH"
        echo "      drop detaches instead of killing the node. Re-run a tab to re-attach."
        echo "      Tear everything down with: ./launch_buckshee_jetson.sh kill"
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

        # Ensure tmux is installed on the Jetson host (needed to survive SSH drops).
        sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_HOST" \
            "command -v tmux >/dev/null || sudo apt-get install -y tmux"

        # Ensure container is running (reuse existing, don't destroy)
        sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_HOST" bash -s <<SETUP_EOF
if docker ps --format '{{.Names}}' | grep -qx $CONTAINER; then
    echo '[OK] Container $CONTAINER is already running.'
elif docker ps -a --format '{{.Names}}' | grep -qx $CONTAINER; then
    echo '[INFO] Container $CONTAINER exists but stopped. Starting...'
    docker start $CONTAINER
else
    echo '[INFO] Creating container $CONTAINER...'
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
fi
SETUP_EOF

        # Verify container is running before opening tabs
        if ! sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_HOST" \
            "docker ps --format '{{.Names}}' | grep -qx $CONTAINER"; then
            echo "ERROR: Container '$CONTAINER' is not running. Aborting."
            exit 1
        fi

        if [ "$MISSION" = "vio" ]; then
            echo ""
            echo "Tab 1: cuVSLAM + RealSense + nvblox"
            echo "Tab 2: VIO Bridge + DDS Agent"
            echo ""
            echo "VIO-only mode: skipping exploration tabs."
            echo "Press Enter in each tab to launch that component."
            echo "============================================================"

            gnome-terminal --tab --title="1: cuVSLAM"    -- bash "$SCRIPT" --tab1
            sleep 0.3
            gnome-terminal --tab --title="2: VIO + DDS"  -- bash "$SCRIPT" --tab2
        else
            echo ""
            echo " Tab 1: cuVSLAM + RealSense + nvblox"
            echo " Tab 2: VIO Bridge + DDS Agent"
            echo " Tab 3: FIS (frontier detection)"
            echo " Tab 4: Reactive Depth Guard"
            echo " Tab 5: Exploration Manager"
            echo " Tab 6: Loop Closure (SuperPoint + LightGlue)"
            echo " Tab 7: OctoMap Exchange (incremental)"
            echo " Tab 8: Mesh + Zenoh (Jetson HOST, requires sudo)"
            echo " Tab 9: Interactive Shell"
            echo "============================================================"
            echo "Press Enter in each tab to launch that component."
            echo ""

            RVIZ_ARG=""
            DEBUG_ARG=""
            [ "$RVIZ_FLAG" = "True" ] && RVIZ_ARG="--rviz"
            [ "$DEBUG_ARM" = "true" ] && DEBUG_ARG="--debug"
            gnome-terminal --tab --title="1: cuVSLAM"      -- bash "$SCRIPT" --tab1 $RVIZ_ARG
            sleep 0.3
            gnome-terminal --tab --title="2: VIO + DDS"    -- bash "$SCRIPT" --tab2
            sleep 0.3
            gnome-terminal --tab --title="3: FIS"          -- bash "$SCRIPT" --tab3
            sleep 0.3
            gnome-terminal --tab --title="4: Depth Guard"  -- bash "$SCRIPT" --tab4
            sleep 0.3
            gnome-terminal --tab --title="5: Explorer"     -- bash "$SCRIPT" --tab5 $DEBUG_ARG
            sleep 0.3
            gnome-terminal --tab --title="6: Loop Closure" -- bash "$SCRIPT" --tab6
            sleep 0.3
            gnome-terminal --tab --title="7: OctoMap"      -- bash "$SCRIPT" --tab7
            sleep 0.3
            gnome-terminal --tab --title="8: Zenoh"        -- bash "$SCRIPT" --tab8
            sleep 0.3
            gnome-terminal --tab --title="9: Shell"        -- bash "$SCRIPT" --tab9
        fi
        ;;
esac
