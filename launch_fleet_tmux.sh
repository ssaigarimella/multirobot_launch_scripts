#!/bin/bash
# launch_fleet_tmux.sh - multi-drone exploration WITH coordination, in the
# same multi-tab style as launch_<drone>_frontierexplore_tmux.sh:
#
#     ./launch_fleet_tmux.sh                       ghost + delta (default)
#     ./launch_fleet_tmux.sh ghost delta --debug   skip arm check (hand-carry)
#     ./launch_fleet_tmux.sh all
#     ./launch_fleet_tmux.sh --headless ghost delta   no tabs, all detached
#     ./launch_fleet_tmux.sh attach ghost          re-attach drone's tab session
#     ./launch_fleet_tmux.sh stop [all]            tear everything down
#
# One terminal WINDOW per drone, with the familiar tabs (press Enter in each,
# in order):
#   1: cuVSLAM + RealSense + nvblox        (self map, collision)
#   2: VIO Bridge + DDS Agent              (odom -> PX4)
#   3: FIS (frontier detection)
#   4: Reactive Depth Guard
#   5: Exploration Planner (team geofence: 35x35 forward-only)
#   6: Shared mapper (2nd nvblox, peers' keyframes)   [multi-drone]
#   7: Coordination (alignment + LoRa claims + keyframe sharing) [multi-drone]
#   8: Zenoh bridge (host side, bench LAN)            [multi-drone]
#
# Peer-map integration uses the yaml prior (trust-prior is ON for fleet
# flights; measure placement into swarm_alignment.yaml first!).
#
# Every tab runs in a named tmux session ON THE DRONE: an SSH drop detaches
# instead of killing the node, re-running the tab re-attaches. Ctrl-b d
# detaches; NEVER Ctrl-C. All pane output is logged on the drone to
# experiment_logs/pane_<sess>_<epoch>.log; collect with ./fleet_ctl collect.

set -u

HERE="$(dirname "$(readlink -f "$0")")"
SCRIPT="$(readlink -f "$0")"
CONTAINER=isaac_ros_realsense
CWS=/workspaces/isaac_ros-dev
WS=/mnt/nova_ssd/workspaces/isaac_ros-dev
YAML=$CWS/src/multi_drone_nvblox/config/swarm_alignment.yaml
declare -A IPS=( [delta]=192.168.0.40 [buckshee]=192.168.0.60 \
                 [ghost]=192.168.0.50 [thunderstrike]=192.168.0.70 )
declare -A IDS=( [delta]=1 [buckshee]=2 [ghost]=3 [thunderstrike]=4 )

# ------------------------------------------------------------- tab child
# --tab <drone> <n> [--debug] [--ekf2] [--drones=a,b,...]
if [ "${1:-}" = "--tab" ]; then
    DRONE="$2"; N="$3"; shift 3
    source "$HERE/credentials.env"
    ID="${IDS[$DRONE]}"; IP="${IPS[$DRONE]}"
    DEBUG=false; POSE=cuvslam; CSV="$DRONE"
    for a in "$@"; do case "$a" in
        --debug) DEBUG=true ;;
        --ekf2)  POSE=ekf2 ;;
        --drones=*) CSV="${a#--drones=}" ;;
    esac; done

    case "$N" in
      1) LABEL="cuVSLAM + RealSense + nvblox"
         CMD="ros2 launch nvblox_examples_bringup realsense_example.launch.py run_rviz:=False pose_source:=$POSE" ;;
      2) LABEL="VIO Bridge + DDS Agent"
         CMD="ros2 launch px4_offboard vio_bridge.launch.py" ;;
      3) LABEL="FIS (frontier detection)"
         CMD="ros2 launch active_exploration fis.launch.py flight_height:=1.0" ;;
      4) LABEL="Reactive Depth Guard"
         CMD="ros2 launch active_exploration reactive_guard.launch.py" ;;
      5) LABEL="Exploration Planner (team geofence)"
         CMD="ros2 launch multi_drone_nvblox planner_stage.launch.py drone_id:=$ID alignment_yaml:=$YAML debug_skip_arm_check:=$DEBUG" ;;
      6) LABEL="Shared mapper (2nd nvblox)"
         CMD="ros2 launch multi_drone_nvblox shared_mapper.launch.py own_depth_topic:=/camera0/depth/image_rect_raw own_camera_info_topic:=/camera0/depth/camera_info global_frame:=odom" ;;
      7) LABEL="Coordination (alignment + LoRa + keyframes)"
         CMD="ros2 launch multi_drone_nvblox coordination_stage.launch.py drone_id:=$ID alignment_yaml:=$YAML trust_prior_alignment:=true" ;;
      8) LABEL="Zenoh bridge (host, bench LAN)" ;;
      *) echo "unknown tab $N"; exit 1 ;;
    esac

    SESS="fleet_tab$N"
    echo "=== $DRONE (d$ID) - Tab $N: $LABEL ==="
    echo "(tmux '$SESS' on the drone - survives SSH drops; re-run to re-attach)"
    read -rp "Press Enter to launch... "
    LOGDIR=$WS/experiment_logs

    if [ "$N" = "8" ]; then
        # host-side zenoh over the existing LAN (no WiFi changes). Foreground
        # inside its tmux so the output is visible in this tab.
        PEERS=""
        IFS=, read -ra ALLD <<< "$CSV"
        for p in "${ALLD[@]}"; do
            [ "$p" = "$DRONE" ] || PEERS+=" ${IDS[$p]}:${IPS[$p]}"
        done
        if [ -z "$PEERS" ]; then echo "single drone: no bridge needed."; read; exit 0; fi
        REMOTE="mkdir -p $LOGDIR; tmux has-session -t $SESS 2>/dev/null || tmux new-session -d -x 220 -y 50 -s $SESS \"echo $JETSON_PASS | sudo -S -p '' env ROS_DOMAIN_ID=$ID bash $WS/src/multi_drone_nvblox/scripts/launch_bench_zenoh.sh $ID $IP$PEERS; exec bash\"; tmux pipe-pane -o -t $SESS \"cat >> $LOGDIR/pane_${SESS}_\$(date +%s).log\"; tmux attach-session -t $SESS"
    else
        REMOTE="docker ps --format '{{.Names}}' | grep -qx $CONTAINER || docker start $CONTAINER >/dev/null 2>&1; sleep 1; mkdir -p $LOGDIR; tmux has-session -t $SESS 2>/dev/null || tmux new-session -d -x 220 -y 50 -s $SESS \"docker exec -it -u admin $CONTAINER bash -c 'source /opt/ros/humble/setup.bash && source $CWS/install/setup.bash && export ROS_DOMAIN_ID=$ID ROS_LOCALHOST_ONLY=0 && $CMD; ec=\\\$?; echo; echo ===== exited code \\\$ec - pane kept open, Ctrl-b d to detach =====; exec bash'\"; tmux pipe-pane -o -t $SESS \"cat >> $LOGDIR/pane_${SESS}_\$(date +%s).log\"; tmux attach-session -t $SESS"
    fi

    sshpass -p "$JETSON_PASS" ssh -t \
        -o StrictHostKeyChecking=no \
        -o ServerAliveInterval=15 -o ServerAliveCountMax=4 \
        "$DRONE@$IP" "$REMOTE"
    echo ""
    echo "[Tab $N detached or exited. Press Enter to close.]"
    read
    exit 0
fi

# ------------------------------------------------------------- parent
case "${1:-}" in
    attach)
        source "$HERE/credentials.env"
        d="${2:?usage: attach <drone> [tabN]}"
        t="${3:-fleet_tab1}"
        exec sshpass -p "$JETSON_PASS" ssh -t -o StrictHostKeyChecking=no \
            "$d@${IPS[$d]}" "tmux attach -t $t"
        ;;
    stop)
        shift; exec "$HERE/fleet_ctl" stop "${@:-all}"
        ;;
    -h|--help|help)
        sed -n '2,30p' "$SCRIPT" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
esac

HEADLESS=0; DRONES=(); FLAGS=()
for a in "$@"; do
    case "$a" in
        --headless) HEADLESS=1 ;;
        all) DRONES=(delta buckshee ghost thunderstrike) ;;
        delta|buckshee|ghost|thunderstrike) DRONES+=("$a") ;;
        *) FLAGS+=("$a") ;;
    esac
done
[ ${#DRONES[@]} -eq 0 ] && DRONES=(ghost delta)
CSV=$(IFS=,; echo "${DRONES[*]}")

if [ "$HEADLESS" = "1" ]; then
    exec "$HERE/fleet_ctl" start "${DRONES[@]}" --trust-prior ${FLAGS[@]+"${FLAGS[@]}"}
fi
command -v sshpass >/dev/null || { echo "need sshpass: sudo apt install sshpass"; exit 1; }
command -v gnome-terminal >/dev/null || { echo "no gnome-terminal; use --headless"; exit 1; }

echo "============================================================"
echo " Multi-drone exploration WITH coordination"
echo " Drones: ${DRONES[*]}    Flags: ${FLAGS[*]:-(none)}"
echo " One window per drone; press Enter in each tab in order 1..8."
echo " Preflight after tabs are up:  ./fleet_ctl preflight ${DRONES[*]}"
echo " Teardown:  ./launch_fleet_tmux.sh stop"
echo "============================================================"

TITLES=("" "1: cuVSLAM+nvblox" "2: VIO + DDS" "3: FIS" "4: Depth Guard" \
        "5: Planner" "6: Shared mapper" "7: Coordination" "8: Zenoh")
for d in "${DRONES[@]}"; do
    # first tab opens the drone's window; the rest attach as tabs to it
    gnome-terminal --window --title="$d ${TITLES[1]}" -- \
        bash "$SCRIPT" --tab "$d" 1 "--drones=$CSV" ${FLAGS[@]+"${FLAGS[@]}"}
    sleep 0.6
    for n in 2 3 4 5 6 7 8; do
        gnome-terminal --tab --title="$d ${TITLES[$n]}" -- \
            bash "$SCRIPT" --tab "$d" "$n" "--drones=$CSV" ${FLAGS[@]+"${FLAGS[@]}"}
        sleep 0.25
    done
done
