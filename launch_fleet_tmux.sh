#!/bin/bash
# launch_fleet_tmux.sh - multi-drone exploration WITH coordination, in the
# same multi-tab style as launch_<drone>_frontierexplore_tmux.sh:
#
#     ./launch_fleet_tmux.sh                       ghost + delta (default)
#     ./launch_fleet_tmux.sh ghost delta --debug   skip arm check (hand-carry)
#     ./launch_fleet_tmux.sh ghost delta --mesh    transport over 802.11s mesh
#     ./launch_fleet_tmux.sh ghost delta --bench   transport over bench LAN (default)
#     ./launch_fleet_tmux.sh all
#     ./launch_fleet_tmux.sh --headless ghost delta   no tabs, all detached
#     ./launch_fleet_tmux.sh attach ghost          re-attach drone's tab session
#     ./launch_fleet_tmux.sh stop [all]            tear everything down
#     ./launch_fleet_tmux.sh stop --mesh [drones]  restore WiFi first, THEN stop.
#                                                  Use this after a --mesh run:
#                                                  drones on the mesh are not on
#                                                  192.168.0.x, so plain `stop`
#                                                  cannot reach them.
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
#   8: Zenoh bridge (host side) + RF stats             [multi-drone]
#      --bench (default): zenoh over the existing LAN, NO WiFi changes.
#      --mesh           : switch wlP1p1s0 to 802.11s @2437 MHz first. This
#                         DROPS the SSH link by design; the tmux session on the
#                         drone survives it. Guarded - see the mesh preflight.
#
# Stats for the paper are recorded automatically, on the drone, into
# experiment_logs/ (collect with ./fleet_ctl collect):
#   tab 7  metrics_recorder.py -> lora_rx (per LoRa packet: RSSI/SNR), swarm,
#          link (delivery/rx-rate 1 Hz), vio_health, keyframes, coverage
#   tab 8  rf_stats sampler    -> wifi_rf_<host>_<ts>.csv (1 Hz per-peer signal,
#          signal avg, per-chain, tx/rx bitrate, packets, bytes, retries; every
#          row stamped with mode/freq so the CSV proves which link it came from)
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
    DEBUG=false; POSE=cuvslam; CSV="$DRONE"; TRANSPORT=bench; FORCE=0
    for a in "$@"; do case "$a" in
        --debug) DEBUG=true ;;
        --ekf2)  POSE=ekf2 ;;
        --mesh)  TRANSPORT=mesh ;;
        --bench) TRANSPORT=bench ;;
        --force) FORCE=1 ;;
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
      8) if [ "$TRANSPORT" = mesh ]; then LABEL="Zenoh bridge (802.11s mesh @2437) + RF stats"
         else                              LABEL="Zenoh bridge (host, bench LAN) + RF stats"; fi ;;
      *) echo "unknown tab $N"; exit 1 ;;
    esac

    SESS="fleet_tab$N"
    echo "=== $DRONE (d$ID) - Tab $N: $LABEL ==="
    echo "(tmux '$SESS' on the drone - survives SSH drops; re-run to re-attach)"
    read -rp "Press Enter to launch... "
    LOGDIR=$WS/experiment_logs

    if [ "$N" = "8" ]; then
        PEERS=""; MESH_PEER_IDS=""
        IFS=, read -ra ALLD <<< "$CSV"
        for p in "${ALLD[@]}"; do
            [ "$p" = "$DRONE" ] || { PEERS+=" ${IDS[$p]}:${IPS[$p]}"; MESH_PEER_IDS+=" ${IDS[$p]}"; }
        done
        if [ -z "$PEERS" ]; then echo "single drone: no bridge needed."; read; exit 0; fi

        # ---- RF stats: start BEFORE any transport change, so the sampler is
        # already running in its own tmux when a mesh switch drops this SSH.
        # SESS=fleet_rf so `fleet_ctl stop` (which sweeps ^fleet) cleans it up.
        echo "[stats] starting RF sampler on $DRONE (tmux 'fleet_rf' -> experiment_logs/wifi_rf_*.csv)"
        SESS=fleet_rf "$HERE/rf_stats.sh" start "$DRONE" 2>&1 | sed 's/^/  /'

        if [ "$TRANSPORT" = mesh ]; then
            # The drone-side script restores a NetworkManager profile on exit.
            # It resolves one itself, but do not leave it to chance: read the
            # profile that is actually active on wlP1p1s0 now and pass it in via
            # NM_RESTORE_CONNECTION, which the script honours. If we cannot read
            # one there is nothing safe to restore to, so refuse.
            HAVE=$(sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$DRONE@$IP" \
                   "nmcli -t -f NAME,DEVICE con show --active | awk -F: '\$2==\"wlP1p1s0\"{print \$1; exit}'" 2>/dev/null)
            # second path = any IPv4 on a non-wireless iface (survives the switch)
            ALT=$(sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$DRONE@$IP" \
                   'for i in $(ls /sys/class/net); do case $i in lo|docker*|veth*) continue;; esac; [ -d /sys/class/net/$i/wireless ] && continue; ip -4 -br addr show dev $i 2>/dev/null | grep -oP "\\d+\\.\\d+\\.\\d+\\.\\d+" | head -1; done | head -1' 2>/dev/null)

            if [ -z "$HAVE" ]; then
                echo ""
                echo "  REFUSING to switch $DRONE to mesh: cannot read the active"
                echo "  wlP1p1s0 NetworkManager profile, so there is no known-good"
                echo "  network to restore on exit."
                read -rp "Press Enter to close this tab. "; exit 1
            fi
            echo "[mesh] on exit $DRONE will restore: [$HAVE]"
            if [ -n "$ALT" ]; then
                echo "[mesh] $DRONE also has a non-WiFi path at $ALT (survives the switch)"
            else
                echo "[mesh] WARNING: $DRONE has NO second network path. If the restore"
                echo "[mesh] fails it needs a monitor/keyboard. Teardown:"
                echo "[mesh]   ./launch_fleet_tmux.sh stop --mesh ${CSV//,/ }"
            fi
            MESH_SCRIPT="$WS/src/multi_drone_nvblox/scripts/launch_mesh_zenoh_v2.sh $ID$MESH_PEER_IDS"
            REMOTE="mkdir -p $LOGDIR; tmux has-session -t $SESS 2>/dev/null || tmux new-session -d -x 220 -y 50 -s $SESS \"echo $JETSON_PASS | sudo -S -p '' env NM_RESTORE_CONNECTION='$HAVE' ROS_DOMAIN_ID=$ID bash $MESH_SCRIPT; exec bash\"; tmux pipe-pane -o -t $SESS \"cat >> $LOGDIR/pane_${SESS}_\$(date +%s).log\"; tmux attach-session -t $SESS"
        else
            # host-side zenoh over the existing LAN (no WiFi changes). Foreground
            # inside its tmux so the output is visible in this tab.
            REMOTE="mkdir -p $LOGDIR; tmux has-session -t $SESS 2>/dev/null || tmux new-session -d -x 220 -y 50 -s $SESS \"echo $JETSON_PASS | sudo -S -p '' env ROS_DOMAIN_ID=$ID bash $WS/src/multi_drone_nvblox/scripts/launch_bench_zenoh.sh $ID $IP$PEERS; exec bash\"; tmux pipe-pane -o -t $SESS \"cat >> $LOGDIR/pane_${SESS}_\$(date +%s).log\"; tmux attach-session -t $SESS"
        fi
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
        shift
        MESHSTOP=0; SD=()
        for a in "$@"; do case "$a" in
            --mesh) MESHSTOP=1 ;;
            all) SD=(delta buckshee ghost thunderstrike) ;;
            delta|buckshee|ghost|thunderstrike) SD+=("$a") ;;
        esac; done
        [ ${#SD[@]} -eq 0 ] && SD=(ghost delta)

        if [ "$MESHSTOP" = 1 ]; then
            source "$HERE/credentials.env"
            SO="-o StrictHostKeyChecking=no -o LogLevel=ERROR -o ConnectTimeout=8"
            # Drones with a wired path stay reachable through a mesh switch and
            # can act as the jump host for the ones that do not.
            declare -A WIRED=( [ghost]=192.168.0.206 )

            # 1) find a bridge: any drone still answering on the bench LAN
            BR=""; BRA=""
            for d in "${SD[@]}"; do
                for a in "${WIRED[$d]:-}" "${IPS[$d]}"; do
                    [ -z "$a" ] && continue
                    if sshpass -p "$JETSON_PASS" ssh $SO "$d@$a" true 2>/dev/null; then
                        BR="$d"; BRA="$a"; break 2
                    fi
                done
            done
            if [ -z "$BR" ]; then
                echo "[stop] NO drone reachable on the bench LAN."
                echo "[stop] Nothing can be restored remotely - the mesh must be exited"
                echo "[stop] from a console on one of the drones."
                exit 1
            fi
            echo "[stop] bridge drone: $BR @ $BRA"

            # 2) restore every OTHER drone by hopping through the bridge over the
            #    mesh. Both auths run HERE (two local sshpass); the drones need
            #    nothing installed. Ctrl-C fires the script EXIT trap -> restore.
            for d in "${SD[@]}"; do
                [ "$d" = "$BR" ] && continue
                MIP="192.168.77.${IDS[$d]}"
                printf "[stop] %-8s restoring via %s (%s) ... " "$d" "$BR" "$MIP"
                if sshpass -p "$JETSON_PASS" ssh $SO \
                     -o ProxyCommand="sshpass -p $JETSON_PASS ssh $SO -W %h:%p $BR@$BRA" \
                     "$d@$MIP" "tmux send-keys -t fleet_tab8 C-c" 2>/dev/null; then
                    echo "signalled"
                else
                    echo "UNREACHABLE over the mesh - restore $d from its console"
                fi
            done

            # 3) wait for them to reappear on the bench LAN
            echo "[stop] waiting for drones to rejoin the bench LAN (up to 90s) ..."
            for i in $(seq 18); do
                back=1
                for d in "${SD[@]}"; do
                    [ "$d" = "$BR" ] && continue
                    sshpass -p "$JETSON_PASS" ssh $SO "$d@${IPS[$d]}" true 2>/dev/null || back=0
                done
                [ "$back" = 1 ] && break
                sleep 5
            done
            for d in "${SD[@]}"; do
                [ "$d" = "$BR" ] && continue
                printf "[stop] %-8s " "$d"
                sshpass -p "$JETSON_PASS" ssh $SO "$d@${IPS[$d]}" \
                    "iw dev wlP1p1s0 info | grep -oP 'type \\K\\w+|ssid \\K.*' | tr '\\n' ' '" 2>/dev/null \
                    && echo "  <- back on the bench LAN" || echo "STILL NOT on ${IPS[$d]}"
            done

            # 4) finally the bridge itself
            printf "[stop] %-8s restoring (bridge, last) ... " "$BR"
            sshpass -p "$JETSON_PASS" ssh $SO "$BR@$BRA" "tmux send-keys -t fleet_tab8 C-c" 2>/dev/null \
                && echo "signalled" || echo "no fleet_tab8 session"
            sleep 8
        fi
        exec "$HERE/fleet_ctl" stop "${SD[@]}"
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
TRANSPORT_SHOW=bench; for f in ${FLAGS[@]+"${FLAGS[@]}"}; do [ "$f" = --mesh ] && TRANSPORT_SHOW=mesh; done
echo " Drones: ${DRONES[*]}    Flags: ${FLAGS[*]:-(none)}"
echo " Transport: $TRANSPORT_SHOW $([ "$TRANSPORT_SHOW" = mesh ] && echo '(802.11s @2437 MHz - tab 8 drops SSH by design)' || echo '(bench LAN - no WiFi changes)')"
echo " Stats -> drone experiment_logs/: lora_rx+link (tab 7), wifi_rf (tab 8). Collect: ./fleet_ctl collect ${DRONES[*]}"
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
