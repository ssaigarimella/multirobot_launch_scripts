#!/bin/bash
# launch_fleet_tmux.sh - multi-drone field stack, one local terminal WINDOW
# per drone, in the same style as the single-drone launchers:
#
#     ./launch_fleet_tmux.sh                      ghost + delta (default)
#     ./launch_fleet_tmux.sh ghost delta --trust-prior
#     ./launch_fleet_tmux.sh all --debug
#     ./launch_fleet_tmux.sh --headless ghost delta   no windows (detached)
#     ./launch_fleet_tmux.sh attach ghost         re-attach to a live drone
#     ./launch_fleet_tmux.sh stop [all]           tear everything down
#
# Each window: press Enter to launch that drone, then watch its output live.
# The stack runs in a tmux session named 'fleet' ON THE DRONE, so an SSH drop
# detaches instead of killing the flight stack, and re-running the window
# re-attaches. Ctrl-b d detaches; NEVER Ctrl-C (that kills the stack).
#
# All output is piped to <ws>/experiment_logs/pane_fleet_<epoch>.log on the
# drone; collect afterwards with ./fleet_ctl collect <drones>.
#
# The launch command itself comes from `fleet_ctl cmdline`, so this script and
# `fleet_ctl start` can never disagree about how the stack is launched.

set -u

HERE="$(dirname "$(readlink -f "$0")")"
SCRIPT="$(readlink -f "$0")"
declare -A IPS=( [delta]=192.168.0.40 [buckshee]=192.168.0.60 \
                 [ghost]=192.168.0.50 [thunderstrike]=192.168.0.70 )

# ---------------------------------------------------------------- child mode
# Re-invoked inside each spawned window: --window <drone> <flags...>
if [ "${1:-}" = "--window" ]; then
    DRONE="$2"; shift 2
    source "$HERE/credentials.env"
    eval "$("$HERE/fleet_ctl" cmdline "$DRONE" "$@")"

    echo "============================================================"
    echo "  $DRONE (d$DOMAIN) @ ${IPS[$DRONE]}"
    echo "============================================================"
    echo "  $CMD" | fold -s -w 74 | sed 's/^/  /'
    echo ""
    echo "  tmux session 'fleet' on the drone - survives SSH drops."
    echo "  Ctrl-b d to detach.  Do NOT press Ctrl-C (kills the stack)."
    echo ""
    echo "  Expect ~2-4 min of 'no valid local position' in QGC while the"
    echo "  uXRCE session re-establishes and EKF2 converges. Arm after that."
    echo ""
    read -rp "Press Enter to launch $DRONE... "

    LOGDIR="$WS/experiment_logs"
    REMOTE="${DOCKER_UP}mkdir -p $LOGDIR; "
    # bring the peer bridge up first (detached, its own tmux session)
    [ -n "$ZENOH" ] && REMOTE+="$ZENOH; "
    REMOTE+="tmux has-session -t fleet 2>/dev/null || tmux new-session -d \
-x 220 -y 50 -s fleet \"docker exec -it -u admin isaac_ros_realsense bash -c \
'source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && \
export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=$LOCALHOST_ONLY && $CMD; \
ec=\\\$?; echo; echo ===== exited code \\\$ec - pane kept open, Ctrl-b d to \
detach =====; exec bash'\"; \
tmux pipe-pane -o -t fleet \"cat >> $LOGDIR/pane_fleet_\\\$(date +%s).log\"; \
tmux attach-session -t fleet"

    sshpass -p "$JETSON_PASS" ssh -t \
        -o StrictHostKeyChecking=no \
        -o ServerAliveInterval=15 -o ServerAliveCountMax=4 \
        "$DRONE@${IPS[$DRONE]}" "$REMOTE"

    echo ""
    echo "[$DRONE detached or exited. Press Enter to close this window.]"
    read
    exit 0
fi

# --------------------------------------------------------------- parent mode
case "${1:-}" in
    attach)
        source "$HERE/credentials.env"
        d="${2:?usage: attach <drone>}"
        exec sshpass -p "$JETSON_PASS" ssh -t -o StrictHostKeyChecking=no \
            "$d@${IPS[$d]}" "tmux attach -t fleet"
        ;;
    stop)
        shift; exec "$HERE/fleet_ctl" stop "${@:-all}"
        ;;
    -h|--help|help)
        sed -n '2,25p' "$SCRIPT" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
esac

HEADLESS=0
DRONES=()
FLAGS=()
for a in "$@"; do
    case "$a" in
        --headless) HEADLESS=1 ;;
        all) DRONES=(delta buckshee ghost thunderstrike) ;;
        delta|buckshee|ghost|thunderstrike) DRONES+=("$a") ;;
        *) FLAGS+=("$a") ;;
    esac
done
[ ${#DRONES[@]} -eq 0 ] && DRONES=(ghost delta)

if [ "$HEADLESS" = "1" ]; then
    exec "$HERE/fleet_ctl" start "${DRONES[@]}" ${FLAGS[@]+"${FLAGS[@]}"}
fi

command -v sshpass >/dev/null || { echo "need sshpass: sudo apt install sshpass"; exit 1; }
command -v gnome-terminal >/dev/null || {
    echo "gnome-terminal not found; falling back to headless start"
    exec "$HERE/fleet_ctl" start "${DRONES[@]}" ${FLAGS[@]+"${FLAGS[@]}"}; }

# so each window's zenoh bridge knows the full flying set
CSV=$(IFS=,; echo "${DRONES[*]}")

echo "============================================================"
echo " Multi-drone field stack"
echo " Drones:  ${DRONES[*]}"
echo " Flags:   ${FLAGS[*]:-(none)}"
echo "============================================================"
echo " One window per drone opens now. Press Enter in each to launch."
echo " Then:  ./fleet_ctl preflight ${DRONES[*]}    (expect 9/9)"
echo " Stop:  ./launch_fleet_tmux.sh stop"
echo " Logs:  ./fleet_ctl collect ${DRONES[*]}"
echo "============================================================"

for d in "${DRONES[@]}"; do
    gnome-terminal --window --title="$d" -- \
        bash "$SCRIPT" --window "$d" "--drones=$CSV" ${FLAGS[@]+"${FLAGS[@]}"}
    sleep 0.4
done
