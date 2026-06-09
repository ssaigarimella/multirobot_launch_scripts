#!/bin/bash
# launch_swarm.sh - Launch one or more drones' exploration stacks RESILIENTLY.
#
# Why this exists:
#   The per-drone launch_*.sh scripts run every ROS node as
#       laptop -> ssh -t JETSON -> docker exec -it CONTAINER ... ros2 launch ...
#   so each node is a child of the SSH session. If that SSH connection drops
#   (e.g. WiFi hiccup while flying 2+ drones), the node receives SIGHUP and the
#   drone's stack dies. Running several drones at once means ~18 live SSH
#   sessions on one WiFi link, and when one drops that whole drone goes down.
#
#   This script instead starts each drone's stack inside a DETACHED tmux session
#   ON THE JETSON. The processes are owned by the Jetson-side tmux server, not by
#   any SSH connection. The laptop only ATTACHES to watch. If the attach drops,
#   the drone keeps flying -- just re-attach. Each drone is fully independent.
#
# Usage:
#   ./launch_swarm.sh ghost buckshee                # launch full stack on both
#   ./launch_swarm.sh delta buckshee ghost thunderstrike
#   ./launch_swarm.sh --vio ghost                   # VIO-only (cuVSLAM + vio bridge)
#   ./launch_swarm.sh --rviz --debug ghost          # RViz on, skip arm check
#   ./launch_swarm.sh --attach ghost buckshee       # just (re)attach to running sessions
#   ./launch_swarm.sh --status delta buckshee ghost thunderstrike
#   ./launch_swarm.sh --kill ghost buckshee         # stop the stacks (kills the nodes!)
#   ./launch_swarm.sh --restart ghost               # tear down + relaunch (kills flying nodes!)
#   ./launch_swarm.sh --no-attach ghost buckshee    # launch but don't open laptop tabs
#   ./launch_swarm.sh --mesh delta ghost            # also start tab-8 mesh+Zenoh bridge
#
# Inside an attach tab: Ctrl-b n / Ctrl-b p switch windows, Ctrl-b w lists them,
#                       Ctrl-b d detaches (drone keeps running).
#
# Drone IDs: 1=delta  2=buckshee  3=ghost  4=thunderstrike

set -uo pipefail

# --- registry ---------------------------------------------------------------
# The per-drone scripts remain the source of truth for host/IP: we grep the
# active JETSON_HOST line out of them so this script never drifts out of date.
declare -A DRONE_SCRIPT=(
    [delta]="launch_delta_jetson_eduroam_python_frontierexplore.sh"
    [buckshee]="launch_buckshee_jetson_eduroam_python_frontierexplore.sh"
    [ghost]="launch_ghost_jetson_eduroam_python_frontierexplore.sh"
    [thunderstrike]="launch_thunderstrike_jetson_eduroam_python_frontierexplore.sh"
)
declare -A DRONE_ID=( [delta]=1 [buckshee]=2 [ghost]=3 [thunderstrike]=4 )

JETSON_PASS="abc123"
CONTAINER="isaac_ros_realsense"
IMAGE="isaac_ros:dev-realsense"
SESSION="swarm"                      # tmux session name on each Jetson
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"

# --- arg parsing ------------------------------------------------------------
DRONES=()
ACTION="launch"
RVIZ="False"; DEBUG="false"; VIO="0"; RESTART="0"; MESH="0"; ATTACH="1"

for arg in "$@"; do
    case "$arg" in
        --vio)       VIO="1" ;;
        --rviz)      RVIZ="True" ;;
        --debug)     DEBUG="true" ;;
        --restart)   RESTART="1" ;;
        --mesh)      MESH="1" ;;
        --no-attach) ATTACH="0" ;;
        --attach)    ACTION="attach" ;;
        --kill)      ACTION="kill" ;;
        --status)    ACTION="status" ;;
        -h|--help|help)
            sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        --*)
            echo "Unknown flag: $arg"; exit 1 ;;
        *)
            if [ -n "${DRONE_SCRIPT[$arg]:-}" ]; then
                DRONES+=("$arg")
            else
                echo "Unknown drone: '$arg' (valid: ${!DRONE_SCRIPT[*]})"; exit 1
            fi ;;
    esac
done

if [ "${#DRONES[@]}" -eq 0 ]; then
    echo "No drones specified. Example: ./launch_swarm.sh ghost buckshee"
    echo "Valid drones: ${!DRONE_SCRIPT[*]}"
    exit 1
fi

if ! command -v sshpass >/dev/null 2>&1; then
    echo "sshpass required: sudo apt install sshpass"; exit 1
fi

# --- helpers ----------------------------------------------------------------

# Read the currently-active (uncommented) JETSON_HOST="..." line from a drone's
# per-drone script, so IP changes only need to be made in one place.
get_host() {
    local d="$1" f="$HERE/${DRONE_SCRIPT[$d]}"
    [ -f "$f" ] || { echo ""; return 1; }
    grep -E '^[[:space:]]*JETSON_HOST=' "$f" | tail -1 \
        | sed -E 's/^[^=]*=//; s/^"//; s/"$//'
}

ssh_run() {  # ssh_run <host> <command>   (no pty -- for setup, returns promptly)
    sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$1" "$2"
}

# Peers of $1 = the other selected drones' IDs (for the mesh/Zenoh bridge).
peers_of() {
    local me="$1" out=""
    for d in "${DRONES[@]}"; do
        [ "$d" = "$me" ] && continue
        out="$out ${DRONE_ID[$d]}"
    done
    echo "${out# }"
}

# --- actions ----------------------------------------------------------------

do_attach() {  # open one laptop terminal tab that attaches to the Jetson tmux
    local d="$1" host="$2"
    gnome-terminal --tab --title="$d :$SESSION" -- bash -c "
        echo '== Attaching to $d ($host) tmux session ':$SESSION' ==';
        echo 'Ctrl-b n/p = next/prev window, Ctrl-b w = list, Ctrl-b d = detach.';
        echo 'If this drops, the drone KEEPS RUNNING. Re-attach: $0 --attach $d';
        echo;
        sshpass -p $JETSON_PASS ssh -t -o StrictHostKeyChecking=no $host 'tmux attach -t $SESSION || { echo \"no live :$SESSION session on \$(hostname)\"; sleep 4; }';
        echo; echo '[detached] $d still running on the Jetson. exec shell:'; exec bash"
}

if [ "$ACTION" = "attach" ]; then
    for d in "${DRONES[@]}"; do
        host="$(get_host "$d")"
        [ -z "$host" ] && { echo "!! could not resolve host for $d"; continue; }
        echo "Attaching to $d ($host)..."
        do_attach "$d" "$host"
        sleep 0.3
    done
    exit 0
fi

if [ "$ACTION" = "kill" ]; then
    for d in "${DRONES[@]}"; do
        host="$(get_host "$d")"
        [ -z "$host" ] && { echo "!! could not resolve host for $d"; continue; }
        echo "Killing :$SESSION on $d ($host)..."
        ssh_run "$host" "tmux kill-session -t $SESSION 2>/dev/null && echo '[killed]' || echo '[no session]'"
    done
    exit 0
fi

if [ "$ACTION" = "status" ]; then
    for d in "${DRONES[@]}"; do
        host="$(get_host "$d")"
        [ -z "$host" ] && { echo "$d: ?? (no host)"; continue; }
        echo "=== $d ($host) ==="
        ssh_run "$host" "tmux list-windows -t $SESSION 2>/dev/null || echo '  (no :$SESSION session)'"
    done
    exit 0
fi

# --- launch -----------------------------------------------------------------
echo "============================================================"
echo " Swarm launch: ${DRONES[*]}"
echo " Mode: $([ "$VIO" = 1 ] && echo VIO-only || echo full-explore)  rviz=$RVIZ debug=$DEBUG mesh=$MESH"
echo " Each stack runs in a Jetson-side tmux session ':$SESSION'."
echo " SSH drops will NOT take the drones down."
echo "============================================================"

for d in "${DRONES[@]}"; do
    host="$(get_host "$d")"
    if [ -z "$host" ]; then echo "!! could not resolve host for $d -- skipping"; continue; fi
    id="${DRONE_ID[$d]}"
    peers="$(peers_of "$d")"
    echo ""
    echo ">>> $d (id=$id, host=$host, peers=[$peers])"

    # Bootstrap on the Jetson. The heredoc is QUOTED ('REMOTE') so nothing is
    # expanded on the laptop; per-invocation values are passed as positional
    # args to `bash -s`. The bootstrap ensures the container is up, then builds
    # the detached tmux session with one window per component.
    ssh_run "$host" "bash -s '$CONTAINER' '$IMAGE' '$SESSION' '$id' '$RVIZ' '$DEBUG' '$VIO' '$RESTART' '$MESH' '$peers'" <<'REMOTE'
set -uo pipefail
CONTAINER="$1"; IMAGE="$2"; SESSION="$3"; ID="$4"; RVIZ="$5"; DEBUG="$6"
VIO="$7"; RESTART="$8"; MESH="$9"; PEERS="${10}"

DOCKER_SRC="export ROS_LOCALHOST_ONLY=0 && source /opt/ros/humble/setup.bash && cd /workspaces/isaac_ros-dev && source install/setup.bash"

if ! command -v tmux >/dev/null 2>&1; then
    echo "ERROR: tmux is not installed on $(hostname). Run: sudo apt-get install -y tmux"
    exit 3
fi

# --- ensure the Isaac ROS container is running (reuse, never destroy) ---
if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "[OK] container $CONTAINER already running"
elif docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "[INFO] starting stopped container $CONTAINER"; docker start "$CONTAINER" >/dev/null
else
    echo "[INFO] creating container $CONTAINER"
    docker run -dit --privileged --network host --ipc host \
        -v /mnt/nova_ssd/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
        -v /mnt/nova_ssd/workspaces/Micro-XRCE-DDS-Agent:/workspaces/Micro-XRCE-DDS-Agent \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$HOME/.Xauthority:/home/admin/.Xauthority:rw" \
        -e DISPLAY -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
        --runtime nvidia --name "$CONTAINER" "$IMAGE" /bin/bash >/dev/null \
        && echo "[OK] container created" || { echo "[ERROR] container failed to start"; exit 4; }
fi

# --- (re)use the tmux session ---
if tmux has-session -t "$SESSION" 2>/dev/null; then
    if [ "$RESTART" = "1" ]; then
        tmux kill-session -t "$SESSION"; echo "[restart] killed existing :$SESSION"
    else
        echo "[skip] :$SESSION already running on $(hostname)."
        echo "       Attach to view it, or pass --restart to recreate."
        exit 0
    fi
fi

# Open a window running a ROS component inside the container. The node is
# followed by 'exec bash' so the window survives a crash (lets you read the
# error / restart it by hand) instead of vanishing.
win() {  # win <window-name> <ros-command>
    tmux new-window -t "$SESSION" -d -n "$1" \
        "docker exec -it -u admin $CONTAINER bash -lc \"$DOCKER_SRC && $2; exec bash\""
}

# component commands (identical across drones except drone_id)
C_CUVSLAM="ros2 launch nvblox_examples_bringup realsense_example.launch.py run_rviz:=$RVIZ"
C_VIO="ros2 launch px4_offboard vio_bridge.launch.py"
C_FIS="ros2 launch active_exploration fis.launch.py flight_height:=1.0"
C_GUARD="ros2 launch active_exploration reactive_guard.launch.py"
C_PLANNER="ros2 launch active_exploration simple_planner_launch.py flight_height:=1.0 debug_skip_arm_check:=$DEBUG"
C_LOOP="python3 src/multi_drone_nvblox/scripts/loop_closure_sp_node.py --ros-args -p robot_namespace:=drone1 -p vocabulary_file:=/workspaces/isaac_ros-dev/src/multi_drone_nvblox/models/sp_vocab_4096.pkl -p process_rate:=1.0 -p use_pcm:=True -p publish_tf:=True"
C_OCTOMAP="ros2 run multi_drone_nvblox octomap_exchange_node --ros-args -p robot_namespace:=drone1 -p drone_id:=$ID -p resolution:=0.05 -p alignment_config:=/workspaces/isaac_ros-dev/src/multi_drone_nvblox/config/swarm_alignment.yaml -p use_sim_time:=False"

# host shell window so the session always has a stable first window
tmux new-session -d -s "$SESSION" -n host

# Staged startup: cuVSLAM/RealSense must initialise before VIO consumes its
# odometry; the rest follow with small settle gaps.
win cuvslam "$C_CUVSLAM"; sleep 10
win vio     "$C_VIO";     sleep 4
if [ "$VIO" != "1" ]; then
    win fis     "$C_FIS";     sleep 3
    win guard   "$C_GUARD";   sleep 3
    win planner "$C_PLANNER"; sleep 3
    win loop    "$C_LOOP";    sleep 3
    win octomap "$C_OCTOMAP"
fi

# Optional tab-8 equivalent: switch to 802.11s mesh + Zenoh bridge (runs on the
# HOST, needs sudo). NOTE: this changes the WiFi, so any laptop attach will drop
# the moment it switches -- the tmux session (and drones) keep running.
if [ "$MESH" = "1" ]; then
    tmux new-window -t "$SESSION" -d -n mesh \
        "sudo bash /mnt/nova_ssd/workspaces/isaac_ros-dev/launch_mesh_zenoh.sh $ID $PEERS; exec bash"
fi

tmux select-window -t "$SESSION:cuvslam" 2>/dev/null || true
echo "[OK] :$SESSION launched on $(hostname)"
tmux list-windows -t "$SESSION"
REMOTE

    rc=$?
    if [ "$rc" -ne 0 ]; then
        echo "!! bootstrap for $d exited with code $rc"
        continue
    fi

    if [ "$ATTACH" = "1" ]; then
        do_attach "$d" "$host"
    fi
    sleep 0.5
done

echo ""
echo "Done. Drones run independently on their Jetsons."
echo "  view/re-attach : $0 --attach ${DRONES[*]}"
echo "  status         : $0 --status ${DRONES[*]}"
echo "  stop a drone   : $0 --kill <drone>"
