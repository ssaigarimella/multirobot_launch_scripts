#!/bin/bash
# colour_check.sh - bench check for the in-flight colour-map stall
# (reports/field/FLIGHT-ANALYSIS-2026-10-23.md section 11.1).
#
#   ./experiments/colour_check.sh start  <run> <drone>... [--seconds 900]
#   ./experiments/colour_check.sh status <run> <drone>...
#   ./experiments/colour_check.sh stop   <run> <drone>...
#   ./experiments/colour_check.sh pull   <run> <drone>...
#
# PRECONDITION: the stack is up on each drone (console BRING UP).
# start runs experiments/colour_check.py in the container (1 CSV row per second)
# and tegrastats on the host, both for --seconds, in tmux session 'colourcheck'
# (outside the fleet session, so a fleet stop does not cut the log short).
# Files: <ws>/experiment_logs/colour_check_<run>/ on the drone;
# pull copies them to experiments/results/colour_check_<run>/<drone>/.
HERE="$(dirname "$(readlink -f "$0")")"; ROOT="$(dirname "$HERE")"
source "$ROOT/credentials.env"
declare -A IPS=( [delta]=192.168.0.40 [buckshee]=192.168.0.60 \
                 [ghost]=192.168.0.50 [thunderstrike]=192.168.0.70 )
declare -A IDS=( [delta]=1 [buckshee]=2 [ghost]=3 [thunderstrike]=4 )
USAGE="usage: $0 start|status|stop|pull <run> <drone>... [--seconds N]"
CMD="${1:?$USAGE}"; RUN="${2:?$USAGE}"; shift 2
SECS=900; DRONES=()
while [ $# -gt 0 ]; do
  case "$1" in --seconds) SECS="$2"; shift 2;; *) DRONES+=("$1"); shift;; esac
done
[ ${#DRONES[@]} -gt 0 ] || { echo "$USAGE"; exit 1; }
HOST_DIR=/mnt/nova_ssd/workspaces/isaac_ros-dev/experiment_logs/colour_check_$RUN
CTR_DIR=/workspaces/isaac_ros-dev/experiment_logs/colour_check_$RUN
rsh() { sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 "$1@${IPS[$1]}" "$2"; }
rcp() { sshpass -p "$JETSON_PASS" scp -q -o StrictHostKeyChecking=no "$@"; }

for d in "${DRONES[@]}"; do
  case "$CMD" in
  start)
    if rsh "$d" "tmux has-session -t colourcheck 2>/dev/null"; then
      echo "$d: already running (stop it first)"; continue
    fi
    rsh "$d" "mkdir -p $HOST_DIR" || { echo "$d: unreachable"; continue; }
    cat > /tmp/colour_check_run_$$.sh <<RUN
#!/bin/bash
docker exec -u admin isaac_ros_realsense bash -c 'source /opt/ros/humble/setup.bash \\
  && source /workspaces/isaac_ros-dev/install/setup.bash \\
  && export ROS_DOMAIN_ID=${IDS[$d]} ROS_LOCALHOST_ONLY=0 \\
  && exec python3 $CTR_DIR/colour_check.py --out $CTR_DIR --seconds $SECS' > $HOST_DIR/colour_check.log 2>&1
RUN
    rcp "$HERE/colour_check.py" "$d@${IPS[$d]}:$HOST_DIR/" &&
    rcp /tmp/colour_check_run_$$.sh "$d@${IPS[$d]}:$HOST_DIR/run.sh" &&
    rsh "$d" "tmux new-session -d -s colourcheck 'bash $HOST_DIR/run.sh' &&
              tmux new-window -t colourcheck 'timeout $SECS tegrastats --interval 1000 --logfile $HOST_DIR/tegrastats.log'" &&
    echo "$d: logging for $SECS s -> $HOST_DIR"
    rm -f /tmp/colour_check_run_$$.sh;;
  status)
    echo "== $d"
    rsh "$d" "tail -n 1 $HOST_DIR/colour_check.csv; grep -v '^\s*$' $HOST_DIR/colour_check.log | tail -n 2; tail -n 1 $HOST_DIR/tegrastats.log | grep -o 'GR3D_FREQ [0-9]*%'";;
  stop)
    rsh "$d" "docker exec isaac_ros_realsense pkill -INT -f '[c]olour_check.py'; pkill -f '[t]egrastats --interval';
              sleep 2; tmux kill-session -t colourcheck 2>/dev/null"
    echo "$d: stopped";;
  pull)
    mkdir -p "$ROOT/experiments/results/colour_check_$RUN/$d"
    # one ssh stream: sshpass answers a single password prompt per call
    rsh "$d" "tar -C $HOST_DIR -cf - colour_check.csv colour_check.log tegrastats.log" |
      tar -C "$ROOT/experiments/results/colour_check_$RUN/$d" -xf - &&
    echo "$d: pulled to experiments/results/colour_check_$RUN/$d/";;
  *) echo "$USAGE"; exit 1;;
  esac
done
