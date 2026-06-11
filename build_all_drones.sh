#!/bin/bash
# build_all_drones.sh — rebuild the isaac_ros-dev workspace on drones using THE
# canonical colcon sequence (drone_build_sequence.sh — exact commands, exact order).
#
# Usage:
#   ./build_all_drones.sh                  # all 4 drones
#   ./build_all_drones.sh ghost delta      # subset
#
# Pushes drone_build_sequence.sh to each drone's workspace, runs it inside the
# isaac_ros_realsense container under nohup (survives SSH drops), then tails all
# logs until every build finishes. Logs land on each drone at:
#   /mnt/nova_ssd/workspaces/isaac_ros-dev/build_seq.log

source "$(dirname "$(readlink -f "$0")")/credentials.env" 2>/dev/null
PASS="${JETSON_PASS:?credentials.env missing — copy credentials.env.example and fill it in}"
WS=/mnt/nova_ssd/workspaces/isaac_ros-dev
SEQ="$(dirname "$(readlink -f "$0")")/drone_build_sequence.sh"

# wired IPs first; thunderstrike via WiFi until it gets a LAN port
declare -A IPS=( [ghost]=192.168.0.206 [delta]=192.168.0.236 [buckshee]=192.168.0.124 [thunderstrike]=192.168.0.70 )

DRONES=("$@")
[ ${#DRONES[@]} -eq 0 ] && DRONES=(ghost delta buckshee thunderstrike)

do_ssh() { sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 \
           -o ServerAliveInterval=15 -o ServerAliveCountMax=4 "$1" "$2"; }

started=()
for d in "${DRONES[@]}"; do
    ip="${IPS[$d]}"
    [ -z "$ip" ] && { echo "[$d] unknown drone, skipping"; continue; }
    echo "[$d] pushing build sequence + starting container..."
    if ! sshpass -p "$PASS" scp -o StrictHostKeyChecking=no "$SEQ" "$d@$ip:$WS/drone_build_sequence.sh"; then
        echo "[$d] UNREACHABLE — skipping"; continue
    fi
    do_ssh "$d@$ip" "
        docker ps --format '{{.Names}}' | grep -qx isaac_ros_realsense || docker start isaac_ros_realsense
        rm -f $WS/build_seq.log
        nohup docker exec -u admin isaac_ros_realsense \
            bash /workspaces/isaac_ros-dev/drone_build_sequence.sh \
            > $WS/build_seq.log 2>&1 &
        echo '[$d] build launched'"
    started+=("$d")
done

[ ${#started[@]} -eq 0 ] && { echo "Nothing started."; exit 1; }

echo ""
echo "Monitoring builds on: ${started[*]} (Ctrl+C is safe — builds keep running on the drones)"
while true; do
    sleep 30
    line=""
    alldone=true
    for d in "${started[@]}"; do
        status=$(do_ssh "$d@${IPS[$d]}" "
            if grep -q 'ALL 7 STEPS SUCCEEDED' $WS/build_seq.log 2>/dev/null; then echo OK;
            elif grep -qE 'Aborted|failed' $WS/build_seq.log 2>/dev/null; then echo FAILED;
            else grep -c '^================ STEP' $WS/build_seq.log 2>/dev/null || echo 0; fi" 2>/dev/null)
        case "$status" in
            OK)     line+="$d:DONE  " ;;
            FAILED) line+="$d:FAILED  " ;;
            *)      line+="$d:step${status:-?}/7  "; alldone=false ;;
        esac
    done
    echo "$(date '+%H:%M:%S')  $line"
    $alldone && break
done

echo ""
for d in "${started[@]}"; do
    echo "=== $d: last lines of build_seq.log ==="
    do_ssh "$d@${IPS[$d]}" "tail -5 $WS/build_seq.log"
done
