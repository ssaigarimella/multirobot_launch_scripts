#!/bin/bash
# lora_power_sweep.sh - LoRa link-margin sweep (Experiment E5).
#
#   ./experiments/lora_power_sweep.sh ghost delta buckshee            # default powers
#   POWERS="-9 -5 0 5 10 14 17 22" DWELL=60 ./experiments/lora_power_sweep.sh ghost delta
#
# PRECONDITIONS
#   * radios flashed to v7.3 (./flash_lora_firmware.sh <drones>) -- a v7.2 radio
#     never confirms a power change and is reported NOT_FLASHED, not counted
#   * coordination stage running on every listed drone (tab 7 of
#     launch_fleet_tmux.sh): it provides the LoRa bridge AND metrics_recorder,
#     whose lora_rx_*.csv records every received packet with RSSI/SNR
#
# WHAT IT DOES
#   For each power P: command every listed radio to P dBm, wait for each radio's
#   own POWER_RESP (the power it ACTUALLY applied), dwell DWELL seconds, and write
#   one marker row. Restores 10 dBm (LORA_PWR, the pre-sweep baseline) at the end,
#   even on Ctrl-C.
#
# OUTPUT  experiments/results/sweep_<stamp>/markers.csv
#   then:  ./fleet_ctl collect <drones>
#          python3 experiments/analyze_sweep.py experiments/results/sweep_<stamp>
set -uo pipefail
HERE="$(dirname "$(readlink -f "$0")")"; ROOT="$(dirname "$HERE")"
source "$ROOT/credentials.env" 2>/dev/null
PASS="${JETSON_PASS:?credentials.env missing}"
POWERS="${POWERS:--9 -5 0 5 10 14 17 22}"
DWELL="${DWELL:-60}"
declare -A IPS=( [delta]=192.168.0.40 [buckshee]=192.168.0.60 \
                 [ghost]=192.168.0.50 [thunderstrike]=192.168.0.70 )
declare -A IDS=( [delta]=1 [buckshee]=2 [ghost]=3 [thunderstrike]=4 )
DRONES=("$@"); [ ${#DRONES[@]} -eq 0 ] && { echo "usage: $0 <drone...>"; exit 1; }
OUT="$ROOT/experiments/results/sweep_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$OUT"
M="$OUT/markers.csv"
echo "t_start,t_end,power_req_dbm,drone,power_confirmed_dbm,status" > "$M"

in_c() {  # in_c <drone> <command>   (inside the container, right domain)
    local d="$1"; shift
    sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 \
        "$d@${IPS[$d]}" "docker exec -u admin isaac_ros_realsense bash -c \
        'source /opt/ros/humble/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash \
         && export ROS_DOMAIN_ID=${IDS[$d]} ROS_LOCALHOST_ONLY=0 && $*'" 2>/dev/null
}

set_power() {  # set_power <drone> <dbm>  -> prints the confirmed dBm or NONE
    local d="$1" p="$2" n="${IDS[$1]}"
    # listen first (POWER_RESP is a one-shot), then command, then collect
    in_c "$d" "(timeout 6 ros2 topic echo --once /d$n/lora/tx_power std_msgs/msg/Int8 \
        2>/dev/null | grep -oE \"data: -?[0-9]+\" | grep -oE \"-?[0-9]+\" > /tmp/txp.out &) ; \
        sleep 1.5; ros2 topic pub --once /d$n/lora/set_tx_power std_msgs/msg/Int8 \
        \"{data: $p}\" >/dev/null 2>&1; sleep 4; cat /tmp/txp.out 2>/dev/null" | tail -1
}

restore() {
    echo ""; echo "[sweep] restoring 10 dBm (pre-sweep baseline) on all radios"
    for d in "${DRONES[@]}"; do echo "  $d -> $(set_power "$d" 10)"; done
}
trap restore EXIT

echo "[sweep] drones: ${DRONES[*]}   powers: $POWERS dBm   dwell: ${DWELL}s"
echo "[sweep] markers -> $M"
for p in $POWERS; do
    echo ""; echo "=== $p dBm ==="
    t0=$(date +%s.%N)
    for d in "${DRONES[@]}"; do
        got=$(set_power "$d" "$p")
        if [ -z "$got" ]; then st=NOT_FLASHED_OR_NO_BRIDGE; got=""
        elif [ "$got" = "$p" ]; then st=OK
        else st=CLAMPED; fi
        echo "  $d: requested $p, radio reports ${got:-nothing} ($st)"
        echo "$d,$got,$st" >> "$OUT/.tmp_$p"
    done
    echo "  dwelling ${DWELL}s ..."; sleep "$DWELL"
    t1=$(date +%s.%N)
    while IFS=, read -r d got st; do
        echo "$t0,$t1,$p,$d,$got,$st" >> "$M"
    done < "$OUT/.tmp_$p"; rm -f "$OUT/.tmp_$p"
done
echo ""; echo "[sweep] done. Next:"
echo "  ./fleet_ctl collect ${DRONES[*]}"
echo "  python3 experiments/analyze_sweep.py $OUT"
