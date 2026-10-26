#!/bin/bash
# coex_run.sh - drive the coexistence matrix and the fade-location study.
#
# The stack already logs continuously (tab 7 -> lora_rx_*.csv, tab 8 -> wifi_rf_*.csv).
# This tool does NOT start or stop logging per condition. It only:
#   1. verifies both drones are actually producing LoRa rows before a run opens (P1)
#   2. stamps condition windows into conditions.csv so analysis can slice the
#      continuous CSVs by condition afterwards
# It changes no interface, starts no node, and installs nothing.
#
#   ./coex_run.sh verify                 P1 + P2 + bench state, changes nothing
#   ./coex_run.sh mark <label> <minutes> hold a condition and record its window
#   ./coex_run.sh show                   print recorded conditions
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/../../credentials.env"
S="-o StrictHostKeyChecking=no -o LogLevel=ERROR -o ConnectTimeout=8"
declare -A IPS=( [delta]=192.168.0.40 [ghost]=192.168.0.50 )
declare -A ALT=( [ghost]=192.168.0.206 )
WS=/mnt/nova_ssd/workspaces/isaac_ros-dev
CSV="$HERE/conditions.csv"
DRONES=(delta ghost)

sd(){ local d="$1"; shift; sshpass -p "$JETSON_PASS" ssh $S "$d@${IPS[$d]}" "$@" 2>/dev/null \
      || { [ -n "${ALT[$d]:-}" ] && sshpass -p "$JETSON_PASS" ssh $S "$d@${ALT[$d]}" "$@" 2>/dev/null; }; }

lora_rows(){ sd "$1" "f=\$(ls -t $WS/experiment_logs/lora_rx_*.csv 2>/dev/null | head -1); [ -n \"\$f\" ] && echo \$(( \$(wc -l < \"\$f\") - 1 )) || echo -1"; }
wifi_rows(){ sd "$1" "f=\$(ls -t $WS/experiment_logs/wifi_rf_*.csv 2>/dev/null | head -1); [ -n \"\$f\" ] && echo \$(( \$(wc -l < \"\$f\") - 1 )) || echo -1"; }
wifi_mode(){ sd "$1" "iw dev wlP1p1s0 info | grep -oP 'type \\K.*|channel [0-9]+ \\(\\K[0-9]+' | tr '\n' ' '"; }

cmd_verify(){
  echo "=== P1: is BOTH drones' LoRa logging alive and GROWING? (6 s apart) ==="
  declare -A a b
  for d in "${DRONES[@]}"; do a[$d]=$(lora_rows "$d"); done
  sleep 6
  ok=1
  for d in "${DRONES[@]}"; do b[$d]=$(lora_rows "$d"); done
  printf "  %-7s %10s %10s %8s  %s\n" DRONE rows_t0 rows_t6 delta VERDICT
  for d in "${DRONES[@]}"; do
    x=${a[$d]:--1}; y=${b[$d]:--1}; dl=$((y-x))
    if [ "$x" = "-1" ]; then v="NO lora_rx FILE - tab 7 not started"; ok=0
    elif [ "$dl" -le 0 ]; then v="STALLED - file exists but no new rows"; ok=0
    else v="OK (~$(awk -v n=$dl 'BEGIN{printf "%.2f", n/6}') rows/s)"; fi
    printf "  %-7s %10s %10s %8s  %s\n" "$d" "$x" "$y" "+$dl" "$v"
  done
  echo
  echo "=== P2: mesh restore target ==="
  for d in "${DRONES[@]}"; do
    v2=$(sd "$d" "grep -m1 -oP 'NM_CONNECTION=\"\\K[^\"]*' $WS/src/multi_drone_nvblox/scripts/launch_mesh_zenoh_v2.sh")
    dyn=$(sd "$d" "grep -c NM_RESTORE_CONNECTION $WS/src/multi_drone_nvblox/scripts/launch_mesh_zenoh_v2.sh")
    act=$(sd "$d" "nmcli -t -f NAME,DEVICE con show --active | awk -F: '\$2==\"wlP1p1s0\"{print \$1; exit}'")
    printf "  %-7s v2 target='%s' dynamic_capture=%s active_profile='%s'\n" "$d" "$v2" "$dyn" "$act"
  done
  echo
  echo "=== bench state ==="
  for d in "${DRONES[@]}"; do
    printf "  %-7s wifi=[%s] lora_rows=%s wifi_rows=%s\n" "$d" "$(wifi_mode "$d")" "$(lora_rows "$d")" "$(wifi_rows "$d")"
  done
  echo
  [ "$ok" = 1 ] && echo "P1 PASS - both drones logging LoRa." || echo "P1 FAIL - do NOT open a run window."
  return $((1-ok))
}

cmd_mark(){
  local label="$1" mins="${2:-10}"
  [ -f "$CSV" ] || echo "condition,t_start_utc,t_end_utc,duration_s,delta_lora_rows,ghost_lora_rows,delta_wifi_rows,ghost_wifi_rows,delta_mode,ghost_mode,notes" > "$CSV"
  echo "[cond] $label : holding ${mins} min. Do not touch the bench."
  local t0 dl0 gl0 dw0 gw0
  t0=$(date -u +%FT%TZ); dl0=$(lora_rows delta); gl0=$(lora_rows ghost)
  dw0=$(wifi_rows delta); gw0=$(wifi_rows ghost)
  echo "  start $t0  lora(delta=$dl0 ghost=$gl0) wifi(delta=$dw0 ghost=$gw0)"
  local end=$(( $(date +%s) + mins*60 ))
  while [ "$(date +%s)" -lt "$end" ]; do
    sleep 60
    printf "  t+%3d min  lora(d=%s g=%s) wifi(d=%s g=%s)\n" \
      $(( (mins*60 - (end - $(date +%s))) / 60 )) "$(lora_rows delta)" "$(lora_rows ghost)" "$(wifi_rows delta)" "$(wifi_rows ghost)"
  done
  local t1 dl1 gl1 dw1 gw1
  t1=$(date -u +%FT%TZ); dl1=$(lora_rows delta); gl1=$(lora_rows ghost)
  dw1=$(wifi_rows delta); gw1=$(wifi_rows ghost)
  echo "$label,$t0,$t1,$((mins*60)),$((dl1-dl0)),$((gl1-gl0)),$((dw1-dw0)),$((gw1-gw0)),$(wifi_mode delta| tr ' ' '_'),$(wifi_mode ghost | tr ' ' '_'),${NOTES:-}" >> "$CSV"
  echo "  end   $t1  captured lora(delta=+$((dl1-dl0)) ghost=+$((gl1-gl0))) wifi(delta=+$((dw1-dw0)) ghost=+$((gw1-gw0)))"
}
cmd_show(){ [ -f "$CSV" ] && column -s, -t < "$CSV" || echo "no conditions recorded yet"; }

case "${1:-}" in
  verify) cmd_verify ;;
  mark)   cmd_mark "${2:?label}" "${3:-10}" ;;
  show)   cmd_show ;;
  *) sed -n '2,16p' "$0"; exit 2 ;;
esac
