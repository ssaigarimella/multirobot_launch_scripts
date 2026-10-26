#!/bin/bash
# rf_stats.sh - passive WiFi RF stats collector for fleet runs.
#
# Runs ALONGSIDE the normal flow (./launch_fleet_tmux.sh ghost delta). It changes
# nothing: no mesh join, no nmcli, no interface reconfiguration, no `iw scan`.
# It only reads `iw dev <if> info` and `iw dev <if> station dump` once a second.
#
#   ./rf_stats.sh start  ghost delta      # start samplers
#   ./rf_stats.sh status ghost delta      # is it running, how many rows so far
#   ./rf_stats.sh stop   ghost delta      # stop samplers
#   ./rf_stats.sh start --iperf ghost delta   # ALSO run a periodic throughput probe
#
# Output (on each drone, picked up by `./fleet_ctl collect <drones>`):
#   experiment_logs/wifi_rf_<host>_<ts>.csv   1 Hz, one row per peer per second
#   experiment_logs/wifi_tp_<host>_<ts>.csv   only with --iperf
#
# NOT duplicated here: the LoRa side. metrics_recorder.py (tab 7) already writes
# lora_rx / swarm / link / vio_health / keyframes / coverage into the same
# experiment_logs dir. Do not add LoRa columns here - join on time instead.
#
# Timebase, so joins are not over-trusted:
#   metrics_recorder stamps with the ROS clock (metrics_recorder.py:155,
#   get_clock().now(), use_sim_time:=False) = CLOCK_REALTIME. This sampler stamps
#   with host `date +%s.%N` = the same clock. So:
#     same-drone joins (wifi_rf x lora_rx on one drone)  : exact, same clock.
#     cross-drone joins                                  : limited by NTP skew.
#   Measured 2026-08-18: delta and ghost both sat +5.6 ms vs the laptop, i.e. a
#   drone-to-drone offset of ~0.2 ms at that instant, but per-sample uncertainty
#   was +/-5.5 to +/-8 ms and the drones discipline against DIFFERENT public
#   pool.ntp.org servers on a 34 min 8 s poll interval, so the guaranteed bound
#   degrades to roughly +/-25-50 ms late in a poll interval.
#   => Cross-drone correlation at 1 Hz is fine. Do NOT claim per-packet
#      cross-drone alignment. Fix, if ever needed: point both drones at one local
#      NTP server with a short poll interval (sub-ms wired, 1-2 ms over WiFi).
#
# tmux session name is 'rfstats' on purpose: `fleet_ctl stop` kills only sessions
# matching ^(fleet|zenoh|rviz|loratest), so this survives a stack teardown and you
# stop it explicitly.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/credentials.env" 2>/dev/null
: "${JETSON_PASS:?credentials.env missing - copy credentials.env.example and fill it in}"

WS=/mnt/nova_ssd/workspaces/isaac_ros-dev
LOGDIR="$WS/experiment_logs"
SESS="${SESS:-rfstats}"
IFACE="${IFACE:-wlP1p1s0}"
HZ="${HZ:-1}"
IPERF_EVERY="${IPERF_EVERY:-60}"
IPERF_SECS="${IPERF_SECS:-5}"

declare -A IPS=( [delta]=192.168.0.40 [buckshee]=192.168.0.60 [ghost]=192.168.0.50 [thunderstrike]=192.168.0.70 )

ssh_d(){ local d="$1"; shift
  sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o LogLevel=ERROR \
    -o ConnectTimeout=8 "$d@${IPS[$d]}" "$@"; }

# ---- the sampler that runs on the drone -----------------------------------
# Pure read. Emits one CSV row per associated peer per tick; if there are no
# peers it still emits a row (peer_mac=NONE) so gaps are visible rather than absent.
remote_sampler(){ cat <<'EOS'
#!/bin/bash
IFACE="$1"; LOGDIR="$2"; HZ="$3"
mkdir -p "$LOGDIR"
CSV="$LOGDIR/wifi_rf_$(hostname)_$(date +%Y%m%d_%H%M%S).csv"
echo "t_utc,t_unix,host,iface,mode,freq_mhz,width_mhz,net_id,peer_count,peer_mac,signal_dbm,signal_avg_dbm,signal_chains,tx_bitrate_mbps,rx_bitrate_mbps,tx_packets,rx_packets,tx_retries,tx_failed,tx_bytes,rx_bytes,inactive_ms,connected_s" > "$CSV"
echo "[rf_stats] writing $CSV" >&2
trap 'echo "[rf_stats] stopped, $(( $(wc -l < "$CSV") - 1 )) rows -> $CSV" >&2; exit 0' INT TERM
while :; do
  INFO=$(iw dev "$IFACE" info 2>/dev/null)
  MODE=$(awk '/^\ttype /{$1="";sub(/^ /,"");print}' <<<"$INFO")
  FREQ=$(grep -oP 'channel [0-9]+ \(\K[0-9]+' <<<"$INFO")
  WIDTH=$(grep -oP 'width: \K[0-9]+' <<<"$INFO")
  NETID=$(awk '/^\tssid /{print $2}' <<<"$INFO")
  # In mesh mode `info` has no ssid. Do NOT call `get mesh_param mesh_id`: on
  # rtw88 it dumps the whole multi-line parameter list, which used to be pasted
  # straight into the CSV and split every mesh row across ~30 lines.
  [ -z "$NETID" ] && NETID=$(iw dev "$IFACE" info 2>/dev/null | grep -oP 'mesh id \K.*' | head -1)
  # Belt and braces: every field that reaches the CSV is forced to one
  # comma-free token, so no command output can ever break the row structure.
  MODE=$(printf '%s' "${MODE:-NA}"  | tr -d '\r\n,' | cut -c1-24)
  FREQ=$(printf '%s' "${FREQ:-NA}"  | tr -d '\r\n,' | cut -c1-8)
  WIDTH=$(printf '%s' "${WIDTH:-NA}"| tr -d '\r\n,' | cut -c1-8)
  NETID=$(printf '%s' "${NETID:-NA}"| tr -d '\r\n,' | cut -c1-40)
  [ -z "$NETID" ] && NETID=NA
  TS=$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ); TU=$(date +%s.%3N); H=$(hostname)
  DUMP=$(iw dev "$IFACE" station dump 2>/dev/null)
  N=$(grep -c '^Station' <<<"$DUMP")
  if [ "${N:-0}" -eq 0 ]; then
    echo "$TS,$TU,$H,$IFACE,${MODE:-NA},${FREQ:-NA},${WIDTH:-NA},${NETID:-NA},0,NONE,,,,,,,,,,,,," >> "$CSV"
  else
    awk -v ts="$TS" -v tu="$TU" -v h="$H" -v ifc="$IFACE" -v mode="${MODE:-NA}" \
        -v fq="${FREQ:-NA}" -v w="${WIDTH:-NA}" -v nid="${NETID:-NA}" -v n="$N" '
      function flush_row(){
        if(mac!=""){
          printf "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
            ts,tu,h,ifc,mode,fq,w,nid,n,mac,sig,sigavg,chains,txr,rxr,txp,rxp,txret,txfail,txb,rxb,inact,conn
        }
        mac="";sig="";sigavg="";chains="";txr="";rxr="";txp="";rxp="";txret="";txfail="";txb="";rxb="";inact="";conn=""
      }
      /^Station/{ flush_row(); mac=$2 }
      /signal:/     { s=$2; gsub(/[^-0-9]/,"",s); sig=s;
                      c=""; for(i=3;i<=NF;i++){t=$i; gsub(/[^-0-9]/,"",t); if(t!="")c=(c==""?t:c "|" t)}; chains=c }
      /signal avg:/ { s=$3; gsub(/[^-0-9]/,"",s); sigavg=s }
      /tx bitrate:/ { txr=$3 }
      /rx bitrate:/ { rxr=$3 }
      /tx packets:/ { txp=$3 }
      /rx packets:/ { rxp=$3 }
      /tx retries:/ { txret=$3 }
      /tx failed:/  { txfail=$3 }
      /tx bytes:/   { txb=$3 }
      /rx bytes:/   { rxb=$3 }
      /inactive time:/ { inact=$3 }
      /connected time:/{ conn=$3 }
      END{ flush_row() }
    ' <<<"$DUMP" >> "$CSV"
  fi
  sleep "$(awk -v hz="$HZ" 'BEGIN{print 1/hz}')"
done
EOS
}

remote_iperf(){ cat <<'EOS'
#!/bin/bash
# Opt-in throughput probe. Injects traffic on the SAME LAN that carries zenoh and
# the SLAM topics - it perturbs what it measures. Off unless --iperf is passed.
PEER_IP="$1"; LOGDIR="$2"; EVERY="$3"; SECS="$4"
CSV="$LOGDIR/wifi_tp_$(hostname)_$(date +%Y%m%d_%H%M%S).csv"
echo "t_utc,host,peer_ip,tcp_mbps,retransmits,secs" > "$CSV"
echo "[rf_stats] throughput probe -> $CSV (every ${EVERY}s, ${SECS}s each)" >&2
trap 'exit 0' INT TERM
while :; do
  J=$(iperf3 -c "$PEER_IP" -t "$SECS" -J 2>/dev/null)
  M=$(python3 -c "import sys,json;d=json.load(sys.stdin);print(round(d['end']['sum_received']['bits_per_second']/1e6,2))" <<<"$J" 2>/dev/null || echo NA)
  R=$(python3 -c "import sys,json;d=json.load(sys.stdin);print(d['end']['sum_sent'].get('retransmits','NA'))" <<<"$J" 2>/dev/null || echo NA)
  echo "$(date -u +%Y-%m-%dT%H:%M:%SZ),$(hostname),$PEER_IP,$M,$R,$SECS" >> "$CSV"
  sleep "$EVERY"
done
EOS
}

# ---- commands --------------------------------------------------------------
DO_IPERF=0
cmd_start(){
  local drones=("$@")
  for d in "${drones[@]}"; do
    [ -n "${IPS[$d]:-}" ] || { echo "  $d: unknown drone"; continue; }
    ssh_d "$d" "mkdir -p $LOGDIR && cat > /tmp/rf_sampler.sh" < <(remote_sampler)
    ssh_d "$d" "chmod +x /tmp/rf_sampler.sh
      tmux has-session -t $SESS 2>/dev/null && { echo '  already running'; exit 0; }
      tmux new-session -d -s $SESS '/tmp/rf_sampler.sh $IFACE $LOGDIR $HZ 2>&1 | tee -a $LOGDIR/rf_stats_\$(hostname).log'
      sleep 1; tmux has-session -t $SESS 2>/dev/null && echo '  sampler started' || echo '  START FAILED'"
    if [ "$DO_IPERF" = 1 ]; then
      # peer = the other named drone
      local peer_ip=""
      for o in "${drones[@]}"; do [ "$o" != "$d" ] && peer_ip="${IPS[$o]}"; done
      if [ -n "$peer_ip" ]; then
        echo "  !! throughput probe ON - injects traffic on the LAN carrying zenoh/SLAM"
        ssh_d "$d" "cat > /tmp/rf_iperf.sh" < <(remote_iperf)
        ssh_d "$d" "chmod +x /tmp/rf_iperf.sh
          pgrep -f '[i]perf3 -s' >/dev/null || (setsid nohup iperf3 -s >/dev/null 2>&1 &)
          tmux has-session -t ${SESS}_tp 2>/dev/null || tmux new-session -d -s ${SESS}_tp '/tmp/rf_iperf.sh $peer_ip $LOGDIR $IPERF_EVERY $IPERF_SECS'
          echo '  throughput probe started -> peer $peer_ip'"
      fi
    fi
    printf '%-14s ' "$d"; echo
  done
}
cmd_stop(){
  for d in "$@"; do
    [ -n "${IPS[$d]:-}" ] || continue
    printf '%-14s ' "$d"
    # bracketed patterns so the sweeping shell never matches its own command line
    ssh_d "$d" "tmux kill-session -t $SESS 2>/dev/null; tmux kill-session -t ${SESS}_tp 2>/dev/null;
      pkill -f '[r]f_sampler.sh' 2>/dev/null; pkill -f '[r]f_iperf.sh' 2>/dev/null;
      pkill -f '[i]perf3 -s' 2>/dev/null; echo stopped" 2>&1
  done
}
cmd_status(){
  printf '%-12s %-9s %-22s %-9s %s\n' DRONE SAMPLER NEWEST_CSV ROWS MODE/FREQ
  for d in "$@"; do
    [ -n "${IPS[$d]:-}" ] || continue
    ssh_d "$d" "
      R=\$(tmux has-session -t $SESS 2>/dev/null && echo running || echo -)
      F=\$(ls -t $LOGDIR/wifi_rf_*.csv 2>/dev/null | head -1)
      N=\$([ -n \"\$F\" ] && echo \$(( \$(wc -l < \"\$F\") - 1 )) || echo 0)
      M=\$(iw dev $IFACE info 2>/dev/null | awk '/type/{t=\$2}/channel/{c=\$0}END{print t\" \"c}' | grep -oP 'managed|mesh point' )
      Q=\$(iw dev $IFACE info 2>/dev/null | grep -oP 'channel [0-9]+ \(\K[0-9]+')
      printf '%-12s %-9s %-22s %-9s %s\n' '$d' \"\$R\" \"\$(basename \${F:-none})\" \"\$N\" \"\$M \$Q MHz\"" 2>/dev/null
  done
}

ARGS=(); for a in "$@"; do case "$a" in --iperf) DO_IPERF=1;; *) ARGS+=("$a");; esac; done
CMD="${ARGS[0]:-}"; DRONES=("${ARGS[@]:1}"); [ ${#DRONES[@]} -eq 0 ] && DRONES=(ghost delta)
case "$CMD" in
  start)  cmd_start  "${DRONES[@]}" ;;
  stop)   cmd_stop   "${DRONES[@]}" ;;
  status) cmd_status "${DRONES[@]}" ;;
  *) sed -n '2,20p' "$0"; exit 2 ;;
esac
