#!/bin/bash
# bench_internet_laptop.sh - share this laptop's eduroam internet with the TP-Link
# LAN and auto-configure EVERY drone on the router, whatever port/IP each is on.
#
# Just plug everything into the router (any ports, wired and/or WiFi) and run this
# after a laptop reboot. It:
#   1. NATs eduroam -> the 192.168.0.x LAN (idempotent).
#   2. Auto-discovers each drone by hostname (via its WiFi static, else a subnet
#      scan) — no hardcoded per-drone IPs.
#   3. Runs each drone's bench_internet.sh, which itself auto-picks the right
#      interface (wired preferred, else WiFi) — so a drone in the router's WAN
#      port (no wired lease) transparently uses WiFi instead.
#   4. Prints a map of drone -> reachable IP -> internet status.

source "$(dirname "$(readlink -f "$0")")/credentials.env" 2>/dev/null
JETSON_PASS="${JETSON_PASS:?credentials.env missing — copy credentials.env.example and fill it in}"

WIFI=wlp2s0                 # eduroam (internet)
ETH=enx6c6e072201de         # USB dongle -> TP-Link router
LAN=192.168.0.0/24
GW_SELF=192.168.0.136       # this laptop's LAN address (the drones' gateway)

DRONES="delta buckshee ghost thunderstrike"
declare -A WIFI_STATIC=( [delta]=192.168.0.40 [buckshee]=192.168.0.60 [ghost]=192.168.0.50 [thunderstrike]=192.168.0.70 )

ssh_d() { sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 "$@"; }

# --- 1) NAT: share eduroam to the LAN (idempotent) ---
sudo sysctl -qw net.ipv4.ip_forward=1
sudo iptables -t nat -C POSTROUTING -o $WIFI -s $LAN -j MASQUERADE 2>/dev/null || \
    sudo iptables -t nat -A POSTROUTING -o $WIFI -s $LAN -j MASQUERADE
sudo iptables -C FORWARD -i $ETH -o $WIFI -j ACCEPT 2>/dev/null || \
    sudo iptables -A FORWARD -i $ETH -o $WIFI -j ACCEPT
sudo iptables -C FORWARD -i $WIFI -o $ETH -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || \
    sudo iptables -A FORWARD -i $WIFI -o $ETH -m state --state RELATED,ESTABLISHED -j ACCEPT
echo "[bench] NAT active: $LAN -> $WIFI (laptop is gateway $GW_SELF)"

# --- 2) scan the LAN for live hosts (for drones whose WiFi static changed) ---
echo "[bench] scanning $LAN ..."
LIVE=$(for i in $(seq 2 254); do (ping -c1 -W1 192.168.0.$i >/dev/null 2>&1 && echo 192.168.0.$i) & done; wait)

# reach <user> <ip>: true if SSH as that user succeeds AND hostname matches (=that drone)
reach() { [ "$(ssh_d "$1@$2" hostname 2>/dev/null)" = "$1" ]; }

# --- 3) find each drone (WiFi static first, else any live IP) and configure it ---
echo "[bench] ---- drones ----"
for d in $DRONES; do
  ip=""
  for cand in "${WIFI_STATIC[$d]}" $LIVE; do
    [ -z "$cand" ] && continue
    if reach "$d" "$cand"; then ip="$cand"; break; fi
  done
  if [ -z "$ip" ]; then
    printf "  %-14s NOT FOUND on the network\n" "$d"
    continue
  fi
  # kick the drone's auto-detecting bench script (routes over wired or WiFi itself)
  out=$(ssh_d "$d@$ip" "echo $JETSON_PASS | sudo -S /home/$d/bench_internet.sh" 2>/dev/null | grep '\[bench\]' | tail -1)
  printf "  %-14s @ %-15s %s\n" "$d" "$ip" "${out#\[bench\] }"
done
echo "[bench] done — drones with 'internet UP' are ready to develop/debug."
