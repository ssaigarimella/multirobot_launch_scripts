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

LAN=192.168.0.0/24
GW_SELF=192.168.0.136       # address the drones' bench_internet.sh expects as gateway

# --- auto-detect interfaces so a swapped USB dongle does not break this ---
# WIFI: the wireless interface that currently has the default route (internet).
# ETH : any non-wireless, non-virtual interface with carrier. Preferred if it
#       already holds a 192.168.0.x address (i.e. it is the one on the TP-Link).
detect_wifi() {
    local d
    for d in /sys/class/net/*; do
        [ -d "$d/wireless" ] || continue
        [ "$(cat "$d/carrier" 2>/dev/null)" = 1 ] || continue
        echo "${d##*/}"; return 0
    done
    return 1
}
detect_eth() {
    local d n best=""
    for d in /sys/class/net/*; do
        n="${d##*/}"
        case "$n" in lo|docker*|veth*|br-*|virbr*|tailscale*|tun*|wg*) continue ;; esac
        [ -d "$d/wireless" ] && continue
        [ "$(cat "$d/carrier" 2>/dev/null)" = 1 ] || continue
        # an interface already on the bench subnet wins outright
        if ip -4 addr show dev "$n" 2>/dev/null | grep -q 'inet 192\.168\.0\.'; then
            echo "$n"; return 0
        fi
        [ -z "$best" ] && best="$n"
    done
    [ -n "$best" ] && { echo "$best"; return 0; }
    return 1
}

WIFI="${WIFI:-$(detect_wifi)}"
ETH="${ETH:-$(detect_eth)}"
[ -z "$ETH" ]  && { echo "[bench] no wired interface with carrier - plug the dongle into the TP-Link and re-run."; exit 1; }
[ -z "$WIFI" ] && { echo "[bench] no wireless interface with carrier - the laptop needs internet to share."; exit 1; }
echo "[bench] detected: eth=$ETH  wifi=$WIFI"

# The bench dongle must NOT provide this laptop's default route: the TP-Link has
# no internet, and a DHCP default via it (metric 100) beats eduroam (metric 600),
# which silently kills the very uplink we are trying to share. Pin never-default
# on whatever profile owns the dongle, then drop any default route it installed.
ETH_CON=$(nmcli -t -f NAME,DEVICE con show --active | awk -F: -v d="$ETH" '$2==d{print $1; exit}')
if [ -n "$ETH_CON" ]; then
    if [ "$(nmcli -g ipv4.never-default con show "$ETH_CON" 2>/dev/null)" != "yes" ]; then
        echo "[bench] $ETH ('$ETH_CON') was allowed to set the default route - pinning never-default"
        sudo nmcli con mod "$ETH_CON" ipv4.never-default yes 2>/dev/null || true
        sudo nmcli con up "$ETH_CON" >/dev/null 2>&1 || true
        sleep 2
    fi
fi
sudo ip route del default dev "$ETH" 2>/dev/null || true

# Sharing internet requires HAVING internet. Check the uplink before promising it.
if ! curl -sS -o /dev/null --max-time 8 http://connectivitycheck.gstatic.com/generate_204 2>/dev/null; then
    echo "[bench] WARNING: this laptop has no internet right now."
    echo "[bench]   default route: $(ip route show default | head -1)"
    echo "[bench]   The drones will get a route but no connectivity until the laptop is online."
fi

# The drones hardcode GW=192.168.0.136 in their own bench_internet.sh, so make
# sure this laptop actually answers on that address whatever the dongle gives us.
if ! ip -4 addr show dev "$ETH" | grep -q "inet ${GW_SELF}/"; then
    echo "[bench] adding $GW_SELF/24 to $ETH (dongle handed us $(ip -4 -br addr show dev "$ETH" | awk '{print $3}'))"
    sudo ip addr add "$GW_SELF/24" dev "$ETH" 2>/dev/null || true
fi

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
# Verify rather than assert: sudo may have failed (no tty, no cached creds) and
# a cheerful "NAT active" on top of a broken setup wastes a bench session.
NAT_OK=1
ip -4 addr show dev "$ETH" | grep -q "inet ${GW_SELF}/" || { echo "[bench] FAILED: $GW_SELF is not on $ETH"; NAT_OK=0; }
[ "$(cat /proc/sys/net/ipv4/ip_forward)" = 1 ]          || { echo "[bench] FAILED: ip_forward is off"; NAT_OK=0; }
sudo -n iptables -t nat -C POSTROUTING -o "$WIFI" -s "$LAN" -j MASQUERADE 2>/dev/null \
    || { echo "[bench] FAILED: MASQUERADE rule not installed for $WIFI"; NAT_OK=0; }
if [ "$NAT_OK" = 1 ]; then
    echo "[bench] NAT active: $LAN -> $WIFI (laptop is gateway $GW_SELF on $ETH)"
else
    echo "[bench] NAT NOT fully configured - the steps above need sudo."
    echo "[bench] Re-run this script from a terminal where sudo can prompt:"
    echo "[bench]     ./bench_internet_laptop.sh"
    echo "[bench] Continuing to the drone scan anyway (they will report 'not on bench')."
fi

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
