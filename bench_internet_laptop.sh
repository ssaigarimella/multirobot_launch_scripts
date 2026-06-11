#!/bin/bash
# bench_internet_laptop.sh - share this laptop's eduroam internet with the TP-Link LAN.
# Run after every LAPTOP reboot (rules are not persistent). Requires sudo.
# Drones pick it up automatically via their bench-internet.service (gateway 192.168.0.136).

source "$(dirname "$(readlink -f "$0")")/credentials.env" 2>/dev/null
JETSON_PASS="${JETSON_PASS:?credentials.env missing — copy credentials.env.example and fill it in}"

WIFI=wlp2s0              # eduroam (internet)
ETH=enx6c6e072201de      # USB dongle -> TP-Link router
LAN=192.168.0.0/24

set -e
sudo sysctl -qw net.ipv4.ip_forward=1

# -C checks if the rule exists so reruns don't add duplicates
sudo iptables -t nat -C POSTROUTING -o $WIFI -s $LAN -j MASQUERADE 2>/dev/null || \
    sudo iptables -t nat -A POSTROUTING -o $WIFI -s $LAN -j MASQUERADE
sudo iptables -C FORWARD -i $ETH -o $WIFI -j ACCEPT 2>/dev/null || \
    sudo iptables -A FORWARD -i $ETH -o $WIFI -j ACCEPT
sudo iptables -C FORWARD -i $WIFI -o $ETH -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || \
    sudo iptables -A FORWARD -i $WIFI -o $ETH -m state --state RELATED,ESTABLISHED -j ACCEPT

echo "[bench] NAT active: $LAN -> $WIFI (laptop is gateway 192.168.0.136)"

# Kick the bench script on any wired drone that's reachable, so drones that booted
# before the laptop was up get their route/DNS without manual SSH.
# Wired (ethernet) IPs only — update as drones get cabled / reserved in the router.
# Wired IPs are DHCP leases — if a kick fails, re-scan and update (or reserve
# these MACs in the TP-Link admin so they stop moving).
DRONES=(
    "ghost:192.168.0.206"
    "delta:192.168.0.236"
    "buckshee:192.168.0.124"
    # thunderstrike: no LAN port free (cable is in WAN port — dead). Reachable via WiFi .70;
    # internet route applied manually until a gigabit switch frees up a LAN port for it.
)
for entry in "${DRONES[@]}"; do
    user="${entry%%:*}"; ip="${entry##*:}"
    if ping -c1 -W1 "$ip" >/dev/null 2>&1; then
        echo "[bench] kicking bench_internet.sh on $user@$ip"
        sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 "$user@$ip" \
            "echo $JETSON_PASS | sudo -S /home/$user/bench_internet.sh" 2>/dev/null | grep "\[bench\]"
    else
        echo "[bench] $user@$ip not reachable, skipping"
    fi
done
