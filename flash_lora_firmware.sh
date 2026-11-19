#!/bin/bash
# flash_lora_firmware.sh - build the Heltec LoRa firmware here, flash it onto
# one or more drones' radios, and verify what actually landed.
#
#   ./flash_lora_firmware.sh ghost delta buckshee
#   ./flash_lora_firmware.sh thunderstrike        # when it comes back
#   ./flash_lora_firmware.sh --check all          # report versions, flash nothing
#
# WHY THIS IS SAFE TO RUN ON A SUBSET (the thunderstrike case):
#   v7.4 (deaf/unheard phase re-roll + relayed sync, 2026-10-26) changes
#   neither the over-the-air packet nor the serial protocol either, so v7.3
#   and v7.4 radios interoperate; the recovery needs v7.4 on every radio.
#   v7.3 changes NO over-the-air packet format. It only adds two serial
#   message types (0x04 SET_POWER, 0x86 POWER_RESP). A v7.2 radio ignores
#   0x04 and never emits 0x86; an old bridge ignores 0x86. So a mixed
#   v7.2/v7.3 mesh interoperates normally -- the usual "never run mixed
#   firmware" rule is about the OTA format, which is untouched here.
#   The only difference is that un-flashed radios cannot be swept: they sit
#   at the compile-time default (10 dBm). Flash the stragglers on return and
#   re-run the sweep.
#
# Build happens on THIS laptop (arduino-cli + esp32 core). Flashing happens
# on the drone via esptool.py over the radio's own USB, so no cables move.

set -uo pipefail
HERE="$(dirname "$(readlink -f "$0")")"
source "$HERE/credentials.env" 2>/dev/null
PASS="${JETSON_PASS:?credentials.env missing}"
INO="$HOME/jfr_dev/radiohive/firmware/lora_mesh_v7_2_ros2.ino"
# Board = what the radios were actually built for (verified 2026-10-19: ghost's
# radio app partition is byte-identical to the June build for this FQBN).
FQBN="esp32:esp32:heltec_wireless_stick_lite_V3"
# The source hardcodes NODE_ID 0; each radio is built with its own id.
declare -A NODE=( [delta]=0 [buckshee]=1 [ghost]=2 [thunderstrike]=3 )
BUILD="/tmp/lora_fw_build"
# pure-python esptool 4.7.0 + pyserial + intelhex, run inside the drone's
# container (the containers have none of them).
# Fetched once from ghost's ~/.local (the one drone that had it) if absent.
EPT="$HERE/tools/esptool_py"
declare -A IPS=( [delta]=192.168.0.40 [buckshee]=192.168.0.60 \
                 [ghost]=192.168.0.50 [thunderstrike]=192.168.0.70 )
# FLEET_IP_<drone>=<address> overrides the table for one run (a drone that is only on the wired port)
for d in "${!IPS[@]}"; do v="FLEET_IP_$d"; [ -n "${!v:-}" ] && IPS[$d]="${!v}"; done; true

CHECK_ONLY=0; DRONES=()
for a in "$@"; do case "$a" in
    --check) CHECK_ONLY=1 ;;
    all) DRONES=(delta buckshee ghost thunderstrike) ;;
    delta|buckshee|ghost|thunderstrike) DRONES+=("$a") ;;
esac; done
[ ${#DRONES[@]} -eq 0 ] && { echo "usage: $0 [--check] <drone...|all>"; exit 1; }

ssh_d() { sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$1@${IPS[$1]}" "$2"; }

# Resolve the radio's tty: the CP2102 (10c4:ea60) in hub socket 3. Since
# 2026-11 the PX4 link is a CP2102 too (same serial number), in hub socket 2,
# and the ttyUSB numbering changes from boot to boot, so neither the chip nor
# the name tells them apart. A CP2102 that is the only one present and is not
# in socket 2 also counts (old layout with an FTDI for PX4). Flashing the wrong
# port would push ESP32 firmware at the flight controller link.
find_radio() {
    ssh_d "$1" 'only=""; n=0
    for t in /dev/ttyUSB*; do
        b=$(basename $t); p=$(readlink -f /sys/class/tty/$b/device)
        for i in 1 2 3 4; do p=$(dirname $p); [ -f "$p/idVendor" ] && break; done
        [ -f "$p/idVendor" ] || continue
        [ "$(cat $p/idVendor)" = "10c4" ] && [ "$(cat $p/idProduct)" = "ea60" ] || continue
        case $(basename $p) in
            *.3) echo $t; exit 0 ;;
            *.2) n=$((n+1)) ;;
            *)   n=$((n+1)); only=$t ;;
        esac
    done
    [ $n -eq 1 ] && [ -n "$only" ] && echo $only' 2>/dev/null | tr -d '\r'
}

echo "=== radios ==="
for d in "${DRONES[@]}"; do
    port=$(find_radio "$d")
    if [ -z "$port" ]; then echo "  $d: NO RADIO FOUND (no CP2102 in hub socket 3, or drone offline)"; continue; fi
    echo "  $d: radio on $port"
done
[ "$CHECK_ONLY" = "1" ] && exit 0

# Build one image per drone with that drone's NODE_ID; sets APP BOOT PART BOOTAPP.
build_for() {
    local d="$1" b="$BUILD/$1" n="lora_$1_node${NODE[$1]}"
    rm -rf "$b"; mkdir -p "$b/$n"
    sed -E "s/^#define NODE_ID[[:space:]]+[0-9]+/#define NODE_ID           ${NODE[$d]}/" \
        "$INO" > "$b/$n/$n.ino" || { echo "firmware source not found: $INO"; return 1; }
    grep -qE "^#define NODE_ID[[:space:]]+${NODE[$d]}\$" "$b/$n/$n.ino" \
        || { echo "could not set NODE_ID for $d"; return 1; }
    # NUM_NODES sizes the over-the-air packet: a mismatch with the radios
    # already flown (4) silently breaks the mesh.
    grep -qE "^#define NUM_NODES[[:space:]]+${#NODE[@]}([^0-9]|\$)" "$b/$n/$n.ino" \
        || { echo "REFUSING: source NUM_NODES != ${#NODE[@]} (fleet size)"; return 1; }
    echo "=== building $n ($FQBN, NODE_ID ${NODE[$d]}) ==="
    arduino-cli compile --fqbn "$FQBN" --output-dir "$b/out" "$b/$n" >/dev/null || return 1
    APP=$(ls "$b/out"/*.ino.bin 2>/dev/null | head -1)
    BOOT=$(ls "$b/out"/*.bootloader.bin 2>/dev/null | head -1)
    PART=$(ls "$b/out"/*.partitions.bin 2>/dev/null | head -1)
    BOOTAPP=$(ls "$b/out"/boot_app0.bin 2>/dev/null | head -1)
    [ -f "$APP" ] && [ -f "$BOOT" ] && [ -f "$PART" ] || { echo "incomplete build for $d"; return 1; }
}

if [ ! -f "$EPT/esptool/__init__.py" ] || [ ! -f "$EPT/serial/__init__.py" ] \
        || [ ! -f "$EPT/intelhex/__init__.py" ]; then
    echo "=== fetching the esptool bundle from ghost ==="
    mkdir -p "$EPT"
    for p in esptool serial intelhex; do
        sshpass -p "$PASS" scp -r -o StrictHostKeyChecking=no \
            "ghost@${IPS[ghost]}:/home/ghost/.local/lib/python3.10/site-packages/$p" "$EPT/" >/dev/null \
            || { echo "could not fetch $p from ghost"; exit 1; }
    done
fi

for d in "${DRONES[@]}"; do
    port=$(find_radio "$d"); [ -z "$port" ] && { echo "SKIP $d (no radio)"; continue; }
    echo ""
    build_for "$d" || { echo "SKIP $d (build failed)"; continue; }
    echo "=== flashing $d on $port ==="
    # the bridge holds the port open; it must be stopped or esptool cannot reset
    ssh_d "$d" "docker exec isaac_ros_realsense bash -c 'pkill -9 -f \"[l]ora_bridge\"' 2>/dev/null; true"
    # esptool runs INSIDE the container (root): on 2026-10-26 only ghost's
    # host user was in dialout or had esptool, so a host-side flash failed on
    # the other three. The pure-python esptool+pyserial bundle travels along.
    sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no "$d@${IPS[$d]}" "rm -rf /tmp/lora_flash; mkdir -p /tmp/lora_flash"
    sshpass -p "$PASS" scp -r -o StrictHostKeyChecking=no "$EPT" "$d@${IPS[$d]}:/tmp/lora_flash/ept" >/dev/null
    for f in "$BOOT" "$PART" "$BOOTAPP" "$APP"; do
        [ -f "$f" ] && sshpass -p "$PASS" scp -o StrictHostKeyChecking=no "$f" "$d@${IPS[$d]}:/tmp/lora_flash/$(basename "$f")" >/dev/null
    done
    ssh_d "$d" "docker exec isaac_ros_realsense rm -rf /tmp/lora_flash; docker cp /tmp/lora_flash isaac_ros_realsense:/tmp/ && \
        docker exec isaac_ros_realsense bash -c 'cd /tmp/lora_flash && PYTHONNOUSERSITE=1 PYTHONPATH=/tmp/lora_flash/ept \
        python3 -m esptool --chip esp32s3 --port $port --baud 460800 write_flash \
        0x0 $(basename "$BOOT") \
        0x8000 $(basename "$PART") \
        ${BOOTAPP:+0xe000 $(basename "$BOOTAPP")} \
        0x10000 $(basename "$APP")' 2>&1 | tail -6"
done

echo ""
echo "Next: restart the stack and confirm the radios are talking again, e.g."
echo "  ./launch_fleet_tmux.sh <drones>   (tab 7)"
echo "and verify TX power control with:"
echo "  ros2 topic pub --once /d<N>/lora/set_tx_power std_msgs/msg/Int8 '{data: 14}'"
echo "  ros2 topic echo /d<N>/lora/tx_power      # radio reports what it APPLIED"
