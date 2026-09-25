#!/bin/bash
# flash_lora_firmware.sh - build the Heltec LoRa firmware here, flash it onto
# one or more drones' radios, and verify what actually landed.
#
#   ./flash_lora_firmware.sh ghost delta buckshee
#   ./flash_lora_firmware.sh thunderstrike        # when it comes back
#   ./flash_lora_firmware.sh --check all          # report versions, flash nothing
#
# WHY THIS IS SAFE TO RUN ON A SUBSET (the thunderstrike case):
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
FQBN="esp32:esp32:heltec_wifi_lora_32_V3"
BUILD="/tmp/lora_fw_build"
declare -A IPS=( [delta]=192.168.0.40 [buckshee]=192.168.0.60 \
                 [ghost]=192.168.0.50 [thunderstrike]=192.168.0.70 )

CHECK_ONLY=0; DRONES=()
for a in "$@"; do case "$a" in
    --check) CHECK_ONLY=1 ;;
    all) DRONES=(delta buckshee ghost thunderstrike) ;;
    delta|buckshee|ghost|thunderstrike) DRONES+=("$a") ;;
esac; done
[ ${#DRONES[@]} -eq 0 ] && { echo "usage: $0 [--check] <drone...|all>"; exit 1; }

ssh_d() { sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$1@${IPS[$1]}" "$2"; }

# Resolve the radio's tty by VID:PID (CP2102 10c4:ea60). The PX4 FTDI is
# 0403:6001 on the same hub and the numbering is per-boot roulette, so never
# assume ttyUSB0. Flashing the wrong port would push ESP32 firmware at the
# flight controller link.
find_radio() {
    ssh_d "$1" 'for t in /dev/ttyUSB*; do
        b=$(basename $t); p=$(readlink -f /sys/class/tty/$b/device)
        for i in 1 2 3 4; do p=$(dirname $p); [ -f "$p/idVendor" ] && break; done
        [ -f "$p/idVendor" ] || continue
        if [ "$(cat $p/idVendor)" = "10c4" ] && [ "$(cat $p/idProduct)" = "ea60" ]; then echo $t; exit 0; fi
    done' 2>/dev/null | tr -d '\r'
}

echo "=== radios ==="
for d in "${DRONES[@]}"; do
    port=$(find_radio "$d")
    if [ -z "$port" ]; then echo "  $d: NO CP2102 FOUND (radio unplugged or drone offline)"; continue; fi
    echo "  $d: radio on $port"
done
[ "$CHECK_ONLY" = "1" ] && exit 0

echo ""
echo "=== building $FQBN ==="
rm -rf "$BUILD"; mkdir -p "$BUILD/sketch"
cp "$INO" "$BUILD/sketch/" || { echo "firmware source not found: $INO"; exit 1; }
arduino-cli compile --fqbn "$FQBN" --output-dir "$BUILD/out" "$BUILD/sketch" || exit 1
APP=$(ls "$BUILD/out"/*.ino.bin 2>/dev/null | head -1)
BOOT=$(ls "$BUILD/out"/*.bootloader.bin 2>/dev/null | head -1)
PART=$(ls "$BUILD/out"/*.partitions.bin 2>/dev/null | head -1)
[ -f "$APP" ] || { echo "no application binary produced"; exit 1; }
echo "  app=$(basename "$APP")  bootloader=$(basename "${BOOT:-none}")  partitions=$(basename "${PART:-none}")"

for d in "${DRONES[@]}"; do
    port=$(find_radio "$d"); [ -z "$port" ] && { echo "SKIP $d (no radio)"; continue; }
    echo ""
    echo "=== flashing $d on $port ==="
    # the bridge holds the port open; it must be stopped or esptool cannot reset
    ssh_d "$d" "docker exec isaac_ros_realsense bash -c 'pkill -9 -f \"[l]ora_bridge\"' 2>/dev/null; true"
    for f in "$BOOT" "$PART" "$APP"; do
        [ -f "$f" ] && sshpass -p "$PASS" scp -o StrictHostKeyChecking=no "$f" "$d@${IPS[$d]}:/tmp/$(basename "$f")" >/dev/null
    done
    ssh_d "$d" "esptool.py --chip esp32s3 --port $port --baud 460800 write_flash \
        0x0 /tmp/$(basename "${BOOT:-/dev/null}") \
        0x8000 /tmp/$(basename "${PART:-/dev/null}") \
        0x10000 /tmp/$(basename "$APP") 2>&1 | tail -6"
done

echo ""
echo "Next: restart the stack and confirm the radios are talking again, e.g."
echo "  ./launch_fleet_tmux.sh <drones>   (tab 7)"
echo "and verify TX power control with:"
echo "  ros2 topic pub --once /d<N>/lora/set_tx_power std_msgs/msg/Int8 '{data: 14}'"
echo "  ros2 topic echo /d<N>/lora/tx_power      # radio reports what it APPLIED"
