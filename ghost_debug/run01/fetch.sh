#!/bin/bash
# run01 fetch-only: grab the A/B bundles the detached Jetson script already produced.
# Run from laptop on TP-Link_073A:  bash ghost_debug/run01/fetch.sh   (~30 s)
set -u
GHOST="ghost@192.168.0.50"
PASS="abc123"
RUNDIR="$(cd "$(dirname "$0")" && pwd)"

SSHQ() { sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$GHOST" "$@"; }

if ! SSHQ 'echo connected'; then
    echo "FAIL: cannot reach ghost. If WiFi never came back on ghost, power-cycle it."
    exit 1
fi

if SSHQ 'test -f /home/ghost/ghost_debug_run01/DONE'; then
    echo "[fetch] test completed on ghost. Downloading..."
else
    echo "[fetch] WARNING: DONE marker missing — test incomplete or still running."
    echo "[fetch] status (last 5 log lines):"
    SSHQ 'tail -5 /home/ghost/gdbg_run01_nohup.log 2>/dev/null'
    echo "[fetch] downloading whatever exists anyway..."
fi

rm -rf "$RUNDIR/data"
sshpass -p "$PASS" scp -r -o StrictHostKeyChecking=no "$GHOST:/home/ghost/ghost_debug_run01" "$RUNDIR/data"

echo ""
echo "=== QUICK LOOK ==="
for ph in A_wifi_on B_radio_off; do
    echo "--- $ph ---"
    for f in c_hz_camera_imu c_hz_vslam_odom c_hz_fmu_in_vvo; do
        echo "  [$f]: $(grep -m1 'average rate' "$RUNDIR/data/$ph/$f.txt" 2>/dev/null || echo 'NO RATE OUTPUT')"
    done
    echo "  [Lost IMU in tab1]: $(grep -c 'Lost IMU' "$RUNDIR/data/$ph/tab1_pane.txt" 2>/dev/null || echo 0) occurrences"
done
echo ""
[ -d "$RUNDIR/data/A_wifi_on" ] && [ -d "$RUNDIR/data/B_radio_off" ] && echo "PASS: both bundles in $RUNDIR/data/" || echo "PARTIAL: see $RUNDIR/data/"
