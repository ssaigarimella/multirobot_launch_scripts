#!/bin/bash
# Fully clears ghost before a fresh launch: tmux sessions AND container processes
# (tmux kill alone leaves docker-exec'd processes alive — the agent keeps /dev/ttyUSB0).
# Run from laptop on TP-Link:  bash ghost_debug/clean_ghost.sh
set -u
sshpass -p abc123 ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 ghost@192.168.0.50 '
tmux kill-server 2>/dev/null
docker exec -u admin isaac_ros_realsense bash -c "
  pkill -9 -f \"MicroXRCEAgen[t]\"; pkill -9 -f \"vio_bridg[e]\";
  pkill -9 -f \"component_containe[r]\"; pkill -9 -f \"realsense_exampl[e]\";
  pkill -9 -f \"ros2 launc[h]\"; true" 2>/dev/null
sleep 3
LEFT=$(docker exec -u admin isaac_ros_realsense bash -c "pgrep -af \"MicroXRCEAgen[t]|vio_bridg[e]|component_containe[r]|ros2 launc[h]\"" 2>/dev/null)
if [ -z "$LEFT" ]; then echo "[OK] ghost is clean — safe to launch."; else echo "[WARN] still running:"; echo "$LEFT"; fi
rm -rf /home/ghost/ghost_debug_run0* /home/ghost/gdbg* 2>/dev/null
'
