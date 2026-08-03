#!/bin/bash
# launch_fleet_tmux.sh - multi-drone field stack entry point (spec block 9).
# Replaces the hand-listed per-node tabs with ONE fleet_bringup launch per
# drone (tmux on the drone, survives SSH drops, pane logged).
#
#   ./launch_fleet_tmux.sh ghost              # bench pose (cuvslam)
#   ./launch_fleet_tmux.sh ghost delta --ekf2 # flight pose source
#   ./launch_fleet_tmux.sh all
#   ./launch_fleet_tmux.sh attach ghost       # view the running launch
#   ./launch_fleet_tmux.sh stop all
#
# Thin wrapper over ./fleet_ctl (same fleet table + credentials).

HERE="$(dirname "$(readlink -f "$0")")"
case "${1:-}" in
    attach)
        source "$HERE/credentials.env"
        declare -A IPS=( [delta]=192.168.0.40 [buckshee]=192.168.0.60 [ghost]=192.168.0.50 [thunderstrike]=192.168.0.70 )
        d="${2:?usage: attach <drone>}"
        exec sshpass -p "$JETSON_PASS" ssh -t -o StrictHostKeyChecking=no \
            "$d@${IPS[$d]}" "tmux attach -t fleet"
        ;;
    stop)
        shift; exec "$HERE/fleet_ctl" stop "$@"
        ;;
    *)
        exec "$HERE/fleet_ctl" start "$@"
        ;;
esac
