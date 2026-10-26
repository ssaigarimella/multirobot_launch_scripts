#!/bin/bash
# replay_fleet.sh - reconstitute the fleet view on the LAPTOP from collected bags.
#
#   ./replay_fleet.sh <run> [drones...] [options]
#   ./replay_fleet.sh viz_verify ghost delta --rviz
#
#   --rate R          playback rate (default 1.0)
#   --start-offset S  extra seconds to skip, ON TOP of the alignment offset
#   --loop            loop each bag
#   --paused          start paused; SPACE steps in the player terminal
#   --rviz [cfg]      also open RViz2 (default viz/fleet_overview.rviz)
#   --no-align        do NOT time-align the bags (play each from its own t=0)
#
# WHY BAGS, NOT A LIVE BRIDGE  (the T3 decision)
# ----------------------------------------------
# Record per drone, merge offline. The drones sit on DIFFERENT ROS_DOMAIN_IDs
# (delta 1, ghost 3) and cannot see each other or the laptop by default. The
# alternatives were considered and rejected:
#   * Widening the zenoh allow list would add the map+marker traffic measured
#     below (~0.7 MB/s ghost, ~4.1 MB/s delta) to the very radio link the RF
#     study is characterising, perturbing the measurement it exists to make.
#   * A domain bridge / per-domain daemon on the laptop only helps while the
#     laptop is present, and flights happen away from it.
# Bags are the only path that works after the fact, which is when figures get
# made, and it adds zero flight-time network load.
#
# TIME ALIGNMENT, AND ITS RESIDUAL
# --------------------------------
# The bags do not start together (measured: ghost 19:11:48.069 UTC, delta
# 19:11:58.244 UTC - 10.2 s apart, because the operator starts them in sequence).
# By default each bag is given --start-offset = (latest_start - own_start), so
# every bag begins at the same absolute UTC instant and the drones' content is
# aligned on WALL CLOCK, not on a shared event.
#
# Residual: the drones discipline against different public NTP servers on a
# 34 min poll, so cross-drone alignment is bounded at roughly +/-25-50 ms
# (see rf_stats.sh for the measurement). That is fine for merging trajectories
# and maps into a figure. Do NOT claim per-message cross-drone synchrony.
#
# CLOCK
# -----
# Exactly ONE player runs with --clock (the drone whose bag starts last, i.e.
# the alignment reference). Two players both publishing /clock would fight.
# RViz is launched with use_sim_time:=true so "now" is bag time - without it,
# RViz treats the recorded transforms as ancient and drops them with
# "Message removed because it is too old".
#
# NODES NEEDED LIVE DURING REPLAY
# -------------------------------
# One viz/fleet_frame_prefix.py per drone. Nothing else: no mesh visualiser and
# no ESDF slicer, because the `figures` profile records already-visualisable
# types (Marker / PointCloud2 / OccupancyGrid) rather than raw depth. If you
# ever need the nvblox mesh proper you must re-run nvblox over recorded depth,
# which the `full` profile exists for and which is a bench-only exercise.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$HERE/bags"
declare -A IDS=( [delta]=1 [buckshee]=2 [ghost]=3 [thunderstrike]=4 )

RUN=""; DR=(); RATE=1.0; EXTRA_OFF=0; LOOP=""; PAUSED=""; RVIZ=""; ALIGN=1
RVIZ_CFG="$HERE/viz/fleet_overview.rviz"
while [ $# -gt 0 ]; do case "$1" in
  --rate) RATE="$2"; shift 2 ;;
  --start-offset) EXTRA_OFF="$2"; shift 2 ;;
  --loop) LOOP="--loop"; shift ;;
  --paused) PAUSED="--start-paused"; shift ;;
  --no-align) ALIGN=0; shift ;;
  --rviz) RVIZ=1; case "${2:-}" in -*|"") ;; *) RVIZ_CFG="$2"; shift ;; esac; shift ;;
  delta|buckshee|ghost|thunderstrike) DR+=("$1"); shift ;;
  -h|--help) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
  *) [ -z "$RUN" ] && RUN="$1" || echo "ignoring arg: $1"; shift ;;
esac; done
[ -z "$RUN" ] && { echo "usage: ./replay_fleet.sh <run> [drones...] [--rviz]"; exit 1; }
[ ${#DR[@]} -eq 0 ] && DR=(ghost delta)

set +u; source /opt/ros/humble/setup.bash 2>/dev/null; set -u

# ---- topics to relay, as TOPIC=type. Only types stock Humble can load: the
# laptop has NO nvblox_msgs / multi_drone_nvblox / px4_msgs (verified
# 2026-08-28), and `ros2 bag play` already warns+skips those, so the six
# custom-typed coordination topics are recorded but not replayable here.
RELAY_TOPICS=(
  "/nvblox_node/color_layer_marker=visualization_msgs/msg/Marker"
  "/nvblox_node/static_esdf_pointcloud=sensor_msgs/msg/PointCloud2"
  "/nvblox_node/static_occupancy_grid=nav_msgs/msg/OccupancyGrid"
  "/nvblox_node/esdf_slice_bounds=visualization_msgs/msg/Marker"
  "/nvblox_node/workspace_bounds=visualization_msgs/msg/Marker"
  "/nvblox_shared_node/color_layer_marker=visualization_msgs/msg/Marker"
  "/nvblox_shared_node/static_esdf_pointcloud=sensor_msgs/msg/PointCloud2"
  "/nvblox_shared_node/static_occupancy_grid=nav_msgs/msg/OccupancyGrid"
  "/visual_slam/tracking/odometry=nav_msgs/msg/Odometry"
  "/fis/frontier_markers=visualization_msgs/msg/MarkerArray"
  "/fis/frontier_centroids=geometry_msgs/msg/PoseArray"
  "/fis/frontier_viewpoints=geometry_msgs/msg/PoseArray"
  "/exploration/markers=visualization_msgs/msg/MarkerArray"
  "/loop_closure_alignment=geometry_msgs/msg/TransformStamped"
)

bag_start(){ grep -oP 'starting_time:\s*\n?\s*nanoseconds_since_epoch:\s*\K[0-9]+' "$1/metadata.yaml" 2>/dev/null \
             || python3 -c "
import sys,re
t=open('$1/metadata.yaml').read()
m=re.search(r'nanoseconds_since_epoch:\s*(\d+)',t)
print(m.group(1) if m else 0)"; }

# ---- find the latest start so every bag can be aligned to it
LATEST=0; declare -A STARTS
for d in "${DR[@]}"; do
  bd="$DEST/$RUN/$d"
  [ -d "$bd" ] || { echo "missing bag: $bd"; exit 1; }
  s=$(bag_start "$bd" | head -1)
  STARTS[$d]="$s"
  [ "${s:-0}" -gt "$LATEST" ] && LATEST="$s"
done
# the drone whose bag starts LAST is the clock master and the alignment reference
CLOCK_MASTER=""
for d in "${DR[@]}"; do [ "${STARTS[$d]}" = "$LATEST" ] && CLOCK_MASTER="$d"; done

echo "=== replay '$RUN' : ${DR[*]} ==="
echo "clock master : $CLOCK_MASTER  (its bag starts last; only it runs --clock)"
PIDS=()
cleanup(){ echo; echo "stopping replay..."; for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
           # bracketed patterns: an unbracketed one matches this shell itself
           pkill -f '[f]leet_frame_prefix.py' 2>/dev/null; true; }
trap cleanup INT TERM EXIT

for d in "${DR[@]}"; do
  id="${IDS[$d]}"; bd="$DEST/$RUN/$d"
  off=0
  [ "$ALIGN" = 1 ] && off=$(python3 -c "print(round((${LATEST} - ${STARTS[$d]})/1e9, 3))")
  total=$(python3 -c "print(round($off + $EXTRA_OFF, 3))")

  # relay first, so it is subscribed before the player starts publishing
  RELAY_ARGS=(); for t in "${RELAY_TOPICS[@]}"; do RELAY_ARGS+=(--topic "$t"); done
  python3 "$HERE/viz/fleet_frame_prefix.py" --drone-id "$id" \
      "${RELAY_ARGS[@]}" --drop-edge "d$id/map->d$id/odom" --bag "$bd" \
      --ros-args -p use_sim_time:=true >/dev/null 2>&1 &
  PIDS+=($!)

  # remap every relayed topic + tf into /raw/d<N>/...
  REMAP=(--remap "/tf:=/raw/d$id/tf" "/tf_static:=/raw/d$id/tf_static")
  for t in "${RELAY_TOPICS[@]}"; do n="${t%%=*}"; REMAP+=("$n:=/raw/d$id$n"); done

  CLK=""; [ "$d" = "$CLOCK_MASTER" ] && CLK="--clock"
  echo "  $d (d$id): start-offset ${total}s ${CLK:+[clock master]}"
  ros2 bag play "$bd" --rate "$RATE" --start-offset "$total" \
       $CLK $LOOP $PAUSED "${REMAP[@]}" &
  PIDS+=($!)
done

if [ -n "$RVIZ" ]; then
  echo "  rviz2: $RVIZ_CFG"
  rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true >/dev/null 2>&1 &
  PIDS+=($!)
fi

echo
echo "Verify the merged TF tree in another shell:"
echo "  source /opt/ros/humble/setup.bash"
echo "  ros2 run tf2_tools view_frames          # one tree rooted at 'world'"
echo "  ros2 run tf2_ros tf2_echo d1/map d3/map # expect [0, -3.500, 0]"
echo
echo "Ctrl-C to stop."
wait
