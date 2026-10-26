# Agent prompt 8: recording and RViz2 visualization for paper figures

## Objective

Make it possible to record a multi-drone exploration run and replay it on the laptop in RViz2,
with both drones' maps, trajectories, frontiers, and coordination state visible together in one
correctly aligned view, good enough to export publication figures.

Two deliverables: a recording path that runs as part of a normal flight, and a replay plus
RViz2 configuration that reconstitutes the fleet view offline. Both checked into the repo.

## Established facts, verified. Do not re-derive these.

**Launch path.** `./launch_fleet_tmux.sh ghost delta` from `/home/ssg/multi-robot-coordination`
on the laptop. One gnome-terminal window per drone, 8 Enter-gated tabs, each tab an SSH session
running a named tmux session `fleet_tab<N>` **on the drone** so an SSH drop detaches instead of
killing the node. Flags: `--debug` (skip arm check), `--ekf2`, `--mesh` / `--bench` (transport),
`--headless`. Teardown `./launch_fleet_tmux.sh stop`, or `stop --mesh` after a mesh run.

**Tab map.**
1. cuVSLAM + RealSense + nvblox (`nvblox_examples_bringup realsense_example.launch.py`)
2. VIO bridge + uXRCE-DDS agent (`px4_offboard vio_bridge.launch.py`)
3. FIS, frontier detection (`active_exploration fis.launch.py`)
4. Reactive depth guard (`active_exploration reactive_guard.launch.py`)
5. Exploration planner (`multi_drone_nvblox planner_stage.launch.py`)
6. Shared mapper, second nvblox over peers' keyframes (`shared_mapper.launch.py`)
7. Coordination: alignment + LoRa + keyframes (`coordination_stage.launch.py`), also starts
   `metrics_recorder.py`
8. Zenoh bridge, host side

**Identity and isolation.** Each drone runs in its **own ROS_DOMAIN_ID equal to its drone id**:
delta 1, buckshee 2, ghost 3, thunderstrike 4. Nodes are namespaced per drone, for example
`-r __ns:=/d3` on ghost. **This is the central problem for a combined view: two drones' topics
are not visible to each other or to the laptop by default.** Cross-drone traffic today goes
through a `zenoh-bridge-ros2dds` with a deliberately narrow allow list (loop closure keyframes,
`/loop_closure_alignment`, `/drone*/octomap`, `/drone*/octomap_incremental`,
`/drone*/odom_decimated`). Everything else stays local to its drone.

**Paths.** Workspace on the drone: `/mnt/nova_ssd/workspaces/isaac_ros-dev` on the host, the
same tree mounted as `/workspaces/isaac_ros-dev` inside the `isaac_ros_realsense` container.
Tabs 1 to 7 run **inside** the container, tab 8 on the host. All pane output is piped to
`experiment_logs/pane_<sess>_<epoch>.log` on the drone. `/mnt/nova_ssd` is the only volume with
room; do not write bags to the eMMC root.

**Collection.** `./fleet_ctl collect ghost delta` rsyncs each drone's `experiment_logs/` into
`collected/<drone>/` on the laptop. Standing rule from the user: **anything that must survive
leaving the lab has to end up in the laptop repo**, not only on a drone.

**Existing per-run CSVs** already written by `metrics_recorder.py` (tab 7): `lora_rx_*`,
`link_*`, `swarm_*`, `vio_health_*`, `keyframes_*`, `coverage_*`.

**Frames.** nvblox and the planner run with `global_frame` = `odom` and a per-drone prefix, base
frame `<prefix>camera0_link`. There is a `swarm_alignment.yaml` prior, and tab 7 runs with
`trust_prior_alignment:=true`. `/loop_closure_alignment` carries the runtime correction between
drones' frames.

**Topics that exist** (partial, from source; **you must enumerate live, see T0**):
`/frontiers`, `/exploration_goal`, `/exploration_path`, `/exploration_tour`,
`/exploration_tour_labels`, `/exploration_waypoints`, `/exploration_geofence`, `/loop_closures`,
`/loop_closure_markers`, `/loop_closure_alignment`, `swarm/peers`, `swarm/pose`,
`/nvblox_node/static_esdf_pointcloud`, `/nvblox_node/dynamic_occupancy_grid`,
`/nvblox_node/back_projected_depth/...`, `/nvblox_node/shapes_to_clear`, `/tf`, `/tf_static`.

**Bandwidth warning, measured.** The one real multi-drone bag in the repo,
`loop_closure_20260224_154605/robot_1_cuvslam`, holds **67859 messages in 41.7 seconds** and is
dominated by raw and compressed RealSense color and depth streams plus IMU. Recording all
`/camera0/**` for a ten minute flight is not viable on either disk or CPU during flight.
**Topic selection is the core design decision of this task, not an afterthought.**

**Clock.** Same-drone timestamps are exact. Cross-drone alignment is bounded at roughly plus or
minus 25 to 50 ms because the drones discipline against different public NTP servers on a 34
minute poll. Good enough to merge trajectories and maps for a figure. Not good enough to claim
per-message cross-drone synchrony.

## Hard constraints

- Do not modify PX4, cuVSLAM, nvblox, Cyclone, FastDDS, or the radio firmware.
- Do not edit `launch_fleet_tmux.sh`, `fleet_ctl`, or the mesh bring-up scripts without saying so
  explicitly and showing the diff first. Prefer adding a new tab or a standalone script.
- Do not install anything, system wide or in a venv. Do not install or upgrade numpy, torch,
  cv2, cupy, or anything CUDA linked. Report missing tools as missing.
- Never name a tmux session matching `^(fleet|zenoh|rviz|loratest)` unless you intend
  `fleet_ctl stop` to sweep it. That filter is how teardown finds sessions.
- Bracket every `pkill` pattern, for example `pkill -f '[m]ynode'`. Unbracketed patterns match
  the sweeping shell itself over SSH. This has bitten this project repeatedly.
- Do not disturb a running flight stack. Recording must be additive.
- Never touch the drones' WiFi or NetworkManager configuration. delta's only network path is its
  WiFi; taking it down strands the vehicle.
- If any established fact above contradicts what you find, stop that task and report the
  contradiction verbatim rather than working around it.

## Tasks

### T0. Enumerate live before designing anything. This is a gate.

Static grep over the source misses runtime remaps, namespaces, and image transport plugins.
Bring the stack up on **both** drones (`./launch_fleet_tmux.sh ghost delta`, tabs 1 to 7) and,
from inside the container on each drone, capture with the correct `ROS_DOMAIN_ID`:

- `ros2 topic list -t` (full list with types)
- `ros2 topic hz` and `ros2 topic bw` for every candidate, sampled at least 10 s each
- `ros2 node list`, `ros2 node info` for the exploration, nvblox, and coordination nodes
- `ros2 run tf2_tools view_frames` on each drone, and save the resulting PDF or the raw
  `/tf_static` dump

Report the full inventory as a table: topic, type, rate, bandwidth, publisher node, drone.
**Everything downstream depends on real numbers here, so do not estimate bandwidth.**

### T1. Topic selection and bandwidth budget

From the T0 inventory, define **three named recording profiles** and justify each inclusion
against measured bandwidth:

- `figures` : the minimum set that reconstitutes the paper visuals. Expect map or ESDF or mesh,
  trajectories, TF, frontiers, goals, paths, coordination and claim markers, peer state. No raw
  camera.
- `diagnostic` : `figures` plus decimated odometry, loop closure keyframes and markers, planner
  internals, depth guard state. For debugging a run after the fact.
- `full` : everything including camera streams. Bench only, explicitly marked as not for flight.

For each profile state total MB per minute per drone, measured, and the projected size of a ten
minute two drone run. State the free space on `/mnt/nova_ssd` on each drone and whether the
profile fits with margin.

If nvblox publishes a mesh or ESDF that is large but low rate, say so and prefer it over
reconstructing a map from raw depth at replay time. If the map is only reconstructable by
replaying depth, say that plainly, because it changes the whole design.

### T2. Recording tooling

Write a script, standalone and additive, that records a chosen profile on each drone:

- Runs `ros2 bag record` **inside the container** with the correct `ROS_DOMAIN_ID` and namespace
  per drone, writing to `/mnt/nova_ssd/workspaces/isaac_ros-dev/experiment_logs/bags/<run>/<drone>/`
- Runs in its own tmux session on the drone so an SSH drop does not kill it. Choose the session
  name deliberately with respect to the `^(fleet|zenoh|rviz|loratest)` sweep filter, and state
  which behavior you chose and why
- `start`, `stop`, `status` subcommands; `status` must prove the bag is **growing**, not merely
  that a file exists. A file with only a header is the failure mode that has already cost this
  project one run
- Records a run manifest: profile, topics, start and end UTC, drone, git SHAs of the three repos,
  ROS_DOMAIN_ID, transport (`--bench` or `--mesh`), and any free-text note
- Uses `--max-bag-size` or duration splitting so one long run does not become a single
  unmanageable file
- Refuses to start if free space on `/mnt/nova_ssd` is below a stated threshold

Verify by running it on both drones for at least 60 s and showing message counts per topic from
`ros2 bag info`.

### T3. Solve the cross-drone visibility problem, and justify the choice

Two drones on different `ROS_DOMAIN_ID`s cannot be viewed together without a deliberate
mechanism. Evaluate at least these and recommend one, with reasons:

1. **Record per drone, merge offline.** Replay both bags on the laptop into one domain with
   namespace remapping. No effect on the flight stack. Probably the right answer, but you must
   show how TF from two drones coexists without collision (see T4).
2. **Widen the zenoh allow list** so the laptop sees more live. Changes flight-time network load
   on the link the radio work is characterizing. If you propose this, quantify the added
   bandwidth and say explicitly that it perturbs the RF measurements.
3. **A domain bridge or `ros2 daemon` on the laptop** joining each domain in turn.

Whatever you recommend, it must work **offline from collected bags** as the primary path,
because flights happen away from the laptop and the user needs to make figures afterward.

### T4. TF and a common frame, the part most likely to go wrong

Both drones publish `odom` and `<prefix>camera0_link`. Merging two bags naively produces a TF
tree with duplicate or conflicting frames and RViz2 will either error or silently show one drone.

Deliver a concrete scheme:

- Per drone TF prefixing or remapping at replay time so frames do not collide
- A single common root, for example `world` or `map`, with a static transform per drone placing
  its `odom` into that root
- Where that transform comes from: `swarm_alignment.yaml`, `/loop_closure_alignment`, or Vicon
  via `vicon_bridge.launch.py`. State which is authoritative, and what to do when the runtime
  alignment differs from the prior
- Handling of `/tf_static`, which is latched and will be replayed at bag start for both drones
- A verification step: `ros2 run tf2_tools view_frames` on the merged replay, showing one
  connected tree with both drones under a common root, plus a numeric check that a known
  physical separation appears correctly

If the alignment between drones is not reliably recoverable offline, say so and propose the
fallback, for example figures showing each drone in its own frame side by side, or a manually
entered static transform recorded in the run manifest.

### T5. Collection to the laptop

Extend or complement `fleet_ctl collect` so bags come back too, into the laptop repo. Bags are
large: use rsync with resume, report transfer size and time, verify integrity with `ros2 bag info`
on the laptop copy, and do not delete the drone copy automatically. State where in the repo bags
should live and whether they belong in git or in a gitignored directory with a manifest tracked.

### T6. Replay

Deliver a replay script that takes a collected run and reconstitutes the fleet view:

- Plays both drones' bags with correct namespace and TF remapping
- `--rate`, `--start-offset`, `--loop`, and pause and step, for figure composition
- Handles the cross-drone clock offset. State the method and its residual, and whether you align
  on wall clock or on a shared event
- Publishes the static alignment transforms from T4
- Works with `--clock` and `use_sim_time` consistently, and says which nodes if any need to run
  live during replay, for example a mesh visualizer or an ESDF slicer

### T7. RViz2 configuration, checked in

Produce at least two `.rviz` configs in the repo:

- `fleet_overview.rviz` : both drones, maps or meshes, trajectories, frontiers, goals, current
  paths, coordination and claim markers, peer state, TF. Sensible colors distinguishing drones.
  This is the figure that goes in the paper
- `single_drone_detail.rviz` : one drone, mapping and planning internals, for a supporting figure

Requirements: fixed frame set to the common root, per drone color coding stated explicitly,
displays named readably rather than left as defaults, decay and history settings chosen so
trajectories accumulate into a visible path rather than flickering, and grid and scale sensible
for the flight volume. State which displays require which topics so a missing topic is
diagnosable.

### T8. Figure export

Describe and script, as far as is practical, the path from a replay to a publication figure:

- Deterministic viewpoint, saved camera pose in the `.rviz`
- High resolution capture, with the actual method verified, not assumed
- Whether RViz2 must run on the laptop with a display, or can run headless, and what works here
- A short recipe for the specific figures the paper needs: fleet overview at a chosen moment,
  coverage growth over time, frontier evolution, and the two drones' maps merged after a loop
  closure

If a capture method does not work in this environment, say so rather than giving a plausible
command that fails.

### T9. Documentation

`docs/` or `reports/visualization/`: a short operator guide covering record, collect, replay,
and export, with the exact commands, the failure modes seen, and how `status` proves a recording
is healthy. Include the T0 topic inventory as an appendix.

## Report

Write `reports/visualization/VISUALIZATION.md` covering every task, with the measured T0
inventory and T1 bandwidth table in full.

Print to stdout, alone, as the last thing in the run: the recommended cross-drone approach from
T3 with its one line justification, the `figures` profile topic list with its measured MB per
minute per drone, and the exact command sequence to record, collect, replay, and open the fleet
RViz2 config.
