# Recording and RViz2 visualisation for paper figures

Field work: **2026-08-28**, ghost (d3) and delta (d1), tabs 1–7 up on both, both
vehicles stationary on the bench. Every rate and bandwidth below was measured
live on the vehicles; none is estimated.

---

## 0. Three findings that changed the design

**(a) `/mnt/nova_ssd` is not a separate volume — contradicts a stated fact.**
The brief states "`/mnt/nova_ssd` is the only volume with room; do not write bags
to the eMMC root." Measured on both drones:

```
$ findmnt -no SOURCE,TARGET,FSTYPE /mnt/nova_ssd   ->  NOT A MOUNTPOINT
$ lsblk        ->  nvme0n1 953.9G, nvme0n1p1 952.4G mounted at /     (no eMMC present)
$ df -h / /mnt/nova_ssd
  /dev/nvme0n1p1  937G  60G  839G   7% /       (ghost)
  /dev/nvme0n1p1  937G  84G  806G  10% /       (delta)
```

`/mnt/nova_ssd` is an ordinary directory on the root filesystem, and there is no
eMMC on either vehicle. Bags still go to `/mnt/nova_ssd/.../experiment_logs/bags`
for consistency, but the free-space guard checks the filesystem backing that
directory — the root fs. Reported rather than worked around, per the brief.

**(b) The laptop cannot deserialise any custom message type.** Verified:

| package | on laptop |
|---|---|
| `nvblox_msgs`, `nvblox_rviz_plugin` | **MISSING** |
| `multi_drone_nvblox` | **MISSING** |
| `isaac_ros_visual_slam_interfaces` | **MISSING** |
| `px4_msgs`, `radiohive` | **MISSING** |
| `realsense2_camera_msgs` | present |

So `/nvblox_node/mesh` (`nvblox_msgs/Mesh`) **cannot be replayed or rendered on
the laptop**, and neither can `DistanceMapSlice` or `VoxelBlockLayer`. Installing
the packages is out of scope. This is the single biggest constraint on the
`figures` profile, and it is why the profile is built from
`visualization_msgs/Marker`, `PointCloud2` and `OccupancyGrid` rather than the
nvblox-native types.

The map is **not** "only reconstructable by replaying depth". nvblox already
publishes a stock-typed rendering of the same map:
`/nvblox_node/color_layer_marker` (`visualization_msgs/Marker`, 4.8 Hz,
1.92 MB/s), which stock RViz2 renders directly. That is the map in every figure
here — no nvblox plugin, no depth replay.

`ros2 bag play` **degrades gracefully** on the missing types (measured, not
assumed): it warns per topic and plays the rest.

```
[WARN] [rosbag2_player]: Ignoring a topic '/swarm/peers', reason: package
       'multi_drone_nvblox' not found, searching: [/opt/ros/humble].
```

So the six custom-typed coordination topics are still recorded (they cost
~1 KB/s combined and are the scientific record), they simply do not appear on
the laptop. To use them, replay on a drone or install the package.

**(c) The brief's topic list is substantially stale.** Of 15 topics named,
**9 do not exist**; the real names differ. This is exactly what the T0 gate is
for.

| brief says | reality |
|---|---|
| `/frontiers` | absent → `/fis/frontier_markers`, `/fis/frontier_centroids`, `/fis/frontier_viewpoints`, `/fis/frontier_gains` |
| `/exploration_goal`, `/exploration_path`, `/exploration_tour`, `/exploration_tour_labels`, `/exploration_waypoints`, `/exploration_geofence` | **all absent** → `/exploration/markers`, `/exploration/state`, `/planning/trajectory_setpoint` |
| `/loop_closures`, `/loop_closure_markers` | **absent** |
| `/swarm/pose` | **absent** → `/swarm/peers` only |
| `/loop_closure_alignment`, `/swarm/peers`, `/nvblox_node/static_esdf_pointcloud`, `/nvblox_node/dynamic_occupancy_grid`, `/nvblox_node/shapes_to_clear` | present |

Also contradicted: the brief says the base frame is `<prefix>camera0_link`. It is
not — the camera frames carry **no prefix** (see §4).

---

## 1. T0 — live inventory

154 topics and 24 nodes on each drone, identical except for the `/d<N>/` set.
Full list: [`t0/topics_ghost.txt`](t0/topics_ghost.txt),
[`t0/topics_delta.txt`](t0/topics_delta.txt),
[`t0/nodes_ghost.txt`](t0/nodes_ghost.txt),
[`t0/nodes_delta.txt`](t0/nodes_delta.txt).
Full measured rate/bandwidth table: [`t0/measurements.tsv`](t0/measurements.tsv)
(83 rows) — reproduced as Appendix A.

**Topic names collide between drones.** The only per-drone-namespaced topics are
`/d<N>/alignment/*`, `/d<N>/keyframes/depth`, `/d<N>/lora/*`,
`/d<N>/swarm/link_stats`, `/d<N>/fault/state`. Everything else — `/tf`,
`/nvblox_node/**`, `/fis/**`, `/exploration/**`, `/visual_slam/**` — is
published under the *same name* by both drones. Nodes are likewise mostly
unnamespaced: only the six `multi_drone_nvblox` coordination nodes sit under
`/d<N>/`; `nvblox_node`, `visual_slam_node`, `camera0/camera`,
`frontier_info_structure`, `simple_exploration_planner`, `reactive_depth_guard`
are all at the root.

### Discovery is eventually-consistent — a real failure mode

During T2 verification `ros2 topic list` on delta returned a partial graph
(4 nodes) while all seven tabs were demonstrably healthy, which silently
narrowed a recording to **13 of 26 topics**. `--no-daemon` is *not* the fix — it
returned *fewer* topics (128 vs 154). `record_fleet.sh` therefore samples the
topic list until two consecutive reads agree, and writes any still-absent topic
into the run manifest.

---

## 2. T1 — profiles and bandwidth budget

Free space at measurement time: **ghost 839 GB, delta 806 GB** (root fs, §0a).

### `figures` — 26 topics. **This is the flight default.**

Chosen so the paper visuals reconstitute on a stock-Humble laptop. Inclusions:

| topic | why | measured (worst of the two drones) |
|---|---|---|
| `/nvblox_node/color_layer_marker` | **the map.** Stock `Marker`; renders without the nvblox plugin | 1.92 MB/s |
| `/nvblox_node/static_esdf_pointcloud` | map as `PointCloud2`; cheap, recolourable per drone | 667 KB/s |
| `/nvblox_node/static_occupancy_grid` | 2-D top-down map for a plan-view figure | 288 KB/s |
| `/nvblox_shared_node/static_esdf_pointcloud` | **peers' map** — the multi-drone claim | 675 KB/s |
| `/nvblox_shared_node/color_layer_marker`, `/nvblox_shared_node/static_occupancy_grid` | peer map, marker + 2-D | 0 / 62 KB/s |
| `/visual_slam/tracking/odometry` | **trajectory.** Chosen over `slam_path`/`vo_path` (435 / 473 KB/s, cumulative `Path`, grows without bound) — RViz accumulates the trail from Odometry at 1/20th the cost | 21.7 KB/s |
| `/tf`, `/tf_static` | frames; `tf_static` is latched | 6.7 KB/s |
| `/fis/frontier_*` ×4 | frontiers | ≤5.2 KB/s total |
| `/exploration/markers`, `/exploration/state` | goals, claim markers, planner state | 323 B/s |
| `/reactive_guard/status`, `/blocked` | guard state | 488 B/s |
| `/nvblox_node/esdf_slice_bounds`, `/workspace_bounds` | geofence/volume outline | 3.6 KB/s |
| `/loop_closure_alignment` | runtime inter-drone correction (`TransformStamped`, native) | event-driven |
| `/swarm/peers`, `/coordination/claim_intent`, `/coordination/mode_state`, `/d<N>/alignment/T_world_self`, `/d<N>/alignment/status`, `/d<N>/swarm/link_stats` | coordination record. Custom-typed → recorded, but not replayable on the laptop (§0b). ~1 KB/s combined, so kept | ≤572 B/s each |

Explicitly **excluded**: `/nvblox_node/back_projected_depth/...` at
**29.44 MB/s**, all raw camera, `mesh`/`tsdf_layer`/`color_layer`/`static_map_slice`
(nvblox-native types the laptop cannot read), and the cumulative SLAM paths.

**Measured end-to-end from the actual bags** (not summed estimates):

| drone | bag | duration | rate | 10 min |
|---|---|---|---|---|
| ghost | 72.7 MiB | 102.3 s | **42.6 MiB/min** | 426 MiB |
| delta | 401.9 MiB | 97.6 s | **247.0 MiB/min** | 2 470 MiB |
| **both** | | | **289.6 MiB/min** | **≈ 2.83 GiB** |

Fits with enormous margin (0.35 % of the smaller drone's free space).

**Why the 5.8× spread between drones:** the map topics scale with mapped surface
area, not with time. delta faced a richer scene, so its `mesh` was 3.80 MB/s
against ghost's 268 KB/s. **These numbers grow during a flight as the map grows.**
Budget on the delta column and treat 250 MiB/min/drone as the planning figure,
not the 42 MiB/min ghost happened to produce.

### `diagnostic` — `figures` + planner/SLAM/radio internals

Adds `slam_path` (435 KB/s), `vo_path` (473 KB/s), `static_map_slice` (1.15 MB/s),
`tsdf_layer_marker` (4.24 MB/s), shared `static_map_slice` (1.11 MB/s),
`keyframes/depth` (51 KB/s), peers' keyframes, all `/d<N>/lora/*`, PX4
`/fmu/out/*`, camera_info, `/diagnostics`, `/rosout`.

Added ≈ **7.5 MB/s**; total ≈ **11.6 MB/s ≈ 696 MiB/min per drone**
→ **≈ 13.6 GiB** for a 10 min two-drone run. *Computed from the per-topic
measurements, not measured end-to-end.* Fits comfortably.

### `full` — `diagnostic` + raw camera. **BENCH ONLY.**

Camera streams measured on ghost: depth raw 22.61, infra1 18.48, infra2 17.55,
color raw 9.26, compressedDepth 5.49 MB/s, plus IMU ≈ 0.2 MB/s ⇒ **74.25 MB/s**
of camera alone; ≈ **86 MB/s ≈ 5.15 GiB/min per drone**, ≈ **103 GiB** for a
10 min two-drone run.

It fits on disk but **must not be used in flight**: it competes with cuVSLAM for
CPU and will not keep up. The script prints a warning when this profile is
selected. This is consistent with the one existing multi-drone bag in the repo
(67 859 messages in 41.7 s, dominated by raw+compressed colour/depth/IMU).

---

## 3. T3 — cross-drone visibility: **record per drone, merge offline**

| option | verdict |
|---|---|
| **1. Record per drone, merge offline** | **Recommended.** Zero flight-time cost, and it is the only option that works after the fact — which is when figures get made. |
| 2. Widen the zenoh allow list | **Rejected.** The `figures` set is 0.71 MB/s (ghost) to 4.12 MB/s (delta) *measured*. Pushing that live would add 5.7–33 Mbit/s to the very radio link the RF study is characterising, **perturbing the measurement it exists to make**. |
| 3. Domain bridge / per-domain daemon on the laptop | **Rejected as primary.** Only works while the laptop is present and joined; flights happen away from it. Useful for live spot-checks, not for figures. |

Offline replay is the primary path, and it is verified working (§4, §6).

---

## 4. T4 — TF and a common frame

### What is actually there (measured, and it contradicts the brief)

Each drone runs **two disconnected TF trees**:

```
map  ->  odom  ->  camera0_link  ->  camera0_{depth,color,infra1,infra2,accel,gyro}_*     ~30 Hz, UNPREFIXED
world -> d<N>/map -> d<N>/odom                                                            1.3–1.5 Hz / static
```

`d<N>/odom` has **no children**. The brief's `<prefix>camera0_link` does not
exist. Merging two such bags naively gives two publishers of
`odom -> camera0_link` at 30 Hz each, and RViz shows one drone or flickers.

### Where the common root comes from — and it is authoritative

`world -> d<N>/map` is already published live by `alignment_manager`, and it
already encodes the yaml prior:

```
ghost:  world -> d3/map  =  [0.000, -3.500, 0.000], rotation identity
delta:  world -> d1/map  =  [0.000,  0.000, 0.000]   (reference_drone_id: 1)
```

which is exactly `swarm_alignment.yaml` (`ghost: range_m 3.5, bearing_deg 90.0`
— 3.5 m to delta's right, and right is −y in ROS FLU). **Authority order:**

1. **`swarm_alignment.yaml` via `world -> d<N>/map` in the bag** — authoritative,
   because the fleet flies with `trust_prior_alignment:=true`. It rides along in
   `/tf` automatically; nothing extra to publish at replay.
2. `/loop_closure_alignment` — the runtime correction. Recorded (native
   `TransformStamped`). If it disagrees with the prior, **the prior is what the
   run actually flew on**; treat the loop-closure value as a measurement of
   prior error and say so in the caption rather than silently re-aligning.
3. Vicon (`vicon_bridge.launch.py`) — ground truth when available, but it was not
   running for this work.

Because the alignment is recorded in `/tf`, it **is** reliably recoverable
offline. The fallback (side-by-side per-drone frames, or a hand-entered static
transform in the manifest) is not needed. Should it ever be, the manifest already
carries the fields to record it.

### The scheme

`viz/fleet_frame_prefix.py`, one instance per drone. Bag plays into
`/raw/d<N>/…`; the node republishes into `/d<N>/…` with every frame rewritten to
`d<N>/<frame>`, and into the global `/tf`, `/tf_static`.

Three details that each cost a debugging cycle, all now fixed and commented in
the source:

- **Never re-prefix a frame that already has a drone prefix.** delta publishes
  `d3/base_link_est` (ghost's peer-pose estimate). Prefixing it with `d1/` gave
  `d1/d3/base_link_est` and detached it. Guard: `^d\d+/`. `world` is likewise
  never prefixed.
- **Drop the `d<N>/map -> d<N>/odom` stub, but only on `/tf_static`.** After
  prefixing, the real dynamic `map -> odom` becomes that same edge; two
  publishers give `TF_REPEATED_DATA` and jitter. Dropping it on *both* streams
  orphans `d<N>/odom` and detaches the whole camera subtree — observed.
- **Read `/tf_static` straight from the bag, not from the player.**
  `--start-offset` skips everything before the offset, and latched `tf_static` is
  recorded at t≈0 — so ghost at `--start-offset 10.176 s` silently lost its
  camera extrinsics. The node reads the bag directly (`--bag`) and re-latches
  every 5 s.

### Verification — passed

`view_frames` on the merged replay: **one tree, single root `world`, 33 edges,
16 d1 frames + 17 d3 frames**, both camera subtrees complete.

```
world -> d1/map -> d1/odom -> d1/camera0_link -> d1/camera0_*
world -> d3/map -> d3/odom -> d3/camera0_link -> d3/camera0_*
world -> d3/base_link_est
```

Numeric check, offline, from two independently recorded bags:

```
$ ros2 run tf2_ros tf2_echo d1/map d3/map
- Translation: [0.000, -3.500, 0.000]
- Rotation: in RPY (degree) [0.000, -0.000, 0.000]
```

**3.500 m, matching the measured physical separation exactly.**

---

## 5. T5 — collection

`./collect_bags.sh <run> ghost delta`. rsync with `--partial --inplace` (resume),
reports size and rate, then verifies the **laptop** copy with `ros2 bag info` —
including an explicit "opens but contains 0 messages" check, and a "no
metadata.yaml" check. The drone copy is never deleted automatically.

Measured on the verification run: ghost 73 MB in 1 s (73 MB/s), delta 402 MB in
8 s (51 MB/s); both verified `OK`.

**Layout and git policy:**

```
bags/<run>/<drone>/                 rosbag2 dir            GITIGNORED (payload)
bags/<run>/<drone>.manifest.json    provenance             TRACKED
bags/<run>/COLLECTION.md            transfer + verify log  TRACKED
```

A 10 min run is ~2.9 GB and does not belong in git. The manifest does: it pins
the git SHA of all nine source repos, the profile, the exact topic list, the UTC
window, transport, domain id, free space at start and a free-text note — so an
archived bag stays fully described by the repo. This satisfies the standing rule
that anything leaving the lab lands in the laptop repo.

---

## 6. T6 — replay

`./replay_fleet.sh <run> ghost delta [--rate R] [--start-offset S] [--loop]
[--paused] [--rviz [cfg]]`. `--paused` starts the players paused; SPACE
resumes/pauses and → steps one message, in the player's own terminal.

**Time alignment.** The bags do not start together — measured 10.176 s apart
(ghost 19:11:48.069, delta 19:11:58.244 UTC), because the operator starts them in
sequence. Each bag gets `--start-offset = latest_start − own_start`, so all bags
begin at the same absolute UTC instant. **Alignment is on wall clock, not on a
shared event.**

**Residual: ±25–50 ms.** The drones discipline against different public NTP
servers on a 34 min poll (per `rf_stats.sh`). Fine for merging trajectories and
maps into a figure; **not** grounds for claiming per-message cross-drone
synchrony.

**Clock.** Exactly one player runs `--clock` — the one whose bag starts last,
i.e. the alignment reference. Two players both publishing `/clock` would fight.
RViz runs `use_sim_time:=true`; without it RViz treats the recorded transforms as
ancient and drops them.

**Nodes needed live during replay:** one `fleet_frame_prefix.py` per drone, and
nothing else. No mesh visualiser, no ESDF slicer — the `figures` profile records
already-visualisable types. Reconstructing the true nvblox mesh would mean
re-running nvblox over recorded depth, which is what `full` exists for and is a
bench exercise.

---

## 7. T7 — RViz2 configs

`viz/fleet_overview.rviz` and `viz/single_drone_detail.rviz`, generated by
`viz/make_rviz_configs.py` so colours and topic bindings stay consistent.

**Fixed frame: `world`.** Per-drone colour coding, stated explicitly:

| drone | colour | RGB | peer-map colour |
|---|---|---|---|
| delta (d1) | amber | `255;140;0` | `160;100;20` |
| ghost (d3) | cyan | `0;190;255` | `20;110;150` |

Peer maps use a desaturated version of the owner's colour so "what I mapped" and
"what my peer told me" are visually separable.

Displays are grouped per drone and named readably (e.g. *"ghost trajectory"*,
*"delta peer map (shared nvblox)"*). Grid is 40 × 1 m, covering the 35 × 35 m
team geofence and nvblox's 20 × 20 × 3 m workspace AABB.

**Trajectory accumulation:** the Odometry display uses `Keep: 100000` with
`Position Tolerance: 0.02`, which is what turns it into an accumulating path
rather than a single flickering arrow. Covariance is off — it clutters a figure.

**Which display needs which topic** (so a blank display is diagnosable):

| display | topic | if missing |
|---|---|---|
| `<drone>` map (nvblox colour layer) | `/d<N>/nvblox_node/color_layer_marker` | tab 1 down |
| `<drone>` ESDF cloud | `/d<N>/nvblox_node/static_esdf_pointcloud` | tab 1 down |
| `<drone>` occupancy grid | `/d<N>/nvblox_node/static_occupancy_grid` | off by default |
| `<drone>` trajectory | `/d<N>/visual_slam/tracking/odometry` | cuVSLAM not tracking |
| `<drone>` frontiers | `/d<N>/fis/frontier_markers` | tab 3 (FIS) down |
| `<drone>` frontier centroids/viewpoints | `/d<N>/fis/frontier_{centroids,viewpoints}` | tab 3 down |
| `<drone>` exploration / claim markers | `/d<N>/exploration/markers` | tab 5 down — **note this topic was silent for the whole bench session (0 msgs)**; it only populates when the planner is actually exploring |
| `<drone>` peer map | `/d<N>/nvblox_shared_node/static_esdf_pointcloud` | tab 6 down, or no peer keyframes |
| `<drone>` workspace bounds | `/d<N>/nvblox_node/workspace_bounds` | tab 1 down |
| TF | `/tf`, `/tf_static` | `fleet_frame_prefix.py` not running |

Verified rendering both drones together — see
[`fig_fleet_overview.png`](fig_fleet_overview.png) and
[`fig_single_drone.png`](fig_single_drone.png). Global Status: Ok, 31 fps.

---

## 8. T8 — figure export

**RViz2 will not start under the default Qt platform plugin on this laptop.**
It dies with

```
RenderingAPIException: Invalid parentWindowHandle (wrong server or screen)
  in GLXWindow::create
```

`QT_QPA_PLATFORM=xcb` fixes it completely — 0 render errors afterwards. The
laptop is GNOME on Wayland with Xwayland at `:0`, Intel UHD 620, direct
rendering yes. `capture_figure.sh` exports this itself.

**Headless is NOT possible here.** `Xvfb` is not installed and installing it is
out of scope. Figure export must run from the laptop's own desktop session; it
will not work over a plain SSH pipe with no display.

**Capture method (verified, not assumed):** ImageMagick `import -window <id>`.
`xdotool` and `wmctrl` are **not** installed, so the window is located with
`xwininfo -root -tree`. `--crop` trims the docks to the 3-D viewport
(1358 × 854).

**Resolution — an honest limit.** `import` captures at on-screen size: **1920 ×
937** here, 1358 × 854 cropped. RViz2 has no offscreen supersampling and no
`--resolution` flag, so there is *no* way to render at 4× from the command line.
To get a bigger figure you must give RViz a bigger window before capturing.
Upscaling afterwards adds no detail and should not be described as a high-res
render. At 1358 px wide and 300 dpi that is a 4.5 in figure — adequate for a
two-column layout, tight for full-page.

**Deterministic viewpoint.** The camera pose is saved in the `.rviz`
(`Distance 11.0, Yaw 1.57, Pitch 0.95, Focal 1.0/−1.75/0.0`, near-plan). RViz
only restores it if the block is exactly the shape it writes itself: the view
**must** be named `Current View`, needs the stereo sub-block, and `Saved` must be
null rather than `[]`. With any other name RViz silently discards
Distance/Yaw/Pitch and auto-frames — observed (config said 22 m, RViz opened at
4.99 m). `Target Frame` is left as `<Fixed Frame>`; setting it to `world` makes
the Orbit view re-centre and override Focal Point.

### Recipes

| figure | recipe |
|---|---|
| **Fleet overview at a chosen moment** | `./replay_fleet.sh <run> ghost delta --paused`, SPACE/→ to the instant, then `./capture_figure.sh out.png viz/fleet_overview.rviz --crop` |
| **Coverage growth over time** | Replay at `--rate 1`; capture at fixed wall intervals. Quantitative coverage should come from `coverage_*.csv` (already written by `metrics_recorder.py`), **not** from counting pixels in a screenshot. |
| **Frontier evolution** | `single_drone_detail.rviz`, enable only frontiers + map, `--rate 0.25`, capture at several offsets. |
| **Two maps merged after a loop closure** | Replay with `--start-offset` bracketing a `/loop_closure_alignment` message. **Caveat: this topic published 0 messages in the whole bench session** — no loop closure occurred with stationary drones. This figure needs a flight where one actually fires. |

---

## 9. Failure modes seen, and how `status` proves health

`./record_fleet.sh status <run> ghost delta` samples the bag's byte count twice,
5 s apart, and reports the delta — existence is never treated as health:

```
ghost    tmux=yes proc=1  files=1   size=14M    HEALTHY - grew 3.9M in 5 s
delta    tmux=yes proc=1  files=1   size=52M    HEALTHY - grew 20M in 5 s
```

Verdicts: `NOT RUNNING` (no session) · `SESSION UP but NO 'bag record' PROCESS`
· `BROKEN - bag dir is EMPTY` · `STALLED - N on disk but NO GROWTH in 5 s` ·
`HEALTHY - grew N in 5 s`.

Real failures hit during this work:

1. **Host path used inside the container.** `ros2 bag record` ran with
   `/mnt/nova_ssd/workspaces/...` inside the container, where only
   `/workspaces/...` exists → `cd: No such file or directory`, session alive, bag
   empty. `status` reported `STALLED … bag dir is EMPTY`. (The repo hit this
   before — commit `0ce7d64`.) Fixed with separate host/container roots.
2. **Partial discovery** silently halving the topic list (§1). Fixed with the
   stable-list loop; absent topics recorded in the manifest.
3. **mcap unreadable on the laptop.** Drones have
   `librosbag2_storage_mcap.so`; the laptop has only
   `rosbag2_storage_default_plugins` (sqlite3). Recording mcap would produce bags
   unopenable exactly where figures are made. **The recorder forces
   `--storage sqlite3`.**
4. **`pkill` matching the sweeping shell.** Bracketing the pattern is not
   sufficient if the *same command block* also contains the literal text
   (e.g. a heredoc that mentions `rviz2`) — the shell's own argv matches. Run
   sweeps in their own isolated command.

### tmux session naming — deliberate

The recorder's session is **`bagrec`**, which does **not** match
`^(fleet|zenoh|rviz|loratest)`, so `fleet_ctl stop` will **not** sweep it. That is
intentional: `ros2 bag record` only writes `metadata.yaml` on SIGINT, and a
session kill would leave volumes that `ros2 bag info` and `ros2 bag play` both
refuse to open. **Stop the recording before tearing the stack down.**
`record_fleet.sh stop` sends SIGINT, waits, then kills the session.

---

## Appendix A — measured inventory

Full machine-readable table: [`t0/measurements.tsv`](t0/measurements.tsv).
10 s samples per topic, `ros2 topic hz` and `ros2 topic bw`, both drones,
tabs 1–7 up, vehicles stationary. `0` means no publisher was active during the
sample — for event-driven topics (`loop_closure_alignment`, `coordination/*`,
`lora/*`) that reflects a stationary bench, not absence. Peers' keyframes read 0
because tab 8 (zenoh) was not started.

| topic | type | drone | Hz | bandwidth |
|---|---|---|---|---|
| `/nvblox_node/mesh` | `nvblox_msgs/Mesh` | ghost | 4.80 | 267.98 KB/s |
| `/nvblox_node/mesh` | `nvblox_msgs/Mesh` | delta | 4.50 | 3.80 MB/s |
| `/nvblox_node/color_layer_marker` | `visualization_msgs/Marker` | delta | 4.80 | 1.92 MB/s |
| `/nvblox_node/tsdf_layer_marker` | `visualization_msgs/Marker` | delta | 3.98 | 4.24 MB/s |
| `/nvblox_node/static_esdf_pointcloud` | `sensor_msgs/PointCloud2` | ghost | 9.15 | 105.75 KB/s |
| `/nvblox_node/static_esdf_pointcloud` | `sensor_msgs/PointCloud2` | delta | 9.23 | 666.68 KB/s |
| `/nvblox_node/static_map_slice` | `nvblox_msgs/DistanceMapSlice` | ghost | 8.99 | 669.14 KB/s |
| `/nvblox_node/static_map_slice` | `nvblox_msgs/DistanceMapSlice` | delta | 9.20 | 1.15 MB/s |
| `/nvblox_node/static_occupancy_grid` | `nav_msgs/OccupancyGrid` | ghost | 9.18 | 163.78 KB/s |
| `/nvblox_node/static_occupancy_grid` | `nav_msgs/OccupancyGrid` | delta | 9.20 | 288.46 KB/s |
| `/nvblox_node/tsdf_layer` | `nvblox_msgs/VoxelBlockLayer` | ghost | 9.57 | 330.82 KB/s |
| `/nvblox_node/tsdf_layer` | `nvblox_msgs/VoxelBlockLayer` | delta | 8.96 | 3.01 MB/s |
| `/nvblox_node/color_layer` | `nvblox_msgs/VoxelBlockLayer` | ghost | 9.74 | 330.08 KB/s |
| `/nvblox_node/color_layer` | `nvblox_msgs/VoxelBlockLayer` | delta | 9.28 | 3.44 MB/s |
| `/nvblox_node/combined_occupancy_grid` | `nav_msgs/OccupancyGrid` | ghost | 0 | 0 B/s |
| `/nvblox_node/dynamic_occupancy_grid` | `nav_msgs/OccupancyGrid` | ghost | 0 | 0 B/s |
| `/nvblox_node/esdf_slice_bounds` | `visualization_msgs/Marker` | ghost | 3.90 | 1.95 KB/s |
| `/nvblox_node/workspace_bounds` | `visualization_msgs/Marker` | ghost | 1.98 | 1.60 KB/s |
| `/nvblox_node/shapes_to_clear` | `visualization_msgs/MarkerArray` | ghost | 0 | 252 B/s |
| `/nvblox_node/back_projected_depth/camera0_depth_optical_frame` | `sensor_msgs/PointCloud2` | ghost | 22.37 | 29.44 MB/s |
| `/nvblox_shared_node/static_esdf_pointcloud` | `sensor_msgs/PointCloud2` | ghost | 9.48 | 92.76 KB/s |
| `/nvblox_shared_node/static_esdf_pointcloud` | `sensor_msgs/PointCloud2` | delta | 9.42 | 675.32 KB/s |
| `/nvblox_shared_node/static_map_slice` | `nvblox_msgs/DistanceMapSlice` | ghost | 9.48 | 243.45 KB/s |
| `/nvblox_shared_node/static_map_slice` | `nvblox_msgs/DistanceMapSlice` | delta | 9.47 | 1.11 MB/s |
| `/nvblox_shared_node/static_occupancy_grid` | `nav_msgs/OccupancyGrid` | ghost | 9.46 | 62.03 KB/s |
| `/nvblox_shared_node/mesh` | `nvblox_msgs/Mesh` | ghost | 0 | 0 B/s |
| `/nvblox_shared_node/color_layer_marker` | `visualization_msgs/Marker` | delta | 0 | 0 B/s |
| `/nvblox_shared_node/tsdf_layer_marker` | `visualization_msgs/Marker` | delta | 0 | 0 B/s |
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | ghost | 29.93 | 21.74 KB/s |
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | delta | 30.21 | 20.90 KB/s |
| `/visual_slam/tracking/slam_path` | `nav_msgs/Path` | ghost | 30.03 | 434.78 KB/s |
| `/visual_slam/tracking/vo_path` | `nav_msgs/Path` | ghost | 29.97 | 472.82 KB/s |
| `/visual_slam/vis/landmarks_cloud` | `sensor_msgs/PointCloud2` | ghost | 0 | 0 B/s |
| `/visual_slam/vis/pose_graph_nodes` | `geometry_msgs/PoseArray` | ghost | 0 | 0 B/s |
| `/fis/frontier_markers` | `visualization_msgs/MarkerArray` | ghost | 1.00 | 257 B/s |
| `/fis/frontier_markers` | `visualization_msgs/MarkerArray` | delta | ~1 | 5.23 KB/s |
| `/fis/frontier_centroids` | `geometry_msgs/PoseArray` | ghost | 1.03 | 31 B/s |
| `/fis/frontier_viewpoints` | `geometry_msgs/PoseArray` | ghost | 1.03 | 30 B/s |
| `/fis/frontier_gains` | `std_msgs/Float64MultiArray` | ghost | 1.03 | 17 B/s |
| `/exploration/state` | `std_msgs/String` | ghost | 20.00 | 320 B/s |
| `/exploration/state` | `std_msgs/String` | delta | 20.00 | 323 B/s |
| `/exploration/markers` | `visualization_msgs/MarkerArray` | ghost | 0 | 0 B/s |
| `/planning/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | ghost | 20.00 | 1.37 KB/s |
| `/reactive_guard/status` | `std_msgs/String` | ghost | 30.30 | 485 B/s |
| `/reactive_guard/blocked` | `std_msgs/Bool` | ghost | 0 | 0 B/s |
| `/coordination/claim_intent` | `multi_drone_nvblox/ClaimIntent` | ghost | 0 | 0 B/s |
| `/coordination/mode_state` | `multi_drone_nvblox/ModeState` | ghost | 0 | 0 B/s |
| `/swarm/peers` | `multi_drone_nvblox/SwarmPeers` | ghost | 5.26 | 150 B/s |
| `/swarm/peers` | `multi_drone_nvblox/SwarmPeers` | delta | 5.17 | 572 B/s |
| `/loop_closure_alignment` | `geometry_msgs/TransformStamped` | ghost | 0 | 0 B/s |
| `/tf` | `tf2_msgs/TFMessage` | ghost | 61.16 | 6.10 KB/s |
| `/tf` | `tf2_msgs/TFMessage` | delta | 67.66 | 6.73 KB/s |
| `/tf_static` | `tf2_msgs/TFMessage` | ghost | 0 (latched) | 0 B/s |
| `/pose` | `geometry_msgs/PoseStamped` | ghost | 0 | 0 B/s |
| `/transform` | `geometry_msgs/TransformStamped` | ghost | 0 | 0 B/s |
| `/d3/keyframes/depth` | `multi_drone_nvblox/KeyframePacket` | ghost | 0.99 | 36.53 KB/s |
| `/d1/keyframes/depth` | `multi_drone_nvblox/KeyframePacket` | delta | 0.98 | 51.47 KB/s |
| `/d1/keyframes/depth (peer, seen from ghost)` | `multi_drone_nvblox/KeyframePacket` | ghost | 0 | 0 B/s |
| `/d3/alignment/T_world_self` | `multi_drone_nvblox/AlignmentUpdate` | ghost | 1.00 | 418 B/s |
| `/d1/alignment/T_world_self` | `multi_drone_nvblox/AlignmentUpdate` | delta | 1.00 | 400 B/s |
| `/d3/alignment/status` | `multi_drone_nvblox/AlignmentStatus` | ghost | 1.00 | 56 B/s |
| `/d3/swarm/link_stats` | `multi_drone_nvblox/LinkStats` | ghost | 1.00 | 38 B/s |
| `/d3/fault/state` | `std_msgs/String` | ghost | 0 | 0 B/s |
| `/d3/lora/peer_pose` | `radiohive/PeerPose` | ghost | 0 | 0 B/s |
| `/d3/lora/peer_msg` | `radiohive/PeerMessage` | ghost | 0 | 0 B/s |
| `/d3/lora/peer_event` | `radiohive/PeerEvent` | ghost | 0 | 0 B/s |
| `/d3/lora/mesh_status` | `radiohive/MeshStatus` | ghost | 0 | 0 B/s |
| `/camera0/color/image_raw` | `sensor_msgs/Image` | ghost | 11.02 | 9.26 MB/s |
| `/camera0/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | ghost | 14.84 | 659.65 KB/s |
| `/camera0/depth/image_rect_raw` | `sensor_msgs/Image` | ghost | 31.50 | 22.61 MB/s |
| `/camera0/depth/image_rect_raw/compressed` | `sensor_msgs/CompressedImage` | ghost | 59.96 | 5.07 KB/s |
| `/camera0/depth/image_rect_raw/compressedDepth` | `sensor_msgs/CompressedImage` | ghost | 56.61 | 5.49 MB/s |
| `/camera0/infra1/image_rect_raw` | `sensor_msgs/Image` | ghost | 59.85 | 18.48 MB/s |
| `/camera0/infra2/image_rect_raw` | `sensor_msgs/Image` | ghost | 55.97 | 17.55 MB/s |
| `/camera0/imu` | `sensor_msgs/Imu` | ghost | 200.26 | 67.97 KB/s |
| `/camera0/accel/sample` | `sensor_msgs/Imu` | ghost | 199.08 | 68.63 KB/s |
| `/camera0/gyro/sample` | `sensor_msgs/Imu` | ghost | 200.26 | 68.14 KB/s |
| `/camera0/color/camera_info` | `sensor_msgs/CameraInfo` | ghost | 17.82 | 5.70 KB/s |
| `/camera0/depth/camera_info` | `sensor_msgs/CameraInfo` | ghost | 59.31 | 22.54 KB/s |
| `/camera0/depth/color/points` | `sensor_msgs/PointCloud2` | ghost | 0 | 0 B/s |
| `/camera0/realsense_splitter_node/output/depth` | `sensor_msgs/Image` | ghost | 21.87 | 16.17 MB/s |
| `/camera0/realsense_splitter_node/output/infra_1` | `sensor_msgs/Image` | ghost | 29.95 | 9.13 MB/s |
| `/camera0/realsense_splitter_node/output/pointcloud` | `sensor_msgs/PointCloud2` | ghost | 0 | 0 B/s |
