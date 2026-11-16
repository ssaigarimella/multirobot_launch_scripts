# Field flights 2026-11-13 (delta + ghost): results and findings

Two flights with delta (vehicle 1) and ghost (vehicle 3) on an outdoor grass field, both
drones exploring autonomously at the same time in each. Times are laptop local (EST).

Source data, all on the laptop:
- `bags/f1113_1455/{delta,ghost}/` (flight 2, complete) and `bags/f1113_1444/` (35 s on the
  ground at the end of flight 1).
- `collected/{delta,ghost}/*_20261113_19{45,57}*.csv` and every tab's console log.
- PX4 ULogs are still on the two flight controllers' SD cards.

## 1. What flew

| | Flight 1 | Flight 2 |
|---|---|---|
| Offboard, delta | 14:47:20–14:52:06 (4 min 46 s) | 14:59:53–15:05:41 (5 min 48 s) |
| Offboard, ghost | 14:47:14–14:51:34 (4 min 20 s) | 14:59:57–15:05:39 (5 min 42 s) |
| Both in Offboard together | 4 min 14 s | 5 min 42 s |
| Path flown in Offboard | delta 104 m, ghost 120 m | delta 166 m, ghost 158 m |
| Furthest from Offboard start | not computed (no bag) | delta 31.2 m, ghost 19.2 m |
| Planner goals (plans) | not counted | delta 41, ghost 39 |
| Bag recorded | no (recording started 14:51:45, after the flight) | yes, both drones |

Flight 2 is the longest simultaneous autonomous flight so far (Oct 23: 91 s).

Flight 2 numbers, window 14:59:35–15:05:55:
- **Map:** 1.15 million voxels (delta) and 1.12 million (ghost). Ground footprint of the 3D
  maps: delta 917 m², ghost 843 m², union 1,415 m² (0.25 m cells under the voxels, after
  levelling, section 3.1).
- **Separation:** 4.4 m at the start, 35.5 m at most, 24.4 m on average.
- **LoRa coordination link:** about 10 packets per second each way. Delivery 99.7 % (delta
  receiving) and 99.8 % (ghost receiving). RSSI median −60 and −59 dBm (range −86 to −46),
  SNR median 11.5 and 12.5 dB. Peer state age at the receiver: median 0.15 s, worst 0.39 s.
- **Claims over LoRa:** delta published 45 claims (43 distinct points) and ghost received all
  43 points; ghost published 43 (35 distinct) and delta received all 35. On Oct 23 claims
  did not get through.
- **Map sharing over WiFi (access point at 5765 MHz, 80 MHz channel, not the mesh):** 356
  keyframes received in each direction (8.8 MB at delta, 8.0 MB at ghost, about 23 kB/s each
  way). A keyframe is a 320 × 240 depth image with the camera pose, one per second. Ghost
  sent 365, so about 97.5 % arrived. WiFi signal at the drones: median −62 dBm (delta) and
  −53 dBm (ghost), worst −75 and −68 dBm; link rate median 260 and 433 Mb/s.
- **The 3D maps are almost entirely ground.** After levelling, 99 % of delta's voxels lie
  within 1.03 m of the lowest level and 99 % of ghost's within 0.58 m; nothing is higher than
  1.4 m. The field is open and the camera looks down.
- **Reactive guard:** delta 17 stops totalling 1.5 s. Ghost 17 stops totalling 15.5 s, of
  which three (3.6, 5.8 and 4.9 s, 15:02:48–15:03:05) were 0.9–1.1 m short of an obstacle
  near the launch line.

Flight 1 link figures are in `reports/visualization/f1113_flight1/altitude_and_link.png`.
Delta received 100 %; ghost's cumulative delivery fell from 100 % to 96.9 % between
14:49:29 and 14:50:19 and ended at 97.8 %.

## 2. What worked

1. **Bring-up.** In both battery bring-ups each drone's camera tab started once and passed.
   The tracker was reset twice per start by the new helper (`console/cuvslam_settle.py`),
   as designed. On Oct 23 the same step needed eight restarts.
2. **Two drones autonomous together for 5 min 42 s**, each planning its own goals, with no
   pilot intervention in flight 2.
3. **LoRa coordination both ways**, including claims, at 99.7 % or better.
4. **Map keyframe exchange over WiFi** both ways for the whole flight.
5. **Recording** of flight 2 on both drones (12 GB), pulled and verified.

## 3. Findings

### 3.1 The map frame is tilted by how the drone sat on the ground

The mapper levels cuVSLAM's frame with a fixed 20° (the camera mount angle). The frame is
actually the camera's attitude at start, which includes how the vehicle rested on the grass.

| Flight | Drone | Camera pitch at start | Left in the map after the fixed 20° |
|---|---|---|---|
| 2 | delta | 24.3° | 4.3° |
| 2 | ghost | 28.7° | 8.7° |
| 1 | delta | 14.3° | −5.7° |
| 1 | ghost | 27.5° | 7.5° |

Flight 2 values come from the bag: the levelled track climbs 0.076 m per metre forward
(delta) and 0.152 m per metre (ghost) while PX4 held 0.9–1.0 m on its rangefinder. Sideways
the slope is under 0.2°. Flight 1 values come from the planner CSV (`viz/flight_from_csv.py`
explains the method; it reproduces the flight 2 values within 0.5°).

Consequences, all measured in flight 2:
- **The mapped ground climbs** 2.4 m over each flight.
- **The planner's 2D map slice cuts into the ground ahead of the drone.** Occupied cells sit
  straight ahead at 7.5–9.1 m for ghost and 8.4–16.5 m for delta at every time sampled, and
  never behind. Expected from the tilt: 7.2 m and 14.5 m. These are the dark arcs in
  `reports/visualization/f1113_1455/team_map.png`. Ghost therefore explored mostly sideways
  (16.5 m forward at most, against 31.5 m of sideways extent).
- **The ground leaves the mapper's height window.** The mapper keeps −0.4 to 3.2 m in its own
  frame. Ghost's voxels reach 3.43 m as flown, so its ground hits that ceiling about 23 m
  forward of its start and is not mapped beyond; behind the start it drops below the floor.
- **The planner's own pose is short along the forward axis** by cos(pitch): 0.91 (delta) and
  0.88 (ghost).
- **PX4's position, which is also what the drones send each other over LoRa, is shorter
  still**, by cos(pitch + 20°): 0.72 (delta) and about 0.65 (ghost). Ghost was 16 m forward
  when its broadcast said 10 m. This is the Oct 23 "40° out of level" finding, now with the
  resting pitch added.

So the planner, PX4 and the peers each use a different forward scale. Sideways all agree.

The fix is to measure the camera pitch (and roll) from gravity at start and use that angle in
`odom_correction`, in `vio_bridge` and in `ekf2_odom_tf`, in place of the constant 20°. It
replaces the sign-pair fix already planned and needs the slide test afterwards.

The videos and the footprint numbers use maps levelled in post-processing to each drone's
own flight plane (one rigid rotation per drone). `team_map.png`, `coverage.png` and
`paths_by_state.png` use the maps as flown.

### 3.2 Delta: onboard stall of about 13 s in flight 2 (15:03:43–15:03:56)

What stopped, from each log's own timestamps:

| Process | Hole in its log |
|---|---|
| metrics recorder (all six CSVs, Python) | 12.8 s from 15:03:43 |
| planner (Python) | 1.7 s from 15:03:46 and 6.4 s from 15:03:49 |
| frontier node | 15 s between outputs |
| reactive guard (C++) | 1.4 s and 2.5 s; it logged "Depth image stale (1.4 s)" |
| WiFi sampler on the host, outside the container | 2.7 s and 3.0 s |

What did not stop: camera, cuVSLAM, the mapper, `vio_bridge` (30 Hz to PX4 throughout), the
LoRa bridge (its own log shows frames arriving throughout) and the bag recorder (no gap over
0.1 s in the bag).

Effect on the flight: the planner logged "PX4 feed stale (6.4 s)" when it woke, went through
INIT and planned again 0.5 s later. The vehicle flew on to its goal during the pause (2.6 m).
The planner CSV shows the Offboard flag off for single rows at 15:03:48.56 and 15:03:55.83;
whether PX4 left Offboard for longer needs the ULog.

Cause: not established. The pattern (processes that append to files stall, the host sampler
included, while compute-only processes run on) points at a storage write stall. It does not
coincide with a bag file split (15:02:24 and 15:04:51). Nothing like it on ghost in flight 2
or on delta in flight 1.

The LoRa "gap" of 11.8 s in delta's `lora_rx` CSV and the "VIO jump 4.155 m" in its
`vio_health` CSV at 15:03:55 are artifacts of the recorder's pause, not events.

### 3.3 Ghost: PX4 serial link dropped at the end of flight 1

- 14:51:34 the pilot took ghost out of Offboard.
- 14:51:54 PX4 data to the Jetson stopped while ghost was still airborne at 1.3 m.
- 14:52:04 the agent logged `open device error, device: /dev/ttyUSB1, errno: 6`.
- 14:52:35 the port came back.

This is the second in-flight loss of this link on ghost (Oct 23, 14:22:25). It did not happen
in flight 2. The watchdog (`tools/ftdi_watchdog.sh`) is still not installed.

### 3.4 Recording

- Flight 1 was not recorded: recording started at 14:51:45, after the flight.
- Ghost's flight 2 bag had a 0-byte `metadata.yaml` (the recorder was cut off before it
  closed). The three data files are intact; the index was rebuilt on the laptop
  (`bags/f1113_1455/ghost/METADATA_REBUILT.txt`). The copy on ghost is unchanged.

### 3.5 Camera start-up kick (found and fixed before the flights)

In daylight the tracker initialises on overexposed first frames and on IMU samples stamped
about 11 s late, and takes a random kick of 0.03–20 m while the vehicle stands still. The
console now resets the tracker at once and again when the stream has settled, and restarts
the camera tab only when the kick between the two resets exceeds 0.5 m. Not committed yet:
`console/fleet_console.py`, `console/cuvslam_settle.py`, `tests/test_console_eval.py`.

### 3.6 Why exploration stalled and the drones revisited ground (flight 2)

The team's 3D map covered 50 % of the 35 × 35 m fence after 1 min, 78 % after 2 min 45 s, then
stayed there (79 % at 4 min, 81 % at 5 min) and ended at 86 %. Delta mapped no new ground for
90 s and ghost for about 2 min. The one large gap is the forward-left quarter (53 % covered;
the other three quarters 92 to 100 %). Figure: `where_the_goals_went.png`.

| | delta | ghost |
|---|---|---|
| Goals | 45 | 43 |
| Median hop to the next goal | 2.8 m | 2.2 m |
| Goals within 2.5 m of a fence edge | 58 % | 88 % (32 on the rear edge) |
| Goals on ground that drone had mapped more than 20 s earlier | 40 % | 60 % |
| Goals on ground either drone had mapped | 64 % | 65 % |
| Path over its own previously mapped ground | 40 % | 59 % |

Causes, each checked in the code or the data:
1. **Nearest frontier wins.** The score is gain / (distance + 1 m), so a small frontier 2 m
   away beats a large unexplored area 20 m away.
2. **The fence edge is a frontier forever.** In the frontier finder a free cell on the box
   boundary always counts as a frontier cell, so the edge can never be finished. A visited
   goal is blocked only for 60 s within 1.5 m, so the drone walks the edge in 2 m steps.
3. **The planning map forgets** (3.1, the slice in a tilted frame), so ground already mapped
   turns unknown and grows new frontiers.
4. **Ghost could not go forward.** Its map ends at the ground cut about 8 m ahead; it never
   got past 16.5 m. The unexplored quarter was on its side.
5. **Neither drone knew what the other had mapped.** The shared (fused) map exists but the
   frontier finder reads the own map (`fis_stage` default `esdf_service_name`).
6. **The lane-sweep fallback cannot switch on.** It needs obstacle density under 2 %, but the
   density counts unknown cells (−1000) as obstacles, and 41 to 75 % of the box was never
   known-free. Its other condition (over 80 % of frontiers at the edge) was not met either
   (at most 24 % for delta, 73 % for ghost). The mode stayed 0 for the whole flight.

### 3.7 Smaller observations

- Ghost spent 26.7 s in one planning period from 15:03:47 without a valid plan.
- Ghost was unreachable from the laptop for about 5 minutes at 13:55 on the bench while its
  own WiFi log shows it associated throughout. No cause found.

## 4. Outputs

In `reports/visualization/f1113_1455/` (flight 2):
- `f1113_1455_showcase_x4.mp4`: 101 s at 4× speed, 1080p (272 MB), and
  `f1113_1455_showcase_x4_share.mp4` (71 MB). 3D maps of both drones as they grow, trails,
  planner goals and frontiers, the LoRa link, a top view, link and footprint panels, and a
  closing summary.
- `f1113_1455_showcase_v2_x4.mp4` and `..._v2_x4_share.mp4`: the same video with, along the
  bottom, each drone's planning map (the 2D occupancy slice its planner reads, as flown, with
  its goal, frontiers and what it holds about the other drone over LoRa) and fuller LoRa and
  WiFi panels; on the right a nearest-obstacle trace from the reactive guards.
- `f1113_1455_showcase_frame_{034,064,098}s.png`: three frames from the first version.
- `f1113_1455_3d_x4.mp4`: the plain 3D replay on a light background.
- `team_map`, `coverage`, `comms`, `paths_by_state` (PNG and PDF), `events.csv`.

In `reports/visualization/f1113_flight1/` (flight 1, from CSVs): `trajectories` and
`altitude_and_link` (PNG and PDF).

New tools: `viz/fleet_showcase.py`, `viz/flight_from_csv.py`; `viz/fleet_3d_video.py` gained
`--level-flight` and `--trail-z`.

What the showcase video shows that is processing, not measurement (also in its footer): maps
levelled to each drone's flight plane; colours contrast-stretched; team frame from the
measured start offsets with no registration of the two maps. Voxels without a camera colour
(63 % of delta's, 77 % of ghost's: the depth camera sees wider than the colour camera) are
tinted with the mapping drone's colour.

## 5. Next steps, in order

1. **Gravity levelling at start** (3.1), then the slide test. Largest effect: it fixes the
   tilted map, the ground "wall" in the planner's slice and the three different forward
   scales.
2. **Pull the PX4 ULogs** of both flights (and Oct 23) before the cards fill:
   `python3 tools/pull_ulogs.py --last 8` with the Cubes on USB.
3. **Ghost's serial link** (3.3): install the watchdog; decide whether two in-flight drops
   change the decision to keep the adapter.
4. **A 1 Hz system logger** (CPU, memory, disk wait, kernel messages) started with the
   recording, so the next stall like 3.2 can be attributed.
5. **Console:** tick "record run" by default and warn when a drone is armed with no
   recording running.
6. **Commit** the console fix and the new viz tools.
