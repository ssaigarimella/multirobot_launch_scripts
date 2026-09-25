# Fleet port baseline — 2026-09-18

Answers to the four questions in the 2026-09-11 handoff. Everything below was
read off the four robots today, not inferred from GitHub.

## 1. Deployed repository SHAs (identical on all four robots)

| Repository | Branch | SHA |
|---|---|---|
| `multi_drone_nvblox` | `feature/jfr-multi-drone-field` | `34501ba` (includes your coordination commit) |
| `active_exploration` | `feature/jfr-multi-drone-field` | `9528a05` |
| `px4_offboard` | `dev` | `87c25c5` |
| `radiohive` | `dev` | `d09503e` |
| `isaac_ros_nvblox` | `ekf2-nvblox-pose` | `7774f42` |
| `isaac_ros_visual_slam` | (unchanged) | `4c7b976` |
| `realsense-ros` | (unchanged) | `100dccc3` |

Verified in the compiled artifacts, not just the source tree: `vio_bridge`
contains the `reset_counter` handling on all four, and the guard binary
contains the new STOP reasons on all four.

## 2. Dirty / uncommitted state on the robots

- `isaac_ros_nvblox` — `nvblox_examples_bringup/config/nvblox/nvblox_base.yaml`
  is modified on **all four**, deliberately and identically:
  `workspace_bounds_min_corner_x_m: -25.0 -> -5.0`,
  `workspace_bounds_max_corner_x_m: 25.0 -> 40.0`.
  This shifts the mapped volume forward so the 35 m forward-only geofence is
  actually mapped. Total volume is smaller than stock, so memory does not
  regress. Uncommitted on purpose: it is field configuration, not a code change.
- `multi_drone_nvblox` on **thunderstrike only** — two staged-but-uncommitted
  binary artifacts: a TensorRT `.engine` built for that specific GPU/TRT
  version, and `models/sp_vocab_4096.pkl`. Machine-specific; must not be
  shared between robots or committed.
- Nothing else is dirty on any robot.

## 3. Actual launch entry point

`./launch_fleet_tmux.sh ghost delta` from the `multi-robot-coordination`
repository on the laptop. It opens one terminal window per robot with eight
Enter-gated tabs, each running in a named tmux session **on the robot** so an
SSH drop detaches instead of killing the stack:

1. cuVSLAM + RealSense + nvblox (self map, collision)
2. VIO bridge + MicroXRCE-DDS agent
3. FIS (frontier detection)
4. Reactive depth guard
5. Exploration planner (team geofence)
6. Shared mapper (2nd nvblox, peer keyframes)
7. Coordination (alignment + LoRa claims + keyframe exchange + metrics)
8. Zenoh bridge (host side, bench LAN) + WiFi RF sampler

`./fleet_ctl {status|preflight|start|stop|collect}` is the headless equivalent
and the same launch command is generated from one definition, so the two paths
cannot drift. `multirobot_launch_scripts` is NOT the hardware entry point.

Important for your item 5: the fleet planner launch **does** pass `vehicle_id`
(from `drone_id`), so the ID-0 alignment rejection does not apply to this path.

## 4. Platform versions (identical on all four)

- L4T **R36.4.0** (GCID 37537400, 2024-09-13), JetPack **6.2.1+b38**
- Ubuntu **22.04**, kernel **5.15.148-tegra**
- ROS 2 **Humble**, running inside the `isaac_ros_realsense` container
  (hand-mutated image, no Dockerfile); the workspace is built with
  `colcon --symlink-install`, so `install/**` symlinks into the container's
  `src/**` and deployed == checked-in source for ROS packages
- Flight controller: CubePilot **Cube Orange+**. PX4 firmware version needs a
  QGC read to state authoritatively; `UXRCE_DDS_DOM_ID` equals the vehicle ID
  on each robot (ghost = 3 confirmed in flight logs)

## 5. What we have already integrated, and what we changed

Your two branches merged onto our heads as fast-forwards with **zero
conflicts** — they are rebased on our field code, so the conflict warning in
the handoff note is stale. 121 tests pass on the merge.

- `multi_drone_nvblox` `34501ba` (your coordination commit) — **deployed
  fleet-wide today**, all four rebuilt together because the `ClaimIntent` type
  hash changes.
- `active_exploration` — we took the ~40-line blacklist subset you identified
  (`_blacklist_goal`, `_goal_is_blacklisted`, `plan_goal_xy`, the EXECUTE-STOP
  call, the candidate filter) rather than the full planner port, as ROS
  parameters instead of `HERC_*` environment variables. Cooldown defaults to
  **30 s**, not 90 s (see below).
- We did **not** take `px4_offboard` or `multirobot_launch_scripts`.

Two fixes of our own that your audit did not have, both relevant to your
items 3 and 4:

1. **The guard's STOP was ambiguous.** `guard_state_ = STOP` was set by five
   conditions (obstacle, depth stale, planner stale, no planner, no position)
   but `reactive_guard/status` published a bare `STOP`. Our sustained-STOP
   escape therefore discarded good goals on a camera hiccup or on the
   planner's own slow cycle. The guard now publishes `STOP:<REASON>` and the
   planner abandons only on `OBSTACLE`. This is why we start the blacklist
   cooldown at 30 s: with reasons gated, only a real obstacle can trigger it.
2. **Your item 7 was real and worse than stated.** The guard calls the planner
   stale at 5 s while our planning budget was 10 s, so a legal slow cycle drove
   the guard to STOP and the planner then discarded the goal it was still
   working on. Budget is now 3 s, below the guard's threshold.

## 6. Validation status, stated plainly

Bench-verified today: guard publishes named reasons live on the real binary;
blacklist parameters load; planner runs with zero errors; LoRa radio up.
Unit tests: blacklist logic 5/5, reason parser 7/7, 121 package tests.

**Not yet validated in flight.** The blacklist can only be exercised when the
planner holds a reachable goal in EXECUTE, which a hand-carried robot cannot
reproduce (walking marks its own frontiers visited and smears the
camera-attached ESDF slice). Our next step is a single-robot flight on ghost,
then ghost+delta, then all four.

Still open on our side before a four-robot flight, independent of your code:
delta's EKF2 external-vision parameters (it has never fused VIO), buckshee and
thunderstrike PX4 parameter verification, and real placement measurements in
`swarm_alignment.yaml` for vehicles 2 and 4.
