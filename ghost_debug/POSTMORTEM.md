# POSTMORTEM — "WiFi radio breaks VIO" on ghost (2026-06-09)

## TL;DR
The radio was never the problem. An uncommitted change added `ROS_DOMAIN_ID=3` to
ghost's SSH launch scripts, while PX4's `UXRCE_DDS_DOM_ID` was still 0. PX4's DDS
topics therefore lived on domain 0, invisible to the domain-3 container nodes, so
EKF2 never received external vision. The "works in airplane mode" tests used
`launch_local.sh`, which sets NO domain (=> 0) — so radio state was perfectly
correlated with which script was used, never causal.

**Fix (applied + validated):** set PX4 param `UXRCE_DDS_DOM_ID = 3` on ghost via
QGC and reboot the FMU. No change to the launch script's behavior; a comment was
added documenting the invariant. (run03: VVO 29.9 Hz, xy/z/v_xy/v_z_valid all
true, nav_state 2 = POSITION, with WiFi ON and the unmodified tmux script.)

## Root cause
`MicroXRCEAgent` creates DDS participants on the domain the *PX4 client* requests
(`UXRCE_DDS_DOM_ID`, read once at uxrce_dds_client startup; default 0). The
container's `ROS_DOMAIN_ID` env has no effect on the agent's entities. With the
container nodes on domain 3 and PX4 on domain 0:
- vio_bridge published `/fmu/in/vehicle_visual_odometry` on domain 3 → PX4 never heard it;
- PX4's `/fmu/out/*` writers existed only on domain 0 → invisible to the stack and CLI.

The per-drone domain IDs (2=buckshee, 3=ghost, 4=thunderstrike) were added during
the multi-drone map-sharing work *after* the last known-good commit (9f5d549,
"tests on all 4 drones successful" — git diff HEAD shows the addition), without
matching PX4 param updates.

## Why it looked like a radio bug
Every radio-ON test ran via the SSH scripts (domain 3); every radio-OFF test ran
via `launch_local.sh` (domain 0). Two variables changed together; the visible one
got blamed. The scary `Lost IMU ... drop ratio = 100%` logs were a startup
transient (one 568k-message gap at init) plus sporadic 1–6-message drops — the
measured IMU rate was a rock-steady 200 Hz even with WiFi on
(run01/data/A_wifi_on/c_hz_camera_imu.txt, tab1_pane.txt:1107-1153).

## Evidence chain
1. **run01 phase A** (wifi ON, exact tmux-script mechanics): `/camera0/imu`
   200.06 Hz, `/visual_slam/tracking/odometry` 30.66 Hz, VVO 30.01 Hz, vio_bridge
   at VIO #3800+, no USB/dmesg errors → kills RF-interference/power-save/IRQ
   theories (H-A1..A4). Files: run01/data/A_wifi_on/.
2. **Agent log**: XRCE session established over serial; participant + datareaders
   + datawriters all created (run01/data/A_wifi_on/tab2_pane.txt:1300-1690) — yet
   no `/fmu/out/*` on domain 3 (c_topic_list.txt) → entities on another domain.
3. **Script diff**: `launch_local.sh:19` has no ROS_DOMAIN_ID; tmux script line 33
   sets ROS_DOMAIN_ID=3; git diff HEAD shows domain IDs are post-success additions.
4. **run03** (after UXRCE_DDS_DOM_ID=3 + FMU reboot, unmodified script env, wifi
   ON): vehicle_status received, xy_valid/z_valid/v_xy_valid/v_z_valid true,
   VVO 29.87 Hz, nav_state 2 (POSCTL). Files: run03/data/.

Ruled out along the way (pre-existing + this investigation): fastdds_loopback
profile, ROS_LOCALHOST_ONLY=1, NTP stepping, RF/USB interference, wifi power-save,
IRQ contention, NetworkManager thrash.

## Permanent fix & fleet notes
- **ghost:** PX4 `UXRCE_DDS_DOM_ID = 3` (set 2026-06-09, persisted in FMU flash).
  Launch script unchanged functionally; comment added at the DOCKER_SOURCE line.
- **Invariant for ALL drones:** PX4 `UXRCE_DDS_DOM_ID` must equal the launch
  script's `ROS_DOMAIN_ID` (1=delta, 2=buckshee, 3=ghost, 4=thunderstrike — set
  the param on each FMU if/when its script pins a domain).
- **thunderstrike WARNING:** its tmux script sets `ROS_DOMAIN_ID=4` AND
  `ROS_LOCALHOST_ONLY=1`. It needs `UXRCE_DDS_DOM_ID=4`; the localhost-only flag
  is fine for the FMU link (agent + nodes share the host) but blocks any
  cross-machine DDS. Not changed here (change discipline: ghost only).
- **Lab hygiene:** when comparing "working" vs "broken" setups, diff the full env
  of both launch paths first — run01's A/B rig is reusable for that.

## Ops lessons (for future test scripts)
- `docker exec bash -c "pkill -f PATTERN"` kills its own shell if PATTERN appears
  in the cmdline → use the `PATTER[N]` bracket trick (this corrupted run01 phase B:
  two stacks ran concurrently, B-phase rates unusable).
- `grep -c X file || echo 0` emits TWO lines when grep finds nothing (grep -c
  already prints 0 and exits 1) → caused run03's cosmetic "FAIL" verdict despite
  all real criteria passing.
- A freshly restarted ros2 daemon may not list agent-bridged topics within ~15 s
  even while `ros2 topic echo` on those topics succeeds — judge liveness by data
  delivery, not `topic list`.

## Cleanup (optional, next time on TP-Link)
sshpass -p abc123 ssh -o StrictHostKeyChecking=no ghost@192.168.0.50 \
  'rm -rf /home/ghost/ghost_debug_run0* /home/ghost/gdbg*'
(The gdbg3 tmux sessions ARE the live stack from run03 — kill via the launch
script's `kill` mode when done flying.)
