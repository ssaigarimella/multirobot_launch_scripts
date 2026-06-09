# ghost WiFi-radio-breaks-VIO — FINDINGS log

**Symptom (as reported):** WiFi radio ON => VIO/FMU pipeline broken; radio OFF
(airplane mode, launched locally) => works. Earlier logs showed RealSense
`Lost IMU msgs ... drop ratio = 100%`.

**FMU link is serial** (`MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600`) — network-independent.

## Ruled out (do NOT retry)
| # | Item | Evidence |
|---|------|----------|
| R1 | `FASTRTPS_DEFAULT_PROFILES_FILE` fastdds_loopback.xml | Tried previously; dead end. |
| R2 | `ROS_LOCALHOST_ONLY=1` | Tried; still broken with radio on. |
| R3 | NTP clock stepping | set-ntp false + stopping chrony/timesyncd did not fix. |
| R4 | **WiFi radio physically breaking RealSense/USB (H-A1..A4)** | run01 phase A: with wifi ON and associated to TP-Link_073A, `/camera0/imu` = 200.06 Hz, `/visual_slam/tracking/odometry` = 30.66 Hz, `/fmu/in/vehicle_visual_odometry` = 30.01 Hz, vio_bridge logged VIO #3800+ steadily. No USB resets in dmesg. The radio does NOT starve the pipeline. |

## Run log
| Run | Variable changed | Verdict | Key evidence |
|-----|------------------|---------|--------------|
| 01 | none — A/B baseline | **A-phase decisive; B-phase contaminated** (cleanup pkill self-match killed its own shell → two stacks ran concurrently in B; B rates unusable) | See Confirmed facts. |
| 02 | ROS_DOMAIN_ID removed (0) for tab1+tab2, wifi ON throughout | SKIPPED | User went straight to the param fix; run03 confirmed H-DOM anyway. |
| 03 | PX4 UXRCE_DDS_DOM_ID 0→3 (QGC + FMU reboot), unmodified script env | **PASS — root cause confirmed, GOAL met** | wifi ON: VVO 29.87 Hz, vehicle_status received, xy/z/v_xy/v_z_valid all true, nav_state 2 (POSITION). run03/data/. The script's "FAIL" verdict was a grep -c cosmetic bug; see POSTMORTEM Ops lessons. |

## Confirmed facts
1. **run01/A (wifi ON, exact tmux-script mechanics, ROS_DOMAIN_ID=3):** entire
   vision pipeline healthy (IMU 200 Hz, cuVSLAM 30 Hz, VVO 30 Hz). The ONLY broken
   link: no `/fmu/out/*` topics in the ROS graph, `vehicle_status`/`vehicle_local_position`
   not received. (data/A_wifi_on/c_topic_list.txt, c_hz_*.txt, c_fmu_*.txt)
2. **Agent session with PX4 IS established over serial** — tab2 pane shows
   `session established`, participant + many datareaders/datawriters created
   (data/A_wifi_on/tab2_pane.txt lines ~1300-1690). PX4 client is alive; its
   entities just live on a different DDS domain than the container nodes.
3. **Config confound discovered:** `launch_local.sh` (the "working airplane mode"
   path) sets NO ROS_DOMAIN_ID (=> domain 0). The broken tmux/eduroam ghost scripts
   set ROS_DOMAIN_ID=3 — an UNCOMMITTED change (git diff HEAD shows domain IDs
   added after the last all-drones-successful commit 9f5d549). PX4 `UXRCE_DDS_DOM_ID`
   presumably still 0 => PX4 topics on domain 0, container nodes on domain 3,
   mutually invisible. Radio state was correlated with WHICH script was used,
   not causal.
4. `Lost IMU msgs ... drop ratio = 100%` warnings: one giant startup transient
   (568897 msgs / 2994 s gap) + sporadic 1-6 msg drops at ~30 Hz frame intervals,
   while measured IMU rate stayed 200 Hz. Cosmetic here; the earlier "100%" logs
   were very likely the startup transient.
5. Ops lesson: `docker exec bash -c "pkill -f PATTERN"` kills its own shell when
   PATTERN appears in the bash -c cmdline → use `pkill -f 'PATTER[N]'` bracket trick
   and verify with pgrep until empty.

## Current hypothesis (run02)
**H-DOM:** ROS_DOMAIN_ID=3 (container) vs UXRCE_DDS_DOM_ID=0 (PX4) mismatch is the
entire bug. Fix candidates: (a) drop ROS_DOMAIN_ID from ghost script (matches
last-known-good commit), or (b) set PX4 param UXRCE_DDS_DOM_ID=3 to keep the
per-drone domain isolation design. run02 tests (a)'s mechanics; choice between
(a)/(b) is a fleet-design decision for the user.
NOTE: thunderstrike tmux script sets ROS_DOMAIN_ID=4 (+ROS_LOCALHOST_ONLY=1) —
likely broken the same way; out of scope per change discipline.
