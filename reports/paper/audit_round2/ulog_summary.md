# PX4 ULog index (audit round 2)

Generated 2026-11-11 12:29 by `audit.py` + `report.py` in `/tmp/claude-1000/-home-ssg-multi-robot-coordination/32c52d39-8711-437c-ad1b-9af6f8a41b57/scratchpad/ulog_audit` (pyulog 1.1.0, read-only). Per-log detail: `ulog_index.csv`.

## Counts

- `.ulg` files found under `/home/ssg` (excluding `PX4-Autopilot`, `.cache`): **125**
- unique by sha256: **120** (5 contents appear at 2 paths)
- failed to parse: **0** (no exceptions; pyulog corruption flag set on 0; message framing walks cleanly to end of file on 120 of 120)

| directory | files |
|---|---|
| `/home/ssg/Desktop` | 25 |
| `/home/ssg/Desktop/GhostPX4Logs` | 60 |
| `/home/ssg/Desktop/PrometheusPX4Logs` | 1 |
| `/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs` | 34 |
| `/home/ssg/lunar_lab/multi-drone-project/code/PX4logs` | 2 |
| `/home/ssg/multi-robot-coordination/logs` | 1 |
| `/home/ssg/multi-robot-coordination/px4_logs` | 2 |

Duplicate contents:

- `5c14cd3b3ff2`: `/home/ssg/Desktop/GhostPX4Logs/log_0_2022-2-23-22-58-18.ulg` = `/home/ssg/Desktop/log_0_2022-2-23-22-58-18.ulg`
- `c9f6a94ac821`: `/home/ssg/Desktop/log_13_2025-10-27-01-33-46.ulg` = `/home/ssg/Desktop/log_13_2025-10-27-01-33-46_1.ulg`
- `36184f6b6915`: `/home/ssg/Desktop/log_57_UnknownDate.ulg` = `/home/ssg/multi-robot-coordination/logs/log_57_UnknownDate.ulg`
- `eaf6046df1ad`: `/home/ssg/Desktop/log_7_2026-8-18-15-55-56.ulg` = `/home/ssg/lunar_lab/multi-drone-project/code/PX4logs/log_7_2026-8-18-15-55-56.ulg`
- `2a260c100d04`: `/home/ssg/lunar_lab/multi-drone-project/code/PX4logs/log_71_UnknownDate.ulg` = `/home/ssg/multi-robot-coordination/px4_logs/log_71_UnknownDate.ulg`

## How the numbers were made (read before quoting)

- **Dates.** No log contains GPS samples (`vehicle_gps_position` / `sensor_gps` have no data in any of the 120), and the `time_ref_utc` info field is 0 in every log (it is the logger UTC offset, not a start time). The only in-log wall clock is `timesync_status.remote_timestamp` when the companion sent epoch time: 26 logs. Where both a filename date and a companion-clock UTC exist they agree to within a minute once the filename is read as local time (UTC-4 or UTC-5), except the conflict listed under Anomalies. `date_best` = filename date (local) if present, else companion-clock UTC, else `unknown`. File mtime is given only as a hint (it is download/copy time, an upper bound).
- **Durations.** `vehicle_status` and `vehicle_land_detected` are treated as sample-and-hold and integrated from log start to the last timestamp. Armed = `arming_state == 2`; Offboard = `nav_state == 14` (armed or not); in-air = armed and `landed == 0` (before the first land-detector sample the vehicle is taken as landed). Log duration is last minus first timestamp of the loaded topics (error below about 1 s).
- **Medians.** Hover thrust and battery medians use the samples whose timestamp falls in an in-air interval; hover-thrust count excludes NaN samples. Battery uses instance 0 (no log has more than one instance), fields `current_a` and `voltage_v`.
- **Parameters** are from the log header (`initial_parameters`). "Changed mid-log" means a parameter message in the data section with a value different from the one in force.
- **Dropouts.** Logs with many logger dropouts have thinned samples; durations are still bridged by sample-and-hold, but medians rest on fewer samples than the rate would suggest.

## EKF2_EV_CTRL value -> logs

| EKF2_EV_CTRL | meaning | logs | dated logs | date range (date_best) | undated logs | their file-mtime range |
|---|---|---|---|---|---|---|
| 0 | EV fusion off | 12 | 7 | 2025-09-01 .. 2025-09-26 | 5 | 2025-10-03 .. 2025-10-03 |
| 9 | hpos+yaw | 1 | 1 | 2026-02-04 .. 2026-02-04 | 0 | - |
| 11 | hpos+vpos+yaw | 28 | 25 | 2025-07-13 .. 2026-08-18 | 3 | 2026-06-10 .. 2026-08-17 |
| 13 | hpos+vel+yaw | 1 | 1 | 2026-02-04 .. 2026-02-04 | 0 | - |
| 15 | hpos+vpos+vel+yaw | 71 | 70 | 2024-12-29 .. 2026-02-04 | 1 | 2026-02-15 .. 2026-02-15 |
| (empty) | parameter absent | 7 | 7 | 2022-02-23 .. 2022-02-23 | 0 | - |

Date ranges mix filename dates (local) and companion-clock dates (UTC), so an end point can be off by one day. The 2025-07-13 start of the EV_CTRL = 11 range rests on the 8 filename dates that conflict with the companion clock (see Anomalies).

The 7 logs with no `EKF2_EV_CTRL` are PX4 v1.11.3 (legacy `EKF2_AID_MASK` = [1], `EKF2_HGT_MODE` = [0]).

## EKF2_HGT_REF value -> logs

| EKF2_HGT_REF | meaning | logs | dated logs | date range (date_best) | undated logs | their file-mtime range |
|---|---|---|---|---|---|---|
| 0 | baro | 23 | 23 | 2025-07-02 .. 2025-09-26 | 0 | - |
| 1 | GNSS | 30 | 30 | 2024-12-29 .. 2025-10-27 | 0 | - |
| 2 | range | 30 | 22 | 2025-09-01 .. 2026-08-18 | 8 | 2025-10-03 .. 2026-08-17 |
| 3 | vision | 30 | 29 | 2025-07-13 .. 2026-02-04 | 1 | 2026-02-15 .. 2026-02-15 |
| (empty) | parameter absent | 7 | 7 | 2022-02-23 .. 2022-02-23 | 0 | - |

### EKF2_EV_CTRL x EKF2_HGT_REF x UXRCE_DDS_DOM_ID combinations

| EV_CTRL | HGT_REF | DOM_ID | logs | with in-air > 30 s | date range (date_best) | undated |
|---|---|---|---|---|---|---|
| 0 | 0 | 0 | 1 | 0 | 2025-09-26 .. 2025-09-26 | 0 |
| 0 | 2 | 0 | 9 | 5 | 2025-09-26 .. 2025-09-26 | 5 |
| 0 | 2 | (empty) | 2 | 2 | 2025-09-01 .. 2025-09-01 | 0 |
| 9 | 2 | (empty) | 1 | 0 | 2026-02-04 .. 2026-02-04 | 0 |
| 11 | 2 | 0 | 11 | 5 | 2026-03-13 .. 2026-03-27 | 2 |
| 11 | 2 | 1 | 2 | 2 | 2026-08-12 .. 2026-08-12 | 0 |
| 11 | 2 | 3 | 3 | 2 | 2026-08-12 .. 2026-08-18 | 1 |
| 11 | 3 | 0 | 12 | 0 | 2025-07-13 .. 2025-11-11 | 0 |
| 13 | 2 | (empty) | 1 | 0 | 2026-02-04 .. 2026-02-04 | 0 |
| 15 | 0 | 0 | 22 | 5 | 2025-07-02 .. 2025-09-26 | 0 |
| 15 | 1 | (empty) | 30 | 7 | 2024-12-29 .. 2025-10-27 | 0 |
| 15 | 2 | (empty) | 1 | 0 | 2025-10-24 .. 2025-10-24 | 0 |
| 15 | 3 | 0 | 15 | 1 | 2025-07-13 .. 2025-11-11 | 0 |
| 15 | 3 | (empty) | 3 | 0 | 2026-02-04 .. 2026-02-04 | 1 |
| (empty) | (empty) | (empty) | 7 | 0 | 2022-02-23 .. 2022-02-23 | 0 |

## Distinct sys_uuid values

| sys_uuid | ver_hw | unique logs | PX4 versions | directories (unique logs present) | date range (date_best) | undated |
|---|---|---|---|---|---|---|
| `00020000000020383236563250140027002e` | PX4_FMU_V5 | 57 | v1.14.4 (5), v1.15.2 (52) | `/home/ssg/Desktop` (5)<br>`/home/ssg/Desktop/GhostPX4Logs` (52) | 2024-12-29 .. 2026-02-04 | 1 |
| `000600000000323136383132510c00290023` | CUBEPILOT_CUBEORANGEPLUS | 46 | v1.15.4 (44), v1.16.0 (2) | `/home/ssg/Desktop` (9)<br>`/home/ssg/Desktop/GhostPX4Logs` (1)<br>`/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs` (34)<br>`/home/ssg/lunar_lab/multi-drone-project/code/PX4logs` (2)<br>`/home/ssg/multi-robot-coordination/logs` (1)<br>`/home/ssg/multi-robot-coordination/px4_logs` (2) | 2025-09-01 .. 2026-08-18 | 8 |
| `00020000000032343338303951190029001f` | PX4_FMU_V5 | 7 | v1.11.3 (7) | `/home/ssg/Desktop` (1)<br>`/home/ssg/Desktop/GhostPX4Logs` (7) | 2022-02-23 .. 2022-02-23 | 0 |
| `0006000000003734343332335102004f002b` | CUBEPILOT_CUBEORANGEPLUS | 5 | v1.15.4 (5) | `/home/ssg/Desktop` (4)<br>`/home/ssg/Desktop/PrometheusPX4Logs` (1) | 2025-10-24 .. 2026-08-12 | 0 |
| `000600000000383937303133510f00340037` | CUBEPILOT_CUBEORANGEPLUS | 5 | v1.15.4 (5) | `/home/ssg/Desktop` (5) | 2025-10-27 .. 2026-03-27 | 0 |

`MAV_SYS_ID` is [1] in every log, so it does not distinguish vehicles.

## Logs with in-air time above 30 s (29 logs)

Sorted by `date_best`; undated logs last, ordered by file mtime. "(UTC)" marks companion-clock dates.

| path | date | in-air s | offboard s | hover thrust median (n) | current median A | uuid (last 8) | EV_CTRL | HGT_REF | VIO msgs | dropouts (s lost) |
|---|---|---|---|---|---|---|---|---|---|---|
| `/home/ssg/Desktop/GhostPX4Logs/log_11_2024-12-29-17-58-50.ulg` | 2024-12-29 17:58 | 40.9 | 0.0 | 0.5000 (1) | 18.254 | 0027002e | 15 | 1 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/GhostPX4Logs/log_13_2024-12-29-18-01-32.ulg` | 2024-12-29 18:01 | 49.1 | 0.0 | 0.4963 (41) | 17.785 | 0027002e | 15 | 1 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/GhostPX4Logs/log_15_2024-12-29-18-05-36.ulg` | 2024-12-29 18:05 | 79.1 | 0.0 | 0.7000 (318) | 17.961 | 0027002e | 15 | 1 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/GhostPX4Logs/log_19_2025-3-16-12-34-14.ulg` | 2025-03-16 12:34 | 35.1 | 0.0 |  | 16.056 | 0027002e | 15 | 1 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/GhostPX4Logs/log_20_2025-3-16-12-36-12.ulg` | 2025-03-16 12:36 | 42.5 | 0.0 |  | 15.002 | 0027002e | 15 | 1 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/GhostPX4Logs/log_21_2025-3-16-12-39-24.ulg` | 2025-03-16 12:39 | 46.3 | 0.0 |  | 12.116 | 0027002e | 15 | 1 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/GhostPX4Logs/log_22_2025-3-16-12-43-50.ulg` | 2025-03-16 12:43 | 32.7 | 0.0 |  | 12.511 | 0027002e | 15 | 1 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/GhostPX4Logs/log_45_2025-7-8-17-15-00.ulg` | 2025-07-08 17:15 | 49.6 | 0.0 |  | 12.57 | 0027002e | 15 | 0 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/GhostPX4Logs/log_50_2025-7-13-12-08-58.ulg` | 2025-07-13 12:08 | 32.2 | 0.0 | 0.5000 (1) | 16.701 | 0027002e | 15 | 3 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/GhostPX4Logs/log_46_2025-8-6-00-03-02.ulg` | 2025-08-06 00:03 | 33.3 | 0.0 | 0.5000 (1) | 16.174 | 0027002e | 15 | 0 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/GhostPX4Logs/log_47_2025-8-6-00-04-38.ulg` | 2025-08-06 00:04 | 63.4 | 0.0 |  | 16.349 | 0027002e | 15 | 0 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/GhostPX4Logs/log_48_2025-8-6-00-07-36.ulg` | 2025-08-06 00:07 | 64.6 | 0.0 |  | 17.053 | 0027002e | 15 | 0 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/GhostPX4Logs/log_49_2025-8-6-00-10-22.ulg` | 2025-08-06 00:10 | 69.9 | 0.0 |  | 16.965 | 0027002e | 15 | 0 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_32_2025-9-1-21-35-08.ulg` | 2025-09-01 21:35 | 59.6 | 0.0 | 0.3136 (494) | 6.832 | 00290023 | 0 | 2 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_33_2025-9-1-21-37-08.ulg` | 2025-09-01 21:37 | 38.2 | 0.0 | 0.3175 (331) | 6.904 | 00290023 | 0 | 2 | 0 | 4 (1.7) |
| `/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_75_2025-9-26-11-35-08.ulg` | 2025-09-26 11:35 | 42.6 | 0.0 | 0.3912 (386) | 9.922 | 00290023 | 0 | 2 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_76_2025-9-26-11-37-12.ulg` | 2025-09-26 11:37 | 100.0 | 0.0 | 0.4032 (958) | 10.071 | 00290023 | 0 | 2 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_77_2025-9-26-11-41-26.ulg` | 2025-09-26 11:41 | 73.1 | 0.0 | 0.4227 (639) | 10.537 | 00290023 | 0 | 2 | 0 | 3 (3.2) |
| `/home/ssg/Desktop/log_40_UnknownDate.ulg` | 2026-03-13 21:56 (UTC) | 54.4 | 36.2 | 0.3960 (184) | 9.944 | 00290023 | 11 | 2 | 900 | 1145 (31.6) |
| `/home/ssg/Desktop/log_42_UnknownDate.ulg` | 2026-03-13 23:19 (UTC) | 66.3 | 44.7 | 0.4195 (261) | 10.227 | 00290023 | 11 | 2 | 983 | 1288 (32.7) |
| `/home/ssg/Desktop/log_58_UnknownDate.ulg` | 2026-03-22 04:15 (UTC) | 94.6 | 0.0 | 0.4040 (238) | 10.138 | 00290023 | 11 | 2 | 1506 | 1774 (52.8) |
| `/home/ssg/Desktop/log_2_UnknownDate.ulg` | 2026-08-12 16:12 (UTC) | 181.3 | 32.5 | 0.4218 (1673) | 10.336 | 004f002b | 11 | 2 | 0 | 2 (2.6) |
| `/home/ssg/Desktop/ghostlog_6_UnknownDate.ulg` | 2026-08-12 16:52 (UTC) | 77.1 | 17.7 | 0.3799 (339) | 9.33 | 00290023 | 11 | 2 | 562 | 1367 (43.7) |
| `/home/ssg/Desktop/log_3_UnknownDate.ulg` | 2026-08-12 16:52 (UTC) | 121.2 | 65.4 | 0.3923 (1174) | 9.653 | 004f002b | 11 | 2 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_83_UnknownDate.ulg` | unknown (file mtime 2025-10-03 12:49) | 81.6 | 0.0 | 0.3792 (777) | 9.724 | 00290023 | 0 | 2 | 0 | 0 (0.0) |
| `/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_84_UnknownDate.ulg` | unknown (file mtime 2025-10-03 12:49) | 61.2 | 0.0 | 0.3950 (560) | 10.074 | 00290023 | 0 | 2 | 0 | 1 (2.4) |
| `/home/ssg/multi-robot-coordination/px4_logs/log_70_UnknownDate.ulg` | unknown (file mtime 2026-06-10 16:44) | 184.1 | 160.2 | 0.3937 (814) | 9.665 | 00290023 | 11 | 2 | 2486 | 3275 (97.9) |
| `/home/ssg/lunar_lab/multi-drone-project/code/PX4logs/log_71_UnknownDate.ulg`<br>`/home/ssg/multi-robot-coordination/px4_logs/log_71_UnknownDate.ulg` | unknown (file mtime 2026-06-10 17:09) | 347.0 | 233.9 | 0.4013 (1862) | 9.765 | 00290023 | 11 | 2 | 4810 | 6623 (161.4) |
| `/home/ssg/Desktop/ghostlog_4_UnknownDate.ulg` | unknown (file mtime 2026-08-17 17:34) | 157.8 | 35.6 | 0.4033 (830) | 9.916 | 00290023 | 11 | 2 | 2496 | 2912 (78.0) |

A hover-thrust n of 1 is a single sample (in these logs 0.5 or 0.497), not a usable estimate; an empty cell means `hover_thrust_estimate` has no in-air samples in that log.

## log_70 and log_71

| field | log_70 | log_71 |
|---|---|---|
| paths | `/home/ssg/multi-robot-coordination/px4_logs/log_70_UnknownDate.ulg` | `/home/ssg/lunar_lab/multi-drone-project/code/PX4logs/log_71_UnknownDate.ulg`<br>`/home/ssg/multi-robot-coordination/px4_logs/log_71_UnknownDate.ulg` |
| sha256 (12) | 9e1d8dc2b8f3 | 2a260c100d04 |
| size MB | 54.162 | 110.655 |
| start date | unknown (no GPS topic; timesync remote time is not epoch). File mtime 2026-06-10 16:44:54 | unknown (no GPS topic; timesync remote time is not epoch). File mtime 2026-06-10 17:09:43 |
| sys_uuid | `000600000000323136383132510c00290023` | `000600000000323136383132510c00290023` |
| ver_hw / PX4 | CUBEPILOT_CUBEORANGEPLUS / v1.15.4 (`99c40407ff`) | CUBEPILOT_CUBEORANGEPLUS / v1.15.4 (`99c40407ff`) |
| duration s | 191.04 | 353.28 |
| EKF2_EV_CTRL | 11 | 11 |
| EKF2_HGT_REF | 2 | 2 |
| EKF2_EV_DELAY | 50.0 | 50.0 |
| EKF2_EV_POS_X | 0.15 | 0.15 |
| EKF2_EV_POS_Y | 0.0 | 0.0 |
| EKF2_EV_POS_Z | 0.0 | 0.0 |
| EKF2_RNG_CTRL | 2 | 2 |
| EKF2_GPS_CTRL | 15 | 15 |
| EKF2_BARO_CTRL | 0 | 0 |
| UXRCE_DDS_DOM_ID | 0 | 0 |
| UXRCE_DDS_CFG | 202 | 202 |
| MAV_SYS_ID | 1 | 1 |
| COM_OF_LOSS_T | 1.0 | 1.0 |
| COM_OBL_RC_ACT | 0 | 0 |
| SDLOG_MODE | 0 | 0 |
| changed mid-log | none (0 parameter messages in data section) | none (0 parameter messages in data section) |
| armed s | 191.04 | 352.3 |
| offboard s | 160.2 | 233.9 |
| in-air s | 184.09 | 346.98 |
| hover thrust median (n in-air samples) | 0.3937 (n = 814) | 0.4013 (n = 1862) |
| hover thrust median, `valid` samples only | 0.3938 (n = 805) | 0.4015 (n = 1853) |
| battery current median A (n) | 9.665 (n = 433) | 9.765 (n = 903) |
| battery voltage median V | 14.746 | 14.447 |
| vehicle_visual_odometry msgs | 2486 | 4810 |
| logger dropouts | 3275 dropouts, 97.9 s lost (51% of log) | 6623 dropouts, 161.4 s lost (46% of log) |

## Anomalies and things not determined

- **Logs dated on or after 2026-10-23: 0.** Latest `date_best` is 2026-08-18 15:55:56; latest file mtime of any `.ulg` is 2026-09-25 15:34 (0 files with mtime on/after 2026-10-23).
- **Start date unknown for 9 logs** (no filename date, no epoch time in the log): `log_83_UnknownDate.ulg`, `log_84_UnknownDate.ulg`, `log_85_UnknownDate.ulg`, `log_86_UnknownDate.ulg`, `log_87_UnknownDate.ulg`, `bee_crash1.ulg`, `log_70_UnknownDate.ulg`, `log_71_UnknownDate.ulg`, `ghostlog_4_UnknownDate.ulg`.
- **Filename date conflicts with companion clock in 8 logs:** `log_51_2025-7-13-12-24-24.ulg`, `log_52_2025-7-13-12-27-22.ulg`, `log_53_2025-7-13-12-32-20.ulg`, `log_54_2025-7-13-12-38-50.ulg`, `log_55_2025-7-13-12-46-10.ulg`, `log_56_2025-7-13-12-51-06.ulg`, `log_57_2025-7-13-12-54-02.ulg`, `log_58_2025-7-13-12-58-10.ulg`. Filenames say 2025-07-13 12:24:24 .. 2025-07-13 12:58:10 (local); the companion clock in the same logs says 2025-08-15 03:14:21 .. 2025-08-15 03:48:22 UTC. Which clock was wrong is not determinable from the logs.
- 7 logs carry filename dates in 2022 (2022-02-23): PX4 v1.11.3, uuid `00020000000032343338303951190029001f`, never armed. Whether 2022 is the true date or an unset clock is not determinable.
- **Mid-log changes of the audited parameters: 0 logs.** 2 logs contain parameter messages in the data section at all (`log_23_2025-4-27-22-02-44.ulg`, `log_38_2025-7-5-22-09-34.ulg`); these are `CAL_GYRO*` rewrites at t = 0.
- **Heavy logger dropouts** (more than 20% of the log duration lost) in 16 logs: `test1_bench_VIO_noOF_noRNG_oct-21-2025.ulg` 77%, `log_15_2025-11-11-19-31-26.ulg` 77%, `log_11_2025-11-11-18-56-50.ulg` 74%, `log_14_2025-11-11-19-27-52.ulg` 59%, `log_13_2025-11-11-19-27-06.ulg` 59%, `log_12_2025-11-11-19-21-30.ulg` 58%, `log_56_2026-3-21-23-39-36.ulg` 53%, `ghostlog_6_UnknownDate.ulg` 52%, `log_70_UnknownDate.ulg` 51%, `log_58_UnknownDate.ulg` 50%, `log_40_UnknownDate.ulg` 49%, `ghostlog_4_UnknownDate.ulg` 48%, `log_71_UnknownDate.ulg` 46%, `log_42_UnknownDate.ulg` 45%, `log_61_UnknownDate.ulg` 37%, `log_57_UnknownDate.ulg` 34%.
- **Battery data unusable or absent** in 14 logs:
  - `log_33_2025-7-2-23-39-46.ulg`: battery_status.connected == 0 throughout; median voltage < 1 V over whole log (no power module reading)
  - `log_34_2025-7-4-00-33-14.ulg`: battery_status topic absent
  - `log_35_2025-7-4-19-12-32.ulg`: battery_status topic absent
  - `log_36_2025-7-4-19-15-40.ulg`: battery_status topic absent
  - `log_37_2025-7-5-22-07-40.ulg`: battery_status topic absent
  - `log_38_2025-7-5-22-09-34.ulg`: battery_status topic absent
  - `log_39_2025-7-8-16-11-52.ulg`: battery_status.connected == 0 throughout; median voltage < 1 V over whole log (no power module reading)
  - `log_40_2025-7-8-16-12-42.ulg`: battery_status.connected == 0 throughout; median voltage < 1 V over whole log (no power module reading)
  - `log_41_2025-7-8-16-12-44.ulg`: battery_status.connected == 0 throughout; median voltage < 1 V over whole log (no power module reading)
  - `log_42_2025-7-8-16-12-50.ulg`: battery_status.connected == 0 throughout; median voltage < 1 V over whole log (no power module reading)
  - `log_88_2025-9-26-21-38-54.ulg`: all zero over whole log (no current sensor)
  - `log_96_2025-9-27-00-34-00.ulg`: all zero over whole log (no current sensor)
  - `log_56_2026-3-21-23-39-36.ulg`: battery_status.connected == 0 throughout; median voltage < 1 V over whole log (no power module reading)
  - `log_7_2026-8-18-15-55-56.ulg`: battery_status.connected == 0 throughout; median voltage < 1 V over whole log (no power module reading)
- `log_20_2026-2-4-14-26-34.ulg`: in-air median battery voltage 27.8 V; the highest in any other log is 16.4 V.
- `log_58_UnknownDate.ulg`: 1 NaN `hover_thrust` sample(s) in air, excluded from the median.
- Offboard time is non-zero in only 10 logs: `log_97_UnknownDate.ulg` 3.5 s, `log_40_UnknownDate.ulg` 36.2 s, `log_42_UnknownDate.ulg` 44.7 s, `log_2_UnknownDate.ulg` 32.5 s, `ghostlog_6_UnknownDate.ulg` 17.7 s, `log_3_UnknownDate.ulg` 65.4 s, `bee_crash1.ulg` 14.4 s, `log_70_UnknownDate.ulg` 160.2 s, `log_71_UnknownDate.ulg` 233.9 s, `ghostlog_4_UnknownDate.ulg` 35.6 s.
- `vehicle_visual_odometry` is present in 21 of 120 logs.
- Vehicle names are not derived here; only `sys_uuid` and directory are reported.
