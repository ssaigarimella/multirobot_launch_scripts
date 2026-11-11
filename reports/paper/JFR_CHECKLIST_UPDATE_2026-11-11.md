# JFR paper: remaining-experiments checklist, updated 2026-11-11

Update from the lab agent to the checklist of remaining experiments. It adds what the
2026-10-23 field day and the bench sessions of 2026-10-26 to 10-30 established.

**How to read this.** Draft line numbers refer to `main.tex` at commit `52adf36` in
`/home/ssg/lunar_lab/multi-drone-project/latex-multi-drone-JFR`. Lab data and reports are in
`/home/ssg/multi-robot-coordination` (called "lab repo" below). The draft currently has 145
empty table cells (`\PH`), 13 `\todo` notes and 11 figure placeholders.

**What was and was not audited.** Items marked CHECKED were verified against code, logs or
reports on disk. Items marked NOT AUDITED were not looked at in this pass; their status is
unchanged from the previous checklist. No experiment was run for this update and `main.tex`
was not edited.

Names used below: the four vehicles are delta (1), buckshee (2), ghost (3), thunderstrike (4).
"Field day" is 2026-10-23; its full analysis is `reports/field/FLIGHT-ANALYSIS-2026-10-23.md`
in the lab repo.

---

## 0. Contradictions between the draft and what flew (new section; resolve these first)

Per the checklist rule, these are reported, not reconciled. Each needs a decision from
Sandilya: change the text to match the vehicles, or change the vehicles and re-test.

**0.1 Mapping frame (Section 6.2, lines 989-1011; Figure 4 caption, lines 977-987; the
lesson "Build the map in the estimator frame", lines 1795-1801).** CHECKED.
The draft says dense mapping and planning use the EKF2 post-fusion pose. On the field day
all four vehicles mapped in the cuVSLAM visual-odometry frame, levelled by a fixed 20 degree
rotation for the camera's downward tilt: the recorded mapping transform equals the raw
cuVSLAM odometry rotated by 20 degrees, with residual under 1 mm, on every vehicle. The EKF2
pose path exists only as an untested option on one vehicle (ghost, branch
`ekf2-nvblox-pose`); it has never flown.

**0.2 Vision input to the autopilot is tilted (Section 6.1, lines 942-975).** CHECKED from
flight data, not yet confirmed by the dedicated ground check.
The node that sends cuVSLAM odometry to PX4 applies the 20 degree tilt correction with the
wrong sign, so PX4 receives a pose that is about 40 degrees out of level instead of level.
Evidence: over each flight, PX4's forward displacement is 0.69 to 0.80 of the map-frame
displacement (cos 40 degrees = 0.77), while sideways displacement matches to within 1 %.
Consequences: PX4 under-reads forward distance by about 21 %; any path length taken from PX4
logs for these flights is short. The map itself is level and correct. Section 6.1's statement
that vision is fused as "horizontal position, vertical position, and heading" also needs
checking against the EKF2_EV_CTRL value (item 1.1): PX4 held height correctly despite the
tilted input, which suggests vision height was not what held it.
The planner separately reads the unlevelled odometry for its own position while planning on
the levelled map, so its forward position is 6 % short; thunderstrike flew 2.8 m past the
35 m geofence edge because of it.

**0.3 Emitter splitter (Section 4.3, lines 697-700).** CHECKED on all four
vehicles on the bench, 2026-10-28.
The draft says emitter-off frames feed cuVSLAM and emitter-on frames feed depth. On the
vehicles it is the reverse: cuVSLAM receives the frames that carry the projector dots, and
depth receives the dot-free frames (the splitter's `invert_emitter_logic` is set to true, and
on this camera firmware the emitter flag is not inverted). A one-vehicle A/B test did not
show the setting causing drift. Sandilya has decided to leave the setting as it is, so the
text has to describe what the vehicles do.

**0.4 LoRa schedule convergence (Section 7.2, lines 1127-1149; "coordination messages never
collide", line 1104; Section 10.2, lines 1547-1560).** CHECKED in simulation of the firmware
rules; bench pass on real radios.
The convergence argument assumes every vehicle hears its neighbours. The radios are
half-duplex: two radios whose slots overlap transmit at the same time and never hear each
other. With the firmware that flew (v7.3) and the one on the radios now (v7.4), the lab's
event simulation of the firmware rules (`tests/lora_tdma_sim.py` in the lab repo; ideal
clocks, no capture effect, 500 runs, 4 radios) fails to converge in 39-48 % of cold starts
and 60 % of reboots of the lowest-ID radio. This agrees with Lucas's boot-lock finding
(item 4.2). It happened in the field: on 2026-10-23 at 15:22 one radio was power-cycled, two
radios stayed mutually deaf, and the only two-vehicle flight had zero LoRa packets.
Firmware v7.5 is written but not yet flashed: a radio skips its own transmission in a random
1 cycle in 10 while any peer is unheard, and re-rolls its timing when no peer reports hearing
it. In the same simulation v7.5 converges in every run, including a chain topology (median
2.4 s, 95th percentile 7.5 s). The text and the proof should describe v7.5 once it is flashed
and bench-tested, and should state the half-duplex assumption.

**0.5 LoRa cycle length for other team sizes (Table 11 bound column, lines 1575-1577).**
CHECKED. The firmware's cycle is fixed at compile time for four vehicles (190 ms) however many
radios are powered. The bounds of 110 ms for n = 2 and 350 ms for n = 8 assume firmware
recompiled for that team size; no such build has been run.

**0.6 Transmit power limitation (Section 13, lines 1853-1855).** CHECKED. Out of date: a
runtime transmit-power command has been in the firmware since v7.3 and on all four radios
since 2026-10-26. No radio has yet confirmed a power change in a test (item 1.5).

**0.7 Per-packet fields (Section 9.3, line 1431: "carries a sequence number, transmit power,
and timestamp").** CHECKED. The frame the radio hands to the onboard computer is 23 bytes:
vehicle ID, position, heading, RSSI, SNR. It has no sequence number, transmit power or
timestamp. Delivery ratio can be computed from the fixed 190 ms schedule; latency cannot be
measured without a firmware change.

**0.8 "When the high-bandwidth link is unavailable, vehicles continue to exchange poses and
frontier claims over the LoRa link" (Section 7.3, lines 1175-1181).** CHECKED. No flight
evidence yet. In the only two-vehicle flight the opposite happened: Wi-Fi map sharing worked
and LoRa was dead (0.4).

---

## 1. Blockers and draft corrections

1.1 **EKF2_EV_CTRL.** NOT AUDITED for log values. New: the field-day ULogs were never copied
off the four vehicles; they are needed here and for 2.3 and 3.4. See 0.2 for the related
contradiction in Section 6.1.
1.2 **Frame size.** CHECKED in the draft only: line 544 says 427 mm, Table 4 (line 764) says
443 mm. No motor-to-motor measurement found in this pass.
1.3 **ESC model, rating, firmware.** NOT AUDITED.
1.4 **Fuse, converter current and voltage.** NOT AUDITED.
1.5 **LoRa firmware.** CHECKED. Runtime transmit power: in the firmware and on all four
radios; one-minute radio check still to do; then the link-margin sweep (Figure 8) is
unblocked. Sequence number and uptime in the frame: NOT done (0.7), so LoRa latency in
Table 15 stays blocked. New dependency: flash and bench-test v7.5 first (0.4), so all LoRa
data for the paper comes from one firmware.
1.6 **LoRa module Wi-Fi/Bluetooth disabled.** CHECKED: not done. The firmware source has no
Wi-Fi or Bluetooth calls, neither enabling nor disabling.
1.7 **micro XRCE-DDS citation.** NOT AUDITED; still open.

## 2. Bench

2.1 **Table 2 mass.** NOT AUDITED.
2.2 **Table 5 cost; Table 4 takeoff mass and endurance.** NOT AUDITED.
2.3 **Table 3 electrical budget.** NO NEW DATA. Hover current and hover thrust need the
field-day ULogs (1.1).
2.4 **Table 7 / Figure 7 thermal.** NO DATA for the matrix. Related data exists from
2026-10-28: four vehicles, full stack, 10-15 min runs with per-second pipeline rates and
tegrastats (`experiments/results/colour_check_bench1028*/` in the lab repo), power mode not
varied.
2.5 **Table 8 process separation.** NO DATA for the two arrangements. Related finding from
the same runs: recording the large ESDF point clouds raised CPU from 55 % to 70 %, opened
0.35 s gaps in the IMU stream, and two of four vehicles lost their pose. This is a second,
measured example of load corrupting the visual front end's inertial input (relevant to
Section 5.1 and the first lesson).
2.6 **Figure 8 link margin.** NOT RUN. Unblocked in firmware (1.5). Must be done at distance:
on the bench, the lowest power still arrives at about -56 dBm.
2.7 **Table 9 coexistence.** PARTIAL. Existing: two-vehicle bench run in
`reports/rf_results/RF_RESULTS.md` (source of the 190.00 ms / 1.22 ms figures at line 1121).
New: 2026-10-28, 54 min, four radios with Wi-Fi traffic running: LoRa delivery 99.7-100 % on
all 12 links. ELRS was off, so this covers one of the seven rows at most.
2.8 **Field range.** NO DATA.

## 3. Flight

3.1 **Table 13 exploration campaign.** PARTIAL, shakedown only. Field day, one site (open
field, 35 m box): four flights, three single-vehicle and one 91 s two-vehicle overlap. No
four-vehicle trial and no repeated trials. Bags are on the laptop (`bags/e4`, `e4b`, `e4c`,
26 GB). These flights ran with faults fixed since (planner control loop at 2-5 Hz instead of
20 Hz; a geofence-edge replanning loop), so they are not comparable with later trials.

| Vehicle | Autonomous time | Path (map frame) | Outcome |
|---|---|---|---|
| ghost | 6+ min | 41 m (PX4 frame) | aborted: autopilot-to-computer serial link froze after 2 min |
| thunderstrike | 6.4 min | 139 m | clean |
| buckshee | 2.0 min | 54 m | clean, short |
| delta | 1.6 min | 39 m | clean, short |

3.2 **Table 14 connectivity degradation.** NO DATA.
3.3 **Table 15 in-flight communication.** PARTIAL. LoRa poses arrived at 5.3 packets/s per
peer in the two solo flights; zero LoRa packets in the two-vehicle flight (0.4). Wi-Fi map
sharing worked both ways in that flight (89 and 84 keyframes in 2 min). No latency data (0.7).
3.4 **Table 16 campaign summary.** PARTIAL. Field day: 4 flights, about 16 min autonomous,
1 aborted. Abort and time-loss causes observed: serial-link freeze (clone USB-serial adapter
stalls; software reset recovers it), geofence-edge loop, LoRa mutual deafness. An index of
armed segments from older ULogs exists at `reports/rc_loss/armed_segments.csv`.
3.5 **Mesh in flight.** CHECKED: none. `reports/wifi_mesh/MESH_CHARACTERIZATION.md` states the
802.11s mesh has never been used in a real run, and the draft's own limitation (lines
1850-1852) says the campaign has used an access point. Section 7.3 (lines 1153-1171) and the
third lesson describe the mesh as the operated link.
3.6 **Figure 13 fused four-vehicle map.** NOT POSSIBLE from existing data: no four-vehicle
flight, the mesh topic was never recorded, and the shared colour layer has no messages.
3.7 **Figures 10-12.** PARTIAL. Trajectory, coverage and communication figures for the field
day exist in `reports/visualization/e4/` and `e4c/` (lab repo). No site photographs or
dimensions found in this pass.

## 4. Simulation (Lucas)

4.1 **Table 10 / Figure 9.** NOT AUDITED. Note 0.1 and 0.2: the ablation compares mapping
frames, and the flown vehicles used the frame the draft argues against, with a tilted vision
input to the autopilot.
4.2 **Table 11.** See 0.4 and 0.5. The lab simulation reproduces boot-lock non-convergence
for the flown firmware and shows v7.5 removing it. Regenerate Table 11 against v7.5 rules.
Timing jitter is already measured (1.22 ms SD, line 1121).

## 5. Non-experimental

5.1-5.4 NOT AUDITED, except: `ohradzansky2022multiagent` is in `refs.bib` (line 635);
`zhang2024gvp` is cited once in `main.tex` but has no entry in `refs.bib`. Repository URL (5.4): the flown code is committed locally
but unpushed in four repositories, so there is nothing public to cite yet.

---

## 6. New items from the live runs

6.1 **Changes to make before the campaign,** each as its own controlled change with a test,
each needing Sandilya's approval:
   a. Correct the tilt sign on the vision input to PX4 (0.2), after the ground rotation check.
   b. Make the planner use the levelled pose (0.2).
   c. Flash LoRa v7.5 on all four radios together and run the bench reboot test (0.4).
   d. Deploy the serial-adapter watchdog, or replace the clone adapters (3.4).
6.2 **Recording rule for every experiment.** Do not record the ESDF point clouds: the load
makes the visual odometry drift (2.5). Add the map-mesh topic to the recording only after a
bandwidth check; Figure 13 needs it.
6.3 **Limitations to state in the draft.** Colour mapping stops while the visual odometry's
height estimate has drifted (seen on three of four field flights; depth mapping continued).
PX4-frame distances from the field day are 21 % short along the forward axis.
6.4 **Data to secure.** Copy the field-day ULogs off all four vehicles. Push the four
repositories.

## 7. Experiments still to run, ordered by what they unblock

1. Decisions on 0.1-0.4, then the changes in 6.1. Everything flown afterwards depends on them.
2. LoRa: one-minute power check, v7.5 flash and bench test, link-margin sweep at distance
   (Figure 8, Table 11 calibration, Table 15 delivery).
3. Flight campaign: trials per Table 13 (1, 2 and 4 vehicles), including one four-vehicle
   flight recorded with the mesh topic (Figure 13), and the degradation conditions of Table 14.
4. Bench matrices: thermal (Table 7, Figure 7), process separation (Table 8), coexistence
   (Table 9, six remaining rows), electrical budget (Table 3).
5. Mesh in flight (3.5), or revise Section 7.3 to describe the access point as flown.
6. Weighing, BOM, ESC and fuse details (1.2-1.4, 2.1, 2.2).
