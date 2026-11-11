# JFR paper audit, round 2 (2026-11-11)

From the lab agent. Draft: `main.tex` in
`/home/ssg/lunar_lab/multi-drone-project/latex-multi-drone-JFR`, branch
`jfr-field-corrections` (10 commits on top of `main` @ `52adf36`, not pushed). Line numbers
below are at commit `332e010`, the head of that branch when this report was written; round 3
(`ROUND3_REPORT.md`) added 6 commits after it. Lab data: `/home/ssg/multi-robot-coordination`
("lab repo"). No experiment was run and nothing on any vehicle was touched; the drones were
not connected.

**Output files** (all in `reports/paper/audit_round2/` of the lab repo):
`ulog_index.csv`, `ulog_summary.md`, `hardware_and_section5_findings.md`,
`table03_electrical_from_ulogs.csv`, `table09_coexistence_candidates.csv`,
`table11_tdma_lab_simulator.csv`, `table15_inflight_comms.csv`,
`table16_campaign_shakedown.csv`, `bench_2026-10-28_load_and_thermal.csv` (Tables 7 and 8),
`vision_tilt_forward_ratio_2026-10-23.csv`, `prompts_for_go.md`.

## Read this first: where I did not follow the edit list, and why

1. **Item g (Table 9 row "Coordination and mesh") was NOT made.** On 2026-10-28 the WiFi
   adapters were associated to the bench access point at 5765 MHz, not in the 802.11s mesh at
   2437 MHz (the bring-up's default transport is the access point; the mesh needs `--mesh`,
   which was not used). The row is the wrong condition for that data, so the cell stays
   `[TBD]`. The delivery figures are also time-means of a cumulative ratio, not final ratios;
   the raw files are still on the vehicles. See C.1 for the numbers and two options.
2. **Item c describes v7.5 as it is written, which differs from the edit list.** v7.5 has no
   listen-before-transmit at boot and no expiry of a departed reference's identifier. What it
   has: random listen-only slots (1 in 10 while any peer is unheard), a timing shift when no
   peer reports hearing the radio for 1.5 s, and relayed reference timing. I checked the
   leader-loss case in the lab simulator (new scenario, 500 runs, full and chain topology):
   after the reference radio departs, the survivors stay on a common frame in 500 of 500 runs
   and all keep the departed radio's identifier. There is no re-election. The text says that.
   v7.5 is not yet on the radios, so Section 7.2 now describes firmware that has not flown.
3. **Item d wording.** "The coordination frame carries no sequence number" is not quite true:
   the over-the-air packet has a sequence number, and claim packets have their own. What lacks
   them is the frame the radio passes to the onboard computer. The bullet says that.
4. **Item e.** Removing the three fields left "so delivery ratio and latency are computed"
   without support, so that sentence now claims delivery ratio only. Latency is still promised
   at lines 1448 and 1479 (Figure 8 text); not edited.
5. **Items h and i contradict each other as written** (h puts shakedown data in Table 15; i
   says shakedown flights are excluded from the tables). The sentence for i now names the
   tables they are excluded from (13, 14, 16), and the Table 15 caption says its two figures
   come from the shakedown flights and that the WiFi link ran through an access point.
6. **Item j numbers.** The bench data gives CPU 53 % without recording and 69 % during the
   recording window, averaged over the four vehicles (per vehicle 52.6-54.1 % and
   66.7-71.1 %). The lesson uses 53 % and 69 %, and "up to 0.35 s" for the inertial gaps.
7. **Item f parameter values are the planned configuration, not what any flight log shows.**
   Every Offboard flight log on disk has EKF2_EV_CTRL 11 with EKF2_HGT_REF 2. One bench log
   (2026-02-04) has 9. See A.7.
8. **One edit outside the named list:** Section 10.1, "as the platform does" became "the
   configuration planned for the flight campaign" (line 1528), since item a makes the old
   clause false.
9. **`checks.py` does not exist on this laptop** (not in the repository or its history). I
   checked every added line for the listed conventions with my own script instead: no em or
   en dashes, no "substrate", `\cite` only, no "design decision", no First/Second/Third
   scaffolds, American spelling, ASCII only. 0 findings on 138 added lines.
10. **Compile:** pdflatex, bibtex, pdflatex, pdflatex all exit 0; 32 pages. One warning,
    present before these edits: citation `zhang2024gvp` (line 242) has no entry in `refs.bib`.

---

## A. Draft contradictions

Status key: EDITED (changed on the branch; final text in section F), OPEN (still in the
draft, needs a decision).

**A.1 Mapping frame.** Field evidence: on all four vehicles the recorded mapping transform
(`odom` to `camera0_link`) equals the raw cuVSLAM odometry rotated by 20 degrees, residual
under 1 mm (lab repo `reports/field/FLIGHT-ANALYSIS-2026-10-23.md` section 4.6; bags
`bags/e4`, `bags/e4c`). Code: `active_exploration/src/odom_correction_node.cpp` lines 7-20,
`kCameraPitchDeg = 20.0`, `R_y(+pitch)`, commit `37c7d32` on
`feature/jfr-multi-drone-field`. The EKF2 pose path exists only on ghost, branch
`ekf2-nvblox-pose`, never flown.
- EDITED: abstract (lines 95-97), Contribution 3 (192-199), Figure 4 text and caption
  (982-986), Section 6.2 (990-1016), Section 10.1 clause (1528), conclusion (1944-1948),
  lessons item (1828-1835).
- OPEN, still say the fused pose drives mapping:
  - line 846, Figure 3 caption: "`odom_correction` republishes the fused pose as the frame
    mapping uses." Not edited because Figure 3 itself (`figures/Figure_3.pdf`) probably draws
    the same thing and needs redrawing.
  - line 1022, Section 6.3: nvblox uses "the post-fusion pose to maintain a truncated
    signed-distance" field.
  - line 1555, Table 10 row label "Map from post-fusion pose (flown)". Table cell, not touched.
  - line 1562, Figure 9 placeholder text, consistent with the simulation study; fine.

**A.2 Vision input to PX4 is about 40 degrees off level.**
- Deployed code: `px4_offboard` branch `dev` @ `87c25c5` (on all four vehicles).
  `src/vio_bridge_node.cpp` lines 8-18: `R_cam_to_body = R_y(-pitch)` with 20 degrees.
  `odom_correction` applies `R_y(+pitch)`. The two disagree by 40 degrees.
- Lucas's fix `7ac8ad7` ("vio_bridge: fix anti-leveling sign ... now R_y(+20)") is on
  `origin/sim` only. It is on no field branch.
- Forward ratio, PX4 displacement over map-frame displacement, per Oct 23 flight
  (`vision_tilt_forward_ratio_2026-10-23.csv`): thunderstrike 0.794, buckshee 0.798, delta
  0.743, ghost 0.689; sideways 0.992-1.001; cos 40 degrees = 0.766.
- Status: inferred from flight data; the ground test has not been run (prompt A in
  `prompts_for_go.md`).
- EDITED: new lessons item (lines 1878-1884). Section 6.2 now says the post-fusion frame
  waits on this validation (1013-1016).
- OPEN: line 976, "the value that gave stable estimation across the flight campaign"
  (latency compensation). No campaign has flown yet. Path lengths for the Oct 23 flights must
  come from the camera track, not PX4 (PX4 is 21 % short along the forward axis).
- The planner reads unleveled odometry for its own position while planning on the leveled
  map (6 % short forward; thunderstrike passed the 35 m fence by 2.8 m). Not in the draft.

**A.3 Splitter.** Evidence: bench 2026-10-28, all four vehicles, camera firmware 5.13.0.55:
frames flagged emitter-on carry the dot pattern (sharpness score 10-16 against 4-6), and the
launch file sets `invert_emitter_logic: True`, so cuVSLAM gets the emitter-on frames
(FLIGHT-ANALYSIS section 4.7). Sandilya's decision: leave the setting.
- EDITED: Section 4.3 (lines 698-701), Figure 3 caption (840-843).

**A.4 LoRa convergence.** Evidence: `table11_tdma_lab_simulator.csv` (lab repo
`tests/lora_tdma_sim.py`; 500 runs each; ideal clocks, no capture effect). Firmware on the
radios (v7.4): not converged in 39.2 % of cold starts and 60.2 % of reference-radio reboots
(4 radios, all in range). v7.5: 0 % in every scenario. Field: 2026-10-23 15:22, two radios
mutually deaf after a power cycle, zero LoRa packets in the two-vehicle flight
(FLIGHT-ANALYSIS section 3).
- EDITED: "never collide" sentence (1110-1112), schedule paragraph and new mechanisms
  paragraph (1133-1151), convergence assumption (1153-1166), Section 10.2
  (1578-1586), new lessons item (1851-1859).
- OPEN: Contribution 2, line 187-188, "We prove that the clock synchronization converges":
  still true under the stated hearing assumption, but the proof now leans on a randomized
  mechanism. Table 11 bound column (lines 1603, 1605): 110 ms and 350 ms assume firmware
  recompiled for 2 and 8 vehicles; the deployed cycle is fixed at 190 ms.

**A.5 Compile-time transmit power (Section 13).** EDITED, lines 1915-1917. Runtime power has
been in the firmware source since v7.3 (2026-09-25) and on the radios since v7.4
(2026-10-26). No radio has confirmed a power change yet.

**A.6 Packet fields (Section 9.3).** EDITED, lines 1452-1454. Firmware
`radiohive/firmware/lora_mesh_v7_2_ros2.ino`: frame 0x81 is 23 bytes: node id, lat, lon, alt,
heading, RSSI, SNR.

**A.7 EKF2_EV_CTRL.** `ulog_index.csv`: 120 unique logs.

| EKF2_EV_CTRL | Logs | Dated range |
|---|---|---|
| 0 | 12 | 2025-09-01 to 2025-09-26 |
| 9 | 1 | 2026-02-04 (bench, not a flight) |
| 11 | 28 | 2025-07-13 to 2026-08-18 |
| 13 | 1 | 2026-02-04 |
| 15 | 71 | 2024-12-29 to 2026-02-04 |
| absent | 7 | 2022 (old firmware) |

All 8 logs with Offboard time have EV_CTRL 11 and HGT_REF 2, including log_70 and log_71. No
log shows the fleet on 9. No log on disk is from 2026-10-23 or later (latest dated log
2026-08-18). EDITED per item f: Section 6.1 (lines 953-958, 961-962) and Figure 4 text
(980-982) now say vision gives horizontal position and heading only, with EV_CTRL 9 and
HGT_REF 2. That matches the plan, not yet any flight log.

**A.8 Other contradictions found.**
- The 802.11s mesh is described as the operated link in 14 places (abstract 100, 153, 185,
  291, 377, 773, 851, 1079, 1177-1191, 1839, 1942, and others), while lines 1912-1914 say the
  campaign has used an access point. No flight or bench session since has used the mesh.
- Line 189, "report both links in flight", and lines 1196-1200 (vehicles keep exchanging
  claims over LoRa when the WiFi link is lost): no flight evidence yet.
- Lines 206, 1615, 1949, "three flight experiments": none of the three environments has a
  campaign flight yet.
- Table 15 caption, "four-vehicle configuration": the filled value comes from flights with
  one or two vehicles airborne.
- Place recognition input: lines 693-694 and 1055-1056 say it runs on the color stream. The
  node subscribes to the infrared pair from the splitter
  (`multi_drone_nvblox/scripts/loop_closure_sp_node.py` lines 359-366).
- The `\todo` at lines 1061-1064 says no pairwise-consistency threshold exists. It does, and
  it is enabled; see B, item 5.2.
- log_70 and log_71 have `UXRCE_DDS_DOM_ID` 0, while the fleet convention is domain equal to
  vehicle id. Not in the draft; noted for Section 5.3 if the draft states the convention.

---

## B. Items that were NOT AUDITED in round 1

**2.3 Table 3, from ULogs** (`table03_electrical_from_ulogs.csv`).

| Log | In air (s) | Hover thrust, in-air median (n) | Current (A) | Voltage (V) | Power (W) |
|---|---|---|---|---|---|
| log_70 | 184.1 | 0.3937 (814) | 9.67 | 14.75 | 142.6 |
| log_71 | 347.0 | 0.4013 (1862) | 9.77 | 14.45 | 141.2 |

Caveats: both logs lost about half their samples to logger dropouts (51 % and 46 % of the
duration); their dates are unknown (files dated 2026-06-10); the current is the whole-vehicle
battery current, not propulsion alone. No Oct 23 ULog is on disk. 29 logs have more than 30 s
in the air (2024-12-29 to 2026-08-12); see `ulog_summary.md`.

**1.1 EV_CTRL:** see A.7.

Full detail with paths: `hardware_and_section5_findings.md`. The place-recognition and mass
table findings were re-checked by hand; the rest is as the search reported it.

**1.2 Frame diagonal: NO DATA.** No measurement, order record or product page on the laptop.
The draft has three values: `[TBD]` mm (line 521), 427 mm (line 546), 443 mm in Table 4
(line 765). The 443 arrived in an Overleaf edit on 2026-11-06 with no supporting file.
Propellers: the draft says 7x4.5 in (lines 541, 766); no record of what is fitted; CAD file
names and two archived drafts say 9x4.5 in or 5 in. Reported, not reconciled.

**1.3 ESC: PARTIAL, CONTRADICTION.** The draft says "LANNRC 45A" (lines 542, 766), added in
the same 2026-11-06 edit; that name appears nowhere else on the laptop. Other records say
"Cyclone 35A": the Figure 1 component list in the 2026-10-19 draft PDF, the CAD part
`Downloads/Drone Assembly Parts/CYCLONE 35A.step`, and a 2025 wiring photo. Firmware: no
record. PX4 parameter exports show delta on DShot600 and ghost on DShot300; no exports for
buckshee or thunderstrike.

**1.4 Fuse, converter current and voltage: NO DATA.** Related only: computer input power on
the 2026-10-28 bench peaks at 12.8-15.3 W per run (tegrastats; power mode not logged; not
converter input current).

**2.1 Table 2 mass: PARTIAL.** Five rows have blank addends, eight slots in all:
line 614 `227 +  +  + 32`; line 615 `28 + + 35 + `; line 617 `34 +  + `; line 618
`7 + 11 + 6 + `; line 619 `467 +  + `. Line 616 has a stray "g" (`5g`). No weighing record
exists on the laptop. The addends and the 1867 g total were typed into Overleaf on
2026-09-18; the filled addends sum to 1060.5 g.

**2.2 Cost, takeoff mass, endurance: NO DATA.** No BOM, price list, order or invoice for the
vehicle. All Table 5 cells and Table 4's takeoff mass and endurance are `[TBD]`.

**5.1 Figures.** 13 figure environments: 3 real with files present (`fig:exploded`,
`fig:arch`, `fig:vibration`), 10 placeholders (`fig:power`, `fig:axes`, `fig:links`,
`fig:thermal`, `fig:linkmargin`, `fig:faults`, `fig:indooroutdoor`, `fig:course`,
`fig:field`, `fig:fusedmap`). Candidate sources exist only for `fig:links` (a lab slide and
`reports/rf_results/figures/fig4_lora_tdma.png`), `fig:field` and `fig:fusedmap`
(`reports/visualization/e4*`, at most two vehicles), and `fig:thermal` (bench tegrastats,
not a full battery). Nothing for the power schematic, axes, faults or course figures.

**5.2 Place recognition: DATA EXISTS, not in the draft.**
- Vocabulary: all four vehicles log "Vocabulary loaded: 4089 words" from a file named
  `sp_vocab_4096.pkl` (lab repo `collected/<vehicle>/pane_Tab_6*`, 2026-06-11 and
  2026-08-03). Nominal size 4096; 4089 loaded.
- Pairwise consistency: implemented (`multi_drone_nvblox/scripts/pcm.py`), enabled in the
  launch (`use_pcm:=True`), thresholds 1.0 m translation and 15 degrees rotation
  (`loop_closure_sp_node.py` lines 190-191). The draft's claim at lines 1051-1055 is
  supported; its `\todo` at 1061-1064 is out of date.

**5.3 Bibliography.** `zhang2024gvp`: cited at line 242, no entry (a possible match,
`gvp2024`, is in `refs.bib` line 61, uncited). `ohradzansky2022multiagent`: not cited;
`refs.bib` line 635 is a placeholder. `semtech2019sx1262`: cited at lines 429 and 1126; the
entry (line 625) has a `% VERIFY` comment. `cao2023representation`: neither cited nor
defined. 21 entries are uncited. Entries were not checked against the published record.

**5.4 End matter: all placeholders.** Middle authors (line 63), repository and video URLs
(line 106), author contributions (line 1955), acknowledgments (line 1961).

---

## C. Items with new data

**C.1 Table 9 coexistence** (`table09_coexistence_candidates.csv`). No row can be filled as
labeled.
- 2026-10-28, four vehicles, 54 min: LoRa claim-packet delivery 0.997 to 1.000 on all 12
  links (3235-3244 one-second samples per link; time-means of the cumulative ratio from
  sequence gaps). WiFi on the access point at 5765 MHz with the topic bridge active. Control
  link off. This is "coordination with access-point WiFi", which is not a row in Table 9.
- 2026-08-25, two vehicles, 22 min: LoRa with the 802.11s mesh up at 2437 MHz: zero missed
  cycles in 7068 intervals at one receiver. This is the right condition for "Coordination and
  mesh", but the mesh was nearly idle (about 1 KiB/s), no iperf3 throughput was taken, and
  the lab's own report calls it "not a coexistence result".
- Options: add a row for the access-point condition, or leave the row `[TBD]` until a mesh
  run with iperf3 load. All seven rows still need: iperf3 load on the mesh, ELRS on with
  RSSI/LQ from the ULog, and the "only" conditions.

**C.2 Tables 7 and 8** (`bench_2026-10-28_load_and_thermal.csv`, 13 vehicle-runs).
Full stack, indoor bench, 8-14 min each, power mode not varied (one mode only; not recorded
in the logs), no run to battery exhaustion.

| Condition | CPU mean | Junction temp, mean / max | Input power | Odometry / depth rate |
|---|---|---|---|---|
| No recording (4 vehicles) | 52.6-54.1 % | 59.4-62.0 / 60.2-62.4 C | 10.8-11.7 W | 30 / 54-58 Hz |
| Recording incl. ESDF clouds (4) | 61.3-64.2 % run mean, 66.7-71.1 % in the 6 min window | 61.3-63.5 / 63.7-65.1 C | 12.2-13.0 W | 30 / 55-56 Hz |
| Recording, current profile (4) | 52.9-60.4 % | 60.5-62.3 / 62.1-62.8 C | 11.2-11.9 W | 30 / 54-57 Hz |

- Can fill: nothing in Table 7 as designed (needs five configurations by three power modes,
  time to first throttle, and battery-exhaustion runs). The baseline row is one data point
  for "single stack, one power mode": no throttling, junction temperature about 62 C.
- Cannot fill Table 8: both container arrangements were not compared; no inertial callback
  interval percentiles were logged.
- Finding (now a lesson, lines 1869-1876): recording the ESDF clouds raised CPU from 53 % to
  69 % and opened inertial gaps of up to 0.35 s; two of four vehicles lost their pose
  (colour layer 46-51 % empty on ghost and buckshee during that run).

**C.3 Tables 13 and 16** (`table16_campaign_shakedown.csv`). Four shakedown flights on
2026-10-23, runs e4 and e4c (bags in the lab repo `bags/`, 26 GB): 16.1+ min in Offboard,
3 completed, 1 aborted (ghost, serial link froze). Marked SHAKEDOWN; nothing entered in
Tables 13, 14, 16 or Figure 13. A sentence before Table 16 says so (lines 1785-1788).

**C.4 Table 15** (`table15_inflight_comms.csv`). Filled: coordination achieved rate 5.3 Hz
(line 1777). Caption carries the keyframe counts. Still `[TBD]`: delivery, latency, all mesh
cells.

**C.5 Figures 10-12 candidates** (lab repo `reports/visualization/`): for each of `e4/` and
`e4c/`: `team_map`, `paths_by_state`, `coverage`, `comms` (PDF and PNG), `events.csv`, a 3D
replay page and MP4; `e4/ghost_feed_death_142225` (the serial-link freeze). No site
photographs or dimensions found.

**C.6 Section 10.2 / Table 11** (`table11_tdma_lab_simulator.csv`). Jitter: median 190.00 ms,
1.22 ms, 7068 intervals (already at line 1121). Simulator: 500 runs per cell; v7.4 not
converged 39.2 % (cold start) and 60.2 % (reference reboot); v7.5 0 %; chain split 96.4 %
against 0 %, median 2.4 s, 95th percentile 7.5 s. Table 11 cells untouched.

---

## D. Blockers before the field campaign (for Sandilya)

| Blocker | Test that closes it |
|---|---|
| Pull the Oct 23 ULogs | `python3 tools/pull_ulogs.py --last 8` with the flight controllers on USB (new tool, untested on a vehicle; fallback QGroundControl log download). Then re-run the ULog index. |
| vio_bridge / ekf2_odom_tf sign pair | Prompt A in `prompts_for_go.md`: rotation check plus a 1.00 m bench slide. Pass: forward 0.95-1.05 m, vertical under 0.05 m. Fail: about 0.77 m and 0.64 m. |
| Planner uses the unleveled pose | After the sign fix: replay thunderstrike's Oct 23 flight through the offline planner harness and confirm the fence test uses the leveled position; then one flight to the fence edge. |
| LoRa v7.5 | Prompt B: flash all four, 15 of 15 reboot trials FULL for the reference and one other radio, 10 cold starts, then the transmit-power confirm (radio reports 14 then 10 dBm). |
| Frame 0x81 has no sequence or uptime | Firmware and bridge change, then a bench check that the fields arrive. Blocks Table 15 latency and the Figure 8 latency panel. Not started. |
| Serial-adapter stall (ghost abort) | Deploy `tools/ftdi_watchdog.sh` and trigger its self-test on each vehicle, or fit genuine adapters and confirm no "urb stopped" in the kernel log over a 30 min stack run. |
| Recording profile | Never record the ESDF clouds. Add the map-mesh topic only after a bench bandwidth check with pose held within 10 cm for 10 min. Figure 13 depends on it. |
| Mesh in flight, or revise the text | One flight with `--mesh`, or change the 14 places listed in A.8. |
| Repository URL | Push the four code repositories (unpushed: active_exploration 2 commits, multi_drone_nvblox 2, radiohive 1 plus v7.5 uncommitted, lab repo 8 plus uncommitted work). |

---

## E. Experiments still to run, by what they fill

| Experiment | Fills | Blocked on |
|---|---|---|
| Slide test, then sign-pair fix and planner pose fix | Section 6.1/6.2 wording; every later flight | Sandilya's go |
| LoRa v7.5 flash and bench tests | Section 7.2, 10.2; all LoRa data | Sandilya's go |
| Link-margin sweep at distance | Figure 8 (delivery), Table 11 channel model, Table 15 delivery | v7.5; power confirm |
| Frame sequence and uptime | Table 15 latency, Figure 8 latency | firmware work |
| Flight campaign, 3 sites x {1, 2, 4} vehicles x N | Tables 13, 15, 16; Figures 10-13 | all of the above; mesh topic in the recording |
| Degradation flights, 4 vehicles, 3 conditions | Table 14 | campaign |
| Thermal matrix, 5 configurations x 3 power modes, to battery exhaustion | Table 7, Figure 7 | none (bench) |
| Process separation, two container arrangements | Table 8 | none (bench) |
| Coexistence, 7 radio sets with iperf3 and ELRS | Table 9 | mesh bring-up |
| Electrical budget | Table 3 | Oct 23 ULogs; a clean hover log |
| Field range walk-out | Section 9.3 | v7.5 |
| Weighing, BOM, ESC, fuse | Tables 2, 4, 5; Sections 4.1, 4.2 | see B |

**Table status.**

| Table | Status |
|---|---|
| 2 mass | see B |
| 3 electrical | PARTIAL: hover thrust and whole-vehicle current from log_70/71; computer power 10.8-13.0 W from tegrastats; nothing else |
| 4 platform, 5 cost | see B |
| 6 vibration | FILLED |
| 7 thermal | NO DATA for the matrix; one related data point (C.2) |
| 8 process separation | NO DATA |
| 9 coexistence | NO DATA as labeled; two candidate data sets (C.1) |
| 10 mapping-frame ablation | NOT AUDITED (Lucas's shared folder is not on this laptop) |
| 11 schedule simulation | DATA EXISTS in the lab simulator for 4 radios; cells left `[TBD]` |
| 13 exploration | NO DATA (shakedown only) |
| 14 degradation | NO DATA |
| 15 in-flight comms | PARTIAL: one cell filled |
| 16 campaign | NO DATA (shakedown excluded) |

---

## F. Edits made to `main.tex`: final text

Branch `jfr-field-corrections`, one commit per item (item j has a second commit correcting
its numbers; item g has none). Each block is the text as it now stands.

### `1f3acdd` Mapping frame: the fleet maps from leveled cuVSLAM odometry

Line 96:

```latex
the visual odometry leveled for the camera tilt, with mapping from the
post-fusion pose evaluated in simulation. Vehicle-to-vehicle communication uses two links built
```

Line 194:

```latex
mapping and planning from the cuVSLAM odometry leveled for the camera tilt, and
isolates the visual front end from the dense mapper across process boundaries.
Each choice is stated with its rationale; mapping from the post-fusion pose, the
configuration planned for the flight campaign, is evaluated by fault injection
in simulation, and the process-isolation choice by direct measurement on the
```

Line 983:

```latex
propagation. The EKF2 post-fusion pose combines
all three with the autopilot inertial measurements. Dense mapping and planning
use the cuVSLAM odometry leveled for the camera tilt.}
\caption{Estimator sensor-to-axis assignment. Mapping uses the leveled cuVSLAM
odometry.}
```

Line 991:

```latex
\subsection{Mapping and planning frame}\label{sec:mapframe}
```

Line 994:

```latex
propagates through the rest of the system. Two poses are available on the
vehicle: the cuVSLAM visual odometry and the EKF2 post-fusion pose, which
```

Line 997:

```latex
rangefinder. The fleet maps from the cuVSLAM odometry, leveled for the camera
tilt. Rather than let cuVSLAM publish the mapping transform, the
```

Line 1000:

```latex
rotates the odometry by the fixed $20^{\circ}$ camera tilt and republishes the
transform that mapping uses, which is where the pose driving the map is
selected.
```

Line 1011:

```latex
Driving the map from the visual odometry lets its drift, worst in the vertical
axis, accumulate into the map and from there into planning. The gain in mapping
accuracy is measured in simulation, where exact pose and geometry references are
available (Section~\ref{sec:sim_ablation}). The post-fusion frame is the
configuration evaluated there and the one planned for the flight campaign, once
the leveling rotation applied to the autopilot's vision input is validated on
the vehicle (Section~\ref{sec:lessons}).
```

Line 1514:

```latex
post-fusion pose, the configuration planned for the flight campaign, or from
the raw cuVSLAM pose. Injecting
```

Line 1802:

```latex
\dg{Evaluate the mapping frame, not only the estimator.} A dense map is only as
consistent as the pose it is integrated in. The fleet maps from visual odometry
leveled for the camera tilt, which keeps the map level but lets odometry drift,
strongest in the vertical axis, accumulate into planning. Feeding the fused
state estimate to mapping keeps the map aligned with the pose the controller
flies, and the simulation study of Section~\ref{sec:sim_ablation} measures the
gain. Any platform that fuses visual odometry with an independent height source
should evaluate mapping in the fused frame for the same reason.
```

Line 1892:

```latex
a rangefinder in the autopilot estimator, drives mapping from the visual
odometry leveled for the camera tilt, with mapping from the post-fusion pose
evaluated in simulation as the way to keep odometry drift out of the map, and
isolates the two accelerator consumers across process boundaries. We reported link margin, radio
```

### `cef9c92` Splitter: cuVSLAM receives the emitter-on infrared frames

Line 700:

```latex
alternates between frames, and a splitter routes frames by emitter state:
cuVSLAM receives the infrared frames captured with the emitter on, and the depth
stream is taken from the alternate frames. Both pipelines therefore draw on the
same physical sensor through different frame sets. The camera is mounted with a fixed
```

Line 842:

```latex
emitter state and passes the emitter-on infrared frames to cuVSLAM. cuVSLAM
produces stereo-inertial visual odometry that crosses the
```

### `7cf03c7` LoRa schedule: reword convergence for half-duplex radios and firmware v7.5

Line 1111:

```latex
so once the team shares a common frame coordination messages do not collide and
the latency of any message is bounded by one frame. For $n$ vehicles, a slot duration $\tau$, and a per-cycle guard
```

Line 1134:

```latex
The schedule is maintained by decentralized clock synchronization with no
coordinator vehicle, so the schedule does not depend on any one vehicle. Each
vehicle holds a reference identifier, the smallest vehicle identifier whose
schedule it is currently following, initialized to its own. Each packet carries
its sender's reference identifier, its slot timing, and the set of vehicles the
sender currently hears, and on hearing a neighbor that follows a smaller
reference identifier a vehicle adopts that reference and aligns its frame to the
neighbor's, compensating for the message airtime.

The radios are half-duplex, so two vehicles whose slots overlap in time transmit
together and cannot hear each other, and the adoption rule alone never separates
them. Three mechanisms restore the hearing that synchronization depends on.
While any teammate is unheard, a vehicle stays silent in a randomly chosen one
in ten of its own slots and listens, so that a vehicle transmitting over its
slot is heard within a few cycles. A vehicle that no neighbor has reported hearing for
1.5\,s shifts its frame by a random offset of at least one slot. A vehicle that
cannot hear its reference directly takes the reference timing relayed by a
neighbor following the same reference, which also keeps the remaining vehicles
on a common frame when the reference vehicle departs.
```

Line 1156:

```latex
and stable, and call a synchronization round an interval, shorter than the
1.5\,s timeout of the timing shift, in which every vehicle hears each of its
neighbors at least once; the listen-only slots are what make such rounds occur
when slots initially overlap. Then within $D$ rounds, where $D$ is the graph
diameter, every vehicle adopts reference identifier $m$ and aligns to a single
common frame. Vehicle $m$ never adopts a
```

Line 1165:

```latex
rounds. Under this assumption no vehicle goes unheard, so no timing shift
occurs, a vehicle changes its reference only to a strictly smaller value, and
the process does not oscillate. Once
```

Line 1580:

```latex
the vehicles actually carry. Slots are disjoint once the team shares a frame, so
the study confirms the absence of collisions after convergence and measures
message latency against the bound of Equation~\eqref{eq:cycle}, the time to
reach a common frame from power-up and after a restart or departure of the
reference vehicle, the number of rounds to converge against the graph diameter,
and the recovery after a partition re-merges.
\todo{Report latency against the $n\tau + g$ bound, convergence from power-up
and after loss of the reference vehicle, convergence rounds against graph
diameter, and re-merge recovery time.}
```

Line 1844:

```latex
\dg{A collision-free schedule still has to be reached.} Disjoint slots hold only
after the team shares a frame, and half-duplex radios whose slots overlap cannot
hear the packets that would align them. In an event simulation of the preceding
firmware revision, four radios failed to reach a common frame in 39\% of cold
starts and in 60\% of restarts of the reference vehicle. Randomized listen-only
slots and a timing shift for a vehicle that nobody hears were added in response,
and removed the deadlock in the same simulation. A time-division link without a
coordinator should be tested from power-up and from a mid-mission restart, not
only in steady state.

```

### `921b6de` Limitations: replace the outdated compile-time transmit power bullet

Line 1891:

```latex
\item The frame that the coordination radio passes to the onboard computer
carries no sequence number or timestamp, so coordination-link latency is not
yet measured in flight.
```

### `a86043b` Radio characterization: remove packet fields the frame does not carry

Line 1453:

```latex
a receiver front end to a transmitter's full output over a cable. The receiver
logs signal strength and signal-to-noise ratio for each packet, so delivery
ratio is computed per link condition. Figure~\ref{fig:linkmargin} reports delivery ratio
```

### `7977fd8` Estimator: vision contributes horizontal position and heading only

Line 953:

```latex
time step. The visual estimate is fused as horizontal position and heading,
with no vision-derived velocity, because velocity from the visual front end
carries latency-correlated error that grows during fast motion. Height comes
from the rangefinder, which is the estimator's height reference, and vision
height is not fused (\texttt{EKF2\_EV\_CTRL} of 9 and \texttt{EKF2\_HGT\_REF}
of 2).
```

Line 962:

```latex
and height comes from the rangefinder. When vision drops out, the rangefinder continues to
```

Line 981:

```latex
and heading come from cuVSLAM, height comes from the downward rangefinder, and
velocity comes from inertial propagation. The EKF2 post-fusion pose combines
```

### `8daa864` Table 15: coordination-link achieved rate and keyframe exchange

Line 1763:

```latex
\caption{In-flight communication performance, four-vehicle configuration. The
coordination-link rate is the pose rate received from each peer in the
shakedown flights that preceded the campaign. In the two-vehicle shakedown
flight the vehicles exchanged 89 and 84 keyframes in 2\,min over the WiFi link,
which was operated through an access point. Remaining values to be filled from
logged trials.}
```

Line 1777:

```latex
Achieved rate (Hz) & 5.3 & \PH\\
```

### `cb4e84c` Results: note the four shakedown flights that preceded the campaign

Line 1785:

```latex
Four shakedown flights on October 23, 2026, with about 16\,min in offboard mode
and one abort caused by a frozen autopilot serial link, preceded the campaign
and are excluded from Tables~\ref{tab:exploration}, \ref{tab:degradation}, and
\ref{tab:campaign}.

```

### `944d500` Lessons: recording load and paired leveling rotations

Line 1869:

```latex
\dg{Treat the recording profile as part of the flight configuration.} Logging
on the onboard computer competes with the pipelines it observes. On the bench,
recording the signed-distance point clouds raised processor load from 55\% to
70\% and opened gaps of 0.35\,s in the inertial stream reaching the visual
front end, which was enough to disturb the visual odometry. What is recorded
should be fixed, tested under load, and versioned with the rest of the flight
configuration.

\dg{Verify paired rotations against a known motion.} The camera tilt is removed
on two separate paths, one to the mapper and one to the autopilot, and each
applies its own leveling rotation. A sign error on the autopilot path left its
vision input $40^{\circ}$ off level while the map stayed correct, and nothing
on a stationary vehicle revealed it. Rotations applied on separate paths should
be verified as a pair, by moving the vehicle a measured distance along one axis
and comparing both paths against that motion.

```

### `332e010` Lessons: use the measured four-vehicle CPU means for the recording load

Line 1871:

```latex
recording the signed-distance point clouds raised processor load from 53\% to
69\%, averaged over the four vehicles, and opened gaps of up to 0.35\,s in the
inertial stream reaching the visual front end, which was enough to disturb the
visual odometry. What is recorded
```
