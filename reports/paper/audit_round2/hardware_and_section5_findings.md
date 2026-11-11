# JFR draft audit, round 2: hardware items (1.2-1.4, 2.1, 2.2) and Section 5 items (5.1-5.4)

Audit date: 2026-11-11. Read-only. No network device was contacted, no package installed,
`main.tex` not edited by this audit.

**Draft revision.** `/home/ssg/lunar_lab/multi-drone-project/latex-multi-drone-JFR/main.tex`
changed while this audit ran. It was commit `52adf36` (1911 lines) at the start and commit
`332e010` (1974 lines, 2026-11-11 12:31) at the end; the intervening commits are text edits by
someone else in Sections 1, 4.3, 6, 7, 9, 12. **Line numbers below are for `332e010`.** Where
the task brief used an older number it is given as "(was N)". None of the lines quoted in this
report had their text changed between the two commits; only their numbers moved.

Status words: FILLED / DATA EXISTS not in draft / PARTIAL / NO DATA FOUND. Contradictions are
reported, not reconciled.

---

## Summary table

| Item | Status | One line |
|---|---|---|
| 1.2 frame size | NO DATA FOUND (measurement or purchase record); draft is internally inconsistent | Draft has three values: `\PH` (L521), 427 mm (L546), 443 mm (L765). No measurement, order or product page on the laptop. Older drafts and CAD names give other frame and propeller sizes. |
| 1.3 ESC | PARTIAL, with a contradiction | Draft says "LANNRC 45A" (added 2026-11-06). Every other record on the laptop says "Cyclone 35A". Firmware: no record. PX4 exports show DShot600 (delta) and DShot300 (ghost). |
| 1.4 fuse, converter current, output voltage | NO DATA FOUND | Only related material: a 2025 photo of a DFRobot 5 A power module, and Jetson-side `VDD_IN` power from 2026-10-28 bench runs. |
| 2.1 mass table | PARTIAL | 5 of 7 rows have blank addends (8 blank slots). No weighing record of any kind on the laptop; 1867 g appears only in the draft and in lab-meeting slides. |
| 2.2 cost, takeoff mass, endurance | NO DATA FOUND | No BOM, price list, order history or invoice for the drones. |
| 5.1 figures | 3 real, 10 placeholders | See table. Candidate sources exist for some placeholders. |
| 5.2a vocabulary | DATA EXISTS not in draft | Runtime logs on all four vehicles: "Vocabulary loaded: 4089 words" from `sp_vocab_4096.pkl`. |
| 5.2b pairwise consistency | DATA EXISTS not in draft; draft `\todo` is contradicted | PCM is implemented and enabled; thresholds 1.0 m and 15 degrees. |
| 5.3 bibliography | PARTIAL | `zhang2024gvp` cited with no bib entry (only undefined citation). Details below. |
| 5.4 end matter | all placeholders | Author contributions, acknowledgments, repository URL, video URL are placeholders; middle authors are a placeholder. |

---

## 1.2 Frame size and propellers

### What the draft says (three different values)

- `main.tex:520-521` (was 518-519): "The airframe is a Mark4 V2 true-X quadrotor frame in 3K
  carbon fiber, with a `\PH`\,mm motor-to-motor diagonal." Still a placeholder.
- `main.tex:545-550` (was 543-548), inside a `\todo`: "Resolve the frame and propeller pairing
  before submission. A 427\,mm diagonal is the 10\,in Mark4 V2 variant, while 7x4.5\,in
  propellers on a 2806.5 1300KV motor are a 7\,in configuration; either the diagonal is that of
  the 7\,in variant or the vehicle runs propellers smaller than the frame accommodates. State
  whichever is true and give the reason. Record ESC current rating and firmware."
- `main.tex:765` (was 764), Table `tab:platform`: "Airframe & Mark4 V2 true-X, 3K carbon fiber,
  443 mm diagonal".
- Propellers in the draft: "7x4.5\,in propellers" at `main.tex:541` and `main.tex:766`.

Provenance of "443 mm": it entered in commit `52adf36` ("Update on Overleaf", 2026-11-06,
author Lewis Busch-Vogel), the same commit that added "LANNRC 45A" and Figures 1 and 3. Before
that commit the table cell was `\PH`. No file on the laptop explains where 443 came from.

### Records found

No motor-to-motor measurement and no purchase or product record for the frame was found.
What exists:

| Record | Path | What it says |
|---|---|---|
| Figure 1 component list in an earlier compiled draft (2026-10-19) | `/home/ssg/Downloads/Multi_drone_exploration_system_JFR_Wiley (1).pdf` (pdftotext lines 346-347) | "1 Airframe RJX Mark4 V2 carbon", "2 Propellers 7x4.5 in". Gives a brand (RJX) but no size. The current `figures/Figure_1.pdf` has numbered callouts only, no component list. |
| Lab meeting slide, 2026-09-22 | `/home/ssg/Downloads/Lab_Meeting_Semantic-RAG_and_Multi-Drone_rev3.pptx`, slide 19 and its speaker notes | Slide: "Mark4 V2 true-X, 3K carbon fiber", "iFlight XING 2806.5 1300KV, 7x4.5″ props", "Diagonal, ESC model, and full cost breakdown: pending final BOM." Notes: "standard 7 inch prop motors ... right in line with a standard 7 inch frame class ... the exact frame diagonal, ESC model, and full itemized cost are still pending the final bill of materials". Same slide in `/home/ssg/lunar_lab/weekly_presentation_Fall2026/Lab_Meeting_Semantic-RAG_and_JFR_Multi-Drone_rev1.pptx` (slide 20). |
| Experiment checklist, 2026-09-17 | `/home/ssg/Downloads/JFR_experiment_checklist.docx` | Unticked: "Measure frame diagonal motor to motor. Resolves the 427 mm vs 7x4.5 in propeller mismatch." |
| Archived draft | `/home/ssg/lunar_lab/multi-drone-project/latex-multi-drone-JFR/archive/main_detailed.tex:227, 520-522, 544-549` | "four identical custom 10-inch quadrotors"; "(1) 10-inch carbon quadrotor frame; (2) 5-inch tri-blade propellers; (3) T-Motor MN3110 700KV motors with 40\,A 4-in-1 ESC"; "all-up weight ~1.6 kg". Provenance of these values unknown. |
| Archived draft | `.../archive/main_JFR.tex:488` | "Quadrotor on $9\times4.5$\,in.\ propellers, 4S LiPo (`\PH` min endurance)". |
| CAD file names (dated 2025-08-28) | `/home/ssg/Downloads/Drone Assembly Parts/` | `FAB_PROPELLER - 9x4.5 CCW.sldprt`, `FAB_PROPELLER - 9x4.5 CW.sldprt`, `3_blade_prop.SLDPRT`, `iflight-xing-2207-1800kv-1.snapshot.1.zip`, `Frame.SLDPRT`, `Frame.SLDASM`, `DroneAssembly.SLDASM`. The SolidWorks files are binary; no dimension could be read from them here. Opening `Frame.SLDPRT` in SolidWorks would give the CAD diagonal. |
| CAD render used on the slide | `/home/ssg/lunar_lab/weekly_presentation_Fall2026/droneCADassembly.png` | Shows two-blade propellers. No dimension. |
| Photos of an earlier build, 2025-03-10 | `/home/ssg/Downloads/drone_documentation/topview.jpeg`, `droneharness1.jpeg` | Three-blade red propellers, Pixhawk 4 mini, a tag reading "PROMETHEUS". This is not one of the four current Cube Orange+ vehicles as built today, so it is not evidence of what is fitted now. |
| PX4 parameter exports | e.g. `/home/ssg/Desktop/deltaparams.params` (2026-08-17) | `CA_ROTOR0_PX 1.0`, `CA_ROTOR0_PY 1.0` (PX4 defaults, not arm lengths), `SYS_AUTOSTART 4001`. No geometry. |
| Earlier audit | `/home/ssg/multi-robot-coordination/jfr_codefacts_report.md:29-36`, `jfr_codefacts.md:676-705` | Same conclusion: frame diagonal NOT FOUND; conflict flagged. |

### Contradictions (not reconciled)

1. Draft against itself: `\PH` (L521), 427 mm (L546), 443 mm (L765).
2. Propellers: draft 7x4.5 in; `archive/main_JFR.tex` 9x4.5 in; CAD file names 9x4.5 in;
   `archive/main_detailed.tex` 5-inch tri-blade on a 10-inch frame.
3. Motors: draft iFlight XING 2806.5 1300KV; CAD archive name XING 2207 1800KV;
   `archive/main_detailed.tex` T-Motor MN3110 700KV.

### Where I looked and found nothing

Text search for `Mark4`, `Mark 4 V2`, `2806.5`, `LANNRC`, `XING 2`, `1867 g` across
`/home/ssg` (excluding node_modules, .cache, site-packages, PX4 source trees, build, install,
conda, venvs): hits only in the draft, its archive, the two `jfr_codefacts*.md` files and
Claude session logs. `/home/ssg/lunar_lab/MASTER_notes.txt` (no hardware content). All
`.docx/.pptx/.xlsx` under Downloads, Documents, Desktop, `lunar_lab` (text extracted and
searched). Chrome history and bookmarks and both Firefox `places.sqlite` files (string search
for the product names): no product or order pages. No file named like order, invoice, receipt
or BOM relates to the drones (the receipt-like PDFs in Downloads are unrelated personal items).
Not opened: `Documents/bank_statements`, tax, visa and passport folders.

---

## 1.3 ESC model, current rating, firmware

### What the draft says

- `main.tex:541-542` (was 539-540): "driven by LANNRC 45A electronic speed controllers".
- `main.tex:766` (was 765): "LANNRC 45A speed controllers".
- `main.tex:550` (was 548), still inside the `\todo`: "Record ESC current rating and firmware."
- Firmware: not stated anywhere in the draft.

"LANNRC 45A" entered in commit `52adf36` (Overleaf, 2026-11-06, Lewis Busch-Vogel), replacing
`\PH{}`. The string "LANNRC" occurs nowhere else on the laptop.

### Records found

| Record | Path | What it says |
|---|---|---|
| Figure 1 component list, compiled draft of 2026-10-19 | `/home/ssg/Downloads/Multi_drone_exploration_system_JFR_Wiley (1).pdf` (pdftotext line 350) | "4 Speed controllers Cyclone 35A" |
| CAD | `/home/ssg/Downloads/Drone Assembly Parts/CYCLONE 35A.step` and `cyclone-35a-esc-1.snapshot.6.zip` | ESC model used in the vehicle CAD is a "Cyclone 35A". |
| Photo, 2025-03-10 | `/home/ssg/Downloads/drone_documentation/ESCwiring.jpeg` | Four individual arm-mounted ESCs, labels read "CYCLONE" and "35A". Smaller print on the label is hard to read at this resolution; it appears to say "BLHeli_S 2-6S Lipo", not confirmed. The vehicle in the photo carries a "PROMETHEUS" tag (earlier build). |
| Archived draft | `.../archive/main_detailed.tex:522` | "40\,A 4-in-1 ESC" (contradicts the draft's "four individual speed controllers", `main.tex:576`). |
| PX4 exports: ESC protocol only | `/home/ssg/Desktop/deltaparams.params` (2026-08-17); `/home/ssg/Documents/QGroundControl/Parameters/deltamar13.params`, `deltamar25params.params`, `ghostmar13.params`, `ghostmar16.params`, `ghostmar25params.params` (2026-03); `/home/ssg/Desktop/GhostPX4Logs/cube_orange_plus_prometheus.params` (2025-09-19) | Motors are on the AUX outputs (`PWM_AUX_FUNC1..4 = 101..104`). delta: `PWM_AUX_TIM0 = -3`. ghost and prometheus: `PWM_AUX_TIM0 = -4`. Decode from `/home/ssg/PX4-Autopilot/src/drivers/pwm_out/module.yaml:24-25`: `-4: DShot300`, `-3: DShot600`. All: `DSHOT_TEL_CFG = 0` (no ESC telemetry port), `MOT_POLE_COUNT = 14`, `DSHOT_MIN = 0.055`. So the ESCs accept DShot; the two vehicles with exports are set to different DShot rates. No model, rating or firmware is recorded in PX4. No export for buckshee or thunderstrike on the laptop. |
| Earlier audit | `/home/ssg/multi-robot-coordination/jfr_codefacts_report.md:40-43` | "A2. ESC: NOT FOUND." |

### Contradiction (not reconciled)

Draft: LANNRC 45A. Earlier compiled Figure 1, CAD and the 2025 photo: Cyclone 35A. Different
maker name and different current rating. Nothing on the laptop shows which is on the four
vehicles today.

Status: model and rating are filled in the draft but contradicted by every other record;
firmware NO DATA FOUND (no BLHeli/AM32/Bluejay configurator export, screenshot or note; searched
for `BLHeli`, `AM32`, `bluejay` in all text and office files listed under 1.2).

---

## 1.4 Fuse rating, peak converter input current at MAXN, converter output voltage

### What the draft says

- `main.tex:563-566`: "a custom power distribution assembly built around a 5\,A, 350\,kHz,
  25\,V synchronous buck converter module, with an inline fuse on the computer branch."
- `main.tex:572-574` (was 570-572): "`\todo{`Record the fuse rating and the measured peak input
  current at the converter under the MAXN power mode, and confirm the converter output voltage
  setting.`}`"
- Fig. 2 placeholder `main.tex:586-591`: "the autopilot supply from `\PH`".

### Records found

NO DATA FOUND for all three values. Related material only:

- `/home/ssg/Downloads/drone_documentation/buck.jpeg` (2025-03-10): a DFRobot board
  silk-screened "Power Module" and "5A", with an ON/OFF push switch, a slide switch beside a
  "5V" marking, a trimmer potentiometer, output wired to a DC barrel plug, input wired to two
  XT60 connectors, and an extra electrolytic capacitor soldered on. No fuse is visible in the
  photo. The photo does not show the output voltage setting.
- `.../archive/main_detailed.tex:604-609`: "separate clean 5\,V regulators feeding the flight
  controller and a separate 4\,A switching regulator feeding the Jetson" (4 A, against the
  current draft's 5 A; provenance unknown).
- Jetson-side power, not converter input current: `tegrastats.log` files from the 2026-10-28
  bench runs, `/home/ssg/multi-robot-coordination/experiments/results/colour_check_bench1028{,_fig,_rec,_split}/<vehicle>/tegrastats.log`,
  1 Hz samples of the module rail `VDD_IN`. Per-run maximum 12.8 to 15.3 W, medians 10.8 to
  13.6 W, full stack running (highest: `colour_check_bench1028_rec/thunderstrike`, 15.25 W).
  The power mode is not recorded in these logs. This is the Jetson module rail as the on-board
  monitor reports it; it excludes carrier-board loads and converter loss, and 1 Hz sampling
  will miss short peaks. It is not a substitute for a current measurement at the converter
  input.
- Power mode: `/home/ssg/multi-robot-coordination/jfr_codefacts_report.md:403-405` recorded
  delta in MAXN (mode 0) at rest in August; no later record.
- `/home/ssg/Downloads/JFR_experiment_checklist.docx`: unticked "Fuse rating, buck converter
  output voltage, autopilot power source".
- `jfr_codefacts_report.md:46-50`: A4 fuse NOT FOUND, A5 buck output voltage NOT FOUND, A6
  Cube power source NOT FOUND.

Looked for: `fuse`, `buck`, `MAXN`, `XL4015`, `LM2596`, `DFR0205` in all text and office files;
schematic, wiring, KiCad, Fritzing, drawio files home-wide. The only wiring photos
(`wiringspeedybee1.jpeg`, `wiringspeedybee2.jpeg`) are a leaflet for an unrelated flight
controller.

---

## 2.1 Table 2 mass budget (`tab:massbudget`, `main.tex:602-625`)

Caption (`main.tex:604-605`) still reads "All values to be filled from measurement."

### Rows exactly as in `main.tex` (was 612-620)

```
614: Airframe and propulsion & Frame, motors, speed controllers, propellers & 227 +  +  + 32\\
615: Compute & Onboard computer, carrier board, case, fan &28 + + 35 + \\
616: Sensing & Camera, isolation mount, rangefinder &72 + 17.5 + 5g\\
617: Autopilot and interface & Autopilot, serial interface adapter, wiring & 34 +  + \\
618: Communication & LoRa module and case, WiFi antennas, receiver & 7 + 11 + 6 + \\
619: Power & Battery, power distribution assembly, wiring & 467 +  + \\
620: Printed structure & Landing gear, battery case & 84 + 35\\
622: Total takeoff mass & & 1867\\
```

| Row | Items listed | Addends present | Blank |
|---|---|---|---|
| Airframe and propulsion (614) | 4 | 227, 32 | 2 (by position: motors, speed controllers) |
| Compute (615) | 4 | 28, 35 | 2 (by position: carrier board, fan) |
| Sensing (616) | 3 | 72, 17.5, 5 | none; stray unit "5g" |
| Autopilot and interface (617) | 3 | 34 | 2 (adapter, wiring) |
| Communication (618) | 3 items but 4 addend slots | 7, 11, 6 | 1 trailing blank; slots do not match the item list |
| Power (619) | 3 | 467 | 2 (power distribution assembly, wiring) |
| Printed structure (620) | 2 | 84, 35 | none |

No row shows a row sum; every row is an unsummed expression. Arithmetic only: the addends that
are present add to 1060.5 g, so 806.5 g of the stated 1867 g total sits in the 8 blank slots.
Table `tab:platform` still has "Takeoff mass & `\PH`\,g" (`main.tex:776`) although Table 2
states 1867.

### Weighing records

NO DATA FOUND. There is no spreadsheet, note, photo of a scale, or message with component or
whole-vehicle masses anywhere searched.

Where the numbers in the draft came from: the addends and the 1867 total entered `main.tex`
in commit `77180d6` ("Update on Overleaf", 2026-09-18, author "ekim612 Kim"). They were typed
into Overleaf; the source sheet is not on this laptop.

Other occurrences of 1867 g (all downstream of the draft, none a measurement record):
- `/home/ssg/Downloads/Lab_Meeting_Semantic-RAG_and_Multi-Drone_rev3.pptx` slide 19 (2026-09-22):
  "Total assembled takeoff mass: 1867 g." Speaker notes: "Total assembled mass is 1867 grams".
- Same text in `/home/ssg/Downloads/Lab_Meeting_Semantic-RAG_and_JFR_Multi-Drone_rev1.pptx`
  (slide 21) and `/home/ssg/lunar_lab/weekly_presentation_Fall2026/Lab_Meeting_Semantic-RAG_and_JFR_Multi-Drone_rev1.pptx` (slide 20).
- Compiled PDFs of the draft in `/home/ssg/Downloads/`.

Conflicting older statement: `.../archive/main_detailed.tex:548-549` "all-up weight ~1.6 kg
including a 4S 5200 mAh LiPo battery" (provenance unknown).

Checklist: `/home/ssg/Downloads/JFR_experiment_checklist.docx`, unticked "Weigh all seven
subsystems, then the whole vehicle".

---

## 2.2 Table 5 cost (`tab:cost`, `main.tex:793-815`), Table 4 takeoff mass and endurance

### Draft state

- `tab:cost`: all seven cells are `\PH` (`main.tex:805-812`); caption "All values to be filled
  from the bill of materials."
- `tab:platform`: "Takeoff mass & `\PH`\,g" (`main.tex:776`), "Endurance & `\PH`\,min in
  exploration flight" (`main.tex:777`). Caption (`main.tex:755-756`) says "Part numbers,
  suppliers, and unit costs are in the released bill of materials."
- `tab:powerbudget`: both endurance rows `\PH` (`main.tex:650-651`).

### Records found

NO DATA FOUND. No BOM, price list, order history or invoice for the drone fleet exists on the
laptop. Specifically:

- Files with BOM-like names are for other projects: `/home/ssg/Downloads/UGV_Agricultural_BOM.docx`
  (ground vehicle), `AUC-S_Lite_Build_Buy_List.xlsx`, `AUC-S_Lite_Mobile_Hardware_Buy_List.xlsx`,
  `Antomata_Modular_Unit_v2/docs/calc/v1_bom.md`, `Lunar_lab_open_robot_sandilya.pdf`. They
  contain prices for parts the drone also uses (for example "Intel RealSense D435i $334" in
  `UGV_Agricultural_Application.docx`), but they are not records of what was paid for the
  drones and are not used here.
- Slide 19 (above): "full cost breakdown: pending final BOM."
- Reviewer comment in `/home/ssg/lunar_lab/multi-drone-project/JFR-Wiley-drafts/Multi_drone_exploration_system_JFR_Wiley (1).pdf`
  (2026-06-25, near Section 4.1): "but jetson is $1200, thats not cheap at all so rephrase."
  A remark, not a price record.
- `.../archive/main_detailed.tex:612-615`: "The component-level cost (excluding labour) is
  approximately \$2.7\,k per drone, dominated by the Jetson Orin NX 16 GB module and the Cube
  Orange Plus." Provenance unknown; same file claims "~10 minutes hover endurance" (line 605).
- `/home/ssg/multi-robot-coordination/jfr_codefacts.md:718`: "No BOM file located despite the
  drafts referencing a 'released bill of materials'."
- Endurance: no timed hover-to-exhaustion or mission-duration record was found in the places
  searched for this item (flight-log extraction belongs to the electrical-budget item and was
  not repeated here).

Searched: file names containing bom, invoice, order, receipt, purchase, parts list, buy list,
cost, price, budget across `/home/ssg`; text of all office documents for `$` amounts next to
drone part names; browser history for product names.

---

## 5.1 Figures

13 figure environments are active in `main.tex` (plus one commented-out duplicate at L508-516).
`figures/` contains exactly three files: `Figure_1.pdf`, `Figure_3.pdf`, `vibration_spectra.pdf`.

| # | Label | Lines (was) | Kind | File exists | Candidate sources for placeholders |
|---|---|---|---|---|---|
| 1 | `fig:exploded` | 493-506 (491) | `\includegraphics{figures/Figure_1.pdf}` | yes (913 kB, 1920x1080 pt, three raster images, numbered callouts 1-16) | Already filled. Sources: `/home/ssg/lunar_lab/weekly_presentation_Fall2026/droneCADassembly.png`, `droneAssembly_expandedview.png`; CAD in `/home/ssg/Downloads/Drone Assembly Parts/`. |
| 2 | `fig:power` | 585-594 (583) | `\figplaceholder` (electrical schematic) | n/a | None. No schematic or diagram source anywhere. Only `/home/ssg/Downloads/drone_documentation/buck.jpeg` and `ESCwiring.jpeg` (2025 photos of an earlier build). |
| 3 | `fig:arch` | 837-855 (836) | `\includegraphics{figures/Figure_3.pdf}` | yes (39 kB, vector, cairo, 2026-11-06) | Already filled. Sources: `/home/ssg/Downloads/software_architecture.drawio`, `software_architecture.svg`, `per_vehicle_software_architecture_v2.svg` (2026-10-20). Note: the figure text reads "Place recognition ... color 15 Hz"; see 5.2 for what the code subscribes to. |
| 4 | `fig:axes` | 979-988 (977) | `\figplaceholder` (sensor-to-axis diagram) | n/a | None found. |
| 5 | `fig:links` | 1075-1084 (1069) | `\figplaceholder` (two-link schematic) | n/a | Partial: slide 21 of `/home/ssg/Downloads/Lab_Meeting_Semantic-RAG_and_Multi-Drone_rev3.pptx` ("Our solution: dual-radio coordination", native PowerPoint shapes plus a TDMA frame strip `ppt/media/image26.png`); `/home/ssg/multi-robot-coordination/reports/rf_results/figures/fig4_lora_tdma.png`. No diagram in `reports/patent/DUAL_RADIO_PATENT_WRITEUP.md` or in the patent `.docx` files (no embedded media). |
| 6 | `fig:vibration` | 1291-1309 (1269) | `\includegraphics{figures/vibration_spectra.pdf}` | yes (19 kB, Matplotlib, 2026-09-25) | Already filled. Script: `/home/ssg/lunar_lab/multi-drone-project/code/vibration_analysis/vib_spectra.py`. |
| 7 | `fig:thermal` | 1385-1393 (1363) | `\figplaceholder` ("To be generated from logged runs") | n/a | Related data only: `tegrastats.log` and `colour_check.csv` under `/home/ssg/multi-robot-coordination/experiments/results/colour_check_bench1028*/` (10 to 15 minute runs, not full-battery, power mode not varied). |
| 8 | `fig:linkmargin` | 1478-1485 (1457) | `\figplaceholder` | n/a | None; the sweep has not been run (`reports/paper/JFR_CHECKLIST_UPDATE_2026-11-11.md`, item 2.6). |
| 9 | `fig:faults` | 1560-1567 (1538) | `\figplaceholder` | n/a | None found. |
| 10 | `fig:indooroutdoor` | 1636-1643 (1611) | `\figplaceholder` (photos plus map) | n/a | Video only: `/home/ssg/lunar_lab/weekly_presentation_Fall2026/indoorflight1.MOV`, `indoorflight2.MOV` (frames could be extracted). No still photos of the site. |
| 11 | `fig:course` | 1645-1650 (1620) | `\figplaceholder` | n/a | None found. |
| 12 | `fig:field` | 1652-1658 (1627) | `\figplaceholder` (photos plus trajectories) | n/a | Trajectory plots from the 2026-10-23 field day: `/home/ssg/multi-robot-coordination/reports/visualization/e4/paths_by_state.{pdf,png}`, `team_map.{pdf,png}`, `coverage.{pdf,png}`, and the same under `e4c/`. Outdoor video: `weekly_presentation_Fall2026/dronevid1.MOV`, `dronevid2.MOV`. No still photos. |
| 13 | `fig:fusedmap` | 1710-1716 (1685) | `\figplaceholder` (four-vehicle fused map) | n/a | Partial: `reports/visualization/e4c/team_map.{pdf,png}`, `e4c_3d_realtime_still.png`, `reports/visualization/fig_fleet_overview.png`. These are from single-vehicle flights and one two-vehicle overlap; no four-vehicle fused map exists. |

Commented-out duplicate: `main.tex:508-516` is the old Figure 1 placeholder with a second
`\label{fig:exploded}` (inside comments, harmless).

---

## 5.2 Place recognition (Section 6.4, `sec:placerec`, `main.tex:1040-1064`, was 1034-1058)

Code read from the laptop copies in `/home/ssg/jfr_dev` (`multi_drone_nvblox` at commit
`3647995`, branch `feature/jfr-multi-drone-field`). The vehicles were not contacted, so "what
is deployed" rests on the collected runtime logs cited below.

### (a) Vocabulary size

Draft: `main.tex:1045-1046` says only "a bag-of-words index over a precomputed SuperPoint
vocabulary"; no size is stated. `main.tex:1061-1064` `\todo`: "The vocabulary size of the file
actually loaded was not confirmed, and no pairwise-consistency threshold was found in the code,
so the claim in this section that pairwise-consistency screening rejects inconsistent
transforms is currently unsupported and must be verified or removed."

DATA EXISTS not in draft:

- **Loaded size, from runtime logs on all four vehicles: 4089 words.**
  - `/home/ssg/multi-robot-coordination/collected/delta/pane_Tab_6__Loop_Closure__SuperPoint___LightGlue__1781224060.log:3-4`
  - `/home/ssg/multi-robot-coordination/collected/buckshee/pane_Tab_6__Loop_Closure__SuperPoint___LightGlue__1781224080.log:3-4`
  - `/home/ssg/multi-robot-coordination/collected/thunderstrike/pane_Tab_6__Loop_Closure__SuperPoint___LightGlue__1781224081.log:3-4`
  - `/home/ssg/multi-robot-coordination/collected/ghost/pane_Tab_6__Loop_Closure__SuperPoint___LightGlue__1785794537.log:5-6`
  - Each reads: "Loading vocabulary from /workspaces/isaac_ros-dev/src/multi_drone_nvblox/models/sp_vocab_4096.pkl..." then "Vocabulary loaded: 4089 words in 0.2s" (0.3 s on buckshee).
  - Log dates: 2026-06-11 (delta, buckshee, thunderstrike) and 2026-08-03 (ghost). No later
    log of this node was found, so the 2026-10-23 field-day value is not directly evidenced.
- Configured file: `vocabulary_file:=.../models/sp_vocab_4096.pkl` in
  `/home/ssg/multi-robot-coordination/launch_delta_frontierexplore_tmux.sh:136`,
  `launch_buckshee_frontierexplore_tmux.sh:136`, `launch_ghost_frontierexplore_tmux.sh:141`,
  `launch_thunderstrike_frontierexplore_tmux.sh:136`.
- Nominal size: the tree is K^L leaf words. Builder defaults
  `/home/ssg/jfr_dev/multi_drone_nvblox/scripts/build_sp_vocabulary.py:150-153`
  `--branch_factor 8`, `--depth 4`; line 159 `vocab_size = args.branch_factor ** args.depth`
  (4096). Node defaults `scripts/loop_closure_sp_node.py:195-196` `vocab_branch_factor 8`,
  `vocab_depth 4` (used only for the online-vocabulary mode, which the launch scripts do not
  enable). The loaded count (4089) is below the nominal 4096; the file name says 4096.
- The `.pkl` file itself is not on the laptop (`/home/ssg/jfr_dev/multi_drone_nvblox/models/`
  holds only `ade20k/`), so its contents were not inspected.
- Other values in the code that differ from this: `scripts/loop_closure_node.py:119`
  (`vocab_branch_factor 10`, a different, older node) and `docs/loop_closure_analysis.md:103`
  (10). `scripts/vocabulary_tree.py:36` class default `branch_factor=10, depth=4`.

Incidental contradiction found while checking this (not reconciled): the draft says the
place-recognition stage "runs on the color stream" (`main.tex:1055-1056`; also `main.tex:691-694`
"the color stream provides the images used for inter-robot place recognition", and the text in
`figures/Figure_3.pdf`). The node subscribes to the stereo infrared pair from the emitter
splitter and to no colour topic: `scripts/loop_closure_sp_node.py:359-366`
(`.../camera0/realsense_splitter_node/output/infra_1` and `infra_2`); its header comment line 6
says "Subscribes to THIS robot's stereo infra + odometry topics". The vocabulary builder also
defaults to the left infrared topic (`build_sp_vocabulary.py:143-145`).

### (b) Pairwise-consistency screening

Draft sentence (`main.tex:1051-1055`): "Accepted matches yield a relative transform that aligns
submaps before signed-distance fusion, and pairwise-consistency screening rejects transforms
inconsistent with the accumulated set of inter-robot constraints, in the manner of
decentralized metric-semantic SLAM `\cite{tian2022kimera}`."

DATA EXISTS not in draft. The check exists, is enabled in the launch scripts, and has
thresholds. This contradicts the draft's own `\todo` ("no pairwise-consistency threshold was
found in the code").

- Implementation: `/home/ssg/jfr_dev/multi_drone_nvblox/scripts/pcm.py` (Pairwise Consistency
  Maximization; the file header cites Mangelson et al., ICRA 2018, lines 3-7). Two closures are
  consistent when `(t_err <= t_thresh) and (r_err_rad <= r_thresh_rad)` (`pcm.py:147`); the
  largest mutually consistent set (maximum clique) is kept.
- Thresholds as used by the node: `scripts/loop_closure_sp_node.py:189-192`
  `use_pcm True`, `pcm_t_thresh 1.0` (metres), `pcm_r_thresh_deg 15.0`, `pcm_inlier_override 0`.
  Passed to the filter at `loop_closure_sp_node.py:1227-1230`. Applied in `_update_alignment`
  (`:1183-1192`) only when at least two closures exist (`if self.use_pcm and len(closures) >= 2`).
- Library defaults differ from the node: `pcm.py:35-36` and `:60-61` `t_thresh = 0.5`,
  `r_thresh_deg = 15.0`. `/home/ssg/jfr_dev/multi_drone_nvblox/AGENTS.md:148`: "pcm_t_thresh
  raised 0.5m → 1.0m."
- Launch: all four `launch_*_frontierexplore_tmux.sh` pass `-p use_pcm:=True` (delta, buckshee,
  thunderstrike line 138; ghost line 143) and do not override the thresholds, so the node
  defaults (1.0 m, 15 degrees) apply.
- A further robust step after PCM also exists: `use_gnc True`, `gnc_c 0.8`
  (`loop_closure_sp_node.py:202-203`).
- The other values in the draft's paragraph match the node defaults: `top_k 5` (:182),
  `lightglue_threshold 0.1` (:186), `min_inliers 15` (:183), `ransac_threshold 3.0` (:185),
  `max_keypoints 500` (:180).
- No runtime log showing PCM accepting or rejecting a closure was found (the four collected
  Tab 6 logs are startup-only, about 1 kB each).

---

## 5.3 Bibliography (`refs.bib`, 641 lines, 53 entries; 33 distinct keys cited in `main.tex`)

### The four named keys

| Key | Cited in `main.tex` | Entry in `refs.bib` | Assessment |
|---|---|---|---|
| `zhang2024gvp` | L242 (was 240) | **none** | Undefined citation. `main.blg:6`: "Warning--I didn't find a database entry for "zhang2024gvp""; `main.log`: "Citation `zhang2024gvp' on page 3 undefined". An uncited entry that may be the intended paper exists under a different key, `gvp2024` (`refs.bib:61-68`): Yichen Zhang, Boyu Zhou, Fei Gao, "GVP-MREP: Generalized Voronoi partitioned multi-robot exploration", IROS 2024, no pages. Whether that is the intended paper, and whether its fields are correct, was not verified. |
| `ohradzansky2022multiagent` | not cited | `refs.bib:635-640` | Placeholder. Quoted: `@article{ohradzansky2022multiagent, author = {{\PH}}, title = {{\PH}}, journal = {Field Robotics}, year = {2022}}` followed by `% VERIFY: your key. Likely the Team MARBLE DARPA SubT Field Robotics paper.` and `% Fill in or remove the citation from Section 2.4.` A complete-looking entry for what may be the same paper exists as `ohradzansky2022darpa` (`refs.bib:297-307`, Field Robotics vol. 2, pp. 1068-1104, 2022), also uncited. |
| `semtech2019sx1262` | L429 (was 427), L1126 (was 1120) | `refs.bib:625-631` | Present but flagged unverified. Quoted: `@misc{semtech2019sx1262, author = {{Semtech Corporation}}, title = {{SX1261/2} Long Range, Low Power, sub-{GHz} {RF} Transceiver Datasheet}, howpublished = {Rev. 1.1}, year = {2019}}` followed by `% VERIFY revision number and year against the datasheet revision you used.` A second, uncited datasheet entry `semtech2019sx126x` (`refs.bib:336-342`) says "(rev.~2.1)" and 2019 with a URL. The two entries give different revision numbers. |
| `cao2023representation` | not cited | **none** | Neither cited nor defined in the current files. It was cited in the older `main_JFR_v2` (`main_JFR_v2.bbl:49`). The nearest current entry is `cao2023tare` (`refs.bib:42-50`, Science Robotics 8(80), 2023, no pages), uncited. `main.tex` now cites `cao2021tare` (L130, L240; `refs.bib:617-623`, RSS 2021, no pages). |

### Other `\cite` keys with no entry

None. `zhang2024gvp` is the only cited key missing from `refs.bib`.

### Entries containing TODO / placeholder / VERIFY wording

Only the two comment lines quoted above (`refs.bib:632`, `refs.bib:641`) and the `{\PH}` fields
of `ohradzansky2022multiagent`. No "TODO", "placeholder" or "unverified" strings elsewhere.

### Cited entries with missing or odd fields (field presence only; not checked against the published record)

- `yamauchi1997frontier` (`refs.bib:392-401`): typed `@article` but carries both `booktitle`
  and `journal = {Proceedings of CIRA}`; no volume. It is a conference paper.
- `liu2024slideslam` (`:96`), `nvidia2025cuvslam` (`:556`): `journal = {arXiv preprint ...}`,
  no volume or pages.
- No `pages` field: `xu2024falcon` (`:52`), `oleynikova2017voxblox` (`:148`),
  `reed2024scenesense` (`:170`), `foehn2022agilicious` (`:182`), `baca2021mrs` (`:195`),
  `hert2023mrsdrone` (`:528`), `millane2024nvblox` (`:545`), `cao2021tare` (`:617`).

### Uncited entries (21), including apparent duplicates of cited ones

`cao2023tare` (42), `gvp2024` (61), `schmuck2019ccmslam` (106; cited twin `schmuck2019ccm`),
`cieslewski2018doppelganger` (117), `nvblox` (128), `cuVSLAM` (140), `hornung2013octomap` (158),
`mohta2018fla` (207; cited twin `mohta2018fast`), `loianno2017estimation` (224),
`ohradzansky2022darpa` (297), `semtech2019sx126x` (336), `qin2018vinsmono` (369),
`campos2021orbslam3` (380), `macenski2022ros2` (405), `dias2006auction` (418),
`burgard2005coordinated` (429), `qin2019general` (456), `rtw88driver` (463), `px4ekf2docs` (471),
`hert2023mrs` (480; cited twin `hert2023mrsdrone`), `ohradzansky2022multiagent` (635).

Note: `main.bbl` is dated 2026-09-25 and predates the current `main.tex`. The checklist
(`JFR_experiment_checklist.docx`) records "One fabricated entry already got through", so the
remaining entries still need checking against the published record; that check needs network
access and was not done here.

---

## 5.4 Author contributions, acknowledgments, availability, URLs

| Element | Location | Current text | Placeholder? |
|---|---|---|---|
| Author list | `main.tex:62-67` | Comment L62: "% AUTHOR ORDER: confirm final order and middle-author affiliations with Lu." L63: `\author{Sandilya Sai Garimella$^{\dagger}$, [middle authors], Lu Gan\\[4pt]` | Yes: "[middle authors]" is literal text that prints in the PDF. |
| Repository URL | `main.tex:105-106` (abstract footnote) | "`\footnote{`Source code and hardware files: `\url{https://github.com/[TBD]}`." | Yes. |
| Video URL | `main.tex:106` | "Supplementary video: `\url{[TBD]}`.`}`" | Yes. |
| Author contributions | `main.tex:1955-1959` (was 1892) | "`\section*{Author contributions}` `\todo{`One sentence per author covering airframe and integration, LoRa firmware and the time-division schedule, mesh networking and kernel work, the perception pipeline, the exploration behavior, flight operations, and manuscript preparation.`}`" | Yes. |
| Acknowledgments | `main.tex:1961-1962` (was 1898) | "`\section*{Acknowledgments}` `\todo{`Funding sources and grant numbers; collaborators and facilities.`}`" | Yes. |
| Data availability | `main.tex:1964-1967` (was 1901) | "All hardware designs, firmware, the software stack, and parameter files that support the findings of this study are released at the repository linked in the abstract." | Real sentence, but it points at the placeholder URL. There is no separate code-availability section. |

Other statements that depend on the release: `main.tex:202` "identically from one repository,
released alongside the vehicle design"; `main.tex:907` "All vehicles run one repository at one
commit"; `main.tex:755-756` "Part numbers, suppliers, and unit
costs are in the released bill of materials" (no BOM exists on the laptop, see 2.2).

For reference only (not in the draft): the local code repositories have these remotes:
`git@github.com:ssaigarimella/active_exploration.git`, `.../multi_drone_nvblox.git`,
`.../px4_offboard.git`, `.../radiohive.git` (all under `/home/ssg/jfr_dev`), and
`.../multirobot_launch_scripts.git` (`/home/ssg/multi-robot-coordination`). Whether any is
public, or is the intended release repository, was not checked (no network access used). No
hardware-design repository and no video link was found.

---

## Contradictions between records and the draft, in one list

1. Frame diagonal: `\PH` (L521) vs 427 mm (L546) vs 443 mm (L765). No measurement on record.
2. ESC: draft "LANNRC 45A" (L542, L766) vs "Cyclone 35A" in the 2026-10-19 Figure 1 component
   list, the vehicle CAD, and a 2025 photo.
3. Propellers and motors: draft 7x4.5 in on XING 2806.5 1300KV vs CAD file names (9x4.5 in,
   XING 2207 1800KV) and archived drafts (9x4.5 in; 5-inch tri-blade on T-Motor MN3110 700KV).
4. Takeoff mass: Table 2 says 1867 g (L622) while Table 4 still has `\PH` (L776); archived
   draft says about 1.6 kg. The filled addends of Table 2 add to 1060.5 g.
5. Buck converter: draft "5 A" module (L564) vs archived draft "4 A switching regulator".
6. Section 6.4 `\todo` says no pairwise-consistency threshold exists in the code; the code has
   one (1.0 m, 15 degrees) and it is enabled.
7. Section 6.4 and Section 4.3 say place recognition uses the colour stream; the node subscribes
   to the infrared pair.
8. Vocabulary file is named `sp_vocab_4096.pkl`; the vehicles log 4089 words loaded.
9. `refs.bib` holds two SX1262 datasheet entries with different revisions (1.1 cited, 2.1
   uncited).
10. ESC protocol differs between the two vehicles with parameter exports (delta DShot600,
    ghost DShot300).

## Not done / limits

- Vehicles not contacted: nothing here confirms what is installed on the four vehicles today.
- SolidWorks files not opened (binary); the CAD diagonal is recoverable from `Frame.SLDPRT`.
- Videos not viewed. Screenshots folder not searched exhaustively (two from 2026-09-18 checked;
  both are pages of other papers).
- Personal-finance, tax, visa and passport folders not opened.
- Bibliography entries were checked for presence and field completeness only, not against the
  published record.
