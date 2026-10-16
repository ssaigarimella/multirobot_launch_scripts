# Experiment schedule — week of 2026-10-19

Everything below is **built, deployed to all four drones, and bench-verified** as of
2026-10-16. Nothing needs building during the week. Run commands from the laptop in
`~/multi-robot-coordination` unless a step says otherwise.

**Rule of the ladder:** each experiment has a GO gate. Pass → move to the next. Fail →
stop, run the collect command, and send me the experiment name + the `collected/` folder.
Do not skip ahead past a failed gate — every later experiment assumes the earlier ones held.

| Day | Experiment | Needs flight? | Time |
|---|---|---|---|
| Any, ideally first | **E6** rotation-sign test | no (disarmed) | 15 min |
| 1 | **E0** session setup → **E1** ghost solo, proven planner | yes | 45 min |
| 2 (optional) | **E2** ghost solo, Lucas's planner (A/B) | yes | 30 min |
| 3 | **E3** ghost + delta | yes | 1 h |
| 4 | **E4** three / four drones | yes | 1.5 h |
| Any, at a site with distance | **E5** LoRa link-margin sweep | no | 1 h per separation |

---

## What is deployed (identical on all four drones)

| Repo | Commit | What it gives you |
|---|---|---|
| `active_exploration` | `6f2c629` | planner + guard: STOP reasons, abandon only on real obstacles, goal blacklist (30 s / 1.2 m), plan budget 3 s, **Lucas's planner selectable** |
| `multi_drone_nvblox` | `994a349` | Lucas's coordination layer, `planner:=` selector, `rotation_check` tool |
| `px4_offboard` | `87c25c5` | VIO bridge with reset_counter on pose jumps |
| `radiohive` | `7e18929` | LoRa bridge with silent-failure detection + heartbeat; firmware v7.3 source |
| `isaac_ros_nvblox` | `7774f42` + local config edit | `nvblox_base.yaml` workspace x −5…40 m so the 35 m geofence is mapped (uncommitted on purpose, identical on all four; thunderstrike also has its own D435i reset fix) |

**No cuVSLAM, nvblox or PX4 source is modified anywhere.** All work is in the
exploration planner, the coordination layer, and our own bridges.

### Fixed on 2026-10-16 — these change what you'll see versus earlier flights
- **Tab 3 (FIS) used a ±10 m box** while the planner used the 35 m box, so FIS only
  produced frontiers within 10 m of takeoff in every tab-launched flight. Now both use the
  same 35 × 35 forward box. Expect exploration to reach well past 10 m.
- **The planner always ran as vehicle 0**, which silently disabled the claim tie-break and
  lane assignment. Now gets its real ID. Verified live: `vehicle_id = 3` on ghost.

---

## E0 — Session setup (start of every flying day, ~10 min)

```bash
./bench_internet_laptop.sh          # bench only: NAT + finds all drones
./fleet_ctl versions all            # MUST end "OK: all N reachable drones run identical software"
```

If `versions` reports a mismatch: stop and tell me. Never fly mismatched software.

Check radios are detected on the drones you'll use (CP2102 found, not the PX4 port):
```bash
./flash_lora_firmware.sh --check ghost delta buckshee thunderstrike
```

---

## E6 — Rotation-sign test (disarmed, no flight, ~15 min)

**Question it answers:** Lucas found that `vio_bridge` (pose to PX4) levels the tilted
camera with R_y(−20°) while `odom_correction` (pose to nvblox) uses R_y(+20°) — a 40°
disagreement, and a strong candidate for the EKF2 z-drift you've seen since June.
This decides which one is right **without changing anything**.

```bash
./launch_fleet_tmux.sh ghost        # in ghost's window: press Enter in tab 1 and tab 2 ONLY
# wait ~1 min for cuVSLAM to start tracking, then in a second terminal:
./experiments/rotation_check.sh ghost
```
Then: pick ghost up, hold it **level**, walk it **straight forward 2–3 m at constant height**,
stop, press **Ctrl-C** in the rotation_check terminal. It prints a verdict:

| Verdict printed | Meaning |
|---|---|
| `nvblox_side LEVEL, px4_side TILTED` | `vio_bridge` sign is wrong (Lucas is right) |
| `px4_side LEVEL, nvblox_side TILTED` | `odom_correction` sign is wrong |
| `RAW cuVSLAM is already level` | cuVSLAM is gravity-aligned; both corrections wrong |
| `INCONCLUSIVE` / `No source is level` | walk further / straighter and rerun |

**Do not change anything based on the verdict.** Fly E1 on the current config (it is the
one that has flown). Send me the transcript — it is saved automatically in
`experiments/results/rotation_check_ghost_*.txt` — and we fix the sign as its own
controlled change with its own validation, never folded into a gate flight.

Tear down: `./launch_fleet_tmux.sh stop ghost`

---

## E1 — Ghost solo, proven planner (THE gate)

**Question:** does the stuck-fix work — guard stops only on real obstacles, refused goals
get blacklisted, and the drone keeps exploring instead of hovering forever?

```bash
./launch_fleet_tmux.sh ghost
```
In ghost's window press Enter in tabs **1 → 7 in order** (tab 8 prints "single drone: no
bridge needed" — that's correct). Wait **3 minutes** (PX4 shows "no valid local position"
during this — normal, it's the uXRCE handshake + EKF2 converging). Then:
```bash
./fleet_ctl preflight ghost         # expect PREFLIGHT PASS: 9/9 (FMU must be powered)
```
Fly: Position → arm → take off → **Offboard**. Aim for 8–10 minutes.

**Watch live:**
- Tab 5: `PLAN OK ... goal=(x,y)` lines keep coming. The tab's first line reads `Exploration Planner [simple]`.
- Tab 5, the fix firing: `Guard blocked (OBSTACLE) > 4s - abandoning goal (x,y),
  blacklisting it for 30s, replanning` → followed by a `PLAN OK` with a **different** goal.
- Tab 2: `VIO discontinuity ... reset_counter` should be rare; it means a cuVSLAM jump was
  absorbed instead of dumping you into Stabilized.

**GO gate (all of these):**
- ≥ 8 minutes of continuous exploration in Offboard
- no hover longer than ~30 s while a goal is active (the old failure was indefinite)
- the drone reaches **more than 10 m** from takeoff (proves the FIS box fix)
- no unexpected drop out of Position/Offboard

**After landing:**
```bash
./launch_fleet_tmux.sh stop ghost
./fleet_ctl collect ghost           # -> collected/ghost/
```

---

## E2 — Ghost solo, Lucas's planner (optional A/B)

Only after E1 passes. Same everything, one flag different — both planners receive
identical geofence, topics, vehicle ID and safety parameters, so the comparison is fair.

```bash
./launch_fleet_tmux.sh ghost --planner=hercules
```
Tab 5's first line reads `Exploration Planner [hercules]`. Same 3-minute wait, same preflight, same flight profile
(start from the same spot facing the same way as E1).

Safety defaults are forced off for this planner: no cruise/velocity feed-forward, no 3D
planning, no simulation-only map-share channel. You are testing its frontier scoring,
allocation and blacklisting only.

**GO gate:** same as E1. **Decision:** compare `collected/ghost/coverage_*.csv` between E1
and E2 (explored area over time). If hercules is clearly better and equally stable, use
`--planner=hercules` in E3/E4; otherwise stay on the default. Skipping E2 is fine.

```bash
./launch_fleet_tmux.sh stop ghost
./fleet_ctl collect ghost
```

---

## E3 — Ghost + delta (first coordinated flight)

**Question:** do two drones divide the area (different goals, no keep-out violation) and
share maps, with coordination staying up for the whole flight?

**Placement and alignment (at the takeoff spot, before launching):** put both drones on a
common start line facing the same direction. Measure from **delta** (the reference) to
ghost, then:
```bash
./experiments/set_alignment.py --ref delta --drone ghost <RANGE_m> <BEARING_deg> 0 0 --push delta ghost
#   BEARING is clockwise from delta's nose: ghost directly RIGHT of delta = 90, LEFT = 270
#   example (ghost 3.5 m to delta's right): --drone ghost 3.5 90 0 0
```
It prints each drone's geofence before pushing — check they describe the same rectangle.
This works with no internet; it copies the file straight to the drones.

```bash
./launch_fleet_tmux.sh ghost delta          # add --planner=hercules only if E2 said so
```
Two windows. In each, Enter tabs **1 → 8** in order. Tab 8 brings up the map-sharing bridge
over the router — both drones must be on the TP-Link WiFi. Wait 3 minutes, then:
```bash
./fleet_ctl preflight ghost delta           # 9/9 on both
```
Fly ghost into Offboard first; once it is exploring, take delta up and switch it to Offboard.

**Watch live:**
- Tab 7 on each: `claim set: (x, y)` — the two drones' claims should point at **different areas**
- Tab 7: `reader alive: ... frames=<climbing>` every few seconds (the radio is genuinely alive)
- Tab 7 peer_map_integrator: `peers { ... ok=<climbing> ...}` (maps being shared)

**GO gate:** both explore ≥ 8 min; goals diverge; drones never within 3 m of each other in
flight; both land clean; no `LoRa link ... reconnecting` storm in tab 7.

```bash
./launch_fleet_tmux.sh stop ghost delta
./fleet_ctl collect ghost delta
```

---

## E4 — Three / four drones

Same procedure as E3, with two extra one-time checks per new drone:

1. **PX4 domain** — in QGC on buckshee confirm `UXRCE_DDS_DOM_ID = 2`; on thunderstrike `= 4`.
   (`fleet_ctl preflight` check 2 fails if this is wrong — that's the symptom.)
2. **Alignment** — every flying drone needs an entry, measured from delta:
```bash
./experiments/set_alignment.py --ref delta \
    --drone ghost <R> <B> 0 0 --drone buckshee <R> <B> 0 0 \
    --push delta ghost buckshee                # add thunderstrike to both lists if flying 4
```
`fleet_ctl preflight` check 3 fails for any drone missing from the yaml, and
`set_alignment.py` refuses to push a set that leaves a flying drone out.

```bash
./launch_fleet_tmux.sh ghost delta buckshee           # or: all
./fleet_ctl preflight ghost delta buckshee
```
Take off one at a time, each into Offboard once the previous is exploring.

**GO gate:** as E3, for every drone, with goals spread across the area.

```bash
./launch_fleet_tmux.sh stop all
./fleet_ctl collect ghost delta buckshee thunderstrike
```

---

## E5 — LoRa link-margin sweep (no flight)

**Gives Lucas the measured channel curve** his Study B is blocked on.

**Important:** must be done **at distance**. On the bench (< 1 m) even the lowest power
(−9 dBm) arrives around −56 dBm — far above the receiver's sensitivity — so every power
level reads delivery 1.0 and the curve is useless. Run it at two or three separations
(for example 30 m, 100 m, 300 m) on stationary drones.

One-time, flash every radio to v7.3 (radios not flashed are detected and excluded):
```bash
./flash_lora_firmware.sh ghost delta buckshee thunderstrike
```
Per separation:
```bash
./launch_fleet_tmux.sh ghost delta          # in each window press Enter in tab 7 ONLY
./experiments/lora_power_sweep.sh ghost delta        # ~9 min: 8 power levels x 60 s
./fleet_ctl collect ghost delta
python3 experiments/analyze_sweep.py experiments/results/sweep_<stamp>
./launch_fleet_tmux.sh stop ghost delta
```
The sweep restores every radio to 10 dBm when it finishes, even on Ctrl-C. Send me (and
Lucas) `experiments/results/sweep_<stamp>/channel_curve_measured.csv` — it is already in
the `rssi_dbm,delivery_ratio` format his simulator takes.

---

## When something goes wrong

| Symptom | Cause | Fix |
|---|---|---|
| Peers expire / coordination goes quiet, everything "running" | stale DDS shared memory | `./launch_fleet_tmux.sh stop <drones>` then relaunch (stop clears it). Check: `ls /dev/shm \| grep -c fastrtps` inside the container |
| "No valid local position" right after launch | uXRCE handshake + EKF2 converging | wait 2–4 min before arming; it clears by itself |
| A tab's process died | Ctrl-C was pressed in it | **never Ctrl-C** — detach with **Ctrl-b d**; rerun the tab |
| `preflight` FAIL "PX4 uXRCE traffic" | FMU unpowered, or wrong `UXRCE_DDS_DOM_ID` | power the FMU; check the domain in QGC |
| `preflight` FAIL "alignment yaml" | drone missing from the yaml | `./experiments/set_alignment.py ... --push ...` |
| Tab 5 stays `INIT ... front=False` | FIS has no viewpoints yet | needs open space ahead; hand-carry won't produce them, flight will |
| Tab 7 prints `LoRa link ... reconnecting` | radio unplugged / USB glitch | it recovers within ~2 s by itself; a continuous storm = check the USB cable |
| `fleet_ctl versions` mismatch | a drone wasn't updated | stop and tell me — never fly mismatched software |
| thunderstrike `git pull` fails "Could not resolve github.com" | its DNS points at the router | `sudo resolvectl dns enP8p1s0 8.8.8.8 1.1.1.1` on thunderstrike |

## What to send me after each experiment

The experiment name, pass/fail against its gate, and the folder from `./fleet_ctl collect`.
Everything I need to diagnose is in there: planner CSVs, guard CSVs, per-packet LoRa logs,
VIO health, coverage curves, and the full console output of every tab.
