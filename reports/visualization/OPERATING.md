# Operator guide: record → collect → replay → export

Companion to [`VISUALIZATION.md`](VISUALIZATION.md), which carries the measured
numbers and the reasoning. This file is the short version you run from.

All commands are run from `/home/ssg/multi-robot-coordination` on the laptop.

---

## 1. Record (during a normal flight)

Bring the stack up exactly as usual — recording is additive and changes nothing:

```bash
./launch_fleet_tmux.sh ghost delta          # tabs 1-8, Enter-gated, as always
```

Then, once the tabs are up and `./fleet_ctl preflight ghost delta` is clean:

```bash
NOTE="field run 3, 35x35 geofence" TRANSPORT=bench \
  ./record_fleet.sh start <run-name> figures ghost delta
```

- `<run-name>` is yours, e.g. `field_20260828_run3`. It becomes the directory name.
- profile is `figures` (default), `diagnostic`, or `full`.
  **`full` is bench only** — ~5.15 GiB/min/drone; it will not keep up in flight.
- `NOTE` and `TRANSPORT` (`bench`/`mesh`) go into the run manifest.

It refuses to start if free space is under 20 GB, if a recording is already
running, or if none of the profile topics exist.

### Prove it is actually recording

```bash
./record_fleet.sh status <run-name> ghost delta
```

```
ghost    tmux=yes proc=1  files=1   size=14M    HEALTHY - grew 3.9M in 5 s
delta    tmux=yes proc=1  files=1   size=52M    HEALTHY - grew 20M in 5 s
```

`status` samples the byte count twice 5 s apart — **a file existing is not
health.** Anything other than `HEALTHY` means the bag is not usable:

| verdict | meaning |
|---|---|
| `NOT RUNNING` | no `bagrec` session — it never started |
| `SESSION UP but NO 'bag record' PROCESS` | the recorder died; check `experiment_logs/pane_bagrec_*.log` |
| `BROKEN - bag dir is EMPTY` | started but wrote nothing (classically: a path that does not exist inside the container) |
| `STALLED - N on disk but NO GROWTH` | header written, data stopped |

Run it once right after starting, and once before you stop. Expect roughly
**250 MiB/min per drone** for `figures`; a drone in a rich scene produces several
times more than one facing a blank wall.

### Stop — always before tearing the stack down

```bash
./record_fleet.sh stop <run-name> ghost delta
```

This SIGINTs `ros2 bag record` so `metadata.yaml` gets written, *then* kills the
session, then prints `ros2 bag info`.

> **The recorder's tmux session is `bagrec`, which `fleet_ctl stop` does NOT
> sweep** (that only touches `^(fleet|zenoh|rviz|loratest)`). This is deliberate:
> a session kill would leave a bag with no `metadata.yaml`, which neither
> `ros2 bag info` nor `ros2 bag play` can open. **Stop the recording first.**

---

## 2. Collect to the laptop

```bash
./collect_bags.sh <run-name> ghost delta
```

rsync with resume, then verifies the laptop copy with `ros2 bag info`:

```
  delta: OK - 19934 msgs, 97.625415738s, 402M
  ghost: OK - 20336 msgs, 102.299837356s, 73M
```

Lands in `bags/<run-name>/`. The bag payload is gitignored; the
`<drone>.manifest.json` and `COLLECTION.md` are tracked, and pin every source
repo's git SHA, the topic list and the UTC window.

**The drone copies are never deleted automatically.** Delete them by hand only
after the verification above passes.

Other forms: `./collect_bags.sh --list ghost delta`,
`./collect_bags.sh --verify <run-name>`.

Note `./fleet_ctl collect` is still the right tool for the CSVs
(`lora_rx_*`, `link_*`, `swarm_*`, `vio_health_*`, `keyframes_*`, `coverage_*`) —
run both.

---

## 3. Replay

```bash
./replay_fleet.sh <run-name> ghost delta --rviz
```

Options: `--rate R`, `--start-offset S`, `--loop`, `--paused` (SPACE resumes,
→ steps), `--rviz [config]`, `--no-align`.

Bags are automatically time-aligned to their latest common start (they are
typically ~10 s apart because you start them in sequence). Cross-drone residual
is **±25–50 ms** — enough to merge trajectories and maps, not enough to claim
per-message synchrony.

### Check the merged frame tree

```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_tools view_frames             # expect ONE tree rooted at 'world'
ros2 run tf2_ros tf2_echo d1/map d3/map    # expect [0.000, -3.500, 0.000]
```

That translation is the physical drone separation from `swarm_alignment.yaml`.
If it is wrong, the yaml did not match the field placement — fix the yaml, the
bags are fine.

### Expected, harmless replay messages

- `Ignoring a topic '/swarm/peers', reason: package 'multi_drone_nvblox' not
  found` — the laptop has no custom message packages. Those six coordination
  topics are recorded but cannot be shown here. Everything visual still works.
- `Message Filter dropping message … timestamp … earlier than all the data in
  the transform cache` — the `--loop` wraparound. Harmless.

---

## 4. Export a figure

```bash
./capture_figure.sh reports/visualization/fig_fleet.png viz/fleet_overview.rviz --crop
```

**Must be run from the laptop's own desktop session.** Headless does not work
here — `Xvfb` is not installed. RViz also needs `QT_QPA_PLATFORM=xcb` (the script
sets it); without it RViz dies with `Invalid parentWindowHandle`.

Output is **1920 × 937**, or **1358 × 854** with `--crop`. That is the window
size — there is no supersampling and no `--resolution` flag in RViz2. For a
larger figure, enlarge the RViz window first. Upscaling afterwards adds no
detail.

The camera pose is saved in the `.rviz`, so the viewpoint is reproducible across
sessions. If you re-frame interactively and want to keep it, edit the defaults in
`viz/make_rviz_configs.py` and re-run it rather than hand-editing the `.rviz`.

---

## 5. Quick reference

```bash
# record
NOTE="run 3" ./record_fleet.sh start field_run3 figures ghost delta
./record_fleet.sh status field_run3 ghost delta      # must say HEALTHY
./record_fleet.sh stop   field_run3 ghost delta      # BEFORE fleet_ctl stop

# collect
./collect_bags.sh field_run3 ghost delta
./fleet_ctl collect ghost delta                      # the CSVs too

# replay + figure
./replay_fleet.sh field_run3 ghost delta --rviz
./capture_figure.sh reports/visualization/fig.png viz/fleet_overview.rviz --crop
```

### Files

| path | what |
|---|---|
| `record_fleet.sh` | per-drone recording, 3 profiles, start/status/stop/list |
| `collect_bags.sh` | resumable fetch + `ros2 bag info` verification |
| `replay_fleet.sh` | time-aligned two-drone replay |
| `capture_figure.sh` | RViz2 → PNG |
| `viz/fleet_frame_prefix.py` | per-drone frame prefixing (the merge) |
| `viz/make_rviz_configs.py` | regenerates both `.rviz` files |
| `viz/fleet_overview.rviz` | **the paper figure** — both drones |
| `viz/single_drone_detail.rviz` | one drone, mapping + planning internals |
