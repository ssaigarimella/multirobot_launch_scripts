# Replaying recorded flights on the laptop

Written for whoever makes the demo videos and paper figures from the onboard
bags (no drone, no ROS graph needed - only this repo, python3, matplotlib,
plotly, ffmpeg).

## Pipeline

```
drone bags (sqlite3)  --[collect_bags.sh / slim pull]-->  bags/<run>/<drone>/
                      --[viz/bag2timeline.py <run>]----->  bags/<run>/timeline/<drone>.pkl
                      --[viz/fleet_player.py ...]-------->  window / MP4 / PDF frame / figure set
                      --[viz/fleet_3d.py ...]------------>  <run>_3d.html (orbit + time slider)
                      --[viz/fleet_3d_video.py ...]------>  <run>_3d_realtime.mp4 (headless 3D render)
```

`bag2timeline.py` decodes a bag once (pure-python `rosbags`, custom message
types registered from `~/jfr_dev/*/msg/*.msg`) and joins the metrics CSVs from
`collected/<drone>/` that overlap the bag in time. The pickle holds, per drone:
odometry, the `map->odom` correction, the alignment prior `T_world_self`,
own + shared occupancy slices (1 Hz, zlib), the union of nvblox colour voxels
with first-seen times, frontiers, planner goal/path/geofence, planner and
guard state changes, claim intents, what the drone believed about its peers
(`/swarm/peers`), per-link LoRa stats, and the CSV rows (planner, guard,
lora_rx, vio_health, ...).

Everything is placed in ONE team frame with the alignment the fleet actually
flew with (`p_world = T_world_map · T_map_odom · p_odom`), so multi-drone
views are what the coordination layer itself believed.

## The replay window (fleet_player.py)

| Panel | Shows |
|---|---|
| Map (left) | Each drone's own occupancy slice tinted in its colour (light = free, dark = occupied) so "who mapped what" is visible; trajectory trail; pose glyph with heading (solid while in Offboard, faded otherwise); planner goal ★ and path; frontier centroids ×; claim (dashed); geofence (dash-dot). **Hollow dotted rings** = where a drone believes a peer is from the last LoRa packet, dotted to the peer's true position - a stale/missing belief is visible at once. **Link lines** between drones: green ≥ 70 % delivery, amber 30–70 %, red < 30 % or stale > 10 s, grey dotted = no LoRa at all. |
| Tiles (top right) | Per drone: OFFBOARD / ARMED / on ground, planner state and goal, guard state, speed, altitude, distance flown, VIO rate, LoRa bridge, packets received in the last 10 s, per-peer delivery, peers believed and their age. |
| Timeline | Offboard bars, guard STOP ticks (above), LoRa packets received (below), claims (dots); cursor = current time. |
| Coverage | Mapped area per drone and the team union (rasterised at 0.25 m). |

Keys in the interactive window: `space` play/pause · `←/→` 1 s · `[ ]` 10 s ·
`+/-` rate · `s` save PNG · drag the slider to scrub.

## Commands for the 2026-10-23 flights

```
./viz/bag2timeline.py e4;  ./viz/bag2timeline.py e4c;  ./viz/bag2timeline.py e4b   # ~10 min total, once

# ghost: explore -> PX4 feed death at 14:22:25 -> manual reposition -> stuck
./viz/fleet_player.py e4 ghost --from 14:20:15 --to 14:26:16
./viz/fleet_player.py e4 ghost --from 14:20:15 --to 14:26:16 --mp4 reports/visualization/e4_ghost.mp4

# thunderstrike: the clean 29 m exploration
./viz/fleet_player.py e4 thunderstrike --from 14:29:30 --to 14:36:10

# both e4 flights with buckshee on the ground listening (peer beliefs, links)
./viz/fleet_player.py e4 --from 14:20:00 --to 14:37:00 --rate 4 --mp4 reports/visualization/e4_fleet_x4.mp4

# delta + buckshee: the only simultaneous Offboard flight (15:29:16-15:31:24)
./viz/fleet_player.py e4c --from 15:29:00 --to 15:31:40
./viz/fleet_player.py e4c --from 15:29:00 --to 15:31:40 --mp4 reports/visualization/e4c_pair.mp4

# single paper frame and the static figure set
./viz/fleet_player.py e4 ghost --frame 14:22:25 --save reports/visualization/e4/ghost_feed_death.pdf
./viz/fleet_player.py e4c --figures reports/visualization/e4c
./viz/fleet_player.py e4 --from 14:20:00 --to 14:37:00 --figures reports/visualization/e4   # window of the committed e4 set

# 3D (WebGL page: drag orbit, shift-drag pan, wheel zoom, space play, 1-4 follow a drone)
./viz/fleet_3d.py e4c --from 15:29:00 --to 15:31:40 --out reports/visualization/e4c/e4c_3d.html
./viz/fleet_3d.py e4 ghost thunderstrike --from 14:20:00 --to 14:37:00 --out reports/visualization/e4/e4_3d.html
#   open ...html#t=118        -> starts at T+118 s;   #t=0&play -> autoplays (for screen recording)

# 3D MP4, rendered headless (no screen recording): real time, 1920x1080, 30 fps, H.264 CRF 20
# e4c: 2 min 40 s of video, ~140 MB, ~8 min to render with --jobs 2
./viz/fleet_3d_video.py e4c --from 15:29:00 --to 15:31:40 \
    --title 'Two-drone coordinated exploration · delta + buckshee' --still 15:30:50 \
    --jobs 2 --out reports/visualization/e4c/e4c_3d_realtime.mp4
# e4: 17 min of video, ~1.1 GB, ~40 min to render with --jobs 3
./viz/fleet_3d_video.py e4 ghost thunderstrike --from 14:20:00 --to 14:37:00 \
    --title 'Solo exploration flights · ghost, then thunderstrike' --still 14:33:00 \
    --jobs 3 --out reports/visualization/e4/e4_3d_realtime.mp4
#   preview a few frames first:  --stills-only +45,+80,+115   (PNGs next to --out, no video)
#   --jobs N renders N frame ranges in parallel (one headless Chrome each) and joins them losslessly
#   each run also writes <out>_still.png at --still (default: the widest moment of the clip)
```

### How the 3D MP4 is made (`viz/fleet_3d_video.py`)

- Same data as `fleet_3d.py`: the colour voxels appear at their first-seen time.
- Positions are the cuVSLAM odometry levelled into the mapper's frame
  (`fleet_player.level_odom`, R_y(20°), or the angle `bag2timeline.py` measured in
  the bag). Raw odometry is in the camera's pitched start frame: its z climbs with
  forward distance (13 m on thunderstrike) and its x/y drift up to 2.3 m off the
  map. The 2D replay and the 3D page use the same levelled track.
- Altitude comes from PX4's local position (planner CSV `ned_z`), for height above
  the ground. Do not use PX4's x/y for distances: PX4's forward axis reads about
  0.79 of the camera track (field report section 4.6).
- A capture-only three.js page draws exactly one frame per `renderAt(f)` call,
  so every frame is a pure function of its index: the same frame renders
  byte-identically twice, and parallel jobs join without seams.
- Headless Chrome runs over its DevTools pipe (standard library only), with WebGL
  on the laptop GPU through ANGLE/EGL. The log line `WebGL: ...` shows which
  renderer it got. SwiftShader there means software rendering, several times slower.
- The camera orbits slowly and backs off just far enough to keep the drones,
  their labels, their whole trail so far and the map so far inside the frame.
  `--frame-trail-s 30` keeps only the last 30 s of trail in frame, for a closer shot.
- Ghost's PX4 feed froze at 14:22:21. From then on its planner log repeats the
  last PX4 values. The renderer detects this: values that stay identical while
  the drone moves. Ghost's tile then reads PX4 FEED LOST with altitude `?`, and
  its trail is held at the last altitude PX4 reported.
- Needs `google-chrome` (or chromium) and `ffmpeg` with libx264. Run it as
  `./viz/...`, not `python3 -s`: the timeline pickles need the user-site NumPy 2.

Options worth knowing: `--map shared` draws what each drone's shared mapper
held (own + received) instead of its own slice; `--rot 0|90` forces the
display orientation (`auto` puts the team's forward axis up when the flown
area is deeper than wide); `--trail 60` limits the trail to the last 60 s.

## Static figure set (`--figures DIR`)

`team_map.pdf` (final merged map + trajectories), `coverage.pdf` (mapped area
vs time, Offboard shaded), `comms.pdf` (per-drone LoRa delivery ratio per peer
with received packets), `paths_by_state.pdf` (per-drone path coloured by
planner state, guard stops ringed), `events.csv` (every mode/planner/guard/
claim event in the window, for captions).

## Caveats

* Cross-drone time alignment is bounded by NTP skew (±25–50 ms). Fine for
  figures; do not claim per-message synchrony.
* The occupancy slice is the planner's 2D view; the 3D voxels are the union
  of the incremental colour-layer markers and are coloured with the camera
  colour nvblox recorded. buckshee's colour layer was published far less
  often than the others' on 2026-10-23 (14.5k voxels in 23 min vs delta's
  249k in 3.5 min), so its 3D map is thin; its 2D slice is complete.
* `team_map` is drawn at the instant of maximum team coverage inside the
  Offboard span (printed in the title), because the slice shrinks around
  landing (delta lost 30 % of its own slice at 15:30:57, 4 s after Offboard
  ended) and the landing VIO jump smears the final one.
* Distance flown in the figures is integrated on a 2 Hz-decimated VIO track;
  the raw 30 Hz track over-counts by ~30 %, and the flight report's numbers
  come from the planner CSV, so expect ±15 % between the two.
* `e4` buckshee and `e4b` are ground/manual periods (no Offboard); they are
  useful only for the communication picture.
* RViz replay (`replay_fleet.sh`) still works on the slim bags for the native
  3D mesh look; it needs the custom message packages, which this path does not.
