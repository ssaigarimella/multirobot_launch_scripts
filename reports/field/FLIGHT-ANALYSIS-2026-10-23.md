# Field day 2026-10-23 — flight analysis and fix plan

Source data: `collected/<drone>/` (planner, guard, swarm, lora_rx, link, vio_health,
keyframes, coverage CSVs + every tab's console log) and the recovered bags
(`experiment_logs/bags/` on each drone). All four drones' logs were power-cut before
the page cache flushed, so every CSV and bag has a zero-filled tail; the last 10–60 s
of each session are missing. Times are laptop local (EDT).

## 1. What flew

| Drone | Offboard window | Length | Max dist from takeoff | Path | Goals reached | Verdict |
|---|---|---|---|---|---|---|
| ghost | 14:20:25–14:26:31+ (log cut) | 6+ min | 15.2 m | 41 m | 8 | explored 2 min, then **PX4 data feed died at 14:22:25** — never moved under the planner again |
| thunderstrike | 14:29:38–14:36:03 | 6.4 min | 29.3 m | 117 m | 35 | **clean**; only time lost to the geofence-edge loop (~30 s) |
| buckshee | 15:29:22–15:31:24 | 2.0 min | 16.0 m | 46 m | 10 | clean, short; VIO jump (2.7 m) at landing |
| delta | 15:29:16–15:30:53 | 1.6 min | 15.0 m | 33 m | 7 | clean, short; last 23 s stuck in the geofence-edge loop |

delta and buckshee were in Offboard **at the same time** 15:29:22–15:30:53 (91 s). That is the
only simultaneous flight of the day. Map sharing over WiFi worked between them; LoRa claims
did not (section 3).

## 2. Per-drone narrative (verify against what you saw)

### ghost (vehicle 3) — 14:18 armed, 14:20:25 Offboard
- 14:20:26 first goal (11.1, 6.0); guard stopped it after 4 s (obstacle at 0.56–0.64 m),
  goal blacklisted, replanned. Then eight goals reached in a row heading forward-left:
  (6,5.7) → (4.3,10.4) → (1.8,10.5) → … → (1.5,3.8) → long leg toward (1.5,16.8).
- 14:22:21 drone at NED (0.83, −15.10), 15 m from takeoff. **From this instant PX4's
  vehicle_local_position, vehicle_status and every other /fmu/out topic stopped updating in
  ROS** — the guard CSV, the planner CSV and the metrics recorder all show the identical
  frozen values for the next 4+ minutes, while cuVSLAM kept tracking normally. The XRCE agent
  printed nothing (no session drop, no re-connect). The Jetson was not stalled (all other
  tabs logged at their normal rates).
- Consequence (corrected 2026-10-26): PX4 **was still in Offboard and still receiving
  setpoints** — only the PX4→Jetson direction died. The planner converts every waypoint to
  NED through an offset it rebuilds on each cuVSLAM tick against the latest PX4 position;
  with that position frozen, every setpoint was anchored to the freeze point. cuVSLAM shows
  the drone moving ~1 m every 2.5 s back and forth around (2.3, 16.4), never reaching a
  waypoint 1–2 m away, from 14:22:25 until you took over at ~14:23:40: the "hovering back and
  forth" you saw. The replan timeout fired three times (14:22:17, 14:22:51, 14:23:22) on top.
  A one-directional failure narrows the cause to the FMU TX → FTDI RX wire or PX4's DDS
  publisher; the ulog will tell which.
- 14:23:40–14:25:10 cuVSLAM shows the drone moving 12 m back toward the launch line
  (your manual flight). It ends at FLU (0.7, 3.8) = 3.8 m to ghost's left = on the launch
  line next to buckshee/delta.
- 14:25:13 onward: the depth guard reports an obstacle at 0.17–0.20 m for 79 s straight
  (it is looking at the parked drones/people), and nvblox marks 1,450 wall cells around the
  drone, so A* could not start ("start_esdf=0.000"). That is the second "stuck". Even with a
  working PX4 feed the planner could not have escaped from there (section 4.3).
- No VIO jumps, 4 cuVSLAM tracking warnings in 15 min. LoRa on ghost was healthy during its
  flight (heard delta, buckshee, thunderstrike; RSSI −27…−89 dBm, 0 reconnects).

### thunderstrike (vehicle 4) — 14:28 armed, 14:29:38 Offboard
- Straight out to x = 35 m (the forward fence), swept right to y = −14.5 (the right fence),
  came back along y ≈ +10, landed at (6, −9). 50 plans, 35 goals reached, 0 A* failures,
  guard STOP only 1 % of the time (36 short episodes, longest 4 s), no VIO events.
- Lost ~30 s in two "Waypoint outside geofence, skipping" loops at (21.8, −14.5) and
  (35.0, −2.9) — goals exactly on the fence (section 4.2).
- LoRa heard delta and buckshee the whole flight (they were on the ground with stacks up).
  No peer map data arrived because nobody else was moving.

### buckshee (vehicle 2) — 15:09 armed, 15:29:22 Offboard
- Explored to its right: (10,−3) → (14,−9) → (17,−7) → (20,−8) → (13,−11) → (9,−13) →
  (6,−15) → (2.5,−14) → (3,−17) → (0,−17). One PATH BLOCKED replan (obstacle 1 m ahead at
  15:31:02). Last 7 s in the geofence-edge loop at goal (0.0, −16.7) (x = 0 is the launch
  line; the planner's 0.3 m margin rejects it).
- 15:31:56, after Offboard ended: VIO discontinuity 2.71 m → EV reset_counter=1 (absorbed by
  the vio_bridge fix; EKF2 re-referenced instead of rejecting). Happened at landing.
- Received delta's keyframes continuously (peer 1: ok 854 → 943). **No LoRa packets at all**
  during the flight (section 3).

### delta (vehicle 1) — 15:28:56 armed, 15:29:16 Offboard
- Explored to its left: (10.4,0.8) → (12.3,5.6) → (11,7.9) → (7.9,10.5) → (8.1,13.5) →
  (6.2,13.5) → (4.3,13.5) → (2.3,14.4). Then 23 s stuck (corrected 2026-10-26 by replaying
  the recorded frontiers through the flown planner): the chosen viewpoint (3.3, 13.5) was
  inside the margin, but its cell was blocked and A* snapped the goal up to 1 m away, to
  (2.3, 14.4) — beyond y_max − 0.3 = 14.2. The 20 Hz loop rejected that waypoint and the
  planner re-made the same plan twice a second until you took over at 15:30:53.
- Guard STOP 0 %, no VIO events. Received buckshee's keyframes (peer 2: ok 112 → 196).
  No LoRa packets during the flight.
- Earlier, 14:50–15:10, delta was armed for 20 min but never entered Offboard (the planner
  logged `armed=True offboard=False`); VIO had a 3.06 m discontinuity at 14:54 (EV reset).
  Four of delta's bringups that afternoon started cuVSLAM 3–170 m off (section 5).

The two drones diverged (delta left, buckshee right) with **no claim exchange**: each saw the
other's map over WiFi, so frontiers in the other's area disappeared. That is the shared-map
path doing the work, not the LoRa claim path.

## 3. Inter-drone communication

| Path | ghost flight 14:20 | thunderstrike flight 14:29 | delta+buckshee 15:29 |
|---|---|---|---|
| LoRa poses (radio mesh) | OK: heard 0,1,3 (5.3 pkt/s each) | OK: heard 0,1 | **DEAD** on both: 0 packets |
| LoRa claims | sent (27 claim sets), nobody flying to receive | sent; nobody flying | not exchanged |
| WiFi keyframes / shared map | sent 27 keyframes; peers on ground, nothing to receive | peers on ground | **OK both ways** (delta→buckshee 89 frames in 2 min, buckshee→delta 84) |
| Alignment | prior (tape) | prior | prior; buckshee reported `src=lc` for delta (loop-closure path engaged) |

What killed LoRa at 15:22 (corrected 2026-10-26; both radios stopped hearing each other in
the same second and never recovered, including through the flight):

- Until 15:22:21 buckshee's radio heard delta's at ~500 pkt/min, and vice-versa — through six
  delta stack restarts between 15:11 and 15:20, each of which closed and reopened delta's
  bridge. **Bridge restarts do not reboot the radio**: measured on the bench 2026-10-26
  (buckshee, 3 open/close cycles: peers stayed alive, no PEER_JOINED, no boot banner). The
  earlier claim that the bridge (or the 14:5x reconnect storm) rebooted radios was wrong.
- **Delta was power-cycled at ~15:22:25** (your message "delta just power cycled" 15:22:36;
  last packet 15:22:21). Its radio rebooted with the Jetson. Delta is LoRa node 0 = the TDMA
  sync master, which never adopts anyone else's clock; it came back at a random phase.
- After the reboot delta's radio answered every 1 Hz status poll (so it was alive, not hung
  in setup) and listed 0 peers for 8+ minutes; buckshee likewise heard nothing from delta.
- Firmware (v7.3 on every radio): a node transmits in its own 40 ms slot of a 190 ms cycle; a
  half-duplex radio cannot hear while it transmits; node 0 never re-syncs and the others
  re-sync only to node 0 heard directly. If node 0's new slot lands on another node's slot,
  they transmit together every cycle and nothing in v7.3 ever moves either of them. With four
  radios the colliding pair also jams the other two, so nobody learns node 0's new phase.
- Measured 2026-10-26: rebooting delta's radio 12× on the bench (v7.3, all four radios on)
  left the mesh broken 15 s later in **9/12**; the event simulation gives 60 % (4 radios),
  20 % (2 radios) and 48 % for a field-day cold start of all four (`tests/lora_tdma_sim.py`).

Fix: firmware v7.4 (section 6). The bridge change planned on 2026-10-23 is dropped: it would
have fixed nothing.

Phantom peers: a radio keeps beaconing the last pose it was fed even when that drone's
stack is down (thunderstrike's swarm table showed delta at 6.6 m age 0.1 s while delta's
stack was stopped). The coordination logic counts every heard radio as a live vehicle for
lane assignment. Only matters for the boustrophedon fallback; keep-out from a parked drone's
real position is correct. Fix in 6.9.

## 4. Root causes (ranked by flight impact)

### 4.0 The planner's 20 Hz control loop ran at 2–5 Hz on every flight (found 2026-10-26)
The planner parses the ESDF slice point by point in Python. Outdoors the slice reached
4.3–8.7 MB per message at ~2 Hz; on the Orin the flown parser takes **1,106 ms** per 8.7 MB
message, so parsing alone needed more than the planner's single thread. Measured from the
planner CSV row spacing during Offboard: ghost 4.4 Hz, thunderstrike 2.3 Hz (92 % of the
time in gaps > 200 ms, max 1.31 s), buckshee 3.7 Hz, delta 4.6 Hz. Setpoints, the
waypoint-reached check and the path-blocked safety check all ran 5–9× slower than designed.
Fixed: vectorized parse, **48 ms** on the Orin, bit-identical map (section 12).

### 4.1 Ghost: PX4 → Jetson data feed died in flight (14:22:25)
All /fmu/out topics froze simultaneously; the agent logged nothing; Jetson healthy; the
vio_bridge kept sending VIO into the agent, and PX4 kept receiving and flying the planner's
setpoints (section 2), so only the PX4 → Jetson direction failed. Candidates: (a) the
TELEM/FTDI cable — it was unplugged/re-plugged on all four drones at ~14:08 ("beeping"),
ghost flew 10 min later; one marginal wire (FMU TX → FTDI RX) gives exactly this signature;
(b) PX4's uXRCE client publisher stalling.
**Needed from you:** pull ghost's PX4 ulog for 14:18–14:27 (QGC → Analyze → Log Download)
and re-seat/inspect ghost's FMU↔Jetson serial cable. Software mitigation (6.5): planner and
guard detect a stale PX4 feed (>1 s), log it loudly and stop publishing setpoints, so PX4's
own offboard-loss failsafe takes over instead of the planner sending setpoints computed from
a frozen position. The GUI shows "PX4 feed age".

### 4.2 Geofence-edge goal loop (all three clean flights)
A* plans on the raw grid; only the 20 Hz loop applies the 0.3 m geofence margin. A path along
the fence, or A*'s 1 m goal snap, can end outside the margin; the loop then logs "Waypoint
outside geofence, skipping", returns to PLAN, and the next plan is the same plan, 2× per
second forever. (Corrected 2026-10-26: on delta the chosen viewpoint was inside the margin;
the snap moved it out.) Cost: delta 23 s (ended its flight), thunderstrike 2×~15 s,
buckshee 7 s. Fix: A* plans on a copy of the map with the margin band blocked, so every plan
is margin-clean by construction (replay: out-of-margin plans delta 5→0, thunderstrike 8→0,
identical goals in all 298 other frames).

### 4.3 No escape when the start cell is in collision / guard blocks repeatedly
Ghost at the launch line: `start_esdf=0.000`, 1,450 wall cells, 71 consecutive NO PATH, 66
guard block/release cycles. The planner has no behaviour for "I am inside an obstacle
according to the map" (back up along the last free path, or climb, or plan from the nearest
free cell). Fix (6.6).

### 4.4 Recording is 40–90 MB/s per drone
86 % of every bag is the two `static_esdf_pointcloud` topics (4.3 MB per message, ~2 Hz each
mapper). 37 GB for thunderstrike's 7 min. Fix (6.7): drop both from the `figures` profile
(occupancy grid + colour mesh + frontier markers + TF + odometry remain; that is what
`replay_fleet.sh` renders). Expected ~0.3 GB/min.

### 4.5 Logs not durable across a power cut
CSV writers flush to the OS every second but never fsync; bags only finalize on SIGINT.
Fix (6.8): fsync every 5 s in metrics_recorder and the planner CSV; `record_fleet.sh` gets a
`--split 60s` style volume split so at most one volume is lost; `collect_bags.sh --repair`
runs the reindex/quarantine procedure that recovered today's bags.

## 5. Why bringup took the whole afternoon

| Failure | Times today | Drones | Root cause | Fix |
|---|---|---|---|---|
| cuVSLAM starts 2–170 m off (position/ESDF slice wrong) | 8 | delta ×5, buckshee ×2, ghost ×1 | bad initialisation on the first frames after the RealSense `initial_reset`; nothing in the log distinguishes a good start from a bad one | bringup verifies pos < 0.5 m and slice Z ≈ camera height 35 s after tab 1, auto-restarts tab 1 (max 3) |
| Camera "NOT found / failed to set power state" | 2 | thunderstrike | the container's `/dev` is a snapshot from container start; a re-enumerated camera (new USB device number) has no node inside the container | `devsync` (mirror host device nodes into the container) runs before tab 1 and again 20 s after; now part of bringup |
| PX4 no XRCE session after cable replug / ESC replug | 3 | buckshee, thunderstrike, delta | FMU side; only a full FMU power-cycle recovered it | bringup detects "agent running, no session after 20 s", restarts tab 2 once, then raises a red alert "power-cycle FMU" instead of waiting |
| LoRa bridge reconnect storm (bridge reopened the port every 2 s; the radio itself did NOT reboot — measured 2026-10-26) | 2 drones all afternoon | all | bridge watchdog treats a quiet radio as dead; firmware only speaks on peer events | `status_poll_hz: 1.0` in coordination_stage (now on all four) |
| LoRa mesh mutually deaf after delta's power cycle | 1 (permanent) | delta↔buckshee | section 3 | firmware v7.4 (phase re-roll + relayed sync) |
| Buckshee TX not arming | 1 | buckshee | RC side (PX4 saw RC valid, throttle −1) — not software | n/a |
| Manual copy/paste of 8 tabs × 4 drones | every bringup | all | no tooling | fleet console (section 7) |

## 6. Fix plan — status 2026-10-26 (all code UNCOMMITTED; deployed to all four drones)

| # | Fix | Where | Status | Evidence (section 12) |
|---|---|---|---|---|
| 1 | **Planner control loop starved** (new, 4.0): vectorized ESDF parse | planner (py) | done, deployed | identical map; 1,106 → 48 ms per 8.7 MB slice on the Orin |
| 2 | **Geofence-edge loop** (4.2): A* on a margin-blocked grid; viewpoints filtered; far out-of-margin waypoint blacklists the goal; DONE when no frontier is inside | planner (py) | done, deployed | replay: out-of-margin plans delta 5→0, thunderstrike 8→0; same goal in 298/298 other frames |
| 3 | **PX4-feed staleness** (4.1): planner `PX4_STALE` (no setpoints, `px4_age` CSV column); guard sends no heartbeat/setpoint after 2 s → PX4 offboard-loss failsafe | planner (py) + guard (C++) | done, deployed; guard rebuilt with the canonical 8 steps (8/8 on all four) | guard on the Orin: heartbeat stops 2.0 s after the feed, resumes on its return |
| 4 | **LoRa firmware v7.4** (3): unheard/deaf phase re-roll + relayed sync; OTA format unchanged | radiohive firmware | done, flashed on all four (hash-verified) | sim + bench, section 12 |
| 5 | LoRa bridge "no reset on reopen" | bridge | **dropped** — reopen does not reboot the radio (measured) | bench probe |
| 6 | `status_poll_hz: 1.0` | coordination launch | done, all four | — |
| 7 | Start-in-collision escape: after 3 NO PATH with the start in collision, A* may snap the start up to 2.5 m; that leg is flown on the depth guard | planner (py) | done, deployed | not exercised by any Friday replay (ghost's case had a dead PX4 feed) |
| 8 | Recording diet; fsync in planner + metrics CSVs | record_fleet, planner, metrics | done | — |
| 8b | Bag split every 60 s; `collect_bags.sh --repair`; guard CSV fsync | record/collect, guard | **not done** | — |
| 9 | Phantom peers: coordination_node flags radio-only peers (bit 7); planner leaves them out of lane assignment, keeps them for keep-out | coordination (Lucas's code) + planner | done, deployed | unit test; 122/122 coordination tests pass; no Friday impact (boustrophedon never ran) |
| 10 | Fleet console GUI + automated bringup (section 7) | console/ | **built**, bench-tested on all four | section 7 |
| 11 | Hardware: ghost FMU↔Jetson serial cable; ghost + delta PX4 ulogs; thunderstrike camera USB | you | pending | — |

Tooling fixed on the way: `flash_lora_firmware.sh` runs esptool inside the container (only ghost's
host user could open the radio port), `build_all_drones.sh` had stale IPs and a monitor that
declared every build FAILED on colcon's harmless "listing git files failed" line.

Open, unexplained: `/coordination/mode_state` was never recorded in any Oct-23 bag although
the planner publishes it every planning cycle with `vehicle_id` > 0 (verified the parameter
reaches the node). No effect on Friday's flights; check the recorder next field day.

## 7. Fleet console — BUILT 2026-10-26

    ./console/fleet_console.py                  # window; tick drones, press BRING UP
    ./tests/console_bringup_headless.py delta buckshee   # same pipeline, text only

One window: a row per drone, a cell per stage (link, USB devices, tabs 1–8, preflight,
recording, READY), green/yellow/red with the reason in words, a READY FOR TAKEOFF banner.
Live status comes from a read-only probe on each drone (`console/probe_host.py` +
`console/probe_ros.py`, pushed and cleaned up automatically; UDP-only DDS so it is immune to
the /dev/shm clean-up). BRING UP runs per drone, in parallel: devsync → tab 2 + PX4 data
(restart once, then "is the flight controller powered? power-cycle it") → tab 1 + camera +
cuVSLAM origin check (3 tries; known camera failures named from the log, e.g. "camera on USB
but will not power up — replug") → tabs 3–7 → LoRa hears every flying radio → tab 8 zenoh
peers → cuVSLAM re-check → preflight → optional recording. Click a cell: restart that tab,
show its log, open its terminal. It never arms, flies, or changes PX4/network settings.

Bench test 2026-10-26 (all four drones, flight controllers unpowered), with no help from me:
delta and ghost green except PX4 ("no data from PX4: is the flight controller powered?" —
correct, all four FMUs sent 0 bytes); buckshee flagged **BAD INIT |p| = 102.7 m** (nvblox
slice at Z = 23.5 m) after passing the 8 s start check — hence the re-check before preflight;
thunderstrike's camera "failed to set power state" three times — the same camera problem as
Friday, needs a physical replug; LoRa: every radio heard the other three; zenoh: every drone
saw all three peers. Status logic: `tests/test_console_eval.py` (12 tests).

## 8. Recovered data

| Bag | State |
|---|---|
| e4/ghost | reindexed, 517 s (14:17:36–14:26:13), last volume quarantined (26 s salvaged separately) |
| e4/thunderstrike | clean, 37 GB, full flight |
| e4/buckshee, e4b/buckshee | ground sessions, reindexed |
| e4c/buckshee | reindexed, 1370 s (15:07–15:30), covers the two-drone flight |
| e4c/delta | reindexed, 212 s (15:27:11–15:30:43), covers the two-drone flight to 10 s before its end |

Collect with `./collect_bags.sh e4 thunderstrike ghost` and `./collect_bags.sh e4c delta
buckshee` over ethernet (ghost and delta are wired now).

## 9. Open questions for you

1. What was ~15 m in front of ghost's start (FLU (1.5, 15))? The guard reported obstacles at
   0.5–0.6 m there at 14:20:40 and 14:23:41 before the feed died — a tree line, fence, people?
2. Ghost: did PX4 show a mode change or "offboard lost" on the RC/QGC at ~14:22:25?
3. Delta 14:50–15:10: armed for 20 min, never Offboard — was that the TX/arming problem, or
   hovering manually?

## 10. Debug strategy (how each root cause gets proven and closed)

Principle: every item below has (a) a hypothesis, (b) a bench test that can
confirm or kill it WITHOUT a flight, (c) the instrumentation that makes the next
flight self-explaining if it recurs, and (d) a pass criterion. Nothing goes on
a drone for the next field day unless it passed (b) on the bench with the real
hardware. Order is by flight impact, and bench items run in parallel.

| # | Fault | Hypothesis (ranked) | Bench test that discriminates | Instrument for the field | Pass |
|---|---|---|---|---|---|
| 1 | ghost PX4 feed death 14:22:25 | H1 FMU↔Jetson FTDI serial cable/connector (all four were re-plugged at 14:08) · H2 PX4 uxrce_dds_client stalled (session alive, no publishes) · H3 Jetson USB-serial driver drop with no re-enumeration | Pull ghost's ulog for 14:18–14:27 (QGC → Analyze → Log Download). In the ulog: `uxrce_dds_client` status topics + `cpuload`; if the client logged "session lost"/timeouts → H2/H3 (link side); if nothing at all → H1 less likely and PX4 itself froze its publishers. Then on the bench: ghost powered, agent running, wiggle/strain-relieve the FMU cable while watching `ros2 topic hz /fmu/out/vehicle_status`; swap the FTDI cable with thunderstrike's and repeat. `dmesg -w` on the host for ttyUSB disconnects during the test | Agent-side watchdog in tab 2: if `/fmu/out/vehicle_status` goes silent > 2 s while the agent process is alive, log loudly, restart the agent, and publish `/fleet/px4_link_down` (consumed by the planner + fleet console). Planner: PX4-feed staleness gate (hold position setpoint, state=`PX4_STALE`, do not re-plan); guard: same gate (C++ rebuild) | No silent gap > 2 s on `/fmu/out/*` over a 30-min bench soak with the cable moved every minute; ulog explains 14:22:25 |
| 2 | Geofence-edge goal loop | Confirmed from CSVs (goal 0.1 m inside, waypoint rejected 2×/s). Fix applied: viewpoints outside the fence filtered during scoring; out-of-fence waypoint blacklists the goal | Replay-driven unit test: feed the planner the recorded frontier sets from delta 15:30:30 (bag `/fis/frontier_*`) and assert no goal is chosen whose first waypoint fails `_in_geofence`; `scripts/test_geofence_loop.py` | splanner CSV already logs `goal_x/goal_y`; add `reject_reason` column | Zero repeated (goal, reject) pairs in a 10-min bench replay; in the field no PLAN↔EXECUTE flip faster than 1 Hz |
| 3 | LoRa mesh mutually deaf after a master reboot | Confirmed from firmware logic + 15:22:11 timestamps. Two causes: bridge reopen resets the radio (DTR/RTS), and node 0's new random slot phase can sit on another node's slot forever | Four radios on the bench, all four bridges up, `rf_stats.sh` counting packets per link. (i) Restart delta's bridge 20 times → with the HUPCL fix the radio must NOT reboot (uptime counter in the status frame keeps counting). (ii) Flash v7.4 (phase re-roll when 0 peers heard for 3 cycles) and power-cycle node 0 20 times → every time all 4 links are back within 2 cycles (< 0.5 s) | `lora_rx` CSV already logs every packet; add `radio_uptime_s` and `sync_src` to the status frame and to the `link` CSV so a reboot or a lost sync is one grep away. Also: `LinkStats.lora_bridge_up` is computed as `bool(peer_table)` (coordination_node.py:421) - it means "heard a peer", NOT "serial bridge up"; add a real `serial_open` field so "radio deaf" and "bridge dead" stop looking identical (the replay shows buckshee's table emptying at 15:22:25, 14 s after the 15:22:11 delta restart, and staying empty through the 15:29 flight) | 20/20 recoveries; 0 radio reboots on bridge restart; four-way delivery ≥ 95 % at 10 m |
| 4 | Phantom peers (ground drones advertised as live peers) | Peers on the ground with the stack up are indistinguishable from flying ones in the packet | Bench: two drones up, one "armed+offboard" simulated → the other must show it as a peer, un-armed drones as `ground` | Put `armed`/`in_air` into the packet flags (bit4) and into `swarm` CSV; planner ignores claims from ground peers | Metrics recorder shows `in_air` consistent with splanner `offboard` on all four |
| 5 | cuVSLAM bad initialisation (×8 on the day) | Cold start with a moving/featureless scene or a stale camera stream; nothing in the log predicts it | Bench loop: start tab 1 50 times with the drone static; record first-10-s |pos| and ESDF slice Z; find the predictor (likely IMU-only start before the first good frame) | Console gate already specified: `|pos| < 0.5 m`, slice Z within ±0.3 m of the camera height, 30 Hz for 10 s, else auto-restart tab 1 | Gate catches 100 % of bad inits in the 50-start loop; false-restart rate < 5 % |
| 6 | Camera / PX4 device nodes missing in the container after a re-plug | `--privileged` snapshot of /dev at container start | Unplug/replug camera and FTDI 10× each with the container up; `devsync` must restore `/dev/video*`, `/dev/media*`, `/dev/ttyUSB*` within 2 s | `devsync` becomes a systemd unit on each drone (host side) with a journal line per node mirrored | 10/10 recoveries without a container restart |
| 7 | Bags 37 GB / 7 min, non-durable logs | Confirmed | Record 5 min with the slimmed `figures` profile on one drone: expect ≤ 6 GB; pull power on a second drone mid-recording and confirm `collect_bags.sh --repair` yields a playable bag | `--max-bag-duration 60` splits, fsync in recorders (done) | Bag ≤ 1.2 GB/min; a power-cut bag loses ≤ 60 s |
| 8 | Delta 14:50–15:10 armed, never Offboard | Unknown: TX/mode switch or operator | Compare delta's ulog `manual_control_setpoint`/`vehicle_status.nav_state` for that window | — | Explained in one line in this report |

Sequencing for the next field day: bench items 3, 5, 6 run this week on the
bench rig (four radios, two drones); items 1 and 8 need the ulogs (ask); items
2, 4, 7 are code + a replay test. The fleet console (section 7) is what turns
the pass criteria above into green cells before takeoff.

## 11. Replaying the recorded flights on the laptop

All Oct-23 bags were slimmed on-drone (`ros2 bag convert`, ESDF clouds dropped,
140 GB → 26 GB) and pulled to `bags/{e4,e4b,e4c}/<drone>/`. Three tools:

    ./viz/bag2timeline.py <run>                 # once: bag + CSVs -> bags/<run>/timeline/<drone>.pkl
    ./viz/fleet_player.py <run> [drones]        # interactive replay (space, arrows, slider)
    ./viz/fleet_player.py <run> --mp4 out.mp4   # real-time video (1x by default, --rate N)
    ./viz/fleet_player.py <run> --frame 14:22:25 --save f.pdf   # paper frame
    ./viz/fleet_player.py <run> --figures reports/visualization/<run>   # figure set
    ./viz/fleet_3d.py <run>                     # 3D voxel reconstruction, HTML, orbit + time slider

See `reports/visualization/REPLAY.md` for what each panel shows and the windows
of the four flights.

## 12. Verification of the 2026-10-26 changes

Everything below was measured, not argued. Harnesses live in `tests/`.

| Change | Test | Before | After |
|---|---|---|---|
| ESDF parse (planner) | `tests/planner_offline/test_planner_offline.py parse` on Friday's largest outdoor slice (8.7 MB, 547k points), both versions' real `_esdf_cb` | 1,106 ms/msg on the Orin (652 ms laptop) | **48 ms/msg** on the Orin (29 ms laptop); raw grid and distance grid **identical** |
| Geofence loop (planner) | `... replay`: every recorded frontier frame of the 4 flights through both versions' real `_run_planning` | plans leaving the margin: delta 5, thunderstrike 8 (3 with the goal itself outside) | **0 and 0**; identical goal in all 298 frames where the flown plan was fine |
| PX4 staleness (guard) | new guard built on thunderstrike, fed VehicleLocalPosition 3 s / silent 4 s / 3 s, heartbeats counted | heartbeat never stops (Friday: ~80 s of Offboard on a frozen feed) | heartbeat **stops 2.0 s** after the feed, status `STOP:PX4_STALE`, resumes immediately when it returns |
| Planner start-up | launched on delta (isolated domain) | — | starts; `px4_stale_s` 2.0, `escape_snap_m` 2.5, `vehicle_id` 1; CSV has `px4_age` |
| LoRa v7.4 (simulation) | `tests/lora_tdma_sim.py`, 1,000 runs each, firmware sync rules reproduced | node-0 reboot: 20 % (2 radios) / 60 % (4) mesh never recovers; cold start of all 4: 48 % | **0 %** in every case; full mesh median 0.2–4.7 s, worst 17.5 s; non-master reboot unchanged |
| LoRa v7.4 (hardware) | `tests/lora_bench_reboot.py`: delta's radio rebooted 12×, all four radios asked who they hear | **9/12** reboots left the mesh broken (v7.3) | **0/12** broken 20 s later (v7.4); the mesh also formed by itself after all four were flashed |
| Bridge reopen | `tests/lora_radio_tool.py` probe on buckshee: open/close ×3 | — | radio did **not** reboot (peers stayed alive, no PEER_JOINED) → bridge change dropped |
| Phantom peers | `multi_drone_nvblox/test` | 121 pass | **122 pass** (new radio-only test) |

What none of this can show: closed-loop flight behaviour. The next flight is the test.

