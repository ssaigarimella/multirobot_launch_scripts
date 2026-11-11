# JFR paper, round 3 (2026-11-11)

From the lab agent. Branch `jfr-field-corrections` in the paper repo now has 16 commits on
`main` @ `52adf36` (6 new this round), not pushed. Four-pass compile: all exit 0, 32 pages,
one warning that predates these edits (`zhang2024gvp` undefined). Style check on every added
line: 0 findings on 180 lines. Line numbers below are at the branch head, `6fa0a41`.

## Not done, and why

- **Item 4 (ESC) is not made.** The instruction reads "the vehicles carry [Cyclone 35A /
  LANNRC 45A]" with the choice left open. Sandilya has to say which. The draft still says
  LANNRC 45A at lines 543 and 770. No record on the laptop names the ESC firmware.
- **Lucas's STUDY-B-V2-REPORT is not on this laptop.** Only his first report is
  (`Downloads/study-results-2026-10-23...zip`, `SIM-STUDY-REPORT.md`, sections 3.6.1 to
  3.6.3). The firmware comparison below is against those three findings and the three
  behaviors as named in the instruction, not against the V2 text.

## Item 2a. v7.5 as written against the three target behaviors

Firmware file: `radiohive/firmware/lora_mesh_v7_2_ros2.ino`, working copy (v7.5,
uncommitted, not flashed). Not edited this round.

| Target behavior | In v7.5? | What v7.5 does instead | Where it would go |
|---|---|---|---|
| Listen before the first transmission at boot | **Missing** | Boots straight onto its own clock and transmits from the first cycle. An overlap is resolved afterwards by random listen-only slots (line 529) and a timing shift when no peer reports hearing the radio (line 568). | `setup()` lines 497-498 (`cycle_epoch = millis(); synced = true;`): start unsynchronized with a randomized listen window. `loop()` line 524 (`if (synced)`): join the schedule when the window ends or a frame is adopted. `handleRx()` lines 683-700: during the window adopt the first frame heard whatever its identifier, so a rebooted lowest-ID radio rejoins the team's frame. |
| Expiry of a departed reference's identifier, re-election of the next smallest | **Missing** | Survivors keep the departed identifier for ever. `findNewSyncSource()` (lines 432-443) takes the minimum over live peers' identifiers and the reference each last advertised (stored at line 717), with no freshness test, so the stale value circulates. Survivors stay together only because each accepts timing relayed by the others (lines 450-453). | Needs a freshness value for the advertised reference in the packet. `Packet` (lines 137-154) has no spare field, so this changes the over-the-air format (or packs into the upper bits of `sync_source`); either way all four radios are flashed together. Then: store it at line 717; ignore expired references in `findNewSyncSource()` (432-443); on timeout of the own reference (lines 586-592) fall back and re-elect; let `shouldSyncTo()` (445-455) adopt a larger identifier once the current one has expired. |
| Periodic re-correction of multi-hop followers | **Partial** | A follower that cannot hear its reference accepts a correction from any peer advertising the same reference, on every packet (lines 450-453). There is no ordering by distance from the reference, so a follower can be corrected by a radio farther from the reference than itself, and each adoption adds the positive per-hop error Lucas measured (2 to 3 ms per hop). | `shouldSyncTo()` lines 445-455, with a hop count carried in the packet (same format change as above), so a follower accepts only from a radio nearer the reference. Also line 686: the airtime compensation uses the base packet airtime even when the received packet was longer, which is the fixed-airtime shift in Lucas's section 3.6.2. |

Lab simulator, reference-departure scenario (new, 500 runs each, full and chain topology):
survivors kept a common schedule in 500 of 500 runs in every firmware version and all kept
the departed identifier. The simulator's clocks are ideal, so it cannot show drift.

**Two cautions about the target as specified.**
1. The three behaviors do not cover two groups that are already running with overlapping
   slots (the split-chain case: 96 % stuck with v7.4, 0 % with v7.5's listen-only slots), and
   listen-before-transmit only separates radios that power up together if its window is
   randomized. I recommend the target firmware keep v7.5's two mechanisms as well. The draft
   text no longer mentions them, as instructed.
2. "Drift bounded only by crystal skew" is exact for v7.3, where survivors free-run. In v7.4
   and v7.5 the survivors correct one another, so they stay together but nothing anchors
   them to a live reference. The lesson says the simulator's clocks are ideal and that a team
   with no live reference stays aligned only as long as crystal skew allows.

## Item 6. Mesh link mentions, classified

(a) = description of the design, unchanged. (b) = claim or promise that it was flown or
measured; a `\todo` was added and nothing else changed.

| Line | Where | Class |
|---|---|---|
| 100 | Abstract, "placed in mesh mode through a kernel modification, carries bulk map data" | a |
| 153 | Introduction, "modified at the kernel level to operate as a peer-to-peer mesh" | a |
| 185 | Contribution 2, "a WiFi mesh link, enabled by a kernel modification" | a |
| 189-190 | Contribution 2, "report both links in flight" | **b**, todo at 190 |
| 292 | Related work, "placed in mesh mode ... requiring no second radio" | a |
| 378 | Comparison table (and its commented copy at 325) | a |
| 777 | Table 4, "Onboard 802.11 WiFi adapter in mesh mode" | a |
| 856 | Figure 3 caption, "the WiFi mesh carries submaps and keyframes" | a |
| 1082-1084 | Figure 5 placeholder text | a |
| 1184-1198 | Section 7.3, kernel changes, "join a single named mesh on channel 6 at 2437 MHz", "Map exchange runs across the mesh" | a |
| 1205-1211 | Section 7.3, loss of the link "which Section 11.3 quantifies in flight" | **b**, todo at 1210 |
| 1469-1472 | Section 9.3, "the mesh link's throughput measured between two vehicles" | **b**, todo at 1472 |
| 1512-1520 | Table 9 header and rows | table, cells `[TBD]`, unchanged |
| 1690-1692 | Section 11.2, "disabling the mesh interface in flight ... beyond mesh range" | **b**, todo at 1691 |
| 1792 | Table 15 header "Mesh link" | table; the caption already says the WiFi link ran through an access point |
| 1856-1861 | Lesson, "operated as a mesh ... What made the pair sufficient" | **b**, todo at 1860 |
| 1936-1938 | Limitation: the campaign has used an access point | true as written, unchanged |
| 1966 | Conclusion, "placed in mesh mode through a kernel modification" | a |

## Notes on the other items

- **Item 1.** One sentence at lines 1475-1479. The 99.7 to 100 % figures are time-means of a
  cumulative claim-packet delivery ratio; the raw files are still on the vehicles.
- **Item 2b.** Section 7.2 (lines 1145-1160, 1164-1173), Section 10.2 (1591-1602), lesson
  (1870-1883). The convergence argument now rests on a plain hearing assumption and no
  longer names a mechanism that provides it.
- **Item 3.** Added to `prompts_for_go.md` as step 1. There is no single fleet parameter
  file; the step saves one export per vehicle in the lab repo's `px4_params/`.
- **Item 5.** Lines 850-851, 1027, 1568. `figures/Figure_3.pdf` itself was not checked and
  may still draw the fused pose going to mapping.
- **Item 7.** Lines 1061 and 1065-1067. Still open, outside the named scope: Section 4.3,
  lines 697-698, says "the color stream provides the images used for inter-robot place
  recognition".
- **Item 8.** Lines 630-637 (caption), 651, 656, 657. Things to know about log_71: its date
  is unknown (file dated 2026-06-10); 234 of its 347 s in the air are autonomous flight, so
  it is not a pure hover; the vehicle's mass on that flight is not recorded; the
  thrust-to-weight of 2.5 is the reciprocal of the 0.40 hover thrust and assumes thrust is
  linear in the command. The total current (9.77 A at 14.45 V) is available; that cell was
  left `[TBD]` as instructed.

## Deliverables (lab repo, `reports/paper/audit_round2/`)

`ROUND2_AUDIT_REPORT.md`, `ROUND3_REPORT.md`, `prompts_for_go.md`,
`table09_coexistence_candidates.csv`, `table15_inflight_comms.csv`,
`jfr-field-corrections.diff` (`git diff main..jfr-field-corrections`), plus the other CSVs
from round 2.

## Final text of every passage changed this round

### `fb71152` Section 9.3: bench coordination-link delivery over the access point

Line 1466 (at that commit):

```latex
arrangement is acceptable. In a bench run of 54\,min with four vehicles, the
coordination link delivered 99.7 to 100\% of claim packets on each of the 12
directed links while the WiFi adapters were associated to an access point at
5765\,MHz, which is not the mesh configuration of
Table~\ref{tab:coexistence}.
```

### `6984c2b` LoRa schedule: describe the target firmware behavior

Line 1144 (at that commit):

```latex
them. In an event simulation of the preceding firmware revision, which had no
provision for this case, four radios failed to reach a common frame in 39\% of
cold starts and in 60\% of restarts of the reference vehicle. Three behaviors
address it. A vehicle listens before it first transmits, so that a vehicle
joining or rejoining a running team adopts the team's frame instead of
transmitting over it. A reference identifier that its owner no longer refreshes
expires from the state the vehicles exchange, so that when the reference vehicle
departs the remaining vehicles re-elect the next smallest identifier instead of
holding a stale one. A vehicle more than one hop from the reference is
re-corrected periodically by the neighbor through which it follows the
reference, so that frame error does not accumulate along a multi-hop path.
\todo{This paragraph, the convergence argument below,
Section~\ref{sec:sim_tdma}, and the corresponding lesson describe the target
firmware and are contingent on the firmware flashed for the campaign.}
```

Line 1161 (at that commit):

```latex
and stable, and call a synchronization round an interval in which every vehicle
hears each of its neighbors at least once. Then within $D$ rounds, where $D$ is
the graph diameter, every vehicle adopts reference identifier $m$ and aligns to
a single common frame. Vehicle $m$ never adopts a
```

Line 1168 (at that commit):

```latex
rounds. With no vehicle departing, no identifier expires, a vehicle changes its
reference only to a strictly smaller value, and the process does not
oscillate. Once
```

Line 1589 (at that commit):

```latex
reach a common frame from power-up and after a restart of the reference vehicle,
the time to re-elect a reference after the reference vehicle departs, the frame
error of vehicles more than one hop from the reference, the number of rounds to
converge against the graph diameter, and the recovery after a partition
re-merges.
```

Line 1595 (at that commit):

```latex
and after a restart of the reference vehicle, re-election time after its
departure, multi-hop frame error, convergence rounds against graph diameter, and
re-merge recovery time.}
```

Line 1862 (at that commit):

```latex
\dg{A collision-free schedule still has to be reached, and kept.} Disjoint
slots hold only after the team shares a frame, and half-duplex radios whose
slots overlap cannot hear the packets that would align them. In an event
simulation of the preceding firmware revision, four radios failed to reach a
common frame in 39\% of cold starts and in 60\% of restarts of the reference
vehicle. When the reference vehicle departed in the same simulation, the
remaining vehicles kept a common schedule in 500 of 500 runs but went on
advertising the departed identifier. The simulator's clocks are ideal; on
hardware a team with no live reference stays aligned only as long as crystal
skew allows. Listening before the first transmission, expiry of a departed
reference, and periodic re-correction of multi-hop followers were specified in
response. A time-division link without a coordinator should be tested from
power-up, through a mid-mission restart, and through loss of the reference
vehicle, not only in steady state.
```

### `1f4acd4` Mapping frame: remaining fused-pose statements

Line 846 (at that commit):

```latex
\texttt{odom\_correction} levels the cuVSLAM odometry for the camera tilt and
republishes it as the frame mapping uses.
```

Line 1023 (at that commit):

```latex
480 at 30\,Hz, and the leveled cuVSLAM odometry to maintain a truncated signed-distance
```

Line 1564 (at that commit):

```latex
Map from post-fusion pose (planned for the campaign) & \PH & \PH & \PH\\
```

### `efb63e5` Mesh link: mark the claims that it was flown or measured

Line 190 (at that commit):

```latex
\todo{The mesh link has not been flown; flights to date used an access point.}
```

Line 1209 (at that commit):

```latex
against the fully connected case. \todo{Not yet flown; the coordination-only
fallback has no flight data.}
```

Line 1471 (at that commit):

```latex
\todo{Mesh throughput has not been measured under load.}
```

Line 1690 (at that commit):

```latex
the team beyond mesh range. \todo{Not yet flown; no flight has used the mesh
interface.} These quantify how much coverage rate and map
```

Line 1859 (at that commit):

```latex
\todo{The mesh has not been operated in flight; this lesson rests on bench
use.}
```

### `3fa1e38` Place recognition: infrared input, screening thresholds, vocabulary size

Line 1058 (at that commit):

```latex
the infrared pair at a rate well below the mapping rate, which keeps it clear of
```

Line 1062 (at that commit):

```latex
per keyframe. The vocabulary holds 4089 visual words, and pairwise-consistency
screening is enabled with thresholds of 1.0\,m in translation and
$15^{\circ}$ in rotation.
```

### `6fa0a41` Table 3: hover thrust, thrust-to-weight and total hover power from log_71

Line 633 (at that commit):

```latex
from the autopilot's hover thrust estimate in flight. The hover thrust, the
thrust-to-weight ratio, and the total power are medians over the in-flight
samples of one flight log in which the logger dropped about half the samples;
the thrust-to-weight ratio is the reciprocal of the hover thrust. Remaining
values to be filled from measurement.}
```

Line 651 (at that commit):

```latex
Total at hover with perception active & \PH & 141\\
```

Line 656 (at that commit):

```latex
Hover thrust, normalized & 0.40 & \\
Thrust-to-weight at takeoff mass & 2.5 & \\
```
