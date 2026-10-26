# Positive control procedure — control-link degradation, v2

**Status: PROCEDURE ONLY. This was written, not executed.**
**Revised 2026-08-21** after the PX4 v1.15.4 source trace settled hypothesis A from source.

## What changed and why

Version 1 of this procedure treated Step 0 as the experiment that would decide between:

- **A.** `rc_lost_frame_count` is never populated on this code path.
- **B.** It is populated, but nothing in the corpus ever degraded the link enough to move it.

**The source read has already settled this: A is true, provably.** In PX4 v1.15.4,
`PX4IO_P_RAW_LOST_FRAME_COUNT` is initialised to `0` at
`src/modules/px4iofirmware/registers.c:108`, is **never written anywhere in the PX4IO
firmware**, and is copied verbatim into `input_rc.rc_lost_frame_count` at
`src/drivers/px4io/px4io.cpp:1094`. There is no code path by which it can become nonzero
while RC arrives via PX4IO.

Step 0 is therefore no longer a discovery experiment. It is a **falsification test**: the
source makes a hard prediction, and one nonzero reading anywhere would overturn it. That is
worth 15 minutes of bench time, because the alternative is trusting a source read against a
binary whose provenance is only asserted by `INFO [px4io] IO FW CRC match`.

The graded ladder below is retained, but its purpose has changed. It no longer calibrates
`rc_lost_frame_count` — that field cannot move. It characterises **`rc_failsafe` duty cycle**,
which the source trace confirms is the only field in `input_rc` that both responds to RF
conditions and survives to the log.

## Safety and reversibility preconditions

- **Propellers physically removed** from all four arms, stored off the bench.
- **Vehicle disarmed** for the whole run. Do not arm at any point. Arming state is recorded in
  the log, so a violation is detectable afterwards.
- Vehicle mechanically restrained so an unexpected output cannot move it.
- Stable battery or bench supply, so brownout is not mistaken for link loss.
- **No firmware change, no radio configuration change.** Do not alter transmitter output
  power, model settings, channel map, protocol, region, or bind. Do not set any PX4 parameter.
  Do not flash anything, FMU or IO. Every condition below degrades the RF path *physically*
  and is undone by removing the obstruction.
- Log the whole run as one continuous ULog so the counters do not reset mid-run.

## Marking condition boundaries inside the log

At each condition change, toggle an **unused RC switch** three times. This moves an existing
channel value only — not a configuration change — and stamps an unambiguous boundary in
`input_rc.values[]` that segment timestamps can be aligned against. Note the wall time in a
bench notebook as a backup.

## What is observable, and what is not

Only four `input_rc` fields carry link state, and the source trace fixes what each can do:

| Field | Source | Can it move? |
|---|---|---|
| `rc_total_frame_count` | `controls.cpp:373` increments per accepted SBUS frame | Yes — the denominator |
| `rc_lost_frame_count` | register never written; `registers.c:108` init 0 | **No. Structurally 0.** |
| `rc_failsafe` | SBUS flags bit 3 → `controls.cpp:271` → `px4io.cpp:1092` | Yes — binary, ~2 Hz in log |
| `rc_lost` | `PX4IO_P_RAW_RC_FLAGS_RC_OK` inverted, 200 ms timeout | Yes — binary, coarse |

The SBUS **frame-lost** bit (flags bit 2) *is* decoded at `sbus.cpp:668` and *does* reach the
FMU as `PX4IO_P_RAW_RC_FLAGS_FRAME_DROP`, but `px4io.cpp` never reads that flag, so it is
**not observable from any log**. Do not design a condition around detecting it.

---

## Step 0 — falsification test (decisive; run first)

The source predicts `rc_lost_frame_count == 0` under **every** condition, including total link
loss. Three sub-steps, increasing in severity, each held **3 minutes** (≥12,000 frames at
66.5 Hz).

**0a — nominal.** Transmitter on, 3 m, clear line of sight.
**0b — partial.** Transmitter antenna fully enclosed in a grounded metal can, lid resting on
top, at 3 m. Severe attenuation, link intermittent.
**0c — total.** Transmitter powered off for 3 minutes, then back on.

### Predicted readings

| Step | Field | Under **A** (source is right) | Under **B** (source is wrong) |
|---|---|---|---|
| 0a | `rc_total_frame_count` | advances ~66.5 or ~110.8 Hz | same |
| 0a | `rc_lost_frame_count` | **0** | 0 (no loss to count) |
| 0a | `rc_failsafe` | 0 throughout | 0 throughout |
| 0b | `rc_total_frame_count` | still advances at full rate | may stall in bursts |
| 0b | `rc_lost_frame_count` | **0** | **> 0 and climbing** |
| 0b | `rc_failsafe` | intermittently 1 | intermittently 1 |
| 0c | `rc_total_frame_count` | continues at full rate (receiver keeps driving the wire) | stalls |
| 0c | `rc_lost_frame_count` | **0** | **climbs steeply** |
| 0c | `rc_failsafe` | 1 continuously | 1 continuously |
| 0c | `rc_lost` | 1 after the 200 ms timeout | 1 |

### Decision rule

- **`rc_lost_frame_count` reads 0 in all three sub-steps → A is confirmed.** The metric is dead
  as an RF instrument. Stop; do not run Step 1 expecting a loss curve. Proceed to Step 2.
- **`rc_lost_frame_count` is nonzero at any point → the source read is wrong**, the deployed IO
  binary differs from `PX4-Autopilot @ v1.15.4-4-g85df8c2281`, and this whole analysis must be
  redone against the actual binary. Report that verbatim before anything else.

### The ordering question the prompt asks for

Record, for every sub-step, **whether `rc_lost_frame_count` moves before `rc_failsafe`
asserts.** This is the test of whether the counter — if it moves at all — is a graded metric
or merely a binary restatement of total loss:

- Moves in **0b, before** failsafe asserts → graded metric, usable for a loss-vs-degradation
  curve.
- Moves only in **0c**, at or after failsafe → binary indicator of total loss, redundant with
  `rc_failsafe`, useless as a graded metric.
- Never moves → dead (the predicted outcome under A).

---

## Step 1 — graded RF attenuation ladder

Run only if Step 0 falsifies A. Under A this ladder still has value, but for `rc_failsafe`
duty cycle rather than for a loss count — see Step 2.

Fixed geometry: transmitter on a tripod, antenna vertical, **3.0 m** from the vehicle, both at
1.0 m height, clear line of sight, nobody moving in the room. The only variable is the number
of identical absorbers in the line of sight, centred at the midpoint, touching neither antenna.

Absorber: **sealed 500 ml water bottles** in a rack. Water is strongly lossy at 2.4 GHz, the
bottles are identical, and stacking them is a genuinely monotonic axis. Nothing is modified and
the ladder is undone by lifting the rack away.

| Condition | Absorbers | Approx. added path loss | Expected loss ratio under **B** | Expected `rc_failsafe` duty |
|---|---|---|---|---|
| C0 | 0 (baseline) | 0 dB | 0 ppm (below floor) | 0% |
| C1 | 1 | ~4–8 dB | 0–100 ppm | 0% |
| C2 | 2 | ~8–15 dB | 100–1,000 ppm | 0% |
| C3 | 3 | ~12–22 dB | 1,000–10,000 ppm | 0–5% |
| C4 | 4 | ~16–30 dB | 10,000–100,000 ppm (1–10%) | 5–20% |
| C5 | 5 | ~20–37 dB | >100,000 ppm | 20–50% |
| C6 | 6 | ~24–45 dB | link at threshold | >50% |

**Under A, the `loss ratio` column reads 0 ppm at every row and only the duty-cycle column
varies.** That contrast is itself the confirmation.

The dB figures are indicative, not calibrated. The ladder's value is that each step is
monotonically worse than the last. A calibrated axis would need an inline SMA attenuator set,
which the fleet does not have and which no inventory file documents.

Run C0→C6 in ascending order, then **repeat C0** to confirm the baseline returns and nothing
drifted.

## Step 2 — `rc_failsafe` duty cycle as the fallback metric

Under A this is the only graded control-link measurement available without new hardware or a
PX4 rebuild. For each condition, compute:

```
failsafe_duty = (samples with rc_failsafe == 1) / (total input_rc samples in condition)
```

`input_rc` logs at ~2 Hz, so a 3-minute condition yields ~360 samples and a duty-cycle
resolution of ~0.3 percentage points. That is coarse but real, and unlike the loss counter it
demonstrably responds — six armed segments in the existing corpus show it asserting.

**Its limitation, stated plainly:** duty cycle at 2 Hz cannot resolve a 40 ms TDMA burst, and
it saturates at 100% well before the link fully fails. It supports an ordinal statement
("condition C5 is worse than C4"), not a frame-loss rate.

---

## Required log duration per condition

At **66.5 Hz** one frame arrives every 15.04 ms; at **110.8 Hz**, every 9.03 ms.

| Goal | Frames needed | At 66.5 Hz | At 110.8 Hz |
|---|---|---|---|
| Resolve 100 ppm at all (1 lost frame in 10,000) | 10,000 | **150.4 s ≈ 2 min 30 s** | 90.3 s |
| 10 lost frames at 100 ppm (±32%, 1σ Poisson) | 100,000 | 1,504 s ≈ 25 min | 903 s ≈ 15 min |
| 100 lost frames at 100 ppm (±10%, 1σ) | 1,000,000 | 15,038 s ≈ 4 h 11 min | 9,025 s ≈ 2 h 30 min |

**Minimum dwell: 2 min 30 s** (3 min used above for margin). The ~2 Hz `input_rc` log rate does
not constrain this — the counters accumulate in hardware between log samples, so a 3-minute
condition yields the full frame count regardless of sampling rate.

## Success criteria

1. **Step 0 returns `rc_lost_frame_count == 0` throughout** → hypothesis A confirmed
   empirically as well as from source. The decision token `RECOVERABLE_IN_SOFTWARE` stands:
   the frame-lost bit is decoded and reaches the FMU but is discarded, and recovering it
   requires the PX4 change named in the report, not a receiver purchase.
2. **Any nonzero `rc_lost_frame_count`** → the source read does not describe the deployed
   binary. Halt and re-derive.
3. **`rc_failsafe` duty cycle increases monotonically across C0→C6** → the fallback ordinal
   metric is valid and the coexistence table's control-link row can carry it, clearly labelled
   as a duty cycle and not a loss rate.
