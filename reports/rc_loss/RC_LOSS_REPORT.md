# Control-link frame loss report

**Date:** 2026-08-21
**Tool:** `rc_frame_loss.py` (this directory)
**Corpus:** 120 unique PX4 ULogs, 3 distinct airframes (by `sys_uuid`), all CUBEPILOT_CUBEORANGEPLUS
**Changes to vehicles:** none. Nothing flashed, no parameter set, no config edited, no service
restarted, no interface touched. Nothing installed.

---

## ⚠ Contradiction with the established facts — reported verbatim, not worked around

The prompt states as verified fact:

> `input_rc.rc_total_frame_count` and `input_rc.rc_lost_frame_count` are real hardware counters.
> Over 17.01 s the total went 1499 to 2630, a delta of 1131 frames, which is 66.5 Hz.
> Lost delta was 0.

**The first half of that is confirmed. The claim about `rc_lost_frame_count` is not.**

Everything stated about `rc_total_frame_count` reproduces exactly (see T2 below). But
`rc_lost_frame_count` is not demonstrably a counter at all:

```
logs inspected                          : 118
MAX rc_lost_frame_count seen ANYWHERE   : 0
logs where it was ever nonzero          : 0
logs with rc_lost   true at any sample  : 0
logs with rc_failsafe true at any sample: 11
```

Across **235,023 SBUS frames**, in **56 logs with a live counter**, on **three different
airframes**, spanning **2022 to 2026** and two PX4 releases, `rc_lost_frame_count` has never
held any value other than exactly `0` — not as a delta, not as an absolute reading, not in a
single sample.

Decisively, it also stayed at 0 through **11 logs in which the receiver asserted
`rc_failsafe`**, six of them in armed flight. In those episodes the total counter kept
advancing at full rate with no stall whatsoever:

```
Stormbringer log_101_2025-10-4-00-28-52.ulg   (failsafe asserted in 12 of 65 samples)
       t_s    total   lost   fs  rcl   d_total
     22.50    46232      0    0    0 +54 (111 Hz)
     23.00    46287      0    1    0 +55 (111 Hz)     <-- failsafe asserts
     23.50    46343      0    1    0 +56 (111 Hz)
     24.02    46400      0    1    0 +57 (110 Hz)
     24.50    46454      0    1    0 +54 (112 Hz)
     25.02    46511      0    1    0 +57 (110 Hz)
     25.50    46565      0    0    0 +54 (112 Hz)
  lost counter over whole log: 0 -> 0
```

**Interpretation.** This is the signature of an SBUS receiver that has lost its *transmitter*
but is still driving the FC serial line normally: it keeps emitting frames at the full
110 Hz and flags the condition inside the frame via the failsafe bit. `rc_total_frame_count`
and `rc_lost_frame_count` therefore describe the **receiver→flight-controller serial wire**,
which never faltered, and not the **transmitter→receiver RF link**, which demonstrably did.

Stopping here as instructed rather than working around it. `rc_lost_frame_count` is either
never populated on this PX4IO SBUS path, or it counts wire-level framing errors that have
never occurred in four years of logs. The two cannot be separated from existing data, which
is exactly what `POSITIVE_CONTROL_PROCEDURE.md` Step 0 is designed to settle.

**Consequence: a "zero loss" result from this counter is not publishable as evidence of a
clean RF link, because the counter has never been shown capable of reading anything else.**

A second, smaller contradiction: the prompt directs the tool and its output to Delta, and
Task T3 to "every reachable drone". **There is not one `.ulg` file on any drone** — Delta and
Ghost both return nothing for `find / -xdev -name "*.ulg"`. The entire ULog corpus lives on
the laptop. The analysis was run there; the code and outputs are placed on Delta as
instructed **and** in the repo, per the standing instruction that reports must be readable
away from the lab.

---

## T4 — direct answers

### 1. Does any log anywhere in the fleet have `frames_lost > 0`?

**No. Not one, in any log, on any vehicle.**

| Quantity | Value |
|---|---|
| Unique logs analysed | 120 |
| Logs containing `input_rc` | 118 |
| Logs whose frame counter actually advanced | 56 |
| Logs with `input_rc` but a counter that never moved (no RC connected) | 62 |
| Segments with `frames_lost > 0` | **0** |
| Total frames observed | **235,023** |
| Total frames lost | **0** |
| Pooled loss ratio | **0.0000 ppm** |
| Pooled detection floor | **4.2549 ppm** (1 frame in 235,023) |
| Counter wraps detected and corrected | 1 |
| Logs split by a counter reset | 0 |

There are no logs to list, because none qualify. Per the contradiction above, this zero
should be read as *"the counter never moved"*, not *"the link never lost a frame."*

### 2. Longest continuous armed segment, and its detection floor

**`multi-robot-coordination/px4_logs/log_71_UnknownDate.ulg` — 350.5 s continuously armed,
23,301 frames at 66.48 Hz, 0 lost, detection floor 42.9 ppm.**

Top continuous armed runs (55 armed runs found across the corpus):

| Log | Armed duration | Frames | Lost | Rate | Floor |
|---|---|---|---|---|---|
| `multi-robot-coordination/px4_logs/log_71_UnknownDate.ulg` | 350.5 s | 23,301 | 0 | 66.48 Hz | **42.9 ppm** |
| `Desktop/log_2_UnknownDate.ulg` | 188.0 s | 20,827 | 0 | 110.78 Hz | 48.0 ppm |
| `Desktop/log_3_UnknownDate.ulg` | 127.0 s | 14,067 | 0 | 110.76 Hz | 71.1 ppm |
| `multi-robot-coordination/px4_logs/log_70_UnknownDate.ulg` | 189.5 s | 12,598 | 0 | 66.48 Hz | 79.4 ppm |
| `Stormbringer/log_76_2025-9-26-11-37-12.ulg` | 105.0 s | 11,632 | 0 | 110.77 Hz | 86.0 ppm |
| `Desktop/ghostlog_4_UnknownDate.ulg` | 162.0 s | 10,770 | 0 | 66.48 Hz | 92.9 ppm |
| `Stormbringer/log_83_UnknownDate.ulg` | 86.0 s | 9,527 | 0 | 110.78 Hz | 105.0 ppm |
| `Stormbringer/log_77_2025-9-26-11-41-26.ulg` | 78.5 s | 8,693 | 0 | 110.73 Hz | 115.0 ppm |
| `Stormbringer/log_32_2025-9-1-21-35-08.ulg` | 67.0 s | 7,426 | 0 | 110.83 Hz | 134.7 ppm |
| `Stormbringer/log_84_UnknownDate.ulg` | 63.5 s | 7,034 | 0 | 110.77 Hz | 142.2 ppm |

**Pooled over all 55 armed runs: 230,351 frames, 0 lost, floor 4.341 ppm.**

Read carefully: the single-segment floor of 42.9 ppm means the longest armed flight could not
have resolved a loss rate below ~1 frame in 23,000. Pooling every armed run in the corpus
pushes that to 4.3 ppm — but pooling is only legitimate if the counter is live, which is
precisely what is in doubt.

### 3. Does any log show `rc_lost` or `rc_failsafe` true at any sample?

- **`rc_lost`: never.** 0 of 118 logs, not one sample. PX4 never declared RC lost.
- **`rc_failsafe`: yes — 11 of 118 logs.**

| Log | Frames | Lost | `rc_lost` | Arming | Rate |
|---|---|---|---|---|---|
| `GhostPX4Logs/log_0_2022-2-23-22-58-18.ulg` | 0 | 0 | 0 | disarmed | — |
| `GhostPX4Logs/log_1_2022-2-23-22-59-06.ulg` | 0 | 0 | 0 | disarmed | — |
| `GhostPX4Logs/log_2_2022-2-23-22-59-30.ulg` | 0 | 0 | 0 | disarmed | — |
| `GhostPX4Logs/log_5_2022-2-23-23-00-38.ulg` | 0 | 0 | 0 | disarmed | — |
| `GhostPX4Logs/log_6_2022-2-23-23-01-14.ulg` | 0 | 0 | 0 | disarmed | — |
| `Desktop/log_2_UnknownDate.ulg` | 20,936 | **0** | 0 | armed\|disarmed | 110.78 Hz |
| `Stormbringer/log_101_2025-10-4-00-28-52.ulg` | 3,547 | **0** | 0 | armed\|disarmed | 110.81 Hz |
| `Stormbringer/log_79_2025-9-26-12-10-26.ulg` | 3,377 | **0** | 0 | armed\|disarmed | 110.70 Hz |
| `Stormbringer/log_81_2025-9-26-19-23-48.ulg` | 1,884 | **0** | 0 | armed\|disarmed | 110.79 Hz |
| `Stormbringer/log_84_UnknownDate.ulg` | 7,144 | **0** | 0 | armed\|disarmed | 110.77 Hz |
| `Stormbringer/log_95_2025-9-26-23-55-28.ulg` | 2,327 | **0** | 0 | armed | 110.80 Hz |

The first five are 2022 bench logs with no receiver connected — the counter never moved and
failsafe was correctly asserted. **The last six are the important ones**: a live 110 Hz link,
in armed flight, asserting failsafe, with the lost counter reading exactly zero throughout.

`rc_failsafe` is the only field in `input_rc` that demonstrably responds to RF conditions. It
is binary and sampled at ~2 Hz, so it yields a duty cycle, not a loss ratio.

### 4. Observed SBUS frame rate — and it is **not** consistent

**Three distinct rates: 66.5 Hz, 74.4 Hz, and 110.8 Hz.** Not consistent across vehicles, and
not even consistent within a single airframe over time.

| `sys_uuid` | Hardware | Segments | Median | Range |
|---|---|---|---|---|
| `…510c00290023` (vehicle A) | CUBEPILOT_CUBEORANGEPLUS | 46 | 110.79 Hz | **66.5 – 110.9 Hz** |
| `…510f00340037` (vehicle B) | CUBEPILOT_CUBEORANGEPLUS | 5 | 74.41 Hz | 74.4 – 74.5 Hz |
| `…5102004f002b` (vehicle C) | CUBEPILOT_CUBEORANGEPLUS | 5 | 110.83 Hz | 110.8 – 111.1 Hz |

Vehicle A — the same airframe as the prompt's reference log — runs at **both** rates
depending on date: 66.48 Hz in its eleven 2026 logs (including the reference log
`log_7_2026-8-18-15-55-56.ulg`), and 110.8 Hz in its thirty-five 2025 logs. The receiver or
its frame-period mode changed between those campaigns.

```
vehicle A at ~66 Hz (11 logs, all 2026):
    66.48 Hz  Desktop/log_7_2026-8-18-15-55-56.ulg          <-- reference log
    66.48 Hz  multi-robot-coordination/px4_logs/log_71_UnknownDate.ulg
    66.48 Hz  multi-robot-coordination/px4_logs/log_70_UnknownDate.ulg
    66.48 Hz  Desktop/ghostlog_4_UnknownDate.ulg
    66.48 Hz  Desktop/ghostlog_6_UnknownDate.ulg
    66.49 Hz  Desktop/log_40_UnknownDate.ulg
    66.48 Hz  Desktop/log_42_UnknownDate.ulg
    66.48 Hz  Desktop/log_56_2026-3-21-23-39-36.ulg
    66.51 Hz  Desktop/log_57_UnknownDate.ulg
    66.49 Hz  Desktop/log_58_UnknownDate.ulg
    66.52 Hz  Desktop/log_61_UnknownDate.ulg
vehicle A at ~110 Hz (35 logs, 2025)
```

The prompt's 66.5 Hz is correct **for the reference log and for that vehicle's 2026
configuration only**. Any experiment mixing vehicles or campaigns must record the rate per
log — the detection floor scales directly with it, and 110.8 Hz buys a 1.67× tighter floor
for the same wall-clock duration.

---

## T2 — verification against the known log

Required: reproduce 1131 frames, 0 lost, 66.5 Hz, 0 wraps. **Passed exactly, on both parser
paths, which agree byte for byte:**

```
$ python3 rc_frame_loss.py --parser pyulog   log_7_2026-8-18-15-55-56.ulg
$ python3 rc_frame_loss.py --parser fallback log_7_2026-8-18-15-55-56.ulg
log_path,segment_index,t_start_s,t_end_s,duration_s,n_samples,sample_rate_hz,frames_total,frames_lost,loss_ratio_ppm,wraps_detected,sbus_frame_rate_hz,rc_lost_any,rc_failsafe_any,arming_states_seen,detection_floor_ppm
.../log_7_2026-8-18-15-55-56.ulg,0,103.798,120.811,17.013,33,1.881,1131,0,0.0,0,66.477,0,0,armed|disarmed,884.173
```

| Established fact | Tool output | Match |
|---|---|---|
| duration 17.01 s | 17.013 s | ✓ |
| counter 1499 → 2630 | 1499 → 2630 | ✓ |
| delta 1131 frames | `frames_total = 1131` | ✓ |
| 66.5 Hz | `sbus_frame_rate_hz = 66.477` | ✓ |
| lost delta 0 | `frames_lost = 0` | ✓ |
| wraps | `wraps_detected = 0` | ✓ |
| `rssi` ∈ {43, 44} | `[43, 44]` | ✓ |
| `link_quality` = −1 | `[-1]` | ✓ |
| `input_source` = 4 | `[4]` | ✓ |

The fallback parser's padding rule was confirmed directly:

```
input_rc struct size WITH trailing _padding : 72
input_rc struct size AFTER stripping padding: 71
trailing padding fields: [('uint8_t', '_padding0', 1)]
```

### Correctness requirements C1–C4

A synthetic self-test (`selftest.py`) exercises the wrap and reset logic, because the corpus
itself contains only one natural wrap and no resets — the real data could not have validated
this code on its own:

```
C1 uint16 wraparound
  PASS  normal step 1499->2630 (dt=17s)   got=(1131, 'normal')
  PASS  wrap 65500->30 (dt=0.5s, +66)     got=(66, 'wrap')
  PASS  wrap 65535->0                     got=(1, 'wrap')
  PASS  wrap 65000->200 (dt=10s, +736)    got=(736, 'wrap')
C2 counter reset on reboot (must NOT be silently bridged)
  PASS  reboot 40000->5 (dt=0.5s)         got=(None, 'reset')
  PASS  reboot 500->12 (dt=0.5s)          got=(None, 'reset')
  PASS  implausible fwd jump 100->60000   got=(None, 'reset')
boundary: corrected delta right at the plausibility ceiling
  PASS  wrap of exactly 350 accepted      got='wrap'
  PASS  wrap of 351 rejected as reset     got='reset'
C3 ratio is summed deltas, not mean of per-sample ratios
  PASS  summed-delta ratio  got=497.5 want=497.5
    (naive per-sample mean would have been 33333.3 ppm - 67x wrong)
FAILURES: 0
```

- **C1** — negative steps are corrected by +65536 only when the corrected delta is physically
  reachable in the elapsed time (ceiling 500 Hz + 100 frames slack). Wraps are counted and
  reported per segment. One real wrap was found and corrected in the corpus.
- **C2** — a backward jump too large to be a single wrap is treated as a reset: the segment is
  closed and a new one opened. No log in this corpus required a split.
- **C3** — `loss_ratio_ppm` is Σ(lost deltas)/Σ(total deltas), never a mean of per-sample
  ratios. The self-test shows the naive form would have been 67× wrong on a simple case.
- **C4** — `detection_floor_ppm = 10⁶/frames_total` is emitted on every row.

> **Bug found and fixed during verification.** `pyulog` returns `numpy.uint16` arrays, and
> numpy unsigned subtraction wraps silently instead of producing a negative value. The first
> corpus run therefore detected **zero** wraps, because `cur - prev` could never be negative.
> Counters are now cast to Python `int` before any arithmetic. After the fix the same corpus
> reports one wrap. Anyone reusing this code against ULog data should be aware that the
> obvious `last - first` formulation fails silently in exactly this way.

---

## T5 — positive control

Written to `POSITIVE_CONTROL_PROCEDURE.md` in this directory. **Not executed**, as instructed.
Given the contradiction above, its Step 0 (counter liveness) is now the decisive experiment
for this whole metric, not a formality.

---

## What the control-link column of the coexistence table can honestly report

The control-link column cannot report a frame-loss figure. Across 235,023 SBUS frames from 56
logs and three airframes spanning four years, `rc_lost_frame_count` has never held a nonzero
value in a single sample, and it remained at zero through six armed segments in which the
receiver asserted `rc_failsafe` while the total frame counter continued advancing at an
undisturbed 110 Hz — the signature of a receiver that has lost its transmitter but is still
driving the serial line perfectly. The counter pair therefore instruments the receiver→FC
wire, not the transmitter→receiver RF link that the coexistence experiment is about, and a
zero reading from it is uninterpretable: it cannot be distinguished from a field that is
simply never populated. The honest entry for this column is that the SBUS frame counters
establish a **detection floor of 4.3 ppm pooled (42.9 ppm on the longest single armed flight,
350.5 s) with zero loss observed**, stated explicitly as *"the counter did not move"* and
accompanied by the caveat that the counter has never been demonstrated to move under any
condition. Until `POSITIVE_CONTROL_PROCEDURE.md` Step 0 shows otherwise, the only field in
`input_rc` with demonstrated RF sensitivity is the binary `rc_failsafe` flag, which appears in
11 of 118 logs and can support a *duty-cycle* statement at the ~2 Hz logging rate — coarse,
but real — while `rssi_dbm` (NaN), `link_quality` (−1) and `rssi` (43/44, unconnected analog
pin) remain unusable as previously established. A quantitative RSSI-versus-interference row
still requires the CRSF/ELRS receiver identified in the RF instrumentation audit.

---

## Full CSV

`rc_frame_loss.csv`, 118 rows, one per log segment. Reproduced in full below.

```csv
log_path,segment_index,t_start_s,t_end_s,duration_s,n_samples,sample_rate_hz,frames_total,frames_lost,loss_ratio_ppm,wraps_detected,sbus_frame_rate_hz,rc_lost_any,rc_failsafe_any,arming_states_seen,detection_floor_ppm
/home/ssg/Desktop/bee_crash1.ulg,0,208.992,239.991,30.999,63,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/ghostlog_4_UnknownDate.ulg,0,410.612,572.61,161.998,153,0.938,10770,0,0.0,0,66.482,0,0,armed,92.851
/home/ssg/Desktop/ghostlog_6_UnknownDate.ulg,0,238.984,322.469,83.485,85,1.006,5550,0,0.0,0,66.479,0,0,armed|disarmed,180.18
/home/ssg/Desktop/GhostPX4Logs/log_0_2022-2-23-22-58-18.ulg,0,17.984,45.989,28.005,58,2.035,0,0,,0,0.0,0,1,disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_10_2024-12-29-17-57-50.ulg,0,217.436,227.436,10.0,21,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_11_2024-12-29-17-58-50.ulg,0,241.833,287.329,45.496,92,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_1_2022-2-23-22-59-06.ulg,0,12.477,42.477,30.0,62,2.033,0,0,,0,0.0,0,1,disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_12_2024-12-29-18-00-08.ulg,0,339.457,364.964,25.508,52,1.999,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_13_2024-12-29-18-01-32.ulg,0,395.604,449.104,53.5,108,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_14_2024-12-29-18-03-10.ulg,0,537.183,547.676,10.493,22,2.001,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_15_2024-12-29-18-05-36.ulg,0,610.534,693.524,82.99,167,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_16_2025-3-16-12-09-00.ulg,0,258.684,268.188,9.504,20,1.999,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_17_2025-3-16-12-30-38.ulg,0,39.51,44.502,4.992,11,2.003,0,0,,0,0.0,0,0,armed,
/home/ssg/Desktop/GhostPX4Logs/log_18_2025-3-16-12-33-04.ulg,0,82.048,105.044,22.996,47,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_19_2025-3-16-12-34-14.ulg,0,136.057,175.552,39.495,80,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_20_2025-3-16-12-36-12.ulg,0,244.105,292.596,48.491,98,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_21_2025-3-16-12-39-24.ulg,0,432.684,485.686,53.002,107,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_2_2022-2-23-22-59-30.ulg,0,10.695,18.699,8.004,18,2.124,0,0,,0,0.0,0,1,disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_22_2025-3-16-12-43-50.ulg,0,713.036,750.545,37.509,76,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_23_2025-4-27-22-02-44.ulg,0,657.268,665.255,7.987,17,2.003,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_24_2025-4-28-19-21-08.ulg,0,369.5,382.0,12.5,26,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_25_2025-4-28-19-23-28.ulg,0,508.162,522.657,14.496,30,2.001,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_26_2025-4-28-19-27-20.ulg,0,48.278,59.782,11.504,24,1.999,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_27_2025-4-28-19-31-54.ulg,0,315.258,332.766,17.508,36,1.999,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_28_2025-4-28-19-37-40.ulg,0,30.6,43.591,12.991,27,2.001,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_29_2025-4-28-19-40-28.ulg,0,34.652,46.156,11.504,24,1.999,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_30_2025-4-28-19-42-54.ulg,0,123.205,131.207,8.002,17,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_31_2025-4-28-19-45-20.ulg,0,85.382,101.882,16.5,34,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_32_2025-4-28-19-48-38.ulg,0,163.731,183.735,20.004,41,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_33_2025-7-2-23-39-46.ulg,0,259.615,267.614,7.999,17,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_34_2025-7-4-00-33-14.ulg,0,168.945,181.443,12.498,26,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_35_2025-7-4-19-12-32.ulg,0,38.107,48.095,9.988,21,2.002,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_36_2025-7-4-19-15-40.ulg,0,9.845,15.846,6.002,13,1.999,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_37_2025-7-5-22-07-40.ulg,0,34.837,45.832,10.995,23,2.001,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_38_2025-7-5-22-09-34.ulg,0,149.323,160.319,10.996,23,2.001,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_39_2025-7-8-16-11-52.ulg,0,90.889,90.889,0.0,1,,0,0,,0,,0,0,disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_40_2025-7-8-16-12-42.ulg,0,140.577,140.577,0.0,1,,0,0,,0,,0,0,armed,
/home/ssg/Desktop/GhostPX4Logs/log_41_2025-7-8-16-12-44.ulg,0,142.435,142.435,0.0,1,,0,0,,0,,0,0,disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_42_2025-7-8-16-12-50.ulg,0,145.562,147.553,1.991,5,2.009,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_43_2025-7-8-17-12-50.ulg,0,55.283,86.291,31.008,63,1.999,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_44_2025-7-8-17-13-38.ulg,0,110.687,133.683,22.996,47,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_45_2025-7-8-17-15-00.ulg,0,159.099,215.103,56.004,113,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_46_2025-8-6-00-03-02.ulg,0,111.183,149.688,38.505,78,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_47_2025-8-6-00-04-38.ulg,0,173.25,244.75,71.5,144,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_48_2025-8-6-00-07-36.ulg,0,352.327,422.327,70.0,141,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_49_2025-8-6-00-10-22.ulg,0,514.878,588.368,73.49,148,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_50_2025-7-13-12-08-58.ulg,0,113.992,155.992,42.0,85,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_51_2025-7-13-12-24-24.ulg,0,257.156,294.656,37.5,76,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_5_2022-2-23-23-00-38.ulg,0,8.878,30.882,22.004,46,2.045,0,0,,0,0.0,0,1,disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_52_2025-7-13-12-27-22.ulg,0,439.222,471.72,32.498,66,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_53_2025-7-13-12-32-20.ulg,0,210.662,229.665,19.004,39,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_54_2025-7-13-12-38-50.ulg,0,243.588,266.087,22.499,46,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_55_2025-7-13-12-46-10.ulg,0,139.525,169.024,29.499,60,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_56_2025-7-13-12-51-06.ulg,0,154.628,178.135,23.507,48,1.999,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_57_2025-7-13-12-54-02.ulg,0,123.785,146.286,22.501,46,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_58_2025-7-13-12-58-10.ulg,0,191.715,215.207,23.492,48,2.001,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_6_2022-2-23-23-01-14.ulg,0,12.562,33.062,20.5,43,2.049,0,0,,0,0.0,0,1,disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_7_2024-12-29-17-55-54.ulg,0,79.56,112.577,33.017,67,1.999,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_8_2024-12-29-17-56-36.ulg,0,120.364,153.365,33.001,67,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/log_9_2024-12-29-17-57-32.ulg,0,198.62,208.624,10.004,21,1.999,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/GhostPX4Logs/test1_bench_VIO_noOF_noRNG_oct-21-2025.ulg,0,117.052,134.056,17.003,10,0.529,1884,0,0.0,0,110.801,0,0,armed,530.786
/home/ssg/Desktop/log_10_2025-10-27-00-41-08.ulg,0,175.301,179.813,4.512,10,1.995,336,0,0.0,0,74.475,0,0,armed|disarmed,2976.19
/home/ssg/Desktop/log_11_2025-10-27-00-58-04.ulg,0,174.257,181.753,7.495,16,2.001,558,0,0.0,0,74.446,0,0,armed|disarmed,1792.115
/home/ssg/Desktop/log_12_2025-10-27-01-08-06.ulg,0,163.25,170.24,6.991,15,2.003,520,0,0.0,0,74.386,0,0,armed|disarmed,1923.077
/home/ssg/Desktop/log_12_2026-2-4-10-49-40.ulg,0,65.481,86.477,20.996,43,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/log_13_2025-10-27-01-33-46_1.ulg,0,76.889,86.405,9.516,20,1.997,708,0,0.0,0,74.401,0,0,armed|disarmed,1412.429
/home/ssg/Desktop/log_14_2026-2-4-11-50-48.ulg,0,116.7,129.195,12.495,26,2.001,0,0,,0,0.0,0,0,armed,
/home/ssg/Desktop/log_16_UnknownDate.ulg,0,102.382,139.381,36.999,75,2.0,2753,0,0.0,0,74.407,0,0,armed|disarmed,363.24
/home/ssg/Desktop/log_185_UnknownDate.ulg,0,143.249,158.735,15.486,32,2.002,1720,0,0.0,0,111.068,0,0,armed,581.395
/home/ssg/Desktop/log_20_2026-2-4-14-26-34.ulg,0,367.651,385.651,18.0,37,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/log_2_UnknownDate.ulg,0,382.395,571.385,188.991,374,1.974,20936,0,0.0,0,110.778,0,1,armed|disarmed,47.765
/home/ssg/Desktop/log_3_UnknownDate.ulg,0,232.12,360.112,127.992,257,2.0,14176,0,0.0,0,110.757,0,0,armed|disarmed,70.542
/home/ssg/Desktop/log_40_UnknownDate.ulg,0,73.151,135.157,62.005,57,0.903,4123,0,0.0,0,66.494,0,0,armed|disarmed,242.542
/home/ssg/Desktop/log_42_UnknownDate.ulg,0,128.907,198.895,69.988,79,1.114,4653,0,0.0,0,66.483,0,0,armed|disarmed,214.915
/home/ssg/Desktop/log_56_2026-3-21-23-39-36.ulg,0,825.828,829.829,4.001,4,0.75,266,0,0.0,0,66.482,0,0,armed|disarmed,3759.398
/home/ssg/Desktop/log_57_UnknownDate.ulg,0,122.793,143.301,20.508,26,1.219,1364,0,0.0,0,66.512,0,0,armed|disarmed,733.138
/home/ssg/Desktop/log_58_UnknownDate.ulg,0,188.988,293.003,104.015,107,1.019,6916,0,0.0,0,66.491,0,0,armed|disarmed,144.592
/home/ssg/Desktop/log_61_UnknownDate.ulg,0,133.838,156.342,22.504,25,1.066,1497,0,0.0,0,66.522,0,0,armed|disarmed,668.003
/home/ssg/Desktop/log_7_2026-8-18-15-55-56.ulg,0,103.798,120.811,17.013,33,1.881,1131,0,0.0,0,66.477,0,0,armed|disarmed,884.173
/home/ssg/Desktop/log_9_2026-2-4-09-29-42.ulg,0,101.1,121.6,20.5,42,2.0,0,0,,0,0.0,0,0,armed|disarmed,
/home/ssg/Desktop/log_97_UnknownDate.ulg,0,235.936,266.423,30.487,62,2.001,3379,0,0.0,0,110.835,0,0,armed|disarmed,295.946
/home/ssg/Desktop/PrometheusPX4Logs/log_25_2025-10-24-15-07-44.ulg,0,107.385,133.895,26.51,54,1.999,2939,0,0.0,0,110.863,0,0,armed|disarmed,340.252
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_100_2025-9-27-00-38-32.ulg,0,448.873,468.873,20.0,41,2.0,2216,0,0.0,0,110.801,0,0,armed|disarmed,451.264
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_101_2025-10-4-00-28-52.ulg,0,388.811,420.821,32.01,65,1.999,3547,0,0.0,0,110.808,0,1,armed|disarmed,281.928
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_11_2025-11-11-18-56-50.ulg,0,1266.044,1297.044,30.999,15,0.452,3433,0,0.0,0,110.745,0,0,armed,291.29
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_12_2025-11-11-19-21-30.ulg,0,446.401,457.915,11.514,13,1.042,1275,0,0.0,0,110.733,0,0,armed,784.314
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_13_2025-11-11-19-27-06.ulg,0,777.48,789.48,12.0,10,0.75,1330,0,0.0,0,110.833,0,0,armed,751.88
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_14_2025-11-11-19-27-52.ulg,0,810.083,840.068,29.984,26,0.834,3322,0,0.0,0,110.791,0,0,armed|disarmed,301.023
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_15_2025-11-11-19-31-26.ulg,0,1041.315,1051.314,9.999,7,0.6,1108,0,0.0,0,110.809,0,0,armed,902.527
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_32_2025-9-1-21-35-08.ulg,0,76.756,144.76,68.004,137,2.0,7537,0,0.0,0,110.832,0,0,armed|disarmed,132.679
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_33_2025-9-1-21-37-08.ulg,0,221.735,265.241,43.507,85,1.931,4822,0,0.0,0,110.833,0,0,armed|disarmed,207.383
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_75_2025-9-26-11-35-08.ulg,0,199.99,248.496,48.506,98,2.0,5372,0,0.0,0,110.75,0,0,armed|disarmed,186.15
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_76_2025-9-26-11-37-12.ulg,0,267.891,373.893,106.003,213,2.0,11742,0,0.0,0,110.771,0,0,armed|disarmed,85.164
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_77_2025-9-26-11-41-26.ulg,0,547.187,626.688,79.502,154,1.924,8803,0,0.0,1,110.727,0,0,armed|disarmed,113.598
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_78_2025-9-26-12-06-08.ulg,0,92.824,115.339,22.514,46,1.999,2493,0,0.0,0,110.729,0,0,armed|disarmed,401.123
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_79_2025-9-26-12-10-26.ulg,0,179.606,210.113,30.507,62,2.0,3377,0,0.0,0,110.698,0,1,armed|disarmed,296.121
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_80_2025-9-26-19-19-48.ulg,0,229.981,248.476,18.495,38,2.001,2050,0,0.0,0,110.841,0,0,armed|disarmed,487.805
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_81_2025-9-26-19-23-48.ulg,0,471.409,488.415,17.006,35,1.999,1884,0,0.0,0,110.785,0,1,armed|disarmed,530.786
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_82_2025-9-26-19-24-32.ulg,0,517.988,531.503,13.515,23,1.628,1498,0,0.0,0,110.839,0,0,armed|disarmed,667.557
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_83_UnknownDate.ulg,0,110.838,197.355,86.517,174,2.0,9585,0,0.0,0,110.787,0,0,armed|disarmed,104.33
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_84_UnknownDate.ulg,0,338.644,403.139,64.495,125,1.923,7144,0,0.0,0,110.768,0,1,armed|disarmed,139.978
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_85_UnknownDate.ulg,0,513.529,554.024,40.495,82,2.0,4485,0,0.0,0,110.755,0,0,armed|disarmed,222.965
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_86_UnknownDate.ulg,0,833.28,855.791,22.51,46,1.999,2492,0,0.0,0,110.704,0,0,armed|disarmed,401.284
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_87_UnknownDate.ulg,0,869.729,898.223,28.495,58,2.0,3155,0,0.0,0,110.723,0,0,armed|disarmed,316.957
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_88_2025-9-26-21-38-54.ulg,0,367.386,368.892,1.506,4,1.991,167,0,0.0,0,110.856,0,0,armed|disarmed,5988.024
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_89_2025-9-26-21-39-24.ulg,0,381.571,397.583,16.011,33,1.999,1775,0,0.0,0,110.859,0,0,armed|disarmed,563.38
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_90_2025-9-26-21-40-34.ulg,0,451.418,468.417,16.999,35,2.0,1884,0,0.0,0,110.828,0,0,armed|disarmed,530.786
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_91_2025-9-26-21-40-58.ulg,0,472.406,491.922,19.516,40,1.998,2164,0,0.0,0,110.881,0,0,armed|disarmed,462.107
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_92_2025-9-26-21-41-40.ulg,0,517.149,534.152,17.003,35,2.0,1885,0,0.0,0,110.862,0,0,armed|disarmed,530.504
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_93_2025-9-26-21-46-44.ulg,0,170.328,183.822,13.494,28,2.001,1495,0,0.0,0,110.789,0,0,armed|disarmed,668.896
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_94_2025-9-26-21-47-30.ulg,0,214.045,231.546,17.501,36,2.0,1939,0,0.0,0,110.794,0,0,armed|disarmed,515.73
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_95_2025-9-26-23-55-28.ulg,0,498.223,519.226,21.003,43,2.0,2327,0,0.0,0,110.795,0,1,armed,429.738
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_96_2025-9-27-00-34-00.ulg,0,195.797,196.781,0.984,3,2.033,109,0,0.0,0,110.791,0,0,armed|disarmed,9174.312
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_97_2025-9-27-00-35-10.ulg,0,251.028,266.524,15.495,32,2.001,1717,0,0.0,0,110.807,0,0,armed|disarmed,582.411
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_98_2025-9-27-00-35-54.ulg,0,285.685,309.191,23.506,48,2.0,2605,0,0.0,0,110.825,0,0,armed|disarmed,383.877
/home/ssg/Desktop/Stormbringer_cubeorange+_px4logs/log_99_2025-9-27-00-37-40.ulg,0,396.505,415.524,19.018,39,1.998,2108,0,0.0,0,110.841,0,0,armed|disarmed,474.383
/home/ssg/multi-robot-coordination/px4_logs/log_70_UnknownDate.ulg,0,156.183,345.67,189.487,168,0.881,12598,0,0.0,0,66.485,0,0,armed,79.378
/home/ssg/multi-robot-coordination/px4_logs/log_71_UnknownDate.ulg,0,204.535,556.519,351.983,342,0.969,23401,0,0.0,0,66.483,0,0,armed|disarmed,42.733
```

---
---

# Addendum — is `rc_lost_frame_count` recoverable, or is the metric dead?

**Date:** 2026-08-21
**Question:** settle hypothesis **A** (counter never populated on this code path) against
**B** (counter populated, but nothing in the corpus ever degraded the link enough to move it).
**PX4 source read:** `/home/ssg/PX4-Autopilot` at **`v1.15.4-4-g85df8c2281`** — v1.15.4 plus
four backport commits, matching the `ver_sw_release` in the logs.
**Changes to vehicles:** none. No flash, no parameter set, no config edit, no service restart,
no interface change, no rewiring.

## Decision

```
A is TRUE, proven from source. Decision token: RECOVERABLE_IN_SOFTWARE
```

## T1 — corpus path correction

**Contradiction with the prompt, reported verbatim.** The prompt states:

> T1. **Correct the corpus path.** Prompt 1 reported no `.ulg` on any drone and analyzed a
> laptop corpus of 120 logs. The fleet logs are at
> `/home/ssg/lunar_lab/multi-drone-project/code/PX4logs/`.

That directory holds **exactly one file**:

```
$ ls -la /home/ssg/lunar_lab/multi-drone-project/code/PX4logs/
total 6672
-rw-rw-r-- 1 ssg ssg 6819969 Aug 18 15:57 log_7_2026-8-18-15-55-56.ulg
  file count: 1
```

It is not a fleet corpus, and it is not a different corpus. It is **the reference log itself**,
and it was already inside the analysed set.

**Verdict: strict subset of size 1, fully contained. Not a superset, not disjoint.**

| Check | Result |
|---|---|
| Files in `PX4logs/` | 1 |
| Files in the Prompt-1 corpus | 124 found, 120 unique by content |
| Overlap | 1 of 1 — the whole directory |
| `PX4logs/log_7_…ulg` md5 | `d0e24027e81407d135b029e472e1067b` |
| `Desktop/log_7_…ulg` md5 | `d0e24027e81407d135b029e472e1067b` — **identical** |
| Present in `all_logs.txt` | lines 83 and 121 (both copies) |
| Present in `logs_unique.txt` | line 81 (Desktop path kept, lunar_lab copy dropped as a byte-identical duplicate) |
| Used for the T2 verification in Prompt 1 | Yes — the lunar_lab path explicitly |

**The 118-log conclusion was not drawn from a different set of files.** The stated directory's
sole log is one of the 120, is the very log the extractor was validated against, and its own
row reads `frames_lost = 0`. Re-running the extractor over that directory alone reproduces
exactly the Prompt-1 row for it and changes nothing:

```
log_7_2026-8-18-15-55-56.ulg,0,103.798,120.811,17.013,33,1.881,1131,0,0.0,0,66.477,0,0,armed|disarmed,884.173
```

**Restated conclusion against the correct corpus:** unchanged, and now strictly stronger than
required. The one-log directory named in the prompt yields `frames_lost = 0` and
`rc_lost_frame_count` max 0; the 120-log superset that contains it yields the same across
235,023 frames and three airframes. There is no corpus, large or small, in which this counter
has ever been nonzero — and the source trace below explains why there cannot be one.

There is still **no `.ulg` on any drone**. Delta and Ghost both return nothing for
`find / -xdev -name "*.ulg"`. That finding from Prompt 1 stands.

## T2 — code path trace

### Q1. Does the SBUS decoder extract the frame-lost flag bit at all?

**Yes.** Both bits live in the same flags byte and both are read, in the same statement:

`src/lib/rc/sbus.cpp:69-71`
```c
#define SBUS_FLAGS_BYTE		23
#define SBUS_FAILSAFE_BIT	3
#define SBUS_FRAMELOST_BIT	2
```

`src/lib/rc/sbus.cpp:662-681`
```c
	/* decode and handle failsafe and frame-lost flags */
	if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) { /* failsafe */
		*sbus_failsafe = true;
		*sbus_frame_drop = true;

	} else if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT)) { /* a frame was lost */
		*sbus_failsafe = false;
		*sbus_frame_drop = true;

	} else {
		*sbus_failsafe = false;
		*sbus_frame_drop = false;
	}
```

Note what the decoder produces: a **boolean per frame**, `*sbus_frame_drop`. It is not a count.
The separate static counter `sbus_frame_drops` (`sbus.cpp:127`) is **not** incremented here —
it is incremented only on parser desync and bad start symbols, at `sbus.cpp:474`, `:507` and
`:574`, all of which are UART framing failures, not the receiver's frame-lost flag.

### Q2. Does the PX4IO→FMU protocol carry the count across the wire, or is it dropped there?

**The protocol defines a register for it. The IO firmware never writes it.**

`src/modules/px4iofirmware/protocol.h:141-142`
```c
#define PX4IO_P_RAW_FRAME_COUNT			4	/* Number of total received frames (wrapping counter) */
#define PX4IO_P_RAW_LOST_FRAME_COUNT		5	/* Number of total dropped frames (wrapping counter) */
```

Every reference to those two symbols in the entire source tree:

```
protocol.h:141   #define PX4IO_P_RAW_FRAME_COUNT        4
protocol.h:142   #define PX4IO_P_RAW_LOST_FRAME_COUNT   5
registers.c:107      [PX4IO_P_RAW_FRAME_COUNT]      = 0,
registers.c:108      [PX4IO_P_RAW_LOST_FRAME_COUNT] = 0,
controls.cpp:373     r_page_raw_rc_input[PX4IO_P_RAW_FRAME_COUNT]++;
px4io.cpp:1029       const uint16_t rc_valid_update_count = io_reg_get(..., PX4IO_P_RAW_FRAME_COUNT);
px4io.cpp:1094       input_rc.rc_lost_frame_count  = regs[PX4IO_P_RAW_LOST_FRAME_COUNT];
px4io.cpp:1095       input_rc.rc_total_frame_count = regs[PX4IO_P_RAW_FRAME_COUNT];
```

`PX4IO_P_RAW_FRAME_COUNT` is incremented at `src/modules/px4iofirmware/controls.cpp:373`.
**`PX4IO_P_RAW_LOST_FRAME_COUNT` appears exactly three times: its definition, its
initialisation to `0`, and the FMU reading it.** There is no `++`, no assignment, no write of
any kind anywhere in the PX4IO firmware.

`src/modules/px4iofirmware/registers.c:102-110`
```c
uint16_t		r_page_raw_rc_input[] = {
	[PX4IO_P_RAW_RC_COUNT]			= 0,
	[PX4IO_P_RAW_RC_FLAGS]			= 0,
	[PX4IO_P_RAW_RC_NRSSI]			= 0,
	[PX4IO_P_RAW_RC_DATA]			= 0,
	[PX4IO_P_RAW_FRAME_COUNT]		= 0,
	[PX4IO_P_RAW_LOST_FRAME_COUNT]		= 0,     // <-- initialised, never written again
	[PX4IO_P_RAW_RC_BASE ...(PX4IO_P_RAW_RC_BASE + PX4IO_RC_INPUT_CHANNELS)] = 0
};
```

`src/drivers/px4io/px4io.cpp:1092-1095`
```c
	input_rc.rc_failsafe = (regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
	input_rc.rc_lost = !(regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_RC_OK);
	input_rc.rc_lost_frame_count = regs[PX4IO_P_RAW_LOST_FRAME_COUNT];   // <-- reads a constant 0
	input_rc.rc_total_frame_count = regs[PX4IO_P_RAW_FRAME_COUNT];
```

So the FMU faithfully transports a register that the IO side leaves at its initial value. The
zero observed in 235,023 frames is not a measurement — it is `registers.c:108` being copied
into every `input_rc` message ever published on this airframe family.

### Q3. Is the count populated on the FMU-direct path but not the PX4IO path?

**Yes — and the two paths are exactly complementary, which is the surprise.**

`src/drivers/rc_input/RCInput.cpp:263-264` (FMU-direct, RC on an FMU serial port)
```c
	_input_rc.rc_lost_frame_count = frame_drops;
	_input_rc.rc_total_frame_count = 0;
```

| Path | `rc_total_frame_count` | `rc_lost_frame_count` |
|---|---|---|
| **PX4IO** (`px4io.cpp`) — what this fleet uses (`input_source = 4`) | **populated** (`controls.cpp:373`) | **always 0** (register never written) |
| **FMU-direct** (`RCInput.cpp`) | **hardwired 0** (`:264`) | **populated** (`:263`) |

In PX4 v1.15.4 you can have the numerator or the denominator, never both.

And the numerator you would get on the FMU-direct path is **not** the receiver's frame-lost
flag: `frame_drops` there is `sbus_frame_drops` from `sbus_parse`, which counts UART desync and
bad start symbols (`sbus.cpp:474, 507, 574`). It measures the **wire**, not the RF link. This
is the source-level confirmation of what Prompt 1 inferred behaviourally from the six armed
failsafe segments.

Full list of writers of `rc_lost_frame_count` across the tree, for completeness:
```
mavlink_receiver.cpp:2002   rc.rc_lost_frame_count = 0;          (MAVLink RC override)
rpi_rc_in.cpp:121           _data.rc_lost_frame_count = 0;
px4io.cpp:1094              = regs[PX4IO_P_RAW_LOST_FRAME_COUNT]  (constant 0)
RCInput.cpp:263             = frame_drops                         (UART parser errors)
voxl2_io.cpp:499            = _sbus_frame_drops
```

## T3 — the flags byte: receiver behaviour or propagation gap?

**Propagation gap. Unambiguously.** Not receiver behaviour.

The receiver's frame-lost bit is decoded from the *same byte 23* as the failsafe bit that
demonstrably asserted six times. It is not ignored at the decoder. It is carried across the
IO↔FMU wire. It is then **thrown away by the FMU driver**, in two independent ways:

**Gap 1 — the flag is transported and never read.**
`sbus.cpp:668` decodes bit 2 → `*sbus_frame_drop`
→ `src/modules/px4iofirmware/controls.cpp:263-268`:
```c
			if (sbus_frame_drop) {
				r_raw_rc_flags |= PX4IO_P_RAW_RC_FLAGS_FRAME_DROP;
			} else {
				r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FRAME_DROP);
			}
```
→ crosses the wire in `PX4IO_P_RAW_RC_FLAGS`
→ **and `src/drivers/px4io/px4io.cpp` contains zero references to
`PX4IO_P_RAW_RC_FLAGS_FRAME_DROP`.** Verified by grep over the whole tree: every one of the
eight references lives inside `src/modules/px4iofirmware/`, none on the FMU side. At
`px4io.cpp:1092-1095` only `FAILSAFE` and `RC_OK` are extracted from that register. The
frame-drop bit arrives at the FMU on every cycle and is discarded.

**Gap 2 — the count register is never incremented.**
`controls.cpp:263-268` sets a *flag*; it never does
`r_page_raw_rc_input[PX4IO_P_RAW_LOST_FRAME_COUNT]++`, which is exactly the one-line
counterpart of the `PX4IO_P_RAW_FRAME_COUNT++` that sits ten lines later at `:373`.

**Evidence that this is not receiver behaviour:** the question "does the receiver set bit 2?"
is *unanswerable from any log*, precisely because bit 2 is discarded before it reaches
`input_rc`. No amount of flying produces evidence either way. What the source does establish is
that if the receiver were setting bit 2 on every single frame, every field visible in the log
would look exactly as it does today. The observed all-zero corpus is therefore fully explained
by the propagation gap alone and carries no information about receiver behaviour.

## T4 — decision

```
RECOVERABLE_IN_SOFTWARE
```

The frame-lost bit **is** decoded and **does** reach the FMU; it is dropped before `input_rc`.
Exact locations of the drop, and the minimal fix at each:

| # | Exact location | What is dropped | Minimal change |
|---|---|---|---|
| 1 | `src/modules/px4iofirmware/controls.cpp:263-268` | `sbus_frame_drop` sets a flag but never increments a counter | add `r_page_raw_rc_input[PX4IO_P_RAW_LOST_FRAME_COUNT]++;` inside the `if (sbus_frame_drop)` branch — the exact counterpart of `:373` |
| 2 | `src/drivers/px4io/px4io.cpp:1092-1095` | `PX4IO_P_RAW_RC_FLAGS_FRAME_DROP` arrives in `regs[PX4IO_P_RAW_RC_FLAGS]` and is never read | optional: also surface the per-cycle flag, if a boolean is wanted alongside the count |

Change 1 alone is sufficient and is the correct fix: it makes `rc_lost_frame_count` a genuine
`uint16` count of receiver-reported frame-lost events, wrapping like its sibling — which the
extractor already handles (C1). Change 2 is cosmetic by comparison.

**Why this is not `RECOVERABLE_BY_REWIRING`, even though the count does exist on the FMU-direct
path.** Moving RC to an FMU serial port would populate `rc_lost_frame_count`, but it would
(a) zero `rc_total_frame_count` (`RCInput.cpp:264`), destroying the denominator the loss ratio
needs, and (b) deliver UART parser-desync counts rather than the receiver's RF frame-lost flag.
It trades a dead field for a misleading one, and it requires physically rewiring the receiver
off the IO RC pin onto an FMU UART plus setting `RC_PORT_CONFIG`. **Not recommended, and not
performed.**

**Why this is not `DEAD`.** The bit is decoded (`sbus.cpp:668`). The `DEAD` criterion requires
that the bit is never decoded or that the receiver never sets it; the first is false, and the
second is unknowable for the reason given in T3.

### Consequence for the RF instrumentation audit

This **downgrades** the control-link frame-loss row from NEEDS HARDWARE to NEEDS CODE. A
one-line PX4IO firmware change plus a rebuild and flash recovers a graded, receiver-reported
RF frame-loss count without buying anything. Note carefully what it does **not** fix: `rssi_dbm`
(NaN), `link_quality` (−1) and `rssi` (dead analog pin) remain unavailable, because SBUS carries
no RSSI field at all. **The CRSF/ELRS receiver recommendation stands for RSSI and link quality;
it is no longer required for frame loss.**

## T5 — receiver configuration across the fleet

### Per-vehicle state

| Vehicle (`sys_uuid`) | Current rate | Newest log | Segments | All rates seen | Receiver model |
|---|---|---|---|---|---|
| `000600000000323136383132510c00290023` (A) | **66.48 Hz** | 2026-08-18 `log_7_2026-8-18-15-55-56.ulg` | 46 | 66.5, 110.7, 110.8, 110.9 | **not determinable** |
| `000600000000383937303133510f00340037` (B) | **74.41 Hz** | 2026-03-27 `log_16_UnknownDate.ulg` | 5 | 74.4, 74.5 | **not determinable** |
| `0006000000003734343332335102004f002b` (C) | **110.76 Hz** | 2026-08-17 `log_3_UnknownDate.ulg` | 5 | 110.8, 110.9, 111.1 | **not determinable** |

All three are `CUBEPILOT_CUBEORANGEPLUS`. Vehicle A alone has run at both 66.5 Hz (its eleven
2026 logs) and ~110.8 Hz (its thirty-five 2025 logs).

**Receiver model is not determinable, and is not guessed.** SBUS carries no receiver identity;
the boot console reports only `INFO [px4io] IO FW CRC match` with no RC device line; and no
inventory or hardware document exists anywhere in the repo — consistent with the earlier RF
audit finding F3. Determining the model requires physically reading the labels.

### Parameters that differ across the fleet

Every SBUS-relevant parameter is **identical on all three vehicles**:

| Parameter | Vehicle A | Vehicle B | Vehicle C |
|---|---|---|---|
| `PWM_SBUS_MODE` | 0 | 0 | 0 |
| `RC_CHAN_CNT` | 18 | 18 | 18 |
| `RC_RSSI_PWM_CHAN` | 0 | 0 | 0 |
| `RC_RSSI_PWM_MIN` / `MAX` | 1000 / 2000 | 1000 / 2000 | 1000 / 2000 |
| `RC_INPUT_PROTO` | ABSENT | ABSENT | ABSENT |
| `SENS_EN_SBUS` | ABSENT | ABSENT | ABSENT |

Parameters that *do* differ, none of which affect frame rate:

| Parameter | Vehicle A | Vehicle B | Vehicle C | Note |
|---|---|---|---|---|
| `COM_RC_LOSS_T` | 0.5, 2.0 | 0.5, 2.0 | 0.5 | RC-loss failsafe delay; affects `rc_lost` latency, not frame rate |
| `COM_RC_IN_MODE` | 0, 3 | 3 | 3 | vehicle A has logs with stick input enabled and disabled |
| `RC_MAP_FLTMODE` | 12, 5, 6 | 0, 8 | 12, 7 | channel mapping only |
| `RC_MAP_THROTTLE` | 3 | 2, 3 | 3 | channel mapping only |

**Therefore the three frame rates are receiver-side, not parameter-side.** No PX4 parameter
change can align them; the receivers themselves differ in model or in their own frame-period
setting. This also means the "same vehicle, two rates" history of vehicle A records a **receiver
swap or a receiver-side frame-period change** between the 2025 and 2026 campaigns, not a
configuration drift in PX4.

Aliasing was ruled out as an explanation: the PX4IO main loop is a free-running `for(;;)` with
no sleep (`src/modules/px4iofirmware/px4io.cpp:354-374`), polling far faster than any SBUS
frame rate, so `PX4IO_P_RAW_FRAME_COUNT` increments once per genuine frame. The measured
66.5 / 74.4 / 110.8 Hz are real SBUS arrival rates, and 110.8 Hz ≈ 9.03 ms period is the
standard "fast SBUS" frame interval.

### Recommended target rate

**Standardise the fleet on ~110.8 Hz (9 ms fast SBUS).** Justification:

1. It is already the majority configuration — 40 of 56 live-counter segments, and it is what
   vehicles A and C ran most recently before A changed.
2. **It buys a 1.67× tighter detection floor for the same wall-clock duration.** At 110.8 Hz a
   150 s condition yields 16,620 frames (60.2 ppm floor) against 9,975 frames at 66.5 Hz
   (100.3 ppm). The floor scales directly with rate, which is exactly why the rows are not
   comparable today.
3. It is a receiver-side setting, so aligning it costs nothing in PX4 configuration.

Until every vehicle runs one rate, **coexistence-table rows from different vehicles are not
comparable** and each row must carry its own frame rate and detection floor.

**Nothing was changed. This is a recommendation only.**

## T6 — revised positive control

`POSITIVE_CONTROL_PROCEDURE.md` has been rewritten. Step 0 is now a **falsification test**
rather than a discovery experiment, since the source has already settled A: it runs a
three-level monotonic severity ladder (nominal → antenna in a grounded can → transmitter off),
tabulates the predicted reading of every field under A and under B, and explicitly records
**whether `rc_lost_frame_count` moves before `rc_failsafe` asserts** — the test of whether the
counter, if it moved at all, would be a graded metric or merely a binary restatement of total
loss. A new Step 2 defines `rc_failsafe` duty cycle as the fallback ordinal metric available
under A without new hardware or a rebuild. Props off, disarmed, reversible, no firmware or
radio configuration changes. **Written only, not executed.**

## Revised statement for the coexistence table

The earlier statement in this report — that the control-link column cannot report a frame-loss
figure — stands as a description of **what the logs can show today**, but its stated cause was
one of two possibilities and the source trace has now selected between them. The counter is not
"a field that might be real but never saw loss": `PX4IO_P_RAW_LOST_FRAME_COUNT` is initialised
to zero at `registers.c:108` and never written by any line of PX4IO firmware, so
`input_rc.rc_lost_frame_count` on this airframe family is `registers.c:108` copied into every
message, and no flight condition whatsoever could have moved it. The 235,023-frame zero carries
no information about the RF link and must not be published as evidence of one; the honest entry
remains a detection floor of 4.3 ppm pooled, 42.9 ppm on the longest single armed flight,
annotated *"counter structurally incapable of being nonzero on the PX4IO path — see
`px4io.cpp:1094`, `registers.c:108`."* What has changed is the remedy: because the receiver's
frame-lost bit **is** decoded at `sbus.cpp:668` and **does** reach the FMU inside
`PX4IO_P_RAW_RC_FLAGS` before being discarded, a graded frame-loss metric is recoverable with a
one-line PX4IO firmware change and a rebuild, and a receiver purchase is **not** required for
frame loss. It is still required for RSSI and link quality, which SBUS does not carry in any
form.
