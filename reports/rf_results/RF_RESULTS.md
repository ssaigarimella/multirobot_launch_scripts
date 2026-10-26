# What the existing radio data can already support

**Date:** 2026-08-25 analysis of the 2026-08-25 bench run. Analysis only. Nothing was
collected, changed, flown, flashed, or installed.
**Run under analysis:** 802.11s mesh point, 2437 MHz, 20 MHz, delta and ghost, mesh IPs
192.168.77.1 and .3, Zenoh over the mesh, both stationary at about 3 m.
**Comparison data:** 10 earlier `wifi_rf_*` files on the same adapter and driver in managed
mode at 5765 MHz.

Figures in `figures/`. Underlying data in `mesh_signal_settled.csv` and
`lora_interarrival.csv`.

---

## Timeline established first (it changes several answers)

| Stream | Window (UTC) | Duration | Rows |
|---|---|---|---|
| delta WiFi mesh | 21:23:13 to 21:38:35 | 922 s | 886 |
| ghost WiFi mesh | 21:23:16 to 21:45:33 | 1337 s | 1285 |
| ghost LoRa (run 2) | 21:23:10 to 21:45:33 | 1343 s | 7136 |
| delta LoRa (run 2) | **21:38:23 to 21:38:35** | **13 s** | 134 |
| delta LoRa (run 1) | 21:10:14 to 21:19:07 | 533 s | 5579 |
| ghost LoRa (run 1) | 21:10:21 to 21:20:13 | 592 s | 5884 |

**delta's LoRa node was not running during the mesh window.** Its second file begins at
21:38:23, twelve seconds before delta was torn down. The established facts describe this as
"delta's LoRa file covers 13 s because tab 7 started late"; the stronger statement supported
by the data is that delta contributed **no** LoRa receive data overlapping the mesh window at
all. All LoRa results below are ghost only.

This does not invalidate ghost's LoRa data: the TDMA schedule runs on the ESP32 boards
independently of the ROS bridge, so ghost kept receiving from delta's board throughout.

---

## T1. The delta / ghost asymmetry: ruled OUT, but not for the expected reason

> **Contradiction with the task framing, reported verbatim.** The task asks whether the low
> samples "cluster at the start of the file, which would make them mesh formation transient".
> **They do not.** delta has exactly **one** sample below -60 dBm and it occurs at
> **t+733 s**, twelve minutes into a fifteen minute run, not at the start.

| | delta | ghost |
|---|---|---|
| mesh samples | 852 | 869 |
| samples below -60 dBm | **1** (at t+733 s) | **0** |
| minimum | -76 dBm | -53 dBm |
| minimum excluding that one sample | **-59 dBm** | -53 dBm |
| mean | -40.80 dBm | -40.15 dBm |
| mean excluding that one sample | -40.76 dBm | (unchanged) |
| 1st percentile | -57 dBm | -51 dBm |

**Verdict: the -76 dBm against -53 dBm asymmetry is one isolated sample, not a link property.**
Removing it moves delta's mean by 0.04 dB. The two links are statistically equivalent in the
mean, with delta showing a slightly heavier lower tail (p1 -57 against -51).

The sample is **real, not a repair artifact.** It appears verbatim once in the raw pre repair
file, both antenna chains dropped together (`-76|-76`), and the driver running average moved
with it (-44 dBm). A parse error would not move the running average or both chains coherently.
It is a genuine single second deep fade.

**Quote instead of the raw minimum:** delta -59 dBm, ghost -53 dBm as settled state minima, or
better, the 1st percentiles, -57 and -51 dBm. Report the -76 dBm sample separately as a single
observed deep fade event.

**There is, separately, a mild formation effect,** which is what the task was reaching for:
delta's first 60 s mean is -44.96 dBm against -40.50 dBm afterward, about 4.5 dB. It does not
produce the -76 dBm sample. All settled window statistics below therefore exclude t < 60 s.

---

## T2. Mesh link, settled window

Settled window is t >= 60 s after peering. Samples where `signal_dbm` is 0 are excluded
throughout: 0 is the driver's "no frame received since the last read" sentinel, not a
measurement.

| Quantity | delta | ghost |
|---|---|---|
| samples / duration | 795 / **826 s** | 811 / **843 s** |
| signal mean, SD | -40.50, 4.09 dB | -40.11, 3.22 dB |
| signal p5 / p50 / p95 | -49 / -40 / -36 dBm | -50 / -39 / -38 dBm |
| signal min / max | -76 / -35 dBm | -53 / -36 dBm |
| interchain spread, mean | **10.39 dB** (max 18) | **18.65 dB** (max 30) |
| tx bitrate mean | 39.9 Mbit/s | 39.3 Mbit/s |
| tx bitrate range | 9 to 54 Mbit/s, 7 distinct | 2 to 54 Mbit/s, 11 distinct |
| rate changes | 178 in 826 s = **12.9 / min** | 180 in 843 s = **12.8 / min** |
| sustained tx | 1.7 pkt/s, 0.2 KiB/s | 1.4 pkt/s, 0.1 KiB/s |
| sustained rx | 20.8 pkt/s, 1.0 KiB/s | 20.1 pkt/s, 1.0 KiB/s |

Three results worth stating explicitly.

**The mesh never leaves legacy OFDM rates.** Every observed bitrate is a legacy 802.11a/g rate
and the maximum is 54 Mbit/s. No HT or VHT rate appears in 1606 settled samples on either
drone. This is consistent with the bring up script setting no channel width or HT parameters
on the mesh join. **The 54 Mbit/s ceiling is a configuration property, not a channel property.**

**Rate adaptation is busy for a static link:** about 13 changes per minute at a fixed 3 m
separation with a stationary pair.

**There are recurring structured fades, distinct from the single outlier.** Taking a threshold
8 dB below each drone's median:

| | delta | ghost |
|---|---|---|
| fade events | 6 in 826 s (0.4 / min) | 17 in 843 s (1.2 / min) |
| duration mean / max | 6.9 s / 16 s | 4.8 s / 13 s |
| depth mean / max below median | 20.2 / 36 dB | 11.6 / 14 dB |
| time below threshold | 5.0 % | 9.7 % |

Multi second, 10 to 20 dB fades on a static bench pair at 3 m are the most operationally
interesting feature in this dataset and are visible directly in Figure 1. Cause is not
established: candidates include 2.4 GHz interference from the co located Bluetooth radio, from
the building, from mesh beaconing collisions, or antenna pattern nulls. **Nothing in this data
distinguishes them.**

Figures: `fig1_mesh_signal_timeseries.png`, `fig2_mesh_settled_distributions.png`.

---

## T3. Mesh at 2437 MHz against bench AP at 5765 MHz, same adapter and driver

| | delta AP | delta mesh | ghost AP | ghost mesh |
|---|---|---|---|---|
| valid samples | 149 | 795 | 841 | 811 |
| signal mean, SD | -32.03, **2.30** | -40.50, **4.09** | -31.79, **3.60** | -40.11, **3.22** |
| signal min / max | -40 / -29 | -76 / -35 | -41 / -19 | -53 / -36 |
| bitrate mean | **796.9** Mbit/s | **39.9** Mbit/s | **677.7** Mbit/s | **39.3** Mbit/s |
| bitrate max | 867 | 54 | 867 | 54 |
| mesh / AP bitrate ratio | **0.050** | | **0.058** | |

61 of 210 delta AP rows and 51 of 892 ghost AP rows were dropped as `signal == 0` sentinels.

**The mesh link runs at about 5 to 6 percent of the infrastructure link's negotiated rate on
identical hardware.** Signal is about 8 to 9 dB lower on the mesh, and signal stability is
worse on delta (SD 2.30 to 4.09 dB) but slightly better on ghost (3.60 to 3.22 dB), so the
stability comparison is not consistent across the pair and should not be claimed as a result.

**What is not controlled, and it is a lot.** Band (5765 against 2437 MHz), channel width
(80 against 20 MHz), and rate set (VHT MCS9 NSS2 against legacy OFDM) all differ, as does the
role (station to AP against peer to peer). Traffic differs too: the AP link carried ordinary
bench traffic, the mesh carried about 1 KiB/s of Zenoh. **The bitrate ratio is very largely a
statement about channel width and HT/VHT configuration, not about 802.11s.** A defensible
version of this comparison requires the mesh brought up with HT enabled at the same width, or
the AP forced to 20 MHz legacy. Present it as a configuration comparison, not a mode
comparison.

Figure: `fig3_mesh_vs_ap.png`.

---

## T4. LoRa receive statistics, ghost, 1343 s

| Quantity | Value |
|---|---|
| pose packets | 7069 from peer node 0 |
| RSSI mean, SD | -32.81, 1.10 dBm |
| RSSI p5 / p50 / p95 | -35 / -33 / -31 dBm (12 distinct values) |
| SNR mean, SD | 12.60, 0.45 dB |
| SNR p5 / p50 / p95 | 12.0 / 12.5 / 13.2 dB (17 distinct values) |

Stability across thirds of the run: RSSI means -32.90, -32.74, -32.80; SNR means 12.57, 12.56,
12.67. **Both are stationary over 22 minutes.** No drift.

### Interarrival against the 190 ms TDMA cycle

| Quantity | Value |
|---|---|
| intervals | 7068 |
| mean / median | **189.98 / 190.00 ms** |
| SD (all) | 2.04 ms |
| **jitter, SD within one cycle band** | **1.22 ms** |
| interquartile range | **0.00 ms** |
| p1 / p5 / p95 / p99 | 186 / 188 / 192 / 193 ms |
| min / max | 53 / 209 ms |
| max absolute deviation from 190 ms | 19.0 ms |
| arrivals within 0.5 to 1.5 cycles | **7067 of 7068 = 100.0 %** |
| intervals at 2 or more cycles | **0** |
| longest gap | 0.2 s |

**Arrivals are consistent with a bounded latency TDMA schedule, and tightly so.** The median
interval is exactly the nominal 190 ms, the interquartile range is zero, jitter is 1.22 ms, and
the worst single deviation observed in 22 minutes is 19.0 ms, which is inside the 40 ms slot.

**A missed transmission in a deterministic TDMA schedule appears as an interval of two or more
cycles. Zero such intervals occurred in 7068 consecutive intervals.** This is a statement about
schedule continuity that needs no sequence number. By the rule of three, zero events in 7068
trials bounds the per cycle miss rate at about **424 ppm at 95 percent confidence.** State it
that way. It is **not** a packet delivery ratio, because without a sequence number a cycle in
which the transmitter did not transmit is indistinguishable from one in which it did and the
packet was lost.

Figure: `fig4_lora_tdma.png`.

---

## T5. The receive rate proxy: uninterpretable, not for publication

`link_*.csv` `delivery` is **not a ratio.** Across ghost's 1343 rows it takes exactly two
values: 0.0 in 68.0 percent of rows and 1.0 in 32.0 percent. **Zero rows fall strictly between.**
It is a binary flag.

Its structure is a single contiguous block:

| | |
|---|---|
| zero run | t+0 s to t+912 s, one contiguous run, 913 rows |
| one run | t+913 s to t+1342 s, 430 rows |
| LoRa pose packets arriving during the zero run | **4799, at 5.26 packets per second** |

The link was fully healthy for the entire period the column reads zero. And t+913 s after
ghost's 21:23:11 start is **21:38:24**, which is when delta's LoRa node came up (21:38:23).

**The 0.320 is 100 percent attributable to delta's tab 7 start time.** The column tracks
whether the peer ROS node is publishing, not whether RF packets are being delivered.
`rx_hz` is similarly not the LoRa rate: its mean over the nonzero period is 19.40 Hz and its
maximum 66.96 Hz, against an actual LoRa arrival rate of 5.26 Hz.

**Mark 0.32 as not for publication.** The defensible replacement for the same claim is the
T4 result: zero missed cycles in 7068, bounding the miss rate at about 424 ppm.

Figure: `fig5_delivery_proxy_artifact.png`.

---

## T6. Cross radio observation, ghost only

Both radios active simultaneously for 903 s on ghost, where the join is exact (both stamp
CLOCK_REALTIME on one machine). 869 paired 1 Hz samples, mean 5.27 LoRa packets per WiFi
sample.

| x | y | Pearson r |
|---|---|---|
| LoRa RSSI | mesh signal | +0.117 |
| LoRa RSSI | mesh tx bitrate | -0.037 |
| LoRa SNR | mesh signal | +0.070 |
| LoRa SNR | mesh tx bitrate | +0.068 |
| LoRa packets per second | mesh tx bitrate | -0.027 |
| LoRa packets per second | mesh signal | -0.024 |

**No covariance detected. Every coefficient is below 0.12.**

**This is an observation at one uncontrolled operating point, not a coexistence result, and it
is close to uninformative by construction.** Confounds, all of which push toward finding
nothing:

1. **The mesh was almost idle.** About 1 KiB/s and 20 packets per second. Duty cycle is tiny,
   so there was very little 2.4 GHz energy to couple into anything.
2. **The bands do not overlap.** LoRa is 915 MHz, the mesh 2437 MHz. Direct interference is not
   expected; only front end desensitization or shared supply effects could couple them.
3. **Both links had large margin.** LoRa SNR 12.6 dB and mesh signal -40 dBm at 3 m. Neither is
   near threshold, so degradation has nowhere to show.
4. **LoRa arrival rate is nearly constant by construction** (5 or 6 packets per bin), so it has
   almost no variance to correlate against.
5. delta contributed nothing, so this is single sided.

A null result here does **not** support a claim that the two radios do not interact. It
supports only that no interaction was observable under a near idle mesh at high margin.

Figure: `fig6_cross_radio.png`.

---

## T7. Figures produced

| File | Supports | Content |
|---|---|---|
| `fig1_mesh_signal_timeseries.png` | T1, T2 | Per sample and running average signal for both drones, formation window shaded, the single deep sample marked |
| `fig2_mesh_settled_distributions.png` | T2 | Settled signal histogram, interchain spread, rate selection |
| `fig3_mesh_vs_ap.png` | T3 | Signal CDF and mean bitrate, mesh against AP, with the uncontrolled variables stated on the figure |
| `fig4_lora_tdma.png` | T4 | Interarrival histogram against the 190 ms cycle, RSSI and SNR time series |
| `fig5_delivery_proxy_artifact.png` | T5 | The delivery column against actual LoRa arrivals, with delta's tab 7 start marked |
| `fig6_cross_radio.png` | T6 | LoRa RSSI against mesh signal and against mesh bitrate |

Captions and labels use ASCII hyphens only, no em dashes, no en dashes, no unicode minus,
American spelling. Underlying data: `mesh_signal_settled.csv` (1721 rows),
`lora_interarrival.csv` (7069 rows).

---

## T8. Gap list

### Publishable from data already on disk

- LoRa RSSI and SNR distributions and their stationarity over 22 minutes, ghost, one geometry.
- LoRa TDMA schedule convergence: 190.00 ms median, 1.22 ms jitter, zero missed cycles in 7068,
  miss rate bounded at about 424 ppm at 95 percent confidence.
- Mesh per peer signal distribution and rate selection at one geometry, both drones, about
  830 s each.
- The recurring fade characterization (counts, durations, depths).
- The mesh against AP configuration comparison, provided it is framed as width and rate set
  rather than as 802.11s against infrastructure.

### Missing, collectable tomorrow with current firmware

1. **Distance sweep, both radios.** One static point at 3 m supports no path loss curve and no
   range claim. Need marked stations at 5, 10, 25, 50, 100, 200, 300 m, both radios logging
   simultaneously, minimum 150 s dwell per station (10,000 LoRa cycles at 5.26 Hz gives a
   100 ppm floor; 300 s is better). Outdoor line of sight, tape or wheel measured, antennas at
   fixed height and orientation, recorded. This is the single highest value missing run.
2. **Both drones logging LoRa simultaneously.** delta's tab 7 must be up before tab 8. The
   entire mesh window has one sided LoRa data, so no link symmetry claim is possible.
3. **Mesh throughput.** Never measured. iperf3 3.9 is installed on both. Must be a dedicated
   idle bench measurement, TCP and UDP, 60 s each, with the SLAM stack down, since it perturbs
   the link it measures. Record separation and antenna orientation.
4. **Mesh with HT enabled.** Every sample is legacy OFDM capped at 54 Mbit/s. Until the mesh is
   brought up at a comparable width, the mesh against AP comparison is a configuration artifact.
   Requires changing the mesh join parameters, which is a change to a script owned by another
   session, not an analysis task.
5. **A loaded coexistence point.** T6 found nothing because the mesh was idle. Need the matrix
   actually run: coordination only, mesh only, both active, with the mesh carrying real load
   and at least one condition where a link has low margin. Three rows, at least 120 s each.
6. **Repeat measurements.** Every number here is n = 1 run. No error bars across runs exist.

### Blocked until the LoRa instrumentation lands (prompt 2)

7. **Per packet delivery ratio.** Needs a sequence number in `PeerPose.msg`. The cycle
   continuity bound in T4 is the best available substitute and is weaker: it cannot separate
   "transmitter did not send" from "packet was lost."
8. **One way latency.** Needs a TX timestamp in the packet forwarded to ROS. Both
   `Packet.sequence` and `Packet.uptime_ms` already travel over the air and are discarded at
   the firmware to ROS serial boundary, so this is a serial frame change with zero on air cost.
9. **PDR and latency against RSSI across a TX power sweep.** Needs the sequence number, a TX
   power field, and a runtime set power command. Currently one reflash per power step.

### Not obtainable with current hardware at all

10. **2.4 GHz channel occupancy and noise floor.** rtw88 implements no `.get_survey`, so
    `iw survey dump` returns nothing on any band or mode. Needs an SDR, about 40 dollars.
    Without it the cause of the recurring fades in T2 cannot be established.
11. **WiFi retry and failure counts.** `tx_retries` and `tx_failed` are structurally zero
    because rtw88 never fills `info->status.rates[]`. No configuration recovers them.

### Cross drone timing caveat that applies to all of the above

Same drone joins are exact. Cross drone is bounded at roughly plus or minus 25 to 50 ms
because the two drones discipline against different public NTP servers on a 34 minute poll.
Adequate for 1 Hz correlation, not for per packet cross drone alignment. Pointing both drones
at one local NTP server with a short poll interval would reduce this to about 1 to 2 ms and
costs nothing.
