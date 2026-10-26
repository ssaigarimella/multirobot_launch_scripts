# WiFi mesh link characterization and two-radio coexistence harness

> ## ⚠ CORRECTION (2026-08-21, after checking the actual launch path)
>
> **The premise this report was written against is wrong. The 802.11s mesh has never been
> used in a real run.** Everything below about "the flight configuration is 2437 MHz" is
> incorrect and is retained only so the reasoning is auditable.
>
> The flown transport is the **bench AP LAN** — TP-Link, managed mode, 5765 MHz, static IPs
> (delta .40, ghost .50). Verified:
> * `launch_fleet_tmux.sh` and `fleet_ctl` contain **zero** references to `2437`, `mesh join`,
>   or `launch_mesh_zenoh` (grep count 0 and 0).
> * Tab 8 of the fleet launcher runs `launch_bench_zenoh.sh`, whose header states it
>   "makes NO network changes whatsoever: no mesh join, no nmcli, no interface
>   reconfiguration, no iptables."
> * `fleet_ctl:206` — "bench zenoh bridge on each (LAN transport, no network switch)".
> * Confirmed independently by two peer sessions: the mesh script exists but has never been
>   invoked in a bench or field run.
>
> **Consequences:**
> 1. The original RF audit's managed-mode 5765 MHz measurements were measuring the *real*
>    flown link all along. They should not have been discarded.
> 2. `coex_harness.sh` in this directory solves a problem the fleet does not have — it brings
>    the mesh up itself, which is what forced the dead-man/second-path machinery and then
>    blocked on Delta. **It is superseded by `rf_stats.sh` in the repo root**, a passive
>    read-only sampler that runs alongside the normal launch flow and changes nothing.
>    `coex_harness.sh` is kept only for the day the mesh is actually flown.
> 3. T1/T2 (rtw88 has no `.get_survey`; only `tx bitrate` is driver-computed) are unaffected —
>    they are source-level facts, independent of band and mode, and they still hold.
> 4. Delta having no second path is moot for normal operation: nothing in the fleet flow
>    touches its WiFi. Standing project rule — never reconfigure the drones' WiFi.


**Date:** 2026-08-21
**Drones reachable:** Delta, Ghost. Buckshee and Thunderstrike unreachable for a third
consecutive session (`No route to host`).
**Vehicle state changed:** none persisting. No interface was placed in mesh mode. See
"Part 2 — not run" and the one incident recorded under "Harness verification".

---

## Executive summary

| Task | Status |
|---|---|
| T0 mode verdict | **802.11s mesh point.** Settled. Not IBSS. |
| T1 survey callback | **Absent from rtw88 entirely.** Settled from source — not band-specific. |
| T2 station fields | rtw88 computes **one** field. The rest are mac80211 generic or unpopulated. |
| T3 iperf3 | Present on both, 3.9. |
| **T4–T8 (Part 2)** | **NOT RUN — blocked.** Delta has no second path; a human presence requirement cannot be met by an unattended session. |
| T9–T12 (harness) | **Written, and its safety machinery verified.** Cannot execute the matrix until Part 2 unblocks. |

---

## T0 — WiFi mode verdict (the gate)

### Verdict: **802.11s mesh point (`iw ... set type mp` + `mesh join`), 2437 MHz, mesh ID `jetson-drone-mesh`.**

**Not IBSS. Nothing anywhere configures IBSS.** Both modes are *not* configured somewhere —
only one is.

**Evidence 1 — every bring-up script agrees.** All four use the identical sequence:

| Script | Mode command | Join |
|---|---|---|
| `kernel_build/test_mesh_pair.sh:140,162` | `iw dev "$IFACE" set type mp` | `mesh join "$MESH_ID" freq "$MESH_FREQ"` |
| `radiohive/scripts/test_radiohive_mesh.sh:284,302` | `set type mp` | `mesh join … freq 2437` |
| `radiohive/scripts/test_radiohive_pair.sh:243,261` | `set type mp` | `mesh join … freq 2437` |
| `radiohive/scripts/test_radiohive_trio.sh:260,278` | `set type mp` | `mesh join … freq 2437` |
| `multi_drone_nvblox/scripts/launch_mesh_zenoh_v2.sh:265,278` | `set type mp` | `mesh join … freq 2437` |

All define `MESH_ID="jetson-drone-mesh"`, `MESH_FREQ="2437"  # Channel 6, 2.4 GHz`. Teardown is
`iw dev "$IFACE" mesh leave`.

A repo-wide search for a *configuration* of IBSS returns nothing:
```
grep -rn -e 'set type ibss' -e 'ibss join' -e 'type adhoc' -e 'mode=ad-hoc' <workspace>/src
  (no output)
```

**Evidence 2 — runtime confirms mesh support is actually compiled in.** The loaded
`mac80211.ko` carries 124 mesh symbols, and `iw phy` offers `mesh point`:
```
/lib/modules/5.15.148-tegra/kernel/net/mac80211/mac80211.ko
kallsyms mesh symbols: 124
ieee80211_mesh_build_beacon   ieee80211_mesh_csa_beacon   ieee80211_mesh_finish_csa
ieee80211_mesh_housekeeping_timer  ieee80211_mesh_init_sdata  ieee80211_mesh_path_root_timer
```

**Evidence 3 — the kernel rebuild claim is corroborated, and `/proc/config.gz` is a trap.**
`/proc/config.gz` reports:
```
# CONFIG_MAC80211_MESH is not set
# CONFIG_RTW88 is not set
```
…while both `rtw88_8822ce` and mesh-capable `mac80211` are loaded and working. That config
describes the **stock JetPack kernel image**, not the out-of-tree modules that replaced it.
This matches `AGENT_HANDOFF.md:339-346` exactly: `/mnt/nova_ssd/workspaces/kernel_build/`
rebuilt `mac80211.ko` with `CONFIG_MAC80211_MESH=y` and patched in-kernel rtw88 to add the
MESH_POINT iftype; the vendor `rtl8822ce` is blacklisted. Originals are in
`kernel_build/backup_modules/` with `04_rollback.sh`.

> **Do not use `/proc/config.gz` to reason about this fleet's wireless capability.** It will
> tell you mesh and rtw88 are absent. Both are present as replaced modules.

**No systemd unit brings the mesh up.** It is script-invoked and non-persistent, which is why
both drones sit in `managed` mode on the bench.

### Documents to correct

The premise that "other project documents refer to IBSS ad-hoc" resolves to **one stale
sentence in one superseded paper draft**, not to a body of documentation:

| Document | Line | States | Action |
|---|---|---|---|
| `~/.local/share/Trash/files/main3.tex` | 487 | `Map-sharing link & Onboard WiFi adapter, ad-hoc mode (bulk map data)` | **The origin of the claim.** Already trashed and superseded — no action beyond not resurrecting it. |
| `jfr_codefacts.md` | 672 | quotes the above verbatim as `[DOC] … /home/ssg/Downloads/main3.tex` | **Correct this citation.** It cites a path that no longer exists and propagates the wrong mode. Repoint to **`main_JFR_v2.tex:692`** (see the mtime note below). |

Already correct, no action needed:

| Document | Line | States |
|---|---|---|
| `Downloads/main_JFR_v2.tex` | 692 | `Map-sharing link & Onboard 802.11 WiFi adapter in mesh mode` ✓ **(newest — cite this)** |
| `Downloads/main_JFR_v5.tex` | 694 | `…in mesh mode` ✓ (older despite the higher version number) |

> **Draft version numbers do not track recency here.** Verified by mtime and md5:
> ```
> 90527  2026-08-18 17:27  main_JFR_v2 (1).tex   md5 34aa0c28…  <- byte-identical to v2
> 90527  2026-08-17 17:45  main_JFR_v2.tex       md5 34aa0c28…  <- NEWEST real draft
> 90527  2026-08-10 15:38  main_JFR_v5.tex       md5 df778671…  <- 7 days OLDER
> ```
> `main_JFR_v2.tex` is a week newer than `main_JFR_v5.tex`. An earlier version of this
> report recommended citing v5 on the strength of its version number; that was wrong.
> Both drafts state "in mesh mode", so the T0 finding is unaffected — only the citation
> target changes.
| `jfr_codefacts.md` | 423 | `Fact: 802.11s mesh (not ad-hoc/IBSS), mesh ID jetson-drone-mesh, MESH_FREQ="2437"` ✓ |
| `AGENT_HANDOFF.md` | 339-346 | 802.11s + kernel patch ✓ |

Note `jfr_codefacts.md` contradicts **itself**: line 423 is right, line 672 quotes a draft that
was wrong. Only line 672 needs the fix.

`reports/rf_audit/RF_INSTRUMENTATION_AUDIT.md:529,638` contain the string `IBSS` **only** as
part of a verbatim `iw phy … Supported interface modes` capability listing. That is a list of
what the chip *can* do, not a claim about what the fleet *does*. No correction needed — but it
is the most likely source of the confusion, since that listing shows `IBSS` and `mesh point`
one line apart.

---

## Part 1 — capability read (no state change)

### T1 — does rtw88 implement the survey callback?

**No. Not for 2.4 GHz, not for 5 GHz, not at all. There is no `get_survey` in the driver.**

`rtw_ops` (`…/rtw88/mac80211.c:858`) implements exactly these callbacks:
```
add_interface  ampdu_action  bss_info_changed  can_aggregate_in_amsdu  change_interface
config  configure_filter  conf_tx  flush  get_antenna  mgd_prepare_tx  reconfig_complete
remove_interface  resume  set_antenna  set_bitrate_mask  set_key  set_rts_threshold
set_tim  set_wakeup  sta_add  sta_rc_update  sta_remove  start  start_ap  sta_statistics
stop  suspend  sw_scan_complete  sw_scan_start  tx  wake_tx_queue
```
`.get_survey` is not among them, and `grep -rn survey` across every `.c`/`.h` in the rtw88 tree
returns **nothing**.

What that means when `iw survey dump` is called — `net/mac80211/driver-ops.h:602`:
```c
static inline int drv_get_survey(struct ieee80211_local *local, int idx,
				struct survey_info *survey)
{
	int ret = -EOPNOTSUPP;
	trace_drv_get_survey(local, idx, survey);
	if (local->ops->get_survey)
		ret = local->ops->get_survey(&local->hw, idx, survey);
	...
}
```
With no `get_survey`, the call returns `-EOPNOTSUPP` and `iw` prints nothing.

**This settles the established-fact caveat.** The prompt states the empty `survey dump` "was
measured on the managed 5 GHz link [and] has to be retried in the real mode and band before the
driver can be called incapable." The retry cannot change the outcome: the return value is
decided by a NULL function pointer, before any band or interface-mode logic is reached. The
driver **can** be called incapable, on source evidence, without Part 2.

### T2 — which station fields the driver populates vs stubs

rtw88's entire `sta_statistics` implementation (`mac80211.c:694-701`):
```c
static void rtw_ops_sta_statistics(struct ieee80211_hw *hw,
				   struct ieee80211_vif *vif,
				   struct ieee80211_sta *sta,
				   struct station_info *sinfo)
{
	struct rtw_sta_info *si = (struct rtw_sta_info *)sta->drv_priv;

	sinfo->txrate = si->ra_report.txrate;
	sinfo->filled |= BIT_ULL(NL80211_STA_INFO_TX_BITRATE);
}
```
**One field.** Everything else in `station dump` comes from mac80211's generic accounting, and
whether it is meaningful depends on what rtw88 feeds it:

| Field | Source | Populated? |
|---|---|---|
| `tx bitrate` | **driver-computed** — `sta_statistics`, the only field it fills | **Yes, real** |
| `signal`, `signal avg` | **driver-fed**: `rx.c:186` `rx_status->signal = pkt_stat->signal_power`; mac80211 averages | **Yes, real** |
| `rx bitrate` | mac80211, from the rate rtw88 sets in `rx_status` | **Yes, real** |
| `tx retries` | mac80211 accumulates `info->status.rates[]`. rtw88 sets **only** the binary `IEEE80211_TX_STAT_ACK` flag (`tx.c:202-206`, `pci.c:1034-1037`) and never fills the rates array | **No — effectively stubbed, pinned at 0** |
| `tx failed` | same path as above | **No — pinned at 0** |
| `beacon loss` | mac80211 managed-mode beacon-miss counter; no driver input | **Not meaningful in mesh** (no AP beacons to miss) |
| `expected throughput` | requires `.get_expected_throughput` in the ops table — **absent** | **No — never populated** |

This explains the earlier observation that `tx retries`/`tx failed` stayed at exactly 0 while
`tx packets` incremented, and — like survey — it is a property of the driver, **not** of the
band or interface mode. Switching to 2437 MHz mesh will not populate them.

### T3 — iperf3

| Drone | Version |
|---|---|
| Delta | `iperf 3.9 (cJSON 1.7.13)` |
| Ghost | `iperf 3.9 (cJSON 1.7.13)` |

---

## Part 2 — NOT RUN. Blocked on the connectivity precondition.

The prompt's own gate: *"Confirm a second independent path to each drone that does not use the
wlan interface… If no second path exists on a drone, **do not change that drone's interface.
Stop and report.**"*

**Delta has no second path.** Stopping and reporting, as instructed.

| Drone | SSH session lands on | Wired `enP8p1s0` | Second path | Safe to switch? |
|---|---|---|---|---|
| **Delta** | `wlP1p1s0` (192.168.0.40) | carrier up, **NetworkManager state `activating`, no IPv4 ever assigned** | **NONE** | **NO** |
| **Ghost** | `wlP1p1s0` (192.168.0.50) | **`192.168.0.206`, SSH confirmed working** | yes — wired | yes |

Every alternative path for Delta was tested and failed:

1. **Wired IPv4** — `enP8p1s0` has no address; NetworkManager reports it stuck `activating`.
   Assigning one would be "changing networking configuration", which the constraints forbid.
2. **IPv6 link-local over the bench ethernet** — `ping6 ff02::1` on the laptop's ethernet
   enumerates the whole segment:
   ```
   fe80::88c3:d323:5de2:bbc5   laptop itself
   fe80::f6dd:bc89:91cf:8c83   ghost enP8p1s0   <-- ghost's wired NIC answers
   fe80::e6fa:c4ff:fe57:73a    TP-Link router
   fe80::7fae:bd51:ee2b:a7c8   ghost wlP1p1s0
   fe80::e76d:ff5a:7a7b:e615   delta wlP1p1s0   <-- only delta's WiFi answers
   ```
   **Delta's `enP8p1s0` (`fe80::c5e6:a607:2e4d:598c`) does not answer at all**, and a direct
   SSH attempt to it times out. Its carrier is up but it is not on this L2 segment.
3. **Tailscale** — no drone is on the tailnet (only `ssg-dellxps`, `aellnar08227l`,
   `arpg-machine`, `soff`).
4. **Serial console** — the laptop's only serial device is a fingerprint reader
   (`usb-HTMicroelectronics_Goodix_Fingerprint_Device`). Delta's own FTDI and CP2102 are the
   PX4 and LoRa links *out of* the Jetson, not a console *into* it.

### A second, independent hazard found in the bring-up script

`test_mesh_pair.sh:19` hardcodes its restore target:
```bash
NM_CONNECTION="eduroam"
```
and its teardown (lines 63-74) ends with `nmcli connection up "$NM_CONNECTION"`. **The drones
are not on `eduroam`** — the active profile on `wlP1p1s0` is `TP-Link_073A 1`. So the script's
own revert path would restore the *wrong* network. On Ghost that is survivable (wired path). On
Delta — whose only route home is that WiFi — running this script would strand the drone even if
it exited cleanly, because its restore would bring up a network that is not present.

The constraints forbid editing the bring-up script, so the harness works around this by
capturing the **active** connection name at arm time and restoring that instead.

### The third blocker

*"A human must be present at the bench for Part 2. Do not run Part 2 unattended."* This session
is non-interactive; presence cannot be confirmed. Even with Delta's path fixed, Part 2 should
not start until someone confirms they are at the bench.

### To unblock Part 2

1. Give Delta's `enP8p1s0` a working wired path (fix the `activating` NM profile, or plug it
   into the bench switch alongside Ghost's) and confirm `ssh delta@<wired-ip>` works **while
   `wlP1p1s0` is down**.
2. Confirm a human is at the bench.
3. Re-run `./coex_harness.sh preflight` — it must print `OK` for both drones.
4. Record separation distance and antenna orientation, then `SEPARATION_M=… ANTENNA=… ./coex_harness.sh run`.

### T4, T5, T7, T8 — not measured

No peering was established, so peer MACs/count/60-second survival (T4), the 120-second 1 Hz
signal series (T5), TCP/UDP throughput, retransmits and jitter (T7), and the before/after
counter monotonicity check (T8) have **no data**. They are not reported as zero or inferred
from the 5 GHz bench link — they are simply not measured. The earlier audit's mistake was
reasoning about the mesh from managed-mode data; that is not repeated here.

### T6 — survey fields, answered from source rather than by retry

The empirical retry T6 asks for could not be run. The source answer is definitive and
band-independent (T1):

| Field T6 asks about | Returned in mesh mode at 2437 MHz | Why |
|---|---|---|
| `noise` | **absent** | `.get_survey` is NULL → `drv_get_survey` returns `-EOPNOTSUPP` |
| `channel active time` | **absent** | same |
| `channel busy time` | **absent** | same |
| `channel receive time` | **absent** | same |
| `channel transmit time` | **absent** | same |
| `frequency` | **absent** | same — `iw` emits nothing at all |

**Channel busy time is not present, therefore channel occupancy is NOT measurable without
additional hardware.** The RF audit's conclusion on this row stands, and is now supported by
source rather than by a single measurement in the wrong band. Corroborating runtime evidence:
`/proc/net/wireless` reports `noise = -256` (the "unset" sentinel) on both drones.

---

## Part 3 — coexistence harness

`coex_harness.sh` in this directory. Written and its safety machinery verified; the matrix
itself cannot run until Part 2 unblocks.

### T9 — the three-row matrix

Rows: `coordination_only`, `mesh_only`, `both_active`. Per row it records coordination-link
delivery ratio, mesh throughput (TCP and UDP), and per-peer signal sampled at 1 Hz, all stamped
on a common timebase. Mesh bring-up is delegated to the **existing** `test_mesh_pair.sh`; the
harness never edits it.

**One of the three metrics has no source.** Coordination-link delivery ratio requires a
sequence number at the receiver. The LoRa instrumentation change (prompt 2) has **not been
run** — verified: `firmware/lora_mesh_v7_2_ros2.ino` is unchanged since Jul 1, `sendPeerPose`
still emits a 23-byte frame, `PeerPose.msg` still carries only
`node_id/latitude/longitude/altitude/heading/rssi/snr`, and `grep -c 'MSG_SET_TXPOWER|setOutputPower'`
returns `0`. The harness therefore emits the sentinel `UNAVAILABLE_NO_SEQ_FIELD` for that
column and auto-detects the field's arrival, rather than fabricating a ratio.

### T10 — timebase and its uncertainty

**`chronyc` is not installed on either drone** — `which chronyc` returns nothing on both. T10's
stated method is unavailable; this is reported rather than worked around. Time sync is
`systemd-timesyncd` against **different** public `pool.ntp.org` servers, poll interval pinned at
34 min 8 s.

The harness uses an SSH round-trip midpoint estimate, best-of-5:
`offset = t_remote − (t_before + t_after)/2`, uncertainty `±rtt/2`.

**Measured uncertainty: ±5.5 to ±8 ms per sample. Cross-drone alignment is ~±10 ms at best,
degrading to ~±25–50 ms late in a poll interval** (NTP jitter 4.991 ms on Delta, 12.008 ms on
Ghost; Ghost's root delay 118 ms; ~10–20 ppm crystal drift over a 2048 s poll interval).

**Stated plainly, as T10 requires:** the drones are **not** synchronized well enough for any
sub-100 ms attribution. That is adequate for row-level aggregation at 1 Hz — the matrix's actual
resolution — and **not** adequate to attribute a mesh throughput dip to an individual 40 ms LoRa
TDMA burst. Anyone reading the matrix should treat each row as a 120-second average, not as a
time-aligned trace. Fixing this needs no hardware: point both drones at one local NTP server
with a short poll interval.

### T11 — CSV schema

`coex_matrix.csv`, one row per matrix row:
```
row, t_start_utc, t_end_utc, duration_s,
coord_tx, coord_rx, coord_delivery_ratio, coord_status,
mesh_tcp_mbps, mesh_udp_mbps, mesh_udp_jitter_ms, mesh_udp_loss_pct, mesh_retransmits,
peer_signal_dbm_mean, peer_signal_dbm_min, peer_signal_dbm_max, peer_signal_n, peer_count,
clock_offset_delta_ms, clock_offset_ghost_ms, clock_method,
separation_m, antenna_orientation
```
`separation_m` and `antenna_orientation` are passed in by the operator and default to
`UNRECORDED` — the harness will not silently omit them.

### T12 — restore on exit, error and interrupt: verified

Cleanup is trapped on `EXIT INT TERM`. Verified by running the dead-man selftest and killing it:

```
=== SIGINT mid-run ===
[19:03:47]   (a) confirming it does NOT fire while heartbeating
[19:04:00] CLEANUP: restoring drones to prior configuration (rc=0, changed:  ghost)
[19:04:09]   ghost restored

=== SIGTERM mid-run ===
[19:04:24] CLEANUP: restoring drones to prior configuration (rc=0, changed:  ghost)
[19:04:34]   ghost restored

=== ghost state after both kills ===
  ssid TP-Link_073A
  type managed
  channel 153 (5765 MHz), width: 80 MHz, center1: 5775 MHz
  deadman gone (good)
  wlP1p1s0  UP  192.168.0.50/24
```

### Dead-man revert — mechanism, timeout, and proof it fires

**Mechanism.** Before any mode change, a detached `setsid nohup` watchdog is written to
`~/wifi_mesh/deadman.sh` on the drone and started. It survives SSH disconnection — which is
essential, because the switch itself kills the session. It polls every 5 s and reverts when
either condition trips:

- **Heartbeat stale > 180 s** (default) — the harness touches `~/wifi_mesh/heartbeat` during
  every sample; if the harness dies, hangs, or loses its link, the file goes stale.
- **Hard deadline 1800 s** from arming — an absolute cap regardless of heartbeats.

Revert action: `mesh leave` → `ip addr flush` → `link down` → `set type managed` → `link up` →
`nmcli device set managed yes` → `nmcli connection up '<active connection captured at arm time>'`.
Cancelled by `~/wifi_mesh/cancel`. All firings are logged to `~/wifi_mesh/deadman.log`.

**Verified by triggering it on purpose**, using a `NOOP` revert action so no interface was
touched:

```
[19:02:18] ghost: arming dead-man (restore 'TP-Link_073A 1', stale=15s, hard=600s)
ARMED
[19:02:19]   (a) confirming it does NOT fire while heartbeating
[19:02:39]   (a) PASS - did not fire while heartbeating
[19:02:39]   (b) stopping heartbeat; it must fire
[19:03:05]   (b) PASS - fired on heartbeat stall
[19:03:05]   (c) cancel path
[19:03:23]   (c) PASS - cancel stops the watchdog
      deadman.log:
      FIRED 2026-08-21T19:02:53Z stale=17 elapsed=35 action=NOOP
      cancelled 2026-08-21T19:03:11Z
[19:03:23] SELFTEST PASSED
```

Note the arm line: the watchdog captured the **real** active profile `TP-Link_073A 1` — not the
`eduroam` that `test_mesh_pair.sh` would have tried to restore.

### Harness verification — one incident, disclosed

The harness's first `run` invocation correctly refused at preflight, **but its cleanup trap then
executed a full interface restore on both drones even though nothing had been changed** — a bug
in my code, which bounced Delta's only link (`ip addr flush` + `nmcli connection up`). Delta
recovered within ~15 s and was verified healthy immediately afterwards: `ssid TP-Link_073A`,
`type managed`, `channel 153`, address `192.168.0.40/24`, default route intact. No lasting
change, but it was an avoidable risk on the one drone that cannot afford it.

Fixed: cleanup now tracks a `MUTATED` flag and only restores drones it actually touched:
```
[18:59:22] CLEANUP: nothing was changed - no restore needed
```

Two further bugs were found and fixed during verification:

1. **`pgrep -f deadman.sh` matched the shell running the check itself**, so the cancel test
   reported a false `FAIL` — the watchdog had in fact stopped. Fixed with the bracket idiom
   `pgrep -f '[d]eadman.sh'`, the same trick `fleet_ctl` already uses for its pkill sweep.
2. **Cleanup hardcoded `nmcli connection up 'TP-Link_073A'`**, but the actual profile name is
   `TP-Link_073A 1`. Now detected from `nmcli -t -f NAME,DEVICE con show`.

---

## What the coexistence table can honestly carry today

Nothing yet. All three rows are unmeasured, and two of the three per-row metrics are blocked
independently of the connectivity problem: coordination-link delivery ratio has no sequence
number to work from until the LoRa instrumentation lands, and channel occupancy is
unmeasurable on this driver at any frequency. Mesh throughput and per-peer signal are both
genuinely measurable — `iperf3` 3.9 is installed on both drones, and `signal` is a real
driver-fed value — but only once Delta has a second path and someone is at the bench. The
harness will collect them the moment preflight passes.
