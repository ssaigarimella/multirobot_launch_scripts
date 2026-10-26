# Corrections required to the existing invention disclosure

**Read before filing.** Verified against the system deployed on the fleet as of 2026-09-25.
Supporting evidence: `reports/rf_results/RF_RESULTS.md`,
`reports/wifi_mesh/MESH_CHARACTERIZATION.md`, `jfr_codefacts_report.md`.

**Read before filing.** The prior disclosure contains technical statements that the deployed
system contradicts. Several are in claim-bearing text.

| # | Disclosure states | Verified system | Severity |
|---|---|---|---|
| 1 | "802.11 WiFi **Independent Basic Service Set, IBSS**", "IBSS configuration", "(ad-hoc WiFi)" | **802.11s mesh point.** `iw ... set type mp` + `mesh join`. A repo-wide and full-git-history search finds **no IBSS configuration anywhere, ever.** IBSS and mesh point are different modes with different peering, beaconing and driver paths | **CRITICAL** |
| 2 | "four nodes at **15 Hz** update rate" | **5.263 Hz** (190 ms cycle), measured 190.00 ms median | **HIGH** |
| 3 | "full cycle completes in about **67 ms**" | **190 ms** = 4 x 40 + 30, measured | **HIGH** |
| 4 | LoRa "low bandwidth (**5 KB/s**)" | **0.80 KB/s** aggregate application payload; 2.73 KB/s PHY ceiling | **HIGH** |
| 5 | WiFi "up to roughly **30 MB/s**" | **39.3 Mbit/s = 4.9 MB/s** mean negotiated rate, and the mesh runs **legacy OFDM capped at 54 Mbit/s** because the join sets no HT/VHT. **Throughput itself has never been measured** (no iperf run has been performed) | **HIGH** |
| 6 | WiFi range "about **30 to 50 m**" | **Never measured.** All data is from a single static bench geometry at ~3 m | **HIGH** |
| 7 | "**loopback multicast enablement** and DDS peer-discovery tuning" | The opposite: multicast is **disabled** (`scouting.multicast.enabled: false`) and DDS is **confined to loopback**, with explicit unicast TCP peers. The mechanism is multicast *avoidance* | **MEDIUM** |
| 8 | "the communication protocol has run across four nodes in **live outdoor tests**" | True for LoRa. **Not true for the WiFi mesh**: prior to the 2026-08-25 two-vehicle bench test, the 802.11s mesh had never been brought up in any bench or field run; all runs used an infrastructure access point | **HIGH (reduction to practice)** |

Item 1 is the most serious: the disclosure names the wrong 802.11 operating mode in its second
claimed innovation, and the kernel and driver work described in Section 3.2 above is
*specifically* mesh-point work (beacon reserved pages, TSF update disable, foreign-BSSID receive
filtering) that would not be required for, and does not describe, IBSS. Items 2 through 4 are
arithmetic and should be corrected to the measured values. Items 5, 6 and 8 assert performance
and reduction to practice that the record does not yet support; either measure them before
filing (throughput and range are each a single short bench session) or state them as design
targets rather than results.

Supporting evidence for every value above is in `reports/rf_results/RF_RESULTS.md`,
`reports/wifi_mesh/MESH_CHARACTERIZATION.md`, and `jfr_codefacts_report.md`.
