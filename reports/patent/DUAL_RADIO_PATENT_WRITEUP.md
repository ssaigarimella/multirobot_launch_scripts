# Dual-Radio Coordination Architecture for Decentralized Multi-Robot Systems

**Technical description for patent disclosure.** All values are from the code and configuration
deployed on the fleet as of 2026-09-25; measured values are labeled. Platform: four UAVs, NVIDIA
Jetson Orin NX (JetPack 6, L4T R36.4.0, kernel 5.15.148-tegra), Heltec Wireless Stick Lite V3
(ESP32-S3 + Semtech SX1262), Realtek RTL8822CE 802.11ac.

## 1. Architecture

Two independent radio links carry disjoint traffic classes, paired so that each covers the
other's failure mode. This is not redundancy: neither link can carry the other's traffic.

| | Link A: LoRa TDMA | Link B: 802.11s WiFi mesh |
|---|---|---|
| Hardware | SX1262 on a separate ESP32-S3 MCU | RTL8822CE, PCIe, on the Jetson |
| Band / access | 915 MHz, **deterministic TDMA, zero contention** | 2.437 GHz (ch 6), CSMA/CA |
| Traffic | Pose, frontier claims, coordination state | Compressed occupancy maps, keyframes |
| Measured | 5.263 Hz/node, 800 B/s aggregate | 39.3 Mbit/s mean negotiated |

## 2. Link A: deterministic LoRa TDMA with transitive lowest-identifier synchronization

Firmware `lora_mesh_v7_2_ros2.ino`, flashed per vehicle with compile-time node identifiers
(delta 0, buckshee 1, ghost 2, thunderstrike 3).

**Slot structure.** `NUM_NODES 4`, `SLOT_MS 40`, `GUARD_MS 30`, `TX_MARGIN_MS 5`,
`CYCLE_MS = NUM_NODES * SLOT_MS + GUARD_MS` = **190 ms**, i.e. **5.263 Hz per node**. Node *i*
transmits only in slot *i*. Slot ownership is exclusive and derived from the identifier, so **two
nodes can never transmit simultaneously regardless of offered load**. Cycle time grows linearly
as nodes join, preserving the zero-collision guarantee at any fleet size.

**Payload-protected scheduling.** A transmission is admitted only if it starts after
`TX_MARGIN_MS` and completes by `SLOT_MS - TX_MARGIN_MS`, a 30 ms window. At boot the firmware
derives the largest slot-safe packet by direct airtime evaluation:

```c
uint32_t slot_budget_us = (SLOT_MS - 2 * TX_MARGIN_MS) * 1000UL;
for (size_t sz = BASE_PACKET_SIZE; sz <= sizeof(Packet); sz++)
    if (radio.getTimeOnAir(sz) <= slot_budget_us) max_tx_size = sz; else break;
```

This yields `max_tx_size = 64 B`. An oversized coordination payload is **dropped, never truncated
or transmitted**, so an application-layer message cannot overrun a slot and desynchronize the
mesh. The schedule is structurally immune to application misbehavior.

**Synchronization.** Every packet carries `sync_source` and `cycle_clock`; there are **no
dedicated synchronization broadcasts**, so synchronization consumes zero additional airtime. On
reception:

```c
bool shouldSyncTo(uint8_t sender_id, uint8_t sender_sync) {
    uint8_t auth = sender_id < sender_sync ? sender_id : sender_sync;
    if (auth < my_sync_source) return true;
    if (auth == my_sync_source && sender_id <= my_sync_source) return true;
    return false;
}
```

A node adopts the timebase of the lowest identifier it can reach **transitively**: `auth` is the
lesser of the sender's identifier and the identifier the sender is itself synchronized to, so a
node two hops away inherits that timebase through an intermediary without direct reception. On
adoption the epoch is recomputed with **propagation-delay compensation derived from the radio's
own airtime model**, not a constant:

```c
uint32_t tx_moment = now - packet_airtime_ms;   // packet_airtime_ms = radio.getTimeOnAir(...)
uint32_t new_epoch = tx_moment - sender_pos - sender_cycle_num * CYCLE_MS;
```

`findNewSyncSource()` reselects on node loss (`NODE_TIMEOUT_MS 3000`); a departed node's slot is
reclaimed within one cycle. There is **no GPS, NTP, PTP, or master node** anywhere in the scheme.

**Packet.**

```c
typedef struct __attribute__((packed)) {
  uint8_t node_id; uint32_t sequence; uint32_t uptime_ms; uint32_t cycle_clock;
  uint8_t sync_source; int8_t rssi_of[NUM_NODES]; uint8_t neighbor_mask;
  float lat, lon, alt, hdg;
  uint8_t has_custom, custom_dest, custom_len; uint8_t custom_data[160];
} Packet;                    // sizeof 198 B; BASE_PACKET_SIZE 38 B actually transmitted
```

Transmission is variable-length (`BASE_PACKET_SIZE + custom_len`). `rssi_of[]` carries each node's
measured RSSI of every peer and `neighbor_mask` the reachability bitmap, so **every ordinary
packet redistributes the full link-quality matrix of the mesh at zero additional airtime**.
Frontier claims ride in `custom_data` as a 19-byte record.

**Radio and measured performance.** 915.0 MHz, SF7, BW 500 kHz, CR 4/5, preamble 8, sync 0x12,
+10 dBm, CRC on. PHY 21,875 bps. Base packet airtime 20.544 ms = **10.81 % duty per node**,
**43.3 % aggregate channel occupancy** at four nodes, **800 B/s aggregate application payload**.

Measured over 7068 consecutive inter-arrivals / 1343 s continuous operation: median inter-arrival
**190.00 ms**, jitter (SD) **1.22 ms**, interquartile range **0.00 ms**, worst single deviation
**19.0 ms** (inside the 40 ms slot), **zero** intervals at two or more cycles, per-cycle miss rate
bounded at **424 ppm** (95 % confidence). Received RSSI -32.8 +/- 1.1 dBm, SNR 12.6 +/- 0.5 dB.
Zero missed cycles is measurable *without* a sequence number precisely because the schedule is
deterministic: a lost transmission necessarily appears as a two-cycle gap.

## 3. Link B: 802.11s mesh point on a patched kernel and driver

**Configuration.** `iw dev <iface> set type mp`, then
`iw dev <iface> mesh join "jetson-drone-mesh" freq 2437`. Channel 6, 2.437 GHz, 20 MHz.
Addressing is **static and identifier-derived**, `192.168.77.<drone_id>`; no DHCP or address
negotiation exists in the fleet. Confirmed operational with per-peer signal -40.1 to -40.5 dBm
mean and 39.3 Mbit/s mean negotiated rate over a 15-minute two-vehicle run.

**Kernel and driver modifications.** Stock JetPack 6 ships `# CONFIG_MAC80211_MESH is not set`
and the in-tree `rtw88` refuses the mesh interface type. Three changes are required:

1. **`mac80211` rebuilt with `CONFIG_MAC80211_MESH=y`** (verified: the loaded module exports 124
   `ieee80211_mesh_*` symbols); the vendor `rtl8822ce` driver is blacklisted in favor of `rtw88`.
2. **Interface-type acceptance with AP-style beaconing**, `rtw88/mac80211.c`:
   ```c
   case NL80211_IFTYPE_AP:
   case NL80211_IFTYPE_MESH_POINT:
       rtw_add_rsvd_page_bcn(rtwdev, rtwvif);
       net_type = RTW_NET_AP_MODE;
       bcn_ctrl = BIT_EN_BCN_FUNCTION | BIT_DIS_TSF_UDT;
   ```
   `BIT_DIS_TSF_UDT` is essential: a mesh node must beacon like an AP **without** adopting a
   peer's TSF, which would otherwise fight the mesh's own timing.
3. **Receive-filter change for foreign BSSIDs**, `rtw88/mac80211.c`. Mesh peers share no BSSID,
   so hardware BSSID filtering must be lifted or peer beacons are silently discarded and peering
   never forms:
   ```c
   if (*new_flags & FIF_OTHER_BSS) {
       rtwdev->hal.rcr |= BIT_AAP;                              // accept all
       rtwdev->hal.rcr &= ~(BIT_CBSSID_BCN | BIT_CBSSID_DATA);  // stop BSSID filtering
   }
   ```
   `rtw88/fw.c` additionally excludes `NL80211_IFTYPE_MESH_POINT` from a guard that would skip a
   required firmware path for mesh interfaces.

**Transport containment.** ROS 2 DDS discovery is multicast-based and does not survive a lossy
wireless mesh. Rather than tune multicast, the architecture **confines DDS to loopback** and
bridges only selected topics: `fastdds_loopback.xml` and `xrce_agent_loopback.refs` restrict
FastDDS and the PX4 uXRCE-DDS agent to `<interfaceWhiteList><address>127.0.0.1</address>`, so no
DDS traffic reaches the radio; each vehicle runs in its own `ROS_DOMAIN_ID` equal to its drone
identifier; and a per-vehicle `zenoh-bridge-ros2dds` carries inter-robot traffic over **TCP 7447**
with `scouting.multicast.enabled: false`, `gossip.enabled: true`, forwarding an explicit allow
list only (`{ns}/loop_closure/.*`, `{ns}/octomap.*`, `{ns}/swarm/.*`, `{ns}/alignment/.*`,
`{ns}/keyframes/.*`, plus global `/loop_closures`, `/loop_closure_alignment`). The wireless link
carries a bounded, enumerated topic set over unicast TCP, never discovery chatter.

## 4. Degradation

The links fail under different physical conditions, which is the point of the pairing: 2.4 GHz
WiFi degrades first with range and obstruction, while 915 MHz LoRa carries roughly 15 dB more
link budget and better material penetration. With both up, coordination is fully deterministic
and maps fuse. With WiFi lost, coordination continues at 5.263 Hz on pose and frontier claims
while vehicles keep independent local maps and avoid redundant coverage. With both lost, each
vehicle explores on its onboard map and coordination resumes automatically on any link return,
since LoRa re-synchronization is self-healing within one cycle. If a node departs, its slot is
reclaimed within one cycle and, if it was the sync source, the next-lowest identifier is elected.

## 5. Novelty

1. **Payload-protected deterministic TDMA.** Boot-time airtime evaluation derives a slot-safe
   maximum packet size and oversized payloads are dropped, making the schedule structurally
   immune to application-layer misbehavior.
2. **Transitive lowest-identifier synchronization, in-band, with airtime compensation.** No sync
   traffic, no master, no external time reference; propagation delay removed using the radio's
   own airtime model. Measured 1.22 ms jitter, zero missed cycles in 7068.
3. **Free link-quality dissemination.** Every packet redistributes the full mesh RSSI matrix and
   reachability bitmap at zero additional airtime.
4. **Complementary-failure dual-radio pairing** carrying disjoint traffic classes matched to each
   radio's physics, rather than one link being a hot spare for the other.
5. **Enabling kernel and driver work.** 802.11s on RTL8822CE/rtw88 under JetPack 6 requires a
   mesh-enabled `mac80211`, an interface-type path with AP-style beaconing and TSF update
   disabled, and a receive filter accepting foreign-BSSID frames.
6. **Loopback-confined DDS with an allow-listed unicast bridge**, making ROS 2 multi-robot
   operation viable over a lossy wireless mesh without multicast discovery.
