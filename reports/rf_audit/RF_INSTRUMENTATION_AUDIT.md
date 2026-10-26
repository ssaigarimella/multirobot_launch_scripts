# RF Instrumentation Audit — read-only

**Date:** 2026-08-18 (audit run 21:37–21:45 UTC)
**Auditor:** automated read-only sweep
**Scope:** can every quantity required by the radio characterization experiments be
measured with hardware already installed, or is a purchase unavoidable?
**Changes made:** none. No firmware flashed, no PX4 parameter set, no config edited,
no service restarted, no interface brought up/down, no `iw ... scan` on any interface,
nothing installed. Only new files are under `~/rf_audit/` on drone 1 (Delta).

---

## 0. Reachability and contradictions

### 0.1 Reachability

| Drone | ID | Address | Reachable | Used for |
|---|---|---|---|---|
| delta | 1 | 192.168.0.40 | YES | A, B, C, D, E, F, G |
| buckshee | 2 | 192.168.0.60 | **NO** — `No route to host` | — |
| ghost | 3 | 192.168.0.50 | YES | A, B, D, E, F (substitute 2nd vehicle) |
| thunderstrike | 4 | 192.168.0.70 | **NO** — `No route to host` | — |

```
--- delta ---            delta / 5.15.148-tegra / up 26 minutes
--- buckshee ---         ssh: connect to host 192.168.0.60 port 22: No route to host
--- ghost ---            ghost / 5.15.148-tegra / up 31 minutes
--- thunderstrike ---    ssh: connect to host 192.168.0.70 port 22: No route to host
```

The prompt directs sections A/B/D/E/F at "drone 1 (Delta) and drone 2 (Buckshee)".
Buckshee is off the network. Per "If a drone is unreachable, record that and continue",
**Ghost (drone 3) was used as the second vehicle** wherever a second vehicle was needed.
This is labelled at every point where it matters.

### 0.2 Contradictions found (reported verbatim, not worked around)

**C-1 — firmware version.** Prompt says:
> "Locate the TDMA v7.1 firmware source tree"

No v7.1 tree exists on the fleet. The only TDMA firmware source is
`/mnt/nova_ssd/workspaces/isaac_ros-dev/src/radiohive/firmware/lora_mesh_v7_2_ros2.ino`,
self-described in its header as `LoRa Decentralized TDMA Mesh v7.2 — ROS2 Serial Bridge
(variable-length TX fix)`. Section A was answered against **v7.2**.

**C-2 — 802.11s mesh is not up.** Prompt says:
> "## Section B: WiFi mesh link (RTL8822CE-CG, rtw88, 802.11s)"
> "do not run `iw dev ... scan` on the mesh interface (it can drop mesh peering)"

There is no mesh interface and no mesh peering on either reachable drone. Both are
**`type managed`** stations associated to SSID `TP-Link_073A` on **channel 153
(5765 MHz)**. `iw dev` reports exactly one interface per drone, `wlP1p1s0`, in managed
mode. 802.11s exists in this project only as a *transient* mode created and torn down by
`radiohive/scripts/test_radiohive_pair.sh` ("1. Switch WiFi to 802.11s mesh … WiFi
restore"). Bringing it up is a networking reconfiguration and is forbidden by the hard
constraints, so it was not done.

Section B was therefore **stopped as a mesh audit**. What follows in Section B is
driver-capability evidence gathered read-only on the existing managed link, because the
same `nl80211`/`rtw88` code path serves both modes and the purchase question turns on
driver capability, not on link mode. Every mesh-specific answer is labelled as such.

**C-3 — `RC_INPUT_PROTO` does not exist on this airframe.** Prompt says:
> "C1. Report the receiver protocol from the autopilot parameters: `RC_INPUT_PROTO` (or
> the equivalent on PX4 v1.15.4)"

`RC_INPUT_PROTO` is **absent** from the ULog's full 1136-parameter set. On
CUBEPILOT_CUBEORANGEPLUS the RC protocol is detected by the PX4IO co-processor and no
such parameter is exposed. The equivalent evidence used instead is
`input_rc.input_source = 4` (constant) = `RC_INPUT_SOURCE_PX4IO_SBUS`, corroborated by
`RC_INPUT_PROTO 2` (= SBUS) in the archived `.params` files. See C1.

**C-4 — no PX4 parameter controls `input_rc` log rate.** Prompt says:
> "C4. … If it cannot, report which PX4 parameter would need to change and stop."

No such parameter exists. Per-topic ULog rates are compile-time in
`src/modules/logger/logged_topics.cpp` (`add_topic("input_rc", 500)` — 500 ms, which
matches the observed 500 ms median exactly). `SDLOG_PROFILE` is already `2047` (all
profiles) and selects *which* topics are logged, not their rate. Section C4 stops there
and reports this; nothing was changed.

**C-5 — deployed node count differs from checked-in source.** The checked-in `.ino` has
`#define NUM_NODES 3` (cycle = 3×40+30 = 150 ms). The measured pose interval in the
existing LoRa logs is **190.1 ms / 185.7 ms**, which is `4×40+30 = 190 ms`, and
`docs/LORA_BUDGET.md` states "n = 4 vehicles, cycle n*tau + g = 190 ms" and "base TDMA
packet is 38 bytes". The flashed firmware is therefore a **NUM_NODES=4** build; the
checked-in source is a 3-node variant. Both byte budgets are reported in A4.

**C-6 — `LORA_BUDGET.md` overstates the slot cap.** The doc lists the 70-byte
configuration (base 38 + 32 B full packet) as "fits, fails 30%" at 32.06 ms. The
firmware's own cap is `SLOT_MS - 2*TX_MARGIN_MS = 30 ms`, so `max_tx_size = 64 B` and a
70-byte packet is **silently dropped** by `doTx()`, not merely low-margin. Verified with
the repo's own airtime function in A4.

---

## Section A — LoRa coordination link (SX1262 on Heltec Wireless Stick Lite V3)

**Firmware source tree (absolute path and mtime):**

```
/mnt/nova_ssd/workspaces/isaac_ros-dev/src/radiohive/firmware/lora_mesh_v7_2_ros2.ino
-rw-rw-r-- 1 delta delta 20768 Jul  1 15:46 lora_mesh_v7_2_ros2.ino     (659 lines)

repo: /mnt/nova_ssd/workspaces/isaac_ros-dev/src/radiohive  (git)
  d09503e Add LoRa mesh serial bridge (lora_bridge_node) + matched firmware
```

### A1. Radio library — RadioLib (via the `heltec_unofficial` wrapper). Version: **NOT DETERMINABLE**

`lora_mesh_v7_2_ros2.ino:34`
```c
#include <heltec_unofficial.h>
```

This is the `Heltec_ESP32_LoRa_v3` ("heltec_unofficial") wrapper, which is a thin shim
over **RadioLib** — not the Heltec `LoRaWan_APP` / `LoRaWan_102` stack. Proof from the
call surface used in this file: `radio.begin(...)` with the RadioLib argument order
(`:371`), `RADIOLIB_ERR_NONE` (`:373, :536, :552`), `radio.setDio1Action()` (`:379`),
`radio.getTimeOnAir()` (`:382`), `radio.startReceive()` (`:173`), `radio.readData()`
(`:551`), `radio.getRSSI()` / `radio.getSNR()` (`:567-568`). The Heltec `LoRaWan_APP`
wrapper has none of these; it uses `Radio.Init()` with an `RadioEvents_t` callback struct.

**Version string: absent.** There is no `library.properties` and no `platformio.ini`
anywhere on any fleet machine:
```
find / -xdev \( -iname "RadioLib*" -o -iname "heltec_unofficial*" -o -iname "library.properties" \) ...
(no output)
find ~ -name platformio.ini -o -name library.properties
(no output)
```
The firmware is built off-fleet (Arduino IDE / PlatformIO on a machine not audited here).
**Reported as missing** — the exact RadioLib version cannot be pinned from the fleet.

### A2. RX path reads RSSI and SNR — YES, and they are transmitted, not discarded

Captured — `lora_mesh_v7_2_ros2.ino:567-568` inside `handleRx()`:
```c
  uint8_t id = p->node_id;
  float r = radio.getRSSI();
  float s = radio.getSNR();
```
Stored — `:599-600`, `:607`:
```c
  nodes[id].rssi = (int16_t)r;
  nodes[id].snr  = s;
  ...
  rssi_of[id] = (int8_t)constrain((int)r, -128, 0);
```
Transmitted three ways — **not discarded**:
1. `:618` `sendPeerPose(id);` on **every** received packet → serial `0x81 PEER_POSE`,
   which packs `rssi(2) snr(4)` at `:217-220`.
2. `:613` `sendPeerJoined(id, r, s);` → `0x85 PEER_JOINED` (`:231-238`).
3. `sendStatusResp()` (`:245-263`) → `0x83 STATUS_RESP`, `[id(1) rssi(2) snr(4)]` per peer.
4. `rssi_of[NUM_NODES]` is additionally echoed **over the air** in every TX packet
   (`:90`, `:500`), giving reciprocal link measurement for free.

The human-readable `printStatus()` (`:631-659`) is entirely commented out, so nothing is
"only printed to stdout".

### A3. Packet struct — sequence YES, TX timestamp YES, TX power **ABSENT**

`lora_mesh_v7_2_ros2.ino:84-106`:
```c
typedef struct __attribute__((packed)) {
  uint8_t  node_id;
  uint32_t sequence;          // <-- SEQUENCE NUMBER: present
  uint32_t uptime_ms;         // <-- TX TIMESTAMP: present (sender millis() at TX)
  uint32_t cycle_clock;
  uint8_t  sync_source;
  int8_t   rssi_of[NUM_NODES];
  uint8_t  neighbor_mask;
  float    lat;
  float    lon;
  float    alt;
  float    hdg;
  // Custom message piggyback
  uint8_t  has_custom;
  uint8_t  custom_dest;
  uint8_t  custom_len;
  uint8_t  custom_data[MAX_CUSTOM_DATA];   // MAX_CUSTOM_DATA = 160
} Packet;

static const size_t BASE_PACKET_SIZE = sizeof(Packet) - MAX_CUSTOM_DATA;
```
Populated at `:489-492`:
```c
  pkt.node_id     = NODE_ID;
  pkt.sequence    = my_seq++;
  pkt.uptime_ms   = now;
  pkt.cycle_clock = getCycleClock(now);
```

`sizeof` computed from source (packed, no padding):

| field | bytes (NUM_NODES=3) | bytes (NUM_NODES=4, deployed) |
|---|---|---|
| node_id | 1 | 1 |
| sequence | 4 | 4 |
| uptime_ms | 4 | 4 |
| cycle_clock | 4 | 4 |
| sync_source | 1 | 1 |
| rssi_of[NUM_NODES] | 3 | 4 |
| neighbor_mask | 1 | 1 |
| lat/lon/alt/hdg | 16 | 16 |
| has_custom/custom_dest/custom_len | 3 | 3 |
| custom_data[160] | 160 | 160 |
| **sizeof(Packet)** | **197** | **198** |
| **BASE_PACKET_SIZE** | **37** | **38** |

`BASE_PACKET_SIZE = 38` for the deployed 4-node build, matching `LORA_BUDGET.md`.

**TX power field: ABSENT.** No member of `Packet` carries the transmit power. The RX side
therefore cannot know what power a received packet was sent at — fatal for a PDR-vs-RSSI
curve across a power sweep unless added (see A5).

**Sequence numbers also exist one layer up**, in the coordination payload codec
`multi_drone_nvblox/mdn_core/lora_packet.py`:
```
  3      1    u8    seq          rolling sequence number
_BODY_FMT = '<BBBBIhhhhhhhHBBBBBB'      # version, src_id, ref_id, seq | t_ms | ...
BODY_SIZE = 30 ; PACKET_SIZE = 32
_CLAIM_FMT = "<BBBBhhhHBBBBB"           # 17 B + crc16 = 19 B (DEPLOYED)
```
The 32-byte full packet carries both `seq` **and** `t_ms` (a TX timestamp). The deployed
19-byte claim packet carries `seq` but **not** `t_ms`.

### A4. Payload size, airtime, and remaining byte budget

Radio config, `:48-54`: `915.0 MHz, BW 500.0 kHz, SF7, CR 4/5, sync 0x12, power 10 dBm,
preamble 8`. TDMA, `:42-45`: `SLOT_MS 40`, `GUARD_MS 30`, `CYCLE_MS = NUM_NODES*40+30`.

Computed with the repo's own Semtech implementation
(`mdn_core/lora_packet.py: lora_airtime_ms`), which self-validates against the standard
reference value:

```
reference check 13B SF7 BW125 = 46.336 (expect 46.336)

SF7 / 500 kHz / CR4-5 / pre8 / explicit hdr / CRC on
    37 B ->  20.544 ms  FITS     vs 30 ms TX window   (40 ms slot slack  19.46 ms)
    38 B ->  20.544 ms  FITS     vs 30 ms TX window   (40 ms slot slack  19.46 ms)
    45 B ->  23.104 ms  FITS     vs 30 ms TX window   (40 ms slot slack  16.90 ms)
    50 B ->  24.384 ms  FITS     vs 30 ms TX window   (40 ms slot slack  15.62 ms)
    56 B ->  26.944 ms  FITS     vs 30 ms TX window   (40 ms slot slack  13.06 ms)
    57 B ->  26.944 ms  FITS     vs 30 ms TX window   (40 ms slot slack  13.06 ms)
    60 B ->  28.224 ms  FITS     vs 30 ms TX window   (40 ms slot slack  11.78 ms)
    63 B ->  29.504 ms  FITS     vs 30 ms TX window   (40 ms slot slack  10.50 ms)
    64 B ->  29.504 ms  FITS     vs 30 ms TX window   (40 ms slot slack  10.50 ms)
    65 B ->  30.784 ms  REFUSED  vs 30 ms TX window   (40 ms slot slack   9.22 ms)
    70 B ->  32.064 ms  REFUSED  vs 30 ms TX window   (40 ms slot slack   7.94 ms)
    80 B ->  35.904 ms  REFUSED  vs 30 ms TX window   (40 ms slot slack   4.10 ms)
   197 B ->  78.144 ms  REFUSED  vs 30 ms TX window   (40 ms slot slack -38.14 ms)

  base=37 B -> firmware max_tx_size=64 B, custom payload budget = 27 B
  base=38 B -> firmware max_tx_size=64 B, custom payload budget = 26 B
```

- **Current on-air payload:** 38 B base (pose-only) = **20.544 ms**; deployed
  base + 19 B claim = 57 B = **26.944 ms**.
- **Slot 40 ms, guard 30 ms** (guard is once per 190 ms cycle, not per slot). The
  firmware's usable TX window is `SLOT_MS - 2*TX_MARGIN_MS = 30 ms` (`:388`).
- **`max_tx_size` = 64 B.** Derived by the firmware at boot (`:388-393`) and reproduced
  exactly above.
- **Remaining byte budget before airtime exceeds the slot: 26 B** (64 − 38). The deployed
  claim payload consumes 19 B, leaving **7 B free right now** without displacing anything.

7 free bytes is sufficient for the two missing fields: `tx_power` (int8, 1 B) and a TX
timestamp (uint32, 4 B) = 5 B. **The byte budget is not a blocker.**

Note the C-6 contradiction: `LORA_BUDGET.md` describes the 70-byte configuration as
fitting the slot; it does not — `doTx()` drops it (`:509-521`).

### A5. TX power is a **compile-time constant**. No runtime path. A 31 dB sweep needs one reflash per step.

`:53`
```c
#define LORA_PWR          10
```
Consumed once, at `:371-372`:
```c
  int st = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                        LORA_SYNC, LORA_PWR, LORA_PRE, 1.6, false);
```
`setOutputPower()` is **never called anywhere in the file.**

The complete serial command parser — `handleSerialMessage()`, `:312-337` — is the only
runtime control path from the Jetson:
```c
void handleSerialMessage(uint8_t type, uint8_t* payload, uint8_t len) {
  switch (type) {
    case MSG_POSE_UPDATE:
      if (len >= 16) {
        memcpy(&my_lat, &payload[0],  4);
        memcpy(&my_lon, &payload[4],  4);
        memcpy(&my_alt, &payload[8],  4);
        memcpy(&my_hdg, &payload[12], 4);
      }
      break;

    case MSG_CUSTOM_MSG:
      if (len >= 2 && !custom_pending) {
        custom_dest = payload[0];
        custom_len = len - 1;
        if (custom_len > MAX_CUSTOM_DATA) custom_len = MAX_CUSTOM_DATA;
        memcpy(custom_data, &payload[1], custom_len);
        custom_pending = true;
      }
      break;

    case MSG_STATUS_REQ:
      sendStatusResp(millis());
      break;
  }
}
```
Three message types: `0x01 POSE_UPDATE`, `0x02 CUSTOM_MSG`, `0x03 STATUS_REQ`. **There is
no set-power command.**

**Explicit statement:** with the firmware as it stands, a +22 → −9 dBm sweep
(SX1262 range, 32 steps at 1 dB or ~9 steps at 4 dB) requires **one reflash per power
step**, on every node, with the mesh torn down and re-formed each time. It **cannot** be
scripted from the Jetson today.

This is a ~15-line firmware change, not a hardware problem: add `#define MSG_SET_TXPOWER
0x04`, a `case` calling `radio.setOutputPower((int8_t)payload[0])`, and a `tx_power` byte
in `Packet` (7 B of budget free, A4). The sweep then becomes a Jetson-side script
publishing on an existing serial channel.

### A6. Instantaneous channel RSSI with no packet present — API exists in the library, **not called by this firmware**, version unverifiable

RadioLib's `SX126x` class provides both:
- `getRSSI(bool packet)` — with `packet = false` it issues the SX1262 `GetRssiInst`
  command and returns the *instantaneous* channel RSSI with no packet required.
- `scanChannel()` — runs the SX1262 CAD (Channel Activity Detection) block.

Neither appears anywhere in `lora_mesh_v7_2_ros2.ino`. The only call is the packet-scoped
`radio.getRSSI()` (no argument, defaults to `packet = true`) at `:567`, valid only for a
just-received packet.

**Honest caveat:** RadioLib is not installed on any fleet machine (A1), so the *installed*
version's exact signature could not be verified on-fleet. `getRSSI(bool)` has been present
in RadioLib's SX126x since v5.x; the `heltec_unofficial` wrapper exposes the underlying
`SX1262` object directly as `radio`, so the API is reachable. Treated below as
**NEEDS CODE**, not NEEDS HARDWARE — but this is the one row whose "no purchase needed"
verdict rests on a library version that could not be read on-fleet.

This is the fleet's **only** 915 MHz power-measurement capability. There is no SDR (F1).

### A7. Jetson-side serial bridge

| Item | Value |
|---|---|
| Node | `lora_bridge_node.py` |
| Package | `radiohive` |
| Source | `/mnt/nova_ssd/workspaces/isaac_ros-dev/src/radiohive/scripts/lora_bridge_node.py` (381 lines) |
| Launch | `radiohive/launch/lora_bridge.launch.py` |
| Device node | auto-detected `/dev/ttyUSB*` or `/dev/ttyACM*` by USB VID:PID **`10c4:ea60`** (Silicon Labs CP2102). Resolves to **`/dev/ttyUSB1` on delta**, **`/dev/ttyUSB0` on ghost** |
| Baud rate | **115200** (`declare_parameter('baud_rate', 115200)`; firmware `#define SERIAL_BAUD 115200`) |
| Running at audit time | **No** — `ps aux` showed no lora/px4/uXRCE processes on delta |

Device-selection logic (`lora_bridge_node.py:25-28, 58, 76-82`):
```
Device selection: the LoRa is a Silicon Labs CP2102 (10c4:ea60); the PX4 link is
an FTDI FT232R (0403:6001) on /dev/ttyUSB0. /dev/serial/by-id is NOT populated
[inside the container], so walk /sys and pick the CP2102 — never the FTDI. This is
robust to ttyUSB* renumbering.

LORA_VID_PID = ('10c4', 'ea60')   # Silicon Labs CP2102 (Heltec)
```

**Published topics (prefix default `lora`):**

| Topic | Type | Carries RSSI/SNR? |
|---|---|---|
| `lora/peer_pose` | `radiohive/msg/PeerPose` | **YES** — `int16 rssi`, `float32 snr` |
| `lora/mesh_status` | `radiohive/msg/MeshStatus` | **YES** — `int16[] peer_rssi`, `float32[] peer_snr` |
| `lora/peer_event` | `radiohive/msg/PeerEvent` | **YES** — `int16 rssi`, `float32 snr` |
| `lora/peer_msg` | `radiohive/msg/PeerMessage` | no (`uint8 src_node`, `uint8[] data`) |

**Subscribed topics:**

| Topic | Type |
|---|---|
| `lora/tx_pose` | `radiohive/msg/LoraPose` |
| `lora/tx_msg` | `radiohive/msg/LoraSend` |
| `lora/status_req` | `std_msgs/msg/Empty` |

`msg/PeerPose.msg` verbatim:
```
# A peer's pose received over the LoRa mesh (firmware PEER_POSE -> ROS).
uint8 node_id
float32 latitude
float32 longitude
float32 altitude
float32 heading
int16 rssi
float32 snr
```

Unpack path (`:63`, `:271-282`):
```python
FMT_PEER_POSE = '<Bffffhf'    # node, lat, lon, alt, hdg, rssi, snr      (23 B)
...
node, lat, lon, alt, hdg, rssi, snr = struct.unpack(FMT_PEER_POSE, payload)
...
m.rssi = rssi
m.snr = snr
```

**Answer:** RSSI and SNR are **first-class fields on published ROS 2 messages**, not
stdout prints. **Sequence number and TX timestamp are NOT forwarded** — the firmware has
both in `Packet` (A3) but the `0x81 PEER_POSE` serial frame omits them, so they are
dropped at the firmware→ROS boundary.

### A8. `/dev/serial/by-id` — Heltec enumerates with a stable symlink

**delta:**
```
$ ls -l /dev/serial/by-id/
total 0
lrwxrwxrwx 1 root root 13 Dec 31  1969 usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0 -> ../../ttyUSB0
lrwxrwxrwx 1 root root 13 Dec 31  1969 usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -> ../../ttyUSB1
```
**ghost:**
```
lrwxrwxrwx 1 root root 13 Dec 31  1969 usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0 -> ../../ttyUSB1
lrwxrwxrwx 1 root root 13 Dec 31  1969 usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -> ../../ttyUSB0
```

**Confirmed:** the Heltec enumerates with a stable by-id symlink on the host. Two caveats:
1. The CP2102 serial number is the generic **`0001`** on both drones, so the by-id name is
   stable *per drone* but **not unique across drones** — it cannot disambiguate two Heltecs
   on one host.
2. `ttyUSB` numbering is **inverted between delta and ghost** (delta: LoRa=USB1, PX4=USB0;
   ghost: LoRa=USB0, PX4=USB1), which is exactly why the bridge auto-detects by VID:PID.
   Note the node's comment that by-id is absent *inside the container*; it is present on
   the host, as shown.

---

## Section B — WiFi link (RTL8822CE, rtw88)

> **See contradiction C-2.** No 802.11s mesh is up. Both drones are managed-mode STAs on
> 5765 MHz. The mesh audit is stopped. What follows is read-only driver-capability
> evidence on the existing link plus the mesh's configured band read from the setup script.

### Raw output — `iw dev` (delta; ghost identical but for MAC)
```
phy#0
	Interface wlP1p1s0
		ifindex 3
		wdev 0x1
		addr dc:4a:9e:37:a1:49
		ssid TP-Link_073A
		type managed
		channel 153 (5765 MHz), width: 80 MHz, center1: 5775 MHz
		txpower 30.00 dBm
```
ghost: `addr dc:4a:9e:37:9e:fb`, otherwise identical.

### Raw output — `cat /proc/net/wireless`
```
delta:  wlP1p1s0: 0000   70.  -256.  -256        0      0      0      0      0        0
ghost:  wlP1p1s0: 0000   70.  -26.   -256        0      0      0      0      1        0
        Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE
         face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22
```
**noise = −256 on both** = the "unset" sentinel. The driver reports no noise floor.

### Raw output — `iw dev wlP1p1s0 station dump` (delta)
```
Station e4:fa:c4:57:07:39 (on wlP1p1s0)
	inactive time:	0 ms
	rx bytes:	38219525
	rx packets:	49905
	tx bytes:	518032
	tx packets:	3762
	tx retries:	0
	tx failed:	0
	beacon loss:	0
	rx drop misc:	0
	signal:  	0 [0, 0] dBm
	signal avg:	-17 [-18, -20] dBm
	tx bitrate:	866.7 MBit/s VHT-MCS 9 80MHz short GI VHT-NSS 2
	tx duration:	0 us
	rx bitrate:	173.4 MBit/s VHT-MCS 9 VHT-NSS 2
	rx duration:	0 us
	authorized:	yes
	authenticated:	yes
	associated:	yes
	preamble:	long
	WMM/WME:	yes
	MFP:		no
	TDLS peer:	no
	DTIM period:	1
	beacon interval:100
	short slot time:yes
	connected time:	1864 seconds
```
### Raw output — `iw dev wlP1p1s0 station dump` (ghost)
```
Station e4:fa:c4:57:07:39 (on wlP1p1s0)
	inactive time:	9188 ms
	rx bytes:	9614676
	rx packets:	45183
	tx bytes:	19163
	tx packets:	191
	tx retries:	0
	tx failed:	0
	beacon loss:	0
	rx drop misc:	1
	signal:  	-28 [-33, -28] dBm
	signal avg:	-27 [-32, -27] dBm
	tx bitrate:	780.0 MBit/s VHT-MCS 8 80MHz short GI VHT-NSS 2
	rx bitrate:	6.0 MBit/s
	connected time:	2173 seconds
```

### Raw output — `iw dev wlP1p1s0 survey dump`
```
delta:  (empty — no output)
ghost:  (empty — no output)
```

### Raw output — `iw phy phy0 info | sed -n '/Supported interface modes/,/Band/p'`
```
	Supported interface modes:
		 * IBSS
		 * managed
		 * AP
		 * AP/VLAN
		 * monitor
		 * mesh point
	Band 1:
```
Identical on both drones.

### Raw output — driver modules
```
rtw88_8822ce           16384  0
rtw88_8822c           471040  1 rtw88_8822ce
rtw88_pci              32768  1 rtw88_8822ce
rtw88_core            155648  2 rtw88_pci,rtw88_8822c
mac80211              921600  2 rtw88_pci,rtw88_core
cfg80211              765952  2 rtw88_core,mac80211
```

### B1. Band and channel — **5765 MHz now; the mesh is configured for 2437 MHz**

- **Currently associated link: 5765 MHz** (channel 153, 5 GHz, 80 MHz wide). This is the
  bench TP-Link AP, **not** a mesh.
- **The 802.11s mesh, when brought up, is 2.4 GHz.** From
  `radiohive/scripts/test_radiohive_mesh.sh:55-57`:
```bash
MESH_ID="jetson-drone-mesh"
MESH_FREQ="2437"          # Channel 6, 2.4 GHz
MESH_CHANNEL="6"
```
and `:301-302`:
```bash
echo "[4/11] JOINING MESH: $MESH_ID on ${MESH_FREQ}MHz..."
if ! iw dev "$IFACE" mesh join "$MESH_ID" freq "$MESH_FREQ" 2>&1; then
```

**Stated plainly:** the WiFi mesh as configured operates at **2437 MHz (channel 6,
2.4 GHz)**, so it **is** a genuine coexistence candidate against a 2.4 GHz manual-control
receiver — they share the band and overlap in spectrum. The 5765 MHz link observed during
this audit is the bench AP and is *not* a coexistence candidate. The coexistence matrix
must be built with the mesh actually joined at 2437 MHz; measuring on the 5 GHz bench link
would show no interference and would be misleading.

### B2. `signal` / `signal avg` — real, varying values (not a placeholder)

Three samples, 5 s apart:

**delta**
```
--- sample 1  t=17:37:45 ---   signal: 0 [0, 0] dBm      signal avg: -18 [-18, -21] dBm
--- sample 2  t=17:37:50 ---   signal: -36 [-36, -42] dBm  signal avg: -35 [-35, -42] dBm
--- sample 3  t=17:37:55 ---   signal: -36 [-36, -43] dBm  signal avg: -35 [-35, -42] dBm
```
**ghost**
```
--- sample 1  t=17:37:55 ---   signal: -26 [-32, -26] dBm  signal avg: -26 [-31, -26] dBm
--- sample 2  t=17:38:00 ---   signal: -27 [-32, -27] dBm  signal avg: -26 [-32, -26] dBm
--- sample 3  t=17:38:05 ---   signal: -26 [-32, -26] dBm  signal avg: -25 [-30, -25] dBm
```

**The value moves.** delta's `signal avg` swung −18 → −35 dBm within 5 s; ghost's per-chain
values vary sample to sample. Per-antenna-chain values are populated (`[-36, -42]` = two
chains). The `signal: 0 [0, 0]` in delta sample 1 is the transient "no frame since last
read" case, not a stuck placeholder — it resolved to −36 dBm on the next sample.
The project's own mesh-mode results corroborate (README): "Signal | -33 to -49 dBm".

**Verdict: usable per-peer signal instrument, in both managed and mesh mode.**

### B3. `survey dump` — **returns nothing at all**

| Field | Present? |
|---|---|
| `noise` | **MISSING** (also −256 in `/proc/net/wireless`) |
| `channel active time` | **MISSING** |
| `channel busy time` | **MISSING** |
| `channel receive time` | **MISSING** |
| `frequency` | **MISSING** |

`iw dev wlP1p1s0 survey dump` produced **zero bytes of output** on both drones. `rtw88`
does not implement the `survey_dump` cfg80211 op for this chip.

**Consequence, stated as the prompt frames it:** busy time is **not** present, so this is
**not** a usable channel occupancy meter, and the "no spectrum analyzer purchase needed"
conclusion **does not hold** for the 2.4 GHz side. See the verdict table.

### B4. `tx bitrate` / `tx retries` / `tx failed` / `beacon loss` — bitrate real, counters dead

| Field | delta | ghost | Monotonic across reads 10 s apart? |
|---|---|---|---|
| `tx bitrate` | 866.7 MBit/s VHT-MCS 9 80MHz short GI VHT-NSS 2 | 780.0 MBit/s VHT-MCS 8 80MHz short GI VHT-NSS 2 | n/a (rate, not counter) — populated and plausible |
| `tx retries` | 0 | 0 | **No — pinned at 0** |
| `tx failed` | 0 | 0 | **No — pinned at 0** |
| `beacon loss` | 0 | 0 | **No — pinned at 0** |
| `rx packets` (control) | 49905 → 50286 → 50387 → 50486 | 45183 → 45740 → 45847 → 45948 | **Yes — increments normally** |
| `tx packets` (control) | 3762 → 3789 → 3793 → 3795 | 191 → 191 → 191 → 191 | delta increments; ghost idle |

The control counters (`rx packets`, `tx packets`) advance across the same reads in which
`tx retries` / `tx failed` / `beacon loss` stay at exactly 0. On delta, 33 packets were
transmitted during the sampling window with zero retries recorded. **These three counters
are not implemented by `rtw88` — they are not a real zero.** The retry/failure column of
the coexistence matrix cannot be filled from `station dump`.

`tx duration` and `rx duration` are also `0 us` on both — likewise unimplemented, which
independently rules out deriving airtime from the station statistics.

### B5. Monitor mode — **supported** (listed only; not entered)
```
	Supported interface modes:
		 * IBSS
		 * managed
		 * AP
		 * AP/VLAN
		 * monitor          <-- supported
		 * mesh point       <-- supported
```
Reported only. No interface was created, and monitor mode was not entered.

### B6. `iperf3` — **installed on both**
```
delta:  iperf 3.9 (cJSON 1.7.13)
ghost:  iperf 3.9 (cJSON 1.7.13)
```

### B7. Peers currently associated

**Zero mesh peers** (there is no mesh — C-2). Each drone has exactly **one** association,
to the bench AP:

| Drone | Peer MAC | Role | Count |
|---|---|---|---|
| delta | `e4:fa:c4:57:07:39` | TP-Link_073A AP | 1 |
| ghost | `e4:fa:c4:57:07:39` | TP-Link_073A AP | 1 |

Both drones are associated to the same AP. No drone-to-drone 802.11 link exists right now.

---

## Section C — manual control receiver link (PX4 ULog)

> **Note on location.** The prompt puts Section C on drone 1. **No `.ulg` file exists on
> any drone** (`find / -xdev -name "*.ulg"` returned nothing on delta and nothing on
> ghost). The fleet's ULogs live on the ground-station laptop. Section C was run there,
> read-only, against the newest log.

**Logs analysed** (newest first):
```
2026-06-10 17:09  110654778  /home/ssg/multi-robot-coordination/px4_logs/log_71_UnknownDate.ulg   <- newest
2026-06-10 16:44   54162069  /home/ssg/multi-robot-coordination/px4_logs/log_70_UnknownDate.ulg
2026-03-21 23:46   10437431  /home/ssg/multi-robot-coordination/logs/log_57_UnknownDate.ulg
```

**Vehicle identity (from `ulog_info -v log_71`):**
```
Logging start time: 0:03:23, duration: 0:05:53
Dropouts: count: 6623, total duration: 161.4 s, max: 3015 ms, mean: 24 ms
SW Version: v1.15.4
 ver_hw: CUBEPILOT_CUBEORANGEPLUS
 sys_mcu: STM32H7[4|5]xxx, rev. V
 sys_os_name: NuttX
 sys_uuid: 000600000000323136383132510c00290023
```
Note the dropout load: **6623 dropouts totalling 161.4 s** in a 353 s log — the logger was
saturated. This inflates every observed logging interval.

### C1. Receiver protocol, port, baud

> **See contradiction C-3: `RC_INPUT_PROTO` is absent from this airframe's 1136 parameters.**

```python
>>> 'RC_INPUT_PROTO' in u.initial_parameters
False
>>> len(u.initial_parameters)
1136
>>> [k for k in ps if 'RSSI' in k or 'CRSF' in k or 'SBUS' in k or 'PROTO' in k]
['GPS_1_PROTOCOL', 'MAV_PROTO_VER', 'PWM_SBUS_MODE', 'RC_RSSI_PWM_CHAN',
 'RC_RSSI_PWM_MAX', 'RC_RSSI_PWM_MIN']
```

Equivalent evidence used instead:

| Source | Value | Meaning |
|---|---|---|
| `input_rc.input_source` (live, in-log, constant) | **4** | `RC_INPUT_SOURCE_PX4IO_SBUS` — **SBUS via the PX4IO co-processor** |
| `input_rc.channel_count` (constant) | 18 | 18-channel SBUS frame |
| `RC_CHAN_CNT` | 18 | matches |
| `PWM_SBUS_MODE` | present | SBUS handling on the IO side |
| `RC_RSSI_PWM_CHAN` | **0** | analog-RSSI-on-a-PWM-channel is **disabled / unmapped** |
| `RC_RSSI_PWM_MIN` / `MAX` | 1000 / 2000 | defaults, unused while CHAN=0 |
| Archived `px4_params/*.params` (July 2025) | `RC_INPUT_PROTO 2` | 2 = **SBUS**, corroborates |

**Serial port assignment:** RC does **not** arrive on a configurable serial port. It enters
through the **PX4IO co-processor's dedicated RC input pin** (`px4io_status` is logged,
`input_source = 4 = PX4IO_SBUS`), so there is no `SER_*` baud to report for the RC link.
For completeness, the configured serial ports are:
```
  SER_TEL1_BAUD  = 57600
  SER_TEL2_BAUD  = 115200
  SER_GPS1_BAUD  = 0        (auto)
  SER_GPS2_BAUD  = 921600
  UXRCE_DDS_CFG  = 202
```
SBUS itself runs at its fixed 100000 baud, 8E2 inverted, into PX4IO — not host-configurable.

### C2. Logged topics — `input_rc` present, `rc_channels` **absent**

| Topic | Present | n | mean interval |
|---|---|---|---|
| `input_rc` | **YES** | 342 | 1032.21 ms |
| `rc_channels` | **NO** | — | — |
| `manual_control_setpoint` | YES | 2758 | 128.08 ms |
| `manual_control_switches` | YES | 172 | 2063.62 ms |

Identical picture in all three logs:
```
log_71: input_rc: True  rc_channels: False
log_70: input_rc: True  rc_channels: False
log_57: input_rc: True  rc_channels: False
```
`SDLOG_PROFILE = 2047` (all profiles enabled) — `rc_channels` is simply not in PX4
v1.15.4's logged-topic set, and no parameter adds it.

### C3. `input_rc` field statistics — **the RSSI column cannot be filled as designed**

`input_rc` fields present:
```
['channel_count', 'input_source', 'link_quality', 'rc_failsafe', 'rc_lost',
 'rc_lost_frame_count', 'rc_ppm_frame_length', 'rc_total_frame_count', 'rssi',
 'rssi_dbm', 'timestamp', 'timestamp_last_signal', 'values[0]'...'values[17]']
```

**log_71 (newest, n=342):**
```
   rssi_dbm               min=       nan max=       nan mean=       nan std=     nan uniq=1
   rssi                   min=    43.000 max=    44.000 mean=    43.170 std=   0.375 uniq=2
   link_quality           min=    -1.000 max=    -1.000 mean=    -1.000 std=   0.000 uniq=1
   rc_lost                min=     0.000 max=     0.000 mean=     0.000 std=   0.000 uniq=1
   rc_failsafe            min=     0.000 max=     0.000 mean=     0.000 std=   0.000 uniq=1
   rc_total_frame_count   min= 12999.000 max= 36400.000 mean= 24504.763 std=6690.275 uniq=342
   rc_lost_frame_count    min=     0.000 max=     0.000 mean=     0.000 std=   0.000 uniq=1
   input_source           min=     4.000 max=     4.000 mean=     4.000 std=   0.000 uniq=1
   channel_count          min=    18.000 max=    18.000 mean=    18.000 std=   0.000 uniq=1
```
**log_70 (n=168):**
```
   rssi_dbm      nan / nan / nan  (uniq=1)
   rssi          43.000 / 44.000 / 43.143 / std 0.350 (uniq=2)
   link_quality  -1.000 / -1.000 / -1.000 / std 0.000 (uniq=1)
   rc_lost       0 / rc_failsafe 0 / rc_lost_frame_count 0
```
**log_57 (n=26):**
```
   rssi_dbm      nan / nan / nan  (uniq=1)
   rssi          43.000 / 44.000 / 43.115 / std 0.319 (uniq=2)
   link_quality  -1.000 / -1.000 / -1.000 / std 0.000 (uniq=1)
   rc_lost       0 / rc_failsafe 0 / rc_lost_frame_count 0
```

**Explicit statement, as the prompt requires it:**

- `rssi_dbm` is **NaN in every sample of every log** — the receiver reports no dBm figure.
- `link_quality` is **constant −1** — PX4's "not supported by this receiver" sentinel.
- `rssi` takes exactly **two values (43, 44)** across a 353-second flight, std 0.375. On a
  0–100 scale that is a **constant placeholder**, not a measurement. It is PX4IO's
  frame-success heuristic, not an RF power reading.
- `rc_lost`, `rc_failsafe`, `rc_lost_frame_count` are **identically zero** throughout —
  no link degradation was ever recorded, in any log.

**Therefore: the receiver protocol is not reporting real RSSI, and the coexistence matrix's
control-link column cannot be filled as designed.** The root cause is protocol-level: SBUS
is a one-way frame format with no RSSI field, and the RSSI-over-PWM fallback is unmapped
(`RC_RSSI_PWM_CHAN = 0`). No amount of logging configuration recovers a quantity the link
never carries.

The one genuinely usable signal here is `rc_total_frame_count` (342 unique values,
monotonic 12999 → 36400), which supports a **frame-rate / frame-loss** metric — a
degradation proxy, but not RSSI and not link quality.

### C4. Temporal resolution vs a 40 ms burst — **cannot resolve it**

| Log | n | mean dt | **median dt** | min dt | max dt |
|---|---|---|---|---|---|
| log_71 | 342 | 1032.2 ms | **987.0 ms** | 478.8 ms | 4988.0 ms |
| log_70 | 168 | 1134.7 ms | **519.0 ms** | 483.4 ms | 6487.8 ms |
| log_57 | 26 | 820.3 ms | **505.3 ms** | 492.0 ms | 3002.8 ms |

**ULog sample interval for `input_rc`: ~500 ms floor (≈2 Hz), degrading to ~1000 ms mean
under logger dropout.**

**It cannot resolve a 40 ms TDMA transmit burst.** The burst is ~12× shorter than the
*best* observed sample interval and ~25× shorter than the mean. Nyquist requires ≥50 Hz
sampling (≤20 ms) to characterise a 40 ms event; the log provides 1–2 Hz. A burst would be
invisible — most bursts fall entirely between two consecutive samples.

**Which PX4 parameter would need to change: none exists.** See contradiction C-4. The
500 ms floor matches PX4 v1.15.4's compile-time `add_topic("input_rc", 500)` in
`src/modules/logger/logged_topics.cpp`. `SDLOG_PROFILE` (already 2047) selects which topics
are logged, not their rate; there is no `SDLOG_RATE_*` or per-topic rate parameter in the
1136-parameter set:
```
SDLOG params: {'SDLOG_BOOT_BAT': 0, 'SDLOG_DIRS_MAX': 0, 'SDLOG_MISSION': 0,
               'SDLOG_MODE': 0, 'SDLOG_PROFILE': 2047, 'SDLOG_UTC_OFFSET': 0,
               'SDLOG_UUID': 1}
```

**Stopping here as instructed. Nothing was changed.**

### C5. `pyulog` — available, no venv, nothing installed

```
$ python3 -c "import pyulog,sys;print('pyulog OK',pyulog.__version__,sys.executable)"
pyulog OK 1.1.0 /usr/bin/python3
$ which ulog_info ulog_messages
/home/ssg/.local/bin/ulog_info
/home/ssg/.local/bin/ulog_messages
```

| Item | Value |
|---|---|
| Location | **Ground-station laptop**, system interpreter `/usr/bin/python3` |
| Version | pyulog **1.1.0** |
| Venv path | **None — not in a venv.** It is a `--user` install; CLI entry points in `/home/ssg/.local/bin` |
| Command used | `python3 <script>.py <log>.ulg` and `ulog_info -v <log>.ulg` |
| On the drones | **ABSENT** — `import pyulog` raises `ModuleNotFoundError` on delta; no `pyulog` directory anywhere on the filesystem |

No venv was created and nothing was installed. `/home/ssg/venvs` exists but was not used
or modified.

---

## Section D — timebase

### D1. Raw output — `timedatectl`, and NTP state (chrony is absent)

**delta**
```
--- date -u ---  2026-08-18T21:41:21.035153900
               Local time: Tue 2026-08-18 17:41:21 EDT
           Universal time: Tue 2026-08-18 21:41:21 UTC
                 RTC time: Tue 2026-08-18 21:41:22
                Time zone: America/New_York (EDT, -0400)
System clock synchronized: yes
              NTP service: active
--- chronyc tracking ---   bash: line 3: chronyc: command not found
--- chronyc sources  ---   bash: line 4: chronyc: command not found
--- systemd-timesyncd chrony chronyd ptp4l phc2sys ---
    active inactive inactive inactive inactive
--- /dev/ptp* ---  ls: cannot access '/dev/ptp*': No such file or directory
```
**ghost**
```
--- date -u ---  2026-08-18T21:41:21.633988939
System clock synchronized: yes
              NTP service: active
--- chronyc tracking ---   bash: line 3: chronyc: command not found
--- systemd-timesyncd chrony chronyd ptp4l phc2sys ---
    active inactive inactive inactive inactive
--- /dev/ptp* ---  ls: cannot access '/dev/ptp*': No such file or directory
```

**`chronyc` is absent on both drones — reported as missing, not installed.** Time sync is
`systemd-timesyncd`. Its state, which carries the equivalent offset/jitter data:

**delta**
```
FallbackNTPServers=0.pool.ntp.org 1.pool.ntp.org 0.fr.pool.ntp.org
ServerName=1.pool.ntp.org
ServerAddress=198.137.202.56
PollIntervalUSec=34min 8s
NTPMessage={ Leap=0, Version=4, Mode=4, Stratum=2, Precision=-24,
             RootDelay=1.174ms, RootDispersion=8.819ms, Reference=11FD0425,
             Ignored=no PacketCount=7, Jitter=4.991ms }
Frequency=1515510
```
**ghost**
```
FallbackNTPServers=0.pool.ntp.org 1.pool.ntp.org 0.fr.pool.ntp.org
ServerName=0.pool.ntp.org
ServerAddress=170.187.147.56
PollIntervalUSec=34min 8s
NTPMessage={ Leap=0, Version=4, Mode=4, Stratum=2, Precision=-23,
             RootDelay=118.164ms, RootDispersion=9.628ms, Reference=5223A292,
             Ignored=no PacketCount=7, Jitter=12.008ms }
Frequency=1970310
```
**laptop (reference)**
```
ServerName=ntp.ubuntu.com
NTPMessage={ Stratum=2, Precision=-25, RootDelay=7.690ms, RootDispersion=747us,
             PacketCount=13, Jitter=6.859ms }
```

**Measured offsets** (persistent SSH, `offset = t_remote − (t_local_before + t_local_after)/2`):
```
--- delta vs laptop ---
  rtt=  84.0 ms  offset=   +39.6 ms  +/-42.0 ms      <- first call, connection warm-up
  rtt=  15.8 ms  offset=    +5.8 ms  +/-7.9 ms
  rtt=  15.5 ms  offset=    +5.6 ms  +/-7.8 ms
  rtt=  16.8 ms  offset=    +3.8 ms  +/-8.4 ms
  rtt=  15.4 ms  offset=    +5.7 ms  +/-7.7 ms
--- ghost vs laptop ---
  rtt=  53.6 ms  offset=   +26.7 ms  +/-26.8 ms      <- first call, connection warm-up
  rtt=  11.1 ms  offset=    +5.2 ms  +/-5.6 ms
  rtt=  11.0 ms  offset=    +5.6 ms  +/-5.5 ms
  rtt=  11.0 ms  offset=    +6.0 ms  +/-5.5 ms
  rtt=  11.3 ms  offset=    +5.5 ms  +/-5.6 ms
```
Both drones sit ≈ **+5.6 ms** ahead of the laptop. **Drone-to-drone offset ≈ 0.1 ms at
this instant**, but the per-sample uncertainty is ±8 ms (delta) / ±5.5 ms (ghost), so the
measurement cannot resolve better than about ±10 ms.

### D2. NTP/PTP between the four Jetsons — **none; they drift independently**

- **No PTP.** No `/dev/ptp*` on either drone; `ptp4l` and `phc2sys` inactive. The RTL8822CE
  and RTL8111 provide no PTP hardware clock in this configuration.
- **No local NTP server.** Neither drone has `LinkNTPServers` or `SystemNTPServers` set;
  both fall through to the **public `pool.ntp.org`**.
- **They do not even share a server.** delta → `1.pool.ntp.org` (198.137.202.56);
  ghost → `0.pool.ntp.org` (170.187.147.56). Their errors are uncorrelated, so the common-mode
  cancellation you would get from a shared reference is absent.
- **Poll interval is at maximum: 34 min 8 s (2048 s).** Between polls each Jetson free-runs
  on its own crystal. At a typical uncorrected 10–20 ppm, that is **20–40 ms of accumulated
  drift** by the end of a poll interval.
- Buckshee and Thunderstrike were unreachable and could not be checked; there is no reason
  to expect a different configuration.

**Each Jetson disciplines its clock independently against a different internet server.**

### D3. PX4 log time vs ROS 2 system time — offset IS recorded

The uXRCE-DDS timesync mechanism is **enabled and logged**.
```
  UXRCE_DDS_CFG   = 202
  UXRCE_DDS_KEY   = 1
  UXRCE_DDS_DOM_ID = 0
  UXRCE_DDS_PTCFG = 0
  UXRCE_DDS_SYNCC = 0      (client-side sync off)
  UXRCE_DDS_SYNCT = 1      (timesync ON)
```
`timesync_status` is a logged uORB topic — **the recorded offset is available in the ULog**:
```
=== timesync_status ===   (n=25, log_71)
 fields: ['estimated_offset', 'observed_offset', 'remote_timestamp',
          'round_trip_time', 'source_protocol', 'timestamp']
   observed_offset    n=25 min=-5461024.0 max=-5458970.0 mean=-5460494.8 std=543.5   [us]
   estimated_offset   n=25 min=-5459496.0 max=-5459355.0 mean=-5459432.8 std= 46.4   [us]
   round_trip_time    n=25 min=   2084.0 max=   6859.0 mean=   5186.5 std=1077.0     [us]
   source_protocol    n=25 min=      2.0 max=      2.0 mean=      2.0 std=  0.0
```
`source_protocol = 2` = uXRCE-DDS. The estimated PX4↔ROS offset was **−5.459 s** with a
**standard deviation of 46.4 µs**, over a round-trip of **5.19 ms** (min 2.08 ms).

**PX4-to-ROS alignment on a single vehicle is excellent: ~±50 µs of jitter on the estimated
offset, bounded by ~±2.6 ms (half the RTT) of absolute uncertainty.** This is the one clock
relationship in the system that is already good enough for 40 ms events, and it is already
being logged. Also worth flagging: `UXRCE_DDS_DOM_ID = 0` in this log, which must match the
vehicle's `ROS_DOMAIN_ID`.

### D4. Achievable cross-vehicle timestamp alignment

| Contributor | Magnitude |
|---|---|
| NTP jitter, delta | 4.991 ms |
| NTP jitter, ghost | 12.008 ms |
| Root dispersion | 8.8 ms / 9.6 ms |
| Root delay, ghost | 118.164 ms (asymmetric-path risk) |
| Different servers → uncorrelated error | no common-mode cancellation |
| Free-run drift over the 2048 s poll interval | ~20–40 ms at 10–20 ppm |
| Measured drone-to-drone offset (this instant) | ≈0.1 ms, uncertainty ±10 ms |

**Achievable cross-vehicle timestamp alignment today: ~±10 ms at best, immediately after a
poll, degrading to ~±25–50 ms late in a poll interval.**

Consequences, both of which the prompt flags as dependent on this number:

1. **One-way LoRa latency cannot be measured on the Jetson wall clock.** The quantity of
   interest is a ~20–30 ms airtime plus queueing; the clock uncertainty (±10–50 ms) is the
   same size or larger. Any such measurement would be dominated by clock error.
   **However** — the TDMA firmware maintains its own shared `cycle_epoch` / `cycle_clock`
   across all nodes, disciplined to well inside one 40 ms slot by construction. Latency
   should be derived on the **firmware's TDMA timebase** (`uptime_ms` / `cycle_clock`,
   already in `Packet`, A3) rather than on NTP. That reduces the problem to forwarding two
   existing fields over serial — code, not hardware.
2. **Burst-aligned degradation on the control link cannot be cross-correlated between
   vehicles** at the current sync quality, independent of the C3/C4 findings which already
   block that measurement for other reasons.

**Fix requires no hardware:** point both drones at a **single local NTP server** (the bench
gateway, 192.168.0.136) with a short `PollIntervalMaxSec`. On the wired LAN that yields
sub-millisecond agreement; over WiFi, ~1–2 ms. Both are comfortably inside a 40 ms slot.
This is a `timesyncd.conf` edit — deliberately **not** performed under the read-only
constraint.

---

## Section E — existing log inventory

### E1. Inventory

**`*.ulg` — none on any drone.**
```
delta:  find / -xdev -name "*.ulg"  ->  (no output)
ghost:  find / -xdev -name "*.ulg"  ->  (no output)
```
All ULogs are on the ground-station laptop (3 total, complete list):
```
2026-06-10 17:09  110654778  /home/ssg/multi-robot-coordination/px4_logs/log_71_UnknownDate.ulg
2026-06-10 16:44   54162069  /home/ssg/multi-robot-coordination/px4_logs/log_70_UnknownDate.ulg
2026-03-21 23:46   10437431  /home/ssg/multi-robot-coordination/logs/log_57_UnknownDate.ulg
```

**rosbag2 directories on delta** (by `metadata.yaml` mtime, newest first — 10 total):
```
2026-02-24 15:49  /mnt/nova_ssd/workspaces/isaac_ros-dev/loop_closure_20260224_154605/robot_2_cuvslam/
2026-02-24 15:47  /mnt/nova_ssd/workspaces/isaac_ros-dev/loop_closure_20260224_154605/robot_1_cuvslam/
2026-01-10 19:35  .../install/nvblox_ros/share/nvblox_ros/test_cases/rosbags/nvblox_pol/          [test fixture]
2026-01-10 19:35  .../install/nvblox_ros/share/nvblox_ros/test/integration_tests/.../nvblox_pol/  [test fixture]
2026-01-10 19:24  .../install/isaac_ros_test_cmake/share/.../dummy_bag/                           [test fixture]
2025-12-12 03:31  .../src/isaac_ros_visual_slam/.../rosbags/r2b_galileo/                          [test fixture]
2025-12-10 02:46  .../src/isaac_ros_nvblox/.../rosbags/nvblox_pol/                                [test fixture]
2025-12-08 02:53  .../src/isaac_ros_common/isaac_ros_test_cmake/resources/dummy_bag/              [test fixture]
2024-05-03 19:40  .../isaac_ros_assets/isaac_ros_visual_slam/quickstart_bag/                      [test fixture]
2024-04-09 04:33  .../isaac_ros_assets/isaac_ros_nvblox/quickstart/                               [test fixture]
```
Only the two `loop_closure_20260224_154605` bags are real captured data; the rest ship with
the Isaac ROS packages.

**LoRa serial / CSV logs:**
```
delta:
2026-08-12 18:17       2583  /mnt/nova_ssd/workspaces/isaac_ros-dev/experiment_logs/lora_rx_20260812_221659.csv
2026-06-11 20:29      11891  .../experiment_logs/pane_Tab_5__Simple_Exploration_Planner_1781224060.log
2026-03-10 19:02       9245  .../src/radiohive/logs/radiohive_mesh_20260310_190057_node1of4.log
2026-03-09 20:49       6231  /mnt/nova_ssd/workspaces/mesh_test_result.log

ghost:
2026-08-12 18:17       2665  /mnt/nova_ssd/workspaces/isaac_ros-dev/experiment_logs/lora_rx_20260812_221657.csv
2026-08-12 18:16         28  .../experiment_logs/link_20260812_221657.csv
2026-08-12 18:16         23  .../experiment_logs/keyframes_20260812_221657.csv
2026-08-12 18:16         14  .../experiment_logs/coverage_20260812_221657.csv
2026-08-12 12:55     549815  .../experiment_logs/guard_20260812_165123.csv
2026-08-12 12:04     144507  .../experiment_logs/guard_20260812_155900.csv
2026-08-12 12:16     119514  .../experiment_logs/guard_20260812_160850.csv
```
Note the filename `radiohive_mesh_20260310_190057_node1of4.log` — independent corroboration
of the 4-node deployment (C-5).

### E2. `ros2 bag info` — newest real bag

`ros2` CLI is not available on the host outside the container, so the bag's own
`metadata.yaml` is quoted instead (same content the CLI renders).

`/mnt/nova_ssd/workspaces/isaac_ros-dev/loop_closure_20260224_154605/robot_2_cuvslam/`
```yaml
rosbag2_bagfile_information:
  version: 5
  storage_identifier: sqlite3
  duration:      {nanoseconds: 41703223360}          # 41.70 s
  starting_time: {nanoseconds_since_epoch: 1771966115221726048}
  message_count: 67859
  topics_with_message_count:
    - /nvblox_node/static_esdf_pointcloud            sensor_msgs/msg/PointCloud2       387
    - /camera0/depth/image_rect_raw/theora           theora_image_transport/msg/Packet   3
    - /nvblox_node/shapes_to_clear                   visualization_msgs/msg/MarkerArray  0
    - /nvblox_node/dynamic_occupancy_grid            nav_msgs/msg/OccupancyGrid          0
    - /nvblox_node/back_projected_depth/
        camera0_depth_optical_frame                  sensor_msgs/msg/PointCloud2         2
    ... (nvblox / cuVSLAM mapping topics; 67859 messages total over 41.7 s)
```
**No LoRa, no WiFi, and no RF topic of any kind is recorded in any rosbag.** These are
pure mapping/loop-closure captures. There is no historical RF dataset to mine.

### E3. Existing LoRa log — header and samples

**delta** — `/mnt/nova_ssd/workspaces/isaac_ros-dev/experiment_logs/lora_rx_20260812_221659.csv`
(2583 B, 63 lines, 2026-08-12 18:17)
```
t,kind,peer,rssi,snr,x,y,z,heading,bytes
1786573019.217,pose,2,-36,13.2,0.00,0.00,0.00,0.0,0
1786573019.218,msg,2,,,,,,,19
1786573019.407,pose,2,-37,13.0,0.00,0.00,0.00,0.0,0
1786573019.409,msg,2,,,,,,,19
1786573019.597,pose,2,-36,13.5,0.00,0.00,0.00,0.0,0
--- tail ---
1786573024.731,msg,2,,,,,,,19
1786573024.919,pose,2,-36,13.0,0.00,0.00,0.00,0.0,0
1786573024.921,msg,2,,,,,,,19
```
**ghost** — `.../experiment_logs/lora_rx_20260812_221657.csv` (2665 B, 65 lines)
```
t,kind,peer,rssi,snr,x,y,z,heading,bytes
1786573017.916,pose,0,-37,13.2,0.00,0.00,0.00,0.0,0
1786573017.916,msg,0,,,,,,,19
1786573017.974,pose,0,-37,12.5,0.00,0.00,0.00,0.0,0
1786573017.976,msg,0,,,,,,,19
1786573018.165,pose,0,-37,13.0,0.00,0.00,0.00,0.0,0
```
Companion `link_20260812_221657.csv` is a header with no rows:
```
t,peer,delivery,rx_hz,age_s
```

**Which of the six required fields are present:**

| Required field | Present? | Where / why not |
|---|---|---|
| **RX timestamp** | **YES** | column `t` — Jetson wall clock, epoch seconds, ms resolution |
| **RSSI** | **YES** | column `rssi` — dBm integer |
| **SNR** | **YES** | column `snr` — dB, 0.25 dB granularity |
| **seq** | **NO** | exists on-air in `Packet.sequence` and in the 19 B claim payload's `seq`, but the `0x81 PEER_POSE` serial frame does not forward it, so it never reaches the CSV |
| **TX power** | **NO** | does not exist anywhere — not in `Packet`, not in the serial protocol, not in the codec (A3/A5) |
| **TX timestamp** | **NO** | exists on-air as `Packet.uptime_ms`, not forwarded over serial; the 32 B full-packet codec has `t_ms` but that variant is not deployed and would be refused by the slot cap (C-6) |

**3 of 6 present.** The two that block a PDR/latency curve (`seq`, TX timestamp) already
exist *on the air* and are lost at the firmware→ROS serial boundary. TX power does not
exist at all.

### E4. Field statistics — real measurements, not placeholders

```
=== delta  rows=62 ===
  columns: ['t','kind','peer','rssi','snr','x','y','z','heading','bytes']
  rssi  n= 31 min=  -37.00 max=  -35.00 mean=  -36.00 std= 0.254 uniq=3
  snr   n= 31 min=   12.20 max=   13.50 mean=   12.91 std= 0.347 uniq=6
  pose interval: mean=190.1 ms  min=189.0  max=192.0  span=5.70 s
  duration 5.7s -> 5.44 pose/s

=== ghost  rows=64 ===
  rssi  n= 32 min=  -37.00 max=  -37.00 mean=  -37.00 std= 0.000 uniq=1
  snr   n= 32 min=   12.00 max=   13.20 mean=   12.62 std= 0.322 uniq=6
  pose interval: mean=185.7 ms  min=58.0  max=191.0  span=5.76 s
  duration 5.8s -> 5.56 pose/s
```

**Distinguishing measurement from placeholder:**
- **SNR is unambiguously a real measurement** on both drones: 6 distinct values each,
  std ≈ 0.33 dB, quantised to 0.25 dB — the native SX1262 SNR granularity. A placeholder
  cannot produce that.
- **RSSI is real but under-exercised.** delta shows 3 distinct values (−35/−36/−37,
  std 0.254). ghost shows a single value (−37, std 0) — this is a **5.8-second capture of
  two radios sitting on a bench a few metres apart**, where the true path loss genuinely
  does not move, not a stuck field. delta's variation over the same span confirms the field
  tracks the channel.
- **Both captures are ~5.7 s long with ~31 packets.** This is a smoke test, not an
  experiment. There is **no existing dataset at range**, across power levels, or with any
  RSSI dynamic range worth fitting a curve to.
- **Cadence 190.1 ms confirms NUM_NODES=4** (4×40+30), see C-5. ghost's `min=58.0 ms`
  outlier is a duplicate/retimed frame at the start of the capture.

---

## Section F — distance and RF hardware inventory

### F1. `lsusb` / `lspci` — no SDR present

**delta — `lsusb`** (ghost byte-identical)
```
Bus 002 Device 003: ID 8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i
Bus 002 Device 002: ID 2109:0822 VIA Labs, Inc. USB3.1 Hub
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 003: ID 0bda:c822 Realtek Semiconductor Corp. Bluetooth Radio
Bus 001 Device 005: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
Bus 001 Device 004: ID 0403:6001 Future Technology Devices International, Ltd FT232 Serial (UART) IC
Bus 001 Device 002: ID 2109:2822 VIA Labs, Inc. USB2.0 Hub
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```
**delta — `lspci`** (ghost byte-identical)
```
0001:00:00.0 PCI bridge: NVIDIA Corporation Device 229e (rev a1)
0001:01:00.0 Network controller: Realtek Semiconductor Co., Ltd. RTL8822CE 802.11ac PCIe Wireless Network Adapter
0004:00:00.0 PCI bridge: NVIDIA Corporation Device 229c (rev a1)
0004:01:00.0 Non-Volatile memory controller: Device fe19:0001
0008:00:00.0 PCI bridge: NVIDIA Corporation Device 229c (rev a1)
0008:01:00.0 Ethernet controller: Realtek Semiconductor Co., Ltd. RTL8111/8168/8411 PCI Express Gigabit Ethernet Controller (rev 15)
```

**RF-capable devices found:**

| Device | Band | Notes |
|---|---|---|
| RTL8822CE (PCIe) | 2.4 / 5 GHz | the known WiFi radio |
| **Realtek Bluetooth Radio `0bda:c822`** | **2.4 GHz** | **beyond the three known radios** — the BT function of the RTL8822CE combo. A fourth 2.4 GHz emitter/receiver on each airframe |
| Heltec SX1262 via CP2102 `10c4:ea60` | 915 MHz | the known LoRa radio |
| PX4 CubeOrange+ via FTDI `0403:6001` | — | serial only, not RF |
| RealSense D435i `8086:0b3a` | — | IR structured light, not RF |

**Explicit statement: NO SDR is present on any drone.** No RTL-SDR (`0bda:2838`/`2832`),
no HackRF (`1d50:6089`), no USRP, no Airspy, no LimeSDR, no bladeRF — on USB or PCI, on
either reachable drone. The fleet has **no general-purpose spectrum measurement device**.

Also worth recording: the Bluetooth radio is an **uncontrolled 2.4 GHz emitter co-located
with the mesh** (ch 6 = 2437 MHz). It should be accounted for in any coexistence matrix.

### F2. GPS — **no GPS data exists in any log; the conclusion is negative**

Parameters suggest a GPS is *configured*:
```
  SYS_HAS_GPS    = 1
  GPS_1_CONFIG   = 201          (GPS1 assigned to a port)
  GPS_1_PROTOCOL = 1            (u-blox)
  GPS_1_GNSS     = 0            (default constellations)
  SER_GPS1_BAUD  = 0            (auto)
  GPS_2_CONFIG   = 0            (none)
  UAVCAN_SUB_GPS = 1
  COM_ARM_WO_GPS = 1            (arming without GPS permitted)
  EKF2_GPS_CTRL  = 15
```
But no GPS data was ever logged, in **any** of the three ULogs:
```
log_71:  vehicle_gps_position: False   sensor_gps: False
log_70:  vehicle_gps_position: False   sensor_gps: False
log_57:  vehicle_gps_position: False   sensor_gps: False
```

- **Satellite count: unavailable** — the topic does not exist in the logs.
- **Horizontal accuracy (`eph`): unavailable** — same reason.
- **There is no outdoor ULog in the inventory.** All three logs are indoor VIO flights:
  `vehicle_visual_odometry` (4810 msgs), `vehicle_optical_flow` (19093),
  `sensor_optical_flow`, `estimator_ev_pos_bias`, and `distance_sensor` are all present,
  while GPS is absent and `COM_ARM_WO_GPS = 1` permits flying without it.

**Conclusion, stated as the negation the prompt asks for:**

> **Logged GPS position does NOT currently substitute for a laser rangefinder at 25 m
> station spacing out to 300 m.** The substitution requires GPS horizontal accuracy better
> than ~3 m to be demonstrated; no GPS fix, no satellite count, and no `eph` value exists
> anywhere in the fleet's logs, so the accuracy precondition is not merely unmet — it is
> **untested**. It cannot even be confirmed from this read-only audit that a GPS module is
> physically attached, since `SYS_HAS_GPS = 1` and `GPS_1_CONFIG = 201` are configuration
> intent, not detection. Confirming presence requires either a physical inspection or an
> outdoor power-up, both outside the scope of a read-only audit.

### F3. u.FL/SMA pigtails, attenuators, spare antennas — **no inventory file exists**

```
find /home/ssg/multi-robot-coordination -iname "*inventory*" -o -iname "*BOM*" \
     -o -iname "*parts*" -o -iname "*pigtail*" -o -iname "*attenuator*"     ->  (no output)
find .../src -maxdepth 4 -iname "*inventory*" -o -iname "*BOM*" -o -iname "*parts*" -> (no output)
grep -rli "pigtail|attenuator|u.fl|SMA " --include=*.md --include=*.txt
     ->  /home/ssg/multi-robot-coordination/ghost_debug/POSTMORTEM.md   (prose only, not an inventory)
```

**There is no hardware inventory file in any repo.** Per the instruction not to guess:
**no claim is made** about whether pigtails, attenuators, or spare antennas exist. This is
unknown and must be resolved by physically checking the lab, not from the repos.

This matters for the TX power sweep: a conducted (cabled) measurement would need
pigtails and a matched attenuator, and none are documented. The sweep as specified is
over-the-air, which does not need them.

---

## Section G — verdict

### Reading notes

- **MEASURABLE NOW** — the instrument exists, is installed, and returns real varying data;
  verified in this audit.
- **NEEDS CODE** — the physical capability exists; software/config drops or never captures
  the quantity. No purchase.
- **NEEDS HARDWARE** — no installed device can produce the quantity. The specific part is
  named, and why nothing on board substitutes.

### Verdict table

| Measurement | Instrument | Status | Blocking gap | Cheapest resolution |
|---|---|---|---|---|
| LoRa RX RSSI and SNR per packet | SX1262 via RadioLib `getRSSI()`/`getSNR()`, fw `:567-568`; published as `radiohive/PeerPose.rssi/.snr` on `/lora/peer_pose` | MEASURABLE NOW | None. Verified end to end: 31 pose records/drone in `lora_rx_*.csv`, SNR 6 distinct values at 0.25 dB granularity, RSSI varying −35..−37 dBm | Nothing to buy. Log at range instead of on the bench — existing captures are 5.7 s at ~3 m |
| LoRa TX power sweep across 31 dB | SX1262; `#define LORA_PWR 10` (fw `:53`) consumed once by `radio.begin()` (fw `:371`) | NEEDS CODE | `setOutputPower()` is never called, and `handleSerialMessage()` (fw `:312-337`) implements only `0x01/0x02/0x03` — no set-power command. Sweep = one reflash per step, per node, today. Also no `tx_power` field in `Packet` so RX cannot label samples | Add `MSG_SET_TXPOWER 0x04` → `radio.setOutputPower(dBm)` (SX1262 range −9..+22 covers the full 31 dB), plus a 1-byte `tx_power` in `Packet`. 7 B of slot budget free (A4). ~15 lines, one reflash total, then fully scriptable from the Jetson |
| LoRa PDR and latency | `Packet.sequence` + `Packet.uptime_ms` (fw `:86-87`), already on air; fw already counts `nodes[].missed` from seq gaps (fw `:593-594`) | NEEDS CODE | Both fields are **dropped at the firmware→ROS boundary**: the `0x81 PEER_POSE` frame (fw `:210-222`) forwards only node/lat/lon/alt/hdg/rssi/snr. `lora_rx_*.csv` has no seq and no TX timestamp, so PDR and latency are both unrecoverable from existing logs | Extend `sendPeerPose()` by 8 B (`sequence` u32 + `uptime_ms` u32), add the two fields to `PeerPose.msg` and the `'<Bffffhf'` unpack in `lora_bridge_node.py:63`. **Zero on-air cost** — both already transmit. Derive latency on the TDMA `cycle_clock`, not NTP (see cross-vehicle row) |
| 915 MHz channel power with no packet | RadioLib SX126x `getRSSI(false)` (`GetRssiInst`) and `scanChannel()` (CAD) | NEEDS CODE | Neither is called anywhere in the firmware; the only call is packet-scoped `getRSSI()` at fw `:567`. Radio is also parked in continuous RX (`goRx()`, fw `:171-176`) with no idle sampling hook. **Caveat:** RadioLib is absent from all fleet machines (A1), so the installed version's signature could not be verified on-fleet — this is the one row whose "no purchase" verdict rests on an unread library version | Add a periodic `getRSSI(false)` sample in the guard interval (30 ms/cycle, currently idle) and emit it on the existing `0x83 STATUS_RESP`. If the pinned RadioLib predates `getRSSI(bool)`, upgrade the library — still no purchase |
| Mesh per peer signal and rate | `iw dev wlP1p1s0 station dump` → `signal`, `signal avg` (per-chain), `tx bitrate`; `rtw88_8822ce` | MEASURABLE NOW | None. Verified varying: delta `signal avg` −18 → −35 dBm across 5 s; ghost per-chain `[-32,-26]` moving; `tx bitrate` 866.7 / 780.0 MBit/s VHT-MCS 9/8. Project's own mesh-mode data agrees (−33..−49 dBm) | Nothing to buy. Note the mesh must actually be joined at 2437 MHz (B1) — it was not up during this audit |
| 2.4 GHz channel occupancy or noise | *(none installed)* | **NEEDS HARDWARE** | `iw survey dump` returns **zero output** on both drones — `noise`, `channel active time`, `channel busy time`, `channel receive time` all **missing**; `rtw88` does not implement `survey_dump` for the 8822CE. `/proc/net/wireless` noise = **−256** (unset sentinel). `tx duration`/`rx duration` = `0 us`, so airtime cannot be back-derived either. **No installed device substitutes:** the RTL8822CE is the only 2.4 GHz receiver with any measurement API and its driver returns nothing (monitor mode would at best give RSSI of *decodable 802.11 frames*, never true busy time or a noise floor including non-WiFi interferers — and entering it drops the mesh); the SX1262 is a 915 MHz part and cannot tune to 2.4 GHz; the Bluetooth radio exposes no arbitrary-channel power API | **RTL-SDR Blog V4** (~$40), covering 2.4 GHz for true channel power and noise floor including non-WiFi interferers. A second USB WiFi adapter with survey support (e.g. MT7612U, ~$25) is cheaper but measures only WiFi airtime and still needs a spare USB port |
| Mesh throughput between two vehicles | `iperf3 3.9` installed on delta and ghost | MEASURABLE NOW | None for the tooling. Blocked only procedurally: the 802.11s mesh was not up during this audit (contradiction C-2) and bringing it up is a networking change forbidden by the read-only constraint | Nothing to buy. Run `test_radiohive_pair.sh` to join the mesh at 2437 MHz, then `iperf3 -s` / `iperf3 -c` between the two vehicles |
| Control link RSSI and link quality | PX4 `input_rc` over **SBUS via PX4IO** (`input_source = 4`, constant; `RC_INPUT_PROTO 2` in archived params) | **NEEDS HARDWARE** | `rssi_dbm` = **NaN in every sample of all 3 logs**; `link_quality` = **constant −1** (PX4's "unsupported" sentinel); `rssi` takes **2 values (43, 44)** over a 353 s flight (std 0.375) — a constant placeholder, PX4IO's frame heuristic, not RF power. `rc_lost`/`rc_failsafe`/`rc_lost_frame_count` identically 0. Root cause is protocol-level: **SBUS is one-way and carries no RSSI field**, and the analog fallback is unmapped (`RC_RSSI_PWM_CHAN = 0`). **No installed device substitutes:** no logging or parameter change recovers a quantity the link never transmits; the RTL8822CE and SX1262 measure different links on different antennas and say nothing about the control receiver's RF path | **ExpressLRS/CRSF receiver** (e.g. RadioMaster RP1 or BetaFPV ELRS Lite, ~$20/vehicle) + matching TX module. CRSF is bidirectional and populates `input_rc.rssi_dbm` and `link_quality` natively at `RC_INPUT_PROTO = 6`. **Cheaper contingency first:** the installed receiver's model is undocumented (F3) — if it happens to expose an analog RSSI pad, mapping `RC_RSSI_PWM_CHAN` costs $0. Verify physically before purchasing. Interim proxy at zero cost: `rc_total_frame_count` (342 unique, monotonic) gives frame-loss, not RSSI |
| Control link temporal resolution vs 40 ms burst | PX4 ULog `input_rc` | NEEDS CODE | Sample interval is **~500 ms floor / 987 ms median / 1032 ms mean** — 12–25× too slow; Nyquist needs ≤20 ms. Bursts fall entirely between samples. **Contradiction C-4: no PX4 parameter controls this.** The 500 ms floor is compile-time `add_topic("input_rc", 500)` in `logged_topics.cpp`; `SDLOG_PROFILE` is already 2047 and selects topics, not rates. The log also suffered 6623 dropouts / 161.4 s, inflating intervals further | Bypass the ULog: stream `input_rc` over the already-enabled uXRCE-DDS link (`UXRCE_DDS_CFG = 202`) by adding it to `dds_topics.yaml`, and record on the Jetson at the uORB publish rate (~71–91 Hz for SBUS). No PX4 reflash if the topic is already in the DDS profile. **Moot until the row above is fixed** — resolving a burst in a constant-valued signal gains nothing |
| Cross vehicle time alignment | `systemd-timesyncd` → public `pool.ntp.org`; uXRCE-DDS `timesync_status` for PX4↔ROS | NEEDS CODE | **~±10 ms at best, ~±25–50 ms late in a poll interval.** Drones use **different** public servers (delta 198.137.202.56, ghost 170.187.147.56) so errors are uncorrelated; jitter 4.991 / 12.008 ms; ghost root delay 118 ms; poll interval pinned at **34 min 8 s**, over which a 10–20 ppm crystal drifts 20–40 ms. No PTP hardware (`/dev/ptp*` absent), `chronyc` not installed. Uncertainty ≥ the 40 ms slot, so one-way LoRa latency on wall clock would be dominated by clock error. **Single-vehicle PX4↔ROS is already fine:** `estimated_offset` std **46.4 µs**, RTT 5.19 ms | Point both drones at **one local NTP server** (bench gateway 192.168.0.136) with a short poll interval — sub-ms wired, 1–2 ms over WiFi. A `timesyncd.conf` edit, no purchase. **Better still for LoRa latency:** use the firmware's own shared TDMA `cycle_clock`/`uptime_ms`, disciplined inside one 40 ms slot by construction, and sidestep NTP entirely |
| Field walk separation distance | *(none verified)* | **NEEDS HARDWARE** | `vehicle_gps_position` and `sensor_gps` are **absent from all three ULogs**; no satellite count and no `eph` exist anywhere. `SYS_HAS_GPS = 1` / `GPS_1_CONFIG = 201` are configuration intent, not detection — it cannot be confirmed from a read-only audit that a GPS module is even attached. All logs are indoor VIO flights (`vehicle_visual_odometry`, `vehicle_optical_flow` present; `COM_ARM_WO_GPS = 1`). **No installed device substitutes:** VIO/optical-flow drift is unbounded over a 300 m walk; the RealSense D435i has ~10 m range; RSSI cannot be used as a distance reference in an experiment whose purpose is to characterise RSSI against distance | **Measuring wheel** (~$30) or a 100 m fibreglass tape (~$20) with pre-marked 25 m stations — fully covers 300 m, needs no GPS, no power, and no logging integration. **Check first:** if a u-blox module is already fitted, an outdoor power-up may show `eph < 3 m` and cost $0; if it is missing, a u-blox M10 (~$35) on GPS1 also serves navigation |

### Bottom line

**A hardware purchase is unavoidable, but only for two of the eleven quantities**, and the total is small:

- **Genuinely blocked on hardware (2):**
  - **Control link RSSI/LQ** — SBUS structurally cannot carry it. **ELRS/CRSF receiver, ~$20/vehicle.** Check the installed receiver for an analog RSSI pad first; that would cost $0.
  - **2.4 GHz channel occupancy/noise** — `rtw88` implements no survey and nothing else on board can measure the band. **RTL-SDR Blog V4, ~$40**, one unit shared across the fleet.
- **Distance (1)** is nominally NEEDS HARDWARE but the cheapest fix is a **~$20–30 measuring wheel or tape**, and a GPS module may already be fitted — verify physically before buying anything.
- **Blocked on code, not money (5):** TX power sweep, PDR/latency, 915 MHz idle channel power, control-link temporal resolution, cross-vehicle time alignment. Notably, **seq and TX timestamp already travel over the air** and are discarded at the serial boundary — an 8-byte serial-frame change with **zero on-air cost** unblocks the entire PDR-and-latency curve.
- **Already measurable (3):** LoRa per-packet RSSI/SNR, mesh per-peer signal and rate, mesh throughput (iperf3 3.9 on both).

**Total unavoidable spend: roughly $40 fleet-wide (RTL-SDR) plus ~$20/vehicle (ELRS receivers), with ~$25 optional for distance measurement.** Everything else is firmware and configuration work on hardware that is already installed.

**Two cautions before committing money:**
1. The **915 MHz idle-channel-power** verdict (NEEDS CODE) depends on a RadioLib version that is not present on any fleet machine and could not be read. If the pinned version predates `getRSSI(bool)`, the fix is a library upgrade — still not a purchase, but confirm before relying on it.
2. The **802.11s mesh was never up during this audit** (contradiction C-2). Every mesh-mode conclusion is inferred from the same driver on a managed 5 GHz link plus the setup script's `MESH_FREQ=2437`. The mesh rows should be re-verified once the mesh is actually joined at 2437 MHz — especially the coexistence premise, since the bench link's 5765 MHz would show no 2.4 GHz interference at all.
