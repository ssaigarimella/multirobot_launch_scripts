# jfr_codefacts_report.md

Read-only fact extraction for the JFR paper. Nothing was modified, installed, launched, armed,
or flashed. Every value below carries its source path, line number, and a
**[CHECKED-IN]** / **[DEPLOYED]** label.

**Reachability.** delta and ghost were reachable. **buckshee and thunderstrike have been
unreachable across four sessions** (`No route to host` on 192.168.0.60 and .70). Every
"per-vehicle" item is therefore two of four unless a laptop-side artifact covers all four.

**Key structural fact affecting every ROS item.** The workspace uses `colcon --symlink-install`:
`install/**` entries are symlinks to `/workspaces/isaac_ros-dev/src/...`, the **container**
path. So for ROS packages **deployed == checked-in source**, and the install tree cannot be
diffed from the host (the symlink targets do not resolve there). Verified:
`install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml ->
/workspaces/isaac_ros-dev/src/.../nvblox_base.yaml`. The LoRa firmware is the opposite case and
genuinely differs (see E2).

**Primary parameter source** for Section B: the parameter block of
`/home/ssg/lunar_lab/multi-drone-project/code/PX4logs/log_7_2026-8-18-15-55-56.ulg`
(`sys_uuid 000600000000323136383132510c00290023`, `ver_hw CUBEPILOT_CUBEORANGEPLUS`), read with
pyulog 1.1.0. **[DEPLOYED]**. Note this log's `UXRCE_DDS_DOM_ID = 3`, i.e. it came from the
airframe configured as drone 3 (ghost) at the time of that flight.

---

## A. Propulsion, power, and physical configuration

**A1. Frame diagonal / Mark4 V2 variant — NOT FOUND.**
No URDF or SDF exists in the workspace. `CA_ROTOR_COUNT = 4`, `SYS_AUTOSTART = 4001`
(quad X) **[DEPLOYED]**, but `CA_ROTOR0_PX = 1.0`, `CA_ROTOR0_PY = 1.0` are PX4 normalized
defaults, not physical arm lengths. Direct measurement not possible (read-only, remote).
Looked in: `px4_params/*.params`, ULog parameter block, `src/**` for `*.urdf` / `*.sdf`.
**Flagged:** `jfr_codefacts.md:665-676` records an author `\todo` that the frame and prop
specification is internally inconsistent (427 mm diagonal implies a 10 in Mark4 variant, while
the platform table states 7x4.5 in props, and the CAD filenames say 9x4.5 with XING 2207
1800KV). Unresolved in the repository; requires the user to state which airframe flies.

**A2. ESC — NOT FOUND.** No make, model, current rating, or firmware version in any config.
`DSHOT_CONFIG` and `MOT_ORDERING` are **absent from the deployed parameter set**. Looked in:
ULog parameter block, `px4_params/*.params`, repo docs. `jfr_codefacts.md:665` records CAD
filename `CYCLONE 35A.step` as a local file only, explicitly not a committed source.

**A3. Propellers — NOT FOUND** as a verified value; see the A1 conflict.

**A4. Inline fuse on the Jetson branch — NOT FOUND.** No electrical documentation in the repo.

**A5. Buck converter output voltage — NOT FOUND.** Same.

**A6. Cube Orange Plus power source — NOT FOUND.** Same.

**A7. Battery — PARTIAL.** `BAT1_N_CELLS = 4` **[DEPLOYED]** confirms 4S.
`BAT1_CAPACITY = -1.0` (unset), so **5200 mAh is not recorded in the vehicle**.
`BAT1_V_CHARGED = 4.2`, `BAT1_V_EMPTY = 3.6`. Make, model, and C rating NOT FOUND.
**Flagged inconsistency:** a legacy `BAT_N_CELLS = 3` coexists with `BAT1_N_CELLS = 4` in the
same parameter set.

---

## B. PX4 and estimator configuration

Source: ULog parameter block, `log_7_2026-8-18-15-55-56.ulg`. **[DEPLOYED]**

| Item | Parameter | Value |
|---|---|---|
| B1 | `EKF2_EV_DELAY` | **50.0 ms** |
| B2 | `EKF2_EV_POS_X` / `_Y` / `_Z` | **0.15 / 0.0 / 0.0 m** |
| B3 | `EKF2_EV_CTRL` | **11** |
| B4 | `EKF2_HGT_REF` | **2** (range sensor) |
| B4 | `EKF2_RNG_CTRL` | **2** (enabled) |
| B5 | `EKF2_BARO_CTRL` | **0** (barometer NOT fused) |
| B6 | `SDLOG_PROFILE` | **769** |
| B6 | `IMU_GYRO_RATEMAX` | **800 Hz** |
| B6 | `IMU_ACCEL_RATEMAX` | **NOT FOUND** (absent from the parameter set) |
| B6 | `IMU_INTEG_RATE` | **200 Hz** |
| B7 | PX4 version | **v1.15.4**, `ver_sw_release 17761535` (0x010F04FF), git `99c40407ffd7ac184e2d7b4b293f36f10fe561ef` |

**B3 bit decode.** `11 = 0b1011` = bit0 (horizontal position) + bit1 (**vertical position**) +
bit3 (yaw). **Bit2 (velocity) is NOT set.** So vision supplies horizontal position, vertical
position, and yaw; no vision-derived velocity. This matters for D2.

**B6 decode.** `769 = 512 + 256 + 1` = Default + Raw FIFO high-rate IMU + Raw IMU gyro.
**Flagged inconsistency:** `log_71_UnknownDate.ulg` in `px4_logs/` has `SDLOG_PROFILE = 2047`
(all profiles) on the same `sys_uuid`. The two logs disagree; 769 is the value in the newest
log. Reported as found rather than reconciled.

**Flagged inconsistency (B4).** `EKF2_HGT_REF = 2` and `EKF2_RNG_CTRL = 2` select a range
sensor as height reference, but **every PX4 rangefinder driver enable parameter is 0**
(`SENS_EN_SF1XX`, `SENS_EN_LL40LS`, `SENS_EN_MB12XX`, `SENS_EN_TRANGER`, `SENS_EN_VL53L1X`,
`SENS_TFMINI_CFG`, `SENS_SF0X_CFG` all 0). The rangefinder therefore does not come from a PX4
serial or I2C driver. See C3.

**B8. Evidence that `EKF2_EV_DELAY = 50 ms` was measured — NOT FOUND.**
No timestamp-difference script, no logged latency measurement, and no commit message
justifying 50 ms was found. Looked in: `src/px4_offboard/**`, `src/multi_drone_nvblox/**`,
`git log` across the vehicle repos, `px4_params/*.params` history, `ghost_debug/`.
**On the available evidence the value appears to have been chosen by hand, not measured.**
State it that way in the paper or measure it before claiming otherwise.

---

## C. Perception stack configuration

**C1. cuVSLAM. PARTIAL.**
`num_cameras: 1` **[CHECKED-IN == DEPLOYED]**
(`src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/launch/realsense_example.launch.py:36,121`).
The most significant non-default: **cuVSLAM's own TF publishing is disabled**,
`publish_odom_to_base_tf: false`, documented at `realsense_example.launch.py:205`.
Image resolution and rate consumed by cuVSLAM follow the RealSense depth profile in C2.
`enable_localization_n_mapping` was **NOT FOUND** by name in the deployed launch path.
The full non-default parameter list could not be enumerated statically because the launch
composes through `lu.component_container` helpers; **it requires a live `ros2 param dump` on the
running node**, which is out of scope here (launching the stack is prohibited).

**C2. RealSense D435i. [CHECKED-IN == DEPLOYED]**
`config/sensors/realsense_emitter_on.yaml`:
- `rgb_camera.profile: '640x480x15'` (line 24)
- `depth_module.profile: '640x480x30'` (line 29)
- `enable_infra1: false`, `enable_infra2: false` (lines 41-42)
- `enable_accel: false`, `enable_gyro: false` (lines 37, 40)
- `gyro_fps: 200`, `accel_fps: 200` (lines 53-54), `unite_imu_method: 2` (line 55)

**Note the internal tension:** `gyro_fps`/`accel_fps` are set to 200 Hz while `enable_gyro` and
`enable_accel` are **false**, so the camera IMU streams are not published. Combined with the
`vio_bridge` path in D1, this indicates cuVSLAM runs **without** camera-IMU fusion in the
deployed configuration. Whether the autopilot IMU feeds cuVSLAM instead was **NOT FOUND** in
the launch path.

**C3. Rangefinder. PARTIAL.**
From `distance_sensor` in the ULog **[DEPLOYED]**: `type = 0` (laser),
`min_distance = 0.02 m`, `max_distance = 8.0 m`, `orientation = 25` (downward facing),
`signal_quality = -1` (not reported), `device_id = 9699606 = 0x940116`, which decodes as
**bus_type 6 (MAVLINK)**, address 0x01, device type 0x94. Rate **1.0 Hz** in
`log_7`, but **54 Hz** (18.53 ms) in `log_71`; the two disagree.
**Make and model NOT FOUND.** No PX4 rangefinder driver parameter is enabled (see B4), so the
measurement arrives over MAVLINK from the companion, not from a PX4 driver. PX4 parameters
configuring its use: `EKF2_RNG_CTRL = 2`, `EKF2_RNG_A_HMAX = 7.0`, `EKF2_MIN_RNG = 0.1`,
`EKF2_RNG_NOISE = 0.1`, `EKF2_RNG_SFE = 0.05`, `EKF2_RNG_DELAY = 5.0 ms`,
`EKF2_RNG_PITCH = 0.0`, `EKF2_RNG_POS_X/Y/Z = 0.0`.

**C4. nvblox. [CHECKED-IN == DEPLOYED]**
`config/nvblox/nvblox_base.yaml`:
- `voxel_size: 0.05` (line 6)
- `mapping_type: "static_tsdf"` (line 9)
- `projective_integrator_truncation_distance_vox: 4.0` (line 83) -> **0.20 m** at 0.05 m voxels
- `projective_integrator_max_integration_distance_m: 10.0` (line 82)
- `esdf_mode: "3d"` (line 35), `update_esdf_rate_hz: 10.0` (line 18)
- `esdf_slice_height_attachment_frame_id: "camera0_link"` (line 61), `esdf_slice_height_offset: 0.0` (line 62)
- workspace bounds (lines 94-100): `bounding_box`, x **[-5.0, 40.0]**, y **[-25.0, 25.0]**,
  height **[-0.4, 3.2]** m
- `esdf_integrator_max_distance_m: 2.0` (line 104), `esdf_integrator_min_weight: 0.1` (line 102)
- `map_clearing_radius_m: -1.0` (line 52, disabled)

`config/nvblox/specializations/nvblox_realsense.yaml` (applied on top):
- `static_mapper.esdf_slice_height: 0.0` (line 13)
- `esdf_slice_min_height: -0.1`, `esdf_slice_max_height: 0.3` (lines 15-16) -> **0.4 m thick slice**

Block pool, `nvblox_ros/nvblox_core/nvblox/include/nvblox/map/internal/block_memory_pool.h`:
- pre-allocation `DefaultPoolBlockCount<BlockType>::value = 8192` blocks per layer (line ~105)
- growth `static constexpr int kExpansionChunkBlocks = 8192` (line 112), **linear**

The header carries the local rationale verbatim: a global 160000 "demanded ~5GB at startup and
OOM'd the Orin NX (exit 99)", and the previous doubling policy "meant single >1GB allocation
bursts mid-flight, which is what OOM-killed nvblox (exit 99) in flights 70/71."

**C5. Container structure. CONFIRMED SEPARATE. [CHECKED-IN == DEPLOYED]**
`realsense_example.launch.py`: `VSLAM_CONTAINER_NAME = 'vslam_container'` (line 25) and
`NVBLOX_CONTAINER_NAME` imported from `nvblox_ros_python_utils.nvblox_constants` (line 22).
The nvblox container is constructed explicitly rather than through the helper, with a comment
at line 245 saying so, using `package='rclcpp_components'`,
`executable='component_container_mt'` (lines 256-257). cuVSLAM nodes are placed with
`'container_name': VSLAM_CONTAINER_NAME` and `forwarding=False` (lines 113-132) specifically to
stop the parent overriding it. **cuVSLAM and nvblox are in separate containers in the deployed
launch path.**

**C6. Place recognition. [CHECKED-IN == DEPLOYED]**
`src/multi_drone_nvblox/scripts/loop_closure_sp_node.py` declared defaults:
- `max_keypoints: 500` (line 180)
- `bow_threshold: 0.15` (line 181)
- `top_k: 5` (line 182)
- `min_inliers: 15` (line 183)
- `ransac_threshold: 3.0` (line 185)
- `lightglue_threshold: 0.1` (line 186)
- `vocabulary_file: ''` default, with a docstring example `sp_vocab_4096.pkl` (line 21)
  implying **vocabulary size 4096**, but the deployed vocabulary file was not located, so
  **4096 is not confirmed as the file actually loaded**.
- Geometric verification method: RANSAC (`ransac_threshold`); the specific solver was NOT
  identified without reading the full node.
- **Pairwise-consistency threshold: NOT FOUND** by that name.
- **Stage rate (keyframes/s or /m): NOT FOUND** in the launch defaults.

**C7. Submaps. PARTIAL.**
Message types in `src/multi_drone_nvblox/msg/`: `AlignmentStatus`, `AlignmentUpdate`,
`ClaimIntent`, `KeyframeBow`, `KeyframeDescriptor`, `KeyframePacket`, `LinkStats`, `ModeState`,
`PeerState`, `SwarmPeers`. Map exchange rides `{ns}/octomap.*` per the zenoh allow list (D7).
Serialization is ROS 2 CDR over `zenoh-bridge-ros2dds`.
**Compression, typical update size in bytes, and transmission rate: NOT FOUND.** These require
a live `ros2 topic bw` on the running stack, which is out of scope here.

---

## D. Claims checked against the code

**D1. "The smooth odometry stream is fused, not the loop-closure-corrected pose." -> TRUE.**
`src/px4_offboard/src/vio_bridge_node.cpp:32-34`:
```cpp
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/visual_slam/tracking/odometry", sensor_qos,
      std::bind(&VioBridgeNode::odomCallback, this, std::placeholders::_1));
```
Published to PX4 at line 28: `"/fmu/in/vehicle_visual_odometry"`.
`/visual_slam/tracking/odometry` is the smooth VO stream, not the loop-closure-corrected pose.

**D2. "Fused as horizontal position and heading only, with no vision-derived velocity."
-> PARTIAL.**
`EKF2_EV_CTRL = 11 = 0b1011`. Horizontal position (bit 0) TRUE. Yaw (bit 3) TRUE.
No velocity (bit 2 clear) TRUE. **But bit 1, vertical position, is also set.**
The code fuses horizontal position, **vertical position**, and yaw. The word "only" is wrong.
Note this interacts with `EKF2_HGT_REF = 2` (range sensor as height reference).

**D3. "A dedicated node consumes the fused state and republishes the transform that mapping
uses." -> TRUE.**
Node: **`odom_correction`**, package **`active_exploration`**, source
`odom_correction_node.cpp`, instantiated at
`realsense_example.launch.py:210-212`. The launch file states at line 205:
"cuVSLAM's own TF publishing is disabled (publish_odom_to_base_tf: false)", and line 206 notes
the tilt constant `kCameraPitchDeg` lives in `odom_correction_node.cpp`.

**D4. "Block memory pre-allocated per layer at startup and grows in fixed increments when a
pool is exhausted." -> TRUE, both halves implemented.**
`block_memory_pool.h`: "Every BlockLayer owns one BlockMemoryPool which EAGERLY preallocates
this many blocks at construction" (line ~27); growth is
`static constexpr int kExpansionChunkBlocks = 8192` (line 112), described as linear and
explicitly contrasted with the previous doubling policy. Expansion is exercised at
`block_memory_pool_impl.h:26` (`expand(num_preallocated_blocks, ...)`) and logged at line 66.

**D5. "The ESDF slice is referenced to the moving camera frame." -> TRUE.**
`nvblox_base.yaml:61`: `esdf_slice_height_attachment_frame_id: "camera0_link"`, with
`esdf_slice_height_offset: 0.0` (line 62). The surrounding comment (line 59) says
"ESDF slice height tracking (for dynamic altitude tracking on drones)".

**D6. "The vehicles join a single named mesh with static per-vehicle addressing." -> TRUE.**
Mesh ID `jetson-drone-mesh`, `MESH_FREQ="2437"` (channel 6, 2.4 GHz), joined with
`iw dev "$IFACE" mesh join "$MESH_ID" freq "$MESH_FREQ"`. Addressing is static and id-derived:
`DEFAULT_SUBNET = "192.168.77"` with `<subnet>.<id>` (`gen_zenoh_config.py:31`), giving
delta 192.168.77.1 and ghost 192.168.77.3. **Empirically confirmed** on 2026-08-25: ghost came
up `type mesh point`, `channel 6 (2437 MHz)`, `192.168.77.3/24`, peered to delta's MAC.
*Caveat for the paper:* the mesh is **script-invoked and non-persistent**, and all bench and
field runs to date used the bench access point instead.

**D7. "A topic bridge that forwards only the map and place-recognition topics." -> PARTIAL.**
`src/multi_drone_nvblox/scripts/gen_zenoh_config.py:36-53`, the complete forwarded set:
```python
NAMESPACED_PATTERNS = ["{ns}/loop_closure/.*", "{ns}/octomap.*", "{ns}/swarm/.*",
                       "{ns}/alignment/.*", "{ns}/keyframes/.*"]
GLOBAL_PATTERNS = ["/loop_closures", "/loop_closure_alignment"]
LEGACY_EXTRA_GLOBAL_PATTERNS = ["/loop_closure/.*"]      # legacy mode only
```
Map (`octomap`) and place recognition (`loop_closure`, `keyframes`, `loop_closures`,
`loop_closure_alignment`) are forwarded as claimed, **but so are `{ns}/swarm/.*` and
`{ns}/alignment/.*`**, which are coordination and frame-alignment topics, not map or place
recognition. The claim understates what crosses the link.

**D8. "The firmware disables the ESP32-S3 onboard WiFi and Bluetooth explicitly at startup."
-> FALSE.**
A search of the deployed sketch
`/home/ssg/lora_radios/flash_ready/lora_delta_node0/lora_delta_node0.ino` for
`WiFi`, `WIFI`, `btStop`, `esp_bt`, `BLE`, `bluedroid`, `esp_wifi`, `setSleep` returns
**no matches**. There is no WiFi or Bluetooth disable statement anywhere in the firmware.
`setup()` (line 365 onward) calls `heltec_setup()`, `Serial.begin()`, `radio.begin()`,
`radio.setCurrentLimit()`, `radio.setCRC()`, `radio.setDio1Action()` and nothing else relevant.
**There are no lines to quote because the code does not do this.**

**D9. "Per-vehicle differences are expressed as configuration files selected by vehicle
identifier at launch." -> PARTIAL.**
Configuration-by-identifier is real: `src/multi_drone_nvblox/config/` holds
`swarm_alignment.yaml`, `alignment_params.yaml`, `domain_bridge.yaml`,
`namespaced_nvblox.rviz.template` and others, and the launcher passes `drone_id` and
`ROS_DOMAIN_ID` per vehicle. A search for per-vehicle branching in
`multi_drone_nvblox/launch` and `/scripts` returned no vehicle-name conditionals (the single
hit, `alignment_core.py:1281`, is a numerical epsilon test, not a vehicle branch).
**However the claim fails on a different axis: the vehicles are not on matching code.** See F7.
What actually differs between the two reachable vehicles:
1. `isaac_ros_nvblox`: delta on branch `dev` @ `6323ff9`; **ghost on `ekf2-nvblox-pose` @ `7774f42`**.
2. LoRa `NODE_ID`: separate per-vehicle sketches, delta 0, buckshee 1, ghost 2, thunderstrike 3.
3. NetworkManager profile name: delta `TP-Link_073A`, ghost `TP-Link_073A 1`.
4. Network path: ghost has a wired `enP8p1s0` at 192.168.0.206; delta has none.

---

## E. Communication links

**E1. LoRa firmware version.**
- **[DEPLOYED]** `v7.2 - ROS2 Serial Bridge (variable-length TX fix)`, from
  `/home/ssg/lora_radios/flash_ready/lora_<vehicle>_node<N>/*.ino` (all four, 20768 bytes,
  mtime 2026-06-12 17:21).
- **[CHECKED-IN]** `v7.2`, same header string,
  `src/radiohive/firmware/lora_mesh_v7_2_ros2.ino` on the vehicles (radiohive @ `d09503e`).
- A `v7.1` sketch exists at `/home/ssg/lora_radios/lora_mesh_v7_1_ros2.ino` (2026-02-16) but is
  superseded. **Flagged:** `jfr_codefacts.md:489` titles its LoRa section "v7.1", which is out
  of date relative to what is flashed.

**E2. `NUM_NODES`. THIS DIFFERS.**
- **[DEPLOYED] = 4** in all four flash-ready sketches.
- **[CHECKED-IN] = 3** at `src/radiohive/firmware/lora_mesh_v7_2_ros2.ino:42`.

Confirmed empirically: measured LoRa inter-arrival is 190.0 ms, which is `4*40+30`, not the
150 ms that `NUM_NODES = 3` would give.

**E3. Slot, guard, cycle, rate. [DEPLOYED]**
`SLOT_MS = 40`, `GUARD_MS = 30`, `CYCLE_MS = (NUM_NODES * SLOT_MS + GUARD_MS)`,
`TX_MARGIN_MS = 5`, `NODE_TIMEOUT_MS = 3000`, `STATUS_MS = 5000`.
With `NUM_NODES = 4`: **cycle 190 ms, update rate 5.263 Hz per vehicle.**
Measured median inter-arrival 190.00 ms, jitter 1.22 ms over 7068 intervals.

**E4. Radio configuration. [DEPLOYED, identical in all four sketches]**
`LORA_FREQ 915.0` MHz, `LORA_SF 7`, `LORA_BW 500.0` kHz, `LORA_CR 5` (4/5),
`LORA_PRE 8` symbols, `LORA_PWR 10` dBm, `LORA_SYNC 0x12`.
CRC **enabled**: `radio.setCRC(1)` (line 378). `radio.setCurrentLimit(140.0)` (line 377).
Note `LORA_PWR` is a compile-time constant consumed once by `radio.begin()` (line 371);
`setOutputPower()` is never called, so TX power cannot be changed at runtime.

**E5. Packet format. [DEPLOYED]**
```c
typedef struct __attribute__((packed)) {
  uint8_t  node_id;                    // 1
  uint32_t sequence;                   // 4
  uint32_t uptime_ms;                  // 4
  uint32_t cycle_clock;                // 4
  uint8_t  sync_source;                // 1
  int8_t   rssi_of[NUM_NODES];         // 4  (NUM_NODES = 4 deployed)
  uint8_t  neighbor_mask;              // 1
  float    lat; float lon; float alt; float hdg;   // 16
  uint8_t  has_custom;                 // 1
  uint8_t  custom_dest;                // 1
  uint8_t  custom_len;                 // 1
  uint8_t  custom_data[MAX_CUSTOM_DATA];  // 160
} Packet;
```
`sizeof(Packet) = 198` bytes deployed; `BASE_PACKET_SIZE = sizeof(Packet) - MAX_CUSTOM_DATA =
38` bytes, which is what is transmitted for a pose-only packet.

Required fields: **sequence number EXISTS** (`sequence`, uint32).
**TX timestamp EXISTS on the air** (`uptime_ms`, sender `millis()` at transmit).
**TX power field DOES NOT EXIST.**
Critically, **neither `sequence` nor `uptime_ms` is forwarded to ROS**: the firmware-to-ROS
`0x81 PEER_POSE` frame is 23 bytes carrying only node/lat/lon/alt/hdg/rssi/snr, so per-packet
delivery ratio and one-way latency are not derivable from logged data today.

**E6. Clock synchronization. The premise of the question does not match the implementation.**
**There are no dedicated synchronization broadcasts.** Sync data is embedded in every TDMA
packet via the `sync_source` and `cycle_clock` fields. `STATUS_MS = 5000` is a human-readable
status print interval (and its body is commented out), not a sync period.
- Airtime compensation: `packet_airtime_ms = radio.getTimeOnAir(BASE_PACKET_SIZE) / 1000`
  (line 382), applied on receive as `tx_moment = now - packet_airtime_ms`.
- Lowest-identifier adoption rule: `shouldSyncTo()` (line 355) and `findNewSyncSource()`
  (line 342), in the same file
  (`/home/ssg/lora_radios/flash_ready/lora_delta_node0/lora_delta_node0.ino`, and identically
  in the checked-in `src/radiohive/firmware/lora_mesh_v7_2_ros2.ino`).

**E7. WiFi mesh.**
Adapter **Realtek RTL8822CE** (`lspci`: "RTL8822CE 802.11ac PCIe Wireless Network Adapter"),
driver **rtw88** (`rtw88_8822ce`, `rtw88_8822c`, `rtw88_pci`, `rtw88_core`),
kernel **5.15.148-tegra**, **L4T R36.4.0** (JetPack 6.x).
Mesh ID **`jetson-drone-mesh`**, channel **6**, **2437 MHz**, 20 MHz, band 2.4 GHz.
Addressing **static**, `192.168.77.<drone_id>`.
The vendor `rtl8822ce` driver is blacklisted (`/etc/modprobe.d/blacklist-rtl88x2ce.conf`).

**E8. Kernel and driver patch. PARTIAL.**
**No `.patch` or `.diff` file exists** in `/mnt/nova_ssd/workspaces/kernel_build/`, and the
kernel tree there is not a git repository, so **the total diff size in lines is NOT FOUND** and
cannot be computed without an upstream reference tree.
What was replaced is evidenced by `kernel_build/backup_modules/`: `mac80211.ko.orig`,
`rtw88_core.ko.orig`, `rtw88_pci.ko.orig`, `rtw88_8822c.ko.orig`, `rtw88_8822ce.ko.orig`,
`rtl8822ce.ko.orig`. Rollback script `kernel_build/04_rollback.sh`.
Files carrying the changes, with one sentence each:
1. `net/mac80211` rebuilt with `CONFIG_MAC80211_MESH=y` (stock JetPack has it unset), which adds
   the 802.11s mesh implementation. Confirmed at runtime: the loaded `mac80211.ko` exports 124
   `ieee80211_mesh_*` symbols.
2. `drivers/net/wireless/realtek/rtw88/mac80211.c:175` adds
   `case NL80211_IFTYPE_MESH_POINT:` to the interface-add path, letting the driver accept a mesh
   interface type.
3. `drivers/net/wireless/realtek/rtw88/mac80211.c:280-291` changes `FIF_OTHER_BSS` handling to
   clear `BIT_CBSSID_BCN | BIT_CBSSID_DATA` so beacons from other BSSIDs are received, which
   mesh peering requires.
4. `drivers/net/wireless/realtek/rtw88/fw.c:1186` adds `vif->type != NL80211_IFTYPE_MESH_POINT`
   to a guard so mesh interfaces are not excluded from that firmware path.

**Trap for the paper:** `/proc/config.gz` on the vehicles reports
`# CONFIG_MAC80211_MESH is not set` and `# CONFIG_RTW88 is not set`. That is the stock boot
image configuration, not the replaced modules. Do not cite it.

---

## F. Platform and tooling

**F1.** L4T **R36.4.0** (`/etc/nv_tegra_release`, GCID 37537400, 2024-09-13) = **JetPack 6.x**.
Kernel **5.15.148-tegra**. ROS 2 **Humble** (`/opt/ros/humble`).
`nvidia-jetpack` is **not installed as an apt package** on the host and `nvcc` is not on the
host PATH, so the **exact JetPack point release and CUDA version are NOT FOUND from the host**;
CUDA lives inside the `isaac_ros_realsense` container.

**F2.** Available modes (`/etc/nvpmodel.conf`): **0 MAXN, 1 10W, 2 15W, 3 25W**.
Current mode: **MAXN (0)** (`nvpmodel -q`). **[DEPLOYED, delta, at rest]**
No evidence was found of a flight-specific power-mode switch in any launch script, so MAXN
appears to be the mode in flight, but that is an absence of evidence, not a positive finding.

**F3. tegrastats. THE CARRIER BOARD DOES EXPOSE RAIL POWER MONITORS. Answer: YES.**
Two verbatim lines, delta, 1 s interval:
```
08-28-2026 14:55:21 RAM 9613/15656MB (lfb 12x4MB) SWAP 0/7828MB (cached 0MB) CPU [83%@1574,29%@1574,28%@1574,33%@1574,41%@1984,39%@1984,38%@1984,36%@1984] GR3D_FREQ 14% cv0@55.281C cpu@57.843C soc2@55.25C soc0@56.156C cv1@55.968C gpu@53.656C tj@59.875C soc1@59.875C cv2@54.062C VDD_IN 9747mW/9747mW VDD_CPU_GPU_CV 2544mW/2544mW VDD_SOC 2988mW/2988mW
08-28-2026 14:55:22 RAM 9614/15656MB (lfb 12x4MB) SWAP 0/7828MB (cached 0MB) CPU [83%@1984,32%@1984,30%@1984,31%@1984,39%@729,42%@729,41%@729,45%@729] GR3D_FREQ 17% cv0@55.312C cpu@57.531C soc2@55.218C soc0@56.25C cv1@55.75C gpu@53.781C tj@59.875C soc1@59.875C cv2@53.968C VDD_IN 9505mW/9626mW VDD_CPU_GPU_CV 2342mW/2443mW VDD_SOC 2952mW/2970mW
```
The output **contains `VDD_IN`, `VDD_CPU_GPU_CV`, and `VDD_SOC` power fields** in mW, in
instantaneous/average pairs, alongside temperatures and clocks.
`/sys/bus/i2c/drivers/ina3221/` **exists** and contains: `1-0040`, `bind`, `module`, `uevent`,
`unbind`. So the INA3221 driver is bound to one device at I2C 1-0x40.
**Total board power at idle-with-stack was about 9.5 to 9.7 W.**

**F4. Thermal zones** under `/sys/devices/virtual/thermal/` **[DEPLOYED, delta]**, with labels
and the temperatures read at the time:
`thermal_zone0 cpu-thermal 57.9C`, `1 gpu-thermal 53.8C`, `2 cv0-thermal 55.5C`,
`3 cv1-thermal 55.9C`, `4 cv2-thermal 54.0C`, `5 soc0-thermal 56.3C`,
`6 soc1-thermal 59.8C`, `7 soc2-thermal 55.3C`, `8 tj-thermal 60.1C`.

**F5. Exploration behavior parameters. PARTIAL.**
From `src/active_exploration/launch/fis.launch.py` **[CHECKED-IN == DEPLOYED]**:
- `min_cluster_size: 10` (line 11)
- `height_band: 1.0` (line 23)
- `cluster_max_size_xy: 2.0` (line 41)

**NOT FOUND:** information-gain weight, travel-cost weight, the claim-discount factor applied
to frontiers claimed by other vehicles, and the condition that switches to the boustrophedon
pattern. A search of `multi_drone_nvblox/launch/planner_stage.launch.py` for
`gain|weight|discount|claim|bous|lane|util` in its `default_value` entries returned nothing, so
these are not exposed as launch arguments. They are likely compiled defaults inside
`exploration_manager_node.cpp` or `simple_exploration_planner.py` and would need those sources
read directly.

**F6. Repository layout. [CHECKED-IN]**
Laptop repo `/home/ssg/multi-robot-coordination` (branch `main`, HEAD `acc87da`):
`archive/` superseded launchers, `collected/` data pulled from vehicles,
`ghost_debug/` the VIO domain-ID postmortem, `logs/` and `px4_logs/` ULogs,
`px4_params/` parameter snapshots, `reports/` generated analyses,
`Python-Vicon-UDP-Parsing/` Vicon utility; operator tooling at top level
(`launch_fleet_tmux.sh`, `fleet_ctl`, `bench_internet_laptop.sh`, `rf_stats.sh`,
`build_all_drones.sh`).
Vehicle workspace `/mnt/nova_ssd/workspaces/isaac_ros-dev` with `src/`, `install/`
(symlink tree), `experiment_logs/`; plus `/mnt/nova_ssd/workspaces/kernel_build/`.
Config directory `src/multi_drone_nvblox/config/`: `ade20k/`, `alignment_params.yaml`,
`bag_qos_overrides.yaml`, `domain_bridge.yaml`, `fastdds_loopback.xml`, `fastdds_unicast.xml`,
`global_view.rviz`, `namespaced_nvblox.rviz.template`, `nvblox_debug.yaml`,
`swarm_alignment.yaml`, `trajectory_view.rviz`, `xrce_agent_loopback.refs`.
Container image: **`isaac_ros:dev-realsense`**, running container name
**`isaac_ros_realsense`** (`launch_delta_frontierexplore_tmux.sh:35-36`). **Tag beyond
`dev-realsense` NOT FOUND.**

**F7. Commit hash and branch per vehicle. THEY DO NOT ALL MATCH.**

| Repository | delta | ghost | buckshee | thunderstrike |
|---|---|---|---|---|
| `multi_drone_nvblox` | `feature/jfr-multi-drone-field` @ `840da03` | same | UNREACHABLE | UNREACHABLE |
| `active_exploration` | `feature/jfr-multi-drone-field` @ `e1f51d2` | same | UNREACHABLE | UNREACHABLE |
| `radiohive` | `dev` @ `d09503e` | same | UNREACHABLE | UNREACHABLE |
| `isaac_ros_nvblox` | `dev` @ **`6323ff9`** | **`ekf2-nvblox-pose`** @ **`7774f42`** | UNREACHABLE | UNREACHABLE |
| `isaac_ros_visual_slam` | `dev` @ `4c7b976` | same | UNREACHABLE | UNREACHABLE |

**delta and ghost are running different nvblox code, on different branches.** buckshee and
thunderstrike could not be checked.

---

## Contradictions and open items

### Claims that came back FALSE or PARTIAL

1. **D8 FALSE.** The firmware does **not** disable the ESP32-S3 WiFi or Bluetooth. There is no
   such code in the deployed sketch. The paper sentence must be deleted or the firmware changed.
2. **D2 PARTIAL.** `EKF2_EV_CTRL = 11` also fuses **vertical position** (bit 1). "Horizontal
   position and heading only" is wrong. The no-velocity part is correct.
3. **D7 PARTIAL.** The bridge also forwards `{ns}/swarm/.*` and `{ns}/alignment/.*`, which are
   coordination and alignment topics, not map or place recognition.
4. **D9 PARTIAL.** Config-by-identifier is real and no vehicle-name branches remain, but the two
   reachable vehicles are on **different branches of `isaac_ros_nvblox`**, so "per-vehicle
   differences are expressed as configuration" does not hold in practice today.

### Deployed / checked-in divergences

5. **E2.** LoRa `NUM_NODES` is **4 deployed**, **3 checked in**. Cycle time and update rate in
   the paper must come from the deployed value: 190 ms, 5.263 Hz.
6. **F7.** delta and ghost differ on `isaac_ros_nvblox` branch and commit.

### Internal inconsistencies flagged, not resolved

7. **B6.** `SDLOG_PROFILE` is 769 in `log_7` and 2047 in `log_71`, same `sys_uuid`.
8. **A7.** `BAT_N_CELLS = 3` coexists with `BAT1_N_CELLS = 4`.
9. **B4 / C3.** `EKF2_HGT_REF = 2` and `EKF2_RNG_CTRL = 2` select a range sensor as height
   reference while every PX4 rangefinder driver parameter is 0; the measurement arrives over
   MAVLINK (`device_id` bus_type 6) from the companion.
10. **C3.** `distance_sensor` rate is 1.0 Hz in `log_7` and 54 Hz in `log_71`.
11. **C2.** `gyro_fps: 200` and `accel_fps: 200` are set while `enable_gyro` and `enable_accel`
    are `false`, so the camera IMU is not published.
12. **A1.** Frame and propeller specification is internally inconsistent across the paper draft
    table, the CAD filenames, and the stated diagonal. Recorded as an author `\todo`.
13. **E1.** `jfr_codefacts.md` labels the LoRa section v7.1; the deployed firmware is v7.2.

### NOT FOUND

14. **A1** frame diagonal and Mark4 V2 variant. **A2** ESC make, model, rating, firmware.
    **A3** propellers as flown. **A4** Jetson-branch fuse rating. **A5** buck output voltage.
    **A6** Cube Orange Plus power source. **A7** battery make, model, C rating, and capacity.
15. **B6** `IMU_ACCEL_RATEMAX` absent from the parameter set.
16. **B8** No evidence that `EKF2_EV_DELAY = 50 ms` was measured. On the available evidence it
    was chosen by hand. Do not claim it was measured.
17. **C1** cuVSLAM full non-default parameter list and `enable_localization_n_mapping`; needs a
    live `ros2 param dump`.
18. **C3** rangefinder make and model.
19. **C6** confirmed vocabulary size of the loaded file, geometric-verification solver,
    pairwise-consistency threshold, stage rate.
20. **C7** compression, typical update size in bytes, transmission rate; needs live
    `ros2 topic bw`.
21. **E8** total patch diff size in lines; no patch file exists and the kernel tree is not a git
    repository.
22. **F1** exact JetPack point release and CUDA version from the host; they are in the container.
23. **F5** information-gain weight, travel-cost weight, claim-discount factor, boustrophedon
    switch condition; not exposed as launch arguments.
24. **F6** container image tag beyond `dev-realsense`.
25. **F7** buckshee and thunderstrike: unreachable for four sessions.
