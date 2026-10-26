# jfr_codefacts.md — verified implementation facts for the journal systems paper

Scope: read-only investigation of local machines + one authorized read-only SSH to lead
drone ghost (Q3/Q4 gap-filling only, per task rules). RACER/SEER/PROWL excluded.
No experimental results reported. Date of investigation: 2026-07-22.
Updated 2026-08-21: paper-draft citations repointed from the deleted `main3.tex` (now in
`~/.local/share/Trash/files/`) to the live drafts `~/Downloads/main_JFR_v2.tex` (newest,
identical md5 to `main_JFR_v2 (1).tex`) / `main_JFR_v5.tex` (platform table identical in both,
offset by 2 lines); new draft-documented hardware facts added to Q5; /proc/config.gz caveat
added to Q3.

Evidence provenance tags used throughout:
- **[CODE]** — source file read directly on this laptop (file:line).
- **[DOC]** — statement in a committed documentation file (file:line). Doc fact, not code-verified.
- **[CAPTURE]** — committed runtime log capture in `ghost_debug/` (startup config echoes only; no results).
- **[ON-DRONE]** — read over authorized SSH on ghost (`ghost@192.168.0.206`; the given
  `prometheus@100.98.184.28` timed out). Files on ghost are not in any git repo; cited by absolute path.
- **[TRANSCRIPT]** — drone-side git/file output cached verbatim in a local Claude session transcript
  (`~/.claude/projects/-home-ssg-multi-robot-coordination/32c52d39-…jsonl`). Secondary evidence;
  commit hashes refer to `github.com/ssaigarimella` forks (branch `dev`) that exist only on the drones.

---

## 1. Repo inventory

| Repo | Path | Remote URL | Branch | HEAD |
|---|---|---|---|---|
| multi-robot-coordination (launch scripts, PX4 params, docs) | `/home/ssg/multi-robot-coordination` | `git@github.com:ssaigarimella/multirobot_launch_scripts.git` | `main` | `6399a56` |
| LoRa TDMA firmware | `/home/ssg/lora_radios` | **NOT a git repo** (`git status` → "fatal: not a git repository") | — | — |
| RadioHive (older Meshtastic python tooling) | `/home/ssg/RadioHive` | `git@github.com:ssaigarimella/RadioHive.git` | `main` | `a3cb9fe` |
| Python-Vicon-UDP-Parsing | `/home/ssg/multi-robot-coordination/Python-Vicon-UDP-Parsing` | `https://github.gatech.edu/GT-AE-Indoor-Flight-Laboratory/Python-Vicon-UDP-Parsing.git` | `master` | `6cbaef0` |
| Micro-XRCE-DDS-Agent | `/home/ssg/Micro-XRCE-DDS-Agent` | `https://github.com/eProsima/Micro-XRCE-DDS-Agent.git` | `master` | `155cfaa` = tag `v3.0.1` |
| PX4-Autopilot | `/home/ssg/Pixhawk-development/PX4-Autopilot` | `https://github.com/PX4/PX4-Autopilot.git` | detached | `99c40407ff` = tag `v1.15.4` |
| PX4-Autopilot (2nd checkout) | `/home/ssg/Pixhawk-development/PX4-Autopilot-v1.15.4/PX4-Autopilot` | same | detached | `99c40407ff` = tag `v1.15.4` |
| PX4-Autopilot (older) | `/home/ssg/Pixhawk-development/PX4-Autopilot-v1.14.4` | same | detached | `1555f2bd22` = tag `v1.14.4` |

**Platform source that is NOT on this laptop** (per committed doc `AGENT_HANDOFF.md:240-273` and
confirmed by exhaustive local search): the drone workspace `/mnt/nova_ssd/workspaces/isaac_ros-dev/`
holds `active_exploration`, `multi_drone_nvblox`, `px4_offboard`, `radiohive`, and vendor forks
`isaac_ros_nvblox`, `isaac_ros_visual_slam`, `isaac_ros_common`, `realsense-ros`, `px4_msgs`
(`release/1.15`), `third_party/LightGlue` — main repos on branch `dev`, remotes under
`github.com/ssaigarimella`. The workspace root itself is not a git repo ([ON-DRONE]
`git -C /mnt/nova_ssd/workspaces/isaac_ros-dev` → not a git repository). SSH was authorized only
for Q3/Q4, so the internals of those repos are reported here only where local evidence exists.

Drone ID mapping (used throughout): 1=delta, 2=buckshee, 3=ghost, 4=thunderstrike
([DOC] `AGENT_HANDOFF.md:8-9`; [CODE] `launch_ghost_frontierexplore_tmux.sh:164`). LoRa firmware
node IDs are 0-indexed: delta=0, buckshee=1, ghost=2, thunderstrike=3 ([CODE] `flash_ready/` dir names + `#define NODE_ID`).

---

## 2. Q1 — cuVSLAM and nvblox modifications

### Upstream bases

Fact: `isaac_ros_nvblox` fork base is "v3.2-14", with 580 changed files; nvblox core vendored in-tree instead of submodule.
Evidence: [DOC] `AGENT_HANDOFF.md:256-258`
Snippet:
```
- `isaac_ros_nvblox` — **heavily modified** (base v3.2-14 + 580 changed files; nvblox core
  vendored in-tree instead of submodule; cuVSLAM/nvblox container split; moving ESDF slice;
  GPU preallocation). Uncommitted: `nvbloxIFLmap1.rviz` tweaks.
```

Fact: `realsense-ros` fork base is 4.51.1; `isaac_ros_visual_slam` fork has rviz configs only; `px4_msgs` pristine `release/1.15` from official PX4 remote.
Evidence: [DOC] `AGENT_HANDOFF.md:259-262`
Snippet:
```
- `realsense-ros` — base 4.51.1 + one functional line: IMU publisher queue 5→100.
- `isaac_ros_visual_slam` — rviz configs only.
- `isaac_ros_common` — run_dev.sh tweak (drops PVA GPU device env on aarch64).
- `px4_msgs` — pristine `release/1.15` (official remote).
```
Upstream remote/tag/commit of the NVIDIA base beyond the string "v3.2-14": NOT FOUND locally
(fork git metadata lives only on the drones).

### Substantive modifications

**M1 — cuVSLAM/nvblox container split (category: other — process isolation).**
WHAT: cuVSLAM+RealSense+splitter load into `vslam_container`; nvblox into a separate
`nvblox_container`, and `nvblox_container` was changed from `lu.component_container()` (which
hardcodes `on_exit=Shutdown()`) to a plain `Node` so a mapper crash cannot kill VIO.
WHY (commit message): nvblox CUDA OOM (exit 99) tore down `vslam_container` too → camera+cuVSLAM
destroyed → VIO to PX4 stopped → EKF2 dead-reckon → ALTCTL.
Evidence: [DOC] `AGENT_HANDOFF.md:235-236`; [CAPTURE] `ghost_debug/run01/data/A_wifi_on/c_node_list.txt`
(nodes `/nvblox_container`, `/vslam_container`) and `tab1_pane.txt:16,20`; [TRANSCRIPT] fork commit
`52af58c` (isaac_ros_nvblox, branch `dev`, 1 file, 15+/3−, `realsense_example.launch.py`).
Snippet ([TRANSCRIPT] commit message):
```
nvblox_container no longer 'required': mapper crash can't kill VIO
lu.component_container() hardcodes on_exit=Shutdown(); when nvblox died
(CUDA OOM exit 99) launch tore down vslam_container too -> camera+cuVSLAM
destroyed -> VIO to PX4 stopped -> EKF2 dead-reckon -> ALTCTL (flights 70/71).
nvblox_container is now a plain Node (same executable/args, no on_exit);
```
Snippet ([DOC] `AGENT_HANDOFF.md:235-236`):
```
Note: cuVSLAM/RealSense/splitter load into `vslam_container`, nvblox into
`nvblox_container` (a local patch, "isolated from nvblox CUDA stalls").
```

**M2 — nvblox block-pool pre-allocation, two iterations (category: memory).**
WHAT (iteration 1, commit `3e2c17c` "expanding block memory pool size"):
`block_memory_pool.h:42` `num_preallocated_blocks` default 65536 → 160000; `nvblox_base.yaml`
workspace bounds `"unbounded"` → `"bounding_box"` x/y ±25 m, height −0.4…3.2 m (~156k blocks ≤
160k prealloc, ~1.9 GB fixed at startup, sized for the 50×50 m field).
WHAT (iteration 2, commit `2c4996f`, supersedes it, "per-layer memory pool sizing + linear
expansion"): per-block-type pool defaults tsdf 160k / color 60k / esdf 40k / rest 8192; pool
exhaustion grows by fixed 8192-block chunks instead of doubling. Files:
`nvblox_ros/nvblox_core/nvblox/include/nvblox/map/internal/block_memory_pool.h` +
`impl/block_memory_pool_impl.h` (2 files, 47+/6−); pulled to all 4 drones.
WHY (commit message of `2c4996f`):
Evidence: [TRANSCRIPT] lines 673, 949-952; corroborated by [CAPTURE] `tab1_pane.txt:54-55`
(pre-fix state 2026-06-09) and [DOC] `AGENT_HANDOFF.md:256-258` ("GPU preallocation").
Snippet ([TRANSCRIPT] `2c4996f` message):
```
Global 160000 prealloc applied to ALL 12 pools (2 mappers x 6 layers) and
demanded ~10GB at startup -> instant CUDA OOM. Per-block-type defaults now:
tsdf 160k (box capacity, never expands), color 60k, esdf 40k, rest 8192.
Pool exhaustion now grows by fixed 8192-block chunks instead of doubling,
```
Snippet ([CAPTURE] `tab1_pane.txt:54-55`, pre-fix run):
```
65] Expanding the memory pool with 65536 blocks. Number of allocated blocks: 655
```

**M3 — bounded workspace tied to geofence (category: memory).**
WHAT: nvblox workspace bounds ±25 m (in `nvblox_base.yaml`, on-drone); exploration geofence set
1 m inside at ±24 m.
Evidence: [CODE] `launch_ghost_frontierexplore_tmux.sh:131-135` (identical comment in the other
three drones' scripts at line ~130); [TRANSCRIPT] line 673 (commit `3e2c17c`, see M2).
Snippet:
```
        # Geofence ±24m = 1m inside the nvblox workspace bounds (±25m, see
        # nvblox_base.yaml) so ESDF stays valid at the fence edge. Sized for the
        # 50x50m FIELD — shrink these for indoor flights (old indoor value: ±10).
        ssh_docker_tab "Tab 5: Simple Exploration Planner" \
            "ros2 launch active_exploration simple_planner_launch.py flight_height:=1.0 debug_skip_arm_check:=${DEBUG_VAL} bbox_min_x:=-24.0 bbox_min_y:=-24.0 bbox_max_x:=24.0 bbox_max_y:=24.0"
```

**M4 — moving ESDF slice (category: frame handling).**
WHAT: ESDF slice bounds/height attachment frames set to the camera frame (`camera0_link`), so the
slice follows the vehicle; slice band −0.1…0.3 m, slice height 0.
Evidence: [DOC] `AGENT_HANDOFF.md:257` ("moving ESDF slice"); [CAPTURE] `tab1_pane.txt:282-286, 510-513`.
Snippet ([CAPTURE], line-wrapped verbatim):
```
apper.esdf_slice_min_height: -0.1
apper.esdf_slice_max_height: 0.3
apper.esdf_slice_height: 0
    esdf_slice_bounds_visualization_attachment_frame_id: camera0_frame? -> camera0_link
    esdf_slice_height_attachment_frame_id: camera0_li
nk
```

**M5 — cuVSLAM operating mode (category: interface/frame handling; configuration of the fork, from docs+capture).**
WHAT: stereo infra + IMU fusion; odometry-only (localization/mapping OFF);
`publish_odom_to_base_tf` FALSE — the odom→camera TF is instead published by the project's own
`odom_correction` node with a +20° pitch rotation (see Q5 tilt).
Evidence: [DOC] `AGENT_HANDOFF.md:208-213`; [CAPTURE] `tab1_pane.txt:25` and `run01/.../c_node_list.txt` (`/odom_correction`).
Snippet ([DOC]):
```
  → cuVSLAM (vslam_container; stereo infra + IMU fusion; odometry-only,
             localization/mapping OFF; publish_odom_to_base_tf FALSE)
      → /visual_slam/tracking/odometry
          ├─→ odom_correction (active_exploration): rotates +20° pitch, publishes
          │     odom→camera0_link TF that nvblox consumes
```
Snippet ([CAPTURE] `tab1_pane.txt:100-101, 1085-1098`, cuVSLAM startup echo):
```
[visual_slam_node]: cuVSLAM version: 12.6
Use use_gpu: true / Enable IMU Fusion: true
gyroscope_noise_density: 0.000244 / gyroscope_random_walk: 0.000019
accelerometer_noise_density: 0.001862 / accelerometer_random_walk: 0.003000
frequency: 200.000000
```

**M6 — alternate pose source for nvblox: `pose_source` launch arg + `ekf2_odom_tf` node (category: interface/timing).**
WHAT: `realsense_example.launch.py` gains a `pose_source` argument (`cuvslam` default / `ekf2`)
([TRANSCRIPT] isaac_ros_nvblox commit `7774f42`, branch `ekf2-nvblox-pose`); companion
`active_exploration` commit `0ad55ec` adds node `ekf2_odom_tf` subscribing
`/fmu/out/vehicle_odometry`, publishing odom→camera0_link with NED→FLU + body→camera R_y(+20°),
using PX4 `timestamp_sample` for stamping.
Evidence: [TRANSCRIPT] lines 2241, 2301, 2308; [CODE, uncommitted working-tree edit]
`launch_ghost_frontierexplore_tmux.sh:112-113` passes the new arg (the committed version and the
other three drones' scripts do not — confirmed via `git diff`):
Snippet:
```
        ssh_docker_tab "Tab 1: cuVSLAM + RealSense + nvblox" \
            "ros2 launch nvblox_examples_bringup realsense_example.launch.py run_rviz:=${RVIZ_FLAG} pose_source:=${POSE_SOURCE}" "$GUI_ARG"
```

**M7 — realsense-ros IMU publisher queue 5→100 (category: timing).**
WHAT/WHY: single functional line on base 4.51.1 (queue depth for the IMU publisher).
Evidence: [DOC] `AGENT_HANDOFF.md:259` (snippet under "Upstream bases" above). Actual diff: NOT FOUND locally.

**M8 — Fast DDS profile / env handling (category: interface).**
WHAT: a FastDDS profiles file `multi_drone_nvblox/config/fastdds_loopback.xml` exists (loopback
pinning); only one of three launchers exports it, and under the legacy env-var name; a known,
currently-unfixed bug is that the container's MicroXRCEAgent links Fast DDS 3.5 which only reads
`FASTDDS_DEFAULT_PROFILES_FILE`, while the stack exports `FASTRTPS_DEFAULT_PROFILES_FILE`.
Additionally a host systemd unit `dds-multicast-lo.service` routes multicast to loopback
(`ip route replace 224.0.0.0/4 dev lo`), deliberately killing cross-host DDS discovery (Zenoh is
the only intended cross-drone path).
Evidence: [DOC] `AGENT_HANDOFF.md:150-151, 293-311`.
Snippet:
```
2. **Fast DDS env-var rename bug (CURRENTLY UNFIXED)**: the container's `MicroXRCEAgent`
   (`/usr/local/bin/MicroXRCEAgent`) links Fast DDS 3.5, which only reads
   `FASTDDS_DEFAULT_PROFILES_FILE`. The launch stack exports only the legacy
   `FASTRTPS_DEFAULT_PROFILES_FILE`, which the agent ignores
```
Snippet:
```
   - `launch_ghost_local.sh`: no profiles file at all (loopback pinning commented out).
   - `run_ghost.sh`: exports `FASTRTPS_DEFAULT_PROFILES_FILE=.../fastdds_loopback.xml`
     (the legacy name — ignored by Fast DDS 3.x components, honored by Humble's FastDDS 2.x).
   - `flight_autonomous.sh`: neither
```
Contents of `fastdds_loopback.xml`: NOT FOUND locally (path only).

**M9 — LightGlue local patches (category: interface).**
WHAT: both LightGlue checkouts (upstream `cvg/LightGlue`, branch `main`) carry uncommitted patches
to `lightglue/__init__.py` disabling the optional extractors (ALIKED/DISK/DoGHardNet/SIFT); the
workspace-root copy (comment-out variant) is the pip-editable install the container actually imports.
Evidence: [DOC] `AGENT_HANDOFF.md:264-269`.
Snippet:
```
- `third_party/LightGlue` + **duplicate checkout at workspace root `./LightGlue`** — the
  root copy is the pip-editable-installed one the container actually imports. **Both have
  uncommitted patches to `lightglue/__init__.py` that disable the optional extractors
  (ALIKED/DISK/DoGHardNet/SIFT), but the patches DIFFER**
```

**Checked but NOT FOUND (Q1 specifics the task asked to confirm):**
- CUDA stream or stream-priority changes: NOT FOUND (searched all local repos, AGENT_HANDOFF.md,
  ghost_debug captures, transcripts).
- ROS QoS changes inside the forks: NOT FOUND locally. (The only QoS facts found are the zenoh
  bridge annotations `keyframe_bow ~4KB RELIABLE` / `keyframe_full ~540KB BEST_EFFORT` —
  [ON-DRONE] `launch_mesh_zenoh.sh` echo lines; see Q2/Q3.)
- Timestamp/frame-id handling changes inside cuVSLAM/nvblox themselves: NOT FOUND locally;
  the timestamp/frame handling evidenced is in project nodes (`odom_correction` +20° TF,
  `ekf2_odom_tf` using `timestamp_sample` — M5/M6).
- The actual 580-file diff: NOT FOUND locally (only the [DOC] claim and the four recovered
  fork commits `3e2c17c`, `2c4996f`, `52af58c`, `7774f42` + `active_exploration@0ad55ec`).

**Build/packaging glue (one line):** `isaac_ros_common` run_dev.sh tweak dropping PVA GPU device
env on aarch64 ([DOC] `AGENT_HANDOFF.md:261`); `isaac_ros_visual_slam` rviz-config-only changes
([DOC] :260); abandoned `run_dev.sh` flow with `CONFIG_IMAGE_KEY=ros2_humble.realsense`
([DOC] :105-107); container image itself is a hand-mutated `docker commit` snapshot with no
reproducing Dockerfile ([DOC] :93-99); BoW vocab pickle must stay protocol-4 ([DOC] :448).

---

## 3. Q2 — Inter-robot place recognition internals

**Feature/matcher stack as evidenced locally.**
Fact: loop closure runs SuperPoint + LightGlue + BoW with vocabulary file `sp_vocab_4096.pkl`
(identical on all drones), implemented in `multi_drone_nvblox/scripts/loop_closure_sp_node.py`,
launched as a plain `python3` process (not a composed/compiled node), at `process_rate:=1.0` (Hz),
with `use_pcm:=True` and `publish_tf:=True`. LightGlue is the pip-editable PyTorch checkout
(torch 2.5.0 jp6 wheel, CUDA works; torchvision not installed).
Evidence: [CODE] `launch_ghost_frontierexplore_tmux.sh:137-144` (identical parameters in the
delta/buckshee/thunderstrike scripts and in every archived variant back to Mar 2026 — values never
changed); [DOC] `AGENT_HANDOFF.md:224-227, 429-430`.
Snippet ([CODE]):
```
        ssh_docker_tab "Tab 6: Loop Closure (SuperPoint + LightGlue)" \
            "python3 src/multi_drone_nvblox/scripts/loop_closure_sp_node.py --ros-args \
                -p robot_namespace:=drone3 \
                -p vocabulary_file:=/workspaces/isaac_ros-dev/src/multi_drone_nvblox/models/sp_vocab_4096.pkl \
                -p process_rate:=1.0 \
                -p use_pcm:=True \
                -p publish_tf:=True"
```
Snippet ([DOC] `AGENT_HANDOFF.md:224-227`):
```
Multi-drone (over zenoh mesh): loop_closure_sp_node.py (SuperPoint+LightGlue+BoW,
  vocab models/sp_vocab_4096.pkl identical on all drones) → /loop_closures,
  /loop_closure_alignment → octomap_exchange_node (octomap sharing + swarm TF from
  config/swarm_alignment.yaml; reference drone = 1/delta)
```
ONNX vs TensorRT: no ONNX/TensorRT model artifacts evidenced anywhere locally; the only evidenced
runtime is the PyTorch pip-editable LightGlue. Exact SuperPoint weights/variant: NOT FOUND.

**What is transmitted between vehicles (topic-level; exact list).**
Fact: the zenoh bridge allowlist admits exactly 7 topic patterns across the mesh, identical for
publishers and subscribers: `/loop_closure/keyframe_bow`, `/loop_closure/keyframe_full`,
`/loop_closure_alignment`, `/loop_closures`, `/drone.*/octomap`, `/drone.*/octomap_merged`,
`/drone.*/swarm/pose`. The launch script's own echo annotates `keyframe_bow ~4KB RELIABLE` and
`keyframe_full ~540KB BEST_EFFORT`.
Evidence: [ON-DRONE] `/mnt/nova_ssd/workspaces/isaac_ros-dev/launch_mesh_zenoh.sh` (491 lines,
dated Apr 3), zenoh config heredoc written to `/tmp/zenoh_slam_mesh.json5`.
Snippet (heredoc, abridged to the allowlist):
```
plugins: { ros2dds: {
    nodename: "zenoh_bridge_${MY_NS}", domain: ${DOMAIN},   // DOMAIN=0
    ros_localhost_only: true,
    allow: { publishers: [ "/loop_closure/keyframe_bow", "/loop_closure/keyframe_full",
      "/loop_closure_alignment", "/loop_closures", "/drone.*/octomap",
      "/drone.*/octomap_merged", "/drone.*/swarm/pose" ],
      subscribers: [ /* identical 7-topic list */ ] } } }
```
Message field definitions and in-code payload sizing for these topics: NOT FOUND locally
(definitions live in `multi_drone_nvblox` on the drones).

**Map exchange node configuration.**
Fact: `octomap_exchange_node` (package `multi_drone_nvblox`) runs per-drone with
`drone_id:=<1..4>`, octomap `resolution:=0.05` (m), alignment from
`config/swarm_alignment.yaml`, `use_sim_time:=False`; reference drone for swarm alignment is
1/delta.
Evidence: [CODE] `launch_ghost_frontierexplore_tmux.sh:146-153` (parallel lines in the other three
scripts and all archive variants); [DOC] `AGENT_HANDOFF.md:226-227`.
Snippet:
```
        ssh_docker_tab "Tab 7: OctoMap Exchange (incremental)" \
            "ros2 run multi_drone_nvblox octomap_exchange_node --ros-args \
                -p robot_namespace:=drone3 \
                -p drone_id:=3 \
                -p resolution:=0.05 \
                -p alignment_config:=/workspaces/isaac_ros-dev/src/multi_drone_nvblox/config/swarm_alignment.yaml \
                -p use_sim_time:=False"
```
Known committed-script bug (flag for the paper's reproducibility notes, not a design fact):
`archive/launch_swarm.sh:236-237` hardcodes `robot_namespace:=drone1` for every drone while
`drone_id:=$ID` varies ([CODE]).

**Execution placement.**
Fact: the loop-closure node is launched as `python3` inside the `isaac_ros_realsense` container on
the Jetson Orin NX; torch with CUDA is available in that container. Explicit GPU-vs-CPU placement
of SuperPoint/LightGlue inference: NOT FOUND locally.
Evidence: [CODE] launch line above; [DOC] `AGENT_HANDOFF.md:429-430`.

**Related but separate radio side-channel (radiohive package; radio hardware NOT attached in
flight config — [DOC] `AGENT_HANDOFF.md:252`).** For completeness of "what is transmitted":
Fact: radiohive's `radio_bridge_node.py` defines packet types incl. `PKT_BOW_SPARSE = 0x03` and
`PKT_ALIGNMENT = 0x04` as "reliable" (ACK required, `MAX_RETRIES 3`, `ACK_TIMEOUT_S 0.5`), frame
sync `0xAA55`, CRC-16/CCITT-FALSE, float16 packing, default 57600 baud; `DroneStatus.msg` carries
`uint32 keyframes_total`, `uint32 loop_closures_total`, `uint32 octomap_nodes`; an
`odom_decimator_node` outputs at 5.0 Hz.
Evidence: [TRANSCRIPT] line 1514 (verbatim drone-side file dump from a prior session).
Snippet:
```
SYNC_WORD = 0xAA55
PKT_HEARTBEAT = 0x01
PKT_POSE = 0x02
PKT_BOW_SPARSE = 0x03
PKT_ALIGNMENT = 0x04
...
RELIABLE_TYPES = {PKT_BOW_SPARSE, PKT_ALIGNMENT, PKT_COMMAND, PKT_EMERGENCY}
MAX_RETRIES = 3
ACK_TIMEOUT_S = 0.5
```

**NOT FOUND for Q2 (all searched: full laptop sweep incl. VS Code history caches, Claude
transcripts, bash history; drone source not accessible under task rules):**
keyframe selection criteria and thresholds; candidate retrieval mechanism beyond "BoW";
geometric verification method and thresholds (RANSAC iterations, inlier threshold, min inliers,
PnP vs essential matrix, any ICP/SDF refinement); accept/reject and re-localization/retry logic;
message field definitions and in-code payload sizes; image stream (color resolution/rate) feeding
the node; SuperPoint model file/weights; PCM parameters (only the boolean `use_pcm:=True` is
evidenced); `sp_vocab_4096.pkl` contents (vocabulary size beyond the filename); contents of
`COMMS_PROTOCOL.md`/`PIPELINE.md`/`AGENTS.md` (paths only, `AGENT_HANDOFF.md:459`);
`swarm_alignment.yaml` contents.

---

## 4. Q3 — High-bandwidth WiFi link

**Adapter and driver.**
Fact: adapter is a Realtek RTL8822CE 802.11ac PCIe wireless NIC, PCI ID `[10ec:c822]`; kernel
`5.15.148-tegra`; driver stack is in-kernel rtw88 (`rtw88_8822ce` → `rtw88_8822c` → `rtw88_pci`
→ `rtw88_core` → patched `mac80211`); the Realtek vendor driver is blacklisted.
Evidence: [ON-DRONE] `lspci -nn`, `uname -r`, `lsmod`, `cat /etc/modprobe.d/blacklist-rtl88x2ce.conf`.
Snippet:
```
0001:01:00.0 Network controller [0280]: Realtek Semiconductor Co., Ltd. RTL8822CE 802.11ac PCIe Wireless Network Adapter [10ec:c822]
---
rtw88_8822ce 16384 0
rtw88_8822c 471040 1 rtw88_8822ce
rtw88_pci 32768 1 rtw88_8822ce
rtw88_core 155648 2 rtw88_pci,rtw88_8822c
mac80211 917504 2 rtw88_pci,rtw88_core
---
blacklist rtl88x2ce
```

**Nature of the kernel/driver modification.**
Fact: three changes, all required (per the on-drone guide `KERNEL_MESH_PATCH_GUIDE.md`,
23,991 bytes, in `/mnt/nova_ssd/workspaces/kernel_build/`):
1. rebuild `mac80211.ko` with `CONFIG_MAC80211_MESH=y` (stock JetPack 6 has it unset; this
   changes the CRC of ~28 of ~38 exported symbols, forcing the driver rebuild);
2. rtw88 Fix A: add `NL80211_IFTYPE_MESH_POINT` to interface modes in `main.c`;
3. rtw88 Fix B: in the `FIF_OTHER_BSS` handler in `mac80211.c`, clear
   `BIT_CBSSID_BCN | BIT_CBSSID_DATA` (without it "the hardware silently drops beacons from other
   BSSIDs... peers never discover each other").
Fact: there are NO standalone .patch/.diff files — the patches are applied inline by
`02_configure_and_build.sh` via `sed` + embedded python; `03_install.sh` backs up originals to
`backup_modules/*.ko.orig`, installs the rebuilt `.ko` files IN PLACE over the stock module paths
(`/lib/modules/5.15.148-tegra/kernel/...`, not `updates/` or `extra/`), writes the blacklist,
runs `depmod -a` and reboots; `04_rollback.sh` restores the `.ko.orig` files. Only `.ko` files are
replaced; `/boot/Image` and cfg80211 untouched.
Evidence: [ON-DRONE] `/mnt/nova_ssd/workspaces/kernel_build/{KERNEL_MESH_PATCH_GUIDE.md,
02_configure_and_build.sh,03_install.sh,04_rollback.sh}`; `modinfo rtw88_8822ce` (filename =
stock path); `find kernel_build -name '*.patch' -o -name '*.diff'` → empty.
Snippet (guide headers):
```
### 1. Rebuild mac80211 with `CONFIG_MAC80211_MESH=y`
**Fix A: Add `NL80211_IFTYPE_MESH_POINT` to interface modes** (`main.c`)
**Fix B: Clear `BIT_CBSSID_BCN | BIT_CBSSID_DATA` in the `FIF_OTHER_BSS` handler** (`mac80211.c`)
### 3. Rebuild the WiFi driver against the mesh-enabled kernel tree
```
Snippet (Fix B exact replacement block, from `02_configure_and_build.sh`):
```c
	if (changed_flags & FIF_OTHER_BSS) {
		if (*new_flags & FIF_OTHER_BSS) {
			rtwdev->hal.rcr |= BIT_AAP;
			rtwdev->hal.rcr &= ~(BIT_CBSSID_BCN | BIT_CBSSID_DATA);
		} else {
			rtwdev->hal.rcr &= ~(BIT_AAP);
			rtwdev->hal.rcr |= BIT_CBSSID_BCN;
		}
	}
```
Fact: patch is installed and active — `iw phy phy0 info` lists `* mesh point` among supported
interface modes ([ON-DRONE]; corroborated [DOC] `AGENT_HANDOFF.md:339-346`).

⚠️ Caveat for anyone auditing this fleet's wireless: `/proc/config.gz` on the drones still
reports `# CONFIG_MAC80211_MESH is not set` and `# CONFIG_RTW88 is not set`. That is expected
and does NOT mean the patch is absent — `/proc/config.gz` reflects the build config of the boot
kernel image, and the patch replaces only `.ko` files while `/boot/Image` is untouched ([ON-DRONE]
`KERNEL_MESH_PATCH_GUIDE.md`: stock config quoted with both symbols unset; `03_install.sh`
replaces modules in place). The loaded `mac80211.ko` carries the mesh symbols and `iw phy` offers
"mesh point". Reasoning from `/proc/config.gz` yields the opposite of the truth here.
(Empirically re-confirmed on-drone 2026-08-21 by a parallel session's read-only check.)

**Operating parameters.**
Fact: 802.11s mesh (not ad-hoc/IBSS), mesh ID `jetson-drone-mesh`, `MESH_FREQ="2437"` with
in-script comment `# Channel 6, 2.4 GHz`; join command is exactly
`iw dev "$IFACE" mesh join "$MESH_ID" freq "$MESH_FREQ"`. The script sets NO channel width, NO
fixed bitrate, NO txpower, NO MTU, NO `mesh_param` values (grep-verified over the whole script).
Mode switch sequence: `nmcli device set $IFACE managed no` → link down → `iw dev $IFACE set type
mp` → link up → mesh join; sysctls `rp_filter=0`, `accept_local=1`; blanket iptables ACCEPT on the
mesh iface; `arp on` + gratuitous `arping`. No tc/qdisc or net buffer tuning anywhere.
Evidence: [ON-DRONE] `/mnt/nova_ssd/workspaces/isaac_ros-dev/launch_mesh_zenoh.sh` (491 lines).
Live interface MTU is 1500 with default `noqueue` qdisc (observed in managed mode on bench WiFi;
mesh was not active at inspection time) ([ON-DRONE] `ip addr`, `tc qdisc show dev wlP1p1s0`).

**Addressing and peer discovery.**
Fact: static addressing `192.168.77.<id>/24` with node map in the script header
`1=delta 2=buckshee 3=ghost 4=thunderstrike`; `ip route replace 192.168.77.0/24 dev $IFACE`;
peer discovery is a ping loop of up to 60 s per launch expecting all listed peers (abort if 0
reachable, warn-and-continue if partial). BSSID handling: nothing in the script — each mesh peer
has its own BSSID and kernel Fix B stops the hardware dropping other-BSSID beacons.
Evidence: [ON-DRONE] `launch_mesh_zenoh.sh`; [DOC] `AGENT_HANDOFF.md:175-176, 329`.

**Transport carrying submap data.**
Fact: the prebuilt Rust `zenoh-bridge-ros2dds` binary at
`/mnt/nova_ssd/workspaces/isaac_ros-dev/zenoh-bridge-ros2dds` (16,252,848 B, dated Mar 9),
version `v1.7.2` (binary's own `--version` output), zenoh peer mode, TCP endpoints
`tcp/0.0.0.0:7447` listen + `tcp/192.168.77.<peer>:7447` connect per peer, multicast scouting
disabled / gossip enabled, ros2dds plugin with `ros_localhost_only: true`, domain 0, and the
7-topic allowlist (full list in Q2). The bridge process runs on the HOST with CycloneDDS pinned to
loopback; container ROS nodes use FastDDS with `ROS_LOCALHOST_ONLY=0`.
Evidence: [ON-DRONE] `launch_mesh_zenoh.sh` step [5/6] env + config heredoc; `zenoh-bridge-ros2dds --version`.
Snippet (bridge launch env, verbatim from script):
```
CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="127.0.0.1"/></Interfaces></General><Discovery><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>' \
  RUST_LOG=zenoh=warn,... ROS_LOCALHOST_ONLY=0 ROS_DISTRO=humble ROS_DOMAIN_ID=$DOMAIN \
  "$ZENOH_BRIDGE" -c /tmp/zenoh_slam_mesh.json5 > /tmp/zenoh_slam.log 2>&1 &
```
In-script comments: "CycloneDDS restricted to loopback only (no mesh traffic) ...
NetworkInterface=127.0.0.1 prevents any traffic on the mesh interface. MaxAutoParticipantIndex=200
ensures a free RTPS port above FastDDS's range." Host-side `ip link set lo multicast on`.

**Restore path (for completeness):** `iw mesh leave`, flush addr/routes, `rp_filter=2`, up to 2×
driver reload (`rmmod rtw88_8822ce/8822c/pci/core; modprobe rtw88_8822ce`), `nmcli connection up
eduroam` — a stale profile name (actual profile `eduroam [f90d3298]`; [DOC] `AGENT_HANDOFF.md:180-184`).

NOT FOUND for Q3: mesh channel width / HT mode (not specified anywhere; `mesh join ... freq 2437`
only); fixed bitrate / txpower / MTU overrides / qdisc-buffer tuning (none exist in the script or
anywhere searched); live mesh-mode `iw ... get mesh_param` values (mesh not active at inspection;
not observable read-only); laptop copies of `launch_mesh_zenoh.sh` / kernel patch files (none);
`prometheus@100.98.184.28` unreachable (`ssh: connect to host 100.98.184.28 port 22: Connection
timed out`) — wired fallback `ghost@192.168.0.206` used instead. Only ghost inspected; the other
three drones were not.

---

## 5. Q4 — LoRa TDMA firmware v7.1 (`/home/ssg/lora_radios/`, not a git repo)

⚠️ Version caveat the paper must get right: the file labeled v7.1
(`lora_mesh_v7_1_ros2.ino`) is a 3-node, fixed-length-TX build (150 ms cycle). The four
per-drone builds in `flash_ready/` are **v7.2** ("variable-length TX fix") with `NUM_NODES 4`
(190 ms cycle); each differs from `lora_mesh_v7_2_ros2.ino` ONLY in `#define NODE_ID` (line 38)
and `NUM_NODES 4` vs 3 (line 42) — verified by diff. Per-drone NODE_IDs: delta=0, buckshee=1,
ghost=2, thunderstrike=3. v7.2's sole functional change vs v7.1: transmits
`BASE_PACKET_SIZE + custom_len` bytes instead of the full ~197 B struct every cycle, drops (never
truncates) oversized custom payloads, underflow-safe TX window, RX length validation — PHY, TDMA
constants, sync logic, serial protocol unchanged (v7_2 comment at :529-535: "The old code sent
sizeof(Packet) (~198 B, ~79 ms) EVERY cycle, overrunning the slot and preventing the mesh from
forming").

**Radio module/chipset as named in firmware.**
Fact: Heltec Wireless Stick Lite V3 (ESP32-S3 + SX1262), via the `heltec_unofficial` library
(installs RadioLib per README).
Evidence: [CODE] `lora_mesh_v7_1_ros2.ino:2-3,34`; [DOC] `lora_radios/README.md:11,38-39`.
Snippet:
```
 * LoRa Decentralized TDMA Mesh v7.1 — ROS2 Serial Bridge
 * For: Heltec Wireless Stick Lite V3 (ESP32-S3 + SX1262)
...
#include <heltec_unofficial.h>
```

**PHY config.**
Fact: 915.0 (MHz per README:73-75 "US (ISM 915 MHz)"), bandwidth 500.0 (kHz), SF7, CR arg 5,
sync word 0x12, TX power 10, preamble 8; `radio.begin(...)` extra args `1.6, false`; current
limit 140.0 mA; hardware CRC on (`radio.setCRC(1)`). Max custom payload 160 B
(`MAX_CUSTOM_DATA 160`); serial buffer 220 B (`SERIAL_BUF_SIZE 220`).
Evidence: [CODE] `lora_mesh_v7_1_ros2.ino:48-54,64,79,365-366,371-372`.
Snippet (:48-54):
```
#define LORA_FREQ         915.0
#define LORA_BW           500.0
#define LORA_SF           7
#define LORA_CR           5
#define LORA_SYNC         0x12
#define LORA_PWR          10
#define LORA_PRE          8
```

**Slot/guard/frame structure.**
Fact: slot τ = 40 ms (`SLOT_MS 40`); guard = 30 ms (`GUARD_MS 30`) appended ONCE per cycle (not
per slot); cycle = `NUM_NODES*40 + 30` → 150 ms at NUM_NODES=3 (v7.1 root file) / 190 ms at
NUM_NODES=4 (deployed flash_ready builds). Slot index = NODE_ID. TX permitted only within
`[TX_MARGIN_MS, SLOT_MS − TX_MARGIN_MS − packet_airtime_ms]`, `TX_MARGIN_MS 5`. During the guard
tail `currentSlot()` returns −1 (nobody transmits). No sleep window: continuous RX except while
transmitting.
Evidence: [CODE] `lora_mesh_v7_1_ros2.ino:42-45,59,176-180,419-431`; NUM_NODES 4 at
`flash_ready/lora_delta_node0/lora_delta_node0.ino:42`.
Snippet (:42-45,59 + :419-425):
```
#define NUM_NODES         3
#define SLOT_MS           40
#define GUARD_MS          30
#define CYCLE_MS          (NUM_NODES * SLOT_MS + GUARD_MS)
#define TX_MARGIN_MS      5
...
      uint32_t slot_start = cycle_epoch + (getCycleClock(now) / CYCLE_MS) * CYCLE_MS
                            + (uint32_t)NODE_ID * SLOT_MS;
      uint32_t time_in_slot = now - slot_start;
      if (time_in_slot >= TX_MARGIN_MS &&
          time_in_slot <= (SLOT_MS - TX_MARGIN_MS - packet_airtime_ms)) {
```

**Sync beacon.**
Fact: there is no separate beacon packet. Every node's once-per-cycle TDMA packet carries the sync
fields `cycle_clock` (ms since sender's cycle epoch) and `sync_source`; effective beacon period =
CYCLE_MS (150/190 ms). Sync authority is decentralized, lowest-node-ID wins (`shouldSyncTo()`).
(`STATUS_MS 5000` at :57 is a serial-side status period, not RF.)
Evidence: [CODE] `lora_mesh_v7_1_ros2.ino:84-101,349-354,462-465`.
Snippet (:349-354):
```
bool shouldSyncTo(uint8_t sender_id, uint8_t sender_sync) {
  uint8_t auth = sender_id < sender_sync ? sender_id : sender_sync;
  if (auth < my_sync_source) return true;
  if (auth == my_sync_source && sender_id <= my_sync_source) return true;
  return false;
}
```

**Airtime compensation on frame alignment.**
Fact: on accepting a sync packet, the receiver back-dates reception by the packet time-on-air
(`tx_moment = now − packet_airtime_ms`), then subtracts the sender's in-frame offset
(`NODE_ID*SLOT_MS + TX_MARGIN_MS`) and whole elapsed cycles to recover the sender's epoch.
`packet_airtime_ms` is precomputed in setup as `radio.getTimeOnAir(sizeof(Packet))/1000` with a
6 ms fallback if 0. (v7.2 computes it from `BASE_PACKET_SIZE` instead — v7_2:382.)
Evidence: [CODE] `lora_mesh_v7_1_ros2.ino:375-376,512-519`.
Snippet (:512-519):
```
  if (shouldSyncTo(id, p->sync_source)) {
    uint32_t sender_pos = (uint32_t)id * SLOT_MS + TX_MARGIN_MS;
    uint32_t tx_moment = now - packet_airtime_ms;
    uint32_t sender_cycle_num = p->cycle_clock / CYCLE_MS;
    uint32_t new_epoch = tx_moment - sender_pos - sender_cycle_num * CYCLE_MS;

    int32_t correction = (int32_t)(new_epoch - cycle_epoch);
    cycle_epoch = new_epoch;
```

**Message types and byte sizes.**
Fact — over the air there is exactly ONE packet type, the packed `Packet` struct; computed packed
size at NUM_NODES=3 is 197 B (1+4+4+4+1+3+1+16+1+1+1+160); v7.1 always transmits
`sizeof(Packet)` (:487). v7.2 defines `BASE_PACKET_SIZE = sizeof(Packet) − MAX_CUSTOM_DATA`
(v7_2:106) = 37 B (NUM_NODES=3) / 38 B (NUM_NODES=4, deployed) and sends
`BASE_PACKET_SIZE + custom_len`.
Evidence: [CODE] `lora_mesh_v7_1_ros2.ino:84-101`.
Snippet:
```
typedef struct __attribute__((packed)) {
  uint8_t  node_id;
  uint32_t sequence;
  uint32_t uptime_ms;
  uint32_t cycle_clock;
  uint8_t  sync_source;
  int8_t   rssi_of[NUM_NODES];
  uint8_t  neighbor_mask;
  float    lat;  float lon;  float alt;  float hdg;
  uint8_t  has_custom;  uint8_t custom_dest;  uint8_t custom_len;
  uint8_t  custom_data[MAX_CUSTOM_DATA];
} Packet;
```
Fact — serial (host-link) message opcodes and sizes ([CODE] defines :67-76, layouts :19-29 +
sender code):
| Opcode | Name | Fields / size |
|---|---|---|
| 0x01 | MSG_POSE_UPDATE | lat(4) lon(4) alt(4) hdg(4) = 16 B |
| 0x02 | MSG_CUSTOM_MSG | dest_node(1) + data(N) |
| 0x03 | MSG_STATUS_REQ | no payload |
| 0x81 | MSG_PEER_POSE | node_id(1) lat(4) lon(4) alt(4) hdg(4) rssi(2) snr(4) = 23 B |
| 0x82 | MSG_PEER_MSG | src_node(1) + data(N) |
| 0x83 | MSG_STATUS_RESP | node_id(1) num_alive(1) + per-peer id(1) rssi(2) snr(4) |
| 0x84 | MSG_PEER_LOST | node_id(1) = 1 B |
| 0x85 | MSG_PEER_JOINED | node_id(1) rssi(2) snr(4) = 7 B |

Fact — "frontier claim" and "reference-id beacon" message types: NOT FOUND in this firmware.
Searched `grep -rni 'frontier|claim|reference|ref_id|heartbeat|beacon'` across all .ino/.py/README
in `/home/ssg/lora_radios/` (incl. v5/v6/v7 and flash_ready). Any such content would ride inside
the generic `custom_data` field (opcodes 0x02/0x82); no schema for it is defined in these files.
(The ROS-side schema may exist in the uncommitted `lora_bridge_node.py` on the drones — see
Unresolved.)

**CRC / ack behavior.**
Fact: hardware radio CRC enabled (`radio.setCRC(1)`, :372); serial framing protected by a 1-byte
XOR checksum over type+len+payload (:194-195; mirrored in `bridge_node.py:107-110`), bad frames
silently dropped (:293-297). NO LoRa-level ACK and NO retry — TX errors ignored; loss only
counted via sequence-number gaps (:534-535).
Evidence: [CODE] `lora_mesh_v7_1_ros2.ino:488-490`.
Snippet:
```
  int st = radio.transmit((uint8_t*)&pkt, sizeof(Packet));
  if (st != RADIOLIB_ERR_NONE) {
    // Silent fail — ROS2 can query status if needed
```

**Version string / where v7.1 is "tagged".**
Fact: no `VERSION` define; "v7.1" appears exactly twice — the header comment
`lora_mesh_v7_1_ros2.ino:2` ("LoRa Decentralized TDMA Mesh v7.1 — ROS2 Serial Bridge") and the
filename. The directory is not under version control, so there is no git tag. flash_ready headers
read "v7.2 — ... (variable-length TX fix)".

**Serial link to ROS 2.**
Fact: 115200 baud both ends (`SERIAL_BAUD 115200` ino:63; `bridge_node.py:59` param default,
port default `/dev/ttyUSB0` :58). Framing both directions `[0xAA][type][len][payload...][xor]`
(ino:11-17; identical 5-state parser in `bridge_node.py:161-199`). `bridge_node.py` packs pose as
`struct.pack('<ffff', pos.x, pos.y, pos.z, yaw)` (:130) — ENU x/y/z/yaw stuffed into the
firmware's lat/lon/alt/hdg floats; enforces the 160 B custom cap (:145-147); status request every
5 s (:79-80); ROS topics `~/mesh/{peer_pose,peer_msg,peer_joined,peer_lost,status}` out,
`~/mesh/send_pose` (PoseStamped) and `~/mesh/send_msg` (JSON with base64 data) in (:6-15,67-77).
Evidence: [CODE] paths as cited.

**README design statements ([DOC] — describes an OLDER (pre-v7) revision; do not cite its numbers
for v7.x):** purpose "replaces Meshtastic with a custom raw LoRa firmware using TDMA" (:5);
claimed "~11 Hz bidirectional updates ... ~5-10ms per packet airtime" (:7); a 3×30 ms slot / 90 ms
cycle diagram (:88-95) and a 30-byte packet table (:104-117) — both superseded by the v7.x code
above; pin map NSS GPIO8 / DIO1 GPIO14 / RESET GPIO12 / BUSY GPIO13 / SCK 9 / MISO 11 / MOSI 10
(:168-179).

---

## 6. Q5 — Hardware row and camera facts

**Airframe.**
- Fact: PX4 airframe param `SYS_AUTOSTART 4001` (generic quadcopter autostart id; both July-15
  exports). Evidence: [CODE] `px4_params/prometheus_uav_VIO_fused_PX4_params_july-15-2025.params:949`
  (`1	1	SYS_AUTOSTART	4001	6`).
- Fact (filename evidence, uncommitted local CAD, `/home/ssg/Downloads/Drone Assembly Parts/`):
  propeller CAD files `FAB_PROPELLER - 9x4.5 CCW.sldprt` / `... 9x4.5 CW.sldprt` (9×4.5 in
  props); motor archive `iflight-xing-2207-1800kv-1.snapshot.1.zip`; ESC `CYCLONE 35A.step`;
  carrier `reComputer-J4012.stp`; `RealSense_D435.SLDPRT`, `D435mount_cutout.SLDPRT`. These are
  local filenames, not committed docs — label accordingly if used.
- Fact ([DOC], uncommitted paper drafts, platform table `main_JFR_v2.tex:684-696` =
  `main_JFR_v5.tex:686-698`, verbatim rows):
```
Airframe & Mark4 V2 true-X, 3K carbon fiber, \PH\,mm diagonal\\
Propulsion & iFlight XING 2806.5 1300KV motors, \PH{} speed controllers, 7x4.5\,in propellers\\
Compute & NVIDIA Jetson Orin NX 16GB (onboard perception and planning)\\
Camera & Intel RealSense D435i on damped mount, $20^{\circ}$ downtilt\\
Height & Downward 1D rangefinder (direct height measurement)\\
Autopilot & Cube Orange Plus, PX4 \cite{meier2015px4} (attitude loop, EKF2)\\
Coordination link & LoRa SX1262 on ESP32-S3 module, 915\,MHz\\
Map-sharing link & Onboard 802.11 WiFi adapter in mesh mode\\
Power & 4S 5200\,mAh LiPo (77\,Wh), custom buck converter assembly, inline fuse\\
```
  (The deleted `main3.tex` said "ad-hoc mode" for the map-sharing link — wrong per Q3's verified
  802.11s mesh; the live drafts fixed it to "mesh mode". Takeoff mass, endurance, ESC model, and
  the diagonal remain literal `\PH` placeholders — `main_JFR_v2.tex:684-685,696-697`.)
- ⚠️ Frame/prop conflict, flagged by the paper's own authors ([DOC] `main_JFR_v2.tex:471-476`,
  `\todo`): "A 427\,mm diagonal is the 10\,in Mark4 V2 variant, while 7x4.5\,in propellers on a
  2806.5 1300KV motor are a 7\,in configuration; either the diagonal is that of the 7\,in variant
  or the vehicle runs propellers smaller than the frame accommodates." This also conflicts with
  the CAD filename evidence below (9×4.5 props, XING 2207 1800KV motor archive). Both records
  reported; unresolved.
- Prop-inch class / wheelbase / frame name in COMMITTED docs: NOT FOUND (draft-only facts above;
  custom CAD `Frame.SLDPRT`/`Frame.SLDASM` carries no dimension in any text file searched).

**Battery.**
- Fact: `BAT1_N_CELLS 4` (4S) and `BAT1_CAPACITY -1.0` (unset); `BAT1_V_CHARGED 4.0999…`,
  `BAT1_V_EMPTY 3.5`. A legacy `BAT_N_CELLS 3` line also exists in the same export (raw
  discrepancy, reported as-is). Evidence: [CODE]
  `px4_params/prometheus_uav_VIO_fused_PX4_params_july-15-2025.params:12,14,18,20,38`.
- Capacity: now documented in the live drafts ([DOC] `main_JFR_v2.tex:693`): `Power & 4S
  5200\,mAh LiPo (77\,Wh), custom buck converter assembly, inline fuse\\` — consistent with
  `BAT1_N_CELLS 4`. Draft-only fact; the PX4 export still has `BAT1_CAPACITY -1` (unset).
- Documented endurance: NOT FOUND as a number — still literal `\PH` placeholders
  (`main_JFR_v2.tex:696` `Endurance & \PH\,min in exploration flight`, also :586-587), and
  `main_JFR_v2.tex:1668` "Endurance is bounded by the current airframe and battery" (no number).
  No BOM file located despite the drafts referencing a "released bill of materials".

**Flight controller.**
- Fact: all three committed ULogs embed `ver_hw CUBEPILOT_CUBEORANGEPLUS`. Evidence: [CODE]
  `strings px4_logs/log_70_UnknownDate.ulg` (also log_71, logs/log_57):
```
char[24] ver_hwCUBEPILOT_CUBEORANGEPLUS
char[40] ver_sw99c40407ffd7ac184e2d7b4b293f36f10fe561ef
```
  (Version facts only — no experimental content read from logs.)

**D435i mount tilt — the 20° vs 25° question.**
Every piece of local evidence says **20°**; NO 25° record was found anywhere on this machine
(searched: whole `multi-robot-coordination` repo + git history, archive/, ghost_debug/, Documents,
Downloads, Desktop, VS Code history caches, Claude transcripts).
- Fact ([DOC] `AGENT_HANDOFF.md:276-283` — the four hardcoded locations, verbatim table):
```
| `src/active_exploration/src/odom_correction_node.cpp:9` | `kCameraPitchDeg = 20.0` (rebuild active_exploration) |
| `src/px4_offboard/include/px4_offboard/vio_bridge_node.hpp:24` | `kCamPitchDeg = 20.0` (C++ bridge actually used; rebuild px4_offboard) |
| `src/px4_offboard/scripts/vio_bridge.py:30` | `CAM_PITCH_DEG = 20.0` (legacy Python bridge) |
| `src/active_exploration/launch/reactive_guard.launch.py` | `camera_tilt_deg` default `20.0` (runtime param) |
```
  plus `AGENT_HANDOFF.md:20-21`: "Camera is physically pitched **20° down**".
- Fact ([CAPTURE] `ghost_debug/run01/data/A_wifi_on/tab1_pane.txt:25`, runtime echo):
  `tion started (pitch=20.0 deg). Publishing gravity-corrected odom -> camera0_link`
- Conflicting older record ([DOC] `archive/VIO_QUICKSTART.txt:79`):
  `CAM_PITCH_DEG = 0.0   # 0 for straight-forward D435i, 30 for angled T265` — a stale 0°/30°
  configuration note for the legacy Python bridge, NOT 25°.
- Additional corroboration ([DOC] `main_JFR_v2.tex:687`): `Camera & Intel RealSense D435i on
  damped mount, $20^{\circ}$ downtilt\\` — the paper draft itself also says 20°.
- URDF/xacro encoding of the tilt: NOT FOUND — no project URDF/xacro exists on this laptop (the
  only URDFs found are unrelated sim/vendor files under `px4_sim_ros2/` and
  `Downloads/realsense-ros-ros2-master/`).

**Camera streams.**
- Fact ([DOC] `AGENT_HANDOFF.md:207-208`):
```
RealSense D435i (/camera0/*, 640x480, depth 60fps, infra pair via realsense_splitter,
                 united IMU 200Hz on /camera0/imu)
```
  IMU 200 Hz corroborated by [CAPTURE] cuVSLAM echo `frequency: 200.000000` (tab1_pane.txt:1098).
  Camera identity: D435i serial `336222070908`, FW 5.13.0.55, USB3 5 Gbps ([DOC]
  `AGENT_HANDOFF.md:18-19`; the doc explicitly flags older D455 mentions as wrong/stale).
- Color stream resolution/rate as distinct numbers: NOT FOUND (launch scripts pass no camera
  args — [CODE] `launch_ghost_frontierexplore_tmux.sh:113` runs
  `realsense_example.launch.py` bare; the profile config lives in the on-drone workspace).

**Rangefinder.**
- Fact: EKF2 range fusion enabled — `EKF2_RNG_CTRL 1` with tuning `EKF2_RNG_A_HMAX 5.0`,
  `RNG_DELAY 5.0`, `RNG_GATE 5.0`, `RNG_NOISE 0.1`, `RNG_POS_X/Y/Z 0.0`, `EKF2_MIN_RNG 0.1`
  ([CODE] `...VIO_fused...july-15....params:397,425-438`, `EKF2_RNG_CTRL` at :428).
- Fact: every distance-sensor DRIVER enable in the exports is 0 (`SENS_TFMINI_CFG 0`,
  `SENS_CM8JL65_CFG 0`, `SENS_SF0X_CFG 0`, `SENS_EN_LL40LS 0`, `SENS_EN_VL53L0X/L1X 0`,
  `SENS_EN_TF02PRO 0`, etc.; [CODE] params :906-944).
- Rangefinder model and output rate: NOT FOUND. Only mention: uncommitted paper draft
  `main_JFR_v2.tex:688` `Height & Downward 1D rangefinder (direct height measurement)\\` (no model/rate).

---

## 7. Q6 — Estimator and mapping configuration

**PX4 version — independent evidence items (reported separately; note the internal discrepancy):**
1. Repo param-export headers (July 2025): `# Version: 1.15.2` ([CODE]
   `px4_params/prometheus_uav_VIO_fused_PX4_params_july-15-2025.params:1-5`, identical in the
   STABLE and july-5 files).
2. Committed ULogs: `ver_sw 99c40407ffd7ac184e2d7b4b293f36f10fe561ef` on
   `CUBEPILOT_CUBEORANGEPLUS` ([CODE] strings of `px4_logs/log_70/71`, `logs/log_57`).
3. `git describe --tags 99c40407ff` in the local PX4 checkout → exactly `v1.15.4` (no offset);
   commit subject `gps: revert ubx changes, point to 1.15 branch` ([CODE]
   `/home/ssg/Pixhawk-development/PX4-Autopilot`).
4. Newer QGC exports on Desktop (uncommitted): `# Version: 1.15.4` + `# Git Revision:
   99c40407ff000000` ([CODE] `/home/ssg/Desktop/px4_1.15.4_prometheus.params:1-6`,
   `/home/ssg/Desktop/GhostPX4Logs/cube_orange_plus_prometheus.params:1-6`).
5. `px4_msgs` pinned `release/1.15`, "must match firmware" ([DOC] `AGENT_HANDOFF.md:26,262`).

**EKF2 external-vision configuration (authoritative export:
`px4_params/prometheus_uav_VIO_fused_PX4_params_july-15-2025.params`; format
`vehicle_id component_id name value type`):**
```
333:1	1	EKF2_BARO_CTRL	1	6
347:1	1	EKF2_EVA_NOISE	0.100000001490116119	9
349:1	1	EKF2_EVP_NOISE	0.100000001490116119	9
351:1	1	EKF2_EVV_NOISE	0.100000001490116119	9
352:1	1	EKF2_EV_CTRL	15	6
353:1	1	EKF2_EV_DELAY	33.000000000000000000	9
354:1	1	EKF2_EV_NOISE_MD	0	6
355:1	1	EKF2_EV_POS_X	0.059999998658895493	9
356:1	1	EKF2_EV_POS_Y	-0.029999999329447746	9
357:1	1	EKF2_EV_POS_Z	-0.079999998211860657	9
```
```
358:1	1	EKF2_EV_QMIN	0	6
364:1	1	EKF2_GPS_CTRL	7	6
380:1	1	EKF2_HGT_REF	3	6
428:1	1	EKF2_RNG_CTRL	1	6
959:1	1	SYS_HAS_BARO	1	6
960:1	1	SYS_HAS_GPS	1	6
961:1	1	SYS_HAS_MAG	1	6
```
- EKF2_EV_CTRL = 15 = binary 1111 (bits 0,1,2,3 set). The repo's own gloss: [DOC]
  `archive/VIO_QUICKSTART.txt:19` `EKF2_EV_CTRL = 15 (fuse position + velocity + yaw)`;
  [DOC] `AGENT_HANDOFF.md:216-217` `EKF2 (EV fusion: EKF2_EV_CTRL=15, EKF2_HGT_REF=3/vision;
  set via QGC)`. Per-bit meanings beyond this gloss: not evidenced in this codebase.
- EKF2_EV_DELAY = 33 ms; EV lever arm (EKF2_EV_POS_X/Y/Z) = (0.06, −0.03, −0.08) m;
  EKF2_HGT_REF = 3 ("vision" per VIO_QUICKSTART:21).
- GPS is NOT disabled in this export: `EKF2_GPS_CTRL 7`, `SYS_HAS_GPS 1` (raw facts).
- Prescription vs export discrepancy: VIO_QUICKSTART.txt:20 prescribes `EKF2_EV_QMIN = 1`; the
  actual export has `EKF2_EV_QMIN 0` (:358).
- Params live in: QGC exports in `px4_params/` (this repo). Set "via QGC" per
  [DOC] `AGENT_HANDOFF.md:217`. No ansible anywhere (searched; none exists).

**STABLE → VIO_fused delta (july-15 pair; EKF2*/UXRCE*/SENS* only — no RTPS_*/MAV_* differ):**
EKF2_EV_DELAY 0.0→33.0; EKF2_EV_POS_X/Y/Z 0/0/0→0.06/−0.03/−0.08; EKF2_GPS_P_NOISE 0.5→0.01;
EKF2_GPS_V_NOISE 0.3→0.01; EKF2_HGT_REF 1→3; EKF2_MAG_TYPE 0→5; SENS_BOARD_X_OFF
−0.6218→0.1973; SENS_BOARD_Y_OFF 1.5859→1.0606; UXRCE_DDS_CFG 0→104; UXRCE_DDS_DOM_ID/KEY/
PTCFG/SYNCC/SYNCT absent→0/1/0/0/1; SER_TEL4_BAUD absent→921600. ([CODE] diff of the two files.)

**uXRCE-DDS link.**
- Fact ([CODE] `...VIO_fused...july-15....params:1076-1081`):
```
1076:1	1	UXRCE_DDS_CFG	104	6
1077:1	1	UXRCE_DDS_DOM_ID	0	6
1078:1	1	UXRCE_DDS_KEY	1	6
1079:1	1	UXRCE_DDS_PTCFG	0	6
1080:1	1	UXRCE_DDS_SYNCC	0	6
1081:1	1	UXRCE_DDS_SYNCT	1	6
```
  `UXRCE_DDS_CFG 104` glossed as TELEM4 ([DOC] VIO_QUICKSTART.txt:17); `SER_TEL4_BAUD 921600`
  (params:947).
- Domain-ID invariant (post-dates the July-2025 export above): each drone's `UXRCE_DDS_DOM_ID`
  must EQUAL its ROS_DOMAIN_ID — delta 1, buckshee 2, ghost 3, thunderstrike 4; ghost set
  2026-06-09. Evidence: [CODE] `launch_ghost_frontierexplore_tmux.sh:39-42`:
```
# NOTE: PX4 param UXRCE_DDS_DOM_ID must EQUAL this ROS_DOMAIN_ID (set via QGC +
# FMU reboot; done for ghost=3 on 2026-06-09). If they differ, PX4's /fmu/* topics
# land on a different DDS domain and EKF2 silently gets no VIO (see ghost_debug/POSTMORTEM.md).
DOCKER_SOURCE="export ROS_DOMAIN_ID=3 ROS_LOCALHOST_ONLY=0 && ..."
```
  plus README.md:19-22 ("Ghost was set 2026-06-09; verify the others before flying").
- Serial path: FTDI FT232R USB-serial → `/dev/ttyUSB0` @ 921600 on the Jetson; PX4 side
  `/dev/ttyS3` (TELEM port); agent `MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600`;
  no MAVLink path on the Jetson ([DOC] `AGENT_HANDOFF.md:21-26,316`; [DOC]
  `archive/VIO_QUICKSTART.txt:52`).
- Odometry topic: `/visual_slam/tracking/odometry` (~30 Hz, [DOC] `AGENT_HANDOFF.md:230`) →
  `vio_bridge` (C++, package `px4_offboard`) → `/fmu/in/vehicle_visual_odometry` ([DOC]
  `AGENT_HANDOFF.md:213-217`). Rate actually delivered on `/fmu/in/vehicle_visual_odometry`:
  NOT FOUND as a stated number.

**nvblox configuration (runtime echoes, ghost, 2026-06-09 run):**
- Fact: voxel size 0.05 m; all layers GPU-resident (`memory_type: kDevice`); mapping type
  `kStaticTsdf`; `esdf_mode: 3d`; `pose_frame: base_link`; max integration distance 10 m; node
  tick 10 ms. Evidence: [CAPTURE] `ghost_debug/run01/data/A_wifi_on/tab1_pane.txt:60-62,290,
  484-487,495,505-507,531`.
Snippet:
```
up nvblox node in frame odom and voxel size 0.05
[component_container_mt-3] [INFO] ... Mapping type: kStaticTsdf
    voxel_size: 0.050000
    esdf_mode: 3d
    pose_frame: base_link
```
- Fact: integration rates — integrate_depth 40 Hz, integrate_color 5 Hz, update_mesh 5 Hz,
  update_esdf 10 Hz, publish_layer 5 Hz. Evidence: [CAPTURE] `tab1_pane.txt:545-551`.
- Committed config file (`nvblox_base.yaml`, on-drone) values: NOT FOUND locally beyond the
  launch-script comment "±25m workspace bounds" ([CODE] `launch_ghost_frontierexplore_tmux.sh:131-132`)
  and the [TRANSCRIPT] commit `3e2c17c` bounds (x/y ±25 m, height −0.4…3.2 m).

**Inter-robot map exchange serialization.**
- Fact: octomaps exchanged at `resolution:=0.05` m via `octomap_exchange_node`
  ([CODE] launch line, Q2); transported over the zenoh-bridged topics `/drone.*/octomap`,
  `/drone.*/octomap_merged` ([ON-DRONE] zenoh allowlist, Q2/Q3); keyframe messages annotated
  `keyframe_bow ~4KB RELIABLE`, `keyframe_full ~540KB BEST_EFFORT` ([ON-DRONE]
  `launch_mesh_zenoh.sh` echo lines).
- Block/message sizing as defined in code (message structs, serialization code): NOT FOUND
  locally (source on drones only).

---

## 8. Unresolved — every NOT FOUND in one list

Q1:
1. Upstream base of `isaac_ros_nvblox` as a concrete remote/tag/commit (only the doc string
   "base v3.2-14"); same for `isaac_ros_visual_slam` (no base version stated anywhere local).
2. The actual 580-file `isaac_ros_nvblox` diff (only 4 fork commit hashes/messages recovered
   via transcript: 3e2c17c, 2c4996f, 52af58c, 7774f42; plus active_exploration@0ad55ec).
3. CUDA stream / stream-priority changes — no evidence they exist.
4. ROS QoS changes inside the forks.
5. realsense-ros IMU queue 5→100 actual diff hunk; isaac_ros_visual_slam rviz diff;
   isaac_ros_common run_dev.sh diff (doc claims only).
6. `fastdds_loopback.xml` contents (path only).

Q2:
7. `loop_closure_sp_node.py` source → keyframe selection criteria/thresholds; candidate
   retrieval mechanism beyond "BoW"; SuperPoint/LightGlue inference settings and model files;
   ONNX vs TensorRT (no artifacts evidenced; PyTorch pip-editable LightGlue is the only
   evidenced runtime); geometric verification method + thresholds (RANSAC iters, inlier
   threshold, min inliers, PnP vs essential, ICP/SDF refinement); accept/reject +
   re-localization/retry logic; PCM parameters (only `use_pcm:=True`).
8. Message field definitions and in-code payload sizes for `/loop_closures`,
   `/loop_closure_alignment`, `/loop_closure/keyframe_bow`, `/loop_closure/keyframe_full`
   (only the ~4KB/~540KB script annotations).
9. Which image stream (color resolution/rate) feeds place recognition.
10. GPU/CPU placement of SP/LG inference (beyond: python3 node, torch CUDA available).
11. `swarm_alignment.yaml`, `COMMS_PROTOCOL.md`, `PIPELINE.md`, `AGENTS.md`,
    `sp_vocab_4096.pkl` contents.
12. octomap_exchange incremental-update scheme and exchange rate.

Q3:
13. Mesh channel width / HT mode (not specified in `launch_mesh_zenoh.sh`; not documented).
14. Fixed bitrate, txpower override, MTU change, qdisc/buffer tuning — none exist (searched
    script + docs; live interface shows default MTU 1500 / noqueue in managed mode).
15. Live mesh_param values (mesh not active at inspection time; not observable read-only).
16. `prometheus@100.98.184.28` unreachable (connect timed out); used `ghost@192.168.0.206`.
    Drones other than ghost not inspected.

Q4:
17. Frontier-claim and reference-id-beacon message types in the LoRa firmware (no such schema in
    `/home/ssg/lora_radios/`; only the generic 160 B `custom_data` container). The ROS-side
    schema may exist in the drones' uncommitted `lora_bridge_node.py` (memory note; not committed
    to any repo, not locally verifiable).
18. A git tag for v7.1 (directory not under version control; version exists only as header
    comment + filename).

Q5:
19. Wheelbase/diagonal as a number (still `\PH` in the live drafts); frame vs prop pairing
    CONFLICTED, not just missing — draft table says Mark4 V2 + 7×4.5 in props with an author
    `\todo` flagging the inconsistency (main_JFR_v2.tex:471-476), while CAD filenames say
    9×4.5 props + XING 2207 1800KV motors. Needs the user to state which is flown.
20. Documented endurance and takeoff mass (still `\PH` placeholders in the live drafts);
    battery capacity is now draft-documented (4S 5200 mAh LiPo 77 Wh, main_JFR_v2.tex:693)
    but BAT1_CAPACITY remains unset (−1) in the PX4 exports; no BOM file found.
21. D435i tilt in URDF/xacro (no project URDF exists locally); NOTE: no 25° record found
    anywhere — all evidence says 20° (four code sites, runtime echo, and the paper draft's
    own "20° downtilt"; plus a stale 0°/30° note in VIO_QUICKSTART.txt:79).
22. Color stream resolution/rate; rangefinder model and output rate (drivers all disabled in
    exports; only "Downward 1D rangefinder" in the live drafts, main_JFR_v2.tex:688).

Q6:
23. PX4 version discrepancy unresolved at the evidence level: repo July-2025 export headers say
    1.15.2 while committed ULogs + Desktop exports + local checkout say v1.15.4 (99c40407ff).
24. Delivered rate on `/fmu/in/vehicle_visual_odometry` (only upstream ~30 Hz documented).
25. `nvblox_base.yaml` committed values (runtime echoes + transcript commit only).
26. Ansible playbooks/roles: none exist anywhere on this machine (searched /home/ssg for
    ansible dirs/yml) — provisioning is manual per AGENT_HANDOFF.md.
27. Project URDF/xacro for the vehicle: none exists locally.
