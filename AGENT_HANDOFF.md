# AGENT HANDOFF — Jetson "ghost" / isaac_ros-dev workspace

Written 2026-06-10 for a remote Claude agent SSH'd into this machine. Everything below was
verified against the live filesystem/container on that date unless marked otherwise.

## What this machine is

Autonomous exploration drone **"ghost"** — drone **3** of a 4-drone fleet
(1=delta, 2=buckshee, 3=ghost, 4=thunderstrike). Georgia Tech project (user:
sgarimella34@gatech.edu, GitHub `ssaigarimella`). The drone runs visual-inertial SLAM
(Isaac ROS cuVSLAM) + nvblox 3D mapping on a RealSense, autonomously explores via frontier
detection, feeds VIO to a PX4 flight controller, and shares loop closures / octomaps with
peer drones over an 802.11s WiFi mesh bridged by Zenoh.

- **Hardware**: Jetson Orin NX 16GB on Seeed reComputer J401 carrier. JetPack 6.2.1
  (L4T R36.4.0), Ubuntu 22.04, kernel `5.15.148-tegra` (custom out-of-tree build — see
  "WiFi mesh kernel patch" below). CUDA 12.6, TensorRT 10.3.0.30.
- **Camera**: Intel RealSense **D435i** (serial `336222070908`, FW 5.13.0.55, USB3 5Gbps).
  Docs that say D455 (DEPLOYMENT_GUIDE.md) are **wrong/stale**. Camera is physically
  pitched **20° down** — see "Camera tilt" section.
- **Flight controller**: Pixhawk-family PX4 (exact model/firmware not recorded anywhere on
  this machine; ULogs live only on the Pixhawk SD card). Connected via FTDI FT232R USB-serial
  → **`/dev/ttyUSB0` @ 921600**, PX4 side `/dev/ttyS3` (a TELEM port). Protocol is
  **uXRCE-DDS** (MicroXRCEAgent), *not* MAVROS/MAVLink — there is no MAVLink path on this
  Jetson at all; PX4 params/console require QGroundControl on an operator laptop.
  `px4_msgs` is pinned to `release/1.15` and must match firmware.
- **Storage**: workspace lives on NVMe at `/mnt/nova_ssd`. Docker data-root was moved to
  `/mnt/nova_ssd/docker`.

## Remote access (read this first)

- **SSH: `ssh ghost@192.168.0.206` (Ethernet) — preferred.** Password `<JETSON_PASS — see credentials.env>`.
  The gatech ed25519 key is in `authorized_keys`. Port 22, all interfaces.
- **Why Ethernet**: WiFi (`wlP1p1s0`, currently static `192.168.0.50` on SSID
  `TP-Link_073A`) gets torn down by `launch_mesh_zenoh.sh` (mesh switch) and
  `flight_autonomous.sh` (IFL network switch). A session on `.50` dies mid-debug;
  `.206` survives.
- **Sudo is passwordless**: `/etc/sudoers.d/claude-debug` → `ghost ALL=(ALL) NOPASSWD: ALL`.
  Always use `sudo -n`. (Scripts in this repo still do `echo <JETSON_PASS — see credentials.env> | sudo -S` — historical.)
- An SSH shell is a normal shell. By contrast, the **VS Code agent running ON this Jetson is
  inside a Flatpak sandbox**: for it, `docker`/`systemctl`/`nmcli` are not on PATH and it must
  use `flatpak-spawn --host <cmd>` (host rootfs visible at `/run/host`). If you are reading
  this over SSH, ignore that and use the tools directly.
- Internet on the bench comes from a **laptop NAT at `192.168.0.136`** (route metric 50,
  installed at boot by `bench-internet.service` → `/home/ghost/bench_internet.sh`). If that
  laptop is off, the Jetson may have no internet even though routes exist. That service also
  runs `timedatectl set-ntp true` at every boot — it silently undoes any manual
  `set-ntp false`.
- Tailscale was installed once and **removed** (`tailscaled.service` is masked → /dev/null,
  binaries gone). Do not assume tailscale connectivity.
- An OLED status display on the drone (`dronedisplay.service`, SSD1306 on I2C bus 7) shows
  user/SSID/WiFi IP/time — physically useful to read the drone's current IP after a network
  switch. Source: `/home/ghost/dronedisplay_bundle/`, runs from `/opt/dronedisplay/`.
- Claude Code is installed on the Jetson itself and has been run as
  `claude --dangerously-skip-permissions`.

## The golden rule: everything ROS happens inside one Docker container

ROS 2 Humble is **not installed on the host** (`/opt/ros` doesn't exist). All ROS work goes
through container **`isaac_ros_realsense`** (image **`isaac_ros:dev-realsense`**, 29.6GB).

```bash
# Aliases in /home/ghost/.bashrc:
alias cdisaacrosdev='cd /mnt/nova_ssd/workspaces/isaac_ros-dev'
alias opendocker='docker exec -it -u admin isaac_ros_realsense /bin/bash'
alias startdocker='docker start isaac_ros_realsense'
export ISAAC_ROS_WS=/mnt/nova_ssd/workspaces/isaac_ros-dev/
```

- Container user is **`admin`** (uid 1000, in `dialout`+`sudo`, NOPASSWD inside container).
- Workspace is bind-mounted: host `/mnt/nova_ssd/workspaces/isaac_ros-dev` =
  container `/workspaces/isaac_ros-dev`. Second mount:
  `/mnt/nova_ssd/workspaces/Micro-XRCE-DDS-Agent` → `/workspaces/Micro-XRCE-DDS-Agent`.
- **Restart policy is `no`** — after any reboot the container shows `Exited (137)` and must
  be `docker start`ed. (Timestamps like "56 years ago" are the boot-clock artifact, see
  "Clock weirdness".)
- Canonical one-liner to run anything ROS:
  ```bash
  docker exec -it -u admin isaac_ros_realsense bash -c \
    "export ROS_LOCALHOST_ONLY=0 && source /opt/ros/humble/setup.bash && \
     cd /workspaces/isaac_ros-dev && source install/setup.bash && <ros2 command>"
  ```
- Recreate command if the container is ever removed (used many times in history):
  ```bash
  docker run -dit --privileged --network host --ipc host \
    -v /mnt/nova_ssd/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
    -v /mnt/nova_ssd/workspaces/Micro-XRCE-DDS-Agent:/workspaces/Micro-XRCE-DDS-Agent \
    -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/.Xauthority:/home/admin/.Xauthority:rw \
    -e DISPLAY -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
    --runtime nvidia --name isaac_ros_realsense isaac_ros:dev-realsense /bin/bash
  ```

### ⚠️ The image is a hand-mutated snapshot — `docker commit` or lose your changes

`isaac_ros:dev-realsense` was built by ~16 successive `docker commit`s. **There is no
Dockerfile that reproduces it.** Anything you install/fix *inside* the container (apt, pip,
files outside the bind mounts) is lost if the container is recreated, unless you run:
```bash
docker commit isaac_ros_realsense isaac_ros:dev-realsense
```
This loss has already happened at least once: the MicroXRCEAgent env-var wrapper and the
`jetson_inference` python install documented in `docs/` are **currently missing** from the
container (see "Known broken things").

(The official `src/isaac_ros_common/scripts/run_dev.sh` flow exists but was abandoned;
its config at `src/isaac_ros_common/scripts/.isaac_ros_common-config` has
`CONFIG_IMAGE_KEY=ros2_humble.realsense`.)

## How the user runs things

### A. Primary interactive launcher (GUI only): `./launch_ghost_local.sh`

Run on the Jetson's own desktop. Opens **9 gnome-terminal tabs**, each `docker exec`s into
the container and waits for Enter. **Fails over plain SSH** (gnome-terminal needs DISPLAY;
a local GUI session usually exists as `:1` but tabs appear on the physical screen).
Flags: `--rviz` (RViz with tab 1), `--debug` (skip arm check, for hand-carry tests).

| Tab | Component | Command (inside container) |
|---|---|---|
| 1 | cuVSLAM + RealSense + nvblox | `ros2 launch nvblox_examples_bringup realsense_example.launch.py run_rviz:=False` |
| 2 | VIO bridge + XRCE agent | `sudo chmod 666 /dev/ttyUSB0; ros2 launch px4_offboard vio_bridge.launch.py` |
| 3 | Frontier Info Structure | `ros2 launch active_exploration fis.launch.py flight_height:=1.0` |
| 4 | Reactive depth guard | `ros2 launch active_exploration reactive_guard.launch.py` |
| 5 | Exploration planner | `ros2 launch active_exploration simple_planner_launch.py flight_height:=1.0 debug_skip_arm_check:=<false/true>` |
| 6 | Loop closure (SuperPoint+LightGlue) | `python3 src/multi_drone_nvblox/scripts/loop_closure_sp_node.py ... -p robot_namespace:=drone3` |
| 7 | OctoMap exchange | `ros2 run multi_drone_nvblox octomap_exchange_node ... -p drone_id:=3` |
| 8 | Mesh + Zenoh (**runs on HOST, sudo**) | `sudo bash launch_mesh_zenoh.sh <my_id> <peer_ids...>` |
| 9 | Interactive container shell | `bash` |

The user's actual pre-flight ritual (from shell history, dozens of repetitions):
```bash
cdisaacrosdev && ./check_repos.sh        # git status/branch of every src/ repo
# git pull inside src/active_exploration and/or src/multi_drone_nvblox
docker ps -a
docker rm -f isaac_ros_realsense         # often recreates container from scratch
./launch_ghost_local.sh --debug --rviz
```

### B. Headless / SSH-safe launcher: `./run_ghost.sh`

Same 7 ROS components as tabs 1–7 but fully detached (`docker exec -d`), no prompts.
Per-component logs go to **`/tmp/ghost_tab1.log` … `/tmp/ghost_tab7.log` INSIDE the
container** (`docker exec isaac_ros_realsense cat /tmp/ghost_tab2.log`). Caveats:

- It first runs `pkill -9 -f ros2; pkill -9 -f python3` in the container (kills *all* python).
- Tab 5 hardcodes `debug_skip_arm_check:=true` — **do not use as-is for a real flight**.
- Its **last line runs `launch_mesh_zenoh.sh 3 1` in the foreground**, which switches WiFi
  to the mesh and will drop a WiFi SSH session. Over SSH, either Ctrl+C right after
  "All Docker tabs launched." or comment out the last line.
- Unlike the other launchers it exports
  `FASTRTPS_DEFAULT_PROFILES_FILE=.../multi_drone_nvblox/config/fastdds_loopback.xml`
  (see "DDS configuration mess").

### C. Field flight orchestrator: `src/active_exploration/scripts/flight_autonomous.sh`

Used for real autonomous flights at the IFL lab (Vicon mocap). Deployed from the ground
laptop **"juggernaut"**:
```bash
sshpass -p <JETSON_PASS — see credentials.env> ssh ghost@<IP> "echo <JETSON_PASS — see credentials.env> | sudo -S nohup bash \
  /mnt/nova_ssd/workspaces/isaac_ros-dev/src/active_exploration/scripts/flight_autonomous.sh \
  > /mnt/nova_ssd/workspaces/isaac_ros-dev/experiment_logs/flight_launch.log 2>&1 &"
```
Flow: **first step is the WiFi switch** to the IFL router (`TP-Link_C624`, PSK `<IFL-PSK — ask operator>`,
ghost = `192.168.10.10`, Vicon PC = `192.168.10.2`, vicon UDP bridge port 51001; SSH from
the bench network drops immediately — expected) → then it starts the Docker/ROS stack →
fly (arm in Position mode on RC, switch to Offboard → exploration starts) → auto-stop after
30s disarmed or `touch experiment_logs/flight_stop` (the only file-flag the script checks;
there is no "flight_confirm" gate — older docs mentioning one are wrong).
`--bench` skips the network switch (SSH-safe); `--timeout <min>` available. Per-flight logs:
`experiment_logs/flight_YYYYMMDD_HHMMSS/{cuvslam,vio_bridge,fis,reactive_guard,explorer,...}.log`,
bag recording via `scripts/record_flight.sh` → `experiment_logs/bags/` (mcap).

### D. Mesh + Zenoh bridge: `sudo bash launch_mesh_zenoh.sh <my_id> <peer_ids...>`

Ghost is node 3 → `sudo bash launch_mesh_zenoh.sh 3 1` (peer delta). Switches the single
WiFi radio to 802.11s mesh `jetson-drone-mesh` (ch 6 / 2437 MHz), IP `192.168.77.<id>/24`,
then runs the prebuilt Rust **`zenoh-bridge-ros2dds`** (workspace root, zenoh 1.7.2,
TCP 7447) with config written to `/tmp/zenoh_slam_mesh.json5`; bridge log
`/tmp/zenoh_slam.log`. Only loop-closure/octomap/swarm-pose topics are allowed across.
Ctrl+C restores WiFi — **but restore targets profile name `eduroam` which currently doesn't
match** (profile is named `eduroam [f90d3298]`, and the bench network is TP-Link anyway).
After a mesh run, expect to fix WiFi manually:
```bash
sudo nmcli connection up "TP-Link_073A 1"   # bench WiFi, static 192.168.0.50
```
WiFi driver is rtw88_8822ce; the script retries a driver reload (`rmmod rtw88_* && modprobe
rtw88_8822ce`) up to 2× on restore.

### E. Building

Always inside the container. Typical:
```bash
docker exec -it -u admin isaac_ros_realsense bash -c \
  "source /opt/ros/humble/setup.bash && cd /workspaces/isaac_ros-dev && \
   colcon build --symlink-install --packages-select active_exploration"
```
- `./build_active_exploration.sh` wraps this **but stops AND REMOVES the container when
  done** — any launcher recreates it, but in-container un-committed state is lost.
- multi_drone_nvblox needs `--allow-overriding multi_drone_nvblox`.
- Build artifacts in `build/`/`install/` are sometimes root-owned (builds run as root);
  scripts `chown admin:admin` as a workaround. `install/setup.bash` is sourced by every
  launcher; never sourced on the host.

## The ROS pipeline (what should be running during a flight)

```
RealSense D435i (/camera0/*, 640x480, depth 60fps, infra pair via realsense_splitter,
                 united IMU 200Hz on /camera0/imu)
  → cuVSLAM (vslam_container; stereo infra + IMU fusion; odometry-only,
             localization/mapping OFF; publish_odom_to_base_tf FALSE)
      → /visual_slam/tracking/odometry
          ├─→ odom_correction (active_exploration): rotates +20° pitch, publishes
          │     odom→camera0_link TF that nvblox consumes
          ├─→ vio_bridge (px4_offboard, C++): same 20° compensation
          │     → /fmu/in/vehicle_visual_odometry → MicroXRCEAgent (serial
          │       /dev/ttyUSB0 @921600) → PX4 EKF2 (EV fusion: EKF2_EV_CTRL=15,
          │       EKF2_HGT_REF=3/vision; set via QGC)
          └─→ nvblox (nvblox_container) → ESDF
                → FIS (frontier clustering/viewpoints, /fis/*)
                → simple_exploration_planner.py → /planning/trajectory_setpoint
                → reactive_depth_guard — SOLE writer of /fmu/in/trajectory_setpoint
                  and /fmu/in/offboard_control_mode (depth-based PASS/STOP gate,
                  also writes guard_*.csv telemetry)
Multi-drone (over zenoh mesh): loop_closure_sp_node.py (SuperPoint+LightGlue+BoW,
  vocab models/sp_vocab_4096.pkl identical on all drones) → /loop_closures,
  /loop_closure_alignment → octomap_exchange_node (octomap sharing + swarm TF from
  config/swarm_alignment.yaml; reference drone = 1/delta)
```

Key topics for debugging: `/visual_slam/tracking/odometry` (~30Hz),
`/fmu/in/vehicle_visual_odometry`, `/fmu/out/vehicle_status` (~1Hz),
`/fmu/out/vehicle_local_position` (~50Hz), `/planning/trajectory_setpoint`,
`/reactive_guard/blocked`, `/nvblox_node/static_esdf_pointcloud`.

Note: cuVSLAM/RealSense/splitter load into `vslam_container`, nvblox into
`nvblox_container` (a local patch, "isolated from nvblox CUDA stalls"). The planner used in
flight is the **Python `simple_exploration_planner.py`**, not the older C++
`exploration_manager`.

## Source tree (`src/`) — main repos on branch `dev`, remotes under `github.com/ssaigarimella`

(Exceptions: `px4_msgs` on `release/1.15` from official PX4; `third_party/jetson-inference`
on `master`; both LightGlue checkouts on `main` from upstream `cvg/LightGlue`.)

**Custom packages (the actual project):**

| Package | What | State (2026-06-10) |
|---|---|---|
| `active_exploration` | **Main brain.** C++ frontier detection (FIS), planners, reactive_depth_guard, odom_correction, flight scripts | clean |
| `multi_drone_nvblox` | Multi-drone: loop closure, octomap exchange, swarm alignment, Vicon tooling, ~45 scripts | **DIRTY**: uncommitted WIP `swarm_pose_broadcaster.py` + launch + CMakeLists edit, untracked `config/xrce_agent_loopback.refs` |
| `px4_offboard` | C++ `vio_bridge` + `vio_bridge.launch.py` (starts MicroXRCEAgent — the only place it's launched); test missions | clean |
| `radiohive` | Dual-link (serial radio + zenoh) swarm comms experiment. **Radio hardware NOT attached**; its default serial port `/dev/ttyUSB0` @57600 would steal the PX4 link if ever launched | clean |

**Vendor forks (patches you must know about):**

- `isaac_ros_nvblox` — **heavily modified** (base v3.2-14 + 580 changed files; nvblox core
  vendored in-tree instead of submodule; cuVSLAM/nvblox container split; moving ESDF slice;
  GPU preallocation). Uncommitted: `nvbloxIFLmap1.rviz` tweaks.
- `realsense-ros` — base 4.51.1 + one functional line: IMU publisher queue 5→100.
- `isaac_ros_visual_slam` — rviz configs only.
- `isaac_ros_common` — run_dev.sh tweak (drops PVA GPU device env on aarch64).
- `px4_msgs` — pristine `release/1.15` (official remote).
- `third_party/jetson-inference` — fork with voxel/pixel-mapping WIP commits.
- `third_party/LightGlue` + **duplicate checkout at workspace root `./LightGlue`** — the
  root copy is the pip-editable-installed one the container actually imports. **Both have
  uncommitted patches to `lightglue/__init__.py` that disable the optional extractors
  (ALIKED/DISK/DoGHardNet/SIFT), but the patches DIFFER**: the root copy comments the
  imports out; the src/third_party copy wraps them in try/except. Don't reset either; the
  root copy is the one that matters at runtime.

Git identity: `ssaigarimella` / `sgarimella34@gatech.edu`, SSH remotes via the on-device
ed25519 key. `./check_repos.sh` prints branch+status of each direct child of `src/` —
it does NOT cover the nested `src/third_party/*` repos or the root `./LightGlue` copy.
(Also: `isaac_ros_common` carries untracked in-source `*/build/` dirs from colcon; harmless.)

## Camera tilt: 20° is hardcoded in FOUR places (keep in sync)

| Where | Symbol |
|---|---|
| `src/active_exploration/src/odom_correction_node.cpp:9` | `kCameraPitchDeg = 20.0` (rebuild active_exploration) |
| `src/px4_offboard/include/px4_offboard/vio_bridge_node.hpp:24` | `kCamPitchDeg = 20.0` (C++ bridge actually used; rebuild px4_offboard) |
| `src/px4_offboard/scripts/vio_bridge.py:30` | `CAM_PITCH_DEG = 20.0` (legacy Python bridge) |
| `src/active_exploration/launch/reactive_guard.launch.py` | `camera_tilt_deg` default `20.0` (runtime param) |

`src/active_exploration/CAMERA_TILT.md` only mentions two of these — it's stale.

## DDS configuration mess (root cause of most "topics don't connect" bugs)

This is the #1 historical debugging sink (`docs/ghost-position-hold-fix.md` documents three
layered fixes). Current state:

1. **Multicast-on-loopback fix (ACTIVE, host)**: systemd `dds-multicast-lo.service` →
   `ip route replace 224.0.0.0/4 dev lo` + lo multicast on. Required for same-host FastDDS
   discovery when WiFi is up. Side effect: cross-host DDS multicast discovery is dead by
   design — Zenoh is the only intended cross-drone path.
2. **Fast DDS env-var rename bug (CURRENTLY UNFIXED)**: the container's `MicroXRCEAgent`
   (`/usr/local/bin/MicroXRCEAgent`) links Fast DDS 3.5, which only reads
   `FASTDDS_DEFAULT_PROFILES_FILE`. The launch stack exports only the legacy
   `FASTRTPS_DEFAULT_PROFILES_FILE`, which the agent ignores → with WiFi up the agent can
   bind DDS to the WiFi IP and never discover loopback-pinned publishers
   (symptom: `ros2 topic info -v /fmu/in/vehicle_visual_odometry` shows
   `publisher=1, subscription=0`; PX4 shows `Payload rx: 0`). The documented fix (a wrapper
   at `/usr/local/bin/MicroXRCEAgent` that copies the env var, real binary renamed to
   `.real` — see `docs/ghost-position-hold-fix.md`) **was lost in a container recreate and
   is NOT installed in the current image**. If you reinstall it, `docker commit`.
3. **Three launchers, three DDS configs** (inconsistent, verify which one started the stack):
   - `launch_ghost_local.sh`: no profiles file at all (loopback pinning commented out).
   - `run_ghost.sh`: exports `FASTRTPS_DEFAULT_PROFILES_FILE=.../fastdds_loopback.xml`
     (the legacy name — ignored by Fast DDS 3.x components, honored by Humble's FastDDS 2.x).
   - `flight_autonomous.sh`: neither (and unlike the other two it also doesn't export
     `ROS_LOCALHOST_ONLY=0`).
4. The zenoh bridge runs **CycloneDDS pinned to 127.0.0.1** on the host while container
   nodes use FastDDS with `ROS_LOCALHOST_ONLY=0`. The bridge config's
   `ros_localhost_only: true` is intentional; don't "fix" it.
5. PX4-side restart after agent changes: QGC MAVLink console →
   `uxrce_dds_client stop` then `uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600`
   (or power-cycle).

## Networking reference

| NM profile | SSID / iface | IP | Use |
|---|---|---|---|
| `TP-Link_073A 1` | TP-Link_073A, wlP1p1s0 | static **192.168.0.50/24**, gw .1 | bench WiFi (current) |
| `Wired connection 1` | enP8p1s0 | DHCP, currently **192.168.0.206/24** | **SSH here** |
| (bench laptop NAT) | enP8p1s0 | default route via **192.168.0.136** metric 50 | internet, set by bench-internet.service |
| `eduroam [f90d3298]` | eduroam | was 10.90.185.57/17 (gw 10.90.128.1) | campus; all 10.90.* IPs in docs are stale |
| `IFL` | TP-Link_C624, PSK <IFL-PSK — ask operator> | static **192.168.10.10/24**; Vicon PC 192.168.10.2 | flight lab |
| `iPhone` | hotspot | static 172.20.10.11/28, gw .1 | field fallback |
| 802.11s mesh | `jetson-drone-mesh` ch6 2437MHz | **192.168.77.3** (ghost) | inter-drone, via launch_mesh_zenoh.sh |

Peer eduroam IPs from DEPLOYMENT_GUIDE (delta 10.90.134.66, buckshee 10.90.130.212,
thunderstrike 10.90.194.74) are stale DHCP-era values. All drones: user = hostname,
password `<JETSON_PASS — see credentials.env>`.

Watch out: WiFi and Ethernet are currently on the *same* subnet with **3 default routes**
(metrics 50/100/600) — asymmetric-routing confusion is possible; prefer explicit interfaces
when tcpdumping.

### WiFi mesh kernel patch (why 802.11s works at all)

JetPack 6's Realtek vendor driver can't do mesh. `/mnt/nova_ssd/workspaces/kernel_build/`
rebuilt `mac80211.ko` (CONFIG_MAC80211_MESH=y) and patched in-kernel **rtw88** (adds
MESH_POINT iftype, fixes beacon filtering in `FIF_OTHER_BSS`); vendor rtl8822ce is
blacklisted (`/etc/modprobe.d/blacklist-rtl88x2ce.conf`). Patch is INSTALLED and ACTIVE
(`iw phy0 info` lists "mesh point"). Originals in `kernel_build/backup_modules/`,
`04_rollback.sh` to revert. If a kernel/L4T update ever reverts modules, mesh breaks.

## Clock weirdness (will bite log forensics)

No usable RTC: every boot starts at the 1970 epoch until NTP syncs (which requires the
bench laptop NAT). Consequences:
- Files/dirs dated `Dec 31 1969` / `1970-01-01` (some owned by `nfsnobody`) are real
  artifacts from pre-sync boots — `guard_19700101_*.csv`, `log/build_1970-01-01_*`, docker
  "56 years ago" statuses.
- **Container clock is UTC, host displays America/New_York (UTC-4)**: flight dir names
  (host time) vs node-written CSV/bag names (container UTC) differ by exactly 4h. Match by
  offset, not equality (e.g. `flight_20260322_002824` ↔ `guard_20260322_042909.csv`).
- User sometimes disables NTP for time-sync experiments; `bench-internet.service` re-enables
  it on every boot.

## Where the data is

```
experiment_logs/                      # mode 777, host path; /workspaces/... in container
├── flight_YYYYMMDD_HHMMSS/           # per-flight node logs (cuvslam.log, vio_bridge.log,
│                                     #   fis.log, reactive_guard.log, explorer.log, ...)
├── guard_*.csv                       # reactive_depth_guard telemetry (flat, container-UTC names)
├── splanner_*.csv                    # simple planner telemetry (was explorer_*.csv in old docs)
├── bags/flight_<ts>[_<utc_ts>]/      # rosbag2 MCAP (record_flight.sh; double-stamped
│                                     #   dirs from flight_autonomous.sh, single from manual runs)
├── flight_launch.log                 # stdout of remote flight_autonomous.sh launch
├── flight_stop                       # (transient) stop flag checked by flight_autonomous.sh
└── watchdog.log, ping_test*.log
```
- guard CSV header: `timestamp,min_horiz,travel_dist,blind,state,pos_*,sp_*,heading,hold_latched,pass_streak,altitude`
- splanner CSV header: `timestamp,state,flu_*,ned_*,yaw_*,wp_idx,wp_total,dist_to_wp,n_clusters,tf_valid,armed,offboard,esdf_age,goal_*`
- Headless-run logs: `/tmp/ghost_tab{1..7}.log` **inside the container** (wiped on container
  restart). GUI launcher writes no logs. Zenoh: `/tmp/zenoh_slam.log` (host).
- Last colcon build: `log/latest_build` (Apr 23, px4_offboard, success).
- `isaac_ros_assets/`: NVIDIA quickstart sample bags only (~2.2GB), not project data.

## Health-check cookbook (run over SSH)

```bash
# 1. Container up?
docker ps --format '{{.Names}}\t{{.Status}}'        # expect isaac_ros_realsense Up
docker start isaac_ros_realsense                     # if not

# 2. Camera present?
lsusb | grep -i realsense                            # 8086:0b3a D435i, bus at 5000M
docker exec -u admin isaac_ros_realsense rs-enumerate-devices -s

# 3. PX4 serial link
ls -la /dev/ttyUSB0                                  # crw-rw---- root dialout
sudo -n lsof /dev/ttyUSB0                            # exactly one holder (MicroXRCEAgent) when up
docker exec isaac_ros_realsense bash -c 'pgrep -af MicroXRCEAgent; ss -uanp | grep -i xrce'
#   healthy: sockets on 0.0.0.0/127.0.0.1 only. Bound to a WiFi IP = env-var bug (see DDS #2)

# 4. Topic liveness (template for ALL ros2 CLI)
docker exec -u admin isaac_ros_realsense bash -c \
  "source /opt/ros/humble/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash && \
   export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/src/multi_drone_nvblox/config/fastdds_loopback.xml && \
   timeout 5 ros2 topic hz /visual_slam/tracking/odometry"
#   also: ros2 topic info -v /fmu/in/vehicle_visual_odometry   (check subscription count!)
#         timeout 5 ros2 topic hz /fmu/out/vehicle_status

# 5. DDS plumbing on host
ip route show | grep 224                             # expect 224.0.0.0/4 dev lo
systemctl is-active dds-multicast-lo.service

# 6. DDS packet-level (tcpdump is installed in container, run as root)
docker exec -u root isaac_ros_realsense tcpdump -i lo -n -c 50 'udp and portrange 7400-7600'

# 7. Mesh status (when mesh is up)
iw dev wlP1p1s0 station dump; ping 192.168.77.1; tail /tmp/zenoh_slam.log
```
Performance pre-flight (from history): `sudo nvpmodel -m 0 && sudo jetson_clocks`,
`sudo iw dev wlP1p1s0 set power_save off`, `sudo rfkill block bluetooth`.

## Known broken / surprising things (as of 2026-06-10)

1. **MicroXRCEAgent FASTDDS wrapper missing** (DDS #2 above) — the documented fix is not in
   the current image; the discovery bug is live again whenever loopback pinning is expected.
2. **`jetson_inference` python is NOT installed in the container** (`import jetson_inference`
   fails) despite docs saying it was extracted to `/usr/lib/python3/dist-packages/`. The
   tarballs to restore it sit in the workspace root (`jetson_inference_python.tar.gz`,
   `jetson_native_libs.tar.gz`, `sun_model.tar.gz`). segnet/`test_sun.py`/`test_trt.py`
   won't work until re-extracted (+ `docker commit`).
3. **torchvision not installed** in container (torch 2.5.0 jp6 wheel + CUDA works;
   numpy 1.26.4; lightglue editable from `/workspaces/isaac_ros-dev/LightGlue`).
4. **WiFi restore after mesh/flight targets the wrong NM profile name** (`eduroam` vs actual
   `eduroam [f90d3298]`, and the bench network is TP-Link anyway). `eduroam-restore.service`
   (enabled, runs at boot) has the same mismatch. Manual: `sudo nmcli connection up "TP-Link_073A 1"`.
5. **DEPLOYMENT_GUIDE.md staleness**: says D455 (actual: D435i), says MAVROS (actual:
   uXRCE-DDS), 7-tab drone1 layout (actual: 9-tab drone3), eduroam IPs stale.
   `docs/flight-debug-pipeline.md`: old IFL SSID (`TP-LINK_8DDA`) and `/tmp/flight_stop`
   (actual: `experiment_logs/flight_stop`) — `flight_autonomous.sh` itself is authoritative.
6. **`launch_ghost_jetson_eduroam_python_frontierexplore.sh`** is the laptop-side launcher
   (SSH from laptop), not executable (`bash` it), has stale eduroam IP `10.90.228.175` and a
   drone1/drone3 namespace inconsistency in tabs 6/7. Prefer the local/headless launchers.
7. **radiohive** defaults to the PX4's serial port — never launch it on ghost without
   overriding `serial_port` (radio hardware isn't attached anyway).
8. Plaintext credentials all over: `<JETSON_PASS — see credentials.env>` (SSH/sudo, all drones), WiFi PSKs in scripts.
   Known and accepted by the user; don't propagate further.
9. `build_active_exploration.sh` **removes the container** when done (see §E).
10. Vendor-doc gotchas inside container (from DEPLOYMENT_GUIDE, still true): torch must come
    from the jetson-ai-lab jp6/cu126 index; no pip `nvidia-*-cu12` packages; `numpy<=1.26.4`;
    BoW vocab pickle must stay protocol-4.
11. On 2026-06-10 the audit behind this file **started the previously-stopped container and
    left it running** (no ROS processes started).

## Authoritative-docs map (newest wins)

| Topic | Read this | Not this |
|---|---|---|
| Launch procedure | `launch_ghost_local.sh` / `run_ghost.sh` (Apr 23) | DEPLOYMENT_GUIDE §tabs |
| Flight ops | `src/active_exploration/scripts/flight_autonomous.sh` (Mar 23) | docs/flight-debug-pipeline.md details |
| VIO→PX4 debugging | `docs/ghost-position-hold-fix.md` (Apr 23) + "DDS mess" above | — |
| Multi-drone internals | `src/multi_drone_nvblox/{PIPELINE,AGENTS,COMMS_PROTOCOL}.md` | — |
| PX4 params | `src/px4_offboard/VIO_QUICKSTART.txt` | — |
| Camera tilt | "Camera tilt" section above | CAMERA_TILT.md (incomplete) |
| Mesh kernel | `kernel_build/KERNEL_MESH_PATCH_GUIDE.md` | — |
