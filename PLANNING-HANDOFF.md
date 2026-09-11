# Planning and coordination handoff — 2026-09-11

This branch connects the selected HERCULES planner and coordination changes to the current field launch scripts. It is an integration branch for Sandilya, not a deployment or flight-qualification result.

Use the matching `lucas/planning-coordination-handoff-2026-09-11` branch in all three repositories:

- [active_exploration](https://github.com/ssaigarimella/active_exploration/tree/lucas/planning-coordination-handoff-2026-09-11): planner/FIS and planning launch arguments.
- [multi_drone_nvblox](https://github.com/ssaigarimella/multi_drone_nvblox/tree/lucas/planning-coordination-handoff-2026-09-11): matching ClaimIntent/EsdfGrid3D interfaces, coordination and shared FIS/planner stages.
- [multirobot_launch_scripts](https://github.com/ssaigarimella/multirobot_launch_scripts/tree/lucas/planning-coordination-handoff-2026-09-11): this operator-launch integration.

The launch branch starts at field `main` commit `acc87da8ba8190708d03d63d02efd4d040259332`. It does not import the simulator branch's 51 commits, AirSim bridge, render/video tooling, PX4/VIO changes, camera calibration or GPU tuning. Existing operator arming/offboard responsibilities, vehicle IDs, network addresses, sensing/guard commands and trusted-prior defaults are retained. Actual deployed revisions still need to be checked before merging.

## What changes here

GUI tab 3 now uses `multi_drone_nvblox/fis_stage.launch.py`. It computes the same field team bounds used by planner tab 5, rather than using FIS's unrelated default box. Both tabs obtain their commands from tested, non-actuating `fleet_ctl` functions and pass the real field ID and alignment YAML.

Two explicit options are shared by direct fleet bringup, GUI and headless use:

| Option | Effect | Default without the option |
|---|---|---|
| `--shared-frontiers` | Select `/nvblox_shared_node/get_esdf_and_gradient` for frontier extraction | Existing self ESDF |
| `--planner-odom=TOPIC` | Set the planner's odometry input | Existing visual-SLAM tracking odometry |

`--ekf2` retains its existing meaning for the perception launch. It does not implicitly choose a different planner odometry stream or certify that the streams share a frame. `--debug` remains an explicit arm-check bypass. Selecting shared frontiers presumes the lab has supplied a valid shared mapper and alignment; it does not create alignment or fix map-revision behavior.

The ROS stages expose the planner's opt-in cruise/feedforward, scoring, blacklist, relocation and allocation parameters. They preserve existing field launch behavior unless explicitly selected. The simulation's numeric profile is not automatically applied to the fleet.

For local inspection without credentials or network activity, this command only prints the planned FIS command:

```bash
./fleet_ctl planning-tab-command ghost 3 --shared-frontiers
```

Tab `5` prints the corresponding planner command. Other `fleet_ctl` commands can contact robots; they were not executed during preparation of this branch.

## Validation and remaining scope

Nine pure command-builder tests cover field IDs/defaults, matching GUI/headless options, team-stage selection, explicit debug behavior and rejection of shell-active/invalid odometry names or malformed new options. Parent GUI validation runs before opening hardware tabs. The tests prohibit credential reads and subprocess execution. `bash -n launch_fleet_tmux.sh` and `git diff --check` also pass. See the planner branch's handoff document for the coupled source-build and interface validation results.

The full stack still needs target-Orin builds, matching deployed PX4 messages, real-sensor/frame checks and validation through the lab's bench/flight process. Shared-map geometry correction after alignment revisions, a real alignment producer, and WiFi-only peer coordination remain separate work. These branches do not import the audited VIO pitch regression, but they do not certify inherited estimator/guard behavior.

The videos demonstrate simulated behavior. The historical 92.3% reachable-coverage claim had a set/denominator error, and retained telemetry does not establish zero contacts. These claims are not used to qualify this branch.

No robot, simulator, ROS node, deployment or message to Sandilya was started by this handoff work.
