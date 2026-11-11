# Prepared prompts (not run). Paste one at a time when the drones are on the bench.

## 0. First, every time the drones are powered: pull the ULogs

With the flight controllers on laptop USB and QGroundControl closed:

    cd ~/multi-robot-coordination && python3 tools/pull_ulogs.py --last 8

It saves the newest 8 logs from each flight controller under
`px4_logs/pulled_<date>/<usb-serial>/`. It only reads; it erases nothing. The tool is new and
untested on a vehicle. Fallback if it fails: QGroundControl, Analyze Tools, Log Download.

---

## 1. Set EKF2_EV_CTRL to 9 on all four (after the log pull, before any test)

Every Offboard flight log on the laptop has `EKF2_EV_CTRL` 11 (vision horizontal position,
vertical position and yaw) with `EKF2_HGT_REF` 2 (rangefinder). The draft now states 9 (no
vision height). For each vehicle, with its flight controller on USB, in QGroundControl's
MAVLink Console:

    param show EKF2_EV_CTRL        # write down the value found (expected 11)
    param set EKF2_EV_CTRL 9
    param show EKF2_EV_CTRL        # must read 9
    param show EKF2_HGT_REF        # must read 2

Reboot the flight controller and run `param show EKF2_EV_CTRL` again to confirm it kept 9.
Then save the vehicle's parameters (QGroundControl, Parameters, Tools, Save to file) as
`px4_params/<vehicle>_<date>.params` in the lab repo. There is no single fleet parameter
file; one export per vehicle is the record. Do this on delta, buckshee, ghost and
thunderstrike. This is a PX4 parameter change, so it is Sandilya's to make or approve.

---

## A. Slide test for the vision tilt sign (one drone, disarmed, about 5 minutes)

```
Run the vision tilt-sign check on ONE drone: <drone name>. Single concern: measure, do not fix.

Setup: drone on the bench, props off, flight controller powered, NOT armed. Bring up only
tabs 1 and 2 on that drone (camera, cuVSLAM, nvblox with odom_correction; and vio_bridge with
the PX4 link). Wait for a clean cuVSLAM start before continuing.

Test 1, the existing check:
    ./experiments/rotation_check.sh <drone>
I will lift the drone level and carry it straight FORWARD 2 to 3 m at constant height, stop,
and press Ctrl-C. Report the printed verdict and the two ratios (vertical over horizontal
displacement) for the map path and the PX4 path.
    Map path about 0.00 and PX4 path about 0.84  -> vio_bridge sign is wrong (expected)
    PX4 path about 0.00 and map path about 0.84  -> odom_correction sign is wrong
    Raw about 0.00 and both corrected about 0.36 -> cuVSLAM is already gravity-aligned

Test 2, the 1 m slide: mark 1.00 m on the bench with a tape. Record
/fmu/in/vehicle_visual_odometry position before and after I slide the drone 1.00 m straight
forward on the bench without lifting it. Report forward and vertical displacement.
    PASS (sign correct): forward 0.95 to 1.05 m, vertical under 0.05 m
    FAIL (40 degrees off): forward about 0.77 m, vertical about 0.64 m
Repeat the slide once to confirm.

Do not change vio_bridge, ekf2_odom_tf, any parameter, or any other file. Stop the two tabs
when done, save the transcript under experiments/results/, and report the numbers. If the
result is FAIL, describe the paired change (vio_bridge to +20 degrees, ekf2_odom_tf inverse to
-20 degrees, both together) and wait for my go before touching anything.
```

---

## B. LoRa v7.5: flash all four, reboot test, transmit-power confirm

```
Flash LoRa firmware v7.5 on all four radios and test it. Single concern: the radios.

Preconditions: all four drones on the bench and reachable, stacks DOWN on all four (the
bridge holds the radio port). Confirm both before starting. Never leave the fleet on mixed
firmware: if any radio fails to flash, stop and tell me before doing anything else.

1. Record what is on the radios now (version check), then flash:
       ./flash_lora_firmware.sh --check all
       ./flash_lora_firmware.sh all
   Confirm each radio's flashed image matches the build (hash check) and report per drone.

2. Reboot test, reference radio (delta, node 0), then a non-reference radio:
       ./tests/lora_bench_reboot.py --trials 15 --wait 20 --reboot delta
       ./tests/lora_bench_reboot.py --trials 15 --wait 20 --reboot ghost
   PASS: the mesh is FULL (every radio lists every other radio) in 15 of 15 trials of each.
   Report the count and the slowest recovery. For reference, v7.4 passed 12 of 12 on the
   bench and v7.3 failed 9 of 12.

3. Cold start: power-cycle or reset all four radios together 10 times; PASS is FULL mesh
   within 20 s every time.

4. One-minute transmit-power confirm on one radio: with the LoRa bridge running on that
   drone, command 14 dBm on lora/set_tx_power and read lora/tx_power; PASS is the radio
   reporting 14. Command 10 dBm again and confirm it reports 10 (the flight setting).

5. Leave all four radios on v7.5 at 10 dBm, stacks down. Report results per step, then
   commit nothing until I say so.

Bench limits to state in the report: all four radios are in range of each other, and at
close range a stronger signal can get through a collision, so the bench cannot show the
chain case; that stays simulation-only until a test at distance.
```
