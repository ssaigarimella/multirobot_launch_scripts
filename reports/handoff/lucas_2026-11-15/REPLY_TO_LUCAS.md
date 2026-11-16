Subject: Re: Simulation state and pipeline coverage, detailed write-up

Hi Lucas,

Thanks, this is thorough and the map-fusion diagnosis matches what the vehicles do. Two things first because they change where your time should go, then your seven questions, then what to run next.


1. The 36 and 39 percent were wrong. The field fuses vision 97 to 98 percent of Offboard time, like your sim.

Those two numbers were the share of logged estimator_status_flags samples with cs_ev_pos set. That topic is logged mostly on change, and the logger dropped about half of everything in both flights (51 and 46 percent), so a share of samples is not a share of time. Time-weighted, from the same two logs:

                                         log_70     log_71
  vision position fused, Offboard time    97 %       98 %
  vision position fused, in-air time      88 %       83 %
  share of samples (the old figure)       36 %       39 %
  last vision message in the log          173 s      303 s
  log ends                                191 s      353 s

In both flights fusion is on from before take-off until vision messages stop arriving at the very end. Nothing is logged on vehicle_visual_odometry after 173 s and 303 s while every other topic keeps logging; PX4 leaves Offboard about 4 s later and the vehicle is flown down without vision. There is no loss at cruise height. The quality field is 100 in every message and the position innovation test ratio never exceeds 0.07 while messages arrive. So to your question: the dropouts do not cluster at low height or at take-off. The fused share below 0.8 m is low (18 to 36 percent) only because that is the descent at the end, after vision has stopped. I do not yet know why the messages stop there.

There is no sim-to-field gap to explain. Plots attached (log_70_ev_fusion.png, log_71_ev_fusion.png); they are the plot you asked for.


2. Two vehicles flew together on Nov 13, and the flight shows things your sim cannot see yet.

Vehicles 1 and 3, 5 min 42 s both in Offboard, outdoor grass, 35 x 35 m fence, 4.0 m apart on the launch line. Full write-up attached (FLIGHT-ANALYSIS-2026-11-13.md). What matters for the sim:

a. The map frame is tilted by how the vehicle rests on the ground. The mapper levels cuVSLAM's frame with a fixed 20 deg, but the frame is the camera's attitude at start. Measured camera pitch at start: 24.3 deg (vehicle 1) and 28.7 deg (vehicle 3) in that flight; 14.3 and 27.5 deg in the flight before it. Consequences, all measured: the mapped ground climbs 0.076 and 0.152 m per metre forward; the planner's 2D slice cuts into the ground about 14 m ahead of vehicle 1 and 8 m ahead of vehicle 3 and reads it as occupied; ground already mapped drops out of the slice as the vehicle moves along x (the slice is taken at the camera's height in that frame). Your vehicles spawn level, so none of this appears in sim.

b. The fleet still flies vio_bridge from px4_offboard dev 87c25c5, without your 7ac8ad7. PX4's forward position is short by cos(pitch + 20 deg): 0.72 on vehicle 1 and about 0.65 on vehicle 3. The peer poses sent over LoRa are PX4's, so they carry the same error. Sideways everything agrees.

c. Exploration stalled. The team's 3D map covered 78 percent of the fence after 2 min 45 s and 86 percent at the end. Median hop to the next goal 2.8 m (vehicle 1) and 2.2 m (vehicle 3); 58 and 88 percent of goals within 2.5 m of a fence edge; 40 and 60 percent on ground that vehicle had mapped more than 20 s earlier. Causes found in the code: score = gain / (distance + 1 m), so the nearest frontier always wins; isFrontierCell returns true for a free cell on the box boundary (frontier_info_structure_node.cpp line 238), so the fence edge is a frontier forever; the lane-sweep fallback never fires because its obstacle density counts unknown cells (simple_exploration_planner.py line 1179, grid < 0.5 with unknown at -1000). The coverage mode stayed 0 for the whole flight.

d. Rates today: IR and depth at 60 Hz at the sensor with the projector alternating, so cuVSLAM gets stereo at 30 Hz, PX4 gets vision at 30 Hz, the mapper gets depth at 30 Hz; colour 15 Hz.

e. The code that flew is two commits ahead of what you have in two repos: active_exploration 37c7d32 (ef3a6f3 planner: 20 Hz control loop, margin-clean plans, PX4-feed staleness, escape; 37c7d32 guard heartbeat) and multi_drone_nvblox 3647995 (ec67e7d, 3647995). They are not on GitHub yet; the four patches are attached. px4_offboard is dev 87c25c5, which you have.


Your questions

1. Logs. log_70 is 54 MB and log_71 is 111 MB, which is why the mail dropped them. I will share both by link. You do not need to wait: every parameter you listed is in field_parameters_for_sim.md (attached), with full dumps of both logs beside it. No parameter changed during either flight and the two logs differ only in accelerometer calibration and flight counters. The ones you named: EVP_NOISE 0.1, EVV_NOISE 0.1, EVA_NOISE 0.05, EV_NOISE_MD 0, EVP_GATE 5, EVV_GATE 3; RNG_NOISE 0.1, RNG_POS_Z 0, RNG_A_HMAX 7; MAG_TYPE 5 (no magnetometer fused; cs_mag is 0 in air), MAG_DECL -5.43; GPS_DELAY 110, REQ_GPS_H 10; MPC_ACC_HOR_MAX 2, MPC_JERK_AUTO 2, MPC_XY_P 2, MPC_Z_P 1.45, MPC_XY_ERR_MAX 1.5, MPC_YAWRAUTO_MAX 20, MPC_YAWRAUTO_ACC 10, MPC_Z_V_AUTO_UP 1.5, MPC_LAND_SPEED 0.6, MPC_LAND_ALT1/2/3 1.0, MPC_LAND_CRWL 0.3; COM_ARM_EKF_VEL 0.5, COM_OF_LOSS_T 1, COM_OBL_RC_ACT 0; SENS_BOARD_X_OFF -1.80, SENS_BOARD_Y_OFF -0.18. The AGP parameters do not exist in this firmware (v1.15.4). Three things to know. The vehicles fly EKF2_EV_CTRL 11 and EV_QMIN 0, and that stays: use 11, not 9. The height reference is the downward rangefinder, not vision and not the barometer (EKF2_HGT_REF 2, EKF2_RNG_CTRL 2, EKF2_BARO_CTRL 0, MPC_ALT_MODE 2), so the vehicle holds its height above the ground and vision height is a secondary source; this is the part your field set must match. And EKF2_GPS_CTRL is 15 but GPS was never fused (cs_gps 0 in air), so your GPS_CTRL 0 is the right choice.

2. Frontier scoring uses the vehicle's own nvblox ESDF. Confirmed in flight: tab 3 passes no esdf_service_name. Study C as you ran it is right. The shared mapper comes back as an arm below.

3. No date for the measured PDR against RSSI curve. It needs a lab session with all four radios that is not scheduled. Table 11 stays provisional. The one field point I have: on Nov 13, delivery was 99.7 and 99.8 percent over RSSI from -86 to -46 dBm (7,880 packets, SF7, 500 kHz, up to 35 m apart).

4. Entry point: the fleet console, which starts each tab through launch_fleet_tmux.sh, so trust prior is on. fleet_ctl start is not used for flights. The Nov 13 logs confirm it: vehicle 1 integrated 466 of vehicle 3's keyframes with source prior, vehicle 3 integrated 422 of vehicle 1's with source lc, none gated. That is your reading of the code exactly.

5. The 15 Hz describes log_70 and log_71, where vision reached PX4 at 15 to 17 Hz. The fleet now runs 30 Hz (2d). Keep v3 as it is and state 15 Hz. Please add a sensitivity block at 30 Hz stereo and depth, same shape as your EV_CTRL 11 block (POSTFUSION, baseline and drift, n = 10), and run anything new at 30 Hz.

6. A.3 r07: handled correctly. Keep it in the table, flagged as contact induced, with the cell reported with and without it as you did.

7. Study C step (b): do not run the room. With 2.4 square metres at the field clearance it would test the room, not the stack, and changing the planner thresholds defeats the purpose. Report (a) as it stands: dense vegetation is outside the field envelope, 11 of 35 flights had contact, and the flagged trial stays flagged. The next step is the field replica below.


ekf2_odom_tf

Thank you for this. On the vehicles that node is not in the flown pipeline: the fleet maps from cuVSLAM odometry levelled by odom_correction, and the EKF2 path exists only on one vehicle's branch and has never flown. So the field maps are not affected by it, but I will check the orientation against cuVSLAM's in the slide test the way you describe. I am about to replace the fixed 20 deg with an angle measured from gravity at start in odom_correction, vio_bridge and ekf2_odom_tf together. Could you push lucas/jfr-sim-studies-2026-09-22, or send the ekf2_odom_tf change as a patch? I want to start from your level_odom and lever-arm parameters and not write a second version.


What to run next, in this order

A. Re-read your existing Study C v3 runs for the stall in 2c. No new flights. Per vehicle: the share of goals within 2.5 m of the exploration box edge, the median hop to the next goal, the share of goals on ground that vehicle had mapped more than 20 s earlier, and whether the coverage mode ever left 0. If the sim shows the same pattern, the explored fractions in Study C (0.47, 0.87, 0.81) are partly the planner and not the grove.

B. Field replica. This replaces step (b).
   - Ground: flat, open, textured like grass, nothing inside the fence.
   - Fence: the field one, 35 m forward by 35 m wide from the launch line, 0 m behind.
   - Vehicles: N = 2, IDs 1 and 3, vehicle 3 placed 4.0 m to the right of vehicle 1, both facing forward; the alignment yaml as in the field (range 4.0 m, bearing 90 deg). Then N = 4 at 2, 4 and 6 m.
   - 360 s in Offboard (the field flight length), 30 Hz stereo and depth, 5 seeds per cell.
   - Code as flown: the attached patches, frontier ESDF from the own mapper, and vio_bridge from dev 87c25c5 (not your corrected one), so that PX4 sees what it sees on the vehicles.
   - Two start attitudes: level (camera 20 deg down), and tilted as measured (camera 24.3 deg down on vehicle 1 and 28.7 deg on vehicle 3 when cuVSLAM starts, by whatever means gives its first frame that pitch).
   - Report, with the same definitions as 2c: share of the fence under the team's 3D map against time, median hop, share of goals within 2.5 m of a fence edge, share of goals on ground already mapped by that vehicle, coverage mode, and for the tilted start how far ahead the planning slice has occupied cells.
   The sim reproduces the field if the tilted start gives occupied cells about 14 m and 8 m ahead, coverage that stalls near 80 percent, and goals strung along the fence.

C. The fixes, one at a time on the tilted start of B, each against the as-flown arm: gravity levelling at start; fence edge not a frontier; lane-sweep density excluding unknown cells; frontiers from the shared mapper (esdf_service_name:=/nvblox_shared_node/get_esdf_and_gradient). I will send these as patches. The second and third are one-line changes at the lines in 2c if you want to prototype them first.

D. The 30 Hz sensitivity block from question 5.

Study D and the visual re-render stay on hold until B is in.

Attached: field_parameters_for_sim.md, log_70_params.txt, log_71_params.txt, log_70_ev_fusion.png, log_71_ev_fusion.png, FLIGHT-ANALYSIS-2026-11-13.md, where_the_goals_went.png, planning_map_vs_3d_map.png, own_vs_fused_map.png, and four patches.

Sandilya
