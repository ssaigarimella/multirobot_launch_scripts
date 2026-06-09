# run03 — validate permanent fix: PX4 UXRCE_DDS_DOM_ID=3 (human sets in QGC)

## Claim
With PX4's UXRCE_DDS_DOM_ID set to 3 (matching the tmux script's ROS_DOMAIN_ID=3)
and the FMU rebooted, the UNMODIFIED launch_ghost_jetson_eduroam_python_frontierexplore_tmux.sh
tab1+tab2 stack meets every GOAL criterion with WiFi ON.

## Mechanism
The agent creates PX4's DDS entities on the domain the client requests. Once the
client requests domain 3, PX4's /fmu/out/* and its /fmu/in/* subscriptions land on
the same domain as cuVSLAM + vio_bridge => EKF2 receives external vision.

## Predicted signatures (domain 3, wifi ON)
1. /fmu/out/* present in `ros2 topic list`.
2. /fmu/in/vehicle_visual_odometry ~30 Hz.
3. vehicle_local_position xy_valid=true, z_valid=true.
4. Human can select POSITION mode in QGC.
If 1 fails: check the param was saved AND the FMU was rebooted (param is read at
uxrce_dds_client startup). If still failing, re-examine with run02 phase-1 style
domain sweep (0..4).

## Variable changed vs run02
PX4 param UXRCE_DDS_DOM_ID 0 -> 3 (set by human in QGC + FMU reboot). Stack env is
the launch script's original (ROS_DOMAIN_ID=3) — no script change.
