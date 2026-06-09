# run02 — DDS domain mismatch confirm + fix validation (wifi ON the whole time)

## Claim (H-DOM)
The bug is ROS_DOMAIN_ID=3 in the ghost tmux script vs PX4's UXRCE_DDS_DOM_ID=0.
The WiFi radio is irrelevant (ruled out by run01/A: full pipeline healthy with
radio on; only the PX4<->ROS link dead).

## Mechanism
MicroXRCEAgent creates DDS participants on the domain the PX4 client requests
(UXRCE_DDS_DOM_ID, default 0), ignoring the container's ROS_DOMAIN_ID. With the
container nodes on domain 3: vio_bridge's /fmu/in/vehicle_visual_odometry never
reaches PX4, and /fmu/out/* never appears on domain 3 => EKF2 gets no vision,
no POSITION mode.

## Test (one variable: ROS_DOMAIN_ID; radio stays ON throughout)
- Phase 1 (confirm): launch tab1+tab2 EXACTLY as the tmux script (domain 3).
  Then from a domain-0 shell in the same container, list topics.
- Phase 2 (validate fix): hard-kill, relaunch identical except NO ROS_DOMAIN_ID
  (=> domain 0). Capture goal metrics.

## Predicted signatures
1. Phase 1: `ros2 topic list` on domain 0 SHOWS /fmu/out/* (and vehicle_status
   echoes), while domain 3 does not — direct proof of the mismatch.
   If domain 0 also shows nothing → H-DOM wrong; check PX4 param/QGC next.
2. Phase 2 (domain 0, wifi ON): /fmu/out/* present, /fmu/in/vehicle_visual_odometry
   ~30 Hz, vehicle_local_position xy_valid=true z_valid=true. That meets every
   GOAL criterion except the human QGC POSITION-mode check.

## Variable changed vs run01
ROS_DOMAIN_ID for the relaunched stack (3 -> unset/0). No rfkill in this run.
