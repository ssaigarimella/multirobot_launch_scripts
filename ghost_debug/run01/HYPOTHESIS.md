# run01 — A/B baseline capture (no experimental change)

## Claim
The broken (wifi radio ON) and working (radio OFF) states differ in a measurable,
physical/driver-level signature — most likely RealSense IMU delivery — and NOT in
ROS/DDS configuration. This run captures both states in one offline trip to
discriminate H-A1..H-A4 / H-B / H-C / H-D.

## Mechanism under test
A single on-Jetson script (survives radio-off because it runs under nohup/setsid,
not over the SSH channel) launches the exact tab1+tab2 stack, captures a full
diagnostic bundle with wifi associated to TP-Link_073A, kills the stack,
`rfkill block wifi`, relaunches, captures the working bundle, then unblocks and
reassociates so the laptop can scp both bundles back.

## Predicted signatures (judge strictly against capture)
1. **A (wifi on):** `/camera0/imu` hz far below nominal (~200 Hz expected) or zero;
   tab1 pane shows `Lost IMU msgs ... drop ratio` warnings; `/fmu/in/vehicle_visual_odometry`
   absent or << 30 Hz. **B (radio off):** `/camera0/imu` ≈ nominal, VVO ≈ 30 Hz,
   `xy_valid/z_valid: true` in vehicle_local_position. If A and B do NOT differ on
   /camera0/imu hz but differ downstream, the IMU-starvation theory is wrong → look at
   cuVSLAM/bridge/agent stages instead.
2. **H-A1:** `iw dev wlP1p1s0 get power_save` = on in A. (Necessary but not sufficient.)
3. **H-A2:** interrupts_t0→t1 deltas show wifi IRQ and xhci IRQ hammering the same CPU
   in A; xhci IRQ rate collapses or error counts rise vs B.
4. **H-A3:** `iw link` freq in 2.4 GHz band (2412–2472); dmesg shows uvcvideo/usb
   resets or EPROTO/babble errors only in A; scan shows whether a 5 GHz 073A BSS exists.
5. **H-B:** journal_nm shows periodic connectivity checks/scans correlated with the
   broken window.
6. **H-C:** ss_tulpn shows agent/bridge sockets bound to wlan address (expected: none —
   serial link).

## Variable changed vs previous run
None — this is the baseline.
