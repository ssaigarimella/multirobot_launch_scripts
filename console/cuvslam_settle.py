#!/usr/bin/env python3
"""cuvslam_settle.py - reset cuVSLAM once the camera stream has settled.

Runs INSIDE the container, fed on stdin by the fleet console right after it
starts tab 1:

    python3 -u - cuvslam_settle [wait_s]      (the marker lets pkill find it)

Why (measured on delta and ghost, outdoors, 2026-11-13): the tracker
initialises on the camera's very first frames. In daylight those are
overexposed (auto-exposure needs about 0.5 s; 94-98 % of pixels saturated),
and the first ~20 IMU samples carry stamps about 11 s old. The tracker takes
a random kick from that: 0.03 m to 20 m with the vehicle standing still,
7 of 12 starts above 0.3 m. A /visual_slam/reset once the stream has settled
puts the pose at the origin exactly and it stays (8 of 8, including from a
pose 14 m off). A reset does not clean the map, and the mapper integrates
whatever depth it gets at the kicked pose (a reset 1 s after start left ground
voxels 4-6 m behind delta after a 7.7 m kick). So the helper resets twice: at
once on the first odometry message, while the depth images are still empty,
and again when the stream has settled. "kick" is the largest excursion between
the two, i.e. what the mapper could have seen; the console restarts tab 1 when
that is large.

Prints one JSON line: {"odom": bool, "kick": m, "resets": n, "after": m, "ok": bool}
"""
import json
import sys
import time

import rclpy
from isaac_ros_visual_slam_interfaces.srv import Reset
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

WAIT_S = float(sys.argv[2]) if len(sys.argv) > 2 else 75.0
SETTLE_S = 0.8       # after the first odometry message: exposure + IMU stamps settled
WATCH_S = 2.5        # how long the pose must hold after a reset
TOL_M = 0.10         # |x|+|y|+|z| allowed while holding
MAX_RESETS = 4


def main():
    rclpy.init()
    n = Node('cuvslam_settle')
    s = dict(first=None, mx=0.0)

    def cb(m):
        if s['first'] is None:
            s['first'] = time.monotonic()
        p = m.pose.pose.position
        s['mx'] = max(s['mx'], abs(p.x) + abs(p.y) + abs(p.z))

    n.create_subscription(Odometry, '/visual_slam/tracking/odometry', cb, qos_profile_sensor_data)
    cli = n.create_client(Reset, '/visual_slam/reset')

    def spin(sec):
        t = time.monotonic()
        while time.monotonic() - t < sec:
            rclpy.spin_once(n, timeout_sec=0.02)

    t0 = time.monotonic()
    while s['first'] is None and time.monotonic() - t0 < WAIT_S:
        rclpy.spin_once(n, timeout_sec=0.05)
    if s['first'] is None:
        print(json.dumps(dict(odom=False)), flush=True)
        return
    # Reset at once, without waiting: the kick comes in the first 0.2 s, while
    # the infrared frames are still overexposed and the depth images therefore
    # empty, so a reset now keeps the mapper from integrating depth at the
    # kicked pose. The stale IMU samples are already behind us by this point.
    early = None
    if cli.wait_for_service(timeout_sec=2.0):
        early = cli.call_async(Reset.Request())
        spin(0.12)               # the reset lands; poses from before it drain
        s['mx'] = 0.0
    spin(SETTLE_S)
    # what is left here is what the mapper saw between the two resets
    out = dict(odom=True, kick=round(s['mx'], 2), early=bool(early and early.done() and early.result()
                                                             and early.result().success),
               resets=0, after=None, ok=False)
    for k in range(MAX_RESETS):
        if not cli.wait_for_service(timeout_sec=5.0):
            out['error'] = 'no reset service'
            break
        f = cli.call_async(Reset.Request())
        rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
        if f.result() is None or not f.result().success:
            out['error'] = 'reset refused'
            break
        out['resets'] = k + 1
        spin(0.4)                # messages from before the reset drain
        s['mx'] = 0.0
        spin(WATCH_S)
        out['after'] = round(s['mx'], 2)
        if s['mx'] <= TOL_M:
            out['ok'] = True
            break
    print(json.dumps(out), flush=True)


if __name__ == '__main__':
    main()
