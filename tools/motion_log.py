#!/usr/bin/env python3
"""motion_log.py - 5 Hz CSV of what the PX4 bridge sends against what PX4 holds, for a hand check
(lift a vehicle, move it forward and sideways, put it down) or a flight. Runs in the container:

    python3 motion_log.py <out.csv> [seconds]

Columns: wall time; raw tracker position (x fwd, y left, z up in the tracker's start frame); levelled position
(same axes, gravity-levelled); what the bridge sends to PX4 (north, east, down); PX4's own position (north, east,
down), heading and height above ground; number of frontier viewpoints.
"""
import csv, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition

out, secs = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 1200.0
rclpy.init(); n = Node('motion_log'); D = {}
be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=5)
n.create_subscription(Odometry, '/visual_slam/tracking/odometry', lambda m: D.__setitem__('raw', m.pose.pose.position), qos_profile_sensor_data)
n.create_subscription(Odometry, '/visual_slam/tracking/odometry_level', lambda m: D.__setitem__('lev', m.pose.pose.position), be)
n.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', lambda m: D.__setitem__('ev', m.position), be)
n.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', lambda m: D.__setitem__('lp', m), be)
n.create_subscription(PoseArray, '/fis/frontier_viewpoints', lambda m: D.__setitem__('vp', len(m.poses)), 10)
f = open(out, 'w', newline=''); w = csv.writer(f)
w.writerow('t raw_x raw_y raw_z lev_x lev_y lev_z ev_n ev_e ev_d px4_n px4_e px4_d px4_heading_deg dist_bottom viewpoints'.split())
t0 = time.time(); nxt = t0
while time.time() - t0 < secs:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() >= nxt:
        nxt += 0.2; r, l, e, p = D.get('raw'), D.get('lev'), D.get('ev'), D.get('lp')
        w.writerow(['%.2f' % time.time()] + (['%.3f' % v for v in (r.x, r.y, r.z)] if r else [''] * 3) + (['%.3f' % v for v in (l.x, l.y, l.z)] if l else [''] * 3) +
                   (['%.3f' % float(v) for v in e] if e is not None else [''] * 3) +
                   (['%.3f' % p.x, '%.3f' % p.y, '%.3f' % p.z, '%.1f' % math.degrees(p.heading), '%.2f' % p.dist_bottom] if p else [''] * 5) + [D.get('vp', '')])
        f.flush()
