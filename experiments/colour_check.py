#!/usr/bin/env python3
"""colour_check.py - why does nvblox's colour map stop growing in flight?
(field report section 11.1: colour markers empty while occupancy keeps growing)

Runs inside a drone's container next to the full stack and writes one CSV row
per second to <out>/colour_check.csv:

  color_*  colour camera: frames/s, header-stamp lag, longest gap, dropped
           frames (metadata frame counter), exposure
  depth_*  depth camera: frames/s, stamp lag
  cam_*    camera position in nvblox's levelled odom frame (R_y(+20) * cuVSLAM)
  cmark_*  /nvblox_node/color_layer_marker: messages, empty ones, voxels, and
           voxel distance from the camera (min / median / max)
  tmark_*  /nvblox_node/tsdf_layer_marker: the same. It goes through the same
           7 m streaming filter as the colour marker, so it is the control:
           colour empty + TSDF full = colour integration stalled;
           both empty = nothing within 7 m of the camera.
  occ_*    /nvblox_node/static_occupancy_grid: messages, known cells

    python3 colour_check.py --out DIR [--seconds 900]
Started and stopped by colour_check.sh; Ctrl-C also ends it cleanly.
"""
import argparse
import csv
import json
import math
import os
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker

LEVEL_DEG = 20.0          # odom_correction: nvblox odom = R_y(+20 deg) * raw cuVSLAM
COLS = ['t_unix', 't_utc',
        'color_hz', 'color_lag_ms', 'color_gap_ms', 'color_drops', 'color_exposure',
        'depth_hz', 'depth_lag_ms', 'odom_hz', 'cam_x', 'cam_y', 'cam_z',
        'cmark_msgs', 'cmark_empty', 'cmark_pts', 'cmark_dmin', 'cmark_dmed', 'cmark_dmax',
        'tmark_msgs', 'tmark_empty', 'tmark_pts', 'tmark_dmin', 'tmark_dmed', 'tmark_dmax',
        'occ_msgs', 'occ_known']


def marker_points(buf):
    """(type, Nx3 points) of a serialized visualization_msgs/Marker, read
    straight from the CDR bytes so a 50k-voxel marker costs no Python objects."""
    e = '<' if buf[1] == 1 else '>'

    def al(o, n):                       # CDR aligns relative to the 4-byte header
        return o + (-(o - 4)) % n

    def u32(o):
        o = al(o, 4)
        return struct.unpack_from(e + 'I', buf, o)[0], o + 4
    o = al(4, 4) + 8                    # header.stamp
    n, o = u32(o)
    o += n                              # header.frame_id
    n, o = u32(o)
    o += n                              # ns
    o = al(o, 4)
    mtype = struct.unpack_from(e + 'i', buf, o + 4)[0]
    o += 12                             # id, type, action
    o = al(o, 8) + 56                   # pose
    o = al(o, 8) + 24                   # scale
    o = al(o, 4) + 16                   # color
    o = al(o, 4) + 8                    # lifetime
    o += 1                              # frame_locked
    n, o = u32(o)
    if n == 0:
        return mtype, np.zeros((0, 3))
    o = al(o, 8)
    return mtype, np.frombuffer(buf, dtype=e + 'f8', count=3 * n, offset=o).reshape(n, 3)


class Win:
    """Per-second accumulator for one marker topic."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.msgs = self.empty = self.pts = 0
        self.d = []

    def add(self, pts, cam):
        self.msgs += 1
        if not len(pts):
            self.empty += 1
            return
        self.pts += len(pts)
        if cam is not None:
            self.d.append(np.linalg.norm(pts - cam, axis=1))

    def row(self):
        if self.d:
            d = np.concatenate(self.d)
            stats = ['%.2f' % d.min(), '%.2f' % np.median(d), '%.2f' % d.max()]
        else:
            stats = ['', '', '']
        r = [self.msgs, self.empty, self.pts] + stats
        self.reset()
        return r


class ColourCheck(Node):
    def __init__(self, out, seconds):
        super().__init__('colour_check')
        self.t_end = time.time() + seconds if seconds > 0 else None
        self.f = open(os.path.join(out, 'colour_check.csv'), 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(COLS)
        a = math.radians(LEVEL_DEG)
        self.c, self.s = math.cos(a), math.sin(a)
        self.cam = None
        self.n = dict(color=0, depth=0, odom=0, occ=0)
        self.lag = dict(color=float('nan'), depth=float('nan'))
        self.color_last = None
        self.color_gap = 0.0
        self.frame_no = None
        self.drops = 0
        self.exposure = ''
        self.occ_known = ''
        self.cmark, self.tmark = Win(), Win()
        self.checked = False
        # best effort matches every publisher; everything is on one host (shared memory)
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(CameraInfo, '/camera0/color/camera_info', self.on_color, be)
        self.create_subscription(CameraInfo, '/camera0/depth/camera_info',
                                 lambda m: self.on_info('depth', m), be)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.on_odom, be)
        self.create_subscription(Marker, '/nvblox_node/color_layer_marker',
                                 lambda b: self.on_marker(self.cmark, b), be, raw=True)
        self.create_subscription(Marker, '/nvblox_node/tsdf_layer_marker',
                                 lambda b: self.on_marker(self.tmark, b), be, raw=True)
        self.create_subscription(OccupancyGrid, '/nvblox_node/static_occupancy_grid', self.on_occ, be, raw=True)
        try:
            from realsense2_camera_msgs.msg import Metadata
            self.create_subscription(Metadata, '/camera0/color/metadata', self.on_meta, be)
        except ImportError:
            self.get_logger().warn('realsense2_camera_msgs not found: no exposure / dropped-frame columns')
        self.create_timer(1.0, self.tick)

    def on_info(self, k, m):
        self.n[k] += 1
        self.lag[k] = (time.time() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)) * 1e3

    def on_color(self, m):
        now = time.monotonic()
        if self.color_last is not None:
            self.color_gap = max(self.color_gap, now - self.color_last)
        self.color_last = now
        self.on_info('color', m)

    def on_meta(self, m):
        try:
            j = json.loads(m.json_data)
        except ValueError:
            return
        fn = j.get('frame_number')
        if fn is not None:
            if self.frame_no is not None and fn > self.frame_no + 1:
                self.drops += fn - self.frame_no - 1
            self.frame_no = fn
        self.exposure = j.get('actual_exposure', self.exposure)

    def on_odom(self, m):
        self.n['odom'] += 1
        p = m.pose.pose.position
        self.cam = np.array([p.x * self.c + p.z * self.s, p.y, -p.x * self.s + p.z * self.c])

    def on_marker(self, win, buf):
        try:
            _, pts = marker_points(buf)
        except (struct.error, ValueError) as e:
            self.get_logger().error('marker parse failed: %r' % e)
            return
        if not self.checked and len(pts):
            # one-time proof that the byte parser agrees with rclpy
            m = deserialize_message(buf, Marker)
            ok = len(m.points) == len(pts) and abs(m.points[-1].z - pts[-1, 2]) < 1e-9
            self.get_logger().info('marker parser self-check %s (%d points, type %d, frame %s)'
                                   % ('OK' if ok else 'MISMATCH', len(pts), m.type, m.header.frame_id))
            self.checked = True
        win.add(pts, self.cam)

    def on_occ(self, buf):
        m = deserialize_message(buf, OccupancyGrid)
        self.n['occ'] += 1
        self.occ_known = int(np.count_nonzero(np.frombuffer(m.data, dtype=np.int8) >= 0))

    def tick(self):
        now = time.time()
        cam = ['%.2f' % v for v in self.cam] if self.cam is not None else ['', '', '']
        gap = self.color_gap * 1e3
        if self.color_last is not None:
            gap = max(gap, (time.monotonic() - self.color_last) * 1e3)
        self.w.writerow([
            '%.3f' % now, time.strftime('%H:%M:%S', time.gmtime(now)),
            self.n['color'], '%.0f' % self.lag['color'], '%.0f' % gap, self.drops, self.exposure,
            self.n['depth'], '%.0f' % self.lag['depth'], self.n['odom']] + cam
            + self.cmark.row() + self.tmark.row() + [self.n['occ'], self.occ_known])
        self.f.flush()
        for k in self.n:
            self.n[k] = 0
        self.color_gap = 0.0
        self.drops = 0
        if self.t_end and now >= self.t_end:
            raise SystemExit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=900)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = ColourCheck(a.out, a.seconds)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
