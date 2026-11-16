#!/usr/bin/env python3
"""probe_ros.py - fleet console probe; runs INSIDE a drone's container.

Read-only: subscribes to the small status topics of the eight tabs and prints
one JSON line per second on stdout. It never publishes. Heavy topics (depth
images, ESDF clouds, keyframes) are never subscribed: their liveness comes
from small companions (camera_info, slice bounds) or from the ROS graph.

Started by probe_host.py with a UDP-only Fast DDS profile, so it neither uses
nor can be broken by /dev/shm segments (fleet_ctl stop deletes those).
"""
import glob
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy, qos_profile_sensor_data)

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from visualization_msgs.msg import Marker

try:
    from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
    PX4_OK = True
except ImportError:
    PX4_OK = False
try:
    from multi_drone_nvblox.msg import SwarmPeers, LinkStats, AlignmentStatus
    MDN_OK = True
except ImportError:
    MDN_OK = False

VID = int(os.environ.get('ROS_DOMAIN_ID', '0'))
FLEET_IDS = [1, 2, 3, 4]

# ros2 launch file of each tab: "is the tab's process up" = its launch is alive
TAB_LAUNCH = {1: 'realsense_example.launch.py', 2: 'vio_bridge.launch.py',
              3: 'fis_stage.launch.py', 4: 'reactive_guard.launch.py',
              5: 'planner_stage.launch.py', 6: 'shared_mapper.launch.py',
              7: 'coordination_stage.launch.py'}


class Feed:
    """Arrival times of one topic: rate over the last 3 s and age."""

    def __init__(self):
        self.t = []
        self.last = None

    def tick(self):
        now = time.monotonic()
        self.t.append(now)
        self.last = now
        while self.t and self.t[0] < now - 3.0:
            self.t.pop(0)

    def hz(self):
        now = time.monotonic()
        return round(sum(1 for x in self.t if x > now - 3.0) / 3.0, 1)

    def age(self):
        return None if self.last is None else round(time.monotonic() - self.last, 1)


def be(depth=5):
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


class Probe(Node):
    def __init__(self):
        super().__init__('fleet_console_probe_d%d' % VID)
        self.f = {k: Feed() for k in ('odom', 'olevel', 'cam', 'px4', 'vstat', 'fis', 'guard',
                                      'planner', 'selfmap', 'shared', 'peers', 'link',
                                      'align')}
        self.pos = None
        self.level = None        # odom_correction's latched status (source, pitch, roll)
        self.fis_src = None      # the map service the frontier finder reads right now
        self.px4 = {}
        self.fis_n = None
        self.guard = None
        self.planner = None
        self.slice_z = None
        self.peers = []
        self.lora = {}
        self.align = None
        self.sub(Odometry, '/visual_slam/tracking/odometry', self._odom, be())
        self.sub(Odometry, '/visual_slam/tracking/odometry_level',
                 lambda m: self.f['olevel'].tick(), be())
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub(String, '/visual_slam/level_status', self._level, latched)
        self.sub(String, '/fis/esdf_source', self._fis_src, latched)
        self.sub(CameraInfo, '/camera0/depth/camera_info',
                 lambda m: self.f['cam'].tick(), qos_profile_sensor_data)
        if PX4_OK:
            px4q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              history=HistoryPolicy.KEEP_LAST, depth=5)
            self.sub(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._lpos, px4q)
            self.sub(VehicleStatus, '/fmu/out/vehicle_status', self._vstat, px4q)
        self.sub(PoseArray, '/fis/frontier_viewpoints', self._fis, 10)
        self.sub(String, '/reactive_guard/status', self._guard, 10)
        self.sub(String, '/exploration/state', self._planner, 10)
        self.sub(Marker, '/nvblox_node/esdf_slice_bounds', self._slice, 5)
        self.sub(OccupancyGrid, '/nvblox_shared_node/static_occupancy_grid',
                 lambda m: self.f['shared'].tick(), 2)
        if MDN_OK:
            self.sub(SwarmPeers, '/swarm/peers', self._peers, 10)
            self.sub(LinkStats, '/d%d/swarm/link_stats' % VID, self._link, 10)
            self.sub(AlignmentStatus, '/d%d/alignment/status' % VID, self._align, 10)
        self.create_timer(1.0, self._emit)

    def sub(self, typ, topic, cb, qos):
        self.create_subscription(typ, topic, cb, qos)

    # ---- callbacks: keep only what the console shows ----
    def _odom(self, m):
        self.f['odom'].tick()
        p = m.pose.pose.position
        self.pos = (round(p.x, 2), round(p.y, 2), round(p.z, 2))

    def _level(self, m):
        try:
            d = json.loads(m.data)
            self.level = dict(source=d.get('source'), pitch=d.get('pitch_deg'), roll=d.get('roll_deg'))
        except ValueError:
            self.level = None

    def _fis_src(self, m):
        self.fis_src = m.data

    def _lpos(self, m):
        self.f['px4'].tick()
        self.px4.update(xy_valid=bool(m.xy_valid), z_valid=bool(m.z_valid),
                        z=round(float(m.z), 2))

    def _vstat(self, m):
        self.f['vstat'].tick()
        self.px4.update(armed=int(m.arming_state) == 2, nav=int(m.nav_state),
                        preflight_ok=bool(getattr(m, 'pre_flight_checks_pass', True)))

    def _fis(self, m):
        self.f['fis'].tick()
        self.fis_n = len(m.poses)

    def _guard(self, m):
        self.f['guard'].tick()
        self.guard = m.data

    def _planner(self, m):
        self.f['planner'].tick()
        self.planner = m.data

    def _slice(self, m):
        self.f['selfmap'].tick()
        zs = [p.z for p in m.points] or [m.pose.position.z]
        self.slice_z = round(sum(zs) / len(zs), 2)

    def _peers(self, m):
        self.f['peers'].tick()
        self.peers = [dict(id=int(p.peer_id), age=round(float(p.age_s), 1),
                           radio_only=bool(int(p.flags) & 0x80)) for p in m.peers]

    def _link(self, m):
        self.f['link'].tick()
        self.lora = dict(heard=[int(i) for i in m.peer_ids],
                         delivery=[round(float(d), 2) for d in m.delivery_ratio])

    def _align(self, m):
        self.f['align'].tick()
        self.align = int(m.own_state)

    # ---- output ----
    def _procs(self):
        out = {}
        try:
            ps = subprocess.run(['ps', '-eo', 'args'], capture_output=True,
                                text=True, timeout=3).stdout
        except Exception:
            return out
        for tab, launch in TAB_LAUNCH.items():
            out[str(tab)] = any(('ros2 launch' in l or '/ros2 launch' in l) and launch in l
                                for l in ps.splitlines())
        out['agent'] = 'MicroXRCEAgent' in ps
        out['lora_bridge'] = 'lora_bridge_node' in ps
        return out

    def _emit(self):
        zenoh_peers = []
        for p in FLEET_IDS:
            if p != VID and self.count_publishers('/d%d/keyframes/depth' % p) > 0:
                zenoh_peers.append(p)
        # A latched status outlives its node. Forget it when the node is gone
        # (tab stopped or restarted), so that the console waits for the new
        # node's word and does not read the old one. Not on a mere pause of
        # the stream: the node says it once and would not say it again.
        if self.count_publishers('/visual_slam/level_status') == 0:
            self.level = None
        if self.count_publishers('/fis/esdf_source') == 0:
            self.fis_src = None
        rec = dict(
            t=round(time.time(), 1), vid=VID,
            odom=dict(hz=self.f['odom'].hz(), age=self.f['odom'].age(), pos=self.pos),
            level=dict(hz=self.f['olevel'].hz(), **(self.level or {})),
            cam=dict(hz=self.f['cam'].hz(), age=self.f['cam'].age()),
            px4=dict(hz=self.f['px4'].hz(), age=self.f['px4'].age(), **self.px4),
            fis=dict(hz=self.f['fis'].hz(), age=self.f['fis'].age(), n=self.fis_n, src=self.fis_src),
            guard=dict(hz=self.f['guard'].hz(), age=self.f['guard'].age(), state=self.guard),
            planner=dict(hz=self.f['planner'].hz(), age=self.f['planner'].age(),
                         state=self.planner),
            selfmap=dict(hz=self.f['selfmap'].hz(), age=self.f['selfmap'].age(),
                         slice_z=self.slice_z),
            shared=dict(hz=self.f['shared'].hz(), age=self.f['shared'].age()),
            coord=dict(hz=self.f['peers'].hz(), age=self.f['peers'].age(), peers=self.peers),
            lora=dict(hz=self.f['link'].hz(), age=self.f['link'].age(), **self.lora),
            align=dict(age=self.f['align'].age(), state=self.align),
            zenoh_peers=zenoh_peers,
            procs=self._procs(),
            devs=dict(video=len(glob.glob('/dev/video*')),
                      tty=sorted(glob.glob('/dev/ttyUSB*'))),
            px4_msgs=PX4_OK, mdn_msgs=MDN_OK)
        sys.stdout.write(json.dumps(rec) + '\n')
        sys.stdout.flush()


def main():
    rclpy.init()
    node = Probe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
