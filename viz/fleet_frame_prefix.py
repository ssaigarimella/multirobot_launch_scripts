#!/usr/bin/env python3
"""fleet_frame_prefix.py - make one drone's replayed bag coexist with another's.

WHY THIS EXISTS
---------------
Both drones record the SAME topic names and the SAME frame names. Measured on
ghost + delta, 2026-08-28, with tabs 1-7 up on both:

  * Topic names collide for everything except the /d<N>/ coordination topics:
    /tf, /nvblox_node/color_layer_marker, /fis/frontier_markers, /pose, ...
  * Each drone runs TWO DISCONNECTED TF TREES:
        map -> odom -> camera0_link -> camera0_*      (the real SLAM tree, ~30 Hz,
                                                       NOT prefixed)
        world -> d<N>/map -> d<N>/odom                (coordination stub, and
                                                       d<N>/odom has NO children)
    So `<prefix>camera0_link` does not exist; the camera frames are bare.

Replaying two such bags into one domain therefore produces two publishers of
`odom -> camera0_link` at 30 Hz each, and RViz shows one drone, or flickers.

WHAT THIS DOES
--------------
Per drone, subscribe under /d<N>/..., rewrite every frame name to `d<N>/<frame>`,
and republish on the global graph:

  * /raw/d<N>/tf, /raw/d<N>/tf_static -> the global /tf, /tf_static, with both
    frame_id and child_frame_id prefixed.
  * /raw/d<N>/<topic> -> /d<N>/<topic>, with header.frame_id (and
    child_frame_id, and MarkerArray members) prefixed.

The raw -> clean split matters: the bag player writes into /raw/d<N>/... and
this node publishes the corrected copy into /d<N>/..., so the two drones never
share a topic name and RViz can show one display per drone. Subscribing and
publishing on the same name would just be a feedback loop.

Two frames are deliberately NOT prefixed:
  * `world`      - the common root, shared by both drones by definition.
  * anything already starting with `d<N>/` - the recorded `world -> d<N>/map`
    alignment edge is already in the right namespace. Prefixing it again would
    yield `d3/world -> d3/d3/map` and detach the tree.

One edge is deliberately DROPPED, and ONLY on the static side: the
`d<N>/map -> d<N>/odom` stub that the coordination stack latches onto
/tf_static. After prefixing, the real dynamic `map -> odom` becomes exactly
that same edge, and two publishers of one edge give TF_REPEATED_DATA warnings
and visible jitter. The dynamic one is the real cuVSLAM/odom_correction output,
so the static stub loses.

The drop is restricted to /tf_static on purpose. Applying it to both streams
drops the dynamic edge too and orphans `d<N>/odom`, leaving the drone's whole
camera subtree detached from `world` - observed 2026-08-28.

/tf_static IS READ STRAIGHT FROM THE BAG (--bag), not taken from the player.
rosbag2's --start-offset skips every message before the offset, and the latched
/tf_static is recorded at t~0, so any offset at all silently drops the camera
extrinsics and leaves that drone's camera subtree detached - observed
2026-08-28 with ghost at --start-offset 10.176 s. Reading the bag directly makes
static TF independent of playback position, and it is re-latched every 5 s so a
late-joining RViz still gets it.

Depends only on stock ROS 2 Humble desktop (rclpy, tf2_msgs, geometry_msgs and
whatever the relayed topics already use). It deliberately does NOT import
nvblox_msgs / multi_drone_nvblox / px4_msgs, because THE LAPTOP DOES NOT HAVE
THEM (verified 2026-08-28) - see --skip-unknown handling in replay_fleet.sh.
"""
import argparse
import re
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage

# frames that must never be prefixed
ROOTS = {'world'}
# any frame already carrying SOME drone's prefix, not just this node's own.
# delta publishes `d3/base_link_est` (ghost's peer-pose estimate, arriving over
# the coordination link). Prefixing that with d1/ yields `d1/d3/base_link_est`,
# which detaches it from the tree - observed 2026-08-28 before this guard.
ALREADY = re.compile(r'^d\d+/')


def prefix_frame(frame: str, pfx: str) -> str:
    if not frame:
        return frame
    f = frame.lstrip('/')
    if f in ROOTS or ALREADY.match(f):
        return f
    return pfx + f


def rewrite(msg, pfx: str):
    """Duck-typed, recursive frame rewrite. Works for any stock message that
    carries a std_msgs/Header, a child_frame_id, or a list of such things."""
    # MarkerArray and friends: a list field of stamped members
    for field in ('markers', 'transforms', 'poses', 'states'):
        seq = getattr(msg, field, None)
        if isinstance(seq, list):
            for item in seq:
                rewrite(item, pfx)
    hdr = getattr(msg, 'header', None)
    if hdr is not None and hasattr(hdr, 'frame_id'):
        hdr.frame_id = prefix_frame(hdr.frame_id, pfx)
    if hasattr(msg, 'child_frame_id'):
        msg.child_frame_id = prefix_frame(msg.child_frame_id, pfx)
    return msg


def static_tf_from_bag(bag_dir):
    """Read every /tf_static message out of a bag, independent of playback."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    out = []
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf_static']))
        while reader.has_next():
            _topic, data, _t = reader.read_next()
            out.append(deserialize_message(data, TFMessage))
    except Exception as e:                            # noqa: BLE001
        print(f'[fleet_frame_prefix] could not read /tf_static from {bag_dir}: {e}',
              file=sys.stderr)
    return out


class FramePrefix(Node):
    def __init__(self, drone_id, topics, drop_edges, bag_dir=None):
        super().__init__(f'fleet_frame_prefix_d{drone_id}')
        self.pfx = f'd{drone_id}/'
        self.drop = set(drop_edges)
        self._keep = []
        self._static_cache = []
        self.inns = f'/raw/d{drone_id}'      # where the bag player writes
        self.outns = f'/d{drone_id}'         # where RViz reads

        # --- TF. /tf is volatile+depth 100; /tf_static MUST be transient_local
        # or late-joining RViz never sees the latched camera extrinsics.
        tf_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE,
                            history=HistoryPolicy.KEEP_LAST, depth=200)
        static_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST, depth=200)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', tf_qos)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', static_qos)
        self.create_subscription(TFMessage, f'{self.inns}/tf',
                                 lambda m: self._tf(m, False), tf_qos)
        self.create_subscription(TFMessage, f'{self.inns}/tf_static',
                                 lambda m: self._tf(m, True), static_qos)

        for spec in topics:
            self._relay(spec)

        # static TF straight from the bag, immune to --start-offset
        self._static_cache = []
        if bag_dir:
            for msg in static_tf_from_bag(bag_dir):
                self._tf(msg, True)
            if self._static_cache:
                self.create_timer(5.0, self._relatch)
                self.get_logger().info(
                    f'latched {len(self._static_cache)} static transforms from the bag')

        self.get_logger().info(
            f'prefixing d{drone_id}: {len(self._keep)} data topics + tf/tf_static '
            f'-> frames become {self.pfx}*   (dropping edges: {sorted(self.drop)})')

    def _tf(self, msg, is_static):
        out = TFMessage()
        for t in msg.transforms:
            parent = prefix_frame(t.header.frame_id, self.pfx)
            child = prefix_frame(t.child_frame_id, self.pfx)
            # static-only: dropping this edge on /tf as well would orphan
            # d<N>/odom and detach the whole camera subtree from `world`.
            if is_static and f'{parent}->{child}' in self.drop:
                continue
            t.header.frame_id, t.child_frame_id = parent, child
            out.transforms.append(t)
        if out.transforms:
            if is_static:
                self._static_cache.extend(out.transforms)
                self.tf_static_pub.publish(out)
            else:
                self.tf_pub.publish(out)

    def _relatch(self):
        if self._static_cache:
            self.tf_static_pub.publish(TFMessage(transforms=self._static_cache))

    def _relay(self, spec):
        topic, type_str = spec.split('=', 1)
        try:
            cls = get_message(type_str)
        except Exception as e:                       # noqa: BLE001
            self.get_logger().warn(f'skipping {topic}: type {type_str} unavailable ({e})')
            return
        # Latched-ish displays (maps, markers) want transient_local so RViz can
        # join late; everything else stays volatile to match the bag player.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        pub = self.create_publisher(cls, f'{self.outns}{topic}', qos)
        sub = self.create_subscription(
            cls, f'{self.inns}{topic}',
            lambda m, p=pub: p.publish(rewrite(m, self.pfx)), qos)
        self._keep.append((pub, sub))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--drone-id', type=int, required=True)
    ap.add_argument('--topic', action='append', default=[],
                    metavar='TOPIC=pkg/msg/Type',
                    help='data topic to relay with frames rewritten')
    ap.add_argument('--drop-edge', action='append', default=[],
                    metavar='parent->child',
                    help='static TF edge to suppress after prefixing')
    ap.add_argument('--bag', default=None,
                    help='bag dir to read /tf_static from, independent of '
                         'playback position (see --start-offset note above)')
    args, rest = ap.parse_known_args()

    rclpy.init(args=rest)
    node = FramePrefix(args.drone_id, args.topic, args.drop_edge, args.bag)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
