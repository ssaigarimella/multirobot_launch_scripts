#!/usr/bin/env python3
"""bag2timeline.py - turn one drone's recorded bag (+ its CSV logs) into a
compact, ROS-free "timeline" the laptop can replay and render.

    ./viz/bag2timeline.py <run> [drones...] [--bags bags] [--logs collected]
                          [--grid-hz 1] [--voxel-hz 0.5] [--out bags/<run>/timeline]

Reads  bags/<run>/<drone>/              (rosbag2 sqlite3, slim or full)
       collected/<drone>/*_<stamp>.csv  (metrics_recorder / planner / guard CSVs
                                         whose time range overlaps the bag)
Writes bags/<run>/timeline/<drone>.pkl  (dict of numpy arrays + small lists)

WHY NOT REPLAY THE BAG LIVE
---------------------------
`ros2 bag play` + RViz (replay_fleet.sh) is the right tool for a 3D mesh look,
but it needs the custom message packages the laptop does not have, it cannot
scrub backwards, and screen-recording RViz is not a paper figure. This file
decodes everything ONCE with the pure-python `rosbags` library (custom types are
registered straight from the .msg files in ~/jfr_dev) and fleet_player.py draws
from the result at any rate, to the screen, to MP4, or to a PDF frame.

FRAMES
------
Everything the drone records is in its own `odom` frame (cuVSLAM). The team
frame is recovered the same way coordination_node does it:

    p_world = T_world_map(from /d<N>/alignment/T_world_self)
            * T_map_odom(from /tf, the odom_correction edge, identity w/o loop closure)
            * p_odom

so all drones land in ONE world frame using the alignment the fleet actually
flew with. The player applies the 2D (yaw + xy) part of that chain to
trajectories, grids, voxels, frontiers, goals and geofences alike.

TIME
----
All timestamps are the drone's CLOCK_REALTIME epoch seconds (bag receive time,
same clock as the CSVs). Cross-drone alignment is therefore bounded by NTP skew
(+/-25-50 ms, see rf_stats.sh) - fine for figures, not for per-message claims.
"""
import argparse
import glob
import json
import math
import os
import pickle
import sys
import zlib
from pathlib import Path

import numpy as np
import yaml

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
IDS = {'delta': 1, 'buckshee': 2, 'ghost': 3, 'thunderstrike': 4}
MSG_PKGS = [Path.home() / 'jfr_dev' / p / 'msg' for p in ('multi_drone_nvblox', 'radiohive')]


def typestore():
    ts = get_typestore(Stores.ROS2_HUMBLE)
    add = {}
    for d in MSG_PKGS:
        for f in glob.glob(str(d / '*.msg')):
            name = f'{d.parent.name}/msg/{Path(f).stem}'
            add.update(get_types_from_msg(Path(f).read_text(), name))
    ts.register(add)
    return ts


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Bucket:
    """Keep at most one sample per 1/hz seconds (the LAST one in each bucket)."""

    def __init__(self, hz):
        self.dt = 1.0 / hz if hz > 0 else 0.0
        self.last_t = -1e18
        self.items = []

    def offer(self, t, item):
        if self.dt and t - self.last_t < self.dt:
            return
        self.last_t = t
        self.items.append(item)


def read_csvs(logs_dir, drone, t0, t1, margin=120.0):
    """Every collected/<drone>/<kind>_<stamp>.csv whose rows overlap [t0,t1].
    Restarts during a flight produce several files per kind; they are
    concatenated and sorted so the player sees one stream.

    Tolerant per ROW, not per file: the 2026-10-23 power cuts left NUL-padded
    tails and half-written lines in otherwise good files, and one bad row must
    not discard a flight's worth of data."""
    out = {}
    for f in sorted(glob.glob(os.path.join(logs_dir, drone, '*_2*.csv'))):
        kind = os.path.basename(f).rsplit('_', 2)[0]
        try:
            with open(f, errors='replace') as fh:
                header = fh.readline().strip().split(',')
                if not header or header[0] not in ('t', 'timestamp'):
                    continue
                keep = []
                for ln in fh:
                    if '\x00' in ln or not ln.strip():
                        continue
                    r = ln.rstrip('\n').split(',')
                    if len(r) != len(header):
                        continue
                    try:
                        t = float(r[0])
                    except ValueError:
                        continue
                    if t0 - margin <= t <= t1 + margin:
                        keep.append(r)
        except OSError:
            continue
        if keep:
            out.setdefault(kind, {'header': header, 'rows': []})['rows'].extend(keep)
    for kind, d in out.items():
        d['rows'].sort(key=lambda r: float(r[0]))
    return out


def odom_level_deg(lev_od, lev_tf):
    """Pitch (deg) that maps the raw cuVSLAM odometry onto the odom->camera0_link
    pose the mapper used, from samples with identical header stamps; a 2D
    rotation fit in the x-z plane. None when the bag has too few matches (the
    replay then falls back to fleet_player.ODOM_LEVEL_PITCH_DEG)."""
    keys = [k for k in lev_od if k in lev_tf]
    if len(keys) < 50:
        return None
    u = np.array([lev_od[k] for k in keys]); v = np.array([lev_tf[k] for k in keys])
    cross = float(np.sum(u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]))
    dot = float(np.sum(u[:, 0] * v[:, 0] + u[:, 1] * v[:, 1]))
    return round(-math.degrees(math.atan2(cross, dot)), 2)


def extract(bag_dir, drone, grid_hz, voxel_hz, logs_dir):
    ts = typestore()
    vid = IDS[drone]
    meta = yaml.safe_load(open(os.path.join(bag_dir, 'metadata.yaml')))['rosbag2_bagfile_information']
    t_start = meta['starting_time']['nanoseconds_since_epoch'] * 1e-9
    t_end = t_start + meta['duration']['nanoseconds'] * 1e-9

    odom = []                      # t x y z yaw  (odom frame, camera0_link)
    tf_map_odom = []               # t x y yaw
    t_world_map = []               # t x y yaw  (alignment prior / loop closure)
    grids = {'own': Bucket(grid_hz), 'shared': Bucket(grid_hz)}
    # nvblox's color_layer_marker is INCREMENTAL (only blocks touched since the
    # last publish; measured: 7k points per message while the slice held 180k
    # known cells), so a 3D map is the UNION over time. Keep each voxel once,
    # keyed by its quantised centre, with the time it was first seen.
    vox_seen = {}
    vox_pts, vox_col, vox_t = [], [], []
    vox_res = [0.05]
    frontiers = []                 # (t, centroids Nx2, viewpoints Nx3(x,y,yaw), gains N)
    fr_tmp = {}
    planner = []                   # (t, goal(x,y)|None, path Nx2, waypoints Nx2, geofence Nx2, blocked_path Nx2)
    state_ev = []                  # (t, 'EXECUTE')  on change only
    guard_ev = []                  # (t, 'PASS'/'STOP'/...) on change only
    claims = []                    # t valid x y z gain reached frontier_exists path_len eta remaining
    modes = []                     # t mode lane
    peers = []                     # t self peer px py pz yaw claim_state cx cy cz gain mode lane flags batt age delivery
    links = []                     # t bridge_up peer delivery rx_hz age
    align = []                     # (t, own_state, {peer: (state, age)})
    workspace = None
    last_state = last_guard = None
    lev_tf, lev_od = {}, {}       # header stamp -> (x, z): odom->camera0_link TF and raw odometry

    with Reader(bag_dir) as r:
        for conn, tn, raw in r.messages():
            t = tn * 1e-9
            tp = conn.topic
            try:
                m = ts.deserialize_cdr(raw, conn.msgtype)
            except Exception as e:  # unknown/mismatched type: skip, keep going
                continue
            if tp == '/visual_slam/tracking/odometry':
                p = m.pose.pose.position
                odom.append((t, p.x, p.y, p.z, yaw_of(m.pose.pose.orientation)))
                lev_od[(m.header.stamp.sec, m.header.stamp.nanosec)] = (p.x, p.z)
            elif tp == '/tf':
                for x in m.transforms:
                    if x.header.frame_id == 'odom' and x.child_frame_id == 'camera0_link':
                        lev_tf[(x.header.stamp.sec, x.header.stamp.nanosec)] = (x.transform.translation.x,
                                                                                x.transform.translation.z)
                    if x.header.frame_id == 'map' and x.child_frame_id == 'odom':
                        tr = x.transform.translation
                        tf_map_odom.append((t, tr.x, tr.y, yaw_of(x.transform.rotation)))
            elif tp.endswith('/alignment/T_world_self'):
                tr = m.transform.translation
                t_world_map.append((t, tr.x, tr.y, yaw_of(m.transform.rotation), int(m.source), int(m.n_closures)))
            elif tp.endswith('/static_occupancy_grid'):
                key = 'shared' if 'shared' in tp else 'own'
                o = m.info.origin.position
                d = np.asarray(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
                # -1 unknown -> 0, 0 free -> 1, 100 occ -> 2 (uint8, compresses well)
                g = np.where(d < 0, 0, np.where(d > 50, 2, 1)).astype(np.uint8)
                # stored zlib-compressed (a 744x417 outdoor slice is 310 KB raw,
                # ~15 KB compressed); fleet_player.Drone.grid() inflates on access
                grids[key].offer(t, (t, o.x, o.y, float(m.info.resolution), g.shape, zlib.compress(g.tobytes(), 6)))
            elif tp.endswith('/color_layer_marker'):
                key = 'shared' if 'shared' in tp else 'own'
                if not m.points:
                    continue
                if key == 'shared' or voxel_hz <= 0:
                    continue
                res = float(m.scale.x) or 0.05
                vox_res[0] = res
                for p, c in zip(m.points, m.colors or [None] * len(m.points)):
                    k = (int(round(p.x / res)), int(round(p.y / res)), int(round(p.z / res)))
                    if k in vox_seen:
                        continue
                    vox_seen[k] = True
                    vox_pts.append((p.x, p.y, p.z))
                    vox_col.append((int(c.r * 255), int(c.g * 255), int(c.b * 255)) if c is not None else (128, 128, 128))
                    vox_t.append(t)
            elif tp == '/nvblox_node/workspace_bounds' and workspace is None and m.points:
                workspace = np.array([(p.x, p.y, p.z) for p in m.points], dtype=np.float32)
            elif tp == '/fis/frontier_centroids':
                fr_tmp['c'] = np.array([(p.position.x, p.position.y) for p in m.poses], dtype=np.float32).reshape(-1, 2)
                fr_tmp['t'] = t
            elif tp == '/fis/frontier_viewpoints':
                fr_tmp['v'] = np.array([(p.position.x, p.position.y, yaw_of(p.orientation)) for p in m.poses],
                                       dtype=np.float32).reshape(-1, 3)
            elif tp == '/fis/frontier_gains':
                fr_tmp['g'] = np.asarray(m.data, dtype=np.float32)
                if 'c' in fr_tmp:
                    frontiers.append((fr_tmp.get('t', t), fr_tmp['c'],
                                      fr_tmp.get('v', np.zeros((0, 3), np.float32)), fr_tmp['g']))
                    fr_tmp = {}
            elif tp == '/exploration/markers':
                goal = None
                path = wps = gf = bpath = np.zeros((0, 2), np.float32)
                for k in m.markers:
                    if k.action == 2 or k.action == 3:   # DELETE / DELETEALL
                        continue
                    if k.ns == 'goal':
                        goal = (k.pose.position.x, k.pose.position.y, yaw_of(k.pose.orientation))
                    elif k.ns == 'path':
                        path = np.array([(p.x, p.y) for p in k.points], np.float32).reshape(-1, 2)
                    elif k.ns == 'waypoints':
                        wps = np.vstack([wps, [[k.pose.position.x, k.pose.position.y]]])
                    elif k.ns == 'geofence':
                        gf = np.array([(p.x, p.y) for p in k.points], np.float32).reshape(-1, 2)
                    elif k.ns == 'blocked_path':
                        bpath = np.array([(p.x, p.y) for p in k.points], np.float32).reshape(-1, 2)
                planner.append((t, goal, path, wps, gf, bpath))
            elif tp == '/exploration/state':
                if m.data != last_state:
                    last_state = m.data
                    state_ev.append((t, m.data))
            elif tp == '/reactive_guard/status':
                if m.data != last_guard:
                    last_guard = m.data
                    guard_ev.append((t, m.data))
            elif tp == '/coordination/claim_intent':
                v = m.viewpoint
                claims.append((t, int(m.valid), v.x, v.y, v.z, float(m.gain), int(m.reached),
                               int(m.frontier_exists), float(getattr(m, 'path_length', 0.0)),
                               float(getattr(m, 'eta_s', 0.0)), float(getattr(m, 'remaining_m', 0.0))))
            elif tp == '/coordination/mode_state':
                modes.append((t, int(m.mode), int(m.lane_id)))
            elif tp == '/swarm/peers':
                for p in m.peers:
                    peers.append((t, int(m.self_id), int(p.peer_id), p.position.x, p.position.y, p.position.z,
                                  float(p.yaw_deg), int(p.claim_state), p.claim.x, p.claim.y, p.claim.z,
                                  int(p.claim_gain), int(p.mode), int(p.lane_id), int(p.flags), int(p.batt),
                                  float(p.age_s), float(p.delivery_ratio)))
                if not m.peers:
                    peers.append((t, int(m.self_id), 0) + (0.0,) * 15)
            elif tp.endswith('/swarm/link_stats'):
                if len(m.peer_ids) == 0:
                    links.append((t, int(m.lora_bridge_up), 0, 0.0, 0.0, 0.0))
                for i, pid in enumerate(m.peer_ids):
                    links.append((t, int(m.lora_bridge_up), int(pid), float(m.delivery_ratio[i]),
                                  float(m.rx_rate_hz[i]), float(m.age_s[i])))
            elif tp.endswith('/alignment/status'):
                align.append((t, int(m.own_state),
                              {int(p): (int(s), float(a)) for p, s, a in zip(m.peer_ids, m.peer_states, m.peer_age_s)}))

    csvs = read_csvs(logs_dir, drone, t_start, t_end) if logs_dir else {}

    def arr(x, w):
        return np.array(x, dtype=np.float64).reshape(-1, w)

    tl = {
        'drone': drone, 'vehicle_id': vid, 'bag': str(bag_dir),
        't_start': t_start, 't_end': t_end,
        'odom': arr(odom, 5), 'tf_map_odom': arr(tf_map_odom, 4), 't_world_map': arr(t_world_map, 6),
        'grid_own': grids['own'].items, 'grid_shared': grids['shared'].items,
        'vox_res': vox_res[0],
        'odom_level_deg': odom_level_deg(lev_od, lev_tf),
        'vox_xyz': np.array(vox_pts, dtype=np.float32).reshape(-1, 3),
        'vox_rgb': np.array(vox_col, dtype=np.uint8).reshape(-1, 3),
        'vox_t': np.array(vox_t, dtype=np.float64),
        'workspace': workspace,
        'frontiers': frontiers, 'planner': planner, 'state_ev': state_ev, 'guard_ev': guard_ev,
        'claims': arr(claims, 11), 'modes': arr(modes, 3), 'peers': arr(peers, 18), 'links': arr(links, 6),
        'align': align, 'csv': csvs,
    }
    return tl


def summarize(tl):
    od = tl['odom']
    s = f"{tl['drone']:13s} {tl['t_start']:.0f}-{tl['t_end']:.0f} ({tl['t_end'] - tl['t_start']:.0f}s) " \
        f"odom {len(od)} grids own/shared {len(tl['grid_own'])}/{len(tl['grid_shared'])} " \
        f"voxels {len(tl['vox_t'])} frontiers {len(tl['frontiers'])} " \
        f"planner {len(tl['planner'])} claims {len(tl['claims'])} peers {len(tl['peers'])} links {len(tl['links'])}"
    if len(od):
        d = np.linalg.norm(np.diff(od[:, 1:3], axis=0), axis=1)
        s += f" path {d.sum():.1f}m"
    s += ' csv:' + ','.join(f"{k}({len(v['rows'])})" for k, v in tl['csv'].items())
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('drones', nargs='*')
    ap.add_argument('--bags', default=str(ROOT / 'bags'))
    ap.add_argument('--logs', default=str(ROOT / 'collected'), help="'' to skip CSV logs")
    ap.add_argument('--grid-hz', type=float, default=1.0)
    ap.add_argument('--voxel-hz', type=float, default=1.0, help='0 disables the 3D voxel union')
    ap.add_argument('--out', default=None)
    ap.add_argument('--csv-only', action='store_true', help='re-join the CSV logs into existing .pkl files (seconds, no bag decode)')
    a = ap.parse_args()
    run_dir = Path(a.bags) / a.run
    drones = a.drones or [d for d in IDS if (run_dir / d / 'metadata.yaml').exists()]
    out = Path(a.out) if a.out else run_dir / 'timeline'
    out.mkdir(parents=True, exist_ok=True)
    for d in drones:
        bag = run_dir / d
        if not (bag / 'metadata.yaml').exists():
            print(f'{d}: no bag at {bag}', file=sys.stderr)
            continue
        if a.csv_only:
            with open(out / f'{d}.pkl', 'rb') as fh:
                tl = pickle.load(fh)
            tl['csv'] = read_csvs(a.logs, d, tl['t_start'], tl['t_end']) if a.logs else {}
        else:
            tl = extract(str(bag), d, a.grid_hz, a.voxel_hz, a.logs or None)
        with open(out / f'{d}.pkl', 'wb') as fh:
            pickle.dump(tl, fh, protocol=pickle.HIGHEST_PROTOCOL)
        print(summarize(tl), f'-> {out / (d + ".pkl")} ({os.path.getsize(out / (d + ".pkl")) / 1e6:.0f} MB)')


if __name__ == '__main__':
    main()
