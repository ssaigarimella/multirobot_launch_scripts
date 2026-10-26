#!/usr/bin/env python3
"""test_planner_offline.py - offline evidence for the planner changes of
2026-10-26, run against the version that flew on 2026-10-23.

On this laptop use the SYSTEM numpy (the user-site NumPy 2 breaks SciPy):

    python3 -s tests/planner_offline/test_planner_offline.py parse \
        --esdf esdf_big.cdr --bbox 0 -14.5 35 20.5
    python3 -s tests/planner_offline/test_planner_offline.py replay \
        --npz delta.npz --from 15:29:16 --to 15:30:53 --day 2026-10-23

In a drone container pass --old/--new explicitly (no ~/jfr_dev there).

parse   Both versions' real _esdf_cb on one recorded ESDF slice: raw_esdf and
        the horizontal-distance grid must be IDENTICAL; prints ms per message.
replay  Every recorded frontier frame of a flight is fed to BOTH versions'
        real _run_planning with the same map and position (map rebuilt from
        the recorded occupancy slice). Reports how often each version picks a
        plan whose waypoints leave the geofence margin - in the flown code
        that is the "Waypoint outside geofence, skipping" loop - and whether
        the two otherwise choose the same goal.

The planner node is not instantiated: its methods are bound to a plain
namespace carrying the same attributes, so no ROS graph, px4_msgs or
multi_drone_nvblox is needed (px4_msgs is stubbed when absent).
"""
import argparse
import datetime as dt
import importlib.util
import os
import statistics
import subprocess
import sys
import tempfile
import time
import types

import numpy as np

# loading the planners must not rewrite the __pycache__ that active_exploration tracks in git
sys.dont_write_bytecode = True

AE = os.path.expanduser('~/jfr_dev/active_exploration')
FLOWN = '6f2c629'   # active_exploration SHA in every 2026-10-23 bag manifest


# ------------------------------------------------------------------ loading
def stub_px4():
    try:
        import px4_msgs.msg  # noqa: F401  (drone container: the real thing)
        return
    except ImportError:
        pass
    pkg, msg = types.ModuleType('px4_msgs'), types.ModuleType('px4_msgs.msg')
    for n in ('TrajectorySetpoint', 'VehicleLocalPosition', 'VehicleStatus'):
        setattr(msg, n, type(n, (), {}))
    pkg.msg = msg
    sys.modules['px4_msgs'], sys.modules['px4_msgs.msg'] = pkg, msg


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def load_versions(old_path, new_path):
    stub_px4()
    if old_path is None:
        src = subprocess.run(
            ['git', '-C', AE, 'show', f'{FLOWN}:scripts/simple_exploration_planner.py'],
            capture_output=True, text=True, check=True).stdout
        old_path = os.path.join(tempfile.mkdtemp(), 'planner_flown.py')
        open(old_path, 'w').write(src)
    new_path = new_path or os.path.join(AE, 'scripts', 'simple_exploration_planner.py')
    return load(old_path, 'planner_flown'), load(new_path, 'planner_new')


# ------------------------------------------------------------- fake node
class _Log:
    def __init__(self):
        self.lines = []

    def _l(self, msg, **_kw):
        self.lines.append(str(msg))

    info = warn = warning = error = debug = fatal = _l


class _Clock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        import rclpy.time
        return rclpy.time.Time(nanoseconds=int(self.t * 1e9))


# the values planner_stage / simple_planner_launch passed on 2026-10-23
PARAMS = dict(collision_threshold=1.5, goal_threshold=1.0, reactive_threshold=0.5,
              reactive_lookahead=2.0, position_tolerance=0.3, waypoint_spacing=0.5,
              replan_timeout_s=30.0, plan_budget_s=3.0, geofence_margin=0.3,
              esdf_max_age_s=2.0, gain_weight=1.0, distance_weight=3.0, voxel_size=0.2,
              flight_height=1.0, score_eps=1.0, r_claim=8.0, w_claim=0.15, r_pose=5.0,
              w_pose=0.15, r_keepout=3.0, bl_cooldown=30.0, bl_radius=1.2,
              visited_cooldown_s=60.0, visited_radius=1.5, visited_penalty=0.1,
              escape_snap_m=2.5, px4_stale_s=2.0, guard_abandon_s=4.0,
              lane_spacing_m=4.0, coverage_margin_m=1.0, obstacle_clearance_m=0.5)


def make_fake(mod, bbox):
    f = types.SimpleNamespace()
    for k, v in mod.SimpleExplorationPlanner.__dict__.items():
        if isinstance(v, types.FunctionType):
            setattr(f, k, types.MethodType(v, f))
    f.log, f.clock = _Log(), _Clock()
    f.get_logger = lambda: f.log
    f.get_clock = lambda: f.clock
    f._publish_markers = lambda *a, **k: None
    f.bbox_min_x, f.bbox_min_y, f.bbox_max_x, f.bbox_max_y = map(float, bbox)
    for k, v in PARAMS.items():
        setattr(f, k, v)
    nx = int((f.bbox_max_x - f.bbox_min_x) / f.voxel_size) + 1
    ny = int((f.bbox_max_y - f.bbox_min_y) / f.voxel_size) + 1
    f.raw_esdf = np.full((nx, ny), -1000.0, dtype=np.float32)
    f.grid = np.full((nx, ny), -1000.0, dtype=np.float32)
    f.esdf_stamp = f.clock.now()
    # coordination branches off: no LoRa peer reached either replayed pair
    f.vehicle_id = 0
    f.peer_claims, f.peer_positions, f.peer_lane_ids = [], [], []
    f.centroids, f.viewpoints, f.gains = [], [], []
    f.flu_x = f.flu_y = f.flu_yaw = 0.0
    f.odom_received = False
    f.bous_wps, f.bous_idx = [], 0
    reset_plan_state(f, mod)
    return f


def reset_plan_state(f, mod):
    f.state = mod.State.PLAN
    f.waypoints, f.wp_idx = [], 0
    f.consecutive_no_path = 0
    f.visited_positions, f._goal_blacklist = [], []
    f.plan_goal_xy = None
    f.hold_latched = False
    f.yaw_aligned_for_wp = -1
    f.yaw_align_latched = False
    f.plan_start_time = f.clock.now()
    f.escape_active = False


# ------------------------------------------------------------- parse test
def pc2_from_arrays(x, y, intensity):
    """A PointCloud2-shaped namespace (x, y, z, intensity float32, 16 B/pt)."""
    rec = np.zeros(len(x), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4')])
    rec['x'], rec['y'], rec['i'] = x, y, intensity
    F = types.SimpleNamespace
    return F(fields=[F(name='x', offset=0), F(name='y', offset=4),
                     F(name='z', offset=8), F(name='intensity', offset=12)],
             point_step=16, data=rec.tobytes())


def cmd_parse(a):
    old, new = load_versions(a.old, a.new)
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    msg = deserialize_message(open(a.esdf, 'rb').read(), PointCloud2)
    npts = len(msg.data) // msg.point_step
    print(f'ESDF slice: {len(msg.data) / 1e6:.1f} MB, {npts} points, fields '
          f'{[f.name for f in msg.fields]}')
    res = {}
    for name, mod, reps in (('flown', old, a.reps_old), ('new', new, a.reps)):
        f = make_fake(mod, a.bbox)
        ts = []
        for _ in range(reps):
            t0 = time.perf_counter()
            f._esdf_cb(msg)
            ts.append(time.perf_counter() - t0)
        res[name] = (f, statistics.median(ts))
        print(f'  {name:6s} _esdf_cb: median {res[name][1] * 1e3:8.1f} ms over {reps} runs')
    fo, fn = res['flown'][0], res['new'][0]
    same_raw = np.array_equal(fo.raw_esdf, fn.raw_esdf)
    same_grid = np.array_equal(fo.grid, fn.grid)
    print(f'  raw_esdf identical: {same_raw}   grid identical: {same_grid}   '
          f'known cells: {int((fn.raw_esdf > -500).sum())}   '
          f'speed-up: x{res["flown"][1] / res["new"][1]:.0f}')
    return 0 if (same_raw and same_grid) else 1


# ------------------------------------------------------------ replay test
def parse_clock(s, day):
    h, m, sec = (s.split(':') + ['0'])[:3]
    d = dt.datetime.strptime(day, '%Y-%m-%d').replace(hour=int(h), minute=int(m))
    return d.timestamp() + float(sec)


def map_from_occupancy(f_new, hdr, G):
    """The planner's own grid, built by feeding its _esdf_cb a synthetic slice:
    every planner cell covered by the recorded occupancy slice is known; it is
    a wall (ESDF 0) if any occupied cell falls in it, else clear (ESDF 1)."""
    ox, oy, res, h, w = hdr
    G = G.reshape(int(h), int(w))
    rows, cols = np.nonzero(G)
    xs = ox + (cols + 0.5) * res
    ys = oy + (rows + 0.5) * res
    gi = ((xs - f_new.bbox_min_x) / f_new.voxel_size).astype(np.int64)
    gj = ((ys - f_new.bbox_min_y) / f_new.voxel_size).astype(np.int64)
    nx, ny = f_new.raw_esdf.shape
    ok = (gi >= 0) & (gi < nx) & (gj >= 0) & (gj < ny)
    val = np.full((nx, ny), np.nan, dtype=np.float32)
    val[gi[ok], gj[ok]] = 1.0
    occ = ok & (G[rows, cols] == 2)
    val[gi[occ], gj[occ]] = 0.0
    ci, cj = np.nonzero(np.isfinite(val))
    cx = f_new.bbox_min_x + (ci + 0.5) * f_new.voxel_size
    cy = f_new.bbox_min_y + (cj + 0.5) * f_new.voxel_size
    f_new._esdf_cb(pc2_from_arrays(cx, cy, val[ci, cj]))
    return f_new.raw_esdf.copy(), f_new.grid.copy()


def cmd_replay(a):
    old, new = load_versions(a.old, a.new)
    D = np.load(a.npz)
    bbox = a.bbox or list(D['bbox'])
    t0, t1 = parse_clock(a.t_from, a.day), parse_clock(a.t_to, a.day)
    od = D['odom']
    fr_t, vp, gains, off = D['fr_t'], D['vp'], D['gains'], D['vp_off']
    g_t, g_hdr, g_off, g_data = D['g_t'], D['g_hdr'], D['g_off'], D['g_data']
    fo, fn = make_fake(old, bbox), make_fake(new, bbox)
    builder = make_fake(new, bbox)
    cache = {}
    rows = []
    for k in np.nonzero((fr_t >= t0) & (fr_t <= t1))[0]:
        t = fr_t[k]
        i = max(0, np.searchsorted(od[:, 0], t, 'right') - 1)
        gidx = max(0, np.searchsorted(g_t, t, 'right') - 1)
        if gidx not in cache:
            cache = {gidx: map_from_occupancy(
                builder, g_hdr[gidx], g_data[g_off[gidx]:g_off[gidx + 1]])}
        raw, grid = cache[gidx]
        V = vp[off[k]:off[k + 1]]
        Gn = gains[off[k]:off[k + 1]]
        out = {}
        for name, mod, f in (('flown', old, fo), ('new', new, fn)):
            reset_plan_state(f, mod)
            f.flu_x, f.flu_y, f.flu_yaw = od[i, 1], od[i, 2], od[i, 4]
            f.viewpoints = [tuple(map(float, v)) for v in V]
            f.gains = [float(g) for g in Gn]
            f.centroids = [(v[0], v[1]) for v in f.viewpoints]
            f.clock.t = t
            f.esdf_stamp = f.clock.now()
            f.raw_esdf[:], f.grid[:] = raw, grid
            f._run_planning()
            wps = list(f.waypoints) if f.state == mod.State.EXECUTE else []
            out[name] = dict(
                state=f.state, goal=f.plan_goal_xy if wps else None,
                outside=any(not f._in_geofence(wx, wy) for wx, wy, _ in wps),
                goal_outside=bool(wps) and not f._in_geofence(*f.plan_goal_xy))
        rows.append((t, len(V), out))

    def fmt(t):
        return dt.datetime.fromtimestamp(t).strftime('%H:%M:%S')

    n = len(rows)
    for name in ('flown', 'new'):
        r = [o[name] for _, _, o in rows]
        print(f'  {name:6s}: {n} frames | planned {sum(x["state"] == "EXECUTE" for x in r)}'
              f' | goal outside margin {sum(x["goal_outside"] for x in r)}'
              f' | any waypoint outside margin {sum(x["outside"] for x in r)}'
              f' | DONE {sum(x["state"] == "DONE" for x in r)}'
              f' | no path {sum(x["state"] == "PLAN" for x in r)}')
    same = [o for _, _, o in rows
            if o['flown']['goal'] and not o['flown']['outside'] and o['new']['goal']]
    agree = sum(np.hypot(o['flown']['goal'][0] - o['new']['goal'][0],
                         o['flown']['goal'][1] - o['new']['goal'][1]) < 0.01 for o in same)
    print(f'  frames where the flown plan stayed inside the margin: {len(same)}; '
          f'new picks the same goal in {agree}')
    if a.verbose:
        for t, nv, o in rows:
            fl, nw = o['flown'], o['new']
            g = lambda x: '(%5.1f,%5.1f)%s' % (x['goal'][0], x['goal'][1], ' OUT' if x['outside'] else '    ') \
                if x['goal'] else f'{x["state"]:16s}'
            print(f'    {fmt(t)} vps={nv:3d}  flown {g(fl)}  new {g(nw)}')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--old', help='flown planner .py (default: git show %s)' % FLOWN)
    ap.add_argument('--new', help='new planner .py (default: working copy)')
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('parse')
    p.add_argument('--esdf', required=True, help='raw CDR of one PointCloud2')
    p.add_argument('--bbox', nargs=4, type=float, required=True)
    p.add_argument('--reps', type=int, default=5)
    p.add_argument('--reps-old', type=int, default=3)
    r = sub.add_parser('replay')
    r.add_argument('--npz', required=True)
    r.add_argument('--from', dest='t_from', required=True)
    r.add_argument('--to', dest='t_to', required=True)
    r.add_argument('--day', default='2026-10-23')
    r.add_argument('--bbox', nargs=4, type=float)
    r.add_argument('-v', '--verbose', action='store_true')
    a = ap.parse_args()
    sys.exit(cmd_parse(a) if a.cmd == 'parse' else cmd_replay(a))


if __name__ == '__main__':
    main()
