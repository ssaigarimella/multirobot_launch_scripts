#!/usr/bin/env python3
"""prep_replay.py - export what the planner replay test needs from a flight
timeline (viz/bag2timeline.py output) into a NumPy-version-neutral .npz.

    python3 tests/planner_offline/prep_replay.py e4c delta OUT.npz

Why a separate step: the timelines are pickled with the laptop's user-site
NumPy 2.x, while the planner needs SciPy, which on this laptop only works
with the system NumPy 1.21 (python3 -s). Plain arrays in an .npz load in both
and in the drone containers.
"""
import pickle
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def main():
    run, drone, out = sys.argv[1], sys.argv[2], sys.argv[3]
    tl = pickle.load(open(ROOT / 'bags' / run / 'timeline' / f'{drone}.pkl', 'rb'))

    # frontier frames as flat arrays + offsets (ragged)
    fr_t, vp_off, vp, gains = [], [0], [], []
    for t, _c, v, g in tl['frontiers']:
        n = min(len(v), len(g))
        fr_t.append(t)
        vp.append(np.asarray(v[:n], np.float64).reshape(-1, 3))
        gains.append(np.asarray(g[:n], np.float64))
        vp_off.append(vp_off[-1] + n)

    # occupancy slices (own map), 1 Hz
    g_t, g_hdr, g_off, g_data = [], [], [0], []
    for t, ox, oy, res, shape, blob in tl['grid_own']:
        import zlib
        G = np.frombuffer(zlib.decompress(blob), np.uint8)
        g_t.append(t)
        g_hdr.append((ox, oy, res, shape[0], shape[1]))
        g_data.append(G)
        g_off.append(g_off[-1] + G.size)

    gfs = [p[4] for p in tl['planner'] if len(p[4])]
    gf = np.asarray(gfs[-1], np.float64) if gfs else np.zeros((0, 2))

    np.savez_compressed(
        out,
        odom=np.asarray(tl['odom'], np.float64),
        fr_t=np.asarray(fr_t, np.float64),
        vp=np.concatenate(vp) if vp else np.zeros((0, 3)),
        gains=np.concatenate(gains) if gains else np.zeros(0),
        vp_off=np.asarray(vp_off, np.int64),
        g_t=np.asarray(g_t, np.float64),
        g_hdr=np.asarray(g_hdr, np.float64).reshape(-1, 5),
        g_off=np.asarray(g_off, np.int64),
        g_data=np.concatenate(g_data) if g_data else np.zeros(0, np.uint8),
        bbox=np.array([gf[:, 0].min(), gf[:, 1].min(), gf[:, 0].max(), gf[:, 1].max()])
        if len(gf) else np.zeros(4))
    print(f'{drone}: {len(fr_t)} frontier frames, {len(g_t)} grids, bbox {np.round(gf.min(0), 2) if len(gf) else "?"}'
          f'..{np.round(gf.max(0), 2) if len(gf) else "?"} -> {out}')


if __name__ == '__main__':
    main()
