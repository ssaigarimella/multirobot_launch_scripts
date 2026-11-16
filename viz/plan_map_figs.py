#!/usr/bin/env python3
"""plan_map_figs.py - two diagnostic figures about the 2D maps a flight planned on.

    ./viz/plan_map_figs.py <run> [drones...] --out reports/visualization/<run> [--at HH:MM:SS]

planning_map_vs_3d_map   per drone: the ground footprint of its 3D map (only
                         grows) against the known area of the 2D slice its
                         planner reads (nvblox static_occupancy_grid), and where
                         along the forward axis that slice has known cells,
                         against where the drone is.
where_the_goals_went     top view of the fence: ground under each drone's 3D map,
                         the planner's goals marked by whether that drone had
                         already mapped the spot more than 20 s earlier, and
                         the share of the fence covered against time.
own_vs_fused_map         per drone at one instant: its own slice (what the
                         planner and the frontier finder read) beside the slice
                         of its shared mapper (own depth + the peers' keyframes
                         received over WiFi), with the cells only the shared
                         mapper knows marked.

Everything here is as flown (no levelling), in the team frame of the flight.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import to_rgb  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from fleet_player import Fleet, parse_t, hhmmss, COLOR, INK, INK2, SURF, GRIDC  # noqa: E402
import fleet_3d_video as V  # noqa: E402
import fleet_showcase as SC  # noqa: E402


def slice_rgba(d, g, win, res, free_rgb, occ_rgb, mask=None, mask_rgb=None):
    """One occupancy slice drawn into a team-frame raster (x up, y left)."""
    X0, X1, Y0, Y1 = win
    Hh, Ww = int((X1 - X0) / res), int((Y1 - Y0) / res)
    t, x0, y0, r, arr = g
    rows, cols = np.nonzero(arr)
    val = arr[rows, cols]
    w = d.to_world_many(t, np.column_stack([x0 + (cols + 0.5) * r, y0 + (rows + 0.5) * r]))
    u = ((Y1 - w[:, 1]) / res).astype(int)
    v = ((X1 - w[:, 0]) / res).astype(int)
    ok = (u >= 0) & (u < Ww) & (v >= 0) & (v < Hh)
    known = np.zeros((Hh, Ww), bool)
    occ = np.zeros((Hh, Ww), bool)
    known[v[ok], u[ok]] = True
    k = ok & (val == 2)
    occ[v[k], u[k]] = True
    img = np.zeros((Hh, Ww, 4))
    img[known, :3], img[known, 3] = free_rgb, 1
    if mask is not None:
        m = known & ~mask
        img[m, :3] = mask_rgb
    img[occ, :3] = occ_rgb
    return img, known


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('drones', nargs='*')
    ap.add_argument('--bags', default=str(HERE.parent / 'bags'))
    ap.add_argument('--out', required=True)
    ap.add_argument('--at', help='instant for own_vs_fused_map (default: 80 %% through the Offboard window)')
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    raw = Fleet(a.run, a.drones, a.bags, rot='0')
    lev = Fleet(a.run, a.drones, a.bags, rot='0')
    V.level_to_flight_plane(lev)
    tw0 = min(d.offboard[0][0] for d in raw.drones if d.offboard)
    tw1 = max(d.offboard[-1][1] for d in raw.drones if d.offboard)
    plt.rcParams.update({'font.size': 9, 'axes.edgecolor': GRIDC, 'xtick.color': INK2, 'ytick.color': INK2,
                         'figure.facecolor': SURF, 'axes.facecolor': SURF, 'savefig.facecolor': SURF})

    # ------------------------------------------------ planning map against the 3D map
    sec = np.arange(0.0, tw1 - tw0 + 1.0, 1.0)
    gz = float(np.percentile(np.concatenate([d.tl['vox_xyz'][:, 2] for d in lev.drones]), 2))
    vox = {d.name: SC.voxels(d, tw1, 10 ** 9, 1.0, 1.0, gz) for d in lev.drones}
    foot, _ = SC.footprint({n: (v[0][:, :2].astype(np.float64), v[2]) for n, v in vox.items()}, tw0 + sec)
    fig, axs = plt.subplots(2, len(raw.drones), figsize=(4.6 * len(raw.drones), 5.6), sharex=True, squeeze=False)
    for j, d in enumerate(raw.drones):
        T, A, XL, XH, PX = [], [], [], [], []
        for g in d.grids:
            if not (tw0 <= g[0] <= tw1):
                continue
            _, x0, y0, r, arr = d.grid(g[0])
            kn = np.argwhere(arr > 0)
            if len(kn) < 50:
                continue
            kx = d.to_world_many(g[0], np.column_stack([x0 + (kn[:, 1] + 0.5) * r, y0 + (kn[:, 0] + 0.5) * r]))[:, 0]
            T.append(g[0] - tw0)
            A.append(len(kn) * r * r)
            XL.append(np.percentile(kx, 2))
            XH.append(np.percentile(kx, 98))
            PX.append(d.world_xy[d.idx(g[0]), 0])
        T, A, XL, XH, PX = map(np.array, (T, A, XL, XH, PX))
        c = COLOR[d.name]
        ax = axs[0, j]
        ax.plot(sec, foot[d.name], color=c, lw=2.0)
        ax.plot(T, A, color=c, lw=1.2, alpha=0.55)
        ax.annotate('3D map footprint', (sec[-1], foot[d.name][-1]), xytext=(-4, 5), textcoords='offset points',
                    ha='right', color=c, fontsize=8, fontweight='bold')
        k = int(np.argmin(A[len(A) // 2:])) + len(A) // 2
        ax.annotate('planning map (2D slice)', (T[k], A[k]), xytext=(0, -13), textcoords='offset points',
                    ha='center', color=c, fontsize=8)
        ax.set_title(d.name, loc='left', color=c, fontweight='bold', fontsize=10)
        ax.set_ylim(0, None)
        ax.grid(color=GRIDC, lw=0.6)
        ax = axs[1, j]
        ax.fill_between(T, XL, XH, color=c, alpha=0.18, lw=0)
        ax.plot(T, PX, color=c, lw=1.8)
        k = int(np.argmax(PX))
        ax.annotate('drone', (T[k], PX[k]), xytext=(4, 3), textcoords='offset points', color=c, fontsize=8,
                    fontweight='bold')
        ax.text(0.98, 0.04, 'shaded: where the planning map has known cells', transform=ax.transAxes,
                ha='right', color=INK2, fontsize=7.5)
        ax.grid(color=GRIDC, lw=0.6)
        ax.set_xlabel(f'time since {hhmmss(tw0)}  [s]')
        print('%-10s planning map: known area median %.0f m2 (max %.0f); 3D footprint at the end %.0f m2; '
              'known cells from %.1f m behind to %.1f m ahead of the drone (medians)' % (
                  d.name, np.median(A), A.max(), foot[d.name][-1], np.median(PX - XL), np.median(XH - PX)))
    axs[0, 0].set_ylabel('area  [m²]')
    axs[1, 0].set_ylabel('forward position x  [m]')
    fig.suptitle(f'{a.run}: the planning map moves with the drone, the 3D map keeps growing', x=0.01, ha='left',
                 fontsize=10.5, fontweight='bold', color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    for ext in ('png', 'pdf'):
        fig.savefig(out / f'planning_map_vs_3d_map.{ext}', dpi=170)
    plt.close(fig)

    # ------------------------------------------------ where the goals went
    gf = None
    for d in lev.drones:
        for p in d.planner:
            if len(p[4]):
                gf = SC.to_world(d, p[0], p[4])
                break
        if gf is not None:
            break
    cell = 0.25
    allw = np.vstack([d.world_xy[(d.od_t >= tw0) & (d.od_t <= tw1)] for d in lev.drones] + ([gf] if gf is not None else []))
    BX0, BX1, BY0, BY1 = allw[:, 0].min() - 8, allw[:, 0].max() + 8, allw[:, 1].min() - 8, allw[:, 1].max() + 8
    NX, NY = int((BX1 - BX0) / cell), int((BY1 - BY0) / cell)
    first = {}
    for d in lev.drones:
        w = d.to_world_many(tw1, d.tl['vox_xyz'][:, :2].astype(np.float64))
        ix, iy = ((w[:, 0] - BX0) / cell).astype(int), ((w[:, 1] - BY0) / cell).astype(int)
        ok = (ix >= 0) & (ix < NX) & (iy >= 0) & (iy < NY)
        F = np.full((NX, NY), np.inf)
        np.minimum.at(F, (ix[ok], iy[ok]), d.tl['vox_t'][ok])
        first[d.name] = F
    team = np.minimum.reduce(list(first.values()))
    xs, ys = BX0 + (np.arange(NX) + 0.5) * cell, BY0 + (np.arange(NY) + 0.5) * cell
    inside = np.ones((NX, NY), bool)
    if gf is not None:
        inside = ((xs[:, None] >= gf[:, 0].min()) & (xs[:, None] <= gf[:, 0].max()) &
                  (ys[None, :] >= gf[:, 1].min()) & (ys[None, :] <= gf[:, 1].max()))

    def old(F, x, y, t, lag=20.0):
        i, j = int((x - BX0) / cell), int((y - BY0) / cell)
        return bool((F[max(0, i - 1):i + 2, max(0, j - 1):j + 2] <= t - lag).any())
    img = np.ones((NX, NY, 3))
    names = [d.name for d in lev.drones]
    for n in names:
        c = np.array(to_rgb(COLOR[n]))
        m = np.isfinite(first[n])
        img[m] = img[m] * 0.55 + (0.72 * np.ones(3) + 0.28 * c) * 0.45 if len(names) > 1 else 0.72 * np.ones(3) + 0.28 * c
    img[~np.isfinite(team)] = 1.0
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(10.6, 6.0), gridspec_kw={'width_ratios': [1.55, 1]})
    ax.imshow(img[::-1], extent=(BY1, BY0, BX0, BX1) if False else (BY0, BY1, BX0, BX1), interpolation='nearest')
    ax.invert_xaxis()
    if gf is not None:
        ax.plot(gf[:, 1], gf[:, 0], color=INK2, lw=1.0, ls=(0, (4, 3)))
        ax.annotate('geofence', (gf[:, 1].max(), gf[:, 0].max()), xytext=(3, 4), textcoords='offset points',
                    color=INK2, fontsize=8)
    for d in lev.drones:
        m = (d.od_t >= tw0) & (d.od_t <= tw1)
        ax.plot(d.world_xy[m, 1], d.world_xy[m, 0], color=COLOR[d.name], lw=0.9, alpha=0.75)
        a_, b_ = d.offboard[0][0], d.offboard[-1][1]
        G = [(p[0], SC.to_world(d, p[0], [p[1][:2]])[0]) for p in d.planner if p[1] is not None and a_ <= p[0] <= b_]
        o = np.array([old(first[d.name], g[0], g[1], t) for t, g in G])
        P_ = np.array([g for _, g in G])
        ax.scatter(P_[~o, 1], P_[~o, 0], s=34, color=COLOR[d.name], edgecolor='white', linewidth=0.8, zorder=5)
        ax.scatter(P_[o, 1], P_[o, 0], s=30, marker='x', color=COLOR[d.name], linewidth=1.5, zorder=5)
        k = int(np.argmax(d.world_xy[m, 0]))
        ax.annotate(d.name, (d.world_xy[m][k, 1], d.world_xy[m][k, 0]), xytext=(5, 5), textcoords='offset points',
                    color=COLOR[d.name], fontweight='bold', fontsize=9)
        print('%-10s goals %d, of which %d at a spot it had mapped itself more than 20 s earlier' % (d.name, len(G), o.sum()))
    ax.scatter([], [], s=34, color=INK2, edgecolor='white', linewidth=0.8, label='goal on ground new to that drone')
    ax.scatter([], [], s=30, marker='x', color=INK2, linewidth=1.5, label='goal on ground it had already mapped')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.09), ncol=2, frameon=False, fontsize=8)
    ax.set_aspect('equal')
    ax.set_xlabel('to the left of the launch line  y  [m]')
    ax.set_ylabel('forward  x  [m]')
    ax.set_title('tinted: ground under each drone\'s 3D map · white: never mapped', loc='left', fontsize=8.5, color=INK2)
    ax.grid(color=GRIDC, lw=0.5)
    tt = np.arange(0.0, tw1 - tw0 + 1.0, 2.0)
    tot = float(inside.sum())
    ax2.plot(tt, [100.0 * ((team <= tw0 + t) & inside).sum() / tot for t in tt], color=INK, lw=2.0)
    for n in names:
        cv = [100.0 * ((first[n] <= tw0 + t) & inside).sum() / tot for t in tt]
        ax2.plot(tt, cv, color=COLOR[n], lw=1.5)
        ax2.annotate(n, (tt[-1], cv[-1]), xytext=(-3, -11), textcoords='offset points', ha='right', color=COLOR[n],
                     fontsize=8.5, fontweight='bold')
    cvT = 100.0 * ((team <= tw1) & inside).sum() / tot
    ax2.annotate('team', (tt[-1], cvT), xytext=(-3, 5), textcoords='offset points', ha='right', color=INK,
                 fontsize=8.5, fontweight='bold')
    ax2.set_ylim(0, 100)
    ax2.set_xlabel(f'time since {hhmmss(tw0)}  [s]')
    ax2.set_ylabel('share of the geofence under the 3D map  [%]')
    ax2.grid(color=GRIDC, lw=0.6)
    fig.suptitle(f'{a.run}: where the planner sent the drones', x=0.01, ha='left', fontsize=10.5, fontweight='bold',
                 color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    for ext_ in ('png', 'pdf'):
        fig.savefig(out / f'where_the_goals_went.{ext_}', dpi=170)
    plt.close(fig)

    # ------------------------------------------------ own slice beside the fused slice
    t_at = parse_t(a.at, tw0) if a.at else tw0 + 0.8 * (tw1 - tw0)
    pts = [d.world_xy[(d.od_t >= tw0) & (d.od_t <= tw1)] for d in raw.drones]
    P = np.vstack(pts)
    win = (P[:, 0].min() - 9, P[:, 0].max() + 9, P[:, 1].min() - 9, P[:, 1].max() + 9)
    res = 0.1
    fig, axs = plt.subplots(len(raw.drones), 2, figsize=(8.4, 4.1 * len(raw.drones)), squeeze=False)
    ext = (win[3], win[2], win[0], win[1])            # y left, x up
    for i, d in enumerate(raw.drones):
        c = np.array(to_rgb(COLOR[d.name]))
        peers = [o for o in raw.drones if o is not d]
        pc = np.array(to_rgb(COLOR[peers[0].name])) if peers else np.array([0.5, 0.5, 0.5])
        free, occ = 0.80 * np.ones(3) + 0.20 * c, 0.25 * c
        own, known_own = slice_rgba(d, d.grid(t_at), win, res, free, occ)
        gs = d.grid(t_at, shared=True)
        fused, known_f = slice_rgba(d, gs, win, res, free, occ, mask=known_own,
                                    mask_rgb=0.62 * np.ones(3) + 0.38 * pc)
        only = known_f & ~known_own
        for k, (img, ttl) in enumerate(((own, 'own map (planner and frontier finder read this)'),
                                        (fused, 'shared mapper (own depth + peer keyframes)'))):
            ax = axs[i, k]
            ax.imshow(img, extent=ext, interpolation='nearest')
            for o in raw.drones:
                m = (o.od_t >= tw0) & (o.od_t <= t_at)
                ax.plot(o.world_xy[m, 1], o.world_xy[m, 0], color=COLOR[o.name], lw=1.1, alpha=0.9)
                ax.plot(o.world_xy[o.idx(t_at), 1], o.world_xy[o.idx(t_at), 0], 'o', ms=6.5, color=COLOR[o.name],
                        mec='white', mew=1.2)
            ax.set_xlim(win[3], win[2])
            ax.set_ylim(win[0], win[1])
            ax.set_aspect('equal')
            ax.grid(color=GRIDC, lw=0.5)
            ax.set_title(('%s · ' % d.name if k == 0 else '') + ttl, loc='left', fontsize=8.5,
                         color=COLOR[d.name] if k == 0 else INK, fontweight='bold')
            if i == len(raw.drones) - 1:
                ax.set_xlabel('to the left of the launch line  y  [m]')
            if k == 0:
                ax.set_ylabel('forward  x  [m]')
        axs[i, 1].text(0.02, 0.02, 'known only to the shared mapper: %.0f m²' % (only.sum() * res * res),
                       transform=axs[i, 1].transAxes, fontsize=8, color=tuple(0.55 * pc))
        print('%-10s at %s: own slice knows %.0f m2, shared mapper knows %.0f m2, only the shared mapper %.0f m2' % (
            d.name, hhmmss(t_at), known_own.sum() * res * res, known_f.sum() * res * res, only.sum() * res * res))
    fig.suptitle(f'{a.run} at {hhmmss(t_at)}: own map and shared (fused) map of each drone, as flown', x=0.01,
                 ha='left', fontsize=10.5, fontweight='bold', color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    for e in ('png', 'pdf'):
        fig.savefig(out / f'own_vs_fused_map.{e}', dpi=170)
    plt.close(fig)
    print('figures ->', out)


if __name__ == '__main__':
    main()
