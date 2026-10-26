#!/usr/bin/env python3
"""fleet_player.py - replay a recorded multi-drone flight on the LAPTOP, as if
it were happening live, from the timelines bag2timeline.py produced.

    ./viz/fleet_player.py <run> [drones...]                 interactive window
    ./viz/fleet_player.py <run> --mp4 out.mp4 [--rate 1] [--fps 30]
    ./viz/fleet_player.py <run> --frame 14:22:25 --save frame.pdf
    ./viz/fleet_player.py <run> --figures reports/visualization/<run>

Window selection (all optional, local wall clock HH:MM[:SS] or epoch seconds):
    --from T --to T     default: 10 s before the first Offboard engage to 10 s
                        after the last disengage (from the planner CSV); falls
                        back to the union of the bags.
    --map own|shared    which occupancy grid to draw per drone (default own =
                        what that drone mapped itself; shared = what its shared
                        mapper held, i.e. own + what it received from peers)

Interactive keys:  SPACE play/pause   <- -> step 1 s   [ ] step 10 s
                   + - rate x2 / /2    s save PNG of the current frame
                   q quit              (drag the slider to scrub)

WHAT IS DRAWN (the team frame; +x is forward for drone 1, see bag2timeline.py)
    * each drone's own occupancy slice (free = light tint, occupied = dark) in
      the drone's colour, so "who mapped what" is visible at a glance
    * trajectory trail, current pose glyph with heading, planner goal + path
    * frontier centroids the FIS found (faint x), the drone's claim (dashed)
    * what each drone BELIEVES about its peers from the LoRa packets: a hollow
      ring at the believed position, dotted to the peer's true position, so a
      stale or missing belief is immediately visible
    * pairwise LoRa link lines coloured by delivery ratio (status palette)
    * right column: per-drone status tiles, event timeline, mapped-area curve

COLOURS are the validated 4-slot categorical palette (delta blue, buckshee
orange, ghost aqua, thunderstrike yellow) and the reserved status palette for
link quality; identity is never colour-alone (every glyph carries the drone's
initial letter).
"""
import argparse
import bisect
import datetime as dt
import math
import os
import pickle
import sys
import zlib
from pathlib import Path

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import transforms as mtr
from matplotlib.colors import ListedColormap, to_rgba
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Polygon, Rectangle

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
IDS = {'delta': 1, 'buckshee': 2, 'ghost': 3, 'thunderstrike': 4}
NAMES = {v: k for k, v in IDS.items()}
ORDER = ['delta', 'buckshee', 'ghost', 'thunderstrike']
# categorical slots 1-4 of references/palette.md (validated), fixed per drone
COLOR = {'delta': '#2a78d6', 'buckshee': '#eb6834', 'ghost': '#1baf7a', 'thunderstrike': '#eda100'}
STATUS = {'good': '#0ca30c', 'warning': '#fab219', 'serious': '#ec835a', 'critical': '#d03b3b'}
INK, INK2, SURF, GRIDC = '#0b0b0b', '#52514e', '#fcfcfb', '#e6e5e1'


def parse_t(s, day_ref):
    """'14:22:25' (local wall clock on the day of day_ref) or epoch seconds."""
    if s is None:
        return None
    try:
        return float(s)
    except ValueError:
        pass
    h, m, *rest = s.split(':')
    sec = float(rest[0]) if rest else 0.0
    d = dt.datetime.fromtimestamp(day_ref).replace(hour=int(h), minute=int(m), second=0, microsecond=0)
    return d.timestamp() + sec


def hhmmss(t):
    return dt.datetime.fromtimestamp(t).strftime('%H:%M:%S')


DISPLAY_ROT = [0.0]   # radians; set by Fleet before Drone objects are built


def rot2(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


class Drone:
    """One drone's timeline with time-indexed accessors (all O(log n))."""

    def __init__(self, tl):
        self.tl = tl
        self.name = tl['drone']
        self.vid = tl['vehicle_id']
        self.color = COLOR[self.name]
        self.letter = self.name[0].upper()
        self.od = tl['odom']
        self.od_t = self.od[:, 0] if len(self.od) else np.zeros(0)
        self.tmo = tl['tf_map_odom']
        self.twm = tl['t_world_map']
        self.grids = tl['grid_own']
        self.grids_shared = tl['grid_shared']
        self.grid_t = [g[0] for g in self.grids]
        self._gcache = (None, None)
        self.grid_shared_t = [g[0] for g in self.grids_shared]
        self.frontiers = tl['frontiers']
        self.fr_t = [f[0] for f in self.frontiers]
        self.planner = tl['planner']
        self.pl_t = [p[0] for p in self.planner]
        self.state_ev = tl['state_ev']
        self.guard_ev = tl['guard_ev']
        self.claims = tl['claims']
        self.peers = tl['peers']
        self.links = tl['links']
        csv = tl.get('csv', {})
        self.sp = self._csv(csv, 'splanner')
        self.gd = self._csv(csv, 'guard')
        self.lora = self._csv(csv, 'lora_rx')
        self.vio = self._csv(csv, 'vio_health')
        # offboard intervals from the planner CSV
        self.offboard = []
        if self.sp is not None and 'offboard' in self.sp['cols']:
            t, ob = self.sp['t'], self.sp['cols']['offboard']
            on = None
            for i in range(len(t)):
                if ob[i] and on is None:
                    on = t[i]
                elif not ob[i] and on is not None:
                    self.offboard.append((on, t[i]))
                    on = None
            if on is not None:
                self.offboard.append((on, t[-1]))
        # world-frame trajectory, precomputed once
        self.world_xy = np.zeros((len(self.od), 2))
        for i in range(len(self.od)):
            self.world_xy[i] = self.to_world(self.od[i, 0], self.od[i, 1:3])
        self.world_yaw = np.array([self.od[i, 4] + self.frame(self.od[i, 0])[0] for i in range(len(self.od))])
        # distance flown on a 2 Hz decimated track: summing 30 Hz VIO jitter
        # inflates the figure by ~30 % (thunderstrike: 157 m raw vs 117 m)
        n = len(self.od)
        if n > 1:
            step = max(1, int(round(0.5 * n / max(1e-3, self.od_t[-1] - self.od_t[0]))))
            idx = np.arange(0, n, step)
            if idx[-1] != n - 1:
                idx = np.append(idx, n - 1)
            cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(self.world_xy[idx], axis=0), axis=1))])
            self.path_len = np.interp(np.arange(n), idx, cum)
        else:
            self.path_len = np.zeros(n)

    @staticmethod
    def _csv(csv, kind):
        if kind not in csv:
            return None
        h, rows = csv[kind]['header'], csv[kind]['rows']
        t = np.array([float(r[0]) for r in rows])
        cols = {}
        for j, name in enumerate(h[1:], 1):
            try:
                cols[name] = np.array([float(r[j]) if r[j] != '' else np.nan for r in rows])
            except ValueError:
                cols[name] = [r[j] for r in rows]
        return {'t': t, 'cols': cols}

    # ---- frames -------------------------------------------------------------
    def frame(self, t):
        """(yaw, tx, ty) of the odom->world 2D transform valid at time t."""
        yaw, tx, ty = 0.0, 0.0, 0.0
        if len(self.twm):
            i = max(0, np.searchsorted(self.twm[:, 0], t, 'right') - 1)
            yaw, tx, ty = self.twm[i, 3], self.twm[i, 1], self.twm[i, 2]
        if DISPLAY_ROT[0]:   # display <- world: a pure rotation about the origin
            tx, ty = rot2(DISPLAY_ROT[0]) @ np.array([tx, ty])
            yaw += DISPLAY_ROT[0]
        if len(self.tmo):
            i = max(0, np.searchsorted(self.tmo[:, 0], t, 'right') - 1)
            # world <- map <- odom
            mo = self.tmo[i]
            R = rot2(yaw)
            tx, ty = (np.array([tx, ty]) + R @ mo[1:3])
            yaw += mo[3]
        return yaw, tx, ty

    def to_world(self, t, xy):
        yaw, tx, ty = self.frame(t)
        return rot2(yaw) @ np.asarray(xy, float) + np.array([tx, ty])

    @staticmethod
    def disp(xy):
        """Team-frame coordinates that arrive already in world (peer packets) -> display."""
        return rot2(DISPLAY_ROT[0]) @ np.asarray(xy, float) if DISPLAY_ROT[0] else np.asarray(xy, float)

    def to_world_many(self, t, pts):
        pts = np.asarray(pts, float).reshape(-1, 2)
        if not len(pts):
            return pts
        yaw, tx, ty = self.frame(t)
        return pts @ rot2(yaw).T + np.array([tx, ty])

    # ---- accessors ----------------------------------------------------------
    def idx(self, t):
        return max(0, np.searchsorted(self.od_t, t, 'right') - 1) if len(self.od_t) else None

    def alive(self, t):
        return len(self.od_t) > 0 and self.od_t[0] - 2 <= t <= self.od_t[-1] + 2

    def pose(self, t):
        i = self.idx(t)
        if i is None:
            return None
        return self.world_xy[i], self.world_yaw[i], self.od[i, 3]

    def speed(self, t):
        i = self.idx(t)
        if i is None or i < 5:
            return 0.0
        d = np.linalg.norm(self.world_xy[i] - self.world_xy[i - 5])
        return d / max(1e-3, self.od_t[i] - self.od_t[i - 5])

    def grid(self, t, shared=False):
        gs, gt = (self.grids_shared, self.grid_shared_t) if shared else (self.grids, self.grid_t)
        i = bisect.bisect_right(gt, t) - 1
        if i < 0:
            return None
        g = gs[i]
        if self._gcache[0] is not g:
            self._gcache = (g, np.frombuffer(zlib.decompress(g[5]), np.uint8).reshape(g[4]))
        return (g[0], g[1], g[2], g[3], self._gcache[1])

    def frontier(self, t):
        i = bisect.bisect_right(self.fr_t, t) - 1
        f = self.frontiers[i] if i >= 0 and t - self.fr_t[i] < 5.0 else None
        return f

    def plan(self, t):
        i = bisect.bisect_right(self.pl_t, t) - 1
        return self.planner[i] if i >= 0 else None

    @staticmethod
    def _last_ev(evs, t):
        lo, hi = 0, len(evs)
        while lo < hi:
            mid = (lo + hi) // 2
            if evs[mid][0] <= t:
                lo = mid + 1
            else:
                hi = mid
        return evs[lo - 1][1] if lo else None

    def state(self, t):
        return self._last_ev(self.state_ev, t) or '-'

    def guard(self, t):
        return self._last_ev(self.guard_ev, t) or '-'

    def csv_at(self, c, t, col, maxage=2.0):
        if c is None or col not in c['cols']:
            return None
        i = np.searchsorted(c['t'], t, 'right') - 1
        if i < 0 or t - c['t'][i] > maxage:
            return None
        return c['cols'][col][i]

    def is_offboard(self, t):
        return any(a <= t <= b for a, b in self.offboard)

    def claim(self, t):
        if not len(self.claims):
            return None
        i = np.searchsorted(self.claims[:, 0], t, 'right') - 1
        if i < 0:
            return None
        c = self.claims[i]
        return c if c[1] > 0 else None

    def peer_beliefs(self, t, maxage=3.0):
        """{peer_id: row} from the last /swarm/peers message at or before t."""
        if not len(self.peers):
            return {}
        i = np.searchsorted(self.peers[:, 0], t, 'right') - 1
        if i < 0 or t - self.peers[i, 0] > maxage:
            return {}
        t_msg = self.peers[i, 0]
        out = {}
        j = i
        while j >= 0 and self.peers[j, 0] == t_msg:
            if self.peers[j, 2] > 0:
                out[int(self.peers[j, 2])] = self.peers[j]
            j -= 1
        return out

    def link(self, t, maxage=3.0):
        """{peer_id: (delivery, rx_hz, age)} and bridge_up from the last LinkStats."""
        if not len(self.links):
            return None, {}
        i = np.searchsorted(self.links[:, 0], t, 'right') - 1
        if i < 0 or t - self.links[i, 0] > maxage:
            return None, {}
        t_msg = self.links[i, 0]
        up, out = bool(self.links[i, 1]), {}
        j = i
        while j >= 0 and self.links[j, 0] == t_msg:
            if self.links[j, 2] > 0:
                out[int(self.links[j, 2])] = tuple(self.links[j, 3:6])
            j -= 1
        return up, out

    def lora_rx_count(self, t0, t1):
        if self.lora is None:
            return 0
        return int(np.searchsorted(self.lora['t'], t1) - np.searchsorted(self.lora['t'], t0))

    def best_grid(self, t0, t1):
        """The own slice with the most known cells in [t0, t1] (the drone's best map)."""
        best, best_n = None, -1
        for g in self.grids:
            if t0 <= g[0] <= t1:
                n = np.count_nonzero(np.frombuffer(zlib.decompress(g[5]), np.uint8))
                if n > best_n:
                    best, best_n = g, n
        if best is None:
            return None
        return (best[0], best[1], best[2], best[3], np.frombuffer(zlib.decompress(best[5]), np.uint8).reshape(best[4]))

    def mapped_area(self, t):
        g = self.grid(t)
        if g is None:
            return 0.0
        return float((g[4] > 0).sum()) * g[3] ** 2


class Fleet:
    def __init__(self, run, drones, bags_dir, rot='auto'):
        tdir = Path(bags_dir) / run / 'timeline'
        self.run = run
        self.drones = []
        DISPLAY_ROT[0] = 0.0 if rot == 'auto' else math.radians(float(rot))
        for d in (drones or ORDER):
            f = tdir / f'{d}.pkl'
            if f.exists():
                with open(f, 'rb') as fh:
                    self.drones.append(Drone(pickle.load(fh)))
        if not self.drones:
            sys.exit(f'no timelines in {tdir}; run viz/bag2timeline.py {run} first')
        self.by_id = {d.vid: d for d in self.drones}
        self.t0 = min(d.tl['t_start'] for d in self.drones)
        self.t1 = max(d.tl['t_end'] for d in self.drones)
        if rot == 'auto':
            # forward (+x of the team frame) is up when the flown area is deeper
            # than it is wide - the usual outdoor case - so the map panel is used.
            x0, x1, y0, y1 = self.extent(self.t0, self.t1)
            if (x1 - x0) > (y1 - y0) * 1.2:
                DISPLAY_ROT[0] = math.pi / 2
                for d in self.drones:
                    d.__init__(d.tl)

    def default_window(self):
        ob = [iv for d in self.drones for iv in d.offboard]
        if ob:
            return min(a for a, _ in ob) - 10.0, max(b for _, b in ob) + 10.0
        return self.t0, self.t1

    def extent(self, tw0, tw1, drones=None):
        xs, ys = [], []
        for d in (drones or self.drones):
            m = (d.od_t >= tw0) & (d.od_t <= tw1)
            if m.any():
                xs += [d.world_xy[m, 0].min(), d.world_xy[m, 0].max()]
                ys += [d.world_xy[m, 1].min(), d.world_xy[m, 1].max()]
            g = d.grid(tw1)
            if g is not None:
                c = np.argwhere(g[4] > 0)
                if len(c):
                    pts = np.stack([g[1] + (c[:, 1] + 0.5) * g[3], g[2] + (c[:, 0] + 0.5) * g[3]], 1)
                    w = d.to_world_many(tw1, pts)
                    xs += [w[:, 0].min(), w[:, 0].max()]
                    ys += [w[:, 1].min(), w[:, 1].max()]
            p = d.plan(tw1)
            if p is not None and len(p[4]):
                w = d.to_world_many(tw1, p[4])
                xs += [w[:, 0].min(), w[:, 0].max()]
                ys += [w[:, 1].min(), w[:, 1].max()]
        if not xs:
            return -10, 10, -10, 10
        pad = 2.0
        return min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad

    def coverage_curve(self, tw0, tw1, step=2.0):
        """Mapped area (m^2) per drone and team union, rasterised at 0.25 m."""
        ts = np.arange(tw0, tw1 + step, step)
        x0, x1, y0, y1 = self.extent(tw0, tw1)
        res = 0.25
        W, H = int((x1 - x0) / res) + 1, int((y1 - y0) / res) + 1
        per = {d.name: np.zeros(len(ts)) for d in self.drones}
        union = np.zeros(len(ts))
        for k, t in enumerate(ts):
            U = np.zeros((H, W), bool)
            for d in self.drones:
                g = d.grid(t)
                if g is None:
                    continue
                c = np.argwhere(g[4] > 0)
                if not len(c):
                    continue
                per[d.name][k] = len(c) * g[3] ** 2
                pts = np.stack([g[1] + (c[:, 1] + 0.5) * g[3], g[2] + (c[:, 0] + 0.5) * g[3]], 1)
                w = d.to_world_many(t, pts)
                ix = ((w[:, 0] - x0) / res).astype(int)
                iy = ((w[:, 1] - y0) / res).astype(int)
                ok = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
                U[iy[ok], ix[ok]] = True
            union[k] = U.sum() * res ** 2
        return ts, per, union


# ---------------------------------------------------------------------------
class View:
    """The figure: map panel + status column, redrawn in place for time t."""

    def __init__(self, fleet, tw0, tw1, map_kind='own', size=(19.2, 10.8), dpi=100, trail_s=None):
        self.f, self.tw0, self.tw1 = fleet, tw0, tw1
        self.map_kind = map_kind
        self.trail_s = trail_s
        self.rate = 1.0
        self.fig = plt.figure(figsize=size, dpi=dpi, facecolor=SURF)
        n = len(fleet.drones)
        gs = self.fig.add_gridspec(3, 2, width_ratios=[2.6, 1], height_ratios=[0.55 * n + 0.2, 1.0, 1.0],
                                   left=0.03, right=0.985, top=0.93, bottom=0.07, wspace=0.08, hspace=0.38)
        self.ax = self.fig.add_subplot(gs[:, 0])
        self.ax_tiles = self.fig.add_subplot(gs[0, 1])
        self.ax_tl = self.fig.add_subplot(gs[1, 1])
        self.ax_cov = self.fig.add_subplot(gs[2, 1])
        self.title = self.fig.text(0.03, 0.965, '', fontsize=15, color=INK, weight='bold', va='center')
        self.clock = self.fig.text(0.985, 0.965, '', fontsize=15, color=INK, ha='right', va='center', family='monospace')
        self._setup_map()
        self._setup_tiles()
        self._setup_timeline()
        self._setup_coverage()

    # ---- map ----------------------------------------------------------------
    def _setup_map(self):
        ax = self.ax
        x0, x1, y0, y1 = self.f.extent(self.tw0, self.tw1)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_aspect('equal')
        ax.set_facecolor(SURF)
        ax.grid(True, color=GRIDC, lw=0.6)
        ax.set_axisbelow(True)
        ax.tick_params(colors=INK2, labelsize=9)
        for s in ax.spines.values():
            s.set_color(GRIDC)
        if DISPLAY_ROT[0]:
            ax.set_xlabel('-y  [m]   (team frame, forward = up)', color=INK2, fontsize=9)
            ax.set_ylabel('x  [m]  forward', color=INK2, fontsize=9)
        else:
            ax.set_xlabel('x  [m]  (team frame, forward = right)', color=INK2, fontsize=9)
            ax.set_ylabel('y  [m]', color=INK2, fontsize=9)
        self.art = {}
        for d in self.f.drones:
            c = d.color
            # occupancy: 0 unknown (transparent), 1 free (tint), 2 occupied (dark)
            cmap = ListedColormap([(0, 0, 0, 0), to_rgba(c, 0.18), to_rgba(c, 0.95)])
            im = ax.imshow(np.zeros((1, 1), np.uint8), cmap=cmap, vmin=0, vmax=2, origin='lower',
                           interpolation='nearest', extent=(0, 1, 0, 1), zorder=1)
            trail, = ax.plot([], [], color=c, lw=1.6, alpha=0.85, zorder=4, solid_capstyle='round')
            path, = ax.plot([], [], color=c, lw=1.2, ls='-', alpha=0.6, zorder=5)
            goal, = ax.plot([], [], marker='*', ms=14, color=c, mec=SURF, mew=0.8, ls='', zorder=7)
            claim, = ax.plot([], [], color=c, lw=1.0, ls='--', alpha=0.9, zorder=5)
            fr = ax.scatter([], [], marker='x', s=18, color=c, alpha=0.45, lw=0.8, zorder=3)
            body = Circle((0, 0), 0.45, fc=c, ec=SURF, lw=1.5, zorder=9)
            body.set_visible(False)
            ax.add_patch(body)
            head, = ax.plot([], [], color=SURF, lw=2.2, zorder=10, solid_capstyle='round')
            lab = ax.text(0, 0, d.letter, ha='center', va='center', fontsize=9, weight='bold', color=SURF, zorder=11)
            lab.set_visible(False)
            gf, = ax.plot([], [], color=c, lw=1.0, ls=(0, (6, 4)), alpha=0.7, zorder=2)
            beliefs = {}
            for other in self.f.drones:
                if other is d:
                    continue
                ring = Circle((0, 0), 0.5, fc='none', ec=other.color, lw=1.4, ls=':', zorder=8)
                ring.set_visible(False)
                ax.add_patch(ring)
                tie, = ax.plot([], [], color=other.color, lw=0.8, ls=':', alpha=0.8, zorder=6)
                txt = ax.text(0, 0, d.letter, ha='center', va='center', fontsize=7, color=other.color, zorder=8)
                txt.set_visible(False)
                beliefs[other.vid] = (ring, tie, txt)
            self.art[d.name] = dict(im=im, trail=trail, path=path, goal=goal, claim=claim, fr=fr, body=body,
                                    head=head, lab=lab, gf=gf, beliefs=beliefs, grid_t=None)
        # pairwise link lines
        self.linklines = {}
        ds = self.f.drones
        for i in range(len(ds)):
            for j in range(i + 1, len(ds)):
                ln, = ax.plot([], [], lw=2.2, alpha=0.75, zorder=3.5, solid_capstyle='round')
                tx = ax.text(0, 0, '', fontsize=7, ha='center', va='center', color=INK2, zorder=3.6,
                             bbox=dict(boxstyle='round,pad=0.15', fc=SURF, ec='none', alpha=0.8))
                self.linklines[(ds[i].vid, ds[j].vid)] = (ln, tx)
        # legend (identity never colour-alone: letter + name)
        handles = [Line2D([], [], marker='o', ms=9, mfc=d.color, mec=SURF, ls='', label=f'{d.letter}  {d.name} (id {d.vid})')
                   for d in ds]
        handles += [Line2D([], [], color=STATUS['good'], lw=2.2, label='LoRa link: delivery ≥ 70 %'),
                    Line2D([], [], color=STATUS['warning'], lw=2.2, label='LoRa link: 30–70 %'),
                    Line2D([], [], color=STATUS['critical'], lw=2.2, label='LoRa link: < 30 % / stale'),
                    Line2D([], [], marker='o', mfc='none', mec=INK2, ls=':', color=INK2, label='peer as believed over LoRa'),
                    Line2D([], [], marker='*', ms=11, color=INK2, ls='', label='planner goal'),
                    Line2D([], [], marker='x', color=INK2, ls='', label='frontier centroid'),
                    Line2D([], [], color=INK2, ls=(0, (6, 4)), label='geofence')]
        ax.legend(handles=handles, loc='upper left', fontsize=8, frameon=True, facecolor=SURF, edgecolor=GRIDC,
                  labelcolor=INK, ncol=1)

    def _update_map(self, t):
        f = self.f
        poses = {}
        for d in f.drones:
            A = self.art[d.name]
            alive = d.alive(t)
            # occupancy grid, only re-blit when the snapshot changes
            g = d.grid(t, shared=(self.map_kind == 'shared'))
            if g is not None and A['grid_t'] != g[0]:
                self.blit_grid(d, g)
            i = d.idx(t)
            if alive and i is not None:
                i0 = 0
                if self.trail_s:
                    i0 = np.searchsorted(d.od_t, t - self.trail_s)
                A['trail'].set_data(d.world_xy[i0:i + 1, 0], d.world_xy[i0:i + 1, 1])
                xy, yaw, z = d.pose(t)
                poses[d.vid] = xy
                A['body'].center = (xy[0], xy[1])
                A['body'].set_visible(True)
                A['body'].set_alpha(1.0 if d.is_offboard(t) else 0.55)
                hx = xy + 0.75 * np.array([math.cos(yaw), math.sin(yaw)])
                A['head'].set_data([xy[0], hx[0]], [xy[1], hx[1]])
                A['lab'].set_position((xy[0], xy[1]))
                A['lab'].set_visible(True)
            else:
                A['body'].set_visible(False)
                A['lab'].set_visible(False)
                A['head'].set_data([], [])
            p = d.plan(t)
            if p is not None and alive:
                _, goal, path, wps, gf, bpath = p
                if goal is not None and d.state(t) == 'EXECUTE':
                    gw = d.to_world(t, goal[:2])
                    A['goal'].set_data([gw[0]], [gw[1]])
                    pw = d.to_world_many(t, path)
                    A['path'].set_data(pw[:, 0], pw[:, 1])
                else:
                    A['goal'].set_data([], [])
                    A['path'].set_data([], [])
                if len(gf):
                    gw = d.to_world_many(t, gf)
                    A['gf'].set_data(gw[:, 0], gw[:, 1])
            else:
                A['goal'].set_data([], [])
                A['path'].set_data([], [])
            fr = d.frontier(t)
            if fr is not None and alive and len(fr[1]):
                A['fr'].set_offsets(d.to_world_many(t, fr[1]))
            else:
                A['fr'].set_offsets(np.zeros((0, 2)))
            c = d.claim(t)
            if c is not None and alive and d.vid in poses:
                cw = d.to_world(t, c[2:4])
                A['claim'].set_data([poses[d.vid][0], cw[0]], [poses[d.vid][1], cw[1]])
            else:
                A['claim'].set_data([], [])
        # beliefs: where d thinks each peer is (already team frame in the packet)
        for d in f.drones:
            A = self.art[d.name]
            bel = d.peer_beliefs(t) if d.alive(t) else {}
            for pid, (ring, tie, txt) in A['beliefs'].items():
                row = bel.get(pid)
                if row is None:
                    ring.set_visible(False)
                    txt.set_visible(False)
                    tie.set_data([], [])
                    continue
                bx, by = d.disp(row[3:5])
                ring.center = (bx, by)
                ring.set_visible(True)
                txt.set_position((bx, by))
                txt.set_visible(True)
                if pid in poses:
                    tie.set_data([bx, poses[pid][0]], [by, poses[pid][1]])
                else:
                    tie.set_data([], [])
        # links: colour by the WORSE of the two directions' delivery ratio
        for (a, b), (ln, tx) in self.linklines.items():
            da, db = f.by_id[a], f.by_id[b]
            if a not in poses or b not in poses:
                ln.set_data([], [])
                tx.set_text('')
                continue
            _, la = da.link(t)
            _, lb = db.link(t)
            vals = [v[0] for v in (la.get(b), lb.get(a)) if v is not None]
            ages = [v[2] for v in (la.get(b), lb.get(a)) if v is not None]
            pa, pb = poses[a], poses[b]
            if not vals:
                ln.set_data([pa[0], pb[0]], [pa[1], pb[1]])
                ln.set_color(GRIDC)
                ln.set_linestyle(':')
                tx.set_position(((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2))
                tx.set_text('no LoRa')
                continue
            dr = min(vals)
            col = STATUS['good'] if dr >= 0.7 else STATUS['warning'] if dr >= 0.3 else STATUS['critical']
            if max(ages) > 10:
                col = STATUS['critical']
            ln.set_data([pa[0], pb[0]], [pa[1], pb[1]])
            ln.set_color(col)
            ln.set_linestyle('-')
            tx.set_position(((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2))
            tx.set_text(f'{dr * 100:.0f} %  age {max(ages):.0f}s')

    def blit_grid(self, d, g):
        A = self.art[d.name]
        A['grid_t'] = g[0]
        _, ox, oy, res, G = g
        h, w = G.shape
        A['im'].set_data(G)
        A['im'].set_extent((ox, ox + w * res, oy, oy + h * res))
        yaw, tx, ty = d.frame(g[0])
        A['im'].set_transform(mtr.Affine2D().rotate(yaw).translate(tx, ty) + self.ax.transData)

    # ---- status tiles -------------------------------------------------------
    def _setup_tiles(self):
        ax = self.ax_tiles
        ax.set_axis_off()
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        n = len(self.f.drones)
        self.tiles = {}
        h = 1.0 / n
        for k, d in enumerate(self.f.drones):
            y1 = 1 - k * h
            ax.add_patch(Rectangle((0, y1 - h + 0.015), 1, h - 0.03, fc=SURF, ec=GRIDC, lw=1, zorder=1))
            ax.add_patch(Rectangle((0, y1 - h + 0.015), 0.03, h - 0.03, fc=d.color, ec='none', zorder=2))
            name = ax.text(0.06, y1 - 0.035, f'{d.letter}  {d.name}', fontsize=11, weight='bold', color=INK, va='top')
            mode = ax.text(0.98, y1 - 0.035, '', fontsize=10, weight='bold', va='top', ha='right')
            body = ax.text(0.06, y1 - 0.035 - 0.26 * h, '', fontsize=9, color=INK2, va='top', family='monospace',
                           linespacing=1.3)
            self.tiles[d.name] = (mode, body)

    def _update_tiles(self, t):
        for d in self.f.drones:
            mode, body = self.tiles[d.name]
            if not d.alive(t):
                mode.set_text('NO DATA')
                mode.set_color(INK2)
                body.set_text('bag not recording at this time')
                continue
            ob = d.is_offboard(t)
            armed = d.csv_at(d.sp, t, 'armed')
            if ob:
                mode.set_text('OFFBOARD')
                mode.set_color(STATUS['good'])
            elif armed:
                mode.set_text('ARMED / manual')
                mode.set_color(STATUS['warning'])
            else:
                mode.set_text('on ground' if armed is not None else 'PX4: n/a')
                mode.set_color(INK2)
            xy, yaw, z = d.pose(t)
            st = d.state(t)
            gd = d.guard(t)
            up, lk = d.link(t)
            bel = d.peer_beliefs(t)
            ned_z = d.csv_at(d.sp, t, 'ned_z')
            alt = f'{-ned_z:4.1f} m' if ned_z is not None and not np.isnan(ned_z) else '  n/a'
            vio = d.csv_at(d.vio, t, 'vio_hz', maxage=2.5)
            vio_s = f'{vio:3.0f} Hz' if vio is not None and not np.isnan(vio) else ' n/a'
            esdf_age = d.csv_at(d.sp, t, 'esdf_age')
            px4 = 'LIVE' if esdf_age is not None else 'n/a'
            i = d.idx(t)
            lora = d.lora_rx_count(t - 10, t)
            lk_s = ' '.join(f'{NAMES[p][0].upper()}:{v[0] * 100:.0f}%' for p, v in sorted(lk.items())) or '-'
            peers_s = ' '.join(f'{NAMES[p][0].upper()}({r[16]:.0f}s)' for p, r in sorted(bel.items())) or 'none'
            goal = d.plan(t)
            goal_s = ''
            if goal and goal[1] is not None and st == 'EXECUTE':
                gw = d.to_world(t, goal[1][:2])
                goal_s = f' -> ({gw[0]:.0f},{gw[1]:.0f})'
            body.set_text(
                f'planner  {st:8s}{goal_s}\n'
                f'guard    {gd:8s}  spd {d.speed(t):3.1f} m/s  alt {alt}\n'
                f'pos      ({xy[0]:6.1f},{xy[1]:6.1f})  flown {d.path_len[i]:5.1f} m\n'
                # LinkStats.lora_bridge_up is really "peer table non-empty"
                # (coordination_node.py), not the serial port state
                f'VIO {vio_s}   LoRa peers {"heard" if up else "NONE" if up is not None else "n/a"}  '
                f'rx(10s) {lora:3d}\n'
                f'link     {lk_s}\n'
                f'peers    {peers_s}')

    # ---- timeline strip -----------------------------------------------------
    def _setup_timeline(self):
        ax = self.ax_tl
        ax.set_facecolor(SURF)
        ds = self.f.drones
        n = len(ds)
        ax.set_xlim(0, self.tw1 - self.tw0)
        ax.set_ylim(-0.5, n - 0.5)
        ax.set_yticks(range(n))
        ax.set_yticklabels([d.name for d in ds][::-1], fontsize=8, color=INK2)
        ax.tick_params(axis='x', labelsize=8, colors=INK2)
        ax.set_xlabel('seconds since window start', fontsize=8, color=INK2)
        for s in ax.spines.values():
            s.set_color(GRIDC)
        ax.grid(True, axis='x', color=GRIDC, lw=0.6)
        ax.set_axisbelow(True)
        for k, d in enumerate(ds):
            y = n - 1 - k
            for a, b in d.offboard:
                ax.add_patch(Rectangle((a - self.tw0, y - 0.32), b - a, 0.64, fc=to_rgba(d.color, 0.35), ec='none'))
            # guard STOP events and LoRa packets received
            stops = [ev[0] - self.tw0 for ev in d.guard_ev if ev[1] != 'PASS' and self.tw0 <= ev[0] <= self.tw1]
            ax.plot(stops, [y + 0.2] * len(stops), '|', color=STATUS['serious'], ms=7, mew=1.2)
            if d.lora is not None:
                lt = d.lora['t']
                lt = lt[(lt >= self.tw0) & (lt <= self.tw1)] - self.tw0
                ax.plot(lt, [y - 0.22] * len(lt), '|', color=INK2, ms=5, mew=0.6, alpha=0.6)
            claims = d.claims[(d.claims[:, 0] >= self.tw0) & (d.claims[:, 0] <= self.tw1) & (d.claims[:, 1] > 0), 0] - self.tw0 \
                if len(d.claims) else []
            ax.plot(claims, [y] * len(claims), '.', color=d.color, ms=3)
        self.cursor = ax.axvline(0, color=INK, lw=1.2)
        ax.set_title('Offboard (bar) · guard STOP (|, above) · LoRa rx (|, below) · claim (·)',
                     fontsize=8, color=INK2, loc='left')

    # ---- coverage -----------------------------------------------------------
    def _setup_coverage(self):
        ax = self.ax_cov
        ax.set_facecolor(SURF)
        ts, per, union = self.f.coverage_curve(self.tw0, self.tw1)
        self.cov = (ts, per, union)
        for d in self.f.drones:
            ax.plot(ts - self.tw0, per[d.name], color=d.color, lw=1.6, label=d.name)
        ax.plot(ts - self.tw0, union, color=INK, lw=1.2, ls='--', label='team (union)')
        ax.set_xlim(0, self.tw1 - self.tw0)
        ax.set_ylabel('mapped area [m²]', fontsize=8, color=INK2)
        ax.set_xlabel('seconds since window start', fontsize=8, color=INK2)
        ax.tick_params(labelsize=8, colors=INK2)
        for s in ax.spines.values():
            s.set_color(GRIDC)
        ax.grid(True, color=GRIDC, lw=0.6)
        ax.set_axisbelow(True)
        ax.legend(fontsize=7, frameon=False, labelcolor=INK, ncol=5, loc='lower left', bbox_to_anchor=(0, 1.0))
        self.cov_cursor = ax.axvline(0, color=INK, lw=1.2)
        self.cov_text = ax.text(0.99, 0.05, '', transform=ax.transAxes, ha='right', fontsize=8, color=INK)

    # ---- frame --------------------------------------------------------------
    def render(self, t):
        self._update_map(t)
        self._update_tiles(t)
        self.cursor.set_xdata([t - self.tw0, t - self.tw0])
        self.cov_cursor.set_xdata([t - self.tw0, t - self.tw0])
        ts, per, union = self.cov
        k = min(len(ts) - 1, max(0, int((t - self.tw0) / (ts[1] - ts[0])) if len(ts) > 1 else 0))
        self.cov_text.set_text(f'team {union[k]:.0f} m²')
        self.title.set_text(f'{self.f.run}  ·  {", ".join(d.name for d in self.f.drones)}  ·  map: {self.map_kind}')
        rate = f'  ×{self.rate:g}' if self.rate != 1.0 else ''
        self.clock.set_text(f'{hhmmss(t)}   T+{t - self.tw0:6.1f} s{rate}')


# ---------------------------------------------------------------------------
def run_interactive(view):
    import matplotlib.widgets as W
    fig = view.fig
    axs = fig.add_axes([0.03, 0.015, 0.6, 0.025])
    sl = W.Slider(axs, '', 0.0, view.tw1 - view.tw0, valinit=0.0, color=INK2)
    sl.valtext.set_visible(False)
    state = {'t': view.tw0, 'play': False, 'last': None}

    def draw(t):
        state['t'] = min(view.tw1, max(view.tw0, t))
        view.render(state['t'])
        sl.eventson = False
        sl.set_val(state['t'] - view.tw0)
        sl.eventson = True
        fig.canvas.draw_idle()

    sl.on_changed(lambda v: draw(view.tw0 + v))

    def key(ev):
        k = ev.key
        if k == ' ':
            state['play'] = not state['play']
            state['last'] = None
        elif k == 'right':
            draw(state['t'] + 1)
        elif k == 'left':
            draw(state['t'] - 1)
        elif k == ']':
            draw(state['t'] + 10)
        elif k == '[':
            draw(state['t'] - 10)
        elif k in ('+', '='):
            view.rate *= 2
        elif k == '-':
            view.rate /= 2
        elif k == 's':
            out = f'frame_{hhmmss(state["t"]).replace(":", "")}.png'
            fig.savefig(out, dpi=150, facecolor=SURF)
            print('saved', out)
        elif k == 'q':
            plt.close(fig)

    fig.canvas.mpl_connect('key_press_event', key)

    import time

    def tick():
        if not state['play']:
            return
        now = time.monotonic()
        if state['last'] is not None:
            draw(state['t'] + (now - state['last']) * view.rate)
            if state['t'] >= view.tw1:
                state['play'] = False
        state['last'] = now

    timer = fig.canvas.new_timer(interval=66)
    timer.add_callback(tick)
    timer.start()
    draw(view.tw0)
    print(__doc__.split('Interactive keys:')[1].split('WHAT IS DRAWN')[0])
    plt.show()


def run_mp4(view, out, fps, rate):
    from matplotlib.animation import FFMpegWriter
    view.rate = rate
    n = int((view.tw1 - view.tw0) / rate * fps)
    w = FFMpegWriter(fps=fps, codec='libx264', bitrate=6000,
                     extra_args=['-pix_fmt', 'yuv420p', '-preset', 'medium'])
    print(f'rendering {n} frames at {fps} fps (x{rate}) -> {out}')
    with w.saving(view.fig, out, dpi=view.fig.dpi):
        for k in range(n + 1):
            view.render(view.tw0 + k / fps * rate)
            w.grab_frame(facecolor=SURF)
            if k % (fps * 10) == 0:
                print(f'  {k}/{n}  T+{k / fps * rate:.0f}s', flush=True)
    print('done', out)


def run_figures(fleet, tw0, tw1, outdir, map_kind):
    """Static, paper-ready figures (PDF + PNG) for the window."""
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.size': 9, 'axes.edgecolor': GRIDC, 'xtick.color': INK2, 'ytick.color': INK2})

    flown = [d for d in fleet.drones if any(b > tw0 and a < tw1 for a, b in d.offboard)] or fleet.drones
    # the map is taken at the end of the last Offboard segment, not at the window
    # end: the landing VIO jump (buckshee 2.7 m, 2026-10-23) smears the final slice
    # ... and the slice can shrink around landing (delta's own map lost 30 % at
    # 15:30:57, 4 s after Offboard ended), so take the instant of maximum team
    # coverage within the span: the best map the fleet had.
    ob_end = [min(b, tw1) for d in flown for a, b in d.offboard if b > tw0 and a < tw1]
    t_map = max(ob_end) if ob_end else tw1
    cts, _, cunion = fleet.coverage_curve(tw0, t_map)
    if len(cts):
        t_map = float(cts[int(np.argmax(cunion))])

    # 1. final team map + full trajectories
    v = View(fleet, tw0, tw1, map_kind, size=(10, 7.5), dpi=150)
    v.render(t_map)
    # a clean map: slices, trajectories, geofence. No live-state clutter.
    for d in fleet.drones:
        A = v.art[d.name]
        for k in ('goal', 'path', 'claim', 'head'):
            A[k].set_data([], [])
        A['fr'].set_offsets(np.zeros((0, 2)))
        A['body'].set_visible(False)
        A['lab'].set_visible(False)
        for ring, tie, txt in A['beliefs'].values():
            ring.set_visible(False)
            txt.set_visible(False)
            tie.set_data([], [])
    for ln, tx in v.linklines.values():
        ln.set_data([], [])
        tx.set_text('')
    for d in fleet.drones:
        A = v.art[d.name]
        if d not in flown:
            # a drone on the ground: its slice is its parking spot and its
            # "trajectory" is VIO teleports (buckshee 2026-10-23: a 60 m line)
            A['trail'].set_data([], [])
            A['gf'].set_data([], [])
            A['im'].set_visible(False)
            continue
        # each drone's own fullest slice of the span (its VIO may have stopped
        # before t_map - delta's did at landing - which would blank it)
        g = d.best_grid(tw0, tw1) if map_kind == 'own' else None
        if g is not None:
            v.blit_grid(d, g)
        ob = [(max(a, tw0), min(b, tw1)) for a, b in d.offboard if b > tw0 and a < tw1] or [(tw0, tw1)]
        m = (d.od_t >= ob[0][0]) & (d.od_t <= ob[-1][1])
        if m.any():
            A['trail'].set_data(d.world_xy[m, 0], d.world_xy[m, 1])
            v.ax.plot(d.world_xy[m][0, 0], d.world_xy[m][0, 1], marker='^', ms=9, color=d.color, mec=SURF, zorder=8)
            v.ax.plot(d.world_xy[m][-1, 0], d.world_xy[m][-1, 1], marker='s', ms=7, color=d.color, mec=SURF, zorder=8)
    x0, x1, y0, y1 = fleet.extent(tw0, tw1, flown)
    v.ax.set_xlim(x0, x1)
    v.ax.set_ylim(y0, y1)
    handles = [Line2D([], [], marker='o', ms=8, mfc=d.color, mec=SURF, ls='', label=f'{d.letter}  {d.name} (id {d.vid})')
               for d in flown]
    handles += [Line2D([], [], marker='^', ms=8, color=INK2, ls='', label='Offboard start'),
                Line2D([], [], marker='s', ms=7, color=INK2, ls='', label='Offboard end'),
                Line2D([], [], color=INK2, ls=(0, (6, 4)), label='geofence'),
                Rectangle((0, 0), 1, 1, fc=to_rgba(INK2, 0.18), ec='none', label='free (light) / occupied (dark)')]
    v.ax.legend(handles=handles, loc='best', fontsize=8, frameon=True, facecolor=SURF, edgecolor=GRIDC, labelcolor=INK)
    for a in (v.ax_tiles, v.ax_tl, v.ax_cov):
        a.remove()
    v.ax.set_position([0.07, 0.07, 0.9, 0.86])
    v.title.set_text(f'{fleet.run}: team map and trajectories  {hhmmss(tw0)}–{hhmmss(t_map)}')
    v.clock.set_text('')
    for ext in ('pdf', 'png'):
        v.fig.savefig(outdir / f'team_map.{ext}', facecolor=SURF)
    plt.close(v.fig)

    # 2. coverage curve
    ts, per, union = fleet.coverage_curve(tw0, tw1)
    fig, ax = plt.subplots(figsize=(6.5, 3.2), dpi=150, facecolor=SURF)
    for d in fleet.drones:
        ax.plot(ts - tw0, per[d.name], color=d.color, lw=1.8, label=d.name)
        for a, b in d.offboard:
            ax.axvspan(max(a, tw0) - tw0, min(b, tw1) - tw0, color=to_rgba(d.color, 0.08), lw=0)
    ax.plot(ts - tw0, union, color=INK, lw=1.4, ls='--', label='team (union)')
    ax.set_xlabel('time since window start [s]')
    ax.set_ylabel('mapped area [m²]')
    ax.grid(True, color=GRIDC, lw=0.6)
    ax.set_axisbelow(True)
    ax.legend(frameon=False, ncol=3, fontsize=8)
    ax.set_title('Mapped area per drone (shaded: Offboard)', loc='left', fontsize=9, color=INK2)
    fig.tight_layout()
    for ext in ('pdf', 'png'):
        fig.savefig(outdir / f'coverage.{ext}', facecolor=SURF)
    plt.close(fig)

    # 3. comms: per-drone LoRa delivery ratio per peer + rx packets + offboard
    n = len(fleet.drones)
    fig, axs = plt.subplots(n, 1, figsize=(6.5, 1.6 * n + 0.6), dpi=150, sharex=True, facecolor=SURF)
    axs = np.atleast_1d(axs)
    for ax, d in zip(axs, fleet.drones):
        for a, b in d.offboard:
            ax.axvspan(max(a, tw0) - tw0, min(b, tw1) - tw0, color=to_rgba(d.color, 0.10), lw=0)
        if len(d.links):
            for pid in sorted(set(d.links[:, 2].astype(int)) - {0}):
                m = (d.links[:, 2] == pid) & (d.links[:, 0] >= tw0) & (d.links[:, 0] <= tw1)
                ax.plot(d.links[m, 0] - tw0, d.links[m, 3] * 100, color=COLOR[NAMES[pid]], lw=1.4,
                        label=f'from {NAMES[pid]}')
        if d.lora is not None:
            lt = d.lora['t']
            lt = lt[(lt >= tw0) & (lt <= tw1)] - tw0
            ax.plot(lt, np.full(len(lt), -6), '|', color=INK2, ms=5, mew=0.6, alpha=0.7)
        ax.set_ylim(-12, 105)
        ax.set_ylabel('delivery %')
        ax.set_title(f'{d.name}: LoRa delivery ratio per peer (ticks: packets received)', loc='left', fontsize=8, color=INK2)
        ax.grid(True, color=GRIDC, lw=0.6)
        ax.set_axisbelow(True)
        ax.legend(frameon=False, fontsize=7, ncol=3, loc='upper right')
    axs[-1].set_xlabel('time since window start [s]')
    fig.tight_layout()
    for ext in ('pdf', 'png'):
        fig.savefig(outdir / f'comms.{ext}', facecolor=SURF)
    plt.close(fig)

    # 4. per-drone path coloured by planner state, with guard stops (flying drones
    #    only, over their Offboard span: a drone on the ground contributes nothing
    #    but its VIO teleports)
    n = len(flown)
    fig, axs = plt.subplots(1, n, figsize=(3.4 * n, 3.6), dpi=150, facecolor=SURF)
    axs = np.atleast_1d(axs)
    for ax, d in zip(axs, flown):
        ob = [(max(a, tw0), min(b, tw1)) for a, b in d.offboard if b > tw0 and a < tw1] or [(tw0, tw1)]
        m = (d.od_t >= ob[0][0]) & (d.od_t <= ob[-1][1])
        xy = d.world_xy[m]
        tt = d.od_t[m]
        for i in range(0, len(xy) - 1, 3):
            st = d.state(tt[i])
            col = d.color if st == 'EXECUTE' else INK2 if st == 'PLAN' else GRIDC
            ax.plot(xy[i:i + 4, 0], xy[i:i + 4, 1], color=col, lw=1.4)
        stops = [ev[0] for ev in d.guard_ev if ev[1] != 'PASS' and tw0 <= ev[0] <= tw1]
        for s in stops:
            p = d.pose(s)
            if p is not None:
                ax.plot(p[0][0], p[0][1], marker='o', ms=4, mfc='none', mec=STATUS['serious'])
        if len(xy):
            ax.plot(xy[0, 0], xy[0, 1], marker='^', ms=7, color=d.color, mec=SURF)
        ax.set_aspect('equal')
        ax.grid(True, color=GRIDC, lw=0.6)
        ax.set_axisbelow(True)
        ax.set_title(f'{d.name}: {d.path_len[m][-1] - d.path_len[m][0] if m.any() else 0:.0f} m flown', loc='left', fontsize=9)
    h = [Line2D([], [], color=INK, lw=1.4, label='EXECUTE (colour = drone)'),
         Line2D([], [], color=INK2, lw=1.4, label='PLAN'),
         Line2D([], [], marker='o', mfc='none', mec=STATUS['serious'], ls='', label='guard STOP')]
    fig.legend(handles=h, loc='lower center', ncol=3, frameon=False, fontsize=8)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    for ext in ('pdf', 'png'):
        fig.savefig(outdir / f'paths_by_state.{ext}', facecolor=SURF)
    plt.close(fig)

    # 5. event table (CSV) for the caption / text
    with open(outdir / 'events.csv', 'w') as fh:
        fh.write('t_epoch,clock,drone,kind,detail\n')
        rows = []
        for d in fleet.drones:
            for a, b in d.offboard:
                rows += [(a, d.name, 'offboard_on', ''), (b, d.name, 'offboard_off', f'{b - a:.0f}s')]
            rows += [(t, d.name, 'planner', s) for t, s in d.state_ev]
            rows += [(t, d.name, 'guard', s) for t, s in d.guard_ev if s != 'PASS']
            if len(d.claims):
                for c in d.claims:
                    if c[1] > 0:
                        rows.append((c[0], d.name, 'claim', f'({c[2]:.1f},{c[3]:.1f}) gain {c[5]:.0f}'
                                     + (' reached' if c[6] else '')))
        for t, dn, k, det in sorted(rows):
            if tw0 <= t <= tw1:
                fh.write(f'{t:.3f},{hhmmss(t)},{dn},{k},{det}\n')
    print('figures ->', outdir, sorted(p.name for p in outdir.iterdir()))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('drones', nargs='*')
    ap.add_argument('--bags', default=str(ROOT / 'bags'))
    ap.add_argument('--from', dest='t_from')
    ap.add_argument('--to', dest='t_to')
    ap.add_argument('--map', default='own', choices=['own', 'shared'])
    ap.add_argument('--rot', default='auto', help="display rotation in deg ('auto': forward=up when the area is deep)")
    ap.add_argument('--trail', type=float, default=None, help='trail length in s (default: whole flight)')
    ap.add_argument('--mp4')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--rate', type=float, default=1.0)
    ap.add_argument('--dpi', type=int, default=100)
    ap.add_argument('--frame', help='render one instant (HH:MM:SS or epoch)')
    ap.add_argument('--save', help='file for --frame (png/pdf/svg)')
    ap.add_argument('--figures', help='output dir for the static figure set')
    a = ap.parse_args()

    fleet = Fleet(a.run, a.drones, a.bags, a.rot)
    tw0, tw1 = fleet.default_window()
    if a.t_from:
        tw0 = parse_t(a.t_from, fleet.t0)
    if a.t_to:
        tw1 = parse_t(a.t_to, fleet.t0)
    tw0, tw1 = max(tw0, fleet.t0), min(tw1, fleet.t1)
    print(f'window {hhmmss(tw0)} - {hhmmss(tw1)} ({tw1 - tw0:.0f} s); drones: '
          + ', '.join(f'{d.name}[{hhmmss(d.tl["t_start"])}-{hhmmss(d.tl["t_end"])}]' for d in fleet.drones))

    if a.figures:
        matplotlib.use('Agg')
        run_figures(fleet, tw0, tw1, a.figures, a.map)
        return
    if a.mp4 or a.frame:
        matplotlib.use('Agg')
    view = View(fleet, tw0, tw1, a.map, dpi=a.dpi, trail_s=a.trail)
    if a.frame:
        t = parse_t(a.frame, fleet.t0)
        view.render(t)
        out = a.save or f'frame_{hhmmss(t).replace(":", "")}.png'
        view.fig.savefig(out, dpi=150, facecolor=SURF)
        print('saved', out)
    elif a.mp4:
        run_mp4(view, a.mp4, a.fps, a.rate)
    else:
        run_interactive(view)


if __name__ == '__main__':
    main()
