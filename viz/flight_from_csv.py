#!/usr/bin/env python3
"""flight_from_csv.py - figures for a flight that has no bag: trajectories,
altitude and the LoRa link, from the CSVs every drone writes anyway.

    ./viz/flight_from_csv.py 20261113_1945 delta ghost --out reports/visualization/f1113_flight1
        [--offset ghost=0,-4]     start offset of a drone in the team frame (x forward, y left), m

<stamp> is the UTC date_time prefix shared by the session's CSVs in
collected/<drone>/ (splanner_<stamp>*.csv, lora_rx_<stamp>*.csv, link_..., guard_...).

WHAT THE TRACK IS
The planner logs its pose from the raw cuVSLAM odometry (flu_x, flu_y). That
frame is pitched nose-down by the camera's attitude at start (theta0), so a
level forward distance d is logged as d*cos(theta0). The same log carries
PX4's position, whose forward axis is short by cos(theta0 + 20 deg) (the
vision input to PX4 is rotated the wrong way by the 20 deg mount angle). The
ratio of the two gives theta0 per drone per flight:

    ned_x / flu_x = cos(theta0 + 20) / cos(theta0) = cos 20 - sin 20 * tan(theta0)

and the track drawn here is (flu_x / cos(theta0), flu_y). Checked on
2026-11-13 flight 2, where the bag gives theta0 directly from the flight
plane: delta 24.3 deg (this method 24.5), ghost 28.7 deg.
"""
import argparse
import csv
import datetime as dt
import glob
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from fleet_player import COLOR, INK, INK2, SURF, GRIDC, STATUS  # noqa: E402

ROOT = HERE.parent


def load(path):
    rows = []
    with open(path, errors='ignore') as f:
        r = csv.reader(l.replace('\0', '') for l in f)
        h = next(r)
        for x in r:
            if len(x) == len(h):
                rows.append(x)
    return h, rows


def col(h, rows, name, cast=float):
    i = h.index(name)
    out = []
    for r in rows:
        try:
            out.append(cast(r[i]))
        except ValueError:
            out.append(np.nan)
    return np.array(out)


def one(drone, kind, stamp):
    fs = sorted(glob.glob(str(ROOT / 'collected' / drone / f'{kind}_{stamp}*.csv')))
    return load(fs[-1]) if fs else (None, None)


def hhmmss(t):
    return dt.datetime.fromtimestamp(t).strftime('%H:%M:%S')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('stamp')
    ap.add_argument('drones', nargs='+')
    ap.add_argument('--out', required=True)
    ap.add_argument('--offset', action='append', default=[], help='drone=x,y start offset in the team frame')
    ap.add_argument('--title')
    a = ap.parse_args()
    off = {k: tuple(float(v) for v in xy.split(',')) for k, xy in (o.split('=') for o in a.offset)}
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    D = {}
    for d in a.drones:
        h, rows = one(d, 'splanner', a.stamp)
        if h is None:
            sys.exit(f'no splanner_{a.stamp}*.csv for {d}')
        t = col(h, rows, 'timestamp')
        fx, fy, nx, nz = (col(h, rows, k) for k in ('flu_x', 'flu_y', 'ned_x', 'ned_z'))
        offb = col(h, rows, 'offboard') > 0
        armed = col(h, rows, 'armed') > 0
        age = col(h, rows, 'px4_age')
        fresh = np.isfinite(age) & (age < 1.0)
        # theta0 from ned_x against flu_x while flying with a live PX4 feed
        m = offb & fresh & np.isfinite(fx) & np.isfinite(nx)
        if m.sum() > 200:
            A = np.column_stack([fx[m], np.ones(m.sum())])
            (k, _), *_ = np.linalg.lstsq(A, nx[m], rcond=None)
        else:
            k = math.cos(math.radians(20))
        tan0 = (math.cos(math.radians(20)) - k) / math.sin(math.radians(20))
        th0 = math.atan(max(0.0, tan0))
        ox, oy = off.get(d, (0.0, 0.0))
        x, y = fx / math.cos(th0) + ox, fy + oy
        iv = []
        on = None
        for i in range(len(t)):
            if offb[i] and on is None:
                on = t[i]
            elif not offb[i] and on is not None:
                iv.append((on, t[i]))
                on = None
        if on is not None:
            iv.append((on, t[-1]))
        iv = [(p, q) for p, q in iv if q - p > 2.0]
        alt = -(nz - np.nanpercentile(nz[fresh], 98)) if fresh.any() else -nz
        air = armed & fresh & (alt > 0.3)
        step = np.r_[0.0, np.hypot(np.diff(x), np.diff(y))]
        dec = np.arange(0, len(t), 10)                      # 2 Hz: do not sum 20 Hz jitter
        dist = float(np.nansum(np.hypot(np.diff(x[dec]), np.diff(y[dec]))[offb[dec][1:]]))
        D[d] = dict(t=t, x=x, y=y, offb=offb, air=air, alt=alt, fresh=fresh, iv=iv, th0=math.degrees(th0), k=k,
                    dist=dist, step=step)
        h, rows = one(d, 'lora_rx', a.stamp)
        if h is not None:
            D[d]['lt'], D[d]['rssi'], D[d]['snr'] = col(h, rows, 't'), col(h, rows, 'rssi'), col(h, rows, 'snr')
        h, rows = one(d, 'link', a.stamp)
        if h is not None:
            D[d]['kt'], D[d]['deliv'] = col(h, rows, 't'), col(h, rows, 'delivery')
        print('%-10s theta0 %.1f deg (ned_x/flu_x %.3f)  Offboard %s  %.0f m in Offboard' % (
            d, D[d]['th0'], k, ', '.join('%s-%s' % (hhmmss(p), hhmmss(q)) for p, q in iv), dist))

    t0 = min(p for d in D.values() for p, _ in d['iv']) - 10
    t1 = max(q for d in D.values() for _, q in d['iv']) + 10
    plt.rcParams.update({'font.size': 9, 'axes.edgecolor': GRIDC, 'xtick.color': INK2, 'ytick.color': INK2,
                         'figure.facecolor': SURF, 'axes.facecolor': SURF, 'savefig.facecolor': SURF})

    # ---- trajectories ----
    fig, ax = plt.subplots(figsize=(7.2, 6.6))
    for d, v in D.items():
        m = (v['t'] >= t0) & (v['t'] <= t1)
        ax.plot(-v['y'][m], v['x'][m], color=COLOR[d], lw=0.9, alpha=0.35, zorder=2)
        for p, q in v['iv']:
            mm = (v['t'] >= p) & (v['t'] <= q)
            ax.plot(-v['y'][mm], v['x'][mm], color=COLOR[d], lw=2.0, zorder=3)
            ax.plot(-v['y'][mm][0], v['x'][mm][0], marker='^', ms=9, color=COLOR[d], mec=SURF, zorder=5)
            ax.plot(-v['y'][mm][-1], v['x'][mm][-1], marker='s', ms=7, color=COLOR[d], mec=SURF, zorder=5)
        mm = (v['t'] >= v['iv'][0][0]) & (v['t'] <= v['iv'][-1][1])
        j = int(np.argmax(v['x'][mm]))
        ax.annotate(d, (-v['y'][mm][j], v['x'][mm][j]), xytext=(6, 6), textcoords='offset points',
                    color=COLOR[d], fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(color=GRIDC, lw=0.6)
    ax.set_xlabel('to the right of the launch line  [m]')
    ax.set_ylabel('forward  [m]')
    ax.plot([], [], color=INK2, lw=2.0, label='Offboard (autonomous)')
    ax.plot([], [], color=INK2, lw=0.9, alpha=0.5, label='manual / on the ground')
    ax.plot([], [], marker='^', ms=8, color=INK2, ls='', label='Offboard start')
    ax.plot([], [], marker='s', ms=6, color=INK2, ls='', label='Offboard end')
    ax.legend(loc='upper left', frameon=False, fontsize=8)
    ttl = a.title or f'{a.stamp}: trajectories'
    ax.set_title(f'{ttl}   {hhmmss(t0 + 10)}–{hhmmss(t1 - 10)}', loc='left', fontsize=10, color=INK, fontweight='bold')
    fig.text(0.01, 0.005, 'planner pose (cuVSLAM), forward axis rescaled by 1/cos(theta0): ' +
             ', '.join('%s %.1f°' % (d, v['th0']) for d, v in D.items()) + ' · team frame from the start offsets',
             fontsize=6.5, color=INK2)
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    for ext in ('png', 'pdf'):
        fig.savefig(out / f'trajectories.{ext}', dpi=170)
    plt.close(fig)

    # ---- time series: altitude, RSSI, SNR, delivery ----
    fig, axs = plt.subplots(4, 1, figsize=(7.6, 7.4), sharex=True)
    rel = lambda t: np.asarray(t) - t0   # noqa: E731
    for d, v in D.items():
        m = (v['t'] >= t0) & (v['t'] <= t1) & v['fresh']
        axs[0].plot(rel(v['t'][m]), v['alt'][m], color=COLOR[d], lw=1.4, label=d)
        for p, q in v['iv']:
            axs[0].axvspan(p - t0, q - t0, color=COLOR[d], alpha=0.07, lw=0)
        if 'lt' in v:
            m = (v['lt'] >= t0) & (v['lt'] <= t1)
            sec = np.floor(v['lt'][m] - t0)
            us = np.unique(sec)
            peer = [o for o in D if o != d]
            lab = '%s → %s' % (peer[0] if peer else 'peer', d)
            axs[1].plot(us, [np.nanmedian(v['rssi'][m][sec == s]) for s in us], color=COLOR[d], lw=1.3, label=lab)
            axs[2].plot(us, [np.nanmedian(v['snr'][m][sec == s]) for s in us], color=COLOR[d], lw=1.3, label=lab)
        if 'kt' in v:
            m = (v['kt'] >= t0) & (v['kt'] <= t1)
            axs[3].plot(rel(v['kt'][m]), 100 * v['deliv'][m], color=COLOR[d], lw=1.4,
                        label='%s receiving' % d)
    for axx, yl in zip(axs, ('altitude, PX4  [m]', 'LoRa RSSI  [dBm]', 'LoRa SNR  [dB]', 'LoRa delivery  [%]')):
        axx.set_ylabel(yl)
        axx.grid(color=GRIDC, lw=0.6)
        axx.legend(loc='lower right', frameon=False, fontsize=7.5, ncol=2)
    axs[3].set_ylim(90, 100.5)
    axs[3].set_xlabel(f'time since {hhmmss(t0)}  [s]   (shaded: Offboard)')
    axs[0].set_title(f'{ttl}: altitude and coordination link', loc='left', fontsize=10, color=INK, fontweight='bold')
    fig.tight_layout()
    for ext in ('png', 'pdf'):
        fig.savefig(out / f'altitude_and_link.{ext}', dpi=170)
    plt.close(fig)
    _ = STATUS
    print('figures ->', out)


if __name__ == '__main__':
    main()
