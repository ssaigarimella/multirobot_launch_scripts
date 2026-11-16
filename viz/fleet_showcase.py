#!/usr/bin/env python3
"""fleet_showcase.py - one composite video of a multi-drone flight: the 3D map
as it grows, the trajectories, what the planner is doing, and the links.

    ./viz/fleet_showcase.py <run> [drones...] --out file.mp4
        [--from T --to T] [--rate 4] [--fps 30] [--size 1920x1080] [--jobs 3]
        [--hold-s 6] [--orbit-deg 150] [--elev-deg 33] [--azim-deg -150]
        [--stills-only T,T,...]

Built on fleet_3d_video.py (same data, same headless-Chrome capture, every
frame a pure function of its index) with a different page:

  3D scene    nvblox voxels of every drone in the team frame, appearing when
              first mapped. Voxels the colour camera covered carry its colour;
              the rest (the depth camera sees wider than the colour camera)
              are tinted with the mapping drone's colour, brighter with height.
              Trails, a small vehicle glyph with the camera's view wedge, the
              planner's current goal and path, the open frontiers, the geofence.
  link        a beam between each pair, with beads running towards the
              receiver while its radio is actually receiving (beads stop when
              no LoRa packet arrived for 0.6 s).
  top view    orthographic inset of the same 3D scene.
  planning    one window per drone at the bottom: the 2D occupancy slice its
  maps        planner reads (nvblox static_occupancy_grid: free / occupied /
              unknown), as flown, with its own track, current goal and path,
              the open frontiers, and what it holds about the other drone from
              the LoRa packets (position ring and claim).
  panels      per-drone state, altitude (PX4), speed, distance flown, mapped
              footprint, plans; LoRa per direction (RSSI, SNR, packet rate,
              delivery, age of the peer state, claims received) with an RSSI
              trace; WiFi per drone (signal, link rate) and the map keyframes
              received over it with a throughput trace; team footprint curve;
              nearest obstacle seen by each reactive guard.

WHAT IS MEASURED AND WHAT IS PROCESSING
- Each drone's map and track are rotated so that its own Offboard flight plane
  is horizontal (fleet_3d_video.level_to_flight_plane). The mapper's frame
  keeps the attitude the vehicle had on the ground; without this the ground
  climbs with distance. The footer states the angles removed.
- The team frame is the alignment the fleet flew with (the pre-flight start
  offsets), not a registration of the two maps.
- "Mapped footprint" is the ground area (0.25 m cells) under the voxels of the
  3D map so far. It is not the planner's 2D slice area of coverage.png.
- Colours of the colour voxels are contrast-stretched for the dark background
  (--color-gain, --color-sat; 1 and 1 leave them as recorded).
- LoRa numbers are from each receiver's lora_rx CSV (per packet), its
  LinkStats delivery ratio (cumulative since the stack started) and the age
  field of /swarm/peers in the bag. The beam's beads follow that age, so they
  do not depend on the metrics recorder.
- WiFi numbers are from rf_stats.sh (iw station dump, 1 Hz) in
  collected/<drone>/wifi_rf_*.csv; the panel title says which mode and
  frequency that log shows (access point or 802.11s mesh).
- The planning maps are NOT levelled: they are what the planner had.
"""
import argparse
import base64
import csv
import glob
import json
import math
import shutil
import subprocess
import sys
import tempfile
import time
import zlib
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from fleet_player import Fleet, parse_t, hhmmss, COLOR  # noqa: E402
import fleet_3d_video as V  # noqa: E402

b64 = V.b64
UNCOLOURED = (127, 128)          # nvblox default / the timeline's fallback grey


def hexrgb(h):
    h = h.lstrip('#')
    return np.array([int(h[i:i + 2], 16) for i in (0, 2, 4)], float)


def level_xy(d, pts):
    """2D points in the drone's own (mapper) frame -> the same points after the
    flight-plane levelling. They are taken to lie on the flight plane."""
    pts = np.asarray(pts, float).reshape(-1, 2)
    if not len(pts) or not hasattr(d, 'level_R'):
        return pts
    sx, sy = d.level_plane
    P = np.column_stack([pts, sx * pts[:, 0] + sy * pts[:, 1]])
    return (P @ d.level_R.T)[:, :2]


def to_world(d, t, pts):
    return d.to_world_many(t, level_xy(d, pts))


def voxels(d, tw1, max_voxels, gain, sat, ground_z):
    """World-frame voxels sorted by first-seen time, display colours, and the
    mask of voxels that carry a camera colour."""
    tl = d.tl
    m = tl['vox_t'] <= tw1
    xyz = tl['vox_xyz'][m].astype(np.float64)
    rgb = tl['vox_rgb'][m].astype(np.float64)
    vt = tl['vox_t'][m]
    if len(xyz) > max_voxels:
        sel = np.sort(np.random.default_rng(0).choice(len(xyz), max_voxels, replace=False))
        xyz, rgb, vt = xyz[sel], rgb[sel], vt[sel]
    o = np.argsort(vt, kind='stable')
    xyz, rgb, vt = xyz[o], rgb[o], vt[o]
    xy = d.to_world_many(tw1, xyz[:, :2])
    xyz = np.column_stack([xy, xyz[:, 2]])
    plain = (rgb[:, 0] == rgb[:, 1]) & (rgb[:, 1] == rgb[:, 2]) & np.isin(rgb[:, 0], UNCOLOURED)
    out = np.empty_like(rgb)
    # camera colours: stretch contrast about the median luminance, then saturation
    c = rgb[~plain]
    if len(c):
        lum = c @ np.array([0.299, 0.587, 0.114])
        l2 = 118.0 + gain * (lum - np.median(lum))
        out[~plain] = np.clip(l2[:, None] + sat * (c - lum[:, None]), 0, 255)
    # no camera colour: the mapping drone's colour, brighter with height above the ground
    h = np.clip((xyz[plain, 2] - ground_z) / 2.2, 0.0, 1.0)
    base = hexrgb(COLOR[d.name])
    out[plain] = np.clip(base[None, :] * (0.26 + 0.50 * h[:, None]) + 10.0, 0, 255)
    return xyz.astype(np.float32), out.astype(np.uint8), vt, ~plain


def footprint(per_drone, t_grid, cell=0.25):
    """Ground area under the voxels seen so far, per drone and for the team."""
    def curve(xy, vt):
        key = np.floor(xy[:, 0] / cell).astype(np.int64) * 1000003 + np.floor(xy[:, 1] / cell).astype(np.int64)
        o = np.argsort(vt, kind='stable')
        _, first = np.unique(key[o], return_index=True)
        tf = np.sort(vt[o][first])
        return np.searchsorted(tf, t_grid, 'right') * cell * cell
    out = {n: curve(xy, vt) for n, (xy, vt) in per_drone.items()}
    allxy = np.vstack([xy for xy, _ in per_drone.values()])
    allt = np.concatenate([vt for _, vt in per_drone.values()])
    return out, curve(allxy, allt)


def camera_path(tracks, trel, fps, orbit_deg, elev_deg, azim_deg, ground_z, fov_deg, aspect,
                recent_s, safe, r_min=19.0, wide_from=None, wide_box=None, wide_elev=None, wide_ymin=None):
    """Like fleet_3d_video.camera_path, but it frames the drones and the last
    `recent_s` of their trails only (the top-view inset shows the whole map)
    and takes the safe area as (x_max, y_min, y_max) in half-frame units.
    From frame `wide_from` on it also keeps `wide_box` (the map's corners) in
    frame, climbs to `wide_elev` and lifts the lower edge of the safe area to
    `wide_ymin` (room for the summary card)."""
    n = len(trel)
    step = max(1, int(2 * fps))
    span = int(recent_s * fps)
    xmax, ymin0, ymax = safe
    blend = np.zeros(n)
    if wide_from is not None:
        u = np.clip((np.arange(n) - (wide_from - 2.5 * fps)) / (4.0 * fps), 0.0, 1.0)
        blend = u * u * (3 - 2 * u)
    elev = elev_deg + ((wide_elev if wide_elev is not None else elev_deg) - elev_deg) * blend
    ymin_f = ymin0 + ((wide_ymin if wide_ymin is not None else ymin0) - ymin0) * blend
    # the safe band is not centred on the optical axis (title above, panels
    # below): shift the picture to its centre (the page applies a view offset)
    yshift = 0.5 * (ymin_f + ymax)

    def keys(f):
        pts = []
        j0 = max(0, f - span)
        j = np.r_[np.arange(f, j0 - 1, -step), j0]
        for t in tracks:
            live = t['alive'][j]
            if live.any():
                jj = j[live]
                pts.append(np.column_stack([t['x'][jj], t['y'][jj], t['z'][jj]]))
            pts.append([[t['x'][f], t['y'][f], t['z'][f] - 1.0], [t['x'][f], t['y'][f], t['z'][f] + 1.6]])
        if wide_from is not None and wide_box is not None and f >= wide_from - 1.5 * fps:
            pts.append(wide_box)
        return np.vstack(pts)

    K = [keys(f) for f in range(n)]
    ctr = np.array([[(k[:, 0].min() + k[:, 0].max()) / 2, (k[:, 1].min() + k[:, 1].max()) / 2] for k in K])
    ctr = V.smooth(ctr, 3.0 * fps)
    tz = ground_z + 0.8
    theta = math.radians(azim_deg) + math.radians(orbit_deg) * np.linspace(0.0, 1.0, n)
    phis = np.radians(90.0 - elev)
    tanv = math.tan(math.radians(fov_deg) / 2)
    tanh = tanv * aspect

    def fits(k, c, r, th, phi, ymin, ymax_):
        u = np.array([math.sin(phi) * math.cos(th), math.sin(phi) * math.sin(th), math.cos(phi)])
        w = -u
        right = np.cross(w, [0.0, 0.0, 1.0])
        right /= np.linalg.norm(right)
        up = np.cross(right, w)
        dd = k - (c + r * u)
        z = dd @ w
        if (z <= 0.5).any():
            return False
        x = (dd @ right) / (z * tanh)
        y = (dd @ up) / (z * tanv)
        return bool((np.abs(x) <= xmax).all() and (y >= ymin).all() and (y <= ymax_).all())

    r_req = np.empty(n)
    for f in range(n):
        c = np.array([ctr[f, 0], ctr[f, 1], tz])
        lo, hi = 5.0, 250.0
        if fits(K[f], c, lo, theta[f], phis[f], ymin_f[f] - yshift[f], ymax - yshift[f]):
            r_req[f] = lo
            continue
        for _ in range(22):
            mid = 0.5 * (lo + hi)
            if fits(K[f], c, mid, theta[f], phis[f], ymin_f[f] - yshift[f], ymax - yshift[f]):
                hi = mid
            else:
                lo = mid
        r_req[f] = hi
    win = int(2.5 * fps)
    padded = np.pad(r_req, win, mode='edge')
    r = np.lib.stride_tricks.sliding_window_view(padded, 2 * win + 1).max(axis=1)
    r = np.clip(V.smooth(r, 2.0 * fps), r_min, 250.0)
    r = np.maximum(r, r_req)
    pos = np.column_stack([ctr[:, 0] + r * np.sin(phis) * np.cos(theta),
                           ctr[:, 1] + r * np.sin(phis) * np.sin(theta),
                           np.full(n, tz) + r * np.cos(phis)])
    tgt = np.column_stack([ctr, np.full(n, tz)])
    return np.hstack([pos, tgt, yshift[:, None]]).astype(np.float32), r


def step_idx(times, ft, maxage=None):
    """Index of the latest event at or before each frame time (-1: none)."""
    times = np.asarray(times, float)
    i = np.searchsorted(times, ft, 'right') - 1
    if maxage is not None and len(times):
        old = (i >= 0) & (ft - times[np.clip(i, 0, len(times) - 1)] > maxage)
        i = np.where(old, -1, i)
    return i.astype(np.int32)


def log_holes(t, min_s):
    """[(start, end)] of the holes longer than min_s in a log's row times."""
    t = np.asarray(t, float)
    if len(t) < 2:
        return []
    k = np.nonzero(np.diff(t) > min_s)[0]
    return [(float(t[i]), float(t[i + 1])) for i in k]


def recorder_holes(d):
    """Intervals in which the drone's metrics recorder wrote nothing at all.
    It writes a vio_health row every second, so a hole there is the recorder
    process pausing; whatever it logs (LoRa packets, link stats, keyframes) is
    unknown in that interval, not absent. Delta, 2026-11-13 15:03:43: 12.8 s
    hole while the LoRa bridge's own log shows frames arriving throughout."""
    return log_holes(d.vio['t'], 2.5) if d.vio is not None else []


def planner_holes(d):
    """Holes in the planner's 20 Hz log that start in EXECUTE: the planner
    process pausing in flight (a hole that starts in PLAN is a long plan)."""
    if d.sp is None:
        return []
    return [(a, b) for a, b in log_holes(d.sp['t'], 2.5) if d.state(a) == 'EXECUTE' and d.is_offboard(a)]


def lora_arrays(d, peer_vid, ft, tw0):
    """Per frame, for d as the receiver: rssi, snr (median of the last 2 s),
    packets since the window start, delivery ratio, seconds since the last packet."""
    n = len(ft)
    out = np.full((n, 5), np.nan, np.float32)
    out[:, 2] = 0
    if d.lora is None or not len(d.lora['t']):
        return out, np.zeros(0), np.zeros(0)
    t = np.asarray(d.lora['t'], float)
    rssi = np.asarray(d.lora['cols']['rssi'], float)
    snr = np.asarray(d.lora['cols']['snr'], float)
    i1 = np.searchsorted(t, ft, 'right')
    i0 = np.searchsorted(t, ft - 2.0, 'right')
    for f in range(n):
        if i1[f] > i0[f]:
            out[f, 0] = np.nanmedian(rssi[i0[f]:i1[f]])
            out[f, 1] = np.nanmedian(snr[i0[f]:i1[f]])
    out[:, 2] = i1 - np.searchsorted(t, tw0, 'left')
    last = np.where(i1 > 0, t[np.clip(i1 - 1, 0, len(t) - 1)], -np.inf)
    out[:, 4] = np.clip(ft - last, 0, 999)
    for a, b in recorder_holes(d):          # the recorder was not running: unknown, not silent
        m = (ft > a + 0.3) & (ft < b + 2.0)
        out[m, 0] = out[m, 1] = np.nan
        out[(ft > a + 0.3) & (ft < b), 4] = -1.0
    if len(d.links):
        L = d.links[d.links[:, 2] == peer_vid]
        if len(L):
            j = step_idx(L[:, 0], ft)
            out[:, 3] = np.where(j >= 0, L[np.clip(j, 0, len(L) - 1), 3], np.nan)
    return out, t, rssi


def keyframe_arrays(d, ft, tw0):
    """Per frame: keyframes received from peers since the window start, and MB."""
    out = np.zeros((len(ft), 2), np.float32)
    kf = d.tl.get('csv', {}).get('keyframes')
    if not kf:
        return out
    rows = [r for r in kf['rows'] if r[1] not in ('', str(d.vid))]
    if not rows:
        return out
    t = np.array([float(r[0]) for r in rows])
    fr = np.array([float(r[2] or 0) for r in rows])
    by = np.array([float(r[3] or 0) for r in rows])
    o = np.argsort(t)
    t, cf, cb = t[o], np.cumsum(fr[o]), np.cumsum(by[o])
    base = np.searchsorted(t, tw0, 'left')
    f0, b0 = (cf[base - 1], cb[base - 1]) if base > 0 else (0.0, 0.0)
    i = np.searchsorted(t, ft, 'right')
    out[:, 0] = np.where(i > 0, cf[np.clip(i - 1, 0, len(t) - 1)], 0) - f0
    out[:, 1] = (np.where(i > 0, cb[np.clip(i - 1, 0, len(t) - 1)], 0) - b0) / 1e6
    return np.clip(out, 0, None)


def plan_extent(raws, tw0, tw1, margin=3.0):
    """Team-frame box (x0, x1, y0, y1) for the planning-map windows: the
    geofence and every drone's track as flown, plus a margin. The slices reach
    further in places (out to the mapper's range); that part is cropped."""
    pts = []
    for d in raws:
        m = (d.od_t >= tw0) & (d.od_t <= tw1)
        if m.any():
            pts.append(d.world_xy[m])
        for p in d.planner:
            if len(p[4]):
                pts.append(d.to_world_many(p[0], p[4]))
                break
    P = np.vstack(pts)
    return (P[:, 0].min() - margin, P[:, 0].max() + margin, P[:, 1].min() - margin, P[:, 1].max() + margin)


def plan_images(d, win, pw, ph, used):
    """PNG data URIs of the planning slices `used` (indices into d.grids),
    drawn into the team-frame window at pw x ph pixels with +x up and +y to
    the left: unknown transparent, free a dark tint of the drone's colour,
    occupied light. A pixel that holds any occupied cell is occupied."""
    X0, X1, Y0, Y1 = win
    base = hexrgb(COLOR[d.name])
    free = (0.34 * base + 0.66 * np.array([11.0, 13.0, 17.0]))[::-1]
    occ = (0.22 * base + 0.78 * 255.0)[::-1]
    kern = np.ones((2, 2), np.uint8)
    out = []
    for i in used:
        g = d.grids[i]
        t, gx, gy, res, shp = g[0], g[1], g[2], g[3], g[4]
        arr = np.frombuffer(zlib.decompress(g[5]), np.uint8).reshape(shp)
        rows, cols = np.nonzero(arr)
        val = arr[rows, cols]
        yaw, tx, ty = d.frame(t)
        xl, yl = gx + (cols + 0.5) * res, gy + (rows + 0.5) * res
        c, s_ = math.cos(yaw), math.sin(yaw)
        X, Y = c * xl - s_ * yl + tx, s_ * xl + c * yl + ty
        u = ((Y1 - Y) / (Y1 - Y0) * pw).astype(np.int32)
        v = ((X1 - X) / (X1 - X0) * ph).astype(np.int32)
        ok = (u >= 0) & (u < pw) & (v >= 0) & (v < ph)
        fm = np.zeros((ph, pw), np.uint8)
        om = np.zeros((ph, pw), np.uint8)
        k = ok & (val == 1)
        fm[v[k], u[k]] = 1
        k = ok & (val == 2)
        om[v[k], u[k]] = 1
        om = cv2.dilate(om, kern)
        img = np.zeros((ph, pw, 4), np.uint8)
        img[fm > 0, :3] = free
        img[fm > 0, 3] = 255
        img[om > 0, :3] = occ
        img[om > 0, 3] = 255
        _, buf = cv2.imencode('.png', img, [cv2.IMWRITE_PNG_COMPRESSION, 9])
        out.append('data:image/png;base64,' + base64.b64encode(buf.tobytes()).decode())
    return out


def peer_arrays(d, peer_vid, ft):
    """Per frame, what d holds about its peer from the LoRa packets (the bag's
    /swarm/peers, so independent of the metrics recorder): believed x, y in the
    team frame, valid, claim x, y, has a claim, age of the peer state [s]
    (-1: none), distinct claim points received since the recording started."""
    out = np.zeros((len(ft), 8), np.float32)
    out[:, 6] = -1
    P = d.peers[d.peers[:, 2] == peer_vid] if len(d.peers) else np.zeros((0, 18))
    if not len(P):
        return out
    j = step_idx(P[:, 0], ft, maxage=3.0)
    ok = j >= 0
    jj = np.clip(j, 0, len(P) - 1)
    out[:, 0], out[:, 1], out[:, 2] = P[jj, 3], P[jj, 4], ok
    out[:, 3], out[:, 4], out[:, 5] = P[jj, 8], P[jj, 9], ok & (P[jj, 7] > 0)
    out[:, 6] = np.where(ok, P[jj, 16], -1)
    seen, first = set(), []
    for t, k in zip(P[P[:, 7] > 0][:, 0], map(tuple, np.round(P[P[:, 7] > 0][:, 8:10], 1))):
        if k not in seen:
            seen.add(k)
            first.append(t)
    out[:, 7] = np.searchsorted(np.array(first), ft, 'right') if first else 0
    return out


def trailing_rate(t, w, ft, win_s, holes):
    """Sum of w over the last win_s seconds, per second, at each frame time;
    NaN where the log has a hole inside that window."""
    t = np.asarray(t, float)
    cw = np.r_[0.0, np.cumsum(np.asarray(w, float))]
    out = (cw[np.searchsorted(t, ft, 'right')] - cw[np.searchsorted(t, ft - win_s, 'right')]) / win_s
    for a, b in holes:
        out[(ft > a + 0.3) & (ft < b + win_s)] = np.nan
    return out


def wifi_log(logs_dir, name, tw0, tw1):
    """The rf_stats.sh log of this drone that covers the window: times, signal
    [dBm], tx link rate [Mb/s], and the mode / frequency it reports."""
    best = None
    for f in sorted(glob.glob(str(Path(logs_dir) / name / ('wifi_rf_%s_*.csv' % name)))):
        with open(f, errors='ignore') as fh:
            rows = [r for r in csv.DictReader(l.replace('\0', '') for l in fh) if r.get('t_unix')]
        try:
            t = np.array([float(r['t_unix']) for r in rows])
        except ValueError:
            continue
        m = (t >= tw0) & (t <= tw1)
        if m.sum() and (best is None or m.sum() > best[0]):
            best = (int(m.sum()), t, rows)
    if best is None:
        return None
    _, t, rows = best

    def num(k):
        o = []
        for r in rows:
            try:
                o.append(float(r[k]))
            except (ValueError, TypeError, KeyError):
                o.append(np.nan)
        return np.array(o)
    sig = num('signal_dbm')
    sig[sig > -5] = np.nan                       # the driver reports 0 now and then
    m = (t >= tw0) & (t <= tw1)
    modes = [r.get('mode', '') for r, k in zip(rows, m) if k]
    freqs = [r.get('freq_mhz', '') for r, k in zip(rows, m) if k]
    return dict(t=t, sig=sig, txr=num('tx_bitrate_mbps'),
                mode=max(set(modes), key=modes.count), freq=max(set(freqs), key=freqs.count))


def wifi_title(modes):
    """Panel title from what the WiFi logs report, e.g. 'WiFi 5765 MHz · access point'."""
    if not modes:
        return 'WiFi'
    mode = max(set(m for m, _ in modes), key=[m for m, _ in modes].count)
    freq = max(set(f for _, f in modes), key=[f for _, f in modes].count)
    kind = {'managed': 'access point', 'mesh point': '802.11s mesh', 'mesh': '802.11s mesh',
            'AP': 'access point'}.get(mode, mode)
    return 'WiFi %s MHz %s' % (freq, kind)


def wifi_note(modes):
    """One line under the WiFi table: what the trace is and, for a router link, that it is not the mesh."""
    managed = any(m == 'managed' for m, _ in modes)
    return 'trace: kB/s received' + (' · router link, not the 802.11s mesh' if managed else '')


def held(t, v, ft, maxage):
    """Latest finite value at or before each frame time, NaN when older than maxage."""
    t, v = np.asarray(t, float), np.asarray(v, float)
    k = np.isfinite(v)
    t, v = t[k], v[k]
    if not len(t):
        return np.full(len(ft), np.nan)
    j = np.searchsorted(t, ft, 'right') - 1
    jj = np.clip(j, 0, len(t) - 1)
    return np.where((j >= 0) & (ft - t[jj] <= maxage), v[jj], np.nan)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('drones', nargs='*')
    ap.add_argument('--bags', default=str(HERE.parent / 'bags'))
    ap.add_argument('--logs', default=str(HERE.parent / 'collected'), help='collected/<drone>/ (for the WiFi logs)')
    ap.add_argument('--from', dest='t_from')
    ap.add_argument('--to', dest='t_to')
    ap.add_argument('--out', required=True)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--rate', type=float, default=4.0)
    ap.add_argument('--size', default='1920x1080')
    ap.add_argument('--crf', type=int, default=18)
    ap.add_argument('--preset', default='slow')
    ap.add_argument('--jpeg-quality', type=int, default=95)
    ap.add_argument('--frame-format', choices=('png', 'jpeg'), default='png')
    ap.add_argument('--max-voxels', type=int, default=1500000)
    ap.add_argument('--point-size', type=float, default=0.085)
    ap.add_argument('--fov', type=float, default=40.0)
    ap.add_argument('--orbit-deg', type=float, default=150.0)
    ap.add_argument('--elev-deg', type=float, default=33.0)
    ap.add_argument('--azim-deg', type=float, default=-150.0)
    ap.add_argument('--elev-end-deg', type=float, default=52.0, help='camera elevation for the closing overview')
    ap.add_argument('--frame-trail-s', type=float, default=45.0, help='seconds of trail the camera keeps in frame')
    ap.add_argument('--hold-s', type=float, default=6.0, help='seconds of video on the last instant, with the summary')
    ap.add_argument('--color-gain', type=float, default=1.7)
    ap.add_argument('--color-sat', type=float, default=1.6)
    ap.add_argument('--no-level', action='store_true', help='do not level the maps to their flight planes')
    ap.add_argument('--title')
    ap.add_argument('--subtitle')
    ap.add_argument('--place', default='', help='short place / context text for the title block')
    ap.add_argument('--lora-label', default='LoRa 915 MHz · SF7 · 500 kHz',
                    help='radio settings for the link panel title (radiohive firmware: LORA_FREQ / LORA_SF / LORA_BW)')
    ap.add_argument('--frames', help='render only frames A:B (testing)')
    ap.add_argument('--stills-only', help='comma-separated times (HH:MM:SS, +s, or end): PNGs only')
    ap.add_argument('--software-gl', action='store_true')
    ap.add_argument('--jobs', type=int, default=1)
    a = ap.parse_args()

    t_start = time.time()
    W, H = (int(v) for v in a.size.lower().split('x'))
    raw = Fleet(a.run, a.drones, a.bags, rot='0')          # as flown: for the planning-map windows
    raw_by = {d.name: d for d in raw.drones}
    fleet = Fleet(a.run, a.drones, a.bags, rot='0')
    level = {} if a.no_level else V.level_to_flight_plane(fleet)
    for name, (tilt, rms) in level.items():
        print('%-13s levelled to its flight plane: %.2f deg removed, flight-height rms %.2f m' % (name, tilt, rms),
              flush=True)
    tw0, tw1 = fleet.default_window()
    if a.t_from:
        tw0 = parse_t(a.t_from, fleet.t0)
    if a.t_to:
        tw1 = parse_t(a.t_to, fleet.t0)
    tw0, tw1 = max(tw0, fleet.t0), min(tw1, fleet.t1)
    n_main = int(round((tw1 - tw0) / a.rate * a.fps)) + 1
    n_hold = int(round(a.hold_s * a.fps))
    n = n_main + n_hold
    frame_t = tw0 + np.minimum(np.arange(n), n_main - 1) / a.fps * a.rate
    trel = frame_t - tw0

    # ---- voxels ----
    vz = [d.tl['vox_xyz'][d.tl['vox_t'] <= tw1][:, 2] for d in fleet.drones if len(d.tl['vox_t'])]
    ground_z = float(np.percentile(np.concatenate(vz), 2)) if vz else -0.4
    vox = {d.name: voxels(d, tw1, a.max_voxels, a.color_gain, a.color_sat, ground_z) for d in fleet.drones}
    sec = np.arange(0.0, tw1 - tw0 + 1.0, 1.0)
    area_d, area_team = footprint({nm: (v[0][:, :2].astype(np.float64), v[2]) for nm, v in vox.items()}, tw0 + sec)
    area_f, area_team_f = footprint({nm: (v[0][:, :2].astype(np.float64), v[2]) for nm, v in vox.items()}, frame_t)

    tracks = V.drone_tracks(fleet, tw0, tw1, frame_t, ground_z, trail_z='odom', radius=0.085)
    if not tracks:
        sys.exit('no odometry in this window')
    aspect = W / H
    view_dx = 190 * W / 1920.0
    t_cam = time.time()
    ax_ = np.concatenate([v[0][:, 0] for v in vox.values()])
    ay_ = np.concatenate([v[0][:, 1] for v in vox.values()])
    qx, qy = np.percentile(ax_, [1.5, 98.5]), np.percentile(ay_, [1.5, 98.5])
    wide_box = np.array([[qx[0], qy[0], ground_z], [qx[0], qy[1], ground_z], [qx[1], qy[0], ground_z],
                         [qx[1], qy[1], ground_z]])
    cam, cam_r = camera_path(tracks, trel, a.fps, a.orbit_deg, a.elev_deg, a.azim_deg, ground_z, a.fov, aspect,
                             a.frame_trail_s, safe=(0.70, -0.42, 0.62),
                             wide_from=n_main if n_hold else None, wide_box=wide_box, wide_elev=a.elev_end_deg)
    print('camera path %.1f s  distance %.0f-%.0f m' % (time.time() - t_cam, cam_r.min(), cam_r.max()), flush=True)

    pwin = plan_extent(raw.drones, tw0, tw1)
    PH = int(round(186 * H / 1080.0))
    PW = int(round(PH * (pwin[3] - pwin[2]) / (pwin[1] - pwin[0])))
    wifi_modes = []
    pdrones, events, summary = [], [], {}
    geofence = geofence_raw = None
    for tr in tracks:
        d = tr['d']
        rd = raw_by[d.name]
        for a_, b_ in planner_holes(d):
            tr['offb'][(frame_t >= a_) & (frame_t <= b_)] = 3
        xyz, rgb, vt, coloured = vox[d.name]
        dist = np.interp(frame_t, d.od_t, d.path_len) - np.interp(tw0, d.od_t, d.path_len)
        track = np.column_stack([tr['x'], tr['y'], tr['z'], tr['yaw'], tr['alt'], tr['speed'], dist,
                                 area_f[d.name]]).astype(np.float32)
        # planner goal + path, frontiers
        r2 = lambda P: [[round(float(x), 2), round(float(y), 2)] for x, y in P]   # noqa: E731
        plans, plan_t = [], []
        for p in d.planner:
            t, goal, path = p[0], p[1], p[2]
            if len(p[4]) and geofence is None:
                geofence = to_world(d, t, p[4])
                geofence_raw = rd.to_world_many(t, p[4])
            if goal is None:
                continue
            g = to_world(d, t, [goal[:2]])[0]
            pw = to_world(d, t, path) if len(path) else np.zeros((0, 2))
            # the same goal and path as flown (not levelled), for the planning-map window
            gr = rd.to_world_many(t, [goal[:2]])[0]
            pr = rd.to_world_many(t, path) if len(path) else np.zeros((0, 2))
            plans.append(dict(g=[round(float(g[0]), 2), round(float(g[1]), 2)], p=r2(pw[:60]),
                              gr=[round(float(gr[0]), 2), round(float(gr[1]), 2)], pr=r2(pr[:60])))
            plan_t.append(t)
        nplans = np.searchsorted(np.array(plan_t), frame_t, 'right').astype(np.int16) if plan_t else np.zeros(n, np.int16)
        gi = step_idx(plan_t, frame_t)
        st = np.array([d.state(t) == 'EXECUTE' for t in frame_t])
        gi = np.where(st & (tr['offb'] == 1), gi, -1).astype(np.int16)
        fronts, fronts_raw, fr_t, fr_n = [], [], [], []
        for fr in d.frontiers:
            c = to_world(d, fr[0], fr[1]) if len(fr[1]) else np.zeros((0, 2))
            fronts.append(r2(c[:200]))
            fronts_raw.append(r2(rd.to_world_many(fr[0], fr[1])[:200]) if len(fr[1]) else [])
            fr_n.append(int(len(c)))
            fr_t.append(fr[0])
        fi = step_idx(fr_t, frame_t, maxage=10.0).astype(np.int16)
        # links: d as the receiver, one peer (the first other drone that it hears)
        peers = [o for o in fleet.drones if o is not d]
        lr = np.zeros((n, 5), np.float32)
        rssi_sec = np.full(len(sec), np.nan, np.float32)
        peer_name = ''
        if peers:
            peer = peers[0]
            peer_name = peer.name
            lr, lt, lrssi = lora_arrays(d, peer.vid, frame_t, tw0)
            if len(lt):
                i1 = np.searchsorted(lt, tw0 + sec, 'right')
                i0 = np.searchsorted(lt, tw0 + sec - 2.0, 'right')
                for k in range(len(sec)):
                    if i1[k] > i0[k]:
                        rssi_sec[k] = np.nanmedian(lrssi[i0[k]:i1[k]])
        kf = keyframe_arrays(d, frame_t, tw0)
        holes = recorder_holes(d)
        # ---- more link numbers ----
        pa = peer_arrays(d, peers[0].vid, frame_t) if peers else np.zeros((n, 8), np.float32)
        rate = np.full(n, np.nan)
        if peers and len(lt):
            rate = trailing_rate(lt, np.ones(len(lt)), frame_t, 5.0, holes)
        lr = np.column_stack([lr, rate, pa[:, 6], pa[:, 7]])
        kfr = np.full(n, np.nan)
        kfr_sec = np.full(len(sec), np.nan)
        kcsv = d.tl.get('csv', {}).get('keyframes')
        if kcsv:
            krows = [r for r in kcsv['rows'] if r[1] not in ('', str(d.vid))]
            if krows:
                kt = np.array([float(r[0]) for r in krows])
                kb = np.array([float(r[3] or 0) for r in krows]) / 1000.0
                o_ = np.argsort(kt)
                kfr = trailing_rate(kt[o_], kb[o_], frame_t, 5.0, holes)
                kfr_sec = trailing_rate(kt[o_], kb[o_], tw0 + sec, 5.0, holes)
        wl = wifi_log(a.logs, d.name, tw0, tw1)
        wf = np.full((n, 2), np.nan)
        if wl is not None:
            wf[:, 0] = held(wl['t'], wl['sig'], frame_t, 4.0)
            wf[:, 1] = held(wl['t'], wl['txr'], frame_t, 4.0)
            wifi_modes.append((wl['mode'], wl['freq']))
        # ---- planning-map window (as flown) ----
        gidx = step_idx(rd.grid_t, frame_t)
        used = sorted(set(int(i) for i in gidx if i >= 0))
        remap = {g_: k for k, g_ in enumerate(used)}
        pgi = np.array([remap.get(int(i), -1) for i in gidx], np.int16)
        pimg = plan_images(rd, pwin, PW, PH, used)
        ptrk = np.column_stack([np.interp(frame_t, rd.od_t, rd.world_xy[:, 0]),
                                np.interp(frame_t, rd.od_t, rd.world_xy[:, 1]),
                                np.interp(frame_t, rd.od_t, np.unwrap(rd.world_yaw))]).astype(np.float32)
        obs = np.full(n, np.nan)
        obs_sec = np.full(len(sec), np.nan)
        if d.gd is not None and 'min_horiz' in d.gd['cols']:
            gv = np.asarray(d.gd['cols']['min_horiz'], float)
            gv[~np.isfinite(gv)] = np.nan
            obs = held(d.gd['t'], gv, frame_t, 2.0)
            obs_sec = held(d.gd['t'], gv, tw0 + sec, 2.0)
        pst = np.array([{'EXECUTE': 2, 'PLAN': 1}.get(d.state(t), 0) for t in frame_t], np.uint8)
        nz = lambda A: np.nan_to_num(np.asarray(A, float), nan=-999.0).astype(np.float32)   # noqa: E731
        sl = lambda A: [None if not np.isfinite(v) else round(float(v), 2) for v in A]      # noqa: E731
        pdrones.append(dict(
            pimg=pimg, pgi=b64(pgi), ptrk=b64(ptrk), pa=b64(pa.astype(np.float32)), obs=b64(nz(obs)),
            pst=b64(pst), nplans=b64(nplans), fronts_raw=fronts_raw, wf=b64(nz(wf)), kfr=b64(nz(kfr)),
            kfr_sec=sl(kfr_sec), obs_sec=sl(np.clip(obs_sec, 0, 6)),
            name=d.name, color=COLOR[d.name], peer=peer_name, n_vox=int(len(vt)), n_col=int(coloured.sum()),
            vox_xyz=b64(xyz), vox_rgb=b64(rgb), vox_t=b64((vt - tw0).astype(np.float32)),
            tube_v=b64(tr['verts']), tube_n=b64(tr['norms']), tube_i=b64(tr['idx']), icount=b64(tr['icount']),
            track=b64(track), alive=b64(tr['status']), offb=b64(tr['offb']),
            plans=plans, gi=b64(gi), fronts=fronts, fn=fr_n, fi=b64(fi),
            lr=b64(np.nan_to_num(lr, nan=-999.0).astype(np.float32)), kf=b64(kf),
            area_sec=[round(float(v), 1) for v in area_d[d.name]],
            rssi_sec=[None if not np.isfinite(v) else round(float(v), 1) for v in rssi_sec]))
        # events for the ticker
        for i, (t0_, t1_) in enumerate(d.offboard):
            if i == 0 or t0_ - d.offboard[i - 1][1] > 3.0:
                events.append((t0_, d.name, 'Offboard engaged: autonomous exploration'))
            if i == len(d.offboard) - 1 or d.offboard[i + 1][0] - t1_ > 3.0:
                events.append((t1_, d.name, 'Offboard released by the pilot'))
        last_stop = None
        for t, s in d.guard_ev:
            if s.startswith('STOP') and d.is_offboard(t):
                last_stop = (t, s)
            elif last_stop is not None:
                why = last_stop[1].split(':', 1)[-1].replace('_', ' ').lower()
                if t - last_stop[0] >= 1.0:
                    events.append((last_stop[0], d.name,
                                   'reactive guard held position (%s), released after %.1f s' % (why, t - last_stop[0])))
                last_stop = None
        for a_, b_ in planner_holes(d):
            events.append((a_, d.name, 'planner process paused for %.1f s (onboard stall, vehicle flew on); '
                                       'exploration resumed' % (b_ - a_)))
        stale = V.px4_stale(d, frame_t[:n_main])
        if stale.any():
            k0 = int(np.argmax(stale))
            k1 = len(stale) - 1 - int(np.argmax(stale[::-1]))
            events.append((frame_t[k0], d.name, 'PX4 feed to the companion computer lost for %.0f s' % (frame_t[k1] - frame_t[k0])))
        if peers and len(lt) > 1:
            m = (lt[1:] >= tw0) & (lt[:-1] <= tw1) & (np.diff(lt) > 3.0)
            for k in np.nonzero(m)[0]:
                if not any(a_ - 1.5 <= lt[k] and lt[k + 1] <= b_ + 1.5 for a_, b_ in holes):
                    events.append((lt[k] + 0.6, d.name, 'no LoRa packets received for %.1f s' % (lt[k + 1] - lt[k])))
        print('%-13s %7d voxels (%d with camera colour)  %d plans  %d frontier sets  trail z from %s' % (
            d.name, len(vt), coloured.sum(), len(plans), len(fronts), tr['zsrc']), flush=True)
        summary[d.name] = dict(dist=float(dist[n_main - 1]), area=float(area_f[d.name][n_main - 1]),
                               pkts=int(lr[n_main - 1, 2]), deliv=float(lr[n_main - 1, 3]),
                               rssi=float(np.nanmedian(rssi_sec)) if np.isfinite(rssi_sec).any() else float('nan'),
                               kf=int(kf[n_main - 1, 0]), kf_mb=float(kf[n_main - 1, 1]))
    events.sort()
    ev = [dict(f=int(np.clip(round((t - tw0) / a.rate * a.fps), 0, n_main - 1)), name=nm, color=COLOR[nm],
               clock=hhmmss(t), text=tx) for t, nm, tx in events if tw0 <= t <= tw1]

    # team: separation, joint Offboard time
    sep = np.zeros(n, np.float32)
    if len(tracks) >= 2:
        sep = np.hypot(tracks[0]['x'] - tracks[1]['x'], tracks[0]['y'] - tracks[1]['y']).astype(np.float32)
    joint = np.ones(n_main, bool)
    for tr in tracks:
        joint &= (tr['offb'][:n_main] >= 1)
    joint_s = float(joint.sum()) / a.fps * a.rate
    team = np.column_stack([sep, area_team_f]).astype(np.float32)

    allx = np.concatenate([v[0][:, 0] for v in vox.values()])
    ally = np.concatenate([v[0][:, 1] for v in vox.values()])
    bx, by = np.percentile(allx, [0.5, 99.5]), np.percentile(ally, [0.5, 99.5])
    if geofence is not None and len(geofence):
        bx = [min(bx[0], geofence[:, 0].min()), max(bx[1], geofence[:, 0].max())]
        by = [min(by[0], geofence[:, 1].min()), max(by[1], geofence[:, 1].max())]
    size = int(math.ceil((max(bx[1] - bx[0], by[1] - by[0]) + 520) / 20.0) * 20)
    names = ' + '.join(t['d'].name for t in tracks)
    day = time.strftime('%Y-%m-%d', time.localtime(tw0))
    lvl = ', '.join('%s %.1f°' % (k, v[0]) for k, v in level.items())
    page_data = dict(
        fps=a.fps, rate=a.rate, n_frames=n, n_main=n_main, t0=tw0, dur=tw1 - tw0, fov=a.fov,
        point_size=a.point_size, view_dx=view_dx, cam=b64(cam), drones=pdrones, team=b64(team),
        area_team_sec=[round(float(v), 1) for v in area_team], events=ev, ground_z=ground_z,
        grid=dict(size=size, cx=float(round((bx[0] + bx[1]) / 20) * 10), cy=float(round((by[0] + by[1]) / 20) * 10)),
        mini=dict(x0=float(bx[0]) - 2, x1=float(bx[1]) + 2, y0=float(by[0]) - 2, y1=float(by[1]) + 2),
        geofence=[[round(float(x), 2), round(float(y), 2)] for x, y in geofence] if geofence is not None else [],
        plan=dict(win=[float(v) for v in pwin], pw=PW, ph=PH,
                  geofence=[[round(float(x), 2), round(float(y), 2)] for x, y in geofence_raw]
                  if geofence_raw is not None else []),
        wifi=wifi_title(wifi_modes), wifi_note=wifi_note(wifi_modes), lora=a.lora_label,
        title=a.title or 'Coordinated multi-drone exploration',
        subtitle=a.subtitle or f'{names} · field flight {day}' + (f' · {a.place}' if a.place else ''),
        footer=('run %s · onboard nvblox map, team frame from the pre-flight start offsets (no map registration) · '
                % a.run + ('each map levelled to its own flight plane (%s) · ' % lvl if lvl else '') +
                'altitude: PX4 · colours contrast-stretched for display · planning maps as flown'),
        summary=dict(joint_s=joint_s, drones=summary, area_team=float(area_team_f[n_main - 1]),
                     n=len(tracks), sep_max=float(sep[:n_main].max()) if len(tracks) >= 2 else 0.0))
    tmpdir = Path(tempfile.mkdtemp(prefix='fleet_showcase_'))
    page = tmpdir / 'capture.html'
    page.write_text(PAGE.replace('__W__', str(W)).replace('__H__', str(H))
                    .replace('__THREE_SRC__', V.three_source())
                    .replace('__DATA__', json.dumps(page_data)))
    print('window %s-%s  %d frames (+%d hold) @ %d fps x%g  page %.1f MB  prep %.1f s' % (
        hhmmss(tw0), hhmmss(tw1), n_main, n_hold, a.fps, a.rate, page.stat().st_size / 1e6, time.time() - t_start),
        flush=True)
    print('summary: both in Offboard %.0f s; team footprint %.0f m2; ' % (joint_s, area_team_f[n_main - 1]) +
          '; '.join('%s %.0f m, %.0f m2, %d LoRa pkts, delivery %.3f, %d keyframes (%.1f MB)' % (
              k, s['dist'], s['area'], s['pkts'], s['deliv'], s['kf'], s['kf_mb']) for k, s in summary.items()),
          flush=True)

    flags = ['--enable-unsafe-swiftshader'] if a.software_gl else V.GPU_FLAGS
    uri = page.as_uri()
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    def frame_of(spec):
        if spec == 'end':
            return n - 1
        ts = tw0 + float(spec[1:]) if spec.startswith('+') else parse_t(spec, tw0)
        return int(np.clip(round((ts - tw0) / a.rate * a.fps), 0, n_main - 1))

    ok = False
    try:
        if a.stills_only:
            chrome = V.open_page(uri, W, H, flags, str(tmpdir / 'chrome.log'))
            try:
                for i, spec in enumerate(a.stills_only.split(',')):
                    f = frame_of(spec)
                    chrome.js('renderAt(%d)' % f)
                    p = out.with_name(out.stem + '_still%d.png' % i)
                    p.write_bytes(chrome.shot('png'))
                    print('still', spec, '-> frame', f, p, flush=True)
            finally:
                chrome.close()
            ok = True
            return
        f0, f1 = 0, n
        if a.frames:
            f0, f1 = (int(v) for v in a.frames.split(':'))
        jobs = max(1, min(a.jobs, (f1 - f0) // 300 or 1))
        t_cap = time.time()
        errors = []
        if jobs == 1:
            V.render_range(uri, W, H, a, f0, f1, out, '', flags, str(tmpdir / 'chrome.log'), errors)
        else:
            import threading
            cuts = np.linspace(f0, f1, jobs + 1).round().astype(int)
            segs = [tmpdir / ('seg%02d.mp4' % i) for i in range(jobs)]
            th = [threading.Thread(target=V.render_range, args=(
                uri, W, H, a, int(cuts[i]), int(cuts[i + 1]), segs[i], '[%d/%d] ' % (i + 1, jobs), flags,
                str(tmpdir / ('chrome%02d.log' % i)), errors)) for i in range(jobs)]
            for t in th:
                t.start()
            for t in th:
                t.join()
            if not errors:
                lst = tmpdir / 'segments.txt'
                lst.write_text(''.join("file '%s'\n" % sg for sg in segs))
                r = subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-f', 'concat',
                                    '-safe', '0', '-i', str(lst), '-c', 'copy', '-movflags', '+faststart', str(out)])
                if r.returncode != 0:
                    errors.append('joining the segments failed')
        if errors:
            raise RuntimeError('; '.join(errors))
        print('captured %d frames in %.0f s (%.1f fps, %d job%s)' % (
            f1 - f0, time.time() - t_cap, (f1 - f0) / (time.time() - t_cap), jobs, 's' if jobs > 1 else ''), flush=True)
        print('wrote %s (%.1f MB)  total %.0f s' % (out, out.stat().st_size / 1e6, time.time() - t_start), flush=True)
        ok = True
    finally:
        if ok:
            shutil.rmtree(tmpdir, ignore_errors=True)
        else:
            print('kept', tmpdir, 'for debugging (capture.html, chrome*.log)', file=sys.stderr)


PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>fleet showcase capture</title>
<style>
:root{--bg:#0b0d11;--panel:rgba(17,20,26,.88);--line:#262b35;--ink:#eef0f3;--ink2:#9aa3b2;--ink3:#687182;
  --good:#3ddc84;--warn:#f5b83d;--bad:#ff5d5d;--mono:ui-monospace,'Ubuntu Mono','DejaVu Sans Mono',monospace}
html,body{margin:0;width:__W__px;height:__H__px;background:var(--bg);color:var(--ink);
  font:15px/1.35 system-ui,'Ubuntu','DejaVu Sans',sans-serif;overflow:hidden}
#c{position:absolute;left:0;top:0;width:__W__px;height:__H__px;display:block}
.mono{font-family:var(--mono)}
#hud{position:absolute;left:40px;top:30px}
#hud h1{font-size:30px;font-weight:700;margin:0 0 3px;letter-spacing:.2px}
#hud .sub{font-size:16px;color:var(--ink2);margin-bottom:12px}
#clock{font:600 34px var(--mono);letter-spacing:.5px}
#tplus{font:16px var(--mono);color:var(--ink2);margin-left:14px}
#rate{display:inline-block;margin-left:12px;padding:1px 9px;border:1px solid var(--line);border-radius:10px;
  font-size:13px;color:var(--ink2);vertical-align:4px}
#col{position:absolute;right:28px;top:26px;width:344px;display:flex;flex-direction:column;gap:9px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:9px 14px 10px;box-sizing:border-box}
.card.d{border-left:5px solid}
.card .h{display:flex;justify-content:space-between;align-items:baseline}
.card .n{font-weight:700;font-size:19px}
.badge{font-size:12px;font-weight:700;letter-spacing:.7px;padding:1px 8px;border-radius:9px;border:1px solid}
.kv{display:grid;grid-template-columns:1fr 1fr;column-gap:14px;row-gap:1px;margin-top:5px}
.kv div{display:flex;justify-content:space-between;font-size:13.5px}
.kv span:first-child{color:var(--ink2)}
.kv span:last-child{font-family:var(--mono)}
.ttl{font-size:11.5px;font-weight:700;letter-spacing:1.1px;color:var(--ink3);margin-bottom:4px;white-space:nowrap}
.row{display:flex;justify-content:space-between;font-size:13.5px;margin-top:1px}
.row .l{color:var(--ink2)}
.row .v{font-family:var(--mono)}
.row.nt{font-size:12.5px;white-space:nowrap}
.tbl{display:grid;column-gap:10px;row-gap:1px;font-size:13.5px;align-items:baseline}
.tbl .hd{font-size:11px;color:var(--ink3);text-align:right}
.tbl .v{font-family:var(--mono);text-align:right;white-space:nowrap}
.tbl .l{white-space:nowrap}
#minibox{position:absolute;right:28px;bottom:26px;width:344px;height:300px;border:1px solid var(--line);
  border-radius:10px;box-sizing:border-box;pointer-events:none}
#minibox .ttl{position:absolute;left:10px;top:7px}
#minibox .sc{position:absolute;right:10px;bottom:7px;font-size:11px;color:var(--ink2);text-shadow:0 0 4px #000}
#band{position:absolute;left:40px;bottom:44px;width:calc(__W__px - 432px);height:236px;display:flex;gap:10px}
#band .card{height:236px}
.pcard{flex:0 0 auto;padding:8px 10px 6px}
.pcard canvas{display:block;border-radius:5px}
.pcard .h{margin-bottom:4px}
.pcard .pn{font-weight:700;font-size:14px}
.pcard .ps{font-size:11.5px;color:var(--ink2);font-family:var(--mono)}
.pcard .lg{font-size:10.5px;color:var(--ink3);margin-top:3px;white-space:nowrap}
.ccard{flex:1 1 0;min-width:0}
.ccard canvas{display:block;margin-top:5px}
#ticker{position:absolute;left:40px;bottom:289px;height:22px;font-size:14.5px;white-space:nowrap;padding:2px 12px 3px;
  border-radius:12px;background:rgba(11,13,17,.82);border:1px solid var(--line)}
#ticker .c{color:var(--ink3);margin-right:8px}
#foot{position:absolute;left:40px;bottom:17px;font-size:11.5px;color:var(--ink3);white-space:nowrap}
#shade{position:absolute;left:0;bottom:0;width:calc(__W__px - 384px);height:350px;pointer-events:none;
  background:linear-gradient(to top,rgba(11,13,17,.94) 0%,rgba(11,13,17,.72) 55%,rgba(11,13,17,0) 100%)}
#shadet{position:absolute;left:0;top:0;width:__W__px;height:190px;pointer-events:none;
  background:linear-gradient(to bottom,rgba(11,13,17,.80) 0%,rgba(11,13,17,0) 100%)}
.lab{position:absolute;transform:translate(-50%,-100%);font-weight:700;font-size:15px;padding:1px 9px;
  border-radius:11px;background:rgba(11,13,17,.82);border:1.5px solid;white-space:nowrap}
.lab2{position:absolute;transform:translate(-50%,-50%);font-size:12.5px;padding:1px 8px;border-radius:9px;
  background:rgba(11,13,17,.80);border:1px solid var(--line);color:var(--ink2);white-space:nowrap}
#intro{position:absolute;left:0;top:0;width:__W__px;height:__H__px;display:flex;align-items:center;
  justify-content:center;pointer-events:none;z-index:20}
#intro .box{background:rgba(11,13,17,.80);border:1px solid var(--line);border-radius:14px;padding:22px 34px;
  text-align:center;margin:0 380px 230px 0}
#intro .big{font-size:34px;font-weight:700}
#intro .sm{font-size:17px;color:var(--ink2);margin-top:6px}
#outro{position:absolute;left:40px;bottom:44px;width:calc(__W__px - 432px);height:236px;box-sizing:border-box;z-index:20;
  background:rgba(13,16,21,.96);border:1px solid var(--line);border-radius:12px;padding:20px 34px 0}
#outro .big{font-size:23px;font-weight:700;margin-bottom:12px}
#outro .grid{display:grid;grid-template-columns:repeat(4,1fr);gap:18px 28px}
#outro .num{font:700 37px var(--mono)}
#outro .cap{font-size:14px;color:var(--ink2);margin-top:1px}
</style></head><body>
<canvas id="c" width="__W__" height="__H__"></canvas>
<div id="shadet"></div><div id="shade"></div>
<div id="hud"><h1 id="title"></h1><div class="sub" id="sub"></div>
<div><span id="clock"></span><span id="tplus"></span><span id="rate"></span></div></div>
<div id="col"></div>
<div id="minibox"><div class="ttl">TOP VIEW · 3D MAP</div><div class="sc" id="minisc"></div></div>
<div id="ticker"></div>
<div id="band"></div>
<div id="outro"><div class="big" id="outro1"></div><div class="grid" id="outrog"></div></div>
<div id="foot"></div>
<div id="intro"><div class="box"><div class="big" id="intro1"></div><div class="sm" id="intro2"></div></div></div>
<script>__THREE_SRC__</script>
<script>
const DATA = __DATA__;
const dec = (s, T) => new T(Uint8Array.from(atob(s), c => c.charCodeAt(0)).buffer);
const W = __W__, H = __H__, N = DATA.n_frames, NM = DATA.n_main, GZ = DATA.ground_z;
const BG = 0x0b0d11;
const renderer = new THREE.WebGLRenderer({canvas: document.getElementById('c'), antialias: true,
                                          preserveDrawingBuffer: true});
renderer.setPixelRatio(1); renderer.setSize(W, H, false); renderer.autoClear = false;
renderer.setClearColor(BG, 1);
const scene = new THREE.Scene();
scene.fog = new THREE.Fog(BG, 60, 200);
const camera = new THREE.PerspectiveCamera(DATA.fov, W / H, 0.1, 3000);
camera.up.set(0, 0, 1);
camera.layers.set(0);
// top view: +x of the team frame up, +y to the left
const MB = {x: W - 28 - 344, y: 26, w: 344, h: 300};           // gl viewport (from the bottom-left)
const M = DATA.mini;
let mhw = (M.y1 - M.y0) / 2, mhh = (M.x1 - M.x0) / 2;
if (mhw / mhh > MB.w / MB.h) mhh = mhw * MB.h / MB.w; else mhw = mhh * MB.w / MB.h;
mhw *= 1.04; mhh *= 1.04;
const mini = new THREE.OrthographicCamera(-mhw, mhw, mhh, -mhh, 1, 600);
mini.up.set(1, 0, 0); mini.position.set((M.x0 + M.x1) / 2, (M.y0 + M.y1) / 2, 300);
mini.lookAt((M.x0 + M.x1) / 2, (M.y0 + M.y1) / 2, 0); mini.layers.set(1);
document.getElementById('minisc').textContent = Math.round(2 * mhw) + ' m × ' + Math.round(2 * mhh) + ' m';
const both = o => { o.layers.enable(1); return o; };
const only1 = o => { o.layers.set(1); return o; };

scene.add(new THREE.AmbientLight(0xffffff, 0.85));
const sun = new THREE.DirectionalLight(0xffffff, 0.6); sun.position.set(-30, -20, 60); scene.add(sun);

const G = DATA.grid;
const minor = new THREE.GridHelper(G.size, G.size / 2, 0x171b22, 0x171b22);
const major = new THREE.GridHelper(G.size, G.size / 10, 0x2a313d, 0x2a313d);
[minor, major].forEach((g, i) => { g.rotation.x = Math.PI / 2; g.position.set(G.cx, G.cy, GZ - 0.03 + 0.002 * i); scene.add(g); });
if (DATA.geofence.length) {
  const pts = DATA.geofence.map(p => new THREE.Vector3(p[0], p[1], GZ + 0.04));
  const gf = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),
    new THREE.LineDashedMaterial({color: 0xe8eaee, dashSize: 0.9, gapSize: 0.6, transparent: true, opacity: 0.55, fog: false}));
  gf.computeLineDistances(); gf.frustumCulled = false; scene.add(both(gf));
}

const cam = dec(DATA.cam, Float32Array);                 // n x 7: position, target, picture shift (half-frames)
const team = dec(DATA.team, Float32Array);
const col = document.getElementById('col'), band = document.getElementById('band');
function glow(color, size) {
  const cv = document.createElement('canvas'); cv.width = cv.height = 128;
  const cx = cv.getContext('2d'); const g = cx.createRadialGradient(64, 64, 0, 64, 64, 64);
  g.addColorStop(0, 'rgba(255,255,255,0.95)'); g.addColorStop(0.18, color + 'cc'); g.addColorStop(0.5, color + '33');
  g.addColorStop(1, color + '00'); cx.fillStyle = g; cx.fillRect(0, 0, 128, 128);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({map: new THREE.CanvasTexture(cv), transparent: true,
    depthWrite: false, blending: THREE.AdditiveBlending, fog: false}));
  s.scale.set(size, size, 1); return s;
}
function quad(color) {
  const g = new THREE.Group();
  const dark = new THREE.MeshLambertMaterial({color: 0xd9dde4});
  [Math.PI / 4, -Math.PI / 4].forEach(a => { const m = new THREE.Mesh(new THREE.BoxGeometry(0.66, 0.05, 0.035), dark); m.rotation.z = a; g.add(m); });
  g.add(new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.16, 0.09), new THREE.MeshLambertMaterial({color})));
  const rm = new THREE.MeshBasicMaterial({color, transparent: true, opacity: 0.6, side: THREE.DoubleSide, depthWrite: false});
  [[1, 1], [1, -1], [-1, 1], [-1, -1]].forEach(([a, b]) => { const r = new THREE.Mesh(new THREE.CircleGeometry(0.15, 24), rm); r.position.set(0.233 * a, 0.233 * b, 0.04); g.add(r); });
  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.05, 0.16, 12), new THREE.MeshBasicMaterial({color: 0xffffff}));
  nose.rotation.z = -Math.PI / 2; nose.position.x = 0.2; g.add(nose);
  g.scale.setScalar(1.8);
  return g;
}
function wedge(color) {
  // the depth camera's view: 87 x 58 deg, pitched 20 deg down, drawn 3.2 m long
  const L = 3.2, th = Math.tan(87 / 2 * Math.PI / 180), tv = Math.tan(58 / 2 * Math.PI / 180), p = 20 * Math.PI / 180;
  const c = [[1, 1], [1, -1], [-1, -1], [-1, 1]].map(([a, b]) => {
    const x = L, y = a * L * th, z = b * L * tv;
    return new THREE.Vector3(x * Math.cos(p) + z * Math.sin(p), y, -x * Math.sin(p) + z * Math.cos(p));
  });
  const o = new THREE.Vector3(0, 0, 0);
  const g = new THREE.BufferGeometry().setFromPoints([o, c[0], c[1], o, c[1], c[2], o, c[2], c[3], o, c[3], c[0]]);
  const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({color, transparent: true, opacity: 0.085, side: THREE.DoubleSide, depthWrite: false}));
  const e = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints([o, c[0], o, c[1], o, c[2], o, c[3], c[0], c[1], c[1], c[2], c[2], c[3], c[3], c[0]]),
    new THREE.LineBasicMaterial({color, transparent: true, opacity: 0.45}));
  const grp = new THREE.Group(); grp.add(m); grp.add(e); return grp;
}
const DISC = (() => { const cv = document.createElement('canvas'); cv.width = cv.height = 64; const cx = cv.getContext('2d');
  cx.fillStyle = '#fff'; cx.beginPath(); cx.arc(32, 32, 28, 0, 6.3); cx.fill(); return new THREE.CanvasTexture(cv); })();
const light = h => { const c = new THREE.Color(h); c.lerp(new THREE.Color(0xffffff), 0.28); return c; };
const PL = DATA.plan, PW = PL.pw, PH = PL.ph;
const allImgs = [];

const drones = DATA.drones.map((d, di) => {
  const c3 = new THREE.Color(d.color), cl = light(d.color);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(dec(d.vox_xyz, Float32Array), 3));
  g.setAttribute('color', new THREE.BufferAttribute(dec(d.vox_rgb, Uint8Array), 3, true));
  const pts = new THREE.Points(g, new THREE.PointsMaterial({size: DATA.point_size, vertexColors: true, sizeAttenuation: true}));
  pts.frustumCulled = false; scene.add(pts);
  const pts1 = new THREE.Points(g, new THREE.PointsMaterial({size: 1.35, vertexColors: true, sizeAttenuation: false, fog: false}));
  pts1.frustumCulled = false; scene.add(only1(pts1));
  const vt = dec(d.vox_t, Float32Array);
  const tg = new THREE.BufferGeometry();
  tg.setAttribute('position', new THREE.BufferAttribute(dec(d.tube_v, Float32Array), 3));
  tg.setIndex(new THREE.BufferAttribute(dec(d.tube_i, Uint32Array), 1));
  const trail = new THREE.Mesh(tg, new THREE.MeshBasicMaterial({color: cl, fog: false}));
  trail.frustumCulled = false; scene.add(trail);
  const tr = dec(d.track, Float32Array);                       // n x 8
  // top-view trail: one vertex per frame
  const lp = new Float32Array(N * 3);
  for (let f = 0; f < N; f++) { lp[3 * f] = tr[8 * f]; lp[3 * f + 1] = tr[8 * f + 1]; lp[3 * f + 2] = 40; }
  const lg = new THREE.BufferGeometry(); lg.setAttribute('position', new THREE.BufferAttribute(lp, 3));
  const line1 = new THREE.Line(lg, new THREE.LineBasicMaterial({color: cl, fog: false})); line1.frustumCulled = false; scene.add(only1(line1));
  const dot1 = new THREE.Mesh(new THREE.CircleGeometry(1.5, 24), new THREE.MeshBasicMaterial({color: 0xffffff, fog: false}));
  const dot1c = new THREE.Mesh(new THREE.CircleGeometry(1.05, 24), new THREE.MeshBasicMaterial({color: cl, fog: false}));
  dot1c.position.z = 0.5; dot1.add(dot1c); scene.add(only1(dot1)); only1(dot1c);
  // vehicle glyph, view wedge, halo, drop line, ground ring
  const body = new THREE.Group(); body.add(quad(c3)); const wd = wedge(cl); body.add(wd); scene.add(body);
  const halo = glow(d.color, 3.4); scene.add(halo);
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.55, 0.68, 40), new THREE.MeshBasicMaterial({color: cl, transparent: true, opacity: 0.75, side: THREE.DoubleSide, depthWrite: false}));
  scene.add(ring);
  const dropG = new THREE.BufferGeometry(); dropG.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
  const drop = new THREE.Line(dropG, new THREE.LineBasicMaterial({color: cl, transparent: true, opacity: 0.6})); drop.frustumCulled = false; scene.add(drop);
  // goal beacon + planned path
  const goal = new THREE.Group();
  goal.add(new THREE.Mesh(new THREE.RingGeometry(0.42, 0.62, 4), new THREE.MeshBasicMaterial({color: cl, side: THREE.DoubleSide, transparent: true, opacity: 0.95, depthWrite: false, fog: false})));
  const beam = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 2.6, 8), new THREE.MeshBasicMaterial({color: cl, transparent: true, opacity: 0.75, fog: false}));
  beam.rotation.x = Math.PI / 2; beam.position.z = 1.3; goal.add(beam);
  const gs = glow(d.color, 1.7); gs.position.z = 2.6; goal.add(gs); scene.add(goal);
  const goal1 = new THREE.Mesh(new THREE.RingGeometry(0.9, 1.5, 4), new THREE.MeshBasicMaterial({color: cl, side: THREE.DoubleSide, fog: false}));
  scene.add(only1(goal1));
  const pathG = new THREE.BufferGeometry(); pathG.setAttribute('position', new THREE.BufferAttribute(new Float32Array(64 * 3), 3));
  const path = new THREE.Line(pathG, new THREE.LineDashedMaterial({color: cl, dashSize: 0.45, gapSize: 0.3, transparent: true, opacity: 0.95, fog: false}));
  path.frustumCulled = false; scene.add(both(path));
  // frontiers
  const frG = new THREE.BufferGeometry(); frG.setAttribute('position', new THREE.BufferAttribute(new Float32Array(200 * 3), 3));
  const fr = new THREE.Points(frG, new THREE.PointsMaterial({size: 0.30, color: cl, map: DISC, alphaTest: 0.4, transparent: true, opacity: 0.9, sizeAttenuation: true, fog: false}));
  fr.frustumCulled = false; scene.add(fr);
  const lab = document.createElement('div'); lab.className = 'lab'; lab.textContent = d.name;
  lab.style.color = cl.getStyle(); lab.style.borderColor = d.color; document.body.appendChild(lab);
  const card = document.createElement('div'); card.className = 'card d'; card.style.borderLeftColor = d.color; col.appendChild(card);
  // planning-map window
  const pc = document.createElement('div'); pc.className = 'card pcard';
  pc.style.width = (PW + 22) + 'px';
  pc.innerHTML = `<div class="h"><span class="pn"><span style="color:${cl.getStyle()}">${d.name}</span> · planning map</span></div>` +
    `<canvas width="${PW}" height="${PH}"></canvas>` +
    `<div class="lg">◆ goal &nbsp;· frontier &nbsp;○ ${d.peer || 'peer'} &nbsp;◇ its claim</div>`;
  band.appendChild(pc);
  const pimgs = d.pimg.map(src => { const im = new Image(); im.src = src; allImgs.push(im); return im; });
  return {d, c3, cl, pts, vt, trail, line1, dot1, body, wd, halo, ring, drop, goal, goal1, path, fr, lab, card, tr,
          alive: dec(d.alive, Uint8Array), offb: dec(d.offb, Uint8Array), icount: dec(d.icount, Int32Array),
          gi: dec(d.gi, Int16Array), fi: dec(d.fi, Int16Array), lr: dec(d.lr, Float32Array), kf: dec(d.kf, Float32Array),
          kfr: dec(d.kfr, Float32Array), wf: dec(d.wf, Float32Array), pa: dec(d.pa, Float32Array),
          obs: dec(d.obs, Float32Array), pst: dec(d.pst, Uint8Array), nplans: dec(d.nplans, Int16Array),
          pgi: dec(d.pgi, Int16Array), ptrk: dec(d.ptrk, Float32Array), pimgs,
          pctx: pc.querySelector('canvas').getContext('2d'),
          lastG: -2, lastF: -2, nFr: 0, nFrTrue: -1};
});
drones.forEach(o => { o.peerObj = drones.find(q => q.d.name === o.d.peer) || null; });

// link beam (pairs) + beads + label
const pairs = [];
for (let i = 0; i < drones.length; i++) for (let j = i + 1; j < drones.length; j++) {
  const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
  const ln = new THREE.Line(g, new THREE.LineBasicMaterial({color: 0xffffff, transparent: true, opacity: 0.5, fog: false}));
  ln.frustumCulled = false; scene.add(both(ln));
  const beads = [i, j].map(k => Array.from({length: 4}, () => { const b = glow(drones[k].d.color, 0.95); scene.add(b); return b; }));
  const lab = document.createElement('div'); lab.className = 'lab2'; document.body.appendChild(lab);
  pairs.push({a: drones[i], b: drones[j], ln, beads, lab});
}

// ---- panels ----
const teamCard = document.createElement('div'); teamCard.className = 'card'; col.appendChild(teamCard);
teamCard.innerHTML = '<div class="ttl">TEAM · 3D MAP FOOTPRINT  m²</div><div id="trows"></div>' +
  '<canvas id="chart" width="314" height="50" style="display:block;margin-top:5px"></canvas>';
const guardCard = document.createElement('div'); guardCard.className = 'card'; col.appendChild(guardCard);
guardCard.innerHTML = '<div class="ttl">REACTIVE GUARD · NEAREST OBSTACLE  m</div><div id="grows"></div>' +
  '<canvas id="gchart" width="314" height="46" style="display:block;margin-top:5px"></canvas>';
const linkCard = document.createElement('div'); linkCard.className = 'card ccard'; band.appendChild(linkCard);
linkCard.innerHTML = `<div class="ttl">COORDINATION LINK · ${DATA.lora}</div>` +
  '<div class="tbl" id="ltbl" style="grid-template-columns:1.3fr 1fr 0.8fr 0.9fr 1fr 1.25fr"></div><div id="lextra"></div><canvas id="spark" height="88"></canvas>';
const wifiCard = document.createElement('div'); wifiCard.className = 'card ccard'; band.appendChild(wifiCard);
wifiCard.innerHTML = `<div class="ttl">MAP SHARING · ${DATA.wifi}</div>` +
  '<div class="tbl" id="wtbl" style="grid-template-columns:0.9fr repeat(5,1fr)"></div><div id="wextra"></div><canvas id="kspark" height="88"></canvas>';
const SPW = Math.max(120, linkCard.clientWidth - 30), KPW = Math.max(120, wifiCard.clientWidth - 30);
document.getElementById('spark').width = SPW; document.getElementById('kspark').width = KPW;
const spark = document.getElementById('spark').getContext('2d');
const kspark = document.getElementById('kspark').getContext('2d');
const chart = document.getElementById('chart').getContext('2d');
const gchart = document.getElementById('gchart').getContext('2d');
document.getElementById('title').textContent = DATA.title;
document.getElementById('sub').textContent = DATA.subtitle;
document.getElementById('foot').textContent = DATA.footer;
document.getElementById('rate').textContent = DATA.rate === 1 ? 'real time' : '×' + DATA.rate + ' speed';
document.getElementById('intro1').textContent = DATA.title;
document.getElementById('intro2').textContent = DATA.summary.n + ' drones · onboard mapping, frontier planning and LoRa coordination · no central coordinator';
const S = DATA.summary;
const fmt = (v, d = 0) => Number(v).toLocaleString('en-US', {minimumFractionDigits: d, maximumFractionDigits: d});
const neg = s => String(s).replace('-', '−');
const dash = '<span style="color:var(--ink3)">–</span>';
(function () {
  const mm = Math.floor(S.joint_s / 60), ss = Math.round(S.joint_s - 60 * mm);
  const names = Object.keys(S.drones);
  const dist = names.reduce((s, k) => s + S.drones[k].dist, 0);
  const del = names.map(k => S.drones[k].deliv).filter(v => v > 0);
  const rs = names.map(k => S.drones[k].rssi).filter(v => isFinite(v));
  const kf = names.reduce((s, k) => s + S.drones[k].kf, 0), kb = names.reduce((s, k) => s + S.drones[k].kf_mb, 0);
  const pk = names.reduce((s, k) => s + S.drones[k].pkts, 0);
  const cells = [
    [mm + ' min ' + String(ss).padStart(2, '0') + ' s', (S.n === 2 ? 'both' : 'all ' + S.n) + ' drones in Offboard at the same time'],
    [fmt(dist) + ' m', 'flown in total (' + names.map(k => k + ' ' + fmt(S.drones[k].dist) + ' m').join(', ') + ')'],
    [fmt(S.area_team) + ' m²', 'team 3D map footprint'],
    [fmt(S.sep_max, 1) + ' m', 'largest separation between vehicles'],
    [del.length ? fmt(100 * Math.min(...del), 1) + ' %' : '–', 'LoRa delivery, worse direction'],
    [rs.length ? neg(fmt(rs.reduce((a, b) => a + b, 0) / rs.length)) + ' dBm' : '–', 'median LoRa RSSI'],
    [fmt(pk), 'LoRa packets received'],
    [fmt(kf), 'map keyframes received over WiFi (' + fmt(kb, 1) + ' MB)']];
  document.getElementById('outro1').textContent = 'Flight summary';
  document.getElementById('outrog').innerHTML = cells.map(c => `<div><div class="num">${c[0]}</div><div class="cap">${c[1]}</div></div>`).join('');
})();

function lowerBound(arr, v) { let lo = 0, hi = arr.length; while (lo < hi) { const m = (lo + hi) >> 1; if (arr[m] <= v) lo = m + 1; else hi = m; } return lo; }
const pad = n => String(n).padStart(2, '0');
function clockText(epoch) { const d = new Date(epoch * 1000); return pad(d.getHours()) + ':' + pad(d.getMinutes()) + ':' + pad(d.getSeconds()); }
const v3 = new THREE.Vector3();
function project(x, y, z) { v3.set(x, y, z).project(camera); return [(v3.x + 1) / 2 * W, (1 - v3.y) / 2 * H, v3.z]; }

function series(ctx, w, h, arrs, cols, n, lo, hi, k, pad2, labTop, labBot) {
  // per-second series drawn up to second k, a dot at the head
  ctx.clearRect(0, 0, w, h);
  const X = i => pad2 + (w - 2 * pad2) * i / Math.max(1, n - 1), Y = v => h - 3 - (h - 8) * (v - lo) / (hi - lo);
  ctx.strokeStyle = '#262b35'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(pad2, h - 2.5); ctx.lineTo(w - pad2, h - 2.5); ctx.stroke();
  arrs.forEach((a, s) => {
    ctx.strokeStyle = cols[s]; ctx.lineWidth = s === arrs.length - 1 && arrs.length > 2 ? 2.2 : 1.6; ctx.beginPath();
    let pen = false, lx = 0, ly = 0;
    for (let i = 0; i <= Math.min(k, a.length - 1); i++) {
      const v = a[i]; if (v === null || !isFinite(v)) { pen = false; continue; }
      const x = X(i), y = Y(Math.max(lo, Math.min(hi, v)));
      if (pen) ctx.lineTo(x, y); else ctx.moveTo(x, y); pen = true; lx = x; ly = y;
    }
    ctx.stroke();
    if (pen) { ctx.fillStyle = cols[s]; ctx.beginPath(); ctx.arc(lx, ly, 3, 0, 6.3); ctx.fill(); }
  });
  ctx.fillStyle = '#7a8496'; ctx.font = '10.5px system-ui';
  if (labTop) ctx.fillText(labTop, pad2 + 2, 10);
  if (labBot) ctx.fillText(labBot, pad2 + 2, h - 6);
}
const NS = DATA.area_team_sec.length;
const AMAX = Math.max(10, ...DATA.area_team_sec) * 1.08;
const lightCols = DATA.drones.map(d => light(d.color).getStyle());
const areaCols = lightCols.concat(['#eef0f3']);
const areaArrs = DATA.drones.map(d => d.area_sec).concat([DATA.area_team_sec]);
const rssiArrs = DATA.drones.map(d => d.rssi_sec);
const kfrArrs = DATA.drones.map(d => d.kfr_sec);
const KMAX = Math.max(20, ...kfrArrs.flat().filter(v => v !== null)) * 1.15;
const obsArrs = DATA.drones.map(d => d.obs_sec);

// ---- planning-map windows: team frame as flown, +x up, +y to the left ----
const pu = y => (PL.win[3] - y) / (PL.win[3] - PL.win[2]) * PW;
const pv = x => (PL.win[1] - x) / (PL.win[1] - PL.win[0]) * PH;
function diamond(g, x, y, r) { g.beginPath(); g.moveTo(x, y - r); g.lineTo(x + r, y); g.lineTo(x, y + r); g.lineTo(x - r, y); g.closePath(); }
function drawPlan(o, f) {
  const g = o.pctx, alive = o.alive[f] === 0;
  g.clearRect(0, 0, PW, PH); g.fillStyle = '#07090c'; g.fillRect(0, 0, PW, PH);
  const gi = o.pgi[f];
  if (gi >= 0) g.drawImage(o.pimgs[gi], 0, 0);
  if (PL.geofence.length) {
    g.strokeStyle = 'rgba(232,234,238,.5)'; g.lineWidth = 1; g.setLineDash([3, 3]); g.beginPath();
    PL.geofence.forEach((p, i) => { const x = pu(p[1]), y = pv(p[0]); if (i) g.lineTo(x, y); else g.moveTo(x, y); });
    g.stroke(); g.setLineDash([]);
  }
  // frontiers
  const fi = alive ? o.fi[f] : -1;
  if (fi >= 0) {
    g.fillStyle = 'rgba(255,255,255,.62)'; const F = o.d.fronts_raw[fi];
    for (let i = 0; i < F.length; i++) g.fillRect(pu(F[i][1]) - 0.7, pv(F[i][0]) - 0.7, 1.4, 1.4);
  }
  // own track so far
  const ox = pu(o.ptrk[3 * f + 1]), oy = pv(o.ptrk[3 * f]);
  g.strokeStyle = o.cl.getStyle(); g.lineWidth = 1.4; g.beginPath();
  for (let j = 0; j < f; j += 5) { const x = pu(o.ptrk[3 * j + 1]), y = pv(o.ptrk[3 * j]); if (j) g.lineTo(x, y); else g.moveTo(x, y); }
  g.lineTo(ox, oy); g.stroke();
  // what it holds about the peer: position ring, claim, a tie between them
  const k = f * 8, pc = o.peerObj ? o.peerObj.cl.getStyle() : '#ffffff';
  if (alive && o.pa[k + 2] > 0) {
    const bx = pu(o.pa[k + 1]), by = pv(o.pa[k]);
    if (o.pa[k + 5] > 0) {
      const cx = pu(o.pa[k + 4]), cy = pv(o.pa[k + 3]);
      g.strokeStyle = pc; g.globalAlpha = 0.55; g.lineWidth = 1; g.setLineDash([2, 3]); g.beginPath(); g.moveTo(bx, by); g.lineTo(cx, cy); g.stroke();
      g.setLineDash([]); g.globalAlpha = 1; g.lineWidth = 1.5; diamond(g, cx, cy, 4.2); g.stroke();
    }
    g.strokeStyle = pc; g.lineWidth = 1.8; g.beginPath(); g.arc(bx, by, 4.6, 0, 6.3); g.stroke();
  }
  // own goal and path
  const pi = alive ? o.gi[f] : -1;
  if (pi >= 0) {
    const pl = o.d.plans[pi], gx = pu(pl.gr[1]), gy = pv(pl.gr[0]);
    g.strokeStyle = '#ffffff'; g.lineWidth = 1.2; g.setLineDash([3, 2]); g.beginPath(); g.moveTo(ox, oy);
    let start = 0, best = 1e9;
    for (let i = 0; i < pl.pr.length; i++) { const dd = Math.hypot(pl.pr[i][0] - o.ptrk[3 * f], pl.pr[i][1] - o.ptrk[3 * f + 1]); if (dd < best) { best = dd; start = i; } }
    for (let i = start + 1; i < pl.pr.length; i++) g.lineTo(pu(pl.pr[i][1]), pv(pl.pr[i][0]));
    g.lineTo(gx, gy); g.stroke(); g.setLineDash([]);
    g.fillStyle = '#ffffff'; diamond(g, gx, gy, 4.6); g.fill();
    g.strokeStyle = o.d.color; g.lineWidth = 1.4; diamond(g, gx, gy, 4.6); g.stroke();
  }
  // own position, pointing along its heading
  if (alive) {
    const yaw = o.ptrk[3 * f + 2], du = -Math.sin(yaw), dv = -Math.cos(yaw);
    g.fillStyle = o.cl.getStyle(); g.strokeStyle = '#ffffff'; g.lineWidth = 1.2; g.beginPath();
    g.moveTo(ox + 8 * du, oy + 8 * dv); g.lineTo(ox - 4.5 * du + 4.6 * dv, oy - 4.5 * dv - 4.6 * du);
    g.lineTo(ox - 4.5 * du - 4.6 * dv, oy - 4.5 * dv + 4.6 * du); g.closePath(); g.fill(); g.stroke();
  }
}

window.renderAt = function (f) {
  f = Math.max(0, Math.min(N - 1, f));
  const fm = Math.min(f, NM - 1);
  const t = fm / DATA.fps * DATA.rate;
  const c = cam.subarray(f * 7, f * 7 + 7);
  camera.setViewOffset(W, H, DATA.view_dx, c[6] * H / 2, W, H);
  camera.position.set(c[0], c[1], c[2]); camera.lookAt(c[3], c[4], c[5]);
  const r = Math.hypot(c[0] - c[3], c[1] - c[4], c[2] - c[5]);
  scene.fog.near = r * 1.5; scene.fog.far = r * 4.5;
  camera.updateMatrixWorld(); camera.updateProjectionMatrix();
  const fade = f < NM ? 1 : Math.max(0.0, 1 - (f - NM) / (0.6 * DATA.fps));
  const P = [];
  drones.forEach((o, di) => {
    const nv = lowerBound(o.vt, t);
    o.pts.geometry.setDrawRange(0, nv);
    o.trail.geometry.setDrawRange(0, o.icount[f]);
    o.line1.geometry.setDrawRange(0, f + 1);
    const k = f * 8, x = o.tr[k], y = o.tr[k + 1], z = o.tr[k + 2], yaw = o.tr[k + 3];
    const alive = o.alive[f] === 0;
    P.push({x, y, z, alive});
    [o.body, o.halo, o.ring, o.drop, o.dot1].forEach(m => m.visible = alive);
    o.body.position.set(x, y, z); o.body.rotation.z = yaw;
    o.halo.position.set(x, y, z);
    o.ring.position.set(x, y, GZ + 0.05);
    o.dot1.position.set(x, y, 60);
    const air = o.tr[k + 4] > 0.3;
    o.wd.visible = air;
    const dp = o.drop.geometry.attributes.position.array;
    dp[0] = x; dp[1] = y; dp[2] = z - 0.1; dp[3] = x; dp[4] = y; dp[5] = GZ + 0.05;
    o.drop.geometry.attributes.position.needsUpdate = true;
    const [sx, sy, sz] = project(x, y, z + 0.75);
    o.lab.style.display = alive && sz < 1 ? 'block' : 'none';
    o.lab.style.opacity = fade;
    o.lab.style.left = sx.toFixed(1) + 'px'; o.lab.style.top = (sy - 6).toFixed(1) + 'px';
    // goal + path
    const gi = alive ? o.gi[f] : -1;
    o.goal.visible = o.goal1.visible = o.path.visible = gi >= 0;
    if (gi >= 0) {
      const pl = o.d.plans[gi];
      o.goal.position.set(pl.g[0], pl.g[1], GZ + 0.06); o.goal.rotation.z = f * 0.02;
      o.goal1.position.set(pl.g[0], pl.g[1], 55);
      const pa = o.path.geometry.attributes.position.array; let m = 0;
      pa[m++] = x; pa[m++] = y; pa[m++] = z;
      // the planner's path starts at the pose it planned from: skip points behind the vehicle
      let start = 0, best = 1e9;
      for (let i = 0; i < pl.p.length; i++) { const dd = Math.hypot(pl.p[i][0] - x, pl.p[i][1] - y); if (dd < best) { best = dd; start = i; } }
      for (let i = start + 1; i < pl.p.length && m < 62 * 3; i++) { pa[m++] = pl.p[i][0]; pa[m++] = pl.p[i][1]; pa[m++] = z; }
      pa[m++] = pl.g[0]; pa[m++] = pl.g[1]; pa[m++] = z;
      o.path.geometry.setDrawRange(0, m / 3); o.path.geometry.attributes.position.needsUpdate = true;
      o.path.computeLineDistances();
    }
    // frontiers
    const fi = alive ? o.fi[f] : -1;
    if (fi !== o.lastF) {
      o.lastF = fi; o.nFr = 0; o.nFrTrue = fi >= 0 ? o.d.fn[fi] : -1;
      if (fi >= 0) {
        const fa = o.fr.geometry.attributes.position.array, F = o.d.fronts[fi];
        for (let i = 0; i < F.length && i < 200; i++) { fa[3 * i] = F[i][0]; fa[3 * i + 1] = F[i][1]; fa[3 * i + 2] = GZ + 0.35; }
        o.nFr = Math.min(200, F.length); o.fr.geometry.attributes.position.needsUpdate = true;
      }
      o.fr.geometry.setDrawRange(0, o.nFr);
    }
    o.fr.visible = o.nFr > 0;
    // card
    const ob = o.offb[f] === 1, lost = o.offb[f] === 2, paused = o.offb[f] === 3;
    const st = !alive ? ['', 'NO DATA YET', 'LOG ENDED', 'NO DATA'][o.alive[f]] :
      lost ? 'PX4 FEED LOST' : paused ? 'PLANNER PAUSED' : ob ? 'OFFBOARD' : air ? 'MANUAL' : 'ON GROUND';
    const stc = lost && alive ? 'var(--bad)' : paused ? 'var(--warn)' : ob ? 'var(--good)' : 'var(--ink2)';
    o.card.innerHTML = `<div class="h"><span class="n" style="color:${o.cl.getStyle()}">${o.d.name}</span>` +
      `<span class="badge" style="color:${stc};border-color:${stc}">${st}</span></div>` +
      `<div class="kv"><div><span>altitude</span><span>${lost ? '?' : o.tr[k + 4].toFixed(1)} m</span></div>` +
      `<div><span>speed</span><span>${o.tr[k + 5].toFixed(1)} m/s</span></div>` +
      `<div><span>flown</span><span>${fmt(o.tr[k + 6])} m</span></div>` +
      `<div><span>mapped</span><span>${fmt(o.tr[k + 7])} m²</span></div>` +
      `<div><span>voxels</span><span>${fmt(nv)}</span></div>` +
      `<div><span>frontiers</span><span>${o.nFrTrue >= 0 ? o.nFrTrue : '–'}</span></div>` +
      `<div><span>goals planned</span><span>${o.nplans[f]}</span></div>` +
      `<div><span>planner</span><span>${['–', 'planning', 'executing'][o.pst[f]]}</span></div></div>`;
    drawPlan(o, f);
  });
  // links: the beads follow the age of the peer state in the bag (/swarm/peers)
  pairs.forEach(pr => {
    const ia = drones.indexOf(pr.a), ib = drones.indexOf(pr.b), A = P[ia], B = P[ib];
    const vis = A.alive && B.alive;
    pr.ln.visible = vis; pr.lab.style.display = 'none';
    pr.beads.forEach(bs => bs.forEach(b => b.visible = false));
    if (!vis) return;
    const lp = pr.ln.geometry.attributes.position.array;
    lp[0] = A.x; lp[1] = A.y; lp[2] = A.z + 0.25; lp[3] = B.x; lp[4] = B.y; lp[5] = B.z + 0.25;
    pr.ln.geometry.attributes.position.needsUpdate = true;
    const ageA = pr.a.lr[f * 8 + 6], ageB = pr.b.lr[f * 8 + 6];       // A's state of B, B's state of A
    const okA = ageA >= 0 && ageA < 1.0, okB = ageB >= 0 && ageB < 1.0;
    pr.ln.material.color.set(okA && okB ? 0x3ddc84 : okA || okB ? 0xf5b83d : 0xff5d5d);
    const ph = (f * 0.021) % 0.25;
    pr.beads[0].forEach((b, i) => { const u = ph + i * 0.25; b.visible = okB; b.position.set(A.x + (B.x - A.x) * u, A.y + (B.y - A.y) * u, A.z + 0.25 + (B.z - A.z) * u); });
    pr.beads[1].forEach((b, i) => { const u = 1 - (ph + 0.125 + i * 0.25) % 1; b.visible = okA; b.position.set(A.x + (B.x - A.x) * u, A.y + (B.y - A.y) * u, A.z + 0.25 + (B.z - A.z) * u); });
    const [mx, my, mz] = project((A.x + B.x) / 2, (A.y + B.y) / 2, (A.z + B.z) / 2 + 0.25);
    pr.lab.style.display = mz < 1 ? 'block' : 'none';
    pr.lab.style.opacity = fade;
    pr.lab.style.left = mx.toFixed(1) + 'px'; pr.lab.style.top = (my - 16).toFixed(1) + 'px';
    pr.lab.textContent = 'LoRa · ' + team[f * 2].toFixed(1) + ' m';
  });
  const hd = t => `<span class="hd">${t}</span>`, val = (ok, s) => `<span class="v">${ok ? s : dash}</span>`;
  let lt = hd('') + hd('RSSI') + hd('SNR') + hd('rate') + hd('delivered') + hd('peer state age');
  let wt = hd('') + hd('signal') + hd('link rate') + hd('keyframes') + hd('data') + hd('rate');
  const claims = [], pkts = [];
  drones.forEach(o => {
    const k = f * 8, rs = o.lr[k], sn = o.lr[k + 1], pk = o.lr[k + 2], dl = o.lr[k + 3], rt = o.lr[k + 5], ag = o.lr[k + 6];
    const lc = o.cl.getStyle();
    lt += `<span class="l" style="color:${lc}">${o.d.peer} → ${o.d.name}</span>` +
      val(rs > -900, neg(rs.toFixed(0)) + ' dBm') + val(sn > -900, neg(sn.toFixed(0)) + ' dB') +
      val(rt > -900, rt.toFixed(1) + ' /s') + val(dl > -900, (100 * dl).toFixed(1) + ' %') +
      val(ag >= 0, ag.toFixed(2) + ' s');
    claims.push(`<span style="color:${lc}">${o.d.name}</span> ${fmt(o.lr[k + 7])}`);
    pkts.push(`<span style="color:${lc}">${o.d.name}</span> ${fmt(pk)}`);
    const sg = o.wf[f * 2], tx = o.wf[f * 2 + 1], kr = o.kfr[f];
    wt += `<span class="l" style="color:${lc}">${o.d.name}</span>` +
      val(sg > -900, neg(sg.toFixed(0)) + ' dBm') + val(tx > -900, tx.toFixed(0) + ' Mb/s') +
      val(true, fmt(o.kf[f * 2])) + val(true, o.kf[f * 2 + 1].toFixed(1) + ' MB') + val(kr > -900, kr.toFixed(0) + ' kB/s');
  });
  document.getElementById('ltbl').innerHTML = lt;
  document.getElementById('wtbl').innerHTML = wt;
  document.getElementById('lextra').innerHTML =
    `<div class="row" style="margin-top:3px"><span class="l">claims received from the peer</span><span class="v">${claims.join(' · ')}</span></div>` +
    `<div class="row"><span class="l">packets received</span><span class="v">${pkts.join(' · ')}</span></div>`;
  document.getElementById('wextra').innerHTML =
    `<div class="row nt" style="margin-top:4px"><span class="l">keyframes received from the peer: 320×240 depth + pose, 1 Hz</span></div>` +
    `<div class="row nt"><span class="l">${DATA.wifi_note}</span></div>`;
  const ks = Math.min(NS - 1, Math.floor(t));
  series(spark, SPW, 88, rssiArrs, lightCols, NS, -95, -40, ks, 2, 'RSSI −40', '−95 dBm');
  series(kspark, KPW, 88, kfrArrs, lightCols, NS, 0, KMAX, ks, 2, fmt(KMAX) + ' kB/s', '0');
  series(chart, 314, 50, areaArrs, areaCols, NS, 0, AMAX, ks, 2, fmt(AMAX), '');
  series(gchart, 314, 46, obsArrs, lightCols, NS, 0, 6, ks, 2, '6 m', '0');
  const sepv = drones.length > 1 ? team[f * 2].toFixed(1) + ' m' : '–';
  document.getElementById('trows').innerHTML =
    `<div class="row"><span class="l">union</span><span class="v">${fmt(team[f * 2 + 1])} m²</span></div>` +
    `<div class="row"><span class="l">separation</span><span class="v">${sepv}</span></div>`;
  document.getElementById('grows').innerHTML = '<div class="row"><span class="l">now</span><span class="v">' +
    drones.map(o => `<span style="color:${o.cl.getStyle()}">${o.d.name}</span> ${o.obs[f] > -900 ? o.obs[f].toFixed(1) + ' m' : '–'}`).join(' · ') + '</span></div>';
  // ticker: the latest event, fading over 4.5 s of video
  let e = null; for (const x of DATA.events) { if (x.f <= fm) e = x; else break; }
  const tk = document.getElementById('ticker');
  if (e && f - e.f < 4.5 * DATA.fps && f < NM) {
    tk.style.opacity = Math.max(0, Math.min(1, (4.5 * DATA.fps - (f - e.f)) / (1.0 * DATA.fps)));
    tk.innerHTML = `<span class="c mono">${e.clock}</span><b style="color:${light(e.color).getStyle()}">${e.name}</b>&nbsp; ${e.text}`;
  } else tk.style.opacity = 0;
  document.getElementById('clock').textContent = clockText(DATA.t0 + t);
  document.getElementById('tplus').textContent = 'T+' + t.toFixed(0).padStart(3, ' ') + ' s';
  document.getElementById('intro').style.opacity = Math.max(0, Math.min(1, (2.6 * DATA.fps - f) / (0.8 * DATA.fps)));
  const oo = Math.max(0, Math.min(1, (f - NM - 0.4 * DATA.fps) / (0.8 * DATA.fps)));
  document.getElementById('outro').style.opacity = oo;
  band.style.opacity = 1 - oo;
  renderer.setViewport(0, 0, W, H); renderer.setScissorTest(false); renderer.clear();
  renderer.render(scene, camera);
  renderer.setViewport(MB.x, MB.y, MB.w, MB.h); renderer.setScissor(MB.x, MB.y, MB.w, MB.h); renderer.setScissorTest(true);
  renderer.setClearColor(0x07090c, 1); renderer.clear(); renderer.render(scene, mini); renderer.setClearColor(BG, 1);
  renderer.setScissorTest(false);
  return f;
};
Promise.all(allImgs.map(im => im.decode().catch(() => null))).then(() => { window.renderAt(0); window.READY = true; });
</script></body></html>
"""


if __name__ == '__main__':
    main()
