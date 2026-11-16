#!/usr/bin/env python3
"""fleet_3d_video.py - render the 3D replay (nvblox colour voxels + drone
trajectories in the team frame) to an MP4, headless and deterministic.

    ./viz/fleet_3d_video.py <run> [drones...] --from T --to T --out file.mp4
        [--fps 30] [--rate 1] [--size 1920x1080] [--crf 20]
        [--still T] [--still-out file.png] [--title TEXT]
        [--orbit-deg 120] [--elev-deg 35] [--azim-deg -150]

Example (the e4c pair flight, same window as e4c_pair_realtime.mp4):

    ./viz/fleet_3d_video.py e4c --from 15:29:00 --to 15:31:40 \\
        --out reports/visualization/e4c/e4c_3d_realtime.mp4

HOW IT WORKS
- Data: fleet_3d.build() (colour voxels = union of the incremental colour-layer
  markers, sorted by first-seen time; "the map at time T" is a draw range).
- Altitude comes from PX4's local position (planner CSV ned_z), not from the
  cuVSLAM odometry: on 2026-10-23 the odometry z grew with forward distance
  (camera pitch) to 5-6 m while PX4 held 1.0 m. x/y are the team-frame
  odometry, exactly as in the 2D replay.
- Rendering: a capture-only three.js page with no animation loop. renderAt(f)
  draws frame f and nothing else, so every frame is a pure function of its
  index. Headless Chrome is driven over the DevTools pipe (stdlib only, no
  websocket package), WebGL on the GPU through ANGLE/EGL when available.
- Camera: precomputed here: follows the smoothed midpoint of the drones, pulls
  back as they separate, and orbits slowly (--orbit-deg over the whole clip).
- Encoding: JPEG frames from Chrome piped into ffmpeg/libx264, yuv420p.
"""
import argparse
import base64
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from fleet_player import Fleet, parse_t, hhmmss, COLOR  # noqa: E402
from fleet_3d import build  # noqa: E402

THREE_URL = 'https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js'
THREE_CACHE = Path.home() / '.cache' / 'fleet_viz' / 'three.r128.min.js'
GPU_FLAGS = ['--use-gl=angle', '--use-angle=gl-egl', '--ignore-gpu-blocklist']


def b64(a):
    return base64.b64encode(np.ascontiguousarray(a).tobytes()).decode()


def three_source():
    if not THREE_CACHE.exists() or THREE_CACHE.stat().st_size < 100000:
        THREE_CACHE.parent.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen(THREE_URL, timeout=30) as r:
            THREE_CACHE.write_bytes(r.read())
    return THREE_CACHE.read_text()


# ---------------------------------------------------------------- geometry --
def smooth(a, sigma):
    """Gaussian smoothing along axis 0 with edge padding (no SciPy needed)."""
    a = np.asarray(a, float)
    if sigma <= 0 or len(a) < 3:
        return a.copy()
    r = int(3 * sigma)
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    pad = np.pad(a, [(r, r)] + [(0, 0)] * (a.ndim - 1), mode='edge')
    if a.ndim == 1:
        return np.convolve(pad, k, mode='valid')
    return np.stack([np.convolve(pad[:, j], k, mode='valid') for j in range(a.shape[1])], axis=1)


def tube(points, radius, sides):
    """Tube mesh along a polyline (parallel-transport frames). Index order
    follows the polyline, so drawing the first (k-1)*sides*6 indices draws
    the first k rings: a time-ordered trail is a draw range."""
    P = np.asarray(points, float)
    M = len(P)
    T = np.zeros_like(P)
    d = np.diff(P, axis=0)
    T[:-1] += d
    T[1:] += d
    n = np.linalg.norm(T, axis=1)
    last = np.array([1.0, 0.0, 0.0])
    for i in range(M):
        if n[i] > 1e-9:
            last = T[i] / n[i]
        T[i] = last
    ref = np.array([0.0, 0.0, 1.0]) if abs(T[0, 2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    N = np.zeros_like(P)
    B = np.zeros_like(P)
    v = np.cross(T[0], ref)
    N[0] = v / np.linalg.norm(v)
    B[0] = np.cross(T[0], N[0])
    for i in range(1, M):
        v = N[i - 1] - np.dot(N[i - 1], T[i]) * T[i]
        nv = np.linalg.norm(v)
        if nv < 1e-9:
            v = np.cross(T[i], ref)
            nv = np.linalg.norm(v) or 1.0
        N[i] = v / nv
        B[i] = np.cross(T[i], N[i])
    ang = 2 * np.pi * np.arange(sides) / sides
    dirs = np.cos(ang)[None, :, None] * N[:, None, :] + np.sin(ang)[None, :, None] * B[:, None, :]
    verts = (P[:, None, :] + radius * dirs).reshape(-1, 3)
    i = np.arange(M - 1)[:, None]
    k = np.arange(sides)[None, :]
    k1 = (k + 1) % sides
    a, b, c, e = i * sides + k, (i + 1) * sides + k, (i + 1) * sides + k1, i * sides + k1
    idx = np.stack([a, b, c, a, c, e], axis=-1).reshape(-1)
    return verts.astype(np.float32), dirs.reshape(-1, 3).astype(np.float32), idx.astype(np.uint32)


def altitude_fn(d, tw0, tw1):
    """PX4 altitude above the ground level seen in this window, as f(t).
    None when the planner CSV has no usable ned_z."""
    if d.sp is None or 'ned_z' not in d.sp['cols']:
        return None
    t = np.asarray(d.sp['t'], float)
    z = -np.asarray(d.sp['cols']['ned_z'], float)
    ok = np.isfinite(t) & np.isfinite(z)
    t, z = t[ok], z[ok]
    m = (t >= tw0 - 60) & (t <= tw1 + 60)
    if m.sum() < 5:
        return None
    ground = float(np.percentile(z[m], 2))
    return lambda q: np.interp(q, t, z) - ground


def level_to_flight_plane(fleet, margin=8.0):
    """Rotate each drone's map and track so that its own Offboard flight plane
    is horizontal. Returns {name: (tilt_deg, residual_rms_m)}.

    Why: the mapper levels cuVSLAM's frame with a fixed 20 deg, but the frame is
    the camera's attitude at start, so however the vehicle sat on the ground
    stays in the map. 2026-11-13: the tracks climb 0.076 m/m (delta, 4.3 deg)
    and 0.152 m/m (ghost, 8.7 deg) along x while PX4 held 0.9-1.0 m above the
    ground on its rangefinder; across y the slope is under 0.2 deg. The flight
    plane is therefore the ground plane shifted up by the flight height, and
    making it horizontal is one rigid rotation per drone. It levels to the
    terrain under the flight, not to gravity."""
    out = {}
    for d in fleet.drones:
        m = np.zeros(len(d.od_t), bool)
        for a, b in d.offboard:
            m |= (d.od_t >= a + margin) & (d.od_t <= b - margin)
        if m.sum() < 200:
            continue
        P = d.od[m, 1:4]
        A = np.column_stack([P[:, 0], P[:, 1], np.ones(len(P))])
        (sx, sy, _), *_ = np.linalg.lstsq(A, P[:, 2], rcond=None)
        nrm = np.array([-sx, -sy, 1.0])
        nrm /= np.linalg.norm(nrm)
        v = np.cross(nrm, [0.0, 0.0, 1.0])
        sn, cs = np.linalg.norm(v), nrm[2]
        R = np.eye(3)
        if sn > 1e-9:
            vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
            R = np.eye(3) + vx + vx @ vx * ((1 - cs) / sn ** 2)
        d.od[:, 1:4] = d.od[:, 1:4] @ R.T
        d.tl['vox_xyz'] = (d.tl['vox_xyz'].astype(np.float64) @ R.T).astype(np.float32)
        for i in range(len(d.od)):
            d.world_xy[i] = d.to_world(d.od[i, 0], d.od[i, 1:3])
        res = d.od[m, 3] - np.mean(d.od[m, 3])
        out[d.name] = (math.degrees(math.atan(math.hypot(sx, sy))), float(res.std()))
        d.level_R, d.level_plane = R, (float(sx), float(sy))     # for 2D items in the drone's frame
    return out


def px4_stale(d, ft, min_s=3.0, min_travel=1.0):
    """Frames where the planner's copy of PX4's position was frozen while the
    drone moved: PX4 -> Jetson feed lost (ghost, 2026-10-23 14:22:25). The
    planner CSV then repeats the last PX4 values, so its altitude and Offboard
    flag mean nothing from that row on."""
    stale = np.zeros(len(ft), bool)
    if d.sp is None or not all(k in d.sp['cols'] for k in ('ned_x', 'ned_y', 'ned_z')):
        return stale
    t = np.asarray(d.sp['t'], float)
    v = np.column_stack([np.asarray(d.sp['cols'][k], float) for k in ('ned_x', 'ned_y', 'ned_z')])
    if len(t) < 3:
        return stale
    same = np.r_[False, np.all(v[1:] == v[:-1], axis=1)]
    i = 0
    while i < len(t):
        if not same[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(t) and same[j + 1]:
            j += 1
        a, b = t[i - 1], t[j]
        to_end = j == len(t) - 1
        # A feed loss is the planner logging on with frozen PX4 values. Two equal
        # rows either side of a hole in the log are the planner process itself
        # pausing (delta, 2026-11-13 15:03:49: 6.4 s without a row while the
        # vehicle flew on and vio_bridge kept feeding PX4), not a lost feed.
        if (j - i + 2) < 2.0 * (b - a):
            i = j + 1
            continue
        if b - a >= min_s:
            m = (d.od_t >= a) & (d.od_t <= b)
            travel = np.linalg.norm(np.diff(d.world_xy[m], axis=0), axis=1).sum() if m.sum() > 1 else 0.0
            if travel >= min_travel:
                stale |= (ft >= a) & ((ft <= b) | to_end)
        i = j + 1
    return stale


def drone_tracks(fleet, tw0, tw1, frame_t, ground_z, ring_step=0.05, radius=0.07, sides=10, trail_z='px4'):
    out = []
    for d in fleet.drones:
        if not len(d.od_t):
            continue
        ft = frame_t
        x = np.interp(ft, d.od_t, d.world_xy[:, 0])
        y = np.interp(ft, d.od_t, d.world_xy[:, 1])
        yaw = np.interp(ft, d.od_t, np.unwrap(d.world_yaw))
        alt = altitude_fn(d, tw0, tw1)
        alt_px4 = None
        if alt is not None and trail_z == 'px4':
            z = ground_z + 0.30 + np.clip(alt(ft), 0.0, None)
            zsrc = 'PX4 altitude'
        else:
            # the map's own frame: the trail then sits where the mapper had the camera
            z = smooth(np.interp(ft, d.od_t, d.od[:, 3]), 4)
            zsrc = 'odometry z' + ('' if alt is not None else ' (no PX4 altitude in the planner CSV)')
            if alt is not None:
                alt_px4 = np.clip(alt(ft), 0.0, None)
        # alive: odometry within 3 s of the frame
        j = np.clip(np.searchsorted(d.od_t, ft), 1, len(d.od_t) - 1)
        gap = np.minimum(np.abs(d.od_t[j] - ft), np.abs(d.od_t[j - 1] - ft))
        alive = gap < 3.0
        # 0 live, 1 log not started yet, 2 log ended, 3 gap in the log
        status = np.where(alive, 0, np.where(ft < d.od_t[0], 1, np.where(ft > d.od_t[-1], 2, 3))).astype(np.uint8)
        # trail rings: a new ring every ring_step metres of travel
        P = np.column_stack([x, y, z])
        keep = [0]
        for f in range(1, len(P)):
            if alive[f] and np.linalg.norm(P[f] - P[keep[-1]]) >= ring_step:
                keep.append(f)
        if len(keep) < 2:
            keep.append(len(P) - 1)
        keep = np.array(keep)
        verts, norms, idx = tube(P[keep], radius, sides)
        rings_at = np.searchsorted(keep, np.arange(len(ft)), 'right')   # rings with frame index <= f
        icount = (np.maximum(rings_at - 1, 0) * sides * 6).astype(np.int32)
        offb = np.zeros(len(ft), np.uint8)
        for a, b in d.offboard:
            offb[(ft >= a) & (ft <= b)] = 1
        stale = px4_stale(d, ft)
        offb[stale] = 2                       # PX4 feed lost: state and altitude unknown
        speed = np.zeros(len(ft))
        if len(ft) > 1:
            dt = max(1e-3, ft[1] - ft[0])
            speed = smooth(np.r_[0.0, np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1) / dt], 6)
        if stale.any():
            k0 = int(np.argmax(stale))
            zsrc += '; PX4 feed lost %s-%s (altitude held at the last value)' % (
                hhmmss(ft[k0]), hhmmss(ft[len(ft) - 1 - int(np.argmax(stale[::-1]))]))
        out.append(dict(d=d, x=x, y=y, z=z, yaw=yaw, alive=alive, status=status, offb=offb, speed=speed,
                        alt=(alt_px4 if alt_px4 is not None else z - ground_z - 0.30),
                        verts=verts, norms=norms, idx=idx, icount=icount,
                        zsrc=zsrc))
    return out


def camera_path(tracks, vox_xy, vox_t, trel, fps, orbit_deg, elev_deg, azim_deg, ground_z,
                fov_deg, aspect, recent_s=0.0):
    """Per-frame camera position + target (team frame, z up).

    The camera orbits slowly at a fixed elevation. For every frame it backs off
    just far enough that the key points project inside a safe area clear of
    the title and the tiles: both drones (with their labels), their trail
    (the last `recent_s` seconds, or all of it so far when recent_s <= 0,
    sampled every 2 s) and the extent of the map mapped so far. The
    distance is max-filtered and smoothed so nothing is cut off and the zoom
    never jitters."""
    n = len(trel)
    # map extent (2-98 percentile box) at 2 s steps
    steps = np.arange(0.0, trel[-1] + 2.001, 2.0)
    boxes = []
    for st in steps:
        k = int(np.searchsorted(vox_t, st, 'right'))
        if k > 200:
            bx = np.percentile(vox_xy[:k, 0], [2, 98])
            by = np.percentile(vox_xy[:k, 1], [2, 98])
            boxes.append(np.array([[bx[0], by[0]], [bx[0], by[1]], [bx[1], by[0]], [bx[1], by[1]]]))
        else:
            boxes.append(np.zeros((0, 2)))
    step = 2 * fps
    span = int(recent_s * fps) if recent_s > 0 else None

    def keys(f):
        pts = []
        j0 = 0 if span is None else max(0, f - span)
        j = np.r_[np.arange(f, j0 - 1, -step), j0]
        for t in tracks:
            live = t['alive'][j]
            if live.any():
                jj = j[live]
                pts.append(np.column_stack([t['x'][jj], t['y'][jj], t['z'][jj]]))
            pts.append([[t['x'][f], t['y'][f], t['z'][f]], [t['x'][f], t['y'][f], t['z'][f] + 1.3]])
        b = boxes[min(len(boxes) - 1, int(trel[f] // 2.0))]
        if len(b):
            pts.append(np.column_stack([b, np.full(len(b), ground_z)]))
        return np.vstack(pts)

    ctr = np.empty((n, 2))
    for f in range(n):
        k = keys(f)
        ctr[f] = [(k[:, 0].min() + k[:, 0].max()) / 2, (k[:, 1].min() + k[:, 1].max()) / 2]
    ctr = smooth(ctr, 2.5 * fps)
    tz = ground_z + 0.8
    theta = math.radians(azim_deg) + math.radians(orbit_deg) * np.linspace(0.0, 1.0, n)
    phi = math.radians(90.0 - elev_deg)
    tanv = math.tan(math.radians(fov_deg) / 2)
    tanh = tanv * aspect

    def fits(k, c, r, th):
        u = np.array([math.sin(phi) * math.cos(th), math.sin(phi) * math.sin(th), math.cos(phi)])
        w = -u
        right = np.cross(w, [0.0, 0.0, 1.0])
        right /= np.linalg.norm(right)
        up = np.cross(right, w)
        d = k - (c + r * u)
        z = d @ w
        if (z <= 0.5).any():
            return False
        x = (d @ right) / (z * tanh)
        y = (d @ up) / (z * tanv)
        return bool((np.abs(x) <= 0.84).all() and (y >= -0.80).all() and (y <= 0.58).all())

    r_req = np.empty(n)
    for f in range(n):
        c = np.array([ctr[f, 0], ctr[f, 1], tz])
        kf = keys(f)
        lo, hi = 6.0, 250.0
        if fits(kf, c, lo, theta[f]):
            r_req[f] = lo
            continue
        for _ in range(22):
            mid = 0.5 * (lo + hi)
            if fits(kf, c, mid, theta[f]):
                hi = mid
            else:
                lo = mid
        r_req[f] = hi
    win = int(2.0 * fps)
    padded = np.pad(r_req, win, mode='edge')
    r = np.lib.stride_tricks.sliding_window_view(padded, 2 * win + 1).max(axis=1)
    r = np.clip(smooth(r, 1.5 * fps), 16.0, 250.0)
    r = np.maximum(r, r_req)          # the smoothing never undercuts a frame's need
    pos = np.column_stack([ctr[:, 0] + r * math.sin(phi) * np.cos(theta),
                           ctr[:, 1] + r * math.sin(phi) * np.sin(theta),
                           np.full(n, tz) + r * math.cos(phi)])
    tgt = np.column_stack([ctr, np.full(n, tz)])
    return np.hstack([pos, tgt]).astype(np.float32), r


# ------------------------------------------------------------------ chrome --
class Chrome:
    """Headless Chrome over --remote-debugging-pipe (fd 3 in, fd 4 out)."""

    def __init__(self, w, h, flags=(), log=os.devnull):
        self.tmp = tempfile.mkdtemp(prefix='fleet3d_chrome_')
        exe = next((p for p in ('google-chrome', 'google-chrome-stable', 'chromium-browser', 'chromium')
                    if shutil.which(p)), None)
        if exe is None:
            sys.exit('no Chrome/Chromium found')
        cmd = [exe, '--headless=new', '--remote-debugging-pipe', '--user-data-dir=' + self.tmp,
               '--no-first-run', '--no-default-browser-check', '--disable-extensions',
               '--hide-scrollbars', '--mute-audio', '--window-size=%d,%d' % (w, h),
               '--force-device-scale-factor=1', '--allow-file-access-from-files',
               *flags, 'about:blank']
        self.p = subprocess.Popen(['sh', '-c', 'exec "$@" 3<&0 4>&1 </dev/null >/dev/null', 'sh'] + cmd,
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=open(log, 'wb'))
        self.buf = bytearray()
        self.n = 0
        self.sid = None

    def _read(self):
        while True:
            i = self.buf.find(b'\0')
            if i >= 0:
                raw = bytes(self.buf[:i])
                del self.buf[:i + 1]
                return json.loads(raw)
            chunk = os.read(self.p.stdout.fileno(), 1 << 22)
            if not chunk:
                raise RuntimeError('Chrome closed the DevTools pipe')
            self.buf += chunk

    def send(self, method, params=None, page=True):
        self.n += 1
        msg = {'id': self.n, 'method': method, 'params': params or {}}
        if page and self.sid:
            msg['sessionId'] = self.sid
        self.p.stdin.write(json.dumps(msg).encode() + b'\0')
        self.p.stdin.flush()
        while True:
            m = self._read()
            if m.get('id') == self.n:
                if 'error' in m:
                    raise RuntimeError('%s: %s' % (method, m['error']))
                return m['result']

    def open(self, url, w, h):
        tid = self.send('Target.createTarget', {'url': 'about:blank'}, page=False)['targetId']
        self.sid = self.send('Target.attachToTarget', {'targetId': tid, 'flatten': True}, page=False)['sessionId']
        self.send('Emulation.setDeviceMetricsOverride',
                  {'width': w, 'height': h, 'deviceScaleFactor': 1, 'mobile': False})
        self.send('Page.navigate', {'url': url})

    def js(self, expr, timeout_s=None):
        r = self.send('Runtime.evaluate', {'expression': expr, 'returnByValue': True, 'awaitPromise': True})
        if 'exceptionDetails' in r:
            raise RuntimeError('JS error: %s' % json.dumps(r['exceptionDetails'])[:800])
        return r['result'].get('value')

    def shot(self, fmt='jpeg', quality=95):
        p = {'format': fmt, 'optimizeForSpeed': True}
        if fmt == 'jpeg':
            p['quality'] = quality
        return base64.b64decode(self.send('Page.captureScreenshot', p)['data'])

    def close(self):
        try:
            self.send('Browser.close', page=False)
        except Exception:
            pass
        try:
            self.p.wait(10)
        except Exception:
            self.p.kill()
        shutil.rmtree(self.tmp, ignore_errors=True)


def start_ffmpeg(a, out):
    return subprocess.Popen(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                             '-f', 'image2pipe', '-c:v', 'png' if a.frame_format == 'png' else 'mjpeg',
                             '-framerate', str(a.fps), '-i', '-',
                             '-c:v', 'libx264', '-preset', a.preset, '-crf', str(a.crf),
                             '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(out)],
                            stdin=subprocess.PIPE)


def open_page(uri, W, H, flags, log):
    chrome = Chrome(W, H, flags=flags, log=log)
    chrome.open(uri, W, H)
    t0 = time.time()
    while not chrome.js('window.READY === true'):
        if time.time() - t0 > 180:
            chrome.close()
            raise RuntimeError('capture page never became ready; see ' + log)
        time.sleep(0.2)
    return chrome


def render_range(uri, W, H, a, f0, f1, out, label, flags, log, errors):
    """Render frames [f0, f1) of the capture page into one MP4."""
    chrome = ff = None
    try:
        chrome = open_page(uri, W, H, flags, log)
        print('  %sWebGL: %s' % (label, chrome.js(
            "(()=>{const c=document.getElementById('c');const g=c.getContext('webgl2')||c.getContext('webgl');"
            "const e=g.getExtension('WEBGL_debug_renderer_info');"
            "return e?g.getParameter(e.UNMASKED_RENDERER_WEBGL):'?'})()")), flush=True)
        ff = start_ffmpeg(a, out)
        t_cap = last = time.time()
        for f in range(f0, f1):
            chrome.js('renderAt(%d)' % f)
            ff.stdin.write(chrome.shot(a.frame_format, a.jpeg_quality))
            if time.time() - last > 30 or f == f1 - 1:
                last = time.time()
                done = f - f0 + 1
                rate = done / (time.time() - t_cap)
                print('  %sframe %d/%d  %.1f fps  eta %.0f s' % (label, done, f1 - f0, rate,
                                                               (f1 - f0 - done) / rate), flush=True)
        ff.stdin.close()
        if ff.wait() != 0:
            raise RuntimeError('ffmpeg failed for ' + str(out))
        ff = None
    except Exception as e:      # reported by the caller
        errors.append('%s%s' % (label, e))
    finally:
        if ff is not None:
            ff.kill()
        if chrome is not None:
            chrome.close()


# -------------------------------------------------------------------- page --
PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>3D replay capture</title>
<style>
:root{--bg:#fcfcfb;--ink:#0b0b0b;--ink2:#52514e;--line:#e6e5e1;--good:#0ca30c}
html,body{margin:0;width:__W__px;height:__H__px;background:var(--bg);color:var(--ink);
  font:15px/1.35 system-ui,'Ubuntu','DejaVu Sans',sans-serif;overflow:hidden}
#c{position:absolute;left:0;top:0;width:__W__px;height:__H__px;display:block}
#hud{position:absolute;left:36px;top:28px}
#hud h1{font-size:25px;font-weight:650;margin:0 0 2px;letter-spacing:.1px}
#hud .sub{font-size:15px;color:var(--ink2);margin-bottom:10px}
#clock{font:600 30px ui-monospace,'DejaVu Sans Mono',monospace;letter-spacing:.5px}
#tplus{font:16px ui-monospace,'DejaVu Sans Mono',monospace;color:var(--ink2);margin-left:14px}
#tiles{position:absolute;right:36px;top:28px;display:flex;flex-direction:column;gap:10px;width:300px}
.tile{border:1px solid var(--line);border-left:6px solid;border-radius:8px;padding:9px 13px;
  background:rgba(252,252,251,.88)}
.tile .h{display:flex;justify-content:space-between;align-items:baseline}
.tile .n{font-weight:650;font-size:18px}
.tile .s{font-size:13px;font-weight:700;letter-spacing:.6px}
.tile .v{font:14px ui-monospace,'DejaVu Sans Mono',monospace;color:var(--ink2);margin-top:3px;white-space:pre}
.lab{position:absolute;transform:translate(-50%,-100%);font-weight:650;font-size:16px;padding:2px 9px;
  border-radius:11px;background:rgba(252,252,251,.9);border:2px solid;white-space:nowrap}
#foot{position:absolute;left:36px;bottom:24px;font-size:13px;color:var(--ink2)}
</style></head><body>
<canvas id="c" width="__W__" height="__H__"></canvas>
<div id="hud"><h1 id="title"></h1><div class="sub" id="sub"></div>
<div><span id="clock"></span><span id="tplus"></span></div></div>
<div id="tiles"></div>
<div id="foot"></div>
<script>__THREE_SRC__</script>
<script>
const DATA = __DATA__;
const dec = (s, T) => new T(Uint8Array.from(atob(s), c => c.charCodeAt(0)).buffer);
const W = __W__, H = __H__;
const renderer = new THREE.WebGLRenderer({canvas: document.getElementById('c'), antialias: true,
                                          preserveDrawingBuffer: true});
renderer.setPixelRatio(1); renderer.setSize(W, H, false);
const scene = new THREE.Scene();
const BG = 0xfcfcfb;
scene.background = new THREE.Color(BG);
scene.fog = new THREE.Fog(BG, 60, 200);
const camera = new THREE.PerspectiveCamera(DATA.fov, W / H, 0.1, 3000);
camera.up.set(0, 0, 1);
scene.add(new THREE.AmbientLight(0xffffff, 0.72));
const sun = new THREE.DirectionalLight(0xffffff, 0.55); sun.position.set(-30, -20, 60); scene.add(sun);

// ground grids: 2 m minor, 10 m major, at the map's ground level
const G = DATA.grid;
const minor = new THREE.GridHelper(G.size, G.size / 2, 0xe9e8e4, 0xe9e8e4);
const major = new THREE.GridHelper(G.size, G.size / 10, 0xcfcdc7, 0xcfcdc7);
[minor, major].forEach((g, i) => { g.rotation.x = Math.PI / 2; g.position.set(G.cx, G.cy, G.z + 0.002 * i); scene.add(g); });

const N = DATA.n_frames;
const cam = dec(DATA.cam, Float32Array);           // n x 6: position xyz, target xyz
const tiles = document.getElementById('tiles');
const drones = DATA.drones.map(d => {
  const col = new THREE.Color(d.color);
  // colour voxels, sorted by first-seen time
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(dec(d.vox_xyz, Float32Array), 3));
  g.setAttribute('color', new THREE.BufferAttribute(dec(d.vox_rgb, Uint8Array), 3, true));
  const pts = new THREE.Points(g, new THREE.PointsMaterial({size: DATA.point_size, vertexColors: true, sizeAttenuation: true}));
  pts.frustumCulled = false; scene.add(pts);
  const vt = dec(d.vox_t, Float32Array);
  // trail tube
  const tg = new THREE.BufferGeometry();
  tg.setAttribute('position', new THREE.BufferAttribute(dec(d.tube_v, Float32Array), 3));
  tg.setAttribute('normal', new THREE.BufferAttribute(dec(d.tube_n, Float32Array), 3));
  tg.setIndex(new THREE.BufferAttribute(dec(d.tube_i, Uint32Array), 1));
  const trail = new THREE.Mesh(tg, new THREE.MeshLambertMaterial({color: col}));
  trail.frustumCulled = false; scene.add(trail);
  // body: sphere + white nose, ground shadow, drop line
  const body = new THREE.Mesh(new THREE.SphereGeometry(0.32, 28, 18), new THREE.MeshLambertMaterial({color: col}));
  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.14, 0.55, 16), new THREE.MeshLambertMaterial({color: 0xffffff}));
  nose.rotation.z = -Math.PI / 2; nose.position.x = 0.5; body.add(nose); scene.add(body);
  const shadow = new THREE.Mesh(new THREE.CircleGeometry(0.45, 32),
    new THREE.MeshBasicMaterial({color: 0x000000, transparent: true, opacity: 0.18, depthWrite: false}));
  scene.add(shadow);
  const dropG = new THREE.BufferGeometry();
  dropG.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
  const drop = new THREE.Line(dropG, new THREE.LineBasicMaterial({color: col, transparent: true, opacity: 0.55}));
  drop.frustumCulled = false; scene.add(drop);
  const lab = document.createElement('div'); lab.className = 'lab'; lab.textContent = d.name;
  lab.style.color = d.color; lab.style.borderColor = d.color; document.body.appendChild(lab);
  const tile = document.createElement('div'); tile.className = 'tile'; tile.style.borderLeftColor = d.color;
  tiles.appendChild(tile);
  return {d, pts, vt, trail, body, shadow, drop, lab, tile, nVox: vt.length,
          tr: dec(d.track, Float32Array), alive: dec(d.alive, Uint8Array), offb: dec(d.offb, Uint8Array),
          icount: dec(d.icount, Int32Array)};
});
document.getElementById('title').textContent = DATA.title;
document.getElementById('sub').textContent = DATA.subtitle;
document.getElementById('foot').textContent = DATA.footer;
function lowerBound(arr, v) { let lo = 0, hi = arr.length; while (lo < hi) { const m = (lo + hi) >> 1; if (arr[m] <= v) lo = m + 1; else hi = m; } return lo; }
const pad = n => String(n).padStart(2, '0');
function clockText(epoch) { const d = new Date(epoch * 1000); return pad(d.getHours()) + ':' + pad(d.getMinutes()) + ':' + pad(d.getSeconds()); }
const v3 = new THREE.Vector3();
window.renderAt = function (f) {
  f = Math.max(0, Math.min(N - 1, f));
  const t = f / DATA.fps * DATA.rate;                 // seconds since the window start
  const c = cam.subarray(f * 6, f * 6 + 6);
  camera.position.set(c[0], c[1], c[2]); camera.lookAt(c[3], c[4], c[5]);
  const r = Math.hypot(c[0] - c[3], c[1] - c[4], c[2] - c[5]);
  scene.fog.near = r * 1.4; scene.fog.far = r * 4.0;
  camera.updateMatrixWorld(); camera.updateProjectionMatrix();
  drones.forEach(o => {
    const nv = lowerBound(o.vt, t);
    o.pts.geometry.setDrawRange(0, nv);
    o.trail.geometry.setDrawRange(0, o.icount[f]);
    const k = f * 7, x = o.tr[k], y = o.tr[k + 1], z = o.tr[k + 2], yaw = o.tr[k + 3];
    const alive = o.alive[f] === 0;
    o.body.visible = o.shadow.visible = o.drop.visible = alive;
    o.body.position.set(x, y, z); o.body.rotation.z = yaw;
    o.shadow.position.set(x, y, DATA.grid.z + 0.01);
    const dp = o.drop.geometry.attributes.position.array;
    dp[0] = x; dp[1] = y; dp[2] = z - 0.32; dp[3] = x; dp[4] = y; dp[5] = DATA.grid.z + 0.01;
    o.drop.geometry.attributes.position.needsUpdate = true;
    v3.set(x, y, z + 0.45).project(camera);
    const sx = (v3.x + 1) / 2 * W, sy = (1 - v3.y) / 2 * H;
    o.lab.style.display = alive && v3.z < 1 ? 'block' : 'none';
    o.lab.style.left = sx.toFixed(1) + 'px'; o.lab.style.top = (sy - 8).toFixed(1) + 'px';
    const ob = o.offb[f] === 1, lost = o.offb[f] === 2, air = o.tr[k + 4] > 0.3;
    const st = !alive ? ['', 'NO DATA YET', 'LOG ENDED', 'NO DATA'][o.alive[f]] :
      lost ? 'PX4 FEED LOST' : ob ? 'OFFBOARD' : air ? 'MANUAL' : 'ON GROUND';
    const stc = lost && alive ? '#d03b3b' : ob ? 'var(--good)' : 'var(--ink2)';
    o.tile.innerHTML = `<div class="h"><span class="n" style="color:${o.d.color}">${o.d.name}</span>` +
      `<span class="s" style="color:${stc}">${st}</span></div>` +
      `<div class="v">alt ${lost ? '  ? ' : o.tr[k + 4].toFixed(1)} m   ${o.tr[k + 5].toFixed(1)} m/s\nmap ${nv.toLocaleString('en-US')} voxels</div>`;
  });
  document.getElementById('clock').textContent = clockText(DATA.t0 + t);
  document.getElementById('tplus').textContent = 'T+' + t.toFixed(1) + ' s' + (DATA.rate !== 1 ? `  (x${DATA.rate})` : '');
  renderer.render(scene, camera);
  return f;
};
window.renderAt(0);
window.READY = true;
</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('drones', nargs='*')
    ap.add_argument('--bags', default=str(HERE.parent / 'bags'))
    ap.add_argument('--from', dest='t_from')
    ap.add_argument('--to', dest='t_to')
    ap.add_argument('--out', required=True, help='output .mp4')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--rate', type=float, default=1.0, help='playback speed (1 = real time)')
    ap.add_argument('--size', default='1920x1080')
    ap.add_argument('--crf', type=int, default=20)
    ap.add_argument('--preset', default='slow')
    ap.add_argument('--jpeg-quality', type=int, default=95)
    ap.add_argument('--frame-format', choices=('png', 'jpeg'), default='png',
                    help='intermediate frames from Chrome (png = lossless, no JPEG noise for x264)')
    ap.add_argument('--voxel-res', type=float, default=0.05)
    ap.add_argument('--max-voxels', type=int, default=1500000)
    ap.add_argument('--point-size', type=float, default=0.10, help='voxel point size (m)')
    ap.add_argument('--fov', type=float, default=42.0, help='vertical field of view (deg)')
    ap.add_argument('--orbit-deg', type=float, default=120.0, help='camera orbit over the whole clip')
    ap.add_argument('--elev-deg', type=float, default=38.0, help='camera elevation above the horizon')
    ap.add_argument('--azim-deg', type=float, default=-150.0,
                    help='starting camera azimuth in the team frame (180 = behind the launch line)')
    ap.add_argument('--frame-trail-s', type=float, default=0.0,
                    help='keep this many seconds of each trail in frame (0 = the whole trail so far)')
    ap.add_argument('--title')
    ap.add_argument('--subtitle')
    ap.add_argument('--still', help='also save a PNG of this time (HH:MM:SS or +s); default: widest moment')
    ap.add_argument('--still-out')
    ap.add_argument('--frames', help='render only frames A:B (testing)')
    ap.add_argument('--stills-only', help='comma-separated times: save PNGs only, no video (testing)')
    ap.add_argument('--software-gl', action='store_true', help='force SwiftShader instead of the GPU')
    ap.add_argument('--jobs', type=int, default=1,
                    help='render N frame ranges in parallel (N Chrome instances), then join losslessly')
    ap.add_argument('--no-still', action='store_true')
    ap.add_argument('--level-flight', action='store_true',
                    help='rotate each map so its own flight plane is horizontal (see level_to_flight_plane)')
    ap.add_argument('--trail-z', choices=('px4', 'odom'), default='px4',
                    help="trail height: PX4 altitude above the lowest voxels, or the odometry z the mapper used")
    a = ap.parse_args()

    t_start = time.time()
    W, H = (int(v) for v in a.size.lower().split('x'))
    fleet = Fleet(a.run, a.drones, a.bags, rot='0')
    if a.level_flight:
        for name, (tilt, rms) in level_to_flight_plane(fleet).items():
            print('%-13s levelled to its flight plane: tilt removed %.2f deg, flight-height rms %.2f m' % (name, tilt, rms),
                  flush=True)
    tw0, tw1 = fleet.default_window()
    if a.t_from:
        tw0 = parse_t(a.t_from, fleet.t0)
    if a.t_to:
        tw1 = parse_t(a.t_to, fleet.t0)
    tw0, tw1 = max(tw0, fleet.t0), min(tw1, fleet.t1)
    n = int(round((tw1 - tw0) / a.rate * a.fps)) + 1
    frame_t = tw0 + np.arange(n) / a.fps * a.rate

    data = build(fleet, tw0, tw1, a.voxel_res, a.max_voxels)
    vz = []
    for d in fleet.drones:
        m = d.tl['vox_t'] <= tw1
        if m.any():
            vz.append(d.tl['vox_xyz'][m][:, 2])
    ground_z = float(np.percentile(np.concatenate(vz), 2)) if vz else -0.4
    tracks = drone_tracks(fleet, tw0, tw1, frame_t, ground_z, trail_z=a.trail_z)
    if not tracks:
        sys.exit('no odometry in this window')
    vxy, vtt = [], []
    for dd in data['drones']:
        if dd['n_vox']:
            vxy.append(np.frombuffer(base64.b64decode(dd['vox_xyz']), np.float32).reshape(-1, 3)[:, :2])
            vtt.append(np.frombuffer(base64.b64decode(dd['vox_t']), np.float32))
    vxy = np.concatenate(vxy) if vxy else np.zeros((0, 2))
    vtt = np.concatenate(vtt) if vtt else np.zeros(0)
    o = np.argsort(vtt, kind='stable')
    t_cam = time.time()
    cam, cam_r = camera_path(tracks, vxy[o], vtt[o], frame_t - tw0, a.fps, a.orbit_deg, a.elev_deg,
                             a.azim_deg, ground_z, a.fov, W / H, a.frame_trail_s)
    print('camera path %.1f s  distance %.0f-%.0f m' % (time.time() - t_cam, cam_r.min(), cam_r.max()), flush=True)

    by_name = {d['name']: d for d in data['drones']}
    pdrones = []
    for tr in tracks:
        dd = by_name[tr['d'].name]
        track = np.column_stack([tr['x'], tr['y'], tr['z'], tr['yaw'], tr['alt'], tr['speed'],
                                 np.zeros(n)]).astype(np.float32)
        pdrones.append(dict(name=dd['name'], color=COLOR[dd['name']], vox_xyz=dd['vox_xyz'],
                            vox_rgb=dd['vox_rgb'], vox_t=dd['vox_t'], n_vox=dd['n_vox'],
                            tube_v=b64(tr['verts']), tube_n=b64(tr['norms']), tube_i=b64(tr['idx']),
                            icount=b64(tr['icount']), track=b64(track),
                            alive=b64(tr['status']), offb=b64(tr['offb'])))
        print('%-13s %7d voxels  %5d trail rings  z from %s' % (
            dd['name'], dd['n_vox'], len(tr['verts']) // 10, tr['zsrc']), flush=True)
    allx = np.concatenate([t['x'] for t in tracks])
    ally = np.concatenate([t['y'] for t in tracks])
    size = int(math.ceil((max(np.ptp(allx), np.ptp(ally)) + 80) / 10.0) * 10)
    names = ' + '.join(t['d'].name for t in tracks)
    day = time.strftime('%Y-%m-%d', time.localtime(tw0))
    page_data = dict(
        fps=a.fps, rate=a.rate, n_frames=n, t0=tw0, fov=a.fov, point_size=a.point_size,
        cam=b64(cam), drones=pdrones,
        grid=dict(size=size, cx=float(round(np.mean(allx) / 10) * 10), cy=float(round(np.mean(ally) / 10) * 10),
                  z=ground_z),
        title=a.title or f'{a.run} · {names} · 3D replay',
        subtitle=a.subtitle or f'field test {day} · nvblox colour voxels appear as each drone maps them',
        footer=f'run {a.run} · team frame · grid 2 m / 10 m · altitude from PX4 local position · '
               f'{"real time" if a.rate == 1 else "x%g" % a.rate}')
    tmpdir = Path(tempfile.mkdtemp(prefix='fleet3d_page_'))
    page = tmpdir / 'capture.html'
    page.write_text(PAGE.replace('__W__', str(W)).replace('__H__', str(H))
                    .replace('__THREE_SRC__', three_source())
                    .replace('__DATA__', json.dumps(page_data)))
    print('window %s-%s  %d frames @ %d fps  page %.1f MB  prep %.1f s' % (
        hhmmss(tw0), hhmmss(tw1), n, a.fps, page.stat().st_size / 1e6, time.time() - t_start), flush=True)

    flags = ['--enable-unsafe-swiftshader'] if a.software_gl else GPU_FLAGS
    uri = page.as_uri()
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    def still_frame(spec):
        if spec:
            ts = parse_t(spec, tw0) if not spec.startswith('+') else tw0 + float(spec[1:])
            return int(np.clip(round((ts - tw0) / a.rate * a.fps), 0, n - 1))
        return int(np.argmax(cam_r))

    ok = False
    try:
        if a.stills_only:
            chrome = open_page(uri, W, H, flags, str(tmpdir / 'chrome.log'))
            try:
                for i, spec in enumerate(a.stills_only.split(',')):
                    f = still_frame(spec)
                    chrome.js('renderAt(%d)' % f)
                    p = out.with_name(out.stem + '_test%d.png' % i)
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
            render_range(uri, W, H, a, f0, f1, out, '', flags, str(tmpdir / 'chrome.log'), errors)
        else:
            import threading
            cuts = np.linspace(f0, f1, jobs + 1).round().astype(int)
            segs = [tmpdir / ('seg%02d.mp4' % i) for i in range(jobs)]
            th = [threading.Thread(target=render_range, args=(
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
                                    '-safe', '0', '-i', str(lst), '-c', 'copy', '-movflags', '+faststart',
                                    str(out)])
                if r.returncode != 0:
                    errors.append('joining the segments failed')
        if errors:
            raise RuntimeError('; '.join(errors))
        print('captured %d frames in %.0f s (%.1f fps, %d job%s)' % (
            f1 - f0, time.time() - t_cap, (f1 - f0) / (time.time() - t_cap), jobs, 's' if jobs > 1 else ''),
            flush=True)
        msg = 'wrote %s (%.1f MB)' % (out, out.stat().st_size / 1e6)
        if not a.no_still:
            f = still_frame(a.still)
            chrome = open_page(uri, W, H, flags, str(tmpdir / 'chrome_still.log'))
            try:
                chrome.js('renderAt(%d)' % f)
                still = Path(a.still_out) if a.still_out else out.with_name(out.stem + '_still.png')
                still.write_bytes(chrome.shot('png'))
            finally:
                chrome.close()
            msg += '  still %s (frame %d, %s)' % (still, f, hhmmss(frame_t[f]))
        print(msg + '  total %.0f s' % (time.time() - t_start), flush=True)
        ok = True
    finally:
        if ok:
            shutil.rmtree(tmpdir, ignore_errors=True)
        else:
            print('kept', tmpdir, 'for debugging (capture.html, chrome*.log)', file=sys.stderr)


if __name__ == '__main__':
    main()
