#!/usr/bin/env python3
"""Shared camera-QC kernels for the v6 fleet cinematic (EXECUTION-PLAN-V6 §2.0).

Extracted from design_camera_v3.py (project(), moving_avg) + export_keys.py
(yaw unwrap), plus the numeric per-shot metrics and the offender-AABB
occlusion rays.  Every design_camera_* script imports this and REFUSES to
write keys that fail its thresholds — bad takes die in <1s of CPU, not in
4-minute renders.

Camera model (verified build_fleet_seq6_clean.py:382-386): 1920x1080,
CineCamera 18.0 mm, filmback 23.76x13.365 -> half-HFOV 33.4 deg, half-VFOV
20.35 deg; drone screen size ~= 145600/d_cm px; near clip 50 cm.
"""
import json
import math

W_PX, H_PX = 1920.0, 1080.0
HFOV2 = math.radians(33.4)
VFOV2 = math.atan(math.tan(HFOV2) * H_PX / W_PX)          # ~20.35 deg
TAN_H, TAN_V = math.tan(HFOV2), math.tan(VFOV2)
NEAR_CM = 50.0
PX_CONST = W_PX * 100.0 / (2.0 * TAN_H)                    # ~145,589 px*cm
import os as _os
PX_CONST *= float(_os.environ.get('DRONE_SCALE', '1.0'))  # rendered mesh scale
DRONES = ["ghost", "delta", "buckshee", "thunderstrike"]


def moving_avg(vals, half):
    n = len(vals)
    return [sum(vals[max(0, i - half):min(n, i + half + 1)]) /
            (min(n, i + half + 1) - max(0, i - half)) for i in range(n)]


def unwrap_deg(seq):
    out = list(seq)
    for i in range(1, len(out)):
        while out[i] - out[i - 1] > 180.0:
            out[i] -= 360.0
        while out[i] - out[i - 1] < -180.0:
            out[i] += 360.0
    return out


def look(cam, tgt, prev_yaw=None):
    dx, dy, dz = tgt[0] - cam[0], tgt[1] - cam[1], tgt[2] - cam[2]
    yaw = math.degrees(math.atan2(dy, dx))
    if prev_yaw is not None:
        while yaw - prev_yaw > 180.0:
            yaw -= 360.0
        while yaw - prev_yaw < -180.0:
            yaw += 360.0
    pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
    return pitch, yaw


def project(cam, pitch, yaw, p):
    """-> (ang_x, ang_y, dist_cm) or None if behind the 50 cm near plane."""
    dx, dy, dz = p[0] - cam[0], p[1] - cam[1], p[2] - cam[2]
    yr, pr = math.radians(yaw), math.radians(pitch)
    x1 = math.cos(yr) * dx + math.sin(yr) * dy
    y1 = -math.sin(yr) * dx + math.cos(yr) * dy
    x2 = math.cos(pr) * x1 + math.sin(pr) * dz
    z2 = -math.sin(pr) * x1 + math.cos(pr) * dz
    if x2 <= NEAR_CM:
        return None
    return (math.atan2(y1, x2), math.atan2(z2, x2),
            math.sqrt(dx * dx + dy * dy + dz * dz))


def frame_frac(pr):
    return max(abs(pr[0]) / HFOV2, abs(pr[1]) / VFOV2)


def screen_px(pr):
    """pinhole pixel position; +x right, +y down, (0,0) top-left."""
    return (W_PX / 2 + (W_PX / 2) * math.tan(pr[0]) / TAN_H,
            H_PX / 2 - (H_PX / 2) * math.tan(pr[1]) / TAN_V)


def px_size(dist_cm):
    return PX_CONST / max(1.0, dist_cm)


def seg_hits_box(p0, p1, wmin, wmax, pad=0.0):
    """segment-AABB intersection (slab method), optional box padding (cm)."""
    lo = [wmin[i] - pad for i in range(3)]
    hi = [wmax[i] + pad for i in range(3)]
    t0, t1 = 0.0, 1.0
    for i in range(3):
        d = p1[i] - p0[i]
        if abs(d) < 1e-9:
            if p0[i] < lo[i] or p0[i] > hi[i]:
                return False
            continue
        ta = (lo[i] - p0[i]) / d
        tb = (hi[i] - p0[i]) / d
        if ta > tb:
            ta, tb = tb, ta
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 > t1:
            return False
    return True


def load_offender_boxes(path=None):
    """Occluder AABBs for design-time ray tests.  Env QC_BOXES overrides the
    default (v6 uses the macro-occluder harvest from query_bounds_v6.py)."""
    import os
    if path is None:
        path = os.environ.get(
            "QC_BOXES",
            "/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo/bamboo_qc_boxes.json")
    try:
        A = json.load(open(path))
    except OSError:
        return []
    return [(b["mesh"] + f"#{b['inst']}", b["wmin"], b["wmax"]) for b in A.get("ism", [])]


class ShotQC:
    """Numeric pre-render QC over one shot's frames.

    cam_rows: [[x,y,z,pitch,yaw]...]; drone_rows: {name: [[x,y,z,yaw]...]}
    (equal length, the shot's own frame range), fps.
    """

    def __init__(self, name, cam_rows, drone_rows, fps, framed=None, boxes=None):
        self.name = name
        self.cam = cam_rows
        self.dr = drone_rows
        self.fps = fps
        self.framed = framed or list(drone_rows.keys())
        self.boxes = boxes if boxes is not None else load_offender_boxes()
        self.n = len(cam_rows)
        self._compute()

    def _compute(self):
        m = {v: dict(worst_frac=0.0, px_min=1e9, px_max=0.0, in_frames=0,
                     nearclip=0, cum_disp_px=0.0, sxy=[], first_off=None)
             for v in self.framed}
        for k in range(self.n):
            c = self.cam[k]
            for v in self.framed:
                p = self.dr[v][k]
                pr = project(c[:3], c[3], c[4], p)
                st = m[v]
                if pr is None:
                    st["nearclip"] += 1
                    st["sxy"].append(None)
                    if st["first_off"] is None:
                        st["first_off"] = k
                    continue
                fr = frame_frac(pr)
                st["worst_frac"] = max(st["worst_frac"], fr)
                if fr <= 1.0:
                    st["in_frames"] += 1
                elif st["first_off"] is None:
                    st["first_off"] = k
                px = px_size(pr[2])
                st["px_min"] = min(st["px_min"], px)
                st["px_max"] = max(st["px_max"], px)
                st["sxy"].append(screen_px(pr))
        for v in self.framed:
            st = m[v]
            sxy = st["sxy"]
            for k in range(1, self.n):
                if sxy[k] and sxy[k - 1]:
                    st["cum_disp_px"] += math.hypot(sxy[k][0] - sxy[k - 1][0],
                                                    sxy[k][1] - sxy[k - 1][1])
            st["in_frac"] = st["in_frames"] / max(1, self.n)
        self.m = m

    # ---- aggregate metrics -------------------------------------------------
    def min_pair_sep_px(self, k):
        """min pairwise screen separation at frame k (px), None if any off."""
        pts = []
        for v in self.framed:
            s = self.m[v]["sxy"][k]
            if s is None:
                return None
            pts.append(s)
        best = 1e18
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                best = min(best, math.hypot(pts[i][0] - pts[j][0],
                                            pts[i][1] - pts[j][1]))
        return best

    def pair_sep_after(self, t_s):
        """min pairwise separation over frames >= t_s (px)."""
        k0 = int(t_s * self.fps)
        vals = [self.min_pair_sep_px(k) for k in range(k0, self.n)]
        vals = [v for v in vals if v is not None]
        return min(vals) if vals else None

    def velocity_bearing_spread(self, win_s=1.0):
        """spread (deg) of mean screen-velocity bearings of framed drones."""
        w = max(1, int(win_s * self.fps))
        bearings = []
        for v in self.framed:
            sxy = [s for s in self.m[v]["sxy"] if s is not None]
            if len(sxy) < w + 1:
                return 0.0
            dx = sxy[-1][0] - sxy[0][0]
            dy = sxy[-1][1] - sxy[0][1]
            bearings.append(math.degrees(math.atan2(dy, dx)))
        bs = sorted(b % 360.0 for b in bearings)
        gap = max(((bs[(i + 1) % len(bs)] - bs[i]) % 360.0)
                  for i in range(len(bs)))
        return 360.0 - gap

    def occlusion_hits(self, sample_every=5, vehs=None):
        """Camera->drone rays blocked by an occluder AABB.

        BAMBOO DELTA: the temple/rural maps had a handful of large occluders,
        so "zero blocked rays" was the right bar.  A bamboo grove has a culm
        every ~2 m, so zero is unreachable AND undesirable — foreground stalks
        sweeping past the subject are exactly what makes the swarm-video
        tracking shots read as depth.  QC_OCC_FRAC is the allowance: if at most
        that fraction of sampled rays is blocked, the shot passes with no hits
        reported.  Set QC_OCC_FRAC=0 to restore the strict temple behaviour.
        """
        import os
        hits = []
        n_rays = 0
        for k in range(0, self.n, sample_every):
            c = self.cam[k]
            for v in (vehs or self.framed):
                n_rays += 1
                p = self.dr[v][k]
                for name, wmin, wmax in self.boxes:
                    if seg_hits_box(c[:3], p[:3], wmin, wmax):
                        hits.append((k, v, name))
                        break
        allow = float(os.environ.get("QC_OCC_FRAC", "0"))
        if n_rays and hits and len(hits) / n_rays <= allow:
            return []
        return hits

    def nearclip_total(self):
        return sum(self.m[v]["nearclip"] for v in self.framed)

    def max_yaw_rate(self):
        yaw = unwrap_deg([c[4] for c in self.cam])
        return max((abs(yaw[k] - yaw[k - 1]) * self.fps
                    for k in range(1, self.n)), default=0.0)

    def report(self):
        out = {"shot": self.name, "frames": self.n, "framed": self.framed,
               "per_drone": {}, "nearclip": self.nearclip_total(),
               "max_yaw_rate_dps": round(self.max_yaw_rate(), 1)}
        for v in self.framed:
            st = self.m[v]
            out["per_drone"][v] = dict(
                worst_frac=round(st["worst_frac"], 3),
                px=[round(st["px_min"], 0), round(st["px_max"], 0)],
                in_frac=round(st["in_frac"], 3),
                cum_disp_px=round(st["cum_disp_px"], 0),
                first_off_frame=st["first_off"])
        return out


def count_moving_blobs(png_a, png_b, diff_thresh=26, min_px=60):
    """Adjacent sparse-frame diff -> number of moving blobs (render gate #2)."""
    from PIL import Image
    import numpy as np
    a = np.asarray(Image.open(png_a).convert("L"), dtype=np.int16)
    b = np.asarray(Image.open(png_b).convert("L"), dtype=np.int16)
    d = (np.abs(a - b) > diff_thresh).astype(np.uint8)
    # 4-connected components via simple BFS on a downsampled mask (fast enough)
    d = d[::2, ::2]
    seen = np.zeros_like(d)
    blobs = 0
    H, W = d.shape
    from collections import deque
    for y in range(H):
        for x in range(W):
            if d[y, x] and not seen[y, x]:
                q = deque([(y, x)])
                seen[y, x] = 1
                size = 0
                while q:
                    cy, cx = q.popleft()
                    size += 1
                    for ny, nx in ((cy+1,cx),(cy-1,cx),(cy,cx+1),(cy,cx-1)):
                        if 0 <= ny < H and 0 <= nx < W and d[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = 1
                            q.append((ny, nx))
                if size >= min_px // 4:      # mask is 2x downsampled
                    blobs += 1
    return blobs


def gate(name, ok, detail=""):
    print(f"  QC {name}: {'PASS' if ok else 'FAIL'} {detail}")
    return ok
