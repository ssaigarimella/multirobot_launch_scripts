#!/usr/bin/env python3
"""SHOT 3 low lateral TRACK with a crosser (EXECUTION-PLAN-V6 §2, shot 3).

Camera 180 cm AGL, 600 cm lateral offset from the hero's 2.5 s smoothed path,
aim = playbook 8-frame finite-diff lookahead on the hero.  Both lateral sides
are tried; each side is QC'd and the best passing side wins.  REFUSES to
write on QC failure (caller then falls back to the v2 elevated hero-track).

QC: hero 120-260 px all frames; crosser in frame >=40% of frames at >=35 px;
hero cumulative screen displacement >= 1.5 frame-widths (2880 px); occlusion
rays = 0; near-clip = 0.

usage: design_camera_track.py <keys_v6.json> <out.json>
"""
import json
import math
import os
import sys

sys.path.insert(0, "/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo")
from shot_qc import DRONES, ShotQC, gate, look, moving_avg, project, px_size

SRC, DST = sys.argv[1], sys.argv[2]
K = json.load(open(SRC))
FPS = K["fps"]
D = K["drones"]
GZ = K["info"]["road_z"]
shots = {s["name"]: s for s in K["info"]["shots"]}
s3 = shots["s3_track"]
if s3.get("fallback"):
    print("s3 is flagged v2-fallback; nothing to design here")
    sys.exit(4)
A, B = s3["start"], s3["end"]
n = B - A
hero, crosser = s3["hero"], s3["crosser"]

# ---------------------------------------------------------------------------
# V2FB=1: the plan's sanctioned fallback — design_camera_v2.py's elevated 3/4
# hero-track (user-accepted grammar) on the s3 range.  Used when the low
# lateral track cannot hold the hero inside 120-260 px (hero jitter around
# its smoothed path forces either near-clips or a too-far offset).
# ---------------------------------------------------------------------------
if os.environ.get("V2FB") == "1":
    import math as _m
    from shot_qc import ShotQC as _QC
    hx = [D[hero][A + i][0] for i in range(n)]
    hy = [D[hero][A + i][1] for i in range(n)]
    hz = [D[hero][A + i][2] for i in range(n)]
    phx, phy, phz = (moving_avg(v, int(2.5 * FPS / 2)) for v in (hx, hy, hz))
    ahx, ahy, ahz = (moving_avg(v, int(0.8 * FPS / 2)) for v in (hx, hy, hz))
    best_fb = None
    _azs = [int(x) for x in os.environ.get(
        'S3_AZ_LIST', ','.join(str(a) for a in range(0, 360, 20))).split(',')]
    for AZ0 in _azs:
        for DD, H in ((1100.0, 780.0), (950.0, 780.0), (1250.0, 900.0)):
            AZ1 = AZ0 - 25.0                       # v2's default drift
            rows = []
            prev_yaw = None
            for i in range(n):
                u = i / (n - 1)
                az = _m.radians(AZ0 + (AZ1 - AZ0) * u)
                c = [phx[i] + DD * _m.cos(az), phy[i] + DD * _m.sin(az),
                     GZ + H]
                pitch, yaw = look(c, (ahx[i], ahy[i], ahz[i]), prev_yaw)
                prev_yaw = yaw
                rows.append([c[0], c[1], c[2], pitch, yaw])
            for idx, rnd in ((0, 2), (1, 2), (2, 2), (3, 3), (4, 3)):
                seg = moving_avg([r[idx] for r in rows], FPS // 4)
                for i, r in enumerate(rows):
                    r[idx] = round(seg[i], rnd)
            q = _QC("s3_v2fb", rows, {v: D[v][A:B] for v in DRONES}, FPS,
                    framed=[hero])
            h = q.report()["per_drone"][hero]
            occ = q.occlusion_hits(vehs=[hero])
            # crosser visibility (INFO + tiebreak)
            vis = 0
            for k in range(q.n):
                pr = project(rows[k][:3], rows[k][3], rows[k][4],
                             D[crosser][A + k])
                if pr is not None and px_size(pr[2]) >= 35:
                    s = q.m[crosser]["sxy"][k] if crosser in q.m else None
                    from shot_qc import frame_frac as _ff
                    if _ff(pr) <= 1.0:
                        vis += 1
            ok = (100 <= h["px"][0] and h["px"][1] <= 260 and
                  h["in_frac"] >= 1.0 and len(occ) == 0 and
                  q.nearclip_total() == 0)
            score = (1 if ok else 0, vis / q.n, h["cum_disp_px"])
            if best_fb is None or score > best_fb[0]:
                best_fb = (score, AZ0, DD, H, rows, h, vis / q.n, occ)
    score, AZ0, DD, H, rows, h, visf, occ = best_fb
    ok = score[0] == 1
    gate("s3fb hero 100-260px, in-frame, occ=0, clip=0", ok,
         f"px={h['px']} az={AZ0} D={DD:.0f} H={H:.0f}")
    print(f"s3fb crosser visible {100*visf:.0f}% (INFO)  hero cum_disp={h['cum_disp_px']}")
    if not ok:
        print("REFUSING: even the v2 fallback failed")
        sys.exit(3)
    cam = [list(r) for r in K["camera"]]
    for i in range(n):
        cam[A + i] = rows[i]
    K["camera"] = cam
    K["info"]["camera_v6_track"] = {"mode": "v2_fallback", "az0": AZ0,
                                    "d": DD, "h": H, "hero": hero,
                                    "crosser": crosser,
                                    "crosser_vis": round(visf, 2),
                                    "hero_px": h["px"]}
    json.dump(K, open(DST, "w"))
    print(f"wrote {DST} (v2 elevated hero-track fallback)")
    sys.exit(0)

hx = [D[hero][A + i][0] for i in range(n)]
hy = [D[hero][A + i][1] for i in range(n)]
hz = [D[hero][A + i][2] for i in range(n)]
half = int(2.5 * FPS / 2)
phx, phy = moving_avg(hx, half), moving_avg(hy, half)
ahx, ahy, ahz = (moving_avg(v, 4) for v in (hx, hy, hz))

Z_CAM = GZ + 180.0
LOOKAHEAD = 8


def build(side, off):
    rows = []
    prev_yaw = None
    for i in range(n):
        j0, j1 = max(0, i - 4), min(n - 1, i + 4)
        tx, ty = phx[j1] - phx[j0], phy[j1] - phy[j0]
        L = math.hypot(tx, ty) or 1.0
        nx, ny = -ty / L * side, tx / L * side
        c = [phx[i] + nx * off, phy[i] + ny * off, Z_CAM]
        ia = min(n - 1, i + LOOKAHEAD)
        pitch, yaw = look(c, (ahx[ia], ahy[ia], ahz[ia]), prev_yaw)
        prev_yaw = yaw
        rows.append([c[0], c[1], c[2], pitch, yaw])
    for idx, rnd in ((0, 2), (1, 2), (2, 2), (3, 3), (4, 3)):
        seg = moving_avg([r[idx] for r in rows], FPS // 3)
        for i, r in enumerate(rows):
            r[idx] = round(seg[i], rnd)
    return rows


def qc(rows, verbose=False):
    q = ShotQC("s3_track", rows, {v: D[v][A:B] for v in DRONES}, FPS,
               framed=[hero, crosser])
    rep = q.report()
    h = rep["per_drone"][hero]
    c = rep["per_drone"][crosser]
    # crosser visibility: frames where in-frame AND >=35px
    vis = 0
    for k in range(q.n):
        s = q.m[crosser]["sxy"][k]
        if s is None:
            continue
        pr = project(rows[k][:3], rows[k][3], rows[k][4], D[crosser][A + k])
        if pr and px_size(pr[2]) >= 35 and q.m[crosser]["sxy"][k]:
            fr = max(abs(pr[0]), abs(pr[1]))
            if 0 <= s[0] <= 1920 and 0 <= s[1] <= 1080:
                vis += 1
    checks = []
    checks.append(("hero 120-260px all frames",
                   h["px"][0] >= 120 and h["px"][1] <= 260 and h["in_frac"] >= 1.0
                   and h["worst_frac"] <= 1.0, str(h["px"])))
    checks.append(("crosser in frame >=40% at >=35px", vis / q.n >= 0.40,
                   f"{100*vis/q.n:.0f}%"))
    checks.append(("hero cum disp >=2880px", h["cum_disp_px"] >= 2880,
                   str(h["cum_disp_px"])))
    occ = q.occlusion_hits(vehs=[hero, crosser])
    checks.append(("occlusion rays = 0", len(occ) == 0, str(occ[:3])))
    checks.append(("near-clip = 0", q.nearclip_total() == 0, ""))
    ok = True
    if verbose:
        for nm, res, det in checks:
            ok &= gate("s3 " + nm, res, det)
    else:
        ok = all(c[1] for c in checks)
    score = h["cum_disp_px"] + 1000 * (vis / q.n)
    return ok, score, rep, checks


# lateral offset is solved, not fixed: the hero deviates from its 2.5 s
# smoothed path by hundreds of cm, so the plan's 600 cm can jump to >260 px /
# near-clip.  Plan-preference order: 600 first, then wider.
results = {}
for off in (600.0, 700.0, 800.0, 900.0, 1000.0, 1200.0):
    for side in (1.0, -1.0):
        rows = build(side, off)
        ok, score, rep, checks = qc(rows)
        results[(side, off)] = (ok, score, rows, rep)
        print(f"side {side:+.0f} off={off:.0f}: ok={ok} score={score:.0f}")
    passing = [(k, r) for k, r in results.items() if r[0]]
    if passing:
        break

passing = [(k, r) for k, r in results.items() if r[0]]
if not passing:
    print("--- diagnostics best combo ---")
    best_k = max(results, key=lambda k: results[k][1])
    qc(results[best_k][2], verbose=True)
    print("REFUSING to write keys: s3 QC failed on all combos -> use v2 fallback")
    sys.exit(3)
(side, off), (ok, score, rows, rep) = max(passing, key=lambda kv: kv[1][1])
print(f"selected side {side:+.0f} off={off:.0f}")
qc(rows, verbose=True)

cam = [list(r) for r in K["camera"]]
for i in range(n):
    cam[A + i] = rows[i]
K["camera"] = cam
K["info"]["camera_v6_track"] = {"side": side, "off": off, "hero": hero,
                                "crosser": crosser, "report": rep}
json.dump(K, open(DST, "w"))
print(f"wrote {DST}")
