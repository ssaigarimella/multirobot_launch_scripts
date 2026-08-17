#!/usr/bin/env python3
"""Clearance sweep + computed lateral dodge for the bamboo cinematic.

The bamboo grove is LUSH by design (canopy 0.16 culms/m2, shoots 0.23/m2), so a
40 m flight footprint cannot be placed collision-free by anchor search alone.
This does what the temple chain's fix_keys_foliage.py did by hand, but computed
and general:

  1. SWEEP  - every drone key vs every obstacle that reaches the flight deck
              (culm clumps, saplings, rocks; ferns/grass/litter top out at
              <=47 cm and are below every drone by >=1 m, so they are excluded).
  2. DODGE  - iterative push-out + low-pass smoothing of a per-frame LATERAL
              offset field.  Altitude is never touched (AGL stays honest) and
              the offset is smoothed over SMOOTH_S so the flown shape is
              preserved; only the position in the grove shifts.
  3. GATE   - refuses to write if the required offset exceeds MAX_OFF (the
              placement is then genuinely bad and the anchor must move).

Solid-radius model: the instance bounding box is the LEAF envelope.  What a
drone can actually hit at 1-4 m AGL is the culm/stem bundle, so the collision
radius is a fraction of the bb radius per asset family (same model as
pick_anchor.py, kept in one place here and imported there).

usage: clearance_bamboo.py <keys_in.json> [keys_out.json]
env: HARVEST, DRONE_R (cm), MARGIN (cm), MAX_OFF (cm), SMOOTH_S, ITERS, SWEEP_ONLY
"""
import json, math, os, sys
from collections import defaultdict

W = "/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo"
HARV = os.environ.get("HARVEST", W + "/grove_harvest.json")
DRONE_R = float(os.environ.get("DRONE_R", "55"))     # quad 98x98x29 cm; body+prop disc, arms are thin
MARGIN = float(os.environ.get("MARGIN", "10"))
CLEAR = DRONE_R + MARGIN
MAX_OFF = float(os.environ.get("MAX_OFF", "220"))
SMOOTH_S = float(os.environ.get("SMOOTH_S", "0.55"))
ITERS = int(os.environ.get("ITERS", "60"))
SWEEP_ONLY = os.environ.get("SWEEP_ONLY", "0") == "1"
CELL = 400.0


def solid_r(mesh, bb_r):
    """Collision radius of the instance AT DRONE ALTITUDE (1-3 m AGL), cm.

    The instance bounding box is the LEAF envelope (bb_r 100-150 cm for a
    mature culm), but at 1-3 m up a bamboo plant is a culm bundle a small
    fraction of that across, with the foliage above.  Scaling the fraction with
    bb_r keeps it proportional to the per-instance scale (0.85-3.59x here).
    """
    if mesh.startswith("SM_Bamboo_Nanite"):      # mature culm clumps
        return 0.22 * bb_r
    if mesh.startswith("SM_Bamboo_Saplings"):    # thin young shoots
        return 0.12 * bb_r
    if mesh.startswith("SM_Rocks"):              # solid all the way through
        return 0.85 * bb_r
    return 0.0                                   # ferns / grass / leaf litter


def build_hash(harv):
    H = json.load(open(harv)) if isinstance(harv, str) else harv
    h = defaultdict(list)
    n = 0
    for x, y, z, bb_r, ztop, mesh in H["inst"]:
        r = solid_r(mesh, bb_r)
        if r <= 0:
            continue
        h[(int(x // CELL), int(y // CELL))].append((x, y, z + ztop, r))
        n += 1
    return h, n, H


def near(h, x, y, rad):
    out = []
    for cx in range(int((x - rad) // CELL), int((x + rad) // CELL) + 1):
        for cy in range(int((y - rad) // CELL), int((y + rad) // CELL) + 1):
            out += h.get((cx, cy), [])
    return out


def sweep(h, drones):
    """-> (n_violations, worst_pen_cm, per-drone counts)"""
    tot, worst, per = 0, 0.0, {}
    for v, rows in drones.items():
        c = 0
        for r in rows:
            for (ox, oy, otop, orad) in near(h, r[0], r[1], 400):
                if r[2] > otop:
                    continue
                need = orad + CLEAR
                d = math.hypot(r[0] - ox, r[1] - oy)
                if d < need:
                    c += 1
                    worst = max(worst, need - d)
                    break
        per[v] = c
        tot += c
    return tot, worst, per


def gauss_smooth(vals, half):
    if half < 1:
        return list(vals)
    ker = [math.exp(-0.5 * (i / (half / 2.0)) ** 2) for i in range(-half, half + 1)]
    s = sum(ker)
    ker = [k / s for k in ker]
    n = len(vals)
    out = []
    for i in range(n):
        acc = wsum = 0.0
        for j, k in enumerate(ker):
            idx = i + j - half
            if 0 <= idx < n:
                acc += vals[idx] * k
                wsum += k
        out.append(acc / wsum)
    return out


def main():
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else None
    K = json.load(open(src))
    h, nobs, H = build_hash(HARV)
    fps = K["fps"]
    half = max(1, int(round(SMOOTH_S * fps / 2)))

    tot0, worst0, per0 = sweep(h, K["drones"])
    npts = sum(len(r) for r in K["drones"].values())
    print(f"obstacles at flight deck: {nobs}  keys: {npts}")
    print(f"SWEEP(before): {tot0} violating keys ({100.0*tot0/npts:.2f}%), "
          f"worst penetration {worst0:.0f} cm  per-drone {per0}")
    if SWEEP_ONLY or dst is None:
        return 0 if tot0 == 0 else 1

    # ---- iterative lateral push-out with smoothing ----
    # Smoothing width decays over the run: wide early (translate the whole
    # path into the open lane, preserving shape), narrow late (let the drone
    # weave locally around the last few stems).  A fixed width stalls at the
    # fixed point where the smoother exactly cancels the push-out.
    off = {v: [[0.0, 0.0] for _ in rows] for v, rows in K["drones"].items()}
    for it in range(ITERS):
        frac = it / max(1, ITERS - 1)
        half = max(1, int(round((SMOOTH_S * (1.0 - 0.75 * frac)) * fps / 2)))
        moved = 0
        for v, rows in K["drones"].items():
            dxs = [o[0] for o in off[v]]
            dys = [o[1] for o in off[v]]
            for i, r in enumerate(rows):
                px, py = r[0] + dxs[i], r[1] + dys[i]
                pushx = pushy = 0.0
                for (bx, by, btop, brad) in near(h, px, py, 400):
                    if r[2] > btop:
                        continue
                    need = brad + CLEAR
                    ddx, ddy = px - bx, py - by
                    d = math.hypot(ddx, ddy)
                    if d < need:
                        if d < 1e-3:
                            ddx, ddy, d = 1.0, 0.0, 1.0
                        pen = need - d
                        pushx += ddx / d * pen
                        pushy += ddy / d * pen
                if pushx or pushy:
                    moved += 1
                    dxs[i] += 0.6 * pushx
                    dys[i] += 0.6 * pushy
            dxs = gauss_smooth(dxs, half)
            dys = gauss_smooth(dys, half)
            for i in range(len(rows)):
                off[v][i][0], off[v][i][1] = dxs[i], dys[i]
        if moved == 0:
            print(f"  converged after {it+1} iterations")
            break
    else:
        print(f"  hit iteration cap ({ITERS})")

    maxoff = max(math.hypot(o[0], o[1]) for v in off for o in off[v])
    for v, rows in K["drones"].items():
        for i, r in enumerate(rows):
            r[0] = round(r[0] + off[v][i][0], 2)
            r[1] = round(r[1] + off[v][i][1], 2)

    tot1, worst1, per1 = sweep(h, K["drones"])
    print(f"SWEEP(after):  {tot1} violating keys ({100.0*tot1/npts:.2f}%), "
          f"worst penetration {worst1:.0f} cm  per-drone {per1}")
    print(f"max lateral dodge applied: {maxoff:.0f} cm (cap {MAX_OFF:.0f})")
    if maxoff > MAX_OFF:
        print("DODGE_REFUSED: placement needs too much correction — move the anchor")
        return 3
    if tot1 > 0:
        print("DODGE_INCOMPLETE: residual violations remain")
    K["info"]["bamboo_clearance"] = dict(
        drone_r=DRONE_R, margin=MARGIN, max_off_cm=round(maxoff, 1),
        before=tot0, after=tot1, worst_before=round(worst0, 1),
        worst_after=round(worst1, 1), smooth_s=SMOOTH_S, harvest=HARV)
    json.dump(K, open(dst, "w"))
    print("wrote", dst)
    return 0 if tot1 == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
