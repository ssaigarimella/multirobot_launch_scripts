#!/usr/bin/env python3
"""DEFECT-2 fix: generate fleet_keys_temple_v3.json from v2 with two local,
smooth, cosmetic trajectory edits (fleet motion otherwise identical):

1. THUNDERSTRIKE tail (the 'drone flew into a bush'): its approach+loiter
   (f~1930..2399) runs through the BPP_LI_FoliageCluster_15/46 fern/monstera
   bed (giant ferns SM_Fern_01_F/G top out at z=1188/1313 -- far above flight
   height, so no sane lift clears them). Fix: RIGID lateral dodge SOUTH (-Y,
   the open root-covered courtyard, screen-left) with a raised-cosine ramp-in;
   the loiter dance shape is preserved exactly (pure translation).
   The dodge magnitude is COMPUTED from the offenders' world AABBs
   (offender_aabbs.json) + drone radius + margin -- no hand tuning.

2. BUCKSHEE tail: clips SM_LillyPad_Large_Standing_01 (top z=1102 vs path
   ~1067) at f2358-2399. Fix: raised-cosine z-lift computed the same way
   (buckshee is behind the camera by then).

Verification is external: re-run scout_foliage_sweep.py on v3 (bounds gate)
and stage A's line-trace sweep (colliding-geometry gate).
"""
import json
import math

W = "/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo"
RAD = 70.0        # drone body radius (quad 98x98x29 cm)
MARGIN = 30.0     # extra clearance
CLEAR = RAD + MARGIN

K = json.load(open(W + "/fleet_keys_temple_v2.json"))
A = json.load(open(W + "/offender_aabbs.json"))
boxes = [(b["mesh"] + f"#{b['inst']}", b["wmin"], b["wmax"]) for b in A["ism"]]

# LillyPadGiant35 world AABB from actor bounds (query_offenders_progress.txt)
LILLY = ([3384 - 260, -1957 - 286, 828 - 274], [3384 + 260, -1957 + 286, 828 + 274])


def dist_box(p, wmin, wmax):
    d = [max(wmin[i] - p[i], 0.0, p[i] - wmax[i]) for i in range(3)]
    return math.sqrt(sum(v * v for v in d))


def cos_ramp(f, f0, f1):
    """0 before f0, 1 after f1, raised-cosine between."""
    if f <= f0:
        return 0.0
    if f >= f1:
        return 1.0
    t = (f - f0) / (f1 - f0)
    return 0.5 - 0.5 * math.cos(math.pi * t)


# ---------------- thunderstrike: compute the rigid dodge D ----------------
TS = K["drones"]["thunderstrike"]
F_RAMP0, F_RAMP1 = 1930, 1985     # full dodge active before first contact ~1986
# find minimal D (south shift) such that every frame >= F_RAMP0, with the
# ramp applied, clears every foliage box by CLEAR.
D = 0.0
for step in (50.0, 10.0, 2.0):
    while True:
        ok = True
        for f in range(F_RAMP0, len(TS)):
            r = TS[f]
            dy = -D * cos_ramp(f, F_RAMP0, F_RAMP1)
            p = (r[0], r[1] + dy, r[2])
            for name, wmin, wmax in boxes:
                if dist_box(p, wmin, wmax) < CLEAR:
                    ok = False
                    break
            if not ok:
                break
        if ok:
            break
        D += step
    if D > 0:
        D -= step  # back off, refine with next step size
D += 2.0  # settle
# final exact pass
worst = 1e9
worst_info = None
for f in range(F_RAMP0, len(TS)):
    r = TS[f]
    dy = -D * cos_ramp(f, F_RAMP0, F_RAMP1)
    p = (r[0], r[1] + dy, r[2])
    for name, wmin, wmax in boxes:
        d = dist_box(p, wmin, wmax)
        if d < worst:
            worst = d
            worst_info = (f, name, round(d, 1))
print(f"[fix] thunderstrike dodge D={D:.0f} cm south, ramp f{F_RAMP0}-{F_RAMP1}, "
      f"worst clearance {worst:.1f} cm at {worst_info}")
assert worst >= CLEAR - 1e-6, "dodge insufficient"

for f in range(F_RAMP0, len(TS)):
    TS[f][1] = round(TS[f][1] - D * cos_ramp(f, F_RAMP0, F_RAMP1), 2)

# ---------------- buckshee: computed z-lift over the lilypad ----------------
BK = K["drones"]["buckshee"]
G_RAMP0, G_RAMP1 = 2260, 2350     # first contact was f2358
H = 0.0
for step in (50.0, 10.0, 2.0):
    while True:
        ok = True
        for f in range(G_RAMP0, len(BK)):
            r = BK[f]
            p = (r[0], r[1], r[2] + H * cos_ramp(f, G_RAMP0, G_RAMP1))
            if dist_box(p, *LILLY) < CLEAR:
                ok = False
                break
        if ok:
            break
        H += step
    if H > 0:
        H -= step
H += 2.0
worst = 1e9
worst_f = None
for f in range(G_RAMP0, len(BK)):
    r = BK[f]
    p = (r[0], r[1], r[2] + H * cos_ramp(f, G_RAMP0, G_RAMP1))
    d = dist_box(p, *LILLY)
    if d < worst:
        worst, worst_f = d, f
print(f"[fix] buckshee lift H={H:.0f} cm, ramp f{G_RAMP0}-{G_RAMP1}, "
      f"worst lilypad clearance {worst:.1f} cm at f{worst_f}")
assert worst >= CLEAR - 1e-6, "lift insufficient"

for f in range(G_RAMP0, len(BK)):
    BK[f][2] = round(BK[f][2] + H * cos_ramp(f, G_RAMP0, G_RAMP1), 2)

K["info"]["foliage_fix_v3"] = dict(
    thunderstrike=dict(dodge_south_cm=D, ramp=[F_RAMP0, F_RAMP1]),
    buckshee=dict(lift_cm=H, ramp=[G_RAMP0, G_RAMP1]),
    radius=RAD, margin=MARGIN,
    note="rigid -Y dodge around FoliageCluster_15/46 fern bed; z-lift over "
         "LillyPadGiant35; computed from world AABBs, no hand tuning")
json.dump(K, open(W + "/fleet_keys_temple_v3.json", "w"))
print("[fix] wrote fleet_keys_temple_v3.json")
