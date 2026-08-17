#!/usr/bin/env python3
"""Pick the flight->world anchor for the bamboo cinematic.

Two-stage search over (anchor_x, anchor_y, yaw, scale) for the rigid transform
in transform_keys_temple.py, so the flight footprint lands on flat ground in
photogenic bamboo with the fewest obstacle intersections.

Stage 1 rasterises every obstacle that reaches the flight deck into a binary
occupancy grid inflated by (solid_radius + drone_radius + margin), then scores
each placement by how many decimated key points land in blocked cells — O(1)
per point.  Stage 2 re-scores the survivors with the full key set and the exact
cylinder test, and reports how much lateral dodge clearance_bamboo.py would
need to finish the job.

env: KEYS, HARVEST, OUT, SCALE_LIST, YAW_STEP, SEARCH_STEP, DRONE_R, MARGIN,
     ZLIFT_MIN_AGL, TOPN, CENTER_BIAS
"""
import json, math, os, sys

sys.path.insert(0, "/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo")
from clearance_bamboo import solid_r

W = "/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo"
KEYS = os.environ.get("KEYS", W + "/keys_bmb2d_b_blocks.json")
HARV = os.environ.get("HARVEST", W + "/grove_harvest.json")
OUT = os.environ.get("OUT", W + "/anchor_report.json")
SCALES = [float(v) for v in os.environ.get("SCALE_LIST", "0.72,0.62,0.52").split(",")]
YAW_STEP = float(os.environ.get("YAW_STEP", "20"))
SEARCH_STEP = float(os.environ.get("SEARCH_STEP", "600"))
DRONE_R = float(os.environ.get("DRONE_R", "70"))
MARGIN = float(os.environ.get("MARGIN", "25"))
MIN_AGL = float(os.environ.get("ZLIFT_MIN_AGL", "100"))   # cm after scaling
TOPN = int(os.environ.get("TOPN", "10"))
CENTER_BIAS = float(os.environ.get("CENTER_BIAS", "0.0025"))  # cost per cm from hero corridor
HERO = [float(v) for v in os.environ.get("HERO_XY", "400,0").split(",")]
GRID = 50.0
BLOCKS_GROUND = 100.0

K = json.load(open(KEYS))
H = json.load(open(HARV))

# ---------- flight footprint in blocks space ----------
DRONES = list(K["drones"].keys())
full = []
for v in DRONES:
    full += [(r[0], r[1], r[2]) for r in K["drones"][v]]
xs = [p[0] for p in full]; ys = [p[1] for p in full]
pvx, pvy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
rel_full = [(p[0] - pvx, p[1] - pvy, p[2] - BLOCKS_GROUND) for p in full]
step_c = max(1, len(rel_full) // 400)
rel_coarse = rel_full[::step_c]
R_FOOT = max(math.hypot(a, b) for a, b, _ in rel_full)
AGL_MIN = min(c for _, _, c in rel_full)
AGL_MAX = max(c for _, _, c in rel_full)
print(f"footprint {max(xs)-min(xs):.0f} x {max(ys)-min(ys):.0f} cm  r={R_FOOT:.0f}cm  "
      f"AGL[{AGL_MIN:.0f},{AGL_MAX:.0f}]cm  keys {len(rel_full)} (coarse {len(rel_coarse)})")

# ---------- ground: where can we stand at all ----------
gsteps = H["trace_step"]
ground = {}
for k, v in H["grid"].items():
    if v:
        a, b = k.split(",")
        ground[(int(a), int(b))] = v[0]
gz = sorted(v for v in ground.values())
GZ = gz[len(gz) // 2]                      # median ground height (flat grove)
print(f"ground cells {len(ground)}  median z {GZ:.1f}  "
      f"p05 {gz[len(gz)//20]:.1f} p95 {gz[-len(gz)//20]:.1f}")

X0, X1, Y0, Y1 = H["roi"]
NX = int((X1 - X0) / GRID) + 2
NY = int((Y1 - Y0) / GRID) + 2
blocked = bytearray(NX * NY)
culm = {}
nobs = 0
for x, y, z, bb_r, ztop, mesh in H["inst"]:
    r = solid_r(mesh, bb_r)
    if mesh.startswith("SM_Bamboo_Nanite"):
        culm[(int(x // 1000), int(y // 1000))] = culm.get((int(x // 1000), int(y // 1000)), 0) + 1
    if r <= 0 or (z + ztop) < GZ + MIN_AGL:
        continue
    nobs += 1
    rad = r + DRONE_R + MARGIN
    i0 = max(0, int((x - rad - X0) / GRID)); i1 = min(NX - 1, int((x + rad - X0) / GRID))
    j0 = max(0, int((y - rad - Y0) / GRID)); j1 = min(NY - 1, int((y + rad - Y0) / GRID))
    r2 = rad * rad
    for i in range(i0, i1 + 1):
        cx = X0 + i * GRID
        for j in range(j0, j1 + 1):
            cy = Y0 + j * GRID
            if (cx - x) ** 2 + (cy - y) ** 2 <= r2:
                blocked[i * NY + j] = 1
occ = sum(blocked) / float(NX * NY)
print(f"obstacles at flight deck {nobs}; occupancy grid {NX}x{NY} @ {GRID:.0f}cm, "
      f"{occ*100:.1f}% blocked")

# ground mask on the same lattice (must be over a GroundTile)
def on_ground(x, y):
    gx = int(round(x / gsteps) * gsteps); gy = int(round(y / gsteps) * gsteps)
    return (gx, gy) in ground


def score(ax, ay, yaw_deg, s, pts):
    cyw, syw = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    hits = 0
    for bx, by, _ in pts:
        dx, dy = s * bx, s * by
        wx = ax + cyw * dx - syw * dy
        wy = ay + syw * dx + cyw * dy
        i = int((wx - X0) / GRID); j = int((wy - Y0) / GRID)
        if i < 0 or j < 0 or i >= NX or j >= NY or blocked[i * NY + j]:
            hits += 1
    return hits


# Scenery gate: the shot has to LOOK like a bamboo forest, so the flight lane
# itself must carry culms — but not so many that the fleet cannot thread them.
# Culm density is measured over the footprint disk in culms/m2.
DENS_MIN = float(os.environ.get("DENS_MIN", "0.055"))
DENS_MAX = float(os.environ.get("DENS_MAX", "0.20"))


def culm_density(ax, ay, s):
    rr = R_FOOT * s
    n = 0
    for i in range(int((ax - rr) // 1000), int((ax + rr) // 1000) + 1):
        for j in range(int((ay - rr) // 1000), int((ay + rr) // 1000) + 1):
            n += culm.get((i, j), 0)
    area = math.pi * (rr / 100.0) ** 2
    return n, n / area


# ---------- stage 1: coarse ----------
cands = []
yaws = [i * YAW_STEP for i in range(int(round(360 / YAW_STEP)))]
ax = X0
n_eval = n_site = 0
while ax <= X1:
    ay = Y0
    while ay <= Y1:
        # The S5 ribbon arc parks the camera at ~1.25x the footprint half-diagonal,
        # so the GROVE (not just the footprint) must extend that far in every
        # direction — otherwise the arc films the bare edge of the level.
        rad = R_FOOT * max(SCALES) * float(os.environ.get("ARC_FACTOR", "1.45")) + 400
        if (on_ground(ax, ay) and on_ground(ax + rad, ay) and on_ground(ax - rad, ay)
                and on_ground(ax, ay + rad) and on_ground(ax, ay - rad)):
            n_site += 1
            for s in SCALES:
                if not (DENS_MIN <= culm_density(ax, ay, s)[1] <= DENS_MAX):
                    continue          # scenery gate first — otherwise the
                                      # shortlist fills with empty clearings
                for yw in yaws:
                    h = score(ax, ay, yw, s, rel_coarse)
                    n_eval += 1
                    cands.append((h, ax, ay, yw, s))
        ay += SEARCH_STEP
    ax += SEARCH_STEP
print(f"stage1: {n_site} sites, {n_eval} placements")
if not cands:
    print("NO VALID SITE"); sys.exit(2)

cands.sort(key=lambda c: c[0] + CENTER_BIAS * math.dist((c[1], c[2]), HERO))
short = cands[:400]

# ---------- stage 2: full-resolution rescore + scenery gate ----------
out = []
for h0, ax, ay, yw, s in short:
    nc, dens = culm_density(ax, ay, s)
    if not (DENS_MIN <= dens <= DENS_MAX):
        continue
    hits = score(ax, ay, yw, s, rel_full)
    zlift = max(0.0, MIN_AGL - s * AGL_MIN)
    out.append(dict(anchor=[round(ax), round(ay), round(GZ)], yaw=yw, scale=s,
                    zlift=round(zlift), hits=hits,
                    hit_frac=round(hits / len(rel_full), 4),
                    agl=[round(s * AGL_MIN + zlift), round(s * AGL_MAX + zlift)],
                    footprint=[round(s * (max(xs) - min(xs))), round(s * (max(ys) - min(ys)))],
                    culms=nc, culm_density=round(dens, 4),
                    dist_hero=round(math.dist((ax, ay), HERO))))
out.sort(key=lambda c: (c["hits"], -c["culms"]))
best = out[:TOPN]
print(f"stage2 (full keys): {len(out)}/{len(short)} passed the scenery gate "
      f"[{DENS_MIN}-{DENS_MAX} culms/m2]")
for c in best:
    print(f"  anchor {c['anchor']} yaw {c['yaw']:5.1f} s {c['scale']:.2f} "
          f"zlift {c['zlift']:3d}  hits {c['hits']:5d}/{len(rel_full)} "
          f"({c['hit_frac']*100:.2f}%)  AGL {c['agl']}  fp {c['footprint']}  "
          f"culms {c['culms']} ({c['culm_density']}/m2)  d_hero {c['dist_hero']}")
json.dump({"footprint_r": R_FOOT, "n_keys": len(rel_full), "ground_z": GZ,
           "occupancy": occ, "cands": best}, open(OUT, "w"), indent=1)
print("wrote", OUT)
