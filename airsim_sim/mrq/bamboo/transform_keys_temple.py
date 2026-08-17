#!/usr/bin/env python3
"""Uniform-transform fleet_keys_mrq6.json (Blocks world) into the temple map.

RIGID transform only (translate + yaw rotation about the footprint center +
uniform scale) so relative fleet motion is NOT distorted:

    new_xy = T_xy + R(yaw) * s * (p_xy - pivot_xy)
    new_z  = T_groundz + s * (z - BLOCKS_ROAD_Z) + ZLIFT     (AGL preserved)
    yaws  += YAW_DEG (drones + camera); camera pitch unchanged.

Env: TEMPLE_ANCHOR="x,y,ground_z" (target of the footprint CENTER),
     YAW_DEG (default 0), SCALE (default 1.0), ZLIFT cm (default 0)
Usage: transform_keys_temple.py <in.json> <out.json>
"""
import json
import math
import os
import sys

BLOCKS_ROAD_Z = 100.0

src, dst = sys.argv[1], sys.argv[2]
ax, ay, agz = (float(v) for v in os.environ["TEMPLE_ANCHOR"].split(","))
yaw = math.radians(float(os.environ.get("YAW_DEG", "0")))
s = float(os.environ.get("SCALE", "1.0"))
zlift = float(os.environ.get("ZLIFT", "0"))
cy, sy = math.cos(yaw), math.sin(yaw)

K = json.load(open(src))

# pivot = center of the drone-XY bounding box (the flown footprint)
xs = [r[0] for rows in K["drones"].values() for r in rows]
ys = [r[1] for rows in K["drones"].values() for r in rows]
px, py = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0


def txy(x, y):
    dx, dy = s * (x - px), s * (y - py)
    return ax + cy * dx - sy * dy, ay + sy * dx + cy * dy


def tz(z):
    return agz + s * (z - BLOCKS_ROAD_Z) + zlift


for name, rows in K["drones"].items():
    for r in rows:
        r[0], r[1] = (round(v, 2) for v in txy(r[0], r[1]))
        r[2] = round(tz(r[2]), 2)
        r[3] = round(r[3] + math.degrees(yaw), 3)

for c in K["camera"]:
    c[0], c[1] = (round(v, 2) for v in txy(c[0], c[1]))
    c[2] = round(tz(c[2]), 2)
    c[4] = round(c[4] + math.degrees(yaw), 3)          # yaw; pitch c[3] unchanged

K["info"]["temple_transform"] = {"anchor": [ax, ay, agz], "yaw_deg": math.degrees(yaw),
                                 "scale": s, "zlift": zlift, "pivot_blocks": [px, py]}
K["info"]["road_z"] = agz
json.dump(K, open(dst, "w"))

for name, rows in K["drones"].items():
    xs = [r[0] for r in rows]; ys = [r[1] for r in rows]; zs = [r[2] for r in rows]
    print(f"  {name}: X[{min(xs):.0f},{max(xs):.0f}] Y[{min(ys):.0f},{max(ys):.0f}] "
          f"Z[{min(zs):.0f},{max(zs):.0f}] AGL[{min(zs)-agz:.0f},{max(zs)-agz:.0f}]cm")
cam = K["camera"]
print(f"  cam: X[{min(c[0] for c in cam):.0f},{max(c[0] for c in cam):.0f}] "
      f"Y[{min(c[1] for c in cam):.0f},{max(c[1] for c in cam):.0f}] "
      f"Z[{min(c[2] for c in cam):.0f},{max(c[2] for c in cam):.0f}]")
print(f"wrote {dst}")
