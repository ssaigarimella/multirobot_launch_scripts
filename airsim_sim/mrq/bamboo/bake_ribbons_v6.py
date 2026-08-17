#!/usr/bin/env python3
"""Bake per-drone reveal-ribbon OBJs for the v6 cinematic.

Unlike bake_ribbons.py (mrq6: window keys + meta pre-window extension), the
v6 ribbons are baked STRAIGHT from the full-run temple keys, so u (TEXCOORD0)
spans the whole flight chronologically: u = frame/(N-1).  Same X-profile
strip + Interchange Y-negation as the proven baker; decimated to every 2nd
frame.

env: KEYS (full temple keys json), OUT_DIR (default temple/), ZOFF (default
-30 cm), WIDTH (default 16 cm)
"""
import json
import math
import os

W = "/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo"
KEYS = os.environ.get("KEYS", W + "/fleet_keys_temple6_full.json")
OUT_DIR = os.environ.get("OUT_DIR", W)
ZOFF = float(os.environ.get("ZOFF", "-30"))
WIDTH = float(os.environ.get("WIDTH", "16"))

K = json.load(open(KEYS))
N = K["nframes"]

for name, rows in K["drones"].items():
    pts = []
    for f in range(0, len(rows), 2):
        r = rows[f]
        pts.append((f / (N - 1), r[0], r[1], r[2] + ZOFF))

    v_lines, vt_lines, f_lines = [], [], []

    def emit_strip(offsets):
        base = len(v_lines)
        for i, (u, X, Y, Z) in enumerate(pts):
            j0, j1 = max(0, i - 1), min(len(pts) - 1, i + 1)
            tx = pts[j1][1] - pts[j0][1]
            tyy = pts[j1][2] - pts[j0][2]
            o = offsets(tx, tyy)
            # OBJ Y negated: UE Interchange mirrors Y on import (proven mrq6)
            v_lines.append(f"v {X + o[0]:.1f} {-(Y + o[1]):.1f} {Z + o[2]:.1f}")
            v_lines.append(f"v {X - o[0]:.1f} {-(Y - o[1]):.1f} {Z - o[2]:.1f}")
            vt_lines.append(f"vt {u:.6f} 0.0")
            vt_lines.append(f"vt {u:.6f} 1.0")
            if i:
                a, b = base + 2 * i - 1, base + 2 * i
                c, d = base + 2 * i + 1, base + 2 * i + 2
                f_lines.append(f"f {a}/{a} {b}/{b} {d}/{d} {c}/{c}")

    def horiz(tx, tyy):
        L = math.hypot(tx, tyy) or 1.0
        return (-tyy / L * WIDTH / 2, tx / L * WIDTH / 2, 0.0)

    emit_strip(horiz)
    emit_strip(lambda tx, tyy: (0.0, 0.0, WIDTH / 2))

    out = os.path.join(OUT_DIR, f"ribbon_{name}.obj")
    with open(out, "w") as fo:
        fo.write("o ribbon_%s\n" % name)
        fo.write("\n".join(v_lines) + "\n")
        fo.write("\n".join(vt_lines) + "\n")
        fo.write("\n".join(f_lines) + "\n")
    print(f"{name}: {len(pts)} pts, verts {len(v_lines)} -> {out}")
print("RIBBONS_OK")
