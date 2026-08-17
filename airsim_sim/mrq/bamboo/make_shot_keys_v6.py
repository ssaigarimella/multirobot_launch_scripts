#!/usr/bin/env python3
"""Assemble the v6 stitched shot timeline (EXECUTION-PLAN-V6 §3).

Reads the FULL-RUN temple keys (export_keys.py full window -> transform_keys
_temple.py) + the window_select report, and writes a stitched keys file whose
frames are the concatenation of the six shots' source windows (s1/s2 retimed
by the window report's factor when takeoff was slow):

  seg 1  s1_starburst   10 s   W_open head
  seg 2  s2_throughline  7 s   W_open tail
  seg 3  s3_track       10 s   W_main approach event (or v2 fallback window)
  seg 4  s4_depthstack  10 s   W_main max-movers window
  seg 5  s5_ribbonarc   12 s   run end state
  seg 6  s6_mapmatch     5 s   last frames

Camera rows are ZERO placeholders — the design_camera_* scripts fill them
per shot and refuse on QC failure.  info.shots records the stitched ranges +
source epochs so compose/chips/QC all agree.

usage: make_shot_keys_v6.py <full_temple_keys.json> <window_report.json> <out.json>
"""
import json
import math
import sys

SHOT_DUR = {"s1_starburst": 10.0, "s2_throughline": 7.0, "s3_track": 10.0,
            "s4_depthstack": 10.0, "s5_ribbonarc": 12.0, "s6_mapmatch": 5.0}
ORDER = ["s1_starburst", "s2_throughline", "s3_track", "s4_depthstack",
         "s5_ribbonarc", "s6_mapmatch"]

full_p, rep_p, out_p = sys.argv[1], sys.argv[2], sys.argv[3]
K = json.load(open(full_p))
R = json.load(open(rep_p))
FPS = K["fps"]
NF = K["nframes"]
T0 = K["t0_epoch"]


def interp_row(rows, f):
    """linear interp of a key row list at fractional frame f (clamped)."""
    if f <= 0:
        return list(rows[0])
    if f >= len(rows) - 1:
        return list(rows[-1])
    i = int(f)
    a, b = rows[i], rows[i + 1]
    t = f - i
    out = [a[j] + t * (b[j] - a[j]) for j in range(len(a))]
    return out


shots = []
cursor = 0
drones_out = {v: [] for v in K["drones"]}
for name in ORDER:
    s = R["shots"][name]
    if s.get("fallback"):
        # v2 hero-track fallback window: use W_main head 10s
        wm = R["w_main"]
        ta = R["t0_epoch"] + wm["a"] * R["dt"]
        tb = ta + SHOT_DUR[name]
        retime = 1.0
        framed = []
    else:
        ta, tb = s["epoch"]
        retime = float(s.get("retime", 1.0))
        framed = s.get("framed", [])
    out_dur = (tb - ta) / retime
    n = int(round(out_dur * FPS))
    # honor the plan's fixed cut lengths (clip if source slightly longer)
    n = min(n, int(round(SHOT_DUR[name] * FPS)))
    for v, rows in K["drones"].items():
        for k in range(n):
            t_src = ta + k / FPS * retime
            f = (t_src - T0) * FPS
            r = interp_row(rows, f)
            drones_out[v].append([round(r[0], 2), round(r[1], 2),
                                  round(r[2], 2), round(r[3], 3)])
    shots.append(dict(name=name, start=cursor, end=cursor + n,
                      src_epoch=[ta, tb], retime=retime, framed=framed,
                      hero=s.get("hero"), crosser=s.get("crosser"),
                      approach_m=s.get("approach_m"),
                      fallback=s.get("fallback")))
    cursor += n

N = cursor
K2 = {"fps": FPS, "nframes": N, "t0_epoch": T0, "t1_epoch": K["t1_epoch"],
      "drones": drones_out,
      "camera": [[0.0, 0.0, 0.0, 0.0, 0.0] for _ in range(N)],
      "info": dict(K["info"], shots=shots, stitched_from=full_p,
                   window_report=rep_p)}
json.dump(K2, open(out_p, "w"))

print(f"stitched {N} frames ({N/FPS:.1f}s) from {full_p}")
for s in shots:
    print(f"  {s['name']:16s} [{s['start']:5d},{s['end']:5d})  "
          f"src t+[{s['src_epoch'][0]-T0:6.1f},{s['src_epoch'][1]-T0:6.1f}]s  "
          f"retime={s['retime']}x  framed={','.join(s['framed']) or '-'}"
          + (f"  hero={s['hero']}/crosser={s['crosser']}" if s.get('hero') else "")
      + (f"  FALLBACK={s['fallback']}" if s.get('fallback') else ""))
# per-drone footprint sanity
for v, rows in drones_out.items():
    xs = [r[0] for r in rows]; ys = [r[1] for r in rows]; zs = [r[2] for r in rows]
    print(f"  {v}: X[{min(xs):.0f},{max(xs):.0f}] Y[{min(ys):.0f},{max(ys):.0f}] "
          f"Z[{min(zs):.0f},{max(zs):.0f}]")
print(f"wrote {out_p}")
