#!/usr/bin/env python3
"""S5 fallback: ribbon reveal from the (already QC-passed) S6 vantage."""
import json, os, sys
sys.path.insert(0, "/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo")
from shot_qc import DRONES, project, frame_frac
K = json.load(open(os.environ["KEYS_IN"]))
shots = {s["name"]: s for s in K["info"]["shots"]}
s5, s6 = shots["s5_ribbonarc"], shots["s6_mapmatch"]
A5, B5, A6 = s5["start"], s5["end"], s6["start"]
cam, D = K["camera"], K["drones"]
c6 = cam[A6]
xs = [r[0] for v in DRONES for r in D[v][A5:B5]]
ys = [r[1] for v in DRONES for r in D[v][A5:B5]]
zs = [r[2] for v in DRONES for r in D[v][A5:B5]]
cx, cy = (min(xs)+max(xs))/2, (min(ys)+max(ys))/2
vx, vy = c6[0]-cx, c6[1]-cy
n5, PULL = B5-A5, 1.12
for i in range(n5):
    u = i/(n5-1); e = u*u*(3-2*u); f = PULL + (1.0-PULL)*e
    cam[A5+i] = [round(cx+vx*f,2), round(cy+vy*f,2),
                 round(c6[2]+(PULL-1)*600*(1-e),2), c6[3], c6[4]]
corners = [(x,y,z) for x in (min(xs),max(xs)) for y in (min(ys),max(ys)) for z in (min(zs),max(zs))]
worst = min(sum(1 for i in range(n5)
    if (lambda p: p is not None and frame_frac(p) <= 0.95)(project(cam[A5+i][:3], cam[A5+i][3], cam[A5+i][4], c)))/n5
    for c in corners)
if worst < 0.90:
    print(f"s5 fallback QC fail {worst:.2f}"); sys.exit(1)
K["camera"] = cam
K["info"]["s5_variant"] = "s6-vantage ribbon reveal"
json.dump(K, open(os.environ["KEYS_OUT"], "w"))
print(f"s5 fallback OK (corner vis {worst:.2f})")
