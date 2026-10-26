#!/usr/bin/env python3
"""fleet_3d.py - interactive 3D replay (nvblox colour voxels + trajectories,
all drones in the team frame) as ONE self-contained HTML page, WebGL.

    ./viz/fleet_3d.py <run> [drones...] [--from T --to T] [--out file.html]
                      [--voxel-res 0.05] [--max-voxels 1500000]

Open the file in any browser. Mouse: drag = orbit, right-drag / shift-drag =
pan, wheel = zoom. Keys: SPACE play/pause, <- -> step 1 s, [ ] 10 s, + - speed,
F follow the selected drone (1-4 select), R reset view. The time slider scrubs.
Screen-record the browser for demo videos; the 2D replay (fleet_player.py)
exports MP4 directly.

HOW IT STAYS SMALL AND SMOOTH
Voxels are the union of the incremental colour-layer markers (bag2timeline.py),
sorted by first-seen time and shipped as base64 typed arrays (12 B position,
3 B colour, 4 B time per voxel - 680k voxels = 13 MB, no per-frame copies).
"The map at time T" is a draw-range on that sorted buffer, so scrubbing is free.
"""
import argparse
import base64
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from fleet_player import Fleet, parse_t, hhmmss, COLOR  # noqa: E402

THREE_URL = 'https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js'


def b64(a):
    return base64.b64encode(np.ascontiguousarray(a).tobytes()).decode()


def build(fleet, tw0, tw1, voxel_res, max_voxels):
    drones = []
    for d in fleet.drones:
        tl = d.tl
        m = (tl['vox_t'] <= tw1)
        xyz = tl['vox_xyz'][m].astype(np.float64)
        rgb = tl['vox_rgb'][m]
        vt = tl['vox_t'][m]
        if voxel_res > tl.get('vox_res', 0.05) * 1.01 and len(xyz):
            # coarsen: first voxel wins per coarse cell (keeps first-seen order)
            key = np.floor(xyz / voxel_res).astype(np.int64)
            _, first = np.unique(key, axis=0, return_index=True)
            first.sort()
            xyz, rgb, vt = xyz[first], rgb[first], vt[first]
        if len(xyz) > max_voxels:
            sel = np.sort(np.random.default_rng(0).choice(len(xyz), max_voxels, replace=False))
            xyz, rgb, vt = xyz[sel], rgb[sel], vt[sel]
        order = np.argsort(vt, kind='stable')
        xyz, rgb, vt = xyz[order], rgb[order], vt[order]
        if len(xyz):
            xy = d.to_world_many(tw1, xyz[:, :2])   # alignment is constant per flight in practice
            xyz = np.column_stack([xy, xyz[:, 2]])
        # odometry at 10 Hz in the team frame
        om = (d.od_t >= tw0 - 5) & (d.od_t <= tw1 + 5)
        idx = np.nonzero(om)[0][::3]
        od = np.column_stack([d.od_t[idx] - tw0, d.world_xy[idx], d.od[idx, 3], d.world_yaw[idx]]).astype(np.float32)
        drones.append(dict(
            name=d.name, letter=d.letter, color=COLOR[d.name], n_vox=int(len(xyz)),
            vox_xyz=b64(xyz.astype(np.float32)), vox_rgb=b64(rgb.astype(np.uint8)),
            vox_t=b64((vt - tw0).astype(np.float32)),
            odom=b64(od), n_odom=int(len(od)),
            offboard=[[a - tw0, b - tw0] for a, b in d.offboard if b > tw0 and a < tw1],
        ))
    return dict(run=fleet.run, t0=tw0, dur=tw1 - tw0, t0_clock=hhmmss(tw0), drones=drones)


HTML = r"""<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>
:root{--bg:#fcfcfb;--ink:#0b0b0b;--ink2:#52514e;--line:#e6e5e1;--good:#0ca30c}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#1a1a19;--ink:#f0efec;--ink2:#c3c2b7;--line:#383835}}
html,body{margin:0;height:100%;background:var(--bg);color:var(--ink);font:13px/1.4 system-ui,sans-serif;overflow:hidden}
#c{position:fixed;inset:0;display:block}
#hud{position:fixed;left:16px;top:12px;pointer-events:none}
#hud h1{font-size:16px;margin:0 0 4px}
#hud .t{font-family:ui-monospace,monospace;font-size:15px}
#bar{position:fixed;left:16px;right:16px;bottom:14px;display:flex;gap:10px;align-items:center;background:color-mix(in srgb,var(--bg) 85%,transparent);border:1px solid var(--line);border-radius:8px;padding:8px 12px}
#bar input[type=range]{flex:1}
button{background:var(--bg);color:var(--ink);border:1px solid var(--line);border-radius:6px;padding:4px 10px;font:inherit;cursor:pointer}
button:hover{border-color:var(--ink2)}
#tiles{position:fixed;right:16px;top:12px;display:flex;flex-direction:column;gap:6px;min-width:230px}
.tile{border:1px solid var(--line);border-left:5px solid;border-radius:6px;padding:6px 10px;background:color-mix(in srgb,var(--bg) 85%,transparent);font-family:ui-monospace,monospace;font-size:12px;white-space:pre}
.tile b{font:600 13px system-ui,sans-serif}
.ob{color:var(--good);font-weight:700}
#help{position:fixed;left:16px;bottom:64px;color:var(--ink2);font-size:11px;pointer-events:none}
</style></head><body>
<canvas id="c"></canvas>
<div id="hud"><h1 id="title"></h1><div class="t" id="clock"></div></div>
<div id="tiles"></div>
<div id="help">drag orbit · shift-drag pan · wheel zoom · space play · ←/→ 1 s · [ ] 10 s · +/- speed · 1-4 follow · F free · R reset</div>
<div id="bar"><button id="play">▶</button><input id="slider" type="range" min="0" max="1000" value="0">
<span id="speed">×1</span><button id="slower">−</button><button id="faster">+</button></div>
<script src="__THREE__"></script>
<script>
const DATA = __DATA__;
const dec = (s, T) => new T(Uint8Array.from(atob(s), c => c.charCodeAt(0)).buffer);
const css = v => getComputedStyle(document.documentElement).getPropertyValue(v).trim();

const canvas = document.getElementById('c');
const renderer = new THREE.WebGLRenderer({canvas, antialias: true});
renderer.setPixelRatio(Math.min(2, devicePixelRatio));
const scene = new THREE.Scene();
scene.background = new THREE.Color(css('--bg'));
const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 2000);
// team frame is x forward, y left, z up: build the scene in that frame and
// tell three.js that z is up.
camera.up.set(0, 0, 1);
scene.add(new THREE.AmbientLight(0xffffff, 0.9));
const grid = new THREE.GridHelper(120, 120, 0xbbbbbb, css('--line'));
grid.rotation.x = Math.PI / 2; scene.add(grid);

const drones = DATA.drones.map(d => {
  const xyz = dec(d.vox_xyz, Float32Array), rgb = dec(d.vox_rgb, Uint8Array), vt = dec(d.vox_t, Float32Array);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(xyz, 3));
  g.setAttribute('color', new THREE.BufferAttribute(rgb, 3, true));
  const pts = new THREE.Points(g, new THREE.PointsMaterial({size: 0.07, vertexColors: true, sizeAttenuation: true}));
  pts.frustumCulled = false; scene.add(pts);
  const od = dec(d.odom, Float32Array);           // t x y z yaw
  const col = new THREE.Color(d.color);
  const trailG = new THREE.BufferGeometry();
  trailG.setAttribute('position', new THREE.BufferAttribute(new Float32Array(d.n_odom * 3), 3));
  const trail = new THREE.Line(trailG, new THREE.LineBasicMaterial({color: col, linewidth: 2}));
  trail.frustumCulled = false; scene.add(trail);
  const body = new THREE.Mesh(new THREE.SphereGeometry(0.28, 20, 14), new THREE.MeshLambertMaterial({color: col}));
  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.12, 0.5, 12), new THREE.MeshLambertMaterial({color: 0xffffff}));
  nose.rotation.z = -Math.PI / 2; nose.position.x = 0.45; body.add(nose); scene.add(body);
  // label sprite
  const cv = document.createElement('canvas'); cv.width = 128; cv.height = 64;
  const cx = cv.getContext('2d'); cx.font = 'bold 44px system-ui'; cx.fillStyle = d.color; cx.textAlign = 'center';
  cx.fillText(d.letter, 64, 48);
  const spr = new THREE.Sprite(new THREE.SpriteMaterial({map: new THREE.CanvasTexture(cv), depthTest: false}));
  spr.scale.set(1.2, 0.6, 1); spr.position.z = 0.7; body.add(spr);
  const tile = document.createElement('div'); tile.className = 'tile'; tile.style.borderLeftColor = d.color;
  document.getElementById('tiles').appendChild(tile);
  return {d, pts, vt, od, trail, body, tile, nVox: xyz.length / 3};
});

// ---- time ----
let t = 0, playing = false, speed = 1, last = null, follow = -1;
const slider = document.getElementById('slider'), clock = document.getElementById('clock');
document.getElementById('title').textContent = `${DATA.run} · 3D replay`;
const fmt = s => { const d = new Date((DATA.t0 + s) * 1000); return d.toTimeString().slice(0, 8); };
function lowerBound(arr, v, stride = 1) { let lo = 0, hi = arr.length / stride; while (lo < hi) { const m = (lo + hi) >> 1; if (arr[m * stride] <= v) lo = m + 1; else hi = m; } return lo; }
function update() {
  slider.value = Math.round(1000 * t / DATA.dur);
  clock.textContent = `${fmt(t)}   T+${t.toFixed(1)} s   ×${speed}`;
  drones.forEach((o, i) => {
    o.pts.geometry.setDrawRange(0, lowerBound(o.vt, t));
    const k = lowerBound(o.od, t, 5);          // odom rows with time <= t
    const pos = o.trail.geometry.attributes.position;
    let n = 0;
    for (let j = 0; j < k; j++) { pos.array[n++] = o.od[j * 5 + 1]; pos.array[n++] = o.od[j * 5 + 2]; pos.array[n++] = o.od[j * 5 + 3]; }
    pos.needsUpdate = true; o.trail.geometry.setDrawRange(0, k);
    const alive = k > 0 && (t - o.od[(k - 1) * 5]) < 3;
    o.body.visible = alive;
    if (alive) {
      const j = (k - 1) * 5;
      o.body.position.set(o.od[j + 1], o.od[j + 2], o.od[j + 3]);
      o.body.rotation.z = o.od[j + 4];
      if (follow === i) controls.target.copy(o.body.position);
    }
    const ob = o.d.offboard.some(([a, b]) => a <= t && t <= b);
    const z = alive ? o.od[(k - 1) * 5 + 3] : 0;
    o.tile.innerHTML = `<b>${o.d.letter}  ${o.d.name}</b>  ${ob ? '<span class="ob">OFFBOARD</span>' : alive ? 'on ground / manual' : 'no data'}\n` +
      `voxels ${o.pts.geometry.drawRange.count.toLocaleString()} / ${o.nVox.toLocaleString()}\n` +
      (alive ? `pos (${o.body.position.x.toFixed(1)}, ${o.body.position.y.toFixed(1)})  z ${z.toFixed(1)} m` : '');
  });
}
function setT(v) { t = Math.max(0, Math.min(DATA.dur, v)); update(); }

// ---- minimal orbit controls (no external module) ----
const controls = {target: new THREE.Vector3(10, 0, 0), theta: -2.2, phi: 1.05, r: 45};
function applyCam() {
  const {target: c, theta, phi, r} = controls;
  camera.position.set(c.x + r * Math.sin(phi) * Math.cos(theta), c.y + r * Math.sin(phi) * Math.sin(theta), c.z + r * Math.cos(phi));
  camera.lookAt(c);
}
let drag = null;
canvas.addEventListener('pointerdown', e => { drag = {x: e.clientX, y: e.clientY, pan: e.shiftKey || e.button === 2}; canvas.setPointerCapture(e.pointerId); });
canvas.addEventListener('pointerup', () => drag = null);
canvas.addEventListener('contextmenu', e => e.preventDefault());
canvas.addEventListener('pointermove', e => {
  if (!drag) return;
  const dx = e.clientX - drag.x, dy = e.clientY - drag.y; drag.x = e.clientX; drag.y = e.clientY;
  if (drag.pan) {
    const right = new THREE.Vector3().crossVectors(camera.getWorldDirection(new THREE.Vector3()), camera.up).normalize();
    const upv = new THREE.Vector3(0, 0, 1);
    controls.target.addScaledVector(right, -dx * controls.r * 0.0015).addScaledVector(upv, dy * controls.r * 0.0015);
    follow = -1;
  } else { controls.theta -= dx * 0.005; controls.phi = Math.max(0.05, Math.min(1.55, controls.phi - dy * 0.005)); }
  applyCam();
});
canvas.addEventListener('wheel', e => { controls.r = Math.max(2, Math.min(300, controls.r * Math.exp(e.deltaY * 0.001))); applyCam(); e.preventDefault(); }, {passive: false});
function resetView() { controls.target.set(10, 0, 0); controls.theta = -2.2; controls.phi = 1.05; controls.r = 45; follow = -1; applyCam(); }

// ---- UI ----
const playBtn = document.getElementById('play');
function togglePlay() { playing = !playing; last = null; playBtn.textContent = playing ? '❚❚' : '▶'; }
playBtn.onclick = togglePlay;
slider.oninput = () => setT(slider.value / 1000 * DATA.dur);
document.getElementById('faster').onclick = () => { speed = Math.min(32, speed * 2); update(); };
document.getElementById('slower').onclick = () => { speed = Math.max(0.125, speed / 2); update(); };
addEventListener('keydown', e => {
  if (e.key === ' ') { togglePlay(); e.preventDefault(); }
  else if (e.key === 'ArrowRight') setT(t + 1); else if (e.key === 'ArrowLeft') setT(t - 1);
  else if (e.key === ']') setT(t + 10); else if (e.key === '[') setT(t - 10);
  else if (e.key === '+' || e.key === '=') { speed = Math.min(32, speed * 2); update(); }
  else if (e.key === '-') { speed = Math.max(0.125, speed / 2); update(); }
  else if (e.key >= '1' && e.key <= '4') { follow = Math.min(drones.length - 1, +e.key - 1); controls.r = Math.min(controls.r, 15); }
  else if (e.key === 'f' || e.key === 'F') follow = -1;
  else if (e.key === 'r' || e.key === 'R') resetView();
});
function resize() { const w = innerWidth, h = innerHeight; renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix(); }
addEventListener('resize', resize); resize(); resetView();
// #t=120 opens at T+120 s, #t=120&play autoplays - handy for screenshots and demos
const hash = Object.fromEntries(location.hash.slice(1).split('&').filter(Boolean).map(kv => kv.split('=')));
if (hash.t) setT(parseFloat(hash.t)); else update();
if ('play' in hash) togglePlay();
function loop(now) {
  if (playing) { if (last !== null) { setT(t + (now - last) / 1000 * speed); if (t >= DATA.dur) togglePlay(); } last = now; }
  if (follow >= 0) applyCam();
  renderer.render(scene, camera); requestAnimationFrame(loop);
}
requestAnimationFrame(loop);
</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('drones', nargs='*')
    ap.add_argument('--bags', default=str(HERE.parent / 'bags'))
    ap.add_argument('--from', dest='t_from')
    ap.add_argument('--to', dest='t_to')
    ap.add_argument('--voxel-res', type=float, default=0.05, help='coarsen the colour voxels to this cell (m)')
    ap.add_argument('--max-voxels', type=int, default=1500000, help='per drone')
    ap.add_argument('--out')
    a = ap.parse_args()

    fleet = Fleet(a.run, a.drones, a.bags, rot='0')
    tw0, tw1 = fleet.default_window()
    if a.t_from:
        tw0 = parse_t(a.t_from, fleet.t0)
    if a.t_to:
        tw1 = parse_t(a.t_to, fleet.t0)
    tw0, tw1 = max(tw0, fleet.t0), min(tw1, fleet.t1)
    data = build(fleet, tw0, tw1, a.voxel_res, a.max_voxels)
    page = (HTML.replace('__TITLE__', f'{a.run} 3D replay').replace('__THREE__', THREE_URL)
            .replace('__DATA__', json.dumps(data)))
    out = a.out or str(Path(a.bags) / a.run / f'{a.run}_3d.html')
    Path(out).write_text(page)
    print(f'window {hhmmss(tw0)}-{hhmmss(tw1)}; ' + ', '.join(f"{d['name']} {d['n_vox']} voxels" for d in data['drones']))
    print('wrote', out, f'({Path(out).stat().st_size / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
