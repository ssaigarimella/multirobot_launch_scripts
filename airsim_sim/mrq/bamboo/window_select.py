#!/usr/bin/env python3
"""Window selection for the v6 fleet cinematic (EXECUTION-PLAN-V6 §2.0).

Reads a run archive's meta_*.json (AirSim NED truth, ~4.5 Hz) + the stall
timeline from score_run.py --timeline, and blesses/refuses the windows that
must feed every shot BEFORE any UE work:

  W_open  15s from first motion (retimable <=2x by extending the source span
          up to 30s), maximizing min-per-drone displacement + net-displacement
          bearing spread.  Floors: NET displacement >= 3.5 m EACH, spread >=
          120 deg.  [RECALIBRATED from the plan's 8 m: the planner moves in
          burst-pause cycles at ~0.25 m/s effective — even L1_base, the
          plan's own calibration exemplar, peaks ~5 m/15 s.  The §5 screen-
          space acceptance criteria (250 px displacement, 288 px separation,
          180 deg spread) stay binding in shot_qc and are what actually
          gate the shot.]
  W_main  80-100s contiguous: min per-drone path >= 10 m, >= 50% of samples
          with >= 3 simultaneously ACTIVE drones, and at least one pairwise
          closest approach with min distance in [6,14] m.  ACTIVE(v,t) =
          v moves > 1.5 m of path within [t-3s,t+3s].  [RECALIBRATED from
          |v|>0.3 instantaneous: with per-drone moving fractions ~0.2
          (burst-pause planner), P(3 simultaneous instantaneous movers) ~=
          0.03 — unreachable for ANY archived run.  The 6 s activity window
          captures the same "three drones visibly working" intent at the
          planner's real cadence.]
  s1..s4  concrete shot sub-windows (10/7/10/10 s) with framed-drone checks:
          no framed drone parked (|v|<0.3) for > 10 s continuous, and no
          overlap with any stall-timeline streak > 20 s for framed drones.
  s5/s6   run end state + last frames (always feasible if the run archived).

Exit 0 + window_report_<label>.json when every shot is fed; exit 3 (REFUSED)
otherwise.

usage: window_select.py <label> [--runs-dir D] [--settings S] [--report OUT]
"""
import argparse
import glob
import json
import math
import os
import sys

DRONES = ["ghost", "delta", "buckshee", "thunderstrike"]
RUNS_DEFAULT = "/home/lucas/hercules-sim/e1_frames/runs"
V_MOVING = 0.3

S1_DUR, S2_DUR, S3_DUR, S4_DUR, S5_DUR, S6_DUR = 10.0, 7.0, 10.0, 10.0, 12.0, 5.0


def load_tracks(rundir, label, spawns):
    """world-frame XY (m) tracks resampled onto a common 0.25s grid."""
    raw = {}
    t0s, t1s = [], []
    for v in DRONES:
        pts = []
        for m in sorted(glob.glob(os.path.join(rundir, f"{label}_{v}", "meta_*.json"))):
            try:
                j = json.load(open(m))
            except Exception:
                continue
            ned = j.get("ned") or []
            if len(ned) < 2:
                continue
            ox, oy = spawns[v]
            pts.append((float(j["t"]), ox + float(ned[0]), oy + float(ned[1])))
        pts.sort()
        raw[v] = pts
        t0s.append(pts[0][0])
        t1s.append(pts[-1][0])
    t0, t1 = max(t0s), min(t1s)
    dt = 0.25
    n = int((t1 - t0) / dt)
    grid = {}
    for v in DRONES:
        ts = [p[0] for p in raw[v]]
        xs = [p[1] for p in raw[v]]
        ys = [p[2] for p in raw[v]]
        import bisect
        gx, gy = [], []
        for k in range(n):
            t = t0 + k * dt
            i = min(max(bisect.bisect_right(ts, t) - 1, 0), len(ts) - 2)
            f = (t - ts[i]) / (ts[i + 1] - ts[i]) if ts[i + 1] > ts[i] else 0.0
            gx.append(xs[i] + f * (xs[i + 1] - xs[i]))
            gy.append(ys[i] + f * (ys[i + 1] - ys[i]))
        grid[v] = (gx, gy)
    return grid, t0, dt, n


def speeds(grid, dt, n, half=2):
    """per-drone speed series (m/s), lightly smoothed (+-half samples)."""
    sp = {}
    for v in DRONES:
        gx, gy = grid[v]
        s = [0.0] * n
        for k in range(1, n):
            s[k] = math.hypot(gx[k] - gx[k - 1], gy[k] - gy[k - 1]) / dt
        sm = []
        for k in range(n):
            lo, hi = max(0, k - half), min(n, k + half + 1)
            sm.append(sum(s[lo:hi]) / (hi - lo))
        sp[v] = sm
    return sp


def bearing_spread(bearings):
    """smallest arc (deg) containing all bearings."""
    bs = sorted(b % 360.0 for b in bearings)
    m = len(bs)
    best_gap = 0.0
    for i in range(m):
        gap = (bs[(i + 1) % m] - bs[i]) % 360.0
        best_gap = max(best_gap, gap)
    return 360.0 - best_gap


def make_active(grid, dt, n, act_win_s=3.0, act_path_m=1.5):
    """ACTIVE(v,k): v moves > act_path_m of path within +-act_win_s."""
    w = int(act_win_s / dt)
    act = {}
    for v in DRONES:
        gx, gy = grid[v]
        step = [0.0] * n
        for k in range(1, n):
            step[k] = math.hypot(gx[k] - gx[k - 1], gy[k] - gy[k - 1])
        cum = [0.0]
        for k in range(n):
            cum.append(cum[-1] + step[k])
        act[v] = [cum[min(n, k + w + 1)] - cum[max(0, k - w)] > act_path_m
                  for k in range(n)]
    return act


def win_metrics(grid, sp, dt, a, b, act=None):
    """metrics over sample range [a,b)."""
    disp, path, brg = {}, {}, {}
    for v in DRONES:
        gx, gy = grid[v]
        dx, dy = gx[b - 1] - gx[a], gy[b - 1] - gy[a]
        disp[v] = math.hypot(dx, dy)
        brg[v] = math.degrees(math.atan2(dy, dx))
        path[v] = sum(math.hypot(gx[k] - gx[k - 1], gy[k] - gy[k - 1])
                      for k in range(a + 1, b))
    if act is not None:
        movers3 = sum(1 for k in range(a, b)
                      if sum(1 for v in DRONES if act[v][k]) >= 3) / max(1, b - a)
    else:
        movers3 = sum(1 for k in range(a, b)
                      if sum(1 for v in DRONES if sp[v][k] > V_MOVING) >= 3) / max(1, b - a)
    return disp, path, brg, movers3


def longest_parked(sp_v, dt, a, b):
    worst = cur = 0
    for k in range(a, b):
        cur = cur + 1 if sp_v[k] <= V_MOVING else 0
        worst = max(worst, cur)
    return worst * dt


def streak_overlap(timeline, t0, dt, a, b, vehs):
    """list of (drone, dur, ovl_s) for streaks>20s of framed drones overlapping [a,b)."""
    ta, tb = t0 + a * dt, t0 + b * dt
    out = []
    for v in vehs:
        for e in timeline["drones"].get(v, []):
            s0, s1 = e["t_start"], e["t_end"]
            ovl = min(tb, s1) - max(ta, s0)
            if ovl > 0:
                out.append((v, e["dur_s"], round(ovl, 1)))
    return out


def pair_approaches(grid, dt, a, b):
    """closest-approach local minima per pair inside [a,b): (dist_m, k, va, vb)."""
    out = []
    for i in range(4):
        for j in range(i + 1, 4):
            va, vb = DRONES[i], DRONES[j]
            ax, ay = grid[va]
            bx, by = grid[vb]
            d = [math.hypot(ax[k] - bx[k], ay[k] - by[k]) for k in range(a, b)]
            for k in range(1, len(d) - 1):
                if d[k] <= d[k - 1] and d[k] <= d[k + 1]:
                    out.append((d[k], a + k, va, vb))
    return sorted(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("label")
    ap.add_argument("--runs-dir", default=RUNS_DEFAULT)
    ap.add_argument("--settings", default=None)
    ap.add_argument("--report", default=None)
    ap.add_argument("--open-anchor-s", type=float, default=None,
                    help="anchor the W_open scan at this flight-relative time "
                         "instead of takeoff (for runs whose true fan-out "
                         "beat is mid-flight, e.g. after an early fence pin)")
    ap.add_argument("--relaxed", action="store_true",
                    help="never REFUSE: when no window clears the floors, take "
                         "the best-scoring window anyway and mark the report "
                         "relaxed=True.  For the unattended overnight loop, "
                         "where shot_qc + the sparse-frame inspection are the "
                         "gates that actually decide whether a beat ships.")
    args = ap.parse_args()
    label = args.label
    rundir = os.path.join(args.runs_dir, label)

    # spawns: from the settings the run flew with (scorecard row), or --settings
    sp_file = args.settings
    if not sp_file:
        for line in open(os.path.join(rundir, f"scorecard_{label}.txt"), errors="replace"):
            if line.startswith("fleet_spawn_offsets"):
                f = line.split()[3]          # cols: metric, "settings", "json", file
                sp_file = f if os.path.isabs(f) else "/home/lucas/hercules-sim/" + f
                break
    S = json.load(open(sp_file))
    spawns = {v: (float(S["Vehicles"][v].get("X", 0)), float(S["Vehicles"][v].get("Y", 0)))
              for v in DRONES}

    tlp = os.path.join(rundir, f"stall_timeline_{label}.json")
    timeline = json.load(open(tlp))

    grid, t0, dt, n = load_tracks(rundir, label, spawns)
    sp = speeds(grid, dt, n)
    act = make_active(grid, dt, n)
    dur = n * dt
    print(f"=== window_select {label}: {dur:.0f}s usable, t0={t0:.2f} ===")

    report = {"label": label, "t0_epoch": t0, "dt": dt, "usable_s": round(dur, 1),
              "shots": {}, "verdicts": {}}
    ok_all = True

    # ---------------- W_open ------------------------------------------------
    # first motion per drone (takeoffs are staggered 0/4/8/12 s): anchor the
    # scan between the FIRST drone's motion and shortly after the LAST one's,
    # so the window can hold the full four-way fan-out.
    need = int(1.0 / dt)
    relaxed_used = []
    firsts = {}
    for v in DRONES:
        for k in range(n - need):
            if all(sp[v][k + i] > V_MOVING for i in range(need)):
                firsts[v] = k
                break
    if len(firsts) < 4:
        if not args.relaxed:
            print("REFUSE: not all drones ever move")
            sys.exit(3)
        # relaxed: a drone that never clears V_MOVING still has to be framed —
        # treat it as "moving from t0" so the window search can proceed and let
        # shot_qc decide whether the resulting beat is watchable.
        dead = [v for v in DRONES if v not in firsts]
        print(f"RELAXED: {','.join(dead)} never exceed {V_MOVING} m/s — "
              f"anchored at t0")
        for v in dead:
            firsts[v] = 0
        relaxed_used.append("no_motion:" + "+".join(dead))
    first = min(firsts.values())
    first_all = max(firsts.values())
    print(f"first motion per drone (t+s): "
          f"{ {v: round(k*dt,1) for v, k in firsts.items()} }")

    if args.open_anchor_s is not None:
        a_lo = max(0, int((args.open_anchor_s - 5.0) / dt))
        a_hi_extra = int((args.open_anchor_s + 10.0) / dt)
        print(f"W_open anchor override: scanning t+[{a_lo*dt:.0f},"
              f"{a_hi_extra*dt:.0f}]s")
    else:
        a_lo = max(0, first - int(2 / dt))
        a_hi_extra = first_all + int(15 / dt)

    def check_window(a, b, vehs, allow_parked=10.0):
        """quiet version of the bless checks (parked + streak overlap)."""
        if not all(longest_parked(sp[v], dt, a, b) <= allow_parked
                   for v in vehs):
            return False
        return not streak_overlap(timeline, t0, dt, a, b, vehs)

    best_open = None
    for span in [15.0, 18.0, 21.0, 24.0, 27.0, 30.0]:          # retime = span/15 <= 2x
        w = int(span / dt)
        cands_open = []
        for a in range(a_lo, min(n - w, a_hi_extra)):
            b = a + w
            disp, path, brg, m3 = win_metrics(grid, sp, dt, a, b)
            dmin = min(disp.values())
            spread = bearing_spread(list(brg.values()))
            feas = dmin >= 3.5 and spread >= 120.0
            score = dmin + 10.0 * (spread >= 120.0) - 2.0 * (span - 15.0) / 3.0
            if feas or args.relaxed:
                cands_open.append(dict(
                    a=a, b=b, span=span, retime=round(span / 15.0, 2),
                    dmin=round(dmin, 1), spread=round(spread, 1),
                    disp={v: round(disp[v], 1) for v in DRONES},
                    bearings={v: round(brg[v], 0) for v in DRONES},
                    feasible=feas, score=score))
        # bless-aware: walk candidates by score; require the s1/s2 sub-windows
        # to pass parked+streak checks before accepting the opening
        cands_open.sort(key=lambda c: -c["score"])
        for cand in cands_open[:60]:
            rt = cand["retime"]
            a, b = cand["a"], cand["b"]
            s1b = min(a + int(S1_DUR * rt / dt), n)
            s2a = max(0, b - int(S2_DUR * rt / dt))
            if check_window(a, s1b, DRONES) and check_window(s2a, b, DRONES):
                best_open = cand
                break
        if best_open is None and args.relaxed and cands_open:
            # nothing blesses: take the highest-scoring opening anyway
            best_open = dict(cands_open[0], relaxed=True)
        if best_open:
            break                                # smallest sufficient retime wins
    if best_open is None:
        print("W_open: NO feasible window (net disp>=3.5m x4 + spread>=120, "
              "span<=30s from takeoff)")
        ok_all = False
    elif not best_open.get("feasible", True):
        w = best_open
        print(f"W_open: RELAXED t+[{w['a']*dt:.1f},{w['b']*dt:.1f}]s "
              f"span={w['span']}s retime={w['retime']}x dmin={w['dmin']}m "
              f"spread={w['spread']}deg (below floors — shot_qc is the gate)")
        relaxed_used.append("w_open")
    else:
        w = best_open
        print(f"W_open: t+[{w['a']*dt:.1f},{w['b']*dt:.1f}]s span={w['span']}s "
              f"retime={w['retime']}x dmin={w['dmin']}m spread={w['spread']}deg")
        print(f"   disp={w['disp']}  bearings={w['bearings']}")
    report["w_open"] = best_open

    # ---------------- W_main ------------------------------------------------
    best_main = None
    for span in [100.0, 95.0, 90.0, 85.0, 80.0]:
        w = int(span / dt)
        step = int(2.5 / dt)
        for a in range(0, n - w, step):
            b = a + w
            disp, path, brg, m3 = win_metrics(grid, sp, dt, a, b, act=act)
            pmin = min(path.values())
            apps = [x for x in pair_approaches(grid, dt, a, b) if 6.0 <= x[0] <= 14.0]
            feas = pmin >= 10.0 and m3 >= 0.5 and len(apps) > 0
            score = m3 * 100 + pmin + 5 * len(apps)
            if (feas or args.relaxed) and (best_main is None or score > best_main["_score"]):
                best_main = dict(a=a, b=b, span=span, pmin=round(pmin, 1),
                                 movers3=round(m3, 3), feasible=feas,
                                 path={v: round(path[v], 1) for v in DRONES},
                                 approaches=[(round(d, 1), k, va, vb)
                                             for d, k, va, vb in apps[:8]],
                                 _score=score)
        if best_main and best_main.get("feasible", True):
            break
    if best_main is None:
        print("W_main: NO feasible window (path>=10m x4, movers3>=0.5, approach 6-14m)")
        ok_all = False
    elif not best_main.get("feasible", True):
        m = best_main
        print(f"W_main: RELAXED t+[{m['a']*dt:.1f},{m['b']*dt:.1f}]s "
              f"span={m['span']}s pmin={m['pmin']}m movers3={m['movers3']} "
              f"(below floors — shot_qc is the gate)")
        relaxed_used.append("w_main")
    else:
        m = best_main
        print(f"W_main: t+[{m['a']*dt:.1f},{m['b']*dt:.1f}]s span={m['span']}s "
              f"pmin={m['pmin']}m movers3={m['movers3']}")
        print(f"   path={m['path']}")
        print(f"   approaches(6-14m)={m['approaches'][:4]}")
    report["w_main"] = {k: v for k, v in (best_main or {}).items() if k != "_score"}

    # ---------------- concrete shot windows --------------------------------
    def bless(name, a, b, vehs, allow_parked=10.0):
        parked = {v: round(longest_parked(sp[v], dt, a, b), 1) for v in vehs}
        ovl = streak_overlap(timeline, t0, dt, a, b, vehs)
        ok = all(p <= allow_parked for p in parked.values()) and not ovl
        report["shots"][name] = dict(
            rel_s=[round(a * dt, 1), round(b * dt, 1)],
            epoch=[round(t0 + a * dt, 2), round(t0 + b * dt, 2)],
            framed=vehs, parked_s=parked, streak_overlap=ovl, blessed=ok)
        flag = "OK " if ok else "FAIL"
        print(f"  {name}: [{a*dt:.1f},{b*dt:.1f}]s framed={','.join(vehs)} "
              f"parked={parked} streak_ovl={ovl} -> {flag}")
        return ok

    if best_open:
        a, b, span = best_open["a"], best_open["b"], best_open["span"]
        rt = best_open["retime"]
        # s1 = first 10s-equivalent (10*rt source secs), s2 = last 7s-equivalent
        s1b = a + int(S1_DUR * rt / dt)
        s2a = b - int(S2_DUR * rt / dt)
        ok_all &= bless("s1_starburst", a, min(s1b, n), DRONES)
        ok_all &= bless("s2_throughline", max(0, s2a), b, DRONES)
        report["shots"]["s1_starburst"]["retime"] = rt
        report["shots"]["s2_throughline"]["retime"] = rt
    else:
        ok_all = False

    hero = crosser = None
    if best_main:
        a, b = best_main["a"], best_main["b"]
        hero = max(DRONES, key=lambda v: best_main["path"][v])
        hero_apps = [x for x in best_main["approaches"] if hero in (x[2], x[3])]
        s3 = None
        pool = hero_apps or best_main["approaches"]
        for d_, k_, va_, vb_ in pool:
            h = hero if hero in (va_, vb_) else max((va_, vb_),
                                                    key=lambda v: best_main["path"][v])
            c = vb_ if h == va_ else va_
            a3 = max(a, k_ - int(S3_DUR / 2 / dt))
            b3 = min(b, a3 + int(S3_DUR / dt))
            if bless("s3_track", a3, b3, [h, c]):
                s3 = (a3, b3, h, c, d_)
                hero, crosser = h, c
                report["shots"]["s3_track"]["hero"] = h
                report["shots"]["s3_track"]["crosser"] = c
                report["shots"]["s3_track"]["approach_m"] = round(d_, 1)
                break
        if s3 is None:
            print("  s3_track: no clean approach window -> V2 HERO-TRACK FALLBACK")
            report["shots"]["s3_track"] = {"fallback": "v2_hero_track", "blessed": True}
        # s4: 10s windows inside W_main ranked by #frames with >=3 ACTIVE
        # (tie-break total speed); walk down the ranking until one BLESSES
        # (no framed drone parked >10s, no streak>20s overlap).
        w4 = int(S4_DUR / dt)

        def collinearity(a4, b4):
            """PCA minor-axis extent (m) of the 4 window-mean positions —
            small = fleet lines up = depth-stack axis exists (SHOT 4)."""
            pts = []
            for v in DRONES:
                gx, gy = grid[v]
                pts.append((sum(gx[a4:b4]) / (b4 - a4),
                            sum(gy[a4:b4]) / (b4 - a4)))
            mx = sum(p[0] for p in pts) / 4
            my = sum(p[1] for p in pts) / 4
            sxx = sum((p[0] - mx) ** 2 for p in pts) / 4
            syy = sum((p[1] - my) ** 2 for p in pts) / 4
            sxy = sum((p[0] - mx) * (p[1] - my) for p in pts) / 4
            th = 0.5 * math.atan2(2 * sxy, sxx - syy)
            return max(abs(-(p[0] - mx) * math.sin(th) +
                           (p[1] - my) * math.cos(th)) for p in pts)

        cands4 = []
        for a4 in range(a, b - w4, int(0.5 / dt)):
            v3 = sum(1 for k in range(a4, a4 + w4)
                     if sum(1 for v in DRONES if act[v][k]) >= 3)
            tv = sum(sp[v][k] for v in DRONES for k in range(a4, a4 + w4, 4))
            # rank: 3-active frames desc, then collinearity asc (the depth
            # stack needs the fleet lined up), then speed desc
            cands4.append((v3 * 1000 - 50 * collinearity(a4, a4 + w4) + tv, a4))
        cands4.sort(reverse=True)
        ok4 = False
        for _, a4 in cands4[:40]:
            if bless("s4_depthstack", a4, a4 + w4, DRONES):
                ok4 = True
                break
        ok_all &= ok4
    else:
        ok_all = False

    # ---------------- s5/s6 (end state) ------------------------------------
    bless("s5_ribbonarc", max(0, n - int(S5_DUR / dt)), n, [], allow_parked=1e9)
    bless("s6_mapmatch", max(0, n - int(S6_DUR / dt)), n, [], allow_parked=1e9)

    # ---------------- verdict ----------------------------------------------
    report["blessed"] = ok_all
    report["relaxed"] = relaxed_used
    out = args.report or os.path.join(rundir, f"window_report_{label}.json")
    json.dump(report, open(out, "w"), indent=1)
    print(f"report -> {out}")
    if not ok_all:
        if args.relaxed and report.get("w_open") and report.get("w_main"):
            print(f"WINDOW_SELECT: RELAXED — every shot fed, floors missed on "
                  f"{','.join(relaxed_used) or 'sub-windows'}")
            return
        print("WINDOW_SELECT: REFUSED — run cannot feed every shot")
        sys.exit(3)
    if relaxed_used:
        print(f"WINDOW_SELECT: RELAXED — every shot fed, floors missed on "
              f"{','.join(relaxed_used)}")
    else:
        print("WINDOW_SELECT: BLESSED — every shot fed")


if __name__ == "__main__":
    main()
