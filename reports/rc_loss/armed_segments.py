#!/usr/bin/env python3
"""Longest CONTINUOUS ARMED run per log, using the same counter arithmetic."""
import csv, sys
import rc_frame_loss as R

rows = []
for path in [l.strip() for l in open(sys.argv[1]) if l.strip()]:
    try:
        data, parser = R.read_topics(path, {'input_rc','actuator_armed','vehicle_status'}, 'auto')
    except Exception:
        continue
    rc = data.get('input_rc')
    if not rc or 'rc_total_frame_count' not in rc:
        continue
    ts   = [int(v) for v in rc['timestamp']]
    tot  = [int(v) for v in rc['rc_total_frame_count']]
    lost = ([int(v) for v in rc['rc_lost_frame_count']]
            if 'rc_lost_frame_count' in rc else [0]*len(ts))
    arm  = R._arming_lookup(data)
    state = [arm(t) for t in ts]

    i = 0
    while i < len(ts):
        if state[i] != 'armed':
            i += 1; continue
        j = i
        ft = fl = wraps = 0
        broke = False
        while j + 1 < len(ts) and state[j+1] == 'armed':
            dt = max((ts[j+1]-ts[j])/1e6, 0.0)
            dtot, k = R._corrected_delta(tot[j], tot[j+1], dt)
            dlost, k2 = R._corrected_delta(lost[j], lost[j+1], dt)
            if k == 'reset' or k2 == 'reset':
                broke = True; break
            ft += dtot; fl += dlost
            wraps += (k=='wrap') + (k2=='wrap')
            j += 1
        dur = (ts[j]-ts[i])/1e6
        if ft > 0:
            rows.append(dict(log_path=path, t_start_s=round(ts[i]/1e6,3),
                             duration_s=round(dur,3), n_samples=j-i+1,
                             frames_total=ft, frames_lost=fl, wraps=wraps,
                             rate_hz=round(ft/dur,3) if dur>0 else 0,
                             floor_ppm=round(1e6/ft,3),
                             split_by_reset=int(broke)))
        i = j + 1

rows.sort(key=lambda r: -r['frames_total'])
w = csv.DictWriter(open('armed_segments.csv','w',newline=''), fieldnames=list(rows[0].keys()))
w.writeheader(); [w.writerow(r) for r in rows]
print("armed runs found:", len(rows))
print()
print("TOP 10 by frames observed (longest continuous armed runs):")
print("  %-58s %8s %8s %7s %8s %9s" % ("log","dur_s","frames","lost","rate_hz","floor_ppm"))
for r in rows[:10]:
    print("  %-58s %8.1f %8d %7d %8.2f %9.1f" % (
        r['log_path'].replace('/home/ssg/',''), r['duration_s'], r['frames_total'],
        r['frames_lost'], r['rate_hz'], r['floor_ppm']))
tf = sum(r['frames_total'] for r in rows); tl = sum(r['frames_lost'] for r in rows)
print()
print("ARMED aggregate: frames=%d lost=%d  -> pooled floor %.3f ppm" % (tf, tl, 1e6/tf if tf else 0))
