#!/usr/bin/env python3
"""Synthetic checks for the wrap (C1) and reset (C2) logic in rc_frame_loss."""
import rc_frame_loss as R

def d(prev, cur, dt=0.5):
    return R._corrected_delta(prev, cur, dt)

fails = 0
def check(label, got, want):
    global fails
    ok = got == want
    if not ok: fails += 1
    print(("  PASS  " if ok else "  FAIL  ") + label + "   got=%r want=%r" % (got, want))

print("C1 uint16 wraparound")
check("normal step 1499->2630 (dt=17s)", d(1499, 2630, 17.0), (1131, 'normal'))
check("wrap 65500->30 (dt=0.5s, +66)",   d(65500, 30),        (66,   'wrap'))
check("wrap 65535->0",                   d(65535, 0),         (1,    'wrap'))
check("wrap 65000->200 (dt=10s, +736)",  d(65000, 200, 10.0), (736,  'wrap'))

print("C2 counter reset on reboot (must NOT be silently bridged)")
check("reboot 40000->5 (dt=0.5s)",       d(40000, 5),         (None, 'reset'))
check("reboot 500->12 (dt=0.5s)",        d(500, 12),          (None, 'reset'))
check("implausible fwd jump 100->60000", d(100, 60000),       (None, 'reset'))

print("boundary: corrected delta right at the plausibility ceiling")
# dt=0.5 -> allowed = 0.5*500 + 100 = 350
check("wrap of exactly 350 accepted",    d(65536-350+0, 0)[1], 'wrap')
check("wrap of 351 rejected as reset",   d(65536-351, 0)[1],  'reset')

print("C3 ratio is summed deltas, not mean of per-sample ratios")
# 3 steps: 1000 frames 0 lost, 10 frames 1 lost, 1000 frames 0 lost
# per-sample mean would be (0 + 100000 + 0)/3 = 33333 ppm; correct = 1/2010 = 497.5 ppm
tot_d = [1000, 10, 1000]; lost_d = [0, 1, 0]
correct = sum(lost_d)/sum(tot_d)*1e6
naive = sum(l/t for l, t in zip(lost_d, tot_d))/3*1e6
check("summed-delta ratio", round(correct, 1), 497.5)
print("    (naive per-sample mean would have been %.1f ppm - 67x wrong)" % naive)

print()
print("FAILURES: %d" % fails)
raise SystemExit(1 if fails else 0)
