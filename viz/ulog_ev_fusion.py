#!/usr/bin/env python3
"""ulog_ev_fusion.py - when did PX4 fuse vision position in a flight, and what
was different when it did not.

    ./viz/ulog_ev_fusion.py <log.ulg> [...] --out <dir>

Per log: one figure (height, cs_ev_pos, vision messages logged per second,
quality, vision position innovation test ratio; Offboard shaded) and one line
of numbers. The fused share is TIME-weighted (each estimator_status_flags
sample counts for the time until the next, capped at 2 s). The plain share of
samples is not a time share: that topic is logged mostly on change and a
logger that drops samples (51 % and 46 % in log_70 / log_71) thins the steady
stretches, which is how those two flights were once read as 36 % and 39 %.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from pyulog import ULog  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from fleet_player import INK, INK2, SURF, GRIDC, STATUS  # noqa: E402

C_EV, C_Q, C_R = '#2a78d6', '#1baf7a', '#eb6834'


def held(t, v, q):
    j = np.searchsorted(t, q, 'right') - 1
    return np.where(j >= 0, v[np.clip(j, 0, len(v) - 1)], np.nan)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.size': 9, 'axes.edgecolor': GRIDC, 'xtick.color': INK2, 'ytick.color': INK2,
                         'figure.facecolor': SURF, 'axes.facecolor': SURF, 'savefig.facecolor': SURF})
    for path in a.logs:
        name = Path(path).stem.replace('_UnknownDate', '')
        u = ULog(path)
        t0 = u.start_timestamp

        def get(n):
            for d in u.data_list:
                if d.name == n and d.multi_id == 0:
                    return d.data
            return None
        S = lambda d: (d['timestamp'] - t0) / 1e6   # noqa: E731
        f, ld, vs = get('estimator_status_flags'), get('vehicle_land_detected'), get('vehicle_status')
        lp, vv, it = get('vehicle_local_position'), get('vehicle_visual_odometry'), get('estimator_innovation_test_ratios')
        tf, tl, tv, tp, tq, ti = S(f), S(ld), S(vs), S(lp), S(vv), S(it)
        dur = (u.last_timestamp - t0) / 1e6
        g = np.arange(0, dur, 1.0)
        cnt = np.array([np.sum((tq >= x) & (tq < x + 1)) for x in g])
        kept = np.array([np.sum((tp >= x) & (tp < x + 1)) for x in g])
        nav = held(tv, vs['nav_state'].astype(float), g)
        fig, axs = plt.subplots(5, 1, figsize=(8.2, 8.4), sharex=True)
        for ax in axs:
            on = None
            for i, x in enumerate(g):
                if nav[i] == 14 and on is None:
                    on = x
                if (nav[i] != 14 or i == len(g) - 1) and on is not None:
                    ax.axvspan(on, x, color=STATUS['good'], alpha=0.07, lw=0)
                    on = None
            ax.grid(color=GRIDC, lw=0.6)
        axs[0].plot(tp, lp['dist_bottom'], color=INK, lw=1.2)
        axs[0].set_ylabel('height above\nground  [m]')
        axs[1].step(tf, (f['cs_ev_pos'] > 0).astype(int), where='post', color=C_EV, lw=1.6)
        axs[1].set_yticks([0, 1])
        axs[1].set_yticklabels(['off', 'on'])
        axs[1].set_ylim(-0.2, 1.2)
        axs[1].set_ylabel('vision position\nfused (cs_ev_pos)')
        axs[2].plot(g + 0.5, kept / max(1, np.percentile(kept, 95)) * np.percentile(cnt, 95), color=INK2, lw=0.8, alpha=0.55)
        axs[2].plot(g + 0.5, cnt, color=C_EV, lw=1.3)
        axs[2].set_ylim(0, max(1, cnt.max()) * 1.45)
        axs[2].set_ylabel('vision messages\nlogged per second')
        axs[2].annotate('grey: how much of each second the logger kept (scaled)', (0.01, 0.88), xycoords='axes fraction',
                        fontsize=7.5, color=INK2)
        axs[3].plot(tq, vv['quality'], color=C_Q, lw=1.0)
        axs[3].set_ylim(-5, 110)
        axs[3].set_ylabel('quality field\nsent to PX4')
        r = np.maximum(it['ev_hpos[0]'], it['ev_hpos[1]']).astype(float)
        r[~np.isfinite(r)] = np.nan
        axs[4].plot(ti, r, color=C_R, lw=1.0)
        axs[4].axhline(1.0, color=INK2, lw=0.8, ls=(0, (4, 3)))
        axs[4].set_ylim(0, 1.5)
        axs[4].annotate('rejected above 1', (0.99, 0.70), xycoords='axes fraction', ha='right', fontsize=7.5, color=INK2)
        axs[4].set_ylabel('vision position\ninnovation test ratio')
        axs[4].set_xlabel('time since log start  [s]   (shaded: Offboard)')
        fig.suptitle('%s: vision fusion in PX4 against message arrival, quality and innovation' % name, x=0.01, ha='left',
                     fontsize=10.5, fontweight='bold', color=INK)
        fig.tight_layout(rect=(0, 0, 1, 0.97))
        fig.savefig(out / ('%s_ev_fusion.png' % name), dpi=160)
        plt.close(fig)
        ev = f['cs_ev_pos'] > 0
        w = np.clip(np.r_[np.diff(tf), 0.0], 0, 2.0)
        air = held(tl, 1 - ld['landed'].astype(float), tf) > 0
        offb = held(tv, (vs['nav_state'] == 14).astype(float), tf) > 0
        print('%s: vision position fused %.0f %% of Offboard time, %.0f %% of in-air time (share of samples: %.0f %%); '
              'last fused %.0f s, last vision message %.0f s, log ends %.0f s; test ratio max while messages arrive %.2f; '
              'quality min %d; logger dropouts %.0f %% of the log' % (
                  name, 100 * w[air & offb & ev].sum() / max(1e-9, w[air & offb].sum()),
                  100 * w[air & ev].sum() / max(1e-9, w[air].sum()), 100 * ev.mean(), tf[ev][-1] if ev.any() else -1,
                  tq[-1], dur, np.nanmax(r[ti < tq[-1]]), int(vv['quality'].min()),
                  100 * sum(d.duration for d in u.dropouts) / 1e3 / dur))


if __name__ == '__main__':
    main()
