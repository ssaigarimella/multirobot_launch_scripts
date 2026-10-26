#!/usr/bin/env python3
"""Figures for the 2026-08-25 mesh bench run. Analysis only, reads collected/ CSVs.
Captions and labels: ASCII hyphens only, American spelling."""
import csv, glob, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

R = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(R, 'figures'); os.makedirs(FIG, exist_ok=True)
COL = {'delta': '#1f77b4', 'ghost': '#d62728'}
plt.rcParams.update({'figure.dpi': 160, 'savefig.dpi': 300, 'font.size': 9,
                     'axes.grid': True, 'grid.alpha': 0.3, 'axes.axisbelow': True,
                     'savefig.bbox': 'tight', 'legend.frameon': False,
                     'axes.unicode_minus': False})   # ASCII hyphen, never a unicode minus

def num(x):
    try: return float(x)
    except Exception: return np.nan

def mesh(d):
    f = sorted(glob.glob(f'{R}/../../collected/{d}/wifi_rf_*_repaired.csv'))[-1]
    r = [x for x in csv.DictReader(open(f)) if x['mode'] == 'mesh' and x['peer_mac'] not in ('NONE', '')]
    t = np.array([float(x['t_unix']) for x in r])
    return dict(rel=t - t.min(),
                sig=np.array([num(x['signal_dbm']) for x in r]),
                avg=np.array([num(x['signal_avg_dbm']) for x in r]),
                br=np.array([num(x['tx_bitrate_mbps']) for x in r]),
                ch=[x['signal_chains'] for x in r])

def ap(d):
    s, b = [], []
    for f in sorted(glob.glob(f'{R}/../../collected/{d}/wifi_rf_*.csv')):
        if 'repaired' in f: continue
        for x in csv.DictReader(open(f)):
            if x.get('mode') != 'managed': continue
            v = num(x['signal_dbm'])
            if not np.isnan(v) and v != 0:      # 0 dBm is the "no frame yet" sentinel
                s.append(v); b.append(num(x['tx_bitrate_mbps']))
    return np.array(s), np.array(b)

M = {d: mesh(d) for d in ('delta', 'ghost')}
AP = {d: ap(d) for d in ('delta', 'ghost')}

# ---- Fig 1: mesh signal time series (T1) --------------------------------
fig, ax = plt.subplots(2, 1, figsize=(7, 4.6), sharex=True)
for i, d in enumerate(('delta', 'ghost')):
    m = M[d]
    ax[i].plot(m['rel'], m['sig'], lw=0.7, color=COL[d], label='per sample')
    ax[i].plot(m['rel'], m['avg'], lw=1.4, color='k', alpha=0.6, label='driver running average')
    ax[i].axvspan(0, 60, color='0.85', zorder=0)
    ax[i].set_ylabel(f'{d}\nsignal (dBm)')
    lo = m['sig'] < -60
    if lo.any():
        ax[i].scatter(m['rel'][lo], m['sig'][lo], s=42, facecolors='none', edgecolors='k', zorder=5)
        for x, y in zip(m['rel'][lo], m['sig'][lo]):
            ax[i].annotate(f'single sample, t+{x:.0f} s', xy=(x, y), xycoords='data',
                           xytext=(0.42, 0.12), textcoords='axes fraction', fontsize=7.5,
                           arrowprops=dict(arrowstyle='->', lw=0.8))
    ax[i].legend(loc='upper right', fontsize=7.5, ncol=2)
ax[0].set_title('802.11s mesh point, 2437 MHz, 20 MHz, static bench pair at about 3 m\n'
                'shaded region is the first 60 s after peering', fontsize=9)
ax[1].set_xlabel('time since mesh formation (s)')
fig.savefig(f'{FIG}/fig1_mesh_signal_timeseries.png'); plt.close(fig)

# ---- Fig 2: settled distributions (T2) ----------------------------------
fig, ax = plt.subplots(1, 3, figsize=(9.5, 2.9))
for d in ('delta', 'ghost'):
    m = M[d]; k = (m['rel'] >= 60) & (m['sig'] != 0)
    ax[0].hist(m['sig'][k], bins=np.arange(-60, -32, 1), alpha=0.6, color=COL[d], label=d)
    sp = []
    for c in np.array(m['ch'])[k]:
        p = [num(v) for v in c.split('|')]
        if len(p) == 2 and not any(np.isnan(p)): sp.append(abs(p[0] - p[1]))
    ax[1].hist(sp, bins=np.arange(0, 34, 2), alpha=0.6, color=COL[d], label=d)
    v, c = np.unique(m['br'][k][~np.isnan(m['br'][k])], return_counts=True)
    ax[2].bar(np.arange(len(v)) + (0.4 if d == 'ghost' else 0), 100 * c / c.sum(), width=0.4,
              color=COL[d], label=d, tick_label=[f'{x:.0f}' for x in v] if d == 'delta' else None)
ax[0].set_xlabel('signal (dBm)'); ax[0].set_ylabel('samples'); ax[0].set_title('Signal, settled window', fontsize=9)
ax[1].set_xlabel('interchain difference (dB)'); ax[1].set_title('Antenna chain spread', fontsize=9)
ax[2].set_xlabel('tx bitrate (Mbit/s)'); ax[2].set_ylabel('percent of samples')
ax[2].set_title('Rate selection (legacy OFDM only)', fontsize=9)
for a in ax: a.legend(fontsize=7.5)
fig.savefig(f'{FIG}/fig2_mesh_settled_distributions.png'); plt.close(fig)

# ---- Fig 3: mesh vs AP (T3) ---------------------------------------------
fig, ax = plt.subplots(1, 2, figsize=(7.4, 3.0))
for d in ('delta', 'ghost'):
    m = M[d]; k = (m['rel'] >= 60) & (m['sig'] != 0)
    s_ap, b_ap = AP[d]
    for arr, ls, lab in ((s_ap, '--', 'AP 5765 MHz'), (m['sig'][k], '-', 'mesh 2437 MHz')):
        x = np.sort(arr); ax[0].plot(x, np.linspace(0, 1, len(x)), ls, color=COL[d], lw=1.3,
                                     label=f'{d}, {lab}')
    ax[1].bar([f'{d}\nAP', f'{d}\nmesh'], [np.nanmean(b_ap), np.nanmean(m['br'][k])],
              color=[COL[d], COL[d]], alpha=[0.45, 1.0][0])
ax[0].set_xlabel('signal (dBm)'); ax[0].set_ylabel('empirical CDF'); ax[0].legend(fontsize=6.8)
ax[0].set_title('Signal: infrastructure vs mesh', fontsize=9)
b = [np.nanmean(AP['delta'][1]), np.nanmean(M['delta']['br']), np.nanmean(AP['ghost'][1]), np.nanmean(M['ghost']['br'])]
ax[1].cla()
ax[1].bar(['delta\nAP', 'delta\nmesh', 'ghost\nAP', 'ghost\nmesh'], b,
          color=[COL['delta'], COL['delta'], COL['ghost'], COL['ghost']], alpha=0.9)
for i, v in enumerate(b): ax[1].text(i, v + 15, f'{v:.0f}', ha='center', fontsize=7.5)
ax[1].set_ylabel('mean tx bitrate (Mbit/s)')
ax[1].set_title('Rate: 80 MHz VHT vs 20 MHz legacy', fontsize=9)
fig.suptitle('Not controlled between the two: band, channel width, and HT/VHT vs legacy rates', fontsize=8, y=1.02)
fig.savefig(f'{FIG}/fig3_mesh_vs_ap.png'); plt.close(fig)

# ---- Fig 4: LoRa TDMA (T4) ----------------------------------------------
lr = [x for x in csv.DictReader(open(f'{R}/../../collected/ghost/lora_rx_20260825_212310.csv'))
      if x['kind'] == 'pose' and x['rssi']]
lt = np.array([float(x['t']) for x in lr]); lt -= lt.min()
lrssi = np.array([float(x['rssi']) for x in lr]); lsnr = np.array([float(x['snr']) for x in lr])
dt = np.diff(lt) * 1000
fig, ax = plt.subplots(1, 3, figsize=(9.5, 2.9))
ax[0].hist(dt, bins=np.arange(180, 201, 1), color='#2ca02c')
ax[0].axvline(190, color='k', ls='--', lw=1.2)
ax[0].text(0.03, 0.80, 'nominal cycle\n4 x 40 ms + 30 ms\n= 190 ms', transform=ax[0].transAxes,
           fontsize=7.5, va='top')
ax[0].set_xlabel('interarrival (ms)'); ax[0].set_ylabel('packets')
ax[0].set_title(f'LoRa TDMA interarrival\njitter (SD) = {dt[(dt>95)&(dt<285)].std():.2f} ms', fontsize=9)
ax[1].plot(lt, lrssi, lw=0.5, color='#2ca02c'); ax[1].set_ylabel('LoRa RSSI (dBm)')
ax[1].set_xlabel('time (s)'); ax[1].set_title('LoRa RSSI, ghost', fontsize=9)
ax[2].plot(lt, lsnr, lw=0.5, color='#9467bd'); ax[2].set_ylabel('LoRa SNR (dB)')
ax[2].set_xlabel('time (s)'); ax[2].set_title('LoRa SNR, ghost', fontsize=9)
fig.savefig(f'{FIG}/fig4_lora_tdma.png'); plt.close(fig)

# ---- Fig 5: the delivery proxy artifact (T5) ----------------------------
ln = list(csv.DictReader(open(f'{R}/../../collected/ghost/link_20260825_212310.csv')))
ta = np.array([float(x['t']) for x in ln]); dl = np.array([float(x['delivery']) for x in ln])
t0 = ta.min(); ta -= t0
lt2 = np.array([float(x['t']) for x in lr]) - t0
rate = np.array([((lt2 >= s) & (lt2 < s + 1)).sum() for s in ta])
fig, ax = plt.subplots(figsize=(7.2, 2.7))
ax.plot(ta, dl, lw=1.1, color='#d62728', label='link_*.csv "delivery" column')
ax.plot(ta, rate / 5.26, lw=0.9, color='#2ca02c', alpha=0.8, label='actual LoRa arrivals, normalized to 5.26 pkt/s')
ax.axvline(913, color='k', ls=':', lw=1.2)
ax.annotate('delta tab 7 starts, t+913 s', (913, 0.55), textcoords='offset points', xytext=(-140, 0), fontsize=7.5)
ax.set_xlabel('time since ghost tab 7 start (s)'); ax.set_ylabel('normalized')
ax.set_title('The "delivery" column tracks peer node presence, not RF delivery', fontsize=9)
ax.legend(fontsize=7.5, loc='center left')
fig.savefig(f'{FIG}/fig5_delivery_proxy_artifact.png'); plt.close(fig)

# ---- Fig 6: cross radio (T6) --------------------------------------------
m = M['ghost']
wt = m['rel']; ok = (~np.isnan(m['sig'])) & (m['sig'] != 0)
lb = np.full(ok.sum(), np.nan)
wt_ok = wt[ok]
for i, tc in enumerate(wt_ok):
    sel = (lt2 >= tc - 0.5) & (lt2 < tc + 0.5)
    if sel.sum(): lb[i] = lrssi[sel].mean()
g = ~np.isnan(lb)
fig, ax = plt.subplots(1, 2, figsize=(7.2, 3.0))
ax[0].scatter(m['sig'][ok][g], lb[g], s=6, alpha=0.35, color='#8c564b')
r = np.corrcoef(m['sig'][ok][g], lb[g])[0, 1]
ax[0].set_xlabel('mesh signal (dBm)'); ax[0].set_ylabel('LoRa RSSI (dBm)')
ax[0].set_title(f'ghost, same drone, exact join\nPearson r = {r:+.3f}', fontsize=9)
brk = m['br'][ok][g]
ax[1].scatter(brk, lb[g], s=6, alpha=0.35, color='#8c564b')
r2 = np.corrcoef(brk[~np.isnan(brk)], lb[g][~np.isnan(brk)])[0, 1]
ax[1].set_xlabel('mesh tx bitrate (Mbit/s)'); ax[1].set_ylabel('LoRa RSSI (dBm)')
ax[1].set_title(f'Pearson r = {r2:+.3f}', fontsize=9)
fig.suptitle('One uncontrolled operating point. Mesh load was about 1 KiB/s, so duty cycle was very low.',
             fontsize=8, y=1.03)
fig.savefig(f'{FIG}/fig6_cross_radio.png'); plt.close(fig)
print("figures written:")
for f in sorted(os.listdir(FIG)): print("  figures/" + f)
