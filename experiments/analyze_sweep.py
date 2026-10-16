#!/usr/bin/env python3
"""analyze_sweep.py - turn a LoRa power sweep into a measured channel curve.

    python3 experiments/analyze_sweep.py experiments/results/sweep_<stamp>

Joins markers.csv (laptop time) with collected/<drone>/lora_rx_*.csv (drone
time, NTP-synced) and, for every power window and directed link, computes the
delivery ratio against the deterministic TDMA cadence.

Expected rate per peer = 1 / 0.190 s = 5.263 packets/s (v7.2/v7.3 firmware,
NUM_NODES=4, tau 40 ms, guard 30 ms -- one beacon per peer per cycle). Only
'pose' rows count (beacons); 'msg' rows ride inside a beacon.

Writes into the sweep directory:
  per_link.csv                 power, rx, tx, packets, delivery, rssi, snr
  channel_curve_measured.csv   rssi_dbm,delivery_ratio  -- drop-in for Lucas's
                               Study B simulator (--curve <file>)
"""
import csv, glob, os, sys
from collections import defaultdict

CADENCE_HZ = 1.0 / 0.190
SETTLE_S = 5.0          # ignore the first seconds after each power change
IDS = {'delta': 1, 'buckshee': 2, 'ghost': 3, 'thunderstrike': 4}
NODE_TO_NAME = {v - 1: k for k, v in IDS.items()}   # mesh node = vehicle_id-1


def load_rx(collected, drone):
    rows = []
    for f in sorted(glob.glob(os.path.join(collected, drone, 'lora_rx_*.csv'))):
        with open(f) as fh:
            for r in csv.DictReader(fh):
                if r.get('kind') != 'pose':
                    continue
                try:
                    rows.append((float(r['t']), int(r['peer']),
                                 float(r['rssi']), float(r['snr'])))
                except (KeyError, ValueError):
                    continue
    return rows


def main(sweep_dir):
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(sweep_dir))))
    collected = os.path.join(root, 'collected')
    windows = defaultdict(dict)       # (t0,t1,p) -> {drone: status}
    with open(os.path.join(sweep_dir, 'markers.csv')) as fh:
        for r in csv.DictReader(fh):
            key = (float(r['t_start']), float(r['t_end']),
                   int(r['power_req_dbm']))
            windows[key][r['drone']] = r['status']
    drones = sorted({d for w in windows.values() for d in w})
    rx = {d: load_rx(collected, d) for d in drones}
    if not any(rx.values()):
        sys.exit('no lora_rx rows under %s -- run ./fleet_ctl collect %s first'
                 % (collected, ' '.join(drones)))

    out, curve = [], []
    for (t0, t1, p), status in sorted(windows.items(), key=lambda k: k[0][2]):
        a, b = t0 + SETTLE_S, t1
        dur = b - a
        for rxd in drones:
            if status.get(rxd) != 'OK':
                continue
            by_peer = defaultdict(list)
            for t, peer, rssi, snr in rx[rxd]:
                if a <= t <= b:
                    by_peer[peer].append((rssi, snr))
            for node, name in NODE_TO_NAME.items():
                if name == rxd or status.get(name) != 'OK':
                    continue          # only links where BOTH ends were at p
                pk = by_peer.get(node, [])
                dr = min(1.0, len(pk) / (CADENCE_HZ * dur)) if dur > 0 else 0
                rssi = sum(x[0] for x in pk) / len(pk) if pk else float('nan')
                snr = sum(x[1] for x in pk) / len(pk) if pk else float('nan')
                out.append((p, rxd, name, len(pk), dr, rssi, snr))
                if pk:
                    curve.append((rssi, dr))

    with open(os.path.join(sweep_dir, 'per_link.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['power_dbm', 'rx', 'tx', 'packets', 'delivery_ratio',
                    'rssi_dbm', 'snr_db'])
        w.writerows([(p, r, t, n, '%.4f' % d, '%.1f' % s, '%.1f' % q)
                     for p, r, t, n, d, s, q in out])
    with open(os.path.join(sweep_dir, 'channel_curve_measured.csv'), 'w',
              newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['rssi_dbm', 'delivery_ratio'])
        w.writerows([('%.1f' % s, '%.4f' % d) for s, d in sorted(curve)])

    print('%-6s %-14s %-14s %7s %8s %8s %6s' % ('dBm', 'rx', 'tx', 'pkts',
                                               'DR', 'RSSI', 'SNR'))
    for p, r, t, n, d, s, q in out:
        print('%-6d %-14s %-14s %7d %8.3f %8.1f %6.1f' % (p, r, t, n, d, s, q))
    print('\nwrote per_link.csv and channel_curve_measured.csv in', sweep_dir)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
