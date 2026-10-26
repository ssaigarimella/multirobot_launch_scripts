#!/usr/bin/env python3
"""Repair wifi_rf CSVs split by a multi-line mesh_id value.

A mesh-mode row was written as:
   <7 good fields>,<first line of mesh_id dump>      -> 8 fields
   <junk lines from the dump>                        -> 1 field each
   <last line of dump>,<15 good fields>              -> 16 fields
Rejoin them into the intended 23-field row. Nothing is invented: the only
field that cannot be recovered is net_id, set to 'mesh(recovered)'.
"""
import csv, sys, os

HDR = ("t_utc,t_unix,host,iface,mode,freq_mhz,width_mhz,net_id,peer_count,peer_mac,"
       "signal_dbm,signal_avg_dbm,signal_chains,tx_bitrate_mbps,rx_bitrate_mbps,"
       "tx_packets,rx_packets,tx_retries,tx_failed,tx_bytes,rx_bytes,inactive_ms,connected_s").split(',')

def repair(path):
    rows, pend = [], None
    kept = dropped = fixed = 0
    with open(path) as fh:
        for line in fh:
            f = line.rstrip('\n').split(',')
            if len(f) == len(HDR):
                if f[0] != 't_utc':
                    rows.append(f); kept += 1
                pend = None
            elif len(f) == 8:
                pend = f[:7]
            elif len(f) == 16 and pend:
                rows.append(pend + ['mesh(recovered)'] + f[1:]); fixed += 1; pend = None
            else:
                dropped += 1
    out = path.replace('.csv', '_repaired.csv')
    with open(out, 'w', newline='') as fh:
        w = csv.writer(fh); w.writerow(HDR); w.writerows(rows)
    return out, kept, fixed, dropped, rows

for path in sys.argv[1:]:
    out, kept, fixed, dropped, rows = repair(path)
    modes = {}
    for r in rows:
        modes[r[4] + '/' + r[5]] = modes.get(r[4] + '/' + r[5], 0) + 1
    print("%s" % os.path.basename(path))
    print("  intact=%d  recovered=%d  junk_dropped=%d  ->  %s" % (kept, fixed, dropped, os.path.basename(out)))
    print("  modes: %s" % ', '.join('%s x%d' % (k, v) for k, v in sorted(modes.items())))
    mesh = [r for r in rows if r[4] == 'mesh']
    if mesh:
        sig = [int(r[10]) for r in mesh if r[10].lstrip('-').isdigit()]
        peers = sorted({r[9] for r in mesh})
        print("  MESH: %d rows, peer(s) %s, signal min/mean/max = %d/%.1f/%d dBm"
              % (len(mesh), peers, min(sig), sum(sig)/len(sig), max(sig)))
