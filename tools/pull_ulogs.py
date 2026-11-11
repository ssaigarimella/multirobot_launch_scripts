#!/usr/bin/env python3
"""pull_ulogs.py - download the newest ULogs from every flight controller
plugged into this laptop by USB (MAVLink log-download protocol).

    python3 tools/pull_ulogs.py                 # newest 6 logs from each Cube on USB
    python3 tools/pull_ulogs.py --last 10       # newest 10
    python3 tools/pull_ulogs.py --list          # list only, download nothing
    python3 tools/pull_ulogs.py --port /dev/ttyACM0 --ids 72 73

Close QGroundControl first (it holds the serial ports). Read-only on the
vehicle: nothing is erased, no parameter is touched. These vehicles have no
GNSS time, so log dates are usually unknown; that is why the selection is by
newest log number, not by date. Files land in
px4_logs/pulled_<today>/<usb-serial>/log_<id>.ulg and are skipped if already
there with the right size.
UNTESTED on a vehicle as of 2026-11-11 (written with no drone connected).
"""
import argparse
import datetime
import glob
import os
import sys
import time

from pymavlink import mavutil

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHUNK = 90


def ports():
    found = []
    for pat in ('*Cube*-if00', '*CubePilot*-if00', '*PX4*-if00', '*Hex*-if00', '*ArduPilot*-if00'):
        found += glob.glob('/dev/serial/by-id/' + pat)
    return sorted(set(found))


def log_list(m, timeout=5.0):
    m.mav.log_request_list_send(m.target_system, m.target_component, 0, 0xFFFF)
    entries, t_last = {}, time.time()
    while time.time() - t_last < timeout:
        msg = m.recv_match(type='LOG_ENTRY', blocking=True, timeout=1.0)
        if msg is None:
            continue
        t_last = time.time()
        if msg.num_logs == 0:
            break
        entries[msg.id] = msg
        if len(entries) >= msg.num_logs:
            break
    return [entries[k] for k in sorted(entries)]


def download(m, entry, path):
    size, pos, t0, t_rx, t_req = entry.size, 0, time.time(), time.time(), 0.0
    with open(path + '.part', 'wb') as fh:
        while pos < size:
            if time.time() - t_req > 1.0 and time.time() - t_rx > 0.5:
                # (re)request from the first byte we do not have yet
                m.mav.log_request_data_send(m.target_system, m.target_component, entry.id, pos, size - pos)
                t_req = time.time()
            msg = m.recv_match(type='LOG_DATA', blocking=True, timeout=0.5)
            if msg is None:
                if time.time() - t_rx > 30:
                    raise RuntimeError('no data for 30 s at byte %d of %d' % (pos, size))
                continue
            if msg.id != entry.id or msg.ofs != pos:
                continue                      # a gap or a stale chunk; the re-request above recovers
            t_rx = time.time()
            if msg.count == 0:
                break
            fh.write(bytes(msg.data[:msg.count]))
            pos += msg.count
            if pos % (CHUNK * 2000) < CHUNK:
                print('\r    log %d: %5.1f %% (%.0f kB/s)' % (entry.id, 100.0 * pos / size,
                      pos / 1024 / max(time.time() - t0, 0.1)), end='', flush=True)
    m.mav.log_request_end_send(m.target_system, m.target_component)
    if pos < size:
        raise RuntimeError('short download: %d of %d bytes' % (pos, size))
    os.replace(path + '.part', path)
    print('\r    log %d: done, %.1f MB in %.0f s%s' % (entry.id, size / 1e6, time.time() - t0, ' ' * 12))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', action='append', help='serial port(s); default: every Cube on USB')
    ap.add_argument('--last', type=int, default=6, help='newest N logs per vehicle (default 6)')
    ap.add_argument('--ids', type=int, nargs='+', help='explicit log ids instead of --last')
    ap.add_argument('--list', action='store_true', help='list logs, download nothing')
    ap.add_argument('--out', default=os.path.join(ROOT, 'px4_logs', 'pulled_' + datetime.date.today().isoformat()))
    a = ap.parse_args()
    plist = a.port or ports()
    if not plist:
        sys.exit('no flight controller found under /dev/serial/by-id (is the USB cable in, QGC closed?)')
    failed = 0
    for port in plist:
        name = os.path.basename(port).replace('usb-', '').replace('-if00', '')
        print('== %s' % name)
        try:
            m = mavutil.mavlink_connection(port, baud=115200)
            if m.wait_heartbeat(timeout=10) is None:
                raise RuntimeError('no heartbeat (QGC still open?)')
            entries = log_list(m)
            print('  %d logs on the card, ids %s..%s' % (len(entries), entries[0].id if entries else '-',
                                                          entries[-1].id if entries else '-'))
            for e in entries[-max(a.last, 12):]:
                when = datetime.datetime.utcfromtimestamp(e.time_utc).strftime('%Y-%m-%d %H:%M') if e.time_utc else 'unknown date'
                print('    id %-4d %7.1f MB  %s' % (e.id, e.size / 1e6, when))
            if a.list:
                continue
            want = [e for e in entries if e.id in a.ids] if a.ids else entries[-a.last:]
            os.makedirs(os.path.join(a.out, name), exist_ok=True)
            for e in want:
                path = os.path.join(a.out, name, 'log_%d.ulg' % e.id)
                if os.path.exists(path) and os.path.getsize(path) == e.size:
                    print('    log %d: already here' % e.id)
                    continue
                download(m, e, path)
            m.close()
        except Exception as ex:               # one vehicle failing must not stop the others
            failed += 1
            print('  FAILED: %s' % ex)
    print('saved under %s' % a.out if not a.list else 'listed only')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
