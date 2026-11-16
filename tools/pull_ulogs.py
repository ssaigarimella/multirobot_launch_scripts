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
The folder is named after the vehicle (from UXRCE_DDS_DOM_ID = vehicle id).
"""
import argparse
import datetime
import glob
import os
import struct
import sys
import time

from pymavlink import mavutil

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHUNK = 90


VEHICLE = {1: 'delta', 2: 'buckshee', 3: 'ghost', 4: 'thunderstrike'}


def ports():
    """Every flight controller on USB. Not /dev/serial/by-id: all four Cubes
    report the same USB serial ("0"), so their by-id links collide and only
    one survives (lab, 2026-11-16). Go by the device's USB product name."""
    found = []
    for dev in sorted(glob.glob('/dev/ttyACM*')):
        try:
            base = os.path.realpath('/sys/class/tty/%s/device/..' % os.path.basename(dev))
            prod = open(os.path.join(base, 'product')).read().strip()
            iface = open(os.path.realpath('/sys/class/tty/%s/device' % os.path.basename(dev)) + '/bInterfaceNumber').read().strip()
        except OSError:
            continue
        if iface == '00' and any(k in prod for k in ('Cube', 'PX4', 'Pixhawk', 'Hex', 'ArduPilot')):
            found.append(dev)
    return found


def gcs_heartbeat(m):
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def read_param(m, name, timeout=8.0):
    """One parameter over MAVLink (read-only). None when it does not answer.
    The request is repeated every second: on a busy USB link the first one is
    often not answered (three of four Cubes, 2026-11-16)."""
    t0, t_req = time.time(), 0.0
    while time.time() - t0 < timeout:
        if time.time() - t_req > 1.0:
            gcs_heartbeat(m)
            m.mav.param_request_read_send(m.target_system, 1, name.encode(), -1)
            t_req = time.time()
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
        if msg is not None and msg.param_id.rstrip('\x00') == name:
            if msg.param_type in (5, 6):                       # INT32 sent in the float's bytes
                return struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
            return msg.param_value
    return None


def who(m):
    """Vehicle name from UXRCE_DDS_DOM_ID, which equals the vehicle id on this fleet."""
    dom = read_param(m, 'UXRCE_DDS_DOM_ID')
    return VEHICLE.get(int(dom), None) if dom is not None else None, dom


def log_list(m, timeout=12.0):
    """LOG_ENTRY for every log on the card; the request is repeated until answered."""
    entries, t0, t_req, t_last = {}, time.time(), 0.0, time.time()
    while time.time() - t0 < timeout:
        if not entries and time.time() - t_req > 1.5:
            gcs_heartbeat(m)
            m.mav.log_request_list_send(m.target_system, 1, 0, 0xFFFF)
            t_req = time.time()
        msg = m.recv_match(type='LOG_ENTRY', blocking=True, timeout=0.5)
        if msg is None:
            if entries and time.time() - t_last > 3.0:
                break
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
                gcs_heartbeat(m)
                m.mav.log_request_data_send(m.target_system, 1, entry.id, pos, size - pos)
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
    m.mav.log_request_end_send(m.target_system, 1)
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
        sys.exit('no flight controller found on /dev/ttyACM* (is the USB cable in, QGC closed?)')
    failed = 0
    for port in plist:
        name = os.path.basename(port)
        try:
            m = mavutil.mavlink_connection(port, baud=115200, source_system=255, source_component=190)
            if m.wait_heartbeat(timeout=10) is None:
                raise RuntimeError('no heartbeat (QGC still open?)')
            veh, dom = who(m)
            name = veh or ('sys%d_%s' % (m.target_system, name))
            print('== %s  (%s, MAVLink system %d, UXRCE_DDS_DOM_ID %s)' % (name, port, m.target_system, dom))
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
                # PX4 drops its log listing at LOG_REQUEST_END, and then ignores
                # data requests ("no data for 30 s at byte 0" on the second file,
                # all four vehicles, 2026-11-16): list again before every file.
                log_list(m)
                download(m, e, path)
            m.close()
        except Exception as ex:               # one vehicle failing must not stop the others
            failed += 1
            print('== %s\n  FAILED: %s' % (name, ex))
    print('saved under %s' % a.out if not a.list else 'listed only')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
