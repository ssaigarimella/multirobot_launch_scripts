#!/usr/bin/env python3
"""set_alignment.py - write swarm_alignment.yaml from field tape measurements
and push it to the drones over the LAN (works with NO internet at the field).

    ./experiments/set_alignment.py --ref delta \
        --drone ghost 3.5 90 0 0 \
        --push delta ghost

    --drone NAME RANGE_M BEARING_DEG YAW_DEG HEIGHT_M   (repeat per drone)

Measurement protocol (stand behind the REFERENCE drone, look along its nose):
  RANGE_M      straight-line distance reference -> this drone
  BEARING_DEG  angle from the reference's nose to this drone, CLOCKWISE
               (directly right = 90, directly left = 270, ahead = 0)
  YAW_DEG      this drone's heading relative to the reference's, clockwise
               (0 = facing the same way -- the geofence assumes this)
  HEIGHT_M     this drone higher than the reference = positive (usually 0)

The reference drone itself is implicit (0 0 0 0). Prints each drone's
geofence in its own frame BEFORE pushing so you can check it on the spot.
The written yaml is also saved in experiments/results/ for the flight record.
"""
import argparse, datetime, os, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
IDS = {'delta': 1, 'buckshee': 2, 'ghost': 3, 'thunderstrike': 4}
IPS = {'delta': '192.168.0.40', 'buckshee': '192.168.0.60',
       'ghost': '192.168.0.50', 'thunderstrike': '192.168.0.70'}
DRONE_PATH = ('/mnt/nova_ssd/workspaces/isaac_ros-dev/src/multi_drone_nvblox'
              '/config/swarm_alignment.yaml')


def password():
    for line in open(os.path.join(ROOT, 'credentials.env')):
        if line.startswith('JETSON_PASS='):
            return line.split('=', 1)[1].strip().strip('"')
    sys.exit('credentials.env missing')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--ref', required=True, choices=sorted(IDS))
    ap.add_argument('--drone', nargs=5, action='append', default=[],
                    metavar=('NAME', 'RANGE', 'BEARING', 'YAW', 'HEIGHT'))
    ap.add_argument('--push', nargs='*', default=[], choices=sorted(IDS))
    ap.add_argument('--forward', type=float, default=35.0)
    ap.add_argument('--width', type=float, default=35.0)
    a = ap.parse_args()

    entries = {IDS[a.ref]: (a.ref, 0.0, 0.0, 0.0, 0.0)}
    for name, r, b, y, h in a.drone:
        if name not in IDS:
            sys.exit('unknown drone %s' % name)
        if name == a.ref:
            sys.exit('%s is the reference; do not list it with --drone' % name)
        entries[IDS[name]] = (name, float(r), float(b) % 360.0,
                              float(y), float(h))
    stamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
    lines = ['# written by experiments/set_alignment.py %s' % stamp,
             '# reference = %s; bearings clockwise from its nose' % a.ref,
             'reference_drone_id: %d' % IDS[a.ref], '', 'drones:']
    for vid in sorted(entries):
        name, r, b, y, h = entries[vid]
        lines += ['  %d:  # %s' % (vid, name),
                  '    range_m: %.3f' % r, '    bearing_deg: %.1f' % b,
                  '    yaw_deg: %.1f' % y, '    height_offset_m: %.3f' % h]
    text = '\n'.join(lines) + '\n'

    rec = os.path.join(HERE, 'results', 'swarm_alignment_%s.yaml'
                       % datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(os.path.dirname(rec), exist_ok=True)
    open(rec, 'w').write(text)
    print(text)

    # geofence preview per drone, using the same code the launch uses
    sys.path.insert(0, os.path.expanduser('~/jfr_dev/multi_drone_nvblox'))
    try:
        from mdn_core.geofence_logic import team_forward_box
        print('geofence each drone will fly (own odom frame):')
        for vid in sorted(entries):
            x0, y0, x1, y1, warn = team_forward_box(rec, vid, a.forward,
                                                    a.width, 0.0)
            print('  %-14s x %6.2f..%6.2f  y %6.2f..%6.2f  %s'
                  % (entries[vid][0], x0, x1, y0, y1, warn or ''))
    except Exception as exc:
        print('(geofence preview unavailable: %s)' % exc)

    if not a.push:
        print('\nnot pushed (add --push <drones>). Saved: %s' % rec)
        return
    missing = [d for d in a.push if IDS[d] not in entries]
    if missing:
        sys.exit('refusing to push: %s not in the yaml -- their maps would '
                 'never merge. Add --drone lines for them.' % ' '.join(missing))
    pw = password()
    for d in a.push:
        r = subprocess.run(['sshpass', '-p', pw, 'scp', '-o',
                            'StrictHostKeyChecking=no', '-o', 'ConnectTimeout=8',
                            rec, '%s@%s:%s' % (d, IPS[d], DRONE_PATH)],
                           capture_output=True, text=True)
        print('  pushed -> %-14s %s' % (d, 'OK' if r.returncode == 0
                                        else 'FAILED: ' + r.stderr.strip()[:80]))
    print('Saved for the record: %s' % rec)
    print('Restart tabs 3, 5 and 7 (or the whole launcher) to pick it up.')


if __name__ == '__main__':
    main()
