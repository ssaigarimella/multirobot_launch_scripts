#!/usr/bin/env python3
"""lora_bench_reboot.py - bench test of the LoRa mesh's recovery from a
reboot of one radio (default: delta = node 0, the sync master), on the real
radios of the powered drones. Stacks must be down (the bridge holds the port).

    ./tests/lora_bench_reboot.py --trials 15 --wait 20 [--reboot delta]

Each trial: reboot the radio (EN pulse via RTS), wait, then ask every radio
which peers it currently lists as alive (heard within 3 s). The mesh is FULL
when every radio lists every other powered radio. Pushes
tests/lora_radio_tool.py into each container first.
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
IPS = {'delta': '192.168.0.40', 'buckshee': '192.168.0.60',
       'ghost': '192.168.0.50', 'thunderstrike': '192.168.0.70'}
NODE = {'delta': 0, 'buckshee': 1, 'ghost': 2, 'thunderstrike': 3}


def password():
    pw = os.environ.get('JETSON_PASS')
    if pw:
        return pw
    for line in open(HERE.parent / 'credentials.env'):
        if line.startswith('JETSON_PASS='):
            return line.split('=', 1)[1].strip().strip('"\'')
    sys.exit('JETSON_PASS not set and not in credentials.env')


def ssh(d, cmd, timeout=40):
    r = subprocess.run(['sshpass', '-p', PW, 'ssh', '-o', 'StrictHostKeyChecking=no',
                        '-o', 'ConnectTimeout=6', f'{d}@{IPS[d]}', cmd],
                       capture_output=True, text=True, timeout=timeout)
    return r.stdout.strip()


def tool(d, args):
    out = ssh(d, f'docker exec isaac_ros_realsense python3 /tmp/lora_radio_tool.py {args}')
    try:
        return json.loads(out.splitlines()[-1])
    except (ValueError, IndexError):
        return {'error': out[-200:]}


def push(d):
    subprocess.run(['sshpass', '-p', PW, 'scp', '-o', 'StrictHostKeyChecking=no',
                    str(HERE / 'lora_radio_tool.py'), f'{d}@{IPS[d]}:/tmp/'],
                   check=True, capture_output=True)
    ssh(d, 'docker cp /tmp/lora_radio_tool.py isaac_ros_realsense:/tmp/')


def poll(drones):
    res = {}

    def one(d):
        res[d] = tool(d, 'status 1.5')
    th = [threading.Thread(target=one, args=(d,)) for d in drones]
    [t.start() for t in th]
    [t.join() for t in th]
    return res


def main():
    global PW
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', type=int, default=15)
    ap.add_argument('--wait', type=float, default=20.0)
    ap.add_argument('--reboot', default='delta')
    ap.add_argument('--drones', nargs='+', default=list(IPS))
    a = ap.parse_args()
    PW = password()
    for d in a.drones:
        push(d)
    ids = {NODE[d] for d in a.drones}
    print('before:', {d: r.get('alive') for d, r in poll(a.drones).items()})
    broken = 0
    for k in range(a.trials):
        tool(a.reboot, 'reset')
        time.sleep(a.wait)
        st = poll(a.drones)
        missing = {d: sorted(ids - {NODE[d]} - set(r.get('alive', []))) for d, r in st.items()}
        full = not any(missing.values())
        broken += not full
        print(f'trial {k + 1:2d}: {"FULL" if full else "BROKEN"}  '
              + '  '.join(f'{d}:{"ok" if not m else "missing " + str(m)}' for d, m in missing.items()),
              flush=True)
    print(f'{a.reboot} rebooted {a.trials}x: mesh broken {a.wait:.0f} s later in {broken}/{a.trials}')


if __name__ == '__main__':
    main()
