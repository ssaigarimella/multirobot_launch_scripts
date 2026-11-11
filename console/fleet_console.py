#!/usr/bin/env python3
"""fleet_console.py - one window to bring the fleet up, watch every tab of
every drone, and know when each one is READY FOR TAKEOFF.

    ./console/fleet_console.py                 # all four drones listed
    ./console/fleet_console.py delta buckshee  # pre-select drones

Operator convenience only, like fleet_ctl: it never arms, takes off, lands,
changes PX4 parameters or touches network settings. It drives the tools you
already use by hand (launch_fleet_tmux.sh tabs, fleet_ctl stop/preflight/
collect, record_fleet.sh) and reads live status from a read-only probe on each
drone (console/probe_host.py + console/probe_ros.py, pushed automatically).

BRING UP runs, per selected drone and in parallel, the sequence and checks
the 2026-10-23 field day needed by hand:
  devices on USB -> mirror device nodes into the container (devsync) ->
  tab 2 PX4 link (restart once; then "power-cycle the FMU") ->
  tab 1 camera + cuVSLAM (restart up to 3x on no camera / bad init) ->
  tabs 3-7 -> LoRa hears the other radios -> tab 8 zenoh (multi-drone) ->
  preflight -> optional recording -> READY FOR TAKEOFF.
Every cell is a live check: green ok, yellow working/degraded, red needs you
(the text says what to do), grey not running. Click a cell for restart / log /
terminal.
"""
import json
import os
import queue
import re
import shlex
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FLEET = {'delta': (1, '192.168.0.40'), 'buckshee': (2, '192.168.0.60'),
         'ghost': (3, '192.168.0.50'), 'thunderstrike': (4, '192.168.0.70')}
WS = '/mnt/nova_ssd/workspaces/isaac_ros-dev'
CONTAINER = 'isaac_ros_realsense'
TAB_NAME = {1: 'cuVSLAM', 2: 'PX4 link', 3: 'FIS', 4: 'Guard', 5: 'Planner',
            6: 'Shared map', 7: 'Coord+LoRa', 8: 'Zenoh'}
TAB_LAUNCH = {1: 'realsense_example.launch.py', 2: 'vio_bridge.launch.py',
              3: 'fis_stage.launch.py', 4: 'reactive_guard.launch.py',
              5: 'planner_stage.launch.py', 6: 'shared_mapper.launch.py',
              7: 'coordination_stage.launch.py'}
COLS = ['Link', 'Devices'] + ['%d %s' % (n, TAB_NAME[n]) for n in range(1, 9)] + \
       ['Preflight', 'Recording', 'READY']
COLOR = {'ok': '#0ca30c', 'wait': '#fab219', 'bad': '#d03b3b', 'off': '#c3c2b7'}
FG = {'ok': 'white', 'wait': 'black', 'bad': 'white', 'off': '#0b0b0b'}
INIT_MAX_POS = 0.5       # m: cuVSLAM must start near its origin (bad init otherwise)
INIT_MAX_Z = 0.3


def password():
    for line in open(os.path.join(ROOT, 'credentials.env')):
        if line.startswith('JETSON_PASS='):
            return line.split('=', 1)[1].strip().strip('"\'')
    raise SystemExit('JETSON_PASS missing from credentials.env')


PW = password()
SSH_OPTS = ['-o', 'StrictHostKeyChecking=no', '-o', 'ConnectTimeout=6',
            '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=3', '-o', 'LogLevel=ERROR']


def ssh(drone, remote, timeout=60):
    _, ip = FLEET[drone]
    try:
        r = subprocess.run(['sshpass', '-p', PW, 'ssh'] + SSH_OPTS + ['%s@%s' % (drone, ip), remote],
                           capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return 124, 'timeout'


def local(cmd, timeout=600, env=None):
    try:
        r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=timeout,
                           env=dict(os.environ, **(env or {})))
        return r.returncode, r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return 124, 'timeout'


# ===================================================================== probe
class Link(threading.Thread):
    """One persistent SSH connection per drone, streaming probe JSON."""

    def __init__(self, drone, log):
        super().__init__(daemon=True)
        self.drone, self.log = drone, log
        self.st, self.t = None, 0.0
        self.proc = None
        self.alive = True
        self.connected = False

    def push(self):
        _, ip = FLEET[self.drone]
        for src, dst in (('probe_host.py', 'fleet_probe_host.py'), ('probe_ros.py', 'fleet_probe_ros.py')):
            subprocess.run(['sshpass', '-p', PW, 'scp'] + SSH_OPTS[:4] +
                           [os.path.join(HERE, src), '%s@%s:/tmp/%s' % (self.drone, ip, dst)],
                           capture_output=True, timeout=30)

    def run(self):
        vid, ip = FLEET[self.drone]
        while self.alive:
            try:
                self.push()
                self.proc = subprocess.Popen(
                    ['sshpass', '-p', PW, 'ssh'] + SSH_OPTS + ['%s@%s' % (self.drone, ip),
                     'python3 -u /tmp/fleet_probe_host.py %s %d' % (self.drone, vid)],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL,
                    text=True)
                for line in self.proc.stdout:
                    try:
                        self.st, self.t = json.loads(line), time.monotonic()
                        if not self.connected:
                            self.connected = True
                            self.log('%s: probe connected' % self.drone)
                    except ValueError:
                        pass
                    if not self.alive:
                        break
            except Exception as e:
                self.log('%s: probe error %s' % (self.drone, e))
            if self.connected:
                self.log('%s: probe link lost - reconnecting' % self.drone)
            self.connected = False
            if self.alive:
                time.sleep(3)

    def status(self):
        if self.st is None or time.monotonic() - self.t > 5:
            return None
        return self.st

    def stop(self):
        self.alive = False
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()


# ============================================================ evaluation
def g(d, *keys, default=None):
    for k in keys:
        if not isinstance(d, dict) or k not in d or d[k] is None:
            return default
        d = d[k]
    return d


def fresh(age, limit):
    """age (s) of the last message is known and under limit (0.0 is fresh)."""
    return age is not None and age < limit


def evaluate(drone, st, flying, extra):
    """Cell -> (level, text). `extra` holds bring-up/preflight/recording facts."""
    cells = {c: ('off', '') for c in COLS}
    if st is None:
        cells['Link'] = ('bad', 'unreachable')
        return cells, False
    host, ros = st.get('host', {}), st.get('ros')
    vid = FLEET[drone][0]
    others = [FLEET[d][0] for d in flying if d != drone]
    multi = drone in flying and len(flying) > 1
    cells['Link'] = ('ok', 'probe %ss' % st.get('ros_age')) if host.get('container') and ros \
        else ('bad', 'container stopped' if not host.get('container') else 'ROS probe starting')
    # ---- devices: as the host sees them, and inside the container ----
    kinds = set(host.get('serial', {}).values())
    missing = [n for n, ok in (('camera', host.get('realsense')), ('PX4 serial', 'px4' in kinds),
                               ('LoRa radio', 'radio' in kinds)) if not ok]
    cdevs = g(ros, 'devs', default={}) or {}
    in_ctr = (cdevs.get('video', 0) >= host.get('video', 0) and
              len(cdevs.get('tty', [])) >= len(host.get('serial', {})))
    if missing:
        cells['Devices'] = ('bad', 'no %s on USB - replug' % ', '.join(missing))
    elif not in_ctr:
        cells['Devices'] = ('wait', 'container missing device nodes - devsync')
    else:
        cells['Devices'] = ('ok', 'cam, PX4, radio')
    procs = g(ros, 'procs', default={}) or {}
    armed = bool(g(ros, 'px4', 'armed'))

    def tab(n, ok, text_ok, text_bad):
        if not procs.get(str(n)):
            return ('off', 'not running')
        return ('ok', text_ok) if ok else ('wait', text_bad)

    # 1 camera + cuVSLAM (+ bad-init check while on the ground)
    cam_hz, odom_hz, pos = g(ros, 'cam', 'hz', default=0), g(ros, 'odom', 'hz', default=0), \
        g(ros, 'odom', 'pos')
    if procs.get('1'):
        if cam_hz < 5:
            cells['1 cuVSLAM'] = ('wait', 'no camera frames')
        elif odom_hz < 10:
            cells['1 cuVSLAM'] = ('wait', 'cam %.0f Hz, cuVSLAM %.0f Hz' % (cam_hz, odom_hz))
        elif pos and not armed and (abs(pos[0]) + abs(pos[1]) > INIT_MAX_POS or abs(pos[2]) > INIT_MAX_Z):
            cells['1 cuVSLAM'] = ('bad', 'BAD INIT |p|=%.1f m - restart tab 1' % (abs(pos[0]) + abs(pos[1])))
        else:
            cells['1 cuVSLAM'] = ('ok', '%.0f Hz (%.2f, %.2f, %.2f)' % ((odom_hz,) + tuple(pos or (0, 0, 0))))
    else:
        cells['1 cuVSLAM'] = ('off', 'not running')
    # 2 PX4 link
    px_hz = g(ros, 'px4', 'hz', default=0)
    if procs.get('2'):
        if px_hz < 5:
            cells['2 PX4 link'] = ('wait', 'no PX4 data (agent %s)' % ('up' if procs.get('agent') else 'down'))
        elif not g(ros, 'px4', 'xy_valid'):
            cells['2 PX4 link'] = ('wait', '%.0f Hz, PX4 position not valid yet' % px_hz)
        elif not g(ros, 'px4', 'preflight_ok', default=True):
            cells['2 PX4 link'] = ('wait', '%.0f Hz, PX4 preflight checks failing' % px_hz)
        else:
            nav = g(ros, 'px4', 'nav')
            mode = {14: 'OFFBOARD', 2: 'POSCTL', 4: 'HOLD', 18: 'LAND'}.get(nav, 'nav %s' % nav)
            cells['2 PX4 link'] = ('ok', '%.0f Hz %s %s' % (px_hz, 'ARMED' if armed else 'disarmed', mode))
    # 3-6
    cells['3 FIS'] = tab(3, fresh(g(ros, 'fis', 'age'), 10),
                         '%s viewpoints' % g(ros, 'fis', 'n', default='?'), 'no frontiers published yet')
    cells['4 Guard'] = tab(4, fresh(g(ros, 'guard', 'age'), 3),
                           g(ros, 'guard', 'state', default='?'), 'no guard status')
    cells['5 Planner'] = tab(5, fresh(g(ros, 'planner', 'age'), 3),
                             g(ros, 'planner', 'state', default='?'), 'no planner state')
    cells['6 Shared map'] = tab(6, fresh(g(ros, 'shared', 'age'), 10),
                                '%.0f Hz' % g(ros, 'shared', 'hz', default=0), 'no shared map yet')
    # 7 coordination + LoRa radios heard
    if procs.get('7'):
        heard = set(g(ros, 'lora', 'heard', default=[]) or [])
        want = set(others)
        if not fresh(g(ros, 'coord', 'age'), 5):
            cells['7 Coord+LoRa'] = ('wait', 'coordination not publishing')
        elif not procs.get('lora_bridge'):
            cells['7 Coord+LoRa'] = ('bad', 'LoRa bridge not running')
        elif want - heard:
            cells['7 Coord+LoRa'] = ('wait', 'LoRa hears %s, missing %s' % (sorted(heard) or 'none',
                                                                         sorted(want - heard)))
        else:
            cells['7 Coord+LoRa'] = ('ok', 'LoRa hears %s' % (sorted(heard) or 'no peers (solo)'))
    # 8 zenoh (multi-drone only)
    if not multi:
        cells['8 Zenoh'] = ('off', 'solo: not needed')
    elif not host.get('zenoh'):
        cells['8 Zenoh'] = ('off' if 'fleet_tab8' not in host.get('sessions', []) else 'wait',
                            'bridge not running')
    else:
        zp = set(g(ros, 'zenoh_peers', default=[]) or [])
        miss = set(others) - zp
        cells['8 Zenoh'] = ('ok', 'peers %s' % sorted(zp)) if not miss else \
            ('wait', 'waiting for peers %s' % sorted(miss))
    # preflight / recording (facts from the console's own actions + probe)
    pf = extra.get('preflight')
    if pf:
        cells['Preflight'] = pf
    if host.get('recording'):
        bag = host.get('bag') or {}
        rate = (bag.get('rate') or 0) / 1e6
        cells['Recording'] = ('ok', '%s %.1f MB/s' % (bag.get('run', '?'), rate)) if rate > 0.05 else \
            ('wait', '%s not growing' % bag.get('run', '?'))
    elif extra.get('want_record'):
        cells['Recording'] = ('wait', 'not recording')
    # overrides from a running bring-up (what it is doing right now)
    for c, v in extra.get('stage', {}).items():
        if c in cells and cells[c][0] != 'ok':
            cells[c] = v
    need = ['Link', 'Devices', '1 cuVSLAM', '2 PX4 link', '3 FIS', '4 Guard', '5 Planner',
            '6 Shared map', '7 Coord+LoRa', 'Preflight']
    if multi:
        need.append('8 Zenoh')
    if extra.get('want_record'):
        need.append('Recording')
    ready = all(cells[c][0] == 'ok' for c in need)
    blockers = [c.split()[0] if c[0].isdigit() else c.lower() for c in need if cells[c][0] != 'ok']
    cells['READY'] = ('ok', 'READY FOR TAKEOFF') if ready else \
        ('bad' if any(cells[c][0] == 'bad' for c in need) else 'wait',
         'waiting on ' + ', '.join(blockers))
    return cells, ready


# ============================================================ actions
def start_tab(drone, n, flying, opts):
    # Only called for a tab that is not running. If its launch died, its tmux
    # session can still be open (the pane is kept for the log), and
    # launch_fleet_tmux.sh would then attach to it instead of starting anew.
    ssh(drone, 'tmux kill-session -t fleet_tab%d 2>/dev/null; true' % n, timeout=20)
    args = ['./launch_fleet_tmux.sh', '--tab', drone, str(n), '--drones=' + ','.join(flying),
            '--planner=' + opts.get('planner', 'simple')]
    if opts.get('debug'):
        args.append('--debug')
    res = local(args, timeout=90, env={'TAB_DETACHED': '1'})
    if n == 1:
        # The camera driver can reset the camera a few seconds after launch
        # (initial_reset: true on thunderstrike). The camera then comes back
        # under a new /dev/bus/usb node that the container cannot see, and
        # librealsense retries 'failed to set power state' forever. Mirroring
        # device nodes while it retries lets the retry succeed; restarting the
        # tab instead resets the camera again (bench-verified 2026-10-27).
        threading.Thread(target=devsync, args=(drone, 45), daemon=True).start()
    return res


def stop_tab(drone, n):
    if n == 8:
        return ssh(drone, "tmux kill-session -t fleet_tab8 2>/dev/null; tmux kill-session -t fleet_rf 2>/dev/null; "
                          "echo %s | sudo -S -p '' pkill -f '[z]enoh-bridge-ros2dds'; true" % shlex.quote(PW))
    pat = '[b]in/ros2 launch .*' + TAB_LAUNCH[n]
    remote = (
        "P=$(docker exec %(c)s pgrep -f '%(p)s' | head -1); "
        "if [ -n \"$P\" ]; then docker exec %(c)s kill -INT $P; "
        "for i in $(seq 24); do docker exec %(c)s kill -0 $P 2>/dev/null || break; sleep 0.5; done; "
        "docker exec %(c)s sh -c \"kill -0 $P 2>/dev/null && { pkill -9 -P $P; kill -9 $P; }\" 2>/dev/null; fi; "
        "tmux kill-session -t fleet_tab%(n)d 2>/dev/null; true" % dict(c=CONTAINER, p=pat, n=n))
    return ssh(drone, remote, timeout=60)


def devsync(drone, secs=4):
    script = os.path.join(ROOT, 'tools', 'devsync.sh')
    _, ip = FLEET[drone]
    subprocess.run(['sshpass', '-p', PW, 'scp'] + SSH_OPTS[:4] + [script, '%s@%s:/tmp/devsync.sh' % (drone, ip)],
                   capture_output=True, timeout=30)
    return ssh(drone, 'sh /tmp/devsync.sh %d' % secs, timeout=secs + 60)


CAMERA_ADVICE = 'unplug and replug the camera USB cable, then Restart tab 1'
TAB1_SIGNATURES = (('failed to set power state', 'camera on USB but will not power up'),
                   ('is NOT found', 'camera not found by librealsense'),
                   ('No RealSense devices were found', 'camera not found by librealsense'))


def tab1_signature(drone):
    """Known camera failure in tab 1's newest pane log, or None."""
    rc, out = ssh(drone, "L=$(ls -t %s/experiment_logs/pane_fleet_tab1_*.log 2>/dev/null | head -1); "
                         "[ -n \"$L\" ] && tail -c 200000 \"$L\" | tr -d '\\000'" % WS, timeout=30)
    for needle, label in TAB1_SIGNATURES:
        if needle in out:
            return label
    return None


def preflight(drone):
    rc, out = local(['./fleet_ctl', 'preflight', drone], timeout=150)
    fails = [re.split(r'\s{2,}', l.split('[FAIL]', 1)[1].strip())[0]
             for l in out.splitlines() if '[FAIL]' in l]
    score = re.search(r'PREFLIGHT \w+: (\d+/\d+) passed', out)
    if rc == 0:
        return ('ok', 'PASS %s' % (score.group(1) if score else '')), out
    return ('bad', 'FAIL %s: %s' % (score.group(1) if score else '',
                                    '; '.join(fails) if fails else 'see log')), out


# ============================================================ bring-up
class BringUp(threading.Thread):
    def __init__(self, console, drone, flying, opts):
        super().__init__(daemon=True)
        self.c, self.d, self.flying, self.opts = console, drone, list(flying), dict(opts)
        self.cancel = False

    def ev(self, msg):
        self.c.log('%s: %s' % (self.d, msg))

    def stage(self, cell, level, text):
        self.c.set_stage(self.d, cell, (level, text))

    def st(self):
        return self.c.links[self.d].status()

    def wait(self, pred, timeout, what):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and not self.cancel:
            s = self.st()
            try:
                if s and pred(s.get('host', {}), s.get('ros') or {}):
                    return True
            except Exception:
                pass
            time.sleep(1)
        if not self.cancel:
            self.ev('%s: not after %ds' % (what, timeout))
        return False

    def running(self, n):
        s = self.st()
        return bool(s and g(s, 'ros', 'procs', str(n)))

    def run(self):
        try:
            self._run()
        except Exception as e:
            self.ev('bring-up crashed: %r' % e)
        finally:
            self.c.bringup_done(self.d)

    def pos_bad(self):
        p = g(self.st(), 'ros', 'odom', 'pos')
        return p if p and (abs(p[0]) + abs(p[1]) > INIT_MAX_POS or abs(p[2]) > INIT_MAX_Z) else None

    def ensure_tab1(self, tries, restart_first=False):
        """Camera + cuVSLAM up and sitting near its origin, restarting tab 1
        up to `tries` times. The failure text names the cause found in the
        tab's log when it is a known one."""
        d, stopped, why = self.d, restart_first, 'unknown'
        if restart_first:
            stop_tab(d, 1)
        for attempt in range(1, tries + 1):
            if self.cancel:
                return False
            if stopped or not self.running(1):
                self.stage('1 cuVSLAM', 'wait', 'starting tab 1 (%d/%d)' % (attempt, tries))
                start_tab(d, 1, self.flying, self.opts)
            stopped = True        # every path below that loops has stopped tab 1
            if not self.wait(lambda h, r: g(r, 'cam', 'hz', default=0) >= 5, 40, 'camera frames'):
                why = tab1_signature(d) or 'no camera frames'
                self.ev('%s - devsync + restart tab 1' % why)
                stop_tab(d, 1)
                devsync(d)
                continue
            if not self.wait(lambda h, r: g(r, 'odom', 'hz', default=0) >= 10, 60, 'cuVSLAM odometry'):
                why = 'cuVSLAM not publishing'
                self.ev('%s - restart tab 1' % why)
                stop_tab(d, 1)
                continue
            # stationary on the ground: must sit near the origin for 8 s
            t0, bad = time.monotonic(), None
            while time.monotonic() - t0 < 8 and not bad:
                bad = self.pos_bad()
                time.sleep(1)
            if bad:
                why = 'cuVSLAM bad init at (%.1f, %.1f, %.1f)' % tuple(bad)
                self.ev('%s - restart tab 1 (%d/%d)' % (why, attempt, tries))
                self.stage('1 cuVSLAM', 'wait', 'bad init - restarting (%d/%d)' % (attempt, tries))
                stop_tab(d, 1)
                continue
            self.ev('camera + cuVSLAM good')
            self.c.clear_stage(d, '1 cuVSLAM')
            return True
        self.stage('1 cuVSLAM', 'bad', '%s (%dx): %s' % (
            why, tries, CAMERA_ADVICE if 'camera' in why else 'check the camera view, then Restart tab 1'))
        return False

    def _run(self):
        d = self.d
        self.ev('bring-up started (flying set: %s)' % ', '.join(self.flying))
        self.stage('Link', 'wait', 'connecting')
        if not self.wait(lambda h, r: True, 30, 'probe'):
            self.stage('Link', 'bad', 'drone unreachable')
            return
        # devices
        self.stage('Devices', 'wait', 'devsync')
        devsync(d)
        # ---- tab 2: PX4 link ----
        for attempt in (1, 2):
            if attempt > 1 or not self.running(2):
                self.stage('2 PX4 link', 'wait', 'starting tab 2 (%d/2)' % attempt)
                start_tab(d, 2, self.flying, self.opts)
            if self.wait(lambda h, r: g(r, 'px4', 'hz', default=0) >= 5, 30, 'PX4 data'):
                self.ev('PX4 link up')
                break
            if attempt == 1:
                self.ev('no PX4 data - restarting tab 2')
                stop_tab(d, 2)
        else:
            self.stage('2 PX4 link', 'bad', 'no data from PX4: is the flight controller powered? '
                                            'If yes, power-cycle it, then Restart tab 2')
            self.ev('no PX4 data after 2 tries - check the flight controller is powered; '
                    'if it is, power-cycle it')
        # ---- tab 1: camera + cuVSLAM, with the bad-init check ----
        self.ensure_tab1(3)
        # ---- tabs 3..7 ----
        for n in (3, 4, 5, 6, 7):
            if self.cancel:
                return
            if not self.running(n):
                self.stage('%d %s' % (n, TAB_NAME[n]), 'wait', 'starting')
                start_tab(d, n, self.flying, self.opts)
                time.sleep(2)
        checks = [(4, lambda h, r: fresh(g(r, 'guard', 'age'), 3), 30, 'guard status'),
                  (5, lambda h, r: fresh(g(r, 'planner', 'age'), 3), 40, 'planner state'),
                  (3, lambda h, r: fresh(g(r, 'fis', 'age'), 10), 60, 'FIS frontiers'),
                  (6, lambda h, r: fresh(g(r, 'shared', 'age'), 10), 90, 'shared map'),
                  (7, lambda h, r: fresh(g(r, 'coord', 'age'), 5), 60, 'coordination')]
        for n, pred, to, what in checks:
            if self.wait(pred, to, what):
                self.c.clear_stage(d, '%d %s' % (n, TAB_NAME[n]))
        others = {FLEET[o][0] for o in self.flying if o != d}
        if others and not self.wait(lambda h, r: others <= set(g(r, 'lora', 'heard', default=[]) or []),
                                    45, 'LoRa hearing %s' % sorted(others)):
            self.ev('LoRa does not hear every flying radio - check the other radios are powered')
        # ---- tab 8: zenoh (multi-drone only) ----
        if len(self.flying) > 1:
            s = self.st() or {}
            if 'fleet_tab8' not in g(s, 'host', 'sessions', default=[]):
                self.stage('8 Zenoh', 'wait', 'starting')
                start_tab(d, 8, self.flying, self.opts)
            if self.wait(lambda h, r: h.get('zenoh') and others <= set(g(r, 'zenoh_peers', default=[]) or []),
                         90, 'zenoh peers'):
                self.c.clear_stage(d, '8 Zenoh')
        # ---- cuVSLAM can go bad after a good start (buckshee on the bench
        # 2026-10-26: passed the 8 s check, then |p| 102 m, slice Z 23.5 m) ----
        if self.pos_bad():
            self.ev('cuVSLAM pose went bad after start-up (%.1f, %.1f, %.1f) - restarting tab 1'
                    % tuple(self.pos_bad()))
            self.ensure_tab1(2, restart_first=True)
        # ---- preflight ----
        self.stage('Preflight', 'wait', 'running')
        res, out = preflight(d)
        self.c.set_extra(d, 'preflight', res)
        self.c.clear_stage(d, 'Preflight')
        self.ev('preflight %s' % res[1])
        # ---- recording (optional) ----
        run = self.opts.get('record_run')
        if run:
            self.c.set_extra(d, 'want_record', True)
            rc, out = local(['./record_fleet.sh', 'start', run, self.opts.get('profile', 'figures'), d],
                            timeout=120)
            self.ev('recording %s: %s' % (run, out.strip().splitlines()[-1] if out.strip() else rc))
        self.ev('bring-up finished')


# ============================================================ the window
class Console:
    def __init__(self, preselect):
        self.root = tk.Tk()
        self.root.title('Fleet Console')
        self.root.geometry('1900x860')
        self.q = queue.Queue()
        self.links = {d: Link(d, self.log) for d in FLEET}
        self.lock = threading.Lock()
        self.stages = {d: {} for d in FLEET}
        self.extra = {d: {} for d in FLEET}
        self.workers = {}
        self.flying = [d for d in FLEET if d in preselect] or list(FLEET)
        self._build()
        for l in self.links.values():
            l.start()
        self.root.protocol('WM_DELETE_WINDOW', self.quit)
        self.root.after(500, self.tick)
        if os.environ.get('FLEET_CONSOLE_AUTOCLOSE'):        # tests: close cleanly after N s
            self.root.after(int(float(os.environ['FLEET_CONSOLE_AUTOCLOSE']) * 1000), self.quit)

    # ---------------------------------------------------------- layout
    def _build(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill='x')
        self.sel = {}
        ttk.Label(top, text='Drones:', font=('', 11, 'bold')).pack(side='left')
        for d in FLEET:
            v = tk.BooleanVar(value=d in self.flying)
            ttk.Checkbutton(top, text=d, variable=v).pack(side='left', padx=3)
            self.sel[d] = v
        ttk.Separator(top, orient='vertical').pack(side='left', fill='y', padx=8)
        ttk.Label(top, text='planner').pack(side='left')
        self.planner = ttk.Combobox(top, values=['simple', 'hercules'], width=9, state='readonly')
        self.planner.set('simple')
        self.planner.pack(side='left', padx=3)
        self.debug = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text='debug (skip arm check)', variable=self.debug).pack(side='left', padx=6)
        ttk.Separator(top, orient='vertical').pack(side='left', fill='y', padx=8)
        self.rec = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text='record run', variable=self.rec).pack(side='left')
        self.run_name = ttk.Entry(top, width=14)
        self.run_name.insert(0, time.strftime('f%m%d_%H%M'))
        self.run_name.pack(side='left', padx=3)
        self.profile = ttk.Combobox(top, values=['figures', 'diagnostic'], width=10, state='readonly')
        self.profile.set('figures')
        self.profile.pack(side='left')
        bar = ttk.Frame(self.root, padding=(6, 0))
        bar.pack(fill='x')
        for text, fn in (('BRING UP', self.bring_up), ('Stop', self.stop), ('Preflight', self.preflight),
                         ('Start recording', self.rec_start), ('Stop recording', self.rec_stop),
                         ('Collect logs', self.collect)):
            b = tk.Button(bar, text=text, command=fn, font=('', 12, 'bold' if text == 'BRING UP' else 'normal'),
                          bg='#2a78d6' if text == 'BRING UP' else None, fg='white' if text == 'BRING UP' else None,
                          padx=12, pady=4)
            b.pack(side='left', padx=4, pady=4)
        self.banner = tk.Label(self.root, text='', font=('', 20, 'bold'), pady=8)
        self.banner.pack(fill='x', padx=6)
        grid = tk.Frame(self.root, padx=6, pady=4)
        grid.pack(fill='x')
        for j in range(len(COLS) + 1):
            grid.columnconfigure(j, weight=1, uniform='cell' if j else '')
        tk.Label(grid, text='', width=12).grid(row=0, column=0)
        for j, c in enumerate(COLS):
            tk.Label(grid, text=c, font=('', 10, 'bold')).grid(row=0, column=j + 1, sticky='nsew')
        self.cells = {}
        for i, d in enumerate(FLEET):
            tk.Label(grid, text='%s\nd%d' % (d, FLEET[d][0]), font=('', 11, 'bold'), width=12) \
                .grid(row=i + 1, column=0, sticky='nsew', pady=2)
            for j, c in enumerate(COLS):
                lab = tk.Label(grid, text='', width=10, height=4, wraplength=118,
                               font=('', 9, 'bold' if c == 'READY' else 'normal'), relief='ridge', bd=1)
                lab.grid(row=i + 1, column=j + 1, sticky='nsew', padx=1, pady=2)
                lab.bind('<Button-1>', lambda e, d=d, c=c: self.cell_menu(e, d, c))
                self.cells[(d, c)] = lab
        self.logbox = ScrolledText(self.root, height=10, font=('monospace', 9))
        self.logbox.pack(fill='both', expand=True, padx=6, pady=6)

    # ---------------------------------------------------------- state
    def log(self, msg):
        self.q.put(time.strftime('%H:%M:%S ') + msg)

    def set_stage(self, d, cell, v):
        with self.lock:
            self.stages[d][cell] = v

    def clear_stage(self, d, cell):
        with self.lock:
            self.stages[d].pop(cell, None)

    def set_extra(self, d, k, v):
        with self.lock:
            self.extra[d][k] = v

    def bringup_done(self, d):
        # keep the red reasons ("power-cycle the FMU") on screen; evaluate()
        # drops an override by itself as soon as the live check turns green
        with self.lock:
            self.stages[d] = {c: v for c, v in self.stages[d].items() if v[0] == 'bad'}
            self.workers.pop(d, None)

    def selected(self):
        return [d for d in FLEET if self.sel[d].get()]

    def opts(self):
        o = dict(planner=self.planner.get(), debug=self.debug.get(), profile=self.profile.get())
        if self.rec.get():
            o['record_run'] = self.run_name.get().strip()
        return o

    # ---------------------------------------------------------- refresh
    def tick(self):
        while not self.q.empty():
            self.logbox.insert('end', self.q.get() + '\n')
            self.logbox.see('end')
        ready, notready = [], []
        for d in FLEET:
            with self.lock:
                extra = dict(self.extra[d], stage=dict(self.stages[d]))
            cells, ok = evaluate(d, self.links[d].status(), self.flying, extra)
            for c in COLS:
                lvl, txt = cells[c]
                self.cells[(d, c)].config(text=txt, bg=COLOR[lvl], fg=FG[lvl])
            if d in self.flying:
                (ready if ok else notready).append((d, cells['READY'][1]))
        if self.flying and not notready:
            self.banner.config(text='READY FOR TAKEOFF:  ' + '   '.join(d for d, _ in ready),
                               bg=COLOR['ok'], fg='white')
        else:
            busy = [d for d in self.flying if d in self.workers]
            txt = ('BRINGING UP: %s   ' % ', '.join(busy) if busy else '') + \
                  ('READY: %s   ' % ', '.join(d for d, _ in ready) if ready else '') + \
                  ('NOT READY: %s' % ', '.join(d for d, _ in notready) if notready else '')
            self.banner.config(text=txt, bg=COLOR['wait'] if busy else '#e6e5e1', fg='black')
        self.root.after(500, self.tick)

    # ---------------------------------------------------------- buttons
    def bring_up(self):
        drones = self.selected()
        if not drones:
            return
        self.flying = drones
        o = self.opts()
        for d in drones:
            with self.lock:
                if d in self.workers:
                    continue
                self.extra[d].pop('preflight', None)
                self.extra[d]['want_record'] = bool(o.get('record_run'))
                w = BringUp(self, d, drones, o)
                self.workers[d] = w
            w.start()

    def _bg(self, fn, *a):
        threading.Thread(target=fn, args=a, daemon=True).start()

    def stop(self):
        drones = self.selected()
        if not drones or not messagebox.askyesno('Stop', 'Stop the stack on %s?' % ', '.join(drones)):
            return
        recording = [d for d in drones if g(self.links[d].status(), 'host', 'recording')]
        stop_rec_first = bool(recording) and messagebox.askyesno(
            'Recording running', 'Recording is running on %s. Stop it first (recommended: '
            'stopping the stack first can leave a bag without metadata)?' % ', '.join(recording))
        for d in drones:
            with self.lock:
                w = self.workers.get(d)
            if w:
                w.cancel = True

        def go():
            if stop_rec_first:          # same thread: recording stops BEFORE the stack
                self.rec_stop(drones=recording, block=True)
            rc, out = local(['./fleet_ctl', 'stop'] + drones, timeout=120)
            self.log('stop: ' + ' | '.join(l for l in out.splitlines() if l.strip()))
            for d in drones:
                self.set_extra(d, 'preflight', None)
        self._bg(go)

    def preflight(self):
        def go(d):
            self.set_extra(d, 'preflight', ('wait', 'running'))
            res, out = preflight(d)
            self.set_extra(d, 'preflight', res)
            self.log('%s: preflight %s' % (d, res[1]))
        for d in self.selected():
            self._bg(go, d)

    def rec_start(self):
        run = self.run_name.get().strip()
        drones = self.selected()
        if not run or not drones:
            return
        for d in drones:
            self.set_extra(d, 'want_record', True)

        def go():
            rc, out = local(['./record_fleet.sh', 'start', run, self.profile.get()] + drones, timeout=180)
            self.log('record start %s: %s' % (run, ' | '.join(l for l in out.splitlines()[-len(drones) - 1:])))
        self._bg(go)

    def rec_stop(self, drones=None, block=False):
        drones = drones or self.selected()
        runs = {g(self.links[d].status(), 'host', 'bag', 'run') for d in drones} - {None}
        run = self.run_name.get().strip() if not runs else sorted(runs)[0]

        def go():
            rc, out = local(['./record_fleet.sh', 'stop', run] + drones, timeout=180)
            self.log('record stop %s: %s' % (run, ' | '.join(out.strip().splitlines()[-len(drones):])))
            for d in drones:
                self.set_extra(d, 'want_record', False)
        if block:
            go()
        else:
            self._bg(go)

    def collect(self):
        drones = self.selected()

        def go():
            rc, out = local(['./fleet_ctl', 'collect'] + drones, timeout=900)
            self.log('collect: ' + ' | '.join(l for l in out.splitlines() if l.strip()))
        self._bg(go)

    # ---------------------------------------------------------- cell menu
    def cell_menu(self, ev, d, c):
        m = tk.Menu(self.root, tearoff=0)
        n = int(c.split()[0]) if c[0].isdigit() else None
        if n:
            m.add_command(label='Restart tab %d on %s' % (n, d), command=lambda: self.restart_tab(d, n))
            m.add_command(label='Show tab %d log' % n, command=lambda: self.show_log(d, n))
            m.add_command(label='Open tab %d terminal' % n, command=lambda: self.open_term(d, n))
        if c == 'Devices':
            m.add_command(label='devsync %s' % d, command=lambda: self._bg(lambda: self.log(
                '%s: devsync %s' % (d, devsync(d)[1].strip().replace('\n', ' ')[-200:] or 'nothing to sync'))))
        if c == 'Preflight':
            m.add_command(label='Run preflight on %s' % d, command=lambda: self._bg(self._pf_one, d))
        m.add_command(label='Bring up %s only' % d, command=lambda: self._bringup_one(d))
        m.tk_popup(ev.x_root, ev.y_root)

    def _pf_one(self, d):
        self.set_extra(d, 'preflight', ('wait', 'running'))
        res, _ = preflight(d)
        self.set_extra(d, 'preflight', res)
        self.log('%s: preflight %s' % (d, res[1]))

    def _bringup_one(self, d):
        with self.lock:
            if d in self.workers:
                return
            w = BringUp(self, d, self.flying if d in self.flying else [d], self.opts())
            self.workers[d] = w
        w.start()

    def restart_tab(self, d, n):
        def go():
            self.log('%s: restarting tab %d' % (d, n))
            stop_tab(d, n)
            if n == 1:
                devsync(d)
            rc, out = start_tab(d, n, self.flying if d in self.flying else [d], self.opts())
            self.log('%s: tab %d %s' % (d, n, 'started' if rc == 0 else 'FAILED: ' + out[-200:]))
        self._bg(go)

    def show_log(self, d, n):
        def go():
            rc, out = ssh(d, "L=$(ls -t %s/experiment_logs/pane_fleet_tab%d_*.log 2>/dev/null | head -1); "
                             "[ -n \"$L\" ] && echo \"$L\" && tail -n 80 \"$L\" | tr -d '\\000' | "
                             "sed 's/\\x1b\\[[0-9;]*[a-zA-Z]//g'" % (WS, n), timeout=30)
            self.root.after(0, lambda: self._text_window('%s tab %d' % (d, n), out))
        self._bg(go)

    def _text_window(self, title, text):
        w = tk.Toplevel(self.root)
        w.title(title)
        t = ScrolledText(w, width=160, height=40, font=('monospace', 9))
        t.pack(fill='both', expand=True)
        t.insert('end', text)
        t.see('end')

    def open_term(self, d, n):
        cmd = '%s/launch_fleet_tmux.sh attach %s fleet_tab%d' % (ROOT, d, n)
        for term in (['gnome-terminal', '--', 'bash', '-c', cmd], ['xterm', '-e', cmd],
                     ['x-terminal-emulator', '-e', cmd]):
            try:
                subprocess.Popen(term, cwd=ROOT)
                return
            except FileNotFoundError:
                continue
        self.log('no terminal emulator found')

    def quit(self):
        for w in list(self.workers.values()):
            w.cancel = True
        for l in self.links.values():
            l.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    Console([a for a in sys.argv[1:] if a in FLEET]).run()
