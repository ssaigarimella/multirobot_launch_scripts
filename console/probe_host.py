#!/usr/bin/env python3
"""probe_host.py - fleet console probe, drone HOST side (stdlib only).

    python3 -u /tmp/fleet_probe_host.py <drone> <vehicle_id>

Starts probe_ros.py inside the container (UDP-only Fast DDS), adds what only
the host can see (tmux sessions, USB devices as the host enumerates them, the
zenoh bridge, the recording, disk) and prints one merged JSON line per second.
When the console's SSH connection goes away the next write fails; it then
stops the in-container probe and exits, so nothing is left behind.
"""
import glob
import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time

CONTAINER = 'isaac_ros_realsense'
WS = '/mnt/nova_ssd/workspaces/isaac_ros-dev'
UDP_XML = """<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor><transport_id>udp</transport_id><type>UDPv4</type></transport_descriptor>
  </transport_descriptors>
  <participant profile_name="udp_only" is_default_profile="true">
    <rtps><userTransports><transport_id>udp</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports></rtps>
  </participant>
</profiles>
"""


def sh(cmd, timeout=5, stdin=None):
    try:
        r = subprocess.run(cmd, shell=isinstance(cmd, str), capture_output=True, text=True,
                           timeout=timeout, input=stdin)
        return r.stdout
    except Exception:
        return ''


def usb_serial():
    """{'/dev/ttyUSBn': 'radio'|'px4'|'other'} by USB VID:PID, as the host sees it."""
    out = {}
    for tty in sorted(glob.glob('/dev/ttyUSB*')):
        d = os.path.realpath('/sys/class/tty/%s/device' % os.path.basename(tty))
        kind = 'other'
        for _ in range(5):
            try:
                vp = (open(d + '/idVendor').read().strip(), open(d + '/idProduct').read().strip())
                kind = {('10c4', 'ea60'): 'radio', ('0403', '6001'): 'px4'}.get(vp, 'other')
                break
            except OSError:
                d = os.path.dirname(d)
        out[tty] = kind
    return out


def realsense_present():
    for f in glob.glob('/sys/bus/usb/devices/*/idVendor'):
        try:
            if open(f).read().strip() == '8086':
                return True
        except OSError:
            pass
    return False


class RosProbe:
    def __init__(self, vid):
        self.vid = vid
        self.proc = None
        self.last = None
        self.last_t = 0.0
        self.started = 0.0
        self.lock = threading.Lock()

    def stop(self):
        sh("docker exec %s pkill -INT -f '[f]leet_probe_ros.py'" % CONTAINER, timeout=5)
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.terminate()
            except Exception:
                pass
        self.proc = None

    def start(self):
        self.stop()
        if CONTAINER not in sh("docker ps --format '{{.Names}}'"):
            return
        sh(['docker', 'exec', '-i', CONTAINER, 'sh', '-c', 'cat > /tmp/fleet_probe_udp.xml'],
           stdin=UDP_XML)
        sh(['docker', 'cp', '/tmp/fleet_probe_ros.py', CONTAINER + ':/tmp/fleet_probe_ros.py'])
        inner = ('source /opt/ros/humble/setup.bash && '
                 'source /workspaces/isaac_ros-dev/install/setup.bash && '
                 'export ROS_DOMAIN_ID=%d ROS_LOCALHOST_ONLY=0 '
                 'FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fleet_probe_udp.xml && '
                 'exec python3 -u /tmp/fleet_probe_ros.py' % self.vid)
        self.proc = subprocess.Popen(['docker', 'exec', '-i', '-u', 'admin', CONTAINER,
                                      'bash', '-c', inner],
                                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                     stdin=subprocess.DEVNULL, text=True)
        self.started = time.monotonic()
        threading.Thread(target=self._read, args=(self.proc,), daemon=True).start()

    def _read(self, proc):
        for line in proc.stdout:
            try:
                obj = json.loads(line)
            except ValueError:
                continue
            with self.lock:
                self.last, self.last_t = obj, time.monotonic()

    def snapshot(self):
        with self.lock:
            age = None if self.last is None else round(time.monotonic() - self.last_t, 1)
            return self.last, age

    def watchdog(self):
        dead = self.proc is None or self.proc.poll() is not None
        stale = (time.monotonic() - max(self.last_t, self.started)) > 15
        if dead or stale:
            self.start()


def bag_state(drone):
    """Newest bag dir for this drone, its size, and whether bagrec runs."""
    dirs = glob.glob('%s/experiment_logs/bags/*/%s' % (WS, drone))
    if not dirs:
        return None
    d = max(dirs, key=os.path.getmtime)
    size = 0
    for root, _, files in os.walk(d):
        for f in files:
            try:
                size += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return dict(run=os.path.basename(os.path.dirname(d)), bytes=size,
                mtime_age=round(time.time() - os.path.getmtime(d), 1))


def main():
    drone, vid = sys.argv[1], int(sys.argv[2])
    probe = RosProbe(vid)
    done = [False]

    def bye(*_):
        done[0] = True
    for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(s, bye)
    probe.start()
    prev_bag = None
    try:
        while not done[0]:
            t0 = time.monotonic()
            probe.watchdog()
            ros, ros_age = probe.snapshot()
            sessions = [s for s in sh("tmux ls -F '#{session_name}' 2>/dev/null").split() if s]
            bag = bag_state(drone)
            if bag and prev_bag and prev_bag['run'] == bag['run']:
                bag['rate'] = round((bag['bytes'] - prev_bag['bytes']) / max(1e-3, t0 - prev_bag['t']))
            if bag:
                bag['t'] = t0
            prev_bag = bag
            du = shutil.disk_usage(WS)
            host = dict(
                container=CONTAINER in sh("docker ps --format '{{.Names}}'"),
                sessions=sessions,
                serial=usb_serial(),
                video=len(glob.glob('/dev/video*')),
                realsense=realsense_present(),
                zenoh=bool(sh("pgrep -f '[z]enoh-bridge-ros2dds'").strip()),
                bag=({k: v for k, v in bag.items() if k != 't'} if bag else None),
                recording='bagrec' in sessions,
                disk_free_gb=round(du.free / 1e9, 1),
                year=time.localtime().tm_year)
            line = json.dumps(dict(drone=drone, vid=vid, host=host, ros=ros, ros_age=ros_age))
            try:
                sys.stdout.write(line + '\n')
                sys.stdout.flush()
            except (BrokenPipeError, OSError):
                break
            time.sleep(max(0.0, 1.0 - (time.monotonic() - t0)))
    finally:
        probe.stop()


if __name__ == '__main__':
    main()
