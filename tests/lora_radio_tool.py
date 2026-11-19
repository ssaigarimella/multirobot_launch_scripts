#!/usr/bin/env python3
"""lora_radio_tool.py - talk to a drone's Heltec radio directly (no ROS).
Runs INSIDE the isaac_ros_realsense container (root; the host users are not
in dialout). Stdlib only. The LoRa bridge must not be running.

    python3 lora_radio_tool.py status [secs]   -> JSON: peers the radio lists
                                                  as alive + poses heard
    python3 lora_radio_tool.py reset           -> reboot the ESP32 into its
                                                  firmware (EN pulse via RTS)

Opening/closing the port does NOT reboot the radio (measured 2026-10-26 on
buckshee: 3 open/close cycles, peers stayed alive, no PEER_JOINED). The reset
here is deliberate: DTR low (GPIO0 high = normal boot), RTS high (EN low),
then RTS low - the sequence esptool uses for --after hard_reset.
"""
import fcntl
import glob
import json
import os
import select
import struct
import sys
import termios
import time


def find_radio():
    """The radio's tty: the CP2102 in hub socket 3, else a CP2102 that is the
    only one and not in socket 2 (the PX4 link, a CP2102 too since 2026-11)."""
    cp = []
    for tty in sorted(glob.glob('/dev/ttyUSB*')):
        d = os.path.realpath('/sys/class/tty/%s/device' % os.path.basename(tty))
        for _ in range(5):
            try:
                vid = open(d + '/idVendor').read().strip()
                pid = open(d + '/idProduct').read().strip()
                if (vid, pid) == ('10c4', 'ea60'):
                    cp.append((tty, os.path.basename(d)))
                break
            except OSError:
                d = os.path.dirname(d)
    for tty, name in cp:
        if name.endswith('.3'):
            return tty
    if len(cp) == 1 and not cp[0][1].endswith('.2'):
        return cp[0][0]
    return None


def open_raw(dev):
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = a[1] = a[3] = 0
    a[2] = ((a[2] & ~termios.CSIZE & ~termios.PARENB & ~termios.CSTOPB & ~termios.CRTSCTS)
            | termios.CS8 | termios.CLOCAL | termios.CREAD)
    a[4] = a[5] = termios.B115200
    a[6][termios.VMIN] = 0
    a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd


def frames(buf):
    i = 0
    while i + 3 < len(buf):
        if buf[i] == 0xAA:
            t, n = buf[i + 1], buf[i + 2]
            if i + 4 + n <= len(buf):
                cs = t ^ n
                for c in buf[i + 3:i + 3 + n]:
                    cs ^= c
                if cs == buf[i + 3 + n]:
                    yield t, buf[i + 3:i + 3 + n]
                    i += 4 + n
                    continue
        i += 1


def status(dev, secs):
    fd = open_raw(dev)
    buf = b''
    t0 = time.monotonic()
    sent = False
    while time.monotonic() - t0 < secs:
        if not sent and time.monotonic() - t0 > secs - 0.4:
            os.write(fd, bytes([0xAA, 0x03, 0x00, 0x03]))   # STATUS_REQ near the end
            sent = True
        if select.select([fd], [], [], 0.05)[0]:
            try:
                buf += os.read(fd, 4096)
            except BlockingIOError:
                pass
    os.close(fd)
    out = {'node': None, 'alive': [], 'poses': {}, 'joined': []}
    for t, p in frames(buf):
        if t == 0x83 and len(p) >= 2:
            out['node'] = p[0]
            out['alive'] = sorted(p[2 + 7 * k] for k in range(p[1]) if 2 + 7 * k < len(p))
        elif t == 0x81 and p:
            out['poses'][str(p[0])] = out['poses'].get(str(p[0]), 0) + 1
        elif t == 0x85 and p:
            out['joined'].append(p[0])
    return out


def reset(dev):
    fd = open_raw(dev)
    dtr = struct.pack('I', termios.TIOCM_DTR)
    rts = struct.pack('I', termios.TIOCM_RTS)
    fcntl.ioctl(fd, termios.TIOCMBIC, dtr)    # GPIO0 high: boot the firmware
    fcntl.ioctl(fd, termios.TIOCMBIS, rts)    # EN low: hold in reset
    time.sleep(0.15)
    fcntl.ioctl(fd, termios.TIOCMBIC, rts)    # EN high: run
    os.close(fd)


def main():
    dev = find_radio()
    if not dev:
        print(json.dumps({'error': 'no CP2102 radio'}))
        sys.exit(1)
    for fd in glob.glob('/proc/[0-9]*/fd/*'):
        try:
            if os.readlink(fd) == dev:
                print(json.dumps({'error': 'port held by pid ' + fd.split('/')[2]}))
                sys.exit(2)
        except OSError:
            pass
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'status'
    if cmd == 'reset':
        reset(dev)
        print(json.dumps({'reset': dev}))
    else:
        r = status(dev, float(sys.argv[2]) if len(sys.argv) > 2 else 1.5)
        r['port'] = dev
        print(json.dumps(r))


if __name__ == '__main__':
    main()
