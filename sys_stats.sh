#!/bin/bash
# sys_stats.sh - 1 Hz system-health log on each drone, for explaining stalls.
#
# Runs ALONGSIDE the stack. Pure read: it changes nothing on the vehicle.
#
#   ./sys_stats.sh start  ghost delta      # tmux 'fleet_sys' on each drone
#   ./sys_stats.sh status ghost delta
#   ./sys_stats.sh stop   ghost delta
#
# Why: on 2026-11-13 delta's planner and loggers paused for up to 13 s in
# flight while the camera, mapper and PX4 feed ran on, and nothing on board
# recorded CPU, memory or disk state, so the cause could not be established.
# Ghost's PX4 serial adapter also dropped off USB in flight twice; the user's
# decision is to LOG that, not to work around it.
#
# Output on the drone (picked up by ./fleet_ctl collect):
#   experiment_logs/sys_<host>_<stamp>.csv         one row per second
#   experiment_logs/sys_events_<host>_<stamp>.log  kernel USB / tty add and remove events (udev)
#
# CSV columns:
#   t_utc,t_unix,host,loop_dt_s     loop_dt_s well above 1 means THIS logger was stalled
#   load1,cpu_busy_pct,cpu_user_pct,cpu_sys_pct,cpu_iowait_pct,cpu_max_core_pct
#   mem_avail_mb,swap_used_mb,dirty_mb,writeback_mb
#   psi_cpu_some10,psi_io_some10,psi_io_full10,psi_mem_some10   pressure stall, % of the last 10 s (empty: kernel has no PSI)
#   disk,rd_kb_s,wr_kb_s,disk_util_pct,wr_await_ms              the disk under the workspace
#   temp_max_c,gpu_load_pct,n_ttyusb,ttyusb                     serial adapters present (PX4 link, LoRa)
#   top                                                         three busiest processes, every 5th row
#
# The session is named 'fleet_sys' so that `fleet_ctl stop` (which sweeps
# ^fleet) ends it with the stack.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/credentials.env" 2>/dev/null
: "${JETSON_PASS:?credentials.env missing - copy credentials.env.example and fill it in}"

WS=/mnt/nova_ssd/workspaces/isaac_ros-dev
LOGDIR="$WS/experiment_logs"
SESS="${SESS:-fleet_sys}"

declare -A IPS=( [delta]=192.168.0.40 [buckshee]=192.168.0.60 [ghost]=192.168.0.50 [thunderstrike]=192.168.0.70 )
# per-run address override, e.g. FLEET_IP_buckshee=192.168.0.125
for d in "${!IPS[@]}"; do v="FLEET_IP_$d"; [ -n "${!v:-}" ] && IPS[$d]="${!v}"; done

ssh_d(){ local d="$1"; shift
  sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o LogLevel=ERROR \
    -o ConnectTimeout=8 "$d@${IPS[$d]}" "$@"; }

# ---- the sampler that runs on the drone (python3, standard library only) ----
remote_sampler(){ cat <<'EOS'
#!/usr/bin/env python3
import glob, os, socket, subprocess, sys, time

logdir, ws = sys.argv[1], sys.argv[2]
host = socket.gethostname()
stamp = time.strftime('%Y%m%d_%H%M%S', time.gmtime())
path = os.path.join(logdir, 'sys_%s_%s.csv' % (host, stamp))


def rd(p):
    try:
        with open(p) as f:
            return f.read()
    except OSError:
        return ''


def cpu():
    out = {}
    for line in rd('/proc/stat').splitlines():
        if line.startswith('cpu'):
            v = line.split()
            out[v[0]] = [int(x) for x in v[1:9]]     # user nice system idle iowait irq softirq steal
    return out


def psi(kind, key):
    for line in rd('/proc/pressure/' + kind).splitlines():
        if line.startswith(key):
            for tok in line.split():
                if tok.startswith('avg10='):
                    return tok[6:]
    return ''


def disk_of(p):
    """Block device (e.g. nvme0n1) that holds path p, from /proc/self/mountinfo."""
    best, dev = '', ''
    try:
        rp = os.path.realpath(p)
        for line in rd('/proc/self/mountinfo').splitlines():
            f = line.split()
            mp, src = f[4], f[f.index('-') + 2]
            if rp.startswith(mp) and len(mp) > len(best) and src.startswith('/dev/'):
                best, dev = mp, os.path.basename(src)
    except (ValueError, IndexError):
        pass
    for base in sorted(os.listdir('/sys/block'), key=len, reverse=True):
        if dev.startswith(base):
            return base
    return dev


def diskstats(dev):
    for line in rd('/proc/diskstats').splitlines():
        f = line.split()
        if len(f) > 13 and f[2] == dev:
            # sectors read, sectors written, ms writing, writes completed, ms doing io
            return int(f[5]), int(f[9]), int(f[10]), int(f[7]), int(f[12])
    return None


def meminfo():
    m = {}
    for line in rd('/proc/meminfo').splitlines():
        k, _, v = line.partition(':')
        m[k] = int(v.split()[0]) if v.split() else 0
    return m


GPU = [p for p in ('/sys/devices/platform/gpu.0/load', '/sys/devices/gpu.0/load',
                   '/sys/devices/platform/17000000.gpu/load', '/sys/devices/17000000.gpu/load') if os.path.exists(p)]
dev = disk_of(ws)
cols = ('t_utc,t_unix,host,loop_dt_s,load1,cpu_busy_pct,cpu_user_pct,cpu_sys_pct,cpu_iowait_pct,cpu_max_core_pct,'
        'mem_avail_mb,swap_used_mb,dirty_mb,writeback_mb,psi_cpu_some10,psi_io_some10,psi_io_full10,psi_mem_some10,'
        'disk,rd_kb_s,wr_kb_s,disk_util_pct,wr_await_ms,temp_max_c,gpu_load_pct,n_ttyusb,ttyusb,top')
f = open(path, 'w', buffering=1)
f.write(cols + '\n')
print('sys sampler -> %s (disk %s)' % (path, dev), flush=True)
c0, d0, t0, n = cpu(), diskstats(dev), time.time(), 0
while True:
    time.sleep(max(0.0, 1.0 - (time.time() - t0) % 1.0) if n else 1.0)
    t = time.time()
    dt = t - t0
    c1, d1 = cpu(), diskstats(dev)
    busy = user = system = iow = mx = 0.0
    for k in c1:
        if k not in c0:
            continue
        a, b = c0[k], c1[k]
        tot = sum(b) - sum(a)
        if tot <= 0:
            continue
        idle = (b[3] - a[3]) + (b[4] - a[4])
        if k == 'cpu':
            busy = 100.0 * (tot - idle) / tot
            user = 100.0 * ((b[0] - a[0]) + (b[1] - a[1])) / tot
            system = 100.0 * (b[2] - a[2]) / tot
            iow = 100.0 * (b[4] - a[4]) / tot
        else:
            mx = max(mx, 100.0 * (tot - idle) / tot)
    rdk = wrk = util = await_ = ''
    if d0 and d1 and dt > 0:
        rdk = '%.0f' % ((d1[0] - d0[0]) * 0.5 / dt)
        wrk = '%.0f' % ((d1[1] - d0[1]) * 0.5 / dt)
        util = '%.0f' % min(100.0, (d1[4] - d0[4]) / (10.0 * dt))
        nw = d1[3] - d0[3]
        await_ = '%.1f' % ((d1[2] - d0[2]) / nw) if nw > 0 else ''
    m = meminfo()
    temps = []
    for z in glob.glob('/sys/devices/virtual/thermal/thermal_zone*/temp'):
        try:
            temps.append(int(rd(z)) / 1000.0)
        except ValueError:
            pass
    gpu = ''
    if GPU:
        try:
            gpu = '%.0f' % (int(rd(GPU[0])) / 10.0)
        except ValueError:
            pass
    tty = sorted(os.path.basename(p) for p in glob.glob('/dev/ttyUSB*'))
    top = ''
    if n % 5 == 0:
        try:
            out = subprocess.run(['ps', '-eo', 'pcpu,comm', '--sort=-pcpu', '--no-headers'], capture_output=True,
                                 text=True, timeout=2).stdout.splitlines()[:3]
            top = ';'.join('%s:%s' % (l.split(None, 1)[1].strip().replace(',', ' '), l.split()[0]) for l in out if l.strip())
        except (OSError, subprocess.SubprocessError, IndexError):
            pass
    f.write(','.join(str(x) for x in (
        time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime(t)) + '.%03dZ' % int((t % 1) * 1000), '%.3f' % t, host,
        '%.3f' % dt, rd('/proc/loadavg').split()[0] if rd('/proc/loadavg') else '',
        '%.1f' % busy, '%.1f' % user, '%.1f' % system, '%.1f' % iow, '%.1f' % mx,
        m.get('MemAvailable', 0) // 1024, (m.get('SwapTotal', 0) - m.get('SwapFree', 0)) // 1024,
        '%.1f' % (m.get('Dirty', 0) / 1024.0), '%.1f' % (m.get('Writeback', 0) / 1024.0),
        psi('cpu', 'some'), psi('io', 'some'), psi('io', 'full'), psi('memory', 'some'),
        dev, rdk, wrk, util, await_, '%.1f' % max(temps) if temps else '', gpu, len(tty), ' '.join(tty), top)) + '\n')
    c0, d0, t0, n = c1, d1, t, n + 1
EOS
}

cmd_start(){
  for d in "$@"; do
    [ -n "${IPS[$d]:-}" ] || { echo "  $d: unknown drone"; continue; }
    ssh_d "$d" "mkdir -p $LOGDIR && cat > /tmp/sys_sampler.py" < <(remote_sampler)
    ssh_d "$d" "
      tmux has-session -t $SESS 2>/dev/null && { echo '  $d: already running'; exit 0; }
      tmux new-session -d -s $SESS \"python3 -u /tmp/sys_sampler.py $LOGDIR $WS 2>&1 | tee -a $LOGDIR/sys_stats_\$(hostname).log\"
      tmux new-window -d -t $SESS \"stdbuf -oL udevadm monitor --kernel --subsystem-match=usb --subsystem-match=tty --subsystem-match=usb-serial 2>&1 | while IFS= read -r l; do echo \\\"\\\$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ) \\\$l\\\"; done >> $LOGDIR/sys_events_\$(hostname)_\$(date -u +%Y%m%d_%H%M%S).log\"
      sleep 1.5; tmux has-session -t $SESS 2>/dev/null && echo '  $d: system logger started' || echo '  $d: START FAILED'"
  done
}

cmd_stop(){
  for d in "$@"; do
    [ -n "${IPS[$d]:-}" ] || continue
    ssh_d "$d" "tmux kill-session -t $SESS 2>/dev/null; echo '  $d: stopped'"
  done
}

cmd_status(){
  printf '%-14s %-8s %-34s %-7s %s\n' drone logger csv rows last_row
  for d in "$@"; do
    [ -n "${IPS[$d]:-}" ] || continue
    ssh_d "$d" "
      R=\$(tmux has-session -t $SESS 2>/dev/null && echo running || echo -)
      F=\$(ls -t $LOGDIR/sys_\$(hostname)_*.csv 2>/dev/null | head -1)
      N=\$([ -n \"\$F\" ] && echo \$(( \$(wc -l < \"\$F\") - 1 )) || echo 0)
      L=\$([ -n \"\$F\" ] && tail -1 \"\$F\" | cut -d, -f1,4-7,11,21-22 || echo -)
      printf '%-14s %-8s %-34s %-7s %s\n' '$d' \"\$R\" \"\$(basename \${F:-none})\" \"\$N\" \"\$L\"" 2>/dev/null
  done
}

CMD="${1:-}"; shift 2>/dev/null
DRONES=("$@"); [ ${#DRONES[@]} -eq 0 ] && DRONES=(ghost delta)
case "$CMD" in
  start)  cmd_start  "${DRONES[@]}" ;;
  stop)   cmd_stop   "${DRONES[@]}" ;;
  status) cmd_status "${DRONES[@]}" ;;
  *) sed -n '2,30p' "$0"; exit 2 ;;
esac
