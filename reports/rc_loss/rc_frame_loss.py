#!/usr/bin/env python3
"""rc_frame_loss.py - manual-control link frame loss extractor for PX4 ULogs.

Computes SBUS frame loss from the input_rc.rc_total_frame_count /
rc_lost_frame_count hardware counters, correcting for uint16 wraparound and
splitting on autopilot reboot.

Parsing: uses pyulog when importable, otherwise a self-contained minimal ULog
reader. Nothing is installed; no numpy/scipy/CUDA dependency.

Usage:
    rc_frame_loss.py [--csv OUT.csv] [--parser auto|pyulog|fallback] LOG.ulg [LOG.ulg ...]
"""

import argparse
import csv
import os
import struct
import sys

# ---------------------------------------------------------------- constants

U16 = 65536                # both counters are uint16_t (requirement C1)
MAX_PLAUSIBLE_RATE_HZ = 500.0   # generous ceiling; real SBUS is 50-150 Hz
RATE_MARGIN_FRAMES = 100        # absolute slack for jitter / logger dropout

CSV_COLUMNS = [
    'log_path', 'segment_index', 't_start_s', 't_end_s', 'duration_s',
    'n_samples', 'sample_rate_hz', 'frames_total', 'frames_lost',
    'loss_ratio_ppm', 'wraps_detected', 'sbus_frame_rate_hz', 'rc_lost_any',
    'rc_failsafe_any', 'arming_states_seen', 'detection_floor_ppm',
]

# ------------------------------------------------------- minimal ULog reader

_TYPE_FMT = {
    'int8_t': 'b', 'uint8_t': 'B', 'int16_t': 'h', 'uint16_t': 'H',
    'int32_t': 'i', 'uint32_t': 'I', 'int64_t': 'q', 'uint64_t': 'Q',
    'float': 'f', 'double': 'd', 'bool': 'B', 'char': 'c',
}

_MSG_FORMAT = ord('F')
_MSG_ADD_LOGGED = ord('A')
_MSG_DATA = ord('D')


def _parse_format(fmt_str):
    """'name:type field;...' -> (name, [(type, field, count), ...])."""
    name, _, fields_str = fmt_str.partition(':')
    fields = []
    for decl in fields_str.split(';'):
        decl = decl.strip()
        if not decl:
            continue
        ftype, _, fname = decl.partition(' ')
        count = 1
        if '[' in ftype:
            ftype, _, rest = ftype.partition('[')
            count = int(rest.rstrip(']'))
        fields.append((ftype, fname, count))
    return name, fields


def _strip_trailing_padding(fields):
    """Trailing _padding fields are NOT written to the data record.

    The input_rc record is 71 bytes, one byte shorter than the format string
    implies, because of exactly this. Strip them before building the struct.
    """
    out = list(fields)
    while out and out[-1][1].startswith('_padding'):
        out.pop()
    return out


def _build_struct(fields):
    """(struct.Struct, [field names in order]) for primitive-only messages."""
    fmt = '<'
    names = []
    for ftype, fname, count in fields:
        if ftype not in _TYPE_FMT:
            raise ValueError('non-primitive field type %r' % ftype)
        fmt += _TYPE_FMT[ftype] * count
        if count == 1:
            names.append(fname)
        else:
            names.extend('%s[%d]' % (fname, i) for i in range(count))
    return struct.Struct(fmt), names


def fallback_read_topics(path, wanted):
    """Minimal ULog reader. Returns {topic: {field: [values]}} for `wanted`."""
    with open(path, 'rb') as fh:
        blob = fh.read()

    if blob[:7] != b'ULog\x01\x12\x35':
        raise ValueError('not a ULog file: %s' % path)
    off = 16  # 8 byte magic+version, 8 byte timestamp

    formats = {}        # topic name -> [(type, name, count), ...]
    subs = {}           # msg_id -> (topic, struct, names)
    out = {t: None for t in wanted}
    n = len(blob)

    while off + 3 <= n:
        try:
            msg_size, msg_type = struct.unpack_from('<HB', blob, off)
        except struct.error:
            break
        body = off + 3
        end = body + msg_size
        if end > n:
            break

        if msg_type == _MSG_FORMAT:
            try:
                name, fields = _parse_format(
                    blob[body:end].decode('utf-8', 'replace'))
                formats[name] = fields
            except Exception:
                pass

        elif msg_type == _MSG_ADD_LOGGED:
            # uint8 multi_id, uint16 msg_id, char[] message_name
            multi_id, msg_id = struct.unpack_from('<BH', blob, body)
            topic = blob[body + 3:end].decode('utf-8', 'replace')
            if topic in wanted and topic in formats:
                fields = _strip_trailing_padding(formats[topic])
                try:
                    st, names = _build_struct(fields)
                except ValueError:
                    off = end
                    continue
                subs[msg_id] = (topic, st, names, multi_id)
                if out[topic] is None:
                    out[topic] = {nm: [] for nm in names}

        elif msg_type == _MSG_DATA:
            msg_id = struct.unpack_from('<H', blob, body)[0]
            sub = subs.get(msg_id)
            if sub is not None:
                topic, st, names, multi_id = sub
                if multi_id == 0:                      # first instance only
                    payload = blob[body + 2:end]
                    if len(payload) >= st.size:
                        vals = st.unpack_from(payload, 0)
                        store = out[topic]
                        for nm, v in zip(names, vals):
                            store[nm].append(v)

        off = end

    return {t: v for t, v in out.items() if v}


def pyulog_read_topics(path, wanted):
    from pyulog import ULog
    ulog = ULog(path, message_name_filter_list=list(wanted))
    out = {}
    for d in ulog.data_list:
        if d.name in wanted and d.multi_id == 0:
            out[d.name] = {k: list(v) for k, v in d.data.items()}
    return out


def read_topics(path, wanted, parser='auto'):
    """Returns (data, parser_used)."""
    if parser in ('auto', 'pyulog'):
        try:
            return pyulog_read_topics(path, wanted), 'pyulog'
        except ImportError:
            if parser == 'pyulog':
                raise
        except Exception:
            if parser == 'pyulog':
                raise
    return fallback_read_topics(path, wanted), 'fallback'


# ------------------------------------------------------- counter arithmetic

def _corrected_delta(prev, cur, dt_s):
    """(delta, kind) where kind in {'normal', 'wrap', 'reset'}.

    C1: a negative step is a uint16 wrap -> add 65536, but only when the
    corrected value is physically reachable in dt_s. C2: otherwise it is a
    counter reset (autopilot reboot) and the caller must start a new segment.
    """
    allowed = dt_s * MAX_PLAUSIBLE_RATE_HZ + RATE_MARGIN_FRAMES
    raw = cur - prev
    if raw >= 0:
        if raw > allowed:
            return None, 'reset'      # implausible forward jump == discontinuity
        return raw, 'normal'
    corrected = raw + U16
    if corrected <= allowed:
        return corrected, 'wrap'
    return None, 'reset'


def _arming_lookup(data):
    """Returns fn(t_us) -> 'armed'/'disarmed'/None, from actuator_armed."""
    src = data.get('actuator_armed')
    if src and 'armed' in src and 'timestamp' in src:
        ts = [int(v) for v in src['timestamp']]
        vals = ['armed' if bool(v) else 'disarmed' for v in src['armed']]
    else:
        src = data.get('vehicle_status')
        if not src or 'arming_state' not in src:
            return lambda t: None
        ts = [int(v) for v in src['timestamp']]
        # PX4 v1.14+: 1 = DISARMED, 2 = ARMED
        vals = ['armed' if v == 2 else 'disarmed' for v in src['arming_state']]
    if not ts:
        return lambda t: None

    def lookup(t_us):
        lo, hi = 0, len(ts) - 1
        if t_us < ts[0]:
            return vals[0]
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if ts[mid] <= t_us:
                lo = mid
            else:
                hi = mid - 1
        return vals[lo]

    return lookup


def analyse(path, parser='auto'):
    """Returns (rows, meta). One row per segment."""
    wanted = {'input_rc', 'actuator_armed', 'vehicle_status'}
    data, parser_used = read_topics(path, wanted, parser)

    rc = data.get('input_rc')
    if not rc or 'rc_total_frame_count' not in rc:
        return [], {'parser': parser_used, 'error': 'no input_rc data'}

    # Cast to Python int: pyulog returns numpy uint16 arrays, and numpy
    # unsigned subtraction wraps silently instead of going negative, which
    # would hide every counter wrap and defeat requirement C1.
    ts = [int(v) for v in rc['timestamp']]
    tot = [int(v) for v in rc['rc_total_frame_count']]
    lost = ([int(v) for v in rc['rc_lost_frame_count']]
            if 'rc_lost_frame_count' in rc else [0] * len(ts))
    rc_lost = rc.get('rc_lost', [0] * len(ts))
    rc_fs = rc.get('rc_failsafe', [0] * len(ts))
    arming_at = _arming_lookup(data)

    # ---- walk samples, accumulating into segments -----------------------
    segments = []
    cur = None

    def _open(i):
        return {
            'i_start': i, 'i_end': i,
            't_start': ts[i], 't_end': ts[i],
            'n_samples': 1, 'frames_total': 0, 'frames_lost': 0,
            'wraps': 0, 'lost_wraps': 0,
            'rc_lost_any': bool(rc_lost[i]), 'rc_fs_any': bool(rc_fs[i]),
            'arming': set(filter(None, [arming_at(ts[i])])),
        }

    for i in range(len(ts)):
        if cur is None:
            cur = _open(i)
            continue
        dt_s = max((ts[i] - ts[i - 1]) / 1e6, 0.0)
        d_tot, kind = _corrected_delta(tot[i - 1], tot[i], dt_s)
        if kind == 'reset':
            segments.append(cur)          # C2: never bridge across a reset
            cur = _open(i)
            continue
        d_lost, lkind = _corrected_delta(lost[i - 1], lost[i], dt_s)
        if lkind == 'reset':
            segments.append(cur)
            cur = _open(i)
            continue

        cur['frames_total'] += d_tot
        cur['frames_lost'] += d_lost
        cur['wraps'] += 1 if kind == 'wrap' else 0
        cur['lost_wraps'] += 1 if lkind == 'wrap' else 0
        cur['n_samples'] += 1
        cur['i_end'] = i
        cur['t_end'] = ts[i]
        cur['rc_lost_any'] = cur['rc_lost_any'] or bool(rc_lost[i])
        cur['rc_fs_any'] = cur['rc_fs_any'] or bool(rc_fs[i])
        a = arming_at(ts[i])
        if a:
            cur['arming'].add(a)

    if cur is not None:
        segments.append(cur)

    rows = []
    for idx, s in enumerate(segments):
        dur = (s['t_end'] - s['t_start']) / 1e6
        ft = s['frames_total']
        fl = s['frames_lost']
        rows.append({
            'log_path': path,
            'segment_index': idx,
            't_start_s': round(s['t_start'] / 1e6, 3),
            't_end_s': round(s['t_end'] / 1e6, 3),
            'duration_s': round(dur, 3),
            'n_samples': s['n_samples'],
            'sample_rate_hz': round((s['n_samples'] - 1) / dur, 3) if dur > 0 else '',
            'frames_total': ft,
            'frames_lost': fl,
            # C3: ratio of summed deltas, never a mean of per-sample ratios
            'loss_ratio_ppm': round(fl / ft * 1e6, 3) if ft > 0 else '',
            'wraps_detected': s['wraps'] + s['lost_wraps'],
            'sbus_frame_rate_hz': round(ft / dur, 3) if dur > 0 else '',
            'rc_lost_any': int(s['rc_lost_any']),
            'rc_failsafe_any': int(s['rc_fs_any']),
            'arming_states_seen': '|'.join(sorted(s['arming'])) or 'unknown',
            # C4: smallest loss this segment could have resolved
            'detection_floor_ppm': round(1.0 / ft * 1e6, 3) if ft > 0 else '',
        })
    return rows, {'parser': parser_used, 'n_segments': len(segments)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--csv', default=None, help='write CSV here (default: stdout)')
    ap.add_argument('--parser', choices=['auto', 'pyulog', 'fallback'],
                    default='auto')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args(argv)

    all_rows = []
    for path in args.logs:
        try:
            rows, meta = analyse(path, args.parser)
        except Exception as exc:
            if not args.quiet:
                print('  !! %s: %s: %s' % (path, type(exc).__name__, exc),
                      file=sys.stderr)
            continue
        if not rows and not args.quiet:
            print('  -- %s: %s' % (path, meta.get('error', 'no segments')),
                  file=sys.stderr)
        all_rows.extend(rows)

    out = open(args.csv, 'w', newline='') if args.csv else sys.stdout
    try:
        w = csv.DictWriter(out, fieldnames=CSV_COLUMNS)
        w.writeheader()
        for r in all_rows:
            w.writerow(r)
    finally:
        if args.csv:
            out.close()
    if args.csv and not args.quiet:
        print('wrote %d rows -> %s' % (len(all_rows), args.csv), file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
