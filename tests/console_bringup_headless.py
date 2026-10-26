#!/usr/bin/env python3
"""console_bringup_headless.py - run the fleet console's BRING UP without a
window: same probe links, same pipeline, same READY logic, printed as text.
Useful as an end-to-end test and as a fallback in the field.

    ./tests/console_bringup_headless.py ghost            # solo
    ./tests/console_bringup_headless.py delta buckshee --record f1027_a
"""
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'console'))
import fleet_console as fc  # noqa: E402


class Headless:
    def __init__(self, drones, opts):
        self.flying = drones
        self.lock = threading.Lock()
        self.stages = {d: {} for d in fc.FLEET}
        self.extra = {d: {} for d in fc.FLEET}
        self.workers = {}
        self.links = {d: fc.Link(d, self.log) for d in drones}
        self.opts = opts

    def log(self, msg):
        print(time.strftime('%H:%M:%S ') + msg, flush=True)

    def set_stage(self, d, c, v):
        with self.lock:
            self.stages[d][c] = v

    def clear_stage(self, d, c):
        with self.lock:
            self.stages[d].pop(c, None)

    def set_extra(self, d, k, v):
        with self.lock:
            self.extra[d][k] = v

    def bringup_done(self, d):
        with self.lock:
            self.stages[d] = {c: v for c, v in self.stages[d].items() if v[0] == 'bad'}
            self.workers.pop(d, None)

    def table(self):
        for d in self.flying:
            with self.lock:
                extra = dict(self.extra[d], stage=dict(self.stages[d]))
            cells, ready = fc.evaluate(d, self.links[d].status(), self.flying, extra)
            print('  %-13s %s' % (d, 'READY FOR TAKEOFF' if ready else cells['READY'][1]))
            for c in fc.COLS[:-1]:
                lvl, txt = cells[c]
                print('      %-14s %-5s %s' % (c, lvl.upper(), txt))
        print(flush=True)

    def run(self):
        for l in self.links.values():
            l.start()
        for d in self.flying:
            if self.opts.get('record_run'):
                self.extra[d]['want_record'] = True
            w = fc.BringUp(self, d, self.flying, self.opts)
            self.workers[d] = w
            w.start()
        t0 = time.monotonic()
        while self.workers:
            time.sleep(1)
            if time.monotonic() - t0 > 900:
                self.log('giving up after 15 min')
                break
        self.table()
        for l in self.links.values():
            l.stop()


def main():
    drones = [a for a in sys.argv[1:] if a in fc.FLEET]
    opts = dict(planner='simple', debug=False, profile='figures')
    if '--record' in sys.argv:
        opts['record_run'] = sys.argv[sys.argv.index('--record') + 1]
    if not drones:
        sys.exit(__doc__)
    Headless(drones, opts).run()


if __name__ == '__main__':
    main()
