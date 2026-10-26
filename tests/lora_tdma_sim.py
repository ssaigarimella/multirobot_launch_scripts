#!/usr/bin/env python3
"""lora_tdma_sim.py - event simulation of the radiohive TDMA mesh sync rules,
v7.3 vs v7.4, for the failure seen on 2026-10-23 (node 0 rebooted at a
random phase, its slot over a peer's, both deaf forever).

    python3 tests/lora_tdma_sim.py [--trials 1000] [--nodes 2 4]

Model (mirrors lora_mesh_v7_2_ros2.ino): the cycle is sized by the compiled
NUM_NODES (4 slots, 190 ms) however many radios are powered; a node
transmits once per cycle at slot_start + TX_MARGIN, packets take AIRTIME ms,
half-duplex (a node cannot receive while transmitting), any two overlapping
packets at a receiver are both lost (no capture effect), clocks are ideal.
handleRx / shouldSyncTo / findNewSyncSource / the 3 s peer timeout, the
neighbor_mask every packet carries, and the v7.4 rules (re-roll when no peer
has listed us in its neighbor_mask for 1.5 s; relayed sync when our authority
is not heard) are reproduced from the firmware. Each trial: the mesh is
synced, node 0 reboots at a random phase at t=0, run 60 s; the pair (0,1) is
"broken" if any radio has not heard any other radio in the last 20 s, and
recovery is the time until every ordered pair hears each other steadily.
"""
import argparse
import heapq
import random

SLOT, GUARD, MARGIN, AIRTIME = 40, 30, 5, 20
NUM_NODES = 4                     # compiled into every radio; sizes the cycle
TIMEOUT, DEAF = 3000, 1500


class Node:
    def __init__(self, nid, n, epoch, v74, rng):
        self.id, self.n, self.v74, self.rng = nid, n, v74, rng
        self.cycle = NUM_NODES * SLOT + GUARD
        self.epoch = epoch
        self.sync = nid
        self.alive = [False] * n
        self.last_seen = [0] * n
        self.peer_sync = list(range(n))
        self.last_rx_any = 0
        self.last_acked = 0   # last packet whose neighbor_mask listed us
        self.last_reroll = 0
        self.gen = 0          # invalidates queued TX events after an epoch change
        self.heard = {}       # sender -> last time heard

    def next_tx(self, after):
        base = self.epoch + self.id * SLOT + MARGIN
        k = (after - base) // self.cycle + 1
        return base + k * self.cycle

    def should_sync(self, sid, ssync):
        auth = min(sid, ssync)
        if auth < self.sync:
            return True
        if auth == self.sync and sid <= self.sync:
            return True
        if (self.v74 and auth == self.sync and self.sync != self.id
                and not self.alive[self.sync]):
            return True
        return False

    def find_new_sync(self):
        best = self.id
        for i in range(self.n):
            if i == self.id or not self.alive[i]:
                continue
            best = min(best, self.peer_sync[i], i)
        self.sync = best

    def timeouts(self, now):
        for i in range(self.n):
            if i != self.id and self.alive[i] and now - self.last_seen[i] > TIMEOUT:
                self.alive[i] = False
                if self.sync == i:
                    self.find_new_sync()

    def mask(self):
        return sum(1 << i for i in range(self.n) if self.alive[i])

    def deaf_check(self, now):
        # unheard (no peer lists us) covers deaf (we hear no one) too
        if self.v74 and now - self.last_acked > DEAF and now - self.last_reroll > DEAF:
            self.last_reroll = now
            self.epoch += SLOT + self.rng.randrange(self.cycle - 2 * SLOT)
            self.sync = self.id
            return True
        return False


def trial(n, v74, rng, horizon=60000, scenario='reboot0'):
    cyc = NUM_NODES * SLOT + GUARD
    base = rng.randrange(cyc)
    nodes = [Node(i, n, base, v74, rng) for i in range(n)]
    for nd in nodes:                      # synced, everyone alive, authority 0
        nd.sync = 0
        nd.alive = [True] * n
        nd.last_seen = [0] * n
        nd.peer_sync = [0] * n
        nd.last_rx_any = 0
        nd.last_acked = 0
    # t=0: one radio reboots (fresh state, random phase), or every radio
    # powers up at its own random phase (field-day cold start)
    fresh = {'reboot0': [0], 'reboot2': [2 % n], 'cold': list(range(n))}[scenario]
    for i in fresh:
        nodes[i] = Node(i, n, rng.randrange(cyc), v74, rng)
        nodes[i].alive[i] = True
    q, txlog = [], []
    for nd in nodes:
        heapq.heappush(q, (nd.next_tx(-1), 0, nd.id, nd.gen))
    while q:
        t, kind, nid, info = heapq.heappop(q)
        if t > horizon:
            break
        nd = nodes[nid]
        if kind == 0:                     # TX start
            if info != nd.gen:
                continue
            nd.timeouts(t)
            if nd.deaf_check(t):
                nd.gen += 1
                heapq.heappush(q, (nd.next_tx(t), 0, nid, nd.gen))
                continue
            txlog.append((t, t + AIRTIME, nid, nd.sync, t - nd.epoch, nd.mask()))
            heapq.heappush(q, (t + AIRTIME, 1, nid, len(txlog) - 1))
            heapq.heappush(q, (nd.next_tx(t), 0, nid, nd.gen))
        else:                             # packet from nid ends; deliver
            s, e, sid, ssync, cclock, nmask = txlog[info]
            for r in nodes:
                if r.id == sid:
                    continue
                lost = any(a < e and b > s and k != info
                           for k, (a, b, *_rest) in enumerate(txlog[-4 * n:], len(txlog) - min(len(txlog), 4 * n)))
                if lost:
                    continue
                r.timeouts(t)
                r.last_rx_any = t
                if nmask & (1 << r.id):
                    r.last_acked = t
                r.heard.setdefault(sid, []).append(t)
                if r.should_sync(sid, ssync):
                    sender_cycle_num = cclock // r.cycle
                    new_epoch = s - (sid * SLOT + MARGIN) - sender_cycle_num * r.cycle
                    if new_epoch != r.epoch:
                        r.epoch = new_epoch
                        r.gen += 1
                        heapq.heappush(q, (r.next_tx(t), 0, r.id, r.gen))
                    r.sync = min(ssync, sid)
                was = r.alive[sid]
                r.alive[sid] = True
                r.last_seen[sid] = t
                r.peer_sync[sid] = ssync
                if not was:
                    r.find_new_sync()
    # whole-mesh health: every ordered pair (i hears j) must be heard in the
    # last 20 s; recovery = when the last pair stopped having >3-cycle gaps
    broken, rec = False, 0
    for r in nodes:
        for sid in range(n):
            if sid == r.id:
                continue
            h = r.heard.get(sid, [])
            if not h or h[-1] < horizon - 20000:
                broken = True
                continue
            gaps = [b - a for a, b in zip(h, h[1:])]
            last_big = max([i for i, g in enumerate(gaps) if g > 3 * cyc] + [-1])
            rec = max(rec, h[last_big + 1])
    return broken, (None if broken else rec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', type=int, default=1000)
    ap.add_argument('--nodes', type=int, nargs='+', default=[2, 4])
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--scenario', default='reboot0', choices=['reboot0', 'reboot2', 'cold'])
    a = ap.parse_args()
    for n in a.nodes:
        for v74 in (False, True):
            rng = random.Random(a.seed)
            res = [trial(n, v74, rng, scenario=a.scenario) for _ in range(a.trials)]
            deaf = sum(d for d, _ in res)
            recs = sorted(r for d, r in res if r is not None)
            pct = lambda q: recs[min(len(recs) - 1, int(q * len(recs)))] / 1000.0 if recs else float('nan')
            print(f'[{a.scenario}] {n} radios, {"v7.4" if v74 else "v7.3"}: mesh NOT fully connected 40-60 s '
                  f'later in {deaf}/{a.trials} ({100.0 * deaf / a.trials:.1f} %); '
                  f'full mesh after median {pct(0.5):.1f} s, 95th {pct(0.95):.1f} s, max {pct(1.0):.1f} s')


if __name__ == '__main__':
    main()
