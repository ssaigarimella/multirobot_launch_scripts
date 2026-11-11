#!/usr/bin/env python3
"""lora_tdma_sim.py - event simulation of the radiohive TDMA mesh sync rules,
v7.3 vs v7.4 vs v7.5.

    python3 tests/lora_tdma_sim.py [--trials 1000] [--nodes 2 4]
                                   [--scenario reboot0|reboot2|cold|split]
                                   [--topology full|chain]

Model (mirrors lora_mesh_v7_2_ros2.ino): the cycle is sized by the compiled
NUM_NODES (4 slots, 190 ms) however many radios are powered; a node
transmits once per cycle at slot_start + TX_MARGIN, packets take AIRTIME ms,
half-duplex (a node cannot receive while transmitting), any two overlapping
packets at a receiver are both lost (no capture effect), clocks are ideal
(no drift, no jitter), so a deadlock here is permanent. handleRx /
shouldSyncTo / findNewSyncSource / the 3 s peer timeout and the
neighbor_mask every packet carries are reproduced from the firmware, and the
per-version rules are:
  v7.3  no recovery
  v7.4  re-roll the phase when NO peer is heard for 1.5 s (last_rx_any; this
        is what was flashed 2026-10-26) + relayed sync when our authority is
        not heard
  v7.5  re-roll when no peer has listed us in its neighbor_mask for 1.5 s
        (covers hearing no one too) + relayed sync + listen-only cycles: while
        any of the NUM_NODES-1 peers is not alive, skip our own TX in a
        random 1 cycle in LISTEN_SKIP_N
Topology: full = every radio hears every other; chain = radio i hears only
i-1 and i+1 (the out-of-mutual-range case).
Scenarios (the mesh starts synced to authority 0 unless noted):
  reboot0  node 0 reboots at a random phase at t=0 (field 2026-10-23 15:22)
  reboot2  node 2 reboots at a random phase
  cold     every radio powers up at its own random phase
  split    two synced halves, {0..n/2-1} on authority 0 and {n/2..n-1} on
           authority n/2, phased so the two middle radios transmit at the
           same time (the chain case from 2026-10-30)
  depart0  node 0 (the reference) goes silent for good at t=0; health is
           judged over the survivors only, and the reference identifier each
           survivor ends on is reported
Each trial runs 60 s; the mesh is "broken" if any in-range ordered pair has
not been heard in the last 20 s, and recovery is the time until every
in-range ordered pair hears each other steadily.
"""
import argparse
import heapq
import random

SLOT, GUARD, MARGIN, AIRTIME = 40, 30, 5, 20
NUM_NODES = 4                     # compiled into every radio; sizes the cycle
TIMEOUT, DEAF = 3000, 1500
LISTEN_SKIP_N = 10                # v7.5: skip 1 TX in N while a peer is unheard
VERSIONS = ('v7.3', 'v7.4', 'v7.5')


class Node:
    def __init__(self, nid, n, epoch, ver, rng):
        self.id, self.n, self.ver, self.rng = nid, n, ver, rng
        self.cycle = NUM_NODES * SLOT + GUARD
        self.epoch = epoch
        self.sync = nid
        self.alive = [False] * n
        self.alive[nid] = True
        self.last_seen = [0] * n
        self.peer_sync = list(range(n))
        self.last_rx_any = 0
        self.last_acked = 0   # last packet whose neighbor_mask listed us
        self.last_reroll = 0
        self.gen = 0          # invalidates queued TX events after an epoch change
        self.heard = {}       # sender -> times heard

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
        if (self.ver != 'v7.3' and auth == self.sync and self.sync != self.id
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
        last = {'v7.3': None, 'v7.4': self.last_rx_any, 'v7.5': self.last_acked}[self.ver]
        if last is not None and now - last > DEAF and now - self.last_reroll > DEAF:
            self.last_reroll = now
            self.epoch += SLOT + self.rng.randrange(self.cycle - 2 * SLOT)
            self.sync = self.id
            return True
        return False

    def listen_only(self):
        # firmware counts the compiled NUM_NODES, so radios that are not
        # powered count as unheard peers
        if self.ver != 'v7.5' or (self.n == NUM_NODES and all(self.alive)):
            return False
        return self.rng.randrange(LISTEN_SKIP_N) == 0


def trial(n, ver, rng, horizon=60000, scenario='reboot0', topology='full'):
    cyc = NUM_NODES * SLOT + GUARD
    hears = [[i != j and (topology == 'full' or abs(i - j) == 1) for j in range(n)] for i in range(n)]
    base = rng.randrange(cyc)
    nodes = [Node(i, n, base, ver, rng) for i in range(n)]
    for nd in nodes:                      # synced, in-range peers alive, authority 0
        nd.sync = 0
        nd.alive = [i == nd.id or hears[nd.id][i] for i in range(n)]
        nd.peer_sync = [0] * n
    if scenario == 'split':
        h = n // 2
        # node h transmits at epoch_b + h*SLOT; put it within the airtime of
        # node h-1, which transmits at base + (h-1)*SLOT
        epoch_b = base - SLOT + rng.randrange(-AIRTIME + 1, AIRTIME)
        for i in range(h, n):
            nodes[i].epoch = epoch_b
            nodes[i].sync = h
            nodes[i].peer_sync = [h] * n
            nodes[i].alive = [j == i or (j >= h and hears[i][j]) for j in range(n)]
        for i in range(h):
            nodes[i].alive = [j == i or (j < h and hears[i][j]) for j in range(n)]
    elif scenario == 'depart0':
        pass
    else:
        # t=0: one radio reboots (fresh state, random phase), or every radio
        # powers up at its own random phase (field-day cold start)
        fresh = {'reboot0': [0], 'reboot2': [2 % n], 'cold': list(range(n))}[scenario]
        for i in fresh:
            nodes[i] = Node(i, n, rng.randrange(cyc), ver, rng)
    q, txlog = [], []
    silent = {0} if scenario == 'depart0' else set()
    for nd in nodes:
        if nd.id not in silent:
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
            heapq.heappush(q, (nd.next_tx(t), 0, nid, nd.gen))
            if nd.listen_only():
                continue
            txlog.append((t, t + AIRTIME, nid, nd.sync, t - nd.epoch, nd.mask()))
            heapq.heappush(q, (t + AIRTIME, 1, nid, len(txlog) - 1))
        else:                             # packet from nid ends; deliver
            s, e, sid, ssync, cclock, nmask = txlog[info]
            recent = range(max(0, len(txlog) - 4 * n), len(txlog))
            for r in nodes:
                if r.id == sid or not hears[r.id][sid]:
                    continue
                # lost if r was transmitting (half-duplex) or another packet
                # r can hear overlapped it
                if any(k != info and txlog[k][0] < e and txlog[k][1] > s
                       and (txlog[k][2] == r.id or hears[r.id][txlog[k][2]]) for k in recent):
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
    # mesh health: every in-range ordered pair (i hears j) must be heard in
    # the last 20 s; recovery = when the last pair stopped having >5-cycle gaps
    # (v7.5 skips 1 TX in 10 for ever in a chain, so a run of 3-4 skips in a
    # row is expected over a minute and is not a sync failure)
    broken, rec = False, 0
    for r in nodes:
        for sid in range(n):
            if not hears[r.id][sid] or r.id in silent or sid in silent:
                continue
            h = r.heard.get(sid, [])
            if not h or h[-1] < horizon - 20000:
                broken = True
                continue
            gaps = [b - a for a, b in zip(h, h[1:])]
            last_big = max([i for i, g in enumerate(gaps) if g > 5 * cyc] + [-1])
            rec = max(rec, h[last_big + 1])
    if scenario == 'depart0':
        trial.final_sync = [nd.sync for nd in nodes if nd.id not in silent]
    return broken, (None if broken else rec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', type=int, default=1000)
    ap.add_argument('--nodes', type=int, nargs='+', default=[2, 4])
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--scenario', default='reboot0', choices=['reboot0', 'reboot2', 'cold', 'split', 'depart0'])
    ap.add_argument('--topology', default='full', choices=['full', 'chain'])
    ap.add_argument('--versions', nargs='+', default=list(VERSIONS), choices=VERSIONS)
    a = ap.parse_args()
    for n in a.nodes:
        for ver in a.versions:
            rng = random.Random(a.seed)
            res, syncs = [], {}
            for _ in range(a.trials):
                res.append(trial(n, ver, rng, scenario=a.scenario, topology=a.topology))
                if a.scenario == 'depart0':
                    k = tuple(trial.final_sync)
                    syncs[k] = syncs.get(k, 0) + 1
            deaf = sum(d for d, _ in res)
            recs = sorted(r for d, r in res if r is not None)
            pct = lambda q: recs[min(len(recs) - 1, int(q * len(recs)))] / 1000.0 if recs else float('nan')
            print(f'[{a.scenario}/{a.topology}] {n} radios, {ver}: mesh NOT fully connected 40-60 s '
                  f'later in {deaf}/{a.trials} ({100.0 * deaf / a.trials:.1f} %); '
                  f'full mesh after median {pct(0.5):.1f} s, 95th {pct(0.95):.1f} s, max {pct(1.0):.1f} s')
            if syncs:
                print('    survivors\' final reference ids: ' +
                      ', '.join(f'{k} x{v}' for k, v in sorted(syncs.items(), key=lambda kv: -kv[1])))


if __name__ == '__main__':
    main()
