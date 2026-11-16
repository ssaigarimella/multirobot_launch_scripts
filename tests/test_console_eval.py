#!/usr/bin/env python3
"""Unit tests for console/fleet_console.py evaluate() - no window, no drones.

    python3 -m pytest -q tests/test_console_eval.py
"""
import copy
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'console'))
os.environ.setdefault('DISPLAY', '')
import fleet_console as fc  # noqa: E402

DOWN = {"drone": "ghost", "vid": 3, "ros_age": 0.2, "host": {
    "container": True, "sessions": [], "serial": {"/dev/ttyUSB0": "px4", "/dev/ttyUSB1": "radio"},
    "video": 6, "realsense": True, "zenoh": False, "bag": None, "recording": False,
    "disk_free_gb": 873.1, "year": 2026},
    "ros": {"odom": {"hz": 0.0, "age": None, "pos": None}, "cam": {"hz": 0.0, "age": None},
            "px4": {"hz": 0.0, "age": None}, "fis": {"hz": 0.0, "age": None, "n": None},
            "guard": {"hz": 0.0, "age": None, "state": None},
            "planner": {"hz": 0.0, "age": None, "state": None},
            "selfmap": {"hz": 0.0, "age": None, "slice_z": None}, "shared": {"hz": 0.0, "age": None},
            "coord": {"hz": 0.0, "age": None, "peers": []}, "lora": {"hz": 0.0, "age": None},
            "align": {"age": None, "state": None}, "zenoh_peers": [],
            "procs": {"1": False, "2": False, "3": False, "4": False, "5": False, "6": False,
                      "7": False, "agent": False, "lora_bridge": False},
            "devs": {"video": 6, "tty": ["/dev/ttyUSB0", "/dev/ttyUSB1"]}}}


def healthy(drone='ghost', peers=(1, 2, 4)):
    s = copy.deepcopy(DOWN)
    s['drone'] = drone
    s['host'].update(sessions=['fleet_tab%d' % n for n in range(1, 9)], zenoh=True)
    r = s['ros']
    r['procs'] = {str(n): True for n in range(1, 8)}
    r['procs'].update(agent=True, lora_bridge=True)
    r['odom'] = {'hz': 30.0, 'age': 0.0, 'pos': [0.01, -0.02, 0.0]}
    r['level'] = {'hz': 30.0, 'source': 'gravity', 'pitch': 26.2, 'roll': -1.7}
    r['cam'] = {'hz': 30.0, 'age': 0.0}
    r['px4'] = {'hz': 50.0, 'age': 0.0, 'xy_valid': True, 'z_valid': True, 'z': -0.02,
                'armed': False, 'nav': 2, 'preflight_ok': True}
    r['fis'] = {'hz': 1.0, 'age': 0.5, 'n': 120, 'src': '/nvblox_shared_node/get_esdf_and_gradient'}
    r['guard'] = {'hz': 30.0, 'age': 0.0, 'state': 'STOP:OBSTACLE'}
    r['planner'] = {'hz': 20.0, 'age': 0.0, 'state': 'INIT'}
    r['shared'] = {'hz': 10.0, 'age': 0.1}
    r['coord'] = {'hz': 5.0, 'age': 0.1, 'peers': []}
    r['lora'] = {'hz': 1.0, 'age': 0.5, 'heard': list(peers), 'delivery': [1.0] * len(peers)}
    r['zenoh_peers'] = list(peers)
    return s


ALL = ['delta', 'buckshee', 'ghost', 'thunderstrike']
PF = {'preflight': ('ok', 'PASS')}


class TestSettle(unittest.TestCase):
    def test_last_json_line_wins(self):
        out = '[INFO] something\n{"odom": true, "kick": 1.42, "resets": 1, "after": 0.0, "ok": true}\n'
        self.assertEqual(fc.parse_settle(out)['kick'], 1.42)
        self.assertTrue(fc.parse_settle(out)['ok'])

    def test_no_json_is_none(self):
        self.assertIsNone(fc.parse_settle('Traceback (most recent call last):\n boom'))
        self.assertIsNone(fc.parse_settle(''))
        self.assertIsNone(fc.parse_settle('{broken'))


class TestEvaluate(unittest.TestCase):
    def test_unreachable(self):
        cells, ready = fc.evaluate('ghost', None, ALL, {})
        self.assertEqual(cells['Link'][0], 'bad')
        self.assertFalse(ready)

    def test_stack_down_is_grey_not_red(self):
        cells, ready = fc.evaluate('ghost', DOWN, ALL, {})
        self.assertEqual(cells['Link'][0], 'ok')
        self.assertEqual(cells['Devices'][0], 'ok')
        for c in ('1 cuVSLAM', '3 FIS', '7 Coord+LoRa'):
            self.assertEqual(cells[c][0], 'off', c)
        self.assertFalse(ready)

    def test_healthy_is_ready_only_after_preflight(self):
        cells, ready = fc.evaluate('ghost', healthy(), ALL, {})
        self.assertFalse(ready)                       # preflight not run yet
        cells, ready = fc.evaluate('ghost', healthy(), ALL, PF)
        self.assertTrue(ready, cells)
        self.assertEqual(cells['READY'], ('ok', 'READY FOR TAKEOFF'))

    def test_zero_age_counts_as_fresh(self):
        s = healthy()
        for k in ('guard', 'planner', 'coord', 'fis', 'shared'):
            s['ros'][k]['age'] = 0.0
        cells, ready = fc.evaluate('ghost', s, ALL, PF)
        self.assertTrue(ready, cells)

    def test_bad_init_on_ground_is_red(self):
        s = healthy()
        s['ros']['odom']['pos'] = [6.1, 0.3, 0.0]
        cells, ready = fc.evaluate('ghost', s, ALL, PF)
        self.assertEqual(cells['1 cuVSLAM'][0], 'bad')
        self.assertIn('BAD INIT', cells['1 cuVSLAM'][1])
        self.assertFalse(ready)

    def test_far_position_is_fine_once_armed(self):
        s = healthy()
        s['ros']['odom']['pos'] = [12.0, 3.0, 1.0]
        s['ros']['px4']['armed'] = True
        cells, _ = fc.evaluate('ghost', s, ALL, PF)
        self.assertEqual(cells['1 cuVSLAM'][0], 'ok')

    def test_no_px4_data(self):
        s = healthy()
        s['ros']['px4'] = {'hz': 0.0, 'age': None}
        cells, ready = fc.evaluate('ghost', s, ALL, PF)
        self.assertEqual(cells['2 PX4 link'][0], 'wait')
        self.assertFalse(ready)

    def test_lora_missing_peer_blocks(self):
        s = healthy(peers=(1, 2))                     # thunderstrike's radio not heard
        cells, ready = fc.evaluate('ghost', s, ALL, PF)
        self.assertEqual(cells['7 Coord+LoRa'][0], 'wait')
        self.assertIn('missing [4]', cells['7 Coord+LoRa'][1])
        self.assertFalse(ready)

    def test_solo_needs_no_zenoh_or_peers(self):
        s = healthy(peers=())
        s['host']['zenoh'] = False
        cells, ready = fc.evaluate('ghost', s, ['ghost'], PF)
        self.assertEqual(cells['8 Zenoh'][0], 'off')
        self.assertTrue(ready, cells)

    def test_camera_node_missing_in_container(self):
        s = healthy()
        s['ros']['devs']['video'] = 0
        cells, _ = fc.evaluate('ghost', s, ALL, PF)
        self.assertEqual(cells['Devices'][0], 'wait')

    def test_camera_not_on_usb(self):
        s = healthy()
        s['host']['realsense'] = False
        cells, ready = fc.evaluate('ghost', s, ALL, PF)
        self.assertEqual(cells['Devices'][0], 'bad')
        self.assertFalse(ready)

    def test_recording_required_when_asked(self):
        cells, ready = fc.evaluate('ghost', healthy(), ALL, dict(PF, want_record=True))
        self.assertFalse(ready)
        s = healthy()
        s['host'].update(recording=True, bag={'run': 'f1026', 'bytes': 1, 'rate': 2.0e6})
        cells, ready = fc.evaluate('ghost', s, ALL, dict(PF, want_record=True))
        self.assertTrue(ready, cells)


class TestLevelAndRecording(unittest.TestCase):
    """2026-11-16: the map frame is levelled from gravity; frontiers come from
    the fused map; a vehicle must not fly unrecorded."""

    def test_level_shown_when_from_gravity(self):
        cells, ready = fc.evaluate('ghost', healthy(), ALL, PF)
        self.assertEqual(cells['1 cuVSLAM'][0], 'ok')
        self.assertIn('26.2 deg down', cells['1 cuVSLAM'][1])
        self.assertTrue(ready)

    def test_no_level_status_yet_waits(self):
        s = healthy()
        s['ros']['level'] = {'hz': 0.0}
        cells, ready = fc.evaluate('ghost', s, ALL, PF)
        self.assertEqual(cells['1 cuVSLAM'][0], 'wait')
        self.assertFalse(ready)

    def test_old_probe_without_level_key_waits(self):
        s = healthy()
        del s['ros']['level']
        cells, ready = fc.evaluate('ghost', s, ALL, PF)
        self.assertEqual(cells['1 cuVSLAM'][0], 'wait')
        self.assertFalse(ready)

    def test_fallback_level_on_the_ground_is_red(self):
        s = healthy()
        s['ros']['level'] = {'hz': 30.0, 'source': 'fallback', 'pitch': 20.0, 'roll': 0.0}
        cells, ready = fc.evaluate('ghost', s, ALL, PF)
        self.assertEqual(cells['1 cuVSLAM'][0], 'bad')
        self.assertIn('NOT FROM GRAVITY', cells['1 cuVSLAM'][1])
        self.assertFalse(ready)

    def test_fallback_level_in_flight_is_reported_not_red(self):
        s = healthy()
        s['ros']['level'] = {'hz': 30.0, 'source': 'fallback', 'pitch': 20.0, 'roll': 0.0}
        s['ros']['px4']['armed'] = True
        s['host'].update(recording=True, bag={'run': 'f', 'bytes': 1, 'rate': 2.0e6})
        cells, _ = fc.evaluate('ghost', s, ALL, PF)
        self.assertEqual(cells['1 cuVSLAM'][0], 'ok')
        self.assertIn('fallback', cells['1 cuVSLAM'][1])

    def test_levelled_odometry_not_flowing_waits(self):
        s = healthy()
        s['ros']['level']['hz'] = 0.0
        cells, ready = fc.evaluate('ghost', s, ALL, PF)
        self.assertEqual(cells['1 cuVSLAM'][0], 'wait')
        self.assertIn('levelled odometry', cells['1 cuVSLAM'][1])
        self.assertFalse(ready)

    def test_odd_level_is_red(self):
        for pitch, roll in ((2.0, 0.0), (60.0, 0.0), (25.0, 14.0)):
            s = healthy()
            s['ros']['level'].update(pitch=pitch, roll=roll)
            cells, ready = fc.evaluate('ghost', s, ALL, PF)
            self.assertEqual(cells['1 cuVSLAM'][0], 'bad', (pitch, roll))
            self.assertFalse(ready)

    def test_frontier_source_is_named(self):
        cells, _ = fc.evaluate('ghost', healthy(), ALL, PF)
        self.assertIn('fused map', cells['3 FIS'][1])
        s = healthy()
        s['ros']['fis']['src'] = '/nvblox_node/get_esdf_and_gradient'
        cells, ready = fc.evaluate('ghost', s, ALL, PF)
        self.assertIn('own map', cells['3 FIS'][1])
        self.assertTrue(ready)          # the own map is the stand-in, not a fault

    def test_armed_without_recording_is_red(self):
        s = healthy()
        s['ros']['px4']['armed'] = True
        cells, ready = fc.evaluate('ghost', s, ALL, PF)
        self.assertEqual(cells['Recording'], ('bad', 'ARMED - NOT RECORDING'))
        s['host'].update(recording=True, bag={'run': 'f', 'bytes': 1, 'rate': 2.0e6})
        cells, _ = fc.evaluate('ghost', s, ALL, PF)
        self.assertEqual(cells['Recording'][0], 'ok')


if __name__ == '__main__':
    unittest.main()
