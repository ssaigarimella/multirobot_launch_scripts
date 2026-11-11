#!/usr/bin/env python3
"""The replay must draw trails in the mapper's levelled frame.

cuVSLAM's odometry topic is in the camera's pitched start frame; nvblox maps in
that frame rotated by R_y(+20 deg). These tests pin fleet_player.level_odom and
bag2timeline.odom_level_deg to that geometry.

    python3 tests/test_replay_level.py      (plain python3: the viz tools need the user-site NumPy)
"""
import math
import os
import sys
import unittest

import numpy as np

os.environ.setdefault('MPLBACKEND', 'Agg')
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'viz'))
import fleet_player as fp  # noqa: E402
import bag2timeline as b2t  # noqa: E402


def pitched_level_flight(dist, deg=20.0):
    """Odometry rows (t x y z yaw) of a level flight `dist` metres forward and
    2 m sideways, seen in a start frame pitched `deg` nose-down."""
    a = math.radians(deg)
    t = np.linspace(0.0, 10.0, 101)
    fwd = np.linspace(0.0, dist, 101)
    side = np.linspace(0.0, 2.0, 101)
    return np.column_stack([t, fwd * math.cos(a), side, fwd * math.sin(a), np.zeros_like(t)])


class TestLevelOdom(unittest.TestCase):
    def test_level_flight_comes_out_level(self):
        od = fp.level_odom(pitched_level_flight(35.0))
        self.assertAlmostEqual(od[-1, 1], 35.0, places=6)   # full forward distance
        self.assertAlmostEqual(od[-1, 2], 2.0, places=6)    # sideways untouched
        self.assertLess(np.abs(od[:, 3]).max(), 1e-9)       # no fake climb

    def test_input_not_modified_and_empty_ok(self):
        raw = pitched_level_flight(10.0)
        keep = raw.copy()
        fp.level_odom(raw)
        np.testing.assert_array_equal(raw, keep)
        self.assertEqual(len(fp.level_odom(np.zeros((0, 5)))), 0)

    def test_zero_degrees_is_identity(self):
        raw = pitched_level_flight(10.0)
        np.testing.assert_allclose(fp.level_odom(raw, 0.0), raw)


class TestOdomLevelFit(unittest.TestCase):
    def _samples(self, deg, n):
        raw = pitched_level_flight(30.0, deg)[:n]
        lev = fp.level_odom(raw, deg)
        od = {(i, 0): (raw[i, 1], raw[i, 3]) for i in range(len(raw))}
        tf = {(i, 0): (lev[i, 1], lev[i, 3]) for i in range(len(lev))}
        return od, tf

    def test_recovers_angle(self):
        for deg in (20.0, 17.5, -5.0):
            self.assertAlmostEqual(b2t.odom_level_deg(*self._samples(deg, 101)), deg, places=2)

    def test_too_few_matches(self):
        self.assertIsNone(b2t.odom_level_deg(*self._samples(20.0, 30)))


if __name__ == '__main__':
    unittest.main()
