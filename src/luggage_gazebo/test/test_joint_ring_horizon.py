#!/usr/bin/env python3
"""The joint ring must cover a whole execute, not a fixed 20 s floor.

The 1838 `place_exit` execute ran 24.3 s against the bare floor and the ring
dropped its first 4.35 s. Divergence happened at t=17.2 s so that run stayed
diagnosable, but the start of an execute is what a planned-versus-measured
trace is compared against, so the next one may not be.
"""

import os
import sys
import unittest
from collections import deque

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from place_smoke_driver import (  # noqa: E402
    JOINT_RING_CEILING_SEC,
    JOINT_RING_FLOOR_SEC,
    joint_ring_horizon_sec,
    prune_joint_ring,
)


class TestJointRingHorizon(unittest.TestCase):

    def test_short_segment_keeps_the_floor(self):
        self.assertEqual(joint_ring_horizon_sec(), JOINT_RING_FLOOR_SEC)
        self.assertEqual(joint_ring_horizon_sec(1.0), JOINT_RING_FLOOR_SEC)

    def test_long_segment_grows_past_the_floor(self):
        self.assertGreater(joint_ring_horizon_sec(24.3), 24.3)

    def test_a_hung_segment_cannot_grow_without_bound(self):
        self.assertEqual(
            joint_ring_horizon_sec(10_000.0), JOINT_RING_CEILING_SEC)

    def test_the_1838_execute_is_no_longer_truncated(self):
        """Replay the measured 50 Hz execute and check t0 survives."""
        exec_t0 = 100.0
        duration = 24.3
        ring = deque()
        horizon = joint_ring_horizon_sec()
        stamp = exec_t0
        while stamp <= exec_t0 + duration:
            ring.append({"stamp": stamp})
            horizon = max(horizon, joint_ring_horizon_sec(stamp - exec_t0))
            prune_joint_ring(ring, stamp, horizon)
            stamp += 0.02
        self.assertAlmostEqual(ring[0]["stamp"], exec_t0, places=6)

    def test_the_old_fixed_floor_did_truncate_it(self):
        ring = deque()
        stamp = 100.0
        while stamp <= 124.3:
            ring.append({"stamp": stamp})
            prune_joint_ring(ring, stamp, JOINT_RING_FLOOR_SEC)
            stamp += 0.02
        self.assertGreater(ring[0]["stamp"] - 100.0, 4.0)


if __name__ == "__main__":
    unittest.main()
