#!/usr/bin/env python3
"""Unit tests for EMS (Maximal Empty Space) maintenance. No roscore."""

import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_packing.ems import EMS, volume  # noqa: E402


class TestEMS(unittest.TestCase):
    INNER = (2.0, 2.0, 2.0)

    def test_initial_space(self):
        e = EMS(self.INNER)
        self.assertEqual(len(e.spaces), 1)
        s = e.spaces[0]
        self.assertAlmostEqual(volume(s), 8.0, places=4)

    def test_split_on_place(self):
        """Placing a box splits the EMS; total volume = container - box."""
        e = EMS(self.INNER, min_useful_edge=0.1)
        box = (-0.25, -0.25, 0.0, 0.25, 0.25, 0.5)  # on the floor, centered
        e.place(box)
        self.assertGreater(len(e.spaces), 1)
        total = sum(volume(s) for s in e.spaces)
        self.assertAlmostEqual(total, 8.0 - 0.125, places=3)
        # No EMS intersects the placed box.
        for s in e.spaces:
            self.assertFalse(EMS.intersects_box(s, box))

    def test_containment_elimination(self):
        """Sub-spaces fully contained in another are removed."""
        e = EMS(self.INNER, min_useful_edge=0.1)
        # Place a small box; the result should have no duplicate/contained spaces.
        e.place((-0.1, -0.1, 0.0, 0.1, 0.1, 0.2))
        for i, s in enumerate(e.spaces):
            for j, t in enumerate(e.spaces):
                if i == j:
                    continue
                # s must not be strictly contained in t.
                contained = (s[0] >= t[0] - 1e-9 and s[1] >= t[1] - 1e-9 and
                             s[2] >= t[2] - 1e-9 and s[3] <= t[3] + 1e-9 and
                             s[4] <= t[4] + 1e-9 and s[5] <= t[5] + 1e-9)
                self.assertFalse(contained, "EMS %d is contained in %d" % (i, j))

    def test_min_useful_edge_filters(self):
        """Sub-spaces below min_useful_edge on any axis are dropped."""
        e = EMS(self.INNER, min_useful_edge=0.5)
        # A thin box produces thin sub-spaces that should be filtered.
        e.place((-0.05, -1.0, 0.0, 0.05, 1.0, 0.5))
        for s in e.spaces:
            self.assertGreaterEqual(s[3] - s[0], 0.5 - 1e-9)
            self.assertGreaterEqual(s[4] - s[1], 0.5 - 1e-9)
            self.assertGreaterEqual(s[5] - s[2], 0.5 - 1e-9)

    def test_regularity_in_range(self):
        e = EMS(self.INNER)
        r0 = e.regularity()
        self.assertGreaterEqual(r0, 0.0)
        self.assertLessEqual(r0, 1.0)
        # Placing a box fragments space -> regularity drops.
        e.place((-0.5, -0.5, 0.0, 0.5, 0.5, 0.5))
        r1 = e.regularity()
        self.assertLess(r1, r0)


if __name__ == "__main__":
    unittest.main()
