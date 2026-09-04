#!/usr/bin/env python3
"""Unit tests for vacuum attach geometry helpers."""

import math
import unittest

from luggage_planning.vacuum_attach_utils import (
    compose_transform,
    contact_distance,
    contact_ok,
    invert_transform,
    top_face_contact_ok,
    top_face_gap,
)


class VacuumAttachUtilsTest(unittest.TestCase):
    def test_compose_and_invert_round_trip(self):
        parent_t = [1.0, 2.0, 3.0]
        parent_q = [0.0, 0.0, 0.0, 1.0]
        child_t = [0.1, 0.2, 0.3]
        child_q = [0.0, 0.0, math.sin(0.25), math.cos(0.25)]
        world_t, world_q = compose_transform(parent_t, parent_q, child_t, child_q)
        inv_t, inv_q = invert_transform(parent_t, parent_q)
        back_t, back_q = compose_transform(inv_t, inv_q, world_t, world_q)
        for a, b in zip(back_t, child_t):
            self.assertAlmostEqual(a, b, places=5)
        for a, b in zip(back_q, child_q):
            self.assertAlmostEqual(a, b, places=5)

    def test_contact_ok_when_panel_on_box_top(self):
        panel = [0.0, 0.0, 0.66]
        box = [0.0, 0.0, 0.50]
        size = [0.80, 0.50, 0.32]
        self.assertTrue(contact_ok(panel, box, size, extra_margin=0.20))
        self.assertGreater(contact_distance(panel, box, size), 0.10)

    def test_contact_rejects_panel_far_from_box(self):
        panel = [2.0, 0.0, 1.0]
        box = [0.0, 0.0, 0.50]
        size = [0.80, 0.50, 0.32]
        self.assertFalse(contact_ok(panel, box, size, extra_margin=0.05))

    def test_top_face_accepts_lid_contact(self):
        panel = [-1.0, 0.0, 1.15]
        box = [-1.0, 0.0, 1.01]
        size = [0.70, 0.45, 0.28]
        ok, gap = top_face_contact_ok(panel, box, size)
        self.assertTrue(ok)
        self.assertAlmostEqual(gap, 0.0, places=6)
        self.assertAlmostEqual(top_face_gap(panel, box, size), 0.0, places=6)

    def test_top_face_rejects_hover_and_side(self):
        box = [-1.0, 0.0, 1.01]
        size = [0.70, 0.45, 0.28]
        hover = [-1.0, 0.0, 1.45]
        ok, gap = top_face_contact_ok(hover, box, size)
        self.assertFalse(ok)
        self.assertAlmostEqual(gap, 0.30, places=6)
        side = [-0.35, 0.0, 1.15]
        ok, _ = top_face_contact_ok(side, box, size)
        self.assertFalse(ok)


if __name__ == "__main__":
    unittest.main()
