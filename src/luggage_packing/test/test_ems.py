#!/usr/bin/env python3
"""Unit tests for EMS (Maximal Empty Space) maintenance. No roscore."""

import os
import sys
import unittest

import yaml

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_description.container_geometry import (  # noqa: E402
    descriptor_from_scene_config,
    normalize_descriptor,
    volume as hull_volume,
)
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


def _chamfer_hull(floor_z=0.0):
    return normalize_descriptor({
        "frame_id": "container_link",
        "length": 2.0,
        "width": 2.0,
        "floor_z": floor_z,
        "ceiling_z": floor_z + 2.0,
        "chamfer": {
            "side": "positive_y",
            "floor_y": 0.20,
            "wall_y": 1.0,
            "wall_z": floor_z + 0.50,
        },
    })


class TestEMSHullClip(unittest.TestCase):
    INNER = (2.0, 2.0, 2.0)

    def test_empty_heptahedron_matches_kernel_volume(self):
        hull = _chamfer_hull()
        e = EMS(self.INNER, hull=hull)
        self.assertEqual(len(e.spaces), 1)
        clipped = e.space_volume(e.spaces[0])
        aabb = volume(e.spaces[0])
        self.assertAlmostEqual(clipped, hull_volume(hull), places=6)
        self.assertGreater(aabb, clipped + 0.1)
        self.assertAlmostEqual(e.container_volume(), hull_volume(hull), places=6)

    def test_empty_scene_example_matches_checked_in_volume(self):
        desc_root = os.path.join(
            os.path.dirname(PKG_ROOT), "luggage_description")
        path = os.path.join(desc_root, "config", "scene_tf.yaml.example")
        with open(path, "r", encoding="utf-8") as handle:
            hull = descriptor_from_scene_config(yaml.safe_load(handle))
        inner = (hull.length, hull.width, hull.height)
        e = EMS(inner, hull=hull)
        self.assertAlmostEqual(
            e.space_volume(e.spaces[0]), hull_volume(hull), places=6)
        self.assertAlmostEqual(hull_volume(hull), 4.22433625, places=6)
        aabb = hull.length * hull.width * hull.height
        self.assertGreater(aabb, hull_volume(hull) + 0.1)

    def test_floor_z_offset_does_not_change_empty_volume(self):
        hull = _chamfer_hull(floor_z=0.53)
        e = EMS(self.INNER, hull=hull)
        self.assertAlmostEqual(
            e.space_volume(e.spaces[0]), hull_volume(hull), places=6)

    def test_place_inside_hull_reduces_clipped_volume(self):
        hull = _chamfer_hull()
        e = EMS(self.INNER, min_useful_edge=0.05, hull=hull)
        box = (-0.25, -0.50, 0.0, 0.25, 0.0, 0.40)
        e.place(box)
        remaining = sum(e.space_volume(space) for space in e.spaces)
        self.assertAlmostEqual(
            remaining, hull_volume(hull) - volume(box), places=2)

    def test_place_in_chamfer_prism_does_not_reduce_usable(self):
        hull = _chamfer_hull()
        e = EMS(self.INNER, min_useful_edge=0.05, hull=hull)
        before = sum(e.space_volume(space) for space in e.spaces)
        # Strictly above the chamfer at this Z band (y_max(0.25) ≈ 0.60).
        prism_box = (0.0, 0.75, 0.0, 0.40, 0.95, 0.25)
        e.place(prism_box)
        after = sum(e.space_volume(space) for space in e.spaces)
        self.assertAlmostEqual(after, before, places=3)

    def test_prism_tile_reports_zero_volume(self):
        hull = _chamfer_hull()
        e = EMS(self.INNER, hull=hull)
        prism = (0.0, 0.75, 0.0, 0.50, 1.0, 0.25)
        self.assertAlmostEqual(e.space_volume(prism), 0.0, places=6)

    def test_regularity_uses_hull_as_reference(self):
        hull = _chamfer_hull()
        e = EMS(self.INNER, hull=hull)
        r0 = e.regularity()
        self.assertGreaterEqual(r0, 0.0)
        self.assertLessEqual(r0, 1.0)
        e.place((-0.5, -0.5, 0.0, 0.0, 0.0, 0.5))
        self.assertLess(e.regularity(), r0)


if __name__ == "__main__":
    unittest.main()
