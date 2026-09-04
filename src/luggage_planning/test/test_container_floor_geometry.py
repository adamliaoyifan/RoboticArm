#!/usr/bin/env python3
"""Regression tests for the E12 STL floor evidence."""
import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_planning.container_floor_geometry import analyze_floor_surfaces  # noqa: E402


class TestContainerFloorGeometry(unittest.TestCase):
    def test_real_container_has_slab_top_near_053(self):
        mesh = os.path.normpath(os.path.join(
            PKG_ROOT, "..", "luggage_gazebo", "models",
            "airport_container_real", "meshes", "container_collision.stl"))
        result = analyze_floor_surfaces(mesh)
        floor = result["floor_candidate"]
        self.assertIsNotNone(floor)
        self.assertAlmostEqual(floor["z"], 0.53, delta=0.005)
        self.assertGreater(floor["area"], 2.5)
        self.assertGreater(floor["x_range"][1] - floor["x_range"][0], 1.4)
        self.assertGreater(floor["y_range"][1] - floor["y_range"][0], 1.5)


if __name__ == "__main__":
    unittest.main()
