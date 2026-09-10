#!/usr/bin/env python3
"""ROS-independent G5 tests for the two migrated bringup metric scripts."""

from __future__ import annotations

import importlib.util
import os
import unittest

from luggage_description.container_geometry import descriptor_from_scene_config
from luggage_description.scene_tf_config_utils import load_scene_tf_config

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.normpath(os.path.join(HERE, "..", "scripts"))
SRC = os.path.normpath(os.path.join(HERE, "..", ".."))


def _load(name, filename):
    path = os.path.join(SCRIPTS, filename)
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestBringupG5Metrics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.harness = _load(
            "active_loading_bag_harness", "active_loading_bag_harness.py")
        cls.matrix = _load(
            "multi_box_gazebo_matrix", "multi_box_gazebo_matrix.py")
        cls.scene = load_scene_tf_config()
        cls.geometry = descriptor_from_scene_config(cls.scene)

    def _placement(self, peak=0.0, x=0.0, y=0.0, size=(0.55, 0.40, 0.25)):
        return {
            "kind": "placement",
            "size": list(size),
            "peak": peak,
            "container_x": x,
            "container_y": y,
            "yaw": 0.0,
            "atlas_status": 2,
            "pose_gate_passed": True,
            "placed_count": 1,
        }

    def test_harness_uses_exact_hull_volume(self):
        records = [
            {"kind": "status", "placed_count": 1},
            {"kind": "detection", "success": True},
            {"kind": "map", "map_revision": 1, "event": "commit"},
            {"kind": "release", "released_at_contact": True,
             "retreat_after_release": True},
            self._placement(),
        ]
        result = self.harness.evaluate_records(
            records, expected_boxes=1, min_floor_items=1, scene_config=self.scene)
        packed = 0.55 * 0.40 * 0.25
        self.assertAlmostEqual(
            result["metrics"]["usable_volume_m3"], 4.22433625, places=9)
        self.assertAlmostEqual(
            result["metrics"]["volume_utilization"],
            packed / 4.22433625, places=9)
        self.assertEqual(
            result["metrics"]["geometry_hash"], self.geometry.geometry_hash)
        self.assertIsNone(result["metrics"]["geometry_metrics_reason"])
        self.assertNotAlmostEqual(
            result["metrics"]["usable_volume_m3"], 4.344, places=3)

    def test_harness_fails_closed_on_invalid_geometry(self):
        result = self.harness.evaluate_records(
            [self._placement()],
            expected_boxes=1,
            min_floor_items=0,
            scene_config={
                "container": {
                    "inner": {
                        "length": float("inf"),
                        "width": 1.0,
                        "floor_z": 0.0,
                        "ceiling_z": 1.0,
                    }
                }
            },
        )
        self.assertFalse(result["passed"])
        self.assertIn("geometry_metrics", result["rejection_reasons"])
        self.assertEqual(
            result["metrics"]["geometry_metrics_reason"], "NON_FINITE_GEOMETRY")

    def test_matrix_capacity_helper_matches_harness(self):
        placements = [self._placement(), self._placement(x=0.4, y=-0.2)]
        report = self.matrix._capacity_from_placements(
            placements, scene_config=self.scene)
        records = [
            {"kind": "status", "placed_count": 2},
            {"kind": "detection", "success": True},
            {"kind": "detection", "success": True},
            {"kind": "map", "map_revision": 1, "event": "commit"},
            {"kind": "map", "map_revision": 2, "event": "commit"},
            {"kind": "release", "released_at_contact": True,
             "retreat_after_release": True},
            {"kind": "release", "released_at_contact": True,
             "retreat_after_release": True},
            placements[0],
            placements[1],
        ]
        harness = self.harness.evaluate_records(
            records, expected_boxes=2, min_floor_items=2, scene_config=self.scene)
        self.assertAlmostEqual(
            report["volume_fraction"],
            harness["metrics"]["volume_utilization"], places=12)
        self.assertAlmostEqual(
            report["floor_coverage"],
            harness["metrics"]["floor_coverage_ratio"], places=12)
        self.assertAlmostEqual(report["usable_volume_m3"], 4.22433625, places=9)


if __name__ == "__main__":
    unittest.main()
