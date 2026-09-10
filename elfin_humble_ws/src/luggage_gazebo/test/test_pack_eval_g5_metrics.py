#!/usr/bin/env python3
"""G5 audit of pack_eval_driver metric emission (no Gazebo)."""

from __future__ import annotations

import os
import unittest

from luggage_description.container_geometry import descriptor_from_scene_config
from luggage_description.scene_tf_config_utils import load_scene_tf_config
from luggage_packing.geometry_metrics import (
    capacity_report,
    packed_volume_from_boxes,
)

HERE = os.path.dirname(os.path.abspath(__file__))
DRIVER = os.path.normpath(os.path.join(
    HERE, "..", "scripts", "pack_eval_driver.py"))


class TestPackEvalG5Metrics(unittest.TestCase):
    def test_driver_calls_geometry_metrics_and_drops_legacy_denoms(self):
        with open(DRIVER, encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn("from luggage_packing.geometry_metrics import", text)
        self.assertIn("capacity_report", text)
        self.assertNotIn("INNER_VOLUME", text)
        self.assertNotIn("INNER_FLOOR_XY", text)
        self.assertNotIn("4.344", text)
        self.assertNotIn("1.49 * 1.97", text)

    def test_suite_fields_can_be_produced_from_committed_boxes(self):
        geometry = descriptor_from_scene_config(load_scene_tf_config())
        boxes = [{
            "center": [0.0, 0.0, geometry.floor_z + 0.125],
            "size": [0.55, 0.40, 0.25],
            "yaw": 0.0,
        }]
        report = capacity_report(
            packed_volume_from_boxes(boxes), boxes, geometry)
        suite = {
            "volume_fraction": round(report["volume_fraction"], 4),
            "floor_coverage": round(report["floor_coverage"], 4),
            "inner_volume_m3": report["usable_volume_m3"],
            "packed_volume_m3": report["packed_volume_m3"],
            "schema_version": report["schema_version"],
            "geometry_hash": report["geometry_hash"],
        }
        self.assertAlmostEqual(suite["inner_volume_m3"], 4.22433625, places=9)
        self.assertEqual(suite["geometry_hash"], geometry.geometry_hash)


if __name__ == "__main__":
    unittest.main()
