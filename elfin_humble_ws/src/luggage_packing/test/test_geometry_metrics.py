#!/usr/bin/env python3
"""G5 exact metric adapter and static audits of migrated entry points."""

from __future__ import annotations

import os
import unittest

import yaml

from luggage_description.container_geometry import (
    descriptor_from_scene_config,
    floor_area,
    normalize_descriptor,
    volume,
)
from luggage_packing.geometry_metrics import (
    FLOOR_CONTACT_TOL_M,
    GeometryMetricsError,
    annotate_replay_runs,
    capacity_report,
    denominators,
    floor_polygon,
    packed_volume_from_boxes,
)

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DESC_ROOT = os.path.normpath(os.path.join(PKG_ROOT, "..", "luggage_description"))
SRC_ROOT = os.path.dirname(PKG_ROOT)
ENTRY_POINTS = (
    os.path.join(SRC_ROOT, "luggage_gazebo", "scripts", "pack_eval_driver.py"),
    os.path.join(SRC_ROOT, "luggage_bringup", "scripts", "active_loading_bag_harness.py"),
    os.path.join(SRC_ROOT, "luggage_bringup", "scripts", "multi_box_gazebo_matrix.py"),
    os.path.join(SRC_ROOT, "luggage_packing", "scripts", "packing_replay_eval.py"),
)
FORBIDDEN_DENOMINATOR_SNIPPETS = (
    "4.344",
    "1.49 * 1.97",
    "1.49*1.97",
    "INNER_VOLUME",
    "INNER_FLOOR_XY",
    "DEFAULT_USABLE_VOLUME_M3",
    "DEFAULT_USABLE_FLOOR_M2",
    "USABLE_VOLUME_M3",
    "USABLE_FLOOR_M2",
)


def _scene_geometry():
    path = os.path.join(DESC_ROOT, "config", "scene_tf.yaml.example")
    with open(path, "r", encoding="utf-8") as handle:
        return descriptor_from_scene_config(yaml.safe_load(handle))


class TestGeometryMetrics(unittest.TestCase):
    def setUp(self):
        self.geometry = _scene_geometry()

    def test_checked_in_seven_face_denominators(self):
        denom = denominators(self.geometry)
        self.assertAlmostEqual(denom["usable_volume_m3"], 4.22433625, places=9)
        self.assertAlmostEqual(denom["floor_area_m2"], 2.28715, places=9)
        self.assertEqual(denom["geometry_hash"], self.geometry.geometry_hash)
        self.assertEqual(denom["schema_version"], self.geometry.schema_version)

    def test_cuboid_fallback_is_lwh_and_lw(self):
        cuboid = normalize_descriptor({
            "frame_id": "container_link",
            "length": 1.5,
            "width": 2.0,
            "floor_z": 0.4,
            "ceiling_z": 1.9,
        })
        denom = denominators(cuboid)
        self.assertAlmostEqual(denom["usable_volume_m3"], 1.5 * 2.0 * 1.5, places=9)
        self.assertAlmostEqual(denom["floor_area_m2"], 1.5 * 2.0, places=9)

    def test_fail_closed_reasons(self):
        cases = (
            (None, "MISSING_GEOMETRY"),
            ({}, "MISSING_GEOMETRY"),
            ({"length": float("inf"), "width": 1.0, "floor_z": 0.0,
              "ceiling_z": 1.0}, "NON_FINITE_GEOMETRY"),
            ({"length": 1.0, "width": 1.0, "floor_z": 0.0, "ceiling_z": 1.0,
              "chamfer": {"side": "negative_y", "floor_y": 0.0, "wall_z": 0.5}},
             "UNSUPPORTED_GEOMETRY"),
        )
        for descriptor, reason in cases:
            with self.subTest(reason=reason):
                with self.assertRaises(GeometryMetricsError) as caught:
                    denominators(descriptor)
                self.assertEqual(caught.exception.reason, reason)
        good = self.geometry.descriptor()
        bad = dict(good)
        bad["width"] = bad["width"] + 0.01
        with self.assertRaises(GeometryMetricsError) as caught:
            denominators(bad)
        self.assertEqual(caught.exception.reason, "HASH_MISMATCH")

    def test_known_box_volume_fraction(self):
        box = {
            "center": [0.0, 0.0, self.geometry.floor_z + 0.125],
            "size": [0.55, 0.40, 0.25],
            "yaw": 0.0,
        }
        packed = packed_volume_from_boxes([box])
        report = capacity_report(packed, [box], self.geometry)
        self.assertAlmostEqual(packed, 0.55 * 0.40 * 0.25, places=12)
        self.assertAlmostEqual(
            report["volume_fraction"], packed / 4.22433625, places=12)
        self.assertEqual(report["geometry_hash"], self.geometry.geometry_hash)
        self.assertIn("packed_volume_m3", report)
        self.assertIn("usable_volume_m3", report)
        self.assertIn("schema_version", report)

    def test_full_real_floor_coverage_excludes_wedge(self):
        poly = floor_polygon(self.geometry)
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        box = {
            "center": [
                0.5 * (min(xs) + max(xs)),
                0.5 * (min(ys) + max(ys)),
                self.geometry.floor_z + 0.1,
            ],
            "size": [max(xs) - min(xs), max(ys) - min(ys), 0.2],
            "yaw": 0.0,
        }
        report = capacity_report(packed_volume_from_boxes([box]), [box], self.geometry)
        self.assertAlmostEqual(report["floor_coverage"], 1.0, places=9)
        self.assertAlmostEqual(report["packed_floor_area_m2"], 2.28715, places=9)
        aabb = {
            "center": [0.0, 0.0, self.geometry.floor_z + 0.1],
            "size": [self.geometry.length, self.geometry.width, 0.2],
            "yaw": 0.0,
        }
        aabb_report = capacity_report(
            packed_volume_from_boxes([aabb]), [aabb], self.geometry)
        self.assertAlmostEqual(aabb_report["floor_coverage"], 1.0, places=9)
        self.assertLess(
            aabb_report["packed_floor_area_m2"],
            self.geometry.length * self.geometry.width - 1e-6)

    def test_stacked_box_excluded_from_floor(self):
        floor_box = {
            "center": [0.0, 0.0, self.geometry.floor_z + 0.1],
            "size": [0.4, 0.3, 0.2],
            "yaw": 0.0,
        }
        stacked = {
            "center": [0.0, 0.0, self.geometry.floor_z + 0.4],
            "size": [0.4, 0.3, 0.2],
            "peak": 0.3,
            "yaw": 0.0,
        }
        report = capacity_report(
            packed_volume_from_boxes([floor_box, stacked]),
            [floor_box, stacked], self.geometry)
        only_floor = capacity_report(
            packed_volume_from_boxes([floor_box]), [floor_box], self.geometry)
        self.assertEqual(report["floor_item_count"], 1)
        self.assertAlmostEqual(
            report["packed_floor_area_m2"],
            only_floor["packed_floor_area_m2"], places=9)
        self.assertGreater(abs(stacked["center"][2] - 0.1 - self.geometry.floor_z),
                           FLOOR_CONTACT_TOL_M)

    def test_overlapping_footprints_use_union(self):
        left = {
            "center": [-0.15, 0.0, self.geometry.floor_z + 0.1],
            "size": [0.5, 0.4, 0.2],
            "yaw": 0.0,
        }
        right = {
            "center": [0.15, 0.0, self.geometry.floor_z + 0.1],
            "size": [0.5, 0.4, 0.2],
            "yaw": 0.0,
        }
        union = capacity_report(
            packed_volume_from_boxes([left, right]), [left, right], self.geometry)
        one = capacity_report(
            packed_volume_from_boxes([left]), [left], self.geometry)
        naive = 2.0 * one["packed_floor_area_m2"]
        self.assertLess(union["packed_floor_area_m2"] + 1e-9, naive)
        self.assertGreater(union["packed_floor_area_m2"], one["packed_floor_area_m2"])
        expected = 0.5 * 0.4 + 0.5 * 0.4 - 0.2 * 0.4
        self.assertAlmostEqual(union["packed_floor_area_m2"], expected, places=6)

    def test_replay_volume_is_authoritative_and_floor_is_legacy(self):
        runs = [{
            "V_placed": 0.55 * 0.40 * 0.25,
            "V_container": 1.49 * 1.97 * 2.01,
            "overall_fill_rate": 0.01,
            "floor_coverage": 0.35,
            "reachable_volume_ratio": 0.2,
            "reachable_fill_rate": 0.05,
        }]
        report = annotate_replay_runs(runs, self.geometry)
        self.assertAlmostEqual(
            report["volume_fraction"]["volume_fraction"],
            (0.55 * 0.40 * 0.25) / 4.22433625, places=12)
        legacy = report["legacy_non_authoritative"]
        self.assertEqual(legacy["status"], "legacy_non_authoritative")
        self.assertEqual(legacy["floor_coverage"], 0.35)
        self.assertEqual(legacy["V_container"], 1.49 * 1.97 * 2.01)
        self.assertIn("cannot satisfy a G5 floor", legacy["note"])

    def test_entry_points_have_no_rectangular_denominators(self):
        for path in ENTRY_POINTS:
            with self.subTest(path=os.path.basename(path)):
                self.assertTrue(os.path.isfile(path), path)
                with open(path, encoding="utf-8") as handle:
                    text = handle.read()
                for snippet in FORBIDDEN_DENOMINATOR_SNIPPETS:
                    self.assertNotIn(snippet, text)
                self.assertIn("geometry_metrics", text)

    def test_volume_and_floor_kernel_agree(self):
        self.assertAlmostEqual(volume(self.geometry), 4.22433625, places=9)
        self.assertAlmostEqual(floor_area(self.geometry), 2.28715, places=9)
        expected_floor = self.geometry.length * (self.geometry.half_y + 0.55)
        self.assertAlmostEqual(floor_area(self.geometry), expected_floor, places=9)


if __name__ == "__main__":
    unittest.main()
