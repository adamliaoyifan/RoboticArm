#!/usr/bin/env python3
"""Gate 2/3 tests for PlatformFreeDetector (no ROS).

Companion to docs/plans/platform_free_height_test_plan.md.
Uses the same synthetic scenes as test_top_support_estimator.py.
"""
from __future__ import division

import unittest

import numpy as np

from luggage_perception.platform_free_pipeline import (
    PlatformFreeDetector,
    SUPPORT_MODES,
)
from luggage_perception.top_support_estimator import (
    DETECT_SUPPORT_STAMP_MISMATCH,
    HEIGHT_SOURCE_CATALOG_PRIOR,
    HEIGHT_SOURCE_CONFIGURED_SUPPORT,
    HEIGHT_SOURCE_MEASURED_SUPPORT,
)

from test_top_support_estimator import (
    CONFIG,
    SIZES,
    YAWS,
    _full_scene,
)


def _detector(**kwargs):
    params = dict(
        config=CONFIG, support_mode="auto",
        stability_window=1, stability_max_z_spread=0.015)
    params.update(kwargs)
    return PlatformFreeDetector(**params)


def _scene(seed=21, drop_sides=(), support_z=0.90):
    w, d, h = SIZES[1]
    cargo, raw, top_z = _full_scene(
        w, d, h, YAWS[1], support_z, seed=seed, drop_sides=drop_sides)
    return cargo, raw, top_z, h, support_z


class TestPipelineStampAndState(unittest.TestCase):
    def test_matching_measure_raw_yields_full_3d(self):
        cargo, raw, top_z, h, support_z = _scene()
        det = _detector()
        out = det.update(cargo, raw, source="measure", geometry_ok=True,
                         raw_same_stamp=True)
        self.assertTrue(out.top_valid)
        self.assertTrue(out.height_valid)
        self.assertEqual(out.height_source, HEIGHT_SOURCE_MEASURED_SUPPORT)
        self.assertAlmostEqual(out.box.top.top_z, top_z, delta=0.01)
        self.assertAlmostEqual(out.box.height, h, delta=0.015)

    def test_hold_track_never_measured_support(self):
        cargo, raw, _, _, _ = _scene()
        det = _detector()
        out = det.update(cargo, raw, source="hold_track", geometry_ok=True,
                         raw_same_stamp=True)
        self.assertTrue(out.top_valid)
        self.assertFalse(out.height_valid)
        self.assertNotEqual(out.height_source, HEIGHT_SOURCE_MEASURED_SUPPORT)
        self.assertEqual(out.support_gate, "hold_track")

    def test_geometry_not_settled_skips_support(self):
        cargo, raw, _, _, _ = _scene()
        det = _detector()
        out = det.update(cargo, raw, source="measure", geometry_ok=False,
                         raw_same_stamp=True)
        self.assertTrue(out.top_valid)
        self.assertFalse(out.height_valid)
        self.assertEqual(out.support_gate, "geometry_not_settled")

    def test_stamp_mismatch_reason_code(self):
        cargo, raw, _, _, _ = _scene()
        det = _detector()
        out = det.update(cargo, raw, source="measure", geometry_ok=True,
                         raw_same_stamp=False)
        self.assertTrue(out.top_valid)
        self.assertFalse(out.height_valid)
        self.assertEqual(out.support_gate, "raw_stamp_mismatch")
        self.assertEqual(out.support.reason, DETECT_SUPPORT_STAMP_MISMATCH)

    def test_missing_raw_is_not_full_3d(self):
        cargo, raw, _, _, _ = _scene()
        det = _detector()
        out = det.update(cargo, None, source="measure", geometry_ok=True,
                         raw_same_stamp=True)
        self.assertTrue(out.top_valid)
        self.assertFalse(out.height_valid)

    def test_missing_cargo_cannot_detect(self):
        cargo, raw, _, _, _ = _scene()
        det = _detector()
        out = det.update(np.zeros((0, 3)), raw, source="measure",
                         geometry_ok=True, raw_same_stamp=True)
        self.assertFalse(out.top_valid)
        self.assertFalse(out.height_valid)

    def test_reset_drops_stability_history(self):
        cargo, raw, _, _, _ = _scene()
        det = _detector(stability_window=3)
        first = det.update(cargo, raw, source="measure", geometry_ok=True,
                           raw_same_stamp=True)
        self.assertFalse(first.height_valid)
        det.reset()
        second = det.update(cargo, raw, source="measure", geometry_ok=True,
                            raw_same_stamp=True)
        self.assertFalse(second.height_valid)
        self.assertIn(second.support.reason, (
            "DETECT_SUPPORT_UNSTABLE", "ok"))


class TestPipelineFaults(unittest.TestCase):
    def test_auto_top_z_invariant_to_platform_z(self):
        cargo, raw, top_z, _, _ = _scene(seed=30)
        det = _detector(support_mode="auto")
        none = det.update(cargo, raw, source="measure", geometry_ok=True,
                          raw_same_stamp=True, platform_z=None)
        det2 = _detector(support_mode="auto")
        high = det2.update(cargo, raw, source="measure", geometry_ok=True,
                           raw_same_stamp=True, platform_z=top_z + 0.20)
        det3 = _detector(support_mode="auto")
        low = det3.update(cargo, raw, source="measure", geometry_ok=True,
                          raw_same_stamp=True, platform_z=top_z - 0.20)
        self.assertAlmostEqual(none.box.top.top_z, high.box.top.top_z, delta=0.005)
        self.assertAlmostEqual(none.box.top.top_z, low.box.top.top_z, delta=0.005)
        self.assertEqual(none.height_source, HEIGHT_SOURCE_MEASURED_SUPPORT)
        self.assertEqual(high.height_source, HEIGHT_SOURCE_MEASURED_SUPPORT)

    def test_missing_support_is_top_only(self):
        cargo, raw, _, _, _ = _scene(
            seed=31, drop_sides=("+u", "-u", "+v", "-v"))
        det = _detector()
        out = det.update(cargo, raw, source="measure", geometry_ok=True,
                         raw_same_stamp=True)
        self.assertTrue(out.top_valid)
        self.assertFalse(out.height_valid)

    def test_configured_mode_uses_prior_and_labels_source(self):
        cargo, raw, top_z, h, support_z = _scene(
            seed=32, drop_sides=("+u", "-u", "+v", "-v"))
        det = _detector(support_mode="configured")
        out = det.update(cargo, raw, source="measure", geometry_ok=True,
                         raw_same_stamp=True, platform_z=support_z)
        self.assertTrue(out.height_valid)
        self.assertEqual(out.height_source, HEIGHT_SOURCE_CONFIGURED_SUPPORT)
        self.assertAlmostEqual(out.box.height, h, delta=0.02)

    def test_bad_configured_prior_not_used_in_auto(self):
        cargo, raw, top_z, h, support_z = _scene(seed=33)
        det = _detector(support_mode="auto")
        out = det.update(cargo, raw, source="measure", geometry_ok=True,
                         raw_same_stamp=True, platform_z=support_z + 0.20)
        self.assertEqual(out.height_source, HEIGHT_SOURCE_MEASURED_SUPPORT)
        self.assertAlmostEqual(out.box.height, h, delta=0.015)
        self.assertNotAlmostEqual(out.box.height, h - 0.20, delta=0.05)

    def test_catalog_prior_stays_invalid(self):
        cargo, raw, _, h, _ = _scene(
            seed=34, drop_sides=("+u", "-u", "+v", "-v"))
        det = _detector(
            support_mode="auto",
            catalog_entries=[{"size": (SIZES[1][0], SIZES[1][1], h)}],
            catalog_tolerance=0.08)
        out = det.update(cargo, None, source="measure", geometry_ok=True,
                         raw_same_stamp=True)
        self.assertTrue(out.top_valid)
        self.assertFalse(out.height_valid)
        self.assertEqual(out.height_source, HEIGHT_SOURCE_CATALOG_PRIOR)

    def test_support_modes_declared(self):
        self.assertEqual(
            set(SUPPORT_MODES),
            {"auto", "configured", "auto_then_configured", "top_only"})


if __name__ == "__main__":
    unittest.main()
