#!/usr/bin/env python3
"""PF-G3A: raw-only cargo input fails closed (PF-R2).

docs/plans/platform_free_height_remediation.md. An unsegmented raw depth
cloud is not a luggage observation: the dominant 0.86 m pickup platform
must never become a valid luggage top, in any support mode, with or
without a configured platform_z.
"""

import math
import unittest

import numpy as np

from luggage_perception.platform_free_pipeline import PlatformFreeDetector
from luggage_perception.top_support_estimator import (
    DETECT_CARGO_SEGMENTATION_REQUIRED,
    GEOMETRY_FULL_3D,
    GEOMETRY_TOP_ONLY,
    HEIGHT_SOURCE_UNAVAILABLE,
    TopSupportConfig,
    estimate_top_surface,
)

CONFIG = TopSupportConfig(
    workspace_center_xy=(0.0, 0.0),
    workspace_half_extents=(1.2, 1.2),
    min_top_points=40,
    min_support_points=40,
    min_inliers_per_side=8,
)
PLATFORM_Z = 0.86


def _platform_scene(rng_seed=0, with_box=True):
    """Unsegmented raw scene: pickup platform plane (+ optional box).

    Mimics what the raw depth topic delivers: platform surface points
    dominate, a box may sit on it.
    """
    rng = np.random.RandomState(rng_seed)
    planes = [(
        np.column_stack([
            rng.uniform(-0.6, 0.6, 4000),
            rng.uniform(-0.6, 0.6, 4000),
            np.full(4000, PLATFORM_Z),
        ]))]
    if with_box:
        planes.append(np.column_stack([
            rng.uniform(-0.275, 0.275, 400) + 0.1,
            rng.uniform(-0.175, 0.175, 400) - 0.05,
            np.full(400, PLATFORM_Z + 0.45),
        ]))
    return np.vstack(planes)


class TestRawOnlyFailsClosed(unittest.TestCase):

    def test_unsegmented_platform_is_a_valid_top_without_the_gate(self):
        """Hazard documentation: the raw estimator *would* fit the platform.

        On a platform-only raw scene the highest supported plane is the
        0.86 m platform itself — this is exactly why raw input must be
        rejected before fitting rather than filtered after.
        """
        est = estimate_top_surface(_platform_scene(with_box=False),
                                   ((0.0, 0.0), (1.2, 1.2)), CONFIG)
        self.assertIsNotNone(est)
        self.assertAlmostEqual(est.top_z, PLATFORM_Z, delta=0.01)

    def test_raw_only_auto_never_fits(self):
        det = PlatformFreeDetector(CONFIG, support_mode="auto",
                                   stability_window=2)
        raw = _platform_scene()
        result = det.update(raw, raw, source="measure", geometry_ok=True,
                            cargo_segmented=False)
        self.assertFalse(result.top_valid)
        self.assertEqual(result.top_reason,
                         DETECT_CARGO_SEGMENTATION_REQUIRED)
        self.assertFalse(result.height_valid)
        self.assertEqual(result.height_source, HEIGHT_SOURCE_UNAVAILABLE)
        self.assertIsNone(result.box)

    def test_raw_only_fails_in_every_support_mode(self):
        raw = _platform_scene()
        for mode in ("auto", "auto_then_configured", "configured",
                     "top_only"):
            det = PlatformFreeDetector(CONFIG, support_mode=mode,
                                       stability_window=2)
            result = det.update(
                raw, raw, source="measure", geometry_ok=True,
                platform_z=PLATFORM_Z, cargo_segmented=False)
            self.assertFalse(result.top_valid, mode)
            self.assertEqual(result.top_reason,
                             DETECT_CARGO_SEGMENTATION_REQUIRED, mode)
            self.assertFalse(result.height_valid, mode)

    def test_configured_platform_z_cannot_promote_raw_points(self):
        """A configured support Z must not turn platform points into cargo."""
        det = PlatformFreeDetector(CONFIG, support_mode="configured",
                                   stability_window=2)
        result = det.update(
            _platform_scene(with_box=False), None, source="measure",
            geometry_ok=True, platform_z=PLATFORM_Z,
            cargo_segmented=False)
        self.assertEqual(result.top_reason,
                         DETECT_CARGO_SEGMENTATION_REQUIRED)
        self.assertFalse(result.height_valid)

    def test_segmented_input_still_reaches_the_estimator(self):
        """use_semantic=true (cargo_segmented=True) keeps the normal path."""
        rng = np.random.RandomState(7)
        w, d, h = 0.55, 0.35, 0.45
        u = rng.uniform(-w / 2, w / 2, 500)
        v = rng.uniform(-d / 2, d / 2, 500)
        cargo = np.column_stack([u + 0.1, v - 0.05,
                                 np.full(500, PLATFORM_Z + h)])
        det = PlatformFreeDetector(CONFIG, support_mode="auto",
                                   stability_window=1)
        result = det.update(cargo, _platform_scene(), source="measure",
                            geometry_ok=True, cargo_segmented=True)
        self.assertTrue(result.top_valid)
        self.assertNotEqual(result.top_reason,
                            DETECT_CARGO_SEGMENTATION_REQUIRED)
        self.assertAlmostEqual(result.box.top.top_z, PLATFORM_Z + h,
                               delta=0.01)

    def test_no_full_3d_or_valid_height_from_raw_only(self):
        det = PlatformFreeDetector(CONFIG, support_mode="auto",
                                   stability_window=2)
        for _ in range(5):
            result = det.update(_platform_scene(), _platform_scene(),
                                source="measure", geometry_ok=True,
                                cargo_segmented=False)
            self.assertFalse(result.height_valid)
            if result.box is not None:
                self.assertEqual(result.box.geometry_level,
                                 GEOMETRY_TOP_ONLY)
                self.assertEqual(result.box.height_source,
                                 HEIGHT_SOURCE_UNAVAILABLE)
        self.assertEqual(det._stability.copy_output(), None)


if __name__ == "__main__":
    unittest.main()
