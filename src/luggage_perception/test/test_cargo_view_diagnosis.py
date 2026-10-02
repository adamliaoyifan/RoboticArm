#!/usr/bin/env python3
"""Empty-view reason order for a post-place cargo view."""

import unittest

import numpy as np

from luggage_perception.cargo_view_diagnosis import (
    REASON_BOX_CROP_EMPTY,
    REASON_DEPTH_INVALID,
    REASON_MAP_FILTERED,
    REASON_TARGET_NOT_IN_FRAME,
    REASON_TF_TIME_MISMATCH,
    classify_empty_view,
    project_box_corners,
    region_depth_stats,
    rim_occlusion_fraction,
)


def _seen(**overrides):
    diag = {
        "in_frame_fraction": 1.0,
        "behind_camera": False,
        "tf_stamped_ok": True,
        "tf_used_latest": False,
        "tf_fallback_age_sec": 0.0,
        "n_valid_depth_in_footprint": 80,
        "n_kept": 40,
        "published": True,
    }
    diag.update(overrides)
    return diag


class TestProject(unittest.TestCase):

    def test_center_corner_is_in_frame(self):
        out = project_box_corners(
            [(0.0, 0.0, 1.0)], fx=100.0, fy=100.0, cx=50.0, cy=40.0,
            width=100, height=80)
        self.assertEqual(out["in_frame"], 1)
        self.assertAlmostEqual(out["pixels"][0][0], 50.0)
        self.assertFalse(out["behind_camera"])

    def test_all_behind_camera(self):
        out = project_box_corners(
            [(-0.1, 0.0, -1.0), (0.1, 0.0, -1.0)],
            fx=100.0, fy=100.0, cx=50.0, cy=40.0, width=100, height=80)
        self.assertTrue(out["behind_camera"])
        self.assertEqual(out["in_frame"], 0)


class TestRegionDepth(unittest.TestCase):

    def test_counts_valid_zero_and_invalid(self):
        depth = np.zeros((6, 8), dtype=np.float64)
        depth[1:4, 1:5] = 1000.0
        depth[1, 1] = 0.0
        depth[2, 2] = np.nan
        polygon = [(1, 1), (4, 1), (4, 3), (1, 3)]
        stats = region_depth_stats(depth, polygon)
        self.assertGreater(stats["n_valid"], 0)
        self.assertEqual(stats["n_zero"], 1)
        self.assertEqual(stats["n_invalid"], 1)
        self.assertAlmostEqual(stats["median_m"], 1.0)

    def test_rim_fraction(self):
        fraction = rim_occlusion_fraction([0.4, 0.4, 1.2, 1.3], 1.0, 0.15)
        self.assertAlmostEqual(fraction, 0.5)

    def test_rim_accepts_a_numpy_vector(self):
        fraction = rim_occlusion_fraction(
            np.array([0.4, 1.2]), 1.0, 0.15)
        self.assertAlmostEqual(fraction, 0.5)


class TestClassify(unittest.TestCase):

    def test_out_of_frame_beats_invalid_depth(self):
        self.assertEqual(
            classify_empty_view(_seen(
                in_frame_fraction=0.0, n_valid_depth_in_footprint=0)),
            REASON_TARGET_NOT_IN_FRAME)

    def test_stale_tf_fallback(self):
        self.assertEqual(
            classify_empty_view(_seen(
                tf_stamped_ok=False, tf_used_latest=True,
                tf_fallback_age_sec=0.5)),
            REASON_TF_TIME_MISMATCH)

    def test_fresh_latest_tf_is_not_a_mismatch(self):
        self.assertNotEqual(
            classify_empty_view(_seen(
                tf_stamped_ok=False, tf_used_latest=True,
                tf_fallback_age_sec=0.05)),
            REASON_TF_TIME_MISMATCH)

    def test_depth_invalid(self):
        self.assertEqual(
            classify_empty_view(_seen(n_valid_depth_in_footprint=4)),
            REASON_DEPTH_INVALID)

    def test_box_crop_empty(self):
        self.assertEqual(
            classify_empty_view(_seen(n_kept=0)),
            REASON_BOX_CROP_EMPTY)

    def test_map_filtered_when_points_were_published(self):
        self.assertEqual(
            classify_empty_view(_seen()),
            REASON_MAP_FILTERED)


if __name__ == "__main__":
    unittest.main()
