#!/usr/bin/env python3
"""Unit tests for luggage_box_estimator – pure numpy, no ROS required."""

import math
import os
import sys
import unittest

import numpy as np

# Put the scripts directory on the path so we can import the module directly.

from luggage_perception.luggage_box_estimator import (  # noqa: E402
    BoxEstimate,
    _pca_rectangle,
    _refine_rectangle,
    estimate_box,
    match_catalog,
    yaw_valid_from_eigen_ratio,
)


def _synthetic_box_cloud(cx, cy, platform_z, width, depth, height, yaw,
                         n_top=500, noise_std=0.002, rng=None):
    """Generate a point cloud of the top surface of a box."""
    if rng is None:
        rng = np.random.RandomState(0)
    top_z = platform_z + height
    # Uniform grid on the top face, then rotate by yaw.
    u = rng.uniform(-width / 2, width / 2, n_top)
    v = rng.uniform(-depth / 2, depth / 2, n_top)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    x = cx + cos_y * u - sin_y * v + rng.normal(0, noise_std, n_top)
    y = cy + sin_y * u + cos_y * v + rng.normal(0, noise_std, n_top)
    z = np.full(n_top, top_z) + rng.normal(0, noise_std, n_top)
    return np.column_stack([x, y, z])


# Catalog matching helpers.
CATALOG = [
    {"id": "carryon", "size": [0.55, 0.40, 0.25]},
    {"id": "standard", "size": [0.70, 0.45, 0.28]},
    {"id": "large", "size": [0.80, 0.50, 0.32]},
]


class TestEstimateBoxBasic(unittest.TestCase):
    def test_percentile_trim_does_not_shrink_the_rectangle(self):
        """Trimming for outliers discards the points that define the edges.

        Uncompensated it measured a 0.70 x 0.45 face ~10 mm small, and
        underestimating a footprint is the dangerous direction: the map then
        believes the box is smaller than it is.
        """
        rng = np.random.RandomState(3)
        width, depth = 0.70, 0.45
        points = np.column_stack([
            rng.uniform(-width * 0.5, width * 0.5, 4000),
            rng.uniform(-depth * 0.5, depth * 0.5, 4000),
        ])
        _yaw, extent0, extent1, _center = _refine_rectangle(points, 0.0)
        self.assertAlmostEqual(max(extent0, extent1), width, delta=0.004)
        self.assertAlmostEqual(min(extent0, extent1), depth, delta=0.004)

    def test_trim_compensation_still_rejects_outliers(self):
        """A few stray points must not blow the extent up."""
        rng = np.random.RandomState(4)
        width, depth = 0.70, 0.45
        clean = np.column_stack([
            rng.uniform(-width * 0.5, width * 0.5, 4000),
            rng.uniform(-depth * 0.5, depth * 0.5, 4000),
        ])
        outliers = np.array([[2.0, 0.0], [-2.0, 0.0], [0.0, 1.5]])
        points = np.vstack([clean, outliers])
        _yaw, extent0, extent1, _center = _refine_rectangle(points, 0.0)
        self.assertLess(max(extent0, extent1), width + 0.02)

    def test_rectangle_refinement_rejects_point_density_bias(self):
        rng = np.random.RandomState(17)
        yaw = 0.31
        local = np.column_stack((
            rng.uniform(-0.4, 0.4, 2000),
            rng.uniform(-0.25, 0.25, 2000),
        ))
        dense_corner = np.column_stack((
            rng.normal(0.32, 0.02, 1200),
            rng.normal(0.18, 0.02, 1200),
        ))
        local = np.vstack((local, dense_corner))
        rotation = np.array([
            [math.cos(yaw), -math.sin(yaw)],
            [math.sin(yaw), math.cos(yaw)],
        ])
        points = local.dot(rotation.T) + np.array([-1.0, 0.1])
        pca_yaw, _width, _depth, _ratio = _pca_rectangle(points)
        refined_yaw, _width, _depth, center = _refine_rectangle(
            points, pca_yaw)
        error = abs(math.atan2(
            math.sin(2.0 * (refined_yaw - yaw)),
            math.cos(2.0 * (refined_yaw - yaw))) * 0.5)
        self.assertLess(error, math.radians(1.0))
        self.assertAlmostEqual(center[0], -1.0, delta=0.02)
        self.assertAlmostEqual(center[1], 0.1, delta=0.02)

    """Basic estimation on a clean synthetic top surface."""

    def test_axis_aligned_standard(self):
        """Standard box at yaw=0 sitting on platform_z=0.86."""
        cloud = _synthetic_box_cloud(
            cx=-1.0, cy=0.0, platform_z=0.86,
            width=0.70, depth=0.45, height=0.28, yaw=0.0,
        )
        est = estimate_box(
            cloud,
            roi_center_xy=(-1.0, 0.0), roi_margin=0.6,
            platform_z=0.86, catalog_entries=CATALOG,
        )
        self.assertIsNotNone(est)
        self.assertAlmostEqual(est.center_xyz[0], -1.0, delta=0.02)
        self.assertAlmostEqual(est.center_xyz[1], 0.0, delta=0.02)
        expected_cz = 0.86 + 0.28 / 2
        self.assertAlmostEqual(est.center_xyz[2], expected_cz, delta=0.02)
        self.assertEqual(est.matched_catalog_id, "standard")
        self.assertAlmostEqual(est.width, 0.70, delta=0.01)
        self.assertAlmostEqual(est.depth, 0.45, delta=0.01)
        self.assertAlmostEqual(est.height, 0.28, delta=0.01)
        self.assertTrue(est.yaw_valid)
        self.assertGreater(est.aspect_ratio, 1.4)

    def test_yaw_45_degrees(self):
        """Box rotated 45 degrees; pose and dims should still match."""
        yaw = math.pi / 4
        cloud = _synthetic_box_cloud(
            cx=-1.0, cy=0.0, platform_z=0.86,
            width=0.70, depth=0.45, height=0.28, yaw=yaw,
        )
        est = estimate_box(
            cloud,
            roi_center_xy=(-1.0, 0.0), roi_margin=0.6,
            platform_z=0.86, catalog_entries=CATALOG,
        )
        self.assertIsNotNone(est)
        # PCA has a 180° direction ambiguity and width/depth can swap adding
        # another 90°.  Reduce the difference modulo π/2.
        est_yaw = math.atan2(
            2 * (est.quaternion_xyzw[3] * est.quaternion_xyzw[2]),
            1 - 2 * est.quaternion_xyzw[2] ** 2,
        )
        diff = est_yaw - yaw
        diff = math.atan2(math.sin(diff), math.cos(diff))  # wrap to [-π, π]
        diff_mod = abs(math.atan2(math.sin(2 * diff), math.cos(2 * diff))) / 2
        self.assertLess(diff_mod, 0.15,
                        "yaw mismatch: est=%.3f expected=%.3f" % (est_yaw, yaw))

    def test_large_box(self):
        cloud = _synthetic_box_cloud(
            cx=-0.95, cy=0.05, platform_z=0.86,
            width=0.80, depth=0.50, height=0.32, yaw=0.3,
        )
        est = estimate_box(
            cloud,
            roi_center_xy=(-1.0, 0.0), roi_margin=0.6,
            platform_z=0.86, catalog_entries=CATALOG,
        )
        self.assertIsNotNone(est)
        self.assertEqual(est.matched_catalog_id, "large")

    def test_xy_offset_detection(self):
        """Box offset from the nominal centre by ±0.08 m."""
        cloud = _synthetic_box_cloud(
            cx=-1.08, cy=0.06, platform_z=0.86,
            width=0.55, depth=0.40, height=0.25, yaw=0.0,
        )
        est = estimate_box(
            cloud,
            roi_center_xy=(-1.0, 0.0), roi_margin=0.5,
            platform_z=0.86, catalog_entries=CATALOG,
        )
        self.assertIsNotNone(est)
        self.assertAlmostEqual(est.center_xyz[0], -1.08, delta=0.03)
        self.assertAlmostEqual(est.center_xyz[1], 0.06, delta=0.03)
        self.assertEqual(est.matched_catalog_id, "carryon")

    def test_raw_depth_platform_plane_does_not_dominate_box_top(self):
        """A dense platform plane must not beat the higher suitcase top."""
        rng = np.random.RandomState(9)
        box = _synthetic_box_cloud(
            cx=-1.04, cy=0.05, platform_z=0.86,
            width=0.70, depth=0.45, height=0.28, yaw=0.35,
            n_top=350, rng=rng,
        )
        platform = np.column_stack([
            rng.uniform(-1.45, -0.55, 2500),
            rng.uniform(-0.45, 0.45, 2500),
            np.full(2500, 0.86) + rng.normal(0, 0.001, 2500),
        ])
        est = estimate_box(
            np.vstack([platform, box]),
            roi_center_xy=(-1.0, 0.0),
            roi_margin=0.5,
            platform_z=0.86,
            catalog_entries=CATALOG,
        )
        self.assertIsNotNone(est)
        self.assertEqual(est.matched_catalog_id, "standard")
        self.assertAlmostEqual(est.center_xyz[0], -1.04, delta=0.03)
        self.assertAlmostEqual(est.center_xyz[1], 0.05, delta=0.03)


class TestEstimateBoxEdgeCases(unittest.TestCase):

    def test_too_few_points(self):
        pts = np.random.rand(10, 3)
        est = estimate_box(pts, min_points=50)
        self.assertIsNone(est)

    def test_empty_array(self):
        est = estimate_box(np.empty((0, 3)))
        self.assertIsNone(est)

    def test_no_roi_filter(self):
        """Estimation works without an ROI centre."""
        cloud = _synthetic_box_cloud(
            cx=0.0, cy=0.0, platform_z=0.0,
            width=0.70, depth=0.45, height=0.28, yaw=0.0,
        )
        est = estimate_box(cloud, roi_center_xy=None, platform_z=0.0)
        self.assertIsNotNone(est)
        self.assertAlmostEqual(est.width, 0.70, delta=0.03)

    def test_no_catalog(self):
        cloud = _synthetic_box_cloud(
            cx=0.0, cy=0.0, platform_z=0.0,
            width=0.70, depth=0.45, height=0.28, yaw=0.0,
        )
        est = estimate_box(cloud, platform_z=0.0, catalog_entries=None)
        self.assertIsNotNone(est)
        self.assertIsNone(est.matched_catalog_id)
        self.assertAlmostEqual(est.width, 0.70, delta=0.03)

    def test_noisy_cloud(self):
        """Higher noise should still produce a reasonable estimate."""
        cloud = _synthetic_box_cloud(
            cx=-1.0, cy=0.0, platform_z=0.86,
            width=0.70, depth=0.45, height=0.28, yaw=0.0,
            noise_std=0.005, n_top=800,
        )
        est = estimate_box(
            cloud,
            roi_center_xy=(-1.0, 0.0), roi_margin=0.6,
            platform_z=0.86, catalog_entries=CATALOG,
            ransac_dist_thresh=0.015,
        )
        self.assertIsNotNone(est)
        self.assertAlmostEqual(est.center_xyz[0], -1.0, delta=0.04)


class TestCatalogMatching(unittest.TestCase):

    def test_exact_match(self):
        mid, w, d, h = match_catalog(0.70, 0.45, 0.28, CATALOG)
        self.assertEqual(mid, "standard")
        self.assertAlmostEqual(w, 0.70)

    def test_swapped_axes(self):
        """Width and depth may be swapped; matcher should handle it."""
        mid, w, d, h = match_catalog(0.45, 0.70, 0.28, CATALOG)
        self.assertEqual(mid, "standard")

    def test_no_match(self):
        mid, w, d, h = match_catalog(1.20, 0.80, 0.50, CATALOG, tolerance=0.05)
        self.assertIsNone(mid)
        self.assertAlmostEqual(w, 1.20)

    def test_within_tolerance(self):
        mid, w, d, h = match_catalog(0.72, 0.46, 0.29, CATALOG, tolerance=0.05)
        self.assertEqual(mid, "standard")
        self.assertAlmostEqual(w, 0.70)


class TestYawValidity(unittest.TestCase):

    def test_eigen_ratio_threshold(self):
        self.assertFalse(yaw_valid_from_eigen_ratio(1.19))
        self.assertTrue(yaw_valid_from_eigen_ratio(1.2))
        self.assertFalse(yaw_valid_from_eigen_ratio(float("nan")))

    def test_square_top_face_marks_yaw_invalid(self):
        cloud = _synthetic_box_cloud(
            cx=0.0, cy=0.0, platform_z=0.0,
            width=0.50, depth=0.50, height=0.25, yaw=0.4,
            n_top=800, noise_std=0.001,
        )
        est = estimate_box(cloud, platform_z=0.0, catalog_entries=None)
        self.assertIsNotNone(est)
        self.assertFalse(est.yaw_valid)
        self.assertAlmostEqual(est.aspect_ratio, 1.0, delta=0.08)

    def test_elongated_pca_eigen_ratio_is_high(self):
        rng = np.random.RandomState(0)
        points = np.column_stack([
            rng.uniform(-0.35, 0.35, 2000),
            rng.uniform(-0.225, 0.225, 2000),
        ])
        _yaw, _e0, _e1, ratio = _pca_rectangle(points)
        self.assertGreater(ratio, 1.2)
        self.assertTrue(yaw_valid_from_eigen_ratio(ratio))


if __name__ == "__main__":
    unittest.main()


class TestVoxelDownsample(unittest.TestCase):
    """voxel_size parameter behavior (performance fix, 2026-08-27)."""

    @staticmethod
    def _dense_box_top(width=0.6, depth=0.4, z=1.115, grid=0.004,
                       x0=-1.0, y0=0.0, noise=0.0008, seed=7):
        """Dense synthetic box-top rectangle plus supporting box sides."""
        rng = np.random.RandomState(seed)
        xs = np.arange(x0 - width / 2, x0 + width / 2, grid)
        ys = np.arange(y0 - depth / 2, y0 + depth / 2, grid)
        gx, gy = np.meshgrid(xs, ys)
        top = np.column_stack((gx.ravel(), gy.ravel(),
                               np.full(gx.size, z)))
        top[:, 2] += rng.uniform(-noise, noise, len(top))
        return top

    def test_voxel_on_off_center_size_invariance(self):
        from luggage_perception.luggage_box_estimator import estimate_box
        pts = self._dense_box_top()
        out0 = estimate_box(pts, roi_center_xy=(-1.0, 0.0), roi_margin=0.5,
                            platform_z=0.86, voxel_size=0.0)
        out1 = estimate_box(pts, roi_center_xy=(-1.0, 0.0), roi_margin=0.5,
                            platform_z=0.86, voxel_size=0.01)
        self.assertIsNotNone(out0)
        self.assertIsNotNone(out1)
        # Center agreement within a centimeter.
        self.assertLess(
            np.linalg.norm(out1.center_xyz - out0.center_xyz), 0.01)
        # Size agreement within 2 cm per axis.
        self.assertLess(abs(out1.width - out0.width), 0.02)
        self.assertLess(abs(out1.depth - out0.depth), 0.02)
        self.assertLess(abs(out1.height - out0.height), 0.01)

    def test_higher_box_top_beats_lower_large_platform(self):
        """Guard for the height-dominant selection rule: a big low plane
        (platform) must not beat a smaller higher box top, with voxel on."""
        from luggage_perception.luggage_box_estimator import estimate_box
        top = self._dense_box_top(width=0.5, depth=0.35)
        xs = np.arange(-1.6, -0.4, 0.004)
        ys = np.arange(-0.55, 0.55, 0.004)
        gx, gy = np.meshgrid(xs, ys)
        platform = np.column_stack((gx.ravel(), gy.ravel(),
                                    np.full(gx.size, 0.86)))
        pts = np.vstack((platform, top))
        out = estimate_box(pts, roi_center_xy=(-1.0, 0.0), roi_margin=0.7,
                           platform_z=0.86, voxel_size=0.01)
        self.assertIsNotNone(out)
        # Box top near 1.115, not the platform at 0.86: center z reflects
        # plane_z - height/2 with height = plane_z - 0.86.
        self.assertAlmostEqual(out.center_xyz[2], 1.115 - (1.115 - 0.86) / 2,
                               delta=0.02)

    def test_panel_mixture_boundary_documents_height_dominance(self):
        """Two sides of the higher-plane (suction-panel) risk boundary.

        A plane only wins via the height-first rule if RANSAC *proposes* it,
        i.e. draws 3 of its points within max_iter. A min_inliers-sized
        cluster (25 points among ~4k) is effectively never sampled, so tiny
        higher planes are filtered by proposal probability; a substantial
        higher plane (~30% of the cloud) is proposed with near certainty
        and then wins on height regardless of counts. Documents the rule,
        not desired behavior; see _ransac_horizontal_plane."""
        from luggage_perception.luggage_box_estimator import estimate_box
        top = self._dense_box_top(width=0.5, depth=0.35)
        rng = np.random.RandomState(3)

        def _plane(n, x_lo, x_hi, y_lo, y_hi, z):
            return np.column_stack((
                rng.uniform(x_lo, x_hi, n),
                rng.uniform(y_lo, y_hi, n),
                np.full(n, z),
            ))

        # Side A: 25 co-planar points 30 cm above the box (min_inliers
        # count) -> selection stays on the box top.
        pts_a = np.vstack((top, _plane(25, -1.3, -0.7, -0.2, 0.2, 1.42)))
        out_a = estimate_box(pts_a, roi_center_xy=(-1.0, 0.0), roi_margin=0.5,
                             platform_z=0.86, voxel_size=0.01)
        self.assertIsNotNone(out_a)
        self.assertLess(out_a.center_xyz[2], 1.05)  # box top, not 1.42

        # Side B: a substantial higher plane (~2500 points, like a visible
        # horizontal panel surface) -> height rule wins over box-top counts.
        pts_b = np.vstack((
            top,
            _plane(2500, -1.3, -0.7, -0.25, 0.25, 1.42),
        ))
        out_b = estimate_box(pts_b, roi_center_xy=(-1.0, 0.0), roi_margin=0.5,
                             platform_z=0.86, voxel_size=0.01)
        self.assertIsNotNone(out_b)
        # Winner is the 1.42 plane: center_z = plane_z - height/2 with
        # height = 1.42 - 0.86, i.e. ~1.14 (vs the box-top's ~0.99).
        self.assertGreater(out_b.center_xyz[2], 1.08)
        self.assertLess(out_b.center_xyz[2], 1.25)

    def test_coarse_voxel_starves_count_based_confidence(self):
        """Documents why voxel_size is capped at ~0.02 m: confidence is
        count-based and a coarse voxel drops it below typical gates."""
        from luggage_perception.luggage_box_estimator import estimate_box
        pts = self._dense_box_top(width=0.5, depth=0.35)
        out_fine = estimate_box(pts, roi_center_xy=(-1.0, 0.0), roi_margin=0.5,
                                platform_z=0.86, voxel_size=0.01)
        out_coarse = estimate_box(pts, roi_center_xy=(-1.0, 0.0),
                                  roi_margin=0.5, platform_z=0.86,
                                  voxel_size=0.05)
        self.assertIsNotNone(out_fine)
        self.assertIsNotNone(out_coarse)
        self.assertGreaterEqual(out_fine.confidence, 0.99)
        self.assertLess(out_coarse.confidence, 0.7)
