#!/usr/bin/env python3
"""Unit tests for top_support_estimator – pure numpy, no ROS required.

Gate 1 of docs/plans/platform_free_height_test_plan.md: synthetic scenes
exercise the top/support split without any platform_z prior. Platform
heights deliberately differ from the legacy 0.86 m simulation value.
"""

import math
import unittest

import numpy as np

from luggage_perception.top_support_estimator import (
    DETECT_HEIGHT_PRIOR_ONLY,
    DETECT_SUPPORT_UNOBSERVABLE,
    DETECT_SUPPORT_UNSTABLE,
    DETECT_TOP_UNOBSERVABLE,
    GEOMETRY_FULL_3D,
    GEOMETRY_TOP_ONLY,
    HEIGHT_SOURCE_CATALOG_PRIOR,
    HEIGHT_SOURCE_CONFIGURED_SUPPORT,
    HEIGHT_SOURCE_MEASURED_SUPPORT,
    HEIGHT_SOURCE_UNAVAILABLE,
    SupportStabilityFilter,
    TopSupportConfig,
    compose_box_geometry,
    estimate_local_support,
    estimate_top_surface,
)

# Three catalog-ish sizes: small carry-on, medium, large check-in.
SIZES = [(0.40, 0.25, 0.30), (0.55, 0.35, 0.45), (0.70, 0.45, 0.55)]
YAWS = [0.0, math.radians(35.0), math.radians(-70.0)]
# Platform heights deliberately != 0.86 m (the legacy sim truth).
SUPPORT_ZS = [0.62, 0.90, 1.10]

WORKSPACE = ((0.0, 0.0), (1.2, 1.2))
CONFIG = TopSupportConfig(
    workspace_center_xy=WORKSPACE[0],
    workspace_half_extents=WORKSPACE[1],
    min_luggage_height=0.15,
    max_luggage_height=0.60,
    min_top_points=40,
    min_support_points=40,
    min_support_sides=2,
    min_inliers_per_side=8,
)


def _rng(seed):
    return np.random.RandomState(seed)


def _top_face(cx, cy, top_z, width, depth, yaw, rng,
              n=600, noise_std=0.003, u_range=None, v_range=None):
    """Uniform top-face samples in the box's rotated frame."""
    u_lo, u_hi = u_range or (-width / 2, width / 2)
    v_lo, v_hi = v_range or (-depth / 2, depth / 2)
    u = rng.uniform(u_lo, u_hi, n)
    v = rng.uniform(v_lo, v_hi, n)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    x = cx + cos_y * u - sin_y * v + rng.normal(0, noise_std, n)
    y = cy + sin_y * u + cos_y * v + rng.normal(0, noise_std, n)
    z = np.full(n, top_z) + rng.normal(0, noise_std, n)
    return np.column_stack([x, y, z])


def _support_ring(cx, cy, support_z, width, depth, yaw, rng,
                  inner_margin=0.03, outer_margin=0.18,
                  n=2000, noise_std=0.003, drop_sides=()):
    """Annulus of platform points around the footprint.

    ``drop_sides`` skips points near the given rectangle edges
    ("+u", "-u", "+v", "-v") to synthesize occlusion.
    """
    u = rng.uniform(
        -(width / 2 + outer_margin), width / 2 + outer_margin, n)
    v = rng.uniform(
        -(depth / 2 + outer_margin), depth / 2 + outer_margin, n)
    in_annulus = ((np.abs(u) > width / 2 + inner_margin)
                  | (np.abs(v) > depth / 2 + inner_margin))
    keep = in_annulus.copy()
    for side in drop_sides:
        if side == "+u":
            keep &= ~(u > width / 2 + inner_margin)
        elif side == "-u":
            keep &= ~(u < -(width / 2 + inner_margin))
        elif side == "+v":
            keep &= ~(v > depth / 2 + inner_margin)
        elif side == "-v":
            keep &= ~(v < -(depth / 2 + inner_margin))
    u, v = u[keep], v[keep]
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    x = cx + cos_y * u - sin_y * v + rng.normal(0, noise_std, len(u))
    y = cy + sin_y * u + cos_y * v + rng.normal(0, noise_std, len(u))
    z = np.full(len(u), support_z) + rng.normal(0, noise_std, len(u))
    return np.column_stack([x, y, z])


def _side_walls(cx, cy, support_z, top_z, width, depth, yaw, rng,
                n=300, noise_std=0.003):
    """Sparse vertical side samples (what RGB-D typically sees of walls)."""
    t = rng.uniform(0.0, 1.0, n)
    side = rng.randint(0, 4, n)
    u = np.where(side == 0, width / 2, np.where(
        side == 1, -width / 2, rng.uniform(-width / 2, width / 2, n)))
    v = np.where(side == 2, depth / 2, np.where(
        side == 3, -depth / 2, rng.uniform(-depth / 2, depth / 2, n)))
    z = support_z + t * (top_z - support_z)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    x = cx + cos_y * u - sin_y * v + rng.normal(0, noise_std, n)
    y = cy + sin_y * u + cos_y * v + rng.normal(0, noise_std, n)
    return np.column_stack([x, y, z])


def _add_outliers(points, fraction, rng, spread=0.15):
    """Displace a fraction of points by a large offset (depth flyers)."""
    out = points.copy()
    n_out = int(len(out) * fraction)
    if n_out == 0:
        return out
    idx = rng.choice(len(out), n_out, replace=False)
    out[idx] += rng.uniform(-spread, spread, (n_out, 3))
    return out


def _full_scene(width, depth, height, yaw, support_z, seed=0,
                cx=0.1, cy=-0.05, drop_sides=(), top_noise=0.003,
                outlier_fraction=0.0):
    rng = _rng(seed)
    top_z = support_z + height
    cargo = _top_face(cx, cy, top_z, width, depth, yaw, rng,
                      noise_std=top_noise)
    if outlier_fraction:
        cargo = _add_outliers(cargo, outlier_fraction, rng)
    raw = np.vstack([
        cargo,
        _side_walls(cx, cy, support_z, top_z, width, depth, yaw, rng),
        _support_ring(cx, cy, support_z, width, depth, yaw, rng,
                      drop_sides=drop_sides),
    ])
    return cargo, raw, top_z


class TestEstimateTopSurface(unittest.TestCase):
    def test_sizes_yaws_supports(self):
        """3 sizes x 3 yaws x 3 support heights: top Z and footprint."""
        for i, (w, d, h) in enumerate(SIZES):
            for j, yaw in enumerate(YAWS):
                for k, support_z in enumerate(SUPPORT_ZS):
                    cargo, raw, top_z = _full_scene(
                        w, d, h, yaw, support_z, seed=10 * i + j + k)
                    est = estimate_top_surface(cargo, WORKSPACE, CONFIG)
                    self.assertIsNotNone(est, (w, d, h, yaw, support_z))
                    self.assertAlmostEqual(est.top_z, top_z, delta=0.01)
                    self.assertAlmostEqual(
                        est.center_xy[0], 0.1, delta=0.02)
                    self.assertAlmostEqual(
                        est.center_xy[1], -0.05, delta=0.02)
                    self.assertAlmostEqual(
                        max(est.width, est.depth), max(w, d), delta=0.03)
                    self.assertAlmostEqual(
                        min(est.width, est.depth), min(w, d), delta=0.03)
                    # Rectangle yaw is ambiguous mod 90 deg (axis swap) and
                    # mod 180 deg (axis direction): reduce into [0, 45 deg].
                    yaw_err = abs(math.atan2(
                        math.sin(est.yaw - yaw), math.cos(est.yaw - yaw)))
                    yaw_err = math.fmod(yaw_err, math.pi / 2)
                    yaw_err = min(yaw_err, math.pi / 2 - yaw_err)
                    self.assertLess(yaw_err, math.radians(6))

    def test_outliers_and_noise(self):
        """10% outliers + 3 mm noise still recover the top plane."""
        w, d, h = SIZES[1]
        yaw = YAWS[1]
        cargo, raw, top_z = _full_scene(
            w, d, h, yaw, 0.62, seed=3, top_noise=0.003,
            outlier_fraction=0.10)
        est = estimate_top_surface(cargo, WORKSPACE, CONFIG)
        self.assertIsNotNone(est)
        self.assertAlmostEqual(est.top_z, top_z, delta=0.015)

    def test_lower_floor_excluded(self):
        """A lower floor far below the height band is not the support."""
        w, d, h = SIZES[1]
        cargo, raw, top_z = _full_scene(w, d, h, YAWS[0], 0.62, seed=4)
        rng = _rng(99)
        # Ground plane at support_z - 1.5 m with dense points.
        floor = _support_ring(
            0.1, -0.05, 0.62 - 1.5, w, d, YAWS[0], rng, n=4000)
        raw = np.vstack([raw, floor])
        est = estimate_local_support(
            raw, estimate_top_surface(cargo, WORKSPACE, CONFIG),
            WORKSPACE, CONFIG)
        self.assertEqual(est.reason, "ok")
        self.assertAlmostEqual(est.support_z, 0.62, delta=0.01)

    def test_too_few_points(self):
        est = estimate_top_surface(np.zeros((5, 3)), WORKSPACE, CONFIG)
        self.assertIsNone(est)

    def test_nonfinite_rejected(self):
        # Only 10 finite points survive the NaN filter (< min_top_points).
        points = np.full((600, 3), np.nan)
        points[:10] = np.random.RandomState(0).uniform(-0.3, 0.3, (10, 3))
        points[:10, 2] = 1.0
        est = estimate_top_surface(points, WORKSPACE, CONFIG)
        self.assertIsNone(est)

    def test_workspace_crop(self):
        """Points outside the pickup workspace are ignored."""
        w, d, h = SIZES[0]
        cargo, raw, top_z = _full_scene(w, d, h, 0.0, 0.62, seed=5,
                                        cx=5.0, cy=5.0)
        est = estimate_top_surface(cargo, WORKSPACE, CONFIG)
        self.assertIsNone(est)


class TestEstimateLocalSupport(unittest.TestCase):
    def test_full_ring_measured(self):
        for i, (w, d, h) in enumerate(SIZES):
            for j, yaw in enumerate(YAWS):
                support_z = SUPPORT_ZS[i]
                cargo, raw, top_z = _full_scene(
                    w, d, h, yaw, support_z, seed=20 + i * 3 + j)
                top = estimate_top_surface(cargo, WORKSPACE, CONFIG)
                support = estimate_local_support(raw, top, WORKSPACE,
                                                 CONFIG)
                self.assertEqual(support.reason, "ok", (w, d, yaw))
                self.assertAlmostEqual(
                    support.support_z, support_z, delta=0.012)
                self.assertGreaterEqual(support.side_coverage, 0.5)
                self.assertGreater(support.inlier_count, 0)

    def test_one_side_missing(self):
        """Occlusion removing one rectangle edge still measures support."""
        w, d, h = SIZES[1]
        yaw = YAWS[1]
        cargo, raw, _ = _full_scene(w, d, h, yaw, 0.90, seed=6,
                                    drop_sides=("+v",))
        top = estimate_top_surface(cargo, WORKSPACE, CONFIG)
        support = estimate_local_support(raw, top, WORKSPACE, CONFIG)
        self.assertEqual(support.reason, "ok")
        self.assertAlmostEqual(support.support_z, 0.90, delta=0.012)

    def test_two_sides_missing_unstable(self):
        """Only two adjacent sides visible -> below min_support_sides=2
        only when coverage collapses; with one side it must fail."""
        w, d, h = SIZES[1]
        yaw = YAWS[1]
        cargo, raw, _ = _full_scene(w, d, h, yaw, 0.90, seed=7,
                                    drop_sides=("+u", "-u", "+v"))
        top = estimate_top_surface(cargo, WORKSPACE, CONFIG)
        support = estimate_local_support(raw, top, WORKSPACE, CONFIG)
        self.assertEqual(support.reason, DETECT_SUPPORT_UNSTABLE)

    def test_all_sides_missing(self):
        """Fully occluded support -> UNOBSERVABLE, never a synthesized Z."""
        w, d, h = SIZES[1]
        yaw = YAWS[0]
        cargo, raw, _ = _full_scene(w, d, h, yaw, 0.62, seed=8,
                                    drop_sides=("+u", "-u", "+v", "-v"))
        top = estimate_top_surface(cargo, WORKSPACE, CONFIG)
        support = estimate_local_support(raw, top, WORKSPACE, CONFIG)
        self.assertEqual(support.reason, DETECT_SUPPORT_UNOBSERVABLE)
        self.assertFalse(np.isfinite(support.support_z))

    def test_deterministic(self):
        """Same input twice -> identical estimate (fixed RANSAC seed)."""
        w, d, h = SIZES[2]
        cargo, raw, _ = _full_scene(w, d, h, YAWS[2], 1.10, seed=9)
        top = estimate_top_surface(cargo, WORKSPACE, CONFIG)
        s1 = estimate_local_support(raw, top, WORKSPACE, CONFIG)
        s2 = estimate_local_support(raw, top, WORKSPACE, CONFIG)
        self.assertEqual(s1.reason, s2.reason)
        self.assertEqual(s1.support_z, s2.support_z)
        self.assertEqual(s1.inlier_count, s2.inlier_count)


class TestComposeBoxGeometry(unittest.TestCase):
    def test_measured_support(self):
        w, d, h = SIZES[1]
        support_z = 0.90
        cargo, raw, top_z = _full_scene(w, d, h, YAWS[1], support_z,
                                        seed=11)
        top = estimate_top_surface(cargo, WORKSPACE, CONFIG)
        support = estimate_local_support(raw, top, WORKSPACE, CONFIG)
        box = compose_box_geometry(top, support)
        self.assertTrue(box.height_valid)
        self.assertEqual(box.height_source, HEIGHT_SOURCE_MEASURED_SUPPORT)
        self.assertEqual(box.geometry_level, GEOMETRY_FULL_3D)
        self.assertAlmostEqual(box.height, h, delta=0.015)
        self.assertAlmostEqual(
            box.center_xyz[2], support_z + h / 2, delta=0.012)
        self.assertNotAlmostEqual(
            box.center_xyz[2], support_z, delta=0.005)

    def test_platform_z_never_overrides_measurement(self):
        w, d, h = SIZES[0]
        cargo, raw, _ = _full_scene(w, d, h, 0.0, 0.62, seed=12)
        top = estimate_top_surface(cargo, WORKSPACE, CONFIG)
        support = estimate_local_support(raw, top, WORKSPACE, CONFIG)
        box = compose_box_geometry(top, support, platform_z=0.86)
        self.assertEqual(box.height_source, HEIGHT_SOURCE_MEASURED_SUPPORT)
        self.assertAlmostEqual(box.height, h, delta=0.015)

    def test_configured_support_mode(self):
        w, d, h = SIZES[0]
        cargo, raw, _ = _full_scene(w, d, h, 0.0, 0.62, seed=13,
                                    drop_sides=("+u", "-u", "+v", "-v"))
        top = estimate_top_surface(cargo, WORKSPACE, CONFIG)
        support = estimate_local_support(raw, top, WORKSPACE, CONFIG)
        self.assertEqual(support.reason, DETECT_SUPPORT_UNOBSERVABLE)
        box = compose_box_geometry(top, support, platform_z=0.62)
        self.assertTrue(box.height_valid)
        self.assertEqual(box.height_source,
                         HEIGHT_SOURCE_CONFIGURED_SUPPORT)
        self.assertAlmostEqual(box.height, h, delta=0.015)

    def test_catalog_prior_is_not_valid_geometry(self):
        w, d, h = SIZES[0]
        cargo, raw, _ = _full_scene(w, d, h, 0.0, 0.62, seed=14,
                                    drop_sides=("+u", "-u", "+v", "-v"))
        top = estimate_top_surface(cargo, WORKSPACE, CONFIG)
        box = compose_box_geometry(top, None, catalog_height=h)
        self.assertFalse(box.height_valid)
        self.assertEqual(box.height_source, HEIGHT_SOURCE_CATALOG_PRIOR)
        self.assertEqual(box.reason, DETECT_HEIGHT_PRIOR_ONLY)
        self.assertEqual(box.geometry_level, GEOMETRY_TOP_ONLY)
        # Numeric prior present for display only.
        self.assertAlmostEqual(box.height, h, delta=1e-9)

    def test_top_only(self):
        w, d, h = SIZES[0]
        cargo, raw, _ = _full_scene(w, d, h, 0.0, 0.62, seed=15,
                                    drop_sides=("+u", "-u", "+v", "-v"))
        top = estimate_top_surface(cargo, WORKSPACE, CONFIG)
        box = compose_box_geometry(top, None)
        self.assertFalse(box.height_valid)
        self.assertEqual(box.height_source, HEIGHT_SOURCE_UNAVAILABLE)
        self.assertEqual(box.geometry_level, GEOMETRY_TOP_ONLY)
        self.assertAlmostEqual(box.width, top.width, delta=1e-9)

    def test_top_unobservable(self):
        box = compose_box_geometry(None, None)
        self.assertFalse(box.height_valid)
        self.assertEqual(box.height_source, HEIGHT_SOURCE_UNAVAILABLE)
        self.assertEqual(box.reason, DETECT_TOP_UNOBSERVABLE)
        self.assertIsNone(box.center_xyz)


class TestSupportStabilityFilter(unittest.TestCase):
    def _support(self, z, reason="ok"):
        from luggage_perception.top_support_estimator import (
            SupportPlaneEstimate)
        return SupportPlaneEstimate(support_z=z, reason=reason)

    def test_converges_when_stable(self):
        filt = SupportStabilityFilter(window=3, max_z_spread=0.015)
        # Partial window is honestly UNSTABLE, not yet a measurement.
        partial = filt.update(self._support(0.62))
        self.assertIsNotNone(partial)
        self.assertEqual(partial.reason, DETECT_SUPPORT_UNSTABLE)
        partial = filt.update(self._support(0.621))
        self.assertEqual(partial.reason, DETECT_SUPPORT_UNSTABLE)
        out = filt.update(self._support(0.619))
        self.assertIsNotNone(out)
        self.assertEqual(out.reason, "ok")
        self.assertAlmostEqual(out.support_z, 0.62, delta=0.002)

    def test_unstable_when_spreading(self):
        filt = SupportStabilityFilter(window=3, max_z_spread=0.015)
        filt.update(self._support(0.62))
        filt.update(self._support(0.64))
        out = filt.update(self._support(0.66))
        self.assertIsNotNone(out)
        self.assertEqual(out.reason, DETECT_SUPPORT_UNSTABLE)

    def test_nonfinite_resets(self):
        filt = SupportStabilityFilter(window=3)
        filt.update(self._support(0.62))
        filt.update(self._support(0.62))
        self.assertIsNone(filt.update(self._support(float("nan"))))
        self.assertIsNone(filt.copy_output())
        # History is cleared too: a new instance must re-fill the window.
        partial = filt.update(self._support(0.90))
        self.assertEqual(partial.reason, DETECT_SUPPORT_UNSTABLE)

    def test_copy_output_is_isolated(self):
        filt = SupportStabilityFilter(window=2)
        filt.update(self._support(0.62))
        out = filt.copy_output()
        out.support_z = 99.0
        again = filt.copy_output()
        self.assertAlmostEqual(again.support_z, 0.62)


if __name__ == "__main__":
    unittest.main()
