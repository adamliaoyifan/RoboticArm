#!/usr/bin/env python3
"""PF-R6 gen3 focused tests for the zmode_median support estimator.

Adoption boundaries (eng/codex/gpt-5.6-sol, 2026-09-07): the estimator
must survive bin-boundary z values, a dense competing plane, wrong-height
clutter, missing sides, insufficient points and non-finite rows, and be
deterministic — all inside the unchanged fail-closed validation.

Pure numpy, no ROS required.
"""

import math
import unittest

import numpy as np

from luggage_perception.top_support_estimator import (
    DETECT_SUPPORT_UNOBSERVABLE,
    DETECT_SUPPORT_UNSTABLE,
    _zmode_support_plane,
    estimate_local_support,
    TopSupportConfig,
    TopSurfaceEstimate,
)

WORKSPACE = ((0.0, 0.0), (0.5, 0.5))
CONFIG = TopSupportConfig(
    min_support_points=40, min_support_sides=2, min_inliers_per_side=8)


def _rng(seed):
    return np.random.RandomState(seed)


def _ring(n, z, rng, w=0.45, d=0.32, inner=0.03, outer=0.18,
          noise=0.003, sector=None):
    """Uniform annulus samples around a yaw-0 footprint at height z."""
    u = rng.uniform(-(w / 2 + outer), w / 2 + outer, n)
    v = rng.uniform(-(d / 2 + outer), d / 2 + outer, n)
    keep = ((np.abs(u) > w / 2 + inner) | (np.abs(v) > d / 2 + inner))
    if sector is not None:
        ang = np.arctan2(v, u)
        keep &= (ang > sector[0]) & (ang < sector[1])
    u, v = u[keep], v[keep]
    zz = z + rng.normal(0.0, noise, len(u))
    return np.column_stack([u, v, zz])


def _top(w=0.45, d=0.32, h=0.58):
    return TopSurfaceEstimate(
        center_xy=np.zeros(2), top_z=float(h), yaw=0.0,
        width=float(w), depth=float(d), confidence=1.0)


class TestZmodeSupportPlane(unittest.TestCase):
    def test_flat_cluster_found(self):
        rng = _rng(1)
        pts = _ring(3000, 0.0, rng)
        mask, z = _zmode_support_plane(
            pts, dist_thresh=0.008, min_inliers=40)
        self.assertIsNotNone(mask)
        self.assertAlmostEqual(z, 0.0, delta=0.004)
        self.assertGreaterEqual(int(mask.sum()), 40)

    def test_bin_boundary_z_is_stable(self):
        """Plane sitting exactly on a bin edge must not oscillate."""
        for z0 in (-0.004, -0.003, -0.002, -0.001, 0.0, 0.001, 0.002,
                   0.003, 0.004):
            rng = _rng(2)
            pts = _ring(3000, z0, rng)
            mask, z = _zmode_support_plane(
                pts, dist_thresh=0.008, min_inliers=40)
            self.assertIsNotNone(mask, z0)
            self.assertAlmostEqual(z, z0, delta=0.004, msg=str(z0))

    def test_no_dense_cluster_returns_none(self):
        rng = _rng(3)
        n = 400
        u = rng.uniform(-0.4, 0.4, n)
        v = rng.uniform(-0.35, 0.35, n)
        z = 0.58 - rng.uniform(0.15, 0.60, n)  # in-band flyers only
        mask, z_out = _zmode_support_plane(
            np.column_stack([u, v, z]), dist_thresh=0.008, min_inliers=40)
        self.assertIsNone(mask)
        self.assertIsNone(z_out)

    def test_highest_dense_cluster_wins(self):
        """A full ring 5 cm below must not steal the support."""
        rng = _rng(4)
        pts = np.vstack([
            _ring(3000, 0.0, rng), _ring(3000, -0.05, rng)])
        mask, z = _zmode_support_plane(
            pts, dist_thresh=0.008, min_inliers=40)
        self.assertIsNotNone(mask)
        self.assertAlmostEqual(z, 0.0, delta=0.004)

    def test_deterministic_repeats(self):
        rng = _rng(5)
        pts = np.vstack([
            _ring(2500, 0.0, rng), _ring(2500, -0.04, rng)])
        outs = []
        for _ in range(3):
            mask, z = _zmode_support_plane(
                pts, dist_thresh=0.008, min_inliers=40)
            outs.append((z, None if mask is None else int(mask.sum())))
        self.assertEqual(len(set(outs)), 1)


class TestEstimateLocalSupportZmode(unittest.TestCase):
    def test_full_ring_measured(self):
        rng = _rng(10)
        raw = _ring(12000, 0.0, rng)
        est = estimate_local_support(raw, _top(), WORKSPACE, CONFIG)
        self.assertEqual(est.reason, "ok")
        self.assertAlmostEqual(est.support_z, 0.0, delta=0.006)

    def test_wrong_height_clutter_partial_sector(self):
        """Clutter 8 cm below on a narrow sector: support still wins."""
        rng = _rng(11)
        raw = np.vstack([
            _ring(12000, 0.0, rng),
            _ring(3000, -0.08, rng, sector=(-math.pi / 12, math.pi / 12))])
        est = estimate_local_support(raw, _top(), WORKSPACE, CONFIG)
        self.assertEqual(est.reason, "ok")
        self.assertAlmostEqual(est.support_z, 0.0, delta=0.006)

    def test_clutter_single_side_fails_side_coverage(self):
        """Only-side coherent plane, no true support: fail closed."""
        rng = _rng(12)
        clutter = _ring(
            3000, -0.08, rng, sector=(-math.pi / 12, math.pi / 12))
        # In-band flyers only (no coherent support anywhere else).
        n = 300
        u = rng.uniform(-0.4, 0.4, n)
        v = rng.uniform(-0.35, 0.35, n)
        z = 0.58 - rng.uniform(0.16, 0.56, n)
        raw = np.vstack([clutter, np.column_stack([u, v, z])])
        est = estimate_local_support(raw, _top(), WORKSPACE, CONFIG)
        self.assertIn(est.reason, (DETECT_SUPPORT_UNSTABLE,
                                   DETECT_SUPPORT_UNOBSERVABLE))

    def test_missing_one_side_still_measured(self):
        rng = _rng(13)
        ring = _ring(12000, 0.0, rng)
        # Drop the +u outer band: side coverage must fall to 3 but pass.
        w = 0.45
        drop = (ring[:, 0] > (w / 2 + 0.18) - 0.09) & (
            np.abs(ring[:, 1]) < (0.32 / 2 + 0.18))
        raw = ring[~drop]
        est = estimate_local_support(raw, _top(), WORKSPACE, CONFIG)
        self.assertEqual(est.reason, "ok")
        self.assertAlmostEqual(est.support_z, 0.0, delta=0.006)

    def test_insufficient_points_unobservable(self):
        rng = _rng(14)
        raw = _ring(30, 0.0, rng)
        est = estimate_local_support(raw, _top(), WORKSPACE, CONFIG)
        self.assertEqual(est.reason, DETECT_SUPPORT_UNOBSERVABLE)

    def test_nonfinite_rows_do_not_crash_or_fabricate(self):
        rng = _rng(15)
        raw = _ring(12000, 0.0, rng)
        raw[::2000, 2] = np.nan
        raw[7, 0] = np.inf
        est = estimate_local_support(raw, _top(), WORKSPACE, CONFIG)
        self.assertEqual(est.reason, "ok")
        self.assertAlmostEqual(est.support_z, 0.0, delta=0.006)
        # Starved variant: nearly all non-finite -> fail closed.
        raw2 = _ring(12000, 0.0, rng)
        raw2[:, 2] = np.nan
        est2 = estimate_local_support(raw2, _top(), WORKSPACE, CONFIG)
        self.assertEqual(est2.reason, DETECT_SUPPORT_UNOBSERVABLE)

    def test_pipeline_deterministic(self):
        rng = _rng(16)
        raw = np.vstack([
            _ring(8000, 0.0, rng), _ring(2000, -0.05, rng)])
        keys = []
        for _ in range(3):
            est = estimate_local_support(raw, _top(), WORKSPACE, CONFIG)
            keys.append((est.reason, round(float(est.support_z), 9),
                         int(est.inlier_count)))
        self.assertEqual(len(set(keys)), 1)

    def test_timing_records_method(self):
        rng = _rng(17)
        raw = _ring(12000, 0.0, rng)
        timing = {}
        estimate_local_support(raw, _top(), WORKSPACE, CONFIG,
                               timing=timing)
        self.assertEqual(timing.get("support_fit_method"), "zmode_median")
        self.assertLess(timing.get("support_ransac_ms", 1e9), 50.0)


if __name__ == "__main__":
    unittest.main()
