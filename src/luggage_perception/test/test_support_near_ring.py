"""Near-ring support priority (G3 wave 3, D1 2026-10-15).

The wave3 geometry chain audit measured support_z = 0.6601 against a
platform top of 0.86 on the inch20 pickup scene: the outer annulus
overhangs the pedestal edge and the container aperture rim (inside the
height band, far more points) out-counted the true adjacent platform
ring, doubling the measured height (0.4174 instead of ~0.2176). The
dominant cluster of the ring ADJACENT to the box must win; the full
annulus stays the fallback when the near ring is too sparse.
"""

import unittest

import numpy as np

from luggage_perception.top_support_estimator import (
    TopSurfaceEstimate,
    estimate_local_support,
)


def _top(width, depth, top_z=1.0776, yaw=0.0, cx=0.0, cy=0.0):
    return TopSurfaceEstimate(
        center_xy=np.array([cx, cy]), top_z=top_z, yaw=yaw,
        width=width, depth=depth, confidence=0.9, yaw_valid=True,
        frame="world")


def _platform_points(top, n_per_side=400, z=0.86, ring=(0.045, 0.17)):
    """True support surface, spread across the annulus toward the
    pedestal edges (the near ring is a subset of it)."""
    rng = np.random.RandomState(7)
    half_w, half_d = top.width * 0.5, top.depth * 0.5
    offsets = rng.uniform(ring[0], ring[1], size=n_per_side)
    sides = rng.randint(0, 4, size=n_per_side)
    points = []
    for off, side in zip(offsets, sides):
        along = rng.uniform(-half_w, half_w)
        if side == 0:
            points.append((along, half_d + off))
        elif side == 1:
            points.append((along, -(half_d + off)))
        elif side == 2:
            points.append((half_w + off, rng.uniform(-half_d, half_d)))
        else:
            points.append((-(half_w + off), rng.uniform(-half_d, half_d)))
    return np.array([[x, y, z + rng.normal(0, 0.001)] for x, y in points])


def _far_structure_points(top, n=1200, z=0.6601, ring=(0.10, 0.18)):
    """Container-aperture-rim-like points: EDGE offsets past the near
    ring but still inside the annulus, and far more numerous than it."""
    return _platform_points(top, n_per_side=max(1, n // 4), z=z, ring=ring)


class TestNearRingSupportPriority(unittest.TestCase):
    def test_far_rim_outcounting_the_platform_loses(self):
        top = _top(0.4757, 0.3234)
        cloud = np.vstack([
            _platform_points(top),
            _far_structure_points(top),
        ])
        estimate = estimate_local_support(cloud, top, workspace=None)
        self.assertEqual(estimate.reason, "ok")
        self.assertAlmostEqual(estimate.support_z, 0.86, delta=0.01)

    def test_near_ring_reports_its_fit_method(self):
        top = _top(0.4757, 0.3234)
        timing = {}
        estimate = estimate_local_support(
            np.vstack([_platform_points(top), _far_structure_points(top)]),
            top, workspace=None, timing=timing)
        self.assertEqual(timing.get("support_fit_method"), "zmode_near_median")
        self.assertAlmostEqual(estimate.support_z, 0.86, delta=0.01)

    def test_sparse_near_ring_falls_back_to_full_annulus(self):
        top = _top(0.4757, 0.3234)
        # Only far-ring points exist (no adjacent support visible): the
        # legacy full-annulus zmode must still answer rather than fail.
        estimate = estimate_local_support(
            _far_structure_points(top, n=1000), top, workspace=None)
        self.assertAlmostEqual(estimate.support_z, 0.6601, delta=0.01)


if __name__ == "__main__":
    unittest.main()
