#!/usr/bin/env python3
"""Pure unit tests for hybrid container opening estimation."""

import os
import sys
import unittest
import json

import numpy as np


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_perception.container_opening_estimator import (  # noqa: E402
    ContainerOpeningEstimator,
    EstimatorConfig,
    OpeningPrior,
    SOURCE_DEPTH_ONLY,
    SOURCE_PRIOR,
    SOURCE_TAG_DEPTH,
    SOURCE_TAG_ONLY,
)


def _opening_points(seed=3, count=500, width=1.20, height=0.80):
    rng = np.random.RandomState(seed)
    center = np.array([0.35, -0.20, 1.10])
    width_axis = np.array([0.0, 1.0, 0.0])
    height_axis = np.array([0.0, 0.0, 1.0])
    normal = np.array([1.0, 0.0, 0.0])
    u = rng.uniform(-width * 0.5, width * 0.5, count)
    v = rng.uniform(-height * 0.5, height * 0.5, count)
    points = (
        center
        + u[:, None] * width_axis
        + v[:, None] * height_axis
        + rng.normal(0.0, 0.003, (count, 1)) * normal
    )
    outliers = rng.uniform([-0.2, -1.2, 0.2], [1.0, 0.8, 2.0], (100, 3))
    return np.vstack((points, outliers)), center, normal, width_axis


def _tag_prior(stamp=10.0):
    return OpeningPrior(
        center=[0.37, -0.18, 1.08],
        normal=[0.999, 0.03, -0.02],
        width_axis=[-0.03, 0.999, 0.0],
        width=1.30,
        height=0.90,
        stamp=stamp,
        confidence=0.85,
        source="tag",
    )


class TestContainerOpeningEstimator(unittest.TestCase):
    def setUp(self):
        self.estimator = ContainerOpeningEstimator(EstimatorConfig(
            plane_distance_threshold=0.012,
            min_depth_points=30,
            random_seed=4,
            max_age_sec=0.75,
        ))

    def test_tag_and_depth_refines_plane_and_aperture(self):
        points, expected_center, expected_normal, _ = _opening_points()
        estimate = self.estimator.estimate(
            prior=_tag_prior(),
            depth_points=points,
            depth_stamp=10.1,
            now=10.2,
            hardware_strict=True,
        )

        self.assertEqual(estimate.source_status, SOURCE_TAG_DEPTH)
        self.assertTrue(estimate.accepted)
        self.assertTrue(estimate.fresh)
        self.assertGreater(estimate.confidence, 0.55)
        self.assertLess(np.linalg.norm(estimate.center - expected_center), 0.04)
        self.assertGreater(float(np.dot(estimate.normal, expected_normal)), 0.995)
        self.assertAlmostEqual(estimate.width, 1.20, delta=0.09)
        self.assertAlmostEqual(estimate.height, 0.80, delta=0.07)
        self.assertEqual(estimate.pose_covariance.shape, (6, 6))
        self.assertEqual(estimate.aperture_covariance.shape, (2, 2))
        self.assertTrue(np.all(np.diag(estimate.pose_covariance) > 0.0))
        self.assertGreater(estimate.diagnostics["inlier_points"], 400)
        self.assertIn('"source_status": "tag_depth"', json.dumps(estimate.as_dict()))

    def test_tag_only_is_an_explicit_fresh_source(self):
        estimate = self.estimator.estimate(
            prior=_tag_prior(), depth_points=[], now=10.2,
            hardware_strict=True,
        )

        self.assertEqual(estimate.source_status, SOURCE_TAG_ONLY)
        self.assertTrue(estimate.accepted)
        self.assertTrue(estimate.fresh)
        self.assertAlmostEqual(estimate.width, 1.30)
        self.assertGreater(estimate.confidence, 0.4)

    def test_depth_only_estimates_orientation_and_extent(self):
        points, expected_center, expected_normal, _ = _opening_points(seed=9)
        estimate = self.estimator.estimate(
            depth_points=points,
            depth_stamp=4.0,
            now=4.1,
            sensor_origin=[2.0, -0.2, 1.1],
            hardware_strict=True,
        )

        self.assertEqual(estimate.source_status, SOURCE_DEPTH_ONLY)
        self.assertTrue(estimate.accepted)
        self.assertGreater(float(np.dot(estimate.normal, expected_normal)), 0.99)
        self.assertLess(np.linalg.norm(estimate.center - expected_center), 0.05)
        self.assertAlmostEqual(max(estimate.width, estimate.height), 1.20, delta=0.10)
        self.assertAlmostEqual(min(estimate.width, estimate.height), 0.80, delta=0.08)

    def test_prior_only_is_rejected_in_hardware_strict(self):
        prior = OpeningPrior(
            center=[0.0, 0.0, 1.0],
            normal=[0.0, 0.0, 1.0],
            width_axis=[1.0, 0.0, 0.0],
            width=1.0,
            height=0.7,
            stamp=8.0,
            source="prior",
        )
        estimate = self.estimator.estimate(
            prior=prior, now=8.1, hardware_strict=True
        )

        self.assertEqual(estimate.source_status, SOURCE_PRIOR)
        self.assertFalse(estimate.accepted)
        self.assertEqual(estimate.rejection_reason, "prior_only_disallowed")

    def test_stale_sensor_estimate_is_rejected_in_hardware_strict(self):
        estimate = self.estimator.estimate(
            prior=_tag_prior(stamp=5.0), now=6.0, hardware_strict=True
        )

        self.assertEqual(estimate.source_status, SOURCE_TAG_ONLY)
        self.assertFalse(estimate.fresh)
        self.assertFalse(estimate.accepted)
        self.assertEqual(estimate.rejection_reason, "stale_estimate")
        self.assertAlmostEqual(estimate.age_sec, 1.0)

    def test_low_confidence_tag_is_rejected_in_hardware_strict(self):
        prior = _tag_prior()
        prior.confidence = 0.2
        estimate = self.estimator.estimate(
            prior=prior, now=10.1, hardware_strict=True)
        self.assertFalse(estimate.accepted)
        self.assertEqual(estimate.rejection_reason, "low_confidence")

    def test_non_strict_reports_stale_but_keeps_fallback_available(self):
        estimate = self.estimator.estimate(
            prior=_tag_prior(stamp=5.0), now=8.0, hardware_strict=False
        )

        self.assertFalse(estimate.fresh)
        self.assertTrue(estimate.accepted)
        self.assertLess(estimate.confidence, 0.05)

    def test_invalid_or_insufficient_depth_falls_back_to_tag(self):
        points = [[float("nan"), 0.0, 0.0], [0.0, 0.0, 0.0]]
        estimate = self.estimator.estimate(
            prior=_tag_prior(), depth_points=points, now=10.1
        )
        self.assertEqual(estimate.source_status, SOURCE_TAG_ONLY)

    def test_no_inputs_returns_no_estimate(self):
        self.assertIsNone(self.estimator.estimate(now=1.0))


if __name__ == "__main__":
    unittest.main()
