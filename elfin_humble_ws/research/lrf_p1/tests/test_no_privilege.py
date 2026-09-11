"""Privileged fields must not reach inference."""

from __future__ import division

import os
import sys
import unittest

import numpy as np

WORKSPACE = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(WORKSPACE, "src", "luggage_description"))
sys.path.insert(0, os.path.join(WORKSPACE, "src", "luggage_perception"))
sys.path.insert(0, WORKSPACE)

from research.lrf_p1.baseline_adapter import estimate_from_observation
from research.lrf_p1.contracts import assert_inference_clean, walk_forbidden
from research.lrf_p1.observe import build_sample, enumerate_specs


class NoPrivilegeTest(unittest.TestCase):
    def test_generated_observation_is_clean(self):
        spec = enumerate_specs()[0]
        obs, ev, _extra = build_sample(
            WORKSPACE, spec, np.random.RandomState(3), n_surface=400)
        self.assertEqual(list(walk_forbidden(obs)), [])
        assert_inference_clean(obs)
        self.assertIn("visible_surface_ratio", ev)
        self.assertIn("mesh_id", ev)
        self.assertNotIn("visible_surface_ratio", obs)
        self.assertNotIn("mesh_id", obs)

    def test_estimator_rejects_privileged_keys(self):
        pts = np.ones((60, 3))
        with self.assertRaises(ValueError):
            estimate_from_observation({
                "points": pts,
                "gazebo_state": {"pose": [0, 0, 0]},
            })

    def test_visible_ratio_rejected(self):
        with self.assertRaises(ValueError):
            assert_inference_clean({
                "points": [[0, 0, 0]],
                "visible_surface_ratio": 0.4,
            })

    def test_mesh_id_rejected(self):
        with self.assertRaises(ValueError):
            assert_inference_clean({
                "points": [[0, 0, 0]],
                "mesh_id": "suitcase_vintage_large",
            })


if __name__ == "__main__":
    unittest.main()
