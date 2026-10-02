#!/usr/bin/env python3
"""Aim candidate order, revision chaining, and footprint stop rule."""

import math
import unittest

from luggage_gazebo.cargo_view_sequence import (
    BLENDS,
    EYE_RAISES_M,
    aim_candidates,
    footprint_sensor_hits,
    lookback_eye,
    next_expected_revision,
    policy_hash,
    view_policy_record,
    view_stop,
)
from luggage_perception.cargo_instance_tracker import rotation_from_xyzw
from luggage_perception.cargo_view_integration import optical_axis_angle_rad


def _axis(quat):
    qx, qy, qz, qw = quat
    return (
        2.0 * (qx * qz + qy * qw),
        2.0 * (qy * qz - qx * qw),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )


class TestAimCandidates(unittest.TestCase):

    def test_order_is_raise_then_blend(self):
        rows = aim_candidates(
            (0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), (1.0, 0.0, 1.0),
            (0.0, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(len(rows), len(EYE_RAISES_M) * len(BLENDS))
        pairs = [(row["eye_raise_m"], row["blend"]) for row in rows]
        self.assertEqual(pairs, [
            (raise_m, blend) for raise_m in EYE_RAISES_M for blend in BLENDS])
        again = aim_candidates(
            (0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), (1.0, 0.0, 1.0),
            (0.0, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(
            [row["suction_xyz"] for row in rows],
            [row["suction_xyz"] for row in again])

    def test_full_blend_looks_at_the_target(self):
        eye = (0.0, 0.0, 1.0)
        target = (1.0, 0.2, 0.4)
        rows = aim_candidates(
            eye, (0.0, 0.0, 0.0, 1.0), target,
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
            eye_raises=(0.0,), blends=(1.0,))
        angle, _distance = optical_axis_angle_rad(
            rows[0]["camera_xyz"], rows[0]["camera_q_xyzw"], target)
        self.assertLess(angle, 1e-6)

    def test_suction_pose_round_trips_through_the_camera_transform(self):
        t_cs = (0.02, -0.01, 0.08)
        r_cs = (0.0, 0.0, 0.0, 1.0)
        rows = aim_candidates(
            (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), (1.0, 2.0, 2.0),
            t_cs, r_cs, eye_raises=(0.15,), blends=(0.7,))
        row = rows[0]
        r_wc = rotation_from_xyzw(*row["camera_q_xyzw"])
        back = r_wc.T.dot(
            [row["suction_xyz"][i] - row["camera_xyz"][i] for i in range(3)])
        for got, want in zip(back, t_cs):
            self.assertAlmostEqual(float(got), want, places=6)

    def test_lookback_raises_and_backs_off(self):
        eye = lookback_eye((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), 0.20, 0.15)
        self.assertAlmostEqual(eye[0], 0.15)
        self.assertAlmostEqual(eye[2], 1.20)


class TestRevisionAndFootprint(unittest.TestCase):

    def test_revision_chains_only_after_an_integrate(self):
        self.assertEqual(next_expected_revision(None, 4), 4)
        self.assertEqual(
            next_expected_revision(
                {"integrated": True, "resulting_map_revision": 7}, 4),
            7)
        self.assertEqual(
            next_expected_revision(
                {"integrated": False, "resulting_map_revision": 7}, 4),
            4)

    def test_footprint_ignores_a_sensor_cell_outside_the_slot(self):
        surface = {
            "resolution": 0.05,
            "nx": 2,
            "ny": 1,
            "origin_local": [0.0, 0.0],
            "state": [["occupied"], ["occupied"]],
            "confidence": [["sensor"], ["sensor"]],
        }
        # Cell centers are 0.025 and 0.075. A box at x=0.025 of width 0.04
        # covers only the first cell.
        hits = footprint_sensor_hits(
            surface, (0.025, 0.025, 0.2), (0.04, 0.04, 0.2), 0.0)
        self.assertEqual(hits, 1)

    def test_stop_requires_a_new_revision(self):
        self.assertFalse(view_stop({
            "success": True, "footprint_hits": 2,
            "expected_map_revision": 3, "resulting_map_revision": 3,
        }))
        self.assertTrue(view_stop({
            "success": True, "footprint_hits": 1,
            "expected_map_revision": 3, "resulting_map_revision": 4,
        }))

    def test_policy_hash_is_stable_and_not_frozen(self):
        record = view_policy_record()
        self.assertFalse(record["frozen"])
        self.assertEqual(policy_hash(record), policy_hash(record))
        self.assertEqual(len(policy_hash()), 64)
        self.assertLess(_axis((0, 0, 0, 1))[0], 1.0)


if __name__ == "__main__":
    unittest.main()
