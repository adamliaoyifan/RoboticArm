#!/usr/bin/env python3
"""Unit tests for place_metrics (no ROS)."""

from __future__ import division

import unittest

from luggage_gazebo.place_metrics import (
    PlaceTrial,
    parse_ign_model_pose,
    place_ok,
    summarize,
    trial_from_dict,
    trial_to_dict,
)


class TestPlaceMetrics(unittest.TestCase):

    def test_goto_failed_before_place_is_a_failure(self):
        rec = PlaceTrial(index=0, fail_code="GOTO_FAILED", place_state="INIT")
        self.assertFalse(place_ok(rec))

    def test_place_fraction_failure_counts(self):
        rec = PlaceTrial(
            index=0, fail_code="PLACE_FRACTION_descend",
            place_state="ABORT_CARRYING")
        self.assertFalse(place_ok(rec))

    def test_summarize_pass_rate(self):
        records = [
            PlaceTrial(index=0, fail_code="", descend_fraction=1.0,
                       used_ompl_fallback_descend=False, inside_inner_box=True),
            PlaceTrial(index=1, fail_code="GOTO_FAILED", place_state="HOME",
                       descend_fraction=1.0,
                       used_ompl_fallback_descend=False, inside_inner_box=True),
            PlaceTrial(index=2, fail_code="PLACE_PLAN_transit",
                       lost_payload=False),
        ]
        out = summarize(records)
        self.assertEqual(out["n"], 3)
        self.assertEqual(out["n_place_ok"], 2)
        self.assertAlmostEqual(out["place_pass_rate"], 2.0 / 3.0)
        self.assertEqual(out["n_descend_ok"], 2)
        self.assertEqual(out["fail_codes"]["PLACE_PLAN_transit"], 1)
        self.assertNotIn("GOTO_FAILED", out["fail_codes"])

    def test_roundtrip_dict(self):
        rec = PlaceTrial(index=4, catalog_id="carryon", extras={"a": 1})
        back = trial_from_dict(trial_to_dict(rec))
        self.assertEqual(back.index, 4)
        self.assertEqual(back.catalog_id, "carryon")
        self.assertEqual(back.extras["a"], 1)

    def test_empty_summary(self):
        out = summarize([])
        self.assertEqual(out["n"], 0)
        self.assertEqual(out["place_pass_rate"], 0.0)

    def test_parse_ign_model_pose_skips_entity_id(self):
        text = (
            "Model: [138]\n"
            "  - Name: pickup_box_0012_carryon\n"
            "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
            "    [1.500000 0.001000 0.655000]\n"
            "    [0.004573 0.027155 -3.136030]\n"
        )
        pose = parse_ign_model_pose(text)
        self.assertAlmostEqual(pose[0], 1.5)
        self.assertAlmostEqual(pose[1], 0.001)
        self.assertAlmostEqual(pose[2], 0.655)
        self.assertAlmostEqual(pose[3], 0.004573)
        self.assertAlmostEqual(pose[4], 0.027155)


if __name__ == "__main__":
    unittest.main()
