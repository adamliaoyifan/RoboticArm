#!/usr/bin/env python3
"""Unit tests for pick/retreat eval_metrics (no ROS)."""

from __future__ import division

import unittest

from luggage_gazebo.eval_metrics import (
    TrialRecord, summarize, trial_from_dict, trial_to_dict,
)


def _trial(**kwargs):
    defaults = dict(index=0)
    defaults.update(kwargs)
    return TrialRecord(**defaults)


class TestEvalMetrics(unittest.TestCase):

    def test_all_success_rates_are_one(self):
        records = [
            _trial(
                index=i, catalog_id="standard", visual_id="vintage",
                accuracy_ok=True, accuracy_reason="ok",
                segments_planned=4, segments_succeeded=4,
                retreat_ok=True, retreat_delta_z=0.35,
                attach_xy_err=0.01, attach_z_err=0.005,
            )
            for i in range(4)
        ]
        out = summarize(records)
        self.assertEqual(out["n"], 4)
        self.assertEqual(out["detect_pass_rate"], 1.0)
        self.assertEqual(out["plan_pass_rate"], 1.0)
        self.assertEqual(out["retreat_pass_rate"], 1.0)
        self.assertEqual(out["pick_pass_rate"], 1.0)

    def test_detect_fail_excluded_from_plan_denominator(self):
        records = [
            _trial(index=0, accuracy_ok=False, accuracy_reason="size",
                   detect_failure="", fail_code="DETECT_GATE"),
            _trial(index=1, accuracy_ok=True, accuracy_reason="ok",
                   segments_planned=4, segments_succeeded=4,
                   retreat_ok=True, retreat_delta_z=0.34),
        ]
        out = summarize(records)
        self.assertEqual(out["n_detect_compared"], 2)
        self.assertEqual(out["detect_pass_rate"], 0.5)
        self.assertEqual(out["n_detect_ok"], 1)
        self.assertEqual(out["plan_pass_rate"], 1.0)
        self.assertEqual(out["pick_pass_rate"], 0.5)

    def test_fail_code_histogram(self):
        records = [
            _trial(index=0, fail_code="DETECT_NO_CLOUD"),
            _trial(index=1, fail_code="DETECT_NO_CLOUD"),
            _trial(index=2, fail_code="PLAN_pre_grasp"),
        ]
        out = summarize(records)
        self.assertEqual(out["fail_codes"]["DETECT_NO_CLOUD"], 2)
        self.assertEqual(out["fail_codes"]["PLAN_pre_grasp"], 1)

    def test_empty_does_not_divide_by_zero(self):
        out = summarize([])
        self.assertEqual(out["n"], 0)
        self.assertEqual(out["detect_pass_rate"], 0.0)
        self.assertEqual(out["plan_pass_rate"], 0.0)
        self.assertEqual(out["retreat_pass_rate"], 0.0)
        self.assertIsNone(out["attach_xy"]["mean"])
        self.assertIsNone(out["attach_xy"]["std"])

    def test_partial_run_still_summarizes(self):
        records = [
            _trial(index=0, accuracy_ok=True, segments_planned=4,
                   segments_succeeded=1,
                   segment_failures=(("approach", "timeout"),),
                   fail_code="PLAN_approach"),
        ]
        out = summarize(records)
        self.assertEqual(out["plan_pass_rate"], 0.0)
        self.assertEqual(out["segment_failures"]["approach"], 1)

    def test_attach_stability_std(self):
        records = [
            _trial(index=0, attach_xy_err=0.02, attach_z_err=0.01),
            _trial(index=1, attach_xy_err=0.04, attach_z_err=-0.01),
        ]
        out = summarize(records)
        self.assertAlmostEqual(out["attach_xy"]["mean"], 0.03)
        self.assertGreater(out["attach_xy"]["std"], 0.0)
        self.assertAlmostEqual(out["attach_z_abs"]["mean"], 0.01)

    def test_by_catalog_counts(self):
        records = [
            _trial(index=0, catalog_id="carryon", accuracy_ok=True,
                   segments_planned=4, segments_succeeded=4, retreat_ok=True),
            _trial(index=1, catalog_id="large", accuracy_ok=False),
        ]
        out = summarize(records)
        self.assertEqual(out["by_catalog"]["carryon"]["retreat_ok"], 1)
        self.assertEqual(out["by_catalog"]["large"]["detect_ok"], 0)

    def test_json_round_trip(self):
        rec = _trial(
            index=3, catalog_id="standard", visual_id="vintage",
            accuracy_ok=True, segments_planned=4, segments_succeeded=3,
            segment_failures=(("pick_retreat", "timeout"),),
            attach_xy_err=0.02, fail_code="PLAN_pick_retreat",
        )
        back = trial_from_dict(trial_to_dict(rec))
        self.assertEqual(back.index, 3)
        self.assertEqual(back.segment_failures, (("pick_retreat", "timeout"),))
        self.assertEqual(back.fail_code, "PLAN_pick_retreat")


if __name__ == "__main__":
    unittest.main()
