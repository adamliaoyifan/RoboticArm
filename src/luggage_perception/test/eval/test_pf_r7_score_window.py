#!/usr/bin/env python3
"""PF-R7 G3 scoring-window and stale-split tests. No ROS."""

from __future__ import division

import json
import os
import unittest

from luggage_perception.eval import gate4_scoring as scoring
from luggage_perception.eval.pf_r7_classifier import (
    CLASS_ELIGIBLE_FAIL,
    CLASS_ELIGIBLE_PASS,
    classify_attempt,
)

from pf_r7_fixtures import pass_record


_FIXTURE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "data", "pfr7_g3_trial01_scores.jsonl")


def _load_trial01():
    rows = []
    with open(_FIXTURE, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


class TestSplitScoreWindowsTrial01(unittest.TestCase):
    def test_corrected_window_is_0_9781(self):
        rows = _load_trial01()
        self.assertEqual(len(rows), 151)
        windows = scoring.split_score_windows(
            rows, "pickup_box_0001_carryon", 2)
        self.assertEqual(len(windows["drain"]), 9)
        self.assertEqual(len(windows["warmup"]), 5)
        self.assertEqual(len(windows["settled"]), 137)
        self.assertEqual(windows["stale_scored_or_fused"], 0)
        self.assertEqual(windows["stale_pre_barrier_observed"], 0)
        summary = scoring.aggregate([{
            "warmup": windows["warmup"],
            "settled": windows["settled"],
            "instance_id": "pickup_box_0001_carryon",
        }])
        self.assertEqual(summary["categories"]["full3d"], 134)
        self.assertAlmostEqual(summary["full3d_rate"], 134 / 137.0, places=6)
        self.assertAlmostEqual(summary["full3d_rate"], 0.9781, places=4)
        self.assertTrue(scoring.gate_pass(summary)["gate4_pass"])

    def test_old_five_frame_warmup_is_below_0_95(self):
        rows = _load_trial01()
        warmup, settled = scoring.split_warmup(rows, warmup_frames=5)
        self.assertEqual(len(settled), 146)
        summary = scoring.aggregate([{
            "warmup": warmup, "settled": settled,
            "instance_id": "pickup_box_0001_carryon",
        }])
        self.assertAlmostEqual(summary["full3d_rate"], 135 / 146.0, places=4)
        self.assertLess(summary["full3d_rate"], 0.95)
        self.assertFalse(scoring.gate_pass(summary)["gate4_pass"])

    def test_corrected_settled_classifies_as_pass(self):
        rows = _load_trial01()
        windows = scoring.split_score_windows(
            rows, "pickup_box_0001_carryon", 2)
        record = pass_record(
            settled=windows["settled"],
            warmup=windows["warmup"],
            n_settled=len(windows["settled"]),
            stale_pre_barrier_observed=windows["stale_pre_barrier_observed"],
            stale_post_barrier_dropped=windows["stale_post_barrier_dropped"],
            stale_scored_or_fused=windows["stale_scored_or_fused"],
        )
        record["recovery"]["n_settled"] = len(windows["settled"])
        classified = classify_attempt(record)
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_PASS, classified)


class TestStaleBarrier(unittest.TestCase):
    def _owned(self, **over):
        row = {
            "instance_id": "pickup_box_0001_carryon",
            "generation": 2,
            "stamp_sec": 10.0,
            "monotonic_sec": 10.0,
            "top_surface_valid": True,
            "height_valid": True,
            "geometry_level": 1,
            "height_source": 1,
            "err_top_m": 0.004,
            "err_support_m": 0.004,
            "err_height_m": 0.006,
            "err_xy_m": 0.008,
            "err_width_m": 0.010,
            "err_depth_m": 0.009,
            "false_measured_height": False,
            "n_cargo_points": 8000,
        }
        row.update(over)
        return row

    def test_queued_old_frames_quarantined_do_not_fail(self):
        drain = [
            self._owned(instance_id="pickup_box_0000_standard", generation=1,
                        stamp_sec=9.9),
            self._owned(stamp_sec=10.0, geometry_level=0, height_valid=False),
        ]
        score = []
        t0 = 10.5
        for i in range(40):
            score.append(self._owned(stamp_sec=t0 + 0.05 * i,
                                     monotonic_sec=t0 + 0.05 * i))
        windows = scoring.bind_collected_windows(
            drain, score, "pickup_box_0001_carryon", 2)
        self.assertGreater(windows["stale_pre_barrier_observed"], 0)
        self.assertEqual(windows["stale_scored_or_fused"], 0)
        self.assertTrue(any(
            item["reason"] == "stale_pre_barrier_observed"
            for item in windows["quarantine"]))
        record = pass_record(
            settled=windows["settled"],
            warmup=windows["warmup"],
            n_settled=len(windows["settled"]),
            stale_pre_barrier_observed=windows["stale_pre_barrier_observed"],
            stale_post_barrier_dropped=windows["stale_post_barrier_dropped"],
            stale_scored_or_fused=windows["stale_scored_or_fused"],
        )
        record["recovery"]["n_settled"] = len(windows["settled"])
        classified = classify_attempt(record)
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_PASS, classified)

    def test_stale_fusion_into_scored_set_fails(self):
        settled = [self._owned(stamp_sec=12.0 + 0.05 * i) for i in range(40)]
        settled[7] = self._owned(
            instance_id="pickup_box_0000_standard", generation=1,
            stamp_sec=12.35)
        windows = scoring.bind_collected_windows(
            [], settled, "pickup_box_0001_carryon", 2)
        self.assertGreater(windows["stale_post_barrier_dropped"], 0)
        self.assertEqual(windows["stale_scored_or_fused"], 0)
        # Dropped from scored set: not fused. Force one fused row.
        fused = list(windows["settled"])
        fused[0] = self._owned(
            instance_id="pickup_box_0000_standard", generation=1)
        record = pass_record(
            settled=fused,
            stale_scored_or_fused=1,
            stale_post_barrier_dropped=windows["stale_post_barrier_dropped"],
        )
        classified = classify_attempt(record)
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_FAIL)
        self.assertIn("stale_scored_or_fused", classified["reasons"])
        self.assertNotEqual(classified["attempt_class"], CLASS_ELIGIBLE_PASS)
