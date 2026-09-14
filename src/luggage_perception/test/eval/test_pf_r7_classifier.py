#!/usr/bin/env python3
"""PF-R7-H1 fail-closed classifier boundaries. No ROS."""

from __future__ import division

import copy
import unittest

from luggage_perception.eval.pf_r7_classifier import (
    CLASS_ELIGIBLE_FAIL,
    CLASS_ELIGIBLE_PASS,
    CLASS_EVIDENCE,
    CLASS_FIXTURE,
    CLASS_INFRA,
    CLASS_KNOWN_MISS,
    classify_attempt,
)

from pf_r7_fixtures import GT_BBOX, det, miss_record, pass_record


class TestKnownDetectorMiss(unittest.TestCase):
    def test_complete_proof_is_unscored_miss(self):
        classified = classify_attempt(miss_record())
        self.assertEqual(classified["attempt_class"], CLASS_KNOWN_MISS)
        self.assertFalse(classified["scored"])
        self.assertFalse(classified["advances_slot"])
        self.assertFalse(classified["stops_campaign"])
        self.assertTrue(classified["exclusion"])

    def test_absent_proposal_is_also_a_miss(self):
        classified = classify_attempt(miss_record(detections=[]))
        self.assertEqual(classified["attempt_class"], CLASS_KNOWN_MISS)

    def test_missing_health_field_cannot_be_miss(self):
        classified = classify_attempt(miss_record(camera_rate_ok=None))
        self.assertNotEqual(classified["attempt_class"], CLASS_KNOWN_MISS)
        self.assertEqual(classified["attempt_class"], CLASS_INFRA)

    def test_production_floor_change_cannot_be_miss(self):
        classified = classify_attempt(miss_record(
            production_confidence_floor=0.15))
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_FAIL)

    def test_incomplete_dump_cannot_be_miss(self):
        classified = classify_attempt(miss_record(dump_health={
            "capture_complete": False,
            "replay_possible": False,
            "missing": ["color.png"],
        }))
        self.assertEqual(classified["attempt_class"], CLASS_EVIDENCE)
        self.assertIn("dump_incomplete", classified["reasons"])


class TestAcceptedProposalIsNeverMiss(unittest.TestCase):
    def test_accepted_then_empty_cloud_is_fail(self):
        classified = classify_attempt(pass_record(
            n_cargo_points=0,
            entered_cargo_cloud=False,
            n_geometry_requests=0,
        ))
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_FAIL)
        self.assertTrue(any("cargo_cloud" in item or "cloud" in item
                            for item in classified["reasons"]))
        self.assertNotEqual(classified["attempt_class"], CLASS_KNOWN_MISS)

    def test_mask_join_failure_is_fail(self):
        classified = classify_attempt(pass_record(mask_join_failed=True))
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_FAIL)
        self.assertIn("mask_or_cloud_join_failed", classified["reasons"])

    def test_geometry_over_limit_is_fail(self):
        settled = pass_record()["settled"]
        for row in settled:
            row["err_top_m"] = 0.040
        classified = classify_attempt(pass_record(settled=settled))
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_FAIL)
        self.assertTrue(any("top_z" in item for item in classified["reasons"]))

    def test_stale_cross_epoch_is_fail(self):
        classified = classify_attempt(pass_record(stale_cross_epoch=2))
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_FAIL)
        self.assertIn("stale_cross_epoch", classified["reasons"])

    def test_pre_barrier_stale_does_not_fail(self):
        classified = classify_attempt(pass_record(
            stale_pre_barrier_observed=3,
            stale_post_barrier_dropped=2,
            stale_scored_or_fused=0,
            stale_instance_frames=5,
        ))
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_PASS)
        self.assertNotIn("stale_instance_frames", classified["reasons"])
        self.assertNotIn("stale_scored_or_fused", classified["reasons"])

    def test_stale_scored_or_fused_is_fail(self):
        classified = classify_attempt(pass_record(stale_scored_or_fused=1))
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_FAIL)
        self.assertIn("stale_scored_or_fused", classified["reasons"])

    def test_legacy_stale_instance_frames_still_fails(self):
        record = pass_record()
        record.pop("stale_scored_or_fused")
        record["stale_instance_frames"] = 2
        classified = classify_attempt(record)
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_FAIL)
        self.assertIn("stale_instance_frames", classified["reasons"])

    def test_c2_over_limit_is_fail(self):
        classified = classify_attempt(pass_record(c2={
            "output_hz": 12.0,
            "executor_lag_q4_sec": 0.30,
            "executor_lag_q1_sec": 0.08,
            "rss_beta": {"luggage_detector": 1.0},
            "residual_processes": 0,
        }))
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_FAIL)
        self.assertTrue(any("executor_lag_q4" in item
                            for item in classified["reasons"]))

    def test_rss_beta_over_limit_is_fail(self):
        classified = classify_attempt(pass_record(c2={
            "output_hz": 12.0,
            "executor_lag_q4_sec": 0.09,
            "executor_lag_q1_sec": 0.08,
            "rss_beta": {"luggage_detector": 2.4},
            "residual_processes": 0,
        }))
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_FAIL)

    def test_fail_without_dump_is_evidence_invalid(self):
        classified = classify_attempt(pass_record(
            mask_join_failed=True,
            dump_health={
                "capture_complete": False,
                "replay_possible": False,
                "missing": ["depth.npy"],
            },
        ))
        self.assertEqual(classified["attempt_class"], CLASS_EVIDENCE)
        self.assertFalse(classified["stops_campaign"])


class TestFalseCargo(unittest.TestCase):
    def test_edge_strip_is_fail(self):
        classified = classify_attempt(miss_record(
            detections=[
                det(GT_BBOX, 0.07, accepted=False, reason="eval_low_conf"),
                det([600, 20, 640, 200], 0.23, accepted=True,
                    reason="predicate_unavailable"),
            ],
            n_accepted_cargo=1,
            edge_fp_accepted=True,
        ))
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_FAIL)
        self.assertIn("false_cargo_accepted", classified["reasons"])

    def test_cabinet_or_platform_fp_is_fail(self):
        classified = classify_attempt(miss_record(cabinet_fp_accepted=True))
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_FAIL)
        classified = classify_attempt(miss_record(platform_fp_accepted=True))
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_FAIL)

    def test_fake_cargo_cloud_is_fail(self):
        classified = classify_attempt(miss_record(
            fake_cargo_cloud=True, n_cargo_points=1200,
            entered_cargo_cloud=True))
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_FAIL)


class TestInfraAndFixture(unittest.TestCase):
    def test_duplicate_clock_is_infra(self):
        classified = classify_attempt(miss_record(
            n_clock_publishers=2, duplicate_clock=True))
        self.assertEqual(classified["attempt_class"], CLASS_INFRA)

    def test_controller_down_is_infra(self):
        classified = classify_attempt(miss_record(controller_ok=False))
        self.assertEqual(classified["attempt_class"], CLASS_INFRA)

    def test_spawn_flip_is_fixture(self):
        classified = classify_attempt(miss_record(
            spawn_message="PLACE_VERIFY_FAILED: tilt=0.21 model=x_large",
            box_stable=False,
        ))
        self.assertEqual(classified["attempt_class"], CLASS_FIXTURE)

    def test_out_of_frame_is_fixture_not_miss(self):
        classified = classify_attempt(miss_record(
            gt_bbox=[-10, 40, 200, 300], gt_in_frame=False))
        self.assertEqual(classified["attempt_class"], CLASS_FIXTURE)
        self.assertNotEqual(classified["attempt_class"], CLASS_KNOWN_MISS)


class TestEligiblePass(unittest.TestCase):
    def test_geometry_and_recovery_pass(self):
        classified = classify_attempt(pass_record())
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_PASS)
        self.assertTrue(classified["advances_slot"])
        self.assertFalse(classified["stops_campaign"])

    def test_replay_is_deterministic(self):
        record = pass_record()
        first = classify_attempt(record)
        second = classify_attempt(copy.deepcopy(record))
        self.assertEqual(first["attempt_class"], second["attempt_class"])
        self.assertEqual(first["reasons"], second["reasons"])
        miss = miss_record()
        self.assertEqual(
            classify_attempt(miss)["attempt_class"],
            classify_attempt(copy.deepcopy(miss))["attempt_class"])


if __name__ == "__main__":
    unittest.main()
