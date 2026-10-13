#!/usr/bin/env python3
"""Epoch and join boundary records. No ROS."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from luggage_perception.join_boundary_trace import (  # noqa: E402
    JoinBoundaryTrace,
    classify_hold_boundary,
    epochs_match,
)


class _Stamp(object):
    def __init__(self, sec, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


class EpochFenceTest(unittest.TestCase):
    def test_same_epoch_commits(self):
        self.assertTrue(epochs_match(4, "box-2", 4, "box-2"))

    def test_generation_change_discards(self):
        self.assertFalse(epochs_match(3, "", 4, "box-2"))

    def test_instance_change_discards(self):
        self.assertFalse(epochs_match(4, "box-1", 4, "box-2"))

    def test_no_epoch_on_either_side_matches(self):
        self.assertTrue(epochs_match(0, "", 0, ""))
        self.assertTrue(epochs_match(None, None, 0, ""))


class JoinBoundaryTraceTest(unittest.TestCase):
    def test_older_stamp_committed_under_new_epoch_is_pre_epoch(self):
        trace = JoinBoundaryTrace()
        trace.note_epoch(4, "box-2", _Stamp(10, 0), depth_stamp=_Stamp(9, 0))
        event = trace.note_join(
            depth_stamp=_Stamp(9, 500000000),
            mask_stamp=_Stamp(9, 500000000),
            mask_nonzero=1200,
            measured_points=400,
            tracked_points_before=0,
            tracked_points=400,
            pre_source="empty",
            post_source="measure",
            acquire_generation=4,
            commit_generation=4,
            commit_instance_id="box-2",
        )
        self.assertTrue(event["pre_epoch_join"])
        self.assertFalse(event["epoch_crossed_during_join"])
        verdict = classify_hold_boundary(trace.snapshot())
        self.assertEqual(verdict["outcome"], "pre_epoch_join_admitted")

    def test_generation_change_during_join_is_pre_epoch(self):
        trace = JoinBoundaryTrace()
        trace.note_epoch(4, "box-2", 10.0)
        event = trace.note_join(
            depth_stamp=10.2,
            measured_points=10,
            pre_source="measure",
            post_source="measure",
            acquire_generation=3,
            commit_generation=4,
        )
        self.assertTrue(event["epoch_crossed_during_join"])
        self.assertTrue(event["pre_epoch_join"])

    def test_post_epoch_measure_is_not_pre_epoch(self):
        trace = JoinBoundaryTrace()
        trace.note_epoch(4, "box-2", 10.0)
        event = trace.note_join(
            depth_stamp=10.2,
            mask_nonzero=80,
            measured_points=80,
            pre_source="empty",
            post_source="measure",
            acquire_generation=4,
            commit_generation=4,
        )
        self.assertFalse(event["pre_epoch_join"])
        verdict = classify_hold_boundary(trace.snapshot())
        self.assertEqual(verdict["outcome"], "post_epoch_measure")

    def test_empty_observation_after_epoch_is_the_first_divergence(self):
        trace = JoinBoundaryTrace()
        trace.note_epoch(4, "box-2", 10.0)
        trace.note_join(
            depth_stamp=10.2,
            measured_points=0,
            tracked_points_before=300,
            tracked_points=300,
            pre_source="empty",
            post_source="hold_track",
            acquire_generation=4,
            commit_generation=4,
            mask_nonzero=0,
        )
        verdict = classify_hold_boundary(trace.snapshot())
        self.assertEqual(verdict["outcome"], "post_epoch_not_measure")
        self.assertEqual(verdict["post_source"], "hold_track")
        self.assertEqual(verdict["measured_points"], 0)
        self.assertEqual(verdict["tracked_points_before"], 300)

    def test_hidden_measure_is_reported_when_joins_are_absent(self):
        reads = [{
            "window": [
                {
                    "stamp": 10.2,
                    "pca_source": "measure",
                    "support_valid": True,
                    "generation": 4,
                    "selected": False,
                },
                {
                    "stamp": 10.4,
                    "pca_source": "hold_track",
                    "support_valid": False,
                    "generation": 4,
                    "selected": True,
                },
            ],
        }]
        verdict = classify_hold_boundary([], auth_reads=reads)
        self.assertEqual(
            verdict["outcome"], "fresh_measure_hidden_by_later_hold")
        self.assertEqual(verdict["measure_stamp"], 10.2)

    def test_measure_then_hold_names_the_hold_and_counts_measures(self):
        trace = JoinBoundaryTrace()
        trace.note_epoch(4, "box-2", 10.0)
        trace.note_join(
            depth_stamp=10.2, measured_points=80, post_source="measure",
            pre_source="empty", acquire_generation=4, commit_generation=4)
        trace.note_join(
            depth_stamp=10.4, measured_points=0, tracked_points_before=80,
            tracked_points=80, post_source="hold_track", pre_source="measure",
            acquire_generation=4, commit_generation=4, mask_nonzero=0)
        verdict = classify_hold_boundary(trace.snapshot())
        self.assertEqual(verdict["outcome"], "post_epoch_not_measure")
        self.assertEqual(verdict["prior_post_epoch_measures"], 1)
        self.assertEqual(verdict["post_source"], "hold_track")

    def test_empty_trace_stays_not_evaluated(self):
        verdict = classify_hold_boundary([])
        self.assertEqual(verdict["outcome"], "not_evaluated")

    def test_ring_drops_the_oldest_event(self):
        trace = JoinBoundaryTrace(maxlen=2)
        trace.note_epoch(1, "a", 1.0)
        trace.note_epoch(2, "b", 2.0)
        trace.note_epoch(3, "c", 3.0)
        generations = [row["generation"] for row in trace.snapshot()]
        self.assertEqual(generations, [2, 3])


if __name__ == "__main__":
    unittest.main()
