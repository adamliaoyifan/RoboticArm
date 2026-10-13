#!/usr/bin/env python3
"""Aim candidate order, revision chaining, and footprint stop rule."""

import math
import unittest

from luggage_gazebo.cargo_view_sequence import (
    BLENDS,
    MAP_CONFIRMATION_REQUIRED,
    EYE_RAISES_M,
    aim_candidates,
    footprint_sensor_hits,
    lookback_eye,
    map_confirmation_record,
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


class TestObservationPolicy(unittest.TestCase):
    """[ST09-I S5.7-4] policy-owned target + visibility prediction."""

    def test_observation_target_lifts_slot_to_box_center(self):
        from luggage_gazebo.cargo_view_sequence import observation_target
        target = observation_target((0.4, -0.2, 0.30), 0.32)
        self.assertEqual(
            target, [0.4, -0.2, 0.30 + 0.16])

    def test_predict_in_frame_without_intrinsics_admits_with_reason(self):
        from luggage_gazebo.cargo_view_sequence import predict_in_frame
        visible, info = predict_in_frame(
            [(1.0, 0.0, 0.0)], (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        self.assertTrue(visible)
        self.assertEqual(info.get("reason"), "no_camera_info")

    def test_predict_in_frame_rejects_behind_camera(self):
        from luggage_gazebo.cargo_view_sequence import predict_in_frame
        # Camera looks along +Z (identity); a target BEHIND it cannot
        # be a candidate.
        visible, _info = predict_in_frame(
            [(-1.0, 0.0, 0.0)], (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
            camera_k=[320.0, 0.0, 320.0, 0.0, 320.0, 240.0],
            width=640, height=480)
        self.assertFalse(visible)

    def test_rank_caps_at_two_and_orders_in_frame_first(self):
        from luggage_gazebo.cargo_view_sequence import (
            rank_observation_aims,
        )
        # A camera looking straight down +Z at a target 1 m below the
        # box: full-blend aims are in frame, tiny-blend aims are not.
        rows = rank_observation_aims(
            (0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
            [(0.0, 0.0, 0.0), (0.05, 0.0, 0.0)],
            camera_k=[320.0, 0.0, 320.0, 0.0, 320.0, 240.0],
            image_wh=(640, 480))
        self.assertLessEqual(len(rows), 2)
        self.assertTrue(all(row["in_frame"] for row in rows))
        # Frozen order is preserved among the selected rows.
        self.assertEqual(
            [row["index"] for row in rows],
            sorted(row["index"] for row in rows))

    def test_policy_record_names_the_promoted_decisions(self):
        record = view_policy_record()
        self.assertEqual(record["max_observation_views"], 2)
        self.assertTrue(record["in_frame_prediction"])
        self.assertIn("slot_world", record["target"])


class TestMapConfirmationRecord(unittest.TestCase):

    def test_footprint_hit_with_advanced_revision_confirms(self):
        record = map_confirmation_record({
            "success": True, "footprint_hits": 4,
            "expected_map_revision": 7, "resulting_map_revision": 8})
        self.assertTrue(record["confirmed"])
        self.assertEqual(record["reason_code"], "")

    def test_no_detection_is_not_an_observation_of_empty(self):
        record = map_confirmation_record({
            "success": True, "footprint_hits": 0,
            "expected_map_revision": 7, "resulting_map_revision": 8,
            "empty_reason": "NO_DEPTH"})
        self.assertFalse(record["confirmed"])
        self.assertEqual(record["reason_code"], MAP_CONFIRMATION_REQUIRED)
        self.assertEqual(record["view_reason"], "NO_DEPTH")

    def test_missing_trace_stays_pending(self):
        record = map_confirmation_record(None)
        self.assertFalse(record["confirmed"])
        self.assertEqual(record["view_reason"], "VIEW_EMPTY")

    def test_mapper_pending_state_outranks_the_cumulative_footprint_count(self):
        trace = {
            "success": True, "footprint_hits": 5,
            "expected_map_revision": 7, "resulting_map_revision": 8,
            "occupancy_confirmed": False}
        self.assertFalse(view_stop(trace))
        record = map_confirmation_record(trace)
        self.assertFalse(record["confirmed"])
        self.assertEqual(record["reason_code"], MAP_CONFIRMATION_REQUIRED)
        trace["occupancy_confirmed"] = True
        self.assertTrue(view_stop(trace))


class TestCommitModeViewStop(unittest.TestCase):
    """A committed box has geometry cells, so only the mapper's settle counts."""

    BASE = {"success": True, "expected_map_revision": 2,
            "resulting_map_revision": 3, "commit_mode": True,
            "occupancy_confirmed": True, "footprint_hits": 0,
            "last_settle": {"cleared": [1], "evidence_points": {"1": 7}}}

    def test_stops_on_cleared_record_without_a_footprint_sensor_cell(self):
        self.assertTrue(view_stop(dict(self.BASE)))

    def test_view_that_cleared_nothing_does_not_stop(self):
        trace = dict(self.BASE, last_settle={"cleared": [], "evidence_points": {}})
        self.assertFalse(view_stop(trace))

    def test_pending_records_left_do_not_stop(self):
        self.assertFalse(view_stop(dict(self.BASE, occupancy_confirmed=False)))

    def test_footprint_hits_do_not_rescue_an_unconfirmed_commit_view(self):
        trace = dict(self.BASE, footprint_hits=50,
                     last_settle={"cleared": [], "evidence_points": {}})
        self.assertFalse(view_stop(trace))

    def test_revision_must_advance(self):
        self.assertFalse(view_stop(dict(self.BASE, resulting_map_revision=2)))

    def test_default_mode_keeps_the_footprint_rule(self):
        trace = {"success": True, "expected_map_revision": 1,
                 "resulting_map_revision": 2, "footprint_hits": 0}
        self.assertFalse(view_stop(trace))
        trace["footprint_hits"] = 3
        self.assertTrue(view_stop(trace))
