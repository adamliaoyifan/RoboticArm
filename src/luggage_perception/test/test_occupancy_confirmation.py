#!/usr/bin/env python3
"""The one transition that clears a pending occupancy change."""

import unittest

import numpy as np

from luggage_perception.cargo_view_integration import (
    apply_integrate,
    footprint_sensor_hits,
)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_confirmation import PendingConfirmation

FLOOR_Z = 0.53
BOX_SIZE = [0.2, 0.2, 0.2]
#: Placed on the floor: spans z 0.53..0.73.
FLOOR_BOX = ([0.0, 0.0, FLOOR_Z + 0.1], BOX_SIZE, 0.0)
#: Stacked on FLOOR_BOX: spans z 0.73..0.93.
STACKED_BOX = ([0.0, 0.0, FLOOR_Z + 0.3], BOX_SIZE, 0.0)
CHANGE_NS = 100_000_000_000
LATER_NS = 101_000_000_000
EYE = (0.0, 0.0, 1.25)


def _face(z, half=0.08, step=0.04, center_xy=(0.0, 0.0)):
    axis = np.arange(-half, half + 1e-9, step)
    return np.array([
        [center_xy[0] + x, center_xy[1] + y, z] for x in axis for y in axis])


ELSEWHERE = _face(FLOOR_Z + 0.1, center_xy=(0.33, 0.18))


def _mapper():
    return CargoVolumeMapper(
        (1.0, 0.6, 0.8), (0.0, 0.0, FLOOR_Z + 0.4), 0.0, resolution=0.05)


def _integrate(mapper, points):
    result = apply_integrate(mapper, points, origin=EYE)
    assert result["ok"], result
    return result["observed_points"]


def _placed(footprint=FLOOR_BOX):
    pending = PendingConfirmation(True)
    token = pending.record("box_placed", CHANGE_NS, footprint)
    return pending, token


class TestPendingConfirmation(unittest.TestCase):

    def test_placement_is_pending_until_the_view_observes_the_new_box(self):
        pending, token = _placed()
        self.assertFalse(pending.confirmed)
        self.assertEqual(pending.surface_fields()["change_event"], "box_placed")
        started = pending.begin()
        self.assertEqual(started, token)
        cleared = pending.settle(
            started, LATER_NS, _face(FLOOR_Z + 0.2), True)
        self.assertEqual(cleared, [token])
        self.assertTrue(pending.confirmed)

    def test_a_view_elsewhere_never_clears(self):
        pending, _token = _placed()
        self.assertEqual(
            pending.settle(pending.begin(), LATER_NS, ELSEWHERE, True), [])
        self.assertFalse(pending.confirmed)

    def test_too_few_points_never_clear(self):
        pending, _token = _placed()
        sparse = _face(FLOOR_Z + 0.2)[:2]
        self.assertEqual(
            pending.settle(pending.begin(), LATER_NS, sparse, True), [])

    def test_a_frame_from_before_the_change_never_clears(self):
        pending, _token = _placed()
        for stamp in (CHANGE_NS - 1, CHANGE_NS, 0):
            self.assertEqual(pending.settle(
                pending.begin(), stamp, _face(FLOOR_Z + 0.2), True), [])
        self.assertFalse(pending.confirmed)

    def test_an_unsettled_view_never_clears(self):
        pending, _token = _placed()
        self.assertEqual(pending.settle(
            pending.begin(), LATER_NS, _face(FLOOR_Z + 0.2), False), [])
        self.assertFalse(pending.confirmed)

    def test_a_change_during_integration_is_not_cleared_by_it(self):
        pending, first = _placed()
        started = pending.begin()
        second = pending.record("box_placed", LATER_NS - 1, FLOOR_BOX)
        self.assertGreater(second, first)
        points = _face(FLOOR_Z + 0.2)
        self.assertEqual(pending.settle(started, LATER_NS, points, True), [first])
        self.assertFalse(pending.confirmed)
        fields = pending.surface_fields()
        self.assertEqual(
            [row["token"] for row in fields["pending_changes"]], [second])
        later = pending.settle(
            pending.begin(), LATER_NS + 1_000_000_000, points, True)
        self.assertEqual(later, [second])
        self.assertTrue(pending.confirmed)

    def test_events_without_an_observation_rule_stay_pending(self):
        pending = PendingConfirmation(True)
        for event in ("box_removed", "box_disturbed", "cargo_edited",
                      "container_pose_changed"):
            pending.record(event, CHANGE_NS)
        points = _face(FLOOR_Z + 0.2)
        self.assertEqual(
            pending.settle(pending.begin(), LATER_NS, points, True), [])
        fields = pending.surface_fields()
        self.assertFalse(fields["occupancy_confirmed"])
        self.assertTrue(all(
            not row["confirmable"] for row in fields["pending_changes"]))
        pending.clear()
        self.assertTrue(pending.confirmed)

    def test_gate_off_records_nothing(self):
        pending = PendingConfirmation(False)
        self.assertEqual(pending.record("box_placed", CHANGE_NS, FLOOR_BOX), 0)
        self.assertTrue(pending.confirmed)
        fields = pending.surface_fields()
        self.assertTrue(fields["occupancy_confirmed"])
        self.assertEqual(fields["change_token"], 0)

    def test_motion_is_not_a_change_and_unknown_events_are_refused(self):
        pending = PendingConfirmation(True)
        for event in ("return", "pickup_spawn", "grasp", "planning"):
            self.assertEqual(pending.record(event, CHANGE_NS), 0)
        self.assertTrue(pending.confirmed)
        with self.assertRaises(ValueError):
            pending.record("tidying", CHANGE_NS)

    def test_clear_keeps_the_token_monotonic(self):
        pending, first = _placed()
        pending.clear()
        self.assertTrue(pending.confirmed)
        self.assertGreater(
            pending.record("box_placed", LATER_NS, FLOOR_BOX), first)


class TestOldEvidenceIsNotConfirmation(unittest.TestCase):
    """Real mapper grid: the accumulated map keeps old observations."""

    def test_old_sensor_cell_in_the_footprint_does_not_clear(self):
        mapper = _mapper()
        center, size, yaw = FLOOR_BOX
        # 1. an old sensor-occupied cell sits in the footprint's column
        #    (above the box, so committing the box does not overwrite it).
        _integrate(mapper, np.array([[0.0, 0.0, FLOOR_Z + 0.42]]))
        # 2. the box is committed; its change is pending.
        self.assertTrue(mapper.mark_placed_box(center, size, yaw))
        pending, token = _placed()
        # 3. a later frame sees only another corner; 4. the map keeps the
        #    old cell, so a cumulative-map test would wrongly be satisfied.
        started = pending.begin()
        observed = _integrate(mapper, ELSEWHERE)
        self.assertGreaterEqual(footprint_sensor_hits(
            mapper.surface_map_2d(), center, size, yaw), 1)
        self.assertEqual(
            pending.settle(started, LATER_NS, observed, True), [])
        self.assertFalse(pending.confirmed)
        self.assertEqual(
            pending.surface_fields()["last_settle"]["evidence_points"],
            {token: 0})
        # A later frame that actually hits the footprint may clear it.
        started = pending.begin()
        observed = _integrate(mapper, _face(FLOOR_Z + 0.2))
        self.assertEqual(
            pending.settle(started, LATER_NS + 1, observed, True), [token])
        self.assertTrue(pending.confirmed)

    def test_the_box_underneath_is_not_evidence_for_a_stacked_box(self):
        mapper = _mapper()
        lower_center, lower_size, lower_yaw = FLOOR_BOX
        mapper.mark_placed_box(lower_center, lower_size, lower_yaw)
        # The lower box's top face (z 0.73) is what a camera sees first.
        _integrate(mapper, _face(FLOOR_Z + 0.2))
        center, size, yaw = STACKED_BOX
        mapper.mark_placed_box(center, size, yaw)
        pending, token = _placed(STACKED_BOX)
        started = pending.begin()
        observed = _integrate(mapper, _face(FLOOR_Z + 0.2))
        self.assertEqual(
            pending.settle(started, LATER_NS, observed, True), [])
        self.assertFalse(pending.confirmed)
        # Points on the stacked box's own top face (z 0.93) do confirm it.
        started = pending.begin()
        observed = _integrate(mapper, _face(FLOOR_Z + 0.4))
        self.assertEqual(
            pending.settle(started, LATER_NS + 1, observed, True), [token])


if __name__ == "__main__":
    unittest.main()
