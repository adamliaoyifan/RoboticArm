#!/usr/bin/env python3
"""[ST09-I] SelectPlacePath service surface after the route switch.

The legacy carry-segment/knot-carrier plumbing is gone; these tests pin
the route-mode filler's contract failures and the typed/v1 leg views of
a certified route response.
"""

from __future__ import division

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

TEST_DIR = Path(__file__).resolve().parent
PKG_DIR = TEST_DIR.parent
for entry in (str(PKG_DIR), str(PKG_DIR / "scripts")):
    if entry not in sys.path:
        sys.path.insert(0, entry)

from luggage_planning.place import place_service  # noqa: E402
from luggage_planning.routing.producer import _leg_record  # noqa: E402
from luggage_planning.planning_contract.fixtures import make_timed_trajectory  # noqa: E402

JOINTS = tuple("elfin_joint%d" % i for i in range(1, 7))


def _legs(count=2):
    trajectory = make_timed_trajectory()
    return [_leg_record("leg%d" % i, trajectory, "req-1", "scene-1", 1000)
            for i in range(count)]


class RouteLegViews(unittest.TestCase):
    def test_carry_segments_are_joint_path_views(self):
        segments = place_service._route_carry_segments(_legs(), None)
        self.assertEqual(len(segments), 2)
        for segment in segments:
            self.assertEqual(segment.type, "joint_path")
            self.assertFalse(segment.allow_ompl_fallback)
            self.assertGreaterEqual(len(segment.joint_waypoints), 2)

    def test_motion_messages_admit_as_typed_joint_paths(self):
        from luggage_description.frame_diagnostics import BoundaryContext
        from luggage_planning.motion.motion_frame_adapters import (
            message_from_admitted,
        )
        from luggage_planning.motion.motion_frame_contracts import (
            TARGET_LINK, ExecutionContext,
        )
        boundary = BoundaryContext(
            interface_id="F15", producer="route_producer",
            consumer="motion_executor", request_id="req-1",
            scene_revision="scene-1", frame_id="world",
            reference="suction_contact_frame", time_basis="robot_state")
        context = ExecutionContext(boundary, TARGET_LINK, "world")
        messages = [message_from_admitted(leg, context) for leg in _legs()]
        self.assertTrue(all(
            m.segment_type == "joint_path" for m in messages))

    def test_knot_carriers_are_gone(self):
        import luggage_planning.motion.motion_executor as executor
        self.assertFalse(hasattr(place_service, "fill_select_response"))
        self.assertTrue(callable(getattr(
            place_service, "fill_select_response_route")))


if __name__ == "__main__":
    unittest.main()
