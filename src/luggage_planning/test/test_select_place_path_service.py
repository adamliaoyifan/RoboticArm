#!/usr/bin/env python3
"""SelectPlacePath service-chain tests against REAL generated messages.

The 2026-09-29 audit found the service had never run end to end: rosidl
fixed-slots messages reject the ``_joint_knots`` dynamic attribute (any
winner crashed the callback) and ``_segments`` never filled waypoints or
target_pose (a cartesian response would have carried the default zero
pose as its goal). These tests use genuine MotionSegment instances so the
typed-message path cannot silently regress behind stand-in objects again.
"""

from __future__ import division

import os
import sys
import unittest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(
    0, os.path.normpath(os.path.join(TEST_DIR, "..")))
sys.path.insert(
    0, os.path.normpath(os.path.join(TEST_DIR, "..", "scripts")))

from geometry_msgs.msg import Pose  # noqa: E402
from luggage_msgs.msg import MotionSegment  # noqa: E402

from luggage_planning.place_selection_client import (  # noqa: E402
    apply_carry_segment,
    read_joint_knots,
    write_joint_knots,
)
from trajectory_selector_node import _segments  # noqa: E402


class TestTypedMessageKnots(unittest.TestCase):
    def test_write_read_roundtrip_on_real_message(self):
        segment = MotionSegment()
        knots = [[0.1] * 6, [0.2] * 6]
        write_joint_knots(segment, knots)  # must not raise
        self.assertEqual(len(segment.joint_waypoints), 2)
        self.assertEqual(
            [list(p.positions) for p in segment.joint_waypoints], knots)
        self.assertEqual(read_joint_knots(segment), knots)

    def test_typed_message_has_no_dynamic_attr_leak(self):
        segment = MotionSegment()
        write_joint_knots(segment, [[0.0] * 6])
        self.assertFalse(hasattr(segment, "_joint_knots"))

    def test_apply_carry_segment_joint_path_on_real_message(self):
        # The driver-side landmine: a grid winner (joint_path spec) applied
        # to a real MotionSegment used to crash on the dynamic attribute.
        segment = MotionSegment()
        segment.name = "traverse"
        segment.type = "cartesian"
        selection = {"segments": {"traverse": {
            "type": "joint_path",
            "waypoints": [(0.0, 0.0, 1.0), (0.2, 0.0, 1.0)],
            "joint_knots": [[0.0] * 6, [0.1] + [0.0] * 5],
            "allow_ompl_fallback": False,
        }}}
        apply_carry_segment(segment, selection)
        self.assertEqual(segment.type, "joint_path")
        self.assertFalse(segment.allow_ompl_fallback)
        self.assertEqual(len(segment.joint_waypoints), 2)
        self.assertEqual(read_joint_knots(segment), [[0.0] * 6,
                                                     [0.1] + [0.0] * 5])


def _winner(source="geometric", waypoints=None, knots=None, yaw=0.0):
    spec = {
        "type": "joint_path" if knots else "cartesian",
        "waypoints": waypoints or [(-0.6, -0.4, 1.5), (0.08, -0.66, 1.2)],
        "joint_knots": knots or [],
        "allow_ompl_fallback": not bool(knots),
    }
    return {
        "slot_index": 0,
        "yaw": yaw,
        "waypoints": waypoints or [(-0.6, -0.4, 1.5), (0.08, -0.66, 1.2)],
        "segments": {"transit": dict(spec), "traverse": dict(spec)},
    }


class TestSegmentsPayload(unittest.TestCase):
    def _shift(self, xyz):
        return [xyz[0] + 1.5, xyz[1], xyz[2]]

    def test_cartesian_payload_is_complete(self):
        out = _segments(_winner(), map_to_world=self._shift)
        self.assertEqual([s.name for s in out], ["transit", "traverse"])
        segment = out[0]
        self.assertEqual(segment.type, "cartesian")
        # Every waypoint but the last is an intermediate pose...
        self.assertEqual(len(segment.waypoints), 1)
        wp = segment.waypoints[0]
        self.assertAlmostEqual(wp.position.x, -0.6 + 1.5, places=6)
        # ...and target_pose carries the LAST waypoint, not the zero pose.
        self.assertAlmostEqual(segment.target_pose.position.x, 0.08 + 1.5)
        self.assertAlmostEqual(segment.target_pose.position.z, 1.2)
        # Tool-down quat at yaw 0 is (c, s, 0, 0) = (1, 0, 0, 0) -- the
        # codebase's tool-down convention (waypoint_generator).
        self.assertAlmostEqual(segment.target_pose.orientation.x, 1.0)
        self.assertAlmostEqual(segment.target_pose.orientation.w, 0.0)

    def test_joint_path_payload_carries_knots(self):
        out = _segments(
            _winner(knots=[[0.0] * 6, [0.1] * 6]), map_to_world=self._shift)
        segment = out[0]
        self.assertEqual(segment.type, "joint_path")
        self.assertFalse(segment.allow_ompl_fallback)
        self.assertEqual(len(segment.joint_waypoints), 2)

    def test_yaw_orientations(self):
        out = _segments(_winner(yaw=1.5707963), map_to_world=self._shift)
        quat = out[0].target_pose.orientation
        # tool-down quat at yaw=pi/2 is (c, s, 0, 0) with c~s~sqrt(2)/2
        self.assertAlmostEqual(quat.x, quat.y, places=6)
        self.assertAlmostEqual(quat.z, 0.0, places=9)
        self.assertAlmostEqual(quat.w, 0.0, places=9)


if __name__ == "__main__":
    unittest.main()
