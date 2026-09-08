#!/usr/bin/env python3
"""PF-G0A: the E0 platform-free contract survives the planning adapter.

docs/plans/platform_free_height_remediation.md PF-R1. The live pick path
is waypoint_generator_node -> pick_from_detected -> build_sequence ->
pick_contact_top_z; every E0 field must arrive intact or the live path
either crashes or silently invents geometry.
"""

import unittest

import pytest

from luggage_msgs.msg import DetectedLuggage

pytest.importorskip("luggage_msgs.msg")

from luggage_planning.ros_message_adapters import pick_from_detected  # noqa: E402
from luggage_planning.waypoint_generator import (  # noqa: E402
    DEFAULT_PICK_CLEARANCES,
    FULL_GEOMETRY_REQUIRED,
    build_sequence,
    pick_contact_top_z,
)
from luggage_planning.pose import Point, Pose, Quaternion  # noqa: E402


def _full_geometry_msg():
    """Measured FULL_3D detection: top at 0.75, center at 0.60, height 0.3.

    The center-derived top (0.60 + 0.15 = 0.75) is deliberately *wrong* by
    5 cm versus top_surface_pose.z so a test can tell which one was used.
    """
    msg = DetectedLuggage()
    msg.id = "suite_0001"
    msg.header.frame_id = "world"
    msg.header.stamp.sec = 1234
    msg.header.stamp.nanosec = 567000000
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
        0.30, -0.10, 0.60)
    msg.pose.orientation.w = 1.0
    msg.top_surface_pose.position.x = 0.30
    msg.top_surface_pose.position.y = -0.10
    msg.top_surface_pose.position.z = 0.75
    msg.top_surface_pose.orientation.w = 1.0
    msg.top_surface_valid = True
    msg.top_surface_confidence = 0.88
    msg.width, msg.depth, msg.height = 0.55, 0.35, 0.30
    msg.yaw_valid = True
    msg.aspect_ratio = 1.57
    msg.height_valid = True
    msg.height_confidence = 0.71
    msg.height_source = DetectedLuggage.HEIGHT_SOURCE_MEASURED_SUPPORT
    return msg


class TestPickFromDetectedContract(unittest.TestCase):

    def test_every_e0_field_survives_conversion(self):
        msg = _full_geometry_msg()
        pick = pick_from_detected(msg)
        self.assertTrue(pick.top_surface_valid)
        self.assertAlmostEqual(pick.top_surface_confidence, 0.88, places=9)
        self.assertAlmostEqual(
            pick.top_surface_pose.position.z, 0.75, places=9)
        self.assertTrue(pick.height_valid)
        self.assertAlmostEqual(pick.height_confidence, 0.71, places=9)
        self.assertEqual(
            pick.height_source,
            DetectedLuggage.HEIGHT_SOURCE_MEASURED_SUPPORT)
        self.assertAlmostEqual(
            pick.acquisition_stamp_sec, 1234.567, places=6)
        self.assertEqual(pick.acquisition_frame, "world")
        self.assertAlmostEqual(pick.height, 0.30, places=9)
        self.assertAlmostEqual(pick.width, 0.55, places=9)
        self.assertAlmostEqual(pick.depth, 0.35, places=9)
        self.assertTrue(pick.yaw_valid)
        self.assertAlmostEqual(pick.aspect_ratio, 1.57, places=9)
        self.assertEqual(pick.detection_id, "suite_0001")
        # Structured pose, not the mutable ROS message.
        self.assertIsInstance(pick.pose, Pose)
        self.assertIsInstance(pick.top_surface_pose, Pose)

    def test_pick_contact_top_z_uses_top_surface_pose(self):
        pick = pick_from_detected(_full_geometry_msg())
        # top_surface_pose.z (0.75) must win over center + height/2 (0.75
        # here would tie; use the message where they differ).
        msg = _full_geometry_msg()
        msg.pose.position.z = 0.58  # center-derived top would be 0.73
        pick = pick_from_detected(msg)
        self.assertAlmostEqual(pick_contact_top_z(pick), 0.75, places=9)

    def test_build_sequence_uses_top_surface_z_through_adapter(self):
        pick = pick_from_detected(_full_geometry_msg())
        segs = build_sequence(pick, _slot(), "pick")
        self.assertAlmostEqual(
            segs[0].target_pose.position.z,
            0.75 + DEFAULT_PICK_CLEARANCES["pre_grasp"], places=9)

    def test_prior_only_height_cannot_drive_pick(self):
        msg = _full_geometry_msg()
        msg.top_surface_valid = False
        msg.height_valid = False  # catalog prior populated the number only
        msg.height_source = DetectedLuggage.HEIGHT_SOURCE_CATALOG_PRIOR
        pick = pick_from_detected(msg)
        self.assertFalse(pick.height_valid)
        with self.assertRaises(ValueError) as ctx:
            pick_contact_top_z(pick)
        self.assertIn(FULL_GEOMETRY_REQUIRED, str(ctx.exception))
        with self.assertRaises(ValueError):
            build_sequence(pick, _slot(), "pick")

    def test_top_only_pick_uses_measured_top_z(self):
        """TOP_ONLY is valid for picking (top measured), invalid for height."""
        msg = _full_geometry_msg()
        msg.height_valid = False
        msg.height_source = DetectedLuggage.HEIGHT_SOURCE_UNAVAILABLE
        pick = pick_from_detected(msg)
        self.assertTrue(pick.top_surface_valid)
        self.assertFalse(pick.height_valid)
        self.assertAlmostEqual(pick_contact_top_z(pick), 0.75, places=9)

    def test_default_message_fails_closed(self):
        """An empty/default DetectedLuggage has no measured geometry."""
        pick = pick_from_detected(DetectedLuggage())
        self.assertFalse(pick.top_surface_valid)
        self.assertFalse(pick.height_valid)
        with self.assertRaises(ValueError):
            pick_contact_top_z(pick)


def _slot():
    from luggage_msgs.msg import SlotSpec

    slot = SlotSpec()
    slot.place_pose.position.x, slot.place_pose.position.y = 0.6, 0.0
    slot.place_pose.position.z = 0.655
    slot.place_pose.orientation.w = 1.0
    slot.height = 0.30
    return slot


if __name__ == "__main__":
    unittest.main()
