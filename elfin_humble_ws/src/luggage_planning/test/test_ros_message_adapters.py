"""dataclass <-> msg round trips for luggage_planning adapters.

Needs a ROS environment (message types); skipped under plain pytest.
"""

import unittest

import pytest

pytest.importorskip("luggage_msgs")

import numpy as np  # noqa: E402

from luggage_planning import ros_message_adapters as adapters  # noqa: E402
from luggage_planning.pose import MotionSegment, Point, Pose, Quaternion  # noqa: E402


def _sample_pose():
    return Pose(
        position=Point(x=-1.0, y=0.05, z=1.2),
        orientation=Quaternion(x=0.0, y=0.0, z=0.38268, w=0.92388),
    )


class TestPoseRoundTrip(unittest.TestCase):

    def test_pose_round_trip(self):
        pose = _sample_pose()
        back = adapters.pose_from_msg(adapters.pose_to_msg(pose))
        self.assertAlmostEqual(back.position.x, pose.position.x, places=9)
        self.assertAlmostEqual(back.position.z, pose.position.z, places=9)
        self.assertAlmostEqual(back.orientation.z, pose.orientation.z, places=9)
        self.assertAlmostEqual(back.orientation.w, pose.orientation.w, places=9)

    def test_segment_round_trip(self):
        seg = MotionSegment(
            name="pick_retreat", type="cartesian", target_pose=_sample_pose(),
            waypoints=[_sample_pose(), _sample_pose()],
            keep_tool_down=True, allow_ompl_fallback=True)
        back = adapters.segment_from_msg(adapters.segment_to_msg(seg))
        self.assertEqual(back.name, seg.name)
        self.assertEqual(back.type, seg.type)
        self.assertTrue(back.keep_tool_down)
        self.assertFalse(back.keep_camera_down)
        self.assertFalse(back.lock_wrist)
        self.assertTrue(back.allow_ompl_fallback)
        self.assertEqual(len(back.waypoints), 2)
        self.assertAlmostEqual(back.target_pose.position.x,
                               seg.target_pose.position.x, places=9)

    def test_pick_from_detected_yaw(self):
        from luggage_msgs.msg import DetectedLuggage

        msg = DetectedLuggage()
        msg.pose.orientation.z = 0.3826834
        msg.pose.orientation.w = 0.9238795  # yaw = pi/4
        msg.width, msg.depth, msg.height = 0.6, 0.4, 0.3
        msg.yaw_valid = True
        pick = adapters.pick_from_detected(msg)
        self.assertAlmostEqual(pick.yaw, np.pi / 4.0, places=4)
        self.assertTrue(pick.yaw_valid)
        self.assertEqual(pick.height, 0.3)
        self.assertAlmostEqual(pick.pose.orientation.z, 0.3826834, places=6)
        # build_sequence accesses these by attribute
        self.assertTrue(hasattr(pick, "pose") and hasattr(pick, "height"))

    def test_container_geometry_round_trip_includes_chamfer_identity(self):
        from luggage_msgs.msg import ContainerOpeningEstimate

        descriptor = {
            "schema_version": 1,
            "frame_id": "container_link",
            "length": 1.49,
            "width": 1.97,
            "floor_z": 0.53,
            "ceiling_z": 2.01,
            "chamfer": {
                "side": "positive_y",
                "floor_y": 0.55,
                "wall_y": 0.985,
                "wall_z": 0.90,
            },
        }
        msg = adapters.geometry_to_opening_estimate(
            ContainerOpeningEstimate(), descriptor, geometry_version=9)
        envelope = adapters.geometry_from_opening_estimate(msg)
        self.assertEqual(envelope.geometry_version, 9)
        self.assertIn("chamfer", envelope.descriptor)
        self.assertEqual(envelope.geometry_hash, msg.geometry_hash)

    def test_container_geometry_adapter_rejects_inner_size_mismatch(self):
        from luggage_msgs.msg import ContainerOpeningEstimate

        descriptor = {
            "schema_version": 1,
            "frame_id": "container_link",
            "length": 1.0,
            "width": 1.0,
            "floor_z": 0.0,
            "ceiling_z": 1.0,
        }
        msg = adapters.geometry_to_opening_estimate(
            ContainerOpeningEstimate(), descriptor, geometry_version=1)
        msg.inner_size[0] = 2.0
        with self.assertRaisesRegex(ValueError, "geometry_identity_mismatch"):
            adapters.geometry_from_opening_estimate(msg)


if __name__ == "__main__":
    unittest.main()
