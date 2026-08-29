#!/usr/bin/env python3
"""Unit tests for SensorPreprocessor (no roscore required)."""

import unittest

import numpy as np

from luggage_perception.motion_stability_filter import MotionStabilityGate
from luggage_perception.sensor_preprocessor import (
    SensorPreprocessor,
    transform_points,
)
from luggage_perception.sensor_types import (
    CameraCloud,
    CameraInfoFrame,
    DepthFrame,
    ImuSample,
    LidarScan,
    RgbFrame,
    JointSample,
)


def _rgb(stamp, value=1):
    image = np.full((2, 2, 3), value, dtype=np.uint8)
    return RgbFrame(stamp=stamp, frame_id="optical", image=image, encoding="rgb8")


def _depth(stamp, value=1000):
    depth = np.full((2, 2), value, dtype=np.uint16)
    return DepthFrame(
        stamp=stamp, frame_id="optical", depth=depth,
        units="millimetres", encoding="16UC1",
    )


def _info(stamp):
    return CameraInfoFrame(
        stamp=stamp, frame_id="optical", width=2, height=2,
        fx=100.0, fy=100.0, cx=1.0, cy=1.0,
    )


def _cloud(stamp, points=None, data_frame="camera_link"):
    if points is None:
        points = np.array([[1.0, 0.0, 0.0]], dtype=np.float64)
    return CameraCloud(
        stamp=stamp, frame_id="optical", data_frame=data_frame, points=points,
    )


def _joint(stamp, position=0.0):
    return JointSample(
        stamp=stamp,
        joint_names=("j1",),
        positions=np.array([position], dtype=np.float64),
        velocities=np.array([0.0], dtype=np.float64),
    )


def _stable_gate():
    return MotionStabilityGate(
        joint_names=["j1"],
        velocity_threshold=0.02,
        settle_time_sec=0.05,
        joint_state_timeout_sec=2.0,
    )


def _hold_stable(pre, t0=1.0):
    for i in range(4):
        pre.update_joint_state(_joint(t0 + 0.05 * i, 0.0))


class TestSensorPreprocessor(unittest.TestCase):
    def test_pairs_rgb_primary_with_nearest_depth_and_cloud(self):
        pre = SensorPreprocessor(
            camera_slop_sec=0.020, motion_gate=_stable_gate(), joint_names=["j1"])
        _hold_stable(pre)
        self.assertIsNone(pre.update_depth(_depth(1.201)))
        self.assertIsNone(pre.update_camera_info(_info(1.200)))
        self.assertIsNone(pre.update_camera_cloud(
            _cloud(1.199), point_transform=np.eye(4)))
        obs = pre.update_rgb(_rgb(1.200))
        self.assertIsNotNone(obs)
        self.assertEqual(obs.primary_stamp, 1.200)
        self.assertTrue(obs.flags.rgb_ok)
        self.assertTrue(obs.flags.depth_ok)
        self.assertTrue(obs.flags.geometry_ok)
        self.assertTrue(obs.flags.cloud_ok)
        self.assertAlmostEqual(obs.depth_dt, 0.001)
        np.testing.assert_allclose(obs.camera_points, [[1.0, 0.0, 0.0]])

    def test_out_of_order_callbacks_still_pair(self):
        pre = SensorPreprocessor(
            camera_slop_sec=0.020, motion_gate=_stable_gate(), joint_names=["j1"])
        _hold_stable(pre)
        self.assertIsNone(pre.update_rgb(_rgb(2.0)))
        self.assertIsNone(pre.update_depth(_depth(2.005)))
        obs = pre.update_camera_cloud(_cloud(2.002), point_transform=np.eye(4))
        self.assertIsNotNone(obs)
        self.assertTrue(obs.flags.depth_ok)
        self.assertEqual(obs.primary_stamp, 2.0)

    def test_duplicate_primary_stamp_emitted_once(self):
        pre = SensorPreprocessor(camera_slop_sec=0.020)
        pre.update_depth(_depth(3.0))
        self.assertIsNone(pre.update_rgb(_rgb(3.0)))
        first = pre.update_camera_cloud(_cloud(3.0), point_transform=np.eye(4))
        second = pre.update_depth(_depth(3.001))
        self.assertIsNotNone(first)
        self.assertIsNone(second)

    def test_slop_rejection_emits_without_depth(self):
        pre = SensorPreprocessor(camera_slop_sec=0.020)
        self.assertIsNone(pre.update_rgb(_rgb(4.0)))
        obs = pre.update_depth(_depth(4.05))
        self.assertIsNotNone(obs)
        self.assertTrue(obs.flags.rgb_ok)
        self.assertFalse(obs.flags.depth_ok)

    def test_zero_stamp_rejected(self):
        pre = SensorPreprocessor()
        self.assertIsNone(pre.update_rgb(_rgb(0.0)))
        self.assertEqual(pre.last_rejection_reason(), "invalid_rgb_stamp")

    def test_time_rollback_resets_stream_buffer(self):
        pre = SensorPreprocessor(camera_slop_sec=0.020, rollback_sec=0.25)
        pre.update_rgb(_rgb(5.0))
        self.assertEqual(pre.buffer_occupancy()["rgb"], 1)
        pre.update_rgb(_rgb(4.6))
        self.assertEqual(pre.buffer_occupancy()["rgb"], 1)
        self.assertEqual(pre._rgb.stamps(), [4.6])

    def test_unexpected_depth_encoding_rejected(self):
        pre = SensorPreprocessor()
        bad = DepthFrame(
            stamp=1.0, frame_id="optical",
            depth=np.ones((2, 2), dtype=np.float32),
            units="metres", encoding="32FC1",
        )
        self.assertIsNone(pre.update_depth(bad))
        self.assertEqual(pre.last_rejection_reason(), "unexpected_depth_encoding")

    def test_nonfinite_points_removed_and_transform_applied(self):
        pre = SensorPreprocessor(
            camera_slop_sec=0.020, motion_gate=_stable_gate(), joint_names=["j1"])
        _hold_stable(pre, t0=6.0)
        points = np.array([
            [1.0, 0.0, 0.0],
            [np.inf, 0.0, 0.0],
            [0.0, np.nan, 1.0],
        ], dtype=np.float64)
        matrix = np.eye(4)
        matrix[0, 3] = 0.5
        pre.update_depth(_depth(6.20))
        pre.update_camera_cloud(
            _cloud(6.20, points=points), point_transform=matrix)
        obs = pre.update_rgb(_rgb(6.20))
        self.assertEqual(obs.dropped_nonfinite, 2)
        np.testing.assert_allclose(obs.camera_points, [[1.5, 0.0, 0.0]])

    def test_copy_output_is_isolated(self):
        pre = SensorPreprocessor(camera_slop_sec=0.020)
        pre.update_depth(_depth(7.0))
        pre.update_camera_cloud(_cloud(7.0), point_transform=np.eye(4))
        obs = pre.update_rgb(_rgb(7.0, value=3))
        copy_a = pre.copy_output()
        copy_a.rgb.image[:] = 9
        copy_b = pre.copy_output()
        self.assertEqual(int(copy_b.rgb.image[0, 0, 0]), 3)
        self.assertIsNot(obs.rgb.image, copy_b.rgb.image)

    def test_missing_joints_suppress_geometry_but_keep_images(self):
        pre = SensorPreprocessor(camera_slop_sec=0.020, joint_names=["j1"])
        pre.update_depth(_depth(8.0))
        pre.update_camera_cloud(_cloud(8.0), point_transform=np.eye(4))
        obs = pre.update_rgb(_rgb(8.0))
        self.assertTrue(obs.flags.rgb_ok)
        self.assertTrue(obs.flags.depth_ok)
        self.assertFalse(obs.flags.geometry_ok)
        self.assertIsNone(obs.camera_points)

    def test_moving_arm_suppresses_geometry(self):
        pre = SensorPreprocessor(
            camera_slop_sec=0.020, motion_gate=_stable_gate(), joint_names=["j1"])
        _hold_stable(pre, t0=9.0)
        pre.update_joint_state(_joint(9.30, 0.2))
        pre.update_depth(_depth(9.30))
        pre.update_camera_cloud(_cloud(9.30), point_transform=np.eye(4))
        obs = pre.update_rgb(_rgb(9.30))
        self.assertTrue(obs.flags.rgb_ok)
        self.assertTrue(obs.flags.motion_too_large)
        self.assertFalse(obs.flags.geometry_ok)
        self.assertIsNone(obs.camera_points)

    def test_lidar_never_blocks_or_attaches(self):
        pre = SensorPreprocessor(camera_slop_sec=0.020)
        scan = LidarScan(
            stamp_start=10.0, stamp_end=10.0, frame_id="livox_frame",
            points=np.array([[0.0, 0.0, 1.0]]),
            point_times=np.array([10.0]),
        )
        pre.update_lidar(scan)
        pre.update_imu(ImuSample(
            stamp=10.0, frame_id="livox_frame",
            angular_velocity=np.zeros(3),
            linear_acceleration=np.array([0.0, 0.0, 9.8]),
        ))
        pre.update_depth(_depth(10.0))
        pre.update_rgb(_rgb(10.0))
        obs = pre.update_camera_cloud(_cloud(10.0), point_transform=np.eye(4))
        self.assertTrue(obs.flags.rgb_ok)
        self.assertFalse(obs.flags.lidar_ok)
        self.assertFalse(obs.flags.deskewed)
        self.assertIsNone(obs.lidar_points)
        self.assertEqual(pre.buffer_occupancy()["lidar"], 1)
        self.assertEqual(pre.buffer_occupancy()["imu"], 1)

    def test_lidar_output_rejected_at_construction(self):
        with self.assertRaises(ValueError):
            SensorPreprocessor(enable_lidar_output=True)

    def test_waits_for_late_cloud_inside_slop(self):
        pre = SensorPreprocessor(
            camera_slop_sec=0.020, motion_gate=_stable_gate(), joint_names=["j1"])
        _hold_stable(pre, t0=16.0)
        pre.update_depth(_depth(16.20))
        self.assertIsNone(pre.update_rgb(_rgb(16.20)))
        obs = pre.update_camera_cloud(_cloud(16.210), point_transform=np.eye(4))
        self.assertIsNotNone(obs)
        self.assertTrue(obs.flags.cloud_ok)
        self.assertTrue(obs.flags.geometry_ok)

    def test_lidar_buffer_stays_bounded(self):
        pre = SensorPreprocessor(lidar_maxlen=4)
        for i in range(6):
            pre.update_lidar(LidarScan(
                stamp_start=11.0 + 0.1 * i,
                stamp_end=11.05 + 0.1 * i,
                frame_id="livox_frame",
                points=np.array([[float(i), 0.0, 0.0]]),
            ))
        self.assertEqual(pre.buffer_occupancy()["lidar"], 4)

    def test_stale_joints_suppress_geometry(self):
        pre = SensorPreprocessor(
            camera_slop_sec=0.020, motion_gate=_stable_gate(), joint_names=["j1"])
        _hold_stable(pre, t0=12.0)
        pre.update_depth(_depth(15.0))
        pre.update_camera_cloud(_cloud(15.0), point_transform=np.eye(4))
        obs = pre.update_rgb(_rgb(15.0))
        self.assertTrue(obs.flags.rgb_ok)
        self.assertFalse(obs.flags.geometry_ok)
        self.assertIsNone(obs.camera_points)

    def test_transform_points_helper(self):
        matrix = np.eye(4)
        matrix[1, 3] = 2.0
        out = transform_points([[0.0, 1.0, 0.0]], matrix)
        np.testing.assert_allclose(out, [[0.0, 3.0, 0.0]])


if __name__ == "__main__":
    unittest.main()
