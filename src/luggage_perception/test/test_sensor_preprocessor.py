#!/usr/bin/env python3
"""Unit tests for SensorPreprocessor (depth-primary, no roscore required).

PF-R9 g2: the canonical acquisition is colour + aligned depth + camera
infos under one exact stamp; the camera cloud path no longer exists.
"""

import unittest

import numpy as np

from luggage_perception.motion_stability_filter import MotionStabilityGate
from luggage_perception.sensor_preprocessor import (
    SensorPreprocessor,
    transform_points,
)
from luggage_perception.sensor_types import (
    CameraInfoFrame,
    DepthFrame,
    ImuSample,
    LidarScan,
    OpaquePayload,
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
    def test_pairs_rgb_primary_with_nearest_depth(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.020, motion_gate=_stable_gate(),
            joint_names=["j1"])
        _hold_stable(pre)
        self.assertIsNone(pre.update_depth(_depth(1.201)))
        self.assertIsNone(pre.update_camera_info(_info(1.200)))
        obs = pre.update_rgb(_rgb(1.200))
        self.assertIsNotNone(obs)
        self.assertEqual(obs.primary_stamp, 1.200)
        self.assertTrue(obs.flags.rgb_ok)
        self.assertTrue(obs.flags.depth_ok)
        self.assertTrue(obs.flags.color_info_ok)
        self.assertTrue(obs.flags.depth_info_ok)
        self.assertTrue(obs.flags.geometry_ok)
        self.assertFalse(obs.flags.cloud_ok)
        self.assertAlmostEqual(obs.depth_dt, 0.001)

    def test_out_of_order_callbacks_still_pair(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.020, motion_gate=_stable_gate(),
            joint_names=["j1"])
        _hold_stable(pre)
        self.assertIsNone(pre.update_rgb(_rgb(2.0)))
        self.assertIsNone(pre.update_camera_info(_info(2.0)))
        obs = pre.update_depth(_depth(2.005))
        self.assertIsNotNone(obs)
        self.assertTrue(obs.flags.depth_ok)
        self.assertEqual(obs.primary_stamp, 2.0)

    def test_duplicate_primary_stamp_emitted_once(self):
        pre = SensorPreprocessor(camera_pair_tolerance_sec=0.020)
        pre.update_depth(_depth(3.0))
        pre.update_camera_info(_info(3.0))
        first = pre.update_rgb(_rgb(3.0))
        second = pre.update_depth(_depth(3.001))
        self.assertIsNotNone(first)
        self.assertIsNone(second)

    def test_depth_deadline_skip_is_flagged_never_partial(self):
        # PF-R9 g2: aligned depth is mandatory. Outside the pairing
        # tolerance the stamp waits for the deadline on the camera clock,
        # then is skipped with a named counter — never a partial emission.
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.020,
            camera_wait_deadline_sec=0.060)
        self.assertIsNone(pre.update_rgb(_rgb(4.0)))
        self.assertIsNone(pre.update_camera_info(_info(4.0)))
        self.assertIsNone(pre.update_depth(_depth(4.05)))
        self.assertIsNone(pre.update_rgb(_rgb(4.21)))
        self.assertEqual("depth_wait_timeout", pre.last_rejection_reason())
        self.assertEqual(1, pre.depth_wait_skips)

    def test_zero_stamp_rejected(self):
        pre = SensorPreprocessor()
        self.assertIsNone(pre.update_rgb(_rgb(0.0)))
        self.assertEqual(pre.last_rejection_reason(), "invalid_rgb_stamp")

    def test_time_rollback_resets_stream_buffer_and_epoch(self):
        pre = SensorPreprocessor(rollback_sec=0.25)
        pre.update_rgb(_rgb(5.0))
        self.assertEqual(pre.buffer_occupancy()["rgb"], 1)
        self.assertEqual(0, pre.camera_epoch)
        pre.update_rgb(_rgb(4.6))
        self.assertEqual(pre.buffer_occupancy()["rgb"], 1)
        self.assertEqual(pre._rgb.stamps(), [4.6])
        self.assertEqual(1, pre.camera_epoch)
        self.assertEqual(1, pre.rollback_events)

    def test_unexpected_depth_encoding_rejected(self):
        pre = SensorPreprocessor()
        bad = DepthFrame(
            stamp=1.0, frame_id="optical",
            depth=np.ones((2, 2), dtype=np.float32),
            units="metres", encoding="32FC1",
        )
        self.assertIsNone(pre.update_depth(bad))
        self.assertEqual(pre.last_rejection_reason(), "unexpected_depth_encoding")

    def test_acquisition_dimension_mismatch_fails_closed(self):
        pre = SensorPreprocessor(camera_pair_tolerance_sec=0.020)
        odd = DepthFrame(
            stamp=7.0, frame_id="optical",
            depth=np.zeros((2, 4), dtype=np.uint16),
            units="millimetres", encoding="16UC1")
        pre.update_depth(odd)
        pre.update_camera_info(_info(7.0))
        self.assertIsNone(pre.update_rgb(_rgb(7.0)))
        self.assertEqual("acquisition_dimension_mismatch",
                         pre.last_rejection_reason())
        self.assertEqual(1, pre.acquisition_mismatches)

    def test_acquisition_frame_mismatch_fails_closed(self):
        pre = SensorPreprocessor(camera_pair_tolerance_sec=0.020)
        pre.update_depth(_depth(7.0))  # frame "optical"
        off = _info(7.0)
        pre.update_camera_info(off)
        rgb = RgbFrame(stamp=7.0, frame_id="other_frame",
                       image=np.zeros((2, 2, 3), dtype=np.uint8),
                       encoding="rgb8")
        self.assertIsNone(pre.update_rgb(rgb))
        self.assertEqual("acquisition_frame_mismatch",
                         pre.last_rejection_reason())

    def test_acquisition_intrinsics_mismatch_fails_closed(self):
        pre = SensorPreprocessor(camera_pair_tolerance_sec=0.020)
        pre.update_depth(_depth(7.0))
        pre.update_camera_info(_info(7.0))
        skewed = CameraInfoFrame(
            stamp=7.0, frame_id="optical", width=2, height=2,
            fx=222.0, fy=100.0, cx=1.0, cy=1.0)
        pre.update_camera_info(skewed, slots=("depth",))
        self.assertIsNone(pre.update_rgb(_rgb(7.0)))
        self.assertEqual("acquisition_intrinsics_mismatch",
                         pre.last_rejection_reason())

    def test_missing_infos_fail_closed(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.020,
            camera_wait_deadline_sec=0.060)
        pre.update_depth(_depth(7.0))
        self.assertIsNone(pre.update_rgb(_rgb(7.0)))
        self.assertIsNone(pre.update_rgb(_rgb(7.21)))
        self.assertEqual("missing_camera_info", pre.last_rejection_reason())
        self.assertEqual(1, pre.info_wait_skips)

    def test_copy_output_shares_payload_and_isolates_metadata(self):
        pre = SensorPreprocessor(camera_pair_tolerance_sec=0.020)
        pre.update_depth(_depth(7.0))
        pre.update_camera_info(_info(7.0))
        obs = pre.update_rgb(_rgb(7.0, value=3))
        copy_a = pre.copy_output()
        # Pixel views are read-only: mutation raises, never corrupts.
        with self.assertRaises(ValueError):
            copy_a.rgb.view()[:] = 9
        copy_b = pre.copy_output()
        self.assertEqual(int(copy_b.rgb.view()[0, 0, 0]), 3)
        # Payload identity survives the copy-out boundary.
        self.assertIs(obs.rgb.payload, copy_b.rgb.payload)

    def test_payload_identity_survives_insert_and_copy(self):
        source = np.full((2, 2, 3), 7, dtype=np.uint8).tobytes()
        import array as _array
        raw = _array.array("B", source)
        frame = RgbFrame(
            stamp=7.0, frame_id="optical", encoding="rgb8",
            payload=OpaquePayload(raw), height=2, width=2, step=6,
            stamp_key=(7, 0))
        pre = SensorPreprocessor(camera_pair_tolerance_sec=0.020)
        pre.update_depth(_depth(7.0))
        pre.update_camera_info(_info(7.0))
        obs = pre.update_rgb(frame)
        self.assertIsNotNone(obs)
        self.assertIs(raw, obs.rgb.payload.ros_data())
        copied = obs.copy()
        self.assertIs(raw, copied.rgb.payload.ros_data())

    def test_missing_joints_suppress_geometry_but_keep_depth(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.020, joint_names=["j1"])
        pre.update_depth(_depth(8.0))
        pre.update_camera_info(_info(8.0))
        obs = pre.update_rgb(_rgb(8.0))
        self.assertTrue(obs.flags.rgb_ok)
        self.assertTrue(obs.flags.depth_ok)
        self.assertFalse(obs.flags.geometry_ok)

    def test_disabled_gate_keeps_geometry_without_joints(self):
        gate = MotionStabilityGate(joint_names=["j1"], enabled=False)
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.020, motion_gate=gate,
            joint_names=["j1"])
        pre.update_depth(_depth(8.0))
        pre.update_camera_info(_info(8.0))
        obs = pre.update_rgb(_rgb(8.0))
        self.assertTrue(obs.flags.geometry_ok)
        self.assertFalse(obs.flags.motion_too_large)
        self.assertAlmostEqual(pre.diagnostics()["last_geometry_ok_stamp"], 8.0)

    def test_moving_arm_flags_geometry(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.020, motion_gate=_stable_gate(),
            joint_names=["j1"])
        _hold_stable(pre, t0=9.0)
        pre.update_joint_state(_joint(9.30, 0.2))
        pre.update_depth(_depth(9.30))
        pre.update_camera_info(_info(9.30))
        obs = pre.update_rgb(_rgb(9.30))
        self.assertTrue(obs.flags.rgb_ok)
        self.assertTrue(obs.flags.motion_too_large)
        self.assertFalse(obs.flags.geometry_ok)
        self.assertTrue(obs.flags.depth_ok)

    def test_last_geometry_ok_stamp_freezes_while_moving(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.020, motion_gate=_stable_gate(),
            joint_names=["j1"])
        _hold_stable(pre, t0=9.0)
        pre.update_depth(_depth(9.20))
        pre.update_camera_info(_info(9.20))
        obs = pre.update_rgb(_rgb(9.20))
        self.assertTrue(obs.flags.geometry_ok)
        diag = pre.diagnostics()
        self.assertAlmostEqual(diag["primary_stamp"], 9.20)
        self.assertAlmostEqual(diag["last_geometry_ok_stamp"], 9.20)

        pre.update_joint_state(_joint(9.40, 0.2))
        pre.update_depth(_depth(9.40))
        pre.update_camera_info(_info(9.40))
        obs = pre.update_rgb(_rgb(9.40))
        self.assertFalse(obs.flags.geometry_ok)
        diag = pre.diagnostics()
        self.assertAlmostEqual(diag["primary_stamp"], 9.40)
        self.assertAlmostEqual(diag["last_geometry_ok_stamp"], 9.20)

        for i in range(4):
            pre.update_joint_state(_joint(9.50 + 0.05 * i, 0.2))
        pre.update_depth(_depth(9.70))
        pre.update_camera_info(_info(9.70))
        obs = pre.update_rgb(_rgb(9.70))
        self.assertTrue(obs.flags.geometry_ok)
        diag = pre.diagnostics()
        self.assertAlmostEqual(diag["primary_stamp"], 9.70)
        self.assertAlmostEqual(diag["last_geometry_ok_stamp"], 9.70)

    def test_missing_joints_leave_geometry_ok_stamp_zero(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.020, joint_names=["j1"])
        pre.update_depth(_depth(8.0))
        pre.update_camera_info(_info(8.0))
        pre.update_rgb(_rgb(8.0))
        self.assertAlmostEqual(pre.diagnostics()["last_geometry_ok_stamp"], 0.0)
        pre = SensorPreprocessor(camera_pair_tolerance_sec=0.020)
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
        pre.update_camera_info(_info(10.0))
        obs = pre.update_rgb(_rgb(10.0))
        self.assertTrue(obs.flags.rgb_ok)
        self.assertFalse(obs.flags.lidar_ok)
        self.assertFalse(obs.flags.deskewed)
        self.assertIsNone(obs.lidar_points)
        self.assertEqual(pre.buffer_occupancy()["lidar"], 1)
        self.assertEqual(pre.buffer_occupancy()["imu"], 1)

    def test_lidar_output_rejected_at_construction(self):
        with self.assertRaises(ValueError):
            SensorPreprocessor(enable_lidar_output=True)

    def test_waits_for_late_depth_inside_tolerance(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.020, motion_gate=_stable_gate(),
            joint_names=["j1"])
        _hold_stable(pre, t0=16.0)
        pre.update_camera_info(_info(16.20))
        self.assertIsNone(pre.update_rgb(_rgb(16.20)))
        obs = pre.update_depth(_depth(16.210))
        self.assertIsNotNone(obs)
        self.assertTrue(obs.flags.depth_ok)
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

    def test_camera_buffers_bounded_at_fifteen(self):
        pre = SensorPreprocessor(camera_maxlen=15, camera_horizon_sec=1.0)
        for i in range(20):
            pre.update_rgb(_rgb(20.0 + 0.02 * i))
            pre.update_depth(_depth(20.0 + 0.02 * i))
            pre.update_camera_info(_info(20.0 + 0.02 * i))
        occupancy = pre.buffer_occupancy()
        self.assertEqual(15, occupancy["rgb"])
        self.assertEqual(15, occupancy["depth"])
        self.assertEqual(15, occupancy["color_info"])
        self.assertEqual(15, occupancy["depth_info"])

    def test_camera_cache_not_aged_by_joint_stamps(self):
        # Joint messages far in the future must not prune camera payloads
        # (camera eviction runs on the canonical camera clock only).
        pre = SensorPreprocessor(camera_maxlen=15, camera_horizon_sec=1.0)
        pre.update_rgb(_rgb(30.0))
        pre.update_depth(_depth(30.0))
        pre.update_camera_info(_info(30.0))
        for i in range(10):
            pre.update_joint_state(_joint(50.0 + 0.05 * i))
        self.assertEqual(1, pre.buffer_occupancy()["rgb"])
        self.assertEqual(1, pre.buffer_occupancy()["depth"])

    def test_same_stamp_replacement_no_occupancy_growth(self):
        pre = SensorPreprocessor(camera_maxlen=15)
        for i in range(5):
            pre.update_rgb(_rgb(30.0, value=i))
        self.assertEqual(1, pre.buffer_occupancy()["rgb"])
        self.assertEqual(4, pre._rgb.replaced)

    def test_stale_joints_suppress_geometry(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.020, motion_gate=_stable_gate(),
            joint_names=["j1"])
        _hold_stable(pre, t0=12.0)
        pre.update_depth(_depth(15.0))
        pre.update_camera_info(_info(15.0))
        obs = pre.update_rgb(_rgb(15.0))
        self.assertTrue(obs.flags.rgb_ok)
        self.assertFalse(obs.flags.geometry_ok)
        self.assertTrue(obs.flags.depth_ok)

    def test_transform_points_helper(self):
        matrix = np.eye(4)
        matrix[1, 3] = 2.0
        out = transform_points([[0.0, 1.0, 0.0]], matrix)
        np.testing.assert_allclose(out, [[0.0, 3.0, 0.0]])


if __name__ == "__main__":
    unittest.main()
