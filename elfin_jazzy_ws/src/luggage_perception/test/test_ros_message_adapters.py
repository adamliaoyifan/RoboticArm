#!/usr/bin/env python3
"""Unit tests for ros_message_adapters (needs sensor_msgs, no roscore)."""

import unittest

import numpy as np
import pytest

pytest.importorskip("sensor_msgs")

from geometry_msgs.msg import TransformStamped  # noqa: E402
from sensor_msgs.msg import (  # noqa: E402
    CameraInfo,
    Image,
    JointState,
    PointCloud2,
    PointField,
)

from luggage_perception import ros_message_adapters as adapters  # noqa: E402
from luggage_perception.sensor_types import (  # noqa: E402
    CameraInfoFrame,
    DepthFrame,
    RgbFrame,
)


def _make_cloud(points, offsets=(0, 4, 8), point_step=12, extra_fields=(),
                datatype=PointField.FLOAT32, truncate=0, is_bigendian=False):
    """Pack (N,3) xyz at arbitrary offsets inside `point_step` bytes."""
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    fmt = ">f4" if is_bigendian else "<f4"
    buffer = np.zeros((len(pts), point_step), dtype=np.uint8)
    for axis, offset in enumerate(offsets):
        column = pts[:, axis].astype(fmt).view(np.uint8).reshape(len(pts), 4)
        buffer[:, offset:offset + 4] = column

    msg = PointCloud2()
    msg.header.frame_id = "camera_depth_optical_frame"
    msg.height = 1
    msg.width = len(pts)
    msg.fields = [
        PointField(name="x", offset=offsets[0], datatype=datatype, count=1),
        PointField(name="y", offset=offsets[1], datatype=datatype, count=1),
        PointField(name="z", offset=offsets[2], datatype=datatype, count=1),
    ] + list(extra_fields)
    msg.is_bigendian = is_bigendian
    msg.point_step = point_step
    msg.row_step = point_step * msg.width
    msg.is_dense = True
    raw = buffer.tobytes()
    msg.data = raw[:len(raw) - truncate] if truncate else raw
    return msg


def _make_image(array, encoding, is_bigendian=0, truncate=0):
    array = np.asarray(array)
    msg = Image()
    msg.header.frame_id = "optical"
    msg.height = int(array.shape[0])
    msg.width = int(array.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = is_bigendian
    msg.step = int(array.strides[0])
    raw = array.tobytes()
    msg.data = raw[:len(raw) - truncate] if truncate else raw
    return msg


POINTS = [[1.0, 2.0, 3.0], [-4.5, 0.0, 7.25]]


class TestCloudDecoding(unittest.TestCase):
    def test_tight_xyz_layout(self):
        out = adapters.cloud_points_from_msg(_make_cloud(POINTS))
        np.testing.assert_allclose(out, POINTS)
        self.assertEqual(out.dtype, np.float64)

    def test_padded_layout_with_trailing_fields(self):
        # XYZRGB-style: 32-byte stride with rgb after a 4-byte pad.
        rgb = PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1)
        msg = _make_cloud(POINTS, point_step=32, extra_fields=(rgb,))
        np.testing.assert_allclose(adapters.cloud_points_from_msg(msg), POINTS)

    def test_reordered_offsets(self):
        msg = _make_cloud(POINTS, offsets=(8, 0, 4))
        np.testing.assert_allclose(adapters.cloud_points_from_msg(msg), POINTS)

    def test_big_endian_payload(self):
        msg = _make_cloud(POINTS, is_bigendian=True)
        np.testing.assert_allclose(adapters.cloud_points_from_msg(msg), POINTS)

    def test_truncated_buffer_keeps_whole_points(self):
        msg = _make_cloud(POINTS, truncate=6)
        out = adapters.cloud_points_from_msg(msg)
        np.testing.assert_allclose(out, POINTS[:1])

    def test_missing_z_field_rejected(self):
        msg = _make_cloud(POINTS)
        msg.fields = [f for f in msg.fields if f.name != "z"]
        self.assertIsNone(adapters.cloud_points_from_msg(msg))

    def test_non_float32_xyz_rejected(self):
        msg = _make_cloud(POINTS, datatype=PointField.FLOAT64)
        self.assertIsNone(adapters.cloud_points_from_msg(msg))

    def test_offset_outside_point_step_rejected(self):
        msg = _make_cloud(POINTS)
        msg.fields[2].offset = 10
        self.assertIsNone(adapters.cloud_points_from_msg(msg))

    def test_zero_point_step_rejected(self):
        msg = _make_cloud(POINTS)
        msg.point_step = 0
        self.assertIsNone(adapters.cloud_points_from_msg(msg))

    def test_empty_cloud_is_not_an_error(self):
        msg = _make_cloud([])
        out = adapters.cloud_points_from_msg(msg)
        self.assertIsNotNone(out)
        self.assertEqual(out.shape, (0, 3))

    def test_encode_decode_round_trip(self):
        stamp = adapters.sec_to_stamp(12.5)
        msg = adapters.cloud_msg_from_points(POINTS, stamp, "optical")
        self.assertEqual(msg.header.frame_id, "optical")
        self.assertEqual(msg.point_step, 12)
        self.assertEqual(msg.row_step, 24)
        np.testing.assert_allclose(adapters.cloud_points_from_msg(msg), POINTS)

    def test_camera_cloud_carries_data_frame(self):
        cloud = adapters.camera_cloud_from_msg(_make_cloud(POINTS), "camera_link")
        self.assertEqual(cloud.data_frame, "camera_link")
        self.assertEqual(cloud.frame_id, "camera_depth_optical_frame")
        np.testing.assert_allclose(cloud.points, POINTS)

    def test_camera_cloud_none_on_bad_layout(self):
        msg = _make_cloud(POINTS, datatype=PointField.FLOAT64)
        self.assertIsNone(adapters.camera_cloud_from_msg(msg, "camera_link"))


class TestImageAdapters(unittest.TestCase):
    def test_rgb8_round_trip(self):
        image = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
        frame = adapters.rgb_frame_from_msg(_make_image(image, "rgb8"))
        np.testing.assert_array_equal(frame.image, image)
        self.assertEqual(frame.encoding, "rgb8")

        out = adapters.image_msg_from_frame(frame, adapters.sec_to_stamp(1.0))
        self.assertEqual(out.step, 9)
        self.assertEqual(out.encoding, "rgb8")
        np.testing.assert_array_equal(
            adapters.image_array_from_msg(out), image)

    def test_mono8_decodes_two_dimensional(self):
        image = np.arange(6, dtype=np.uint8).reshape(2, 3)
        out = adapters.image_array_from_msg(_make_image(image, "mono8"))
        np.testing.assert_array_equal(out, image)

    def test_unsupported_encoding_rejected(self):
        image = np.zeros((2, 2, 3), dtype=np.uint8)
        self.assertIsNone(
            adapters.image_array_from_msg(_make_image(image, "bayer_rggb8")))
        self.assertIsNone(
            adapters.rgb_frame_from_msg(_make_image(image, "bayer_rggb8")))

    def test_truncated_image_rejected(self):
        image = np.zeros((2, 3, 3), dtype=np.uint8)
        self.assertIsNone(
            adapters.image_array_from_msg(_make_image(image, "rgb8", truncate=4)))

    def test_decoded_array_does_not_alias_message(self):
        image = np.ones((2, 2, 3), dtype=np.uint8)
        msg = _make_image(image, "rgb8")
        out = adapters.image_array_from_msg(msg)
        out[:] = 9
        np.testing.assert_array_equal(
            adapters.image_array_from_msg(msg), image)


class TestDepthAdapters(unittest.TestCase):
    def test_16uc1_round_trip(self):
        depth = np.array([[1000, 0], [65535, 250]], dtype=np.uint16)
        frame = adapters.depth_frame_from_msg(_make_image(depth, "16UC1"))
        np.testing.assert_array_equal(frame.depth, depth)
        self.assertEqual(frame.units, "millimetres")

        out = adapters.depth_msg_from_frame(frame, adapters.sec_to_stamp(2.0))
        self.assertEqual(out.encoding, "16UC1")
        self.assertEqual(out.step, 4)
        np.testing.assert_array_equal(
            adapters.depth_array_from_msg(out), depth)

    def test_big_endian_depth(self):
        depth = np.array([[1000, 2000]], dtype=np.uint16)
        msg = _make_image(depth.astype(">u2"), "16UC1", is_bigendian=1)
        np.testing.assert_array_equal(
            adapters.depth_array_from_msg(msg), depth)

    def test_float_depth_rejected(self):
        depth = np.ones((2, 2), dtype=np.float32)
        self.assertIsNone(
            adapters.depth_frame_from_msg(_make_image(depth, "32FC1")))

    def test_truncated_depth_rejected(self):
        depth = np.ones((2, 3), dtype=np.uint16)
        self.assertIsNone(
            adapters.depth_array_from_msg(_make_image(depth, "16UC1", truncate=2)))


class TestCameraInfoAdapters(unittest.TestCase):
    def _msg(self):
        msg = CameraInfo()
        msg.header.frame_id = "optical"
        msg.width = 640
        msg.height = 480
        msg.distortion_model = "plumb_bob"
        msg.d = [0.1, 0.2, 0.0, 0.0, 0.0]
        msg.k = [500.0, 0.0, 320.0, 0.0, 501.0, 240.0, 0.0, 0.0, 1.0]
        msg.r = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [500.0, 0.0, 320.0, -25.0,
                 0.0, 501.0, 240.0, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        msg.binning_x = 2
        msg.binning_y = 3
        return msg

    def test_intrinsics_decoded(self):
        frame = adapters.camera_info_frame_from_msg(self._msg())
        self.assertEqual((frame.fx, frame.fy), (500.0, 501.0))
        self.assertEqual((frame.cx, frame.cy), (320.0, 240.0))
        self.assertEqual(frame.distortion_coeffs, (0.1, 0.2, 0.0, 0.0, 0.0))

    def test_rectification_and_projection_survive_round_trip(self):
        source = self._msg()
        frame = adapters.camera_info_frame_from_msg(source)
        out = adapters.camera_info_msg_from_frame(
            frame, adapters.sec_to_stamp(3.0))
        self.assertEqual(list(out.r), list(source.r))
        self.assertEqual(list(out.p), list(source.p))
        self.assertEqual(list(out.k), list(source.k))
        self.assertEqual(out.binning_x, 2)
        self.assertEqual(out.binning_y, 3)

    def test_missing_projection_falls_back_to_intrinsics(self):
        frame = CameraInfoFrame(
            stamp=1.0, frame_id="optical", width=640, height=480,
            fx=500.0, fy=501.0, cx=320.0, cy=240.0,
        )
        out = adapters.camera_info_msg_from_frame(
            frame, adapters.sec_to_stamp(1.0))
        self.assertEqual(list(out.r), [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.assertEqual(list(out.p)[:4], [500.0, 0.0, 320.0, 0.0])

    def test_copy_keeps_new_fields(self):
        frame = adapters.camera_info_frame_from_msg(self._msg())
        clone = frame.copy()
        self.assertEqual(clone.rectification, frame.rectification)
        self.assertEqual(clone.projection, frame.projection)


class TestStampAndTransform(unittest.TestCase):
    def test_stamp_round_trip(self):
        stamp = adapters.sec_to_stamp(1234.5)
        self.assertEqual(stamp.sec, 1234)
        self.assertEqual(stamp.nanosec, 500000000)
        self.assertAlmostEqual(adapters.stamp_to_sec(stamp), 1234.5, places=9)

    def test_nanosecond_carry(self):
        stamp = adapters.sec_to_stamp(2.9999999999)
        self.assertEqual(stamp.sec, 3)
        self.assertEqual(stamp.nanosec, 0)

    def test_non_positive_seconds_give_zero_stamp(self):
        stamp = adapters.sec_to_stamp(0.0)
        self.assertEqual((stamp.sec, stamp.nanosec), (0, 0))

    def test_transform_matrix_rotates_and_translates(self):
        tf_msg = TransformStamped()
        # +90 deg about Z, then translate +1 in x.
        tf_msg.transform.rotation.z = np.sqrt(0.5)
        tf_msg.transform.rotation.w = np.sqrt(0.5)
        tf_msg.transform.translation.x = 1.0
        matrix = adapters.transform_matrix(tf_msg)
        point = np.array([1.0, 0.0, 0.0, 1.0])
        np.testing.assert_allclose(
            matrix.dot(point)[:3], [1.0, 1.0, 0.0], atol=1e-9)


class TestJointAdapters(unittest.TestCase):
    def _msg(self, sec=5, nanosec=0, velocity=(0.1, 0.2)):
        msg = JointState()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec
        msg.name = ["j1", "j2"]
        msg.position = [0.5, 1.5]
        msg.velocity = list(velocity)
        return msg

    def test_decodes_positions_and_velocities(self):
        sample = adapters.joint_sample_from_msg(self._msg())
        self.assertEqual(sample.joint_names, ("j1", "j2"))
        np.testing.assert_allclose(sample.positions, [0.5, 1.5])
        np.testing.assert_allclose(sample.velocities, [0.1, 0.2])
        self.assertEqual(sample.stamp, 5.0)

    def test_absent_velocity_is_none_not_zeros(self):
        sample = adapters.joint_sample_from_msg(self._msg(velocity=()))
        self.assertIsNone(sample.velocities)

    def test_zero_stamp_uses_fallback(self):
        sample = adapters.joint_sample_from_msg(
            self._msg(sec=0), fallback_stamp_sec=42.0)
        self.assertEqual(sample.stamp, 42.0)

    def test_zero_stamp_without_fallback_is_rejected(self):
        self.assertIsNone(adapters.joint_sample_from_msg(self._msg(sec=0)))


if __name__ == "__main__":
    unittest.main()


class TestMaskAdapters(unittest.TestCase):
    """mono8 / mono16 mask encode-decode round trips (Todo 1)."""

    def test_mono8_mask_round_trip(self):
        label_map = np.zeros((24, 32), dtype=np.uint8)
        label_map[4:20, 8:24] = 2
        label_map[0:2, 0:2] = 4
        stamp = adapters.sec_to_stamp(12.5)
        msg = adapters.mask_msg_from_array(label_map, stamp, "camera_depth_optical_frame")
        self.assertEqual(msg.encoding, "mono8")
        self.assertEqual((msg.height, msg.width), (24, 32))
        self.assertEqual(msg.step, 32)
        back = adapters.image_array_from_msg(msg)
        np.testing.assert_array_equal(back, label_map)
        self.assertEqual(msg.header.frame_id, "camera_depth_optical_frame")

    def test_mono16_instance_round_trip(self):
        instance_map = np.zeros((24, 32), dtype=np.uint16)
        instance_map[4:10, 4:10] = 1
        instance_map[12:20, 12:20] = 2
        stamp = adapters.sec_to_stamp(12.5)
        msg = adapters.instance_mask_msg_from_array(instance_map, stamp, "f")
        self.assertEqual(msg.encoding, "mono16")
        self.assertEqual(msg.step, 64)
        back = adapters.mono16_array_from_msg(msg)
        np.testing.assert_array_equal(back, instance_map)

    def test_mono16_rejects_other_encoding(self):
        label_map = np.zeros((4, 4), dtype=np.uint8)
        msg = adapters.mask_msg_from_array(label_map, adapters.sec_to_stamp(1.0), "f")
        self.assertIsNone(adapters.mono16_array_from_msg(msg))

    def test_mask_rejects_wrong_ndim(self):
        with self.assertRaises(ValueError):
            adapters.mask_msg_from_array(
                np.zeros((4, 4, 3), dtype=np.uint8), adapters.sec_to_stamp(1.0), "f")
        with self.assertRaises(ValueError):
            adapters.instance_mask_msg_from_array(
                np.zeros((4,), dtype=np.uint16), adapters.sec_to_stamp(1.0), "f")
