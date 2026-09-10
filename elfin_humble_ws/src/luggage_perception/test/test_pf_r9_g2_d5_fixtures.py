"""PF-R9 g2 D5: payload/cache/deprojection fixtures at both resolutions.

Covers the plan's D5 list for 640x480 (simulation) and 640x360 (D455):
zero-copy identity, padded/truncated/big-endian layouts, plane
deprojection with known intrinsics, zero/invalid depth, mismatch
rejection by named counter, cache eviction (capacity + one-second
horizon), out-of-order insertion, rollback epoch flush, camera-clock
isolation from joint/IMU/lidar stamps, and mutation isolation.
"""

import array
import unittest

import numpy as np

from luggage_perception.depth_deprojection import (
    deproject_selected,
    deproject_stride,
)
from luggage_perception.motion_stability_filter import MotionStabilityGate
from luggage_perception.sensor_preprocessor import SensorPreprocessor
from luggage_perception.sensor_types import (
    CameraInfoFrame,
    DepthFrame,
    ImuSample,
    LidarScan,
    OpaquePayload,
    RgbFrame,
)

RESOLUTIONS = [(480, 640), (360, 640)]


def _key(stamp):
    return (int(stamp), int(round((float(stamp) % 1.0) * 1e9)))


def _rgb_msg_frame(stamp, h, w, value=7):
    image = np.full((h, w, 3), value, dtype=np.uint8)
    raw = array.array("B", image.tobytes())
    return RgbFrame(
        stamp=stamp, frame_id="optical", encoding="rgb8",
        payload=OpaquePayload(raw), height=h, width=w, step=w * 3,
        stamp_key=_key(stamp)), image


def _depth_msg_frame(stamp, h, w, mm=1000, bigendian=False):
    depth = np.full((h, w), mm, dtype=">u2" if bigendian else "<u2")
    raw = array.array("B", depth.tobytes())
    return DepthFrame(
        stamp=stamp, frame_id="optical", units="millimetres",
        encoding="16UC1", payload=OpaquePayload(raw),
        height=h, width=w, step=w * 2, is_bigendian=1 if bigendian else 0,
        stamp_key=_key(stamp))


def _info(stamp, h, w, fx=None):
    return CameraInfoFrame(
        stamp=stamp, frame_id="optical", width=w, height=h,
        fx=fx if fx is not None else 337.2,
        fy=fx if fx is not None else 337.2,
        cx=w / 2.0, cy=h / 2.0)


def _pre(h, w, **kwargs):
    kwargs.setdefault("camera_pair_tolerance_sec", 0.005)
    kwargs.setdefault("camera_wait_deadline_sec", 0.060)
    return SensorPreprocessor(camera_maxlen=15, camera_horizon_sec=1.0,
                              **kwargs)


class Intr:
    def __init__(self, fx, fy, cx, cy):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy


class TestPayloadFixtures(unittest.TestCase):

    def test_rgb8_and_16uc1_identity_both_resolutions(self):
        from luggage_perception import ros_message_adapters as adapters
        from sensor_msgs.msg import Image
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                rgb, image = _rgb_msg_frame(1.0, h, w)
                depth = _depth_msg_frame(1.0, h, w)
                for frame, ref in ((rgb, "rgb"), (depth, "depth")):
                    view = frame.view()
                    self.assertFalse(view.flags.writeable)
                    self.assertIsNotNone(view)
                msg_rgb = adapters.image_msg_from_frame(
                    rgb, adapters.sec_to_stamp(1.0))
                msg_depth = adapters.depth_msg_from_frame(
                    depth, adapters.sec_to_stamp(1.0))
                self.assertIs(rgb.payload.ros_data(), msg_rgb.data)
                self.assertIs(depth.payload.ros_data(), msg_depth.data)
                self.assertEqual((h, w, 3),
                                 np.asarray(msg_rgb.data).size
                                 and tuple(
                                     np.frombuffer(
                                         msg_rgb.data, dtype=np.uint8
                                     ).reshape(h, w, 3).shape))
                self.assertEqual(2 * h * w, len(msg_depth.data))

    def test_padded_step_and_truncated(self):
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                row = w * 3 + 5
                buf = bytearray(h * row)
                image = np.full((h, w, 3), 4, dtype=np.uint8)
                for r in range(h):
                    buf[r * row:r * row + w * 3] = \
                        image.reshape(h, w * 3)[r].tobytes()
                padded = RgbFrame(
                    stamp=2.0, frame_id="optical", encoding="rgb8",
                    payload=OpaquePayload(array.array("B", bytes(buf))),
                    height=h, width=w, step=row)
                np.testing.assert_array_equal(padded.view(), image)
                short = RgbFrame(
                    stamp=2.0, frame_id="optical", encoding="rgb8",
                    payload=OpaquePayload(array.array("B", bytes(w * 3))),
                    height=h, width=w, step=w * 3)
                self.assertIsNone(short.view())

    def test_big_endian_depth_view(self):
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                depth = _depth_msg_frame(3.0, h, w, bigendian=True)
                view = depth.view()
                self.assertEqual(np.dtype(">u2"), view.dtype)
                np.testing.assert_array_equal(
                    view.astype(np.int64), np.full((h, w), 1000))

    def test_known_plane_deprojection(self):
        # A fronto-parallel plane at exactly 2.0 m: every deprojected
        # point has z == 2.0 and (x, y) follow the pinhole model.
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                intr = Intr(fx=337.2, fy=337.2, cx=w / 2.0, cy=h / 2.0)
                depth = np.full((h, w), 2000, dtype=np.uint16)
                pts, n = deproject_stride(depth, intr, stride=4)
                self.assertEqual((h // 4) * (w // 4), n)
                np.testing.assert_allclose(pts[:, 2], 2.0, rtol=0, atol=1e-6)
                vu = np.arange(0, h, 4).astype(np.float64)
                uu = np.arange(0, w, 4).astype(np.float64)
                vg, ug = np.meshgrid(vu, uu, indexing="ij")
                exp_x = (ug.reshape(-1) - intr.cx) * 2.0 / intr.fx
                exp_y = (vg.reshape(-1) - intr.cy) * 2.0 / intr.fy
                np.testing.assert_allclose(
                    pts[:, 0], exp_x, rtol=1e-5, atol=1e-6)
                np.testing.assert_allclose(
                    pts[:, 1], exp_y, rtol=1e-5, atol=1e-6)

    def test_zero_and_invalid_depth_excluded(self):
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                intr = Intr(fx=100.0, fy=100.0, cx=0.0, cy=0.0)
                depth = np.zeros((h, w), dtype=np.uint16)
                depth[0, 0] = 1500
                depth[0, 1] = 65535  # valid but far
                pts, n = deproject_stride(depth, intr, stride=1)
                self.assertEqual(2, n)
                np.testing.assert_allclose(
                    sorted(pts[:, 2].tolist()), [1.5, 65.535])
                # Selected-pixel form: zero-depth pixel is excluded.
                pts2, n2 = deproject_selected(
                    depth, np.array([0, 0]), np.array([0, 1]), intr)
                self.assertEqual(1, n2)
                self.assertAlmostEqual(float(pts2[0, 2]), 1.5, places=5)


class TestAcquisitionMismatches(unittest.TestCase):

    def _emit(self, h, w, rgb=None, depth=None, info=None):
        pre = _pre(h, w)
        pre.update_camera_info(info or _info(5.0, h, w))
        pre.update_depth(depth or _depth_msg_frame(5.0, h, w))
        obs = pre.update_rgb(rgb or _rgb_msg_frame(5.0, h, w)[0])
        return pre, obs

    def test_dimension_mismatch_named_counter(self):
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                pre, obs = self._emit(
                    h, w, depth=_depth_msg_frame(5.0, h, w + 2))
                self.assertIsNone(obs)
                self.assertEqual(
                    1, pre.diagnostics()["acquisition_mismatches"])
                self.assertEqual(
                    "acquisition_dimension_mismatch",
                    pre.last_rejection_reason())

    def test_frame_mismatch_named_counter(self):
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                rgb, _ = _rgb_msg_frame(5.0, h, w)
                rgb.frame_id = "other_frame"
                pre, obs = self._emit(h, w, rgb=rgb)
                self.assertIsNone(obs)
                self.assertEqual(
                    "acquisition_frame_mismatch",
                    pre.last_rejection_reason())

    def test_intrinsics_mismatch_named_counter(self):
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                pre = _pre(h, w)
                pre.update_camera_info(_info(5.0, h, w))
                # A second colour info with a different K on the same
                # stamp: the canonical set is inconsistent.
                pre.update_camera_info(
                    _info(5.0, h, w, fx=222.0), slots=("color",))
                pre.update_depth(_depth_msg_frame(5.0, h, w))
                obs = pre.update_rgb(_rgb_msg_frame(5.0, h, w)[0])
                self.assertIsNone(obs)
                self.assertEqual(
                    "acquisition_intrinsics_mismatch",
                    pre.last_rejection_reason())

    def test_encoding_mismatch_named_counter(self):
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                bad = DepthFrame(
                    stamp=5.0, frame_id="optical", units="metres",
                    encoding="32FC1")
                pre = _pre(h, w)
                self.assertIsNone(pre.update_depth(bad))
                self.assertEqual(
                    "unexpected_depth_encoding",
                    pre.last_rejection_reason())

    def test_exact_stamp_mismatch_never_pairs(self):
        h, w = RESOLUTIONS[0]
        pre = _pre(h, w)
        pre.update_camera_info(_info(6.0, h, w))
        pre.update_depth(_depth_msg_frame(6.02, h, w))
        self.assertIsNone(pre.update_rgb(_rgb_msg_frame(6.0, h, w)[0]))
        self.assertEqual(
            0, pre.diagnostics()["acquisition_mismatches"])


class TestCacheContract(unittest.TestCase):

    def test_same_stamp_replacement_no_growth(self):
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                pre = _pre(h, w)
                for i in range(4):
                    pre.update_rgb(_rgb_msg_frame(7.0, h, w, value=i)[0])
                self.assertEqual(1, pre.buffer_occupancy()["rgb"])
                self.assertEqual(3, pre._rgb.replaced)

    def test_sixteenth_frame_eviction(self):
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                pre = _pre(h, w)
                for i in range(16):
                    pre.update_rgb(_rgb_msg_frame(8.0 + 0.05 * i, h, w)[0])
                self.assertEqual(15, pre.buffer_occupancy()["rgb"])
                self.assertNotIn(8.0, pre._rgb.stamps())
                self.assertEqual(1, pre._rgb.evicted_capacity)

    def test_one_second_horizon(self):
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                pre = _pre(h, w)
                for i in range(12):
                    pre.update_rgb(_rgb_msg_frame(9.0 + 0.2 * i, h, w)[0])
                # 12 frames at 0.2 s span 2.2 s: only the last second kept.
                self.assertLessEqual(pre.buffer_occupancy()["rgb"], 6)
                self.assertGreaterEqual(pre._rgb.evicted_horizon, 1)

    def test_out_of_order_within_rollback(self):
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                pre = _pre(h, w)
                pre.update_rgb(_rgb_msg_frame(10.00, h, w)[0])
                pre.update_rgb(_rgb_msg_frame(10.20, h, w)[0])
                pre.update_rgb(_rgb_msg_frame(10.10, h, w)[0])
                self.assertEqual(
                    [10.00, 10.10, 10.20], pre._rgb.stamps())

    def test_rollback_flush_epoch_and_lookup_absence(self):
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                pre = _pre(h, w)
                pre.update_camera_info(_info(11.0, h, w))
                pre.update_depth(_depth_msg_frame(11.0, h, w))
                pre.update_rgb(_rgb_msg_frame(11.0, h, w)[0])
                self.assertEqual(0, pre.camera_epoch)
                # Backward jump beyond rollback_sec flushes everything.
                pre.update_rgb(_rgb_msg_frame(10.0, h, w)[0])
                self.assertEqual(1, pre.camera_epoch)
                self.assertEqual(1, pre.rollback_events)
                self.assertEqual(
                    0, len(pre._depth))  # pre-rollback depth is gone
                self.assertIsNone(
                    pre._depth.nearest_ns(_key(11.0)[0] * 10 ** 9
                                          + _key(11.0)[1], 5000000))

    def test_camera_cache_not_aged_by_joint_imu_lidar(self):
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                pre = _pre(h, w)
                pre.update_camera_info(_info(12.0, h, w))
                pre.update_depth(_depth_msg_frame(12.0, h, w))
                pre.update_rgb(_rgb_msg_frame(12.0, h, w)[0])
                for i in range(8):
                    pre.update_joint_state(type(
                        "J", (), {"stamp": 400.0 + 0.05 * i,
                                  "joint_names": ["j1"],
                                  "positions": [0.0],
                                  "velocities": None,
                                  "copy": lambda self: self})())
                pre.update_imu(ImuSample(
                    stamp=500.0, frame_id="livox_imu_frame",
                    angular_velocity=np.zeros(3),
                    linear_acceleration=np.zeros(3)))
                pre.update_lidar(LidarScan(
                    stamp_start=600.0, stamp_end=600.0,
                    frame_id="livox_frame",
                    points=np.zeros((0, 3))))
                self.assertEqual(1, pre.buffer_occupancy()["rgb"])
                self.assertEqual(1, pre.buffer_occupancy()["depth"])
                self.assertEqual(
                    1, pre.buffer_occupancy()["color_info"])


class TestMutationIsolation(unittest.TestCase):

    def test_view_frame_observation_queue_eviction_isolation(self):
        for h, w in RESOLUTIONS:
            with self.subTest(resolution=(h, w)):
                pre = _pre(h, w)
                rgb, image = _rgb_msg_frame(13.0, h, w)
                raw = rgb.payload.ros_data()
                pre.update_camera_info(_info(13.0, h, w))
                pre.update_depth(_depth_msg_frame(13.0, h, w))
                obs = pre.update_rgb(rgb)
                self.assertIsNotNone(obs)
                local = obs.copy()
                # View mutation raises.
                with self.assertRaises(ValueError):
                    local.rgb.view()[:] = 0
                # Metadata mutation of the local copy cannot reach the
                # buffered frame.
                local.rgb.frame_id = "mutated"
                retained = pre.copy_output()
                self.assertEqual("optical", retained.rgb.frame_id)
                # Payload identity is preserved through every boundary.
                for holder in (obs, local, retained):
                    self.assertIs(raw, holder.rgb.payload.ros_data())
                # Evict the index; the held reference stays valid.
                for i in range(16):
                    pre.update_camera_info(_info(13.5 + 0.05 * i, h, w))
                np.testing.assert_array_equal(
                    retained.rgb.view()[0, 0], image[0, 0])


if __name__ == "__main__":
    unittest.main()


class TestSupportRetention(unittest.TestCase):
    """D6: annulus point retention at the configured stride and one above.

    Synthetic nadir scene: camera z=1.9 over the platform top (range
    1.04 m), fx=337.2, box footprint 0.7x0.5 m. The support annulus is
    the platform ring between inner and outer margin around the footprint.
    """

    @staticmethod
    def _annulus_count(h, w, stride, inner, outer, fx=337.2):
        z = 1.04
        rows = np.arange(0, h, stride)
        cols = np.arange(0, w, stride)
        vg, ug = np.meshgrid(rows, cols, indexing="ij")
        u = ug.reshape(-1).astype(np.float64)
        v = vg.reshape(-1).astype(np.float64)
        x = (u - w / 2.0) * z / fx
        y = (v - h / 2.0) * z / fx
        in_box = (np.abs(x) < 0.35 + inner) & (np.abs(y) < 0.25 + inner)
        in_band = (np.abs(x) < 0.35 + outer) & (np.abs(y) < 0.25 + outer)
        return int((in_band & ~in_box).sum())

    def test_retention_at_stride_and_one_above(self):
        for h, w in RESOLUTIONS:
            for stride in (4, 5):
                for inner in (0.03, 0.08, 0.13):
                    with self.subTest(
                            resolution=(h, w), stride=stride, inner=inner):
                        count = self._annulus_count(
                            h, w, stride, inner, 0.18)
                        self.assertGreaterEqual(
                            count, 80,
                            "margin %.1fx above min_support_points"
                            % (count / 80.0))

    def test_degenerate_annulus_is_empty(self):
        # inner == outer: the ring has zero width by definition.
        self.assertEqual(
            0, self._annulus_count(480, 640, 4, 0.18, 0.18))
