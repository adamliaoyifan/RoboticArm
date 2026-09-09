"""PF-R9 g2 B2/B6 regressions (adapted from the generation-1 split tests).

B2 split: ``camera_pair_tolerance_sec`` bounds |depth stamp - rgb stamp|
for attachment (aligned streams are co-stamped); ``camera_wait_deadline_sec``
bounds how long a depthless RGB stamp waits, measured on the canonical
camera clock — never advanced by /joint_states. B6: payload/view mutation
isolation and the fixed 15/1.0 cache contract.
"""

import array
import unittest

import numpy as np

from luggage_perception.motion_stability_filter import MotionStabilityGate
from luggage_perception.sensor_preprocessor import SensorPreprocessor
from luggage_perception.sensor_types import (
    CameraInfoFrame,
    DepthFrame,
    OpaquePayload,
    RgbFrame,
)


def _stamp_key(stamp):
    return (int(stamp), int(round((float(stamp) % 1.0) * 1e9)))


def _rgb(stamp, h=4, w=8, payload=False):
    image = np.full((h, w, 3), 7, dtype=np.uint8)
    if not payload:
        return RgbFrame(stamp=stamp, frame_id="optical", image=image,
                        encoding="rgb8")
    raw = array.array("B", image.tobytes())
    return RgbFrame(
        stamp=stamp, frame_id="optical", encoding="rgb8",
        payload=OpaquePayload(raw), height=h, width=w, step=w * 3,
        is_bigendian=0, stamp_key=_stamp_key(stamp))


def _depth(stamp, h=4, w=8, mm=1000, bigendian=False):
    depth = np.full((h, w), mm, dtype="<u2" if not bigendian else ">u2")
    raw = array.array("B", depth.tobytes())
    return DepthFrame(
        stamp=stamp, frame_id="optical", units="millimetres",
        encoding="16UC1", payload=OpaquePayload(raw),
        height=h, width=w, step=w * 2, is_bigendian=1 if bigendian else 0,
        stamp_key=_stamp_key(stamp))


def _info(stamp, h=4, w=8):
    return CameraInfoFrame(
        stamp=stamp, frame_id="optical", width=w, height=h,
        fx=337.2, fy=337.2, cx=w / 2.0, cy=h / 2.0)


def _stable_gate():
    return MotionStabilityGate(
        joint_names=["j1"], velocity_threshold=1e9,
        settle_time_sec=0.0, joint_state_timeout_sec=1e9, enabled=True)


def _hold_stable(pre, t0=10.0):
    gate = pre._gate
    gate.update(["j1"], [0.0], None, stamp=t0, now=t0)
    pre.update_joint_state(type(
        "J", (), {"stamp": t0, "joint_names": ["j1"],
                  "positions": [0.0], "velocities": None,
                  "copy": lambda self: self})())


class TestSplitParameters(unittest.TestCase):

    def test_late_but_co_stamped_depth_attaches_within_deadline(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.005,
            camera_wait_deadline_sec=0.060,
            motion_gate=_stable_gate())
        _hold_stable(pre, t0=16.0)
        pre.update_camera_info(_info(16.00))
        self.assertIsNone(pre.update_rgb(_rgb(16.00)))
        # Same-stamp depth arriving "late" (after intermediate RGB frames
        # still inside the deadline) still attaches.
        pre.update_rgb(_rgb(16.03))
        obs = pre.update_depth(_depth(16.00))
        self.assertIsNotNone(obs)
        self.assertEqual(16.00, obs.primary_stamp)
        self.assertTrue(obs.flags.depth_ok)
        self.assertEqual(0.0, obs.depth_dt)

    def test_out_of_tolerance_depth_not_attached(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.005,
            camera_wait_deadline_sec=0.060)
        pre.update_camera_info(_info(20.00))
        self.assertIsNone(pre.update_rgb(_rgb(20.00)))
        # Depth for a different stamp: pairing must not attach it.
        self.assertIsNone(pre.update_depth(_depth(20.02)))
        obs = pre.update_rgb(_rgb(20.07))  # deadline for 20.00 expires
        self.assertIsNone(obs)
        self.assertEqual("depth_wait_timeout", pre.last_rejection_reason())

    def test_deadline_not_advanced_by_joint_states(self):
        # 50 Hz joints arrive between RGB frames; under the old
        # now_hint-driven rule one joint message 20 ms after the RGB stamp
        # declared the depth dead. The deadline runs on the camera clock.
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.005,
            camera_wait_deadline_sec=0.060,
            motion_gate=_stable_gate())
        _hold_stable(pre, t0=30.0)
        pre.update_camera_info(_info(30.00))
        self.assertIsNone(pre.update_rgb(_rgb(30.00)))
        for jt in (30.02, 30.04, 30.058):
            pre.update_joint_state(type(
                "J", (), {"stamp": jt, "joint_names": ["j1"],
                          "positions": [0.0], "velocities": None,
                          "copy": lambda self: self})())
        # The co-stamped depth still attaches after all those joints.
        obs = pre.update_depth(_depth(30.00))
        self.assertIsNotNone(obs)
        self.assertTrue(obs.flags.depth_ok)

    def test_missing_depth_fails_closed_after_deadline(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.005,
            camera_wait_deadline_sec=0.060)
        pre.update_camera_info(_info(40.00))
        self.assertIsNone(pre.update_rgb(_rgb(40.00)))
        self.assertIsNone(pre.update_rgb(_rgb(40.07)))
        self.assertEqual("depth_wait_timeout", pre.last_rejection_reason())
        diag = pre.diagnostics()
        self.assertEqual(1, diag["depth_wait_skips"])

    def test_sixteenth_frame_capacity_eviction_keeps_newest_fifteen(self):
        pre = SensorPreprocessor(camera_maxlen=15, camera_horizon_sec=1.0)
        stamps = [50.0 + 0.05 * i for i in range(16)]
        for s in stamps:
            pre.update_rgb(_rgb(s))
            pre.update_depth(_depth(s))
            pre.update_camera_info(_info(s))
        occupancy = pre.buffer_occupancy()
        self.assertEqual(15, occupancy["rgb"])
        kept = pre._rgb.stamps()
        self.assertNotIn(50.0, kept)
        self.assertIn(stamps[-1], kept)

    def test_one_second_horizon_eviction(self):
        pre = SensorPreprocessor(camera_maxlen=15, camera_horizon_sec=1.0)
        for i in range(12):
            pre.update_rgb(_rgb(60.0 + 0.1 * i))
        self.assertEqual(11, pre.buffer_occupancy()["rgb"])
        self.assertGreaterEqual(
            pre._rgb.evicted_horizon + pre._rgb.evicted_capacity, 1)


class TestPayloadIdentityAndIsolation(unittest.TestCase):
    """D2/B6: zero materialisation + read-only views + shared payloads."""

    def test_complete_path_zero_materialisation(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.005,
            camera_wait_deadline_sec=0.060,
            motion_gate=_stable_gate())
        _hold_stable(pre, t0=70.0)
        pre.update_camera_info(_info(70.0))
        rgb = _rgb(70.0, payload=True)
        depth = _depth(70.0)
        self.assertIsNone(pre.update_depth(depth))
        obs = pre.update_rgb(rgb)
        self.assertIsNotNone(obs)
        # Payload identity survives insert, observation build, copy-out.
        self.assertIs(rgb.payload.ros_data(), obs.rgb.payload.ros_data())
        self.assertIs(depth.payload.ros_data(),
                      obs.depth.payload.ros_data())
        copied = obs.copy()
        self.assertIs(rgb.payload.ros_data(),
                      copied.rgb.payload.ros_data())
        self.assertEqual(0, pre.payload_materialisations)

    def test_views_are_readonly(self):
        rgb = _rgb(1.0, payload=True)
        depth = _depth(1.0)
        view = rgb.view()
        self.assertFalse(view.flags.writeable)
        with self.assertRaises(ValueError):
            view.setflags(write=True)
        with self.assertRaises(ValueError):
            view[:] = 0
        dview = depth.view()
        self.assertFalse(dview.flags.writeable)
        self.assertEqual(np.dtype("<u2"), dview.dtype)

    def test_big_endian_depth_stays_big_endian_view(self):
        depth = _depth(2.0, bigendian=True)
        view = depth.view()
        self.assertEqual(np.dtype(">u2"), view.dtype)
        self.assertFalse(view.flags.writeable)
        np.testing.assert_array_equal(
            view.astype(np.int64), np.full((4, 8), 1000))

    def test_padded_step_view(self):
        h, w = 4, 8
        row = w * 3 + 5  # 5 bytes of row padding
        image = np.full((h, w, 3), 9, dtype=np.uint8)
        buf = bytearray(h * row)
        flat = image.reshape(h, w * 3)
        for r in range(h):
            buf[r * row:(r * row + w * 3)] = flat[r].tobytes()
        frame = RgbFrame(
            stamp=3.0, frame_id="optical", encoding="rgb8",
            payload=OpaquePayload(array.array("B", bytes(buf))),
            height=h, width=w, step=row)
        view = frame.view()
        self.assertEqual((h, w, 3), view.shape)
        np.testing.assert_array_equal(view, image)

    def test_truncated_buffer_fails_closed(self):
        h, w = 4, 8
        short = array.array("B", bytes(w * 3 * 2))  # only 2 of 4 rows
        frame = RgbFrame(
            stamp=4.0, frame_id="optical", encoding="rgb8",
            payload=OpaquePayload(short), height=h, width=w, step=w * 3)
        self.assertIsNone(frame.view())
        pre = SensorPreprocessor()
        self.assertIsNone(pre.update_rgb(frame))
        self.assertEqual("invalid_rgb_image", pre.last_rejection_reason())

    def test_evicted_index_does_not_invalidate_queued_reference(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.005,
            camera_wait_deadline_sec=0.060,
            motion_gate=_stable_gate())
        _hold_stable(pre, t0=80.0)
        pre.update_camera_info(_info(80.0))
        obs = None
        for i in range(16):
            s = 80.0 + 0.05 * i
            pre.update_camera_info(_info(s))
            pre.update_depth(_depth(s))
            out = pre.update_rgb(_rgb(s, payload=True))
            if out is not None:
                obs = out
        # The queued/returned observation still holds a live payload even
        # though its stamp has been evicted from every index.
        self.assertIsNotNone(obs)
        self.assertIsNotNone(obs.rgb.payload.ros_data())
        np.testing.assert_array_equal(
            obs.rgb.view()[0, 0], np.array([7, 7, 7], dtype=np.uint8))

    def test_mutating_copied_frame_metadata_cannot_corrupt_buffer(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.005,
            camera_wait_deadline_sec=0.060,
            motion_gate=_stable_gate())
        _hold_stable(pre, t0=90.0)
        pre.update_camera_info(_info(90.0))
        pre.update_depth(_depth(90.0))
        obs = pre.update_rgb(_rgb(90.0, payload=True))
        local = obs.copy()
        # Mutating the local copy's frame metadata cannot reach the
        # buffered frame: they are separate frame objects over one shared
        # payload, and the payload itself stays identical.
        local.rgb.frame_id = "mutated"
        retained = pre.copy_output()
        self.assertIs(obs.rgb.payload.ros_data(),
                      retained.rgb.payload.ros_data())
        np.testing.assert_array_equal(
            retained.rgb.view()[0, 0], np.array([7, 7, 7], dtype=np.uint8))


if __name__ == "__main__":
    unittest.main()
