"""PF-R9 B6: split-parameter semantics and mutation isolation.

B2 split: ``camera_pair_tolerance_sec`` bounds |cloud stamp - rgb stamp|
for attachment (gz co-stamps, so ~exact); ``camera_wait_deadline_sec``
bounds how long a cloudless RGB stamp waits, measured on the RGB stream's
own clock — never advanced by /joint_states. B5: the observation returned
to consumers must stay isolated from preprocessor state even though the
internal build now aliases the buffer's stored cloud array.
"""

import unittest

import numpy as np

from luggage_perception.motion_stability_filter import MotionStabilityGate
from luggage_perception.sensor_preprocessor import SensorPreprocessor
from luggage_perception.sensor_types import CameraCloud, DepthFrame, RgbFrame


def _rgb(stamp):
    return RgbFrame(stamp=stamp, frame_id="camera", image=np.zeros((4, 4, 3), dtype=np.uint8))


def _depth(stamp):
    return DepthFrame(
        stamp=stamp, frame_id="camera",
        depth=np.zeros((4, 4), dtype=np.uint16),
        units="millimetres", encoding="16UC1")


def _cloud(stamp, n=8):
    return CameraCloud(
        stamp=stamp, frame_id="camera", data_frame="camera",
        points=np.arange(3 * n, dtype=np.float32).reshape(n, 3))


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

    def test_late_but_co_stamped_cloud_attaches_within_deadline(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.005,
            camera_wait_deadline_sec=0.060,
            motion_gate=_stable_gate())
        _hold_stable(pre, t0=16.0)
        pre.update_depth(_depth(16.00))
        self.assertIsNone(pre.update_rgb(_rgb(16.00)))
        # A same-stamp cloud arriving "late" (after intermediate RGB frames
        # that are still within the deadline) still attaches.
        pre.update_rgb(_rgb(16.03))
        obs = pre.update_camera_cloud(_cloud(16.00))
        self.assertIsNotNone(obs)
        self.assertEqual(16.00, obs.primary_stamp)
        self.assertTrue(obs.flags.cloud_ok)
        self.assertEqual(0.0, obs.cloud_dt)

    def test_out_of_tolerance_cloud_not_attached(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.005,
            camera_wait_deadline_sec=0.060,
            camera_emit_rgb_only=True)
        pre.update_depth(_depth(20.00))
        self.assertIsNone(pre.update_rgb(_rgb(20.00)))
        # Cloud for a different stamp: pairing must not attach it.
        obs = pre.update_camera_cloud(_cloud(20.02))
        self.assertIsNone(obs)  # rgb 20.00 still within its wait deadline
        obs = pre.update_rgb(_rgb(20.07))  # deadline for 20.00 expires
        self.assertIsNotNone(obs)
        self.assertEqual(20.00, obs.primary_stamp)
        self.assertFalse(obs.flags.cloud_ok)

    def test_deadline_not_advanced_by_joint_states(self):
        # 50 Hz joints arrive between RGB frames; under the old
        # now_hint-driven rule one joint message 20 ms after the RGB stamp
        # declared the cloud dead. The deadline now runs on the RGB clock.
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.005,
            camera_wait_deadline_sec=0.060,
            motion_gate=_stable_gate())
        _hold_stable(pre, t0=30.0)
        pre.update_depth(_depth(30.00))
        self.assertIsNone(pre.update_rgb(_rgb(30.00)))
        # Joint messages at +20, +40, +58 ms — none may expire the window.
        for jt in (30.02, 30.04, 30.058):
            pre.update_joint_state(type(
                "J", (), {"stamp": jt, "joint_names": ["j1"],
                          "positions": [0.0], "velocities": None,
                          "copy": lambda self: self})())
        # The co-stamped cloud still attaches after all those joints.
        obs = pre.update_camera_cloud(_cloud(30.00))
        self.assertIsNotNone(obs)
        self.assertTrue(obs.flags.cloud_ok)

    def test_deadline_expires_on_rgb_clock_and_skips(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.005,
            camera_wait_deadline_sec=0.060)
        self.assertIsNone(pre.update_rgb(_rgb(40.00)))
        # Newer RGB at +70 ms expires the 60 ms deadline for 40.00.
        self.assertIsNone(pre.update_rgb(_rgb(40.07)))
        self.assertEqual("cloud_wait_timeout", pre.last_rejection_reason())
        diag = pre.diagnostics()
        self.assertEqual(1, diag["cloud_timeout_skips"])

    def test_rgb_only_emission_flagged_never_faked(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.005,
            camera_wait_deadline_sec=0.060,
            camera_emit_rgb_only=True)
        self.assertIsNone(pre.update_rgb(_rgb(50.00)))
        obs = pre.update_rgb(_rgb(50.07))
        self.assertIsNotNone(obs)
        self.assertTrue(obs.flags.rgb_ok)
        self.assertFalse(obs.flags.cloud_ok)
        self.assertIsNone(obs.camera_points)

    def test_horizon_still_prunes_cloudless_skips(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.005,
            camera_wait_deadline_sec=0.060,
            camera_horizon_sec=0.35)
        for i in range(12):
            pre.update_rgb(_rgb(60.0 + 0.07 * i))
        diag = pre.diagnostics()
        self.assertGreaterEqual(diag["cloud_timeout_skips"], 1)
        # The skip set is pruned with the emitted set; buffers stay bounded.
        self.assertLessEqual(diag["buffers"]["rgb"], 10)


class TestMutationIsolation(unittest.TestCase):
    """B5 non-negotiable: consumers cannot mutate preprocessor state."""

    def test_returned_observation_cloud_is_isolated(self):
        pre = SensorPreprocessor(
            camera_pair_tolerance_sec=0.005,
            camera_wait_deadline_sec=0.060,
            motion_gate=_stable_gate())
        _hold_stable(pre, t0=16.0)
        pre.update_depth(_depth(16.00))
        pre.update_rgb(_rgb(16.00))
        obs = pre.update_camera_cloud(_cloud(16.00, n=64))
        self.assertIsNotNone(obs)
        before = obs.camera_points.copy()
        # Mutate the consumer's copy...
        obs.camera_points[:] = -1.0
        # ...the preprocessor's retained output must be untouched.
        retained = pre.copy_output()
        np.testing.assert_array_equal(before, retained.camera_points)

    def test_cloud_dtype_is_float32(self):
        pre = SensorPreprocessor()
        pre.update_rgb(_rgb(1.0))
        pre.update_depth(_depth(1.0))
        obs = pre.update_camera_cloud(
            _cloud(1.0, n=4), point_transform=np.eye(4))
        self.assertIsNotNone(obs)
        self.assertEqual(np.float32, obs.camera_points.dtype)


if __name__ == "__main__":
    unittest.main()
