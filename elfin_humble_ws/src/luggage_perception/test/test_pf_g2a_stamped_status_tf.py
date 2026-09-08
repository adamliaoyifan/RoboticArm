#!/usr/bin/env python3
"""PF-G2A: stamped geometry status + acquisition-stamp TF (PF-R3).

docs/plans/platform_free_height_remediation.md. Missing, malformed, or
stale status may still permit a top-only result, but never a
MEASURED_SUPPORT height. Semantic TF lookups receive the acquisition
stamp and never fall back to latest TF.
"""

import math
import unittest
from types import SimpleNamespace

import numpy as np
import pytest

from luggage_perception.detection_frame_join import ExactStampJoin, stamp_key
from luggage_perception.platform_free_pipeline import (
    GeometryStatusGate,
    PlatformFreeDetector,
)
from luggage_perception.top_support_estimator import (
    DETECT_SUPPORT_STAMP_MISMATCH,
    DETECT_SUPPORT_STATUS_MALFORMED,
    DETECT_SUPPORT_STATUS_MISSING,
    DETECT_SUPPORT_STATUS_STALE,
    HEIGHT_SOURCE_MEASURED_SUPPORT,
    HEIGHT_SOURCE_UNAVAILABLE,
    TopSupportConfig,
)

CONFIG = TopSupportConfig(
    workspace_center_xy=(0.0, 0.0),
    workspace_half_extents=(1.2, 1.2),
    min_top_points=40,
    min_support_points=40,
    min_inliers_per_side=8,
)


def _scene(support_z=0.90, seed=0):
    """Cargo top + raw ring scene at the given support height."""
    rng = np.random.RandomState(seed)
    w, d, h = 0.55, 0.35, 0.45
    yaw = math.radians(20.0)
    u = rng.uniform(-w / 2, w / 2, 500)
    v = rng.uniform(-d / 2, d / 2, 500)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    cargo = np.column_stack([
        cos_y * u - sin_y * v, sin_y * u + cos_y * v,
        np.full(500, support_z + h)])
    u2 = rng.uniform(-(w / 2 + 0.18), w / 2 + 0.18, 4000)
    v2 = rng.uniform(-(d / 2 + 0.18), d / 2 + 0.18, 4000)
    keep = (np.abs(u2) > w / 2 + 0.03) | (np.abs(v2) > d / 2 + 0.03)
    u2, v2 = u2[keep], v2[keep]
    raw = np.column_stack([
        cos_y * u2 - sin_y * v2, sin_y * u2 + cos_y * v2,
        np.full(len(u2), support_z)])
    return cargo, raw


def _status(cloud_stamp, geometry_ok=True):
    whole = int(cloud_stamp)
    nanosec = int(round((cloud_stamp - whole) * 1e9))
    return {
        "flags": {"geometry_ok": geometry_ok},
        "primary_stamp": float(cloud_stamp),
        "primary_stamp_sec": whole,
        "primary_stamp_nanosec": nanosec,
        "last_geometry_ok_stamp": float(cloud_stamp),
    }


class TestGeometryStatusGate(unittest.TestCase):

    def test_matching_stamp_permits_support(self):
        gate = GeometryStatusGate()
        gate.update(_status(100.0))
        ok, reason = gate.evaluate(100, cloud_stamp_nanosec=0)
        self.assertTrue(ok)
        self.assertEqual(reason, "")

    def test_missing_status(self):
        gate = GeometryStatusGate()
        ok, reason = gate.evaluate(100, cloud_stamp_nanosec=0)
        self.assertFalse(ok)
        self.assertEqual(reason, "status_missing")

    def test_malformed_status(self):
        for payload in ("not-a-dict", {}, {"flags": {}},
                        {"flags": {"geometry_ok": True}},  # no stamp
                        {"flags": {"geometry_ok": True},
                         "primary_stamp": "abc"}):
            gate = GeometryStatusGate()
            gate.update(payload)
            ok, reason = gate.evaluate(100, cloud_stamp_nanosec=0)
            self.assertFalse(ok, payload)
            self.assertEqual(reason, "status_malformed", payload)

    def test_n_minus_1_status_cannot_authorize_n(self):
        """One frame (~0.25 s) older status is NOT evidence for cloud N.

        The robot can start moving between frames: only the exact
        acquisition's status may authorize support fitting.
        """
        gate = GeometryStatusGate()
        gate.update(_status(100.0))
        ok, reason = gate.evaluate(100, cloud_stamp_nanosec=250000000)
        self.assertFalse(ok)
        self.assertEqual(reason, "status_stale")

    def test_out_of_order_status_fails_closed(self):
        """Status for N+1 arriving before cloud N cannot authorize N."""
        gate = GeometryStatusGate()
        gate.update(_status(100.5))
        ok, reason = gate.evaluate(100, cloud_stamp_nanosec=0)
        self.assertFalse(ok)
        self.assertIn(reason, ("status_missing", "status_stale"))

    def test_float_fallback_covers_representation_only(self):
        """Payload without int fields still joins within 1 us, not more."""
        gate = GeometryStatusGate()
        payload = _status(100.0)
        del payload["primary_stamp_sec"]
        del payload["primary_stamp_nanosec"]
        gate.update(payload)
        ok, _ = gate.evaluate(100.0000005)   # 0.5 us: float error range
        self.assertTrue(ok)
        ok, reason = gate.evaluate(100.0002)  # 200 us: a different frame
        self.assertFalse(ok)

    def test_buffer_is_bounded(self):
        gate = GeometryStatusGate(maxlen=4)
        for i in range(10):
            gate.update(_status(100.0 + 0.25 * i))
        # The 4 newest entries remain authorized...
        ok, _ = gate.evaluate(102, cloud_stamp_nanosec=250000000)  # 9th
        self.assertTrue(ok)
        # ...but evicted stamps cannot be resurrected.
        ok, reason = gate.evaluate(100, cloud_stamp_nanosec=0)
        self.assertFalse(ok)
        self.assertIn(reason, ("status_stale", "status_missing"))

    def test_geometry_not_settled(self):
        gate = GeometryStatusGate()
        gate.update(_status(100.0, geometry_ok=False))
        ok, reason = gate.evaluate(100, cloud_stamp_nanosec=0)
        self.assertFalse(ok)
        self.assertEqual(reason, "geometry_not_settled")

    def test_exact_match_beats_recent_malformed(self):
        """A valid buffered entry still authorizes after a later
        malformed payload (which has no key of its own)."""
        gate = GeometryStatusGate()
        gate.update(_status(100.0))
        gate.update("garbage")
        ok, reason = gate.evaluate(100, cloud_stamp_nanosec=0)
        self.assertTrue(ok)
        self.assertEqual(reason, "")


class TestPipelineStatusFailClosed(unittest.TestCase):

    def _detector(self):
        return PlatformFreeDetector(CONFIG, support_mode="auto",
                                    stability_window=1)

    def test_fresh_status_reaches_measured_support(self):
        cargo, raw = _scene()
        det = self._detector()
        result = det.update(
            cargo, raw, source="measure", geometry_ok=True,
            stamp_sec=100.0)
        self.assertTrue(result.height_valid)
        self.assertEqual(result.height_source,
                         HEIGHT_SOURCE_MEASURED_SUPPORT)

    def test_status_gate_reasons_block_measured_support(self):
        cargo, raw = _scene()
        for gate_reason, expected in (
                ("status_missing", DETECT_SUPPORT_STATUS_MISSING),
                ("status_malformed", DETECT_SUPPORT_STATUS_MALFORMED),
                ("status_stale", DETECT_SUPPORT_STATUS_STALE)):
            det = self._detector()
            result = det.update(
                cargo, raw, source="measure", geometry_ok=False,
                geometry_gate_reason=gate_reason, stamp_sec=100.0)
            # Top stays valid; only the height is withheld.
            self.assertTrue(result.top_valid, gate_reason)
            self.assertFalse(result.height_valid, gate_reason)
            self.assertEqual(result.height_source,
                             HEIGHT_SOURCE_UNAVAILABLE)
            self.assertEqual(result.support_gate, gate_reason)
            self.assertIsNone(result.support)
            self.assertEqual(result.box.reason, expected, gate_reason)

    def test_hold_track_stays_top_only_with_fresh_status(self):
        cargo, raw = _scene()
        det = self._detector()
        result = det.update(
            cargo, raw, source="hold_track", geometry_ok=True,
            stamp_sec=100.0)
        self.assertTrue(result.top_valid)
        self.assertFalse(result.height_valid)
        self.assertEqual(result.support_gate, "hold_track")
        self.assertEqual(result.box.reason, DETECT_SUPPORT_STAMP_MISMATCH)

    def test_stale_support_not_relabeled(self):
        """Absence-of-evidence misses do not reset the stability window;
        positive motion evidence does (PF-R5 field evidence)."""
        cargo, raw = _scene()
        det = PlatformFreeDetector(CONFIG, support_mode="auto",
                                   stability_window=3)
        det.update(cargo, raw, source="measure", geometry_ok=True,
                   stamp_sec=100.0)
        det.update(cargo, raw, source="measure", geometry_ok=True,
                   stamp_sec=100.25)
        det.update(cargo, raw, source="measure", geometry_ok=True,
                   stamp_sec=100.5)
        # Stale status frame: TOP_ONLY, but the fitted window survives.
        result = det.update(cargo, raw, source="measure", geometry_ok=False,
                            geometry_gate_reason="status_stale",
                            stamp_sec=100.75)
        self.assertFalse(result.height_valid)
        result = det.update(cargo, raw, source="measure", geometry_ok=True,
                            stamp_sec=101.0)
        self.assertTrue(result.height_valid)
        # Raw-buffer miss: same skip semantics.
        result = det.update(cargo, None, source="measure", geometry_ok=True,
                            stamp_sec=101.25)
        self.assertFalse(result.height_valid)
        result = det.update(cargo, raw, source="measure", geometry_ok=True,
                            stamp_sec=101.5)
        self.assertTrue(result.height_valid)
        # Motion evidence (geometry_not_settled) DOES reset the window:
        # the scene may have moved, past measurements are stale.
        result = det.update(cargo, raw, source="measure", geometry_ok=False,
                            geometry_gate_reason=None,
                            stamp_sec=101.75)
        self.assertFalse(result.height_valid)
        result = det.update(cargo, raw, source="measure", geometry_ok=True,
                            stamp_sec=102.0)
        self.assertFalse(result.height_valid)  # warming up again


class TestExactStampJoinOneNanosecond(unittest.TestCase):

    def test_one_nanosecond_mismatch_does_not_fuse(self):
        join = ExactStampJoin(maxlen=4)
        from std_msgs.msg import Header

        yolo = SimpleNamespace(header=Header())
        yolo.header.stamp.sec, yolo.header.stamp.nanosec = 100, 500000000
        pair = join.push_left(stamp_key(yolo.header.stamp), yolo)
        self.assertIsNone(pair)
        cloud = SimpleNamespace(header=Header())
        cloud.header.stamp.sec = 100
        cloud.header.stamp.nanosec = 500000001  # +1 ns
        pair = join.push_right(stamp_key(cloud.header.stamp), cloud)
        self.assertIsNone(pair)


class TestSemanticStampedTfLookup(unittest.TestCase):
    """_lookup_rt uses the acquisition stamp; no latest-TF fallback."""

    def _node(self, buffer):
        import importlib.util
        import os

        script = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "scripts", "semantic_point_filter_node.py")
        spec = importlib.util.spec_from_file_location(
            "semantic_point_filter_node", script)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        node = object.__new__(mod.SemanticPointFilterNode)
        node._tf_buffer = buffer
        return node, mod

    def test_lookup_receives_acquisition_stamp(self):
        import rclpy

        class RecordingBuffer(object):
            def __init__(self):
                self.calls = []

            def lookup_transform(self, target, source, time, timeout=None):
                self.calls.append((target, source, time, timeout))
                from tf2_ros import TransformException
                raise TransformException("miss")

        class FakeStamp(object):
            sec, nanosec = 1234, 567000000

        buffer = RecordingBuffer()
        node, _ = self._node(buffer)
        node._lookup_rt("world", "camera_link", FakeStamp())
        self.assertGreaterEqual(len(buffer.calls), 1)
        target, source, time, _timeout = buffer.calls[0]
        self.assertEqual((target, source), ("world", "camera_link"))
        # Stamp equality: nanoseconds must survive the conversion.
        self.assertEqual(
            (time.nanoseconds // 10**9, time.nanoseconds % 10**9),
            (1234, 567000000))
        self.assertIsInstance(time, rclpy.time.Time)
        self.assertNotEqual(time.nanoseconds, 0)

    def test_missing_stamped_tf_is_explicit_miss(self):
        from tf2_ros import TransformException

        class MissingBuffer(object):
            def __init__(self):
                self.calls = []

            def lookup_transform(self, target, source, time,
                                 timeout=None):
                self.calls.append(time.nanoseconds)
                raise TransformException("no stamped transform")

        buffer = MissingBuffer()
        node, _ = self._node(buffer)

        class FakeStamp(object):
            sec, nanosec = 1234, 567000000
        result = node._lookup_rt("world", "camera_link", FakeStamp())
        self.assertIsNone(result)
        # Wall-bounded retry only: every attempt carries the SAME
        # acquisition stamp — a latest-TF (time=0) fallback never appears.
        self.assertGreaterEqual(len(buffer.calls), 1)
        self.assertTrue(all(ns == 1234567000000 for ns in buffer.calls))
        # And the retry is bounded (no infinite loop; we returned).


if __name__ == "__main__":
    unittest.main()
