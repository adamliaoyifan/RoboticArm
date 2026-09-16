#!/usr/bin/env python3
"""DSIM-3 fail-closed matrix and remaining active camera-cloud scan."""

from __future__ import annotations

import collections
import importlib.util
import os
import threading
import unittest

import numpy as np
import pytest

from luggage_perception.depth_deprojection import deproject_stride
from luggage_perception.semantic_point_filter import (
    CameraIntrinsics,
    DepthToColorExtrinsics,
    SemanticPointFilter,
)
from luggage_perception.sensor_preprocessor import SensorPreprocessor
from luggage_perception.sensor_types import (
    CameraInfoFrame,
    DepthFrame,
    OpaquePayload,
    RgbFrame,
)


WORKSPACE = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", ".."))

FORBIDDEN_TOPICS = (
    "/camera/depth/points",
    "/d435/points",
    "/luggage/preprocessed/camera/depth/points",
)

ACTIVE_FILES = (
    "src/luggage_gazebo/launch/sim_world.launch.py",
    "src/luggage_description/config/realsense_d435.yaml",
    "src/luggage_perception/config/sensor_preprocessor.yaml",
    "src/luggage_perception/config/semantic_segmenter.yaml",
    "src/luggage_perception/test/pf_r9_b34_probe.py",
    "src/luggage_perception/test/pf_r9_g2_d1_probe.py",
    "src/luggage_perception/test/pf_r9_period_probe.py",
    "scripts/platform_free_height_gate4_eval.py",
    "research/pf_r6_ransac/pfr6bench/capture_sim.py",
    "docs/CONTEXT.md",
)


def _load_script(rel, attr):
    path = os.path.join(WORKSPACE, rel)
    spec = importlib.util.spec_from_file_location(attr, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _key(stamp):
    return (int(stamp), int(round((float(stamp) % 1.0) * 1e9)))


def _rgb(stamp, h=360, w=640):
    import array
    image = np.full((h, w, 3), 7, dtype=np.uint8)
    raw = array.array("B", image.tobytes())
    return RgbFrame(
        stamp=stamp, frame_id="optical", encoding="rgb8",
        payload=OpaquePayload(raw), height=h, width=w, step=w * 3,
        stamp_key=_key(stamp))


def _depth(stamp, h=360, w=640):
    import array
    depth = np.full((h, w), 1000, dtype="<u2")
    raw = array.array("B", depth.tobytes())
    return DepthFrame(
        stamp=stamp, frame_id="optical", units="millimetres",
        encoding="16UC1", payload=OpaquePayload(raw),
        height=h, width=w, step=w * 2, is_bigendian=0,
        stamp_key=_key(stamp))


def _info(stamp, h=360, w=640, fx=323.1775):
    return CameraInfoFrame(
        stamp=stamp, frame_id="optical", width=w, height=h,
        fx=fx, fy=fx, cx=w / 2.0, cy=h / 2.0)


class TestDsim3ActiveCameraCloudGone(unittest.TestCase):
    def test_migrated_active_files_have_no_camera_cloud_topics(self):
        hits = []
        for rel in ACTIVE_FILES:
            text = open(os.path.join(WORKSPACE, rel), encoding="utf-8").read()
            for needle in FORBIDDEN_TOPICS:
                if needle in text:
                    hits.append("%s:%s" % (rel, needle))
        self.assertEqual(hits, [])


class TestDsim3FaultMatrix(unittest.TestCase):
    def test_missing_aligned_depth_named_counter(self):
        pre = SensorPreprocessor(camera_maxlen=15, camera_horizon_sec=1.0)
        pre.update_camera_info(_info(5.0))
        self.assertIsNone(pre.update_rgb(_rgb(5.0)))
        self.assertIsNone(pre.update_rgb(_rgb(5.1)))
        self.assertEqual(pre.last_rejection_reason(), "depth_wait_timeout")
        self.assertGreaterEqual(pre.depth_wait_skips, 1)

    def test_missing_depth_detector_lookup_named_counter(self):
        mod = _load_script(
            "src/luggage_perception/scripts/luggage_detector_node.py",
            "luggage_detector_node")
        node = object.__new__(mod.LuggageDetector)
        node._raw_lock = threading.Lock()
        node._raw_buffer = collections.OrderedDict()
        node._raw_counts = collections.Counter()
        node._raw_buffer_maxlen = 15
        node._last_raw_lookup = {}
        out = node._pop_raw_world_with_retry((1, 0), attempts=1)
        self.assertIsNone(out)
        self.assertEqual(node._raw_counts["raw_lookup_miss"], 1)

    def test_stamp_mismatch_no_pair(self):
        pre = SensorPreprocessor(camera_maxlen=15, camera_horizon_sec=1.0)
        pre.update_camera_info(_info(6.0))
        pre.update_depth(_depth(6.02))
        self.assertIsNone(pre.update_rgb(_rgb(6.0)))

    def test_dimension_mismatch_named_counter(self):
        pre = SensorPreprocessor(camera_maxlen=15, camera_horizon_sec=1.0)
        pre.update_camera_info(_info(5.0))
        pre.update_depth(_depth(5.0, w=642))
        self.assertIsNone(pre.update_rgb(_rgb(5.0)))
        self.assertEqual(pre.last_rejection_reason(),
                         "acquisition_dimension_mismatch")
        self.assertEqual(pre.diagnostics()["acquisition_mismatches"], 1)

    def test_frame_mismatch_named_counter(self):
        rgb = _rgb(5.0)
        rgb.frame_id = "other_frame"
        pre = SensorPreprocessor(camera_maxlen=15, camera_horizon_sec=1.0)
        pre.update_camera_info(_info(5.0))
        pre.update_depth(_depth(5.0))
        self.assertIsNone(pre.update_rgb(rgb))
        self.assertEqual(pre.last_rejection_reason(),
                         "acquisition_frame_mismatch")

    def test_invalid_k_named_counter(self):
        pre = SensorPreprocessor(camera_maxlen=15, camera_horizon_sec=1.0)
        pre.update_camera_info(_info(5.0))
        pre.update_camera_info(_info(5.0, fx=222.0), slots=("color",))
        pre.update_depth(_depth(5.0))
        self.assertIsNone(pre.update_rgb(_rgb(5.0)))
        self.assertEqual(pre.last_rejection_reason(),
                         "acquisition_intrinsics_mismatch")

    def test_unsupported_and_truncated_depth_fail_closed(self):
        pytest.importorskip("sensor_msgs")
        from sensor_msgs.msg import Image
        from luggage_perception import ros_message_adapters as adapters

        bad = Image()
        bad.encoding = "32FC1"
        bad.height = 2
        bad.width = 2
        bad.step = 8
        bad.data = b"\x00" * 16
        self.assertIsNone(adapters.depth_array_from_msg(bad))

        truncated = Image()
        truncated.encoding = "16UC1"
        truncated.height = 2
        truncated.width = 2
        truncated.step = 4
        truncated.data = b"\x00\x01"
        self.assertIsNone(adapters.depth_array_from_msg(truncated))

        pre = SensorPreprocessor(camera_maxlen=15, camera_horizon_sec=1.0)
        self.assertIsNone(pre.update_depth(DepthFrame(
            stamp=5.0, frame_id="optical", units="metres",
            encoding="32FC1")))
        self.assertEqual(pre.last_rejection_reason(),
                         "unexpected_depth_encoding")

    def test_filter_size_mismatch_empty_geometry(self):
        intr = CameraIntrinsics(
            fx=323.1775, fy=322.8994, cx=317.7526, cy=178.0294,
            width=640, height=360)
        filt = SemanticPointFilter(
            color_intrinsics=intr, depth_intrinsics=intr,
            depth_to_color=DepthToColorExtrinsics.identity(),
            cargo_labels=[2], obstacle_labels=[4])
        depth = np.full((360, 640), 700, dtype="<u2")
        labels = np.full((180, 320), 2, dtype=np.uint8)
        cargo, obstacle = filt.filter_depth(depth, labels)
        self.assertEqual(cargo.shape[0], 0)
        self.assertEqual(obstacle.shape[0], 0)
        self.assertEqual(filt.last_stats["dimension_mismatch"], 1)

    def test_zero_depth_is_not_geometry(self):
        class K:
            fx = fy = 323.0
            cx = 320.0
            cy = 180.0
        mm = np.zeros((8, 8), dtype="<u2")
        pts, n = deproject_stride(mm, K(), stride=1)
        self.assertEqual(n, 0)
        self.assertEqual(len(pts), 0)

    def test_newer_tf_present_exact_stamp_missing(self):
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import TransformException

        class NewerOnlyBuffer(object):
            def __init__(self):
                self.calls = []

            def lookup_transform(self, target, source, time, timeout=None):
                ns = int(time.nanoseconds)
                self.calls.append(ns)
                if ns == 0:
                    msg = TransformStamped()
                    msg.transform.rotation.w = 1.0
                    return msg
                raise TransformException("exact stamp missing; newer TF exists")

        mod = _load_script(
            "src/luggage_perception/scripts/semantic_point_filter_node.py",
            "semantic_point_filter_node")
        node = object.__new__(mod.SemanticPointFilterNode)
        node._tf_buffer = NewerOnlyBuffer()
        node._world_frame = "world"

        class FakeStamp(object):
            sec, nanosec = 10, 250000000

        result = node._lookup_rt("world", "camera_depth_optical_frame",
                                 FakeStamp())
        self.assertIsNone(result)
        self.assertGreaterEqual(len(node._tf_buffer.calls), 1)
        self.assertNotIn(0, node._tf_buffer.calls)
        self.assertTrue(all(ns == 10250000000 for ns in node._tf_buffer.calls))


if __name__ == "__main__":
    unittest.main()
