#!/usr/bin/env python3
"""Failure-time capture: occ snapshot + source cloud + depth frame.

Everything here is best effort: a missing message records itself under
``missing`` and never raises, and the artifacts must round-trip (PLY with
a parseable header, npy that loads back, sidecar that names them all).
"""

import json
import os
import shutil
import struct
import tempfile
import unittest

import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs.msg import PointField

from luggage_planning.debug_capture import (
    write_failure_capture,
    write_ply_xyz,
)


def _cloud_msg(n_points, frame="camera_depth_optical_frame"):
    xyz = np.linspace(-1.0, 1.0, n_points * 3, dtype="<f4")
    msg = PointCloud2()
    msg.header.frame_id = frame
    msg.header.stamp.sec = 1789440000
    msg.header.stamp.nanosec = 123456789
    msg.height = 1
    msg.width = n_points
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * n_points
    msg.is_dense = True
    for i, name in enumerate(("x", "y", "z")):
        field = PointField()
        field.name = name
        field.offset = 4 * i
        field.datatype = PointField.FLOAT32
        field.count = 1
        msg.fields.append(field)
    msg.data = xyz.tobytes()
    return msg


def _depth_msg(height=4, width=6, frame="camera_depth_optical_frame"):
    msg = Image()
    msg.header.frame_id = frame
    msg.header.stamp.sec = 1789440000
    msg.header.stamp.nanosec = 200000000
    msg.height = height
    msg.width = width
    msg.encoding = "16UC1"
    msg.is_bigendian = False
    msg.step = 2 * width
    msg.data = (np.arange(height * width, dtype="<u2") * 10).tobytes()
    return msg


class PlyWriterTest(unittest.TestCase):
    def test_header_and_bytes(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "cloud.ply")
            write_ply_xyz(path, np.array([[0.0, 1.0, 2.0]]))
            with open(path, "rb") as handle:
                blob = handle.read()
        head, body = blob.split(b"end_header\n", 1)
        self.assertIn(b"format binary_little_endian 1.0", head)
        self.assertIn(b"element vertex 1", head)
        self.assertIn(b"element face 0", head)
        self.assertEqual(len(body), 12)
        self.assertEqual(struct.unpack("<3f", body), (0.0, 1.0, 2.0))


class FailureCaptureTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self._tmp, True)

    def _files(self):
        return sorted(os.listdir(self._tmp))

    def test_full_capture_round_trip(self):
        surface = {"map_revision": 7, "geometry_hash": "abc",
                   "height": [[0.0]], "state": [["free"]]}
        capture = write_failure_capture(
            self._tmp, "1789440000123_traverse", surface=surface,
            surface_age_s=0.25,
            cloud_msg=_cloud_msg(30000), depth_msg=_depth_msg())
        self.assertEqual(capture["missing"], [])
        self.assertEqual(capture["map_revision"], 7)
        # Cloud decimated to the cap.
        self.assertLessEqual(capture["cloud"]["n_points"], 20001)
        self.assertGreater(capture["cloud"]["decimation"], 1)
        self.assertAlmostEqual(
            capture["cloud"]["stamp"], 1789440000.123456789, places=4)
        # Surface and sidecar parse back.
        with open(capture["surface_2d_dump"], encoding="utf-8") as handle:
            self.assertEqual(json.load(handle)["geometry_hash"], "abc")
        with open(capture["sidecar"], encoding="utf-8") as handle:
            sidecar = json.load(handle)
        self.assertEqual(sidecar["tag"], "1789440000123_traverse")
        self.assertEqual(sidecar["cloud"]["n_points"],
                         capture["cloud"]["n_points"])
        # Depth npy loads with the original values.
        depth = np.load(capture["depth"]["path"])
        self.assertEqual(depth.shape, (4, 6))
        self.assertEqual(int(depth[1, 2]), 80)
        # All artifacts are on disk.
        for key in ("surface_2d_dump", "sidecar"):
            self.assertTrue(os.path.isfile(capture[key]))
        self.assertTrue(os.path.isfile(capture["cloud"]["path"]))
        # No temp files left behind.
        self.assertEqual(
            [name for name in self._files() if name.endswith(".tmp")], [])

    def test_missing_messages_record_not_raise(self):
        capture = write_failure_capture(
            self._tmp, "1_descend", surface=None,
            cloud_msg=None, depth_msg=None)
        self.assertIn("surface_2d_absent", capture["missing"])
        self.assertIn("cloud_absent", capture["missing"])
        self.assertIn("depth_absent", capture["missing"])
        self.assertIsNone(capture["surface_2d_dump"])
        self.assertIsNone(capture["cloud"]["path"])
        # Sidecar still written so the gap is itself recorded.
        self.assertTrue(os.path.isfile(capture["sidecar"]))

    def test_small_cloud_not_decimated(self):
        capture = write_failure_capture(
            self._tmp, "2_retreat", cloud_msg=_cloud_msg(50))
        self.assertEqual(capture["cloud"]["decimation"], 1)
        self.assertEqual(capture["cloud"]["n_points"], 50)

    def test_undecodable_cloud_records_reason(self):
        bad = _cloud_msg(10)
        bad.fields = [bad.fields[0]]  # y/z missing
        capture = write_failure_capture(self._tmp, "3_insert",
                                        cloud_msg=bad)
        self.assertIn("cloud_decode_failed", capture["missing"])


if __name__ == "__main__":
    unittest.main()
