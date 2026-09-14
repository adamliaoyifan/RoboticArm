#!/usr/bin/env python3
"""Tests for Livox CustomMsg CDR decode and the calib dump."""
import os
import struct
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "fixtures"))
from make_tiny_replay_bag import build_fixture  # noqa: E402

from luggage_perception.eval.bag_mcap_source import (  # noqa: E402
    LIDAR_TOPIC,
    LivoxCustomScan,
    decode_lidar_scan,
    decode_livox_custom_cdr,
    iter_bag_messages,
)
from luggage_perception.eval.bag_tf import BagTfBuffer  # noqa: E402
from luggage_perception.eval.lidar_camera_calib_export import (  # noqa: E402
    check_table_corner,
    export_lidar_camera_calib,
    ransac_plane,
)


def _cdr_align_buf(buf, align, origin=4):
    payload = len(buf) - origin
    pad = (align - (payload % align)) % align
    buf.extend(b"\x00" * pad)


def encode_livox_custom(xyz, frame="livox_frame", sec=1, nsec=2,
                        timebase=1000, lidar_id=1):
    buf = bytearray(b"\x00\x01\x00\x00")
    buf += struct.pack("<iI", int(sec), int(nsec))
    payload = frame.encode("utf-8") + b"\x00"
    buf += struct.pack("<I", len(payload))
    buf += payload
    _cdr_align_buf(buf, 8)
    buf += struct.pack("<Q", int(timebase))
    buf += struct.pack("<I", len(xyz))
    buf += struct.pack("<B", int(lidar_id))
    buf += b"\x00\x00\x00"
    _cdr_align_buf(buf, 4)
    buf += struct.pack("<I", len(xyz))
    for i, (x, y, z) in enumerate(xyz):
        buf += struct.pack("<IfffBBB", i * 1000, float(x), float(y),
                           float(z), 10 + i, 1, i % 4)
        buf += b"\x00"
    return bytes(buf)


class TestLivoxCustomCdr(unittest.TestCase):
    def test_roundtrip_points_and_times(self):
        raw = encode_livox_custom([(1.0, 2.0, 3.0), (0.1, -0.2, 0.3)])
        scan = decode_livox_custom_cdr(raw)
        self.assertIsInstance(scan, LivoxCustomScan)
        self.assertEqual(scan.frame_id, "livox_frame")
        self.assertEqual(scan.header_stamp_ns, 1_000_000_002)
        self.assertEqual(scan.lidar_id, 1)
        pts = decode_lidar_scan(scan)
        self.assertEqual(len(pts), 2)
        self.assertAlmostEqual(float(pts[0]["x"]), 1.0, places=5)
        self.assertAlmostEqual(float(pts[1]["z"]), 0.3, places=5)
        self.assertEqual(int(pts[0]["tag"]), 1)
        self.assertAlmostEqual(float(pts[1]["timestamp"]), 1000 + 1000, delta=0.5)

    def test_real_bag_first_scan_if_present(self):
        bag = os.path.expanduser(
            "~/work/robotarm_bags/record_site_pendant_20260911_220406")
        if not os.path.isdir(bag):
            self.skipTest("site calib bag not on disk")
        rows = list(iter_bag_messages(bag, topics=[LIDAR_TOPIC]))
        self.assertGreaterEqual(len(rows), 1)
        self.assertIsInstance(rows[0].message, LivoxCustomScan)
        pts = decode_lidar_scan(rows[0].message)
        self.assertGreater(len(pts), 1000)
        self.assertEqual(rows[0].message.frame_id, "livox_frame")
        finite = np.isfinite(pts["x"]) & np.isfinite(pts["y"]) & np.isfinite(
            pts["z"])
        self.assertGreater(int(finite.sum()), 1000)


class TestCalibExportSmoke(unittest.TestCase):
    def test_tiny_fixture_writes_summary(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        bag = build_fixture(os.path.join(tmp.name, "tiny.mcap"))
        out = os.path.join(tmp.name, "out")
        summary = export_lidar_camera_calib(
            bag, out, camera_stride=1, apply_xacro=False, extra_frames=())
        self.assertTrue(os.path.isfile(os.path.join(out, "summary.json")))
        self.assertGreaterEqual(summary["n_camera"], 1)
        self.assertGreaterEqual(summary["n_lidar"], 1)

    def test_ransac_plane_recovers_z_plane(self):
        rng = np.random.RandomState(1)
        xy = rng.uniform(-0.4, 0.4, size=(400, 2))
        z = np.full((400, 1), 0.8) + rng.normal(0, 0.002, size=(400, 1))
        pts = np.hstack([xy, z])
        normal, offset, mask = ransac_plane(pts, thresh=0.01)
        self.assertGreater(float(mask.mean()), 0.8)
        self.assertGreater(abs(normal[2]), 0.95)

    def test_set_static_changes_lookup(self):
        buf = BagTfBuffer()
        from geometry_msgs.msg import TransformStamped
        tf = TransformStamped()
        tf.header.frame_id = "world"
        tf.child_frame_id = "livox_frame"
        tf.transform.rotation.w = 1.0
        buf.add_transform(tf, static=True)
        mat = np.eye(4)
        mat[0, 3] = 0.25
        buf.set_static("world", "livox_frame", mat)
        got = buf.lookup_matrix("world", "livox_frame", 0)
        self.assertAlmostEqual(float(got[0, 3]), 0.25)

    def test_table_corner_near_zero_when_clouds_match(self):
        rng = np.random.RandomState(0)
        xs = np.linspace(0.0, 0.6, 25)
        ys = np.linspace(0.0, 0.4, 17)
        xx, yy = np.meshgrid(xs, ys)
        cam = np.stack([xx.ravel(), yy.ravel(), np.full(xx.size, 0.9)], axis=1)
        rgb = np.full((len(cam), 3), 220, dtype=np.uint8)
        lid = cam + rng.normal(0.0, 0.003, cam.shape)
        report = check_table_corner(cam, rgb, lid)
        self.assertTrue(report["ok"])
        self.assertLess(report["corner_residual_m"], 0.03)
        self.assertTrue(report["coincide_table_corner"])


if __name__ == "__main__":
    unittest.main()
