"""Layer 3 from CC600 hand-eye matches CAD + driver optical round-trip."""

from __future__ import division

import os
import unittest

import numpy as np

from luggage_description.handeye_layer3 import (
    T_d555_optical,
    T_end_adapter,
    average_handeye,
    freeze_from_handeye_json,
    layer3_from_end_optical,
    load_handeye_json,
    parse_xacro_xyz_rpy,
)

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
JSON = os.path.join(ROOT, "config", "handeye_cc600_20260911_21-38.json")
XACRO = os.path.join(ROOT, "config", "camera_mount_origin.xacro")


class HandeyeLayer3Test(unittest.TestCase):
    def test_roundtrip_end_optical(self):
        blob = load_handeye_json(JSON)
        T_x = average_handeye(blob["methods"])
        T3 = layer3_from_end_optical(T_x)
        rebuilt = T_end_adapter().dot(T3).dot(T_d555_optical())
        err = np.linalg.norm(rebuilt[:3, 3] - T_x[:3, 3])
        self.assertLess(err, 1e-9)

    def test_xacro_matches_frozen_json(self):
        frozen = freeze_from_handeye_json(JSON)
        xyz, rpy = parse_xacro_xyz_rpy(XACRO, "cam_mount_xyz", "cam_mount_rpy")
        self.assertTrue(all(
            abs(a - b) < 5e-7 for a, b in zip(xyz, frozen["xyz"])))
        self.assertTrue(all(
            abs(a - b) < 5e-9 for a, b in zip(rpy, frozen["rpy"])))
        self.assertEqual(frozen["n"], 18)
