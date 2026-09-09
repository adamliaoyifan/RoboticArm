"""HE-1 offline geometry and board checks (no URDF edits)."""

from __future__ import division

import json
import math
import os
import tempfile
import unittest

import numpy as np

from luggage_description.he1.board import rendered_pitch_check, write_pdf
from luggage_description.he1.geometry import make_T, rpy_to_R, R_to_rpy
from luggage_description.he1.register import (
    enumerate_bar_pairs,
    gui_delta,
    roundtrip_points,
    two_hole_plane_frame,
)


class GeometryTests(unittest.TestCase):
    def test_rpy_roundtrip(self):
        rpy = np.array([0.03769911, 1.36345121, 1.57079633])
        recovered = R_to_rpy(rpy_to_R(rpy))
        self.assertTrue(np.allclose(rpy, recovered, atol=1e-9))

    def test_roundtrip_four_points(self):
        T = two_hole_plane_frame(
            np.array([-62.7, 0.0, -48.0]),
            np.array([62.7, 0.0, -48.0]),
            np.array([0.0, 0.0, 1.0]),
        )
        T[:3, 3] /= 1000.0
        pts = np.array([
            [0.01, 0.02, 0.03],
            [-0.04, 0.00, 0.01],
            [0.00, -0.05, 0.02],
            [0.03, 0.01, -0.02],
        ])
        err = roundtrip_points(T, pts)
        self.assertLess(err, 1e-9)

    def test_recompute_from_feature_table(self):
        cad_a = np.array([-62.7, 0.0, -48.0])
        cad_b = np.array([62.7, 0.0, -48.0])
        stl_a = np.array([-40.0, 88.0, -2.0])
        stl_b = np.array([85.0, 88.0, -2.0])
        n = np.array([0.0, 0.2, -0.98])
        from luggage_description.he1.register import _candidate_transform
        T1 = _candidate_transform(cad_a, cad_b, stl_a, stl_b, np.array([0.0, 0.0, 1.0]), n)
        table = {
            "cad_a": cad_a.tolist(),
            "cad_b": cad_b.tolist(),
            "stl_a": stl_a.tolist(),
            "stl_b": stl_b.tolist(),
            "n": n.tolist(),
        }
        T2 = _candidate_transform(
            np.array(table["cad_a"]), np.array(table["cad_b"]),
            np.array(table["stl_a"]), np.array(table["stl_b"]),
            np.array([0.0, 0.0, 1.0]), np.array(table["n"]),
        )
        self.assertLess(float(np.max(np.abs(T1 - T2))), 1e-12)

    def test_gui_delta_identity(self):
        from luggage_description.he1.register import GUI_RPY, GUI_XYZ_M
        T = make_T(rpy_to_R(GUI_RPY), GUI_XYZ_M)
        delta = gui_delta(T)
        self.assertLess(delta["translation_norm_m"], 1e-12)
        self.assertLess(delta["rotation_angle_deg"], 1e-6)

    def test_pair_enumeration_marks_125mm(self):
        holes = [
            {"id": "STL_BAR_H0", "centre": np.array([-40.0, 88.0, 0.0])},
            {"id": "STL_BAR_H1", "centre": np.array([-25.0, 88.0, 0.0])},
            {"id": "STL_BAR_H2", "centre": np.array([70.0, 88.0, 0.0])},
            {"id": "STL_BAR_H3", "centre": np.array([85.0, 88.0, 0.0])},
        ]
        recs = enumerate_bar_pairs(holes)
        compatible = [r for r in recs if r["compatible"]]
        self.assertEqual(len(compatible), 1)
        self.assertEqual(compatible[0]["ids"], ["STL_BAR_H0", "STL_BAR_H3"])


class MountInventoryTests(unittest.TestCase):
    def test_bar_eef_mid360_distinguished(self):
        from luggage_description.he1.mount_features import (
            extract_mount_features,
            load_mount_mesh,
        )
        root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
        stl = os.path.join(
            root, "src", "luggage_gazebo", "models", "arm_realsense",
            "arm_realsense_v1.3.stl")
        if not os.path.isfile(stl):
            stl = os.path.join(
                os.path.dirname(__file__), "..", "..", "luggage_gazebo",
                "models", "arm_realsense", "arm_realsense_v1.3.stl")
        mesh = load_mount_mesh(stl)
        features = extract_mount_features(mesh)
        self.assertEqual(len(features["bar_holes"]), 4)
        self.assertEqual(len(features["eef_holes"]), 2)
        self.assertEqual(len(features["mid360_holes"]), 4)
        xs = [float(h["centre"][0]) for h in features["bar_holes"]]
        self.assertLess(min(xs), -30.0)
        self.assertGreater(max(xs), 80.0)
        span = max(xs) - min(xs)
        self.assertLess(abs(span - 125.0), 1.0)


class BoardTests(unittest.TestCase):
    def test_true_scale_pitch(self):
        check = rendered_pitch_check()
        self.assertTrue(check["pass"], check)
        self.assertGreaterEqual(check["corners"], 20)

    def test_pdf_page_size(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "board.pdf")
            meta = write_pdf(path)
            self.assertTrue(os.path.isfile(path))
            self.assertGreater(os.path.getsize(path), 1000)
            width, height = meta["page_size_mm"]
            self.assertGreaterEqual(width, 600.0 - 1.0)
            self.assertGreaterEqual(height, 500.0 - 1.0)


if __name__ == "__main__":
    unittest.main()
