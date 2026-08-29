#!/usr/bin/env python3
"""Unit tests for wrist suction-panel silhouette projection. No ROS."""

import math
import os
import struct
import tempfile
import unittest

import numpy as np

from luggage_perception.wrist_self_body import (
    dilate_mask,
    load_binary_stl,
    optical_from_panel_matrix,
    panel_mask_from_meshes,
    rasterize_triangles,
    transform_matrix,
    transform_triangles,
)


def _write_binary_stl(path, triangles):
    tris = np.asarray(triangles, dtype=np.float32).reshape(-1, 3, 3)
    buf = bytearray(80)
    buf += struct.pack("<I", len(tris))
    for tri in tris:
        buf += struct.pack("<3f", 0.0, 0.0, 1.0)
        buf += struct.pack("<9f", *tri.reshape(-1))
        buf += struct.pack("<H", 0)
    with open(path, "wb") as handle:
        handle.write(buf)


class TestRasterize(unittest.TestCase):
    def test_front_triangle_covers_principal_point(self):
        # z=1 plane, triangle around optical axis -> pixel near (cx, cy).
        tris = np.array([[
            [-0.05, -0.05, 1.0],
            [0.05, -0.05, 1.0],
            [0.0, 0.05, 1.0],
        ]], dtype=np.float64)
        mask = rasterize_triangles(tris, fx=100.0, fy=100.0,
                                   cx=20.0, cy=15.0, width=40, height=30)
        self.assertTrue(mask[15, 20])
        self.assertFalse(mask[0, 0])

    def test_behind_camera_is_skipped(self):
        tris = np.array([[
            [-0.05, -0.05, -1.0],
            [0.05, -0.05, -1.0],
            [0.0, 0.05, -1.0],
        ]], dtype=np.float64)
        mask = rasterize_triangles(tris, fx=100.0, fy=100.0,
                                   cx=20.0, cy=15.0, width=40, height=30)
        self.assertFalse(mask.any())


class TestDilate(unittest.TestCase):
    def test_radius_grows_a_pixel(self):
        mask = np.zeros((5, 5), dtype=bool)
        mask[2, 2] = True
        out = dilate_mask(mask, 1)
        self.assertTrue(out[2, 3])
        self.assertTrue(out[1, 2])
        self.assertFalse(out[0, 0])


class TestPanelMaskFromCad(unittest.TestCase):
    def test_p04_projects_to_bottom_arc(self):
        mesh = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__)))),
            "luggage_gazebo", "models", "suction_panel", "collision",
            "p04.stl")
        if not os.path.isfile(mesh):
            self.skipTest("p04.stl not in workspace")
        t_opt = optical_from_panel_matrix(
            adapter_xyz=(0.0168, -0.0156, 0.0702),
            adapter_rpy=(1.57707939, -0.00003948, -3.13530959),
            camera_xyz=(0.013, 0.097, -0.021),
            camera_rpy=(0.03769911, 1.36345121, 1.57079633),
        )
        mask = panel_mask_from_meshes(
            [mesh],
            mesh_origin=(-0.0909, -0.28583, -0.0909),
            t_optical_from_panel=t_opt,
            fx=337.22194822727283, fy=337.22194822727283,
            cx=320.0, cy=240.0, width=640, height=480, dilate_px=0)
        self.assertGreater(int(mask.sum()), 1000)
        rows = np.where(mask.any(axis=1))[0]
        cols = np.where(mask.any(axis=0))[0]
        self.assertGreaterEqual(int(rows[0]), 420)
        self.assertEqual(int(rows[-1]), 479)
        self.assertGreater(int(cols[0]), 150)
        self.assertLess(int(cols[-1]), 530)
        # Not a full-width strip.
        self.assertLess(int(cols[-1] - cols[0]), 400)
        self.assertFalse(mask[240, 320])

    def test_roundtrip_tiny_stl(self):
        tris = np.array([[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ]], dtype=np.float32)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "t.stl")
            _write_binary_stl(path, tris)
            loaded = load_binary_stl(path)
        np.testing.assert_allclose(loaded, tris, atol=1e-6)
        moved = transform_triangles(loaded, transform_matrix((0, 0, 1), (0, 0, 0)))
        self.assertAlmostEqual(moved[0, 0, 2], 1.0)


class TestOpticalChain(unittest.TestCase):
    def test_optical_z_is_camera_forward(self):
        # A point on camera_link +X (forward) should have optical +Z.
        t_opt_from_panel = optical_from_panel_matrix(
            (0.0168, -0.0156, 0.0702),
            (1.57707939, -0.00003948, -3.13530959),
            (0.013, 0.097, -0.021),
            (0.03769911, 1.36345121, 1.57079633),
        )
        self.assertEqual(t_opt_from_panel.shape, (4, 4))
        self.assertAlmostEqual(np.linalg.det(t_opt_from_panel[:3, :3]), 1.0,
                               places=5)
        self.assertFalse(math.isnan(float(t_opt_from_panel[2, 3])))


if __name__ == "__main__":
    unittest.main()
