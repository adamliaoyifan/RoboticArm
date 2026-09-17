#!/usr/bin/env python3
"""Unit tests for Livox monitoring metrics. No ROS."""

import unittest

import numpy as np

from luggage_perception.eval import dsim_livox_metrics as livox


class TestLivoxMetrics(unittest.TestCase):
    def test_decode_xyz_and_intensity(self):
        n = 4
        step = 16
        buf = bytearray(n * step)
        xyz = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 2], [3, 4, 5]], np.float32)
        inten = np.array([1, 2, 3, 4], np.float32)
        for i in range(n):
            buf[i * step:i * step + 12] = xyz[i].tobytes()
            buf[i * step + 12:i * step + 16] = inten[i].tobytes()
        got = livox.xyz_from_structured(bytes(buf), step, n)
        np.testing.assert_allclose(got, xyz.astype(np.float64), atol=1e-6)
        got_i = livox.intensity_from_structured(bytes(buf), step, n, offset=12)
        np.testing.assert_allclose(got_i, inten.astype(np.float64), atol=1e-6)

    def test_finite_ratio_and_range(self):
        xyz = np.array([
            [1.0, 0.0, 0.0],
            [0.0, np.nan, 0.0],
            [0.0, 0.0, 4.0],
        ], dtype=np.float64)
        rec = livox.summarize_scan(xyz, n_raw=3, frame_id="livox_frame")
        self.assertEqual(rec["n_finite"], 2)
        self.assertAlmostEqual(rec["finite_ratio"], 2.0 / 3.0)
        self.assertEqual(rec["frame_id"], "livox_frame")
        self.assertFalse(rec["deskewed"])
        self.assertEqual(rec["configured_grid"]["h_samples"], 360)
        self.assertEqual(rec["configured_grid"]["v_samples"], 32)
        self.assertFalse(rec["intensity"]["present"])

    def test_raster_plane_nn_and_residuals(self):
        xs, ys = np.meshgrid(np.linspace(-1.0, 1.0, 40),
                             np.linspace(-0.5, 0.5, 16), indexing="xy")
        zs = np.full(xs.shape, 2.0)
        xyz = np.stack([xs, ys, zs], axis=-1).reshape(-1, 3)
        rec = livox.summarize_scan(xyz)
        self.assertGreater(rec["nn_spacing_m"]["mean"], 0.02)
        self.assertLess(rec["nn_spacing_m"]["mean"], 0.20)
        self.assertGreater(rec["dominant_plane"]["inlier_ratio"], 0.8)
        self.assertLess(rec["dominant_plane"]["residual_mean"], 0.01)

    def test_window_does_not_join_rgb(self):
        scans = [livox.summarize_scan(np.array([[1.0, 0.0, 0.0], [1.1, 0.0, 0.0]]))]
        window = livox.summarize_window(scans, [1.0, 1.1])
        self.assertIn("not exact-paired", window["note"])
        self.assertFalse(window["deskewed"])
        self.assertEqual(window["n_scans"], 1)


if __name__ == "__main__":
    unittest.main()
