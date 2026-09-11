#!/usr/bin/env python3
import math
import os
import unittest

import numpy as np

from luggage_description.handeye_layer3 import T_xyz_rpy, rpy_to_R
from luggage_description.livox_d555_align import (
    CAD_MOUNT_RPY,
    CAD_MOUNT_XYZ,
    HANDBOOK_OPTICAL_XYZ,
    apply_cloud_correction,
    frustum_mask,
    icp_point_to_plane,
    invert_T,
    mount_from_adapter_livox,
    nn_rmse,
    replace_mid360_mount,
    rotation_deg,
    T_adapter_livox_from_xacro,
    T_handbook_optical,
    transform_points,
    translation_m,
    voxel_downsample,
)

ORIGIN = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "config", "mid360_origin.xacro")
)


class TestLivoxD555Align(unittest.TestCase):
    def test_handbook_optical_stays_47mm(self):
        T = T_handbook_optical()
        np.testing.assert_allclose(T[:3, 3], HANDBOOK_OPTICAL_XYZ, atol=1e-9)

    def test_xacro_cad_matches_constants(self):
        T = T_adapter_livox_from_xacro()
        T_cad = T_xyz_rpy(CAD_MOUNT_XYZ, CAD_MOUNT_RPY).dot(T_handbook_optical())
        # After a freeze this still must be mount*optical; CAD constants
        # describe the pad seed, not necessarily the live xacro.
        xyz, rpy = mount_from_adapter_livox(T)
        T_round = T_xyz_rpy(xyz, rpy).dot(T_handbook_optical())
        np.testing.assert_allclose(T_round, T, atol=1e-8)

    def test_frustum_keeps_forward_points(self):
        pts = np.array(
            [
                [0.0, 0.0, 1.0],
                [0.0, 0.0, 0.05],
                [2.0, 0.0, 1.0],
            ]
        )
        mask = frustum_mask(pts, 300.0, 300.0, 320.0, 180.0, 640, 360, 0.2, 4.0)
        self.assertTrue(mask[0])
        self.assertFalse(mask[1])
        self.assertFalse(mask[2])

    def test_icp_recovers_known_offset(self):
        rng = np.random.default_rng(0)
        xs = np.linspace(-0.6, 0.6, 25)
        ys = np.linspace(-0.4, 0.4, 21)
        xx, yy = np.meshgrid(xs, ys)
        floor = np.stack([xx.ravel(), yy.ravel(), np.full(xx.size, 1.2)], axis=1)
        zz, yy2 = np.meshgrid(np.linspace(0.8, 1.6, 18), ys)
        wall = np.stack([np.full(zz.size, 0.55), yy2.ravel(), zz.ravel()], axis=1)
        dst = np.vstack([floor, wall])
        T_true = np.eye(4)
        T_true[:3, :3] = rpy_to_R(0.025, -0.02, 0.04)
        T_true[:3, 3] = [0.035, -0.02, 0.015]
        src = transform_points(invert_T(T_true), dst)
        src = src + rng.normal(0.0, 0.002, src.shape)
        T_est, stats = icp_point_to_plane(
            src, dst, max_iter=20, max_dist=0.12, voxel=0.03
        )
        recovered = transform_points(T_est, src)
        rmse, pairs = nn_rmse(recovered, dst, max_dist=0.05)
        self.assertGreater(pairs, 200)
        self.assertLess(rmse, 0.01)
        self.assertLess(translation_m(invert_T(T_est).dot(T_true)), 0.01)
        self.assertLess(rotation_deg(invert_T(T_est).dot(T_true)), 0.8)
        self.assertGreater(stats["pairs"], 80)

    def test_apply_correction_updates_mount_only(self):
        T_old = T_xyz_rpy(CAD_MOUNT_XYZ, CAD_MOUNT_RPY).dot(T_handbook_optical())
        T_corr = np.eye(4)
        T_corr[:3, 3] = [0.01, 0.0, 0.0]
        T_new = apply_cloud_correction(T_old, T_corr)
        xyz, rpy = mount_from_adapter_livox(T_new)
        T_rebuild = T_xyz_rpy(xyz, rpy).dot(T_handbook_optical())
        np.testing.assert_allclose(T_rebuild, T_new, atol=1e-8)
        self.assertAlmostEqual(xyz[0] - CAD_MOUNT_XYZ[0], 0.01, places=6)

    def test_rpy_near_seed_stays_on_cad_branch(self):
        from luggage_description.livox_d555_align import rpy_near_seed

        rot = rpy_to_R(*CAD_MOUNT_RPY)
        rpy = rpy_near_seed(rot, CAD_MOUNT_RPY)
        np.testing.assert_allclose(rpy_to_R(*rpy), rot, atol=1e-8)
        self.assertLess(abs(rpy[1] - CAD_MOUNT_RPY[1]), 0.05)
        self.assertTrue(all(abs(v) <= math.pi + 1e-9 for v in rpy))

    def test_replace_mount_keeps_optical(self):
        text = open(ORIGIN, encoding="utf-8").read()
        out = replace_mid360_mount(
            text, [0.03, 0.11, 0.04], [0.1, 1.5, 1.6], "unit test note"
        )
        self.assertIn('name="livox_optical_xyz" value="0.000 0.000 0.047"', out)
        self.assertIn("0.030000 0.110000 0.040000", out)
        self.assertIn("1.50000000", out)

    def test_set_origin_height_slides_along_normal(self):
        from luggage_description.livox_d555_align import (
            fit_horizontal_plane,
            height_above_plane,
            set_origin_height,
        )

        rng = np.random.default_rng(4)
        floor = np.column_stack(
            [
                rng.uniform(-1.0, 1.0, 400),
                rng.uniform(-1.0, 1.0, 400),
                rng.normal(-0.87, 0.003, 400),
            ]
        )
        nrm, offset = fit_horizontal_plane(floor, rng=rng)
        T = np.eye(4)
        T[:3, 3] = [1.4, 0.0, -0.31]
        T_new, before = set_origin_height(T, nrm, offset, 0.58)
        self.assertAlmostEqual(before, height_above_plane(T[:3, 3], nrm, offset), places=9)
        self.assertAlmostEqual(
            height_above_plane(T_new[:3, 3], nrm, offset), 0.58, places=6
        )
        np.testing.assert_allclose(T_new[:3, :3], np.eye(3))

    def test_fit_horizontal_plane_prefers_pedestal_floor(self):
        from luggage_description.livox_d555_align import fit_horizontal_plane

        rng = np.random.default_rng(5)
        floor = np.column_stack(
            [
                rng.uniform(-2.0, 2.0, 250),
                rng.uniform(-2.0, 2.0, 250),
                rng.normal(-0.86, 0.004, 250),
            ]
        )
        table = np.column_stack(
            [
                rng.uniform(-0.4, 0.4, 400),
                rng.uniform(-0.4, 0.4, 400),
                rng.normal(-0.40, 0.004, 400),
            ]
        )
        nrm, offset = fit_horizontal_plane(
            np.vstack([floor, table]),
            rng=rng,
            expected_origin_height=0.86,
            height_tol=0.10,
        )
        self.assertGreater(nrm[2], 0.95)
        self.assertAlmostEqual(offset, -0.86, places=1)


if __name__ == "__main__":
    unittest.main()
