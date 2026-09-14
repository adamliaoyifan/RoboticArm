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
    fold_base_livox_correction_into_adapter,
    frustum_mask,
    icp_point_to_plane,
    invert_T,
    livox_pose_after_frozen_icp,
    mid360_mount_to_realize_base_livox,
    mount_from_adapter_livox,
    nn_rmse,
    replace_mid360_mount,
    rotation_deg,
    seat_cad_pocket_under_fixed_livox,
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

    def test_xacro_mount_roundtrips_optical(self):
        T = T_adapter_livox_from_xacro()
        xyz, rpy = mount_from_adapter_livox(T)
        T_round = T_xyz_rpy(xyz, rpy).dot(T_handbook_optical())
        np.testing.assert_allclose(T_round, T, atol=1e-8)

    def test_mount_realizes_requested_base_livox(self):
        T_base_adp = np.eye(4)
        T_base_adp[:3, 3] = [0.48, -0.20, 0.31]
        T_want = T_base_adp.dot(
            T_xyz_rpy(CAD_MOUNT_XYZ, CAD_MOUNT_RPY)).dot(T_handbook_optical())
        xyz, rpy = mid360_mount_to_realize_base_livox(
            T_want, T_base_adp, rpy_seed=CAD_MOUNT_RPY)
        T_got = T_base_adp.dot(T_xyz_rpy(xyz, rpy)).dot(T_handbook_optical())
        np.testing.assert_allclose(T_got, T_want, atol=1e-8)
        np.testing.assert_allclose(xyz, CAD_MOUNT_XYZ, atol=1e-8)

    def test_live_pair_realizes_icp_livox_keeps_camera(self):
        import yaml
        from luggage_description.handeye_layer3 import parse_xacro_xyz_rpy
        frozen_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "..", "luggage_perception",
            "config", "livox_d555_table_icp_frozen.yaml"))
        with open(frozen_path, encoding="utf-8") as handle:
            frozen = yaml.safe_load(handle)
        T_corr = np.asarray(
            frozen["accepted_tf"]["T_livox_to_camera"], dtype=np.float64)
        T_adp_before = np.asarray(
            frozen["urdf_fold"]["T_base_adapter_before"], dtype=np.float64)
        backup = os.path.join(os.path.dirname(ORIGIN), "backups",
                              "20260914_livox_pocket_adapter")
        ax0, ar0 = parse_xacro_xyz_rpy(
            os.path.join(backup, "eef_mount_adapter_origin.xacro"),
            "adapter_mount_xyz", "adapter_mount_rpy")
        mx0, mr0 = parse_xacro_xyz_rpy(
            os.path.join(backup, "mid360_origin.xacro"),
            "mid360_mount_xyz", "mid360_mount_rpy")
        cx0, cr0 = parse_xacro_xyz_rpy(
            os.path.join(backup, "camera_mount_origin.xacro"),
            "cam_mount_xyz", "cam_mount_rpy")
        ax1, ar1 = parse_xacro_xyz_rpy(
            os.path.join(os.path.dirname(ORIGIN), "eef_mount_adapter_origin.xacro"),
            "adapter_mount_xyz", "adapter_mount_rpy")
        mx1, mr1 = parse_xacro_xyz_rpy(
            ORIGIN, "mid360_mount_xyz", "mid360_mount_rpy")
        cx1, cr1 = parse_xacro_xyz_rpy(
            os.path.join(os.path.dirname(ORIGIN), "camera_mount_origin.xacro"),
            "cam_mount_xyz", "cam_mount_rpy")
        T_panel_old = T_xyz_rpy(ax0, ar0)
        T_panel_new = T_xyz_rpy(ax1, ar1)
        T_al_old = T_xyz_rpy(mx0, mr0).dot(T_handbook_optical())
        T_al_new = T_xyz_rpy(mx1, mr1).dot(T_handbook_optical())
        T_cam_old = T_xyz_rpy(cx0, cr0)
        T_cam_new = T_xyz_rpy(cx1, cr1)
        T_adp_after = T_adp_before.dot(invert_T(T_panel_old)).dot(T_panel_new)
        T_liv_old = T_adp_before.dot(T_al_old)
        T_liv_new = T_adp_after.dot(T_al_new)
        T_want = livox_pose_after_frozen_icp(T_corr, T_liv_old)
        np.testing.assert_allclose(T_liv_new, T_want, atol=1e-6)
        np.testing.assert_allclose(
            T_panel_new.dot(T_cam_new), T_panel_old.dot(T_cam_old), atol=1e-6)
        np.testing.assert_allclose(mx1, CAD_MOUNT_XYZ, atol=1e-9)

    def test_locked_eef_livox_tree_matches_live_xacro(self):
        import yaml
        from luggage_description.handeye_layer3 import parse_xacro_xyz_rpy
        from luggage_description.livox_d555_align import locked_eef_livox_tree
        frozen_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "..", "luggage_perception",
            "config", "livox_d555_table_icp_frozen.yaml"))
        with open(frozen_path, encoding="utf-8") as handle:
            frozen = yaml.safe_load(handle)
        self.assertTrue(frozen.get("locked"))
        cfg = os.path.dirname(ORIGIN)
        backup = os.path.join(cfg, "backups", "20260914_livox_pocket_adapter")
        tree = locked_eef_livox_tree(frozen, config_dir=cfg)
        ax, ar = parse_xacro_xyz_rpy(
            os.path.join(cfg, "eef_mount_adapter_origin.xacro"),
            "adapter_mount_xyz", "adapter_mount_rpy")
        mx, mr = parse_xacro_xyz_rpy(
            ORIGIN, "mid360_mount_xyz", "mid360_mount_rpy")
        cx, cr = parse_xacro_xyz_rpy(
            os.path.join(cfg, "camera_mount_origin.xacro"),
            "cam_mount_xyz", "cam_mount_rpy")
        fx, fr = parse_xacro_xyz_rpy(
            os.path.join(cfg, "suction_flange_origin.xacro"),
            "suction_flange_xyz", "suction_flange_rpy")
        ax0, ar0 = parse_xacro_xyz_rpy(
            os.path.join(backup, "eef_mount_adapter_origin.xacro"),
            "adapter_mount_xyz", "adapter_mount_rpy")
        cx0, cr0 = parse_xacro_xyz_rpy(
            os.path.join(backup, "camera_mount_origin.xacro"),
            "cam_mount_xyz", "cam_mount_rpy")
        np.testing.assert_allclose(ax, tree["adapter_xyz"], atol=5e-7)
        np.testing.assert_allclose(ar, tree["adapter_rpy"], atol=5e-8)
        np.testing.assert_allclose(cx, tree["camera_xyz"], atol=5e-7)
        np.testing.assert_allclose(cr, tree["camera_rpy"], atol=5e-8)
        np.testing.assert_allclose(mx, CAD_MOUNT_XYZ, atol=1e-9)
        np.testing.assert_allclose(mr, CAD_MOUNT_RPY, atol=1e-8)
        np.testing.assert_allclose(fx, tree["flange_xyz"], atol=1e-9)
        np.testing.assert_allclose(fr, tree["flange_rpy"], atol=1e-8)
        T_panel_liv_live = T_xyz_rpy(ax, ar).dot(
            T_xyz_rpy(mx, mr)).dot(T_handbook_optical())
        np.testing.assert_allclose(
            T_panel_liv_live, tree["T_panel_livox"], atol=1e-6)
        np.testing.assert_allclose(
            T_xyz_rpy(ax, ar).dot(T_xyz_rpy(cx, cr)),
            T_xyz_rpy(ax0, ar0).dot(T_xyz_rpy(cx0, cr0)),
            atol=1e-6)
        T_lock = np.asarray(
            frozen["accepted_tf"]["T_elfin_base_link_from_livox"],
            dtype=np.float64)
        np.testing.assert_allclose(tree["T_base_livox"], T_lock, atol=1e-9)
        joints = frozen["joints"]
        np.testing.assert_allclose(
            ax, joints["suction_panel_to_eef_mount_adapter"]["xyz"], atol=5e-7)
        np.testing.assert_allclose(
            cx, joints["eef_mount_adapter_to_camera_link"]["xyz"], atol=5e-7)
        np.testing.assert_allclose(
            tree["base_livox_xyz"],
            frozen["accepted_tf"]["elfin_base_link_from_livox_xyz_m"],
            atol=5e-7)
        np.testing.assert_allclose(
            joints["eef_mount_adapter_to_mid360_mount_frame"]["xyz"],
            CAD_MOUNT_XYZ, atol=1e-9)

    def test_seat_keeps_livox_and_camera_vs_panel(self):
        T_panel = T_xyz_rpy(
            (0.0168, -0.0156, 0.0702),
            (1.57707939, -3.948e-05, -3.13530959),
        )
        T_al_cad = T_xyz_rpy(CAD_MOUNT_XYZ, CAD_MOUNT_RPY).dot(
            T_handbook_optical())
        T_al_cur = T_xyz_rpy(
            (0.01, 0.13, 0.015),
            (0.16221799, 1.63740799, 1.77427038),
        ).dot(T_handbook_optical())
        T_cam = T_xyz_rpy(
            (-0.028833, 0.10791, -0.07712),
            (-0.01784105, 1.39066481, 1.56765511),
        )
        out = seat_cad_pocket_under_fixed_livox(
            T_panel, T_al_cur, T_al_cad, T_cam)
        np.testing.assert_allclose(
            out["T_panel_adapter_new"].dot(T_al_cad),
            T_panel.dot(T_al_cur),
            atol=1e-9,
        )
        np.testing.assert_allclose(
            out["T_panel_adapter_new"].dot(out["T_adapter_camera_new"]),
            T_panel.dot(T_cam),
            atol=1e-9,
        )

    def test_fold_keeps_camera_and_seats_cad_livox(self):
        T_base_adp = np.eye(4)
        T_base_adp[:3, 3] = [0.48, -0.20, 0.31]
        T_panel = T_xyz_rpy(
            (0.0168, -0.0156, 0.0702),
            (1.57707939, -3.948e-05, -3.13530959),
        )
        T_al_cad = T_xyz_rpy(CAD_MOUNT_XYZ, CAD_MOUNT_RPY).dot(
            T_handbook_optical())
        T_al_cur = T_xyz_rpy(
            (0.01, 0.13, 0.015),
            (0.16221799, 1.63740799, 1.77427038),
        ).dot(T_handbook_optical())
        T_cam = T_xyz_rpy(
            (-0.028833, 0.10791, -0.07712),
            (-0.01784105, 1.39066481, 1.56765511),
        )
        T_corr = np.eye(4)
        T_corr[:3, :3] = rpy_to_R(0.0, 0.0, 0.04244687770070764)
        T_corr[:3, 3] = [0.013292688847106516, 0.028229033332821382, -0.00061779]
        out = fold_base_livox_correction_into_adapter(
            T_corr, T_base_adp, T_panel, T_al_cur, T_al_cad, T_cam)
        np.testing.assert_allclose(
            out["T_panel_adapter_new"].dot(out["T_adapter_camera_new"]),
            T_panel.dot(T_cam),
            atol=1e-9,
        )
        np.testing.assert_allclose(
            out["T_panel_adapter_new"].dot(T_al_cad),
            T_panel.dot(out["T_adapter_livox_aligned"]),
            atol=1e-9,
        )

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
