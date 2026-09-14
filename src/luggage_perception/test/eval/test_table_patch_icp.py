#!/usr/bin/env python3
"""Highest-table crop + ICP between synthetic D555 and Livox clouds."""
import os
import tempfile
import unittest

import numpy as np

from luggage_description.livox_d555_align import transform_points
from luggage_perception.eval.gate4_dump import write_ply_xyz
from luggage_perception.eval.lidar_camera_calib_export import write_ply_xyzrgb
from luggage_perception.eval.table_patch_icp import (
    FROZEN_REVISION,
    LABEL_BOTTOM_EDGE,
    LABEL_CORNER,
    LABEL_EDGE,
    LABEL_RIGHT_EDGE,
    bottom_right_table_corner,
    classify_table_features,
    dump_projected_table_features,
    dump_table_patch_icp,
    dump_xy_full_edges_mean_z,
    edge_mask,
    focus_anchor_corners,
    icp_balanced_corners_edges,
    livox_balanced_corner_edge_weights,
    highest_mean_height_patch,
    label_br_incident_edges,
    load_frozen_table_icp,
    reject_livox_outliers,
    run_table_patch_icp,
    run_xy_full_edges_mean_z_icp,
    visible_table_corner,
)


def _table_and_floor(rng, z_table=0.80, z_floor=0.00):
    xs = np.linspace(0.20, 0.70, 41)
    ys = np.linspace(-0.40, -0.20, 25)
    xx, yy = np.meshgrid(xs, ys)
    table = np.stack(
        [xx.ravel(), yy.ravel(), np.full(xx.size, z_table)], axis=1)
    table = table + rng.normal(0.0, 0.001, table.shape)
    floor = np.column_stack([
        rng.uniform(-0.4, 1.0, 800),
        rng.uniform(-1.0, 0.2, 800),
        rng.normal(z_floor, 0.003, 800),
    ])
    return table, floor


class TestTablePatchIcp(unittest.TestCase):
    def test_highest_patch_is_table_not_floor(self):
        rng = np.random.RandomState(0)
        table, floor = _table_and_floor(rng)
        cam = np.vstack([table, floor])
        patch = highest_mean_height_patch(cam, cell=0.05, z_band=0.03, min_cell_n=8)
        self.assertTrue(patch["ok"])
        self.assertGreater(patch["patch_mean_z"], 0.70)
        self.assertLess(abs(patch["patch_mean_z"] - 0.80), 0.03)

    def test_icp_recovers_table_offset(self):
        rng = np.random.RandomState(1)
        table, floor = _table_and_floor(rng)
        cam = np.vstack([table, floor])
        shift = np.array([0.018, -0.012, 0.004])
        lid_table = table + shift + rng.normal(0.0, 0.002, table.shape)
        lid = np.vstack([lid_table, floor + np.array([0.3, 0.0, 0.0])])
        report = run_table_patch_icp(
            cam, lid, cell=0.05, z_band=0.03, min_cell_n=8,
            xy_margin=0.10, z_margin=0.05, near_radius=0.08,
            inlier_radius=0.04, hull_band=0.03,
            icp_max_dist=0.12, icp_voxel=0.02)
        self.assertTrue(report["ok"])
        recovered = transform_points(
            np.asarray(report["T_livox_to_camera"]), lid_table)
        delta = recovered.mean(axis=0) - table.mean(axis=0)
        self.assertLess(float(np.linalg.norm(delta)), 0.02)
        self.assertLess(report["after"]["nn_rmse_m"], 0.015)
        self.assertLess(report["icp_translation_m"], 0.04)

    def test_classify_finds_rectangle_corners(self):
        xs = np.linspace(0.0, 0.50, 26)
        ys = np.linspace(0.0, 0.30, 16)
        xx, yy = np.meshgrid(xs, ys)
        table = np.stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)], axis=1)
        labels, corners = classify_table_features(table, cell=0.02, min_cell_n=2)
        self.assertGreaterEqual(int((labels == LABEL_CORNER).sum()), 4)
        self.assertGreaterEqual(len(corners), 4)
        vis = visible_table_corner(table, labels)
        self.assertLess(float(vis[0]), 0.06)
        self.assertGreater(float(vis[1]), 0.24)
        br = bottom_right_table_corner(table, labels)
        self.assertGreater(float(br[0]), 0.44)
        self.assertLess(float(br[1]), 0.06)

    def test_focus_keeps_only_bottom_right_corner(self):
        xs = np.linspace(0.0, 0.50, 26)
        ys = np.linspace(0.0, 0.30, 16)
        xx, yy = np.meshgrid(xs, ys)
        table = np.stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)], axis=1)
        labels, _ = classify_table_features(table, cell=0.02, min_cell_n=2)
        br = bottom_right_table_corner(table, labels)
        focused = focus_anchor_corners(table, labels, br, radius=0.05)
        corners = table[focused == LABEL_CORNER]
        self.assertGreater(len(corners), 4)
        self.assertGreater(float(corners[:, 0].min()), 0.40)
        self.assertLess(float(corners[:, 1].max()), 0.10)
        self.assertGreater(int((focused == LABEL_EDGE).sum()), 0)

    def test_label_br_bottom_and_right_edges(self):
        xs = np.linspace(0.0, 0.50, 26)
        ys = np.linspace(0.0, 0.30, 16)
        xx, yy = np.meshgrid(xs, ys)
        table = np.stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)], axis=1)
        labels, _ = classify_table_features(table, cell=0.02, min_cell_n=2)
        br = bottom_right_table_corner(table, labels)
        labels = focus_anchor_corners(table, labels, br, radius=0.04)
        labels = label_br_incident_edges(table, labels, br, band=0.02)
        bottom = table[labels == LABEL_BOTTOM_EDGE]
        right = table[labels == LABEL_RIGHT_EDGE]
        self.assertGreater(len(bottom), 8)
        self.assertGreater(len(right), 8)
        self.assertLess(float(bottom[:, 1].mean()), 0.08)
        self.assertGreater(float(right[:, 0].mean()), 0.42)

    def test_corner_weight_keeps_visible_corner(self):
        rng = np.random.RandomState(3)
        table, floor = _table_and_floor(rng)
        cam = np.vstack([table, floor])
        shift = np.array([0.02, -0.01, 0.0])
        blob = np.column_stack([
            rng.uniform(0.05, 0.18, 80),
            rng.uniform(-0.42, -0.28, 80),
            np.full(80, 0.80) + rng.normal(0, 0.002, 80),
        ])
        lid = np.vstack([table + shift, blob, floor])
        report = run_table_patch_icp(
            cam, lid, cell=0.05, z_band=0.03, min_cell_n=8,
            xy_margin=0.12, z_margin=0.05, near_radius=0.10,
            inlier_radius=0.04, hull_band=0.03,
            interior_w=1.0, edge_w=12.0, corner_w=400.0)
        self.assertTrue(report["ok"])
        before = report["br_corner_before_m"]
        after = report["br_corner_after_m"]
        self.assertIsNotNone(before)
        self.assertIsNotNone(after)
        self.assertLessEqual(after, before + 0.005)
        self.assertGreater(float(report["after"]["edges"]["br_corner"]["livox_xyz"][0]), 0.55)
        self.assertEqual(report.get("icp_mode"), "br_corner_and_sides")
        self.assertGreater(int(report["n_livox_bottom_edge"]), 5)
        self.assertGreater(int(report["n_livox_right_edge"]), 5)

    def test_reject_far_left_livox_blob(self):
        rng = np.random.RandomState(4)
        table, _floor = _table_and_floor(rng)
        blob = np.column_stack([
            rng.uniform(0.02, 0.12, 120),
            rng.uniform(-0.42, -0.28, 120),
            np.full(120, 0.80) + rng.normal(0, 0.002, 120),
        ])
        keep, info = reject_livox_outliers(
            np.vstack([table, blob]), table, inlier_radius=0.04, hull_band=0.03)
        kept = np.vstack([table, blob])[keep]
        self.assertGreater(int(info["n_far_from_camera"]), 50)
        self.assertGreater(kept[:, 0].min(), 0.15)

    def test_reject_near_left_orphan_cluster(self):
        rng = np.random.RandomState(5)
        table, _floor = _table_and_floor(rng)
        # 4 cm left of table min-x=0.20. NN/hull gates are loose on purpose so
        # only the disconnected-component check should drop the island.
        blob = np.column_stack([
            rng.uniform(0.14, 0.16, 80),
            rng.uniform(-0.38, -0.32, 80),
            np.full(80, 0.80) + rng.normal(0, 0.002, 80),
        ])
        keep, info = reject_livox_outliers(
            np.vstack([table, blob]), table,
            inlier_radius=0.06, hull_band=0.06, component_cell=0.012)
        kept = np.vstack([table, blob])[keep]
        self.assertGreater(int(info["n_orphan_component"]), 40)
        self.assertGreater(kept[:, 0].min(), 0.19)

    def test_dump_writes_cropped_ply(self):
        rng = np.random.RandomState(2)
        table, floor = _table_and_floor(rng)
        cam = np.vstack([table, floor])
        rgb = np.full((len(cam), 3), 200, dtype=np.uint8)
        lid = table + np.array([0.01, 0.0, 0.0])
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        cam_path = os.path.join(tmp.name, "cam.ply")
        lid_path = os.path.join(tmp.name, "lid.ply")
        out = os.path.join(tmp.name, "out")
        write_ply_xyzrgb(cam_path, cam, rgb)
        write_ply_xyz(lid_path, lid)
        report = dump_table_patch_icp(
            cam_path, lid_path, out, cell=0.05, min_cell_n=8, near_radius=0.08)
        self.assertTrue(os.path.isfile(os.path.join(out, "table_camera.ply")))
        self.assertTrue(os.path.isfile(os.path.join(out, "table_livox_before.ply")))
        self.assertTrue(os.path.isfile(os.path.join(out, "table_camera_bottom_edge.ply")))
        self.assertTrue(os.path.isfile(os.path.join(out, "table_camera_right_edge.ply")))
        self.assertTrue(os.path.isfile(os.path.join(out, "table_icp.json")))
        self.assertTrue(report["ok"])
        self.assertEqual(report["frozen_revision"], FROZEN_REVISION)

    def test_dump_projected_writes_corners_and_edges_separately(self):
        rng = np.random.RandomState(6)
        table, floor = _table_and_floor(rng)
        cam = np.vstack([table, floor])
        rgb = np.full((len(cam), 3), 200, dtype=np.uint8)
        lid = table + np.array([0.008, -0.004, 0.0])
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        cam_path = os.path.join(tmp.name, "cam.ply")
        lid_path = os.path.join(tmp.name, "lid.ply")
        out = os.path.join(tmp.name, "features")
        write_ply_xyzrgb(cam_path, cam, rgb)
        write_ply_xyz(lid_path, lid)
        report = dump_projected_table_features(
            cam_path, lid_path, out, cell=0.05, min_cell_n=8, near_radius=0.08)
        self.assertTrue(report["ok"])
        self.assertFalse(report["applied_extra_icp"])
        self.assertTrue(os.path.isfile(os.path.join(out, "corners", "overlay.ply")))
        self.assertTrue(os.path.isfile(os.path.join(out, "corners", "overlay.html")))
        self.assertTrue(os.path.isfile(os.path.join(out, "edges", "camera_bottom.ply")))
        self.assertTrue(os.path.isfile(os.path.join(out, "edges", "camera_right.ply")))
        self.assertTrue(os.path.isfile(os.path.join(out, "edges", "livox_bottom.ply")))
        self.assertTrue(os.path.isfile(os.path.join(out, "edges", "livox_right.ply")))
        self.assertTrue(os.path.isfile(os.path.join(out, "edges", "overlay.html")))
        self.assertFalse(os.path.isfile(os.path.join(out, "overlay_after_icp.ply")))
        self.assertGreater(int(report["n_camera_corner"]), 0)
        self.assertGreater(int(report["n_livox_corner"]), 0)
        self.assertGreater(int(report["n_camera_bottom_edge"]), 0)
        self.assertGreater(int(report["n_camera_right_edge"]), 0)
        self.assertIsNotNone(report["br_corner_residual_m"])

    def test_edge_mask_includes_bottom_and_right(self):
        labels = np.array([0, LABEL_EDGE, LABEL_BOTTOM_EDGE, LABEL_RIGHT_EDGE, LABEL_CORNER])
        mask = edge_mask(labels)
        self.assertEqual(list(mask.astype(int)), [0, 1, 1, 1, 0])

    def test_xy_full_edges_mean_z_recovers_offset(self):
        rng = np.random.RandomState(7)
        table, floor = _table_and_floor(rng)
        cam = np.vstack([table, floor])
        yaw = 0.03
        c, s = np.cos(yaw), np.sin(yaw)
        mid = np.array([0.45, -0.30])
        xy = table[:, :2] - mid
        lid_table = table.copy()
        lid_table[:, 0] = c * xy[:, 0] - s * xy[:, 1] + mid[0] + 0.012
        lid_table[:, 1] = s * xy[:, 0] + c * xy[:, 1] + mid[1] - 0.008
        lid_table[:, 2] += 0.006
        lid = np.vstack([lid_table, floor])
        report = run_xy_full_edges_mean_z_icp(
            cam, lid, cell=0.05, min_cell_n=8, near_radius=0.10,
            inlier_radius=0.04, hull_band=0.03)
        self.assertTrue(report["ok"])
        self.assertEqual(
            report["icp_mode"], "xy_full_balanced_corners_edges_mean_z")
        T = np.asarray(report["T_livox_to_camera"])
        recovered = transform_points(T, lid_table)
        delta = recovered.mean(axis=0) - table.mean(axis=0)
        self.assertLess(float(np.linalg.norm(delta[:2])), 0.015)
        self.assertLess(abs(float(delta[2])), 0.005)
        self.assertLess(
            report["xy_rmse_all_after_m"], report["xy_rmse_all_before_m"])
        self.assertGreater(float(report["w_corner"]), float(report["w_edge"]))
        self.assertAlmostEqual(
            float(report["corner_mass"]), float(report["edge_mass"]), places=5)

    def test_dump_xy_full_edges_writes_after_ply(self):
        rng = np.random.RandomState(8)
        table, floor = _table_and_floor(rng)
        cam = np.vstack([table, floor])
        rgb = np.full((len(cam), 3), 200, dtype=np.uint8)
        lid = table + np.array([0.01, -0.006, 0.004])
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        cam_path = os.path.join(tmp.name, "cam.ply")
        lid_path = os.path.join(tmp.name, "lid.ply")
        out = os.path.join(tmp.name, "out")
        write_ply_xyzrgb(cam_path, cam, rgb)
        write_ply_xyz(lid_path, lid)
        report = dump_xy_full_edges_mean_z(
            cam_path, lid_path, out, cell=0.05, min_cell_n=8, near_radius=0.08)
        self.assertTrue(report["ok"])
        self.assertTrue(os.path.isfile(os.path.join(out, "table_livox_after_icp.ply")))
        self.assertTrue(os.path.isfile(os.path.join(out, "table_xy_after_icp.png")))
        self.assertTrue(os.path.isfile(os.path.join(out, "overlay_after_icp.ply")))
        self.assertTrue(os.path.isfile(os.path.join(out, "table_icp.json")))
        self.assertTrue(os.path.isfile(os.path.join(out, "corners", "xy.png")))
        self.assertTrue(os.path.isfile(os.path.join(out, "edges", "xy.png")))

    def test_livox_corner_weight_balances_edge_mass(self):
        labels = np.array(
            [LABEL_CORNER] * 4
            + [LABEL_BOTTOM_EDGE] * 12
            + [LABEL_EDGE] * 8)
        w, info = livox_balanced_corner_edge_weights(labels)
        self.assertEqual(info["n_livox_corner"], 4)
        self.assertEqual(info["n_livox_edge"], 20)
        self.assertAlmostEqual(info["w_corner"], 5.0)
        self.assertAlmostEqual(info["corner_mass"], info["edge_mass"])
        self.assertTrue(np.allclose(w[:4], 5.0))
        self.assertTrue(np.allclose(w[4:], 1.0))

    def test_balanced_icp_does_not_ignore_sparse_corners(self):
        cam_c = np.column_stack(
            [np.full(8, 0.50), np.full(8, -0.36), np.zeros(8)])
        cam_e = np.column_stack(
            [np.linspace(0.30, 0.48, 80), np.full(80, -0.36), np.zeros(80)])
        lid_c = cam_c + np.array([0.020, 0.0, 0.0])
        lid_e = cam_e.copy()
        cam = np.vstack([cam_c, cam_e])
        lid = np.vstack([lid_c, lid_e])
        labs = np.array([LABEL_CORNER] * 8 + [LABEL_BOTTOM_EDGE] * 80)
        T, stats = icp_balanced_corners_edges(
            lid, labs, cam, labs, voxel=0.0, min_pairs=4, max_dist=0.08,
            allow_yaw=False)
        self.assertAlmostEqual(float(stats["w_corner"]), 10.0, places=5)
        self.assertAlmostEqual(float(stats["corner_mass"]), float(stats["edge_mass"]))
        self.assertLess(float(T[0, 3]), -0.006)

    def test_frozen_yaml_is_accepted_revision(self):
        frozen = load_frozen_table_icp()
        self.assertEqual(frozen["revision"], FROZEN_REVISION)
        self.assertEqual(frozen["status"], "accepted")
        self.assertTrue(frozen["locked"])
        self.assertTrue(frozen["applied_to_urdf"])
        self.assertEqual(frozen["urdf_fold"]["mode"],
                         "cad_pocket_on_icp_livox")
        self.assertEqual(frozen["urdf_fold"]["restored_mid360_mount"],
                         "cad_square_pocket")
        self.assertEqual(len(frozen["urdf_fold"]["T_base_adapter_before"]), 4)
        icp = frozen["icp"]
        self.assertEqual(icp["mode"], "xy_full_balanced_corners_edges_mean_z")
        self.assertFalse(icp["corners_only"])
        self.assertAlmostEqual(float(icp["inlier_radius_m"]), 0.02)
        self.assertAlmostEqual(float(icp["hull_band_m"]), 0.01)
        self.assertAlmostEqual(float(icp["anchor_radius_m"]), 0.025)
        self.assertTrue(bool(icp.get("balance_corners_edges", True)))
        tf = frozen["accepted_tf"]
        self.assertAlmostEqual(float(tf["translation_m"]), 0.025317, places=5)
        self.assertAlmostEqual(float(tf["rotation_deg"]), 1.566311, places=5)
        self.assertEqual(len(tf["T_livox_to_camera"]), 4)
        self.assertEqual(len(tf["T_delta"]), 4)
        self.assertAlmostEqual(float(tf["w_corner"]), 10.817073, places=5)
        self.assertEqual(len(tf["T_elfin_base_link_from_livox"]), 4)
        self.assertAlmostEqual(
            float(tf["elfin_base_link_from_livox_xyz_m"][0]), 0.464195, places=6)
        joints = frozen["joints"]
        self.assertEqual(
            joints["eef_mount_adapter_to_mid360_mount_frame"]["source"],
            "cad_square_pocket")
        self.assertEqual(
            joints["suction_panel_to_eef_mount_adapter"]["source"],
            "solved_to_realize_T_icp")
        self.assertAlmostEqual(
            float(joints["mid360_mount_frame_to_livox_frame"]["xyz"][2]),
            0.047, places=6)


if __name__ == "__main__":
    unittest.main()
