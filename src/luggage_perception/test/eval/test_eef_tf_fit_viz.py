#!/usr/bin/env python3
"""Tests for the eval-only EEF TF-fit visualizer."""
import json
import os
import tempfile
import unittest

import numpy as np

from luggage_perception.eval.eef_tf_fit_viz import (
    CAM_BOX_ORIGIN,
    CAM_BOX_SIZE,
    CAM_LENS_ORIGIN,
    CAM_LENS_SIZE,
    box_mesh,
    dump_eef_tf_fit,
    frame_poses_from_tree,
    hops_to_T,
    projection_recipe,
    transform_mesh,
)
from luggage_perception.eval.gate4_dump import write_ply_xyz
from luggage_perception.eval.lidar_camera_calib_export import write_ply_xyzrgb


def _hop(parent, child, dx):
    mat = np.eye(4, dtype=np.float64)
    mat[0, 3] = dx
    return {
        "parent": parent,
        "child": child,
        "xyz": [dx, 0.0, 0.0],
        "rpy": [0.0, 0.0, 0.0],
        "matrix": mat.tolist(),
    }


def _tiny_tree():
    arm = [
        _hop("elfin_base_link", "elfin_end_link", 0.4),
    ]
    eef = [
        _hop("elfin_end_link", "suction_panel", 0.01),
        _hop("suction_panel", "eef_mount_adapter", 0.03),
        _hop("eef_mount_adapter", "mid360_mount_frame", 0.02),
        _hop("mid360_mount_frame", "livox_frame", 0.047),
    ]
    cam = [
        _hop("elfin_end_link", "suction_panel", 0.01),
        _hop("suction_panel", "eef_mount_adapter", 0.03),
        _hop("eef_mount_adapter", "camera_link", 0.05),
        _hop("camera_link", "d555_link", 0.0),
        _hop("d555_link", "d555_color_frame", 0.001),
        _hop("d555_color_frame", "d555_color_optical_frame", 0.0),
    ]
    T_liv = hops_to_T(arm + eef)
    T_cam = hops_to_T(arm + cam)
    return {
        "stamp_ns": 1,
        "eof_frame": "elfin_end_link",
        "panel_frame": "suction_panel",
        "mounter_frame": "eef_mount_adapter",
        "base_frame": "elfin_base_link",
        "optical_frame": "d555_color_optical_frame",
        "arm_base_to_eof": {"ok": True, "hops": arm, "T_first_from_last": hops_to_T(arm).tolist()},
        "eof_to_livox": {"ok": True, "hops": eef, "T_first_from_last": hops_to_T(eef).tolist()},
        "eof_to_optical": {"ok": True, "hops": cam, "T_first_from_last": hops_to_T(cam).tolist()},
        "base_to_livox": {
            "ok": True, "hops": arm + eef,
            "T_first_from_last": T_liv.tolist(),
        },
        "base_to_optical": {
            "ok": True, "hops": arm + cam,
            "T_first_from_last": T_cam.tolist(),
        },
        "T_elfin_base_link_from_livox_frame": T_liv.tolist(),
        "T_elfin_base_link_from_optical": T_cam.tolist(),
    }


class TestEefTfFitViz(unittest.TestCase):
    def test_hops_to_T_sums_translations(self):
        T = hops_to_T([_hop("a", "b", 0.1), _hop("b", "c", 0.2)])
        np.testing.assert_allclose(T[:3, 3], [0.3, 0.0, 0.0])

    def test_frame_poses_match_composed_lock(self):
        tree = _tiny_tree()
        poses = frame_poses_from_tree(tree)
        np.testing.assert_allclose(
            poses["livox_frame"],
            np.asarray(tree["T_elfin_base_link_from_livox_frame"]))
        np.testing.assert_allclose(
            poses["d555_color_optical_frame"],
            np.asarray(tree["T_elfin_base_link_from_optical"]))
        self.assertAlmostEqual(float(poses["livox_frame"][0, 3]), 0.507)

    def test_projection_recipe_lists_sensor_hops(self):
        recipe = projection_recipe(_tiny_tree())
        self.assertEqual(recipe["common_frame"], "elfin_base_link")
        self.assertIn("elfin_base_link -> elfin_end_link", recipe["livox"]["hops"])
        self.assertIn("mid360_mount_frame -> livox_frame", recipe["livox"]["hops"])
        self.assertIn(
            "d555_color_frame -> d555_color_optical_frame",
            recipe["camera_depth"]["hops"])
        self.assertIn("lookup_via_eof_mounter", recipe["lookup"])

    def test_camera_box_is_d555_datasheet_envelope(self):
        self.assertEqual(CAM_BOX_SIZE, (0.048, 0.167, 0.042))
        self.assertEqual(CAM_BOX_ORIGIN, (-0.024, -0.0475, 0.0))
        self.assertEqual(CAM_LENS_SIZE, (0.001, 0.167, 0.038))
        self.assertEqual(CAM_LENS_ORIGIN, (0.0005, -0.0475, 0.0))

    def test_box_mesh_and_transform_origin(self):
        verts, faces = box_mesh((0.2, 0.4, 0.6), origin=(1.0, 2.0, 3.0))
        self.assertEqual(len(verts), 8)
        self.assertEqual(len(faces), 12)
        np.testing.assert_allclose(verts.mean(axis=0), [1.0, 2.0, 3.0])
        T = np.eye(4)
        T[:3, 3] = [0.5, 0.0, 0.0]
        out, _f = transform_mesh(T, [[0.0, 0.0, 0.0]], [[0, 0, 0]], scale=1.0)
        np.testing.assert_allclose(out[0], [0.5, 0.0, 0.0])

    def test_dump_writes_html_and_recipe(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        dump = os.path.join(tmp.name, "dump")
        fused = os.path.join(dump, "fused")
        os.makedirs(fused)
        tree = _tiny_tree()
        with open(os.path.join(dump, "tf_tree.json"), "w", encoding="utf-8") as handle:
            json.dump(tree, handle)
        cam = np.array([[0.5, -0.3, 0.2], [0.51, -0.31, 0.21]])
        lid = np.array([[0.52, -0.32, 0.22]])
        write_ply_xyzrgb(
            os.path.join(fused, "camera_depth_in_elfin_base_link.ply"),
            cam, np.array([[10, 20, 30], [40, 50, 60]], dtype=np.uint8))
        write_ply_xyz(os.path.join(fused, "livox_in_elfin_base_link.ply"), lid)
        out = os.path.join(tmp.name, "viz")
        summary = dump_eef_tf_fit(dump, out_dir=out, cloud_cap=10)
        self.assertTrue(os.path.isfile(os.path.join(out, "tf_fit.html")))
        self.assertTrue(os.path.isfile(os.path.join(out, "projection.md")))
        self.assertTrue(os.path.isfile(os.path.join(out, "projection.json")))
        self.assertGreaterEqual(summary["n_meshes"], 2)
        self.assertEqual(summary["n_camera"], 2)
        self.assertEqual(summary["n_livox"], 1)
        with open(os.path.join(out, "tf_fit.html"), encoding="utf-8") as handle:
            html = handle.read()
        self.assertIn("elfin_base_link", html)
        self.assertIn("lookup_via_eof_mounter", html)
        with open(os.path.join(out, "projection.json"), encoding="utf-8") as handle:
            recipe = json.load(handle)
        self.assertTrue(os.path.isfile(os.path.join(out, "eef_iso.png")))
        self.assertTrue(os.path.isfile(os.path.join(out, "eef_pocket.png")))
        self.assertTrue(os.path.isfile(os.path.join(out, "clouds_xy.png")))
        self.assertEqual(recipe["livox"]["sensor_frame"], "livox_frame")


if __name__ == "__main__":
    unittest.main()
