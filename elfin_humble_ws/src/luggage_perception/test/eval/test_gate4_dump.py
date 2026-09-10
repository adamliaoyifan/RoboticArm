#!/usr/bin/env python3
"""Unit tests for Gate-4 failure dump helpers. No ROS."""

import json
import os
import tempfile
import unittest

import numpy as np

from luggage_perception.eval.gate4_dump import (
    cargo_summary,
    crop_workspace_xy,
    extract_top_ransac,
    parse_gz_pose_info,
    select_dump_stamps,
    trial_folder_name,
    trial_is_failure,
    write_index,
    write_ply_xyz,
    write_snapshot_dir,
    write_top_ransac_dir,
    write_xyz,
)


class TestTrialFailure(unittest.TestCase):
    def test_ok_trial(self):
        rec = {
            "spawn_ok": True, "n_settled": 80,
            "t_first_valid_sec": 0.3, "t_first_full3d_sec": 0.4,
        }
        settled = [{"top_surface_valid": True, "pca_reason": "ok"}] * 80
        self.assertFalse(trial_is_failure(rec, settled))

    def test_no_top(self):
        rec = {
            "spawn_ok": True, "n_settled": 189,
            "t_first_valid_sec": None, "t_first_full3d_sec": None,
        }
        settled = [{"top_surface_valid": False,
                    "pca_reason": "DETECT_TOP_UNOBSERVABLE"}]
        self.assertTrue(trial_is_failure(rec, settled))

    def test_spawn_fail(self):
        self.assertTrue(trial_is_failure({"spawn_ok": False, "n_settled": 0}))


class TestSelectStamps(unittest.TestCase):
    def test_first_mid_last(self):
        keys = [(0, i) for i in range(5)]
        picked = select_dump_stamps(keys, count=3)
        self.assertEqual(picked[0], (0, 0))
        self.assertEqual(picked[-1], (0, 4))
        self.assertEqual(len(picked), 3)

    def test_empty(self):
        self.assertEqual(select_dump_stamps([], count=3), [])


class TestClouds(unittest.TestCase):
    def test_ply_and_crop(self):
        points = np.array([
            [-1.0, 0.0, 1.1],
            [-2.0, 0.0, 1.1],
            [-1.0, 0.4, 1.2],
        ], dtype=np.float64)
        cropped = crop_workspace_xy(points, (-1.0, 0.0), (0.5, 0.5))
        self.assertEqual(len(cropped), 2)
        summary = cargo_summary(points, "camera_color_optical_frame")
        self.assertEqual(summary["n"], 3)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "cloud.ply")
            n = write_ply_xyz(path, points)
            self.assertEqual(n, 3)
            raw = open(path, "rb").read()
            self.assertTrue(raw.startswith(b"ply\n"))
            self.assertIn(b"format binary_little_endian 1.0", raw)
            self.assertIn(b"element vertex 3", raw)
            self.assertIn(b"element face 0", raw)
            xyz_path = os.path.join(tmp, "cloud.xyz")
            self.assertEqual(write_xyz(xyz_path, points), 3)
            self.assertEqual(len(open(xyz_path).read().splitlines()), 3)

    def test_extract_top_ransac_picks_higher_plane(self):
        rng = np.random.RandomState(0)
        platform = np.column_stack((
            rng.uniform(-1.2, -0.8, 400),
            rng.uniform(-0.2, 0.2, 400),
            np.full(400, 0.86),
        ))
        top = np.column_stack((
            rng.uniform(-1.15, -0.85, 120),
            rng.uniform(-0.12, 0.12, 120),
            np.full(120, 1.16),
        ))
        rec = extract_top_ransac(np.vstack((platform, top)))
        self.assertTrue(rec["ok"])
        self.assertGreater(rec["plane_z"], 1.10)
        self.assertGreater(rec["n_inliers"], 50)
        with tempfile.TemporaryDirectory() as tmp:
            write_top_ransac_dir(tmp, np.vstack((platform, top)))
            self.assertTrue(os.path.isfile(os.path.join(tmp, "top_inliers.xyz")))
            self.assertTrue(os.path.isfile(os.path.join(tmp, "top_ransac.json")))

    def test_snapshot_and_index(self):
        color = np.zeros((8, 8, 3), dtype=np.uint8)
        color[2:6, 2:6] = (255, 0, 0)
        depth = np.ones((8, 8), dtype=np.float32) * 1.2
        with tempfile.TemporaryDirectory() as tmp:
            snap = os.path.join(tmp, "late")
            write_snapshot_dir(
                snap,
                images={"color": color, "depth": (depth / 2.5 * 255).astype(np.uint8)},
                arrays={"depth": depth},
                extras={"pca_reason": "DETECT_TOP_UNOBSERVABLE"},
                clouds={"cargo_camera": {
                    "points": [[0.0, 0.0, 1.0]], "frame_id": "cam"}},
            )
            self.assertTrue(os.path.isfile(os.path.join(snap, "color.png")))
            self.assertTrue(os.path.isfile(os.path.join(snap, "cargo_camera.ply")))
            meta = json.loads(open(os.path.join(snap, "meta.json")).read())
            self.assertEqual(meta["pca_reason"], "DETECT_TOP_UNOBSERVABLE")
            recs = [{
                "trial": 5, "folder": "trial_05_fail_x", "failed": True,
                "box_id": "pickup_box_0006_large",
                "pca_reason": "DETECT_TOP_UNOBSERVABLE",
                "n_cargo_points": 16706,
                "t_first_valid_sec": None, "t_first_full3d_sec": None,
            }]
            write_index(tmp, recs)
            self.assertTrue(os.path.isfile(os.path.join(tmp, "INDEX.md")))

    def test_folder_name_fail(self):
        name = trial_folder_name(
            5,
            {"spawn_ok": True, "n_settled": 189,
             "t_first_valid_sec": None, "t_first_full3d_sec": None},
            "pickup_box_0006_large",
            [{"top_surface_valid": False,
              "pca_reason": "DETECT_TOP_UNOBSERVABLE"}],
        )
        self.assertTrue(name.startswith("trial_05_fail_"))
        self.assertIn("DETECT_TOP_UNOBSERVABLE", name)

    def test_folder_name_slow_full3d(self):
        name = trial_folder_name(
            3,
            {"spawn_ok": True, "n_settled": 101,
             "t_first_valid_sec": 0.3, "t_first_full3d_sec": 2.4},
            "pickup_box_0004_carryon",
            [{"top_surface_valid": True, "pca_reason": "ok"}],
        )
        self.assertIn("slow_full3d", name)


class TestGzPose(unittest.TestCase):
    def test_parse_named_pose(self):
        text = """
        pose {
          name: "pickup_box_0006_large"
          position { x: -1.1 y: -0.1 z: 1.01 }
          orientation { x: 0 y: 0 z: 0.23 w: 0.97 }
        }
        """
        poses = parse_gz_pose_info(text)
        pose = poses["pickup_box_0006_large"]
        self.assertAlmostEqual(pose["position"]["x"], -1.1)
        self.assertLess(pose["tilt_rad"], 0.05)


if __name__ == "__main__":
    unittest.main()
