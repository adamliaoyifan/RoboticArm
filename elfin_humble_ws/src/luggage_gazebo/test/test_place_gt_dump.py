#!/usr/bin/env python3
"""Unit tests for place GT occupancy image and interior cloud."""

from __future__ import division

import os
import tempfile
import unittest

from luggage_description.scene_tf_config_utils import (
    load_scene_tf_config,
    resolve_scene_tf_config_path,
    xyz_world_to_base_link,
    yaw_world_to_base_link,
)
from luggage_gazebo.place_gt_dump import (
    build_place_gt,
    write_ply_xyzrgb,
)


class TestPlaceGtDump(unittest.TestCase):

    def setUp(self):
        self.cfg = load_scene_tf_config(resolve_scene_tf_config_path())

    def test_occupancy_marks_center_box(self):
        world = [1.5, 0.0, 0.655]
        size = [0.55, 0.40, 0.25]
        base = xyz_world_to_base_link(self.cfg, world)
        yaw_b = yaw_world_to_base_link(self.cfg, 0.0)
        out = build_place_gt(
            self.cfg, base, size, yaw_b,
            box_center_world=world, box_rpy_world=[0.0, 0.0, 0.0],
            resolution=0.05, spacing=0.08)
        self.assertGreater(out["stats"]["occupied_count"], 0)
        self.assertGreater(out["stats"]["free_count"], out["stats"]["occupied_count"])
        self.assertEqual(out["stats"]["unknown_count"], 0)
        self.assertEqual(out["stats"]["committed_box_count"], 1)
        rows = out["occupancy_image"]
        self.assertGreater(len(rows), 1)
        self.assertEqual(len(rows[0][0]), 3)
        occupied_px = sum(1 for row in rows for pix in row if pix[0] > 80)
        self.assertGreater(occupied_px, 10)

    def test_gt_cloud_has_walls_and_box(self):
        empty = build_place_gt(
            self.cfg, None, None, 0.0, resolution=0.05, spacing=0.12)
        placed = build_place_gt(
            self.cfg,
            xyz_world_to_base_link(self.cfg, [1.5, 0.0, 0.655]),
            [0.55, 0.40, 0.25], 0.0,
            box_center_world=[1.5, 0.0, 0.655],
            box_rpy_world=[0.0, 0.0, 0.0],
            resolution=0.05, spacing=0.12)
        self.assertGreater(empty["stats"]["n_wall_points"], 100)
        self.assertEqual(empty["stats"]["n_box_points"], 0)
        self.assertEqual(empty["stats"]["occupied_count"], 0)
        self.assertGreater(
            placed["stats"]["n_box_points"], empty["stats"]["n_box_points"])
        self.assertEqual(
            placed["stats"]["n_gt_points"],
            placed["stats"]["n_wall_points"] + placed["stats"]["n_box_points"])

    def test_ply_roundtrip_header(self):
        pts = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]
        rgb = [(255, 0, 0), (0, 255, 0)]
        handle, path = tempfile.mkstemp(suffix=".ply")
        os.close(handle)
        try:
            n = write_ply_xyzrgb(path, pts, rgb)
            self.assertEqual(n, 2)
            with open(path, "rb") as fh:
                text = fh.read(200)
            self.assertIn(b"element vertex 2", text)
            self.assertIn(b"property uchar red", text)
        finally:
            os.remove(path)


if __name__ == "__main__":
    unittest.main()
