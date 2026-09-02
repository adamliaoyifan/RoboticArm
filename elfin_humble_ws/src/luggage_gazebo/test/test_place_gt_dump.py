#!/usr/bin/env python3
"""Unit tests for place GT occupancy image and interior cloud."""

from __future__ import division

import os
import tempfile
import unittest

from luggage_description.scene_tf_config_utils import (
    load_scene_tf_config,
    origin_in_world,
    point_inside_container_inner_hull_container,
    resolve_scene_tf_config_path,
    xyz_world_to_base_link,
    yaw_world_to_base_link,
)
from luggage_gazebo.place_gt_dump import (
    build_place_gt,
    build_pack_layout,
    container_aperture_wireframe_world,
    container_wireframe_world,
    sample_container_inner_cloud,
    write_pack_layout_dump,
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

    def test_pack_layout_marks_all_boxes_and_free_space(self):
        boxes = [
            {
                "seq": 0, "catalog_id": "carryon",
                "size_wdh": [0.55, 0.40, 0.25],
                "pose_world": {
                    "position": [1.2, -0.4, 0.655],
                    "rpy": [0.0, 0.0, 0.0],
                },
            },
            {
                "seq": 1, "catalog_id": "carryon",
                "size_wdh": [0.55, 0.40, 0.25],
                "pose_world": {
                    "position": [1.2, 0.4, 0.655],
                    "rpy": [0.0, 0.0, 0.0],
                },
            },
        ]
        one = build_pack_layout(self.cfg, boxes[:1], resolution=0.05, spacing=0.12)
        two = build_pack_layout(self.cfg, boxes, resolution=0.05, spacing=0.12)
        self.assertEqual(two["stats"]["committed_box_count"], 2)
        self.assertGreater(
            two["stats"]["occupied_count"], one["stats"]["occupied_count"])
        self.assertGreater(
            two["stats"]["n_box_points"], one["stats"]["n_box_points"])
        self.assertGreater(two["stats"]["n_free_voxels"], 100)
        self.assertEqual(len(two["boxes"]), 2)
        self.assertEqual(
            two["stats"]["n_gt_points"],
            two["stats"]["n_wall_points"] + two["stats"]["n_box_points"])

    def test_write_pack_layout_dump_files(self):
        dest = tempfile.mkdtemp()
        try:
            meta = write_pack_layout_dump(
                dest, self.cfg,
                [{
                    "seq": 0, "catalog_id": "carryon",
                    "size_wdh": [0.55, 0.40, 0.25],
                    "pose_world": {
                        "position": [1.5, 0.0, 0.655],
                        "rpy": [0.0, 0.0, 0.0],
                    },
                }],
                termination="BIN_FULL")
            for name in (
                "boxes.json", "occupancy_gt.json", "occupancy_gt.png",
                "container_and_boxes.ply", "interior_free.ply", "layout.ply",
                "layout.html", "meta.json",
            ):
                self.assertTrue(os.path.isfile(os.path.join(dest, name)), name)
            self.assertEqual(meta["termination"], "BIN_FULL")
            self.assertEqual(meta["n_boxes"], 1)
            with open(os.path.join(dest, "layout.html"), encoding="utf-8") as handle:
                html = handle.read()
            self.assertIn("mesh3d", html)
            self.assertIn("free interior", html)
            self.assertIn("container hull", html)
            self.assertIn("aperture", html)
        finally:
            import shutil
            shutil.rmtree(dest)

    def test_inner_cloud_is_heptahedron_with_aperture_hole(self):
        origin, _ = origin_in_world(self.cfg)
        points, colors = sample_container_inner_cloud(self.cfg, spacing=0.08)
        locals_ = [
            [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]]
            for p in points
        ]
        floor_pts = [p for p in locals_ if abs(p[2] - 0.53) < 0.02]
        self.assertTrue(floor_pts)
        self.assertLess(max(p[1] for p in floor_pts), 0.62)
        chamfer_pts = [
            p for p, c in zip(locals_, colors)
            if c == (196, 148, 88)
        ]
        self.assertGreater(len(chamfer_pts), 10)
        hx = 0.5 * 1.49
        hole = [
            p for p in locals_
            if abs(p[0] + hx) < 0.02
            and -0.70 < p[1] < 0.20
            and 0.80 < p[2] < 1.70
        ]
        self.assertEqual(hole, [])
        edges = container_wireframe_world(self.cfg)
        self.assertGreaterEqual(len(edges), 15)
        self.assertEqual(len(container_aperture_wireframe_world(self.cfg)), 4)

    def test_free_voxels_stay_inside_hull(self):
        layout = build_pack_layout(self.cfg, [], resolution=0.05, spacing=0.12)
        origin, _ = origin_in_world(self.cfg)
        self.assertGreater(layout["stats"]["inactive_count"], 0)
        self.assertEqual(layout["stats"]["unknown_count"], 0)
        for world in layout["free_xyz"]:
            local = [
                world[0] - origin[0], world[1] - origin[1], world[2] - origin[2],
            ]
            self.assertTrue(
                point_inside_container_inner_hull_container(local, self.cfg),
                local)


if __name__ == "__main__":
    unittest.main()
