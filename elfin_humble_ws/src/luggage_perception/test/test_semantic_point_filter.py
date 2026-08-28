#!/usr/bin/env python3
"""Unit tests for semantic_point_filter (no roscore required).

Verifies the 3D → RGB projection against a known-good setup using the
project's own realsense_d435.yaml intrinsics, plus synthetic point/mask
fixtures.
"""

import math
import os
import sys
import unittest

import numpy as np

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_perception.semantic_point_filter import (  # noqa: E402
    CameraIntrinsics,
    DepthToColorExtrinsics,
    SemanticPointFilter,
    _project_to_color,
)


REALSENSE_CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "..", "luggage_description", "config", "realsense_d435.yaml",
)


def _load_realsense_intrinsics():
    import yaml
    with open(REALSENSE_CONFIG, "r") as handle:
        data = yaml.safe_load(handle)
    cam = data["camera"]
    color = CameraIntrinsics.from_dict(cam["color"])
    depth = CameraIntrinsics.from_dict(cam["depth"])
    extr = DepthToColorExtrinsics.from_dict(
        cam["extrinsics"]["depth_to_color"]
    )
    return color, depth, extr


class TestProjectionKnownPoint(unittest.TestCase):
    """A point at the center of the depth optical frame should project to
    near the color principal point (after applying depth→color extrinsics)."""

    def test_center_point_projects_near_principal_point(self):
        color, _depth, extr = _load_realsense_intrinsics()
        # RealSense D435 has a 15mm baseline along color-Y; the extrinsics
        # in realsense_d435.yaml are identity rotation with translation
        # [0, 0.015, 0]. A point on the depth +Z axis at z=1.0 maps to
        # (0, 0.015, 1.0) in color frame, which projects to
        #   u = fx * 0 + cx = cx
        #   v = fy * 0.015 + cy
        pts = np.array([[0.0, 0.0, 1.0]])
        uv, _z = _project_to_color(pts, extr.rotation, extr.translation, color)
        self.assertEqual(uv.shape, (1, 2))
        expected_u = int(round(color.cx))
        expected_v = int(round(color.fy * extr.translation[1] + color.cy))
        self.assertEqual(uv[0, 0], expected_u)
        self.assertEqual(uv[0, 1], expected_v)

    def test_behind_camera_marked_invalid(self):
        color, _depth, extr = _load_realsense_intrinsics()
        pts = np.array([[0.0, 0.0, -1.0]])
        uv, _z = _project_to_color(pts, extr.rotation, extr.translation, color)
        self.assertEqual(uv[0, 0], -1)
        self.assertEqual(uv[0, 1], -1)


class TestFilterRouting(unittest.TestCase):
    """Synthetic 2x2 mask + a single point should route to the right stream."""

    def setUp(self):
        self.color, self.depth, extr = _load_realsense_intrinsics()
        # A point on the depth optical Z-axis at z=1.0 maps, through the
        # depth→color extrinsics (identity R, t=[0, 0.015, 0]), to
        # (0, 0.015, 1.0) in color frame. That projects to
        #   u = fx * 0 / 1 + cx
        #   v = fy * 0.015 / 1 + cy
        # so the projection is below the principal point by the 15mm baseline.
        self.point = (0.0, 0.0, 1.0)
        uv, _ = _project_to_color(
            np.array([self.point]), extr.rotation, extr.translation, self.color
        )
        expected_u = int(round(self.color.cx))
        expected_v = int(round(self.color.fy * extr.translation[1] + self.color.cy))
        self.assertEqual(uv[0].tolist(), [expected_u, expected_v])
        self.projected_uv = (int(uv[0, 0]), int(uv[0, 1]))
        self.extr = extr

    def _make_filter(self, cargo_labels, obstacle_labels):
        f = SemanticPointFilter(
            color_intrinsics=self.color,
            depth_intrinsics=self.depth,
            depth_to_color=self.extr,
            cargo_labels=cargo_labels,
            obstacle_labels=obstacle_labels,
        )
        return f

    def test_cargo_label_routes_to_cargo_stream(self):
        f = self._make_filter(cargo_labels=[2], obstacle_labels=[2, 4])
        mask = np.zeros((480, 640), dtype=np.uint8)
        mask[self.projected_uv[1], self.projected_uv[0]] = 2  # cargo at projected px
        cargo, obstacle = f.filter_points([self.point], mask)
        self.assertEqual(len(cargo), 1)
        self.assertEqual(len(obstacle), 1)
        # filter_points returns lists of lists (from numpy tolist()).
        self.assertAlmostEqual(cargo[0][0], self.point[0])
        self.assertAlmostEqual(cargo[0][1], self.point[1])
        self.assertAlmostEqual(cargo[0][2], self.point[2])

    def test_background_label_is_dropped(self):
        f = self._make_filter(cargo_labels=[2], obstacle_labels=[2, 4])
        mask = np.zeros((480, 640), dtype=np.uint8)
        cargo, obstacle = f.filter_points([self.point], mask)
        self.assertEqual(cargo, [])
        self.assertEqual(obstacle, [])
        stats = f.last_stats
        self.assertEqual(stats["raw_count"], 1)
        self.assertEqual(stats["excluded_count"], 1)

    def test_unknown_label_routes_to_obstacle_only(self):
        f = self._make_filter(cargo_labels=[2], obstacle_labels=[2, 4])
        mask = np.full((480, 640), 4, dtype=np.uint8)  # unknown everywhere
        cargo, obstacle = f.filter_points([self.point], mask)
        self.assertEqual(cargo, [])
        self.assertEqual(len(obstacle), 1)

    def test_out_of_frame_point_is_counted(self):
        # Pick a point that projects far off the image.
        # With depth→color identity rotation, a point at (x=10, y=0, z=1)
        # projects to u = fx * 10 / 1 + cx, well outside 640.
        f = self._make_filter(cargo_labels=[2], obstacle_labels=[2, 4])
        mask = np.zeros((480, 640), dtype=np.uint8)
        cargo, obstacle = f.filter_points([(10.0, 0.0, 1.0)], mask)
        self.assertEqual(cargo, [])
        self.assertEqual(obstacle, [])
        stats = f.last_stats
        self.assertEqual(stats["out_of_frame_count"], 1)

    def test_empty_input_returns_empty(self):
        f = self._make_filter(cargo_labels=[2], obstacle_labels=[2, 4])
        mask = np.zeros((480, 640), dtype=np.uint8)
        cargo, obstacle = f.filter_points([], mask)
        self.assertEqual(cargo, [])
        self.assertEqual(obstacle, [])
        stats = f.last_stats
        self.assertEqual(stats["raw_count"], 0)


class TestFilterWithInstanceMap(unittest.TestCase):
    """Verify that label and instance_id propagate through filter_points."""

    def setUp(self):
        self.color, self.depth, self.extr = _load_realsense_intrinsics()
        self.point = (0.0, 0.0, 1.0)
        uv, _ = _project_to_color(
            np.array([self.point]), self.extr.rotation, self.extr.translation,
            self.color,
        )
        self.projected_uv = (int(uv[0, 0]), int(uv[0, 1]))

    def _make_filter(self):
        return SemanticPointFilter(
            color_intrinsics=self.color,
            depth_intrinsics=self.depth,
            depth_to_color=self.extr,
            cargo_labels=[2],
            obstacle_labels=[2, 4],
        )

    def test_with_instance_map_returns_5_tuples(self):
        f = self._make_filter()
        mask = np.zeros((480, 640), dtype=np.uint8)
        mask[self.projected_uv[1], self.projected_uv[0]] = 2
        inst = np.zeros((480, 640), dtype=np.uint16)
        inst[self.projected_uv[1], self.projected_uv[0]] = 7

        cargo, obstacle = f.filter_points([self.point], mask, instance_map=inst)
        self.assertEqual(len(cargo), 1)
        self.assertEqual(len(cargo[0]), 5)
        x, y, z, label, instance_id = cargo[0]
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(z, 1.0)
        self.assertEqual(label, 2)
        self.assertEqual(instance_id, 7)

    def test_without_instance_map_returns_3_tuples(self):
        f = self._make_filter()
        mask = np.zeros((480, 640), dtype=np.uint8)
        mask[self.projected_uv[1], self.projected_uv[0]] = 2

        cargo, obstacle = f.filter_points([self.point], mask)
        self.assertEqual(len(cargo), 1)
        self.assertEqual(len(cargo[0]), 3)

    def test_instance_zero_when_no_instance_at_pixel(self):
        f = self._make_filter()
        mask = np.zeros((480, 640), dtype=np.uint8)
        mask[self.projected_uv[1], self.projected_uv[0]] = 2
        inst = np.zeros((480, 640), dtype=np.uint16)

        cargo, _obstacle = f.filter_points([self.point], mask, instance_map=inst)
        self.assertEqual(len(cargo), 1)
        self.assertEqual(cargo[0][4], 0)


class TestIntrinsicsFromConfig(unittest.TestCase):
    def test_intrinsics_load_from_realsense_yaml(self):
        color, depth, extr = _load_realsense_intrinsics()
        self.assertEqual(color.width, 640)
        self.assertEqual(color.height, 480)
        # Gazebo rgbd_camera K: fx = (W/2) / tan(1.5184/2), Intel D435 depth 87°.
        expected_fx = (640.0 / 2.0) / math.tan(1.5184 / 2.0)
        self.assertAlmostEqual(color.fx, expected_fx, places=9)
        self.assertAlmostEqual(color.fy, expected_fx, places=9)
        self.assertAlmostEqual(depth.fx, expected_fx, places=9)
        self.assertAlmostEqual(depth.fy, expected_fx, places=9)
        # depth_to_color translation: 15mm along color-Y per realsense_d435.yaml
        self.assertAlmostEqual(extr.translation[1], 0.015, places=4)


if __name__ == "__main__":
    unittest.main()
