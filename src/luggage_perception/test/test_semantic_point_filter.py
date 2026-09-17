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
    JoinStampTracker,
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
        self.assertAlmostEqual(float(cargo[0][0]), self.point[0])
        self.assertAlmostEqual(cargo[0][1], self.point[1])
        self.assertAlmostEqual(cargo[0][2], self.point[2])

    def test_background_label_is_dropped(self):
        f = self._make_filter(cargo_labels=[2], obstacle_labels=[2, 4])
        mask = np.zeros((480, 640), dtype=np.uint8)
        cargo, obstacle = f.filter_points([self.point], mask)
        self.assertEqual(len(cargo), 0)
        self.assertEqual(len(obstacle), 0)
        stats = f.last_stats
        self.assertEqual(stats["raw_count"], 1)
        self.assertEqual(stats["excluded_count"], 1)

    def test_unknown_label_routes_to_obstacle_only(self):
        f = self._make_filter(cargo_labels=[2], obstacle_labels=[2, 4])
        mask = np.full((480, 640), 4, dtype=np.uint8)  # unknown everywhere
        cargo, obstacle = f.filter_points([self.point], mask)
        self.assertEqual(len(cargo), 0)
        self.assertEqual(len(obstacle), 1)

    def test_out_of_frame_point_is_counted(self):
        # Pick a point that projects far off the image.
        # With depth→color identity rotation, a point at (x=10, y=0, z=1)
        # projects to u = fx * 10 / 1 + cx, well outside 640.
        f = self._make_filter(cargo_labels=[2], obstacle_labels=[2, 4])
        mask = np.zeros((480, 640), dtype=np.uint8)
        cargo, obstacle = f.filter_points([(10.0, 0.0, 1.0)], mask)
        self.assertEqual(len(cargo), 0)
        self.assertEqual(len(obstacle), 0)
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
        self.assertEqual(color.height, 360)
        self.assertAlmostEqual(color.fx, 323.1775, places=4)
        self.assertAlmostEqual(color.fy, 322.8994, places=4)
        self.assertAlmostEqual(color.cx, 317.7526, places=4)
        self.assertAlmostEqual(color.cy, 178.0294, places=4)
        self.assertAlmostEqual(depth.fx, color.fx, places=4)
        self.assertAlmostEqual(depth.fy, color.fy, places=4)
        # depth_to_color translation: 15mm along color-Y per realsense_d435.yaml
        self.assertAlmostEqual(extr.translation[1], 0.015, places=4)


class TestJoinStampTracker(unittest.TestCase):
    def test_cloud_without_join_does_not_invent_join_stamp(self):
        tracker = JoinStampTracker()
        tracker.note_depth(1.2)
        tracker.note_mask(1.1)
        payload = tracker.as_dict()
        self.assertAlmostEqual(payload["last_depth_stamp"], 1.2)
        self.assertAlmostEqual(payload["last_mask_stamp"], 1.1)
        self.assertEqual(payload["last_join_stamp"], 0.0)
        self.assertEqual(payload["last_cargo_n_points"], -1)
        self.assertEqual(payload["generation"], 0)
        self.assertEqual(payload["instance_id"], "")

    def test_join_zero_cargo_is_not_never_joined(self):
        tracker = JoinStampTracker()
        tracker.note_join(2.0, 0)
        payload = tracker.as_dict()
        self.assertAlmostEqual(payload["last_join_stamp"], 2.0)
        self.assertEqual(payload["last_cargo_n_points"], 0)
        self.assertEqual(payload["join_count"], 1)

    def test_join_diagnostics_count_misses_and_stale_drops(self):
        tracker = JoinStampTracker()
        tracker.note_depth(10.0)
        tracker.note_depth_waiting_for_mask()
        tracker.note_mask(9.75)
        tracker.note_mask_waiting_for_depth()
        tracker.note_stale_drop(3)
        payload = tracker.as_dict()
        self.assertEqual(payload["depth_count"], 1)
        self.assertEqual(payload["mask_count"], 1)
        self.assertEqual(payload["depth_waiting_for_mask"], 1)
        self.assertEqual(payload["mask_waiting_for_depth"], 1)
        self.assertEqual(payload["stale_drop_count"], 3)
        self.assertAlmostEqual(payload["latest_depth_mask_gap_sec"], 0.25)

    def test_note_epoch_is_in_payload(self):
        tracker = JoinStampTracker()
        tracker.note_epoch(4, "pickup_box_0004_carryon")
        payload = tracker.as_dict()
        self.assertEqual(payload["generation"], 4)
        self.assertEqual(payload["instance_id"], "pickup_box_0004_carryon")


def _tiny_intrinsics(h=64, w=64):
    return CameraIntrinsics(
        fx=100.0, fy=100.0, cx=w / 2.0, cy=h / 2.0, width=w, height=h)


class TestGrowCargoSelByDepth(unittest.TestCase):
    def test_grows_raised_lid_not_platform(self):
        from luggage_perception.semantic_point_filter import (
            grow_cargo_sel_by_depth)
        h, w = 64, 64
        depth = np.full((h, w), 800, dtype=np.uint16)
        depth[16:48, 16:48] = 500
        seed = np.zeros((h, w), dtype=bool)
        seed[28:36, 28:36] = True
        grown, stats = grow_cargo_sel_by_depth(
            depth, seed, depth_tol_mm=30, max_pixels=5000)
        self.assertTrue(stats["cargo_grow_enabled"])
        self.assertEqual(stats["cargo_grow_aborted"], 0)
        self.assertTrue(np.all(grown[16:48, 16:48]))
        self.assertFalse(np.any(grown[0:16, :]))
        self.assertGreater(stats["cargo_pixels_grown"], 0)

    def test_aborts_platform_sized_flood(self):
        from luggage_perception.semantic_point_filter import (
            grow_cargo_sel_by_depth)
        h, w = 64, 64
        depth = np.full((h, w), 800, dtype=np.uint16)
        seed = np.zeros((h, w), dtype=bool)
        seed[30:34, 30:34] = True
        grown, stats = grow_cargo_sel_by_depth(
            depth, seed, depth_tol_mm=30, max_pixels=100)
        self.assertEqual(stats["cargo_grow_aborted"], 1)
        self.assertTrue(np.array_equal(grown, seed))

    def test_disabled_tol_is_noop(self):
        from luggage_perception.semantic_point_filter import (
            grow_cargo_sel_by_depth)
        seed = np.zeros((8, 8), dtype=bool)
        seed[3:5, 3:5] = True
        depth = np.full((8, 8), 500, dtype=np.uint16)
        grown, stats = grow_cargo_sel_by_depth(
            depth, seed, depth_tol_mm=0, max_pixels=100)
        self.assertFalse(stats["cargo_grow_enabled"])
        self.assertTrue(np.array_equal(grown, seed))

    def test_does_not_grow_into_robot_arm_block(self):
        from luggage_perception.semantic_point_filter import (
            grow_cargo_sel_by_depth)
        depth = np.full((16, 16), 500, dtype=np.uint16)
        seed = np.zeros((16, 16), dtype=bool)
        seed[6:10, 6:10] = True
        blocked = np.zeros((16, 16), dtype=bool)
        blocked[:, 12:] = True
        grown, _stats = grow_cargo_sel_by_depth(
            depth, seed, blocked=blocked, depth_tol_mm=30, max_pixels=400)
        self.assertFalse(np.any(grown[:, 12:]))

    def test_filter_depth_uses_grown_mask(self):
        from luggage_perception.semantic_point_filter import SemanticPointFilter
        h, w = 32, 32
        intr = _tiny_intrinsics(h, w)
        filt = SemanticPointFilter(
            intr, intr, DepthToColorExtrinsics.identity(),
            cargo_labels=[2], obstacle_labels=[2, 4],
            grow_depth_tol_mm=30, grow_max_pixels=2000)
        depth = np.full((h, w), 800, dtype=np.uint16)
        depth[8:24, 8:24] = 500
        labels = np.zeros((h, w), dtype=np.uint8)
        labels[14:18, 14:18] = 2
        cargo, _obs = filt.filter_depth(depth, labels, pixel_stride=1)
        self.assertGreater(int(filt.last_stats["cargo_pixels_grown"]), 0)
        self.assertGreaterEqual(cargo.shape[0], 16 * 16)

    def test_radius_cap_blocks_far_similar_depth(self):
        from luggage_perception.semantic_point_filter import (
            grow_cargo_sel_by_depth)
        h, w = 64, 64
        depth = np.full((h, w), 500, dtype=np.uint16)
        seed = np.zeros((h, w), dtype=bool)
        seed[28:36, 28:36] = True
        grown, stats = grow_cargo_sel_by_depth(
            depth, seed, depth_tol_mm=30, max_pixels=5000, max_radius_px=10)
        self.assertEqual(stats["cargo_grow_aborted"], 0)
        self.assertFalse(np.any(grown[0:8, :]))
        self.assertTrue(np.all(grown[28:36, 28:36]))

    def test_closest_band_drops_farther_front_face(self):
        from luggage_perception.semantic_point_filter import (
            grow_cargo_sel_by_depth)
        h, w = 48, 48
        depth = np.full((h, w), 800, dtype=np.uint16)
        depth[8:20, 8:40] = 500
        depth[20:40, 8:40] = 580
        seed = np.zeros((h, w), dtype=bool)
        seed[8:40, 8:40] = True
        grown, stats = grow_cargo_sel_by_depth(
            depth, seed, depth_tol_mm=30, max_pixels=8000)
        self.assertEqual(stats["cargo_grow_aborted"], 0)
        self.assertTrue(np.all(grown[8:20, 8:40]))
        self.assertFalse(np.any(grown[20:40, 8:40]))

    def test_large_lid_floods_connected_same_depth(self):
        from luggage_perception.semantic_point_filter import (
            grow_cargo_sel_by_depth)
        h, w = 120, 120
        depth = np.full((h, w), 800, dtype=np.uint16)
        depth[10:100, 10:110] = 500
        depth[100:118, 10:110] = 500
        seed = np.zeros((h, w), dtype=bool)
        seed[10:100, 10:110] = True
        grown, stats = grow_cargo_sel_by_depth(
            depth, seed, depth_tol_mm=30, max_pixels=60000)
        self.assertEqual(stats["cargo_grow_aborted"], 0)
        self.assertTrue(np.all(grown[10:100, 10:110]))
        self.assertTrue(np.any(grown[100:118, 10:110]))

    def test_large_vertical_origin_is_peeled_not_skipped(self):
        from luggage_perception.semantic_point_filter import (
            grow_cargo_sel_by_depth)
        h, w = 100, 100
        yy = np.arange(h, dtype=np.uint16)[:, None]
        depth = np.broadcast_to((400 + 3 * yy).astype(np.uint16), (h, w)).copy()
        seed = np.zeros((h, w), dtype=bool)
        seed[10:90, 10:90] = True
        grown, stats = grow_cargo_sel_by_depth(
            depth, seed, depth_tol_mm=30, max_pixels=60000)
        self.assertEqual(stats["cargo_grow_flood_skipped"], 0)
        self.assertTrue(
            stats["cargo_grow_vertical_peel"] or stats["cargo_grow_vertical_abort"])
        self.assertLess(int(grown.sum()), int(seed.sum()))
        self.assertFalse(np.any(grown[80:90, 10:90]))

    def test_vertical_flood_aborts_to_origin(self):
        from luggage_perception.semantic_point_filter import (
            grow_cargo_sel_by_depth)
        h, w = 48, 32
        yy = np.arange(h, dtype=np.uint16)[:, None]
        depth = np.broadcast_to((400 + 3 * yy).astype(np.uint16), (h, w)).copy()
        seed = np.zeros((h, w), dtype=bool)
        seed[16:24, 8:16] = True
        grown, stats = grow_cargo_sel_by_depth(
            depth, seed, depth_tol_mm=30, max_pixels=8000, max_radius_px=40)
        self.assertTrue(
            stats["cargo_grow_vertical_abort"] or stats["cargo_grow_vertical_peel"])
        self.assertTrue(np.any(grown[16:24, 8:16]))
        self.assertFalse(np.any(grown[0:8, :]))
        self.assertFalse(np.any(grown[40:, :]))

    def test_search_radius_finds_nearby_lid(self):
        from luggage_perception.semantic_point_filter import (
            grow_cargo_sel_by_depth)
        h, w = 64, 80
        depth = np.full((h, w), 800, dtype=np.uint16)
        depth[8:28, 8:40] = 500
        seed = np.zeros((h, w), dtype=bool)
        seed[32:40, 48:60] = True
        grown, stats = grow_cargo_sel_by_depth(
            depth, seed, depth_tol_mm=30, max_pixels=8000,
            search_radius_px=24)
        self.assertTrue(np.any(grown[8:28, 8:40]))
        self.assertFalse(np.any(grown[32:40, 48:60]))

    def test_drops_unraised_platform_patch(self):
        from luggage_perception.semantic_point_filter import (
            drop_unraised_cargo_sel)
        h, w = 40, 40
        depth = np.full((h, w), 800, dtype=np.uint16)
        cargo = np.zeros((h, w), dtype=bool)
        cargo[16:24, 16:24] = True
        out, stats = drop_unraised_cargo_sel(depth, cargo, raise_mm=50)
        self.assertEqual(stats["cargo_unraised_drop"], 1)
        self.assertFalse(out.any())

    def test_drops_large_far_surface_mask(self):
        from luggage_perception.semantic_point_filter import (
            drop_unraised_cargo_sel)
        h, w = 40, 40
        depth = np.full((h, w), 800, dtype=np.uint16)
        cargo = np.ones((h, w), dtype=bool)
        cargo[0:2, :] = False
        out, stats = drop_unraised_cargo_sel(depth, cargo, raise_mm=50)
        self.assertEqual(stats["cargo_unraised_drop"], 1)
        self.assertFalse(out.any())

    def test_keeps_raised_lid(self):
        from luggage_perception.semantic_point_filter import (
            drop_unraised_cargo_sel)
        h, w = 40, 40
        depth = np.full((h, w), 800, dtype=np.uint16)
        depth[16:24, 16:24] = 500
        cargo = np.zeros((h, w), dtype=bool)
        cargo[16:24, 16:24] = True
        out, stats = drop_unraised_cargo_sel(depth, cargo, raise_mm=50)
        self.assertEqual(stats["cargo_unraised_drop"], 0)
        self.assertTrue(np.array_equal(out, cargo))


if __name__ == "__main__":
    unittest.main()
