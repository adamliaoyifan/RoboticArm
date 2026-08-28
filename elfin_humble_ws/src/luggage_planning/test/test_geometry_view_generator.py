#!/usr/bin/env python3
"""Unit tests for geometry_view_generator (no roscore required).

Verifies that:
- Phase 0 (opening) and Phase 1 (interior) views satisfy the camera-down
  tilt constraint (optical +Z within ``max_tilt_deg`` of world -Z).
- ``filter_reachable_views`` correctly drops viewpoints beyond the arm
  reach radius.
- ``validate_tilt`` agrees with the per-view ``tilt_deg`` field.
- Generated views include the required dict keys for IK / coverage scoring.
- The tilt filter uses the optical +Z axis (camera toward look_at), not a
  trivial world-frame check.
"""

from __future__ import division

import math
import os
import sys
import unittest
import copy

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_description.scene_tf_config_utils import load_scene_tf_config
import luggage_planning.geometry_view_generator as g


def _example_scene_path():
    return os.path.join(
        PKG_ROOT, "..", "luggage_description", "config", "scene_tf.yaml.example")


def _reachable_scene():
    """Build a synthetic scene where the container is within arm reach.

    The real example scene places the container at world [1.5, 0, 0] which
    is ~1.8m from the robot base — beyond the S20 arm reach (~1.56m). For
    tilt + reach validation we use a synthetic scene with the container
    tucked just behind the robot base in world (so the opening face sits
    in front of the arm at base_link x ~ 0.83m, within reach).
    """
    return {
        "world_frame": "world",
        "pedestal": {
            "enabled": True,
            "size": [1.05, 0.74, 0.86],
            "translation": [0.0, 0.0, 0.0],
            "rotation_rpy": [0.0, 0.0, math.pi / 2.0],
            "gazebo_model": "robot_pedestal",
        },
        "robot": {
            "base_frame": "elfin_base_link",
            "rotation_rpy": [0.0, 0.0, 0.0],
        },
        "container": {
            "outer": {"length": 1.57, "width": 2.07, "height": 2.12},
            "inner": {"length": 1.49, "width": 1.97, "height": 2.01},
            "opening": {
                "side": "positive_y",
                "width": 1.18,
                "height": 1.93,
                "frame": {"xyz": [0.0, 1.03, 1.06], "rpy": [0.0, 0.0, 0.0]},
            },
        },
        "static_transforms": [
            {
                "parent": "world",
                "child": "container_link",
                "translation": [0.0, -0.2, 0.0],
                "rotation_rpy": [0.0, 0.0, 0.0],
            },
            {
                "parent": "container_link",
                "child": "container_opening_frame",
                "translation": [0.0, 1.03, 1.06],
                "rotation_rpy": [0.0, 0.0, 0.0],
            },
        ],
    }


def _asymmetric_probe_scene():
    """Synthetic -X opening with off-center aperture corners (real-container shape)."""
    scene = copy.deepcopy(_reachable_scene())
    inner_l, inner_w, inner_h = 1.49, 1.97, 2.01
    scene["container"]["opening"] = {
        "side": "negative_x",
        "width": 1.33,
        "height": 1.41,
        "aperture": {
            "corners": [
                [-inner_l * 0.5, -0.928, 0.597],
                [-inner_l * 0.5, 0.398, 0.597],
                [-inner_l * 0.5, 0.398, 2.009],
                [-inner_l * 0.5, -0.928, 2.009],
            ],
        },
    }
    for item in scene["static_transforms"]:
        if item["child"] == "container_opening_frame":
            item["translation"] = [-inner_l * 0.5, -0.27, 1.30]
    return scene


class TestGeometryViewGenerator(unittest.TestCase):
    def setUp(self):
        self.reachable_scene = _reachable_scene()
        self.real_scene = load_scene_tf_config(_example_scene_path())

    # ---- tilt validation ----

    def test_validate_tilt_straight_down(self):
        ok, tilt, axis = g.validate_tilt(
            [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], max_tilt_deg=45.0
        )
        self.assertTrue(ok)
        self.assertAlmostEqual(tilt, 0.0, places=4)
        self.assertAlmostEqual(axis[2], -1.0, places=4)

    def test_validate_tilt_horizontal_rejected(self):
        # Camera looking horizontally: optical +Z = +X, perpendicular to -Z.
        ok, tilt, _ = g.validate_tilt(
            [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], max_tilt_deg=45.0
        )
        self.assertFalse(ok)
        self.assertAlmostEqual(tilt, 90.0, places=4)

    def test_validate_tilt_45deg_boundary(self):
        # Camera at 45 deg from vertical: tilt should be exactly 45.
        # Place camera so view_dir = normalize([-1, 0, -1]) -> 45 deg from -Z.
        ok, tilt, _ = g.validate_tilt(
            [1.0, 0.0, 1.0], [0.0, 0.0, 0.0], max_tilt_deg=45.0
        )
        # Allow a tiny epsilon for floating-point acos at the boundary.
        self.assertLessEqual(tilt, 45.0 + 1e-6)
        self.assertTrue(ok or abs(tilt - 45.0) < 1e-6)

    # ---- Phase 0 (opening) ----

    def test_phase0_views_satisfy_tilt_constraint(self):
        views = g.generate_opening_views(
            self.reachable_scene, num_views=5,
            arc_radius=0.30, height_above_opening=0.60,
            max_tilt_deg=45.0,
        )
        self.assertEqual(len(views), 5)
        for v in views:
            self.assertEqual(v["stage"], "opening")
            self.assertIn("camera_xyz", v)
            self.assertIn("look_at", v)
            self.assertIn("orientation_quat", v)
            self.assertTrue(v["valid_tilt"],
                            "view %s tilt=%.1f deg exceeds 45 deg" % (
                                v["name"], v["tilt_deg"],
                            ))
            self.assertLessEqual(v["tilt_deg"], 45.0 + 1e-3)

    def test_phase0_views_look_at_opening_center(self):
        from luggage_description.scene_tf_config_utils import container_opening_target_point
        opening_center = container_opening_target_point(self.reachable_scene)
        views = g.generate_opening_views(self.reachable_scene, num_views=5)
        for v in views:
            for i in range(3):
                self.assertAlmostEqual(
                    v["look_at"][i], opening_center[i], places=4,
                )

    def test_phase0_rotation_aware_arc_in_opening_plane(self):
        # A rotated + tilted container must still produce an arc whose lateral
        # offsets lie in the opening plane (perpendicular to the opening
        # normal), using the container's own axes rather than a fixed world +Z.
        from luggage_description.scene_tf_config_utils import container_opening_normal_in_base_link
        scene = copy.deepcopy(self.reachable_scene)
        for item in scene["static_transforms"]:
            if item["child"] == "container_link":
                item["rotation_rpy"] = [0.3, 0.0, 0.5]
        normal = container_opening_normal_in_base_link(scene)
        views = g.generate_opening_views(scene, num_views=5, max_tilt_deg=45.0)
        self.assertEqual(len(views), 5)
        # Arc center ~= mean of the symmetric lateral offsets.
        n = len(views)
        mean = [sum(v["camera_xyz"][i] for v in views) / n for i in range(3)]
        spreads = [
            math.sqrt(sum((v["camera_xyz"][i] - mean[i]) ** 2 for i in range(3)))
            for v in views
        ]
        # The arc is spread (not collapsed to a single point).
        self.assertGreater(max(spreads), 1e-3)
        # Each lateral offset is perpendicular to the opening normal.
        for v in views:
            offset = [v["camera_xyz"][i] - mean[i] for i in range(3)]
            dot = sum(offset[i] * normal[i] for i in range(3))
            self.assertLess(
                abs(dot), 1e-6,
                "lateral offset %s not in opening plane (normal=%s)" % (offset, normal),
            )

    # ---- Phase 1 (interior) ----

    def test_phase1_views_satisfy_tilt_constraint(self):
        views = g.generate_interior_views(
            self.reachable_scene, num_lateral=3, num_height=3,
            standoff_values=[0.05, 0.15, 0.30],
            max_tilt_deg=60.0, look_depth_ratio=0.3,
        )
        # 3 lateral * 3 height * 3 standoff = 27 candidates
        self.assertEqual(len(views), 27)
        for v in views:
            self.assertEqual(v["stage"], "interior")
            self.assertIn("orientation_quat", v)
            # Each view carries a tilt_deg and valid_tilt flag; views that
            # look up (camera below the look_at point) will have tilt > 90
            # and valid_tilt=False. The filter_valid_views function is
            # responsible for dropping them (tested separately).
            self.assertGreaterEqual(v["tilt_deg"], 0.0)
            self.assertLessEqual(v["tilt_deg"], 180.0)
            self.assertEqual(
                v["valid_tilt"], v["tilt_deg"] <= 60.0,
            )

    def test_phase1_filtered_views_satisfy_tilt_constraint(self):
        """After tilt filtering, all remaining interior views are valid."""
        views = g.generate_interior_views(
            self.reachable_scene, num_lateral=3, num_height=3,
            standoff_values=[0.05, 0.15, 0.30],
            max_tilt_deg=60.0, look_depth_ratio=0.3,
        )
        valid = g.filter_valid_views(views)
        self.assertGreater(len(valid), 0,
                           "expected at least one valid interior view")
        for v in valid:
            self.assertTrue(v["valid_tilt"])
            self.assertLessEqual(v["tilt_deg"], 60.0 + 1e-3)

    # ---- reach filtering ----

    def test_filter_reachable_views_drops_far_views(self):
        far_scene = copy.deepcopy(self.real_scene)
        for item in far_scene["static_transforms"]:
            if item["child"] == "container_link":
                item["translation"] = [3.0, 0.0, 0.0]
        views = g.generate_opening_views(far_scene, num_views=5)
        reachable = g.filter_reachable_views(views, max_reach=1.6)
        self.assertEqual(len(reachable), 0)

    def test_filter_reachable_views_keeps_close_views(self):
        views = g.generate_opening_views(self.reachable_scene, num_views=5)
        reachable = g.filter_reachable_views(views, max_reach=1.6)
        # The synthetic scene puts the opening within reach; at least the
        # center view (index 2) should survive the reach filter.
        self.assertGreater(len(reachable), 0,
                           "expected at least one reachable opening view")

    def test_distance_from_base_zero_at_origin(self):
        self.assertEqual(g.distance_from_base([0.0, 0.0, 0.0]), 0.0)
        self.assertAlmostEqual(
            g.distance_from_base([1.0, 0.0, 0.0]), 1.0, places=6,
        )
        self.assertAlmostEqual(
            g.distance_from_base([0.0, 1.0, 0.0]), 1.0, places=6,
        )

    # ---- combined ----

    def test_phase0_then_filter_produces_usable_candidates(self):
        """End-to-end: Phase 0 views on the reachable scene pass tilt and
        reach filters, so they are usable IK candidates."""
        views = g.generate_opening_views(self.reachable_scene, num_views=5)
        views = g.filter_valid_views(views)
        views = g.filter_reachable_views(views, max_reach=1.6)
        self.assertGreater(len(views), 0)
        for v in views:
            self.assertLessEqual(
                g.distance_from_base(v["camera_xyz"]), 1.6,
            )
            self.assertLessEqual(v["tilt_deg"], 45.0 + 1e-3)

    def test_views_to_candidates_preserves_required_keys(self):
        views = g.generate_opening_views(self.reachable_scene, num_views=3)
        candidates = g.views_to_candidates(views)
        self.assertEqual(len(candidates), 3)
        for c in candidates:
            for key in ("name", "camera_xyz", "look_at", "orientation_quat"):
                self.assertIn(key, c)

    def test_phase0_quaternion_is_unit(self):
        views = g.generate_opening_views(self.reachable_scene, num_views=5)
        for v in views:
            q = v["orientation_quat"]
            norm = math.sqrt(sum(c * c for c in q))
            self.assertAlmostEqual(norm, 1.0, places=4,
                                   msg="non-unit quaternion for %s" % v["name"])

    # ---- interior camera-down probes ----

    def test_interior_probe_candidates_are_camera_down(self):
        views = g.generate_interior_downward_views(
            self.reachable_scene, num_lateral=3, num_depth=3,
            camera_z=0.75, wall_clearance=0.10, aperture_margin=0.10,
        )
        self.assertEqual(len(views), 9)
        for view in views:
            self.assertEqual(view["stage"], "interior_probe")
            self.assertEqual(view["orientation_quat"], [1.0, 0.0, 0.0, 0.0])
            self.assertAlmostEqual(view["look_at"][0], view["camera_xyz"][0])
            self.assertAlmostEqual(view["look_at"][1], view["camera_xyz"][1])
            self.assertLess(view["look_at"][2], view["camera_xyz"][2])
            self.assertEqual(view["tilt_deg"], 0.0)

    def test_interior_probe_lane_id_groups_by_lateral_offset(self):
        # Phase 1 selection collapses same-lane candidates to the shallowest
        # one when no lane is active yet (cargo_exploration_planner_node's
        # shallowest-by-lane dedup). Each lateral offset must be its own
        # lane so all of them remain candidates at the shallowest depth,
        # instead of colliding into a single shared "" lane_id.
        views = g.generate_interior_downward_views(
            self.reachable_scene, num_lateral=3, num_depth=2,
            camera_z=0.75, wall_clearance=0.10, aperture_margin=0.10,
        )
        by_depth = {}
        for view in views:
            by_depth.setdefault(view["depth"], []).append(view)
        self.assertEqual(len(by_depth), 2)
        for depth, depth_views in by_depth.items():
            lane_ids = [v["lane_id"] for v in depth_views]
            self.assertEqual(
                len(set(lane_ids)), 3,
                "expected 3 distinct lanes at depth=%s, got %s" % (
                    depth, lane_ids))
        # Same lateral offset shares the same lane_id across depths.
        depths = sorted(by_depth)
        shallow_lanes = {v["lateral_offset"]: v["lane_id"]
                          for v in by_depth[depths[0]]}
        deep_lanes = {v["lateral_offset"]: v["lane_id"]
                      for v in by_depth[depths[1]]}
        self.assertEqual(shallow_lanes, deep_lanes)

    def test_geometry_valid_probe_candidates_are_inside_inner_box(self):
        from luggage_description.scene_tf_config_utils import container_inner_box_in_base_link
        mins, maxs = container_inner_box_in_base_link(self.reachable_scene)
        views = g.generate_interior_downward_views(
            self.reachable_scene, num_lateral=3, num_depth=3,
            camera_z=0.75, wall_clearance=0.10, aperture_margin=0.10,
        )
        valid = g.filter_geometry_valid_views(views)
        self.assertGreater(len(valid), 0)
        for view in valid:
            for axis in range(3):
                self.assertGreaterEqual(view["camera_xyz"][axis], mins[axis] + 0.10 - 1e-6)
                self.assertLessEqual(view["camera_xyz"][axis], maxs[axis] - 0.10 + 1e-6)

    def test_probe_aperture_points_respect_safe_window(self):
        from luggage_description.scene_tf_config_utils import point_inside_opening_aperture
        views = g.generate_interior_downward_views(
            self.reachable_scene, num_lateral=3, num_depth=2,
            camera_z=0.75, wall_clearance=0.10, aperture_margin=0.10,
        )
        for view in g.filter_geometry_valid_views(views):
            self.assertTrue(point_inside_opening_aperture(
                view["aperture_xyz"], self.reachable_scene, margin=0.10,
            ))

    def test_probe_reports_wall_clearance_rejection(self):
        views = g.generate_interior_downward_views(
            self.reachable_scene, num_lateral=1, num_depth=1,
            camera_z=0.01, wall_clearance=0.20, aperture_margin=0.10,
        )
        self.assertFalse(views[0]["valid_geometry"])
        self.assertEqual(views[0]["reject_reason"], "wall_clearance")

    def test_probe_reports_aperture_safe_window_rejection(self):
        views = g.generate_interior_downward_views(
            self.reachable_scene, num_lateral=1, num_depth=1,
            camera_z=1.10, wall_clearance=0.0, aperture_margin=0.10,
        )
        self.assertFalse(views[0]["valid_geometry"])
        self.assertEqual(views[0]["reject_reason"], "aperture_blocked")

    def test_probe_distinguishes_outside_box_from_clearance(self):
        views = g.generate_interior_downward_views(
            self.reachable_scene, num_lateral=1, num_depth=1,
            camera_z=-1.00, wall_clearance=0.10, aperture_margin=0.10,
        )
        self.assertFalse(views[0]["valid_geometry"])
        self.assertEqual(views[0]["reject_reason"], "outside_inner_box")

    def test_opening_safe_window_is_inset_on_all_sides(self):
        from luggage_description.scene_tf_config_utils import container_opening_aperture_corners
        outer = container_opening_aperture_corners(self.reachable_scene, margin=0.0)
        safe = container_opening_aperture_corners(self.reachable_scene, margin=0.10)
        outer_width = math.dist(outer[0], outer[1])
        safe_width = math.dist(safe[0], safe[1])
        outer_height = math.dist(outer[1], outer[2])
        safe_height = math.dist(safe[1], safe[2])
        self.assertAlmostEqual(outer_width - safe_width, 0.20, places=6)
        self.assertAlmostEqual(outer_height - safe_height, 0.20, places=6)

    def test_asymmetric_aperture_lateral_offsets_follow_inset_bounds(self):
        from luggage_description.scene_tf_config_utils import container_opening_aperture_lateral_offsets
        scene = _asymmetric_probe_scene()
        offsets = container_opening_aperture_lateral_offsets(scene, 3, margin=0.10)
        self.assertEqual(len(offsets), 3)
        self.assertLess(offsets[0], offsets[-1])
        self.assertLess(offsets[0], 0.0)
        self.assertGreater(offsets[-1], 0.0)

    def test_asymmetric_aperture_allows_valid_probes_inside_opening(self):
        scene = _asymmetric_probe_scene()
        views = g.generate_interior_downward_views(
            scene, num_lateral=3, num_depth=2,
            camera_z=0.75, wall_clearance=0.10, aperture_margin=0.10,
        )
        valid = g.filter_geometry_valid_views(views)
        self.assertGreater(len(valid), 0)
        for view in valid:
            self.assertEqual(view.get("reject_reason", ""), "")

    def test_real_scene_asymmetric_aperture_probes_geometry_valid(self):
        views = g.generate_interior_downward_views(
            self.real_scene, num_lateral=3, num_depth=2,
            wall_clearance=0.15, aperture_margin=0.12,
        )
        valid = g.filter_geometry_valid_views(views)
        self.assertEqual(len(valid), len(views))

    def test_inner_box_clearance_uses_container_orientation_not_aabb(self):
        from luggage_description.scene_tf_config_utils import (
            container_in_base_link,
            point_inside_container_inner_box,
        )
        scene = copy.deepcopy(self.reachable_scene)
        scene["static_transforms"][0]["rotation_rpy"] = [0.0, 0.0, math.pi / 4.0]
        origin, rpy = container_in_base_link(scene)

        def to_base(local):
            c = math.cos(rpy[2])
            s = math.sin(rpy[2])
            return [
                origin[0] + c * local[0] - s * local[1],
                origin[1] + s * local[0] + c * local[1],
                origin[2] + local[2],
            ]

        self.assertTrue(point_inside_container_inner_box(
            to_base([0.0, 0.0, 1.0]), scene
        ))
        self.assertFalse(point_inside_container_inner_box(
            to_base([0.85, 0.0, 1.0]), scene
        ))


class TestUncertaintyAwareCorridors(unittest.TestCase):
    @staticmethod
    def _geometry(**updates):
        geometry = {
            "opening_xyz": [1.0, 0.0, 1.0],
            "normal": [0.0, 1.0, 0.0],
            "lateral": [1.0, 0.0, 0.0],
            "up": [0.0, 0.0, 1.0],
            "aperture_width": 1.0,
            "aperture_height": 0.8,
            "inner_depth": 1.2,
            "geometry_version": 7,
            "source": "tag_depth",
            "age": 0.1,
        }
        geometry.update(updates)
        return geometry

    def test_depths_are_monotonic_and_limited_by_observed_free(self):
        views = g.generate_uncertainty_aware_corridor_views(
            self._geometry(), [1.0, 0.0, 0.0, 0.0],
            observed_free_depth=0.55, num_lateral=1,
            min_depth=0.10, depth_step=0.20,
        )
        depths = [v["depth"] for v in views]
        self.assertEqual(depths, sorted(depths))
        self.assertAlmostEqual(depths[-1], 0.55)
        self.assertTrue(all(v["lane_id"] == "lane_00" for v in views))

    def test_uncertainty_erodes_aperture(self):
        low = g.generate_uncertainty_aware_corridor_views(
            self._geometry(), [1.0, 0.0, 0.0, 0.0], 0.4,
            uncertainty_margin=0.01,
        )
        high = g.generate_uncertainty_aware_corridor_views(
            self._geometry(), [1.0, 0.0, 0.0, 0.0], 0.4,
            uncertainty_margin=0.15,
        )
        low_span = max(abs(v["lateral_offset"]) for v in low)
        high_span = max(abs(v["lateral_offset"]) for v in high)
        self.assertLess(high_span, low_span)

    def test_empty_eroded_aperture_rejects_all_candidates(self):
        views = g.generate_uncertainty_aware_corridor_views(
            self._geometry(aperture_width=0.20),
            [1.0, 0.0, 0.0, 0.0], 0.4,
            camera_half_width=0.08, physical_clearance=0.05,
            uncertainty_margin=0.03,
        )
        self.assertEqual(views, [])

    def test_candidates_have_stable_geometry_version_ids(self):
        views = g.generate_uncertainty_aware_corridor_views(
            self._geometry(), [1.0, 0.0, 0.0, 0.0], 0.25,
            num_lateral=1,
        )
        self.assertTrue(views)
        self.assertTrue(all(
            v["candidate_id"].startswith("g7:") for v in views))


if __name__ == "__main__":
    unittest.main()
