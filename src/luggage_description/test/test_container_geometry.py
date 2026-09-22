#!/usr/bin/env python3
"""G1 tests for ROS-free container geometry."""

import importlib
import math
import os
import unittest

import yaml

from luggage_description.container_geometry import (  # noqa: E402
    aabb_intersection_volume,
    clip_y_interval_to_hull,
    contains_oriented_box,
    contains_oriented_box_through_aperture,
    contains_point,
    contains_point_through_aperture,
    contains_swept_box,
    descriptor_from_scene_config,
    floor_area,
    floor_support_area,
    geometry_hash,
    hull_edges,
    normalize_descriptor,
    payload_center_y_interval,
    polygon_area,
    sum_aabb_tiles,
    volume,
    y_bounds_for_z_interval,
    y_max_at_z,
    yz_polygon,
)


PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _example_scene():
    path = os.path.join(PKG_ROOT, "config", "scene_tf.yaml.example")
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


class TestContainerGeometry(unittest.TestCase):
    def setUp(self):
        self.geometry = descriptor_from_scene_config(_example_scene())

    def test_checked_in_volume_and_floor_area_are_exact(self):
        self.assertAlmostEqual(volume(self.geometry), 4.22433625, places=9)
        self.assertAlmostEqual(floor_area(self.geometry), 2.28715, places=9)
        self.assertAlmostEqual(polygon_area(yz_polygon(self.geometry)), 2.835125, places=9)

    def test_cuboid_fallback_volume_and_floor_area(self):
        geometry = normalize_descriptor(
            {
                "frame_id": "container_link",
                "length": 1.5,
                "width": 2.0,
                "floor_z": 0.4,
                "ceiling_z": 1.9,
            }
        )
        self.assertAlmostEqual(volume(geometry), 4.5, places=9)
        self.assertAlmostEqual(floor_area(geometry), 3.0, places=9)
        self.assertTrue(contains_point(geometry, [0.0, 0.99, 0.41]))

    def test_hash_is_normalized_and_mismatch_fails_closed(self):
        descriptor = self.geometry.descriptor()
        self.assertEqual(descriptor["geometry_hash"], geometry_hash(descriptor))
        reordered = {
            "width": descriptor["width"],
            "length": descriptor["length"],
            "ceiling_z": descriptor["ceiling_z"],
            "floor_z": descriptor["floor_z"],
            "frame_id": descriptor["frame_id"],
            "schema_version": descriptor["schema_version"],
            "chamfer": dict(descriptor["chamfer"]),
        }
        self.assertEqual(geometry_hash(reordered), descriptor["geometry_hash"])
        bad = dict(descriptor)
        bad["width"] = bad["width"] + 0.01
        with self.assertRaises(ValueError):
            normalize_descriptor(bad)

    def test_point_faces_and_chamfer_boundary(self):
        g = self.geometry
        delta = 1e-4
        interior_z = 1.20
        faces = (
            ((-g.half_x, 0.0, interior_z), (-1.0, 0.0, 0.0)),
            ((g.half_x, 0.0, interior_z), (1.0, 0.0, 0.0)),
            ((0.0, -g.half_y, interior_z), (0.0, -1.0, 0.0)),
            ((0.0, g.half_y, interior_z), (0.0, 1.0, 0.0)),
            ((0.0, 0.0, g.floor_z), (0.0, 0.0, -1.0)),
            ((0.0, 0.0, g.ceiling_z), (0.0, 0.0, 1.0)),
        )
        for on_face, outward in faces:
            just_in = [on_face[i] - outward[i] * delta for i in range(3)]
            just_out = [on_face[i] + outward[i] * delta for i in range(3)]
            self.assertTrue(contains_point(g, on_face))
            self.assertTrue(contains_point(g, just_in))
            self.assertFalse(contains_point(g, just_out))
        y_s = y_max_at_z(g, 0.70)
        self.assertTrue(contains_point(g, [0.0, y_s, 0.70]))
        self.assertTrue(contains_point(g, [0.0, y_s - delta, 0.70]))
        self.assertFalse(contains_point(g, [0.0, y_s + delta, 0.70]))
        self.assertFalse(contains_point(g, [0.0, 0.90, 0.55]))
        self.assertTrue(contains_point(g, [0.0, 0.90, 1.20]))

    def test_boxes_on_each_face_just_inside_and_outside(self):
        g = self.geometry
        size = [0.08, 0.08, 0.08]
        half = 0.04
        delta = 1e-4
        z = 1.20
        self.assertTrue(contains_oriented_box(g, [g.half_x - half - delta, 0.0, z], size))
        self.assertFalse(contains_oriented_box(g, [g.half_x - half + delta, 0.0, z], size))
        self.assertTrue(contains_oriented_box(g, [-g.half_x + half + delta, 0.0, z], size))
        self.assertFalse(contains_oriented_box(g, [-g.half_x + half - delta, 0.0, z], size))
        self.assertTrue(contains_oriented_box(g, [0.0, g.half_y - half - delta, z], size))
        self.assertFalse(contains_oriented_box(g, [0.0, g.half_y - half + delta, z], size))
        self.assertTrue(contains_oriented_box(g, [0.0, -g.half_y + half + delta, z], size))
        self.assertFalse(contains_oriented_box(g, [0.0, -g.half_y + half - delta, z], size))
        self.assertTrue(contains_oriented_box(g, [0.0, 0.0, g.floor_z + half + delta], size))
        self.assertFalse(contains_oriented_box(g, [0.0, 0.0, g.floor_z + half - delta], size))
        self.assertTrue(contains_oriented_box(g, [0.0, 0.0, g.ceiling_z - half - delta], size))
        self.assertFalse(contains_oriented_box(g, [0.0, 0.0, g.ceiling_z - half + delta], size))
        z_s = 0.70
        y_low = y_max_at_z(g, z_s - half)
        self.assertTrue(contains_oriented_box(g, [0.0, y_low - half - delta, z_s], size))
        self.assertFalse(contains_oriented_box(g, [0.0, y_low - half + delta, z_s], size))

    def test_margin_insets_slanted_and_axis_aligned_faces(self):
        g = self.geometry
        z = 0.70
        without_margin = y_max_at_z(g, z, margin=0.0)
        with_margin = y_max_at_z(g, z, margin=0.05)
        self.assertLess(with_margin, without_margin - 0.05)
        self.assertTrue(contains_point(g, [0.0, with_margin - 1e-5, z], margin=0.05))
        self.assertFalse(contains_point(g, [0.0, without_margin - 1e-5, z], margin=0.05))
        m = 0.05
        interior_z = 1.20
        self.assertTrue(contains_point(g, [g.half_x - 0.01, 0.0, interior_z]))
        self.assertFalse(contains_point(g, [g.half_x - 0.01, 0.0, interior_z], margin=m))
        self.assertTrue(contains_point(g, [-g.half_x + 0.01, 0.0, interior_z]))
        self.assertFalse(contains_point(g, [-g.half_x + 0.01, 0.0, interior_z], margin=m))
        self.assertTrue(contains_point(g, [0.0, g.half_y - 0.01, interior_z]))
        self.assertFalse(contains_point(g, [0.0, g.half_y - 0.01, interior_z], margin=m))
        self.assertTrue(contains_point(g, [0.0, -g.half_y + 0.01, interior_z]))
        self.assertFalse(contains_point(g, [0.0, -g.half_y + 0.01, interior_z], margin=m))
        self.assertTrue(contains_point(g, [0.0, 0.0, g.floor_z + 0.01]))
        self.assertFalse(contains_point(g, [0.0, 0.0, g.floor_z + 0.01], margin=m))
        self.assertTrue(contains_point(g, [0.0, 0.0, g.ceiling_z - 0.01]))
        self.assertFalse(contains_point(g, [0.0, 0.0, g.ceiling_z - 0.01], margin=m))
        self.assertEqual(yz_polygon(g, margin=10.0), [])
        self.assertFalse(contains_point(g, [0.0, 0.0, interior_z], margin=10.0))

    def test_invalid_descriptors_fail_closed(self):
        base = {
            "frame_id": "container_link",
            "length": 1.5,
            "width": 2.0,
            "floor_z": 0.4,
            "ceiling_z": 1.9,
        }
        with self.assertRaises(ValueError):
            normalize_descriptor(dict(base, length=float("nan")))
        with self.assertRaises(ValueError):
            normalize_descriptor(
                dict(
                    base,
                    chamfer={
                        "side": "negative_x",
                        "floor_y": 0.0,
                        "wall_y": 0.5,
                        "wall_z": 0.9,
                    },
                )
            )
        with self.assertRaises(ValueError):
            normalize_descriptor(
                dict(
                    base,
                    chamfer={
                        "side": "positive_y",
                        "floor_y": 0.0,
                        "wall_y": 0.5,
                        "wall_z": 0.4,
                    },
                )
            )
        with self.assertRaises(ValueError):
            normalize_descriptor(
                dict(
                    base,
                    chamfer={
                        "side": "positive_y",
                        "floor_y": 1.5,
                        "wall_y": 1.5,
                        "wall_z": 0.9,
                    },
                )
            )

    def test_aperture_variant_opens_only_the_minus_x_face(self):
        """An insertion path stands in the doorway; every other face holds.

        Enforcing full containment at the aperture rejects every place path:
        the entry waypoint sits at ``x = -half_x``, so a box centred on it has
        corners beyond that face whatever its size.
        """
        g = self.geometry
        size = [0.08, 0.08, 0.08]
        half = 0.04
        delta = 1e-4
        z = 1.20
        out_of_door = [-g.half_x + half - delta, 0.0, z]
        self.assertFalse(contains_oriented_box(g, out_of_door, size))
        self.assertTrue(
            contains_oriented_box_through_aperture(g, out_of_door, size))
        # Fully outside in -X is still allowed: that is the approach.
        self.assertTrue(contains_oriented_box_through_aperture(
            g, [-g.half_x - 0.5, 0.0, z], size))
        for blocked in (
                [g.half_x - half + delta, 0.0, z],
                [0.0, g.half_y - half + delta, z],
                [0.0, -g.half_y + half - delta, z],
                [0.0, 0.0, g.floor_z + half - delta],
                [0.0, 0.0, g.ceiling_z - half + delta]):
            self.assertFalse(
                contains_oriented_box_through_aperture(g, blocked, size),
                blocked)

    def test_aperture_variant_still_enforces_the_chamfer(self):
        g = self.geometry
        z = 0.70
        y_slant = y_max_at_z(g, z)
        self.assertTrue(contains_point_through_aperture(
            g, [-g.half_x - 0.2, y_slant - 1e-4, z]))
        self.assertFalse(contains_point_through_aperture(
            g, [-g.half_x - 0.2, y_slant + 1e-4, z]))

    def test_oriented_box_checks_all_corners(self):
        g = self.geometry
        self.assertTrue(contains_oriented_box(g, [0.0, -0.3, 1.1], [0.4, 0.3, 0.3]))
        self.assertFalse(contains_oriented_box(g, [0.0, 0.50, 0.65], [0.4, 0.3, 0.2]))
        self.assertTrue(
            contains_oriented_box(
                g,
                [0.0, -0.2, 1.2],
                [0.4, 0.3, 0.2],
                yaw=math.radians(30.0),
            )
        )

    def test_swept_payload_rejects_slanted_face_crossing(self):
        g = self.geometry
        size = [0.2, 0.2, 0.2]
        self.assertTrue(contains_swept_box(g, [-0.2, -0.4, 1.1], [0.2, -0.2, 1.1], size))
        self.assertFalse(contains_swept_box(g, [-0.2, 0.45, 0.62], [0.2, 0.70, 0.62], size))

    def test_aabb_intersection_boundary_weights_sum_to_hull_volume(self):
        g = self.geometry
        self.assertAlmostEqual(
            aabb_intersection_volume(
                g,
                [-g.half_x, -g.half_y, g.floor_z],
                [g.half_x, g.half_y, g.ceiling_z],
            ),
            volume(g),
            places=9,
        )
        self.assertAlmostEqual(
            aabb_intersection_volume(g, [-0.2, 0.80, 0.54], [0.2, 0.95, 0.70]),
            0.0,
            places=9,
        )
        x_edges = [-g.half_x, -0.1, g.half_x]
        y_edges = [-g.half_y, -0.2, 0.55, g.half_y]
        z_edges = [g.floor_z, 0.70, g.chamfer.wall_z, g.ceiling_z]
        self.assertAlmostEqual(sum_aabb_tiles(g, x_edges, y_edges, z_edges), volume(g), places=9)

    def test_floor_support_and_payload_center_interval(self):
        g = self.geometry
        self.assertAlmostEqual(
            floor_support_area(g, [-g.half_x, -g.half_y], [g.half_x, g.half_y]),
            floor_area(g),
            places=9,
        )
        self.assertAlmostEqual(
            floor_support_area(g, [-0.2, 0.70], [0.2, 0.90]),
            0.0,
            places=9,
        )
        y_min, y_max = payload_center_y_interval(g, [0.4, 0.2, 0.3], 0.62, 0.82)
        self.assertLess(y_min, y_max)
        self.assertLess(y_max, 0.57)

    def test_z_interval_y_clip_uses_tightest_chamfer(self):
        g = self.geometry
        y_min, y_max = y_bounds_for_z_interval(g, g.floor_z, g.floor_z + 0.05)
        self.assertAlmostEqual(y_min, -g.half_y, places=9)
        self.assertAlmostEqual(y_max, y_max_at_z(g, g.floor_z), places=9)
        self.assertLess(y_max, g.half_y - 0.3)
        lo, hi = clip_y_interval_to_hull(
            g, -g.half_y, g.half_y, 0.0, 0.05, z_is_floor_relative=True)
        self.assertAlmostEqual(lo, y_min, places=9)
        self.assertAlmostEqual(hi, y_max, places=9)
        cuboid = normalize_descriptor(
            {
                "frame_id": "container_link",
                "length": 2.0,
                "width": 2.0,
                "floor_z": 0.0,
                "ceiling_z": 2.0,
            }
        )
        c_lo, c_hi = clip_y_interval_to_hull(
            cuboid, -0.4, 0.4, 0.0, 0.5, z_is_floor_relative=True)
        self.assertAlmostEqual(c_lo, -0.4, places=9)
        self.assertAlmostEqual(c_hi, 0.4, places=9)

    def test_hull_edges_are_seven_face_not_cuboid(self):
        edges = hull_edges(self.geometry)
        self.assertGreaterEqual(len(edges), 15)
        cuboid = normalize_descriptor(
            {
                "frame_id": "container_link",
                "length": 1.5,
                "width": 2.0,
                "floor_z": 0.4,
                "ceiling_z": 1.9,
            }
        )
        self.assertEqual(len(hull_edges(cuboid)), 12)

    def test_module_imports_without_ros(self):
        module = importlib.import_module("luggage_description.container_geometry")
        self.assertTrue(hasattr(module, "ContainerGeometry"))


if __name__ == "__main__":
    unittest.main()
