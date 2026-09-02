#!/usr/bin/env python3
"""Unit tests for suitcase mesh SDF + lying spawn pose (no Gazebo)."""

import os
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET

from luggage_description.scene_tf_config_utils import (
    load_scene_tf_config,
    pickup_platform_top_in_world,
    pickup_source_in_world,
)
from luggage_description.suitcase_visual import (
    DEFAULT_MODEL_VISUAL,
    SIZE_TIERS,
    VISUAL_LOAFBRR,
    VISUAL_VINTAGE,
    cuboid_inertia,
    file_uri,
    load_sized_suitcases_manifest,
    mesh_top_footprint,
    mesh_uri,
    pickup_box_pose,
    pickup_visual_sdf,
    scaled_stl_path,
    size_tier_name,
    sized_mesh_uri,
    sized_model_name,
    sized_stl_path,
    source_stl_path,
    suitcase_sdf,
    visual_id_for_entry,
    write_scaled_stl,
)
from luggage_description.box_catalog_utils import (
    box_catalog_entries,
    load_box_catalog,
)

PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
GAZEBO_MODELS = os.path.normpath(os.path.join(
    PKG_ROOT, "..", "luggage_gazebo", "models"))


class TestVisualSelection(unittest.TestCase):
    def test_catalog_visual_field(self):
        catalog = load_box_catalog(
            os.path.join(PKG_ROOT, "config", "box_catalog.yaml.example"))
        by_id = {e["id"]: e for e in box_catalog_entries(catalog)}
        self.assertEqual(visual_id_for_entry(by_id["carryon"]), VISUAL_LOAFBRR)
        self.assertEqual(visual_id_for_entry(by_id["standard"]), VISUAL_VINTAGE)
        self.assertEqual(visual_id_for_entry(by_id["large"]), VISUAL_LOAFBRR)

    def test_falls_back_from_model_name(self):
        self.assertEqual(
            visual_id_for_entry({"model": "suitcase_standard"}),
            DEFAULT_MODEL_VISUAL["suitcase_standard"])


class TestSuitcaseSdf(unittest.TestCase):
    def test_mesh_visual_and_collision_share_uri(self):
        size = [0.55, 0.40, 0.25]
        xml = suitcase_sdf("pickup_box_1", size, 8.0, VISUAL_LOAFBRR)
        root = ET.fromstring(xml)
        vis_uri = root.find(".//visual/geometry/mesh/uri").text
        col_uri = root.find(".//collision/geometry/mesh/uri").text
        vis_scale = [float(v) for v in root.find(
            ".//visual/geometry/mesh/scale").text.split()]
        col_scale = [float(v) for v in root.find(
            ".//collision/geometry/mesh/scale").text.split()]
        self.assertEqual(vis_uri, mesh_uri(VISUAL_LOAFBRR))
        self.assertEqual(col_uri, vis_uri)
        self.assertEqual(vis_scale, size)
        self.assertEqual(col_scale, size)
        self.assertIsNone(root.find(".//collision/geometry/box"))
        self.assertIsNotNone(root.find(".//visual/material/diffuse"))

    def test_box_visual_matches_collision(self):
        size = [0.73, 0.48, 0.28]
        xml = suitcase_sdf(
            "pickup_box_1", size, 12.0, VISUAL_VINTAGE, visual_kind="box")
        root = ET.fromstring(xml)
        visual = [float(v) for v in root.find(
            ".//visual/geometry/box/size").text.split()]
        collision = [float(v) for v in root.find(
            ".//collision/geometry/box/size").text.split()]
        self.assertIsNone(root.find(".//visual/geometry/mesh"))
        self.assertEqual(visual, size)
        self.assertEqual(collision, size)

    def test_already_scaled_keeps_unit_mesh_scale_on_both(self):
        size = [0.77, 0.48, 0.25]
        xml = suitcase_sdf(
            "pickup_box_1", size, 8.0, VISUAL_LOAFBRR,
            mesh_uri_override="file:///tmp/pickup_box_1.stl",
            mesh_already_scaled=True)
        root = ET.fromstring(xml)
        vis_uri = root.find(".//visual/geometry/mesh/uri").text
        col_uri = root.find(".//collision/geometry/mesh/uri").text
        vis_scale = [float(v) for v in root.find(
            ".//visual/geometry/mesh/scale").text.split()]
        col_scale = [float(v) for v in root.find(
            ".//collision/geometry/mesh/scale").text.split()]
        self.assertEqual(vis_uri, "file:///tmp/pickup_box_1.stl")
        self.assertEqual(col_uri, vis_uri)
        self.assertEqual(vis_scale, [1.0, 1.0, 1.0])
        self.assertEqual(col_scale, [1.0, 1.0, 1.0])
        self.assertIsNone(root.find(".//collision/geometry/box"))

    def test_inertia_is_solid_cuboid(self):
        size = [0.70, 0.45, 0.28]
        xml = suitcase_sdf("m", size, 15.0, VISUAL_VINTAGE)
        root = ET.fromstring(xml)
        ixx, iyy, izz = cuboid_inertia(size, 15.0)
        self.assertAlmostEqual(
            float(root.find(".//inertia/ixx").text), ixx, places=6)
        self.assertAlmostEqual(
            float(root.find(".//inertia/iyy").text), iyy, places=6)
        self.assertAlmostEqual(
            float(root.find(".//inertia/izz").text), izz, places=6)


class TestLyingSpawnPose(unittest.TestCase):
    def test_bottom_sits_on_pickup_source(self):
        source_xyz = (1.0, -0.8, 0.86)
        size = [0.55, 0.40, 0.25]
        xyz, rpy = pickup_box_pose(source_xyz, (0.0, 0.0, 0.1), size, yaw_offset=0.2)
        self.assertAlmostEqual(xyz[0], source_xyz[0])
        self.assertAlmostEqual(xyz[1], source_xyz[1])
        self.assertAlmostEqual(xyz[2], source_xyz[2] + size[2] / 2.0)
        self.assertAlmostEqual(rpy[2], 0.3)
        bottom_z = xyz[2] - size[2] / 2.0
        top_z = xyz[2] + size[2] / 2.0
        self.assertAlmostEqual(bottom_z, source_xyz[2])
        self.assertAlmostEqual(top_z, source_xyz[2] + size[2])

    def test_scene_tf_platform_is_flat_identity(self):
        scene = load_scene_tf_config(
            os.path.join(PKG_ROOT, "config", "scene_tf.yaml"))
        src_xyz, src_rpy = pickup_source_in_world(scene)
        top_xyz, _ = pickup_platform_top_in_world(scene)
        self.assertAlmostEqual(src_xyz[2], top_xyz[2], places=6)
        self.assertEqual(src_rpy, [0.0, 0.0, 0.0])
        size = [0.80, 0.50, 0.32]
        xyz, rpy = pickup_box_pose(src_xyz, src_rpy, size)
        self.assertAlmostEqual(xyz[2] - size[2] / 2.0, top_xyz[2], places=6)
        self.assertEqual(rpy, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(xyz[0], top_xyz[0], places=6)
        self.assertAlmostEqual(xyz[1], top_xyz[1], places=6)


class TestUnitMeshesOnDisk(unittest.TestCase):
    def test_mesh_files_exist(self):
        for visual in (VISUAL_LOAFBRR, VISUAL_VINTAGE):
            path = os.path.join(GAZEBO_MODELS, visual, "meshes", "suitcase.stl")
            self.assertTrue(os.path.isfile(path), path)

    def test_unit_aabb_is_centered_and_scales_to_catalog(self):
        import numpy as np
        import trimesh

        size = np.array([0.55, 0.40, 0.25])
        for visual in (VISUAL_LOAFBRR, VISUAL_VINTAGE):
            path = os.path.join(GAZEBO_MODELS, visual, "meshes", "suitcase.stl")
            mesh = trimesh.load(path, force="mesh", skip_materials=True)
            extents = mesh.extents
            center = (mesh.bounds[0] + mesh.bounds[1]) * 0.5
            self.assertTrue(np.allclose(extents, 1.0, atol=1e-3), (visual, extents))
            self.assertTrue(np.allclose(center, 0.0, atol=1e-3), (visual, center))
            scaled = mesh.copy()
            scaled.apply_scale(size)
            self.assertTrue(
                np.allclose(scaled.extents, size, atol=1e-3),
                (visual, scaled.extents))
            self.assertAlmostEqual(
                float(scaled.bounds[0, 2]), -size[2] / 2.0, places=3)
            self.assertAlmostEqual(
                float(scaled.bounds[1, 2]), size[2] / 2.0, places=3)


def _stl_aabb(path):
    with open(path, "rb") as handle:
        handle.read(80)
        count = struct.unpack("<I", handle.read(4))[0]
        mins = [1e9, 1e9, 1e9]
        maxs = [-1e9, -1e9, -1e9]
        for _ in range(count):
            nums = struct.unpack("<12fH", handle.read(50))
            for vertex in (nums[3:6], nums[6:9], nums[9:12]):
                for axis in range(3):
                    mins[axis] = min(mins[axis], vertex[axis])
                    maxs[axis] = max(maxs[axis], vertex[axis])
    return mins, maxs


class TestWriteScaledStl(unittest.TestCase):
    def test_scales_synthetic_triangle(self):
        src = os.path.join(tempfile.mkdtemp(), "unit.stl")
        dest_dir = tempfile.mkdtemp()
        dest = scaled_stl_path("box_a", dest_dir)
        with open(src, "wb") as handle:
            handle.write(b"unit triangle".ljust(80, b" "))
            handle.write(struct.pack("<I", 1))
            handle.write(struct.pack(
                "<12fH",
                0.0, 0.0, 1.0,
                0.0, 0.0, 0.0,
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0))
        write_scaled_stl(src, dest, (2.0, 3.0, 4.0))
        mins, maxs = _stl_aabb(dest)
        self.assertEqual(mins, [0.0, 0.0, 0.0])
        self.assertEqual(maxs, [2.0, 3.0, 0.0])
        self.assertTrue(file_uri(dest).startswith("file://"))
        self.assertTrue(file_uri(dest).endswith("box_a.stl"))

    def test_real_mesh_aabb_matches_spawn_size(self):
        size = [0.77, 0.41, 0.26]
        src = source_stl_path(VISUAL_LOAFBRR, GAZEBO_MODELS)
        dest = os.path.join(tempfile.mkdtemp(), "scaled.stl")
        write_scaled_stl(src, dest, size)
        mins, maxs = _stl_aabb(dest)
        extents = [maxs[i] - mins[i] for i in range(3)]
        for axis in range(3):
            self.assertAlmostEqual(extents[axis], size[axis], places=3)
        for axis in range(3):
            self.assertAlmostEqual(mins[axis], -size[axis] / 2.0, places=3)
            self.assertAlmostEqual(maxs[axis], size[axis] / 2.0, places=3)


class TestPickupVisualSdf(unittest.TestCase):
    def test_box_kind_has_no_mesh(self):
        xml = pickup_visual_sdf(
            "pickup_box_1", [0.55, 0.40, 0.25], 8.0, VISUAL_LOAFBRR,
            visual_kind="box")
        root = ET.fromstring(xml)
        self.assertIsNone(root.find(".//visual/geometry/mesh"))
        visual = [float(v) for v in root.find(
            ".//visual/geometry/box/size").text.split()]
        self.assertEqual(visual, [0.55, 0.40, 0.25])

    def test_mesh_kind_uses_prebuilt_uri_and_writes_no_stl(self):
        dest_dir = tempfile.mkdtemp()
        xml = pickup_visual_sdf(
            "pickup_box_eval", [0.55, 0.40, 0.25], 8.0, VISUAL_LOAFBRR,
            visual_kind="mesh", models_root=GAZEBO_MODELS, scaled_dir=dest_dir)
        root = ET.fromstring(xml)
        vis_uri = root.find(".//visual/geometry/mesh/uri").text
        col_uri = root.find(".//collision/geometry/mesh/uri").text
        scale = [float(v) for v in root.find(
            ".//visual/geometry/mesh/scale").text.split()]
        self.assertEqual(vis_uri, sized_mesh_uri(VISUAL_LOAFBRR, "small"))
        self.assertEqual(col_uri, vis_uri)
        self.assertEqual(scale, [1.0, 1.0, 1.0])
        self.assertEqual(os.listdir(dest_dir), [])

    def test_mesh_kind_rejects_non_catalog_size(self):
        with self.assertRaises(ValueError):
            pickup_visual_sdf(
                "pickup_box_eval", [0.61, 0.44, 0.27], 8.0, VISUAL_LOAFBRR,
                visual_kind="mesh")


class TestSizedPickupAssets(unittest.TestCase):
    def test_six_models_exist_with_matching_visual_collision(self):
        uris = set()
        for visual in (VISUAL_LOAFBRR, VISUAL_VINTAGE):
            for tier, size, _mass, _cid in SIZE_TIERS:
                name = sized_model_name(visual, tier)
                stl = sized_stl_path(visual, tier, GAZEBO_MODELS)
                sdf_path = os.path.join(GAZEBO_MODELS, name, "model.sdf")
                self.assertTrue(os.path.isfile(stl), stl)
                self.assertTrue(os.path.isfile(sdf_path), sdf_path)
                mins, maxs = _stl_aabb(stl)
                for axis in range(3):
                    self.assertAlmostEqual(
                        maxs[axis] - mins[axis], size[axis], places=3)
                root = ET.parse(sdf_path).getroot()
                vis_uri = root.find(".//visual/geometry/mesh/uri").text
                col_uri = root.find(".//collision/geometry/mesh/uri").text
                self.assertEqual(vis_uri, sized_mesh_uri(visual, tier))
                self.assertEqual(col_uri, vis_uri)
                self.assertIsNone(root.find(".//collision/geometry/box"))
                self.assertNotIn(vis_uri, uris)
                uris.add(vis_uri)
        self.assertEqual(len(uris), 6)

    def test_top_footprint_uses_lid_band_not_full_aabb(self):
        """Vintage GT is the lid XY, not the handle/body vertex AABB."""
        stl = sized_stl_path(VISUAL_VINTAGE, "large", GAZEBO_MODELS)
        measure = mesh_top_footprint(stl)
        mins, maxs = _stl_aabb(stl)
        aabb = [maxs[i] - mins[i] for i in range(3)]
        self.assertLess(measure[0], aabb[0] - 0.05)
        self.assertLess(measure[1], aabb[1] - 0.02)
        self.assertAlmostEqual(measure[2], aabb[2], places=5)

    def test_vintage_large_gt_is_lid_band(self):
        stl = sized_stl_path(VISUAL_VINTAGE, "large", GAZEBO_MODELS)
        measure = mesh_top_footprint(stl)
        # Full AABB is 0.80 x 0.50; the lid band is ~0.61 x 0.43.
        self.assertAlmostEqual(measure[0], 0.61, places=2)
        self.assertAlmostEqual(measure[1], 0.43, places=2)
        self.assertAlmostEqual(measure[2], 0.32, places=3)

    def test_top_footprint_scales_linearly(self):
        small = mesh_top_footprint(
            sized_stl_path(VISUAL_VINTAGE, "small", GAZEBO_MODELS),
            z_band_frac=0.01, z_band_min=0.0)
        large = mesh_top_footprint(
            sized_stl_path(VISUAL_VINTAGE, "large", GAZEBO_MODELS),
            z_band_frac=0.01, z_band_min=0.0)
        self.assertAlmostEqual(large[0] / small[0], 0.80 / 0.55, places=2)
        self.assertAlmostEqual(large[1] / small[1], 0.50 / 0.40, places=2)

    def test_manifest_measure_size_matches_stl(self):
        manifest = load_sized_suitcases_manifest(GAZEBO_MODELS)
        self.assertEqual(len(manifest), 6)
        rec = manifest[sized_model_name(VISUAL_LOAFBRR, "small")]
        stl = sized_stl_path(VISUAL_LOAFBRR, "small", GAZEBO_MODELS)
        measure = mesh_top_footprint(stl)
        for axis in range(3):
            self.assertAlmostEqual(
                rec["measure_size"][axis], measure[axis], places=5)

    def test_size_tier_name_round_trip(self):
        self.assertEqual(size_tier_name([0.55, 0.40, 0.25]), "small")
        self.assertEqual(size_tier_name([0.70, 0.45, 0.28]), "medium")
        self.assertEqual(size_tier_name([0.80, 0.50, 0.32]), "large")
        self.assertIsNone(size_tier_name([0.61, 0.44, 0.27]))

