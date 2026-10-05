"""Path discovery and band accounting for the place-scene debug figure."""

import importlib.util
import os
import tempfile
import unittest


def _load_renderer():
    path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "scripts",
        "render_place_scene_3d.py"))
    spec = importlib.util.spec_from_file_location(
        "render_place_scene_3d", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RenderPlaceScene3dTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = _load_renderer()

    def test_resolve_accepts_run_dir_and_single_box_dir(self):
        with tempfile.TemporaryDirectory() as root:
            single = os.path.join(root, "single_box", "dumps")
            os.makedirs(os.path.join(single, "box_00_paths"))
            os.makedirs(os.path.join(single, "trial_00_PLANNING_BUDGET_EXHAUSTED"))
            from_run = self.mod.resolve_evidence(root)
            from_box = self.mod.resolve_evidence(os.path.join(root, "single_box"))
            self.assertEqual(from_run["root"], os.path.abspath(root))
            self.assertEqual(from_box["trial"], from_run["trial"])
            self.assertTrue(from_run["trial"].endswith(
                "trial_00_PLANNING_BUDGET_EXHAUSTED"))

    def test_missing_atlas_directory_is_an_error(self):
        with self.assertRaises(SystemExit):
            self.mod.atlas_dir_from_identity("not_a_real_atlas_dir")
        self.assertEqual(
            self.mod.atlas_dir_from_identity("ignored", "/tmp/atlas"),
            "/tmp/atlas")

    def test_resolve_rejects_a_directory_without_the_dump(self):
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaises(SystemExit):
                self.mod.resolve_evidence(root)

    def test_logged_bands_must_match_the_recompute(self):
        logged = (
            "slot id=c022_002_f1_h0 score=1.000 depth=1.300 layer=0.000 "
            "feasible=768 rejected={'outside_hull': 145} "
            "returned=768/768 bands=marginal:22,reachable:245,unknown:501 "
            "depth=0.200..1.300")
        prefix = (
            "returned=768/768 bands=marginal:22,reachable:245,unknown:501 "
            "depth=0.200..1.300")
        self.assertTrue(self.mod.placement_matches_log(logged, prefix, 768))
        self.assertFalse(self.mod.placement_matches_log(logged, prefix, 767))
        drifted = prefix.replace("marginal:22", "marginal:0")
        self.assertFalse(self.mod.placement_matches_log(logged, drifted, 768))

    def test_story_names_the_complete_route_and_the_unentered_band(self):
        text = self.mod.scene_story({
            "motion": {
                "n_reachable_tried": 10,
                "n_marginal_band": 22,
                "complete_route_slots": [656],
                "complete_route_edges_built": 7,
                "complete_route_failure": "RETREAT_TARGET_MISSING",
            },
        })
        self.assertIn("slot 656", text)
        self.assertIn("7 edges", text)
        self.assertIn("RETREAT_TARGET_MISSING", text)
        self.assertIn("22 marginal", text)

    def test_band_names_keep_marginal_distinct_from_reachable(self):
        self.assertEqual(self.mod._reason_of(
            {"feasible": True, "atlas_status": 3}), "reachable")
        self.assertEqual(self.mod._reason_of(
            {"feasible": True, "atlas_status": 2}), "marginal")
        self.assertEqual(self.mod._reason_of(
            {"feasible": True, "atlas_status": 0}), "unknown")
        self.assertEqual(self.mod._reason_of(
            {"feasible": False, "reason": "outside_hull"}), "outside_hull")

    def test_atlas_status_is_not_drawn_on_the_box_center(self):
        center = [0.005, 0.015, 0.6535717432182216]
        height = 0.24714348643644302
        contact_z = center[2] + 0.5 * height
        query_z = 0.827199
        layers, offset = self.mod.candidate_layers(
            [{
                "feasible": True,
                "atlas_status": 3,
                "center_container": center,
            }],
            [{
                "slot_pose": {"position": center},
                "query_offset": [0.0, 0.0, 0.0],
                "atlas_queries": [{
                    "query_xyz": [center[0], center[1], query_z],
                    "status": 3,
                }],
            }],
            height)
        self.assertEqual(offset, [0.0, 0.0, 0.0])
        self.assertAlmostEqual(layers["box_center"][0][2], center[2], places=6)
        self.assertAlmostEqual(
            layers["suction_contact"][0][2], contact_z, places=6)
        self.assertAlmostEqual(layers["reachable"][0][2], query_z, places=6)
        self.assertGreater(
            layers["reachable"][0][2] - layers["box_center"][0][2], 0.15)
        self.assertNotIn(center[2], [point[2] for point in layers["reachable"]])

    def test_hull_keeps_the_chamfer_and_cuts_the_door(self):
        from luggage_description.container_geometry import normalize_descriptor
        geometry = normalize_descriptor({
            "frame_id": "container_link",
            "length": 1.49,
            "width": 1.97,
            "floor_z": 0.53,
            "ceiling_z": 2.01,
            "chamfer": {
                "side": "positive_y",
                "floor_y": 0.55,
                "wall_y": 0.985,
                "wall_z": 0.90,
            },
        })
        faces, _poly, aper = self.mod._hull_faces(geometry, {
            "y0": -0.928, "y1": 0.398, "z0": 0.597, "z1": 2.009,
        })
        self.assertTrue(faces["chamfer"])
        self.assertTrue(faces["door_wall"])
        self.assertEqual(aper["y1"], 0.398)
        # The doorway is not a filled triangle of the -X pentagon.
        door_xs = {round(float(point[0]), 3) for tri in faces["door_wall"] for point in tri}
        self.assertEqual(door_xs, {round(-geometry.half_x, 3)})


if __name__ == "__main__":
    unittest.main()
