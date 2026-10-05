"""Directed deploy seed graph, built in memory from the deploy atlas."""

from __future__ import division

import importlib.util
import os
import unittest

ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
SCRIPT = os.path.join(ROOT, "scripts", "build_directed_seed_graph.py")
DEPLOY_NPZ = os.path.join(
    ROOT, "src", "luggage_planning", "data", "reachability_atlas_deploy",
    "s20_container_collision_aware.npz")
DEPLOY_SCENE = os.path.join(
    ROOT, "src", "luggage_description", "config", "scene_tf_deploy.yaml")

_spec = importlib.util.spec_from_file_location("build_directed_seed_graph", SCRIPT)
directed = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(directed)


class DirectedSeedGraphTest(unittest.TestCase):
    def test_deploy_empty_atlas_is_directed_and_opening_connected(self):
        self.assertTrue(DEPLOY_NPZ.endswith(os.path.join(
            "reachability_atlas_deploy", "s20_container_collision_aware.npz")))
        self.assertNotIn(
            os.path.join("data", "reachability_atlas", "s20_"), DEPLOY_NPZ)
        from luggage_planning.reachability_atlas import ReachabilityAtlas
        atlas = ReachabilityAtlas.load(DEPLOY_NPZ, DEPLOY_NPZ[:-4] + ".yaml")
        scene_hash, _urdf = directed.scene_identity(atlas, DEPLOY_SCENE)
        stored = atlas.meta["dependencies"]["scene_tf_hash"]
        self.assertEqual(scene_hash, stored)
        self.assertTrue(scene_hash.startswith("98313fb8"))
        records = directed.build_directed_records(atlas)
        self.assertEqual(records["opening_side"], "negative_x")
        self.assertGreater(records["node_index"].shape[0], 0)
        self.assertGreater(records["edge_src"].shape[0], 0)
        centers = records["node_center"]
        for src, dst, kind in zip(
                records["edge_src"], records["edge_dst"], records["edge_kind"]):
            self.assertGreaterEqual(int(src), 0)
            self.assertLess(int(src), centers.shape[0])
            self.assertLess(int(dst), centers.shape[0])
            dx = float(centers[int(dst), 0] - centers[int(src), 0])
            if int(kind) == directed.KIND_INWARD:
                self.assertGreater(dx, 0.0)
            elif int(kind) == directed.KIND_OUTWARD:
                self.assertLess(dx, 0.0)
        seen = set()
        for index, center, yaw in zip(
                records["node_index"], records["node_center"], records["node_yaw"]):
            key = tuple(int(v) for v in index)
            self.assertNotIn(key, seen)
            seen.add(key)
            result = atlas.query(
                float(center[0]), float(center[1]), float(center[2]),
                yaw=float(yaw))
            self.assertTrue(result.opening_connected)
            self.assertEqual(tuple(result.indices), key)

    def test_inward_is_deeper_for_a_negative_x_door(self):
        self.assertEqual(
            directed.edge_kind("negative_x", (0.0, 0.0, 0.5), (0.15, 0.0, 0.5)),
            directed.KIND_INWARD)
        self.assertEqual(
            directed.edge_kind("negative_x", (0.15, 0.0, 0.5), (0.0, 0.0, 0.5)),
            directed.KIND_OUTWARD)
        self.assertEqual(
            directed.edge_kind("negative_x", (0.0, 0.0, 0.5), (0.0, 0.15, 0.5)),
            directed.KIND_LATERAL)
        self.assertEqual(
            directed.edge_kind("negative_x", (0.0, 0.0, 0.5), (0.0, 0.0, 0.65)),
            directed.KIND_VERTICAL)


if __name__ == "__main__":
    unittest.main()
