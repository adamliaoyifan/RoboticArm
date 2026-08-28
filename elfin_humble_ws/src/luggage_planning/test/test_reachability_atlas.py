#!/usr/bin/env python3
"""Unit tests for reachability_atlas (no roscore required)."""

import os
import sys
import math
import tempfile
import unittest

import numpy as np
import yaml


from luggage_planning.reachability_atlas import (  # noqa: E402
    MARGINAL,
    REACHABLE,
    UNKNOWN,
    UNREACHABLE,
    ReachabilityAtlas,
)


def _make_mock_atlas(nx=4, ny=4, nz=3, nyaw=2, resolution=0.15):
    """Build a small mock atlas for testing."""
    reachable = np.zeros((nx, ny, nz, nyaw), dtype=np.bool_)
    contact_ik = np.zeros_like(reachable)
    transit_ik = np.zeros_like(reachable)
    seeds = np.zeros((nx, ny, nz, nyaw, 6), dtype=np.float64)
    joint_margin = np.full((nx, ny, nz, nyaw), 0.5, dtype=np.float32)
    manipulability = np.zeros((nx, ny, nz, nyaw), dtype=np.float32)

    # Mark the center region as reachable.
    for ix in range(1, nx - 1):
        for iy in range(1, ny - 1):
            for iz in range(nz):
                for iyaw in range(nyaw):
                    reachable[ix, iy, iz, iyaw] = True
                    contact_ik[ix, iy, iz, iyaw] = True
                    transit_ik[ix, iy, iz, iyaw] = True
                    seeds[ix, iy, iz, iyaw] = [0.0, -1.0, 1.0, 0.0, 1.5, 0.3]
                    joint_margin[ix, iy, iz, iyaw] = 0.3
                    manipulability[ix, iy, iz, iyaw] = 0.5

    # Set a marginal cell (low joint margin).
    joint_margin[1, 1, 0, 0] = 0.05

    meta = {
        "atlas_version": "1.0",
        "grid": {
            "frame": "container_link",
            "resolution_xyz": resolution,
            "origin": [-0.3, -0.3, 0.0],
            "size": [nx, ny, nz],
            "yaw_bins": [0.0, math.pi / 2],
            "transit_clearance": 0.30,
        },
        "dependencies": {
            "robot_model": "elfin_s20_with_camera",
            "scene_tf_hash": "a1b2c3d4e5f6",
            "urdf_hash": "aabbccdd",
            "ik_link": "suction_contact_frame",
            "ik_group": "elfin_arm",
        },
        "stats": {
            "total_cells": nx * ny * nz * nyaw,
            "reachable_cells": int(np.count_nonzero(reachable)),
            "reachability_rate": float(np.count_nonzero(reachable)) / (nx * ny * nz * nyaw),
        },
    }
    return ReachabilityAtlas.from_builder(
        reachable, contact_ik, transit_ik, seeds,
        joint_margin, manipulability, meta)


def _make_v2_atlas():
    """Build a v2 atlas with one cell for each status."""
    shape = (5, 5, 5, 2)
    status = np.full(shape, UNKNOWN, dtype=np.uint8)
    opening_connected = np.zeros(shape, dtype=np.bool_)
    contact_ik = np.zeros(shape, dtype=np.bool_)
    transit_ik = np.zeros(shape, dtype=np.bool_)
    contact_seeds = np.zeros(shape + (2, 6), dtype=np.float64)
    transit_seeds = np.zeros(shape + (2, 6), dtype=np.float64)
    solution_count = np.zeros(shape, dtype=np.uint8)
    joint_margin = np.zeros(shape, dtype=np.float32)
    manipulability = np.zeros(shape, dtype=np.float32)
    confidence = np.zeros(shape, dtype=np.float32)

    # Cell centers for indices 1, 2, and 3 are 0.15, 0.25, and 0.35.
    unreachable_idx = (2, 2, 2, 0)
    marginal_idx = (1, 2, 2, 0)
    reachable_idx = (3, 2, 2, 0)
    status[unreachable_idx] = UNREACHABLE
    status[marginal_idx] = MARGINAL
    status[reachable_idx] = REACHABLE
    confidence[unreachable_idx] = 0.95

    for idx in (marginal_idx, reachable_idx):
        opening_connected[idx] = True
        contact_ik[idx] = True
        transit_ik[idx] = True
        solution_count[idx] = 2
        joint_margin[idx] = 0.05 if idx == marginal_idx else 0.4
        manipulability[idx] = 0.6
        confidence[idx] = 1.0
        contact_seeds[idx + (0,)] = [1, 2, 3, 4, 5, 6]
        contact_seeds[idx + (1,)] = [11, 12, 13, 14, 15, 16]
        transit_seeds[idx + (0,)] = [-1, -2, -3, -4, -5, -6]
        transit_seeds[idx + (1,)] = [-11, -12, -13, -14, -15, -16]

    meta = {
        "atlas_version": "2.0",
        "grid": {
            "frame": "container_link",
            "resolution_xyz": 0.1,
            "origin": [0.0, 0.0, 0.0],
            "size": list(shape[:3]),
            "yaw_bins": [0.0, math.pi / 2],
        },
        "query": {
            "yaw_tolerance": 0.1,
            "hard_reject_min_neighbor_confidence": 0.9,
            "hard_reject_interior_fraction": 0.1,
        },
    }
    return ReachabilityAtlas.from_builder(
        status=status,
        opening_connected=opening_connected,
        contact_ik=contact_ik,
        transit_ik=transit_ik,
        contact_seeds=contact_seeds,
        transit_seeds=transit_seeds,
        solution_count=solution_count,
        joint_margin=joint_margin,
        manipulability=manipulability,
        neighbor_confidence=confidence,
        meta=meta,
    )


class TestReachabilityAtlas(unittest.TestCase):
    def setUp(self):
        self.atlas = _make_mock_atlas()
        # origin = [-0.3, -0.3, 0.0], resolution=0.15
        # cell 0: x in [-0.3, -0.15), cell 1: [-0.15, 0.0), cell 2: [0.0, 0.15), cell 3: [0.15, 0.3)
        # reachable cells: ix in [1,2], iy in [1,2], all iz

    def test_reachable_center(self):
        # Cell (1,1,0) is reachable.
        ok, seed = self.atlas.is_reachable(-0.1, -0.1, 0.0, yaw=0.0)
        self.assertTrue(ok)
        self.assertIsNotNone(seed)
        self.assertEqual(len(seed), 6)

    def test_unreachable_edge(self):
        # Cell (0,0,0) is unreachable (edge).
        ok, seed = self.atlas.is_reachable(-0.25, -0.25, 0.0, yaw=0.0)
        self.assertFalse(ok)
        self.assertIsNone(seed)

    def test_out_of_bounds(self):
        ok, seed = self.atlas.is_reachable(10.0, 10.0, 10.0, yaw=0.0)
        self.assertFalse(ok)
        self.assertIsNone(seed)

    def test_yaw_nearest_bin(self):
        # yaw=0.1 should snap to bin 0 (yaw=0.0).
        ok0, _ = self.atlas.is_reachable(-0.1, -0.1, 0.0, yaw=0.1)
        # yaw=1.5 should snap to bin 1 (yaw=pi/2 ~ 1.5708).
        ok1, _ = self.atlas.is_reachable(-0.1, -0.1, 0.0, yaw=1.5)
        self.assertEqual(ok0, ok1)  # both in the reachable center

    def test_filter_candidates(self):
        candidates = [
            {"center_x": -0.1, "center_y": -0.1, "center_z": 0.0, "yaw": 0.0, "id": "a"},
            {"center_x": -0.25, "center_y": -0.25, "center_z": 0.0, "yaw": 0.0, "id": "b"},
            {"center_x": 10.0, "center_y": 10.0, "center_z": 0.0, "yaw": 0.0, "id": "c"},
        ]
        reachable, unreachable = self.atlas.filter_candidates(candidates)
        self.assertEqual(len(reachable), 1)
        self.assertEqual(len(unreachable), 2)
        self.assertEqual(reachable[0]["id"], "a")
        self.assertEqual(unreachable[0]["atlas_reason"], "atlas_unreachable")
        self.assertEqual(unreachable[1]["atlas_reason"], "out_of_bounds")
        self.assertIsNotNone(reachable[0]["atlas_seed"])

    def test_marginal(self):
        # Cell (1,1,0) has joint_margin=0.05 -> marginal.
        self.assertTrue(self.atlas.is_marginal(-0.1, -0.1, 0.0, yaw=0.0, threshold=0.1))
        # Cell (2,2,0) has joint_margin=0.3 -> not marginal.
        self.assertFalse(self.atlas.is_marginal(0.1, 0.1, 0.0, yaw=0.0, threshold=0.1))

    def test_stats(self):
        stats = self.atlas.stats()
        self.assertEqual(stats["total_cells"], 4 * 4 * 3 * 2)
        self.assertGreater(stats["reachable_cells"], 0)
        self.assertGreater(stats["reachability_rate"], 0.0)

    def test_v1_from_builder_mixed_arguments_remain_supported(self):
        data = self.atlas
        rebuilt = ReachabilityAtlas.from_builder(
            data._reachable, data._contact_ik, data._transit_ik, data._seeds,
            data._joint_margin, data._manipulability, meta=data.meta)
        ok, seed = rebuilt.is_reachable(-0.1, -0.1, 0.0)
        self.assertTrue(ok)
        self.assertIsNotNone(seed)

    def test_verify_version_match(self):
        ok, msg = self.atlas.verify_version(
            scene_tf_hash="a1b2c3d4e5f6", urdf_hash="aabbccdd")
        self.assertTrue(ok)

    def test_verify_version_mismatch(self):
        ok, msg = self.atlas.verify_version(
            scene_tf_hash="different", urdf_hash="aabbccdd")
        self.assertFalse(ok)
        self.assertIn("mismatch", msg)

    def test_save_load_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            npz = os.path.join(tmp, "atlas.npz")
            meta = os.path.join(tmp, "atlas.yaml")
            self.atlas.save(npz, meta)
            loaded = ReachabilityAtlas.load(npz, meta)
            ok, seed = loaded.is_reachable(-0.1, -0.1, 0.0, yaw=0.0)
            self.assertTrue(ok)
            self.assertIsNotNone(seed)
            self.assertEqual(loaded.stats()["total_cells"],
                             self.atlas.stats()["total_cells"])
            ok2, _ = loaded.verify_version(
                scene_tf_hash="a1b2c3d4e5f6", urdf_hash="aabbccdd")
            self.assertTrue(ok2)


class TestReachabilityAtlasV2(unittest.TestCase):
    def setUp(self):
        self.atlas = _make_v2_atlas()

    def test_query_tri_state_and_separate_seeds(self):
        unreachable = self.atlas.query(0.25, 0.25, 0.25, 0.0)
        marginal = self.atlas.query(0.15, 0.25, 0.25, 0.0)
        reachable = self.atlas.query(0.35, 0.25, 0.25, 0.0)

        self.assertEqual(unreachable.status, UNREACHABLE)
        self.assertEqual(marginal.status, MARGINAL)
        self.assertEqual(reachable.status, REACHABLE)
        self.assertTrue(reachable.opening_connected)
        self.assertEqual(reachable.solution_count, 2)
        self.assertEqual(reachable.contact_seeds[0], [1, 2, 3, 4, 5, 6])
        self.assertEqual(reachable.transit_seeds[0], [-1, -2, -3, -4, -5, -6])
        self.assertNotEqual(
            reachable.contact_seeds[0], reachable.transit_seeds[0])

    def test_out_of_bounds_and_yaw_mismatch_are_unknown(self):
        out_of_bounds = self.atlas.query(-0.01, 0.25, 0.25, 0.0)
        yaw_mismatch = self.atlas.query(0.25, 0.25, 0.25, 0.2)

        self.assertEqual(out_of_bounds.status, UNKNOWN)
        self.assertEqual(out_of_bounds.reason, "out_of_bounds")
        self.assertFalse(out_of_bounds.hard_reject_safe)
        self.assertEqual(yaw_mismatch.status, UNKNOWN)
        self.assertEqual(yaw_mismatch.reason, "yaw_mismatch")
        self.assertFalse(yaw_mismatch.hard_reject_safe)

    def test_yaw_tolerance_is_wrapped_and_configurable(self):
        wrapped = self.atlas.query(
            0.25, 0.25, 0.25, 2.0 * math.pi - 0.05)
        strict = self.atlas.query(
            0.25, 0.25, 0.25, 0.05, yaw_tolerance=0.01)
        self.assertEqual(wrapped.status, UNREACHABLE)
        self.assertAlmostEqual(wrapped.yaw_error, 0.05)
        self.assertEqual(strict.status, UNKNOWN)

    def test_hard_reject_requires_confident_interior_unreachable(self):
        safe = self.atlas.query(0.25, 0.25, 0.25, 0.0)
        self.assertTrue(safe.hard_reject_safe)

        # Same cell, but exactly on a cell face.
        cell_boundary = self.atlas.query(0.20, 0.25, 0.25, 0.0)
        self.assertEqual(cell_boundary.status, UNREACHABLE)
        self.assertFalse(cell_boundary.hard_reject_safe)

        # An edge-grid cell is never safe to hard reject.
        self.atlas._status[0, 2, 2, 0] = UNREACHABLE
        self.atlas._neighbor_confidence[0, 2, 2, 0] = 1.0
        grid_edge = self.atlas.query(0.05, 0.25, 0.25, 0.0)
        self.assertFalse(grid_edge.hard_reject_safe)

        # Confidence below the configured threshold is also insufficient.
        self.atlas._neighbor_confidence[2, 2, 2, 0] = 0.89
        low_confidence = self.atlas.query(0.25, 0.25, 0.25, 0.0)
        self.assertFalse(low_confidence.hard_reject_safe)

    def test_marginal_is_reachable_for_legacy_api(self):
        ok, seed = self.atlas.is_reachable(0.15, 0.25, 0.25, 0.0)
        self.assertTrue(ok)
        self.assertEqual(seed, [1, 2, 3, 4, 5, 6])
        # A v2 cell can be marginal because it used a repair branch or has
        # weak neighbor agreement even when its joint margin is healthy.
        self.assertTrue(
            self.atlas.is_marginal(0.15, 0.25, 0.25, 0.0, threshold=0.01))

    def test_filter_candidates_keeps_legacy_split_and_annotations(self):
        candidates = [
            {"id": "reachable", "center_x": 0.35, "center_y": 0.25,
             "center_z": 0.25, "yaw": 0.0},
            {"id": "unreachable", "center_x": 0.25, "center_y": 0.25,
             "center_z": 0.25, "yaw": 0.0},
        ]
        reachable, unreachable = self.atlas.filter_candidates(candidates)
        self.assertEqual([c["id"] for c in reachable], ["reachable"])
        self.assertEqual([c["id"] for c in unreachable], ["unreachable"])
        self.assertEqual(reachable[0]["atlas_status"], REACHABLE)
        self.assertEqual(len(reachable[0]["contact_atlas_seeds"]), 2)
        self.assertTrue(unreachable[0]["atlas_hard_reject_safe"])

    def test_annotate_candidates_only_rejects_safe_unreachable(self):
        candidates = [
            {"center_x": 0.25, "center_y": 0.25, "center_z": 0.25, "yaw": 0.0},
            {"center_x": 0.20, "center_y": 0.25, "center_z": 0.25, "yaw": 0.0},
            {"center_x": 10.0, "center_y": 0.25, "center_z": 0.25, "yaw": 0.0},
        ]
        accepted, rejected = self.atlas.annotate_candidates(candidates)
        self.assertEqual(len(rejected), 1)
        self.assertEqual(len(accepted), 2)
        self.assertFalse(accepted[0]["atlas_hard_reject_safe"])
        self.assertEqual(accepted[1]["atlas_status"], UNKNOWN)

    def test_v2_roundtrip_saves_complete_schema(self):
        with tempfile.TemporaryDirectory() as tmp:
            npz_path = os.path.join(tmp, "atlas.npz")
            meta_path = os.path.join(tmp, "atlas.yaml")
            self.atlas.save(npz_path, meta_path)

            with np.load(npz_path, allow_pickle=False) as saved:
                for key in (
                        "status", "opening_connected", "contact_seeds",
                        "transit_seeds", "solution_count", "joint_margin",
                        "manipulability", "neighbor_confidence"):
                    self.assertIn(key, saved.files)
                self.assertEqual(saved["status"].dtype, np.uint8)
                self.assertEqual(saved["opening_connected"].dtype, np.bool_)
                self.assertEqual(saved["contact_seeds"].shape[-2:], (2, 6))
                # Compatibility arrays remain available to v1 consumers.
                self.assertIn("reachable", saved.files)
                self.assertIn("seed_joints", saved.files)

            loaded = ReachabilityAtlas.load(npz_path, meta_path)
            result = loaded.query(0.35, 0.25, 0.25, 0.0)
            self.assertEqual(result.status, REACHABLE)
            self.assertEqual(len(result.contact_seeds), 2)
            self.assertEqual(loaded.stats()["nseed"], 2)
            with open(meta_path, "r", encoding="utf-8") as stream:
                saved_meta = yaml.safe_load(stream)
            self.assertEqual(saved_meta["schema_version"], 2)

    def test_loads_v1_npz_and_normalizes_fields(self):
        shape = (3, 3, 3, 1)
        reachable = np.zeros(shape, dtype=np.bool_)
        reachable[1, 1, 1, 0] = True
        seeds = np.zeros(shape + (6,), dtype=np.float64)
        seeds[1, 1, 1, 0] = [1, 2, 3, 4, 5, 6]
        contact_ik = reachable.copy()
        transit_ik = reachable.copy()
        margin = np.full(shape, 0.5, dtype=np.float32)
        manip = np.full(shape, 0.2, dtype=np.float32)
        meta = {
            "atlas_version": "1.0",
            "grid": {
                "resolution_xyz": 0.1,
                "origin": [0.0, 0.0, 0.0],
                "size": [3, 3, 3],
                "yaw_bins": [0.0],
            },
        }

        with tempfile.TemporaryDirectory() as tmp:
            npz_path = os.path.join(tmp, "v1.npz")
            meta_path = os.path.join(tmp, "v1.yaml")
            np.savez_compressed(
                npz_path, reachable=reachable, contact_ik=contact_ik,
                transit_ik=transit_ik, seed_joints=seeds,
                joint_margin=margin, manipulability=manip)
            with open(meta_path, "w", encoding="utf-8") as stream:
                yaml.safe_dump(meta, stream)

            loaded = ReachabilityAtlas.load(npz_path, meta_path)
            reachable_result = loaded.query(0.15, 0.15, 0.15, 0.0)
            unreachable_result = loaded.query(0.05, 0.05, 0.05, 0.0)
            self.assertEqual(reachable_result.status, REACHABLE)
            self.assertEqual(
                reachable_result.contact_seeds[0], [1, 2, 3, 4, 5, 6])
            self.assertEqual(unreachable_result.status, UNREACHABLE)
            # V1 has no confidence evidence, so it cannot hard reject safely.
            self.assertFalse(unreachable_result.hard_reject_safe)


if __name__ == "__main__":
    unittest.main()
