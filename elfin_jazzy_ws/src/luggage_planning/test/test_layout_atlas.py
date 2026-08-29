#!/usr/bin/env python3
"""Unit tests for layout_atlas (no roscore required)."""

import copy
import os
import sys
import unittest

import numpy as np


import luggage_planning.layout_atlas as la  # noqa: E402

UNKNOWN = la.UNKNOWN
UNREACHABLE = la.UNREACHABLE
MARGINAL = la.MARGINAL
REACHABLE = la.REACHABLE


def _mock_scene_tf(container_y=-1.5):
    return {
        "static_transforms": [
            {"parent": "world", "child": "container_link",
             "translation": [0.0, container_y, -0.86],
             "rotation_rpy": [0.0, 0.0, 0.0]},
            {"parent": "world", "child": "pedestal_link",
             "translation": [0.0, 0.0, 0.0],
             "rotation_rpy": [0.0, 0.0, 0.0]},
        ],
        "container": {"opening": {"side": "negative_x"}},
    }


def _mock_atlas_data(nx=4, ny=4, nz=3, nyaw=2, reachable_fraction=0.5):
    """Create mock atlas data with some reachable + opening_connected cells."""
    shape = (nx, ny, nz, nyaw)
    status = np.full(shape, UNREACHABLE, dtype=np.uint8)
    opening_connected = np.zeros(shape, dtype=np.bool_)
    joint_margin = np.full(shape, 0.3, dtype=np.float32)
    neighbor_conf = np.ones(shape, dtype=np.float32)

    # Mark center cells as REACHABLE + opening_connected.
    for ix in range(1, nx - 1):
        for iy in range(1, ny - 1):
            for iz in range(nz):
                for iyaw in range(nyaw):
                    status[ix, iy, iz, iyaw] = REACHABLE
                    opening_connected[ix, iy, iz, iyaw] = True
                    joint_margin[ix, iy, iz, iyaw] = 0.4
    # Mark one cell as MARGINAL.
    status[1, 1, 0, 0] = MARGINAL

    return {
        "status": status,
        "opening_connected": opening_connected,
        "joint_margin": joint_margin,
        "neighbor_confidence": neighbor_conf,
    }


def _mock_meta(opening_side="negative_x"):
    return {
        "grid": {
            "resolution_xyz": 0.15,
            "origin": [-0.3, -0.3, 0.0],
            "size": [4, 4, 3],
            "yaw_bins": [0.0, 1.57079632679],
        },
        "container": {"opening_side": opening_side},
    }


class TestEffectiveSceneTf(unittest.TestCase):
    def test_container_y_shifted(self):
        base = _mock_scene_tf(container_y=-1.5)
        eff = la.effective_scene_tf(base, base_y_offset=0.3)
        # Container moves -0.3 in Y: -1.5 - 0.3 = -1.8
        eff_y = la.baseline_container_y(eff)
        self.assertAlmostEqual(eff_y, -1.8, places=4)

    def test_baseline_unchanged(self):
        base = _mock_scene_tf(container_y=-1.5)
        original = copy.deepcopy(base)
        la.effective_scene_tf(base, base_y_offset=0.5)
        self.assertEqual(base, original)  # baseline not mutated

    def test_negative_offset(self):
        base = _mock_scene_tf(container_y=-1.5)
        eff = la.effective_scene_tf(base, base_y_offset=-0.3)
        eff_y = la.baseline_container_y(eff)
        self.assertAlmostEqual(eff_y, -1.2, places=4)


class TestReliableCoverage(unittest.TestCase):
    def test_only_reachable_and_connected(self):
        data = _mock_atlas_data()
        mask = la.reliable_coverage_mask(data)
        # MARGINAL cell should NOT be in mask.
        self.assertFalse(mask[1, 1, 0, 0])
        # REACHABLE + connected cell should be.
        self.assertTrue(mask[2, 2, 0, 0])

    def test_unreachable_excluded(self):
        data = _mock_atlas_data()
        mask = la.reliable_coverage_mask(data)
        self.assertFalse(mask[0, 0, 0, 0])  # edge = UNREACHABLE


class TestGridCompatibility(unittest.TestCase):
    def test_compatible(self):
        m = _mock_meta()
        ok, _ = la.verify_grid_compatibility(m, m)
        self.assertTrue(ok)

    def test_incompatible_size(self):
        m1 = _mock_meta()
        m2 = _mock_meta()
        m2["grid"]["size"] = [5, 4, 3]
        ok, _ = la.verify_grid_compatibility(m1, m2)
        self.assertFalse(ok)


class TestScoreFixedLayout(unittest.TestCase):
    def test_no_anchor(self):
        data = _mock_atlas_data(reachable_fraction=0)
        data["status"][:] = UNREACHABLE
        data["opening_connected"][:] = False
        score = la.score_fixed_layout(data, _mock_meta())
        self.assertFalse(score["has_opening_anchor"])
        self.assertEqual(score["score"], 0.0)

    def test_with_coverage(self):
        data = _mock_atlas_data()
        score = la.score_fixed_layout(data, _mock_meta())
        self.assertTrue(score["has_opening_anchor"])
        self.assertGreater(score["coverage_rate"], 0.0)
        self.assertGreater(score["score"], 0.0)
        self.assertIn("depth_front", score["region_stats"])


class TestGreedySetCover(unittest.TestCase):
    def test_single_slice(self):
        data = _mock_atlas_data()
        mask = la.reliable_coverage_mask(data)
        slices = [{"mask": mask, "base_y": 0.0,
                   "joint_margin": data["joint_margin"],
                   "neighbor_confidence": data["neighbor_confidence"],
                   "meta": _mock_meta()}]
        result = la.greedy_set_cover(slices)
        self.assertEqual(len(result["selected"]), 1)
        self.assertTrue(np.array_equal(result["union_mask"], mask))
        self.assertGreater(result["coverage_rate"], 0.0)

    def test_two_complementary_slices(self):
        nx, ny, nz, nyaw = 4, 4, 3, 2
        data = _mock_atlas_data(nx, ny, nz, nyaw)
        mask = la.reliable_coverage_mask(data)

        # Create a complementary mask (different cells).
        mask2 = np.zeros_like(mask)
        mask2[0, 0, :, :] = True  # cells not in mask1

        slices = [
            {"mask": mask, "base_y": 0.0,
             "joint_margin": data["joint_margin"],
             "neighbor_confidence": data["neighbor_confidence"],
             "meta": _mock_meta()},
            {"mask": mask2, "base_y": 0.3,
             "joint_margin": data["joint_margin"],
             "neighbor_confidence": data["neighbor_confidence"],
             "meta": _mock_meta()},
        ]
        result = la.greedy_set_cover(slices)
        self.assertEqual(len(result["selected"]), 2)
        self.assertGreater(result["coverage_rate"],
                           float(np.count_nonzero(mask)) / mask.size)

    def test_empty_slices(self):
        result = la.greedy_set_cover([])
        self.assertEqual(result["coverage_rate"], 0.0)


class TestEvaluateDecision(unittest.TestCase):
    def test_no_anchor_insufficient(self):
        decision = la.evaluate_decision(
            baseline_score={"coverage_rate": 0.0},
            best_fixed_score={"has_opening_anchor": False, "coverage_rate": 0.0},
            union_result={"coverage_rate": 0.0})
        self.assertEqual(decision["recommendation"], "y_axis_insufficient")

    def test_fixed_sufficient(self):
        decision = la.evaluate_decision(
            baseline_score={"coverage_rate": 0.2},
            best_fixed_score={"has_opening_anchor": True, "coverage_rate": 0.5},
            union_result={"coverage_rate": 0.55})
        self.assertEqual(decision["recommendation"], "fixed")

    def test_multi_stop_promising(self):
        decision = la.evaluate_decision(
            baseline_score={"coverage_rate": 0.2},
            best_fixed_score={"has_opening_anchor": True, "coverage_rate": 0.4},
            union_result={"coverage_rate": 0.7})
        self.assertEqual(decision["recommendation"], "multi_stop_promising")

    def test_no_improvement_insufficient(self):
        decision = la.evaluate_decision(
            baseline_score={"coverage_rate": 0.3},
            best_fixed_score={"has_opening_anchor": True, "coverage_rate": 0.31},
            union_result={"coverage_rate": 0.35})
        self.assertEqual(decision["recommendation"], "y_axis_insufficient")


# ── 3-axis (X/Y/Z) extensions ─────────────────────────────────────────


class TestEffectiveSceneTfXyz(unittest.TestCase):
    def test_xyz_shift(self):
        base = _mock_scene_tf(container_y=-1.5)  # container at [0, -1.5, -0.86]
        eff = la.effective_scene_tf_xyz(base, 0.3, -0.2, 0.1)
        # container moves by (-dx, -dy, -dz) = (-0.3, 0.2, -0.1)
        xyz = la.baseline_container_xyz(eff)
        self.assertAlmostEqual(xyz[0], -0.3, places=4)
        self.assertAlmostEqual(xyz[1], -1.3, places=4)
        self.assertAlmostEqual(xyz[2], -0.96, places=4)

    def test_baseline_untouched(self):
        base = _mock_scene_tf(container_y=-1.5)
        original = copy.deepcopy(base)
        la.effective_scene_tf_xyz(base, 0.5, 0.5, 0.5)
        self.assertEqual(base, original)

    def test_y_only_wrapper_matches_xyz(self):
        base = _mock_scene_tf(container_y=-1.5)
        eff1 = la.effective_scene_tf(base, 0.3)
        eff2 = la.effective_scene_tf_xyz(base, 0.0, 0.3, 0.0)
        self.assertEqual(la.baseline_container_xyz(eff1),
                         la.baseline_container_xyz(eff2))


class TestGreedySetCover3D(unittest.TestCase):
    def test_base_offset_preferred_xyz(self):
        data = _mock_atlas_data()
        mask = la.reliable_coverage_mask(data)
        slices = [{"mask": mask, "base_offset": [0.3, -0.2, 0.1],
                   "joint_margin": data["joint_margin"],
                   "neighbor_confidence": data["neighbor_confidence"],
                   "meta": _mock_meta()}]
        result = la.greedy_set_cover(slices)
        self.assertEqual(len(result["selected"]), 1)
        self.assertEqual(result["selected_offsets"], [(0.3, -0.2, 0.1)])
        covered = result["union_mask"]
        self.assertTrue(np.allclose(result["preferred_base_xyz"][covered],
                                    [0.3, -0.2, 0.1]))
        # Back-compat: preferred_base_y == dy.
        self.assertTrue(np.allclose(result["preferred_base_y"][covered], -0.2))

    def test_legacy_base_y_still_works(self):
        data = _mock_atlas_data()
        mask = la.reliable_coverage_mask(data)
        slices = [{"mask": mask, "base_y": 0.4,
                   "joint_margin": data["joint_margin"],
                   "neighbor_confidence": data["neighbor_confidence"],
                   "meta": _mock_meta()}]
        result = la.greedy_set_cover(slices)
        self.assertEqual(result["selected_offsets"], [(0.0, 0.4, 0.0)])

    def test_complementary_3d_slices(self):
        data = _mock_atlas_data()
        mask = la.reliable_coverage_mask(data)
        mask2 = np.zeros_like(mask)
        mask2[0, 0, :, :] = True  # cells not in mask1
        slices = [
            {"mask": mask, "base_offset": [0.3, 0.0, 0.0],
             "joint_margin": data["joint_margin"],
             "neighbor_confidence": data["neighbor_confidence"],
             "meta": _mock_meta()},
            {"mask": mask2, "base_offset": [-0.3, 0.2, 0.1],
             "joint_margin": data["joint_margin"],
             "neighbor_confidence": data["neighbor_confidence"],
             "meta": _mock_meta()},
        ]
        result = la.greedy_set_cover(slices)
        self.assertEqual(len(result["selected"]), 2)
        env_offsets = set(result["selected_offsets"])
        self.assertIn((0.3, 0.0, 0.0), env_offsets)
        self.assertIn((-0.3, 0.2, 0.1), env_offsets)


class TestBaseMovementEnvelope(unittest.TestCase):
    def test_empty(self):
        env = la.base_movement_envelope([])
        self.assertEqual(env["x"]["count"], 0)
        self.assertIsNone(env["x"]["min"])

    def test_per_axis_minmax(self):
        env = la.base_movement_envelope(
            [(0.3, -0.2, 0.1), (-0.1, 0.4, -0.3), (0.2, 0.0, 0.2)])
        self.assertAlmostEqual(env["x"]["min"], -0.1)
        self.assertAlmostEqual(env["x"]["max"], 0.3)
        self.assertAlmostEqual(env["y"]["min"], -0.2)
        self.assertAlmostEqual(env["y"]["max"], 0.4)
        self.assertAlmostEqual(env["z"]["min"], -0.3)
        self.assertAlmostEqual(env["z"]["max"], 0.2)
        self.assertEqual(env["x"]["count"], 3)


class TestBuildUnionArtifact3D(unittest.TestCase):
    def test_contains_preferred_base_xyz(self):
        data = _mock_atlas_data()
        mask = la.reliable_coverage_mask(data)
        slices = [{"mask": mask, "base_offset": [0.1, 0.2, 0.3],
                   "opening_connected": data["opening_connected"],
                   "joint_margin": data["joint_margin"],
                   "neighbor_confidence": data["neighbor_confidence"],
                   "meta": _mock_meta()}]
        sc = la.greedy_set_cover(slices)
        art = la.build_union_artifact(sc, slices)
        self.assertIn("preferred_base_xyz", art)
        self.assertEqual(art["preferred_base_xyz"].shape, mask.shape + (3,))
        self.assertIn("preferred_base_y", art)  # back-compat retained


class TestEvaluateDecisionMultiAxis(unittest.TestCase):
    def test_promising(self):
        d = la.evaluate_decision(
            baseline_score={"coverage_rate": 0.2},
            best_fixed_score={"has_opening_anchor": True, "coverage_rate": 0.4},
            union_result={"coverage_rate": 0.7}, multi_axis=True)
        self.assertEqual(d["recommendation"], "multi_axis_promising")

    def test_insufficient_no_anchor(self):
        d = la.evaluate_decision(
            baseline_score={"coverage_rate": 0.0},
            best_fixed_score={"has_opening_anchor": False, "coverage_rate": 0.0},
            union_result={"coverage_rate": 0.0}, multi_axis=True)
        self.assertEqual(d["recommendation"], "multi_axis_insufficient")

    def test_default_keeps_y_axis_strings(self):
        d = la.evaluate_decision(
            baseline_score={"coverage_rate": 0.0},
            best_fixed_score={"has_opening_anchor": False, "coverage_rate": 0.0},
            union_result={"coverage_rate": 0.0})
        self.assertEqual(d["recommendation"], "y_axis_insufficient")


if __name__ == "__main__":
    unittest.main()
