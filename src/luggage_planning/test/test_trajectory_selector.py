#!/usr/bin/env python3
"""A0: reachable-grid paths, unified selector costs, joint-path checks."""

from __future__ import division

import time
import unittest

import numpy as np

from luggage_description.container_geometry import cuboid_from_inner_size
from luggage_planning.grid_path_search import search_paths
from luggage_planning.joint_path import (
    JointPathError,
    densify,
    max_joint_step,
    validate_joint_path,
)
from luggage_planning.occupancy_place_paths import PLACE_PATH_INFEASIBLE
from luggage_planning.reachability_atlas import ReachabilityAtlas
from luggage_planning.reachable_graph import (
    build_reachable_graph,
    grid_disable_reason,
)
from luggage_planning.trajectory_selector import (
    TrajectorySelector,
    score_row,
)
from luggage_planning.trajectory_timing import time_parameterize_cartesian


def _surface(nx=20, ny=10, rev=3):
    inner = [1.0, 0.5, 1.0]
    desc = cuboid_from_inner_size(inner, floor_z=0.0).descriptor()
    return {
        "geometry_hash": desc["geometry_hash"],
        "geometry_descriptor": desc,
        "map_revision": rev,
        "resolution": 0.05,
        "nx": nx,
        "ny": ny,
        "inner_size": inner,
        "floor_z": 0.0,
        "center_base": [0.0, 0.0, 0.5],
        "yaw": 0.0,
        "height": [[0.0] * ny for _ in range(nx)],
        "state": [["free"] * ny for _ in range(nx)],
        "confidence": [["none"] * ny for _ in range(nx)],
    }


def _occupy_ix(surface, ix, height=0.40):
    for iy in range(surface["ny"]):
        surface["state"][ix][iy] = "occupied"
        surface["height"][ix][iy] = height
        surface["confidence"][ix][iy] = "sensor"


def _toy_atlas(break_ix=None, scene="scene", urdf="urdf"):
    nx, ny, nz, nyaw = 4, 3, 3, 1
    shape = (nx, ny, nz, nyaw)
    status = np.full(shape, 1, np.uint8)
    opening = np.zeros(shape, dtype=bool)
    ik = np.zeros(shape, dtype=bool)
    seeds = np.zeros(shape + (1, 6), dtype=np.float64)
    for ix in range(nx):
        for iz, z_joint in ((1, 0.0), (2, 0.4)):
            status[ix, 1, iz, 0] = 3
            opening[ix, 1, iz, 0] = True
            ik[ix, 1, iz, 0] = True
            seeds[ix, 1, iz, 0, 0, 0] = 0.1 * ix
            seeds[ix, 1, iz, 0, 0, 1] = z_joint
    if break_ix is not None:
        seeds[break_ix, 1, 1, 0, 0, 0] = 5.0
        seeds[break_ix, 1, 2, 0, 0, 0] = 5.0
    data = {
        "status": status,
        "opening_connected": opening,
        "contact_seeds": seeds,
        "transit_seeds": seeds,
        "contact_ik": ik,
        "transit_ik": ik,
        "solution_count": np.ones(shape, np.uint8),
        "neighbor_confidence": np.ones(shape, np.float32),
        "joint_margin": np.ones(shape, np.float32),
        "manipulability": np.ones(shape, np.float32),
    }
    meta = {
        "grid": {
            "resolution_xyz": 0.15,
            "origin": [0.0, 0.0, 0.0],
            "size": [nx, ny, nz],
            "yaw_bins": [0.0],
        },
        "container": {"opening_side": "negative_x"},
        "dependencies": {"scene_tf_hash": scene, "urdf_hash": urdf},
    }
    return ReachabilityAtlas(data, meta)


class TestReachableGraph(unittest.TestCase):
    def test_hash_mismatch_disables_grid(self):
        atlas = _toy_atlas()
        reason = grid_disable_reason(
            atlas, scene_tf_hash="other", urdf_hash="urdf", policy="strict")
        self.assertEqual(reason, "GRID_DISABLED_SCENE_TF")
        self.assertEqual(grid_disable_reason(None), "GRID_DISABLED_NO_ATLAS")
        self.assertEqual(
            grid_disable_reason(
                atlas, scene_tf_hash="scene", urdf_hash="", policy="strict"),
            "GRID_DISABLED_URDF")
        self.assertEqual(
            grid_disable_reason(
                atlas, scene_tf_hash="scene", urdf_hash="urdf", policy="strict"),
            "")

    def test_blocked_cell_and_joint_step_are_counted(self):
        atlas = _toy_atlas()
        blocked = {(2, 1, 1, 0)}

        def blocked_fn(index, _center):
            return "occupancy" if index in blocked else ""

        graph = build_reachable_graph(
            atlas, max_joint_step_rad=1.2, blocked_fn=blocked_fn)
        self.assertNotIn((2, 1, 1, 0), graph.nodes)
        self.assertGreater(graph.reject_counts.get("occupancy", 0), 0)
        self.assertTrue(all(node.index[3] == 0 for node in graph.nodes.values()))
        jumped = build_reachable_graph(
            _toy_atlas(break_ix=2), max_joint_step_rad=0.5)
        self.assertGreater(jumped.reject_counts.get("joint_step", 0), 0)
        neighbors = [item[0] for item in jumped.adjacency.get((1, 1, 1, 0), [])]
        self.assertNotIn((2, 1, 1, 0), neighbors)


class TestGridSearch(unittest.TestCase):
    def test_profiles_stay_distinct_and_repeatable(self):
        atlas = _toy_atlas()
        graph = build_reachable_graph(atlas, max_joint_step_rad=1.2)
        start = (-0.2, 0.225, 0.375)
        goal = (0.525, 0.225, 0.225)
        first = search_paths(graph, atlas, start, goal, high_z_gain=8.0)
        second = search_paths(graph, atlas, start, goal, high_z_gain=8.0)
        profiles = [path.profile for path in first]
        self.assertGreaterEqual(len(first), 2)
        self.assertEqual(profiles, [path.profile for path in second])
        cells = [tuple(path.cells) for path in first]
        self.assertEqual(cells, [tuple(path.cells) for path in second])
        short = next(path for path in first if path.profile == "short")
        high = next(path for path in first if path.profile == "high")
        overlap = len(set(short.cells) & set(high.cells)) / float(
            len(set(short.cells) | set(high.cells)))
        self.assertLessEqual(overlap, 0.7)
        self.assertGreater(
            sum(node.center[2] for node in high.nodes),
            sum(node.center[2] for node in short.nodes))

    def test_broken_corridor_has_no_path(self):
        atlas = _toy_atlas(break_ix=2)
        graph = build_reachable_graph(atlas, max_joint_step_rad=0.3)
        paths = search_paths(
            graph, atlas, (-0.2, 0.225, 0.225), (0.525, 0.225, 0.225),
            continuity_rad=0.3)
        self.assertEqual(paths, [])


class TestSelectorCosts(unittest.TestCase):
    def _cfg(self):
        return {
            "w_placement": 1.0, "w_dimension": 1.0, "w_safety": 1.5,
            "w_efficiency": 0.5, "safety_ref_m": 0.10,
            "placement_score_ref": 1.0, "joint_travel_ref_rad": 3.0,
        }

    def test_dimension_and_safety_differ(self):
        narrow = {"feasible": True, "slot_score": 0.5, "min_clearance": 0.20,
                  "free_gap_m": 0.0, "waypoints": [(0, 0, 1), (1, 0, 1)],
                  "cartesian_fraction": 1.0, "joint_seeds": None}
        score_row(narrow, self._cfg(), 0.4)
        self.assertGreater(narrow["C_dim"], 0.9)
        self.assertLess(narrow["C_safe"], 0.1)

    def test_placement_cost_ignores_other_slots(self):
        cfg = self._cfg()
        good = {"feasible": True, "slot_score": 0.8, "min_clearance": 0.2,
                "free_gap_m": None, "waypoints": [(0, 0, 1), (1, 0, 1)],
                "cartesian_fraction": 1.0}
        other = dict(good)
        other["slot_score"] = 0.2
        score_row(good, cfg, 0.1)
        score_row(other, cfg, 0.1)
        before = (good["C_place"], other["C_place"])
        worse = dict(good)
        worse["slot_score"] = 0.05
        score_row(worse, cfg, 0.1)
        score_row(good, cfg, 0.1)
        score_row(other, cfg, 0.1)
        self.assertEqual(before, (good["C_place"], other["C_place"]))
        self.assertLess(good["C_place"], other["C_place"])

    def test_direct_blocked_still_selects_clear_variant(self):
        surface = _surface()
        _occupy_ix(surface, 10, 0.40)
        selector = TrajectorySelector(config={
            "mode": "unified", "inflate_m": 0.0, "arm_radius_m": 0.0,
            "max_candidates_per_slot": 4, "probe_budget": 3,
            "hash_policy": "off",
        })
        winner, rows, reason, _snap = selector.select(
            surface, [{"index": 0, "score": 1.0, "target": (0.35, 0.0, 0.50)}],
            portal=(-0.35, 0.0, 0.50), payload_wdh=(0.08, 0.08, 0.15),
            expected_hash=surface["geometry_hash"], expected_revision=3)
        self.assertEqual(reason, "")
        self.assertIsNotNone(winner)
        self.assertTrue(winner["method"].startswith("clear"), winner["method"])
        direct = next(row for row in rows if row["method"] == "direct")
        self.assertFalse(direct["feasible"])
        self.assertNotIn("BIN_FULL", reason)

    def test_second_slot_and_fail_closed_name(self):
        surface = _surface()
        _occupy_ix(surface, 10, 0.90)
        selector = TrajectorySelector(config={
            "inflate_m": 0.0, "arm_radius_m": 0.0, "max_candidates_per_slot": 4,
        })
        winner, rows, reason, _snap = selector.select(
            surface,
            [
                {"index": 0, "score": 0.9, "target": (0.35, 0.0, 0.50)},
                {"index": 1, "score": 0.4, "target": (-0.20, 0.0, 0.80)},
            ],
            portal=(-0.35, 0.0, 0.80), payload_wdh=(0.08, 0.08, 0.15))
        self.assertEqual(reason, "")
        self.assertEqual(winner["slot_index"], 1)
        self.assertEqual({row["slot_index"] for row in rows}, {0, 1})
        blocked = _surface()
        for ix in range(blocked["nx"]):
            _occupy_ix(blocked, ix, 0.95)
        _winner, _rows, empty_reason, _snap = selector.select(
            blocked, [{"index": 0, "score": 1.0, "target": (0.35, 0.0, 0.20)}],
            portal=(-0.35, 0.0, 0.20), payload_wdh=(0.4, 0.4, 0.4))
        self.assertEqual(empty_reason, PLACE_PATH_INFEASIBLE)
        self.assertNotIn("BIN_FULL", empty_reason)
        self.assertNotIn("PLACE_CANDIDATE_EXHAUSTED", empty_reason)

    def test_hash_mismatch_keeps_geometric(self):
        surface = _surface()
        selector = TrajectorySelector(config={
            "hash_policy": "strict", "inflate_m": 0.0, "arm_radius_m": 0.0,
        })
        winner, _rows, reason, _snap = selector.select(
            surface, [{"index": 0, "score": 0.8, "target": (0.1, 0.0, 0.4)}],
            portal=(-0.2, 0.0, 0.5), payload_wdh=(0.08, 0.08, 0.1),
            atlas=_toy_atlas(), scene_tf_hash="other", urdf_hash="urdf")
        self.assertEqual(reason, "")
        self.assertEqual(winner["source"], "geometric")
        self.assertEqual(
            winner["diagnostics"]["slots"][0]["disabled"],
            "GRID_DISABLED_SCENE_TF")

    def test_selection_id_is_stable(self):
        surface = _surface()
        selector = TrajectorySelector(config={
            "inflate_m": 0.0, "arm_radius_m": 0.0,
        })
        ids = []
        for _ in range(20):
            winner, _rows, reason, _snap = selector.select(
                surface, [{"index": 0, "score": 0.7, "target": (0.1, 0.0, 0.4)}],
                portal=(-0.2, 0.0, 0.5), payload_wdh=(0.08, 0.08, 0.1),
                expected_hash=surface["geometry_hash"], expected_revision=3)
            self.assertEqual(reason, "")
            ids.append(winner["selection_id"])
        self.assertEqual(len(set(ids)), 1)

    def test_probe_budget_stops(self):
        surface = _surface()
        calls = []

        def probe(row):
            calls.append(row["method"])
            row["ik_ok"] = True
            row["cartesian_fraction"] = 1.0

        selector = TrajectorySelector(config={
            "inflate_m": 0.0, "arm_radius_m": 0.0, "probe_budget": 3,
            "max_candidates_per_slot": 8,
        })
        _winner, rows, reason, _snap = selector.select(
            surface,
            [
                {"index": 0, "score": 0.9, "target": (0.2, 0.0, 0.4)},
                {"index": 1, "score": 0.5, "target": (-0.1, 0.0, 0.4)},
            ],
            portal=(-0.3, 0.0, 0.5), payload_wdh=(0.08, 0.08, 0.1),
            probe_fn=probe)
        self.assertEqual(reason, "")
        self.assertLessEqual(len(calls), 3)
        deferred = [row for row in rows if row.get("reason") == "probe_budget_deferred"]
        self.assertTrue(deferred or len(calls) <= 3)

    def test_selector_latency_without_probe(self):
        surface = _surface()
        selector = TrajectorySelector(config={
            "inflate_m": 0.0, "arm_radius_m": 0.0, "probe_budget": 3,
        })
        samples = []
        for _ in range(15):
            t0 = time.perf_counter()
            selector.select(
                surface, [{"index": 0, "score": 0.7, "target": (0.1, 0.0, 0.4)}],
                portal=(-0.2, 0.0, 0.5), payload_wdh=(0.08, 0.08, 0.1))
            samples.append(time.perf_counter() - t0)
        samples.sort()
        p95 = samples[int(0.95 * (len(samples) - 1))]
        self.assertLess(p95, 0.25)


class TestJointPath(unittest.TestCase):
    def test_branch_jump_rejected_and_dense_step_bounded(self):
        with self.assertRaises(JointPathError) as raised:
            densify([[0.0] * 6, [3.0] + [0.0] * 5], continuity_rad=1.2, sample_rad=0.02)
        self.assertEqual(raised.exception.reason, "joint_branch_jump")
        dense = densify([[0.0] * 6, [0.05] + [0.0] * 5], 1.2, sample_rad=0.02)
        self.assertGreater(len(dense), 2)
        for left, right in zip(dense, dense[1:]):
            self.assertLessEqual(max_joint_step(left, right), 0.02 + 1e-9)

    def test_sweep_rejection_and_timing(self):
        def fk(_q):
            return (0.0, 0.0, 2.0)

        class Sweep(object):
            collides = True
            reason = "outside_hull"

        with self.assertRaises(JointPathError) as raised:
            validate_joint_path(
                [[0.0] * 6, [0.04] + [0.0] * 5], 1.2, fk_fn=fk, sweep_fn=lambda _p: Sweep())
        self.assertEqual(raised.exception.reason, "outside_hull")
        dense = densify([[0.0] * 6, [0.2] + [0.0] * 5], 1.2, sample_rad=0.02)
        timed = time_parameterize_cartesian(dense, 0.5, 0.5)
        self.assertGreater(timed.duration, 0.0)
        self.assertLessEqual(timed.v_peak, timed.v_max + 1e-6)


if __name__ == "__main__":
    unittest.main()


def test_requested_probe_cannot_be_bypassed_by_grid_seeds():
    from luggage_planning.trajectory_selector import _apply_probe_gate
    row = {"source": "grid", "joint_seeds": [[0.0] * 6], "feasible": True}
    calls = []
    def unavailable(candidate):
        calls.append(candidate)
    _apply_probe_gate(row, unavailable, 0.95)
    assert len(calls) == 1
    assert row["feasible"] is False
    assert row["reason"] == "PLACE_PROBE_UNAVAILABLE"
