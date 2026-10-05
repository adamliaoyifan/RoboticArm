#!/usr/bin/env python3
"""Candidate generators for the unified place-path selector.

Geometric variants are the existing occupancy polylines. Grid paths are
shortest paths on the reachability atlas. Both emit the same row dict.
"""

from __future__ import division

import math

from luggage_planning.grid_path_search import search_paths
from luggage_planning.occupancy_place_paths import (
    generate_path_variants,
    sweep_polyline,
)
from luggage_planning.reachable_graph import (
    build_reachable_graph,
    grid_disable_reason,
)


def _dist(a, b):
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


class GeometricVariantGenerator(object):
    def __init__(self, max_candidates, arm_radius, weights):
        self.max_candidates = int(max_candidates)
        self.arm_radius = float(arm_radius)
        self.weights = dict(weights)

    def rows(self, snapshot, slot, portal, payload_wdh, opening_tangent, yaw):
        index = int(slot.get("index", 0))
        target = tuple(float(v) for v in slot["target"])
        score = float(slot.get("score", 0.0))
        slot_yaw = float(slot.get("yaw", yaw))
        inner_h = float(snapshot.inner_size[2])
        variants = generate_path_variants(
            portal, target, snapshot=snapshot,
            opening_tangent=opening_tangent, inner_h=inner_h,
            max_candidates=self.max_candidates,
            payload_wdh=payload_wdh, yaw=slot_yaw,
            arm_radius=self.arm_radius,
            carry_margin=self.weights.get("carry_margin_m"),
            collision_pad=self.weights.get("collision_pad_m"),
            arm_overhead=self.weights.get("arm_overhead_m"),
            corridor_tool_z=self.weights.get("corridor_tool_z"))
        rows = []
        for variant in variants:
            swept = sweep_polyline(
                snapshot, variant.waypoints, payload_wdh, yaw=slot_yaw,
                arm_radius=self.arm_radius,
                collision_pad=self.weights.get("collision_pad_m"))
            rows.append(_row(
                index, score, variant.method, list(variant.waypoints),
                swept, slot_yaw, source="geometric", seeds=None))
        return rows


class GridPathGenerator(object):
    def __init__(self, continuity_rad, high_z_gain, diversity_penalty,
                 overlap_max, arm_radius):
        self.continuity_rad = float(continuity_rad)
        self.high_z_gain = float(high_z_gain)
        self.diversity_penalty = float(diversity_penalty)
        self.overlap_max = float(overlap_max)
        self.arm_radius = float(arm_radius)

    def rows(self, atlas, snapshot, slot, portal, payload_wdh, yaw,
             scene_tf_hash=None, urdf_hash=None, policy="off"):
        index = int(slot.get("index", 0))
        target = tuple(float(v) for v in slot["target"])
        score = float(slot.get("score", 0.0))
        slot_yaw = float(slot.get("yaw", yaw))
        disabled = grid_disable_reason(
            atlas, scene_tf_hash=scene_tf_hash, urdf_hash=urdf_hash,
            policy=policy)
        diag = {
            "disabled": disabled,
            "usable": 0,
            "rejects": {},
            "n_paths": 0,
        }
        if disabled:
            return [], diag
        graph = build_reachable_graph(
            atlas, yaw=slot_yaw, max_joint_step_rad=self.continuity_rad,
            snapshot=snapshot, payload_wdh=payload_wdh,
            arm_radius=self.arm_radius)
        diag["usable"] = len(graph)
        diag["rejects"] = dict(graph.reject_counts)
        if graph.disabled_reason or len(graph) == 0:
            diag["disabled"] = graph.disabled_reason or "GRID_DISABLED_NO_CELLS"
            return [], diag

        def chord_free(left, right):
            swept = sweep_polyline(
                snapshot, [left.center, right.center], payload_wdh,
                yaw=slot_yaw, arm_radius=self.arm_radius)
            return not swept.collides

        paths = search_paths(
            graph, atlas, tuple(float(v) for v in portal), target,
            continuity_rad=self.continuity_rad, high_z_gain=self.high_z_gain,
            diversity_penalty=self.diversity_penalty,
            overlap_max=self.overlap_max, chord_free=chord_free)
        rows = []
        for path in paths:
            waypoints = list(path.waypoints)
            if not waypoints or _dist(portal, waypoints[0]) > 1e-3:
                waypoints = [tuple(float(v) for v in portal)] + waypoints
            swept = sweep_polyline(
                snapshot, waypoints, payload_wdh, yaw=slot_yaw,
                arm_radius=self.arm_radius)
            row = _row(
                index, score, "grid_" + path.profile, waypoints, swept,
                slot_yaw, source="grid", seeds=path.seeds)
            row["cells"] = [list(cell) for cell in path.cells]
            rows.append(row)
        diag["n_paths"] = len(rows)
        return rows, diag


def _row(index, score, method, waypoints, swept, yaw, source, seeds):
    collides = bool(swept.collides)
    return {
        "slot_index": index,
        "slot_score": score,
        "method": method,
        "source": source,
        "waypoints": waypoints,
        "joint_seeds": seeds,
        "feasible": not collides,
        "reason": swept.reason or ("ok" if not collides else "occupancy_collision"),
        "min_clearance": float(swept.min_clearance),
        "min_aperture": float(getattr(swept, "min_aperture", swept.min_clearance)),
        "first_hit_cell": swept.first_hit_cell,
        "cartesian_fraction": None,
        "ik_ok": None,
        "yaw": float(yaw),
    }
