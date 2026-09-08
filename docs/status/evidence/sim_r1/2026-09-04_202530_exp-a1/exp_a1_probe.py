#!/usr/bin/env python3
"""EXP-A1 evidence-local NBV readiness probe.

This script intentionally lives under docs/status/evidence. It imports an
immutable git archive snapshot passed by --repo and does not modify production
source or start ROS.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import subprocess
import sys
import types
from pathlib import Path


BASE_REV = "408f6d5dd9aeb8536377f0f14d06155be7a904d5"


def stub_ament_index() -> None:
    root = types.ModuleType("ament_index_python")
    packages = types.ModuleType("ament_index_python.packages")

    class PackageNotFoundError(Exception):
        pass

    def get_package_share_directory(_name: str) -> str:
        raise PackageNotFoundError(_name)

    packages.PackageNotFoundError = PackageNotFoundError
    packages.get_package_share_directory = get_package_share_directory
    sys.modules.setdefault("ament_index_python", root)
    sys.modules.setdefault("ament_index_python.packages", packages)


def add_paths(repo: Path) -> None:
    for rel in ("src/luggage_planning", "src/luggage_description"):
        path = str(repo / rel)
        if path not in sys.path:
            sys.path.insert(0, path)
    stub_ament_index()


def record(rows: list[dict], probe: str, status: str, summary: str, **extra) -> None:
    row = {"probe": probe, "status": status, "summary": summary}
    row.update(extra)
    rows.append(row)


def finite_vector(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def strict_json_value(value):
    if isinstance(value, float):
        if math.isnan(value):
            return "NaN"
        if math.isinf(value):
            return "Infinity" if value > 0 else "-Infinity"
        return value
    if isinstance(value, dict):
        return {str(key): strict_json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [strict_json_value(item) for item in value]
    return value


def deterministic_payload(repo: Path) -> dict:
    add_paths(repo)
    from luggage_planning.cargo_nbv_planner import CargoNBVPlanner
    from luggage_planning.geometry_view_generator import (
        generate_uncertainty_aware_corridor_views,
    )
    from luggage_planning.interior_probe_planner import (
        evaluate_probe_termination,
        rank_probe_candidates,
    )
    from luggage_planning.interior_view_scorer import (
        CameraIntrinsics,
        RaycastConfig,
        SparseOccupancyGrid,
        UNKNOWN,
        score_candidate,
    )

    opening = {
        "opening_xyz": [0.0, 0.0, 1.0],
        "normal": [1.0, 0.0, 0.0],
        "lateral": [0.0, 1.0, 0.0],
        "up": [0.0, 0.0, 1.0],
        "aperture_width": 1.2,
        "aperture_height": 0.8,
        "inner_depth": 1.0,
        "geometry_version": 7,
        "source": "synthetic",
    }
    views = generate_uncertainty_aware_corridor_views(
        opening, [1.0, 0.0, 0.0, 0.0], observed_free_depth=0.5,
        num_lateral=3, min_depth=0.1, depth_step=0.2,
    )
    scored, states = rank_probe_candidates(
        views, frontier_points=[(0.1, 0.0, 0.8), (0.2, 0.2, 0.8)],
        used_indices=set(), coverage_radius=0.5,
    )
    grid = SparseOccupancyGrid(
        origin=(0.0, 0.0, 0.0), shape=(3, 1, 1), resolution=1.0,
        cells={(0, 0, 0): UNKNOWN},
    )
    candidate = {
        "candidate_id": "ray-a",
        "camera_xyz": (-1.0, 0.5, 0.5),
        "look_at": (0.5, 0.5, 0.5),
        "hard_feasible": True,
    }
    scored_candidate = score_candidate(
        candidate, grid, CameraIntrinsics(1, 1, 1.0, 1.0),
        config=RaycastConfig(max_range=3.0, pixel_stride=1),
    )
    planner = CargoNBVPlanner(
        candidates=[
            {"name": "a", "values": [0.0, 0.0]},
            {"name": "b", "values": [0.2, 0.0]},
        ],
        joint_names=["j1", "j2"],
        weights={"coverage_weight": 1.0, "path_weight": 0.0, "smooth_weight": 0.0},
        unknown_threshold=0.1,
        max_views=2,
    )
    selected = planner.plan_next(
        {"unknown_ratio": 0.5, "frontier_count": 2},
        [(0.1, 0.0, 0.8), (0.2, 0.2, 0.8)],
        [0.0, 0.0],
        0,
    )
    termination = evaluate_probe_termination(
        unknown_ratio=0.49,
        views_used=2,
        config={
            "termination": {"max_views": 5, "unknown_threshold": 0.1},
            "min_improvement": 0.05,
            "stagnation_limit": 2,
        },
        last_unknown=0.50,
        stagnant_count=1,
    )
    return {
        "candidate_ids": [view["candidate_id"] for view in views],
        "poses": [
            [round(value, 9) for value in view["camera_xyz"]]
            for view in views
        ],
        "ranked_indices": [item[1] for item in scored],
        "rank_states": states,
        "selected_index": selected["view_index"],
        "selected_message": selected["message"],
        "score": scored_candidate,
        "termination_reason": termination["reason"],
        "termination_done": termination["done"],
    }


def run_child(repo: Path) -> None:
    print(json.dumps(deterministic_payload(repo), sort_keys=True, separators=(",", ":")))


def probe(repo: Path, json_out: Path) -> int:
    add_paths(repo)
    rows: list[dict] = []

    from luggage_description.scene_tf_config_utils import point_inside_container_inner_box
    from luggage_planning.cargo_nbv_planner import CargoNBVPlanner
    from luggage_planning.geometry_view_generator import (
        generate_uncertainty_aware_corridor_views,
    )
    from luggage_planning.interior_probe_planner import (
        evaluate_probe_termination,
        rank_probe_candidates,
    )
    from luggage_planning.interior_view_scorer import (
        CameraIntrinsics,
        RaycastConfig,
        SparseOccupancyGrid,
        UNKNOWN,
        score_candidate,
    )
    from luggage_planning.layout_atlas import verify_grid_compatibility
    from luggage_planning.reachability_atlas import (
        REACHABLE,
        ReachabilityAtlas,
    )

    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        [str(repo / "src/luggage_planning"), str(repo / "src/luggage_description")]
    )
    child_cmd = [sys.executable, str(Path(__file__)), "--repo", str(repo), "--child"]
    first = subprocess.check_output(child_cmd, env=env, text=True).strip()
    second = subprocess.check_output(child_cmd, env=env, text=True).strip()
    record(
        rows,
        "fresh_process_determinism",
        "PASS" if first == second else "FAIL",
        "Two fresh Python processes produced byte-identical deterministic payloads.",
        evidence={"first": json.loads(first), "second": json.loads(second)},
    )

    intrinsics = CameraIntrinsics(1, 1, 1.0, 1.0)
    config = RaycastConfig(max_range=3.0, pixel_stride=1)
    grid_a = SparseOccupancyGrid(
        origin=(0.0, 0.0, 0.0), shape=(3, 1, 1), resolution=1.0,
        cells={(0, 0, 0): UNKNOWN},
    )
    candidate_a = {
        "candidate_id": "sees-unknown",
        "camera_xyz": (-1.0, 0.5, 0.5),
        "look_at": (0.5, 0.5, 0.5),
        "hard_feasible": True,
    }
    candidate_b = {
        "candidate_id": "misses-unknown",
        "camera_xyz": (-1.0, 2.5, 0.5),
        "look_at": (0.5, 2.5, 0.5),
        "hard_feasible": True,
    }
    score_a = score_candidate(candidate_a, grid_a, intrinsics, config=config)
    score_b = score_candidate(candidate_b, grid_a, intrinsics, config=config)
    rank_ok = score_a["score"] > score_b["score"]
    planner = CargoNBVPlanner(
        candidates=[{"values": [0.0]}, {"values": [1.0]}],
        joint_names=["j1"],
        weights={"coverage_weight": 1.0, "path_weight": 0.0, "smooth_weight": 0.0},
        unknown_threshold=0.1,
        max_views=2,
    )
    coverage_a = planner._coverage_score([(0, 0, 0), (1, 0, 0)], [0.0])
    coverage_b = planner._coverage_score([(0, 0, 0), (1, 0, 0)], [1.0])
    nbv_candidate_specific = coverage_a != coverage_b
    record(
        rows,
        "candidate_specific_information_gain",
        "FAIL" if not nbv_candidate_specific else ("PASS" if rank_ok else "FAIL"),
        "interior_view_scorer is candidate-specific, but CargoNBVPlanner._coverage_score ignores candidate values.",
        evidence={
            "interior_view_scorer_scores": {
                "sees_unknown": score_a["score"],
                "misses_unknown": score_b["score"],
            },
            "higher_gain_ranked_first": rank_ok,
            "cargo_nbv_coverage_scores": [coverage_a, coverage_b],
            "cargo_nbv_candidate_specific": nbv_candidate_specific,
        },
    )

    invalid_results = {}
    try:
        score_candidate(
            {"candidate_id": "nan", "camera_xyz": (math.nan, 0, 0), "look_at": (0, 0, 0)},
            grid_a,
            intrinsics,
            config=config,
        )
        invalid_results["nan_pose"] = "accepted"
    except ValueError as exc:
        invalid_results["nan_pose"] = str(exc)
    try:
        score_candidate(
            {"candidate_id": "inf", "camera_xyz": (math.inf, 0, 0), "look_at": (0, 0, 0)},
            grid_a,
            intrinsics,
            config=config,
        )
        invalid_results["inf_pose"] = "accepted"
    except ValueError as exc:
        invalid_results["inf_pose"] = str(exc)
    repeated = [
        {"candidate_id": "dup", "camera_xyz": (-1, 0.5, 0.5), "look_at": (0, 0.5, 0.5)},
        {"candidate_id": "dup", "camera_xyz": (-1, 2.5, 0.5), "look_at": (0, 2.5, 0.5)},
    ]
    repeated_scores = [
        score_candidate(item, grid_a, intrinsics, config=config, source_index=index)
        for index, item in enumerate(repeated)
    ]
    termination_cases = {
        "exhausted_views": evaluate_probe_termination(
            0.9, 3,
            {"termination": {"max_views": 3, "unknown_threshold": 0.1}, "min_improvement": 0.01, "stagnation_limit": 2},
        ),
        "unknown_threshold": evaluate_probe_termination(
            0.05, 1,
            {"termination": {"max_views": 3, "unknown_threshold": 0.1}, "min_improvement": 0.01, "stagnation_limit": 2},
        ),
        "stagnation": evaluate_probe_termination(
            0.99, 2,
            {"termination": {"max_views": 5, "unknown_threshold": 0.1}, "min_improvement": 0.05, "stagnation_limit": 2},
            last_unknown=1.0,
            stagnant_count=1,
        ),
    }
    record(
        rows,
        "invalid_and_termination_classification",
        "FAIL",
        "NaN/Inf poses and termination reasons are stable, but repeated IDs, candidate dimensions, and quaternion validation are not enforced by the baseline contracts.",
        evidence={
            "invalid_results": invalid_results,
            "repeated_candidate_ids_preserved": [item["candidate_id"] for item in repeated_scores],
            "termination_cases": termination_cases,
            "missing_contracts": [
                "candidate dimensions are not a validated candidate field",
                "orientation_quat is not validated by score_candidate",
                "duplicate candidate_id is not rejected",
            ],
        },
    )

    opening = {
        "opening_xyz": [0.0, 0.0, 1.0],
        "normal": [1.0, 0.0, 0.0],
        "lateral": [0.0, 1.0, 0.0],
        "up": [0.0, 0.0, 1.0],
        "aperture_width": 1.2,
        "aperture_height": 0.8,
        "inner_depth": 1.0,
        "geometry_version": 7,
        "source": "synthetic",
    }
    views = generate_uncertainty_aware_corridor_views(
        opening, [1.0, 0.0, 0.0, 0.0], observed_free_depth=0.5,
        num_lateral=3, min_depth=0.1, depth_step=0.2,
    )
    pose_ok = bool(views) and all(
        finite_vector(view["camera_xyz"])
        and abs(math.sqrt(sum(float(q) ** 2 for q in view["orientation_quat"])) - 1.0) <= 1e-6
        and view.get("valid_tilt") is True
        and view.get("valid_geometry") is True
        and view.get("aperture_clearance", 0.0) >= 0.0
        for view in views
    )
    record(
        rows,
        "generated_pose_constraints",
        "PASS" if pose_ok else "FAIL",
        "Synthetic corridor views are finite, normalized, and carry tilt/aperture feasibility fields.",
        evidence={"view_count": len(views), "first_view": views[0] if views else None},
    )

    hull_config = {
        "static_transforms": [
            {
                "parent": "world",
                "child": "container_link",
                "translation": [0.0, 0.0, 0.0],
                "rotation_rpy": [0.0, 0.0, 0.0],
            }
        ],
        "container": {
            "inner": {
                "schema_version": 1,
                "length": 2.0,
                "width": 2.0,
                "floor_z": 0.0,
                "ceiling_z": 2.0,
                "chamfer": {
                    "side": "positive_y",
                    "floor_y": 0.5,
                    "wall_y": 1.0,
                    "wall_z": 1.0,
                },
            }
        },
    }
    inside_aabb_outside_hull = [0.0, 0.8, 0.1]
    hull_rejected = not point_inside_container_inner_box(
        inside_aabb_outside_hull, hull_config
    )
    record(
        rows,
        "seven_face_hull_containment",
        "PASS" if hull_rejected else "FAIL",
        "A point inside the inner AABB but outside the +Y chamfered seven-face hull is rejected.",
        evidence={"point": inside_aabb_outside_hull, "rejected": hull_rejected},
    )

    import numpy as np

    shape = (3, 3, 3, 1)
    status = np.full(shape, REACHABLE, dtype=np.uint8)
    opening_connected = np.ones(shape, dtype=np.bool_)
    seeds = np.zeros(shape + (1, 6), dtype=np.float64)
    solution_count = np.ones(shape, dtype=np.uint8)
    joint_margin = np.ones(shape, dtype=np.float32)
    manipulability = np.ones(shape, dtype=np.float32)
    confidence = np.ones(shape, dtype=np.float32)
    meta = {
        "atlas_version": "2.0",
        "grid": {
            "frame": "container_link",
            "resolution_xyz": 0.1,
            "origin": [0.0, 0.0, 0.0],
            "size": [3, 3, 3],
            "yaw_bins": [0.0],
        },
        "dependencies": {"scene_tf_hash": "scene-a", "urdf_hash": "urdf-a"},
    }
    atlas = ReachabilityAtlas.from_builder(
        status=status,
        opening_connected=opening_connected,
        contact_ik=np.ones(shape, dtype=np.bool_),
        transit_ik=np.ones(shape, dtype=np.bool_),
        contact_seeds=seeds,
        transit_seeds=seeds,
        solution_count=solution_count,
        joint_margin=joint_margin,
        manipulability=manipulability,
        neighbor_confidence=confidence,
        meta=meta,
    )
    valid_query = atlas.query(0.15, 0.15, 0.15, 0.0)._asdict()
    missing_query = atlas.query(10.0, 10.0, 10.0, 0.0)._asdict()
    version_ok = atlas.verify_version("scene-a", "urdf-a")
    version_bad = atlas.verify_version("scene-b", "urdf-a")
    grid_ok = verify_grid_compatibility(meta, dict(meta))
    changed_meta = json.loads(json.dumps(meta))
    changed_meta["grid"]["size"] = [4, 3, 3]
    grid_bad = verify_grid_compatibility(meta, changed_meta)
    record(
        rows,
        "atlas_hash_revision_stamp_correlation",
        "BLOCKED",
        "Atlas lookup/version checks are deterministic for existing scene/URDF hashes, but the schema lacks production geometry_hash, map_revision, and stamp correlation fields.",
        evidence={
            "valid_query": valid_query,
            "missing_query": missing_query,
            "version_ok": version_ok,
            "version_bad": version_bad,
            "layout_grid_ok": grid_ok,
            "layout_grid_bad": grid_bad,
            "missing_fields": ["geometry_hash", "map_revision", "source_acquisition_stamp"],
        },
    )

    component_map = {
        "cargo_nbv_planner": {
            "reset": "CargoNBVPlanner.reset exists but uses mutable visited indices.",
            "propose": "CargoNBVPlanner.plan_next can propose joints but not ExplorationContext/Snapshot/PolicyProposal.",
            "observe": "No observe(outcome) hook; visited is mutated during plan_next.",
            "reuse": "needs SIM-R1-2 adapter and candidate-specific gain repair",
        },
        "geometry_view_generator": {
            "reset": "stateless",
            "propose": "generates plain candidate dicts",
            "observe": "not applicable",
            "reuse": "adapter needed; TCIG/hard-gate correlation must be enforced by coordinator",
        },
        "interior_view_scorer": {
            "reset": "stateless",
            "propose": "score_candidate provides candidate-specific plain diagnostics",
            "observe": "not applicable",
            "reuse": "reusable behind coordinator hard gates",
        },
        "smart_explore_termination": {
            "reset": "caller-owned counters",
            "propose": "termination decision helper only",
            "observe": "caller passes last_unknown/stagnant_count",
            "reuse": "adapter needed; metric absence currently fails open in phase0_low_fov",
        },
        "interior_probe_planner": {
            "reset": "caller-owned used indices/counters",
            "propose": "rank_probe_candidates ranks views",
            "observe": "termination helper consumes observations",
            "reuse": "adapter needed for immutable snapshot and duplicate-ID gates",
        },
        "reachability_atlas": {
            "reset": "loaded immutable-ish atlas object",
            "propose": "query/filter/annotate candidates",
            "observe": "not applicable",
            "reuse": "adapter needed; lacks production geometry_hash/map_revision/stamp correlation",
        },
        "layout_atlas": {
            "reset": "stateless",
            "propose": "offline layout scoring/set cover",
            "observe": "not applicable",
            "reuse": "offline or coordinator input only",
        },
    }
    record(
        rows,
        "exploration_policy_mapping",
        "BLOCKED",
        "No audited component directly implements ExplorationPolicy.reset/propose/observe; SIM-R1-2 needs an adapter/coordinator layer.",
        evidence=component_map,
    )

    result = {
        "base_revision": BASE_REV,
        "repo": str(repo),
        "python": sys.version,
        "platform": platform.platform(),
        "test_command": "python3 exp_a1_probe.py --repo <audit_root> --json-out <probe_results.json>",
        "audit_outcome": "pass",
        "nbv_readiness": "blocked",
        "rows": rows,
        "summary": {
            "pass": sum(1 for row in rows if row["status"] == "PASS"),
            "fail": sum(1 for row in rows if row["status"] == "FAIL"),
            "blocked": sum(1 for row in rows if row["status"] == "BLOCKED"),
            "not_applicable": sum(1 for row in rows if row["status"] == "NOT_APPLICABLE"),
        },
    }
    result = strict_json_value(result)
    json_out.write_text(
        json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result["summary"], sort_keys=True))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True, type=Path)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--child", action="store_true")
    args = parser.parse_args()
    repo = args.repo.resolve()
    if args.child:
        run_child(repo)
        return 0
    if args.json_out is None:
        raise SystemExit("--json-out is required unless --child is set")
    return probe(repo, args.json_out)


if __name__ == "__main__":
    raise SystemExit(main())
