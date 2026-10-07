#!/usr/bin/env python3
"""Placement-point robustness and the capacity-test manifest.

Candidates come from ``generate_candidates`` / ``solve_placement``. Atlas
labels a point and does not decide it. A point passes only when the
corridor lattice returns a certified route with retreat and return.

Subcommands:

* ``freeze`` writes the safety pins, the stratified scan list, the six
  capacity manifests, and the offline geometric counts.
* ``plan`` runs each scan point alone on isolated MoveIt, then one
  same-request fallback from a failed point to a passed point.
* ``summarize`` writes the rates and the depth-by-lateral heatmap.

Safety pins are asserted, not edited. A drifted threshold aborts.
"""

from __future__ import division

import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS",
             "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import json
import math
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
HERE = os.path.dirname(os.path.abspath(__file__))
for _path in (
        HERE,
        os.path.join(ROOT, "src", "luggage_packing"),
        os.path.join(ROOT, "src", "luggage_packing", "scripts"),
        os.path.join(ROOT, "src", "luggage_planning"),
        os.path.join(ROOT, "src", "luggage_description"),
        os.path.join(ROOT, "st09_contracts"),
):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from capacity_orientation_analysis import (  # noqa: E402
    SOLVER_PARAMS,
    SceneContext,
    apply_placement,
    floor_prior_surface,
    make_validator,
)
from luggage_packing.placement_reachability import (  # noqa: E402
    make_atlas_annotate,
)
from luggage_packing.placement_solver import (  # noqa: E402
    SAME_LAYER_CLEARANCE_M,
    _default_params,
    generate_candidates,
    solve_placement,
)
from luggage_planning.corridor_lattice import (  # noqa: E402
    BRANCH_DISTANCE_RAD,
    LINE_MAX_JUMP_RAD,
    LINE_STEP_M,
    WRIST_SINGULARITY_MARGIN_RAD,
)
from luggage_planning.grid_path_search import ROUTE_TOTAL_WALL_S  # noqa: E402
from st09_contracts.attachment import ATTACHMENT_MAX_AGE_NS  # noqa: E402

SCENE = os.path.join(
    ROOT, "src", "luggage_description", "config", "scene_tf_deploy.yaml")
ATLAS_CONFIG = os.path.join(
    ROOT, "src", "luggage_planning", "config",
    "atlas_runtime_deploy_sim.yaml")
CONTRACT = os.path.join(
    ROOT, "st09_contracts", "resolved_planning_contract.json")
OUT = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_placement_robustness_20261005")

# Catalog carryon is the 20-inch class already spawned in this scene.
BOXES = {
    "20in": {
        "catalog_id": "carryon",
        "wdh": [0.55, 0.40, 0.25],
    },
    "standard": {
        "catalog_id": "standard",
        "wdh": [0.70, 0.45, 0.28],
    },
}
# Cycle-start joints from the lattice three-box home anchor
# (g3_lattice_60s_attach box 0, region home).
CYCLE_START_Q = [
    -1.389163, -1.751519, -1.672512, -1.28724, 1.576975, -1.391346]
YAWS = (0.0, math.pi / 2.0)
SCAN_CAP = 24
SIZE_JITTER_M = 0.01
PICK_XY_M = 0.015
PICK_Z_M = 0.005
PICK_YAW_RAD = 0.05
CAPACITY_SEEDS = {
    "20in_nominal": [4101, 4102, 4103],
    "20in_size_jitter": [4201, 4202, 4203],
    "20in_pick_offset": [4301, 4302, 4303],
    "standard_nominal": [4401, 4402, 4403],
    "mixed_alternating": [4501, 4502, 4503],
    "stacking_enabled": [4601, 4602, 4603],
}
REQUIRED_EDGE_CLASSES = ("TRAVERSE", "INSERT", "DESCEND", "RETREAT", "RETURN")
STOP_REASONS = (
    "BIN_FULL_GEOMETRIC",
    "NO_COMPLETE_ROUTE",
    "PLANNING_BUDGET_EXHAUSTED",
    "EXECUTION_FAILED",
    "PERCEPTION_FAILED",
    "PLACEMENT_VERIFY_FAILED",
    "STACK_NOT_ENABLED",
)


def _dump(name, document):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
    return path


def _unit(seed, index):
    """Deterministic unit interval. Same seed and index always match."""
    x = (int(seed) + 0x9E3779B9 * (int(index) + 1)) & 0xFFFFFFFF
    x ^= (x >> 16)
    x = (x * 0x7FEB352D) & 0xFFFFFFFF
    x ^= (x >> 15)
    x = (x * 0x846CA68B) & 0xFFFFFFFF
    x ^= (x >> 16)
    return (x & 0xFFFFFF) / float(0x1000000)


def _span(seed, index, half):
    return (_unit(seed, index) * 2.0 - 1.0) * float(half)


def assert_safety_pins():
    with open(CONTRACT, encoding="utf-8") as handle:
        contract = json.load(handle)
    cartesian = float(
        contract["time_parameterization"]["cartesian_min_fraction"])
    wall = float(contract["planning_budgets"]["route_total_wall_s"]["value"])
    clearance = float(
        contract["collision_margins"]["same_layer_clearance_m"]["value"])
    problems = []
    if cartesian != 0.95:
        problems.append("cartesian_min_fraction %s" % cartesian)
    if wall != 60.0 or float(ROUTE_TOTAL_WALL_S) != 60.0:
        problems.append("route budget %s / %s" % (wall, ROUTE_TOTAL_WALL_S))
    if clearance != 0.05 or float(SAME_LAYER_CLEARANCE_M) != 0.05:
        problems.append("same_layer %s / %s" % (
            clearance, SAME_LAYER_CLEARANCE_M))
    if float(_default_params()["min_support_ratio"]) != 0.6:
        problems.append("min_support_ratio drifted")
    if float(LINE_MAX_JUMP_RAD) != 0.3:
        problems.append("jump %s" % LINE_MAX_JUMP_RAD)
    if float(BRANCH_DISTANCE_RAD) != 0.25:
        problems.append("branch %s" % BRANCH_DISTANCE_RAD)
    if float(WRIST_SINGULARITY_MARGIN_RAD) != 0.50:
        problems.append("wrist %s" % WRIST_SINGULARITY_MARGIN_RAD)
    if float(LINE_STEP_M) != 0.02:
        problems.append("line step %s" % LINE_STEP_M)
    if int(ATTACHMENT_MAX_AGE_NS) < 60_000_000_000:
        problems.append("attachment window %s ns is under the 60 s budget"
                        % ATTACHMENT_MAX_AGE_NS)
    if problems:
        raise SystemExit("safety pins drifted: %s" % "; ".join(problems))
    return {
        "cartesian_min_fraction": cartesian,
        "route_total_wall_s": wall,
        "same_layer_clearance_m": clearance,
        "min_support_ratio": 0.6,
        "support_tol_m": 0.05,
        "line_step_m": float(LINE_STEP_M),
        "line_max_jump_rad": float(LINE_MAX_JUMP_RAD),
        "branch_distance_rad": float(BRANCH_DISTANCE_RAD),
        "wrist_singularity_margin_rad": float(WRIST_SINGULARITY_MARGIN_RAD),
        "attachment_max_age_ns": int(ATTACHMENT_MAX_AGE_NS),
        "collision_checking": True,
        "corridor_planner": "lattice",
        "stop_reasons": list(STOP_REASONS),
    }


def _tertile_edges(values):
    ordered = sorted(float(v) for v in values)
    count = len(ordered)
    if count == 0:
        return 0.0, 0.0
    return ordered[count // 3], ordered[(2 * count) // 3]


def _band(value, low, high, names):
    if float(value) < float(low):
        return names[0]
    if float(value) < float(high):
        return names[1]
    return names[2]


def _solver_params():
    params = dict(SOLVER_PARAMS)
    params["top_n"] = 100000
    params["keep_rejected"] = 0
    params["payload_symmetry"] = "none"
    return params


def _feasible(ctx, surface, size, placed):
    totals = {}
    candidates = generate_candidates(
        surface, size, allowed_yaws=list(YAWS), params=_solver_params(),
        candidate_validator=make_validator(ctx, placed, size),
        totals=totals)
    feasible = [row for row in candidates if row.get("feasible")]
    return feasible, totals


def _annotate(ctx, store, size, candidates):
    # The sim atlas config is strict and refuses a scene-hash mismatch.
    # Labels use the warn policy so a mismatch stays visible and does not
    # drop the point. The lattice, not this label, decides the route.
    cfg = dict(ctx.atlas_cfg)
    cfg["hash_policy"] = "warn"
    from luggage_planning.atlas_runtime import load_runtime_atlas
    atlas, diag = load_runtime_atlas(
        cfg, box_size=list(size), scene_tf_hash=ctx.scene_hash)
    del store
    hook = make_atlas_annotate(
        atlas, ctx.floor_z, ctx.inner_size[2],
        score_weight=0.0, hard_reject=False, dump_only=True,
        query_offset=ctx.atlas_cfg.get("query_offset"))
    _rows, report = hook(candidates)
    report["npz"] = diag.get("npz_basename")
    return report


def _tag(candidates, depth_edges, lateral_edges, support):
    tagged = []
    for row in candidates:
        yaw = float(row["box_yaw"])
        yaw_deg = 0 if abs(yaw) < 1e-6 else 90
        tagged.append({
            "candidate_id": row["candidate_id"],
            "center_container": [float(v) for v in row["center_container"]],
            "box_yaw": yaw,
            "heading_theta": float(row["heading_theta"]),
            "footprint": [float(v) for v in row["footprint"]],
            "size": [float(v) for v in row["size"]],
            "depth_m": float(row["depth_m"]),
            "support_layer_z": float(row["support_layer_z"]),
            "support_score": float(row["support_score"]),
            "score": float(row["score"]),
            "support": support,
            "depth_band": _band(
                row["depth_m"], depth_edges[0], depth_edges[1],
                ("near", "mid", "deep")),
            "lateral_band": _band(
                row["center_container"][1], lateral_edges[0],
                lateral_edges[1], ("left", "center", "right")),
            "yaw_deg": yaw_deg,
            "atlas_status_name": row.get("atlas_status_name") or "no_atlas",
            "raw": row,
        })
    return tagged


def _select(tagged, cap):
    cells = {}
    for row in tagged:
        key = (row["depth_band"], row["lateral_band"],
               row["yaw_deg"], row["support"])
        cells.setdefault(key, []).append(row)
    for group in cells.values():
        group.sort(key=lambda row: (-row["score"], row["candidate_id"]))
    selected = []
    used = set()
    keys = list(cells)

    def gained(key):
        depth, lateral, yaw, support = key
        have = {
            "depth": {row["depth_band"] for row in selected},
            "lateral": {row["lateral_band"] for row in selected},
            "yaw": {row["yaw_deg"] for row in selected},
            "support": {row["support"] for row in selected},
        }
        return sum((
            depth not in have["depth"],
            lateral not in have["lateral"],
            yaw not in have["yaw"],
            support not in have["support"],
        ))

    while len(selected) < cap and len(used) < len(keys):
        ranked = sorted(
            (key for key in keys if key not in used),
            key=lambda key: (
                -gained(key), -cells[key][0]["score"], key))
        if not ranked:
            break
        key = ranked[0]
        used.add(key)
        selected.append(cells[key][0])
    # Fill toward the cap from the next solver candidate in each cell.
    round_index = 1
    while len(selected) < cap:
        added = False
        for key in sorted(cells):
            group = cells[key]
            if round_index < len(group) and len(selected) < cap:
                selected.append(group[round_index])
                added = True
        if not added:
            break
        round_index += 1
    return selected


def _public(row, box_name, ordinal):
    return {
        "scan_id": "%s-%02d" % (box_name, ordinal),
        "box": box_name,
        "candidate_id": row["candidate_id"],
        "center_container": row["center_container"],
        "quaternion_xyzw": [
            0.0, 0.0,
            math.sin(0.5 * row["heading_theta"]),
            math.cos(0.5 * row["heading_theta"])],
        "box_yaw": row["box_yaw"],
        "heading_theta": row["heading_theta"],
        "footprint": row["footprint"],
        "size": row["size"],
        "depth_m": row["depth_m"],
        "depth_band": row["depth_band"],
        "lateral_band": row["lateral_band"],
        "lateral_y_m": row["center_container"][1],
        "yaw_deg": row["yaw_deg"],
        "support": row["support"],
        "support_layer_z": row["support_layer_z"],
        "support_score": row["support_score"],
        "solver_score": row["score"],
        "atlas_status_name": row["atlas_status_name"],
    }


def _support_parent(floor_tagged):
    preferred = [
        row for row in floor_tagged
        if row["depth_band"] == "mid" and row["lateral_band"] == "center"]
    pool = preferred or floor_tagged
    pool = sorted(pool, key=lambda row: (-row["score"], row["candidate_id"]))
    return pool[0] if pool else None


def build_scan(ctx, store):
    points = []
    reports = {}
    for box_name, spec in BOXES.items():
        size = list(spec["wdh"])
        surface = floor_prior_surface(ctx)
        floor, floor_totals = _feasible(ctx, surface, size, [])
        _annotate(ctx, store, size, floor)
        depths = _tertile_edges(row["depth_m"] for row in floor)
        laterals = _tertile_edges(
            row["center_container"][1] for row in floor)
        floor_tagged = _tag(floor, depths, laterals, "floor")
        parent = _support_parent(floor_tagged)
        stack_tagged = []
        stack_totals = {"n_enumerated": 0, "n_capacity_feasible": 0}
        if parent is not None:
            stacked = floor_prior_surface(ctx)
            apply_placement(stacked, parent["raw"], 0.0)
            from luggage_packing.placement_solver import candidate_aabb
            placed = [candidate_aabb(parent["raw"], ctx.inner_size[2])]
            stack, stack_totals = _feasible(ctx, stacked, size, placed)
            stack = [
                row for row in stack
                if float(row["support_layer_z"]) > 1e-4]
            _annotate(ctx, store, size, stack)
            stack_tagged = _tag(stack, depths, laterals, "stack")
        chosen = _select(floor_tagged + stack_tagged, SCAN_CAP)
        for ordinal, row in enumerate(chosen, start=1):
            public = _public(row, box_name, ordinal)
            if row["support"] == "stack" and parent is not None:
                public["support_parent_id"] = parent["candidate_id"]
                public["support_parent_center"] = parent["center_container"]
                public["support_parent_footprint"] = parent["footprint"]
                public["support_parent_height"] = parent["size"][2]
                public["support_parent_yaw"] = parent["heading_theta"]
            points.append(public)
        reports[box_name] = {
            "floor_feasible": len(floor_tagged),
            "stack_feasible": len(stack_tagged),
            "selected": len(chosen),
            "floor_totals": floor_totals,
            "stack_totals": stack_totals,
            "depth_edges_m": [depths[0], depths[1]],
            "lateral_edges_m": [laterals[0], laterals[1]],
            "lateral_axis": (
                "container y; left is the lower third, right the upper"),
            "depth_axis": (
                "solver depth_m = dot(center - opening, inward); "
                "near is the lower third, deep the upper third"),
            "support_parent_id": (
                parent["candidate_id"] if parent is not None else None),
        }
    return points, reports


def _jittered_size(wdh, seed):
    return [
        round(float(wdh[index]) + _span(seed, index, SIZE_JITTER_M), 6)
        for index in range(3)]


def _sequence_for(setting, seed, count):
    nominal = BOXES["20in"]["wdh"]
    standard = BOXES["standard"]["wdh"]
    sizes = []
    for index in range(count):
        if setting == "standard_nominal":
            sizes.append(list(standard))
        elif setting == "mixed_alternating":
            sizes.append(list(nominal if index % 2 == 0 else standard))
        elif setting == "20in_size_jitter":
            # One draw per box, still determined by the repeat seed.
            sizes.append(_jittered_size(
                nominal, int(seed) * 100 + index))
        else:
            sizes.append(list(nominal))
    return sizes


def _geometric(ctx, setting, seed):
    """How many boxes the solver will commit before it has no legal slot.

    Settings other than ``stacking_enabled`` stop when the next solver
    choice is a stacked layer. That stop is ``STACK_NOT_ENABLED``, not
    ``BIN_FULL_GEOMETRIC``: stack slots can still be legal.
    """
    allow_stack = setting == "stacking_enabled"
    placed = []
    surface = floor_prior_surface(ctx)
    aabbs = []
    stop = "BIN_FULL_GEOMETRIC"
    for index in range(40):
        size = _sequence_for(setting, seed, index + 1)[index]
        result = solve_placement(
            surface, size, allowed_yaws=list(YAWS),
            params=_solver_params(),
            candidate_validator=make_validator(ctx, aabbs, size))
        if not result["success"]:
            code = result["reason_code"]
            stop = ("BIN_FULL_GEOMETRIC"
                    if code == "BIN_FULL" else "NO_COMPLETE_ROUTE")
            if code == "BIN_FULL":
                stop = "BIN_FULL_GEOMETRIC"
            elif int(result.get("capacity_feasible_count") or 0) == 0:
                stop = "BIN_FULL_GEOMETRIC"
            else:
                stop = "NO_COMPLETE_ROUTE"
            break
        selected = result["selected"]
        stacked = float(selected.get("support_layer_z") or 0.0) > 1e-4
        if stacked and not allow_stack:
            stop = "STACK_NOT_ENABLED"
            break
        apply_placement(surface, selected, 0.0)
        from luggage_packing.placement_solver import candidate_aabb
        aabbs.append(candidate_aabb(selected, ctx.inner_size[2]))
        placed.append({
            "index": index,
            "support": "stack" if stacked else "floor",
            "size": [round(float(v), 6) for v in size],
            "depth_m": selected.get("depth_m"),
            "candidate_id": selected.get("candidate_id"),
        })
    return {
        "setting": setting,
        "seed": seed,
        "geometric_capacity_count": len(placed),
        "floor_count": sum(1 for row in placed if row["support"] == "floor"),
        "stacked_count": sum(1 for row in placed if row["support"] == "stack"),
        "stop_reason": stop,
        "placed": placed,
        "pick_offset_note": (
            "The solver places the box body. Pick offset does not change "
            "this geometric count; it changes the route."
            if setting == "20in_pick_offset" else ""),
    }


def _pick_offset(seed):
    return {
        "xy_m": [
            round(_span(seed, 0, PICK_XY_M), 6),
            round(_span(seed, 1, PICK_XY_M), 6)],
        "z_m": round(_span(seed, 2, PICK_Z_M), 6),
        "yaw_rad": round(_span(seed, 3, PICK_YAW_RAD), 6),
    }


def freeze():
    pins = assert_safety_pins()
    ctx = SceneContext(SCENE, ATLAS_CONFIG)
    from capacity_orientation_analysis import AtlasStore
    store = AtlasStore(ctx)
    points, reports = build_scan(ctx, store)
    geometric = []
    for setting, seeds in CAPACITY_SEEDS.items():
        for seed in seeds:
            geometric.append(_geometric(ctx, setting, seed))
    manifest = {
        "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "scene_tf": SCENE,
        "atlas_config": ATLAS_CONFIG,
        "scene_hash": ctx.scene_hash,
        "geometry_hash": str(ctx.hull.geometry_hash),
        "inner_size_m": ctx.inner_size,
        "safety": pins,
        "boxes": BOXES,
        "box_note": (
            "20in is the catalog carryon, the 20-inch class in this "
            "scene. standard is the catalog standard row."),
        "yaws_rad": list(YAWS),
        "cycle_start_joints": CYCLE_START_Q,
        "scan_cap": SCAN_CAP,
        "strata": reports,
        "size_jitter_half_m": SIZE_JITTER_M,
        "pick_offset": {
            "xy_half_m": PICK_XY_M,
            "z_half_m": PICK_Z_M,
            "yaw_half_rad": PICK_YAW_RAD,
        },
        "capacity_seeds": {
            setting: [
                {"seed": seed, "pick_offset": _pick_offset(seed)
                 if setting == "20in_pick_offset" else None}
                for seed in seeds]
            for setting, seeds in CAPACITY_SEEDS.items()},
        "capacity_policy": {
            "stacking_enabled": "floor first, then solver-legal stacks",
            "other_settings": (
                "stop at STACK_NOT_ENABLED when the next solver choice "
                "is a stacked layer; do not call that BIN_FULL"),
        },
        "sim_overfit": [
            "Floor-prior surface, not a live cargo map.",
            "Nominal top-center suction except the pick-offset setting.",
            "Isolated MoveIt has no controller tracking error.",
            "Gazebo contact and the real suitcase shell differ.",
            "A sim committed count is not a field capacity.",
        ],
    }
    _dump("frozen_contract.json", manifest)
    _dump("scan_list.json", {"points": points, "strata": reports})
    _dump("geometric_capacity.json", {"runs": geometric})
    print(json.dumps({
        "points": len(points),
        "by_box": {
            name: report["selected"] for name, report in reports.items()},
        "geometric": [
            {"setting": row["setting"], "seed": row["seed"],
             "count": row["geometric_capacity_count"],
             "stop": row["stop_reason"]}
            for row in geometric],
    }, indent=2))
    return 0


def _grasp(size, offset=None):
    """Top-center grasp. See ``scan_grasp.synthetic_top_grasp``."""
    from scan_grasp import synthetic_top_grasp
    return synthetic_top_grasp(size, offset)


def _surface_record(ctx, stamp_ns, occupied=None):
    surface = floor_prior_surface(ctx)
    if occupied is not None:
        surface = occupied
    half_l = 0.5 * float(ctx.inner_size[0])
    half_w = 0.5 * float(ctx.inner_size[1])
    return {
        "frame_id": "container_local",
        "resolution_m": float(surface["resolution"]),
        "origin_xy_container": [-half_l, -half_w],
        "state": surface["state"],
        "stamp_ns": int(stamp_ns),
        "map_source": "static_empty" if occupied is None else "measured",
        "placed_count": 0 if occupied is None else 1,
        "cargo_observed": occupied is not None,
        "commit_revision": 1,
    }


def _request(ctx, transform, points, request_id, stamp_ns, occupied=None):
    size = points[0]["size"]
    grasp_pos, grasp_quat = _grasp(size)
    return {
        "request_id": request_id,
        "scene_revision": "placement-robustness",
        "box_wdh": [float(v) for v in size],
        "box_id": points[0]["box"],
        "generation": 1,
        "slots": [{
            "request_id": point["scan_id"],
            "position": [float(v) for v in point["center_container"]],
            "quaternion_xyzw": [float(v) for v in point["quaternion_xyzw"]],
        } for point in points],
        "surface": _surface_record(ctx, stamp_ns, occupied),
        "inner_size": [float(v) for v in ctx.inner_size],
        "geometry_hash": str(ctx.hull.geometry_hash),
        "map_revision": 1,
        "start_joints": [float(v) for v in CYCLE_START_Q],
        "grasp": {
            "position": grasp_pos,
            "quaternion_xyzw": grasp_quat,
            "stamp_ns": int(stamp_ns),
        },
        "container_transform": {
            "translation": [float(v) for v in transform[2]],
            "quaternion_xyzw": [float(v) for v in transform[3]],
        },
    }


def _edge_rows(result):
    rows = []
    for edge in result.get("edges") or ():
        attributes = edge.attributes or {}
        rows.append({
            "edge_class": attributes.get("edge_class"),
            "planner": attributes.get("planner"),
            "load": str(edge.load_state),
            "states": len(edge.trajectory.positions),
        })
    return rows


def _complete(result, edges):
    classes = [row["edge_class"] for row in edges]
    needed = all(name in classes for name in REQUIRED_EDGE_CLASSES)
    retreats = sum(1 for name in classes if name == "RETREAT")
    returns = sum(1 for name in classes if name == "RETURN")
    certified = bool(
        result.get("certification") is not None
        and result["certification"].passed)
    return bool(result.get("success") and certified and needed
                and retreats >= 3 and returns >= 1)


def _reason_bucket(reason):
    text = str(reason or "")
    if "LATTICE_NO_CHAIN" in text:
        return "LATTICE_NO_CHAIN"
    if "LATTICE_LAYER_EMPTY" in text:
        return "LATTICE_LAYER_EMPTY"
    if "COLLISION" in text or "contact" in text.lower():
        return "collision"
    if "IK" in text or "SEED_MISS" in text:
        return "ik"
    if text.startswith("CERT_") or "CERT" in text:
        return "certification"
    if "BUDGET" in text:
        return "PLANNING_BUDGET_EXHAUSTED"
    if "EDGE_BUILD" in text or "GOAL_STATE" in text:
        return "edge_build"
    return text or "other"


def _plan_one(producer, request):
    from luggage_planning.route_producer import register_attach_from_request
    register_attach_from_request(
        producer.attachment_provider, request,
        box_wdh=request["box_wdh"], box_id=request["box_id"],
        generation=1)
    started = time.perf_counter()
    result = producer.plan_place_route(request)
    wall = time.perf_counter() - started
    reset = getattr(producer.edge_builder, "reset_scene", None)
    if callable(reset):
        reset()
    edges = _edge_rows(result)
    rows = result.get("rows") or []
    tried = [
        row.get("slot_index", row.get("candidate_id"))
        for row in rows if isinstance(row, dict)
        and row.get("reason") != "ATTEMPT_SUMMARY"]
    winner = None
    for row in rows:
        if isinstance(row, dict) and row.get("reason") in ("OK", "ok"):
            winner = row.get("candidate_id") or row.get("slot_index")
    return {
        "success": bool(result.get("success")),
        "complete_route": _complete(result, edges),
        "reason_code": result.get("reason_code"),
        "wall_s": round(wall, 3),
        "edges": edges,
        "row_reasons": [
            row.get("reason") for row in rows if isinstance(row, dict)],
        "candidates_tried": len([
            row for row in rows if isinstance(row, dict)
            and row.get("reason") not in ("ATTEMPT_SUMMARY", None)
            and row.get("candidate_id")]),
        "tried_ids": tried,
        "winner": winner,
        "bucket": _reason_bucket(result.get("reason_code") or (
            rows[-2].get("reason") if len(rows) > 1 else "")),
    }


def _rotate(q, vector):
    qx, qy, qz, qw = q
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def _qmul(left, right):
    ax, ay, az, aw = left
    bx, by, bz, bw = right
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _add_support(client, transform, point):
    parent_yaw = float(point["support_parent_yaw"])
    box_q = (0.0, 0.0, math.sin(0.5 * parent_yaw), math.cos(0.5 * parent_yaw))
    world_q = _qmul(tuple(transform[3]), box_q)
    center = _rotate(tuple(transform[3]), point["support_parent_center"])
    origin = tuple(transform[2])
    world = [center[i] + origin[i] for i in range(3)]
    footprint = point["support_parent_footprint"]
    size = (float(footprint[0]), float(footprint[1]),
            float(point["support_parent_height"]))
    client.remove_object("robustness_support")
    return client.add_collision_box(
        "robustness_support", world, world_q, size, frame_id="world")


def plan():
    assert_safety_pins()
    import corridor_lattice_score as score
    import slot64_classify_replay as classify

    list_path = os.path.join(OUT, "scan_list.json")
    if not os.path.isfile(list_path):
        raise SystemExit("run freeze first")
    points = json.load(open(list_path, encoding="utf-8"))["points"]
    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        raise SystemExit("residual sim processes: %s" % residuals)
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_perception.eval.isolated_domain import isolated_replay_env
    from luggage_planning.lattice_route import LatticeStateChecker
    from luggage_planning.planning_scene_client import PlanningSceneClient
    from luggage_planning.typed_driver_adapter import (
        container_transform_fields,
    )
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    ctx = SceneContext(SCENE, ATLAS_CONFIG)
    scene_config = load_scene_tf_config(SCENE)
    transform = container_transform_fields(scene_config)
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    log_path = os.path.join(OUT, "moveit.log")
    proc = score._start_moveit(env, SCENE, log_path)  # noqa: SLF001
    rclpy.init()
    node = Node("placement_robustness")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = __import__("threading").Thread(
        target=executor.spin, daemon=True)
    spinner.start()
    jsonl_path = os.path.join(OUT, "plan_results.jsonl")
    results = []
    try:
        from luggage_msgs.srv import ValidateRouteV2
        probe = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not probe.wait_for_service(timeout_sec=90.0):
            raise SystemExit("validate_route_v2 did not come up")
        classify._sync_world(  # noqa: SLF001
            node, scene_config, {"world_objects": []})
        scene_client = PlanningSceneClient(node)
        stamp = time.monotonic_ns()
        producer, _urdf, _tcp = classify._producer(  # noqa: SLF001
            node, scene_config, SCENE, None, stamp,
            budget_s=float(ROUTE_TOTAL_WALL_S))
        producer._clock_ns = time.monotonic_ns
        producer.corridor_planner = "lattice"
        producer.lattice_checker_factory = (
            lambda scene_states, start_q: LatticeStateChecker(
                node, scene_client, scene_states, start_q))
        producer.vacuum_state_fn = lambda: {
            "attached": True, "vacuum_on": True,
            "received_ns": time.monotonic_ns(),
        }
        occupied = None
        parent_added = False
        with open(jsonl_path, "w", encoding="utf-8") as handle:
            ordered = sorted(points, key=lambda point: (
                0 if point["box"] == "20in" else 1,
                0 if point["support"] == "floor" else 1,
                point["scan_id"]))
            for point in ordered:
                if point["support"] == "stack":
                    _add_support(scene_client, transform, point)
                    parent_added = True
                    surface = floor_prior_surface(ctx)
                    parent = {
                        "center_local": None,
                        "center_container": point["support_parent_center"],
                        "footprint": point["support_parent_footprint"],
                        "size": [
                            point["support_parent_footprint"][0],
                            point["support_parent_footprint"][1],
                            point["support_parent_height"]],
                    }
                    center = point["support_parent_center"]
                    map_center = surface["center_base"]
                    parent["center_local"] = [
                        float(center[0]), float(center[1]),
                        float(center[2]) - float(map_center[2])]
                    apply_placement(surface, parent, 0.0)
                    occupied = surface
                elif parent_added:
                    scene_client.remove_object("robustness_support")
                    parent_added = False
                    occupied = None
                stamp_ns = time.monotonic_ns()
                request = _request(
                    ctx, transform, [point], point["scan_id"], stamp_ns,
                    occupied if point["support"] == "stack" else None)
                try:
                    outcome = _plan_one(producer, request)
                except Exception as exc:  # noqa: BLE001 - one point must not drop the rest
                    outcome = {
                        "success": False,
                        "complete_route": False,
                        "reason_code": "SCAN_EXCEPTION",
                        "wall_s": 0.0,
                        "edges": [],
                        "row_reasons": [],
                        "candidates_tried": 0,
                        "bucket": "other",
                        "error": "%s: %s" % (type(exc).__name__, exc),
                    }
                outcome.update({
                    "scan_id": point["scan_id"],
                    "box": point["box"],
                    "depth_band": point["depth_band"],
                    "lateral_band": point["lateral_band"],
                    "yaw_deg": point["yaw_deg"],
                    "support": point["support"],
                    "atlas_status_name": point["atlas_status_name"],
                    "candidate_id": point["candidate_id"],
                })
                results.append(outcome)
                handle.write(json.dumps(outcome, sort_keys=True) + "\n")
                handle.flush()
                node.get_logger().info(
                    "%s %s wall %.1fs" % (
                        point["scan_id"], outcome["reason_code"],
                        outcome["wall_s"]))
            failures = [
                row for row in results
                if not row["complete_route"] and row["wall_s"] < 25.0]
            successes = [row for row in results if row["complete_route"]]
            fallback_rows = []
            if successes and failures:
                good = successes[0]
                good_point = next(
                    point for point in points
                    if point["scan_id"] == good["scan_id"])
                for bad in failures[:6]:
                    if bad["box"] != good["box"]:
                        continue
                    bad_point = next(
                        point for point in points
                        if point["scan_id"] == bad["scan_id"])
                    stamp_ns = time.monotonic_ns()
                    request = _request(
                        ctx, transform, [bad_point, good_point],
                        "fallback-%s" % bad["scan_id"], stamp_ns, None)
                    outcome = _plan_one(producer, request)
                    outcome.update({
                        "kind": "fallback",
                        "first": bad["scan_id"],
                        "second": good["scan_id"],
                        "box": bad["box"],
                    })
                    fallback_rows.append(outcome)
                    handle.write(json.dumps(outcome, sort_keys=True) + "\n")
                    handle.flush()
        _dump("fallback_results.json", {"rows": fallback_rows})
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        score._stop_moveit(proc)  # noqa: SLF001
    print(json.dumps({
        "planned": len(results),
        "complete": sum(1 for row in results if row["complete_route"]),
        "residuals": score._residuals(),  # noqa: SLF001
    }, indent=2))
    return 0


def _percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(math.ceil(
        fraction * len(ordered))) - 1))
    return ordered[index]


def _rate(rows):
    if not rows:
        return None
    return round(sum(1 for row in rows if row.get("complete_route"))
                 / float(len(rows)), 4)


def _group(rows, key):
    groups = {}
    for row in rows:
        groups.setdefault(row.get(key), []).append(row)
    return {
        str(name): {"n": len(group), "complete_route_rate": _rate(group)}
        for name, group in sorted(groups.items(), key=lambda item: str(item[0]))
    }


def summarize():
    path = os.path.join(OUT, "plan_results.jsonl")
    rows = []
    fallback = []
    if os.path.isfile(path):
        for line in open(path, encoding="utf-8"):
            row = json.loads(line)
            if row.get("kind") == "fallback":
                fallback.append(row)
            else:
                rows.append(row)
    fallback_path = os.path.join(OUT, "fallback_results.json")
    if os.path.isfile(fallback_path):
        fallback.extend(json.load(open(
            fallback_path, encoding="utf-8")).get("rows") or [])
    walls = [float(row["wall_s"]) for row in rows]
    tried = [
        int(row["candidates_tried"]) for row in fallback
        if row.get("complete_route")]
    buckets = {}
    for row in rows:
        if row.get("complete_route"):
            continue
        reasons = [
            name for name in (row.get("row_reasons") or [])
            if name != "ATTEMPT_SUMMARY"]
        name = _reason_bucket(reasons[0] if reasons else row.get("bucket"))
        row["failure_bucket"] = name
        buckets[name] = buckets.get(name, 0) + 1
    summary = {
        "n_points": len(rows),
        "complete_route_rate": _rate(rows),
        "by_box": _group(rows, "box"),
        "by_depth": _group(rows, "depth_band"),
        "by_lateral": _group(rows, "lateral_band"),
        "by_yaw_deg": _group(rows, "yaw_deg"),
        "by_support": _group(rows, "support"),
        "by_atlas": _group(rows, "atlas_status_name"),
        "candidate_fallback_success_rate": _rate(fallback) if fallback else None,
        "fallback_n": len(fallback),
        "candidates_tried_before_success": {
            "mean": (round(sum(tried) / float(len(tried)), 3) if tried else None),
            "p50": _percentile(tried, 0.50),
            "p95": _percentile(tried, 0.95),
            "max": max(tried) if tried else None,
        },
        "placement_request_wall_s": {
            "p50": _percentile(walls, 0.50),
            "p95": _percentile(walls, 0.95),
            "max": max(walls) if walls else None,
        },
        "failure_buckets": buckets,
        "sim_overfit": [
            "Plan-only on a floor-prior surface. No controller, no live map.",
            "A passed lattice route is not a committed box.",
            "Geometric capacity in geometric_capacity.json is the solver count, not a field capacity.",
            "Live multi-box runs are not in this summary until the capacity runner records them.",
        ],
    }
    _dump("summary.json", summary)
    if rows:
        _heatmap(rows)
    print(json.dumps(summary, indent=2))
    return 0


def _heatmap(rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ink = "#0b0b0b"
    ink2 = "#52514e"
    neutral = "#efeeea"
    ramp = ("#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#104281")
    depths = ("near", "mid", "deep")
    laterals = ("left", "center", "right")
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 4.6), dpi=140)
    fig.suptitle(
        "Lattice complete-route rate by opening depth and lateral third",
        color=ink, fontsize=11)
    for axis, box_name in zip(axes, ("20in", "standard")):
        grid = []
        labels = []
        subset = [row for row in rows if row["box"] == box_name]
        for depth in depths:
            rate_row = []
            label_row = []
            for lateral in laterals:
                cell = [
                    row for row in subset
                    if row["depth_band"] == depth
                    and row["lateral_band"] == lateral]
                if not cell:
                    rate_row.append(float("nan"))
                    label_row.append("n=0")
                else:
                    rate = sum(1 for row in cell if row["complete_route"])
                    rate_row.append(rate / float(len(cell)))
                    label_row.append("%d/%d" % (rate, len(cell)))
            grid.append(rate_row)
            labels.append(label_row)
        image = axis.imshow(
            grid, vmin=0.0, vmax=1.0, cmap=_ramp_cmap(ramp),
            aspect="equal")
        axis.set_xticks(range(3))
        axis.set_xticklabels(laterals, color=ink, fontsize=9)
        axis.set_yticks(range(3))
        axis.set_yticklabels(depths, color=ink, fontsize=9)
        axis.set_title(box_name, color=ink, fontsize=11)
        for i in range(3):
            for j in range(3):
                color = ink if (grid[i][j] != grid[i][j]
                                or grid[i][j] < 0.65) else "white"
                if grid[i][j] != grid[i][j]:
                    color = ink2
                axis.text(j, i, labels[i][j], ha="center", va="center",
                          color=color, fontsize=9)
        axis.tick_params(length=0)
        for spine in axis.spines.values():
            spine.set_color(ink2)
    fig.colorbar(image, ax=list(axes), fraction=0.046, pad=0.04
                 ).set_label("complete route rate", color=ink2)
    fig.tight_layout(rect=(0, 0.02, 1, 0.92))
    png = os.path.join(OUT, "region_rate.png")
    fig.savefig(png)
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    bad = 0
    for axis in fig.axes:
        boxes = [
            (text.get_text().split("\n")[0], text.get_window_extent(renderer))
            for text in axis.texts if text.get_text().strip()]
        for label, box in boxes:
            if (box.x0 < 1 or box.x1 > fig.bbox.x1 - 1
                    or box.y0 < 1 or box.y1 > fig.bbox.y1 - 1):
                print("CLIP", label)
                bad += 1
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                left, right = boxes[i][1], boxes[j][1]
                ox = min(left.x1, right.x1) - max(left.x0, right.x0)
                oy = min(left.y1, right.y1) - max(left.y0, right.y0)
                if (ox > 2 and oy > 2
                        and ox * oy > 0.10 * min(
                            left.width * left.height,
                            right.width * right.height)):
                    print("OVERLAP", boxes[i][0], boxes[j][0])
                    bad += 1
    print("AUDIT:", "CLEAN" if bad == 0 else bad)
    plt.close(fig)
    _dump("region_rate.json", {
        "png": png,
        "cell": "successes/attempts",
        "empty": neutral,
    })


def _ramp_cmap(colors):
    from matplotlib.colors import LinearSegmentedColormap
    return LinearSegmentedColormap.from_list("route", list(colors))


def fallback():
    """One request: a fast empty layer, then a no-chain point of the same box.

    The stratified scan had no complete route, so this cannot raise the
    success rate. It records whether the producer tries the second slot
    inside the same 60 s request.
    """
    assert_safety_pins()
    import corridor_lattice_score as score
    import slot64_classify_replay as classify
    points = {
        row["scan_id"]: row for row in json.load(open(
            os.path.join(OUT, "scan_list.json"), encoding="utf-8"))["points"]}
    results = [
        json.loads(line) for line in open(
            os.path.join(OUT, "plan_results.jsonl"), encoding="utf-8")
        if line.strip() and json.loads(line).get("kind") != "fallback"]
    empty = [
        row for row in results
        if "LATTICE_LAYER_EMPTY" in (row.get("row_reasons") or [])]
    chain = [
        row for row in results
        if "LATTICE_NO_CHAIN" in (row.get("row_reasons") or [])]
    empty.sort(key=lambda row: row["wall_s"])
    if not empty or not chain:
        raise SystemExit("need one empty-layer point and one no-chain point")
    bad = empty[0]
    good = next(row for row in chain if row["box"] == bad["box"])
    pair = [points[bad["scan_id"]], points[good["scan_id"]]]
    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        raise SystemExit("residual sim processes: %s" % residuals)
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_perception.eval.isolated_domain import isolated_replay_env
    from luggage_planning.lattice_route import LatticeStateChecker
    from luggage_planning.planning_scene_client import PlanningSceneClient
    from luggage_planning.typed_driver_adapter import container_transform_fields
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    ctx = SceneContext(SCENE, ATLAS_CONFIG)
    scene_config = load_scene_tf_config(SCENE)
    transform = container_transform_fields(scene_config)
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    proc = score._start_moveit(  # noqa: SLF001
        env, SCENE, os.path.join(OUT, "moveit_fallback.log"))
    rclpy.init()
    node = Node("placement_fallback")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = __import__("threading").Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        from luggage_msgs.srv import ValidateRouteV2
        probe = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not probe.wait_for_service(timeout_sec=90.0):
            raise SystemExit("validate_route_v2 did not come up")
        classify._sync_world(node, scene_config, {"world_objects": []})  # noqa: SLF001
        scene_client = PlanningSceneClient(node)
        producer, _urdf, _tcp = classify._producer(  # noqa: SLF001
            node, scene_config, SCENE, None, time.monotonic_ns(),
            budget_s=float(ROUTE_TOTAL_WALL_S))
        producer._clock_ns = time.monotonic_ns
        producer.corridor_planner = "lattice"
        producer.lattice_checker_factory = (
            lambda scene_states, start_q: LatticeStateChecker(
                node, scene_client, scene_states, start_q))
        producer.vacuum_state_fn = lambda: {
            "attached": True, "vacuum_on": True,
            "received_ns": time.monotonic_ns(),
        }
        request = _request(
            ctx, transform, pair, "fallback-pair", time.monotonic_ns(), None)
        outcome = _plan_one(producer, request)
        outcome.update({
            "kind": "fallback",
            "first": bad["scan_id"],
            "second": good["scan_id"],
            "box": bad["box"],
            "note": (
                "Second slot is another scan failure (no complete route "
                "exists in the stratified set). tried>=2 means the "
                "producer switched points inside the request."),
        })
        _dump("fallback_results.json", {"rows": [outcome]})
        print(json.dumps({
            "first": bad["scan_id"],
            "second": good["scan_id"],
            "tried": outcome["candidates_tried"],
            "reasons": outcome["row_reasons"],
            "complete_route": outcome["complete_route"],
            "wall_s": outcome["wall_s"],
            "residuals": score._residuals(),  # noqa: SLF001
        }, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        score._stop_moveit(proc)  # noqa: SLF001
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command", choices=("freeze", "plan", "fallback", "summarize"))
    args = parser.parse_args()
    if args.command == "freeze":
        return freeze()
    if args.command == "plan":
        return plan()
    if args.command == "fallback":
        return fallback()
    return summarize()


if __name__ == "__main__":
    sys.exit(main())
