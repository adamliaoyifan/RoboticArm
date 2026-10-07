#!/usr/bin/env python3
"""Replay the standard -10 cm second-box witness through production Select.

A0 plans only that probe. A1 inserts it into the captured place-v2-2 slot
list. A2 reruns solve_placement offline. Nothing here changes a budget,
a threshold, or the production planner. Isolated MoveIt is domain 42.
"""

from __future__ import division

import hashlib
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for path in (
        HERE, ROOT,
        os.path.join(ROOT, "src", "luggage_planning"),
        os.path.join(ROOT, "src", "luggage_packing"),
        os.path.join(ROOT, "src", "luggage_description"),
        os.path.join(ROOT, "src", "luggage_perception"),
):
    if path not in sys.path:
        sys.path.insert(0, path)

import corridor_lattice_score as score  # noqa: E402
import probe_opening_certify as probes  # noqa: E402
import slot64_classify_replay as classify  # noqa: E402

OUT = os.path.join(
    ROOT, "docs/status/evidence/frame_migration",
    "ST09-parallel-I-s6-readiness/witness_select_replay")
SCENE_TF = os.path.join(
    ROOT, "src/luggage_description/config/scene_tf_deploy.yaml")
# Online second-box used the Gazebo atlas. Isolated MoveIt publishes the
# mock URDF (22ed00b5); the atlas meta stores the Gazebo URDF
# (689838e5). The package documents those samples as the same NPZ.
# The replay loads that atlas with the stored URDF hash and records the
# live hash beside it.
ATLAS_CFG = os.path.join(
    ROOT, "src/luggage_planning/config/atlas_runtime_deploy_sim.yaml")
ATLAS_URDF_HASH = "689838e5ab5948877ea9c87f388a0474"
CHAIN_PATH = os.path.join(
    ROOT, "docs/status/evidence/frame_migration",
    "ST09-parallel-I-s6-readiness/placement_corridor_supergraph_g1",
    "opening_cert/standard_m10_run_0.json")
XY_M = 0.025
Z_M = 0.01
YAW_RAD = 0.2
DIAGNOSTIC_BUDGET_S = 30.0


def _dump(name, document):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2, default=str)
        handle.write("\n")
    return path


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git():
    rev = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    dirty = subprocess.check_output(
        ["git", "status", "--porcelain"], cwd=ROOT, text=True)
    return rev, len([line for line in dirty.splitlines() if line.strip()])


def _yaw(quaternion):
    x, y, z, w = (float(v) for v in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _angle(left, right):
    delta = abs((_yaw(left) - _yaw(right) + math.pi) % (2.0 * math.pi) - math.pi)
    return min(delta, abs(delta - math.pi))


def _near(center, quaternion, other_center, other_quaternion):
    if abs(float(center[2]) - float(other_center[2])) > Z_M:
        return False
    if math.hypot(float(center[0]) - float(other_center[0]),
                  float(center[1]) - float(other_center[1])) > XY_M:
        return False
    return _angle(quaternion, other_quaternion) < YAW_RAD


def _status(ok, reason="", scope=""):
    if ok is None:
        return "not_evaluated"
    if ok:
        return "present"
    return "rejected"


def _witness():
    found = [row for row in probes._probes() if row["name"] == "standard_m10"]
    if len(found) != 1:
        raise SystemExit("standard_m10 probe missing")
    return found[0]


def _load(path):
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def _compare(name, left, right, tol):
    if left is None or right is None:
        return {"item": name, "status": "not_evaluated",
                "left": left, "right": right}
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        delta = float(left) - float(right)
        ok = abs(delta) <= tol
        return {"item": name, "status": "match" if ok else "mismatch",
                "delta": delta, "left": left, "right": right}
    if isinstance(left, (list, tuple)) and isinstance(right, (list, tuple)):
        if len(left) != len(right):
            return {"item": name, "status": "mismatch",
                    "left": list(left), "right": list(right)}
        delta = [float(a) - float(b) for a, b in zip(left, right)]
        ok = all(abs(v) <= tol for v in delta)
        return {"item": name, "status": "match" if ok else "mismatch",
                "delta": delta, "left": list(left), "right": list(right)}
    ok = left == right
    return {"item": name, "status": "match" if ok else "mismatch",
            "left": left, "right": right}


def _joint_near_request(request_joints, ring_path):
    best = None
    with open(ring_path, encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            row = json.loads(line)
            position = row.get("position") or []
            if len(position) != len(request_joints):
                continue
            delta = max(abs(float(a) - float(b))
                        for a, b in zip(request_joints, position))
            if best is None or delta < best[0]:
                best = (delta, position, row.get("stamp"))
    if best is None:
        return {"item": "start_joints", "status": "not_evaluated"}
    return {"item": "start_joints",
            "status": "match" if best[0] <= 1e-3 else "mismatch",
            "max_abs_rad": best[0], "ring_stamp": best[2],
            "request": [float(v) for v in request_joints],
            "nearest_ring": [float(v) for v in best[1]]}


def _gt_pose(path):
    if not os.path.isfile(path):
        return None
    with open(path, encoding="utf-8") as handle:
        line = handle.readline()
    if not line.strip():
        return None
    return json.loads(line)


def alignment(witness):
    request = _load(witness["request_path"])
    surface = _load(witness["surface_path"])
    scene = _load(witness["scene_path"])
    placed = next(
        obj for obj in scene.get("world_objects") or []
        if obj.get("id") == "placed_0_0_0")
    gt_path = os.path.join(
        os.path.dirname(os.path.dirname(witness["scene_path"])),
        "box_00_release_gt.jsonl")
    gt = _gt_pose(gt_path)
    ring = os.path.join(
        os.path.dirname(witness["surface_path"]), "joint_ring.jsonl")
    from ament_index_python.packages import get_package_share_directory
    contract_path = os.path.join(
        get_package_share_directory("st09_contracts"),
        "resolved_planning_contract.json")
    contract = _load(contract_path)
    budgets = contract["planning_budgets"]
    rows = [
        _compare("box_wdh", request["box_wdh"], witness["box_wdh"], 1e-9),
        _compare(
            "attachment_translation",
            request["attachment_transform"]["translation"],
            request["attachment_transform"]["translation"], 0.0),
        _compare(
            "attachment_quaternion",
            request["attachment_transform"]["quaternion_xyzw"],
            request["attachment_transform"]["quaternion_xyzw"], 0.0),
        _joint_near_request(request["start_joints"], ring),
        _compare("map_revision", request["map_revision"],
                 surface["map_revision"], 0),
        _compare("geometry_hash", request["geometry_hash"],
                 surface["geometry_hash"], 0),
        _compare("map_source", request["map_source"],
                 surface["map_source"], 0),
        _compare(
            "certification_timeout_s",
            budgets["certification_timeout_s"]["value"], 2.0, 0.0),
        _compare(
            "sample_dt_ns",
            contract["certification_sampling"]["max_sample_dt_ns"],
            10000000, 0),
        _compare(
            "route_budget_per_box_s",
            budgets["route_total_wall_s"]["value"], 60.0, 0.0),
    ]
    if gt is None:
        rows.append({"item": "placed_vs_release_gt", "status": "not_evaluated"})
    else:
        rows.append(_compare(
            "placed_vs_release_gt_xyz", placed["xyz"], gt["xyz"], 0.02))
    occupied = sum(
        1 for row in surface["state"] for cell in row if cell == "occupied")
    document = {
        "witness": witness["name"],
        "center": witness["center"],
        "source_slot": witness["source_slot"],
        "request_id": request["request_id"],
        "rows": rows,
        "placed_0_0_0": {
            "xyz": placed["xyz"], "quat": placed["quat"],
            "size": placed.get("size")},
        "release_gt": gt,
        "occupancy_cells": occupied,
        "surface_placed_count": surface.get("placed_count"),
        "captured_slot_count": len(request.get("candidates") or []),
        "notes": [
            "attachment_transform is the captured request value; the "
            "registered T_tcp_box is compared again when Select runs",
            "stats stub keeps revision, hash, source and placed_count, "
            "and refreshes last_acquisition_stamp_ns to the replay clock "
            "so the 10 s freshness window is not failed by a frozen stamp",
            "carrying edges do not add snapshot occupancy boxes; unloaded "
            "edges do, at z 0.15 m and height 0.30 m",
        ],
    }
    _dump("alignment.json", document)
    return document


def _solver_params():
    from luggage_packing.placement_solver import SAME_LAYER_CLEARANCE_M
    return {
        "clearance_margin": 0.03,
        "same_layer_clearance_m": float(SAME_LAYER_CLEARANCE_M),
        "min_support_ratio": 0.6,
        "top_n": 2000,
        "keep_rejected": 200,
        "hull_margin": 0.0,
    }


def _validator(scene_config, placed_world, box_wdh):
    from luggage_description.container_geometry import y_max_at_z
    from luggage_description.scene_tf_config_utils import (
        container_inner_ceiling_z,
        container_inner_floor_z,
        container_opening_aperture_corners_in_container,
        descriptor_from_scene_config,
    )
    from luggage_packing.placement_solver import placement_constraint_verdict

    hull = descriptor_from_scene_config(scene_config)
    floor_z = float(container_inner_floor_z(scene_config))
    inner_h = float(container_inner_ceiling_z(scene_config) - floor_z)
    corners = container_opening_aperture_corners_in_container(
        scene_config, 0.0)
    ys = [float(corner[1]) for corner in corners]
    aperture_y = (min(ys), max(ys))
    world = [float(v) for v in placed_world["xyz"]]
    center = (-(world[0] + 1.5), 0.6 - world[1], world[2])
    size = [float(v) for v in placed_world["size"]]
    yaw = _yaw(placed_world["quat"])
    rotated = abs(abs(yaw) - math.pi / 2.0) < math.radians(10.0)
    length = size[1] if rotated else size[0]
    width = size[0] if rotated else size[1]
    z_rel = center[2] - floor_z
    placed = [(
        center[0] - length * 0.5, center[1] - width * 0.5,
        max(0.0, z_rel - size[2] * 0.5),
        center[0] + length * 0.5, center[1] + width * 0.5,
        min(inner_h, z_rel + size[2] * 0.5),
    )]

    def hull_contains(point):
        x, y, z_rel_point = point
        z = z_rel_point + floor_z
        if not (float(hull.floor_z) - 1e-9 <= z <= float(hull.ceiling_z)):
            return False
        if not (-hull.half_x <= x <= hull.half_x):
            return False
        if y < -hull.half_y:
            return False
        return y <= y_max_at_z(hull, z, margin=0.0) + 1e-9

    def validator(candidate):
        return placement_constraint_verdict(
            candidate, inner_h, placed_aabbs=placed, aperture_y=aperture_y,
            hull_contains=hull_contains, inner_size=[
                hull.half_x * 2.0, hull.half_y * 2.0, inner_h],
            smallest_size=box_wdh, hull=hull, floor_z=0.0,
            same_layer_clearance_m=_solver_params()["same_layer_clearance_m"])

    return validator


def _candidate_pose(candidate):
    center = candidate.get("center_container")
    if center is None:
        return None
    yaw = float(candidate.get("box_yaw") or 0.0)
    quat = (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))
    return [float(v) for v in center], quat


def _find_pose(rows, center, quaternion, pose_of):
    exact = None
    neighbor = None
    best = None
    for index, row in enumerate(rows):
        pose = pose_of(row)
        if pose is None:
            continue
        other, quat = pose
        dist = math.hypot(float(center[0]) - other[0],
                          float(center[1]) - other[1])
        z = abs(float(center[2]) - other[2])
        yaw = _angle(quaternion, quat)
        rank = (dist, z, yaw)
        if best is None or rank < best[0]:
            best = (rank, index, other)
        if dist <= 1e-4 and z <= 1e-4 and yaw < 1e-3:
            exact = index
        elif neighbor is None and _near(center, quaternion, other, quat):
            neighbor = index
    return {
        "exact_index": exact,
        "neighbor_index": neighbor,
        "nearest_m": None if best is None else {
            "xy": best[0][0], "z": best[0][1], "yaw": best[0][2],
            "index": best[1], "center": best[2],
        },
    }


def generation(witness):
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_packing.placement_solver import (
        generate_candidates,
        ranked_feasible_candidates,
        solve_placement,
    )

    request = _load(witness["request_path"])
    surface = _load(witness["surface_path"])
    scene = _load(witness["scene_path"])
    placed = next(
        obj for obj in scene["world_objects"] if obj.get("id") == "placed_0_0_0")
    scene_config = load_scene_tf_config(SCENE_TF)
    params = _solver_params()
    full_params = dict(params)
    full_params["top_n"] = 1000000
    full_params["keep_rejected"] = 1000000
    validator = _validator(scene_config, placed, witness["box_wdh"])
    totals = {}
    enumerated = generate_candidates(
        surface, witness["box_wdh"],
        allowed_yaws=[0.0, 1.5707963, 3.14159265, -1.5707963],
        params=full_params, candidate_validator=validator, totals=totals)
    result = solve_placement(
        surface, witness["box_wdh"],
        allowed_yaws=[0.0, 1.5707963, 3.14159265, -1.5707963],
        params=params, candidate_validator=validator, atlas_annotate=None)
    prefix = ranked_feasible_candidates(result, 2000)
    captured = [
        row for row in _load(os.path.join(
            os.path.dirname(witness["request_path"]), "candidates.json"))["rows"]
        if row.get("reason") != "ATTEMPT_SUMMARY"]
    center = witness["center"]
    quat = witness["quaternion_xyzw"]

    def solver_pose(row):
        return _candidate_pose(row)

    def captured_pose(row):
        pose = row.get("slot_pose") or {}
        position = pose.get("position")
        if not position:
            return None
        return [float(v) for v in position], pose.get("quaternion_xyzw")

    found = {
        "B0": _find_pose(enumerated, center, quat, solver_pose),
        "B2": _find_pose(prefix, center, quat, solver_pose),
        "B3": _find_pose(captured, center, quat, captured_pose),
    }
    histogram = {}
    for row in enumerated:
        reason = str(row.get("reason") or "ok")
        histogram[reason] = histogram.get(reason, 0) + 1
    document = {
        "atlas_annotate": "not_evaluated",
        "atlas_reason": "offline strict atlas needs the live URDF hash",
        "n_enumerated": int(totals.get("n_enumerated") or len(enumerated)),
        "n_capacity_feasible": int(totals.get("n_capacity_feasible") or 0),
        "n_returned_production_params": len(result.get("candidates") or []),
        "n_feasible_prefix": len(prefix),
        "n_captured_slots": len(captured),
        "reject_histogram": histogram,
        "solve_success": bool(result.get("success")),
        "solve_reason": result.get("reason_code"),
        "boundaries": found,
        "B1": "not_evaluated",
        "saved_rows": len(captured),
        "online_returned": 196,
        "online_untried_not_in_dump": 35,
        "saved_row_note": (
            "The dump keeps 134 atlas drops and 27 tried slots. "
            "35 ordered slots were not written. Absence from the "
            "161 saved poses does not prove the solver omitted it."),
    }
    _dump("generation.json", document)
    return document


def _boundary_state(found, name):
    item = found.get(name) or {}
    if item.get("exact_index") is not None:
        return "present", "exact"
    if item.get("neighbor_index") is not None:
        return "present", "neighbor"
    if item.get("nearest_m") is None:
        return "not_evaluated", ""
    return "absent", "no_equivalent"


def _request_slots(witness, scene_config):
    from luggage_description.scene_tf_config_utils import (
        container_opening_normal_in_world,
        container_opening_target_point_in_world,
    )
    from luggage_planning.route_producer import _opening_depth

    rows = [
        row for row in _load(os.path.join(
            os.path.dirname(witness["request_path"]), "candidates.json"))["rows"]
        if row.get("slot_pose")]
    inward = container_opening_normal_in_world(scene_config)
    point = container_opening_target_point_in_world(scene_config)

    def depth(position):
        return float(_opening_depth({"position": position}, inward, point))

    scored = []
    for row in rows:
        position = [float(v) for v in row["slot_pose"]["position"]]
        scored.append((depth(position), row))
    witness_depth = depth(witness["center"])
    index = 0
    for position_depth, _row in scored:
        if position_depth <= witness_depth:
            index += 1
    witness_row = {
        "slot_pose": {
            "position": [float(v) for v in witness["center"]],
            "quaternion_xyzw": [
                float(v) for v in witness["quaternion_xyzw"]],
        },
        "placement_score": 0.0,
        "witness": True,
        "depth_m": witness_depth,
    }
    ordered = [row for _depth, row in scored]
    ordered.insert(index, witness_row)
    return ordered, index, witness_depth


def _build_many(document, surface, slots, stamp_ns):
    from st09_contracts.attachment import invert_transform

    attachment = document["attachment_transform"]
    grasp_pos, grasp_quat = invert_transform(
        tuple(attachment["translation"]),
        tuple(attachment["quaternion_xyzw"]))
    return {
        "request_id": "place-v2-2",
        "scene_revision": "witness-revision-2",
        "box_wdh": [float(v) for v in document["box_wdh"]],
        "box_id": "place-v2-2",
        "generation": 1,
        "slots": [{
            "request_id": "place-v2-2",
            "position": [float(v) for v in row["slot_pose"]["position"]],
            "quaternion_xyzw": [
                float(v) for v in row["slot_pose"]["quaternion_xyzw"]],
        } for row in slots],
        "slot_scores": [
            float(row.get("placement_score") or 0.0) for row in slots],
        "surface": {
            "frame_id": surface.get("frame") or "container_local",
            "resolution_m": float(surface["resolution"]),
            "origin_xy_container": [
                float(v) for v in surface["origin_local"]],
            "state": surface["state"],
            "stamp_ns": int(surface.get("acquisition_stamp_ns") or stamp_ns),
            "map_source": surface.get("map_source") or "measured",
            "placed_count": int(surface.get("placed_count") or 0),
            "cargo_observed": bool(surface.get("cargo_observed")),
            "commit_revision": int(surface.get("map_revision") or 2),
        },
        "inner_size": [float(v) for v in surface["inner_size"]],
        "geometry_hash": document["geometry_hash"],
        "map_revision": int(document["map_revision"]),
        "start_joints": [float(v) for v in document["start_joints"]],
        "grasp": {
            "position": [float(v) for v in grasp_pos],
            "quaternion_xyzw": [float(v) for v in grasp_quat],
            "stamp_ns": int(stamp_ns),
        },
        "container_transform": document["container_transform"],
    }


def _stats_for(surface, clock_ns):
    def stats():
        return {
            "success": True,
            "last_acquisition_stamp_ns": int(clock_ns()) - 1_000_000_000,
            "map_source": str(surface.get("map_source") or "measured"),
            "placed_count": int(surface.get("placed_count") or 0),
            "cargo_observed": bool(surface.get("cargo_observed")),
            "map_revision": int(surface.get("map_revision") or 0),
            "geometry_hash": str(surface.get("geometry_hash") or ""),
        }
    return stats


def _host(node, scene_config, clock_ns):
    from luggage_planning.atlas_runtime import (
        compute_reachability_scene_hash,
        compute_scene_tf_hash,
        load_atlas_runtime_for_scene,
        load_runtime_atlas,
    )
    from luggage_planning.urdf_identity import UrdfIdentity

    node._scene = scene_config
    node._scene_tf_hash = compute_scene_tf_hash(SCENE_TF)
    node._reachability_scene_hash = compute_reachability_scene_hash(SCENE_TF)
    node._atlas_cfg = load_atlas_runtime_for_scene(SCENE_TF, ATLAS_CFG)
    node._urdf_identity = UrdfIdentity(node)
    node._urdf_identity.start()
    node._atlas = None
    node._atlas_diag = {"loaded": False, "reason": "pending"}
    node._atlas_key = None
    node._clock_ns = clock_ns

    def runtime_atlas():
        node._urdf_identity.poll()
        urdf_hash = (
            node._urdf_identity.hash if node._urdf_identity.ready else "")
        key = (node._reachability_scene_hash, node._scene_tf_hash, urdf_hash)
        if key != node._atlas_key:
            atlas, diag = load_runtime_atlas(
                node._atlas_cfg, scene_tf_hash=node._scene_tf_hash,
                urdf_hash=urdf_hash,
                reachability_scene_hash=node._reachability_scene_hash)
            if (not diag.get("loaded") and diag.get("scene_tf_ok")
                    and not diag.get("urdf_ok")):
                atlas, diag = load_runtime_atlas(
                    node._atlas_cfg, scene_tf_hash=node._scene_tf_hash,
                    urdf_hash=ATLAS_URDF_HASH,
                    reachability_scene_hash=node._reachability_scene_hash)
                diag = dict(diag)
                diag["live_urdf_hash"] = urdf_hash
                diag["urdf_hash_override"] = ATLAS_URDF_HASH
            node._atlas, node._atlas_diag = atlas, diag
            node._atlas_key = key
        return node._atlas, node._atlas_diag

    node._runtime_atlas = runtime_atlas
    return node


def _wait_urdf(node, stop, timeout_s=30.0):
    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        node._urdf_identity.poll()
        if node._urdf_identity.ready:
            return True
        time.sleep(0.2)
    return False


def _sync(node, witness):
    from luggage_description.scene_tf_config_utils import load_scene_tf_config

    scene = _load(witness["scene_path"])
    surface = _load(witness["surface_path"])
    world = classify._sync_world(  # noqa: SLF001
        node, load_scene_tf_config(SCENE_TF), scene)
    probes._clear_occupancy(node)  # noqa: SLF001
    cells = probes._add_occupancy(node, surface)  # noqa: SLF001
    return world, cells


def _chain_diff(edges, reference):
    joints = []
    for edge in edges:
        trajectory = getattr(edge, "trajectory", None)
        for point in list(getattr(trajectory, "positions", []) or []):
            joints.append([float(v) for v in point])
    if not reference or len(joints) != len(reference):
        return {"match": False, "n_replay": len(joints),
                "n_reference": len(reference or []), "max_abs": None}
    delta = 0.0
    for left, right in zip(joints, reference):
        delta = max(delta, max(abs(float(a) - float(b))
                               for a, b in zip(left, right)))
    return {"match": delta == 0.0, "n_replay": len(joints),
            "n_reference": len(reference), "max_abs": delta}


def _scene_brief(states):
    brief = []
    for edge_id, state in (states or {}).items():
        placed = []
        for box in getattr(state, "placed_boxes", ()) or ():
            center = [float(v) for v in box.center]
            size = [float(v) for v in box.size_whd]
            placed.append({
                "id": box.object_id,
                "center": center,
                "size": size,
                "z_low": center[2] - 0.5 * size[2],
                "z_high": center[2] + 0.5 * size[2],
            })
        released = getattr(state, "released_box", None)
        brief.append({
            "edge_id": edge_id,
            "load": getattr(state, "load_state", ""),
            "payload": getattr(state, "payload", None) is not None,
            "released": None if released is None else released.object_id,
            "placed_boxes": placed,
        })
    return brief


def _row_brief(row):
    keep = {
        "slot_index", "reason", "reachability", "depth_m", "seed_count",
        "route_build_count", "placement_score", "certified", "decision",
    }
    brief = {key: row.get(key) for key in keep if key in row}
    detail = row.get("detail") or row.get("cert_detail") or {}
    if detail:
        brief["detail"] = detail
    return brief


def _classify_row(row):
    reason = str((row or {}).get("reason") or "")
    if reason in ("", "OK") and row and row.get("certified"):
        return "B9", "present", ""
    if reason.startswith("MAP_") or reason in (
            "MAP_REVALIDATION_UNAVAILABLE", "OBSERVATION_REQUIRED"):
        return "B9", "rejected", reason
    if reason in ("CERTIFICATION_TIMEOUT", "CERT_SERVICE_REJECTED"):
        return "B8", "budget_exceeded", reason
    if reason in ("CERT_COLLISION",) or "contact" in reason.lower():
        return "B8", "contact", reason
    if reason.startswith("LATTICE_") or reason in (
            "NO_FEASIBLE_ROUTE", "EDGE_BUILD_FAILED", "CANDIDATE_COLLISION",
            "SWEEP_COLLISION", "CARTESIAN_FRACTION", "EXCURSION",
            "RETREAT_TARGET_MISSING", "EGRESS_INFEASIBLE", "PATH_NOT_ROBUST"):
        return "B7", "rejected", reason
    if reason in ("IK_UNAVAILABLE", "ATLAS_SEED_MISS"):
        return "B6", "rejected", reason
    if reason.startswith("ATLAS_") or reason == "TARGET_SYNTHESIS_EMPTY":
        return "B4", "rejected", reason
    if reason == "PLANNING_BUDGET_EXHAUSTED":
        return "B5", "not_reached", reason
    return "B7", "rejected", reason or "unknown"


def _run_select(node, witness, slots, witness_index, clock_ns):
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_planning.route_producer import register_attach_from_request
    from luggage_planning.route_producer_factory import (
        build_production_route_producer,
    )

    document = _load(witness["request_path"])
    surface = _load(witness["surface_path"])
    stamp_ns = int(surface.get("acquisition_stamp_ns") or 1)
    scene_config = load_scene_tf_config(SCENE_TF)
    started = time.perf_counter()

    def clock():
        return stamp_ns + 1_000_000_000 + int(
            (time.perf_counter() - started) * 1e9)

    clock_ns[0] = clock
    _host(node, scene_config, clock)
    producer = build_production_route_producer(
        node, SCENE_TF, lambda: surface, _stats_for(surface, clock),
        scene=scene_config, clock_ns=clock, box_count=3,
        corridor_planner="lattice",
        scene_revision="witness-revision-2",
        vacuum_state_fn=lambda: {
            "attached": True, "vacuum_on": True, "received_ns": clock(),
        },
        site_poses_fn=lambda: None)
    if producer is None:
        return {"outcome": "not_evaluated", "reason": "producer_unavailable"}
    ready = producer.edge_builder.wait_ready(timeout_sec=60.0)
    if not ready:
        return {"outcome": "not_evaluated", "reason": "move_group_unavailable"}
    if not _wait_urdf(node, None):
        node.get_logger().info(
            "urdf identity not ready: %s" % node._urdf_identity.reason)
    saved = {}
    original = producer._certify

    def certify(*args, **kwargs):
        saved["args"] = args
        return original(*args, **kwargs)

    producer._certify = certify
    request = _build_many(document, surface, slots, stamp_ns)
    register_attach_from_request(
        producer.attachment_provider, request,
        box_wdh=request["box_wdh"], box_id=request["box_id"], generation=1)
    registered = producer.attachment_provider.latest_record(request["request_id"])
    wall_started = time.perf_counter()
    result = producer.plan_place_route(request)
    wall_s = time.perf_counter() - wall_started
    rows = list(result.get("rows") or [])
    witness_row = next((
        row for row in rows
        if int(row.get("slot_index", -1)) == int(witness_index)), None)
    summary = next((
        row for row in rows if row.get("reason") == "ATTEMPT_SUMMARY"), None)
    tried = list((summary or {}).get("tried_slot_indices") or [])
    if witness_row is None and witness_index not in tried:
        stage, state, reason = "B5", "not_reached", str(
            result.get("reason_code") or "")
    else:
        stage, state, reason = _classify_row(witness_row)
    edges = ()
    scene_states = None
    if saved.get("args"):
        route, snapshot, attachment, edges, candidate, request = saved["args"][:6]
        scene_states = producer.edge_scene_states(
            route, edges, snapshot, attachment, candidate, request)
    reference = _load(CHAIN_PATH).get("selected_chain") if os.path.isfile(
        CHAIN_PATH) else None
    return {
        "success": bool(result.get("success")),
        "reason_code": result.get("reason_code"),
        "wall_s": round(wall_s, 3),
        "n_rows": len(rows),
        "witness_index": witness_index,
        "witness_stage": stage,
        "witness_state": state,
        "witness_reason": reason,
        "witness_row": _row_brief(witness_row) if witness_row else None,
        "attempt": summary,
        "attachment_translation": [
            float(v) for v in registered.t_tcp_box_translation],
        "chain": _chain_diff(edges, reference),
        "scene_states": _scene_brief(scene_states),
        "n_edges": len(tuple(edges)),
        "atlas_diag": getattr(producer, "last_atlas_diag", None),
        "_edges": edges,
        "_saved": saved,
        "_producer": producer,
        "_request": request,
    }


def _diagnostic(node, packed):
    saved = packed.get("_saved") or {}
    edges = packed.get("_edges") or ()
    if not saved.get("args") or not edges:
        return {"status": "not_evaluated", "reason": "no_chain"}
    from luggage_planning.moveit_state_validity import MoveItStateValidityAdapter
    from luggage_planning.planning_scene_client import PlanningSceneClient

    producer = packed["_producer"]
    route, snapshot, attachment, edges, candidate, request = saved["args"][:6]
    states = producer.edge_scene_states(
        route, edges, snapshot, attachment, candidate, request)
    call_log = []
    adapter = MoveItStateValidityAdapter(
        node, fk=producer.fk, scene_client=PlanningSceneClient(node),
        service_timeout_s=DIAGNOSTIC_BUDGET_S + 20.0,
        budget_s=DIAGNOSTIC_BUDGET_S, call_log=call_log)
    adapter.load_edges(edges, scene_states=states)
    error = None
    try:
        adapter._ensure_verdicts()  # noqa: SLF001
    except Exception as exc:  # noqa: BLE001 - the refusal is the observation
        error = str(exc)
    record = call_log[-1] if call_log else {"detail": error}
    record["diagnostic"] = True
    record["budget_s"] = DIAGNOSTIC_BUDGET_S
    return record


def _strip(packed):
    return {
        key: value for key, value in packed.items()
        if not key.startswith("_")}


def moveit_runs(witness):
    import rclpy
    from rclpy.node import Node
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_perception.eval.isolated_domain import isolated_replay_env

    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        _dump("moveit.json", {"outcome": "not_evaluated", "residuals": residuals})
        return {"outcome": "not_evaluated", "residuals": residuals}
    scene_config = load_scene_tf_config(SCENE_TF)
    slots, index, depth = _request_slots(witness, scene_config)
    single = [slots[index]]
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    proc = score._start_moveit(  # noqa: SLF001
        env, SCENE_TF, os.path.join(OUT, "moveit.log"))
    rclpy.init()
    node = Node("witness_select_replay")
    stop, thread = score._spin(node)  # noqa: SLF001
    reports = {"a0": [], "a1": []}
    try:
        from luggage_msgs.srv import ValidateRouteV2
        client = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not client.wait_for_service(timeout_sec=90.0):
            return {"outcome": "not_evaluated", "reason": "validate_route_unavailable"}
        for repeat in range(2):
            world, cells = _sync(node, witness)
            clock_ns = [None]
            packed = _run_select(node, witness, single, 0, clock_ns)
            packed["repeat"] = repeat
            packed["world"] = world
            packed["occupancy_cells"] = cells
            if repeat == 0 and packed.get("n_edges"):
                packed["diagnostic"] = _diagnostic(node, packed)
                _dump("scene_objects.json", {
                    "world": world,
                    "occupancy_cells": cells,
                    "edges": packed.get("scene_states"),
                })
            reports["a0"].append(_strip(packed))
            _dump("a0_run_%d.json" % repeat, reports["a0"][-1])
            reset = getattr(
                getattr(packed.get("_producer"), "edge_builder", None),
                "reset_scene", None)
            if callable(reset):
                reset()
        a0_ok = all(
            row.get("n_edges", 0) > 0 and row.get("witness_stage") == "B8"
            for row in reports["a0"])
        if not a0_ok:
            reports["a1"] = {"skipped": "A0 did not reach certification"}
            return reports
        for repeat in range(2):
            _sync(node, witness)
            clock_ns = [None]
            packed = _run_select(node, witness, slots, index, clock_ns)
            packed["repeat"] = repeat
            packed["witness_depth"] = depth
            reports["a1"].append(_strip(packed))
            _dump("a1_run_%d.json" % repeat, reports["a1"][-1])
        return reports
    finally:
        stop.set()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        score._stop_moveit(proc)  # noqa: SLF001


def _decide(generated, runs):
    if not runs or runs.get("outcome") == "not_evaluated":
        return {"code": "not_evaluated", "reason": runs}
    a0 = runs.get("a0") or []
    if len(a0) != 2:
        return {"code": "not_evaluated", "reason": "a0_incomplete"}
    stages = [row.get("witness_stage") for row in a0]
    if any(stage != "B8" for stage in stages):
        return {"code": "WITNESS_NOT_REPRODUCIBLE", "a0_stages": stages,
                "a0_reasons": [row.get("witness_reason") for row in a0]}
    if stages[0] != stages[1] or [
            row.get("witness_reason") for row in a0][0] != [
            row.get("witness_reason") for row in a0][1]:
        return {"code": "NONDETERMINISTIC", "phase": "A0"}
    b0_state, b0_how = _boundary_state(generated["boundaries"], "B0")
    b2_state, _b2_how = _boundary_state(generated["boundaries"], "B2")
    b3_state, _b3_how = _boundary_state(generated["boundaries"], "B3")
    if b0_state == "absent":
        code = "DIVERGENCE_GENERATION"
    elif b2_state == "absent":
        code = "DIVERGENCE_PREFIX_TRUNCATION"
    elif b3_state == "absent":
        code = "ABSENT_FROM_CAPTURED_REQUEST"
    else:
        code = None
    a1 = runs.get("a1") or []
    if isinstance(a1, dict):
        return {"code": "WITNESS_NOT_REPRODUCIBLE", "detail": a1}
    if len(a1) == 2 and (
            a1[0].get("witness_stage") != a1[1].get("witness_stage")
            or a1[0].get("witness_reason") != a1[1].get("witness_reason")):
        return {
            "code": "NONDETERMINISTIC",
            "phase": "A1",
            "a1": [row.get("witness_reason") for row in a1],
            "a1_stage": [row.get("witness_stage") for row in a1],
            "b0": b0_state,
            "b2": b2_state,
            "b3": b3_state,
            "a0_reason": [row.get("witness_reason") for row in a0],
        }
    injected = None
    if a1:
        stage = a1[0].get("witness_stage")
        state = a1[0].get("witness_state")
        reason = a1[0].get("witness_reason")
        mapping = {
            ("B4", "rejected"): "DIVERGENCE_ADMISSION",
            ("B5", "not_reached"): "DIVERGENCE_SCHEDULING",
            ("B6", "rejected"): "DIVERGENCE_IK",
            ("B7", "rejected"): "DIVERGENCE_LATTICE",
            ("B8", "budget_exceeded"): "DIVERGENCE_CERT_TIMEOUT",
            ("B8", "contact"): "DIVERGENCE_CERT_CONTACT",
            ("B9", "rejected"): "DIVERGENCE_REVALIDATION",
            ("B9", "present"): "NO_DIVERGENCE",
        }
        injected = mapping.get((stage, state))
        if injected is None and stage == "B7":
            injected = "DIVERGENCE_LATTICE"
        if code is None:
            code = injected or "not_evaluated"
    return {
        "code": code or "not_evaluated",
        "b0": b0_state,
        "b2": b2_state,
        "b3": b3_state,
        "injected_select": injected,
        "a0_reason": [row.get("witness_reason") for row in a0],
        "a1_reason": [
            row.get("witness_reason") for row in a1] if a1 else [],
    }


def main():
    os.makedirs(OUT, exist_ok=True)
    started = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    witness = _witness()
    rev, dirty = _git()
    aligned = alignment(witness)
    generated = generation(witness)
    runs = moveit_runs(witness)
    if not isinstance(runs, dict) or "a0" in runs:
        _dump("moveit.json", runs)
    decision = _decide(generated, runs if isinstance(runs, dict) else {})
    _dump("verdict.json", decision)
    finished = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    residuals = score._residuals()  # noqa: SLF001
    inputs = [
        witness["request_path"], witness["surface_path"],
        witness["scene_path"], CHAIN_PATH, ATLAS_CFG, SCENE_TF]
    _dump("manifest.json", {
        "commit": rev,
        "dirty_count": dirty,
        "witness": "standard_m10",
        "domain": score.DOMAIN,
        "started": started,
        "finished": finished,
        "inputs_sha256": {path: _sha256(path) for path in inputs
                          if os.path.isfile(path)},
        "alignment_mismatches": [
            row["item"] for row in aligned["rows"]
            if row.get("status") == "mismatch"],
        "decision": decision.get("code"),
        "capture_complete": decision.get("code") != "not_evaluated",
        "replay_possible": True,
        "residuals_after": residuals,
        "gazebo": False,
        "replay_command": (
            "python3 scripts/st09_i/witness_select_replay.py"),
    })
    print(json.dumps(decision, indent=2, default=str))
    return 0 if decision.get("code") not in ("not_evaluated",) else 2


if __name__ == "__main__":
    sys.exit(main())
