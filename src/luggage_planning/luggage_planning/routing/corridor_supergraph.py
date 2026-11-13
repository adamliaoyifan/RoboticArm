"""Offline corridor supergraph for one frozen placement pair.

This module does not import RouteProducer and does not change online
planning. It reads the captured second-box dumps, rebuilds slot 656 from
the executed segments, and evaluates diagnostic shifts with container
geometry, the measured height map, and the placed-box body. Fields the
dumps do not contain are the string ``not_captured``.
"""

from __future__ import division

import hashlib
import json
import math
import os
import subprocess

import numpy as np

from luggage_description.container_geometry import (
    contains_oriented_box_through_aperture,
    descriptor_from_scene_config,
    oriented_box_corners,
)
import yaml

WORLD_T = (-1.5, 0.6, 0.0)
APERTURE = {"y0": -0.928, "y1": 0.398, "z0": 0.597, "z1": 2.009}
SUPPORT_MIN = 0.6
SAME_LAYER_M = 0.05
SUPPORT_TOL_M = 0.05
ROUTE_REASONS = (
    "EDGE_BUILD_FAILED",
    "LATTICE_NO_CHAIN",
    "LATTICE_LAYER_EMPTY",
    "LATTICE_OVER_BUDGET",
    "CERT_SERVICE_REJECTED",
)
STEPS = (
    "portal",
    "slot_approach",
    "slot_upper",
    "slot_contact",
    "detach",
    "slot_upper_retreat",
    "slot_approach_retreat",
    "portal_retreat",
    "home",
)
BASE = (
    "/tmp/elfin-st09-converge-i/docs/status/evidence/"
    "frame_migration/ST09-parallel-I-s5/m3"
)
SCENE_YAML = (
    "/tmp/elfin-st09-converge-i/src/luggage_description/"
    "config/scene_tf_deploy_update.yaml"
)
RUNS = (
    ("std60", "g3_online_n3_0d856c7", "standard"),
    ("std180", "g3_online_n3_budget180", "standard"),
    ("c180", "g3_online_n3_carryon3", "carryon"),
)


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def to_local(xyz):
    x, y, z = (float(v) for v in xyz)
    return [-(x - WORLD_T[0]), WORLD_T[1] - y, z]


def to_world(xyz):
    x, y, z = (float(v) for v in xyz)
    return [-x + WORLD_T[0], -y + WORLD_T[1], z]


def yaw_of(quat):
    x, y, z, w = (float(v) for v in quat)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def halves(size, yaw):
    hx, hy, hz = (0.5 * float(v) for v in size)
    if abs(math.sin(yaw)) > abs(math.cos(yaw)):
        hx, hy = hy, hx
    return (hx, hy, hz)


def aabb_clearance(center_a, half_a, center_b, half_b):
    """Euclidean gap between two axis-aligned boxes. Overlap returns 0."""
    outside = []
    penetration = []
    for index in range(3):
        gap = (abs(center_a[index] - center_b[index])
               - half_a[index] - half_b[index])
        outside.append(max(0.0, gap))
        penetration.append(gap)
    if max(penetration) < 0.0:
        return 0.0, round(-max(penetration), 4)
    return round(math.sqrt(sum(value * value for value in outside)), 4), 0.0


def lattice_of(row):
    detail = row.get("detail") or {}
    lattice = detail.get("lattice") or row.get("lattice") or {}
    return lattice if isinstance(lattice, dict) else {}


def first_contact(row):
    block = lattice_of(row).get("first_block")
    if not isinstance(block, dict):
        return None, None
    contacts = block.get("contacts") or []
    if not contacts:
        return block.get("load"), None
    contact = contacts[0]
    return block.get("load"), (contact.get("body_1"), contact.get("body_2"))


def classify_route(row):
    """Map one captured route row onto the fixed taxonomy.

    A taxonomy value is set only when the dump itself names the stage.
    ``GOAL_STATE_INVALID`` on a portal edge has no contact pair, so its
    refined class stays ``not_captured`` and the dump reason is kept.
    """
    reason = row.get("reason")
    lattice = lattice_of(row)
    load, pair = first_contact(row)
    bodies = set(pair or ())
    gates = {}
    attempts = lattice.get("anchor_attempts") or []
    if attempts and isinstance(attempts[0], dict):
        gates = attempts[0].get("gate_counts") or {}
    taxonomy = "not_captured"
    stage = "not_captured"
    if reason == "LATTICE_LAYER_EMPTY":
        taxonomy = "IK_LAYER_EMPTY"
        stage = str(lattice.get("lattice_reason") or "IK_LAYER_EMPTY")
    elif reason == "LATTICE_NO_CHAIN" and "placed_0_0_0" in bodies:
        taxonomy = "PAYLOAD_HITS_PLACED_BOX"
        stage = "carrying"
    elif reason == "LATTICE_NO_CHAIN" and (
            "elfin_link3" in bodies or "airport_container_real" in bodies):
        taxonomy = "ARM_HITS_CONTAINER"
        stage = "carrying"
    elif reason == "LATTICE_NO_CHAIN" and not pair:
        taxonomy = "BRANCH_DISCONNECTED"
        stage = "jump_or_branch_gate"
    elif reason == "OK":
        taxonomy = "COMPLETE_ROUTE"
        stage = "certified"
    record = {
        "dump_reason": reason,
        "taxonomy": taxonomy,
        "failure_stage": stage,
        "collision_pair": list(pair) if pair else "not_captured",
        "collision_load": load or "not_captured",
        "contact_point": "not_captured",
        "min_clearance_m": lattice.get("min_collision_distance_m", "not_captured"),
        "gate_counts": gates or "not_captured",
        "ik_solutions": "not_captured",
        "waypoints": "not_captured" if not row.get("waypoints") else row["waypoints"],
        "retreat_edge_failed": False,
    }
    edges = row.get("edges") or []
    record["edge_classes"] = [edge.get("edge_class") for edge in edges]
    record["edge_error_codes"] = [edge.get("error_code") for edge in edges]
    record["retreat_edge_failed"] = any(
        edge.get("edge_class") in ("RETREAT", "RETURN")
        and edge.get("error_code") not in (1, 0, None)
        for edge in edges)
    if reason == "EDGE_BUILD_FAILED":
        record["failure_stage"] = "portal_transit"
    elif reason == "CERT_SERVICE_REJECTED":
        record["failure_stage"] = "traverse_certification"
        record["return_edges_moveit_success"] = all(
            edge.get("error_code") == 1
            for edge in edges if edge.get("edge_class") == "RETURN")
    elif reason == "LATTICE_OVER_BUDGET":
        record["failure_stage"] = "lattice_budget"
    return record


def support_ratio(surface, center, size, yaw):
    hx, hy, _hz = halves(size, yaw)
    origin = surface["origin_local"]
    resolution = float(surface["resolution"])
    heights = surface["height"]
    samples = []
    occupied = 0
    for ix, row in enumerate(heights):
        for iy, height in enumerate(row):
            x = origin[0] + (ix + 0.5) * resolution
            y = origin[1] + (iy + 0.5) * resolution
            if abs(x - center[0]) <= hx and abs(y - center[1]) <= hy:
                samples.append(float(height))
                if surface["state"][ix][iy] == "occupied":
                    occupied += 1
    if not samples:
        return "not_captured", occupied
    peak = max(samples)
    ratio = sum(1 for value in samples if value >= peak - SUPPORT_TOL_M) / len(samples)
    return round(ratio, 4), occupied


def aperture_ok(geometry, center, size, yaw):
    """Closed faces via the hull kernel, and the -X protrusion via the door.

    A captured center can sit a fraction of a millimetre under the floor
    because the height was rounded. The kernel rejects that. Lift at most
    1 mm so the test matches a box that is resting on the floor.
    """
    bottom = float(center[2]) - 0.5 * float(size[2])
    if geometry.floor_z - 1e-3 <= bottom < geometry.floor_z:
        center = [
            center[0], center[1],
            geometry.floor_z + 0.5 * float(size[2]),
        ]
    if not contains_oriented_box_through_aperture(geometry, center, size, yaw):
        return False
    for corner in oriented_box_corners(center, size, yaw):
        if corner[0] < -geometry.half_x:
            if not (APERTURE["y0"] <= corner[1] <= APERTURE["y1"]
                    and APERTURE["z0"] <= corner[2] <= APERTURE["z1"]):
                return False
    return True


def payload_center_from_tcp(tcp_local, height):
    return [tcp_local[0], tcp_local[1], tcp_local[2] - 0.5 * height]


def sweep_against_placed(tcp_locals, size, yaw, placed_center, placed_half):
    """Carrying payload boxes along captured TCP samples."""
    if not tcp_locals:
        return {
            "min_clearance_m": "not_captured",
            "first_collision_sample": "not_captured",
            "collision_pair": "not_captured",
        }
    half = halves(size, yaw)
    best = None
    first = None
    for index, tcp in enumerate(tcp_locals):
        center = payload_center_from_tcp(tcp, size[2])
        clearance, penetration = aabb_clearance(
            center, half, placed_center, placed_half)
        if best is None or clearance < best:
            best = clearance
        if clearance <= 0.0 and first is None:
            first = {
                "sample_index": index,
                "payload_center": [round(v, 4) for v in center],
                "penetration_m": penetration,
                "collision_pair": ["cert-payload", "placed_0_0_0"],
            }
    return {
        "min_clearance_m": best,
        "first_collision_sample": first or "none",
        "collision_pair": (
            ["cert-payload", "placed_0_0_0"] if first else "none"),
        "kind": "carrying_swept_aabb",
    }


def row_tcp_local(row):
    targets = row.get("targets") or []
    if targets and targets[0] and targets[0][0]:
        return to_local(targets[0][0])
    return None


def row_anchors_local(row):
    lattice = lattice_of(row)
    request = lattice.get("request") or {}
    anchors = request.get("anchors") or {}
    local = {}
    for name in ("portal", "slot_approach", "slot_upper", "slot"):
        point = anchors.get(name)
        if point:
            local[name] = to_local(point)
    return local


def load_run(run_id, directory, box_label):
    root = os.path.join(BASE, directory, "dumps")
    request = json.load(open(os.path.join(root, "box_01_paths/planning_request.json")))
    document = json.load(open(os.path.join(root, "box_01_paths/candidates.json")))
    surface = json.load(open(os.path.join(
        root, "trial_01_PLANNING_BUDGET_EXHAUSTED/replay/surface_2d.json")))
    rows = []
    for row in document["rows"]:
        if row.get("reason") not in ROUTE_REASONS:
            continue
        classified = classify_route(row)
        pose = row["slot_pose"]
        center = [float(v) for v in pose["position"]]
        yaw = yaw_of(pose["quaternion_xyzw"])
        size = [float(v) for v in request["box_wdh"]]
        tcp = row_tcp_local(row)
        rows.append({
            "run": run_id,
            "box": box_label,
            "slot_index": int(row["slot_index"]),
            "candidate_id": row.get("candidate_id"),
            "center": [round(v, 6) for v in center],
            "yaw_rad": round(yaw, 6),
            "size_m": [round(v, 6) for v in size],
            "tcp_local": [round(v, 6) for v in tcp] if tcp else "not_captured",
            "anchors_local": {
                key: [round(v, 4) for v in value]
                for key, value in row_anchors_local(row).items()
            } or "not_captured",
            "atlas_status": _atlas_status(row),
            "hard_reject_safe": _hard_reject(row),
            "reachability": row.get("reachability"),
            "depth_m": row.get("depth_m", "not_captured"),
            "seed_count": len(row.get("atlas_seed_q") or []),
            **classified,
        })
    return {
        "id": run_id,
        "box": box_label,
        "request": request,
        "surface": surface,
        "routes": rows,
        "candidate_count": len(request.get("candidates") or []),
        "paths": {
            "candidates": os.path.join(root, "box_01_paths/candidates.json"),
            "request": os.path.join(root, "box_01_paths/planning_request.json"),
            "surface": os.path.join(
                root, "trial_01_PLANNING_BUDGET_EXHAUSTED/replay/surface_2d.json"),
        },
    }


def _atlas_status(row):
    queries = row.get("atlas_queries") or []
    if not queries:
        return "not_captured"
    return max(int(item.get("status") or 0) for item in queries)


def _hard_reject(row):
    queries = row.get("atlas_queries") or []
    if not queries:
        return "not_captured"
    return any(bool(item.get("hard_reject_safe")) for item in queries)


def load_placed():
    root = os.path.join(BASE, "g3_online_n3_budget180", "dumps")
    document = json.load(open(os.path.join(root, "box_00_paths/candidates.json")))
    request = json.load(open(os.path.join(root, "box_00_paths/planning_request.json")))
    ok = next(row for row in document["rows"] if row.get("reason") == "OK")
    refused = next(
        row for row in document["rows"]
        if row.get("slot_index") == 639 and row.get("reason") == "LATTICE_NO_CHAIN")
    segments = []
    with open(os.path.join(root, "trial_00_ok/segments.jsonl"), encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                segments.append(json.loads(line))
    pose = ok["slot_pose"]
    scene = json.load(open(os.path.join(
        root, "trial_01_PLANNING_BUDGET_EXHAUSTED/scene.json")))
    body = next(item for item in scene["world_objects"] if item["id"] == "placed_0_0_0")
    # The collision body is the authority for the contact. Its world yaw is
    # pi, so in container_link the 0.495 m side lies on X. The slot quaternion
    # is a different yaw and is kept only as a captured field.
    center = to_local(body["xyz"])
    world_yaw = yaw_of(body["quat"])
    container_yaw = world_yaw - math.pi
    return {
        "slot_index": 656,
        "center": center,
        "yaw_rad": container_yaw,
        "slot_quaternion_yaw_rad": yaw_of(pose["quaternion_xyzw"]),
        "collision_quaternion_xyzw": body["quat"],
        "size_m": [float(v) for v in body["size"]],
        "taxonomy": "COMPLETE_ROUTE",
        "certified": bool(ok.get("certified")),
        "segment_names": [item.get("name") for item in segments],
        "segment_fractions": [item.get("fraction") for item in segments],
        "refused_639": classify_route(refused),
        "urdf_hash": ok.get("urdf_hash"),
        "paths": {
            "candidates": os.path.join(root, "box_00_paths/candidates.json"),
            "request": os.path.join(root, "box_00_paths/planning_request.json"),
            "segments": os.path.join(root, "trial_00_ok/segments.jsonl"),
        },
    }


def git_freeze(repo):
    commit = subprocess.check_output(
        ["git", "-C", repo, "rev-parse", "HEAD"], text=True).strip()
    porcelain = subprocess.check_output(
        ["git", "-C", repo, "status", "--porcelain"], text=True)
    return {
        "repo": repo,
        "commit": commit,
        "dirty_count": len([line for line in porcelain.splitlines() if line.strip()]),
        "porcelain_sha256": hashlib.sha256(porcelain.encode()).hexdigest(),
    }


def evaluate_pose(geometry, surface, placed, center, size, yaw, tcp_locals,
                  scene_mode):
    half = halves(size, yaw)
    placed_half = halves(placed["size_m"], placed["yaw_rad"])
    bounds = aperture_ok(geometry, center, size, yaw)
    ratio, occupied_cells = support_ratio(surface, center, size, yaw)
    final_clearance, final_penetration = aabb_clearance(
        center, half, placed["center"], placed_half)
    xy_clearance, _xy_pen = aabb_clearance(
        [center[0], center[1], 0.0], (half[0], half[1], 1.0),
        [placed["center"][0], placed["center"][1], 0.0],
        (placed_half[0], placed_half[1], 1.0))
    sweep = sweep_against_placed(
        tcp_locals, size, yaw, placed["center"], placed_half)
    occupancy_hit = occupied_cells > 0
    exact_hit = final_clearance <= 0.0 or (
        sweep["min_clearance_m"] != "not_captured"
        and sweep["min_clearance_m"] <= 0.0)
    if scene_mode == "occupancy":
        scene_hit = occupancy_hit
    elif scene_mode == "exact_box":
        scene_hit = exact_hit
    else:
        scene_hit = occupancy_hit or exact_hit
    support_ok = ratio != "not_captured" and ratio >= SUPPORT_MIN
    return {
        "bounds_ok": bool(bounds),
        "support_ratio": ratio,
        "support_ok": bool(support_ok),
        "same_layer_gap_m": xy_clearance,
        "same_layer_ok": xy_clearance >= SAME_LAYER_M,
        "final_footprint_clearance_m": final_clearance,
        "final_footprint_penetration_m": final_penetration,
        "final_footprint_invalid": final_clearance <= 0.0,
        "sweep": sweep,
        "occupied_cells": occupied_cells,
        "scene_mode": scene_mode,
        "scene_blocked": bool(scene_hit),
        "certification": "not_captured",
        "arm_collision": "not_captured",
    }


def opening_tcp_shift(tcp_locals, dx):
    if not tcp_locals:
        return []
    return [[point[0] + dx, point[1], point[2]] for point in tcp_locals]


def red_column(routes):
    """Outermost carrying-collision column: smallest container x."""
    hits = [row for row in routes if row["taxonomy"] == "PAYLOAD_HITS_PLACED_BOX"]
    if not hits:
        return None
    xs = sorted({round(row["center"][0], 3) for row in hits})
    outer = xs[0]
    column = [row for row in hits if abs(row["center"][0] - outer) < 1e-3]
    return column


def build_probes(geometry, runs, placed):
    probes = []
    by_id = {run["id"]: run for run in runs}
    for run_id, label in (("std180", "standard"), ("c180", "carryon")):
        column = red_column(by_id[run_id]["routes"])
        if not column:
            continue
        seed = column[0]
        anchors = seed["anchors_local"]
        tcp_locals = []
        if isinstance(anchors, dict):
            for name in ("portal", "slot_approach", "slot_upper", "slot"):
                if name in anchors:
                    tcp_locals.append(anchors[name])
        if not tcp_locals and seed["tcp_local"] != "not_captured":
            tcp_locals = [seed["tcp_local"]]
        for shift in (0.05, 0.10, 0.15):
            center = [seed["center"][0] - shift, seed["center"][1], seed["center"][2]]
            probes.append(_probe_record(
                geometry, by_id[run_id]["surface"], placed, seed, center,
                seed["yaw_rad"], opening_tcp_shift(tcp_locals, -shift),
                "both", "opening_%.0fcm" % (shift * 100), label))
        flipped = seed["yaw_rad"] + math.pi / 2.0
        probes.append(_probe_record(
            geometry, by_id[run_id]["surface"], placed, seed, list(seed["center"]),
            flipped, tcp_locals, "both", "yaw_toggle", label))
        for mode in ("occupancy", "exact_box", "both"):
            probes.append(_probe_record(
                geometry, by_id[run_id]["surface"], placed, seed,
                list(seed["center"]), seed["yaw_rad"], tcp_locals, mode,
                "scene_%s" % mode, label))
    return probes


def _probe_record(geometry, surface, placed, seed, center, yaw, tcp_locals,
                  scene_mode, kind, box):
    result = evaluate_pose(
        geometry, surface, placed, center, seed["size_m"], yaw,
        tcp_locals, scene_mode)
    geometric_block = (
        (not result["bounds_ok"])
        or (not result["support_ok"])
        or (not result["same_layer_ok"])
        or result["final_footprint_invalid"]
        or (result["sweep"]["min_clearance_m"] != "not_captured"
            and result["sweep"]["min_clearance_m"] <= 0.0
            and scene_mode != "occupancy")
    )
    return {
        "kind": kind,
        "box": box,
        "diagnostic_candidate": True,
        "source_slot": seed["slot_index"],
        "source_run": seed["run"],
        "center": [round(v, 4) for v in center],
        "yaw_rad": round(yaw, 6),
        "size_m": seed["size_m"],
        "ik_contact": "not_captured",
        "ik_approach": "not_captured",
        "ik_retreat": "not_captured",
        "detach": "not_captured",
        "return_path": "not_captured",
        "geometric_block": bool(geometric_block),
        **result,
    }


def try_ik(probes, placed_request_start):
    """Refine one captured seed onto probe TCP poses. Failure is not emptiness."""
    try:
        from luggage_planning.motion.offline_ik import load_urdf, refine_to_pose
        urdf = load_urdf(scene_tf_config=SCENE_YAML)
        path = urdf.path_to("suction_contact_frame")
    except Exception as exc:  # noqa: BLE001 — capture boundary, not a guess
        for probe in probes:
            probe["ik_error"] = "not_captured"
            probe["ik_error_type"] = type(exc).__name__
        return
    seed = placed_request_start
    for probe in probes:
        if probe["kind"].startswith("scene_"):
            continue
        tcp = to_world([
            probe["center"][0], probe["center"][1],
            probe["center"][2] + 0.5 * probe["size_m"][2] + 0.05])
        quat = [0.0, 1.0, 0.0, 0.0]
        joints, info = refine_to_pose(urdf, path, tcp, quat, seed)
        probe["ik_contact"] = {
            "ok": bool(info.get("ok")),
            "pos_err_m": info.get("pos_err_m"),
            "from_seed": "placed_box_start_joints",
            "other_branches": "not_captured",
        }
        probe["ik_solution_id"] = (
            "seed0" if joints else "not_captured")


def build_supergraph(routes, placed, probes):
    nodes = []
    edges = []
    nodes.append({
        "id": "slot656",
        "candidate_id": "slot-656",
        "corridor_step": "complete",
        "ik_solution_id": "not_captured",
        "load_state": "both",
        "taxonomy": "COMPLETE_ROUTE",
        "segment_names": placed["segment_names"],
    })
    for route in routes:
        node_id = "%s-%s" % (route["run"], route["slot_index"])
        nodes.append({
            "id": node_id,
            "candidate_id": route["candidate_id"],
            "corridor_step": route["failure_stage"],
            "ik_solution_id": "not_captured",
            "load_state": route["collision_load"],
            "taxonomy": route["taxonomy"],
            "dump_reason": route["dump_reason"],
            "atlas_status": route["atlas_status"],
            "hard_reject_safe": route["hard_reject_safe"],
        })
        edges.append({
            "src": "portal",
            "dst": node_id,
            "valid": route["taxonomy"] == "COMPLETE_ROUTE",
            "collision_pair": route["collision_pair"],
            "clearance_m": route["min_clearance_m"],
        })
    for index, probe in enumerate(probes):
        nodes.append({
            "id": "probe-%d" % index,
            "candidate_id": "diagnostic_candidate",
            "corridor_step": "diagnostic",
            "ik_solution_id": probe.get("ik_solution_id", "not_captured"),
            "load_state": "carrying",
            "taxonomy": (
                "FINAL_FOOTPRINT_INVALID" if probe["final_footprint_invalid"]
                else "PAYLOAD_HITS_PLACED_BOX"
                if probe["sweep"]["min_clearance_m"] not in ("not_captured", None)
                and probe["sweep"]["min_clearance_m"] <= 0.0
                else "not_captured"),
            "kind": probe["kind"],
            "box": probe["box"],
        })
    return {"nodes": nodes, "edges": edges, "steps": list(STEPS)}


def decide(routes, probes):
    opening = [probe for probe in probes if probe["kind"].startswith("opening_")]
    if len(opening) < 6:
        return {
            "code": None,
            "reason": "fewer than 6 opening probes",
        }
    blocked = [probe for probe in opening if probe["geometric_block"]]
    if len(blocked) == len(opening):
        return {
            "code": "GEOMETRICALLY_BLOCKED",
            "reason": (
                "Each 5/10/15 cm opening shift violates the hull, "
                "support, same-layer gap, final footprint, or the "
                "carrying swept box against slot 656."
            ),
            "evidence": [
                {
                    "box": probe["box"],
                    "kind": probe["kind"],
                    "center": probe["center"],
                    "bounds_ok": probe["bounds_ok"],
                    "support_ratio": probe["support_ratio"],
                    "same_layer_gap_m": probe["same_layer_gap_m"],
                    "final_footprint_clearance_m": probe["final_footprint_clearance_m"],
                    "sweep_min_clearance_m": probe["sweep"]["min_clearance_m"],
                }
                for probe in opening
            ],
        }
    clear = [probe for probe in opening if not probe["geometric_block"]]
    return {
        "code": None,
        "reason": (
            "At least one opening probe is geometrically clear. "
            "Certification is not_captured, so none of the six "
            "decision codes is earned."
        ),
        "geometrically_clear": [
            {"box": probe["box"], "kind": probe["kind"], "center": probe["center"]}
            for probe in clear
        ],
    }


def write_html(path, routes, placed, probes, decision):
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    colors = {
        "PAYLOAD_HITS_PLACED_BOX": "#d03b3b",
        "ARM_HITS_CONTAINER": "#1c5cab",
        "IK_LAYER_EMPTY": "#1baf7a",
        "BRANCH_DISCONNECTED": "#52514e",
        "COMPLETE_ROUTE": "#1baf7a",
        "not_captured": "#2a78d6",
    }
    fig = make_subplots(
        rows=2, cols=2,
        specs=[[{"type": "xy"}, {"type": "xy"}],
               [{"type": "xy"}, {"type": "scene"}]],
        subplot_titles=(
            "Container XY, TCP targets",
            "Candidate x failure stage",
            "Sweep clearance",
            "TCP and placed box",
        ))
    placed_half = halves(placed["size_m"], placed["yaw_rad"])
    fig.add_trace(go.Scatter(
        x=[placed["center"][0]], y=[placed["center"][1]],
        mode="markers", marker=dict(size=14, color="#efeeea",
                                     line=dict(color="#0b0b0b", width=1)),
        name="slot 656",
        customdata=[["slot 656 COMPLETE_ROUTE"] * 1],
        hovertext=["slot 656 complete certified route"],
    ), row=1, col=1)
    # footprint rectangle via a closed line
    c, h = placed["center"], placed_half
    fig.add_trace(go.Scatter(
        x=[c[0] - h[0], c[0] + h[0], c[0] + h[0], c[0] - h[0], c[0] - h[0]],
        y=[c[1] - h[1], c[1] - h[1], c[1] + h[1], c[1] + h[1], c[1] - h[1]],
        mode="lines", line=dict(color="#0b0b0b"), name="656 footprint",
        hoverinfo="skip",
    ), row=1, col=1)
    for taxonomy in sorted({row["taxonomy"] for row in routes}):
        group = [row for row in routes if row["taxonomy"] == taxonomy]
        fig.add_trace(go.Scatter(
            x=[row["center"][0] for row in group],
            y=[row["center"][1] for row in group],
            mode="markers",
            marker=dict(size=8, color=colors.get(taxonomy, "#2a78d6"),
                        symbol="x" if taxonomy == "PAYLOAD_HITS_PLACED_BOX" else "circle"),
            name=taxonomy,
            customdata=[[
                row["run"], row["slot_index"], row["dump_reason"],
                row["taxonomy"], row["collision_pair"], row["failure_stage"],
            ] for row in group],
            hovertemplate=(
                "%{customdata[0]} slot %{customdata[1]}<br>"
                "%{customdata[2]} → %{customdata[3]}<br>"
                "pair %{customdata[4]}<extra></extra>"),
        ), row=1, col=1)
    stage_index = {}
    for row in routes:
        stage_index.setdefault(row["failure_stage"], len(stage_index))
    fig.add_trace(go.Scatter(
        x=[row["slot_index"] for row in routes],
        y=[stage_index[row["failure_stage"]] for row in routes],
        mode="markers",
        marker=dict(
            color=[colors.get(row["taxonomy"], "#2a78d6") for row in routes],
            size=7),
        name="stages",
        showlegend=False,
    ), row=1, col=2)
    clearances = []
    labels = []
    for row in routes:
        value = row["min_clearance_m"]
        if isinstance(value, (int, float)):
            clearances.append(value)
            labels.append("%s:%s" % (row["run"], row["slot_index"]))
    fig.add_trace(go.Bar(
        x=labels, y=clearances, marker_color="#2a78d6", name="captured clearance",
        showlegend=False,
    ), row=2, col=1)
    fig.add_trace(go.Scatter3d(
        x=[row["center"][0] for row in routes if row["tcp_local"] != "not_captured"],
        y=[row["center"][1] for row in routes if row["tcp_local"] != "not_captured"],
        z=[row["tcp_local"][2] for row in routes if row["tcp_local"] != "not_captured"],
        mode="markers",
        marker=dict(size=3, color="#2a78d6"),
        name="TCP",
    ), row=2, col=2)
    fig.add_trace(go.Scatter3d(
        x=[placed["center"][0]], y=[placed["center"][1]], z=[placed["center"][2]],
        mode="markers", marker=dict(size=6, color="#0b0b0b"), name="656 center",
    ), row=2, col=2)
    fig.update_layout(
        height=900, width=1200,
        title="Placement corridor explorer — diagnostic, not a production plan",
        legend=dict(orientation="h"),
    )
    fig.update_xaxes(title_text="container x (m)", row=1, col=1)
    fig.update_yaxes(title_text="container y (m)", scaleanchor="x", row=1, col=1)
    body = fig.to_html(full_html=False, include_plotlyjs=True)
    probe_rows = "".join(
        "<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td>"
        "<td>%s</td><td>%s</td><td>%s</td></tr>" % (
            probe["box"], probe["kind"], probe["center"],
            probe["bounds_ok"], probe["support_ratio"],
            probe["same_layer_gap_m"],
            probe["final_footprint_clearance_m"],
            probe["sweep"]["min_clearance_m"],
        )
        for probe in probes)
    code = decision.get("code") or "not_earned"
    html = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>placement corridor explorer</title>
<style>
body { font-family: sans-serif; color: #0b0b0b; margin: 16px; }
table { border-collapse: collapse; font-size: 13px; }
td, th { border: 1px solid #ccc; padding: 4px 6px; }
#detail { white-space: pre-wrap; background: #efeeea; padding: 8px; }
</style></head><body>
<h1>Placement corridor explorer</h1>
<p>Decision: <strong>%s</strong>. Diagnostic candidates are not written to the planner.</p>
<div id="detail">Click a TCP marker. Captured collision pairs are in the hover.</div>
%s
<h2>Diagnostic probes</h2>
<table>
<tr><th>box</th><th>probe</th><th>center</th><th>bounds</th><th>support</th>
<th>same-layer gap</th><th>final clearance</th><th>sweep clearance</th></tr>
%s
</table>
<script>
var plot = document.querySelector('.plotly-graph-div');
if (plot) {
  plot.on('plotly_click', function(event) {
    var point = event.points[0];
    var data = point.customdata ? JSON.stringify(point.customdata) : point.hovertext || '';
    document.getElementById('detail').textContent = data;
  });
}
</script>
</body></html>
""" % (code, body, probe_rows)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(html)
    return os.path.getsize(path)


def main(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    scene = yaml.safe_load(open(SCENE_YAML, encoding="utf-8"))
    geometry = descriptor_from_scene_config(scene)
    runs = [load_run(*spec) for spec in RUNS]
    placed = load_placed()
    routes = [route for run in runs for route in run["routes"]]
    expected = {"std60": 12, "std180": 23, "c180": 26}
    for run in runs:
        got = len(run["routes"])
        if got != expected[run["id"]]:
            raise SystemExit("%s routes %d != %d" % (run["id"], got, expected[run["id"]]))
    if len(routes) != 61:
        raise SystemExit("route count %d != 61" % len(routes))
    # 100 percent dump agreement: re-read reasons.
    for run in runs:
        document = json.load(open(run["paths"]["candidates"]))
        source = {
            (int(row["slot_index"]), row["reason"])
            for row in document["rows"] if row.get("reason") in ROUTE_REASONS
        }
        built = {(row["slot_index"], row["dump_reason"]) for row in run["routes"]}
        if built != source:
            raise SystemExit("dump mismatch %s" % run["id"])
    if not placed["certified"] or "descend" not in placed["segment_names"]:
        raise SystemExit("slot 656 route was not reconstructed")
    probes = build_probes(geometry, runs, placed)
    opening = [probe for probe in probes if probe["kind"].startswith("opening_")]
    if len(opening) < 6:
        raise SystemExit("opening probes %d" % len(opening))
    baselines = [
        probe for probe in probes if probe["kind"] == "scene_exact_box"]
    for probe in baselines:
        if not probe["final_footprint_invalid"]:
            raise SystemExit(
                "placed-box model does not reproduce %s red column" % probe["box"])
    start = runs[1]["request"]["start_joints"]
    try_ik(probes, start)
    graph = build_supergraph(routes, placed, probes)
    decision = decide(routes, probes)
    hashes = {}
    for run in runs:
        for label, path in run["paths"].items():
            hashes["%s:%s" % (run["id"], label)] = sha256_file(path)
    for label, path in placed["paths"].items():
        hashes["box0:%s" % label] = sha256_file(path)
    hashes["scene_yaml"] = sha256_file(SCENE_YAML)
    manifest = {
        "generation": "g1",
        "git": git_freeze("/tmp/elfin-st09-converge-i"),
        "input_sha256": hashes,
        "scene_revision": {
            "map_revision": runs[1]["request"]["map_revision"],
            "map_source": runs[1]["request"]["map_source"],
            "geometry_hash": runs[1]["request"]["geometry_hash"],
        },
        "urdf_hash": placed["urdf_hash"],
        "atlas_identity": runs[1]["request"]["atlas_identity"],
        "attachment_transform": runs[1]["request"]["attachment_transform"],
        "thresholds": {
            "cartesian_fraction_min": 0.95,
            "same_layer_clearance_m": SAME_LAYER_M,
            "support_coverage_min": SUPPORT_MIN,
            "atlas_hard_reject_only": True,
        },
        "capture_gaps": [
            "selector candidate list omits the untried seeded tail",
            "ik branch signatures",
            "waypoint cartesian samples",
            "collision contact xyz",
            "ValidateRouteV2 certification of diagnostic probes",
            "arm link versus container mesh on diagnostic probes",
        ],
        "route_count": 61,
        "slot_656_segments": placed["segment_names"],
        "online_planner_modified": False,
        "scored_simulation_started": False,
    }
    _write(out_dir, "manifest.json", manifest)
    _write(out_dir, "supergraph.json", graph)
    _write_jsonl(out_dir, "candidate_results.jsonl", routes)
    sweeps = []
    for route in routes:
        if route["taxonomy"] in (
                "PAYLOAD_HITS_PLACED_BOX", "ARM_HITS_CONTAINER"):
            sweeps.append({
                "run": route["run"],
                "slot_index": route["slot_index"],
                "kind": "captured_first_block",
                "collision_pair": route["collision_pair"],
                "min_clearance_m": route["min_clearance_m"],
                "contact_point": "not_captured",
                "load_state": route["collision_load"],
            })
    for probe in probes:
        sweeps.append({
            "run": probe["source_run"],
            "kind": probe["kind"],
            "box": probe["box"],
            "scene_mode": probe["scene_mode"],
            "center": probe["center"],
            "bounds_ok": probe["bounds_ok"],
            "support_ratio": probe["support_ratio"],
            "same_layer_gap_m": probe["same_layer_gap_m"],
            "sweep": probe["sweep"],
            "final_footprint_clearance_m": probe["final_footprint_clearance_m"],
            "final_footprint_invalid": probe["final_footprint_invalid"],
            "ik_contact": probe.get("ik_contact", "not_captured"),
            "geometric_block": probe["geometric_block"],
        })
    _write_jsonl(out_dir, "collision_sweeps.jsonl", sweeps)
    _write(out_dir, "decision.json", decision)
    html_size = write_html(
        os.path.join(out_dir, "placement_corridor_explorer.html"),
        routes, placed, probes, decision)
    readme = _readme(decision, html_size, placed)
    with open(os.path.join(out_dir, "README.md"), "w", encoding="utf-8") as handle:
        handle.write(readme)
    print(json.dumps({
        "routes": len(routes),
        "opening_probes": len(opening),
        "decision": decision.get("code"),
        "html_bytes": html_size,
        "out": out_dir,
    }, indent=2))
    return decision


def _write(directory, name, payload):
    with open(os.path.join(directory, name), "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, default=str)
        handle.write("\n")


def _write_jsonl(directory, name, rows):
    with open(os.path.join(directory, name), "w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, default=str) + "\n")


def _readme(decision, html_size, placed):
    clear = decision.get("geometrically_clear") or []
    lines = "\n".join(
        "- %s %s at %s" % (item["box"], item["kind"], item["center"])
        for item in clear) or "- none"
    return """# Placement corridor supergraph g1

Offline diagnostic over the frozen slot 656 / second-box dumps.
The online planner was not modified and no scored simulation was started.

## Decision

`%s`

%s

Geometrically clear opening probes (hull, support >= 0.6, same-layer
gap >= 0.05 m, payload AABB clear of the scene `placed_0_0_0` body,
offline IK to the contact TCP from the captured start joints):

%s

`ValidateRouteV2` was not run, so these probes are not certified routes.
That is why none of the six decision codes is claimed.

## Slot 656

Executed segments: %s. Certified flag is %s.
The blocking body is `scene.json` `placed_0_0_0` (world yaw pi, so the
0.495 m side lies on container X). The slot quaternion is stored beside
it and is not the collision yaw.

## Capture gaps

IK branch sets, waypoint samples, collision contact coordinates, and
probe certification are `not_captured`. The 61 route rows keep the dump
reason verbatim. The red-column final footprint overlaps that placed
body; the 5 cm shifts leave a gap under 0.05 m.

## HTML

`placement_corridor_explorer.html` is %d bytes. Click a marker for the
captured failure chain.
""" % (
        decision.get("code") or "not_earned",
        decision.get("reason", ""),
        lines,
        ", ".join(placed["segment_names"]),
        placed["certified"],
        html_size,
    )


if __name__ == "__main__":
    destination = (
        "/tmp/elfin-st09-converge-i/docs/status/evidence/"
        "frame_migration/ST09-parallel-I-s6-readiness/"
        "placement_corridor_supergraph_g1"
    )
    main(destination)
