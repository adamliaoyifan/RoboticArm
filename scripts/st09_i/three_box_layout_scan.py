#!/usr/bin/env python3
"""Offline three-box layout scan: carryon, standard, carryon.

Candidates come from solve_placement on the floor-prior surface. Atlas
status is recorded and is not a pass. A candidate passes only when the
Cartesian corridor reaches fraction 0.95 with collision checking on:

    portal -> approach -> upper -> slot -> upper -> approach -> portal

The first two boxes stay on the floor. The third carryon may stack on a
support the solver already accepts. Standard is not placed on a carryon.
Gates are unchanged: fraction 0.95, refine 2 mm / 0.02 rad.
"""

from __future__ import division

import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS",
             "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import copy
import json
import math
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _path in (
        os.path.join(ROOT, "src", "luggage_packing"),
        os.path.join(ROOT, "src", "luggage_packing", "scripts"),
        os.path.join(ROOT, "src", "luggage_planning"),
        os.path.join(ROOT, "src", "luggage_description"),
):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from capacity_orientation_analysis import (  # noqa: E402
    ALLOWED_YAWS,
    SOLVER_PARAMS,
    AtlasStore,
    SceneContext,
    apply_placement,
    floor_prior_surface,
    make_hook,
    make_validator,
)
from luggage_packing.placement_solver import (  # noqa: E402
    candidate_aabb,
    solve_placement,
)
from luggage_planning.offline_ik import load_urdf, refine_to_pose  # noqa: E402
from luggage_planning.reachable_graph import make_offline_fk  # noqa: E402
from luggage_planning.route_producer import (  # noqa: E402
    RELEASE_DROP_M,
    anchor_plan,
    tool_down_branch_seeds,
)
from st09_contracts.attachment import (  # noqa: E402
    compose_transform,
    invert_transform,
)

SCENE = os.path.join(
    ROOT, "src", "luggage_description", "config", "scene_tf_deploy.yaml")
ATLAS_CONFIG = os.path.join(
    ROOT, "src", "luggage_planning", "config", "atlas_runtime_deploy.yaml")
OUT = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_three_box_layout_20261005")
SEQUENCE = (
    ("carryon", (0.55, 0.40, 0.25), False),
    ("standard", (0.70, 0.45, 0.28), False),
    ("carryon", (0.55, 0.40, 0.25), True),
)
# How many of the closest endpoint-reachable candidates get a MoveIt chain
# at one stage. This bounds the scan. It is not a new product gate.
MOVEIT_LIMIT = 12
KEEP = 4
POS_TOL = 2e-3
ROT_TOL = 2e-2
FRACTION_MIN = 0.95
EXCURSION_MAX = 3.0
# Same-branch tolerance used by the route search. A short hop whose
# wrapped start-to-end change exceeds this has changed wrist heading.
BRANCH_TOL_RAD = 1.0
JOINT_NAMES = tuple("elfin_joint%d" % i for i in range(1, 7))
CHAIN = (
    ("slot_approach", "TRAVERSE", "carrying"),
    ("slot_upper", "INSERT", "carrying"),
    ("slot", "DESCEND", "carrying"),
    ("slot_upper", "RETREAT", "unloaded"),
    ("slot_approach", "RETREAT", "unloaded"),
    ("portal", "RETREAT", "unloaded"),
)


def _yaw_quat(yaw):
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _container_pose(scene_config):
    from luggage_description.scene_tf_publisher import rpy_to_quaternion
    item = next(row for row in scene_config["static_transforms"]
                if row["child"] == "container_link")
    translation = tuple(float(v) for v in item["translation"])
    quaternion = rpy_to_quaternion(item["rotation_rpy"])
    return translation, tuple(float(v) for v in quaternion)


def _tcp_world(center_container, size, container_tf):
    """Nominal centered grasp, then the declared 50 mm release gap."""
    height = float(size[2])
    translation, quaternion = invert_transform(
        (0.0, 0.0, height / 2.0), (1.0, 0.0, 0.0, 0.0))
    tcp_c, quat_c = compose_transform(
        tuple(float(v) for v in center_container), (0.0, 0.0, 0.0, 1.0),
        translation, quaternion)
    world_t, world_q = container_tf
    gap = compose_transform(
        world_t, world_q, (0.0, 0.0, RELEASE_DROP_M), (0.0, 0.0, 0.0, 1.0))
    tcp, quat = compose_transform(world_t, world_q, tcp_c, quat_c)
    tcp = tuple(tcp[i] + gap[0][i] - world_t[i] for i in range(3))
    return tcp, quat


def _box_world(center_container, yaw, container_tf):
    world_t, world_q = container_tf
    box_q = _yaw_quat(yaw)
    return compose_transform(
        world_t, world_q, tuple(float(v) for v in center_container), box_q)


def _refine_seeds(urdf, tcp_path, position, quaternion, preferred=None):
    """First seed inside the 2 mm / 0.02 rad gate. Both tool-down basins."""
    seeds = []
    if preferred is not None:
        seeds.append(("preferred", tuple(preferred)))
    for name, seed in tool_down_branch_seeds():
        if preferred is None or max(
                abs(float(a) - float(b)) for a, b in zip(seed, preferred)) > 1e-6:
            seeds.append((name, seed))
    best = None
    for name, seed in seeds:
        joints, info = refine_to_pose(
            urdf, tcp_path, position, quaternion, list(seed),
            pos_tol=POS_TOL, rot_tol=ROT_TOL)
        if joints is not None:
            return joints, info, name
        if best is None or (info or {}).get("pos_err_m", 9) < best[1]:
            best = (info, (info or {}).get("pos_err_m", 9), name)
    info = best[0] if best else {}
    return None, info, None


def _sigma(urdf, tcp_path, q):
    import numpy as np
    from scipy.spatial.transform import Rotation as Rot

    base = urdf.tcp(q, tcp_path)
    jacobian = np.zeros((6, 6))
    delta = 1e-6
    for index in range(6):
        nudged = [float(v) for v in q]
        nudged[index] += delta
        tf = urdf.tcp(nudged, tcp_path)
        jacobian[:3, index] = (tf[:3, 3] - base[:3, 3]) / delta
        rot = (Rot.from_matrix(base[:3, :3]).inv()
               * Rot.from_matrix(tf[:3, :3])).as_rotvec()
        jacobian[3:, index] = np.asarray(rot, dtype=float) / delta
    return float(np.linalg.svd(jacobian, compute_uv=False)[-1])


def _path_stats(urdf, tcp_path, positions):
    if not positions:
        return None
    count = len(positions)
    stride = max(1, count // 8)
    indexes = list(range(0, count, stride))
    if indexes[-1] != count - 1:
        indexes.append(count - 1)
    sigmas = []
    abs_j3 = []
    for index in indexes:
        q = positions[index]
        sigmas.append(_sigma(urdf, tcp_path, q))
        abs_j3.append(abs(float(q[2])))
    travel = 0.0
    if len(positions) >= 2:
        travel = sum(abs(float(b) - float(a))
                     for a, b in zip(positions[0], positions[-1]))
    return {
        "min_sigma": round(min(sigmas), 6),
        "min_abs_j3": round(min(abs_j3), 6),
        "joint_travel_rad": round(travel, 6),
    }


def _feasible(ctx, store, surface, placed_aabbs, size):
    atlas, diag = store.get(list(size), "production")
    result = solve_placement(
        surface, list(size), allowed_yaws=ALLOWED_YAWS,
        params=dict(SOLVER_PARAMS),
        candidate_validator=make_validator(ctx, placed_aabbs, list(size)),
        atlas_annotate=make_hook(ctx, atlas))
    feasible = [row for row in result["candidates"] if row.get("feasible")]
    return feasible, {
        "atlas_loaded": atlas is not None,
        "atlas_reason": (diag or {}).get("reason"),
        "n_feasible": len(feasible),
        "message": result.get("message"),
        "histogram": result.get("reject_histogram"),
    }


def _dedupe(candidates):
    """One window per footprint. A pi heading shares the centered TCP."""
    kept = []
    seen = set()
    for candidate in candidates:
        yaw = float(candidate.get("box_yaw") or 0.0) % math.pi
        key = (int(candidate.get("ix0", -1)), int(candidate.get("iy0", -1)),
               int(candidate.get("footprint_class", -1)), round(yaw, 5))
        if key in seen:
            continue
        seen.add(key)
        kept.append(candidate)
    return kept


def _geometry_row(candidate, size, container_tf, urdf, tcp_path, scene_config):
    from luggage_description.scene_tf_config_utils import (
        container_opening_normal_in_world,
        container_opening_target_point_in_world,
        pedestal_dimensions,
        pedestal_enabled,
    )

    center = [float(v) for v in candidate["center_container"]]
    tcp, quat = _tcp_world(center, size, container_tf)
    box_t, box_q = _box_world(center, candidate.get("box_yaw") or 0.0,
                              container_tf)
    radius = math.hypot(tcp[0], tcp[1])
    inward = [-float(v) for v in container_opening_normal_in_world(
        scene_config)]
    _t, container_q = container_tf
    plan = anchor_plan(
        tcp, container_q, float(size[2]),
        opening=container_opening_target_point_in_world(scene_config),
        inward_normal=inward,
        transit_clearance_z=(
            pedestal_dimensions(scene_config)[2]
            if pedestal_enabled(scene_config) else None))
    joints, info, seed_name = _refine_seeds(urdf, tcp_path, tcp, quat)
    return {
        "candidate_id": candidate["candidate_id"],
        "center_container": [round(v, 4) for v in center],
        "center_world": [round(float(v), 4) for v in box_t],
        "tcp_world": [round(float(v), 4) for v in tcp],
        "radius_m": round(radius, 4),
        "depth_m": candidate.get("depth_m"),
        "support_score": candidate.get("support_score"),
        "support_source": candidate.get("support_source"),
        "box_yaw": candidate.get("box_yaw"),
        "atlas_status_name": candidate.get("atlas_status_name"),
        "endpoint_ik": joints is not None,
        "endpoint_seed": seed_name,
        "endpoint_j3": None if joints is None else round(float(joints[2]), 4),
        "endpoint_pos_err_m": (info or {}).get("pos_err_m"),
        "endpoint_rot_err_rad": (info or {}).get("rot_err_rad"),
        "_seed": None if joints is None else tuple(joints),
        "_tcp": tcp,
        "_quat": quat,
        "_plan": {
            "portal": plan.portal,
            "slot_approach": plan.slot_approach,
            "slot_upper": plan.slot_upper,
            "slot": plan.slot,
        },
        "_candidate": candidate,
        "_box_world": (box_t, box_q),
    }


def _cartesian(node, from_q, poses, avoid):
    from geometry_msgs.msg import Point, Pose, Quaternion
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetCartesianPath
    from sensor_msgs.msg import JointState

    client = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    if not client.wait_for_service(timeout_sec=15.0):
        raise RuntimeError("compute_cartesian_path unavailable")
    request = GetCartesianPath.Request()
    request.header.frame_id = "world"
    request.group_name = "elfin_arm"
    request.link_name = "suction_contact_frame"
    request.max_step = 0.01
    request.jump_threshold = 0.0
    request.avoid_collisions = bool(avoid)
    start = RobotState()
    start.is_diff = True
    start.joint_state = JointState()
    start.joint_state.name = list(JOINT_NAMES)
    start.joint_state.position = [float(v) for v in from_q]
    request.start_state = start
    for position, quaternion in poses:
        pose = Pose()
        pose.position = Point(x=float(position[0]), y=float(position[1]),
                              z=float(position[2]))
        pose.orientation = Quaternion(
            x=float(quaternion[0]), y=float(quaternion[1]),
            z=float(quaternion[2]), w=float(quaternion[3]))
        request.waypoints.append(pose)
    future = client.call_async(request)
    deadline = time.monotonic() + 30.0
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    if not future.done() or future.result() is None:
        return {"fraction": None, "positions": []}
    response = future.result()
    points = response.solution.joint_trajectory.points
    return {
        "fraction": round(float(response.fraction), 6),
        "positions": [[float(v) for v in point.positions] for point in points],
    }


def _segment_motion(positions, distance):
    from luggage_planning.joint_path import joint_motion_report

    report = joint_motion_report(positions, distance)
    if report is None:
        return None
    names = ("elfin_joint%d" % index for index in range(1, 7))
    report["worst_joint"] = list(names)[int(report["worst_joint_index"])]
    report["ratio"] = round(float(report["ratio"]), 4)
    return report


def _run_chain(node, adapter, urdf, tcp_path, row, size, placed_boxes):
    from luggage_planning.moveit_edge_adapter import (
        BoxSpec,
        PayloadSpec,
        SceneState,
        synthesize_waypoints,
    )

    plan = row["_plan"]
    quat = row["_quat"]
    portal_q, info, _seed_name = _refine_seeds(
        urdf, tcp_path, plan["portal"], quat, preferred=row.get("_seed"))
    if portal_q is None:
        return {
            "passed": False,
            "stopped_at": "portal",
            "cause": "PORTAL_IK_MISS",
            "pos_err_m": (info or {}).get("pos_err_m"),
            "segments": [],
        }
    box_t, box_q = row["_box_world"]
    released = BoxSpec(
        object_id="released-scan",
        center=tuple(float(v) for v in box_t),
        size_whd=tuple(float(v) for v in size),
        quaternion_xyzw=tuple(float(v) for v in box_q))
    payload = PayloadSpec(
        dimensions_wdh=tuple(float(v) for v in size),
        t_tcp_box_translation=(0.0, 0.0, float(size[2]) / 2.0),
        t_tcp_box_quaternion_xyzw=(1.0, 0.0, 0.0, 0.0))
    current = list(portal_q)
    fk = make_offline_fk(urdf, tcp_path)
    segments = []
    min_sigma = None
    min_abs_j3 = None
    travel = 0.0
    length = 0.0
    try:
        for region, edge_class, load in CHAIN:
            target = plan[region]
            position, quaternion = fk(current)
            # Short hops keep the heading the previous segment actually
            # arrived on. Forcing the candidate quaternion here is what
            # turns a 5 cm insert into a pi wrist flip.
            poses, distance = synthesize_waypoints(
                position, quaternion, target, quaternion, edge_class)
            if load == "carrying":
                adapter._ensure_scene(SceneState(
                    load_state="carrying", payload=payload,
                    placed_boxes=tuple(placed_boxes)))
            else:
                adapter._ensure_scene(SceneState(
                    load_state="unloaded", released_box=released,
                    placed_boxes=tuple(placed_boxes)))
            on = _cartesian(node, current, poses, True)
            segment = {
                "to": region,
                "edge_class": edge_class,
                "load": load,
                "fraction_on": on.get("fraction"),
                "path_length_m": round(float(distance), 4),
            }
            stats = _path_stats(urdf, tcp_path, on.get("positions") or [])
            if stats:
                segment.update(stats)
                min_sigma = stats["min_sigma"] if min_sigma is None \
                    else min(min_sigma, stats["min_sigma"])
                min_abs_j3 = stats["min_abs_j3"] if min_abs_j3 is None \
                    else min(min_abs_j3, stats["min_abs_j3"])
                travel += stats["joint_travel_rad"]
            length += float(distance)
            motion = _segment_motion(on.get("positions") or [], distance)
            if motion:
                segment["joint_motion"] = motion
            fraction = on.get("fraction")
            cause = None
            if fraction is None or fraction < FRACTION_MIN:
                off = _cartesian(node, current, poses, False)
                segment["fraction_off"] = off.get("fraction")
                if off.get("fraction") is not None and off["fraction"] >= FRACTION_MIN:
                    cause = "COLLISION_ALONG_PATH"
                else:
                    cause = "CARTESIAN_SHORT"
            elif motion and motion["ratio"] > EXCURSION_MAX:
                cause = "EXCURSION"
            elif edge_class in ("INSERT", "DESCEND", "RETREAT") and motion \
                    and max(abs(value) for value in motion["end_delta_rad"]) \
                    > BRANCH_TOL_RAD:
                cause = "BRANCH_DISCONTINUITY"
            if cause is not None:
                segment["cause"] = cause
                segments.append(segment)
                return {
                    "passed": False,
                    "stopped_at": region,
                    "cause": cause,
                    "segments": segments,
                    "min_sigma": min_sigma,
                    "min_abs_j3": min_abs_j3,
                }
            segments.append(segment)
            current = [float(v) for v in on["positions"][-1]]
    finally:
        adapter.reset_scene()
    return {
        "passed": True,
        "stopped_at": None,
        "cause": None,
        "segments": segments,
        "min_sigma": min_sigma,
        "min_abs_j3": min_abs_j3,
        "joint_travel_rad": round(travel, 4),
        "path_length_m": round(length, 4),
    }


def _public(row):
    return {key: value for key, value in row.items() if not key.startswith("_")}


def _rank(row):
    chain = row.get("chain") or {}
    return (
        row["radius_m"],
        -(chain.get("min_sigma") or 0.0),
        -(chain.get("min_abs_j3") or 0.0),
        chain.get("joint_travel_rad") or 0.0,
        chain.get("path_length_m") or 0.0,
        row.get("depth_m") or 0.0,
    )


def _stage(ctx, store, surface, placed, placed_boxes, size, allow_stack,
           container_tf, urdf, tcp_path, scene_config, node, adapter):
    feasible, meta = _feasible(ctx, store, surface, placed, size)
    if not allow_stack:
        feasible = [row for row in feasible
                    if row.get("support_source") == "floor_prior"]
    feasible = _dedupe(feasible)
    rows = [_geometry_row(candidate, size, container_tf, urdf, tcp_path,
                          scene_config)
            for candidate in feasible]
    reachable = [row for row in rows if row["endpoint_ik"]]
    reachable.sort(key=lambda row: row["radius_m"])
    checked = reachable[:MOVEIT_LIMIT] if node is not None else []
    for row in checked:
        row["chain"] = _run_chain(
            node, adapter, urdf, tcp_path, row, size, placed_boxes)
        chain = row["chain"]
        row["passed"] = bool(chain.get("passed"))
        row["min_sigma"] = chain.get("min_sigma")
        row["min_abs_j3"] = chain.get("min_abs_j3")
    for row in rows:
        if "passed" not in row:
            row["passed"] = False
            row["chain"] = None
    passed = sorted((row for row in rows if row["passed"]), key=_rank)
    return {
        "meta": meta,
        "n_feasible": len(feasible),
        "n_endpoint_ik": len(reachable),
        "n_moveit": len(checked),
        "n_passed": len(passed),
        "rows": rows,
        "passed": passed,
    }


def _apply(ctx, surface, placed, candidate):
    surface = copy.deepcopy(surface)
    placed = list(placed)
    apply_placement(surface, candidate, 0.0)
    placed.append(candidate_aabb(candidate, ctx.inner_size[2], ctx.floor_z))
    return surface, placed


def _box_spec(row, size, index):
    from luggage_planning.moveit_edge_adapter import BoxSpec
    center, quat = row["_box_world"]
    return BoxSpec(
        object_id="placed-%d" % index,
        center=tuple(float(v) for v in center),
        size_whd=tuple(float(v) for v in size),
        quaternion_xyzw=tuple(float(v) for v in quat))


def _combo_key(rows):
    return (
        max(row["radius_m"] for row in rows),
        -min((row.get("min_sigma") or 0.0) for row in rows),
        -min((row.get("min_abs_j3") or 0.0) for row in rows),
        sum(row.get("depth_m") or 0.0 for row in rows),
    )


def scan(node=None, adapter=None):
    ctx = SceneContext(SCENE, ATLAS_CONFIG)
    ctx.atlas_cfg = dict(ctx.atlas_cfg)
    ctx.atlas_cfg["hash_policy"] = "off"
    placement = dict(ctx.atlas_cfg.get("placement") or {})
    placement["hard_reject"] = False
    ctx.atlas_cfg["placement"] = placement
    store = AtlasStore(ctx)
    urdf = load_urdf(scene_tf_config=SCENE)
    tcp_path = urdf.path_to("suction_contact_frame")
    container_tf = _container_pose(ctx.scene)
    surface = floor_prior_surface(ctx)
    placed = []
    world_boxes = []
    stages = []
    chosen = []
    beam = [([], surface, placed, world_boxes)]
    for name, size, allow_stack in SEQUENCE:
        nxt = []
        stage_report = []
        for history, surface_i, placed_i, boxes_i in beam:
            report = _stage(
                ctx, store, surface_i, placed_i, boxes_i, size, allow_stack,
                container_tf, urdf, tcp_path, ctx.scene, node, adapter)
            stage_report.append({
                "after": [row["candidate_id"] for row in history],
                "n_feasible": report["n_feasible"],
                "n_endpoint_ik": report["n_endpoint_ik"],
                "n_moveit": report["n_moveit"],
                "n_passed": report["n_passed"],
                "closest": [_public(row) for row in sorted(
                    report["rows"], key=lambda item: item["radius_m"])[:8]],
                "passed": [_public(row) for row in report["passed"][:KEEP]],
            })
            for row in report["passed"][:KEEP]:
                surface_n, placed_n = _apply(
                    ctx, surface_i, placed_i, row["_candidate"])
                boxes_n = list(boxes_i) + [_box_spec(row, size, len(history))]
                nxt.append((history + [row], surface_n, placed_n, boxes_n))
        stages.append({"box": name, "size": list(size),
                       "allow_stack": allow_stack, "parents": stage_report})
        if not nxt:
            beam = []
            break
        nxt.sort(key=lambda item: _combo_key(item[0]))
        beam = nxt[:KEEP]
        chosen = beam
    best = chosen[0][0] if chosen else []
    return {
        "sequence": [name for name, _size, _stack in SEQUENCE],
        "gates": {
            "cartesian_fraction": FRACTION_MIN,
            "refine_pos_m": POS_TOL,
            "refine_rot_rad": ROT_TOL,
            "release_gap_m": RELEASE_DROP_M,
            "atlas_hard_reject": False,
        },
        "moveit": node is not None,
        "stages": stages,
        "best": [_public(row) for row in best],
        "best_key": list(_combo_key(best)) if best else None,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--moveit", action="store_true")
    parser.add_argument("--out", default=OUT)
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    node = None
    adapter = None
    if args.moveit:
        import rclpy
        import threading
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter
        from luggage_planning.planning_scene_client import PlanningSceneClient
        from slot64_classify_replay import _sync_world

        rclpy.init()
        node = Node("three_box_layout_scan")
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        spinner = threading.Thread(target=executor.spin, daemon=True)
        spinner.start()
        scene = SceneContext(SCENE, ATLAS_CONFIG).scene
        _sync_world(node, scene, {"world_objects": []})
        adapter = MoveItEdgeAdapter(
            node, scene_client=PlanningSceneClient(node))
        adapter.wait_ready(timeout_sec=60.0)
    try:
        report = scan(node, adapter)
    finally:
        if node is not None:
            if adapter is not None:
                adapter.reset_scene()
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
    path = os.path.join(args.out, "layout_scan.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=1, sort_keys=True, default=str)
        handle.write("\n")
    print(json.dumps({
        "out": path,
        "moveit": report["moveit"],
        "stages": [
            {"box": stage["box"],
             "passed": [parent["n_passed"] for parent in stage["parents"]]}
            for stage in report["stages"]],
        "best": [row["candidate_id"] for row in report["best"]],
    }, indent=1))
    return 0 if report["best"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
