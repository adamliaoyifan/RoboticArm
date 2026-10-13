#!/usr/bin/env python3
"""Per-point lattice failure attribution for the frozen 48-point baseline.

Writes a new evidence directory. Does not rewrite the robustness or
ablation runs. Collision contacts are filled by ``collide`` for the
points whose kinematics already form a chain.
"""

from __future__ import division

import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS",
             "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import json
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

import corridor_ablation as ablation  # noqa: E402
import placement_robustness as scan  # noqa: E402
from luggage_planning.corridor_lattice import (  # noqa: E402
    boundary_counts,
    kinematic_chain_joints,
    layer_family,
)
from luggage_planning.grid_path_search import _wrap_rad  # noqa: E402
from luggage_planning.lattice_route import _chain_sigma_min  # noqa: E402
from luggage_planning.reachability_wavefront import (  # noqa: E402
    joint_branch_distance,
)

OUT = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_n3_failure_attribute_20261005")
FREEZE = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_n3_baseline_freeze_20261005", "scan_list.json")


def _zeros():
    return {
        "jump_reject": 0,
        "branch_reject": 0,
        "wrist_reject": 0,
        "carrying_collision_reject": 0,
        "unloaded_collision_reject": 0,
    }


def _chain_motion(chain):
    if not chain or len(chain) < 2:
        return {"max_joint_step_rad": None, "max_branch_distance_rad": None}
    steps = []
    branches = []
    for left, right in zip(chain, chain[1:]):
        delta = [abs(_wrap_rad(float(b) - float(a))) for a, b in zip(left, right)]
        steps.append(max(delta))
        branches.append(joint_branch_distance(left, right))
    return {
        "max_joint_step_rad": round(max(steps), 6),
        "max_branch_distance_rad": round(max(branches), 6),
    }


def _row(point, prepared, fk):
    attempt = prepared.attempts[-1] if prepared.attempts else {}
    family = None
    retreat = False
    empty = str(prepared.reason or "")
    if empty.startswith("LATTICE_LAYER_EMPTY:"):
        family, retreat = layer_family(empty.split(":", 1)[1])
    boundaries, disconnect = boundary_counts(prepared)
    chain = kinematic_chain_joints(prepared)
    layers = []
    for record, solution in zip(prepared.attempts, prepared.solutions):
        layers.append({
            "layer": record.get("layer"),
            "family": record.get("layer_family"),
            "nodes": len(solution),
            "seeds": record.get("seeds"),
            "raw_ik_ok": record.get("raw_ik_ok"),
            "joint_limit_reject": record.get("joint_limit_reject"),
            "branch_reject": record.get("branch_reject"),
        })
    if empty.startswith("LATTICE_LAYER_EMPTY") and prepared.attempts:
        failed = prepared.attempts[-1]
        if not layers or layers[-1].get("layer") != failed.get("layer"):
            layers.append({
                "layer": failed.get("layer"),
                "family": failed.get("layer_family"),
                "nodes": 0,
                "seeds": failed.get("seeds"),
                "raw_ik_ok": failed.get("raw_ik_ok"),
                "joint_limit_reject": failed.get("joint_limit_reject"),
                "branch_reject": failed.get("branch_reject"),
            })
    code = "LATTICE_LAYER_EMPTY" if empty.startswith("LATTICE_LAYER_EMPTY") \
        else ("KINEMATIC_CHAIN" if chain is not None else "LATTICE_NO_CHAIN")
    gates = _zeros()
    if disconnect is not None:
        edges = disconnect["edges"]
        gates["jump_reject"] = edges["jump"]
        gates["branch_reject"] = edges["branch"]
        gates["wrist_reject"] = edges["wrist"]
    motion = _chain_motion(chain)
    return {
        "scan_id": point["scan_id"],
        "box": point["box"],
        "depth_band": point["depth_band"],
        "lateral_band": point["lateral_band"],
        "yaw_deg": point["yaw_deg"],
        "size_m": [float(v) for v in point["size"]],
        "code": code,
        "reason": empty or code,
        "empty_layer": attempt.get("layer") if code == "LATTICE_LAYER_EMPTY" else None,
        "empty_family": family,
        "retreat": bool(retreat),
        "seeds": attempt.get("seeds") if code == "LATTICE_LAYER_EMPTY" else None,
        "raw_ik_ok": attempt.get("raw_ik_ok") if code == "LATTICE_LAYER_EMPTY" else None,
        "joint_limit_reject": (
            attempt.get("joint_limit_reject")
            if code == "LATTICE_LAYER_EMPTY" else None),
        "ik_branch_reject": (
            attempt.get("branch_reject")
            if code == "LATTICE_LAYER_EMPTY" else None),
        "jump_reject": gates["jump_reject"],
        "branch_edge_reject": gates["branch_reject"],
        "wrist_reject": gates["wrist_reject"],
        "carrying_collision_reject": 0,
        "unloaded_collision_reject": 0,
        "contact_pairs": [],
        "layers": layers,
        "boundaries": [
            {"from": row["from_layer"], "to": row["to_layer"],
             "load": row["load"],
             "nodes_from": row["nodes_from"], "nodes_to": row["nodes_to"],
             "edges": row["edges"]}
            for row in boundaries],
        "first_disconnect": None if disconnect is None else {
            "from": disconnect["from_layer"],
            "to": disconnect["to_layer"],
            "load": disconnect["load"],
            "edges": disconnect["edges"],
            "reason": "kinematic",
        },
        "min_sigma": _chain_sigma_min(fk, chain) if chain is not None else None,
        "max_joint_step_rad": motion["max_joint_step_rad"],
        "max_branch_distance_rad": motion["max_branch_distance_rad"],
        "collision_checked": False,
    }


def kinematic(limit=None):
    scan.assert_safety_pins()
    os.makedirs(OUT, exist_ok=True)
    points = json.load(open(FREEZE, encoding="utf-8"))["points"]
    if limit:
        points = points[:int(limit)]
    _config, transform, opening, transit = ablation._scene()  # noqa: SLF001
    ctx = scan.SceneContext(scan.SCENE, scan.ATLAS_CONFIG)
    urdf, fk, refine = ablation._models()  # noqa: SLF001
    path = os.path.join(OUT, "attribution.jsonl")
    rows = []
    with open(path, "w", encoding="utf-8") as handle:
        for point in points:
            request = scan._request(  # noqa: SLF001
                ctx, transform, [point], point["scan_id"],
                time.monotonic_ns(), None)
            _attachment, tcp = ablation._targets(request)  # noqa: SLF001
            anchors, _plan = ablation._anchors_for(  # noqa: SLF001
                point, transform, opening, transit, tcp, request["box_wdh"][2])
            prepared, ik_s = ablation._prepare(  # noqa: SLF001
                anchors, tcp.quaternion_xyzw, urdf, refine,
                request["start_joints"])
            row = _row(point, prepared, fk)
            row["ik_s"] = round(ik_s, 3)
            rows.append(row)
            handle.write(json.dumps(row, sort_keys=True) + "\n")
            handle.flush()
            print("%s %s layer=%s seeds=%s raw_ik=%s" % (
                row["scan_id"], row["code"], row["empty_layer"],
                row["seeds"], row["raw_ik_ok"]), flush=True)
    _summarize(rows)
    print("wrote %s" % path)
    return 0


def _summarize(rows):
    summary = {"n": len(rows), "by_code": {}, "empty_family": {},
               "empty_ik": {"raw_ik_zero": 0, "joint_limit": 0, "ik_branch": 0}}
    for row in rows:
        summary["by_code"][row["code"]] = summary["by_code"].get(row["code"], 0) + 1
        if row["code"] != "LATTICE_LAYER_EMPTY":
            continue
        family = row["empty_family"] or "unknown"
        summary["empty_family"][family] = summary["empty_family"].get(family, 0) + 1
        if int(row["raw_ik_ok"] or 0) == 0:
            summary["empty_ik"]["raw_ik_zero"] += 1
        if int(row["joint_limit_reject"] or 0) > 0:
            summary["empty_ik"]["joint_limit"] += 1
        if int(row["ik_branch_reject"] or 0) > 0:
            summary["empty_ik"]["ik_branch"] += 1
    path = os.path.join(OUT, "attribution_summary.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return summary


def collide():
    """Collision-cut the kinematic chains. Empty layers are not rechecked."""
    import threading

    import corridor_lattice_score as score
    import slot64_classify_replay as classify

    scan.assert_safety_pins()
    rows = []
    with open(os.path.join(OUT, "attribution.jsonl"), encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                rows.append(json.loads(line))
    wanted = {row["scan_id"] for row in rows if row["code"] == "KINEMATIC_CHAIN"}
    points = [
        point for point in json.load(open(FREEZE, encoding="utf-8"))["points"]
        if point["scan_id"] in wanted]
    points.sort(key=lambda point: point["scan_id"])
    if not points:
        raise SystemExit("no kinematic chains to collision-check")
    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        raise SystemExit("residual sim processes: %s" % residuals)
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_perception.eval.isolated_domain import isolated_replay_env
    from luggage_planning.grid_path_search import ROUTE_TOTAL_WALL_S
    from luggage_planning.lattice_route import LatticeStateChecker
    from luggage_planning.planning_scene_client import PlanningSceneClient
    from luggage_planning.typed_driver_adapter import container_transform_fields
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    ctx = scan.SceneContext(scan.SCENE, scan.ATLAS_CONFIG)
    scene_config = load_scene_tf_config(scan.SCENE)
    transform = container_transform_fields(scene_config)
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.makedirs(OUT, exist_ok=True)
    proc = score._start_moveit(  # noqa: SLF001
        env, scan.SCENE, os.path.join(OUT, "moveit_collision.log"))
    rclpy.init()
    node = Node("route_failure_attribute")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    by_id = {row["scan_id"]: row for row in rows}
    try:
        from luggage_msgs.srv import ValidateRouteV2
        probe = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not probe.wait_for_service(timeout_sec=90.0):
            raise SystemExit("validate_route_v2 did not come up")
        classify._sync_world(node, scene_config, {"world_objects": []})  # noqa: SLF001
        scene_client = PlanningSceneClient(node)
        producer, _urdf, _tcp = classify._producer(  # noqa: SLF001
            node, scene_config, scan.SCENE, None, time.monotonic_ns(),
            budget_s=float(ROUTE_TOTAL_WALL_S))
        producer._clock_ns = time.monotonic_ns
        producer.corridor_planner = "lattice"
        producer.lattice_checker_factory = (
            lambda scene_states, start_q: LatticeStateChecker(
                node, scene_client, scene_states, start_q, distance_stride=1))
        producer.vacuum_state_fn = lambda: {
            "attached": True, "vacuum_on": True,
            "received_ns": time.monotonic_ns(),
        }
        parent_added = False
        for point in points:
            occupied = None
            if point["support"] == "stack":
                scan._add_support(scene_client, transform, point)  # noqa: SLF001
                parent_added = True
                surface = scan.floor_prior_surface(ctx)
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
                parent["center_local"] = [
                    float(center[0]), float(center[1]),
                    float(center[2]) - float(surface["center_base"][2])]
                scan.apply_placement(surface, parent, 0.0)
                occupied = surface
            elif parent_added:
                scene_client.remove_object("robustness_support")
                parent_added = False
            request = scan._request(  # noqa: SLF001
                ctx, transform, [point], point["scan_id"],
                time.monotonic_ns(), occupied if point["support"] == "stack" else None)
            _outcome, detail = ablation._plan_recorded(producer, request)  # noqa: SLF001
            block = detail.get("first_block") or {}
            by_load = detail.get("collision_by_load") or {}
            row = by_id[point["scan_id"]]
            row["collision_checked"] = True
            row["carrying_collision_reject"] = int(by_load.get("carrying") or 0)
            row["unloaded_collision_reject"] = int(by_load.get("unloaded") or 0)
            row["contact_pairs"] = list(block.get("contacts") or [])
            row["first_disconnect"] = {
                "load": block.get("load"),
                "reason": block.get("reason") or detail.get("lattice_reason"),
                "contacts": row["contact_pairs"],
            }
            row["min_collision_distance_m"] = detail.get(
                "min_collision_distance_m")
            if row["contact_pairs"] or int(by_load.get("carrying") or 0) \
                    or int(by_load.get("unloaded") or 0):
                row["code"] = "LATTICE_NO_CHAIN"
            print("%s %s load=%s contacts=%s carry=%s unload=%s" % (
                point["scan_id"], row["code"], block.get("load"),
                len(row["contact_pairs"]),
                row["carrying_collision_reject"],
                row["unloaded_collision_reject"]), flush=True)
    finally:
        score._stop_moveit(proc)  # noqa: SLF001
        if rclpy.ok():
            rclpy.shutdown()
    path = os.path.join(OUT, "attribution.jsonl")
    with open(path, "w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")
    _summarize(rows)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("kinematic", "collide"))
    parser.add_argument("--limit", type=int, default=0)
    args = parser.parse_args()
    if args.command == "collide":
        return collide()
    return kinematic(args.limit or None)


if __name__ == "__main__":
    raise SystemExit(main())
