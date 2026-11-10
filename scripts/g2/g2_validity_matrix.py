#!/usr/bin/env python3
"""G2-1: validity of the 4 frozen portal goals, 2 cold scene resets.

Per target and round, from the exact `.scene.bin`:
  - URDF limits (structured XML) and MoveIt effective limits (move_group params)
  - goal-constraint consistency
  - MoveIt state validity (GetStateValidity) with the payload attached
  - certifier validity (ValidateRouteV2) under the certifier's composed ACM
The two checkers are different programs; their outputs are stored separately.

usage: g2_validity_matrix.py <out_dir> [--rounds 2]
"""

import argparse
import json
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g2_common as common  # noqa: E402

JOINTS = common.score.JOINTS


def urdf_limits_from_text(text):
    """Structured XML read; independent of attribute order and quoting."""
    root = ET.fromstring(text)
    limits = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if name in JOINTS and limit is not None:
            lower, upper = limit.get("lower"), limit.get("upper")
            if lower is not None and upper is not None:
                limits[name] = (float(lower), float(upper))
    return limits


def moveit_effective_limits(node):
    """Position limits as MoveIt reports them (robot_description_planning)."""
    from rcl_interfaces.srv import GetParameters

    client = node.create_client(GetParameters, "/move_group/get_parameters")
    if not client.wait_for_service(timeout_sec=15.0):
        return {"status": "not_evaluated", "reason": "move_group_params_unavailable"}
    names = []
    for joint in JOINTS:
        base = "robot_description_planning.joint_limits.%s." % joint
        names += [base + "has_position_limits", base + "min_position", base + "max_position"]
    request = GetParameters.Request()
    request.names = names
    done = threading.Event()
    future = client.call_async(request)
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(15.0):
        return {"status": "not_evaluated", "reason": "timeout"}
    values = future.result().values
    out = {}
    for name, value in zip(names, values):
        # type 0 == PARAMETER_NOT_SET
        out[name] = None if value.type == 0 else (
            value.bool_value if value.type == 1 else value.double_value)
    return {"status": "queried", "params": out,
            "note": "NOT_SET entries mean MoveIt falls back to the URDF limit"}


def robot_description_text(node):
    from rcl_interfaces.srv import GetParameters

    client = node.create_client(GetParameters, "/robot_state_publisher/get_parameters")
    if not client.wait_for_service(timeout_sec=30.0):
        return ""
    request = GetParameters.Request()
    request.names = ["robot_description"]
    done = threading.Event()
    future = client.call_async(request)
    future.add_done_callback(lambda _f: done.set())
    done.wait(15.0)
    result = future.result()
    return str(result.values[0].string_value) if result and result.values else ""


def apply_full_scene(client, scene):
    import copy

    msg = copy.deepcopy(scene)
    msg.is_diff = False
    msg.robot_state.is_diff = False
    return client._apply(msg, timeout=15.0)  # noqa: SLF001


def one_target(node, client, record, scene, effective, urdf_limits, rnd):
    import copy
    from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter
    from luggage_planning.planning_scene_client import acm_pair_allowed

    goal = [float(v) for v in record["goal_q"]]
    ok, why = apply_full_scene(client, scene)
    time.sleep(0.4)
    inventory = common.paired._inventory(client, names_only=False)  # noqa: SLF001
    result = {
        "round": rnd, "exact_sha256": record["exact_sha256"], "goal_q": goal,
        "applied": ok, "apply_detail": why,
        "scene_inventory": {k: inventory.get(k) for k in (
            "world_object_ids", "attached", "acm_pair_count", "digest")},
    }
    # limits
    out_urdf = [n for n, v in zip(JOINTS, goal)
                if n not in urdf_limits or not (urdf_limits[n][0] <= v <= urdf_limits[n][1])]
    result["urdf_limit_violations"] = out_urdf
    # constraints (same rule as the classifier)
    constraints = {c["joint_name"]: c for c in record["goal_constraints"]}
    result["constraint_defects"] = [
        n for n, v in zip(JOINTS, goal)
        if n not in constraints or abs(constraints[n]["position"] - v) > 1e-9
        or constraints[n]["tolerance_above"] <= 0 or constraints[n]["tolerance_below"] <= 0]
    # MoveIt's own checker, scene as applied (original ACM)
    result["moveit_state_validity"] = common.paired._goal_validity(node, goal)  # noqa: SLF001
    # certifier on the same scene, composed ACM (mutates its copy)
    class Host(object):
        _node = node
    original_acm_allowed = acm_pair_allowed(
        scene.allowed_collision_matrix, "elfin_base", "robot_pedestal")
    verdict = MoveItEdgeAdapter._goal_on_submission_scene(  # noqa: SLF001
        Host(), copy.deepcopy(scene), goal)
    verdict["original_acm_base_pedestal_allowed"] = original_acm_allowed
    result["certifier_validity"] = verdict
    result["checkers_agree"] = (
        result["moveit_state_validity"].get("valid") ==
        verdict.get("valid")
        if result["moveit_state_validity"].get("status") == "queried"
        and verdict.get("status") == "queried" else None)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("out_dir")
    parser.add_argument("--rounds", type=int, default=2)
    args = parser.parse_args()
    import rclpy
    from rclpy.node import Node

    out = args.out_dir
    stack = common.MoveItStack(out, "validity_matrix")
    stack.start()
    rclpy.init()
    node = Node("g2_validity_matrix")
    stop, thread = common.score._spin(node)  # noqa: SLF001
    document = {"rounds": [], "status": "incomplete"}
    try:
        client = common.paired._scene_client(node)  # noqa: SLF001
        text = robot_description_text(node)
        document["robot_description_sha256"] = common.hashlib.sha256(text.encode()).hexdigest()
        document["robot_description_ready"] = bool(text)
        urdf_limits = urdf_limits_from_text(text) if text else {}
        document["urdf_limits"] = urdf_limits
        document["moveit_effective_limits"] = moveit_effective_limits(node)
        for rnd in range(1, args.rounds + 1):
            rows = []
            for index in range(4):
                record, scene = common.load_portal(index)
                row = one_target(node, client, record, scene, None, urdf_limits, rnd)
                row["portal"] = index
                rows.append(row)
                print(json.dumps({"round": rnd, "portal": index,
                                  "moveit_valid": row["moveit_state_validity"].get("valid"),
                                  "certifier_valid": row["certifier_validity"].get("valid")}),
                      flush=True)
            document["rounds"].append(rows)
        document["status"] = "complete"
    finally:
        stop.set()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stack.stop()
        common.dump(out, "validity_matrix.json", document)
    return 0


if __name__ == "__main__":
    sys.exit(main())
