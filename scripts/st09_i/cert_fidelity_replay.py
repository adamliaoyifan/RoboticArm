#!/usr/bin/env python3
"""Replay one frozen ValidateRouteV2 request. No re-planning.

Each cell starts a fresh isolated MoveIt process and sends that same
request three times: cold, then two warm calls. The 2 s service budget
inside the request is not changed. ``measure_distance:=false`` is a
diagnostic switch, not a product setting.

The request file is the ``cert_request.json`` written when
``ST09_I_CERT_CAPTURE_DIR`` is set.
"""

from __future__ import division

import argparse
import base64
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for path in (
        HERE, ROOT,
        os.path.join(ROOT, "src", "luggage_planning"),
        os.path.join(ROOT, "src", "luggage_description"),
        os.path.join(ROOT, "src", "luggage_perception"),
):
    if path not in sys.path:
        sys.path.insert(0, path)

import corridor_lattice_score as score  # noqa: E402

SCENE_TF = os.path.join(
    ROOT, "src/luggage_description/config/scene_tf_deploy.yaml")


def _mesh_counts():
    from luggage_description.scene_mesh_utils import (
        container_collision_mesh_path,
    )
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_planning.planning_scene_client import _read_stl_arrays

    config = load_scene_tf_config(SCENE_TF)
    path = container_collision_mesh_path(config)
    _vertices, faces = _read_stl_arrays(path)
    return {
        "path": path,
        "file_triangles": int(len(faces)),
        "offline_max_faces": 8000,
        "online_scene_manager_default_max_faces": 0,
    }


def _load_request(path):
    from luggage_msgs.srv import ValidateRouteV2
    from rclpy.serialization import deserialize_message

    document = json.load(open(path, encoding="utf-8"))
    request = deserialize_message(
        base64.b64decode(document["request_b64"]), ValidateRouteV2.Request)
    return document, request


def _leg_row(leg):
    return {
        "edge": str(leg.edge_id),
        "valid": bool(leg.valid),
        "reason": str(leg.reason),
        "n_samples": int(leg.n_samples),
        "first_invalid": int(leg.first_invalid_sample),
        "leg_time_ms": round(float(leg.leg_time_ns) / 1e6, 1),
        "collision_time_ms": round(
            float(getattr(leg, "collision_time_ns", 0) or 0) / 1e6, 1),
        "distance_time_ms": round(
            float(getattr(leg, "distance_time_ns", 0) or 0) / 1e6, 1),
        "distance_queries": int(getattr(leg, "distance_queries", 0) or 0),
    }


def _call(node, client, request):
    import rclpy

    started = time.perf_counter()
    future = client.call_async(request)
    deadline = time.monotonic() + 15.0
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    wall = time.perf_counter() - started
    if not future.done():
        return {"wall_s": round(wall, 3), "error": "client_timeout_15s"}
    response = future.result()
    if response is None:
        return {"wall_s": round(wall, 3), "error": "empty_response"}
    return {
        "wall_s": round(wall, 3),
        "result": int(response.result),
        "detail": str(response.detail),
        "all_valid": bool(response.all_valid),
        "model_sha256": str(response.model_sha256),
        "scene_digest": str(response.scene_digest),
        "service_time_s": round(float(response.service_time_ns) / 1e9, 3),
        "legs": [_leg_row(leg) for leg in response.leg_results],
    }


def _variants():
    joints = (
        "initial_joint1:=-1.250000,initial_joint2:=-1.746600,"
        "initial_joint3:=-0.997400,initial_joint4:=-1.915300,"
        "initial_joint5:=1.771400,initial_joint6:=1.861000")
    return (
        ("gz_observe_pose", {
            "use_gz_sim": "true",
            "hardware_plugin": "gz_ros2_control/GazeboSimSystem",
            "xacro_extra": joints,
        }),
        ("mock_distance_on", {}),
        ("mock_distance_off", {"measure_distance": "false"}),
        ("gz_distance_on", {
            "use_gz_sim": "true",
            "hardware_plugin": "gz_ros2_control/GazeboSimSystem",
        }),
        ("gz_distance_off", {
            "use_gz_sim": "true",
            "hardware_plugin": "gz_ros2_control/GazeboSimSystem",
            "measure_distance": "false",
        }),
    )


def _replay_cell(node, env, name, launch_args, request, out_dir):
    import rclpy
    from luggage_msgs.srv import ValidateRouteV2

    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        return {"cell": name, "error": "residuals_before", "residuals": residuals}
    log_path = os.path.join(out_dir, "moveit_%s.log" % name)
    proc = score._start_moveit(env, SCENE_TF, log_path, launch_args)  # noqa: SLF001
    calls = []
    try:
        client = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not client.wait_for_service(timeout_sec=90.0):
            return {"cell": name, "error": "service_unavailable", "calls": calls}
        for repeat in range(3):
            row = _call(node, client, request)
            row["repeat"] = repeat
            row["phase"] = "cold" if repeat == 0 else "warm"
            calls.append(row)
            print(json.dumps({
                "cell": name, "repeat": repeat,
                "result": row.get("result"),
                "service_time_s": row.get("service_time_s"),
                "wall_s": row.get("wall_s"),
                "all_valid": row.get("all_valid"),
            }), flush=True)
        node.destroy_client(client)
    finally:
        score._stop_moveit(proc)  # noqa: SLF001
    return {
        "cell": name,
        "launch_args": launch_args,
        "calls": calls,
        "residuals_after": score._residuals(),  # noqa: SLF001
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--cells", default="",
                        help="Comma-separated cell names. Empty runs all.")
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    from luggage_perception.eval.isolated_domain import isolated_replay_env
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    summary = json.load(open(args.request, encoding="utf-8"))
    kept = {key: summary[key] for key in summary if key != "request_b64"}
    with open(os.path.join(args.out, "request_summary.json"), "w",
              encoding="utf-8") as handle:
        json.dump(kept, handle, indent=1)
        handle.write("\n")
    import rclpy
    rclpy.init()
    from rclpy.node import Node
    node = Node("cert_fidelity_replay")
    cells = []
    try:
        _document, request = _load_request(args.request)
        mesh = _mesh_counts()
        with open(os.path.join(args.out, "mesh.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(mesh, handle, indent=1)
            handle.write("\n")
        wanted = {
            item.strip() for item in args.cells.split(",") if item.strip()}
        for name, launch_args in _variants():
            if wanted and name not in wanted:
                continue
            cells.append(_replay_cell(
                node, env, name, launch_args, request, args.out))
            with open(os.path.join(args.out, "cells.json"), "w",
                      encoding="utf-8") as handle:
                json.dump({"cells": cells, "mesh": mesh}, handle, indent=1)
                handle.write("\n")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps({
        "cells": [{
            "cell": cell.get("cell"),
            "error": cell.get("error"),
            "service_time_s": [
                call.get("service_time_s") for call in cell.get("calls") or []
            ],
            "all_valid": [
                call.get("all_valid") for call in cell.get("calls") or []
            ],
        } for cell in cells],
    }, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
