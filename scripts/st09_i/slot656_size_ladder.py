#!/usr/bin/env python3
"""Slot 656 size ladder. Pose, start joints, grasp and scene stay fixed.

Sizes are the positive control, one intermediate, and the nominal 20-inch
box. Only ``box_wdh`` changes. Evidence goes to a new directory.
"""

from __future__ import division

import json
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import corridor_lattice_score as score  # noqa: E402
import lattice_route_replay as replay  # noqa: E402
import slot64_classify_replay as classify  # noqa: E402

ROOT = score.ROOT
OUT = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_n3_size_ladder_20261005")
SIZES = (
    ("detected_0.50", [0.50, 0.36, 0.25]),
    ("mid_0.52", [0.52, 0.38, 0.25]),
    ("nominal_0.55", [0.55, 0.40, 0.25]),
)
BUDGET_S = 60.0


def main():
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_perception.eval.isolated_domain import isolated_replay_env
    from luggage_planning.lattice_route import LatticeStateChecker
    from luggage_planning.planning_scene_client import PlanningSceneClient
    from luggage_planning.route_producer import register_attach_from_request

    os.makedirs(OUT, exist_ok=True)
    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        raise SystemExit("residual sim processes: %s" % residuals)
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    proc = score._start_moveit(  # noqa: SLF001
        env, replay.SCENE_TF, os.path.join(OUT, "moveit.log"))
    rclpy.init()
    node = Node("slot656_size_ladder")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    rows = []
    try:
        from luggage_msgs.srv import ValidateRouteV2

        probe = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not probe.wait_for_service(timeout_sec=90.0):
            raise SystemExit("validate_route_v2 did not come up")
        scene_config = load_scene_tf_config(replay.SCENE_TF)
        classify._sync_world(  # noqa: SLF001
            node, scene_config, json.load(open(replay.SCENE_JSON, encoding="utf-8")))
        document, surface, _scene, slot = classify._request_document(  # noqa: SLF001
            replay.REQUEST, replay.SURFACE, replay.SCENE_JSON, 656)
        stamp_ns = int(surface.get("acquisition_stamp_ns") or 1)
        producer, _urdf, _tcp = classify._producer(  # noqa: SLF001
            node, scene_config, replay.SCENE_TF, None, stamp_ns,
            budget_s=BUDGET_S)
        scene_client = PlanningSceneClient(node)
        producer.corridor_planner = "lattice"
        producer.lattice_checker_factory = (
            lambda scene_states, start_q: LatticeStateChecker(
                node, scene_client, scene_states, start_q, distance_stride=1))
        for name, size in SIZES:
            document = dict(document)
            document["box_wdh"] = list(size)
            request = classify._build_request(  # noqa: SLF001
                document, surface, slot, "slot656-size-ladder", stamp_ns)
            request["box_wdh"] = [float(v) for v in size]
            request["request_id"] = "slot656-%s" % name
            request["box_id"] = "slot656-%s" % name
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
            detail = {}
            for row in result.get("rows") or []:
                lattice = (row.get("detail") or {}).get("lattice") if isinstance(row, dict) else None
                if lattice:
                    detail = lattice
            edges = replay._edge_rows(result)  # noqa: SLF001
            record = {
                "name": name,
                "size_m": list(size),
                "fixture_size_m": [float(v) for v in json.load(open(
                    replay.REQUEST, encoding="utf-8"))["box_wdh"]],
                "success": bool(result.get("success")),
                "reason_code": result.get("reason_code"),
                "certified": bool(
                    result.get("certification") is not None
                    and result["certification"].passed),
                "wall_s": round(wall, 3),
                "edges": edges,
                "lattice_reason": detail.get("lattice_reason"),
                "first_block": detail.get("first_block"),
                "collision_by_load": detail.get("collision_by_load"),
                "min_collision_distance_m": detail.get(
                    "min_collision_distance_m"),
                "min_sigma": detail.get("min_sigma"),
            }
            rows.append(record)
            with open(os.path.join(OUT, "%s.json" % name), "w", encoding="utf-8") as handle:
                json.dump(record, handle, indent=2, sort_keys=True, default=str)
                handle.write("\n")
            print("%s success=%s reason=%s lattice=%s wall=%.1fs" % (
                name, record["success"], record["reason_code"],
                record["lattice_reason"], wall), flush=True)
    finally:
        score._stop_moveit(proc)  # noqa: SLF001
        if rclpy.ok():
            rclpy.shutdown()
    with open(os.path.join(OUT, "summary.json"), "w", encoding="utf-8") as handle:
        json.dump({"budget_s": BUDGET_S, "slot": 656, "runs": rows},
                  handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
    return 0 if any(row["success"] for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
