#!/usr/bin/env python3
"""Full second-box Select, original slot order and 180 s budget, twice.

The request is the captured place-v2-2 candidate list in file order.
Production admission reorders that list; this script does not. The scene
is reset before each repeat. A GOAL_STATE_INVALID edge keeps its goal
joints and the live scene.
"""

from __future__ import division

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
import paired_box2_scene_replay as paired  # noqa: E402
import slot64_classify_replay as classify  # noqa: E402
import witness_select_replay as witness  # noqa: E402

DUMP = os.environ.get("SECOND_BOX_DUMP") or os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_online_n3_budget180/dumps")
OUT = os.environ.get("SECOND_BOX_SELECT_OUT") or os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_second_box_select_20261007")
REQUEST = os.path.join(DUMP, "box_01_paths/planning_request.json")
SURFACE = os.path.join(
    DUMP, "trial_01_PLANNING_BUDGET_EXHAUSTED/replay/surface_2d.json")
SCENE = os.path.join(DUMP, "trial_01_PLANNING_BUDGET_EXHAUSTED/scene.json")
GOAL_STATE_INVALID = -27


def _dump(name, document):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name), "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=1, sort_keys=True, default=str)
        handle.write("\n")


def _surface():
    from luggage_description.scene_tf_config_utils import (
        container_inner_floor_z,
        load_scene_tf_config,
    )

    surface = json.load(open(SURFACE, encoding="utf-8"))
    surface["floor_z_container"] = float(container_inner_floor_z(
        load_scene_tf_config(paired.SCENE_TF)))
    surface["height_above_floor"] = surface.get("height")
    surface["origin_xy_container"] = surface.get("origin_local")
    surface["resolution_m"] = surface.get("resolution")
    return surface


def _slots(document):
    slots = []
    for row in document["candidates"]:
        pose = row.get("slot_pose") or {}
        if not pose.get("position"):
            continue
        slots.append({
            "slot_index": int(row.get("slot_index") or 0),
            "placement_score": float(row.get("placement_score") or 0.0),
            "slot_pose": {
                "position": [float(v) for v in pose["position"]],
                "quaternion_xyzw": [
                    float(v) for v in pose["quaternion_xyzw"]],
            },
        })
    return slots


def _watch_edges(builder, node, client, sink):
    original = builder.build_edge

    def wrapped(from_node, to_node, edge_class, candidate=None, scene=None):
        before = len(builder._request_dumps)
        try:
            return original(
                from_node, to_node, edge_class,
                candidate=candidate, scene=scene)
        except Exception:
            dumps = list(builder._request_dumps[before:])
            last = dumps[-1] if dumps else {}
            if int(last.get("error_code") or 0) == GOAL_STATE_INVALID:
                sink.append({
                    "edge_class": str(getattr(edge_class, "value", edge_class)),
                    "from_q": last.get("from_q"),
                    "to_q": last.get("to_q"),
                    "error_code": GOAL_STATE_INVALID,
                    "scene": paired._inventory(client, names_only=False),
                })
            raise

    builder.build_edge = wrapped


def _one(node, client, document, surface, slots, repeat):
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_planning.route_producer import register_attach_from_request
    from luggage_planning.route_producer_factory import (
        build_production_route_producer,
    )

    scene_config = load_scene_tf_config(paired.SCENE_TF)
    stamp_ns = int(surface.get("acquisition_stamp_ns") or 1)
    started = time.perf_counter()

    def clock():
        return stamp_ns + 1_000_000_000 + int(
            (time.perf_counter() - started) * 1e9)

    paired._host(node, scene_config, clock)
    producer = build_production_route_producer(
        node, paired.SCENE_TF, lambda: surface,
        paired._stats_for(surface, clock),
        scene=scene_config, clock_ns=clock, box_count=3,
        corridor_planner="lattice",
        scene_revision="second-box-select",
        vacuum_state_fn=lambda: {
            "attached": True, "vacuum_on": True, "received_ns": clock(),
        },
        site_poses_fn=lambda: None)
    if producer is None or not producer.edge_builder.wait_ready(timeout_sec=60.0):
        return {"success": False, "reason_code": "MOVEIT_UNAVAILABLE"}
    goals = []
    _watch_edges(producer.edge_builder, node, client, goals)
    witness_stub = {"scene_path": SCENE}
    pre = paired._reset_case(
        client, "production", document, witness_stub)
    request = witness._build_many(document, surface, slots, stamp_ns)
    for key in (
            "floor_z_container", "height_above_floor",
            "origin_xy_container", "resolution_m"):
        if key in surface:
            request["surface"][key] = surface[key]
    register_attach_from_request(
        producer.attachment_provider, request,
        box_wdh=request["box_wdh"], box_id=request["box_id"], generation=1)
    wall_started = time.perf_counter()
    result = producer.plan_place_route(request)
    wall_s = time.perf_counter() - wall_started
    reset = getattr(producer.edge_builder, "reset_scene", None)
    if callable(reset):
        reset()
    rows = list(result.get("rows") or [])
    summary = next(
        (row for row in rows if row.get("reason") == "ATTEMPT_SUMMARY"), {})
    selected = next(
        (row for row in rows if row.get("reason") == "OK" and row.get("certified")),
        None)
    if goals:
        _dump("goal_state_invalid_%d.json" % repeat, goals)
    return {
        "repeat": repeat,
        "success": bool(result.get("success")),
        "reason_code": result.get("reason_code"),
        "wall_s": round(wall_s, 3),
        "n_slots": len(slots),
        "n_rows": len(rows),
        "n_tried": summary.get("n_tried"),
        "tried_slot_indices": summary.get("tried_slot_indices"),
        "selected_slot": None if selected is None else selected.get("slot_index"),
        "selected_reason": None if selected is None else selected.get("reason"),
        "pre_digest": pre["inventory"].get("digest"),
        "pre_ids": pre["inventory"].get("world_object_ids"),
        "goal_state_invalid": len(goals),
        "atlas_diag": getattr(producer, "last_atlas_diag", None),
    }


def main():
    import rclpy
    from rclpy.node import Node
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_perception.eval.isolated_domain import isolated_replay_env

    os.makedirs(OUT, exist_ok=True)
    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        _dump("summary.json", {"outcome": "not_evaluated", "residuals": residuals})
        print(json.dumps({"outcome": "not_evaluated"}))
        return 2
    document = json.load(open(REQUEST, encoding="utf-8"))
    surface = _surface()
    slots = _slots(document)
    _dump("request_identity.json", {
        "n_slots": len(slots),
        "first_slot": slots[0]["slot_index"] if slots else None,
        "last_slot": slots[-1]["slot_index"] if slots else None,
        "floor_z_container": surface["floor_z_container"],
        "occupied": paired._occupied(surface) if hasattr(paired, "_occupied") else None,
        "box_count": 3,
        "commit": "ddd1eb22b74d230e1216e86857ea099d84a6f600",
        "urdf_hash_override": "",
    })
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    proc = score._start_moveit(  # noqa: SLF001
        env, paired.SCENE_TF, os.path.join(OUT, "moveit.log"), paired.GZ_LAUNCH)
    rclpy.init()
    node = Node("second_box_select_repeat")
    stop, thread = score._spin(node)  # noqa: SLF001
    reports = []
    repeats = max(1, int(os.environ.get("SECOND_BOX_REPEATS") or "2"))
    try:
        from luggage_msgs.srv import ValidateRouteV2

        client = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not client.wait_for_service(timeout_sec=90.0):
            _dump("summary.json", {"outcome": "not_evaluated"})
            return 2
        scene_client = paired._scene_client(node)
        classify._sync_world(  # noqa: SLF001
            node, load_scene_tf_config(paired.SCENE_TF), {"world_objects": []})
        for repeat in range(repeats):
            print(json.dumps({"repeat": repeat, "event": "start"}), flush=True)
            report = _one(node, scene_client, document, surface, slots, repeat)
            reports.append(report)
            _dump("repeats.json", reports)
            print(json.dumps({
                "repeat": repeat,
                "success": report.get("success"),
                "reason": report.get("reason_code"),
                "wall_s": report.get("wall_s"),
                "selected_slot": report.get("selected_slot"),
                "goal_state_invalid": report.get("goal_state_invalid"),
            }), flush=True)
        return 0
    finally:
        stop.set()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        score._stop_moveit(proc)  # noqa: SLF001
        _dump("summary.json", {
            "repeats": reports,
            "both_certified": all(row.get("success") for row in reports)
            and len(reports) == repeats,
            "residuals_after": score._residuals(),  # noqa: SLF001
        })


if __name__ == "__main__":
    sys.exit(main())
