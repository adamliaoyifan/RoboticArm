#!/usr/bin/env python3
"""G2-3: the frozen 828-slot SelectPlacePathV2 request through the real entry.

Entry: ``place_service.fill_select_response_route`` (what
``trajectory_selector._select_v2`` calls) over a producer built by
``build_production_route_producer`` with graph planner, 1 box (60 s budget) and
the production certification contract. The MoveIt process, scene, producer and
attachment record are created fresh for every run; nothing is reused.

Clock: the harness clock starts at the request's measured grasp stamp plus 1 s
and advances with the REAL monotonic wall clock; the search budget is the
production one and is not frozen.

usage: g2_frozen_select.py <out_dir> <run_label>
"""

import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g2_common as common  # noqa: E402


def build_host_and_producer(node, request_msg, surface, scene_config):
    from luggage_planning.route_producer_factory import (
        build_production_route_producer,
    )

    stamp_ns = int(request_msg.grasp.identity.stamp_ns)
    started = time.perf_counter()

    def clock():
        return stamp_ns + 1_000_000_000 + int((time.perf_counter() - started) * 1e9)

    common.paired._host(node, scene_config, clock)  # noqa: SLF001
    producer = build_production_route_producer(
        node, common.SCENE_TF, lambda: surface,
        common.paired._stats_for(surface, clock),  # noqa: SLF001
        scene=scene_config, clock_ns=clock, box_count=1,
        corridor_planner="graph",
        scene_revision=str(request_msg.grasp.identity.scene_revision),
        vacuum_state_fn=lambda: {
            "attached": True, "vacuum_on": True, "received_ns": clock()},
        site_poses_fn=lambda: None)
    return producer, clock, started


def run_once(out, label):
    import rclpy
    from rclpy.node import Node
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_msgs.srv import SelectPlacePathV2
    from luggage_planning.place_service import fill_select_response_route
    from luggage_planning.grid_path_search import route_budget_ns
    from rclpy.serialization import serialize_message

    report = {"label": label, "status": "incomplete", "env": {
        k: os.environ.get(k) for k in (
            "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "ST09_PORTAL_CAPTURE_DIR")}}
    os.makedirs(out, exist_ok=True)
    os.environ["ST09_PORTAL_CAPTURE_DIR"] = os.path.join(out, "portal_capture")
    stack = common.MoveItStack(out, label)
    t_launch = time.time()
    stack.start()
    rclpy.init()
    node = Node("g2_frozen_select_%s" % label)
    stop, thread = common.score._spin(node)  # noqa: SLF001
    try:
        from luggage_msgs.srv import ValidateRouteV2

        request_msg = common.load_request()
        report["request_sha256"] = common.sha256_file(common.REQUEST_BIN)
        report["n_slots"] = len(request_msg.slots)
        surface = json.load(open(common.SURFACE_JSON, encoding="utf-8"))
        surface["floor_z_container"] = float(request_msg.surface.floor_z_container)
        scene_config = load_scene_tf_config(common.SCENE_TF)
        validate = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not validate.wait_for_service(timeout_sec=120.0):
            report["status"] = "startup_failed"
            report["detail"] = "validate_route_v2 not ready in 120 s"
            return report
        report["startup_s"] = round(time.time() - t_launch, 1)
        scene_client = common.paired._scene_client(node)  # noqa: SLF001
        import slot64_classify_replay as classify
        classify._sync_world(  # noqa: SLF001
            node, scene_config, {"world_objects": []})
        common.paired._purge(scene_client)  # noqa: SLF001
        producer, clock, _ = build_host_and_producer(
            node, request_msg, surface, scene_config)
        if producer is None or not producer.edge_builder.wait_ready(timeout_sec=60.0):
            report["status"] = "startup_failed"
            report["detail"] = "producer or edge builder unavailable"
            return report
        report["params"] = {
            "corridor_planner": producer.corridor_planner,
            "budget_s": producer.budget_ns / 1e9,
            "route_budget_for_1_box_s": route_budget_ns(1) / 1e9,
            "transit_clearance_z": producer.transit_clearance_z,
        }
        report["pre_inventory"] = {
            k: v for k, v in common.paired._inventory(  # noqa: SLF001
                scene_client, names_only=False).items() if k != "objects"}
        response = SelectPlacePathV2.Response()
        began = time.perf_counter()
        response = fill_select_response_route(
            request_msg, response, producer,
            surface_validity={"occupancy_confirmed": True, "change_event": ""})
        wall_s = time.perf_counter() - began
        rows = json.loads(response.rows_json or "[]")
        timings = json.loads(response.stage_timings_json or "{}")
        summary = next((r for r in rows if r.get("reason") == "ATTEMPT_SUMMARY"), {})
        winner = next((r for r in rows if r.get("reason") == "OK" and r.get("certified")), None)
        report.update({
            "success": bool(response.success), "reason_code": response.reason_code,
            "wall_s": round(wall_s, 3), "stage_timings": timings,
            "slot_index": int(response.slot_index),
            "n_motion_segments": len(response.motion_segments),
            "n_carry_segments": len(response.carry_segments),
            "path_hash": response.path_hash, "selection_id": response.selection_id,
            "n_rows": len(rows), "n_tried": summary.get("n_tried"),
            "tried_slot_indices": summary.get("tried_slot_indices"),
            "winner_slot": None if winner is None else winner.get("slot_index"),
            "winner_certified": None if winner is None else bool(winner.get("certified")),
            "move_group_calls": dict(getattr(producer.edge_builder, "_call_counts", {})),
            "edge_failures": [
                {"from": d.get("from"), "to": d.get("to"), "error_code": d.get("error_code")}
                for d in list(getattr(producer.edge_builder, "_request_dumps", []))
                if int(d.get("error_code") or 1) != 1][:200],
        })
        common.dump(out, "rows.json", rows)
        with open(os.path.join(out, "portal_provenance.jsonl"), "w", encoding="utf-8") as handle:
            for item in getattr(producer, "anchor_provenance", []):
                handle.write(json.dumps(item, sort_keys=True) + "\n")
        report["anchor_gate"] = {
            "rows": len(getattr(producer, "anchor_provenance", [])),
            "rejected": sum(1 for i in getattr(producer, "anchor_provenance", [])
                            if i["verdict"] == "rejected"),
            "not_evaluated": sum(1 for i in getattr(producer, "anchor_provenance", [])
                                 if i["verdict"] == "not_evaluated"),
            "checker_calls": getattr(getattr(producer, "_anchor_gate", None), "submitted", None),
            "cache_hits": getattr(getattr(producer, "_anchor_gate", None), "cache_hits", None)}
        with open(os.path.join(out, "response.bin"), "wb") as handle:
            handle.write(bytes(serialize_message(response)))
        report["status"] = "complete"
    finally:
        stop.set()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stack.stop()
        common.dump(out, "run_report.json", report)
    return report


def main():
    out, label = sys.argv[1], sys.argv[2]
    report = run_once(out, label)
    print(json.dumps({k: report.get(k) for k in (
        "label", "status", "success", "reason_code", "wall_s", "winner_slot",
        "n_tried", "stage_timings")}, default=str))
    return 0 if report.get("success") else 1


if __name__ == "__main__":
    sys.exit(main())
