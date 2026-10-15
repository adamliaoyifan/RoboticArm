#!/usr/bin/env python3
"""Replay the scored run's BOX-2 SelectPlacePathV2 request offline (wave 4).

Same harness as scripts/g2/g2_frozen_select.py but loads the captured
box-2 request (627 candidates, rev-2 map with box-1 occupancy) and syncs
the MoveIt world WITH box 1 as a placed collision box, so the replay is
faithful to the scored scene. Verifies the scene-policy atlas load
(seeded ordering) and whether a route certifies inside the frozen 60 s.

usage: replay_box2_select.py <out_dir> <label>
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, "/tmp/elfin-master-pack-baseline-g3/scripts/g2")
import g2_common as common  # noqa: E402
from g2_frozen_select import build_host_and_producer  # noqa: E402

SCORE = ("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
         "master_pack_recovery/2026-10-14_g3/score_run01")
common.REQUEST_BIN = os.path.join(
    SCORE, "select_capture", "select_place_path_v2_01.bin")
common.SURFACE_JSON = os.path.join(
    SCORE, "dumps", "box_01_PLANNING_BUDGET_EXHAUSTED", "surface_2d.json")

# Box 1 as placed in the scored run (chosen slot, world frame).
WORLD_OBJECTS = [{
    "type": "box", "id": "placed_box_1",
    "xyz": [-2.03, 0.46, 0.639],
    "quat": [0.0, 0.0, 0.0, 1.0],
    "size": [0.5099, 0.332, 0.2175],
}]


def run_once(out, label):
    import rclpy
    from rclpy.node import Node
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_msgs.srv import SelectPlacePathV2
    from luggage_planning.place_service import fill_select_response_route
    from luggage_planning.grid_path_search import route_budget_ns

    report = {"label": label, "status": "incomplete", "request": "box2"}
    os.makedirs(out, exist_ok=True)
    os.environ["ST09_PORTAL_CAPTURE_DIR"] = os.path.join(out, "portal_capture")
    os.environ["ST09_I_CERT_CAPTURE_DIR"] = os.path.join(out, "cert_capture")
    stack = common.MoveItStack(out, label)
    t_launch = time.time()
    stack.start()
    rclpy.init()
    node = Node("g3_box2_replay_%s" % label)
    stop, thread = common.score._spin(node)  # noqa: SLF001
    try:
        from luggage_msgs.srv import ValidateRouteV2

        request_msg = common.load_request()
        report["n_slots"] = len(request_msg.slots)
        surface = json.load(open(common.SURFACE_JSON, encoding="utf-8"))
        surface["floor_z_container"] = float(
            request_msg.surface.floor_z_container)
        scene_config = load_scene_tf_config(common.SCENE_TF)
        validate = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not validate.wait_for_service(timeout_sec=120.0):
            report["status"] = "startup_failed"
            return report
        report["startup_s"] = round(time.time() - t_launch, 1)
        scene_client = common.paired._scene_client(node)  # noqa: SLF001
        import slot64_classify_replay as classify
        classify._sync_world(  # noqa: SLF001
            node, scene_config, {"world_objects": WORLD_OBJECTS})
        common.paired._purge(scene_client)  # noqa: SLF001
        producer, clock, _ = build_host_and_producer(
            node, request_msg, surface, scene_config)
        if producer is None or not producer.edge_builder.wait_ready(
                timeout_sec=60.0):
            report["status"] = "startup_failed"
            return report
        report["atlas_diag"] = getattr(producer, "last_atlas_diag", None)
        response = SelectPlacePathV2.Response()
        began = time.perf_counter()
        response = fill_select_response_route(
            request_msg, response, producer,
            surface_validity={"occupancy_confirmed": True, "change_event": ""})
        wall_s = time.perf_counter() - began
        rows = json.loads(response.rows_json or "[]")
        with open(os.path.join(out, "rows.json"), "w") as handle:
            json.dump(rows, handle, indent=1)
        timings = json.loads(response.stage_timings_json or "{}")
        report["stage_timings"] = timings
        winner = next(
            (r for r in rows if r.get("reason") == "OK" and r.get("certified")),
            None)
        report.update({
            "success": bool(response.success),
            "reason_code": response.reason_code,
            "wall_s": round(wall_s, 3),
            "slot_index": int(response.slot_index),
            "winner_slot": None if winner is None else winner.get("slot_index"),
            "winner_certified": bool(winner),
        })
        return report
    finally:
        stop.set()
        thread.join(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown()
        stack.stop()


def main():
    out, label = sys.argv[1], sys.argv[2]
    report = run_once(out, label)
    with open(os.path.join(out, "run_report.json"), "w") as handle:
        json.dump(report, handle, indent=1, default=str)
        handle.write("\n")
    print(json.dumps({k: report.get(k) for k in (
        "success", "reason_code", "wall_s", "winner_slot", "n_slots",
        "startup_s")}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
