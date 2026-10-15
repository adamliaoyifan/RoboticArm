#!/usr/bin/env python3
"""G2 baseline runner: replay each frozen fixture in isolated MoveIt.

Records per-trial: success, wall_s, candidate count, edge call results
(error codes from request_dumps), stage timings (G1 instrumentation),
and first-certified-route timing. 3 reps per fixture, alternating.

usage: g2_baseline_runner.py <campaign_dir> [condition=isolated]
"""
import hashlib
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, "/tmp/elfin-master-pack-baseline-g3/scripts/g2")
import g2_common as common  # noqa: E402
from g2_frozen_select import build_host_and_producer  # noqa: E402

WS = "/tmp/elfin-master-pack-baseline-g3"


def replay_once(fixture, out_dir, label):
    import rclpy
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message
    from moveit_msgs.msg import PlanningScene
    from luggage_msgs.srv import SelectPlacePathV2, ValidateRouteV2
    from geometry_msgs.msg import Pose
    from shape_msgs.msg import SolidPrimitive
    from luggage_planning.place_service import fill_select_response_route

    os.makedirs(out_dir, exist_ok=True)
    os.environ["ST09_I_CERT_CAPTURE_DIR"] = os.path.join(out_dir, "cert_capture")
    stack = common.MoveItStack(out_dir, label)
    t_launch = time.time()
    stack.start()
    rclpy.init()
    node = Node("g2_%s" % label.replace("-", "_").replace(".", "_"))
    stop, thread = common.score._spin(node)  # noqa: SLF001

    trial = {"label": label, "fixture": fixture["id"],
             "stratum": fixture["stratum"], "status": "incomplete"}
    try:
        request_msg = deserialize_message(
            open(fixture["select_bin"], "rb").read(),
            SelectPlacePathV2.Request)
        trial["n_slots"] = len(request_msg.slots)
        surface = json.load(open(fixture["surface_json"]))
        surface["floor_z_container"] = float(
            request_msg.surface.floor_z_container)

        from luggage_description.scene_tf_config_utils import load_scene_tf_config
        scene_config = load_scene_tf_config(os.path.join(
            WS, "src/luggage_description/config/scene_tf_deploy_update.yaml"))

        validate = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not validate.wait_for_service(timeout_sec=120.0):
            trial["status"] = "startup_failed"
            return trial
        trial["startup_s"] = round(time.time() - t_launch, 1)

        scene_client = common.paired._scene_client(node)  # noqa: SLF001
        import slot64_classify_replay as classify
        world_json = {"world_objects": [
            {"type": "box", "id": o["id"], "xyz": o["xyz"],
             "quat": o["quat"], "size": o["size"]}
            for o in fixture.get("world_objects", [])]}
        classify._sync_world(node, scene_config, world_json)  # noqa: SLF001
        common.paired._purge(scene_client)  # noqa: SLF001

        producer, clock, _ = build_host_and_producer(
            node, request_msg, surface, scene_config)
        if producer is None or not producer.edge_builder.wait_ready(timeout_sec=60.0):
            trial["status"] = "producer_unavailable"
            return trial

        # Snapshot adapter counters before the call
        counts_before = dict(producer.edge_builder.moveit_call_counts())
        trial["adapter_counts_before"] = counts_before

        response = SelectPlacePathV2.Response()
        began = time.perf_counter()
        response = fill_select_response_route(
            request_msg, response, producer,
            surface_validity={"occupancy_confirmed": True,
                              "change_event": ""})
        wall_s = time.perf_counter() - began

        rows = json.loads(response.rows_json or "[]")
        timings = json.loads(response.stage_timings_json or "{}")
        winner = next(
            (r for r in rows
             if r.get("reason") == "OK" and r.get("certified")), None)

        # Collect G1 instrumentation from the adapter
        edge_builder = producer.edge_builder
        dumps = edge_builder.request_dumps() if callable(
            getattr(edge_builder, "request_dumps", None)) else []
        stage_times = edge_builder.stage_timings() if callable(
            getattr(edge_builder, "stage_timings", None)) else {}

        error_codes = [d.get("error_code") for d in dumps]
        trial.update({
            "success": bool(response.success),
            "reason_code": response.reason_code,
            "wall_s": round(wall_s, 3),
            "winner_slot": None if winner is None else winner.get("slot_index"),
            "stage_timings_response": timings,
            "adapter_stage_timings": {
                k: {"n": len(v), "total": round(sum(v), 4)}
                for k, v in stage_times.items() if v},
            "edge_calls": {
                "total": len(dumps),
                "error_codes": error_codes,
                "success_count": sum(1 for c in error_codes if c == 1),
                "invalid_motion_plan": sum(1 for c in error_codes if c == -2),
                "timed_out": sum(1 for c in error_codes if c == -6)},
            "candidates_tried": len(rows),
            "status": "complete",
        })
        # capped time: failures charged 60 s
        trial["capped_s"] = min(wall_s, 60.0) if not trial["success"] \
            else wall_s
        return trial
    finally:
        stop.set()
        thread.join(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown()
        stack.stop()


def main():
    campaign_dir = sys.argv[1]
    condition = sys.argv[2] if len(sys.argv) > 2 else "isolated"
    fixtures_doc = json.load(open(os.path.join(campaign_dir, "fixtures.json")))
    fixtures = {k: v for k, v in fixtures_doc["fixtures"].items() if v}
    reps = 3
    out_root = os.path.join(campaign_dir, "g2_%s" % condition)
    os.makedirs(out_root, exist_ok=True)

    trials = []
    # alternating: rep1 of all fixtures, rep2 of all, rep3 of all
    for rep in range(1, reps + 1):
        for fid in sorted(fixtures):
            fixture = fixtures[fid]
            label = "%s_r%d" % (fid, rep)
            out_dir = os.path.join(out_root, label)
            print("running %s..." % label)
            trial = replay_once(fixture, out_dir, label)
            trials.append(trial)
            print(json.dumps({k: trial.get(k) for k in (
                "label", "success", "wall_s", "winner_slot",
                "edge_calls.total", "status")}))
            # [G1] save the trial's evidence
            json.dump(trial, open(os.path.join(out_dir, "trial.json"), "w"),
                      indent=1, default=str)

    # aggregate per stratum
    by_stratum = {}
    for t in trials:
        by_stratum.setdefault(t["stratum"], []).append(t)
    summary = {}
    for stratum, stratum_trials in by_stratum.items():
        successes = [t for t in stratum_trials if t.get("success")]
        all_walls = sorted(t.get("capped_s", 60.0) for t in stratum_trials)
        success_walls = sorted(
            t["wall_s"] for t in successes) if successes else []
        summary[stratum] = {
            "n": len(stratum_trials),
            "successes": len(successes),
            "capped_p50": all_walls[len(all_walls)//2] if all_walls else None,
            "capped_max": all_walls[-1] if all_walls else None,
            "success_only_p50": (
                success_walls[len(success_walls)//2]
                if success_walls else None),
            "edge_calls_total": sum(
                t.get("edge_calls", {}).get("total", 0)
                for t in stratum_trials),
            "error_code_counts": {
                str(c): sum(
                    1 for t in stratum_trials
                    for e in t.get("edge_calls", {}).get("error_codes", [])
                    if e == c)
                for c in set(
                    e for t in stratum_trials
                    for e in t.get("edge_calls", {}).get("error_codes", []))},
            "adapter_stage_totals": {
                key: round(sum(
                    t.get("adapter_stage_timings", {}).get(key, {}).get("total", 0)
                    for t in stratum_trials), 4)
                for key in ("scene_apply", "move_group",
                            "retime_and_interpolation")},
        }
    report = {"condition": condition, "reps": reps, "trials": trials,
              "summary": summary}
    json.dump(report,
              open(os.path.join(out_root, "baseline_report.json"), "w"),
              indent=1, default=str)
    print("\n=== %s baseline summary ===" % condition)
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
