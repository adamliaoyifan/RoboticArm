#!/usr/bin/env python3
"""Replay a captured ValidateRouteV2 request and separate sampling from scene.

usage: g2_cert_replay.py <run_dir_with_cert_capture> <out_json>

Variants (same frozen scenes, same model, only the sample set changes):
  exact    the captured request unchanged
  knots    leg samples reduced to the original knots (planner-side states)
  sample   knots plus the single first-invalid sample of the exact replay
Verdict classification is written to <out_json>; nothing is inferred beyond
what these three calls show.
"""
import base64, copy, json, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g2_common as common  # noqa: E402


def main():
    run_dir, out_json = sys.argv[1], sys.argv[2]
    import rclpy
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message
    from luggage_msgs.srv import ValidateRouteV2
    document = json.load(open(os.path.join(run_dir, "cert_capture", "cert_request.json")))
    request = deserialize_message(base64.b64decode(document["request_b64"]), ValidateRouteV2.Request)
    stack = common.MoveItStack(os.path.dirname(out_json), "cert_replay")
    stack.start()
    rclpy.init()
    node = Node("g2_cert_replay")
    stop, thread = common.score._spin(node)
    report = {"source": run_dir, "request_id": request.request_id, "variants": {}}
    try:
        client = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not client.wait_for_service(timeout_sec=120.0):
            raise RuntimeError("validate_route_v2 unavailable")

        def call(req):
            future = client.call_async(req)
            end = time.time() + 120
            while not future.done() and time.time() < end:
                time.sleep(0.05)
            response = future.result()
            return {"result": int(response.result), "detail": str(response.detail),
                    "all_valid": bool(response.all_valid),
                    "model_sha256": str(response.model_sha256),
                    "scene_digest": str(response.scene_digest),
                    "legs": [{"edge": l.edge_id, "valid": bool(l.valid), "reason": str(l.reason),
                              "first_invalid": int(l.first_invalid_sample), "n_samples": int(l.n_samples),
                              "contacts": [{"b1": str(c.body_1), "b2": str(c.body_2),
                                            "depth": float(c.depth)} for c in getattr(l, "contacts", [])][:4]}
                             for l in response.leg_results]}, response

        exact, response = call(request)
        report["variants"]["exact"] = exact
        report["captured_scene_digest"] = document.get("scene_digest")
        failing = [i for i, l in enumerate(exact["legs"]) if not l["valid"]]
        report["failing_legs"] = failing
        n_joints = len(request.legs[0].joint_names)
        for index in failing[:1]:
            leg = request.legs[index]
            first = exact["legs"][index]["first_invalid"]
            times = list(leg.sample_times_ns)
            knot_times = [p.time_from_start.sec * 10**9 + p.time_from_start.nanosec for p in leg.knots.points]
            bracket = [i for i, t in enumerate(knot_times) if t <= times[first]]
            report["failing_sample"] = {
                "leg": index, "edge_class": leg.edge_class, "sample_index": first,
                "time_ns": times[first], "n_samples": len(times), "n_knots": len(knot_times),
                "between_knots": [bracket[-1] if bracket else None,
                                  (bracket[-1] + 1) if bracket and bracket[-1] + 1 < len(knot_times) else None],
                "is_a_knot": times[first] in knot_times,
                "sample_q": list(leg.sample_positions_flat[first * n_joints:(first + 1) * n_joints])}

            def variant(keep_indices):
                req = copy.deepcopy(request)
                l = req.legs[index]
                l.n_samples = len(keep_indices)
                l.sample_times_ns = [times[i] for i in keep_indices]
                flat = []
                for i in keep_indices:
                    flat.extend(leg.sample_positions_flat[i * n_joints:(i + 1) * n_joints])
                l.sample_positions_flat = flat
                req.max_sample_dt_ns = 1e18
                req.max_sample_step_m = 1e9
                req.legs = [l]
                req.load_state_scenes = list(req.load_state_scenes)
                return req
            knot_idx = [i for i, t in enumerate(times) if t in set(knot_times)]
            report["variants"]["knots_only"], _ = call(variant(knot_idx))
            report["variants"]["knots_plus_failing_sample"], _ = call(variant(sorted(set(knot_idx + [first]))))
            report["variants"]["failing_sample_only_with_endpoints"], _ = call(
                variant(sorted({0, len(times) - 1, first})))
    finally:
        json.dump(report, open(out_json, "w"), indent=1)
        stop.set(); thread.join(timeout=2.0); node.destroy_node(); rclpy.shutdown(); stack.stop()
    print(json.dumps({k: {"all_valid": v["all_valid"], "result": v["result"],
                          "first_invalid": [l["first_invalid"] for l in v["legs"] if not l["valid"]]}
                      for k, v in report["variants"].items()}, indent=1))
    print(json.dumps(report.get("failing_sample", {}))[:600])


main()
