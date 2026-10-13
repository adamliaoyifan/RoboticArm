#!/usr/bin/env python3
"""Diagnostic: which existing IK seeds give a payload-valid carrying anchor?

For the first candidate of the frozen request, capture the anchor plan that
``RouteProducer._prepare_candidate`` builds, then refine every carrying anchor
from each EXISTING seed family (live start, tool-down prior, tool-down
j1-minus-pi basin) and judge each IK solution with MoveIt's own
GetStateValidity in the frozen payload-attached scene. No new seed family.

usage: g2_anchor_probe.py <out_dir>
"""

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g2_common as common  # noqa: E402
import g2_frozen_select as frozen  # noqa: E402
import g2_validity_matrix as matrix  # noqa: E402


class _Stop(BaseException):
    pass


def main():
    out = sys.argv[1]
    import rclpy
    from rclpy.node import Node
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_msgs.srv import SelectPlacePathV2
    from luggage_planning.place_service import fill_select_response_route
    from luggage_planning.route_producer import tool_down_branch_seeds

    stack = common.MoveItStack(out, "anchor_probe")
    stack.start()
    rclpy.init()
    node = Node("g2_anchor_probe")
    stop, thread = common.score._spin(node)  # noqa: SLF001
    document = {"status": "incomplete", "anchors": []}
    try:
        import slot64_classify_replay as classify
        from luggage_msgs.srv import ValidateRouteV2

        request_msg = common.load_request()
        surface = json.load(open(common.SURFACE_JSON, encoding="utf-8"))
        surface["floor_z_container"] = float(request_msg.surface.floor_z_container)
        scene_config = load_scene_tf_config(common.SCENE_TF)
        node.create_client(ValidateRouteV2, "/validate_route_v2").wait_for_service(120.0)
        client = common.paired._scene_client(node)  # noqa: SLF001
        classify._sync_world(node, scene_config, {"world_objects": []})  # noqa: SLF001
        common.paired._purge(client)  # noqa: SLF001
        producer, clock, _ = frozen.build_host_and_producer(
            node, request_msg, surface, scene_config)
        producer.edge_builder.wait_ready(timeout_sec=60.0)
        captured = []
        original = producer._prepare_candidate  # noqa: SLF001

        def wrapped(base_candidate, attachment, request, start_q, tcp_orientation, tcp_position):
            result = original(base_candidate, attachment, request, start_q,
                              tcp_orientation=tcp_orientation, tcp_position=tcp_position)
            captured.append({"plan": result[2], "orientation": tuple(tcp_orientation),
                             "start_q": tuple(start_q), "slot": base_candidate.candidate_id})
            raise _Stop()

        producer._prepare_candidate = wrapped  # noqa: SLF001
        try:
            fill_select_response_route(
                request_msg, SelectPlacePathV2.Response(), producer,
                surface_validity={"occupancy_confirmed": True, "change_event": ""})
        except _Stop:
            pass
        producer.edge_builder.reset_scene()
        item = captured[0]
        plan, orientation, start_q = item["plan"], item["orientation"], item["start_q"]
        document["candidate"] = item["slot"]
        document["orientation_xyzw"] = list(orientation)
        # Frozen payload-attached scene (same bytes as G1 portal capture 0).
        record, scene = common.load_portal(0)
        matrix.apply_full_scene(client, scene)
        time.sleep(0.5)
        seeds = [("start", start_q)] + list(tool_down_branch_seeds())
        for region in ("staging", "portal", "slot_approach", "slot_upper"):
            position = getattr(plan, region)
            if position is None:
                continue
            for label, seed in seeds:
                q, info = producer.refine(tuple(position), tuple(orientation), list(seed))
                row = {"region": region, "position": list(position), "seed": label,
                       "ik_ok": q is not None}
                if q is not None:
                    row["q"] = [float(v) for v in q]
                    row["moveit_validity"] = common.paired._goal_validity(node, q)  # noqa: SLF001
                document["anchors"].append(row)
                print(json.dumps({"region": region, "seed": label, "ik_ok": row["ik_ok"],
                                  "valid": (row.get("moveit_validity") or {}).get("valid")}),
                      flush=True)
        document["status"] = "complete"
    finally:
        stop.set()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stack.stop()
        common.dump(out, "anchor_probe.json", document)


if __name__ == "__main__":
    main()
