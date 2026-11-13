#!/usr/bin/env python3
"""Resubmit the saved PORTAL_TRANSIT goals through the edge builder.

The scene is built from the new request's surface and attachment, then
read before MoveGroup is called. That read is this submission's snapshot.
"""

from __future__ import division

import json
import os
import sys

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

DUMP = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_twobox_rehearsal_20261007b/dumps")
GOALS = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_twobox_rehearsal_20261007b/goal_state_invalid.json")
OUT = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_portal_edge_replay_20261007")


class _Node(object):
    def __init__(self, name, q):
        from luggage_planning.planning_contract.graph import LoadState

        self.node_id = name
        self.q = tuple(float(v) for v in q)
        self.load_state = LoadState.CARRYING


def _distinct(path):
    rows = json.load(open(path, encoding="utf-8"))["edges"]
    seen = []
    keys = set()
    for row in rows:
        key = tuple(round(float(v), 3) for v in row["to_q"])
        if key in keys:
            continue
        keys.add(key)
        seen.append(row)
    return seen


def main():
    import rclpy
    from rclpy.node import Node
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_perception.eval.isolated_domain import isolated_replay_env
    from luggage_planning.routing.moveit_edge_adapter import (
        EdgeClass,
        MoveItEdgeAdapter,
        PayloadSpec,
        SceneState,
        occupancy_boxes_from_surface,
    )

    os.makedirs(OUT, exist_ok=True)
    os.environ["ST09_PORTAL_CAPTURE_DIR"] = os.path.join(OUT, "portal_capture")
    document = json.load(open(
        os.path.join(DUMP, "box_01_paths/planning_request.json"),
        encoding="utf-8"))
    surface = json.load(open(
        os.path.join(DUMP, "trial_01_PLANNING_BUDGET_EXHAUSTED/replay/surface_2d.json"),
        encoding="utf-8"))
    from luggage_description.scene_tf_config_utils import container_inner_floor_z
    surface["floor_z_container"] = float(container_inner_floor_z(
        load_scene_tf_config(paired.SCENE_TF)))
    surface["height_above_floor"] = surface.get("height")
    surface["origin_xy_container"] = surface.get("origin_local")
    surface["resolution_m"] = surface.get("resolution")
    goals = _distinct(GOALS)
    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        print(json.dumps({"outcome": "not_evaluated"}))
        return 2
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    proc = score._start_moveit(  # noqa: SLF001
        env, paired.SCENE_TF, os.path.join(OUT, "moveit.log"), paired.GZ_LAUNCH)
    rclpy.init()
    node = Node("portal_edge_replay")
    stop, thread = score._spin(node)  # noqa: SLF001
    results = []
    try:
        from luggage_msgs.srv import ValidateRouteV2
        client = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not client.wait_for_service(timeout_sec=90.0):
            return 2
        scene_client = paired._scene_client(node)
        classify._sync_world(  # noqa: SLF001
            node, load_scene_tf_config(paired.SCENE_TF), {"world_objects": []})
        paired._reset_case(
            scene_client, "production", document,
            {"scene_path": os.path.join(
                DUMP, "trial_01_PLANNING_BUDGET_EXHAUSTED/scene.json")})
        builder = MoveItEdgeAdapter(node, scene_client=scene_client)
        if not builder.wait_ready(timeout_sec=60.0):
            return 2
        attachment = document["attachment_transform"]
        payload = PayloadSpec(
            dimensions_wdh=tuple(float(v) for v in document["box_wdh"]),
            t_tcp_box_translation=tuple(
                float(v) for v in attachment["translation"]),
            t_tcp_box_quaternion_xyzw=tuple(
                float(v) for v in attachment["quaternion_xyzw"]))
        boxes = occupancy_boxes_from_surface(
            surface, document["container_transform"])
        state = SceneState(
            load_state="carrying", payload=payload, placed_boxes=boxes)
        builder._ensure_scene(state)  # noqa: SLF001
        submission = builder._portal_submission_scene()  # noqa: SLF001
        if submission is not None:
            from luggage_planning.motion.planning_scene_client import (
                summarize_planning_scene,
            )
            summary = summarize_planning_scene(submission)
            acm = submission.allowed_collision_matrix
            names = [str(name) for name in acm.entry_names]
            allowed = None
            if "elfin_base" in names and "robot_pedestal" in names:
                left = names.index("elfin_base")
                right = names.index("robot_pedestal")
                row = acm.entry_values[left].enabled if left < len(
                    acm.entry_values) else []
                allowed = bool(row[right]) if right < len(row) else None
            with open(os.path.join(OUT, "submission_scene.json"), "w",
                      encoding="utf-8") as handle:
                json.dump({
                    "when": "before_move_group_call",
                    "source": "reconstructed_from_saved_request",
                    "world_object_ids": summary.get("world_object_ids"),
                    "attached": summary.get("attached"),
                    "n_world_objects": len(summary.get("world_objects") or []),
                    "acm_n_names": len(names),
                    "elfin_base_robot_pedestal_allowed": allowed,
                }, handle, indent=1)
                handle.write("\n")
        for index, goal in enumerate(goals):
            validity = builder._goal_on_submission_scene(  # noqa: SLF001
                submission, goal["to_q"]) if submission is not None else {
                    "status": "not_evaluated"}
            try:
                builder.build_edge(
                    _Node("from-%d" % index, goal["from_q"]),
                    _Node("to-%d" % index, goal["to_q"]),
                    EdgeClass.PORTAL_TRANSIT, scene=state)
                outcome = "planned"
            except Exception as exc:  # noqa: BLE001 - the refusal is the observation
                outcome = "%s: %s" % (type(exc).__name__, exc)
            last = builder._request_dumps[-1] if builder._request_dumps else {}
            results.append({
                "index": index,
                "goal_q": goal["to_q"],
                "outcome": outcome,
                "moveit_error_code": last.get("error_code"),
                "validity_before_call": validity,
            })
            print(json.dumps(results[-1]), flush=True)
        return 0
    finally:
        stop.set()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        score._stop_moveit(proc)  # noqa: SLF001
        with open(os.path.join(OUT, "edges.json"), "w", encoding="utf-8") as handle:
            json.dump(results, handle, indent=1)
            handle.write("\n")


if __name__ == "__main__":
    sys.exit(main())
