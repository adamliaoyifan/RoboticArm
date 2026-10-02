"""ST09-I Phase 1 controls 3/4: the original Cartesian probe design,
varied only in its first waypoint and its waypoint count.

Runs against a FRESH stack (move_group's CartesianPathService is serial:
the frozen 656-waypoint batch probe blocks it for minutes, so the fast
single-waypoint variants must run first). The payload rides in the
REQUEST's start_state (request-local attach) so the live planning scene
is never mutated; the scene digest is verified unchanged across the run.

* c3_single (reproduce the fraction-0.000 signature): start joints =
  the failing edge's first sample, ONE waypoint = FK(sample 0) — a
  zero-length first Cartesian segment, exactly the original batched
  probe's first segment.
* c4b_single (nonzero first hop): the same, but the single waypoint is
  FK(sample 1) — a genuinely nonzero segment.
* c3_batch / c4_drop_batch: the original 656-waypoint request (and the
  same minus the duplicate first waypoint), capped at 30 s each — the
  frozen run shows this probe cannot answer within any production
  budget; the timeout outcome is itself the reproducible finding.

Every variant repeats 3 times.
"""

from __future__ import division

import argparse
import hashlib
import json
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for entry in (str(ROOT / "scripts"),):
    if entry not in sys.path:
        sys.path.insert(0, entry)

BATCH_CAP_S = 30.0


def scene_digest(scene_text):
    return hashlib.sha256(scene_text.encode("utf-8")).hexdigest()[:16]


def main():
    parser = argparse.ArgumentParser()
    default_out = ROOT / "docs" / "status" / "evidence" / "frame_migration" / \
        "ST09-parallel-I-interim" / "phase1"
    parser.add_argument("--frozen", required=True)
    parser.add_argument("--evidence-dir", default=str(default_out))
    args = parser.parse_args()
    out_dir = Path(args.evidence_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    frozen = json.loads(Path(args.frozen).read_text())

    # the failing edge: the one the producer certified first (the
    # portal transit); match its first sample against the frozen samples
    edges = frozen.get("edges", {})
    edge_id = None
    for candidate, edge in edges.items():
        if edge.get("edge_class") == "PORTAL_TRANSIT":
            edge_id = candidate
            break
    if edge_id is None and edges:
        edge_id = sorted(edges)[0]
    samples = frozen["samples"][edge_id]
    sample_fk = frozen["sample_fk"][edge_id]
    print("failing edge: %s (%d samples)" % (edge_id, len(samples)))

    import rclpy
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import AttachedCollisionObject
    from moveit_msgs.srv import GetCartesianPath, GetPlanningScene
    from std_msgs.msg import Header
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node("st09_i_control_cartesian")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    import threading as _threading

    spin_stop = _threading.Event()

    def _spin():
        while not spin_stop.is_set():
            executor.spin_once(timeout_sec=0.05)

    spin_thread = _threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    cartesian = node.create_client(GetCartesianPath,
                                   "/compute_cartesian_path")
    scene_client = node.create_client(GetPlanningScene,
                                      "/get_planning_scene")
    if not cartesian.wait_for_service(timeout_sec=15.0):
        print("no /compute_cartesian_path")
        return 3
    if not scene_client.wait_for_service(timeout_sec=15.0):
        print("no /get_planning_scene")
        return 3

    live = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: live.setdefault("q", m), 10)

    def current_scene_text():
        request = GetPlanningScene.Request()
        request.components.components = 255
        future = scene_client.call_async(request)
        deadline = time.monotonic() + 8.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            return None
        scene = future.result().scene
        parts = []
        for obj in scene.world.collision_objects:
            operation = obj.operation
            if isinstance(operation, (bytes, bytearray)):
                operation = int.from_bytes(bytes(operation), "little")
            parts.append("obj:%s:%d:%r" % (
                obj.id, operation,
                [round(p.position.x, 6) for p in obj.primitive_poses]))
        for obj in scene.robot_state.attached_collision_objects:
            parts.append("att:%s:%s:%d" % (
                obj.object.id, obj.link_name, obj.operation))
        return ";".join(sorted(parts))

    # payload from the frozen scene, request-local
    import base64

    from moveit_msgs.msg import PlanningScene as PlanningSceneMsg
    from rclpy.serialization import deserialize_message

    frozen_scene = deserialize_message(
        base64.b64decode(frozen["scene_blob_b64"]), PlanningSceneMsg)
    payload = frozen_scene.robot_state.attached_collision_objects
    print("frozen attached: %s" % [
        (a.object.id, a.link_name, len(a.touch_links)) for a in payload])

    def pose_from(entry):
        pose = Pose()
        pose.position.x = float(entry["position"][0])
        pose.position.y = float(entry["position"][1])
        pose.position.z = float(entry["position"][2])
        pose.orientation.x = float(entry["orientation_xyzw"][0])
        pose.orientation.y = float(entry["orientation_xyzw"][1])
        pose.orientation.z = float(entry["orientation_xyzw"][2])
        pose.orientation.w = float(entry["orientation_xyzw"][3])
        return pose

    def build(start_q, waypoints, *, with_payload=True):
        request = GetCartesianPath.Request()
        request.header = Header(frame_id="world")
        request.start_state.joint_state.name = list(
            frozen["edges"][edge_id]["trajectory"]["joint_names"])
        request.start_state.joint_state.position = [
            float(v) for v in start_q]
        request.start_state.is_diff = True
        if with_payload:
            for attached in payload:
                copy = AttachedCollisionObject()
                copy.link_name = attached.link_name
                copy.object = attached.object
                copy.touch_links = list(attached.touch_links)
                request.start_state.attached_collision_objects.append(copy)
        request.group_name = "elfin_arm"
        request.link_name = "suction_contact_frame"
        for waypoint in waypoints:
            request.waypoints.append(pose_from(waypoint))
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        return request

    def run_probe(name, request, timeout_s):
        event = threading.Event()
        future = cartesian.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        event.wait(timeout=timeout_s)
        if not future.done():
            return {"variant": name, "outcome": "timeout",
                    "budget_s": timeout_s}
        response = future.result()
        return {
            "variant": name, "outcome": "answered",
            "error_code": int(response.error_code.val),
            "fraction": float(response.fraction),
            "solution_points": len(
                response.solution.joint_trajectory.points),
        }

    zero_waypoint = {
        "position": sample_fk[0][0],
        "orientation_xyzw": sample_fk[0][1],
    }
    hop_waypoint = {
        "position": sample_fk[1][0],
        "orientation_xyzw": sample_fk[1][1],
    }
    full_waypoints = [
        {"position": pose[0], "orientation_xyzw": pose[1]}
        for pose in sample_fk]

    scene_before = current_scene_text()
    results = []
    plan = [
        ("c3_single_zero_length",
         build(samples[0], [zero_waypoint]), 15.0),
        ("c4b_single_nonzero_hop",
         build(samples[0], [hop_waypoint]), 15.0),
        # no-payload twins: does the request-local payload alone cause
        # the fraction-0 start? (the state oracle then answers whether
        # the payload genuinely collides)
        ("c3_single_zero_length_no_payload",
         build(samples[0], [zero_waypoint], with_payload=False), 15.0),
        ("c4b_single_nonzero_hop_no_payload",
         build(samples[0], [hop_waypoint], with_payload=False), 15.0),
        ("c3_batch_original",
         build(samples[0], full_waypoints), BATCH_CAP_S),
        ("c4_batch_drop_duplicate",
         build(samples[0], full_waypoints[1:]), BATCH_CAP_S),
        ("c5_batch_no_payload",
         build(samples[0], full_waypoints, with_payload=False),
         BATCH_CAP_S),
    ]
    for name, request, timeout_s in plan:
        for repetition in range(3):
            outcome = run_probe("%s#%d" % (name, repetition), request,
                                timeout_s)
            outcome["repetition"] = repetition
            outcome["n_waypoints"] = len(request.waypoints)
            outcome["with_payload"] = bool(
                request.start_state.attached_collision_objects)
            results.append(outcome)
            print(outcome)

    scene_after = current_scene_text()
    outcome = {
        "frozen": str(Path(args.frozen)),
        "failing_edge": edge_id,
        "n_samples": len(samples),
        "scene_digest_before": scene_digest(scene_before or ""),
        "scene_digest_after": scene_digest(scene_after or ""),
        "scene_unchanged": scene_before == scene_after,
        "results": results,
    }
    message = live.get("q")
    if message is not None:
        outcome["joint_state_now"] = [
            float(message.position[list(message.name).index(
                "elfin_joint%d" % i)]) for i in range(1, 7)]
    out_path = out_dir / "controls_cartesian.json"
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(outcome, handle, indent=1)
    print("wrote", out_path, "scene_unchanged=", outcome["scene_unchanged"])
    spin_stop.set()
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
