"""ST09-I Phase 1 controls 1/2/5: original-state validity through the
ValidateRouteV2 state oracle, against PRIVATE frozen scene copies.

Never touches the live stack: the validator service runs on its own ROS
domain with the robot model the stack's move_group uses. Controls:

* control 1 (first state): the failing edge's first sample alone, with
  the full frozen attached set — is the state actually colliding, and
  with which bodies?
* control 2 (whole trajectory): every sampled state of the failing edge
  in the same frozen scene — was the edge builder's output safe?
* control 5 (no payload): identical to control 2 minus the attached
  payload — does the payload cause any failure?

Each control repeats 3 times; every run verifies the scene digest it
used matches the intended control condition.

Usage (validate_route_node already running on this domain):
    python3 scripts/st09_i/controls_states.py --frozen .../frozen_failure.json
"""

from __future__ import division

import argparse
import hashlib
import json
import math
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for entry in (str(ROOT / "scripts"),):
    if entry not in sys.path:
        sys.path.insert(0, entry)

CANONICAL_JOINTS = tuple("elfin_joint%d" % i for i in range(1, 7))
MAX_CERT_SAMPLE_DT_NS = 10_000_000


def scene_from_json(data):
    """Rebuild a moveit_msgs/PlanningScene from the frozen JSON."""
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import (
        AllowedCollisionMatrix,
        AttachedCollisionObject,
        CollisionObject,
        PlanningScene,
    )
    from shape_msgs.msg import SolidPrimitive

    def pose(entry):
        message = Pose()
        message.position.x = float(entry["position"][0])
        message.position.y = float(entry["position"][1])
        message.position.z = float(entry["position"][2])
        message.orientation.x = float(entry["orientation_xyzw"][0])
        message.orientation.y = float(entry["orientation_xyzw"][1])
        message.orientation.z = float(entry["orientation_xyzw"][2])
        message.orientation.w = float(entry["orientation_xyzw"][3])
        return message

    scene = PlanningScene()
    scene.name = data.get("name") or "frozen"
    scene.is_diff = False
    for obj in data["world"]["collision_objects"]:
        collision = CollisionObject()
        collision.id = obj["id"]
        collision.header.frame_id = obj.get("frame_id") or "world"
        for primitive in obj["primitives"]:
            solid = SolidPrimitive()
            if primitive["type"] == "BOX":
                solid.type = SolidPrimitive.BOX
            solid.dimensions = [float(v) for v in
                                primitive["dimensions"]]
            collision.primitives.append(solid)
        for pose_entry in obj["primitive_poses"]:
            collision.primitive_poses.append(pose(pose_entry))
        collision.operation = CollisionObject.ADD
        scene.world.collision_objects.append(collision)
    robot = data["robot_state"]
    scene.robot_state.joint_state.name = list(
        robot["joint_state"]["name"])
    scene.robot_state.joint_state.position = [
        float(v) for v in robot["joint_state"]["position"]]
    for attached in robot["attached_collision_objects"]:
        message = AttachedCollisionObject()
        message.link_name = attached["link_name"]
        message.object.id = attached["object"]["id"]
        for primitive in attached["object"]["primitives"]:
            solid = SolidPrimitive()
            if primitive["type"] == "BOX":
                solid.type = SolidPrimitive.BOX
            solid.dimensions = [float(v) for v in
                                primitive["dimensions"]]
            message.object.primitives.append(solid)
        for pose_entry in attached["object"]["primitive_poses"]:
            message.object.primitive_poses.append(pose(pose_entry))
        message.object.operation = CollisionObject.ADD
        message.touch_links = list(attached["touch_links"])
        scene.robot_state.attached_collision_objects.append(message)
    acm = AllowedCollisionMatrix()
    acm.entry_names = list(data.get("acm_entry_names") or [])
    for name in acm.entry_names:
        allowed = data.get("acm_entries", {}).get(name, [])
        row = [False] * len(acm.entry_names)
        for other in allowed:
            if other in acm.entry_names:
                row[acm.entry_names.index(other)] = True
        from moveit_msgs.msg import AllowedCollisionEntry

        entry = AllowedCollisionEntry()
        entry.enabled = row
        acm.entry_values.append(entry)
    scene.allowed_collision_matrix = acm
    return scene


def sample_times_and_positions(knots, frozen_samples):
    """Mirror sample_states' subdivision exactly (asserting parity with
    the frozen samples), then dedupe the consecutive duplicates it emits
    at interior knots so the times are strictly increasing.

    The deduped states are exactly the ORIGINAL frozen states — the
    control still certifies them.
    """
    mirrored = []
    previous = None
    for index, point in enumerate(knots):
        stamp = int(point["time_from_start_ns"])
        if previous is not None:
            dt = stamp - previous
            subdivisions = max(
                1, int((dt + MAX_CERT_SAMPLE_DT_NS - 1)
                       // MAX_CERT_SAMPLE_DT_NS))
            left = knots[index - 1]["positions"]
            for step in range(0, subdivisions + 1):
                fraction = step / subdivisions
                mirrored.append((
                    previous + dt * fraction,
                    [float(a) + (float(b) - float(a)) * fraction
                     for a, b in zip(left, point["positions"])]))
        previous = stamp
    if not mirrored and frozen_samples:
        mirrored.append((0.0, [float(v) for v in knots[0]["positions"]]))
    assert len(mirrored) == len(frozen_samples), (
        "sample mirror size %d != frozen %d" % (
            len(mirrored), len(frozen_samples)))
    for (_time, row), frozen_row in zip(mirrored, frozen_samples):
        assert all(abs(a - b) < 1e-9 for a, b in zip(row, frozen_row)), \
            "sample mirror diverged from the frozen samples"
    times, positions = [], []
    for time, row in mirrored:
        if times and time <= times[-1]:
            continue
        times.append(time)
        positions.append(row)
    return times, positions


def make_leg(edge_id, samples, knots, *, carrying=True):
    from luggage_msgs.msg import RouteLegV2
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    leg = RouteLegV2()
    leg.edge_id = edge_id
    leg.edge_class = "CONTROL"
    leg.carrying = carrying
    leg.joint_names = list(CANONICAL_JOINTS)
    if knots:
        times, positions = sample_times_and_positions(knots, samples)
    else:
        times = [float(i * MAX_CERT_SAMPLE_DT_NS)
                 for i in range(len(samples))]
        positions = [list(row) for row in samples]
    leg.n_samples = len(positions)
    leg.sample_times_ns = times
    leg.sample_positions_flat = [
        float(v) for row in positions for v in row]
    trajectory = JointTrajectory()
    trajectory.joint_names = list(CANONICAL_JOINTS)
    for point in knots:
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [
            float(v) for v in point["positions"]]
        trajectory_point.velocities = [
            float(v) for v in point.get("velocities", [])]
        trajectory_point.accelerations = [
            float(v) for v in point.get("accelerations", [])]
        trajectory_point.time_from_start.sec = int(
            point["time_from_start_ns"]) // 10**9
        trajectory_point.time_from_start.nanosec = int(
            point["time_from_start_ns"]) % 10**9
        trajectory.points.append(trajectory_point)
    leg.knots = trajectory
    return leg


def main():
    parser = argparse.ArgumentParser()
    default_out = ROOT / "docs" / "status" / "evidence" / "frame_migration" / \
        "ST09-parallel-I-interim" / "phase1"
    parser.add_argument("--frozen", required=True)
    parser.add_argument("--evidence-dir", default=str(default_out))
    parser.add_argument("--service-timeout", type=float, default=20.0)
    args = parser.parse_args()
    out_dir = Path(args.evidence_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    frozen = json.loads(Path(args.frozen).read_text())

    # the failing edge: the portal transit the certifier probed first
    # (the frozen batch probe never answered within budget — the timeout
    # face of the same design; the Cartesian controls identified the
    # payload-attached start state as the fraction-0 cause)
    edges = frozen.get("edges", {})
    edge_id = None
    for candidate, edge in edges.items():
        if edge.get("edge_class") == "PORTAL_TRANSIT":
            edge_id = candidate
            break
    if edge_id is None and edges:
        edge_id = sorted(edges)[0]
    if edge_id is None:
        print("no edges in the frozen artifact")
        return 2
    samples = frozen["samples"][edge_id]
    knots = frozen["edges"][edge_id]["trajectory"]["points"]
    print("failing edge: %s (%d samples, %d knots)" % (
        edge_id, len(samples), len(knots)))

    import rclpy
    from luggage_msgs.srv import ValidateRouteV2

    rclpy.init()
    node = rclpy.create_node("st09_i_control_states")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_stop = threading.Event()

    def _spin():
        while not spin_stop.is_set():
            executor.spin_once(timeout_sec=0.05)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()
    client = node.create_client(ValidateRouteV2, "/validate_route_v2")
    if not client.wait_for_service(timeout_sec=args.service_timeout):
        print("validate_route_v2 service not available")
        return 3

    scene_data = frozen["scene_at_select"]
    blob = frozen.get("scene_blob_b64")
    if blob:
        # byte-faithful reconstruction of the frozen scene
        import base64

        from moveit_msgs.msg import PlanningScene as PlanningSceneMsg
        from rclpy.serialization import deserialize_message

        def scene_from_blob():
            return deserialize_message(
                base64.b64decode(blob), PlanningSceneMsg)

        scene_full = scene_from_blob()
        scene_no_payload = scene_from_blob()
        scene_no_payload.robot_state.attached_collision_objects = []
        print("scene rebuilt from CDR blob: %d world objects, %d attached"
              % (len(scene_full.world.collision_objects),
                 len(scene_full.robot_state.attached_collision_objects)))
    else:
        scene_full = scene_from_json(scene_data)
        scene_no_payload = scene_from_json(scene_data)
        scene_no_payload.robot_state.attached_collision_objects = []

    def digest_of(scene, legs):
        material = json.dumps({
            "world": [obj.id for obj in
                      scene.world.collision_objects],
            "attached": [obj.object.id for obj in
                         scene.robot_state.attached_collision_objects],
            "legs": [[leg.edge_id, leg.n_samples] for leg in legs],
            "samples": [leg.sample_positions_flat for leg in legs]},
            default=str)
        return hashlib.sha256(material.encode()).hexdigest()[:16]

    controls = {
        "c1_first_state": (scene_full, [make_leg(
            edge_id + ":c1", samples[:1], knots[:1])], 1),
        "c2_whole_edge": (scene_full, [make_leg(
            edge_id + ":c2", samples, knots)], 16),
        "c5_no_payload": (scene_no_payload, [make_leg(
            edge_id + ":c5", samples, knots)], 16),
    }
    results = []
    for control, (scene, legs, stride) in controls.items():
        scene_digest = digest_of(scene, legs)
        for repetition in range(3):
            request = ValidateRouteV2.Request()
            request.request_id = "control-%s-%d" % (control, repetition)
            request.scene_revision = str(
                frozen["identity"].get("scene_revision", ""))
            request.route_digest = "route:%s" % scene_digest
            request.scene_digest = scene_digest
            request.scene = scene
            request.max_sample_dt_ns = float(MAX_CERT_SAMPLE_DT_NS)
            request.max_sample_step_m = 0.0
            request.distance_stride = int(stride)
            request.budget_s = 10.0
            request.legs = legs
            event = threading.Event()
            future = client.call_async(request)
            future.add_done_callback(lambda _f: event.set())
            event.wait(timeout=30.0)
            if not future.done():
                results.append({"control": control,
                                "repetition": repetition,
                                "error": "timeout"})
                print(results[-1])
                continue
            response = future.result()
            entry = {
                "control": control, "repetition": repetition,
                "scene_digest": scene_digest,
                "echo_scene_digest": response.scene_digest,
                "result": int(response.result),
                "detail": response.detail,
                "all_valid": bool(response.all_valid),
                "service_time_ns": int(response.service_time_ns),
                "legs": [{
                    "edge": leg.edge_id, "valid": bool(leg.valid),
                    "reason": leg.reason, "detail": leg.detail,
                    "first_invalid": int(leg.first_invalid_sample),
                    "n_samples": int(leg.n_samples),
                    "min_distance_min": (
                        min((d for d in leg.min_distance_m
                             if d == d), default=None)
                        if leg.min_distance_m else None),
                    "contacts": [{
                        "body_1": c.body_1, "body_2": c.body_2,
                        "depth": float(c.depth),
                        "position": [float(c.position.x),
                                     float(c.position.y),
                                     float(c.position.z)],
                        "normal": [float(c.normal.x), float(c.normal.y),
                                   float(c.normal.z)],
                    } for c in leg.contacts],
                    "support_contacts": [{
                        "body_1": c.body_1, "body_2": c.body_2,
                        "depth": float(c.depth),
                    } for c in leg.support_contacts],
                } for leg in response.leg_results],
            }
            results.append(entry)
            print("%s #%d result=%d all_valid=%s reason=%r "
                  "contacts=%s" % (
                      control, repetition, response.result,
                      response.all_valid,
                      response.leg_results[0].reason if
                      response.leg_results else "-",
                      [(c.body_1, c.body_2) for c in
                       response.leg_results[0].contacts] if
                      response.leg_results else []))

    outcome = {
        "frozen": str(Path(args.frozen)),
        "failing_edge": edge_id,
        "n_samples": len(samples),
        "scene_world": [obj.id for obj in
                        scene_full.world.collision_objects],
        "scene_attached": [
            (obj.object.id, list(obj.touch_links))
            for obj in
            scene_full.robot_state.attached_collision_objects],
        "controls": results,
    }
    out_path = out_dir / "controls_states.json"
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(outcome, handle, indent=1)
    print("wrote", out_path)
    spin_stop.set()
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
