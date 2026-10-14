#!/usr/bin/env python3
"""B3 gate: production ValidateRouteV2 with an obstacle inserted, then removed.

Base: the G2 final_run01 captured certification request (a certified,
contact-free route). Three calls against the isolated production MoveIt
stack, same robot/payload/ACM throughout:

  exact      captured request unchanged          -> all_valid (control)
  obstacle   same request + one world box        -> COLLISION naming the box
  removed    captured request unchanged, again   -> all_valid (state restored)

The obstacle spans the container opening plane (scene_tf_deploy_update
aperture), so every PORTAL_TRANSIT/INSERT/DESCEND leg must cross it.

usage: b3_obstacle_validation.py <out_dir>
env: G2_EVID must be writable (moveit_usage.jsonl is appended there).
"""
import base64
import copy
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, "/tmp/elfin-master-pack-baseline-g3/scripts/g2")
import g2_common as common  # noqa: E402

G2_FINAL01 = ("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
              "master_pack_recovery/2026-10-13_g2/final_run01/cert_capture/"
              "cert_request.json")
OBSTACLE_ID = "b3_placed_box"
# Container opening (negative_x face): x=-0.745, y in [-0.928, 0.398],
# z in [0.597, 2.009]. A 0.12 m thick slab across the aperture.
OBSTACLE_CENTER = (-0.745, (-0.928 + 0.398) / 2.0, (0.597 + 2.009) / 2.0)
OBSTACLE_SIZE = (0.12, 1.33, 1.41)


def scene_inventory(scene):
    objs = sorted(o.id for o in scene.world.collision_objects)
    attached = sorted(
        a.object.id for a in getattr(
            scene.robot_state, "attached_collision_objects", []))
    return {"world_objects": objs, "attached": attached}


def add_obstacle(request):
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject

    req = copy.deepcopy(request)
    for scene in req.load_state_scenes:
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = OBSTACLE_ID
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = OBSTACLE_CENTER
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(OBSTACLE_SIZE)
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        scene.world.collision_objects.append(obj)
    return req


def main():
    out_dir = sys.argv[1]
    os.makedirs(out_dir, exist_ok=True)
    import rclpy
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message
    from luggage_msgs.srv import ValidateRouteV2
    from luggage_planning.moveit_state_validity import _scene_digest

    document = json.load(open(G2_FINAL01))
    request = deserialize_message(
        base64.b64decode(document["request_b64"]), ValidateRouteV2.Request)

    stack = common.MoveItStack(out_dir, "b3_obstacle")
    stack.start()
    rclpy.init()
    node = Node("g3_b3_obstacle")
    stop, thread = common.score._spin(node)  # noqa: SLF001
    report = {
        "source": G2_FINAL01,
        "request_id": request.request_id,
        "captured_scene_digest": document.get("scene_digest"),
        "obstacle": {"id": OBSTACLE_ID, "center": OBSTACLE_CENTER,
                     "size": OBSTACLE_SIZE},
        "variants": {}, "scenes": {},
    }
    try:
        client = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not client.wait_for_service(timeout_sec=120.0):
            raise RuntimeError("validate_route_v2 unavailable")

        def call(req, label):
            future = client.call_async(req)
            end = time.time() + 120
            while not future.done() and time.time() < end:
                time.sleep(0.05)
            response = future.result()
            entry = {"result": int(response.result), "detail": str(response.detail),
                     "all_valid": bool(response.all_valid),
                     "model_sha256": str(response.model_sha256),
                     "scene_digest": str(response.scene_digest),
                     "legs": [{"edge": str(l.edge_id), "valid": bool(l.valid),
                               "reason": str(l.reason),
                               "first_invalid": int(l.first_invalid_sample),
                               "n_samples": int(l.n_samples),
                               "contacts": [{"b1": str(c.body_1), "b2": str(c.body_2),
                                             "depth": float(c.depth)}
                                            for c in getattr(l, "contacts", [])][:6]}
                              for l in response.leg_results]}
            report["variants"][label] = entry
            report["scenes"][label] = [scene_inventory(s)
                                       for s in req.load_state_scenes]
            return entry

        exact = call(request, "exact")
        obstacle_req = add_obstacle(request)
        obstacle_req.scene_digest = _scene_digest(obstacle_req.load_state_scenes)
        obstacle_req.route_digest = obstacle_req.scene_digest
        obstacle = call(obstacle_req, "obstacle")
        removed = call(request, "removed")

        obstacle_contacts = sorted({
            name
            for leg in obstacle["legs"] if not leg["valid"]
            for contact in leg["contacts"]
            for name in (contact["b1"], contact["b2"])
            if OBSTACLE_ID in name})
        invalid_legs = [leg["edge"] for leg in obstacle["legs"] if not leg["valid"]]
        checks = {
            "exact_control_valid": exact["all_valid"],
            "obstacle_not_all_valid": not obstacle["all_valid"],
            "obstacle_collision_named": OBSTACLE_ID in obstacle_contacts,
            "invalid_legs_crossing_opening": bool(invalid_legs),
            "removed_valid_again": removed["all_valid"],
            "scenes_differ_only_by_obstacle": all(
                set(inv["world_objects"]) - {OBSTACLE_ID} ==
                set(rep["world_objects"])
                for inv, rep in zip(report["scenes"]["obstacle"],
                                    report["scenes"]["exact"]))
            and report["scenes"]["exact"] == report["scenes"]["removed"],
            "same_model": (exact["model_sha256"] == obstacle["model_sha256"]
                           == removed["model_sha256"]),
        }
        report["obstacle_contact_names"] = obstacle_contacts
        report["invalid_legs"] = invalid_legs
        report["checks"] = checks
        report["gate_pass"] = all(checks.values())
        with open(os.path.join(out_dir, "b3_report.json"), "w") as handle:
            json.dump(report, handle, indent=1, default=str)
            handle.write("\n")
        print(json.dumps(checks))
        return 0 if report["gate_pass"] else 1
    finally:
        stop.set()
        thread.join(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown()
        stack.stop()


if __name__ == "__main__":
    sys.exit(main())
