#!/usr/bin/env python3
"""G2-0: audit the frozen G1 capture inputs (hash, size, semantics, identity)."""
import hashlib
import json
import math
import os
import sys

from rclpy.serialization import deserialize_message, serialize_message
from luggage_msgs.srv import SelectPlacePathV2
from moveit_msgs.msg import PlanningScene
from luggage_planning.planning_contract.attachment import invert_transform

CAP = os.environ["G1_CAPTURE"]
EXPECT_SHA = "33de07dd031b33bb2ec1ffaf36afbeaffe9d1ce40e08ebfa57e6c9dd0cf3e220"
EXPECT_SIZE = 222500


def quat_angle(a, b):
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return 2.0 * math.acos(min(1.0, dot))


def main(out_path):
    audit = {"checks": [], "failures": []}

    def check(name, ok, **detail):
        audit["checks"].append({"check": name, "ok": bool(ok), **detail})
        if not ok:
            audit["failures"].append(name)

    raw = open(os.path.join(CAP, "select_capture/select_place_path_v2_00.bin"), "rb").read()
    sha = hashlib.sha256(raw).hexdigest()
    index = json.load(open(os.path.join(CAP, "select_capture/select_place_path_v2_00.json")))
    check("request_size", len(raw) == EXPECT_SIZE, size=len(raw))
    check("request_sha256", sha == EXPECT_SHA == index["sha256"], sha=sha)
    request = deserialize_message(raw, SelectPlacePathV2.Request)
    again = deserialize_message(bytes(serialize_message(request)), SelectPlacePathV2.Request)
    check("semantic_round_trip", again == request)
    check("n_slots_828", len(request.slots) == 828 and len(request.slot_scores) in (0, 828),
          n_slots=len(request.slots), n_scores=len(request.slot_scores))
    audit["request"] = {
        "request_id": request.request_id, "box_id": request.box_id,
        "generation": int(request.generation), "geometry_hash": request.geometry_hash,
        "map_revision": int(request.map_revision),
        "box_wdh": [float(v) for v in request.box_wdh],
        "start_joints": [float(v) for v in request.start_joints],
        "grasp_scene_revision": request.grasp.identity.scene_revision,
        "grasp_stamp_ns": int(request.grasp.identity.stamp_ns),
        "container_stamp_ns": int(request.container_stamp_ns),
        "container_stamp_is_null": bool(request.container_stamp_is_null),
    }
    grasp_pos = request.grasp.pose.position
    grasp_q = request.grasp.pose.orientation
    wdh = [float(v) for v in request.box_wdh]
    portals = []
    for i in range(4):
        rec = json.load(open(os.path.join(CAP, "portal_capture/portal_%02d.json" % i)))
        scene = deserialize_message(
            open(os.path.join(CAP, "portal_capture", rec["scene_bin"]), "rb").read(), PlanningScene)
        row = {"portal": i}
        start_err = max(abs(a - b) for a, b in zip(rec["start_q"], request.start_joints))
        check("portal%d_start_equals_request" % i, start_err <= 1e-3, max_abs_err=start_err,
              note="request start_joints is 3-decimal rounded in the frozen message")
        attached = list(scene.robot_state.attached_collision_objects)
        check("portal%d_one_attached" % i, len(attached) == 1)
        obj = attached[0].object
        dims = [float(v) for v in obj.primitives[0].dimensions]
        dim_err = max(abs(a - b) for a, b in zip(dims, wdh))
        check("portal%d_payload_dims_equal_request" % i, dim_err <= 1e-9, max_abs_err=dim_err,
              attached_dims=dims, request_dims=wdh)
        p = obj.pose.position
        # Production attach: T_tcp_box = invert_transform(measured grasp)
        # (route_producer.register_attach_from_request).
        inv_p, inv_q = invert_transform(
            (grasp_pos.x, grasp_pos.y, grasp_pos.z),
            (grasp_q.x, grasp_q.y, grasp_q.z, grasp_q.w))
        pos_err = max(abs(p.x - inv_p[0]), abs(p.y - inv_p[1]), abs(p.z - inv_p[2]))
        q_err = quat_angle([obj.pose.orientation.x, obj.pose.orientation.y,
                            obj.pose.orientation.z, obj.pose.orientation.w], list(inv_q))
        raw_pos_err = max(abs(p.x - grasp_pos.x), abs(p.y - grasp_pos.y), abs(p.z - grasp_pos.z))
        check("portal%d_attach_pose_equals_inverse_grasp" % i, pos_err <= 1e-9 and q_err <= 1e-8,
              pos_err_m=pos_err, quat_angle_rad=q_err, raw_vs_grasp_pos_err_m=raw_pos_err,
              link=attached[0].link_name, frame=obj.header.frame_id,
              touch_links=list(attached[0].touch_links))
        row.update({"dims": dims, "attach_link": attached[0].link_name,
                    "world_objects": [w.id for w in scene.world.collision_objects]})
        portals.append(row)
    audit["portals"] = portals
    audit["half_height_vs_offset"] = {
        "box_h_over_2": wdh[2] / 2.0, "grasp_z": grasp_pos.z,
        "diff_m": grasp_pos.z - wdh[2] / 2.0}
    audit["verdict"] = "pass" if not audit["failures"] else "fail"
    json.dump(audit, open(out_path, "w"), indent=1, default=str)
    print(audit["verdict"], audit["failures"])
    return 0 if audit["verdict"] == "pass" else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
