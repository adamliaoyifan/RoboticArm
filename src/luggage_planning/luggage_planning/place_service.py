"""ROS adapters for SelectPlacePathV2 and BuildMotionSequenceV2.

The numeric conversion lives in ``place_tcp``. This module only moves
fields across the generated messages and calls that helper.
"""

from __future__ import division

import json

from geometry_msgs.msg import Pose
from luggage_msgs.msg import MotionSegment, PoseContractV2
from luggage_msgs.srv import BuildMotionSequenceV2, SelectPlacePathV2

from luggage_planning.place_tcp import (
    build_place_legs_v2,
    container_transform_dict,
    select_place_v2,
)
from luggage_planning.waypoint_generator import build_sequence
from luggage_description.frame_contracts import ContractError


def _xyz(values):
    return [float(values[0]), float(values[1]), float(values[2])]


def _xyzw(orientation):
    return [float(orientation.x), float(orientation.y),
            float(orientation.z), float(orientation.w)]


def _pose_record(message):
    identity = message.identity
    return {
        "contract_version": int(identity.contract_version),
        "frame_id": str(identity.frame_id),
        "length_unit": str(identity.length_unit),
        "stamp_ns": int(identity.stamp_ns),
        "time_basis": str(identity.time_basis),
        "scene_revision": str(identity.scene_revision),
        "request_id": str(identity.request_id),
        "kind": "pose",
        "reference": str(message.reference),
        "orientation_semantics": str(message.orientation_semantics),
        "position": _xyz((message.pose.position.x, message.pose.position.y,
                          message.pose.position.z)),
        "quaternion_xyzw": _xyzw(message.pose.orientation),
    }


def _fill_identity(identity, record):
    identity.contract_version = int(record["contract_version"])
    identity.frame_id = str(record["frame_id"])
    identity.length_unit = str(record["length_unit"])
    identity.stamp_valid = True
    identity.stamp_ns = int(record["stamp_ns"])
    identity.time_basis = str(record["time_basis"])
    identity.scene_revision = str(record["scene_revision"])
    identity.request_id = str(record["request_id"])


def pose_message(record):
    message = PoseContractV2()
    _fill_identity(message.identity, record)
    message.reference = str(record["reference"])
    message.orientation_semantics = str(record["orientation_semantics"])
    message.pose.position.x = float(record["position"][0])
    message.pose.position.y = float(record["position"][1])
    message.pose.position.z = float(record["position"][2])
    quat = record["quaternion_xyzw"]
    message.pose.orientation.x = float(quat[0])
    message.pose.orientation.y = float(quat[1])
    message.pose.orientation.z = float(quat[2])
    message.pose.orientation.w = float(quat[3])
    return message


def _pose_msg(record):
    pose = Pose()
    pose.position.x = float(record["position"][0])
    pose.position.y = float(record["position"][1])
    pose.position.z = float(record["position"][2])
    quat = record["quaternion_xyzw"]
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose


def _surface_record(message, map_revision):
    rows = int(message.rows)
    cols = int(message.columns)
    tops = list(message.surface_z_container)
    states = list(message.state)
    confidence = list(message.confidence)
    grid_z, grid_s, grid_c = [], [], []
    floor = float(message.floor_z_container)
    for ix in range(rows):
        z_row, s_row, c_row = [], [], []
        for iy in range(cols):
            index = ix * cols + iy
            state = str(states[index])
            top = float(tops[index])
            if state == "unknown" and abs(top - floor) <= 1e-9:
                z_row.append(None)
            else:
                z_row.append(top)
            s_row.append(state)
            c_row.append(str(confidence[index]))
        grid_z.append(z_row)
        grid_s.append(s_row)
        grid_c.append(c_row)
    identity = message.identity
    return {
        "kind": "surface",
        "contract_version": int(identity.contract_version),
        "frame_id": str(identity.frame_id or message.identity.frame_id),
        "length_unit": str(identity.length_unit or "m"),
        "stamp_ns": int(identity.stamp_ns),
        "time_basis": str(identity.time_basis or "scene"),
        "scene_revision": str(identity.scene_revision),
        "request_id": str(identity.request_id),
        "height_datum": str(message.height_datum),
        "floor_z_container": floor,
        "ceiling_z_container": float(message.ceiling_z_container),
        "origin_xy_container": [float(message.origin_xy_container[0]),
                                float(message.origin_xy_container[1])],
        "resolution_m": float(message.resolution_m),
        "map_revision": int(map_revision),
        "geometry_hash": "",
        "surface_z_container": grid_z,
        "state": grid_s,
        "confidence": grid_c,
    }


def request_dict(message):
    stamp = None if message.container_stamp_is_null else int(message.container_stamp_ns)
    scene = str(message.slots[0].identity.scene_revision) if message.slots else ""
    transform = container_transform_dict(
        message.container_source_frame, message.container_target_frame,
        message.container_translation, message.container_quaternion_xyzw,
        stamp, scene)
    grasp = _pose_record(message.grasp)
    return {
        "request_id": str(message.request_id),
        "scene_revision": scene,
        "box_wdh": _xyz(message.box_wdh),
        "slots": [_pose_record(slot) for slot in message.slots],
        "slot_scores": [float(v) for v in message.slot_scores],
        "surface": _surface_record(message.surface, message.map_revision),
        "inner_size": _xyz(message.inner_size),
        "geometry_hash": str(message.geometry_hash),
        "map_revision": int(message.map_revision),
        "start_tcp": _pose_record(message.start_tcp),
        "start_joints": [float(v) for v in message.start_joints],
        "grasp": grasp,
        "container_transform": transform,
        "excluded_ids": [str(v) for v in message.excluded_ids],
    }


def _carry_segments(world_poses):
    segments = []
    for name in ("transit", "traverse"):
        segment = MotionSegment()
        segment.name = name
        segment.type = "cartesian"
        segment.allow_ompl_fallback = True
        segment.keep_tool_down = True
        for pose in world_poses[:-1]:
            segment.waypoints.append(_pose_msg(pose))
        if world_poses:
            segment.target_pose = _pose_msg(world_poses[-1])
        segments.append(segment)
    return segments


def _waypoint_messages(result, template):
    messages = []
    for pose in result["world_waypoints"]:
        record = _pose_record(template)
        record["frame_id"] = "world"
        record["reference"] = "suction_contact_frame"
        record["orientation_semantics"] = "tool_quaternion"
        record["time_basis"] = "scene"
        record["position"] = list(pose["position"])
        record["quaternion_xyzw"] = list(pose["quaternion_xyzw"])
        messages.append(pose_message(record))
    return messages


def fill_select_response(message, response, atlas=None, config=None,
                         probe_fn=None, moveit_client=None, counters=None):
    try:
        request = request_dict(message)
    except ContractError as exc:
        response.success = False
        response.reason_code = exc.code
        return response
    request["surface"]["geometry_hash"] = request["geometry_hash"]
    result = select_place_v2(
        request, atlas=atlas, config=config, probe_fn=probe_fn,
        moveit_client=moveit_client, counters=counters)
    response.success = bool(result["success"])
    response.reason_code = str(result["reason_code"])
    response.slot_index = int(result["slot_index"])
    response.selection_id = str(result["selection_id"])
    response.path_source = str(result["path_source"])
    response.path_hash = str(result["path_hash"])
    response.rows_json = json.dumps({"method": result.get("method"),
                                     "path_hash": result.get("path_hash")})
    if result["success"]:
        response.carry_segments = _carry_segments(result["world_waypoints"])
        response.waypoint_poses = _waypoint_messages(result, message.start_tcp)
    return response


def handle_select_v2(request, response, **kwargs):
    return fill_select_response(request, response, **kwargs)


def _copy_segments(segments):
    copied = []
    for seg in segments:
        ros = MotionSegment()
        ros.name = str(seg.name)
        ros.type = str(seg.type)
        ros.keep_tool_down = bool(seg.keep_tool_down)
        ros.keep_camera_down = bool(seg.keep_camera_down)
        ros.allow_ompl_fallback = bool(seg.allow_ompl_fallback)
        ros.lock_wrist = bool(seg.lock_wrist)
        pose = seg.target_pose
        ros.target_pose.position.x = float(pose.position.x)
        ros.target_pose.position.y = float(pose.position.y)
        ros.target_pose.position.z = float(pose.position.z)
        ros.target_pose.orientation.x = float(pose.orientation.x)
        ros.target_pose.orientation.y = float(pose.orientation.y)
        ros.target_pose.orientation.z = float(pose.orientation.z)
        ros.target_pose.orientation.w = float(pose.orientation.w)
        copied.append(ros)
    return copied


def handle_build_v2(request, response, pick=None):
    phase = str(request.phase or "")
    try:
        if phase == "pick":
            if pick is None:
                raise ContractError("MISSING_IDENTITY", "pick", "pick observation", None)
            segments = build_sequence(pick, None, "pick")
            response.segments = _copy_segments(segments)
            response.success = bool(segments)
            response.message = "pick delegated"
            response.reason_code = ""
            return response
        slot = _pose_record(request.slot)
        grasp_record = _pose_record(request.grasp)
        grasp = {
            "source_frame": grasp_record["reference"],
            "target_frame": grasp_record["orientation_semantics"],
            "translation": grasp_record["position"],
            "quaternion_xyzw": grasp_record["quaternion_xyzw"],
            "scene_revision": grasp_record["scene_revision"],
        }
        stamp = None if request.container_stamp_is_null else int(request.container_stamp_ns)
        transform = container_transform_dict(
            request.container_source_frame, request.container_target_frame,
            request.container_translation, request.container_quaternion_xyzw,
            stamp, slot["scene_revision"])
        segments, contact = build_place_legs_v2(
            slot, grasp, transform, _xyz(request.box_wdh))
        response.segments = _copy_segments(segments)
        response.success = bool(segments)
        response.message = "%d place legs" % len(segments)
        response.reason_code = ""
        contact_record = dict(slot)
        contact_record["frame_id"] = "world"
        contact_record["reference"] = "suction_contact_frame"
        contact_record["orientation_semantics"] = "tool_quaternion"
        contact_record["position"] = contact["position"]
        contact_record["quaternion_xyzw"] = contact["quaternion_xyzw"]
        response.contact_pose = pose_message(contact_record)
    except ContractError as exc:
        response.success = False
        response.reason_code = exc.code
        response.message = exc.code
    return response


def empty_select_response():
    return SelectPlacePathV2.Response()


def empty_build_response():
    return BuildMotionSequenceV2.Response()
