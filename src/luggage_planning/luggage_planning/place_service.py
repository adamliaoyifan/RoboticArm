"""ROS adapters for SelectPlacePathV2 and BuildMotionSequenceV2.

Numeric conversion lives in ``place_tcp``. This module decodes generated
messages through the v2 bindings before that helper runs. Producer map
identity stays on the surface; the request hash and revision are only the
consumer expectation.
"""

from __future__ import division

import json

from geometry_msgs.msg import Pose
from luggage_msgs.msg import MotionSegment, PoseContractV2
from luggage_msgs.srv import BuildMotionSequenceV2, SelectPlacePathV2

from luggage_description.frame_contracts import (
    ContractError,
    finite_number,
    require_equal,
    unit_quaternion,
    validate_envelope,
)
from luggage_description.frame_diagnostics import BoundaryContext
from luggage_description.ros_frame_bindings.identity import identity_from_message
from luggage_description.ros_frame_bindings.poses import pose_from_message
from luggage_description.ros_frame_bindings.surfaces import surface_from_message
from luggage_planning.occupancy_place_paths import dump_rows
from luggage_planning.place_tcp import (
    build_place_legs_v2,
    container_transform_dict,
    grasp_dict_from_contract,
    select_place_v2,
)
from luggage_planning.waypoint_generator import build_sequence


def boundary_context(request_id, scene_revision, frame_id, reference,
                     geometry_hash="", map_revision=None, interface_id="F13"):
    """Consumer expectation. Callers pass request fields, not surface fields."""
    return BoundaryContext(
        interface_id=interface_id,
        producer="place_service",
        consumer="trajectory_selector",
        request_id=str(request_id),
        scene_revision=str(scene_revision),
        frame_id=str(frame_id),
        reference=str(reference or ""),
        time_basis="scene",
        geometry_hash=str(geometry_hash or ""),
        map_revision=map_revision,
    )


def _xyz(values):
    return [float(values[0]), float(values[1]), float(values[2])]


def _require_box_wdh(values):
    """Three finite, positive payload dimensions. NaN must not build legs."""
    numbers = list(values)
    if len(numbers) != 3:
        raise ContractError(
            "INVALID_NUMERIC_PAYLOAD", "box_wdh", "3 positive finite numbers",
            numbers)
    admitted = []
    for index, value in enumerate(numbers):
        number = finite_number(float(value), "box_wdh[%d]" % index)
        if number <= 0.0:
            raise ContractError(
                "INVALID_NUMERIC_PAYLOAD", "box_wdh[%d]" % index,
                "positive", number)
        admitted.append(number)
    return admitted


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


def _require_request_id(request_id):
    if not isinstance(request_id, str) or not request_id.strip():
        raise ContractError(
            "MISSING_IDENTITY", "request_id", "nonempty", request_id)
    return request_id


def _admit_grasp(message, request_id, scene_revision):
    """Grasp is a box-frame offset, not a pose profile.

    Identity still has to be v2: version, stamp, unit, and scene. The
    reference is the source frame and orientation_semantics is the target.
    """
    identity = identity_from_message(message.identity)
    validate_envelope(identity, scene_revision)
    require_equal(identity, "request_id", request_id, "REQUEST_IDENTITY_MISMATCH")
    position = [
        finite_number(float(getattr(message.pose.position, name)), "grasp.position")
        for name in ("x", "y", "z")]
    quaternion = [
        float(getattr(message.pose.orientation, name))
        for name in ("x", "y", "z", "w")]
    unit_quaternion(quaternion, "grasp.quaternion_xyzw")
    return {
        "reference": str(message.reference),
        "orientation_semantics": str(message.orientation_semantics),
        "position": position,
        "quaternion_xyzw": quaternion,
        "scene_revision": identity["scene_revision"],
        "contract_version": identity["contract_version"],
        "request_id": identity["request_id"],
        "frame_id": identity["frame_id"],
        "stamp_ns": identity["stamp_ns"],
    }


def _admit_slot(message, request_id, scene_revision, interface_id):
    context = boundary_context(
        request_id, scene_revision, "container_link", "box_center",
        interface_id=interface_id)
    return pose_from_message(message, "slot", context)


def request_dict(message):
    if not message.slots:
        raise ContractError("MISSING_IDENTITY", "slots", "nonempty", [])
    request_id = _require_request_id(str(message.request_id))
    scene = str(message.slots[0].identity.scene_revision)
    slots = [
        _admit_slot(slot, request_id, scene, "F13") for slot in message.slots]
    start = pose_from_message(
        message.start_tcp, "map_tcp",
        boundary_context(
            request_id, scene, "container_link", "suction_contact_frame",
            interface_id="F13"))
    grasp = _admit_grasp(message.grasp, request_id, scene)
    # The latched map correlates as map-rev-<revision>, not as this
    # select request. Scene, hull, and revision still have to match.
    surface_request = str(message.surface.identity.request_id or "")
    surface = surface_from_message(
        message.surface,
        boundary_context(
            surface_request, scene, "container_link", "",
            geometry_hash=str(message.geometry_hash),
            map_revision=int(message.map_revision),
            interface_id="F13"))
    stamp = None if message.container_stamp_is_null else int(
        message.container_stamp_ns)
    transform = container_transform_dict(
        message.container_source_frame, message.container_target_frame,
        message.container_translation, message.container_quaternion_xyzw,
        stamp, scene)
    return {
        "request_id": request_id,
        "scene_revision": scene,
        "box_wdh": _xyz(message.box_wdh),
        "slots": slots,
        "slot_scores": [float(v) for v in message.slot_scores],
        "surface": surface,
        "inner_size": _xyz(message.inner_size),
        "geometry_hash": str(message.geometry_hash),
        "map_revision": int(message.map_revision),
        "start_tcp": start,
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


def _typed_motion_segments(world_poses, identity):
    """MotionSegmentV2 cartesian legs. Existing MotionSegment fields stay too.
    """
    from luggage_description.frame_diagnostics import BoundaryContext
    from luggage_planning.motion_frame_adapters import message_from_admitted
    from luggage_planning.motion_frame_contracts import TARGET_LINK, ExecutionContext
    messages = []
    if not world_poses:
        return messages
    boundary = BoundaryContext(
        interface_id="F15",
        producer="place_service",
        consumer="motion_executor",
        request_id=str(identity["request_id"]),
        scene_revision=str(identity["scene_revision"]),
        frame_id="world",
        reference="suction_contact_frame",
        time_basis=str(identity.get("time_basis") or "scene"),
    )
    context = ExecutionContext(boundary, TARGET_LINK, "world")
    for name in ("transit", "traverse"):
        target = dict(identity)
        target.update(
            kind="pose", frame_id="world", reference="suction_contact_frame",
            orientation_semantics="tool_quaternion", length_unit="m",
            position=list(world_poses[-1]["position"]),
            quaternion_xyzw=list(world_poses[-1]["quaternion_xyzw"]))
        waypoints = []
        for pose in world_poses[:-1]:
            point = dict(target)
            point["position"] = list(pose["position"])
            point["quaternion_xyzw"] = list(pose["quaternion_xyzw"])
            waypoints.append(point)
        record = {
            "contract_version": 2,
            "request_id": boundary.request_id,
            "scene_revision": boundary.scene_revision,
            "stamp_ns": int(identity["stamp_ns"]),
            "time_basis": boundary.time_basis,
            "name": name,
            "segment_type": "cartesian",
            "target_link": TARGET_LINK,
            "policy_valid": True,
            "keep_tool_down": True,
            "keep_camera_down": False,
            "lock_wrist": False,
            "allow_ompl_fallback": True,
            "required_cartesian_fraction": 0.0,
            "target_pose": target,
            "waypoints": waypoints,
            "joint_trajectory": None,
        }
        messages.append(message_from_admitted(record, context))
    return messages


def _waypoint_messages(result, start_record):
    messages = []
    for pose in result["world_waypoints"]:
        record = dict(start_record)
        record["frame_id"] = "world"
        record["reference"] = "suction_contact_frame"
        record["orientation_semantics"] = "tool_quaternion"
        record["time_basis"] = "scene"
        record["position"] = list(pose["position"])
        record["quaternion_xyzw"] = list(pose["quaternion_xyzw"])
        messages.append(pose_message(record))
    return messages


def _reject(response, exc):
    response.success = False
    response.reason_code = exc.code
    response.slot_index = -1
    response.selection_id = ""
    response.path_source = ""
    response.path_hash = ""
    response.rows_json = "[]"
    return response


def fill_select_response(message, response, atlas=None, config=None,
                         probe_fn=None, moveit_client=None, counters=None,
                         geometry_descriptor=None):
    try:
        request = request_dict(message)
        result = select_place_v2(
            request, atlas=atlas, config=config, probe_fn=probe_fn,
            moveit_client=moveit_client, counters=counters,
            geometry_descriptor=geometry_descriptor)
    except ContractError as exc:
        return _reject(response, exc)
    response.success = bool(result["success"])
    response.reason_code = str(result["reason_code"])
    response.slot_index = int(result["slot_index"])
    response.selection_id = str(result["selection_id"])
    response.path_source = str(result["path_source"])
    response.path_hash = str(result["path_hash"])
    response.rows_json = json.dumps(dump_rows(result.get("rows") or []))
    if result["success"]:
        response.carry_segments = _carry_segments(result["world_waypoints"])
        response.waypoint_poses = _waypoint_messages(
            result, request["start_tcp"])
        response.motion_segments = _typed_motion_segments(
            result["world_waypoints"], request["start_tcp"])
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


def _build_motion_messages(segments, identity):
    """Encode each generated leg, preserving its target, waypoints and policy."""
    from luggage_planning.motion_frame_adapters import message_from_admitted
    from luggage_planning.motion_frame_contracts import TARGET_LINK, ExecutionContext
    boundary = BoundaryContext(
        interface_id="F15", producer="waypoint_generator", consumer="motion_executor",
        request_id=identity["request_id"], scene_revision=identity["scene_revision"],
        frame_id="world", reference="suction_contact_frame", time_basis=identity["time_basis"])
    context = ExecutionContext(boundary, TARGET_LINK, "world")

    def pose_record(pose):
        return dict(contract_version=2, kind="pose", frame_id="world", length_unit="m",
                    request_id=identity["request_id"], scene_revision=identity["scene_revision"],
                    stamp_ns=identity["stamp_ns"], time_basis=identity["time_basis"],
                    reference="suction_contact_frame", orientation_semantics="tool_quaternion",
                    position=[pose.position.x, pose.position.y, pose.position.z],
                    quaternion_xyzw=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    return [message_from_admitted(dict(
        contract_version=2, request_id=identity["request_id"], scene_revision=identity["scene_revision"],
        stamp_ns=identity["stamp_ns"], time_basis=identity["time_basis"], name=segment.name,
        segment_type=segment.type, target_link=TARGET_LINK, policy_valid=True,
        keep_tool_down=segment.keep_tool_down, keep_camera_down=segment.keep_camera_down,
        lock_wrist=segment.lock_wrist, allow_ompl_fallback=segment.allow_ompl_fallback,
        required_cartesian_fraction=segment.required_cartesian_fraction,
        target_pose=pose_record(segment.target_pose),
        waypoints=[pose_record(pose) for pose in segment.waypoints], joint_trajectory=None), context)
        for segment in segments]


def handle_build_v2(request, response, pick=None):
    phase = str(request.phase or "")
    try:
        if phase == "pick":
            if pick is None:
                raise ContractError(
                    "MISSING_IDENTITY", "pick", "pick observation", None)
            _require_request_id(str(request.request_id))
            if not str(request.scene_revision).strip():
                raise ContractError(
                    "MISSING_IDENTITY", "scene_revision", "nonempty", request.scene_revision)
            if int(request.pick_pose.identity.contract_version) == 0:
                raise ContractError(
                    "MISSING_IDENTITY", "pick_pose", "detection pose", None)
            context = BoundaryContext(
                interface_id="F11", producer="driver", consumer="waypoint_generator",
                request_id=str(request.request_id), scene_revision=str(request.scene_revision),
                frame_id="world", reference="box_center", time_basis="acquisition")
            pick_record = pose_from_message(request.pick_pose, "detection", context)
            from luggage_planning.attachment_frame_adapters import measured_dict
            from luggage_planning.attachment_frame_contracts import require_measured_pose_agreement
            require_measured_pose_agreement(pick_record, measured_dict(request.pick, request.pick.id))
            segments = build_sequence(pick, None, "pick")
            response.motion_segments = _build_motion_messages(segments, pick_record)
            response.segments = _copy_segments(segments)
            response.success = bool(segments)
            response.message = "pick delegated"
            response.reason_code = ""
            return response
        if int(request.pick_pose.identity.contract_version) != 0 or str(
                request.pick_pose.identity.frame_id):
            raise ContractError(
                "INVALID_RECORD", "pick_pose", "inactive on place", request.phase)
        if phase != "place":
            raise ContractError("INVALID_RECORD", "phase", "pick or place", phase)
        scene = str(request.slot.identity.scene_revision)
        request_id = _require_request_id(str(request.slot.identity.request_id))
        slot = _admit_slot(request.slot, request_id, scene, "F11")
        grasp = grasp_dict_from_contract(
            _admit_grasp(request.grasp, request_id, scene), scene)
        stamp = None if request.container_stamp_is_null else int(
            request.container_stamp_ns)
        transform = container_transform_dict(
            request.container_source_frame, request.container_target_frame,
            request.container_translation, request.container_quaternion_xyzw,
            stamp, scene)
        segments, contact = build_place_legs_v2(
            slot, grasp, transform, _require_box_wdh(request.box_wdh))
        response.motion_segments = _build_motion_messages(segments, slot)
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
        response.motion_segments = []
        response.segments = []
        response.success = False
        response.reason_code = exc.code
        response.message = exc.code
    return response


def empty_select_response():
    return SelectPlacePathV2.Response()


def empty_build_response():
    return BuildMotionSequenceV2.Response()
