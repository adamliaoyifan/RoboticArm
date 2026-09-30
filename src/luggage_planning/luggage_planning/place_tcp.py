"""Shared container-slot and grasp conversion for v2 place paths.

The selector node and the in-process caller both use this module.
Occupancy samples the box-center polyline. Motion waypoints are the TCP
polyline. They differ by the grasp translation rotated by the box quaternion.
"""

from __future__ import division

import math

from luggage_description.container_geometry import cuboid_from_inner_size
from luggage_description.frame_contracts import (
    ContractError,
    finite_number,
    require_horizontal,
    validate_pose,
)
from luggage_description.frame_transforms import (
    box_to_tool_target,
    rotate_vector,
    transform_pose,
)
from luggage_planning.occupancy_place_paths import (
    OccupancyMismatch,
    PLACE_PATH_INFEASIBLE,
    _legacy_view_of_surface_contract,
)
from luggage_planning.pose import MotionSegment, Point, Pose, Quaternion
from luggage_planning.trajectory_selector import TrajectorySelector
from luggage_planning.waypoint_generator import (
    DEFAULT_CORRIDOR_MARGIN,
    DEFAULT_PLACE_CLEARANCE_Z,
    build_sequence,
    corridor_clearance,
    insertion_clearance,
    staging_offset,
)
from luggage_planning.reachability_wavefront import retreat_height


FIXTURE_SELECTOR = {
    "inflate_m": 0.0,
    "arm_radius_m": 0.0,
    "mode": "unified",
    "max_candidates_per_slot": 5,
    "probe_budget": 0,
    "w_arm": 0.0,
}


def _pose_dict(position, quaternion):
    return {
        "position": [float(v) for v in position],
        "quaternion_xyzw": [float(v) for v in quaternion],
    }


def container_transform_dict(source_frame, target_frame, translation,
                             quaternion_xyzw, stamp_ns, scene_revision):
    """Admit a container-to-world transform before any place selection.

    Source is ``container_link`` and target is ``world``. A null stamp is
    the static scene transform. Translation must be three finite metres.
    """
    if not isinstance(source_frame, str) or not source_frame.strip():
        raise ContractError(
            "MISSING_IDENTITY", "container_source_frame", "nonempty", source_frame)
    if not isinstance(target_frame, str) or not target_frame.strip():
        raise ContractError(
            "MISSING_IDENTITY", "container_target_frame", "nonempty", target_frame)
    if source_frame != "container_link" or target_frame != "world":
        raise ContractError(
            "FRAME_CONTRACT_MISMATCH", "container_transform",
            ["container_link", "world"], [source_frame, target_frame])
    if stamp_ns is not None and (type(stamp_ns) is not int or stamp_ns < 0):
        raise ContractError(
            "INVALID_STAMP", "container_stamp_ns",
            "nonnegative integer or explicit null", stamp_ns)
    values = list(translation)
    if len(values) != 3:
        raise ContractError(
            "INVALID_NUMERIC_PAYLOAD", "translation", "3 finite numbers", values)
    admitted = [
        finite_number(float(value), "translation[%d]" % index)
        for index, value in enumerate(values)]
    quaternion_xyzw = [float(v) for v in quaternion_xyzw]
    require_horizontal(quaternion_xyzw)
    return {
        "source_frame": source_frame,
        "target_frame": target_frame,
        "translation": admitted,
        "quaternion_xyzw": quaternion_xyzw,
        "stamp_ns": stamp_ns,
        "scene_revision": scene_revision,
    }


def grasp_dict_from_contract(grasp, scene_revision):
    """PoseContract-shaped grasp: reference is the source frame, semantics the target."""
    if not isinstance(grasp, dict):
        raise ContractError("MISSING_IDENTITY", "grasp", "object", None)
    source = grasp.get("reference")
    target = grasp.get("orientation_semantics")
    if not isinstance(source, str) or not source.strip() or not isinstance(target, str) or not target.strip():
        raise ContractError("MISSING_IDENTITY", "grasp", "source and target frames", grasp)
    pose = grasp.get("position"), grasp.get("quaternion_xyzw")
    return {
        "source_frame": source,
        "target_frame": target,
        "translation": pose[0],
        "quaternion_xyzw": pose[1],
        "scene_revision": grasp.get("scene_revision", scene_revision),
    }


def rotated_grasp_offset(slot, grasp):
    return rotate_vector(slot["quaternion_xyzw"], grasp["translation"])


def world_slot_record(slot, transform):
    return transform_pose(slot, transform)


def world_tcp_record(slot, grasp, transform):
    tcp = box_to_tool_target(slot, grasp)
    return transform_pose(tcp, transform)


def lift_world_point(position, container_quaternion, lift):
    delta = rotate_vector(container_quaternion, (0.0, 0.0, float(lift)))
    return [position[i] + delta[i] for i in range(3)]


def _planning_pose(position, quaternion):
    return Pose(
        position=Point(x=position[0], y=position[1], z=position[2]),
        orientation=Quaternion(
            x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]),
    )


def _segment(name, pose, segment_type="cartesian", allow_ompl=True,
             keep_camera_down=False, truncation=0.0):
    return MotionSegment(
        name=name,
        type=segment_type,
        target_pose=pose,
        keep_tool_down=True,
        keep_camera_down=keep_camera_down,
        allow_ompl_fallback=allow_ompl,
        lock_wrist=False,
        clearance_truncation_m=float(truncation),
    )


def build_place_legs_v2(slot, grasp, transform, box_wdh,
                        place_clearance_z=None, opening_info=None,
                        corridor_surface_max=None,
                        corridor_margin=DEFAULT_CORRIDOR_MARGIN,
                        retreat_safe_tool_z=None):
    """Place legs whose contact pose is the world TCP.

    Vertical offsets are ``rotate_vector(container_quaternion, (0, 0, lift))``
    applied to that TCP. Lengths come from the existing clearance helpers.
    The v1 builder is not used.
    """
    world = world_tcp_record(slot, grasp, transform)
    contact = world["position"]
    quaternion = world["quaternion_xyzw"]
    container_q = transform["quaternion_xyzw"]
    height = float(box_wdh[2])
    if place_clearance_z is None:
        place_clearance_z = DEFAULT_PLACE_CLEARANCE_Z
    clearance = corridor_clearance(
        corridor_surface_max, height, contact[2], place_clearance_z,
        margin=corridor_margin)
    insert_len = insertion_clearance(height, clearance)
    retreat_len = clearance
    truncation = 0.0
    if retreat_safe_tool_z is not None:
        retreat_z, truncation = retreat_height(
            contact[2], clearance, retreat_safe_tool_z)
        retreat_len = float(retreat_z) - float(contact[2])

    def at(length):
        return _planning_pose(
            lift_world_point(contact, container_q, length), quaternion)

    contact_pose = at(0.0)
    above = at(clearance)
    insert_pose = at(insert_len)
    retreat_pose = at(retreat_len)
    transit = above
    segments = []
    if opening_info:
        normal = opening_info["normal"]
        opening = opening_info["point"]
        outward = float(opening_info.get("outward_clearance", 0.15))
        current = [above.position.x, above.position.y, above.position.z]
        signed = sum((current[i] - opening[i]) * normal[i] for i in range(3))
        portal = [current[i] + normal[i] * (outward - signed) for i in range(3)]
        transit = _planning_pose(portal, quaternion)
        stage_distance = float(opening_info.get("stage_outward_clearance", 0.65))
        offset = staging_offset(normal, stage_distance)
        stage = _planning_pose(
            [portal[0] + offset[0], portal[1] + offset[1], portal[2] + offset[2]],
            quaternion)
        segments.append(_segment("stage", stage))
    segments.extend([
        _segment("transit", transit, segment_type="pose_target", allow_ompl=False),
        _segment("traverse", above),
        _segment("insert", insert_pose, allow_ompl=False, keep_camera_down=True),
        _segment("descend", contact_pose, allow_ompl=False, keep_camera_down=True),
        _segment("retreat", retreat_pose, allow_ompl=False, keep_camera_down=True,
                 truncation=truncation),
    ])
    return segments, _pose_dict(contact, quaternion)


def build_motion_sequence_v2(phase, slot, grasp, transform, box_wdh, pick=None,
                             **place_kwargs):
    if phase == "pick":
        if pick is None:
            raise ContractError("MISSING_IDENTITY", "pick", "pick observation", None)
        return build_sequence(pick, None, "pick"), None
    if phase != "place":
        raise ContractError("INVALID_RECORD", "phase", "pick or place", phase)
    return build_place_legs_v2(
        slot, grasp, transform, box_wdh, **place_kwargs)


def _fail(code, counters):
    return {
        "success": False,
        "reason_code": code,
        "slot_index": -1,
        "selection_id": "",
        "path_source": "",
        "path_hash": "",
        "method": "",
        "world_waypoints": [],
        "box_center_waypoints": [],
        "rows": [],
        "probe_calls": counters.get("probe", 0),
        "moveit_calls": counters.get("moveit", 0),
        "sweep_mutations": counters.get("sweep_mutation", 0),
        "commit_calls": counters.get("commit", 0),
    }


def _admit_slot(slot, scene_revision):
    validate_pose(slot, "slot", scene_revision)


def admit_select_request(request):
    """Validate identity before any sweep, probe, or commit."""
    if not isinstance(request, dict):
        raise ContractError("INVALID_RECORD", "request", "object", request)
    scene = request.get("scene_revision")
    if not isinstance(scene, str) or not scene.strip():
        raise ContractError("MISSING_IDENTITY", "scene_revision", "nonempty", scene)
    slots = request.get("slots")
    if not isinstance(slots, list) or not slots:
        raise ContractError("MISSING_IDENTITY", "slots", "nonempty", slots)
    for slot in slots:
        _admit_slot(slot, scene)
    start = request.get("start_tcp")
    validate_pose(start, "map_tcp", scene)
    grasp = grasp_dict_from_contract(request.get("grasp"), scene)
    transform = request.get("container_transform")
    if not isinstance(transform, dict):
        raise ContractError("MISSING_IDENTITY", "container_transform", "object", transform)
    container_transform_dict(
        transform.get("source_frame"), transform.get("target_frame"),
        transform.get("translation"), transform.get("quaternion_xyzw"),
        transform.get("stamp_ns"), scene)
    box = request.get("box_wdh")
    if not isinstance(box, (list, tuple)) or len(box) != 3:
        raise ContractError("INVALID_NUMERIC_PAYLOAD", "box_wdh", "3 finite numbers", box)
    for value in box:
        if type(value) not in (int, float) or not math.isfinite(float(value)):
            raise ContractError("INVALID_NUMERIC_PAYLOAD", "box_wdh", "finite", value)
    return grasp


def _yaw_of(slot):
    quat = slot["quaternion_xyzw"]
    return 2.0 * math.atan2(float(quat[2]), float(quat[3]))


def _world_waypoints(tcp_record, container_points, transform):
    poses = []
    for point in container_points:
        record = dict(tcp_record)
        record["position"] = [float(v) for v in point]
        world = transform_pose(record, transform)
        poses.append(_pose_dict(world["position"], world["quaternion_xyzw"]))
    return poses


def select_place_v2(request, atlas=None, config=None, probe_fn=None,
                    moveit_client=None, counters=None):
    """Select one place path. Invalid requests return before any side effect.

    ``probe_fn`` and ``moveit_client`` are accepted so callers can prove they
    stay unused. Occupancy selection does not spend a cartesian probe.
    """
    del probe_fn, moveit_client
    counters = counters if counters is not None else {
        "probe": 0, "moveit": 0, "sweep_mutation": 0, "commit": 0}
    try:
        grasp = admit_select_request(request)
    except ContractError as exc:
        return _fail(exc.code, counters)
    scene = request["scene_revision"]
    transform = dict(request["container_transform"])
    transform["scene_revision"] = scene
    surface = request.get("surface")
    try:
        inner = [float(v) for v in request["inner_size"]]
        if isinstance(surface, dict) and surface.get("kind") == "surface":
            floor = float(surface["floor_z_container"])
            hull = cuboid_from_inner_size(inner, floor_z=floor)
            surface = _legacy_view_of_surface_contract(
                surface, inner, hull.descriptor())
        selector = TrajectorySelector(config or dict(FIXTURE_SELECTOR))
        slots = []
        scores = list(request.get("slot_scores") or [])
        for index, slot in enumerate(request["slots"]):
            tcp = box_to_tool_target(slot, grasp)
            slots.append({
                "index": index,
                "score": float(scores[index]) if index < len(scores) else 0.0,
                "target": tuple(tcp["position"]),
                "yaw": _yaw_of(slot),
                "quaternion_xyzw": list(slot["quaternion_xyzw"]),
                "grasp_translation": list(grasp["translation"]),
            })
        start = tuple(request["start_tcp"]["position"])
        winner, rows, reason, _snapshot = selector.select(
            surface, slots, start, request["box_wdh"],
            atlas=atlas,
            expected_hash=request.get("geometry_hash"),
            expected_revision=request.get("map_revision"),
            start_joints=request.get("start_joints"),
            excluded_ids=request.get("excluded_ids"),
            probe_fn=None)
    except OccupancyMismatch as exc:
        text = str(exc)
        if "map_revision" in text:
            return _fail("MAP_REVISION_MISMATCH", counters)
        return _fail(text, counters)
    except ContractError as exc:
        return _fail(exc.code, counters)
    if winner is None:
        failed = _fail(reason or PLACE_PATH_INFEASIBLE, counters)
        failed["rows"] = rows
        return failed
    try:
        tcp_record = box_to_tool_target(
            request["slots"][int(winner["slot_index"])], grasp)
        world_points = _world_waypoints(
            tcp_record, winner.get("waypoints") or [], transform)
    except ContractError as exc:
        failed = _fail(exc.code, counters)
        failed["rows"] = rows
        return failed
    return {
        "success": True,
        "reason_code": "",
        "slot_index": int(winner["slot_index"]),
        "selection_id": str(winner.get("selection_id") or ""),
        "path_source": str(winner.get("path_source") or ""),
        "path_hash": str(winner.get("path_hash") or ""),
        "method": str(winner.get("method") or ""),
        "world_waypoints": world_points,
        "box_center_waypoints": list(winner.get("box_center_waypoints") or []),
        "container_waypoints": list(winner.get("waypoints") or []),
        "rows": rows,
        "probe_calls": counters["probe"],
        "moveit_calls": counters["moveit"],
        "sweep_mutations": counters["sweep_mutation"],
        "commit_calls": counters["commit"],
        "yaw": float(winner.get("yaw") or 0.0),
    }
