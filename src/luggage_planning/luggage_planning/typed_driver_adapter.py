"""Shared typed-v2 client adapter for eval and hardware drivers (ST09 F22).

Every driver calls the production v2 endpoints through this one adapter:
typed request records, request-id correlation, the real service path, and
graph readiness required before each call. Ground truth stays a separate
oracle — nothing here reads physics state. The identity is ADOPTED from a
healthy stack heartbeat, never declared by the driver.
"""
import math

from luggage_description.frame_contracts import ContractError
from luggage_planning.place_service import pose_message


# --------------------------------------------------------------------------
# Typed request records (canonical shapes; the same ones the services admit)

def slot_record(request_id, scene_revision, center, yaw):
    """Slot profile: container_link box center with box_axis_pi yaw."""
    return {
        "contract_version": 2,
        "frame_id": "container_link",
        "length_unit": "m",
        "stamp_ns": 0,
        "time_basis": "scene",
        "scene_revision": str(scene_revision),
        "request_id": str(request_id),
        "kind": "pose",
        "reference": "box_center",
        "orientation_semantics": "box_axis_pi",
        "position": [float(center[0]), float(center[1]), float(center[2])],
        "quaternion_xyzw": [0.0, 0.0,
                            math.sin(0.5 * float(yaw)), math.cos(0.5 * float(yaw))],
    }


def grasp_record(request_id, scene_revision, translation, quaternion_xyzw,
                 stamp_ns=0):
    """Grasp transport: suction_contact_frame -> box offset of the grasp.

    ``stamp_ns`` is the attach acquisition stamp. Zero is not a live
    measurement; the selector refuses it instead of ageing it from epoch.
    """
    return {
        "reference": "suction_contact_frame",
        "orientation_semantics": "box",
        "position": [float(v) for v in translation],
        "quaternion_xyzw": [float(v) for v in quaternion_xyzw],
        "scene_revision": str(scene_revision),
        "contract_version": 2,
        "request_id": str(request_id),
        "frame_id": "suction_contact_frame",
        "length_unit": "m",
        "stamp_ns": int(stamp_ns),
        "time_basis": "scene",
    }


def tcp_record(request_id, scene_revision, position, quaternion_xyzw,
               time_basis="scene"):
    """map_tcp profile: suction contact in container_link."""
    return {
        "contract_version": 2,
        "frame_id": "container_link",
        "length_unit": "m",
        "stamp_ns": 0,
        "time_basis": str(time_basis),
        "scene_revision": str(scene_revision),
        "request_id": str(request_id),
        "kind": "pose",
        "reference": "suction_contact_frame",
        "orientation_semantics": "tool_quaternion",
        "position": [float(v) for v in position],
        "quaternion_xyzw": [float(v) for v in quaternion_xyzw],
    }


def detection_record_from_box(box_message, request_id, scene_revision):
    """Detection-profile record built FROM the measured box message.

    Position, orientation and stamp are the box's own fields, so the
    service-side measured-pose agreement check passes by construction.
    """
    stamp = box_message.header.stamp
    return {
        "contract_version": 2,
        "frame_id": str(box_message.header.frame_id or "world"),
        "length_unit": "m",
        "stamp_ns": int(stamp.sec) * 10 ** 9 + int(stamp.nanosec),
        "time_basis": "acquisition",
        "scene_revision": str(scene_revision),
        "request_id": str(request_id),
        "kind": "pose",
        "reference": "box_center",
        "orientation_semantics": "box_axis_pi",
        "position": [float(box_message.pose.position.x),
                     float(box_message.pose.position.y),
                     float(box_message.pose.position.z)],
        "quaternion_xyzw": [
            float(box_message.pose.orientation.x),
            float(box_message.pose.orientation.y),
            float(box_message.pose.orientation.z),
            float(box_message.pose.orientation.w)],
    }


def grasp_from_attach(request_id, scene_revision, box_position,
                      box_quaternion, tool_position, tool_quaternion,
                      stamp_ns=0):
    """Box-frame grasp transport measured at attach.

    Exact inverse math on the measured world poses: the box-frame offset
    of the suction contact and the relative orientation. The same relation
    the place-leg builder applies forward, so a round trip reproduces the
    measured TCP to machine precision.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation
    box_position = np.array(box_position, dtype=float)
    box_rotation = Rotation.from_quat(
        [float(v) for v in box_quaternion])
    translation = box_rotation.inv().apply(
        np.array(tool_position, dtype=float) - box_position)
    quaternion = (box_rotation.inv()
                  * Rotation.from_quat([float(v) for v in tool_quaternion])
                  ).as_quat()
    return grasp_record(
        request_id, scene_revision, translation, quaternion,
        stamp_ns=int(stamp_ns))


def pose_segment_message(name, target_position, target_quaternion,
                         request_id, scene_revision, *,
                         segment_type="pose_target", keep_tool_down=False,
                         keep_camera_down=False, allow_ompl_fallback=True,
                         cartesian_fraction=0.95):
    """One world-frame target segment as MotionSegmentV2 (no waypoints)."""
    return probe_segment_message(
        {"waypoints": [list(target_position)],
         "target_quaternion_xyzw": list(target_quaternion),
         "type": segment_type, "keep_tool_down": keep_tool_down,
         "keep_camera_down": keep_camera_down,
         "allow_ompl_fallback": allow_ompl_fallback},
        request_id, scene_revision, name=name,
        cartesian_fraction=cartesian_fraction)


def slot_spec_from_record(record, scene_config, box_wdh):
    """v1 SlotSpec view of a slot record (container -> base_link, shared
    scene helpers only). For legacy bookkeeping; the wire stays typed."""
    from luggage_msgs.msg import SlotSpec
    from luggage_description.scene_tf_config_utils import _local_point_to_base_link
    base_xyz = _local_point_to_base_link(list(record["position"]),
                                         scene_config)
    yaw = 2.0 * math.atan2(record["quaternion_xyzw"][2],
                           record["quaternion_xyzw"][3])
    slot = SlotSpec()
    slot.place_pose.position.x = float(base_xyz[0])
    slot.place_pose.position.y = float(base_xyz[1])
    slot.place_pose.position.z = float(base_xyz[2])
    slot.place_pose.orientation.z = math.sin(0.5 * yaw)
    slot.place_pose.orientation.w = math.cos(0.5 * yaw)
    slot.width = float(box_wdh[0])
    slot.height = float(box_wdh[2])
    slot.depth = float(box_wdh[1])
    return slot


def segment_message_to_v2(message, request_id, scene_revision):
    """Wrap one v1 MotionSegment message as MotionSegmentV2, verbatim.

    Field-preserving transport conversion for callers whose segments come
    from the shared builders (pick sessions: pose targets, no joint knots).
    """
    waypoints = [[float(pose.position.x), float(pose.position.y),
                  float(pose.position.z)] for pose in message.waypoints]
    waypoints.append([float(message.target_pose.position.x),
                      float(message.target_pose.position.y),
                      float(message.target_pose.position.z)])
    return probe_segment_message(
        {"waypoints": waypoints,
         "target_quaternion_xyzw": [
             float(message.target_pose.orientation.x),
             float(message.target_pose.orientation.y),
             float(message.target_pose.orientation.z),
             float(message.target_pose.orientation.w)],
         "type": str(message.type),
         "keep_tool_down": bool(message.keep_tool_down),
         "keep_camera_down": bool(message.keep_camera_down),
         "allow_ompl_fallback": bool(message.allow_ompl_fallback)},
        request_id, scene_revision, name=str(message.name))


def container_transform_fields(scene_config):
    """(source, target, translation, quaternion_xyzw) from the scene config.

    The shared scene helpers own the conversion; drivers never hand-roll it.
    """
    from luggage_description.scene_tf_config_utils import origin_in_world
    origin, rpy = origin_in_world(scene_config)
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = math.cos(0.5 * roll), math.sin(0.5 * roll)
    cp, sp = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
    cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    quaternion = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
    return ("container_link", "world",
            [float(v) for v in origin], [float(v) for v in quaternion])


def world_xyz_to_container(scene_config, xyz):
    """World-frame point to container_link (inverse of the published TF).

    Uses the same single scene-config transform as
    ``container_transform_fields`` — drivers never hand-roll the inverse.
    """
    _source, _target, translation, quaternion = container_transform_fields(
        scene_config)
    qx, qy, qz, qw = (float(v) for v in quaternion)
    dx = float(xyz[0]) - float(translation[0])
    dy = float(xyz[1]) - float(translation[1])
    dz = float(xyz[2]) - float(translation[2])
    # p_container = R(q)^T * (p_world - t); rows below are R's columns.
    return [
        (1.0 - 2.0 * (qy * qy + qz * qz)) * dx
        + 2.0 * (qx * qy + qw * qz) * dy
        + 2.0 * (qx * qz - qw * qy) * dz,
        2.0 * (qx * qy - qw * qz) * dx
        + (1.0 - 2.0 * (qx * qx + qz * qz)) * dy
        + 2.0 * (qy * qz + qw * qx) * dz,
        2.0 * (qx * qz + qw * qy) * dx
        + 2.0 * (qy * qz - qw * qx) * dy
        + (1.0 - 2.0 * (qx * qx + qy * qy)) * dz,
    ]


def probe_segment_message(record, request_id, scene_revision, *,
                          name="place_probe", cartesian_fraction=0.95):
    """MotionSegmentV2 for ProbeMotionSegmentV2 from world waypoints."""
    from luggage_description.frame_diagnostics import BoundaryContext
    from luggage_planning.motion_frame_adapters import message_from_admitted
    from luggage_planning.motion_frame_contracts import (
        TARGET_LINK, ExecutionContext)

    boundary = BoundaryContext(
        interface_id="F15", producer="typed_driver_adapter",
        consumer="motion_planner", request_id=str(request_id),
        scene_revision=str(scene_revision), frame_id="world",
        reference="suction_contact_frame", time_basis="scene")
    context = ExecutionContext(boundary, TARGET_LINK, "world")

    def pose(position, quaternion):
        return dict(
            contract_version=2, kind="pose", frame_id="world", length_unit="m",
            request_id=str(request_id), scene_revision=str(scene_revision),
            stamp_ns=0, time_basis="scene", reference="suction_contact_frame",
            orientation_semantics="tool_quaternion",
            position=[float(v) for v in position],
            quaternion_xyzw=[float(v) for v in quaternion])

    world = [list(map(float, point)) for point in record["waypoints"]]
    if not world:
        raise ContractError("INVALID_NUMERIC_PAYLOAD", "waypoints",
                            "nonempty world path", [])
    target = world[-1]
    segment = dict(
        contract_version=2, request_id=str(request_id),
        scene_revision=str(scene_revision), stamp_ns=0, time_basis="scene",
        name=name, segment_type=str(record.get("type") or "cartesian"),
        target_link=TARGET_LINK, policy_valid=True,
        keep_tool_down=bool(record.get("keep_tool_down", True)),
        keep_camera_down=bool(record.get("keep_camera_down", False)),
        lock_wrist=False,
        allow_ompl_fallback=bool(record.get("allow_ompl_fallback", True)),
        required_cartesian_fraction=float(cartesian_fraction),
        target_pose=pose(target, record["target_quaternion_xyzw"]),
        waypoints=[pose(point, record["target_quaternion_xyzw"])
                   for point in world[:-1]],
        joint_trajectory=None)
    return message_from_admitted(segment, context)


def apply_selected_path_to_v2(message, waypoints, frame_convert=None):
    """Rewrite a built MotionSegmentV2 carry leg onto selected TCP points.

    Identity, stamp, and orientation stay on ``message``. Positions become
    the selected path. ``frame_convert`` is identity when the points are
    already world (SelectPlacePathV2 ``waypoint_poses``).
    """
    from copy import deepcopy

    points = [list(xyz) for xyz in (waypoints or [])]
    if not points:
        return message
    convert = frame_convert or (lambda xyz: xyz)
    world = [list(convert(xyz)) for xyz in points]
    message.segment_type = "cartesian"
    message.allow_ompl_fallback = True
    dest = world[-1]
    message.target_pose.pose.position.x = float(dest[0])
    message.target_pose.pose.position.y = float(dest[1])
    message.target_pose.pose.position.z = float(dest[2])
    intermediates = []
    for xyz in world[:-1]:
        waypoint = deepcopy(message.target_pose)
        waypoint.pose.position.x = float(xyz[0])
        waypoint.pose.position.y = float(xyz[1])
        waypoint.pose.position.z = float(xyz[2])
        intermediates.append(waypoint)
    message.waypoints = intermediates
    return message


def adopt_v2_identity(dst, src):
    """Copy request/scene/stamp identity from a built leg onto a selected path.

    SelectPlacePathV2 cartesian legs are executable, but PlanMotionV2
    admits against the BuildMotionSequenceV2 request identity.
    """
    dst.request_id = src.request_id
    dst.scene_revision = src.scene_revision
    dst.stamp_valid = src.stamp_valid
    dst.stamp_ns = src.stamp_ns
    dst.time_basis = src.time_basis

    def _copy_identity(left, right):
        left.contract_version = right.contract_version
        left.frame_id = right.frame_id
        left.length_unit = right.length_unit
        left.stamp_valid = right.stamp_valid
        left.stamp_ns = right.stamp_ns
        left.time_basis = right.time_basis
        left.scene_revision = right.scene_revision
        left.request_id = right.request_id

    _copy_identity(dst.target_pose.identity, src.target_pose.identity)
    template = src.target_pose.identity
    if src.waypoints:
        template = src.waypoints[0].identity
    for waypoint in dst.waypoints:
        _copy_identity(waypoint.identity, template)
    return dst


# --------------------------------------------------------------------------
# Identity adoption from the live graph

def adopt_runtime_identity(node, *, peer, timeout_sec=10.0):
    """Take identity + peer set from a healthy stack heartbeat.

    Returns ``(runtime, registry)`` with a NodeRuntime for this driver peer.
    The stack was launched with ``frame_driver_peer`` naming this node; its
    registry therefore already includes us. A graph that never speaks fails
    closed with MISSING_PEER.
    """
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from luggage_description.ros_frame_bindings.runtime import (
        RuntimeIdentityV2, TOPIC, NodeRuntime, report_from_message)
    from luggage_description.runtime_graph import RUNTIME_IDENTITY_TOPIC
    import time as _time

    received = {}

    def on_message(message):
        received["message"] = message

    subscription = node.create_subscription(
        RuntimeIdentityV2, TOPIC, on_message,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.VOLATILE))
    # Drivers adopt before their main executor claims the node, so a private
    # single-threaded executor pumps the subscription here.
    from rclpy.executors import SingleThreadedExecutor
    spin_executor = SingleThreadedExecutor()
    spin_executor.add_node(node)
    try:
        deadline = _time.monotonic() + float(timeout_sec)
        while "message" not in received and _time.monotonic() < deadline:
            spin_executor.spin_once(timeout_sec=0.05)
        if "message" not in received:
            raise ContractError("MISSING_PEER", "runtime_identity",
                                "live stack heartbeat", "none")
        message = received["message"]
        endpoints = {}
        for item in message.endpoints:
            endpoints[item.name] = {
                "type": str(item.type_name),
                "provider": list(item.providers),
                "clients": list(item.clients),
            }
        if RUNTIME_IDENTITY_TOPIC not in endpoints:
            raise ContractError("MISSING_ENDPOINT", "runtime_identity",
                                "identity topic in registry", sorted(endpoints))
        peers = [str(name) for name in
                 endpoints[RUNTIME_IDENTITY_TOPIC]["provider"]]
        if str(peer).lstrip("/") not in peers:
            raise ContractError(
                "ENDPOINT_PROVIDER_MISMATCH", "frame_driver_peer",
                "driver named by the graph", peer)
        from luggage_description.runtime_profile import validate_identity
        from luggage_description.ros_frame_bindings.runtime import (
            SCALAR_IDENTITY_FIELDS)
        identity = {field: getattr(message, field)
                    for field in SCALAR_IDENTITY_FIELDS}
        identity.update(
            contract_version=int(message.contract_version),
            joint_names=list(message.joint_names),
            endpoints={name: entry["type"] for name, entry in endpoints.items()})
        validate_identity(identity)
        import sys as _sys
        adapter_module = _sys.modules[__name__]
        runtime = NodeRuntime(
            node, peer=peer, identity=identity,
            resolved_config={"driver": str(peer).lstrip("/")},
            peers=peers,
            modules=[(adapter_module, "module", "application")])
        return runtime, endpoints
    finally:
        spin_executor.remove_node(node)
        node.destroy_subscription(subscription)


# --------------------------------------------------------------------------
# The adapter

#: rclpy action goal terminal statuses the cancel-receipt check names
#: (4 SUCCEEDED / 5 CANCELED / 6 ABORTED; anything else is UNKNOWN,
#: a receipt that never landed is TIMEOUT).
_CANCEL_STATUS_NAMES = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}


class TypedDriverAdapter:
    """v2 service clients for one driver node, readiness-gated per call."""

    def __init__(self, node, runtime, *, group=None):
        from luggage_msgs.srv import (
            AddPlacedBoxV2, BuildMotionSequenceV2, ComputePlacementV2,
            ProbeMotionSegmentV2, RemovePlacedBoxV2, SelectPlacePathV2,
            VacuumCommandV2)
        self.node = node
        self.runtime = runtime
        self._placement_prior = None
        group = group or getattr(node, "_group", None)
        self._build = node.create_client(
            BuildMotionSequenceV2, "/waypoint_generator/build_motion_sequence_v2",
            callback_group=group)
        from rclpy.action import ActionClient
        from luggage_msgs.action import PlanMotionV2
        self._plan = ActionClient(
            node, PlanMotionV2, "/motion_planner/plan_motion_v2",
            callback_group=group)
        self._compute = node.create_client(
            ComputePlacementV2, "/placement_planner/compute_placement_v2",
            callback_group=group)
        self._select = node.create_client(
            SelectPlacePathV2, "/trajectory_selector/select_place_path_v2",
            callback_group=group)
        self._probe = node.create_client(
            ProbeMotionSegmentV2, "/motion_planner/probe_motion_segment_v2",
            callback_group=group)
        self._vacuum = node.create_client(
            VacuumCommandV2, "/vacuum/command_v2", callback_group=group)
        self._add_placed = node.create_client(
            AddPlacedBoxV2, "/cargo_map/add_placed_box_v2",
            callback_group=group)
        self._remove_placed = node.create_client(
            RemovePlacedBoxV2, "/cargo_map/remove_placed_box_v2",
            callback_group=group)

    def _call(self, client, request, timeout_sec):
        """Readiness-gated synchronous call; None on timeout/unavailable."""
        self.runtime.require_graph_ready()
        if not client.wait_for_service(timeout_sec=2.0):
            raise ContractError("MISSING_ENDPOINT", client.srv_name,
                                "v2 service", "unavailable")
        recorder = getattr(self, '_boundary_recorder', None)
        if recorder is not None:
            recorder(client.srv_name + ':request', request)
        future = client.call_async(request)
        import time as _time
        deadline = _time.monotonic() + float(timeout_sec)
        while not future.done() and _time.monotonic() < deadline:
            _time.sleep(0.02)
        if not future.done():
            raise ContractError(
                "BOUNDARY_TIMEOUT", client.srv_name, "response", timeout_sec)
        response = future.result()
        if recorder is not None:
            recorder(client.srv_name + ':response', response)
        return response

    def set_boundary_recorder(self, recorder):
        """Opt-in exact service capture before caller remapping or mutation."""
        self._boundary_recorder = recorder

    def set_placement_prior(self, prior):
        """Bind an optional measured pre-pick hint for this driver's next Select.

        Actual attachment, map and dimensions are checked again at Select. A
        caller starts a new cycle or abandons this grasp by passing None.
        """
        import copy
        if prior is not None and (not prior.success or not prior.measured):
            raise ValueError("only a successful measured prior may be bound")
        self._placement_prior = copy.deepcopy(prior)

    # -- placement ---------------------------------------------------------
    def compute_placement(self, *, request_id, box_wdh, height_measured,
                          scene_revision, placed=(), placed_wdh_flat=(),
                          geometry_hash="", expected_map_revision=0,
                          max_candidates=5, conservative_geometry=None):
        """ComputePlacementV2 with an empty (first-box) placed list by default."""
        from luggage_msgs.srv import ComputePlacementV2
        request = ComputePlacementV2.Request()
        dimensions = box_wdh
        if conservative_geometry is not None:
            if tuple(float(v) for v in box_wdh) != conservative_geometry.measured_dimensions_wdh:
                raise ValueError('ATTACHMENT_GEOMETRY_BINDING_MISMATCH')
            dimensions = conservative_geometry.conservative.dimensions_wdh
        request.box_wdh = [float(v) for v in dimensions]
        request.height_measured = bool(height_measured)
        for record in placed:
            request.placed.append(pose_message(record))
        request.placed_wdh_flat = [float(v) for v in placed_wdh_flat]
        request.geometry_hash = str(geometry_hash)
        request.expected_map_revision = int(expected_map_revision)
        request.max_candidates = int(max_candidates)
        response = self._call(self._compute, request, 30.0)
        if conservative_geometry is None:
            return response
        from luggage_planning.attachment_placement_adapter import nominal_placement_response
        return nominal_placement_response(response, conservative_geometry)

    # -- selection ---------------------------------------------------------
    def select_place_path(self, *, request_id, box_wdh, slots, slot_scores,
                          surface_message, inner_size, geometry_hash,
                          map_revision, start_tcp, start_joints, grasp,
                          container_transform, excluded_ids=(),
                          box_id="", generation=0, timeout_sec=30.0, placement_prior=None,
                          conservative_geometry=None):
        """SelectPlacePathV2; the mapper's surface message rides verbatim."""
        from luggage_msgs.srv import SelectPlacePathV2
        source, target, translation, quaternion = container_transform
        request = SelectPlacePathV2.Request()
        if conservative_geometry is not None:
            from luggage_planning.attachment_geometry_transport import geometry_to_message
            request.conservative_geometry = geometry_to_message(conservative_geometry)
        request.request_id = str(request_id)
        request.box_wdh = [float(v) for v in box_wdh]
        from luggage_msgs.msg import PoseContractV2
        for record in slots:
            request.slots.append(
                record if isinstance(record, PoseContractV2) else pose_message(record))
        request.slot_scores = [float(v) for v in slot_scores]
        request.surface = surface_message
        request.inner_size = [float(v) for v in inner_size]
        request.geometry_hash = str(geometry_hash)
        request.map_revision = int(map_revision)
        request.start_tcp = pose_message(start_tcp)
        request.start_joints = [float(v) for v in start_joints]
        request.grasp = pose_message(grasp)
        request.container_source_frame = str(source)
        request.container_target_frame = str(target)
        request.container_translation = [float(v) for v in translation]
        request.container_quaternion_xyzw = [float(v) for v in quaternion]
        request.container_stamp_ns = 0
        request.container_stamp_is_null = True
        request.excluded_ids = [str(v) for v in excluded_ids]
        request.box_id = str(box_id or "")
        request.generation = int(generation or 0)
        from luggage_planning.placement_prior_adoption import order_select_candidates
        if placement_prior is None:
            placement_prior = getattr(self, "_placement_prior", None)
        order = order_select_candidates(request, placement_prior)
        response = self._call(self._select, request, timeout_sec)
        if response is not None and response.success and 0 <= response.slot_index < len(order):
            response.slot_index = order[response.slot_index]
        return response

    # -- probe ---------------------------------------------------------------
    def probe_segment(self, segment_message, timeout_sec=30.0):
        from luggage_msgs.srv import ProbeMotionSegmentV2
        request = ProbeMotionSegmentV2.Request()
        request.segment = segment_message
        return self._call(self._probe, request, timeout_sec)

    # -- attachment ----------------------------------------------------------
    def vacuum_command(self, *, enable, request_id, scene_revision, box_id,
                       generation, measured_box=None, box_pose=None,
                       grasp=None):
        from luggage_msgs.srv import VacuumCommandV2
        request = VacuumCommandV2.Request()
        request.enable = bool(enable)
        request.request_id = str(request_id)
        request.scene_revision = str(scene_revision)
        request.box_id = str(box_id)
        request.generation = int(generation)
        if measured_box is not None:
            request.measured_box = measured_box
        if box_pose is not None:
            request.box_pose = pose_message(box_pose)
        request.has_grasp = grasp is not None
        if grasp is not None:
            request.grasp = pose_message(grasp)
        return self._call(self._vacuum, request, 15.0)

    # -- map commits ---------------------------------------------------------
    def add_placed_box(self, *, slot, size_wdh, measured_source,
                       expected_map_revision, geometry_hash,
                       verified_center_container=None,
                       verify_tolerance_m=0.0, box_id="",
                       verify_source=""):
        from luggage_msgs.srv import AddPlacedBoxV2
        request = AddPlacedBoxV2.Request()
        request.slot_pose = pose_message(slot)
        request.size_wdh = [float(v) for v in size_wdh]
        request.measured_source = str(measured_source)
        request.expected_map_revision = int(expected_map_revision)
        request.geometry_hash = str(geometry_hash)
        # [release-failure contract] Correlation identity (must match the
        # box_released fact) and the DECLARED verify measurement source.
        request.box_id = str(box_id or "")
        request.verify_source = str(verify_source or "")
        # [commit anchor] Optional verify-measured pose; tolerance <= 0
        # leaves the legacy un-anchored commit.
        if verified_center_container is not None:
            request.verified_center_container = [
                float(v) for v in verified_center_container]
            request.verify_tolerance_m = float(verify_tolerance_m)
        return self._call(self._add_placed, request, 15.0)

    def remove_placed_box(self, *, slot, size_wdh, expected_map_revision,
                          geometry_hash):
        from luggage_msgs.srv import RemovePlacedBoxV2
        request = RemovePlacedBoxV2.Request()
        request.slot_pose = pose_message(slot)
        request.size_wdh = [float(v) for v in size_wdh]
        request.expected_map_revision = int(expected_map_revision)
        request.geometry_hash = str(geometry_hash)
        return self._call(self._remove_placed, request, 15.0)

    # -- waypoint/execution chain ---------------------------------------------
    def build_pick(self, *, request_id, scene_revision, pick_message,
                   timeout_sec=60.0):
        """BuildMotionSequenceV2 pick phase from the measured detection.

        The edge builder's first MoveIt call pays the kinematics-loader
        warmup, so the default budget is generous like the drivers'.
        """
        from luggage_msgs.srv import BuildMotionSequenceV2
        request = BuildMotionSequenceV2.Request()
        request.phase = "pick"
        request.pick = pick_message
        candidates = list(getattr(pick_message, "suction_candidates", ()) or ())
        if candidates:
            selected = min(candidates, key=lambda item: (item.rank, item.candidate_id))
            request.suction_candidate_id = str(selected.candidate_id)
        request.request_id = str(request_id)
        request.scene_revision = str(scene_revision)
        request.pick_pose = pose_message(detection_record_from_box(
            pick_message, request_id, scene_revision))
        return self._call(self._build, request, timeout_sec)

    def build_place(self, *, slot, grasp, box_wdh, container_transform):
        """BuildMotionSequenceV2 place phase from the admitted slot record."""
        from luggage_msgs.srv import BuildMotionSequenceV2
        source, target, translation, quaternion = container_transform
        request = BuildMotionSequenceV2.Request()
        request.phase = "place"
        request.request_id = str(slot["request_id"])
        request.slot = pose_message(slot)
        request.grasp = pose_message(grasp)
        request.box_wdh = [float(v) for v in box_wdh]
        request.container_source_frame = str(source)
        request.container_target_frame = str(target)
        request.container_translation = [float(v) for v in translation]
        request.container_quaternion_xyzw = [float(v) for v in quaternion]
        request.container_stamp_ns = 0
        request.container_stamp_is_null = True
        return self._call(self._build, request, 15.0)

    def _cancel_and_close(self, handle, deadline):
        """[ST09-I G0-4] Cancel an accepted goal and consume the
        controller's terminal receipt within the caller's deadline —
        stop/hold must be acknowledged, never assumed. A receipt that
        never lands is cancel_status=TIMEOUT and the refusal stands:
        no detach, no map commit, no auto-recovery here."""
        import time as _time
        result_future = handle.get_result_async()
        handle.cancel_goal()
        cancel_status = "TIMEOUT"
        result = None
        while not result_future.done() and _time.monotonic() < deadline:
            _time.sleep(0.02)
        if result_future.done():
            outcome = result_future.result()
            cancel_status = _CANCEL_STATUS_NAMES.get(
                int(getattr(outcome, "status", 0)), "UNKNOWN")
            result = getattr(outcome, "result", None)
        return False, ("CARRY_FAULT_PAYLOAD_LOST cancel_status=%s"
                       % cancel_status), result

    def plan_motion(self, motion_segment, timeout_sec, name="PlanMotionV2",
                    admission=None, should_abort=None):
        """Execute one typed segment through PlanMotionV2.

        Returns ``(ok, message, result)`` with the same convention as the
        drivers' v1 send_action helper. ``admission`` ([ST09-I S5.7-3b])
        carries the certified route digest / certificate / validated map
        revision so the executor can check the generation is still live
        before submitting. ``should_abort`` ([ST09-I S5.7-6]) is polled
        in BOTH wait loops: a payload loss mid-execution cancels the
        goal and consumes the controller's terminal receipt; a payload
        loss while the goal is still pending CANCELS THE LATE
        ACCEPTANCE — "future not done" is never treated as "server did
        not accept" (G0-4). Fail-closed either way, zero controller
        submissions after the refusal.
        """
        import time as _time
        from luggage_description.runtime_action_readiness import certified_motion_scope
        scope = certified_motion_scope(admission)
        if scope is None:
            self.runtime.require_graph_ready()
        else:
            self.runtime.require_graph_ready(scope=scope)
        if not self._plan.wait_for_server(timeout_sec=2.0):
            return False, "plan_motion_v2 unavailable", None
        from luggage_msgs.action import PlanMotionV2
        goal = PlanMotionV2.Goal()
        goal.segment = motion_segment
        if admission:
            goal.route_digest = str(
                admission.get("route_digest", "") or "")
            goal.certification_id = str(
                admission.get("certification_id", "") or "")
            goal.admission_map_revision = int(
                admission.get("admission_map_revision", 0) or 0)
            # [ST09-I S5.7-6] per-leg scope: the leg executes under its
            # certified load-state index (carrying -> release -> unloaded).
            goal.admission_leg_digest = str(
                admission.get("leg_digest", "") or "")
            goal.admission_load_state = str(
                admission.get("load_state", "") or "")
        # send_goal() is the blocking helper and returns the result response.
        # The executor is already spinning, so the async goal keeps the
        # caller's deadline and does not deadlock the main thread.
        future = self._plan.send_goal_async(goal)
        deadline = _time.monotonic() + float(timeout_sec)
        while not future.done() and _time.monotonic() < deadline:
            if should_abort is not None and should_abort():
                # [G0-4] the goal was already handed to the server: a
                # late acceptance would execute it after the caller
                # walked away. Resolve the handle within the caller's
                # deadline and CANCEL it — only a server rejection or a
                # consumed terminal receipt closes the fault; a goal
                # that never resolves is a named TIMEOUT refusal, never
                # a silent zero-submission claim.
                while (not future.done()
                        and _time.monotonic() < deadline):
                    _time.sleep(0.02)
                if not future.done():
                    return False, ("CARRY_FAULT_PAYLOAD_LOST "
                                   "cancel_status=TIMEOUT goal_unresolved"
                                   ), None
                handle = future.result()
                if not handle.accepted:
                    return False, (
                        "CARRY_FAULT_PAYLOAD_LOST server_rejected"), None
                return self._cancel_and_close(handle, deadline)
            _time.sleep(0.02)
        if not future.done():
            return False, "%s: goal timeout" % name, None
        handle = future.result()
        if not handle.accepted:
            return False, "%s: rejected" % name, None
        result_future = handle.get_result_async()
        aborted = False
        while not result_future.done() and _time.monotonic() < deadline:
            if should_abort is not None and should_abort():
                aborted = True
                break
            _time.sleep(0.02)
        if aborted:
            # Mid-execution payload loss: cancel, then consume the
            # controller's terminal receipt (bounded by the same
            # deadline).
            return self._cancel_and_close(handle, deadline)
        if not result_future.done():
            handle.cancel_goal()
            return False, "%s: result timeout" % name, None
        result = result_future.result().result
        return bool(result.success), str(result.message or ""), result
