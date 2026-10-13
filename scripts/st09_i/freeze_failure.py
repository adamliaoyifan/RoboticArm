"""ST09-I Phase 1: freeze the original portal-certification failure.

Runs the unified select through the REAL RouteProducer construction (the
trajectory_selector_node code path, in this process against the live mock
stack) and captures everything needed to re-run the failure offline:

* source/install/URDF/SRDF/config identity hashes
* the reposition leg and the live joint states around the select
* the full PlanningScene at select time (world, attached, touch links, ACM)
* the placement/select requests and responses, attachment record
* every planning-scene diff applied and every GetCartesianPath probe
  (request AND response, including the solver trajectory) verbatim
* the certified-input trajectories per edge (positions, velocities,
  accelerations, ns times, joint order) via the capturing adapter

Artifacts land in the evidence tree (never only /tmp):
    docs/status/evidence/frame_migration/ST09-parallel-I-interim/phase1/

Usage (stack already up via stack_launch.py, same domain):
    python3 scripts/st09_i/freeze_failure.py [--evidence-dir DIR]
"""

from __future__ import division

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
# Import DISCIPLINE: the frame runtime's overlay provenance check rejects
# any process whose imported luggage_* modules live outside the install
# root (WRONG_OVERLAY). Run with the install overlay sourced and do NOT
# prepend src/ — only the repo scripts dir (probe helpers) is inserted.
for entry in (str(ROOT / "scripts"),):
    if entry not in sys.path:
        sys.path.insert(0, entry)

CARRY_POSE_Q = (0.936, -1.025, 1.268, -1.819, -1.567, -0.635)


# ---------------------------------------------------------------- helpers


def sha256_text(text):
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(65536), b""):
            digest.update(block)
    return digest.hexdigest()


def pose_json(pose):
    return {
        "position": [float(pose.position.x), float(pose.position.y),
                     float(pose.position.z)],
        "orientation_xyzw": [
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w)],
    }


def primitive_json(primitive):
    from shape_msgs.msg import SolidPrimitive

    types = {SolidPrimitive.BOX: "BOX", SolidPrimitive.SPHERE: "SPHERE",
             SolidPrimitive.CYLINDER: "CYLINDER", SolidPrimitive.CONE: "CONE"}
    return {"type": types.get(int(primitive.type), str(primitive.type)),
            "dimensions": [float(v) for v in primitive.dimensions]}


def as_int(value):
    if isinstance(value, (bytes, bytearray)):
        return int.from_bytes(bytes(value), "little")
    return int(value)


def collision_object_json(obj):
    return {
        "id": str(obj.id),
        "frame_id": str(obj.header.frame_id),
        "primitives": [primitive_json(p) for p in obj.primitives],
        "primitive_poses": [pose_json(p) for p in obj.primitive_poses],
        "mesh_triangle_counts": [len(m.triangles) for m in obj.meshes],
        "mesh_poses": [pose_json(p) for p in obj.mesh_poses],
        "operation": as_int(obj.operation),
        "pose": pose_json(obj.pose) if hasattr(obj, "pose") else None,
    }


def attached_json(attached):
    return {
        "link_name": str(attached.link_name),
        "object": collision_object_json(attached.object),
        "touch_links": [str(link) for link in attached.touch_links],
        "detach_posture": None,
        "weight": float(getattr(attached, "weight", 0.0)),
    }


def joint_state_json(joint_state):
    return {
        "name": [str(n) for n in joint_state.name],
        "position": [float(v) for v in joint_state.position],
        "velocity": [float(v) for v in joint_state.velocity],
        "effort": [float(v) for v in joint_state.effort],
        "stamp_ns": int(joint_state.header.stamp.sec) * 10**9
        + int(joint_state.header.stamp.nanosec) if joint_state.header.stamp.sec
        else None,
    }


def scene_json(scene):
    acm = scene.allowed_collision_matrix
    entries = {}
    for index, name in enumerate(acm.entry_names):
        enabled = []
        values = (acm.entry_values[index].enabled
                  if len(acm.entry_values) > index else [])
        for other_index, flag in enumerate(values):
            if bool(flag):
                enabled.append(str(acm.entry_names[other_index]))
        entries[str(name)] = enabled
    return {
        "name": str(scene.name),
        "is_diff": bool(scene.is_diff),
        "frame_id": str(scene.robot_state.joint_state.header.frame_id
                        if scene.robot_state.joint_state.header.frame_id
                        else ""),
        "world": {
            "collision_objects": [
                collision_object_json(obj)
                for obj in scene.world.collision_objects],
            "octomap": bool(scene.world.octomap.octomap.data),
        },
        "robot_state": {
            "joint_state": joint_state_json(scene.robot_state.joint_state),
            "attached_collision_objects": [
                attached_json(obj)
                for obj in scene.robot_state.attached_collision_objects],
            "is_diff": bool(scene.robot_state.is_diff),
        },
        "acm_entry_names": [str(n) for n in acm.entry_names],
        "acm_default_entries": [
            [str(pair[0]), str(pair[1])] for pair in acm.default_entry_names],
        "acm_entries": entries,
    }


def cartesian_request_json(request):
    return {
        "header": {"frame_id": str(request.header.frame_id)},
        "start_state": {
            "joint_state": joint_state_json(request.start_state.joint_state),
            "is_diff": bool(request.start_state.is_diff),
            "attached_collision_objects": [
                attached_json(obj) for obj in
                request.start_state.attached_collision_objects],
        },
        "group_name": str(request.group_name),
        "link_name": str(request.link_name),
        "n_waypoints": len(request.waypoints),
        "waypoints": [pose_json(pose) for pose in request.waypoints],
        "max_step": float(request.max_step),
        "jump_threshold": float(request.jump_threshold),
        "avoid_collisions": bool(request.avoid_collisions),
    }


def cartesian_response_json(response):
    trajectory = response.solution.joint_trajectory
    return {
        "error_code": int(response.error_code.val),
        "fraction": float(response.fraction),
        "start_state": joint_state_json(
            response.solution.start_state.joint_state),
        "trajectory_joint_names": [str(n) for n in trajectory.joint_names],
        "trajectory_points": [
            {"positions": [float(v) for v in point.positions],
             "velocities": [float(v) for v in point.velocities],
             "accelerations": [float(v) for v in point.accelerations],
             "time_from_start_ns": int(
                 point.time_from_start.sec) * 10**9 + int(
                 point.time_from_start.nanosec)}
            for point in trajectory.points],
    }


def trajectory_json(trajectory):
    """TimedTrajectory / leg joint spec as JSON (knots preserved)."""
    if isinstance(trajectory, dict):
        return trajectory
    if hasattr(trajectory, "time_from_start_ns"):
        return {
            "kind": "timed_trajectory",
            "joint_names": [str(n) for n in trajectory.joint_names],
            "points": [
                {"positions": [float(v) for v in row],
                 "velocities": [float(v) for v in velocity_row],
                 "accelerations": [float(v) for v in accel_row],
                 "time_from_start_ns": int(stamp)}
                for row, velocity_row, accel_row, stamp in zip(
                    trajectory.positions, trajectory.velocities,
                    trajectory.accelerations,
                    trajectory.time_from_start_ns)],
        }
    joint = getattr(trajectory, "joint_trajectory", trajectory)
    return {
        "kind": "joint_trajectory",
        "joint_names": [str(n) for n in joint.joint_names],
        "points": [
            {"positions": [float(v) for v in point.positions],
             "velocities": [float(v) for v in point.velocities],
             "accelerations": [float(v) for v in point.accelerations],
             "time_from_start_ns": int(point.time_from_start_ns)}
            for point in joint.points],
    }


# ---------------------------------------------------------------- capture


class CapturingSceneClient(object):
    """PlanningSceneClient wrapper recording every applied diff."""

    def __init__(self, inner, record):
        self._inner = inner
        self._record = record

    def apply_diff(self, scene, timeout=None):
        entry = {"kind": "apply_diff", "at_ns": time.monotonic_ns(),
                 "scene": scene_json(scene)}
        self._record.append(entry)
        ok, reason = self._inner.apply_diff(scene, timeout=timeout)
        entry["ok"] = bool(ok)
        entry["reason"] = str(reason)
        return ok, reason

    def __getattr__(self, name):
        return getattr(self._inner, name)


class CapturingValidityAdapter(object):
    """The real MoveItStateValidityAdapter with probe call recording.

    Every GetCartesianPath request (verbatim) and response (including the
    solver trajectory) is recorded; the certified input edges, per-edge
    samples and scene states are stashed for offline replay.
    """

    def __init__(self, factory, record):
        self._factory = factory
        self._record = record
        self.inner = None
        self.edges = {}
        self.scene_states = {}
        self.samples = {}

    def __call__(self, *, scene_states, edges):
        adapter = self._factory(scene_states=scene_states, edges=edges)
        for edge in edges:
            from luggage_planning.moveit_state_validity import sample_states

            edge_id = str(edge.edge_id)
            self.edges[edge_id] = {
                "edge_id": edge_id,
                "edge_class": str(getattr(edge, "attributes", {}).get(
                    "edge_class", "")),
                "source": str(edge.source_node_id),
                "destination": str(edge.destination_node_id),
                "load_state": str(getattr(edge, "load_state", "")),
                "trajectory": trajectory_json(edge.trajectory),
            }
            state = scene_states.get(edge_id)
            payload = getattr(state, "payload", None)
            self.scene_states[edge_id] = {
                "load_state": str(getattr(state, "load_state", "")),
                "payload": None if payload is None else {
                    "t_tcp_box_translation": list(
                        payload.t_tcp_box_translation),
                    "t_tcp_box_quaternion_xyzw": list(
                        payload.t_tcp_box_quaternion_xyzw),
                    "dimensions_wdh": list(payload.dimensions_wdh),
                },
                "boxes": [
                    {"object_id": str(box.object_id),
                     "center": list(box.center),
                     "size_whd": list(box.size_whd)}
                    for box in tuple(getattr(state, "boxes", ()) or ())],
                "placed_boxes": [
                    {"object_id": str(box.object_id),
                     "center": list(box.center),
                     "size_whd": list(box.size_whd)}
                    for box in tuple(getattr(state, "placed_boxes", ()) or ())],
                "released_box": (
                    None if getattr(state, "released_box", None) is None else
                    {"object_id": str(state.released_box.object_id),
                     "center": list(state.released_box.center),
                     "size_whd": list(state.released_box.size_whd)}),
            }
            self.samples[edge_id] = sample_states(edge.trajectory)

        # probe recording is installed by the _ensure_clients class patch
        self.inner = adapter
        return adapter


def build_producer(node, runtime, record):
    """The trajectory_selector_node producer construction, verbatim,
    with the capturing scene client and validity adapter."""
    from ament_index_python.packages import get_package_share_directory

    from luggage_description.scene_tf_config_utils import (
        container_opening_normal_in_world,
        container_opening_target_point_in_world,
        container_usable_dimensions,
        load_scene_tf_config,
        resolve_scene_tf_config_path,
    )
    from luggage_planning.attachment_provider import (
        MeasuredAttachmentProvider,
    )
    from luggage_planning.map_revalidation import MapRevalidator
    from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter
    from luggage_planning.moveit_state_validity import (
        MoveItStateValidityAdapter,
    )
    from luggage_planning.offline_ik import load_urdf, refine_to_pose
    from luggage_planning.planning_scene_client import PlanningSceneClient
    from luggage_planning.reachable_graph import (
        RouteGraph,
        make_offline_fk,
    )
    from luggage_planning.route_producer import (
        RouteProducer,
        certification_policy_from_contract,
    )
    from luggage_planning.trajectory_certifier import TrajectoryCertifier

    with open(os.path.join(
            get_package_share_directory("st09_contracts"),
            "resolved_planning_contract.json"), encoding="utf-8") as handle:
        cert_policy = certification_policy_from_contract(json.load(handle))
    scene_tf_path = resolve_scene_tf_config_path(os.path.join(
        get_package_share_directory("luggage_description"), "config",
        "scene_tf.yaml.example"))
    scene_cfg = load_scene_tf_config(scene_tf_path)
    urdf_model = load_urdf(scene_tf_config=scene_tf_path)
    tcp_path = urdf_model.path_to("suction_contact_frame")
    fk = make_offline_fk(urdf_model, tcp_path)

    def refine(position, quaternion, seed):
        return refine_to_pose(urdf_model, tcp_path, position, quaternion,
                              seed, pos_tol=2e-3, rot_tol=2e-2)

    scene_client = PlanningSceneClient(node)
    capturing_scene = CapturingSceneClient(scene_client, record)

    def validity_factory(*, scene_states, edges):
        adapter = MoveItStateValidityAdapter(
            node, fk=fk, scene_client=capturing_scene,
            log=lambda message: record.append(
                {"kind": "log", "at_ns": time.monotonic_ns(),
                 "message": str(message)}))
        adapter.load_edges(edges, scene_states=scene_states)
        return adapter

    capturing_validity = CapturingValidityAdapter(
        validity_factory, record)

    producer = RouteProducer(
        graph=RouteGraph(static_identity="st09-i-route:freeze"),
        edge_builder=MoveItEdgeAdapter(node, scene_client=capturing_scene),
        certifier=TrajectoryCertifier(),
        cert_policy=cert_policy,
        validity_adapter_factory=capturing_validity,
        attachment_provider=MeasuredAttachmentProvider(
            str(runtime.identity.get("scene_revision", "scene")),
            "route-selector", tcp_frame="suction_contact_frame",
            box_frame="box"),
        revalidator=MapRevalidator(),
        fk=fk, refine=refine, urdf=urdf_model, tcp_path=tcp_path,
        clock_ns=lambda: node.get_clock().now().nanoseconds,
        opening_info={
            "point": container_opening_target_point_in_world(scene_cfg),
            "normal": container_opening_normal_in_world(scene_cfg)},
        log=lambda message: record.append(
            {"kind": "producer-log", "at_ns": time.monotonic_ns(),
             "message": str(message)}))
    producer.scene_cfg = scene_cfg  # for the request builder
    return producer, capturing_validity, fk


def timed_joint_leg(positions, times):
    trajectory = type("T", (), {
        "joint_names": tuple("elfin_joint%d" % i for i in range(1, 7)),
        "positions": tuple(tuple(float(v) for v in row)
                           for row in positions),
        "velocities": ((0.0,) * 6,) * len(positions),
        "accelerations": ((0.0,) * 6,) * len(positions),
        "time_from_start_ns": tuple(int(t) for t in times)})()
    return trajectory


def main():
    parser = argparse.ArgumentParser()
    default_out = ROOT / "docs" / "status" / "evidence" / "frame_migration" / \
        "ST09-parallel-I-interim" / "phase1" / "freeze"
    parser.add_argument("--evidence-dir", default=str(default_out))
    parser.add_argument("--skip-reposition", action="store_true")
    args = parser.parse_args()
    out_dir = Path(args.evidence_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    import rclpy

    rclpy.init()
    from frame_migration_st09_graph import GraphProbe

    record = []
    probe = GraphProbe("motion_occ_driver")
    runtime = probe.adopt(timeout=90.0)
    node = probe.node

    # Surface any spin-thread death with its full traceback — a dead
    # spin thread silently turns every later service call into a
    # timeout, which must never masquerade as a system failure.
    import threading
    import traceback as _traceback

    def _excepthook(args):
        try:
            record.append({
                "kind": "thread-exception",
                "thread": str(getattr(args.thread, "name", "?")),
                "traceback": "".join(_traceback.format_exception(
                    args.exc_type, args.exc_value, args.exc_traceback)
                )[:4000]})
        except Exception:  # noqa: BLE001
            pass

    threading.excepthook = _excepthook
    # wait until the whole runtime graph reports READY (adopt alone may
    # return while peers are still joining)
    from luggage_description.frame_contracts import ContractError

    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        try:
            runtime.require_graph_ready()
            break
        except ContractError:
            time.sleep(2.0)
    else:
        print("runtime graph never reached READY")
        return 3

    # -- identity hashes -------------------------------------------------
    git_head = subprocess.run(
        ["git", "-C", str(ROOT), "rev-parse", "HEAD"],
        capture_output=True, text=True).stdout.strip()
    git_dirty = subprocess.run(
        ["git", "-C", str(ROOT), "status", "--porcelain"],
        capture_output=True, text=True).stdout.strip()
    from ament_index_python.packages import get_package_share_directory

    install_files = {}
    for package, relative in (
            ("st09_contracts", "resolved_planning_contract.json"),
            ("elfin_control", "config/elfin_controllers.yaml"),
            ("elfin_moveit_config", "config/S20.srdf"),
            ("luggage_description", "config/scene_tf.yaml.example")):
        path = os.path.join(get_package_share_directory(package), relative)
        install_files["%s/%s" % (package, relative)] = (
            sha256_file(path) if os.path.exists(path) else "MISSING")
    urdf_text = subprocess.run(
        ["ros2", "param", "get", "/move_group", "robot_description"],
        capture_output=True, text=True,
        env=dict(os.environ)).stdout
    srdf_text = subprocess.run(
        ["ros2", "param", "get", "/move_group",
         "robot_description_semantic"],
        capture_output=True, text=True,
        env=dict(os.environ)).stdout

    # -- joint state ------------------------------------------------------
    from sensor_msgs.msg import JointState

    latest = {}

    def on_joint_state(message):
        latest["message"] = message

    node.create_subscription(JointState, "/joint_states", on_joint_state, 10)

    def joints():
        message = latest.get("message")
        names = list(message.name)
        return [float(message.position[names.index(
            "elfin_joint%d" % i)]) for i in range(1, 7)]

    deadline = time.monotonic() + 15.0
    while "message" not in latest and time.monotonic() < deadline:
        time.sleep(0.2)
    q_before = joints()

    from luggage_planning.typed_driver_adapter import TypedDriverAdapter

    adapter = TypedDriverAdapter(node, runtime)
    scene_revision = str(runtime.identity.get("scene_revision", ""))

    # -- reposition to the carry pose ------------------------------------
    if not args.skip_reposition:
        from luggage_planning.route_producer import _leg_record
        from luggage_description.frame_diagnostics import BoundaryContext
        from luggage_planning.motion_frame_adapters import (
            message_from_admitted,
        )
        from luggage_planning.motion_frame_contracts import (
            TARGET_LINK, ExecutionContext,
        )

        start = joints()
        steps = 6
        positions = [tuple(s + (t - s) * k / steps for s, t in
                           zip(start, CARRY_POSE_Q))
                     for k in range(steps + 1)]
        times = [int(round(4e9 * k / steps)) for k in range(steps + 1)]
        leg = _leg_record("carry_pose", timed_joint_leg(positions, times),
                          "freeze-carry-1", scene_revision, 1000)
        boundary = BoundaryContext(
            interface_id="F15", producer="route_producer",
            consumer="motion_executor", request_id="freeze-carry-1",
            scene_revision=scene_revision, frame_id="world",
            reference="suction_contact_frame", time_basis="robot_state")
        context = ExecutionContext(boundary, TARGET_LINK, "world")
        ok, message_text, _detail = adapter.plan_motion(
            message_from_admitted(leg, context), 30.0, name="carry")
        record.append({"kind": "reposition", "ok": bool(ok),
                       "message": str(message_text)[:400]})
        time.sleep(1.5)
    q_at_select = joints()

    # -- placement --------------------------------------------------------
    from luggage_msgs.msg import PoseContractV2, SurfaceContractV2
    from rclpy.qos import (DurabilityPolicy, QoSProfile,
                           ReliabilityPolicy)

    received = {}
    node.create_subscription(
        SurfaceContractV2, "/luggage/cargo_map/surface_v2",
        lambda message: received.setdefault("surface", message),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    deadline = time.monotonic() + 10.0
    while "surface" not in received and time.monotonic() < deadline:
        time.sleep(0.1)
    surface = received["surface"]
    placement = adapter.compute_placement(
        request_id="freeze-p", box_wdh=[0.35, 0.25, 0.20],
        height_measured=True, scene_revision=scene_revision,
        geometry_hash=str(surface.geometry_hash or ""),
        expected_map_revision=int(surface.map_revision), max_candidates=400)
    placement_summary = {
        "success": bool(placement.success),
        "reason_code": str(placement.reason_code),
        "n_candidates": len(placement.candidates),
        "scores": [float(v) for v in placement.candidate_scores],
        "map_revision": int(getattr(placement, "map_revision", 0) or 0),
    }
    record.append({"kind": "placement", **placement_summary})

    # -- the select, through the real producer construction ---------------
    producer, capturing_validity, fk = build_producer(node, runtime, record)
    from luggage_description.scene_tf_config_utils import (
        container_usable_dimensions,
    )
    from luggage_planning.place_service import request_dict
    from luggage_planning.typed_driver_adapter import (
        container_transform_fields,
        grasp_record,
        pose_message,
        tcp_record,
    )
    from luggage_msgs.srv import SelectPlacePathV2

    columns = int(surface.columns)
    count = int(surface.rows) * columns
    state_grid = [list(surface.state[i:i + columns])
                  for i in range(0, count, columns)]
    slots = list(placement.candidates)
    if not slots:
        out_path = out_dir / "frozen_failure.json"
        with open(out_path, "w", encoding="utf-8") as handle:
            json.dump({"meta": {"git_head": git_head},
                       "placement": placement_summary,
                       "record": record}, handle, indent=1, default=str)
        print("PLACEMENT returned no candidates: %s" % placement_summary)
        node.destroy_node()
        rclpy.shutdown()
        return 2
    near = PoseContractV2()
    for field in ("contract_version", "frame_id", "length_unit",
                  "stamp_valid", "stamp_ns", "time_basis", "scene_revision",
                  "request_id"):
        setattr(near.identity, field, getattr(slots[0].identity, field))
    near.reference = str(slots[0].reference)
    near.orientation_semantics = str(slots[0].orientation_semantics)
    near.pose.position.x = -0.55
    near.pose.position.y = 0.0
    near.pose.position.z = float(slots[0].pose.position.z)
    near.pose.orientation.w = 1.0
    scene_cfg = producer.scene_cfg
    stamp = int(node.get_clock().now().nanoseconds)
    # The select request id must match the slots' identity (the placement
    # request id), exactly as the whole-chain test does.
    slot_request = str(slots[0].identity.request_id or "") or "freeze-p"
    start_tcp = tcp_record(slot_request, scene_revision,
                           (0.45, 0.0, 1.15), (1.0, 0.0, 0.0, 0.0))
    start_tcp["stamp_ns"] = stamp
    grasp = grasp_record(slot_request, scene_revision, [0.0, 0.0, 0.10],
                         [1.0, 0.0, 0.0, 0.0])
    grasp["stamp_ns"] = stamp
    # Build the SERVICE request exactly as the adapter does, then convert
    # through place_service.request_dict — the same path the selector
    # node's producer consumes.
    source, target, translation, quaternion = container_transform_fields(
        scene_cfg)
    service = SelectPlacePathV2.Request()
    service.request_id = slot_request
    service.box_wdh = [0.35, 0.25, 0.20]
    for slot_message in slots + [near]:
        service.slots.append(slot_message)
    service.slot_scores = [float(v) for v in placement.candidate_scores] + [0.1]
    service.surface = surface
    service.inner_size = [float(v) for v in
                          container_usable_dimensions(scene_cfg)]
    service.geometry_hash = str(surface.geometry_hash or "")
    service.map_revision = int(surface.map_revision)
    service.start_tcp = pose_message(start_tcp)
    service.start_joints = [float(v) for v in joints()]
    service.grasp = pose_message(grasp)
    service.container_source_frame = str(source)
    service.container_target_frame = str(target)
    service.container_translation = [float(v) for v in translation]
    service.container_quaternion_xyzw = [float(v) for v in quaternion]
    service.container_stamp_ns = 0
    service.container_stamp_is_null = True
    service.excluded_ids = []
    request = request_dict(service)

    # patch the cartesian client lazily: wrap at the first probe. The
    # adapter creates its client inside _ensure_verdicts, so wrap the
    # factory product right after plan_place_route builds it — instead,
    # pre-wrap by monkeypatching _ensure_clients on the class for this run.
    from luggage_planning.moveit_state_validity import (
        MoveItStateValidityAdapter,
    )

    original_ensure_clients = MoveItStateValidityAdapter._ensure_clients

    def ensure_clients_and_wrap(inner_self):
        original_ensure_clients(inner_self)
        if not getattr(inner_self, "_freeze_wrapped", False):
            inner_self._freeze_wrapped = True
            original = inner_self._cartesian.call_async

            def recorded(request_message, _original=original):
                entry = {"kind": "batch_probe",
                         "at_ns": time.monotonic_ns(),
                         "request": cartesian_request_json(request_message)}
                record.append(entry)
                future = _original(request_message)

                def on_done(done):
                    try:
                        entry["response"] = cartesian_response_json(
                            done.result())
                    except Exception as error:  # noqa: BLE001
                        entry["response_error"] = repr(error)

                future.add_done_callback(on_done)
                return future

            inner_self._cartesian.call_async = recorded

    MoveItStateValidityAdapter._ensure_clients = ensure_clients_and_wrap
    try:
        from luggage_planning.route_producer import register_attach_from_request

        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        attachment = producer.attachment_provider.latest_record(slot_request)
        result = producer.plan_place_route(request)
    finally:
        MoveItStateValidityAdapter._ensure_clients = original_ensure_clients

    # -- the live planning scene at select time ---------------------------
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene

    scene_client = node.create_client(GetPlanningScene,
                                      "/get_planning_scene")
    scene_msg = None
    if scene_client.wait_for_service(timeout_sec=5.0):
        scene_request = GetPlanningScene.Request()
        scene_request.components.components = 255
        future = scene_client.call_async(scene_request)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.1)
        if future.done():
            scene_msg = future.result().scene
        scene_client.destroy()

    # -- dump --------------------------------------------------------------
    result_json = {
        k: v for k, v in result.items()
        if k not in ("legs",)} if isinstance(result, dict) else str(result)
    if isinstance(result, dict):
        result_json["legs"] = [
            {"name": leg.get("name"),
             "joint_trajectory": leg.get("joint_trajectory")}
            for leg in result.get("legs", [])]
    frozen_edges = dict(capturing_validity.edges)
    frozen_samples = {
        edge_id: [list(q) for q in samples]
        for edge_id, samples in capturing_validity.samples.items()}
    frozen_scene_states = dict(capturing_validity.scene_states)
    scene_blob = None
    if scene_msg is not None:
        import base64

        from rclpy.serialization import serialize_message

        scene_blob = base64.b64encode(
            serialize_message(scene_msg)).decode("ascii")
    fk_samples = {}
    for edge_id, samples in capturing_validity.samples.items():
        fk_samples[edge_id] = [
            [list(fk(q)[0]), list(fk(q)[1])] for q in samples]

    artifacts = {
        "meta": {
            "frozen_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "git_head": git_head,
            "git_dirty": git_dirty,
            "install_hashes": install_files,
            "urdf_param_sha256": sha256_text(urdf_text),
            "srdf_param_sha256": sha256_text(srdf_text),
            "urdf_param_bytes": len(urdf_text),
            "probe_node": "motion_occ_driver",
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        },
        "identity": {k: str(v) for k, v in runtime.identity.items()},
        "joints": {"before_reposition": q_before,
                   "at_select": q_at_select,
                   "after_select": joints()},
        "placement": placement_summary,
        "request": request,
        "attachment": None if attachment is None else {
            "request_id": str(attachment.request_id),
            "t_tcp_box_translation": list(attachment.t_tcp_box_translation),
            "t_tcp_box_quaternion_xyzw": list(
                attachment.t_tcp_box_quaternion_xyzw),
            "dimensions_wdh": list(attachment.dimensions_wdh),
            "attachment_incarnation": int(
                getattr(attachment, "attachment_incarnation", 0)),
            "digest": attachment.digest(),
        },
        "result": result_json,
        "edges": frozen_edges,
        "samples": frozen_samples,
        "sample_fk": fk_samples,
        "edge_scene_states": frozen_scene_states,
        "scene_at_select": None if scene_msg is None else scene_json(
            scene_msg),
        "scene_blob_b64": scene_blob,
        "record": record,
    }
    out_path = out_dir / "frozen_failure.json"
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(artifacts, handle, indent=1, default=str)
    print("wrote", out_path)
    print("result.success=%r reason=%r" % (
        result.get("success") if isinstance(result, dict) else None,
        result.get("reason_code") if isinstance(result, dict) else None))
    probes = [entry for entry in record if entry.get("kind") == "batch_probe"]
    print("probes captured: %d" % len(probes))
    for entry in probes[:8]:
        response = entry.get("response", {})
        print("  probe error=%s fraction=%s waypoints=%s" % (
            response.get("error_code"), response.get("fraction"),
            entry["request"].get("n_waypoints")))
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    # A crashed driver peer poisons the runtime lease for the whole
    # stack (PEER_LEASE_EXPIRED is sticky), so never propagate: dump the
    # partial record and exit cleanly.
    try:
        sys.exit(main())
    except Exception as error:  # noqa: BLE001
        import traceback

        traceback.print_exc()
        try:
            fallback = Path(
                __import__("__main__").__dict__.get(
                    "_freeze_out", ".")) / "frozen_failure.partial.json"
        except Exception:  # noqa: BLE001
            fallback = Path("/tmp/frozen_failure.partial.json")
        with open(fallback, "w", encoding="utf-8") as handle:
            json.dump({"error": repr(error)}, handle, indent=1)
        sys.exit(1)
