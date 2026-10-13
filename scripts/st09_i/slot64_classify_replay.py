#!/usr/bin/env python3
"""Production-strategy replay of the captured slot-64 place request.

Does not change budgets, tolerances, atlas use, or candidate order.
Search decisions are the production ones. This script only records the
order keys, the node counts, and the stage times, then diagnoses the
first failing Cartesian edge with collision checking on and off.

Random seeds: Humble exposes no per-request OMPL or KDL seed. The three
repeats share one move_group process. Agreement is on the first failure
stage, not on identical joint values.
"""

from __future__ import division

import argparse
import hashlib
import json
import math
import os
import subprocess
import time
import xml.etree.ElementTree as ET

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CAPTURE = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_p10_capture_repro_20261005")
REQUEST_PATH = os.path.join(CAPTURE, "dumps/box_01_paths/planning_request.json")
SURFACE_PATH = os.path.join(
    CAPTURE, "dumps/trial_01_PLANNING_BUDGET_EXHAUSTED/replay/surface_2d.json")
SCENE_JSON = os.path.join(
    CAPTURE, "dumps/trial_01_PLANNING_BUDGET_EXHAUSTED/scene.json")
SCENE_TF = os.path.join(
    ROOT, "install-st09/luggage_description/share/luggage_description/"
    "config/scene_tf_deploy.yaml")
OUT = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_slot64_classify_20261005")
BASELINE_REPLAY = os.path.join(OUT, "replay_1.json")
REVERSE_OUT = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_reverse_egress_20261005")

# Declared before any diagnosis result is read.
SIGMA_MIN_SINGULAR = 0.02
STEP_JUMP_RAD = 0.5
JOINT_NAMES = ("elfin_joint1", "elfin_joint2", "elfin_joint3",
               "elfin_joint4", "elfin_joint5", "elfin_joint6")


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git():
    rev = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    dirty = subprocess.check_output(
        ["git", "status", "--porcelain"], cwd=ROOT, text=True)
    return rev, len([line for line in dirty.splitlines() if line.strip()])


def _write_manifest():
    os.makedirs(OUT, exist_ok=True)
    rev, dirty = _git()
    manifest = {
        "commit": rev,
        "dirty_count": dirty,
        "request_sha256": _sha256(REQUEST_PATH),
        "surface_sha256": _sha256(SURFACE_PATH),
        "scene_json_sha256": _sha256(SCENE_JSON),
        "scene_tf": SCENE_TF,
        "scene_tf_sha256": _sha256(SCENE_TF),
        "seed_control": (
            "no per-request OMPL/KDL seed in Humble; one move_group "
            "process for all repeats; gate is the first failure stage"),
        "sigma_min_singular": SIGMA_MIN_SINGULAR,
        "step_jump_rad": STEP_JUMP_RAD,
        "planning_behavior": "unchanged; trace is append-only",
        "inputs": {
            "request": REQUEST_PATH,
            "surface": SURFACE_PATH,
            "scene": SCENE_JSON,
        },
    }
    with open(os.path.join(OUT, "manifest.json"), "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=1, sort_keys=True)
        handle.write("\n")
    return manifest


def _request_document(request_path, surface_path, scene_path, slot_index):
    with open(request_path, encoding="utf-8") as handle:
        document = json.load(handle)
    with open(surface_path, encoding="utf-8") as handle:
        surface = json.load(handle)
    with open(scene_path, encoding="utf-8") as handle:
        scene = json.load(handle)
    slot = next(row for row in document["candidates"]
                if row.get("slot_index") == int(slot_index))
    return document, surface, scene, slot


def _build_request(document, surface, slot, scene_revision, stamp_ns):
    from st09_contracts.attachment import invert_transform

    attachment = document["attachment_transform"]
    grasp_pos, grasp_quat = invert_transform(
        tuple(attachment["translation"]),
        tuple(attachment["quaternion_xyzw"]))
    return {
        "request_id": "slot64-classify",
        "scene_revision": scene_revision,
        "box_wdh": [float(v) for v in document["box_wdh"]],
        "box_id": "slot64-classify",
        "generation": 1,
        "slots": [{
            "request_id": "place-v2-2",
            "position": [float(v) for v in slot["slot_pose"]["position"]],
            "quaternion_xyzw": [
                float(v) for v in slot["slot_pose"]["quaternion_xyzw"]],
        }],
        "surface": {
            "frame_id": surface.get("frame") or "container_local",
            "resolution_m": float(surface["resolution"]),
            "origin_xy_container": [
                float(v) for v in surface["origin_local"]],
            "state": surface["state"],
            "stamp_ns": int(surface.get("acquisition_stamp_ns") or stamp_ns),
            "map_source": surface.get("map_source") or "measured",
            "placed_count": int(surface.get("placed_count") or 0),
            "cargo_observed": bool(surface.get("cargo_observed")),
            "commit_revision": int(surface.get("map_revision") or 2),
        },
        "inner_size": [float(v) for v in surface["inner_size"]],
        "geometry_hash": document["geometry_hash"],
        "map_revision": int(document["map_revision"]),
        "start_joints": [float(v) for v in document["start_joints"]],
        "grasp": {
            "position": [float(v) for v in grasp_pos],
            "quaternion_xyzw": [float(v) for v in grasp_quat],
            "stamp_ns": int(stamp_ns),
        },
        "container_transform": document["container_transform"],
    }


def _applied(result):
    if isinstance(result, tuple):
        return bool(result[0])
    return bool(result)


def _sync_world(node, scene_config, scene_json):
    from luggage_description.scene_mesh_utils import (
        container_collision_mesh_path,
        container_model_name,
        require_existing_mesh,
    )
    from luggage_description.scene_tf_config_utils import (
        origin_in_world,
        pedestal_collision_pose_in_world,
        pedestal_dimensions,
        pedestal_enabled,
    )
    from luggage_description.scene_tf_publisher import rpy_to_quaternion
    from luggage_planning.planning_scene_client import (
        PlanningSceneClient,
        load_stl_mesh_msg,
    )

    client = PlanningSceneClient(node)
    client.wait_ready(timeout_sec=30.0)
    mesh_path = require_existing_mesh(
        container_collision_mesh_path(scene_config))
    mesh = load_stl_mesh_msg(mesh_path, max_faces=8000)
    obj_id = container_model_name(scene_config)
    xyz, rpy = origin_in_world(scene_config)
    client.remove_object(obj_id)
    applied = client.add_collision_mesh(
        obj_id, xyz, rpy_to_quaternion(rpy), mesh, frame_id="world")
    if not _applied(applied):
        raise RuntimeError("container mesh apply failed: %r" % (applied,))
    client.remove_object("robot_pedestal")
    if pedestal_enabled(scene_config):
        center, ped_rpy = pedestal_collision_pose_in_world(scene_config)
        length, width, height = pedestal_dimensions(scene_config)
        ok = client.add_collision_box(
            "robot_pedestal", center, rpy_to_quaternion(ped_rpy),
            (length, width, height), frame_id="world")
        if not _applied(ok):
            raise RuntimeError("pedestal apply failed: %r" % (ok,))
        from luggage_planning.moveit_state_validity import (
            DECLARED_STRUCTURE_PAIRS,
        )

        allowed, message = client.set_acm_pairs(
            DECLARED_STRUCTURE_PAIRS, True)
        if not allowed:
            raise RuntimeError("pedestal structure acm failed: %s" % message)
    for obj in scene_json.get("world_objects") or []:
        if obj.get("type") != "box" or obj.get("id") == "robot_pedestal":
            continue
        client.remove_object(obj["id"])
        ok = client.add_collision_box(
            obj["id"], obj["xyz"], obj["quat"], obj["size"], frame_id="world")
        if not _applied(ok):
            raise RuntimeError("placed box apply failed: %s %r" % (obj["id"], ok))
    return {
        "container": obj_id,
        "mesh_triangles": len(mesh.triangles),
        "world_boxes": [
            obj["id"] for obj in scene_json.get("world_objects") or []
            if obj.get("type") == "box"],
    }


def _producer(node, scene_config, scene_path, atlas, stamp_ns, budget_s=None):
    from luggage_description.scene_tf_config_utils import (
        container_opening_normal_in_world,
        container_opening_target_point_in_world,
        pedestal_dimensions,
        pedestal_enabled,
    )
    from luggage_planning.attachment_provider import MeasuredAttachmentProvider
    from luggage_planning.map_revalidation import MapRevalidator
    from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter
    from luggage_planning.moveit_state_validity import MoveItStateValidityAdapter
    from luggage_planning.offline_ik import load_urdf, refine_to_pose
    from luggage_planning.planning_scene_client import PlanningSceneClient
    from luggage_planning.reachable_graph import RouteGraph, make_offline_fk
    from luggage_planning.route_producer import (
        ROUTE_TOTAL_WALL_NS,
        RouteProducer,
        certification_policy_from_contract,
        revalidation_policy_from_contract,
    )
    from luggage_planning.trajectory_certifier import TrajectoryCertifier
    from ament_index_python.packages import get_package_share_directory

    contract_path = os.path.join(
        get_package_share_directory("st09_contracts"),
        "resolved_planning_contract.json")
    with open(contract_path, encoding="utf-8") as handle:
        contract = json.load(handle)
    urdf = load_urdf(scene_tf_config=scene_path)
    tcp_path = urdf.path_to("suction_contact_frame")
    fk = make_offline_fk(urdf, tcp_path)
    scene_client = PlanningSceneClient(node)

    def refine(position, quaternion, seed):
        return refine_to_pose(
            urdf, tcp_path, position, quaternion, seed,
            pos_tol=2e-3, rot_tol=2e-2)

    def atlas_prior(targets, request):
        from luggage_planning.atlas_prior import prior_for_targets
        if atlas is None or not targets:
            return None
        return prior_for_targets(
            targets, request["container_transform"], atlas.query)

    def validity_factory(*, scene_states, edges):
        adapter = MoveItStateValidityAdapter(node, fk=fk, scene_client=scene_client)
        adapter.load_edges(edges, scene_states=scene_states)
        return adapter

    producer = RouteProducer(
        graph=RouteGraph(static_identity="slot64-classify"),
        edge_builder=MoveItEdgeAdapter(node, scene_client=scene_client),
        certifier=TrajectoryCertifier(),
        cert_policy=certification_policy_from_contract(contract),
        validity_adapter_factory=validity_factory,
        attachment_provider=MeasuredAttachmentProvider(
            "slot64-classify", "slot64-classify",
            tcp_frame="suction_contact_frame", box_frame="box"),
        revalidator=MapRevalidator(revalidation_policy_from_contract(contract)),
        fk=fk, refine=refine, urdf=urdf, tcp_path=tcp_path,
        clock_ns=lambda: int(stamp_ns) + 1_000_000_000,
        log=lambda message: node.get_logger().info(message),
        opening_info={
            "point": container_opening_target_point_in_world(scene_config),
            "normal": container_opening_normal_in_world(scene_config),
        },
        transit_clearance_z=(
            pedestal_dimensions(scene_config)[2]
            if pedestal_enabled(scene_config) else None),
        atlas_prior=atlas_prior,
        vacuum_state_fn=lambda: {
            "attached": True, "vacuum_on": True,
            "received_ns": int(stamp_ns) + 1_000_000_000,
        },
        scene_config=scene_config,
        scene_revision_binding="slot64-classify",
        budget_ns=(ROUTE_TOTAL_WALL_NS if budget_s is None
                   else int(float(budget_s) * 1e9)))
    producer.edge_builder.wait_ready(timeout_sec=60.0)
    return producer, urdf, tcp_path


def _wrap_timing(producer):
    buckets = {"refine_s": 0.0, "edge_s": 0.0, "refine_calls": 0, "edge_calls": 0}
    in_edge = [0]
    refine = producer.refine
    builder = producer.edge_builder

    def timed_refine(*args, **kwargs):
        started = time.perf_counter()
        try:
            return refine(*args, **kwargs)
        finally:
            buckets["refine_s"] += time.perf_counter() - started
            buckets["refine_calls"] += 1

    class _Timed(object):
        def __getattr__(self, name):
            return getattr(builder, name)

        def __call__(self, *args, **kwargs):
            in_edge[0] += 1
            started = time.perf_counter()
            try:
                return builder(*args, **kwargs)
            finally:
                in_edge[0] -= 1
                buckets["edge_s"] += time.perf_counter() - started
                buckets["edge_calls"] += 1

    producer.refine = timed_refine
    producer.edge_builder = _Timed()
    return buckets


def _first_failure(trace, failure_log):
    for attempt in trace.attempts:
        if attempt["built"]:
            continue
        return {
            "edge_class": attempt["edge_class"],
            "frontier_region": attempt["frontier_region"],
            "dest_region": attempt["dest_region"],
            "frontier_j6": attempt["frontier_q"][5],
            "dest_j6": attempt["dest_q"][5],
            "log_reason": (failure_log[0].get("reason") if failure_log else ""),
        }
    if failure_log:
        row = failure_log[0]
        return {
            "edge_class": row.get("edge_class"),
            "frontier_region": "",
            "dest_region": "",
            "log_reason": row.get("reason"),
        }
    return {"edge_class": "none", "log_reason": ""}


def _growth(trace):
    series = []
    for row in trace.regions:
        series.append({
            "when": row["when"],
            "n": row["n"],
            "slot_approach": row["counts"].get("slot_approach", 0),
            "slot_upper": row["counts"].get("slot_upper", 0),
        })
    added = []
    approach = None
    for attempt in trace.attempts:
        if attempt["edge_class"] != "TRAVERSE":
            continue
        if approach is None:
            approach = 0
        if attempt["new_node_ids"]:
            added.append({
                "dest_region": attempt["dest_region"],
                "new_node_ids": attempt["new_node_ids"],
                "built": attempt["built"],
            })
    traverse_orders = [
        row["n_dest"] for row in trace.orders if row["edge_class"] == "TRAVERSE"
        and row["to_region"] == "slot_approach"]
    return {
        "regions": series,
        "endpoint_nodes_on_traverse": added[:20],
        "endpoint_node_count": len(added),
        "slot_approach_dest_counts": traverse_orders[:30],
    }


def _first_insert(trace):
    for row in trace.orders:
        if row["edge_class"] != "INSERT" or not row["dests"]:
            continue
        first = row["dests"][0]
        return {
            "frontier_q": row["frontier_q"],
            "frontier_region": row["frontier_region"],
            "n_dest": row["n_dest"],
            "first_q": first["q"],
            "pair_vs_frontier": first["pair_vs_frontier"],
            "best_vs_goals": first["best_vs_goals"],
        }
    return None


def _kdl_chain(urdf_xml):
    import PyKDL
    root = ET.fromstring(urdf_xml)
    joints = {}
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        axis = joint.find("axis")
        xyz = (0.0, 0.0, 0.0)
        rpy = (0.0, 0.0, 0.0)
        if origin is not None:
            xyz = tuple(float(v) for v in origin.get("xyz", "0 0 0").split())
            rpy = tuple(float(v) for v in origin.get("rpy", "0 0 0").split())
        ax = (0.0, 0.0, 1.0)
        if axis is not None:
            ax = tuple(float(v) for v in axis.get("xyz", "0 0 1").split())
        joints[joint.get("name")] = (xyz, rpy, ax, joint.get("type"))
    chain = PyKDL.Chain()
    for name in JOINT_NAMES:
        xyz, rpy, ax, _kind = joints[name]
        frame = PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))
        joint = PyKDL.Joint(
            name, PyKDL.Vector(0, 0, 0), PyKDL.Vector(*ax), PyKDL.Joint.RotAxis)
        chain.addSegment(PyKDL.Segment(joint, frame))
    return chain


def _sigma_min(chain, q):
    import numpy as np
    import PyKDL
    solver = PyKDL.ChainJntToJacSolver(chain)
    joints = PyKDL.JntArray(chain.getNrOfJoints())
    for index, value in enumerate(q):
        joints[index] = float(value)
    jacobian = PyKDL.Jacobian(chain.getNrOfJoints())
    if solver.JntToJac(joints, jacobian) != 0:
        return None
    rows = []
    for row in range(jacobian.rows()):
        rows.append([jacobian[row, col] for col in range(jacobian.columns())])
    values = np.linalg.svd(np.array(rows), compute_uv=False)
    return float(values[-1])


def _classify_steps(urdf, tcp_path, chain, limits, start_q, poses):
    from luggage_planning.offline_ik import refine_to_pose

    def wrap(delta):
        return (float(delta) + math.pi) % (2.0 * math.pi) - math.pi

    previous = list(start_q)
    steps = []
    cause = ""
    for index, (position, quaternion) in enumerate(poses, start=1):
        joints, info = refine_to_pose(
            urdf, tcp_path, position, quaternion, previous,
            pos_tol=2e-3, rot_tol=2e-2)
        sigma = _sigma_min(chain, previous)
        row = {"index": index, "sigma_min_at_previous": sigma, "ik": joints is not None}
        if joints is None:
            row["cause"] = (
                "SINGULAR_REGION" if sigma is not None and sigma < SIGMA_MIN_SINGULAR
                else "IK_CONTINUITY_FAILED")
            steps.append(row)
            cause = cause or row["cause"]
            break
        outside = [
            name for name, value, (lo, hi) in zip(JOINT_NAMES, joints, limits)
            if value < lo - 1e-6 or value > hi + 1e-6]
        jump = max(abs(wrap(b - a)) for a, b in zip(previous, joints))
        row["jump_rad"] = round(jump, 4)
        row["q"] = [round(float(v), 4) for v in joints]
        if outside:
            row["cause"] = "JOINT_LIMIT"
            row["joints"] = outside
        elif jump > STEP_JUMP_RAD:
            row["cause"] = "BRANCH_DISCONTINUITY"
        elif sigma is not None and sigma < SIGMA_MIN_SINGULAR:
            row["cause"] = "SINGULAR_REGION"
        steps.append(row)
        if row.get("cause"):
            cause = cause or row["cause"]
            break
        previous = list(joints)
    return {"cause": cause or "IK_CONTINUOUS", "steps": steps, "n_ok": sum(
        1 for row in steps if row.get("ik") and not row.get("cause"))}


def _cartesian_fraction(node, from_q, poses, avoid):
    from geometry_msgs.msg import Point, Pose, Quaternion
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetCartesianPath
    from sensor_msgs.msg import JointState

    client = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    if not client.wait_for_service(timeout_sec=15.0):
        raise RuntimeError("compute_cartesian_path unavailable")
    request = GetCartesianPath.Request()
    request.header.frame_id = "world"
    request.group_name = "elfin_arm"
    request.link_name = "suction_contact_frame"
    request.max_step = 0.01
    request.jump_threshold = 0.0
    request.avoid_collisions = bool(avoid)
    start = RobotState()
    start.is_diff = True
    start.joint_state = JointState()
    start.joint_state.name = list(JOINT_NAMES)
    start.joint_state.position = [float(v) for v in from_q]
    request.start_state = start
    for position, quaternion in poses:
        pose = Pose()
        pose.position = Point(x=float(position[0]), y=float(position[1]),
                              z=float(position[2]))
        pose.orientation = Quaternion(
            x=float(quaternion[0]), y=float(quaternion[1]),
            z=float(quaternion[2]), w=float(quaternion[3]))
        request.waypoints.append(pose)
    future = client.call_async(request)
    deadline = time.monotonic() + 30.0
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not future.done() or future.result() is None:
        return {"fraction": None, "error": "timeout"}
    response = future.result()
    trajectory = response.solution.joint_trajectory.points
    return {
        "fraction": round(float(response.fraction), 6),
        "points": len(trajectory),
        "positions": [[float(v) for v in point.positions] for point in trajectory],
    }


def _contacts(node, q):
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity
    from sensor_msgs.msg import JointState

    client = node.create_client(GetStateValidity, "/check_state_validity")
    if not client.wait_for_service(timeout_sec=10.0):
        return {"error": "check_state_validity unavailable"}
    request = GetStateValidity.Request()
    request.group_name = "elfin_arm"
    state = RobotState()
    state.joint_state = JointState()
    state.joint_state.name = list(JOINT_NAMES)
    state.joint_state.position = [float(v) for v in q]
    request.robot_state = state
    future = client.call_async(request)
    deadline = time.monotonic() + 15.0
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not future.done() or future.result() is None:
        return {"error": "timeout"}
    response = future.result()
    pairs = []
    for contact in response.contacts:
        pairs.append([contact.contact_body_1, contact.contact_body_2])
    return {"valid": bool(response.valid), "contacts": pairs[:8]}


def _diagnose(node, builder, urdf, tcp_path, trace, chain, attachment, box_wdh):
    from luggage_planning.moveit_edge_adapter import (
        PayloadSpec,
        SceneState,
        synthesize_waypoints,
    )

    builder._ensure_scene(SceneState(
        load_state="carrying",
        payload=PayloadSpec(
            dimensions_wdh=tuple(float(v) for v in box_wdh),
            t_tcp_box_translation=tuple(attachment["translation"]),
            t_tcp_box_quaternion_xyzw=tuple(attachment["quaternion_xyzw"]))))
    attempt = next((row for row in trace.attempts if not row["built"]), None)
    if attempt is None:
        return {"status": "no_failed_attempt"}
    poses, _distance = synthesize_waypoints(
        attempt["frontier_pose"][0], attempt["frontier_pose"][1],
        attempt["dest_pose"][0], attempt["dest_pose"][1],
        attempt["edge_class"])
    off = _cartesian_fraction(node, attempt["frontier_q"], poses, False)
    on = _cartesian_fraction(node, attempt["frontier_q"], poses, True)
    limits = list(urdf.joint_limits())
    stepped = _classify_steps(
        urdf, tcp_path, chain, limits, attempt["frontier_q"], poses)
    report = {
        "edge_class": attempt["edge_class"],
        "frontier_q": attempt["frontier_q"],
        "dest_q": attempt["dest_q"],
        "collision_off": off,
        "collision_on": on,
        "stepped_ik": stepped,
    }
    if off.get("fraction") is not None and on.get("fraction") is not None:
        if off["fraction"] >= 0.95 and on["fraction"] < 0.95:
            report["split"] = "COLLISION_ALONG_PATH"
            contacts = []
            for position, quaternion in poses[:3]:
                from luggage_planning.offline_ik import refine_to_pose
                joints, _info = refine_to_pose(
                    urdf, tcp_path, position, quaternion,
                    list(attempt["frontier_q"]), pos_tol=2e-3, rot_tol=2e-2)
                if joints is None:
                    break
                contacts.append(_contacts(node, joints))
            report["contacts"] = contacts
        elif off["fraction"] < 0.95:
            report["split"] = stepped["cause"]
        else:
            report["split"] = "both_passed"
    return report


def _descend_and_exit(document, slot, scene_config):
    """Geometric exit and the joints the baseline retreat actually started from."""
    from luggage_description.scene_tf_config_utils import (
        container_opening_normal_in_world,
        container_opening_target_point_in_world,
        pedestal_dimensions,
        pedestal_enabled,
    )
    from luggage_planning.route_producer import anchor_plan

    replay = json.load(open(os.path.join(OUT, "replay_1.json"), encoding="utf-8"))
    retreat = next(
        row for row in replay["orders"] if row.get("edge_class") == "RETREAT")
    descend = next(
        row for row in replay["attempts"]
        if row.get("edge_class") == "DESCEND" and row.get("built"))
    contact = tuple(descend["dest_pose"][0])
    orientation = tuple(descend["dest_pose"][1])
    inward = [-float(v) for v in container_opening_normal_in_world(scene_config)]
    plan = anchor_plan(
        contact, document["container_transform"]["quaternion_xyzw"],
        float(document["box_wdh"][2]),
        opening=container_opening_target_point_in_world(scene_config),
        inward_normal=inward,
        transit_clearance_z=(
            pedestal_dimensions(scene_config)[2]
            if pedestal_enabled(scene_config) else None))
    return {
        "start_q": [float(v) for v in retreat["frontier_q"]],
        "contact": [float(v) for v in contact],
        "orientation": [float(v) for v in orientation],
        "exit": [float(v) for v in plan.exit],
        "slot_pose": slot["slot_pose"],
    }


def _apply_unloaded(builder, document, slot):
    """Detached tool, released box at the candidate, no second payload."""
    from luggage_planning.moveit_edge_adapter import BoxSpec, SceneState
    from st09_contracts.attachment import compose_transform

    transform = document["container_transform"]
    pose = slot["slot_pose"]
    center, quaternion = compose_transform(
        tuple(transform["translation"]), tuple(transform["quaternion_xyzw"]),
        tuple(pose["position"]), tuple(pose["quaternion_xyzw"]))
    builder._ensure_scene(SceneState(
        load_state="unloaded",
        released_box=BoxSpec(
            object_id="released-slot64-classify",
            center=tuple(center),
            size_whd=tuple(float(v) for v in document["box_wdh"]),
            quaternion_xyzw=tuple(quaternion))))
    return {
        "released_center": [float(v) for v in center],
        "released_quaternion": [float(v) for v in quaternion],
    }


def _pose_error(fk, joints, target):
    position, quaternion = fk(joints)
    delta = math.sqrt(sum(
        (float(position[i]) - float(target[i])) ** 2 for i in range(3)))
    return {
        "fk_position": [round(float(v), 6) for v in position],
        "fk_quaternion": [round(float(v), 6) for v in quaternion],
        "position_error_m": round(delta, 6),
    }


def _first_failed_waypoint(fraction, waypoint_count):
    from luggage_planning.moveit_edge_adapter import cartesian_failure_detail
    if fraction is None or fraction >= 0.95:
        return None
    detail = cartesian_failure_detail(
        [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0],
        [0, 0, 0], [0, 0, 0], fraction, 0.95, waypoint_count)
    return {
        "fraction": detail["fraction"],
        "failed_waypoint_index": detail["failed_waypoint_index"],
        "waypoint_count": detail["waypoint_count"],
    }


def run_retreat_probe(node, scene_config, document, slot, urdf, tcp_path):
    """Cartesian retreat from the captured descend endpoint. No search change."""
    from luggage_planning.moveit_edge_adapter import (
        MoveItEdgeAdapter,
        synthesize_waypoints,
    )
    from luggage_planning.planning_scene_client import PlanningSceneClient
    from luggage_planning.reachable_graph import make_offline_fk

    geometry = _descend_and_exit(document, slot, scene_config)
    fk = make_offline_fk(urdf, tcp_path)
    start_pose = _pose_error(fk, geometry["start_q"], geometry["exit"])
    poses, distance = synthesize_waypoints(
        start_pose["fk_position"], start_pose["fk_quaternion"],
        geometry["exit"], geometry["orientation"], "RETREAT")
    adapter = MoveItEdgeAdapter(node, scene_client=PlanningSceneClient(node))
    adapter.wait_ready(timeout_sec=60.0)
    released = _apply_unloaded(adapter, document, slot)
    off = _cartesian_fraction(node, geometry["start_q"], poses, False)
    on = _cartesian_fraction(node, geometry["start_q"], poses, True)
    report = {
        "start_q": geometry["start_q"],
        "start_fk": start_pose,
        "exit_position": geometry["exit"],
        "exit_orientation_booked": geometry["orientation"],
        "path_length_m": round(float(distance), 6),
        "waypoint_count": len(poses),
        "orientation_mode": "hold_source",
        "released_box": released,
        "collision_off": {k: off[k] for k in ("fraction", "points") if k in off},
        "collision_on": {k: on[k] for k in ("fraction", "points") if k in on},
        "collision_off_endpoint": None,
        "collision_on_endpoint": None,
        "failed_waypoint_off": _first_failed_waypoint(
            off.get("fraction"), len(poses)),
        "failed_waypoint_on": _first_failed_waypoint(
            on.get("fraction"), len(poses)),
    }
    if off.get("positions"):
        report["collision_off_endpoint"] = _pose_error(
            fk, off["positions"][-1], geometry["exit"])
    if on.get("positions"):
        report["collision_on_endpoint"] = _pose_error(
            fk, on["positions"][-1], geometry["exit"])
    off_fraction = off.get("fraction")
    on_fraction = on.get("fraction")
    if off_fraction is not None and off_fraction < 0.95:
        chain = _kdl_chain  # filled by caller if needed
        report["ik_note"] = "collision-off below 0.95; stepped IK follows"
        report["stepped_ik"] = None
        report["_poses"] = poses
        report["_off_positions"] = off.get("positions")
    elif off_fraction is not None and off_fraction >= 0.95 and (
            on_fraction is None or on_fraction < 0.95):
        contacts = []
        for joints in off.get("positions") or []:
            verdict = _contacts(node, joints)
            contacts.append(verdict)
            if verdict.get("valid") is False:
                break
        report["contacts_along_collision_off_path"] = contacts
    report["branch"] = (
        "build_from_endpoint" if (
            on_fraction is not None and on_fraction >= 0.95)
        else "do_not_add_builder")
    adapter._scene.remove_object("released-slot64-classify")
    adapter.reset_scene()
    return report


def _sigma_urdf(urdf, tcp_path, q):
    """Smallest singular value of a finite-difference Jacobian of urdf.tcp."""
    import numpy as np
    from scipy.spatial.transform import Rotation as Rot

    base = urdf.tcp(q, tcp_path)
    jacobian = np.zeros((6, 6))
    delta = 1e-6
    for index in range(6):
        nudged = [float(v) for v in q]
        nudged[index] += delta
        tf = urdf.tcp(nudged, tcp_path)
        jacobian[:3, index] = (tf[:3, 3] - base[:3, 3]) / delta
        rot = (Rot.from_matrix(base[:3, :3]).inv()
               * Rot.from_matrix(tf[:3, :3])).as_rotvec()
        jacobian[3:, index] = np.asarray(rot, dtype=float) / delta
    return float(np.linalg.svd(jacobian, compute_uv=False)[-1])


def _path_sigma(urdf, tcp_path, positions):
    if not positions:
        return None
    count = len(positions)
    stride = max(1, count // 16)
    indexes = list(range(0, count, stride))
    if indexes[-1] != count - 1:
        indexes.append(count - 1)
    values = []
    for index in indexes:
        sigma = _sigma_urdf(urdf, tcp_path, positions[index])
        values.append({"index": index, "sigma_min": round(sigma, 6)})
    smallest = min(values, key=lambda row: row["sigma_min"])
    return {"samples": values, "min": smallest}


def _slim_cartesian(result):
    return {key: result[key] for key in ("fraction", "points", "error")
            if key in result}


def _release_anchors(document, scene_config):
    """Captured DESCEND endpoint plus the entry-corridor anchor positions."""
    from luggage_description.scene_tf_config_utils import (
        container_opening_normal_in_world,
        container_opening_target_point_in_world,
        pedestal_dimensions,
        pedestal_enabled,
    )
    from luggage_planning.route_producer import anchor_plan

    replay = json.load(open(BASELINE_REPLAY, encoding="utf-8"))
    retreat = next(
        row for row in replay["orders"] if row.get("edge_class") == "RETREAT")
    descend = next(
        row for row in replay["attempts"]
        if row.get("edge_class") == "DESCEND" and row.get("built"))
    contact = tuple(descend["dest_pose"][0])
    inward = [-float(v) for v in container_opening_normal_in_world(scene_config)]
    plan = anchor_plan(
        contact, document["container_transform"]["quaternion_xyzw"],
        float(document["box_wdh"][2]),
        opening=container_opening_target_point_in_world(scene_config),
        inward_normal=inward,
        transit_clearance_z=(
            pedestal_dimensions(scene_config)[2]
            if pedestal_enabled(scene_config) else None))
    anchors = [
        ("slot_upper", plan.slot_upper),
        ("slot_approach", plan.slot_approach),
        ("portal", plan.portal),
    ]
    if plan.staging is not None:
        anchors.append(("staging", plan.staging))
    return {
        "start_q": [float(v) for v in retreat["frontier_q"]],
        "contact": [float(v) for v in contact],
        "anchors": anchors,
        "staging_present": plan.staging is not None,
    }


def run_reverse_probe(node, scene_config, document, slot, urdf, tcp_path):
    """Unloaded Cartesian reverse of the entry corridor, one segment at a time.

    Each hop starts at the previous collision-on endpoint. A segment that
    misses 0.95 stops the probe. The released box is removed afterwards.
    """
    from luggage_planning.moveit_edge_adapter import (
        MoveItEdgeAdapter,
        synthesize_waypoints,
    )
    from luggage_planning.offline_ik import refine_to_pose
    from luggage_planning.planning_scene_client import PlanningSceneClient
    from luggage_planning.reachable_graph import make_offline_fk

    geometry = _release_anchors(document, scene_config)
    fk = make_offline_fk(urdf, tcp_path)
    adapter = MoveItEdgeAdapter(node, scene_client=PlanningSceneClient(node))
    adapter.wait_ready(timeout_sec=60.0)
    released = _apply_unloaded(adapter, document, slot)
    current_q = list(geometry["start_q"])
    segments = []
    passed = True
    stop = None
    try:
        for region, target in geometry["anchors"]:
            position, quaternion = fk(current_q)
            joints, info = refine_to_pose(
                urdf, tcp_path, target, quaternion, current_q,
                pos_tol=2e-3, rot_tol=2e-2)
            poses, distance = synthesize_waypoints(
                position, quaternion, target, quaternion, "RETREAT")
            off = _cartesian_fraction(node, current_q, poses, False)
            on = _cartesian_fraction(node, current_q, poses, True)
            on_positions = on.get("positions") or []
            row = {
                "to_region": region,
                "target": [round(float(v), 6) for v in target],
                "path_length_m": round(float(distance), 6),
                "waypoint_count": len(poses),
                "orientation_mode": "hold_source",
                "refine": {
                    "ok": joints is not None,
                    "pos_err_m": (info or {}).get("pos_err_m"),
                    "rot_err_rad": (info or {}).get("rot_err_rad"),
                },
                "collision_off": _slim_cartesian(off),
                "collision_on": _slim_cartesian(on),
                "sigma_min": _path_sigma(urdf, tcp_path, on_positions),
            }
            if off.get("positions"):
                row["collision_off_endpoint"] = _pose_error(
                    fk, off["positions"][-1], target)
            if on_positions:
                row["collision_on_endpoint"] = _pose_error(
                    fk, on_positions[-1], target)
            on_fraction = on.get("fraction")
            if joints is None:
                row["cause"] = "IK_REFINE_MISS"
                segments.append(row)
                passed = False
                stop = region
                break
            if on_fraction is None or on_fraction < 0.95:
                if on_positions:
                    row["cartesian_endpoint_q"] = [
                        round(float(v), 4) for v in on_positions[-1]]
                # The edge builder accepts one waypoint when the hop is
                # shorter than its spacing. MoveIt still interpolates at
                # 0.01 m, so name the first 0.01 m sample the 2 mm refine
                # misses. This does not change the gate.
                dense = []
                steps = max(1, int(math.ceil(float(distance) / 0.01)))
                previous = list(current_q)
                for step in range(1, steps + 1):
                    t = float(step) / steps
                    sample = tuple(
                        float(position[i]) + (float(target[i]) - float(position[i])) * t
                        for i in range(3))
                    solved, sample_info = refine_to_pose(
                        urdf, tcp_path, sample, quaternion, previous,
                        pos_tol=2e-3, rot_tol=2e-2)
                    sigma = _sigma_urdf(urdf, tcp_path, previous)
                    dense.append({
                        "step": step,
                        "z": round(sample[2], 6),
                        "refine_ok": solved is not None,
                        "pos_err_m": (sample_info or {}).get("pos_err_m"),
                        "rot_err_rad": (sample_info or {}).get("rot_err_rad"),
                        "sigma_min_at_previous": round(sigma, 6),
                    })
                    if solved is None:
                        break
                    previous = list(solved)
                row["dense_refine"] = dense
                contacts = []
                samples = on_positions[-1:] if on_positions else [joints]
                for sample in samples:
                    contacts.append(_contacts(node, sample))
                row["contacts"] = contacts
                missed = next((item for item in dense if not item["refine_ok"]), None)
                if off.get("fraction") is not None and off["fraction"] >= 0.95 \
                        and (on_fraction is None or on_fraction < 0.95):
                    row["cause"] = "COLLISION_ALONG_PATH"
                elif missed is not None:
                    row["cause"] = "IK_REFINE_MISS"
                    row["first_dense_miss"] = missed
                else:
                    row["cause"] = "CARTESIAN_INTERPOLATION_STOP"
                segments.append(row)
                passed = False
                stop = region
                break
            segments.append(row)
            current_q = [float(v) for v in on_positions[-1]]
    finally:
        adapter._scene.remove_object("released-slot64-classify")
        adapter.reset_scene()
    return {
        "start_q": geometry["start_q"],
        "contact": geometry["contact"],
        "staging_present": geometry["staging_present"],
        "released_box": released,
        "segments": segments,
        "passed": passed and bool(segments),
        "stopped_at": stop,
        "gate": "each collision-on fraction >= 0.95; refine 2 mm / 0.02 rad",
    }


def _dump(path, payload):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=1, sort_keys=True, default=str)
        handle.write("\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--hashes-only", action="store_true")
    parser.add_argument("--request", default=REQUEST_PATH)
    parser.add_argument("--surface", default=SURFACE_PATH)
    parser.add_argument("--scene", default=SCENE_JSON)
    parser.add_argument("--slot-index", type=int, default=64)
    parser.add_argument("--tag", default="")
    parser.add_argument("--retreat-probe", action="store_true")
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument("--budget-s", type=float, default=None)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()
    global OUT
    if args.out:
        OUT = os.path.abspath(args.out)
    elif args.reverse:
        OUT = REVERSE_OUT
    manifest = _write_manifest()
    if args.hashes_only:
        print(json.dumps({"manifest": os.path.join(OUT, "manifest.json")}))
        return 0

    import rclpy
    from rclpy.node import Node
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_planning.atlas_runtime import (
        compute_reachability_scene_hash,
        compute_scene_tf_hash,
        compute_urdf_hash,
        load_atlas_runtime_for_scene,
        load_runtime_atlas,
    )
    from luggage_planning.route_producer import register_attach_from_request
    from luggage_planning.grid_path_search import SearchTrace

    import threading
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    node = Node("slot64_classify")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    scene_config = load_scene_tf_config(SCENE_TF)
    document, surface, scene_json, slot = _request_document(
        args.request, args.surface, args.scene, args.slot_index)
    stamp_ns = int(surface.get("acquisition_stamp_ns") or 1)
    world = _sync_world(node, scene_config, scene_json)
    manifest["world"] = world

    # Match the selector: hash the live robot_description, then load atlas.
    from rcl_interfaces.srv import GetParameters
    getter = node.create_client(GetParameters, "/move_group/get_parameters")
    if not getter.wait_for_service(timeout_sec=30.0):
        raise RuntimeError("move_group parameters unavailable")
    request = GetParameters.Request()
    request.names = ["robot_description"]
    future = getter.call_async(request)
    deadline = time.monotonic() + 20.0
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    urdf_xml = future.result().values[0].string_value
    urdf_hash = compute_urdf_hash(urdf_xml)
    cfg = load_atlas_runtime_for_scene(SCENE_TF, None)
    atlas, diag = load_runtime_atlas(
        cfg, scene_tf_hash=compute_scene_tf_hash(SCENE_TF),
        urdf_hash=urdf_hash,
        reachability_scene_hash=compute_reachability_scene_hash(SCENE_TF))
    manifest["atlas"] = {
        "loaded": atlas is not None,
        "reason": diag.get("reason"),
        "hash_message": diag.get("hash_message"),
        "urdf_hash": urdf_hash,
    }
    _dump(os.path.join(OUT, "manifest.json"), manifest)
    if args.retreat_probe:
        from luggage_planning.offline_ik import load_urdf
        urdf = load_urdf(scene_tf_config=SCENE_TF)
        tcp_path = urdf.path_to("suction_contact_frame")
        report = run_retreat_probe(
            node, scene_config, document, slot, urdf, tcp_path)
        if report.get("stepped_ik") is None and report.get("_poses"):
            report["stepped_ik"] = _classify_steps(
                urdf, tcp_path, _kdl_chain(urdf_xml),
                list(urdf.joint_limits()), report["start_q"], report["_poses"])
        report.pop("_poses", None)
        report.pop("_off_positions", None)
        _dump(os.path.join(OUT, "retreat_probe.json"), report)
        print(json.dumps({
            "branch": report["branch"],
            "off": report["collision_off"],
            "on": report["collision_on"],
            "off_err": (report.get("collision_off_endpoint") or {}).get(
                "position_error_m"),
            "on_err": (report.get("collision_on_endpoint") or {}).get(
                "position_error_m"),
        }, indent=1))
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        return 0
    if args.reverse:
        from luggage_planning.offline_ik import load_urdf
        urdf = load_urdf(scene_tf_config=SCENE_TF)
        tcp_path = urdf.path_to("suction_contact_frame")
        report = run_reverse_probe(
            node, scene_config, document, slot, urdf, tcp_path)
        _dump(os.path.join(OUT, "reverse_segments.json"), report)
        print(json.dumps({
            "passed": report["passed"],
            "stopped_at": report["stopped_at"],
            "segments": [
                {"to": row["to_region"],
                 "off": row["collision_off"].get("fraction"),
                 "on": row["collision_on"].get("fraction"),
                 "refine_ok": row["refine"]["ok"],
                 "cause": row.get("cause"),
                 "endpoint_m": (row.get("collision_on_endpoint") or {}).get(
                     "position_error_m")}
                for row in report["segments"]],
        }, indent=1))
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        return 0 if report["passed"] else 2

    producer, urdf, tcp_path = _producer(
        node, scene_config, SCENE_TF, atlas, stamp_ns,
        budget_s=args.budget_s)
    chain = _kdl_chain(urdf_xml)
    original_refine = producer.refine
    original_builder = producer.edge_builder
    stages = []
    last_trace = None
    for index in range(int(args.repeats)):
        buckets = _wrap_timing(producer)
        trace = SearchTrace()
        producer.search_trace = trace
        request_dict = _build_request(
            document, surface, slot, "slot64-classify", stamp_ns)
        register_attach_from_request(
            producer.attachment_provider, request_dict,
            box_wdh=request_dict["box_wdh"], box_id="slot64-classify",
            generation=1)
        started = time.perf_counter()
        try:
            result = producer.plan_place_route(request_dict)
        finally:
            reset = getattr(producer.edge_builder, "reset_scene", None)
            if callable(reset):
                reset()
            producer.refine = original_refine
            producer.edge_builder = original_builder
        wall = time.perf_counter() - started
        failure_log = list(getattr(
            original_builder, "edge_failure_log", lambda: [])())
        durations = original_builder.call_durations()
        stage = {
            "index": index,
            "reason": result.get("reason_code"),
            "success": bool(result.get("success")),
            "wall_s": round(wall, 3),
            "refine_s": round(buckets["refine_s"], 3),
            "edge_s": round(buckets["edge_s"], 3),
            "other_s": round(wall - buckets["refine_s"] - buckets["edge_s"], 3),
            "refine_calls": buckets["refine_calls"],
            "edge_calls": buckets["edge_calls"],
            "moveit_cartesian_s": round(sum(durations.get("cartesian") or []), 3),
            "moveit_joint_s": round(sum(durations.get("move_group") or []), 3),
            "moveit_scene_s": round(sum(durations.get("scene_apply") or []), 3),
            "first_failure": _first_failure(trace, failure_log),
            "first_insert": _first_insert(trace),
            "growth": _growth(trace),
            "n_orders": len(trace.orders),
            "n_attempts": len(trace.attempts),
        }
        # Durations accumulate on the adapter across repeats. Snapshot the
        # delta by clearing after the copy.
        original_builder._call_durations = {
            key: [] for key in original_builder._call_durations}
        original_builder._failure_log = []
        name = "replay_%s%d.json" % (args.tag + "_" if args.tag else "", index)
        _dump(os.path.join(OUT, name), {
            "stage": stage,
            "orders": trace.orders,
            "attempts": trace.attempts,
            "regions": trace.regions,
            "rows": result.get("rows"),
        })
        stages.append(stage)
        last_trace = trace
        node.get_logger().info(
            "replay %d reason=%s wall=%.1f first=%s" % (
                index, stage["reason"], wall,
                stage["first_failure"].get("edge_class")))

    keys = [
        (row["first_failure"].get("edge_class"),
         row["first_failure"].get("frontier_region"),
         row["first_failure"].get("dest_region"),
         row["first_failure"].get("log_reason"))
        for row in stages]
    diagnosis = _diagnose(
        node, original_builder, urdf, tcp_path, last_trace, chain,
        document["attachment_transform"], document["box_wdh"])
    summary = {
        "repeats": stages,
        "same_first_failure_stage": len(set(keys)) == 1,
        "first_failure_stages": keys,
        "diagnosis": diagnosis,
    }
    _dump(os.path.join(OUT, "summary.json"), summary)
    print(json.dumps({
        "same_first_failure_stage": summary["same_first_failure_stage"],
        "stages": keys,
        "walls": [row["wall_s"] for row in stages],
        "other_s": [row["other_s"] for row in stages],
        "diagnosis_split": diagnosis.get("split"),
    }, indent=1))
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if summary["same_first_failure_stage"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
