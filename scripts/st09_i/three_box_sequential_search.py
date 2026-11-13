#!/usr/bin/env python3
"""Sequential three-box layout search.

The Cartesian corridor scan can pass a pair whose descend sweep still
hits the previous box. This script first records that contact for the
frozen pair, then searches in placement order:

    carryon floor -> commit -> standard floor -> commit -> carryon

A candidate is kept only when the production route is certified and
every Cartesian edge stays inside the existing gates. The old triple is
not a seed.
"""

from __future__ import division

import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS",
             "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import json
import math
import time

from three_box_layout_scan import (  # noqa: E402
    ATLAS_CONFIG,
    BRANCH_TOL_RAD,
    EXCURSION_MAX,
    FRACTION_MIN,
    SCENE,
    _box_world,
    _container_pose,
    _dedupe,
    _feasible,
    _geometry_row,
    _sigma,
    _tcp_world,
)
from capacity_orientation_analysis import (  # noqa: E402
    AtlasStore,
    SceneContext,
    apply_placement,
    floor_prior_surface,
)
from luggage_packing.placement_solver import candidate_aabb  # noqa: E402

OUT = os.path.join(
    os.path.dirname(__file__), "..", "..",
    "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_three_box_layout_20261005")
GEO = "cb41f55f243786d50ab30997291083ab40467507abcf78a00e55da77ffc00856"
START = [-1.389, -1.752, -1.673, -1.287, 1.577, -1.391]
SUPPORT_MIN = 0.6
CLEARANCE_MIN_M = 0.02
SHORT = ("INSERT", "DESCEND", "RETREAT")
NEED = ("PORTAL_TRANSIT", "TRAVERSE", "INSERT", "DESCEND",
        "RETREAT", "RETURN")
SIZES = {
    "carryon": (0.55, 0.40, 0.25),
    "standard": (0.70, 0.45, 0.28),
}
# Bounds the MoveIt spend. Geometry ranking, not a product gate.
CARRYON_PLANS = 4
STANDARD_PLANS = 5
THIRD_PLANS = 4
POOL = 24


def _yaw_of(quat):
    x, y, z, w = (float(v) for v in quat)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _corners(center, quat, size):
    from luggage_planning.planning_contract.attachment import transform_point

    hx, hy, hz = (0.5 * float(v) for v in size)
    offsets = []
    for sx in (-hx, hx):
        for sy in (-hy, hy):
            for sz in (-hz, hz):
                offsets.append(transform_point(center, quat, (sx, sy, sz)))
    return offsets


def _aabb(center, quat, size):
    corners = _corners(center, quat, size)
    low = [min(point[i] for point in corners) for i in range(3)]
    high = [max(point[i] for point in corners) for i in range(3)]
    return low, high


def _gap(a_low, a_high, b_low, b_high):
    gaps = []
    separated = False
    for index in range(3):
        if a_high[index] < b_low[index]:
            gaps.append(b_low[index] - a_high[index])
            separated = True
        elif b_high[index] < a_low[index]:
            gaps.append(a_low[index] - b_high[index])
            separated = True
        else:
            gaps.append(0.0)
    if not separated:
        penetration = min(
            min(a_high[i], b_high[i]) - max(a_low[i], b_low[i])
            for i in range(3))
        return -float(penetration)
    return math.sqrt(sum(value * value for value in gaps))


def _payload_pose(tcp, quat, size):
    from luggage_planning.planning_contract.attachment import compose_transform

    height = float(size[2])
    return compose_transform(
        tuple(tcp), tuple(quat),
        (0.0, 0.0, height / 2.0), (1.0, 0.0, 0.0, 0.0))


def _samples(plan, quat):
    names = ("portal", "slot_approach", "slot_upper", "slot")
    points = [tuple(float(v) for v in plan[name]) for name in names]
    out = []
    for start, end in zip(points, points[1:]):
        for step in range(5):
            blend = step / 4.0
            out.append((tuple(start[i] + blend * (end[i] - start[i])
                              for i in range(3)), quat))
    out.append((points[-1], quat))
    return out


def _body(row, size):
    return {
        "candidate_id": row["candidate_id"],
        "center": tuple(float(v) for v in row["center_world"]),
        "quat": tuple(float(v) for v in row["_box_world"][1]),
        "size": tuple(float(v) for v in size),
        "yaw": float(row.get("box_yaw") or 0.0),
    }


def sweep_clearance(row, size, obstacles):
    """Minimum AABB gap between the carried box and already placed boxes.

    Negative means the nominal corridor prism intersects a placed box.
    """
    if not obstacles:
        return None
    quat = tuple(float(v) for v in row["_quat"])
    gaps = []
    for tcp, heading in _samples(row["_plan"], quat):
        center, payload_q = _payload_pose(tcp, heading, size)
        payload = _aabb(center, payload_q, size)
        for obstacle in obstacles:
            other = _aabb(obstacle["center"], obstacle["quat"],
                          obstacle["size"])
            gaps.append(_gap(payload[0], payload[1], other[0], other[1]))
    return min(gaps) if gaps else None


def _rank_key(row, clearance):
    sigma = row.get("endpoint_sigma") or 0.0
    j3 = row.get("endpoint_abs_j3") or 0.0
    gap = -1.0 if clearance is None else float(clearance)
    return (-gap, -sigma, -j3, row["radius_m"], row.get("depth_m") or 0.0)


def _annotate(row, urdf, tcp_path):
    seed = row.get("_seed")
    row["endpoint_sigma"] = None if seed is None else round(
        _sigma(urdf, tcp_path, seed), 6)
    row["endpoint_abs_j3"] = None if seed is None else round(
        abs(float(seed[2])), 6)
    score = row.get("support_score")
    row["support_ok"] = score is not None and float(score) >= SUPPORT_MIN


def _pool(ctx, store, surface, placed, size, allow_stack, container_tf,
          urdf, tcp_path, scene):
    feasible, meta = _feasible(ctx, store, surface, placed, size)
    if not allow_stack:
        feasible = [row for row in feasible
                    if row.get("support_source") == "floor_prior"]
    feasible = _dedupe(feasible)
    ranked = []
    for candidate in feasible:
        tcp, _quat = _tcp_world(
            candidate["center_container"], size, container_tf)
        ranked.append((math.hypot(tcp[0], tcp[1]), candidate))
    ranked.sort(key=lambda item: item[0])
    rows = []
    for _radius, candidate in ranked[:POOL]:
        row = _geometry_row(
            candidate, size, container_tf, urdf, tcp_path, scene)
        if not row["endpoint_ik"] or row.get("support_score") is None:
            continue
        if float(row["support_score"]) < SUPPORT_MIN:
            continue
        _annotate(row, urdf, tcp_path)
        rows.append(row)
    return rows, meta


def _sync_static(node, scene):
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
    from luggage_planning.motion.planning_scene_client import (
        PlanningSceneClient,
        load_stl_mesh_msg,
    )

    client = PlanningSceneClient(node)
    if not client.wait_ready(timeout_sec=30.0):
        raise RuntimeError("apply_planning_scene unavailable")
    mesh = load_stl_mesh_msg(
        require_existing_mesh(container_collision_mesh_path(scene)),
        max_faces=0)
    xyz, rpy = origin_in_world(scene)
    client.add_collision_mesh(
        container_model_name(scene), xyz, rpy_to_quaternion(rpy), mesh)
    if pedestal_enabled(scene):
        center, ped_rpy = pedestal_collision_pose_in_world(scene)
        length, width, height = pedestal_dimensions(scene)
        client.add_collision_box(
            "robot_pedestal", center, rpy_to_quaternion(ped_rpy),
            (length, width, height))
    return client


def _scene_boxes(client):
    from moveit_msgs.msg import PlanningSceneComponents
    from luggage_planning.motion.planning_scene_client import summarize_planning_scene

    scene = client.get_scene(
        PlanningSceneComponents.WORLD_OBJECT_NAMES
        | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS,
        timeout=5.0)
    summary = summarize_planning_scene(scene)
    counts = {}
    for obj in summary.get("world_objects") or []:
        counts[obj.get("id")] = counts.get(obj.get("id"), 0) + 1
    summary["id_counts"] = counts
    summary["duplicate_ids"] = sorted(
        key for key, count in counts.items() if count > 1)
    return summary


def _contacts_at(node, joints, attached=()):
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity
    from sensor_msgs.msg import JointState

    client = node.create_client(GetStateValidity, "/check_state_validity")
    if not client.wait_for_service(timeout_sec=10.0):
        return {"error": "check_state_validity unavailable"}
    request = GetStateValidity.Request()
    request.group_name = "elfin_arm"
    state = RobotState()
    # A full state would drop the attached payload. A diff keeps the
    # payload that Cartesian collision checking is already using.
    state.is_diff = True
    state.joint_state = JointState()
    state.joint_state.name = ["elfin_joint%d" % i for i in range(1, 7)]
    state.joint_state.position = [float(v) for v in joints]
    if attached:
        state.attached_collision_objects = list(attached)
    request.robot_state = state
    future = client.call_async(request)
    deadline = time.monotonic() + 15.0
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    if not future.done() or future.result() is None:
        return {"error": "timeout"}
    response = future.result()
    contacts = []
    for contact in response.contacts:
        contacts.append({
            "body_1": contact.contact_body_1,
            "body_2": contact.contact_body_2,
            "depth": round(float(contact.depth), 6),
            "position": [
                round(float(contact.position.x), 4),
                round(float(contact.position.y), 4),
                round(float(contact.position.z), 4)],
        })
    return {"valid": bool(response.valid), "contacts": contacts[:12]}


def _legs(producer):
    from luggage_planning.motion.joint_path import joint_motion_report

    route = getattr(producer, "last_route", None)
    if route is None:
        return []
    legs = []
    for phase, ids in (("entry", route.entry_edge_ids),
                       ("retreat", route.retreat_edge_ids)):
        for edge_id in ids:
            edge = producer.graph.edges.get(edge_id)
            if edge is None:
                continue
            source = producer.graph.node(edge.source_node_id)
            dest = producer.graph.node(edge.destination_node_id)
            motion = joint_motion_report(edge.trajectory.positions)
            legs.append({
                "phase": phase,
                "edge_class": edge.attributes.get("edge_class"),
                "planner": edge.attributes.get("planner"),
                "from": source.region,
                "to": dest.region,
                "fraction": edge.attributes.get("fraction"),
                "excursion": edge.attributes.get("excursion_rad_per_m"),
                "end_delta_rad": None if motion is None else motion[
                    "end_delta_rad"],
                "n_points": len(edge.trajectory.positions),
            })
    return legs


def _gates(legs):
    classes = [leg["edge_class"] for leg in legs]
    if not all(name in classes for name in NEED):
        return False, "PHASE_MISSING"
    for leg in legs:
        if leg["planner"] != "cartesian":
            continue
        fraction = leg["fraction"]
        if fraction is None or float(fraction) < FRACTION_MIN:
            return False, "FRACTION"
        excursion = leg["excursion"]
        if excursion is None or float(excursion) > EXCURSION_MAX:
            return False, "EXCURSION"
        if leg["edge_class"] in SHORT and leg["end_delta_rad"] is not None:
            if max(abs(float(v)) for v in leg["end_delta_rad"]) > BRANCH_TOL_RAD:
                return False, "BRANCH_DISCONTINUITY"
    return True, "OK"


def _request(spec, placed_count):
    from luggage_planning.planning_contract.attachment import invert_transform

    size = tuple(float(v) for v in spec["size"])
    yaw = float(spec["yaw"])
    half = 0.5 * yaw
    slot_q = (0.0, 0.0, math.sin(half), math.cos(half))
    placed = int(placed_count)
    # static_empty is legal only before the first release. Later plans
    # use the integrated map and a stamp inside the producer clock window.
    if placed == 0:
        map_source = "static_empty"
        map_stamp = 1
    else:
        map_source = "release_integrated"
        map_stamp = 2_000_000_000
    grasp_t, grasp_q = invert_transform(
        (0.0, 0.0, size[2] / 2.0), (1.0, 0.0, 0.0, 0.0))
    return {
        "request_id": spec["request_id"],
        "scene_revision": "layout-scan",
        "box_wdh": list(size),
        "box_id": spec["candidate_id"],
        "generation": 1,
        "slots": [{
            "request_id": spec["candidate_id"],
            "position": list(spec["center_container"]),
            "quaternion_xyzw": list(slot_q),
        }],
        "surface": {
            "frame_id": "container_local", "resolution_m": 0.05,
            "origin_xy_container": [0.0, 0.0],
            "state": [["free", "free"], ["free", "free"]],
            "stamp_ns": map_stamp, "map_source": map_source,
            "placed_count": placed, "cargo_observed": placed > 0,
            "commit_revision": 1,
        },
        "inner_size": list(spec["inner_size"]),
        "geometry_hash": GEO,
        "map_revision": 1,
        "start_tcp": {"stamp_ns": 1},
        "start_joints": list(START),
        "grasp": {
            "position": list(grasp_t),
            "quaternion_xyzw": list(grasp_q),
            "stamp_ns": 1,
        },
        "container_transform": {
            "translation": list(spec["container_tf"][0]),
            "quaternion_xyzw": list(spec["container_tf"][1]),
        },
    }


def _spec(row, size, container_tf, inner_size, request_id):
    return {
        "candidate_id": row["candidate_id"],
        "center_container": list(row["center_container"]),
        "center_world": list(row["center_world"]),
        "size": tuple(float(v) for v in size),
        "yaw": float(row.get("box_yaw") or 0.0),
        "quat": tuple(float(v) for v in row["_box_world"][1]),
        "radius_m": row["radius_m"],
        "depth_m": row.get("depth_m"),
        "support_score": row.get("support_score"),
        "support_source": row.get("support_source"),
        "endpoint_sigma": row.get("endpoint_sigma"),
        "endpoint_abs_j3": row.get("endpoint_abs_j3"),
        "container_tf": container_tf,
        "inner_size": inner_size,
        "request_id": request_id,
    }


class Planner(object):
    def __init__(self, node, scene, ctx):
        from luggage_planning.atlas.graph import RouteGraph
        from luggage_planning.routing.producer import register_attach_from_request
        from slot64_classify_replay import _producer

        self.node = node
        self.scene = scene
        self.ctx = ctx
        self.container_tf = _container_pose(ctx.scene)
        self.client = _sync_static(node, scene)
        stale = _scene_boxes(self.client)
        extras = [obj_id for obj_id in (stale.get("id_counts") or {})
                  if obj_id not in ("airport_container_real", "robot_pedestal")]
        if extras:
            self.client.remove_objects(extras)
        self.producer, self.urdf, self.tcp_path = _producer(
            node, scene, SCENE, None, 1_000_000_000)
        # Keep the producer's clock on the vacuum stamp. RouteBudget
        # still enforces the 10 s wall on time.monotonic.
        self._register = register_attach_from_request
        self._graph_cls = RouteGraph
        self.adapters = []
        factory = self.producer.validity_adapter_factory

        def capturing(**kwargs):
            adapter = factory(**kwargs)
            self.adapters.append(adapter)
            return adapter

        self.producer.validity_adapter_factory = capturing
        self._cartesian = []
        real = self.producer.edge_builder._send_cartesian_request

        def remember(request):
            response = real(request)
            points = []
            fraction = None
            if response is not None:
                fraction = float(response.fraction)
                points = [[float(v) for v in point.positions]
                          for point in response.solution.joint_trajectory.points]
            waypoints = []
            for pose in request.waypoints:
                waypoints.append((
                    [float(pose.position.x), float(pose.position.y),
                     float(pose.position.z)],
                    [float(pose.orientation.x), float(pose.orientation.y),
                     float(pose.orientation.z), float(pose.orientation.w)],
                ))
            start_q = [float(v) for v in
                       request.start_state.joint_state.position]
            self._cartesian.append({
                "fraction": fraction,
                "n_points": len(points),
                "positions": points,
                "waypoints": waypoints,
                "start_q": start_q,
            })
            return response

        self.producer.edge_builder._send_cartesian_request = remember

    def plan(self, spec, placed_count, reset_after=True):
        from luggage_planning.atlas.graph import RouteGraph

        self.producer.graph = RouteGraph(
            static_identity="seq-%s" % spec["request_id"])
        before = len(self.adapters)
        cart_before = len(self._cartesian)
        fail_before = len(self.producer.edge_builder.edge_failure_log())
        request = _request(spec, placed_count)
        self._register(
            self.producer.attachment_provider, request,
            box_wdh=list(spec["size"]), box_id=spec["candidate_id"],
            generation=1)
        started = time.perf_counter()
        result = self.producer.plan_place_route(request)
        wall = time.perf_counter() - started
        last = (result.get("rows") or [{}])[-1]
        legs = _legs(self.producer)
        gates_ok, gate = _gates(legs)
        ok = bool(result.get("success")) and bool(last.get("certified")) \
            and gates_ok
        row = {
            "candidate_id": spec["candidate_id"],
            "success": bool(result.get("success")),
            "certified": bool(last.get("certified")),
            "reason": result.get("reason_code"),
            "row_reason": last.get("reason"),
            "gate": gate,
            "accepted": ok,
            "wall_s": round(wall, 3),
            "legs": legs,
            "support_score": spec.get("support_score"),
            "support_source": spec.get("support_source"),
            "radius_m": spec.get("radius_m"),
            "depth_m": spec.get("depth_m"),
            "endpoint_sigma": spec.get("endpoint_sigma"),
            "endpoint_abs_j3": spec.get("endpoint_abs_j3"),
            "failures": self.producer.edge_builder.edge_failure_log()[
                fail_before:],
            "adapters": self.adapters[before:],
            "cartesian": self._cartesian[cart_before:],
        }
        if reset_after:
            reset = getattr(self.producer.edge_builder, "reset_scene", None)
            if callable(reset):
                reset()
        print(json.dumps({
            "id": spec["candidate_id"], "accepted": ok,
            "reason": row["reason"], "gate": gate, "wall_s": row["wall_s"],
        }), flush=True)
        return row

    def add_box(self, object_id, spec):
        return self.client.add_collision_box(
            object_id, list(spec["center_world"]), list(spec["quat"]),
            tuple(spec["size"]))

    def remove(self, object_ids):
        if object_ids:
            self.client.remove_objects(object_ids)


def _match_object(summary, object_id):
    rows = [obj for obj in summary.get("world_objects") or []
            if obj.get("id") == object_id]
    return rows


def _nominal_box(slot, container_tf):
    center, quat = _box_world(
        slot["center_container"], slot["box_yaw"], container_tf)
    return {
        "candidate_id": slot["candidate_id"],
        "center_container": list(slot["center_container"]),
        "center_world": [round(float(v), 6) for v in center],
        "quaternion_xyzw": [round(float(v), 6) for v in quat],
        "yaw": float(slot["box_yaw"]),
        "size_lwh": list(slot["size_lwh"]),
        "source": "frozen_layout nominal, no Gazebo settle for this pair",
    }


def _quat_dot(left, right):
    return abs(sum(float(a) * float(b) for a, b in zip(left, right)))


def _pose_delta(nominal, observed):
    if not observed:
        return None
    xyz = observed.get("xyz") or []
    quat = observed.get("quat") or []
    if len(xyz) < 3 or len(quat) < 4:
        return None
    delta = [float(xyz[i]) - float(nominal["center_world"][i])
             for i in range(3)]
    return {
        "translation_m": [round(v, 6) for v in delta],
        "translation_norm_m": round(math.sqrt(sum(v * v for v in delta)), 6),
        "quaternion_dot": round(
            _quat_dot(quat, nominal["quaternion_xyzw"]), 6),
        "world_yaw_rad": round(_yaw_of(quat), 6),
        "container_yaw_rad": float(nominal["yaw"]),
        "size": observed.get("size"),
    }


def _first_contact(node, producer, plan_row, object_id, size):
    """Joint state and contact pairs at the first invalid sample."""
    for adapter in plan_row["adapters"]:
        for record in adapter.requests:
            for leg in record.get("legs") or []:
                contacts = leg.get("contacts") or []
                if not contacts and leg.get("valid", True):
                    continue
                edge_id = leg.get("edge")
                samples = adapter._samples.get(edge_id) or ()
                index = int(leg.get("first_invalid") or 0)
                joints = None
                if samples and 0 <= index < len(samples):
                    joints = list(samples[index])
                live = _contacts_at(node, joints) if joints else None
                tcp = None
                payload = None
                if joints is not None and producer.fk is not None:
                    position, quaternion = producer.fk(joints)
                    tcp = {
                        "position": [round(float(v), 4) for v in position],
                        "quaternion_xyzw": [round(float(v), 6)
                                            for v in quaternion],
                    }
                    center, payload_q = _payload_pose(position, quaternion, size)
                    payload = {
                        "center_world": [round(float(v), 4) for v in center],
                        "quaternion_xyzw": [round(float(v), 6)
                                            for v in payload_q],
                        "size_lwh": list(size),
                        "link": "suction_contact_frame",
                        "t_tcp_box_translation": [0.0, 0.0, float(size[2]) / 2.0],
                        "t_tcp_box_quaternion_xyzw": [1.0, 0.0, 0.0, 0.0],
                    }
                return {
                    "source": "certification",
                    "edge": edge_id,
                    "reason": leg.get("reason"),
                    "first_invalid_sample": index,
                    "service_contacts": contacts[:8],
                    "live_contacts": live,
                    "joints": None if joints is None else [
                        round(float(v), 6) for v in joints],
                    "tcp": tcp,
                    "attached_payload": payload,
                    "names_placed_object": any(
                        object_id in (item.get("body_1", ""),
                                      item.get("body_2", ""))
                        for item in (contacts + (live or {}).get(
                            "contacts") or [])),
                }
    carts = [item for item in plan_row["cartesian"]
             if item.get("fraction") is not None
             and item["fraction"] < FRACTION_MIN and item["positions"]]
    if not carts:
        return {"source": "none"}
    partial = carts[0]
    joints = partial["positions"][-1]
    live = _contacts_at(node, joints)
    position, quaternion = producer.fk(joints)
    center, payload_q = _payload_pose(position, quaternion, size)
    return {
        "source": "cartesian_partial_last_point",
        "fraction": partial["fraction"],
        "n_points": partial["n_points"],
        "live_contacts": live,
        "joints": [round(float(v), 6) for v in joints],
        "tcp": {
            "position": [round(float(v), 4) for v in position],
            "quaternion_xyzw": [round(float(v), 6) for v in quaternion],
        },
        "attached_payload": {
            "center_world": [round(float(v), 4) for v in center],
            "quaternion_xyzw": [round(float(v), 6) for v in payload_q],
            "size_lwh": list(size),
            "link": "suction_contact_frame",
            "t_tcp_box_translation": [0.0, 0.0, float(size[2]) / 2.0],
            "t_tcp_box_quaternion_xyzw": [1.0, 0.0, 0.0, 0.0],
        },
        "names_placed_object": any(
            object_id in (item.get("body_1", ""), item.get("body_2", ""))
            for item in (live or {}).get("contacts") or []),
        "note": (
            "MoveIt stops the Cartesian path on the last collision-free "
            "sample, so this point can still be valid. The placed-object "
            "id in a later certification contact is the collision proof."),
    }


def _cartesian_again(node, start_q, waypoints, avoid):
    from geometry_msgs.msg import Point, Pose, Quaternion
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetCartesianPath
    from sensor_msgs.msg import JointState

    client = node.create_client(GetCartesianPath, "/compute_cartesian_path")
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
    start.joint_state.name = ["elfin_joint%d" % i for i in range(1, 7)]
    start.joint_state.position = [float(v) for v in start_q]
    request.start_state = start
    for position, quaternion in waypoints:
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
        time.sleep(0.02)
    if not future.done() or future.result() is None:
        return {"fraction": None, "positions": []}
    response = future.result()
    return {
        "fraction": round(float(response.fraction), 6),
        "positions": [[float(v) for v in point.positions]
                      for point in response.solution.joint_trajectory.points],
    }


def _beyond_contact(node, producer, plan_row, object_id, size):
    """Step past the last collision-free Cartesian sample and read contacts."""
    from luggage_planning.motion.offline_ik import refine_to_pose

    failure = next((item for item in plan_row["failures"]
                    if item.get("reason") == "fraction_below_min"), None)
    if failure is None:
        return None
    fraction = float(failure["fraction"])
    partial = next((item for item in plan_row["cartesian"]
                    if item.get("positions") and item.get("fraction") is not None
                    and abs(float(item["fraction"]) - fraction) < 1e-4), None)
    if partial is None:
        return {"error": "partial trajectory not captured", "failure": failure}
    from moveit_msgs.msg import PlanningSceneComponents
    from luggage_planning.motion.planning_scene_client import PlanningSceneClient

    attached = []
    live_scene = PlanningSceneClient(node).get_scene(
        PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS, timeout=5.0)
    if live_scene is not None:
        attached = list(live_scene.robot_state.attached_collision_objects)
    off = _cartesian_again(
        node, partial["start_q"], partial["waypoints"], False)
    positions = off.get("positions") or []
    first_invalid = None
    if positions:
        begin = max(0, int(float(fraction) * len(positions)) - 1)
        index = begin
        while index > 0 and not _contacts_at(
                node, positions[index], attached).get("valid", True):
            index -= 1
        while index < len(positions):
            live = _contacts_at(node, positions[index], attached)
            if not live.get("valid", True):
                first_invalid = (index, positions[index], live)
                break
            index += 1
    payload = None
    tcp = None
    step_contacts = None
    if partial.get("positions"):
        from luggage_planning.motion.offline_ik import refine_to_pose

        seed = partial["positions"][-1]
        here, heading = producer.fk(seed)
        start = failure["from_position"]
        goal = failure["to_position"]
        span = math.sqrt(sum((float(goal[i]) - float(start[i])) ** 2
                             for i in range(3)))
        step = min(span, max(0.01, span * (1.0 - fraction)))
        direction = [(float(goal[i]) - float(start[i])) / max(span, 1e-6)
                     for i in range(3)]
        target = tuple(float(here[i]) + step * direction[i] for i in range(3))
        solved, info = refine_to_pose(
            producer.urdf, producer.tcp_path, target, heading, list(seed),
            pos_tol=0.01, rot_tol=0.05)
        if solved is not None:
            step_contacts = _contacts_at(node, solved, attached)
            step_contacts["pos_err_m"] = None if not info else info.get(
                "pos_err_m")
            step_contacts["target"] = [round(float(v), 4) for v in target]
            if not step_contacts.get("valid", True):
                first_invalid = (-1, solved, step_contacts)
    if first_invalid is not None:
        _index, joints, probed = first_invalid
        position, quaternion = producer.fk(joints)
        tcp = [round(float(v), 4) for v in position]
        center, payload_q = _payload_pose(position, quaternion, size)
        payload = {
            "center_world": [round(float(v), 4) for v in center],
            "quaternion_xyzw": [round(float(v), 6) for v in payload_q],
            "size_lwh": list(size),
            "link": "suction_contact_frame",
            "t_tcp_box_translation": [0.0, 0.0, float(size[2]) / 2.0],
            "t_tcp_box_quaternion_xyzw": [1.0, 0.0, 0.0, 0.0],
        }
    else:
        probed = step_contacts
    contacts = (probed or {}).get("contacts") or []
    return {
        "edge_class": failure.get("edge_class"),
        "attached_ids": [str(item.object.id) for item in attached if item.object],
        "fraction_on": fraction,
        "fraction_off": off.get("fraction"),
        "off_points": len(positions),
        "failed_waypoint_index": failure.get("failed_waypoint_index"),
        "from_position": failure.get("from_position"),
        "to_position": failure.get("to_position"),
        "first_invalid_index": None if first_invalid is None else first_invalid[0],
        "first_invalid_joints": None if first_invalid is None else [
            round(float(v), 6) for v in first_invalid[1]],
        "tcp_at_contact": tcp,
        "step_probe": step_contacts,
        "live_contacts": probed,
        "attached_payload": payload,
        "names_placed_object": any(
            object_id in (item.get("body_1", ""), item.get("body_2", ""))
            for item in contacts),
        "split": (
            "COLLISION_ALONG_PATH"
            if off.get("fraction") is not None and off["fraction"] >= FRACTION_MIN
            and fraction < FRACTION_MIN else "KINEMATIC"),
    }


def diagnose(planner, frozen):
    slots = frozen["slots"]
    box1 = _nominal_box(slots[0], planner.container_tf)
    box2 = _nominal_box(slots[1], planner.container_tf)
    object_id = "diag-box1"
    spec = {
        "candidate_id": slots[1]["candidate_id"],
        "center_container": slots[1]["center_container"],
        "center_world": box2["center_world"],
        "size": tuple(slots[1]["size_lwh"]),
        "yaw": float(slots[1]["box_yaw"]),
        "quat": tuple(box2["quaternion_xyzw"]),
        "radius_m": slots[1].get("radius_m"),
        "depth_m": slots[1].get("depth_m"),
        "support_score": slots[1].get("support_score"),
        "support_source": slots[1].get("support_source"),
        "endpoint_sigma": slots[1].get("min_sigma"),
        "endpoint_abs_j3": slots[1].get("min_abs_j3"),
        "container_tf": planner.container_tf,
        "inner_size": list(planner.ctx.inner_size),
        "request_id": "diag-box2",
    }
    planner.add_box(object_id, {
        "center_world": box1["center_world"],
        "quat": box1["quaternion_xyzw"],
        "size": tuple(box1["size_lwh"]),
    })
    before = _scene_boxes(planner.client)
    observed = _match_object(before, object_id)
    # The plan mutates attached objects. Capture contacts before reset
    # by reading them inside plan(), which resets afterwards. Snapshot
    # the world first; contacts are read from the certifier record.
    plan_row = planner.plan(spec, 1, reset_after=False)
    contact = _first_contact(
        planner.node, planner.producer, plan_row, object_id,
        tuple(slots[1]["size_lwh"]))
    contact["beyond"] = _beyond_contact(
        planner.node, planner.producer, plan_row, object_id,
        tuple(slots[1]["size_lwh"]))
    after = _scene_boxes(planner.client)
    reset = getattr(planner.producer.edge_builder, "reset_scene", None)
    if callable(reset):
        reset()
    planner.remove([object_id])
    names = []
    for item in (contact.get("service_contacts") or []):
        names.append((item.get("body_1"), item.get("body_2")))
    for item in (contact.get("live_contacts") or {}).get("contacts") or []:
        names.append((item.get("body_1"), item.get("body_2")))
    beyond = contact.get("beyond") or {}
    for item in (beyond.get("live_contacts") or {}).get("contacts") or []:
        names.append((item.get("body_1"), item.get("body_2")))
    placed_named = any(object_id in pair for pair in names)
    arm_or_payload = any(
        any(token in (left + " " + right).lower() for token in (
            "payload", "elfin", "suction", "link"))
        for left, right in names)
    single = len(observed) == 1 and not before.get("duplicate_ids")
    delta = _pose_delta(box1, observed[0] if observed else None)
    pose_ok = delta is not None and delta["translation_norm_m"] < 0.005 \
        and delta["quaternion_dot"] > 0.999
    verdict = "SEQUENTIAL_PATH_COLLISION" if (
        placed_named and arm_or_payload and single and pose_ok) else (
        "CONTACT_NOT_CONFIRMED")
    return {
        "verdict": verdict,
        "frozen_ids": [slot["candidate_id"] for slot in slots],
        "plan": {key: plan_row[key] for key in (
            "candidate_id", "success", "certified", "reason",
            "row_reason", "gate", "wall_s", "legs")},
        "first_failure": contact,
        "box1_nominal": box1,
        "box1_scene_objects": observed,
        "box1_pose_delta": delta,
        "box2_nominal": box2,
        "scene_before": {
            "ids": before.get("id_counts"),
            "duplicate_ids": before.get("duplicate_ids"),
            "attached": before.get("attached"),
        },
        "scene_after_plan": {
            "ids": after.get("id_counts"),
            "duplicate_ids": after.get("duplicate_ids"),
            "attached": after.get("attached"),
        },
        "checks": {
            "contact_names_box1": placed_named,
            "contact_names_arm_or_payload": arm_or_payload,
            "single_box1_object": single,
            "scene_pose_matches_nominal": pose_ok,
        },
    }


def _public_spec(spec, clearance):
    return {
        "candidate_id": spec["candidate_id"],
        "order": spec.get("order"),
        "size_lwh": list(spec["size"]),
        "center_container": spec["center_container"],
        "center_world": spec["center_world"],
        "box_yaw": spec["yaw"],
        "radius_m": spec["radius_m"],
        "depth_m": spec["depth_m"],
        "support_score": spec["support_score"],
        "support_source": spec["support_source"],
        "endpoint_sigma": spec["endpoint_sigma"],
        "endpoint_abs_j3": spec["endpoint_abs_j3"],
        "sweep_clearance_m": None if clearance is None else round(
            float(clearance), 4),
    }


def search(planner, exclude=()):
    ctx = planner.ctx
    store = AtlasStore(ctx)
    surface0 = floor_prior_surface(ctx)
    container_tf = planner.container_tf
    urdf, tcp = planner.urdf, planner.tcp_path
    scene = ctx.scene
    carry_size = SIZES["carryon"]
    std_size = SIZES["standard"]
    print(json.dumps({"stage": "pool", "box": "carryon"}), flush=True)
    carry_rows, carry_meta = _pool(
        ctx, store, surface0, [], carry_size, False, container_tf,
        urdf, tcp, scene)
    std_rows, std_meta = _pool(
        ctx, store, surface0, [], std_size, False, container_tf,
        urdf, tcp, scene)
    standards = sorted(std_rows, key=lambda row: row["radius_m"])[:8]
    std_bodies = [_body(row, std_size) for row in standards]

    def carry_key(row):
        gaps = [sweep_clearance(std, std_size, [_body(row, carry_size)])
                for std in standards]
        clear = [gap for gap in gaps if gap is not None and gap >= CLEARANCE_MIN_M]
        best_gap = max(gaps) if gaps else -1.0
        return (
            -len(clear),
            -best_gap,
            -(row.get("endpoint_sigma") or 0.0),
            -(row.get("endpoint_abs_j3") or 0.0),
            row["radius_m"],
            row.get("depth_m") or 0.0,
        )

    excluded = set(exclude or ())
    carry_rows = [row for row in carry_rows
                  if row["candidate_id"] not in excluded]
    carry_rows.sort(key=carry_key)
    print(json.dumps({
        "stage": "rank",
        "carry": [row["candidate_id"] for row in carry_rows[:CARRYON_PLANS]],
        "standards": [row["candidate_id"] for row in standards],
    }), flush=True)
    tried = []
    for carry in carry_rows[:CARRYON_PLANS]:
        spec1 = _spec(carry, carry_size, container_tf, ctx.inner_size,
                      "search-b1-" + carry["candidate_id"])
        spec1["order"] = "carryon"
        gap1 = None
        row1 = planner.plan(spec1, 0)
        tried.append({"stage": 1, "id": spec1["candidate_id"],
                      "accepted": row1["accepted"], "reason": row1["reason"],
                      "gate": row1["gate"]})
        if not row1["accepted"]:
            continue
        object1 = "search-box1"
        planner.add_box(object1, spec1)
        body1 = _body(carry, carry_size)
        surface1 = floor_prior_surface(ctx)
        placed1 = [candidate_aabb(
            carry["_candidate"], ctx.inner_size[2], ctx.floor_z)]
        apply_placement(surface1, carry["_candidate"], 0.0)
        stage2, _meta2 = _pool(
            ctx, store, surface1, placed1, std_size, False, container_tf,
            urdf, tcp, scene)
        ranked2 = []
        for row in stage2:
            gap = sweep_clearance(row, std_size, [body1])
            if gap is None or gap < CLEARANCE_MIN_M:
                continue
            ranked2.append((gap, row))
        ranked2.sort(key=lambda item: _rank_key(item[1], item[0]))
        found = None
        for gap2, standard in ranked2[:STANDARD_PLANS]:
            spec2 = _spec(standard, std_size, container_tf, ctx.inner_size,
                          "search-b2-" + standard["candidate_id"])
            spec2["order"] = "standard"
            row2 = planner.plan(spec2, 1)
            tried.append({"stage": 2, "after": spec1["candidate_id"],
                          "id": spec2["candidate_id"],
                          "sweep_clearance_m": round(gap2, 4),
                          "accepted": row2["accepted"],
                          "reason": row2["reason"], "gate": row2["gate"]})
            if not row2["accepted"]:
                continue
            object2 = "search-box2"
            planner.add_box(object2, spec2)
            body2 = _body(standard, std_size)
            surface2 = floor_prior_surface(ctx)
            placed2 = list(placed1)
            apply_placement(surface2, standard["_candidate"], 0.0)
            placed2.append(candidate_aabb(
                standard["_candidate"], ctx.inner_size[2], ctx.floor_z))
            stage3, _meta3 = _pool(
                ctx, store, surface2, placed2, carry_size, True,
                container_tf, urdf, tcp, scene)
            ranked3 = []
            for row in stage3:
                gap = sweep_clearance(row, carry_size, [body1, body2])
                if gap is None or gap < CLEARANCE_MIN_M:
                    continue
                ranked3.append((gap, row))
            ranked3.sort(key=lambda item: _rank_key(item[1], item[0]))
            for gap3, third in ranked3[:THIRD_PLANS]:
                spec3 = _spec(third, carry_size, container_tf,
                              ctx.inner_size,
                              "search-b3-" + third["candidate_id"])
                spec3["order"] = "carryon"
                row3 = planner.plan(spec3, 2)
                tried.append({"stage": 3,
                              "after": [spec1["candidate_id"],
                                        spec2["candidate_id"]],
                              "id": spec3["candidate_id"],
                              "sweep_clearance_m": round(gap3, 4),
                              "accepted": row3["accepted"],
                              "reason": row3["reason"], "gate": row3["gate"]})
                if row3["accepted"]:
                    found = (spec1, spec2, spec3, gap2, gap3, row1, row2, row3)
                    break
            planner.remove([object2])
            if found:
                break
        planner.remove([object1])
        if found:
            spec1, spec2, spec3, gap2, gap3, row1, row2, row3 = found
            return {
                "found": True,
                "carry_pool": carry_meta,
                "standard_pool": std_meta,
                "tried": tried,
                "slots": [
                    _public_spec(spec1, None),
                    _public_spec(spec2, gap2),
                    _public_spec(spec3, gap3),
                ],
                "plans": [{
                    "candidate_id": row["candidate_id"],
                    "wall_s": row["wall_s"],
                    "reason": row["reason"],
                    "legs": row["legs"],
                } for row in (row1, row2, row3)],
            }
    return {
        "found": False,
        "carry_pool": carry_meta,
        "standard_pool": std_meta,
        "tried": tried,
        "slots": [],
    }


def confirm(planner, slots):
    repeats = []
    for repeat in range(3):
        placed = []
        rows = []
        ok = True
        for index, slot in enumerate(slots):
            spec = dict(slot)
            spec["size"] = tuple(slot["size_lwh"])
            spec["yaw"] = float(slot["box_yaw"])
            spec["quat"] = tuple(_box_world(
                slot["center_container"], slot["box_yaw"],
                planner.container_tf)[1])
            spec["center_world"] = list(_box_world(
                slot["center_container"], slot["box_yaw"],
                planner.container_tf)[0])
            spec["container_tf"] = planner.container_tf
            spec["inner_size"] = list(planner.ctx.inner_size)
            spec["request_id"] = "confirm-r%d-%d" % (repeat, index)
            row = planner.plan(spec, len(placed))
            public = {key: row[key] for key in (
                "candidate_id", "success", "certified", "reason",
                "row_reason", "gate", "accepted", "wall_s")}
            public["classes"] = [leg["edge_class"] for leg in row["legs"]]
            public["placed_before"] = list(placed)
            rows.append(public)
            if not row["accepted"]:
                ok = False
                break
            object_id = "confirm-r%d-%d" % (repeat, index)
            planner.add_box(object_id, spec)
            placed.append(object_id)
        planner.remove(placed)
        repeats.append({"repeat": repeat, "success": ok, "rows": rows})
        print("CONFIRM", repeat, ok, flush=True)
    return {
        "passed": sum(1 for item in repeats if item["success"]),
        "repeats": repeats,
    }


def _dump(name, payload):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=1, sort_keys=True, default=str)
        handle.write("\n")
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--diagnose-only", action="store_true")
    parser.add_argument("--skip-diagnose", action="store_true")
    parser.add_argument("--exclude", default="",
                        help="comma-separated carryon ids to skip")
    args = parser.parse_args()
    import rclpy
    import threading
    import yaml
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    frozen = json.load(open(os.path.join(OUT, "frozen_layout.json"),
                            encoding="utf-8"))
    scene = yaml.safe_load(open(SCENE, encoding="utf-8"))
    ctx = SceneContext(SCENE, ATLAS_CONFIG)
    ctx.atlas_cfg = dict(ctx.atlas_cfg)
    ctx.atlas_cfg["hash_policy"] = "off"
    placement = dict(ctx.atlas_cfg.get("placement") or {})
    placement["hard_reject"] = False
    ctx.atlas_cfg["placement"] = placement
    rclpy.init()
    node = Node("three_box_sequential_search")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    planner = Planner(node, scene, ctx)
    try:
        diagnosis = None
        if not args.skip_diagnose:
            diagnosis = diagnose(planner, frozen)
            path = _dump("sequential_collision_diag.json", diagnosis)
            print("DIAG", diagnosis["verdict"], path, flush=True)
            if args.diagnose_only or diagnosis["verdict"] != \
                    "SEQUENTIAL_PATH_COLLISION":
                return 0 if diagnosis and diagnosis["verdict"] == \
                    "SEQUENTIAL_PATH_COLLISION" else 2
        found = search(planner, exclude=[
            item for item in args.exclude.split(",") if item])
        path = _dump("sequential_search.json", {
            key: value for key, value in found.items() if key != "plans"
        } | {"plans": found.get("plans")})
        print("SEARCH", found["found"], path, flush=True)
        if not found["found"]:
            return 3
        confirmation = confirm(planner, found["slots"])
        _dump("sequential_confirm.json", confirmation)
        frozen_out = {
            "scene": SCENE,
            "sequence": ["carryon", "standard", "carryon"],
            "selection": "sequential_full_route",
            "previous_combo": {
                "ids": [slot["candidate_id"] for slot in frozen["slots"]],
                "verdict": "SEQUENTIAL_PATH_COLLISION",
            },
            "attempt2": "not_started",
            "confirm_passed": confirmation["passed"],
            "slots": found["slots"],
        }
        if confirmation["passed"] == 3:
            _dump("frozen_layout.json", frozen_out)
        print("CONFIRM_PASSED", confirmation["passed"], flush=True)
        return 0 if confirmation["passed"] == 3 else 4
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
