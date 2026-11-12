#!/usr/bin/env python3
"""Replay Run02 slot 64 DESCEND pairing. No budget or tolerance change.

Proves, from the saved slot-64 targets and hover joints:

- the first DESCEND the route search attempts is same-heading and
  same-branch, even when the heading-pi goal is registered first;
- a Cartesian line refined from that hover reaches the goal. The
  reported fraction is successful steps / 20. The 0.95 gate is the
  same number MoveIt enforces as ``CARTESIAN_MIN_FRACTION``.

``--moveit`` also calls ``/compute_cartesian_path`` with the edge
adapter's DESCEND request (one hold-destination waypoint). That needs
an already-running plan-only MoveIt on this process's ROS domain.
"""
import argparse
import json
import math
import os

from luggage_gazebo.integration_acceptance import RETREAT_START_TOLERANCE_RAD
from luggage_planning.routing.grid_search import (
    MAX_EDGE_REPAIR_ATTEMPTS,
    ROUTE_TOTAL_WALL_S,
    RouteBudget,
    RouteSearch,
    SINGLE_NEW_EDGE_WALL_S,
    descend_pair_rank,
)
from luggage_planning.routing.moveit_edge_adapter import (
    CARTESIAN_MIN_FRACTION,
    synthesize_waypoints,
)
from luggage_planning.offline_ik import load_urdf, refine_to_pose
from luggage_planning.atlas.graph import RouteGraph, make_offline_fk
from st09_contracts.graph import GraphEdge, LoadState
from st09_contracts.fixtures import make_timed_trajectory

RUN02 = (
    "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3/"
    "g3_three_box_run02_20261004")
RUN01 = (
    "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3/"
    "g3_three_box_run01_20261004")
SCENE = (
    "install-st09/luggage_description/share/luggage_description/"
    "config/scene_tf_deploy.yaml")


def _wrap(delta):
    return (float(delta) + math.pi) % (2.0 * math.pi) - math.pi


def _retreat_margin(run_dir):
    path = os.path.join(run_dir, "m3_evidence.json")
    with open(path, "r", encoding="utf-8") as handle:
        evidence = json.load(handle)
    retreat = evidence["cases"][0]["measured"]["retreat_start"]
    return {
        "run": os.path.basename(run_dir),
        "status": retreat["status"],
        "max_delta_rad": retreat["max_delta"],
        "tolerance_rad": retreat["tolerance_rad"],
    }


class _RecordingBuilder(object):
    def __init__(self, graph):
        self.graph = graph
        self.calls = []

    def __call__(self, from_node, to_node, edge_class, candidate=None,
                 scene=None):
        self.calls.append((from_node, to_node, edge_class))
        base = make_timed_trajectory()
        trajectory = type(base)(
            joint_names=base.joint_names,
            positions=(base.positions[0], tuple(float(v) for v in to_node.q)),
            velocities=tuple(base.velocities[:2]),
            accelerations=tuple(base.accelerations[:2]),
            time_from_start_ns=tuple(base.time_from_start_ns[:2]))
        return GraphEdge(
            edge_id="e-%03d" % len(self.calls),
            source_node_id=from_node.node_id,
            destination_node_id=to_node.node_id,
            load_state=from_node.load_state,
            static_identity="replay",
            trajectory=trajectory,
            validation_revision=None,
            attributes={"edge_class": edge_class, "duration_ns": 1,
                        "joint_travel_rad": 0.1})


def _line_fraction(urdf, tcp, start_q, start_pos, goal_pos, goal_quat,
                   steps=20):
    """Fraction of a straight Cartesian descend that stays on one branch."""
    prev = list(start_q)
    ok = 0
    for index in range(1, steps + 1):
        scale = float(index) / steps
        position = [start_pos[k] + (goal_pos[k] - start_pos[k]) * scale
                    for k in range(3)]
        joints, _info = refine_to_pose(
            urdf, tcp, position, goal_quat, prev, pos_tol=2e-3, rot_tol=2e-2)
        if joints is None:
            break
        jump = max(abs(_wrap(b - a)) for a, b in zip(prev, joints))
        if jump > 0.3:
            break
        prev = list(joints)
        ok = index
    return ok / float(steps), ok


def _moveit_fraction(from_q, from_pos, from_quat, to_pos, to_quat):
    import rclpy
    from geometry_msgs.msg import Point, Pose, Quaternion
    from moveit_msgs.srv import GetCartesianPath
    from sensor_msgs.msg import JointState
    from moveit_msgs.msg import RobotState

    rclpy.init()
    node = rclpy.create_node("slot64_descend_replay")
    client = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    if not client.wait_for_service(timeout_sec=30.0):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError("compute_cartesian_path unavailable")
    poses, _distance = synthesize_waypoints(
        from_pos, from_quat, to_pos, to_quat, "DESCEND")
    request = GetCartesianPath.Request()
    request.header.frame_id = "world"
    request.group_name = "elfin_arm"
    request.link_name = "suction_contact_frame"
    request.max_step = 0.01
    request.jump_threshold = 0.0
    request.avoid_collisions = True
    start = RobotState()
    start.is_diff = True
    start.joint_state = JointState()
    start.joint_state.name = [
        "elfin_joint1", "elfin_joint2", "elfin_joint3",
        "elfin_joint4", "elfin_joint5", "elfin_joint6"]
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
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    response = future.result()
    fraction = None if response is None else float(response.fraction)
    node.destroy_node()
    rclpy.shutdown()
    return fraction, len(poses)


def replay(scene, candidates_path, moveit=False):
    with open(candidates_path, "r", encoding="utf-8") as handle:
        row = next(item for item in json.load(handle)["rows"]
                   if item.get("slot_index") == 64)
    targets = []
    for record in row["ik_refine"]:
        if record["target_world"] not in targets:
            targets.append(record["target_world"])
    hovers = []
    for failure in row["detail"]["edge_failures"]:
        joints = tuple(failure["detail"]["from_q"])
        if joints not in hovers:
            hovers.append(joints)
    urdf = load_urdf(scene_tf_config=scene)
    tcp = urdf.path_to("suction_contact_frame")
    fk = make_offline_fk(urdf, tcp)
    graph = RouteGraph(static_identity="replay")
    home = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def add(q, region, load, family="", tcp_pos=None):
        position, quat = fk(q) if tcp_pos is None else (tcp_pos, fk(q)[1])
        return graph.register_node(
            q, position, quat, region, load, family)

    add(home, "home", LoadState.CARRYING)
    portal = tuple(v + 0.05 for v in hovers[0])
    add(portal, "portal", LoadState.CARRYING)
    add(tuple(v + 0.02 for v in hovers[0]), "slot_approach", LoadState.CARRYING)
    upper_nodes = []
    for joints in hovers:
        position, quat = fk(joints)
        upper_nodes.append(graph.register_node(
            joints, position, quat, "slot_upper", LoadState.CARRYING, "hover"))
    # Heading-pi goals first, the order Run02 actually searched.
    goal_nodes = []
    for joints in hovers:
        for position, quaternion in targets:
            refined, info = refine_to_pose(
                urdf, tcp, position, quaternion, list(joints),
                pos_tol=2e-3, rot_tol=2e-2)
            if refined is None:
                continue
            rank = descend_pair_rank(joints, refined)
            family = "continuous" if rank[0] == 0 else "fallback"
            node = graph.register_node(
                refined, tuple(position), tuple(quaternion),
                "slot", LoadState.CARRYING, family)
            graph.register_node(
                refined, tuple(position), tuple(quaternion),
                "slot", LoadState.UNLOADED, family)
            goal_nodes.append(node)
    # Stable-unique by node id, fallbacks before continuous, matching the
    # old registration bias the search must override.
    by_id = {}
    for node in goal_nodes:
        by_id.setdefault(node.node_id, node)
    ordered = sorted(by_id.values(), key=lambda node: (
        0 if node.ik_family == "fallback" else 1, node.node_id))
    add(home, "home", LoadState.UNLOADED)
    add((0.1, 0.0, 0.0, 0.0, 0.0, 0.0), "exit", LoadState.UNLOADED)
    builder = _RecordingBuilder(graph)
    search = RouteSearch(graph, builder, RouteBudget())
    start = graph.node_for_q(LoadState.CARRYING, home)
    _entry, _retreat, goal = search.best_full_route(start.node_id, ordered)
    first = next(call for call in builder.calls if call[2] == "DESCEND")
    source, dest, _klass = first
    rank = descend_pair_rank(source.q, dest.q)
    fraction, steps_ok = _line_fraction(
        urdf, tcp, source.q, source.tcp_position,
        dest.tcp_position, dest.tcp_quaternion_xyzw)
    # The pair Run02 attempted first: hover A joined to the heading-pi goal.
    bad_from = hovers[0]
    bad_pos, bad_quat = fk((
        0.986457, -2.808695, -0.386532, -1.520632, 1.576033, 4.128033))
    bad_start_pos, _bad_start_quat = fk(bad_from)
    bad_fraction, bad_steps = _line_fraction(
        urdf, tcp, bad_from, bad_start_pos, bad_pos, bad_quat)
    moveit_fraction = None
    waypoint_count = None
    bad_moveit = None
    if moveit:
        moveit_fraction, waypoint_count = _moveit_fraction(
            source.q, source.tcp_position, source.tcp_quaternion_xyzw,
            dest.tcp_position, dest.tcp_quaternion_xyzw)
        bad_moveit, _bad_n = _moveit_fraction(
            bad_from, bad_start_pos, fk(bad_from)[1], bad_pos, bad_quat)
    margins = [_retreat_margin(RUN01), _retreat_margin(RUN02)]
    return {
        "slot_index": 64,
        "first_descend": {
            "from_q": [round(v, 6) for v in source.q],
            "to_q": [round(v, 6) for v in dest.q],
            "joint6_delta_rad": round(_wrap(dest.q[5] - source.q[5]), 6),
            "rank_tier": rank[0],
            "same_heading_same_branch": rank[0] == 0,
            "ik_family": dest.ik_family,
        },
        "ik_line_fraction": fraction,
        "ik_line_steps_ok": steps_ok,
        "ik_line_steps": 20,
        "cartesian_min_fraction": CARTESIAN_MIN_FRACTION,
        "moveit_fraction": moveit_fraction,
        "moveit_waypoint_count": waypoint_count,
        "moveit_scene": (
            "isolated_moveit, empty planning scene, avoid_collisions "
            "true, jump_threshold 0, one DESCEND waypoint"
            if moveit else None),
        "run02_heading_pi_pair": {
            "ik_line_fraction": bad_fraction,
            "ik_line_steps_ok": bad_steps,
            "moveit_fraction_empty_scene": bad_moveit,
            "note": (
                "Pair Run02 attempted first. An empty planning scene "
                "does not reproduce the in-scene fraction 0.1. The IK "
                "line is what separates it from the new first pair."),
        },
        "goal_node": goal.node_id,
        "retreat_start": {
            "gate_rad": RETREAT_START_TOLERANCE_RAD,
            "gate_relaxed": RETREAT_START_TOLERANCE_RAD != 0.01,
            "observed": margins,
        },
        "budgets": {
            "route_total_wall_s": ROUTE_TOTAL_WALL_S,
            "single_new_edge_wall_s": SINGLE_NEW_EDGE_WALL_S,
            "max_edge_repair_attempts": MAX_EDGE_REPAIR_ATTEMPTS,
            "changed": False,
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default=SCENE)
    parser.add_argument(
        "--candidates",
        default=os.path.join(RUN02, "dumps/box_01_paths/candidates.json"))
    parser.add_argument("--out", required=True)
    parser.add_argument("--moveit", action="store_true")
    args = parser.parse_args()
    report = replay(args.scene, args.candidates, moveit=args.moveit)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=1)
        handle.write("\n")
    print(json.dumps(report, indent=1))
    ok = (report["first_descend"]["same_heading_same_branch"]
          and report["ik_line_fraction"] >= CARTESIAN_MIN_FRACTION
          and report["retreat_start"]["gate_rad"] == 0.01
          and not report["retreat_start"]["gate_relaxed"])
    if args.moveit:
        ok = ok and report["moveit_fraction"] is not None \
            and report["moveit_fraction"] >= CARTESIAN_MIN_FRACTION
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
