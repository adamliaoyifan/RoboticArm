#!/usr/bin/env python3
"""Plan-only MoveIt 2 edge adapter for the ST09-B q-state route graph.

MoveIt 2 is the only edge generator: Cartesian (GetCartesianPath) and
RRTConnect (MoveGroup ``plan_only``) are internal means of the one edge
builder behind :meth:`MoveItEdgeAdapter.__call__`, which matches the
signature the bounded search invokes (``from_node, to_node, edge_class,
candidate=None``). The adapter never executes: it holds no
ExecuteTrajectory client and produces complete timed trajectories
(joint order, velocities, accelerations, monotonic ns) retimed with the
site rest-to-rest trapezoid.

Orientation semantics (technical baseline 5.2): transport orientation is
fixed through portal/narrow traversal, insert/descend keep the final
heading, and continuous orientation transitions happen only where the
caller places an ORIENTATION_TRANSITION hop (rotation accompanies
translation — slerp along a translating path, no in-place spins).

Judges: only cartesian fraction (< ``cartesian_min_fraction``) and joint
excursion per metre (> ``cartesian_excursion_rad_per_m``) refuse; wrist
singularity margin and j6 rotation are recorded as diagnostics and never
reject. Payload/envelope modelling rides the planning scene: carrying
edges attach the payload at the real ``T_tcp_box`` (true eccentric
geometry), unloaded retreat edges detach and add the released box as a
world obstacle at the placed heading.

ROS imports are lazy: importing this module must not require a ROS
environment (offline test collection stays possible); only constructing
:class:`MoveItEdgeAdapter` does.
"""

from __future__ import division

import json
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple

#: Frozen judge values (a B test pins equality against
#: st09_contracts/resolved_planning_contract.json).
CARTESIAN_MIN_FRACTION = 0.95
CARTESIAN_EXCURSION_RAD_PER_M = 3.0
MOVEIT_VELOCITY_SCALING = 0.3
MOVEIT_ACCELERATION_SCALING = 0.3
SINGLE_EDGE_WALL_S = 2.0

#: Canonical joint order (motion_frame_contracts.assert_canonical_joints).
JOINT_NAMES = ("elfin_joint1", "elfin_joint2", "elfin_joint3",
               "elfin_joint4", "elfin_joint5", "elfin_joint6")

#: Waypoint spacing for synthesized cartesian chains (m); the solver's
#: own interpolation step stays at the executor's 0.01 m.
WAYPOINT_SPACING_M = 0.10


class EdgeClass(str, Enum):
    """Route edge classes; joint classes plan via MoveGroup, cartesian
    classes via GetCartesianPath."""

    PORTAL_TRANSIT = "PORTAL_TRANSIT"
    ORIENTATION_TRANSITION = "ORIENTATION_TRANSITION"
    TRAVERSE = "TRAVERSE"
    INSERT = "INSERT"
    DESCEND = "DESCEND"
    RETREAT = "RETREAT"
    RETURN = "RETURN"

    @property
    def planner(self) -> str:
        if self in (EdgeClass.PORTAL_TRANSIT, EdgeClass.RETURN):
            return "joint"
        return "cartesian"


#: Orientation policy per cartesian class: hold source (transport), hold
#: destination (final heading), or slerp across the hop (clearance-zone
#: transition with accompanying translation).
ORIENTATION_MODE = {
    EdgeClass.TRAVERSE: "hold_source",
    EdgeClass.INSERT: "hold_destination",
    EdgeClass.DESCEND: "hold_destination",
    EdgeClass.RETREAT: "hold_source",
    EdgeClass.ORIENTATION_TRANSITION: "slerp",
}


@dataclass(frozen=True)
class BoxSpec:
    """Axis-aligned box obstacle in the planning frame."""

    object_id: str
    center: Tuple[float, float, float]
    size_whd: Tuple[float, float, float]
    quaternion_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class PayloadSpec:
    """Attached payload geometry at the real T_tcp_box (5.1 semantics)."""

    dimensions_wdh: Tuple[float, float, float]
    t_tcp_box_translation: Tuple[float, float, float]
    t_tcp_box_quaternion_xyzw: Tuple[float, float, float, float]


@dataclass(frozen=True)
class SceneState:
    """What the planning scene must hold while one edge is planned.

    ``load_state`` selects the carrying/unloaded scene; ``payload`` is
    required when carrying and attached at the measured transform;
    ``released_box`` (the just-placed box at its actual heading) and
    ``placed_boxes`` are world obstacles for the unloaded retreat.
    """

    load_state: str  # "carrying" | "unloaded"
    payload: Optional[PayloadSpec] = None
    released_box: Optional[BoxSpec] = None
    placed_boxes: Tuple[BoxSpec, ...] = ()
    attach_touch_links: Tuple[str, ...] = ()


def scene_boxes_from_snapshot(snapshot, *, inflate_m=0.05,
                              cell_height_m=0.30) -> Tuple[BoxSpec, ...]:
    """Coarse prior obstacles from snapshot OCCUPIED cells.

    A dev approximation for graph-side scene diffs: each occupied surface
    cell becomes one box. The released box and the attached payload are
    modelled exactly elsewhere; this never turns UNKNOWN into free.
    """
    boxes = []
    for position, (cell_i, cell_j) in enumerate(
            zip(snapshot.cells_i, snapshot.cells_j)):
        if snapshot.state_at(int(cell_i), int(cell_j)).value != "occupied":
            continue
        center_x = snapshot.origin_xy[0] + (int(cell_i) + 0.5) * snapshot.resolution_m
        center_y = snapshot.origin_xy[1] + (int(cell_j) + 0.5) * snapshot.resolution_m
        size = snapshot.resolution_m + 2.0 * float(inflate_m)
        boxes.append(BoxSpec(
            object_id="snapshot-cell-%d-%d" % (int(cell_i), int(cell_j)),
            center=(center_x, center_y, 0.5 * cell_height_m),
            size_whd=(size, size, float(cell_height_m))))
    return tuple(boxes)


@dataclass
class EdgeOutcome:
    """Diagnostics for one build attempt (evidence deliverable)."""

    edge_class: str
    planner: str
    ok: bool
    reason: str = ""
    fraction: Optional[float] = None
    excursion_rad_per_m: Optional[float] = None
    wrist_margin_rad: Optional[float] = None
    j6_rotation_rad: Optional[float] = None
    moveit_error_code: int = 0
    attempts: int = 0
    wall_s: float = 0.0
    waypoints: int = 0


def _slerp(a_xyzw, b_xyzw, t):
    from st09_contracts.attachment import quaternion_angle_between
    angle = quaternion_angle_between(a_xyzw, b_xyzw)
    if angle <= 1e-12:
        return tuple(float(v) for v in a_xyzw)
    # Relative rotation axis via quaternion division (xyzw Hamilton).
    ax, ay, az, aw = a_xyzw
    bx, by, bz, bw = b_xyzw
    # conj(a) * b
    rx = aw * bx - ax * bw - ay * bz + az * by
    ry = aw * by + ax * bz - ay * bw - az * bx
    rz = aw * bz - ax * by + ay * bx - az * bw
    rw = aw * bw + ax * bx + ay * by + az * bz
    # relative(t) = exp(t * log(rel)) via axis-angle
    norm = math.sqrt(rx * rx + ry * ry + rz * rz)
    if norm <= 1e-12:
        return tuple(float(v) for v in a_xyzw)
    half_angle = math.atan2(norm, rw) * float(t)
    scale = math.sin(half_angle) / norm
    dx, dy, dz = rx * scale, ry * scale, rz * scale
    dw = math.cos(half_angle)
    # compose a * delta
    qx = aw * dx + ax * dw + ay * dz - az * dy
    qy = aw * dy - ax * dz + ay * dw + az * dx
    qz = aw * dz + ax * dy - ay * dx + az * dw
    qw = aw * dw - ax * dx - ay * dy - az * dz
    norm_q = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return (qx / norm_q, qy / norm_q, qz / norm_q, qw / norm_q)


def _lerp(a, b, t):
    return tuple(float(a[i]) + (float(b[i]) - float(a[i])) * float(t)
                 for i in range(3))


def synthesize_waypoints(source_position, source_quaternion,
                         destination_position, destination_quaternion,
                         edge_class, spacing_m=WAYPOINT_SPACING_M):
    """Cartesian pose chain for one edge class (position linear,
    orientation per the class policy)."""
    mode = ORIENTATION_MODE[edge_class if isinstance(edge_class, EdgeClass)
                            else EdgeClass(str(edge_class))]
    distance = math.sqrt(sum(
        (float(destination_position[i]) - float(source_position[i])) ** 2
        for i in range(3)))
    steps = max(1, int(math.ceil(distance / max(float(spacing_m), 1e-3))))
    poses = []
    for step in range(1, steps + 1):
        t = float(step) / steps
        position = _lerp(source_position, destination_position, t)
        if mode == "hold_source":
            quaternion = tuple(float(v) for v in source_quaternion)
        elif mode == "hold_destination":
            quaternion = tuple(float(v) for v in destination_quaternion)
        else:
            quaternion = _slerp(source_quaternion, destination_quaternion, t)
        poses.append((position, quaternion))
    return poses, distance


class MoveItEdgeAdapter(object):
    """Builds route edges through MoveIt planning only (no execution).

    ``__call__(from_node, to_node, edge_class, candidate=None)`` is the
    builder the bounded search invokes; a stored ``candidate`` edge is
    re-probed by rebuilding its trajectory fresh (cached unverified
    polylines never pass through untouched).
    """

    def __init__(self, node, *, group_name="elfin_arm",
                 link_name="suction_contact_frame", planner_id="RRTConnect",
                 allowed_planning_time=SINGLE_EDGE_WALL_S,
                 num_planning_attempts=1, cartesian_max_step=0.01,
                 jump_threshold=0.0, avoid_collisions=True,
                 velocity_scaling=MOVEIT_VELOCITY_SCALING,
                 acceleration_scaling=MOVEIT_ACCELERATION_SCALING,
                 scene_client=None):
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.srv import GetCartesianPath
        from rclpy.action import ActionClient

        self._node = node
        self._group = str(group_name)
        self._link = str(link_name)
        self._planner_id = str(planner_id)
        self._allowed_planning_time = float(allowed_planning_time)
        self._num_planning_attempts = int(num_planning_attempts)
        self._cartesian_max_step = float(cartesian_max_step)
        self._jump_threshold = float(jump_threshold)
        self._avoid_collisions = bool(avoid_collisions)
        self._velocity_scaling = float(velocity_scaling)
        self._acceleration_scaling = float(acceleration_scaling)
        self._move_group_type = MoveGroup
        self._cartesian_type = GetCartesianPath
        self._action = ActionClient(node, MoveGroup, "/move_action")
        self._cartesian = node.create_client(
            GetCartesianPath, "/compute_cartesian_path")
        if scene_client is None:
            from luggage_planning.planning_scene_client import (
                PlanningSceneClient)
            scene_client = PlanningSceneClient(node)
        self._scene = scene_client
        self._call_counts: Dict[str, int] = {"move_group": 0,
                                             "cartesian": 0,
                                             "scene_apply": 0}
        self._call_durations: Dict[str, List[float]] = {
            "move_group": [], "cartesian": [], "scene_apply": []}
        self._request_dumps: List[dict] = []
        self._scene_dumps: List[dict] = []
        self._failure_log: List[dict] = []
        self._outcomes: List[EdgeOutcome] = []
        self._scene_state_applied: Optional[str] = None

    # -- lifecycle --------------------------------------------------------

    def wait_ready(self, timeout_sec=60.0):
        """Block until both planning services are available (fail loud)."""
        if not self._action.wait_for_server(timeout_sec=float(timeout_sec)):
            raise RuntimeError("move_group action unavailable after %.1fs"
                               % float(timeout_sec))
        if not self._cartesian.wait_for_service(timeout_sec=float(timeout_sec)):
            raise RuntimeError("compute_cartesian_path unavailable after %.1fs"
                               % float(timeout_sec))
        self._scene.wait_ready(timeout_sec=float(timeout_sec))
        return True

    # -- evidence ---------------------------------------------------------

    def moveit_call_counts(self):
        return dict(self._call_counts)

    def call_durations(self):
        return {key: list(values) for key, values in self._durations_items()}

    def _durations_items(self):
        return self._call_durations.items()

    def request_dumps(self):
        return list(self._request_dumps)

    def scene_dumps(self):
        return list(self._scene_dumps)

    def edge_failure_log(self):
        return list(self._failure_log)

    def outcomes(self):
        return list(self._outcomes)

    def _record_failure(self, edge_class, reason, **detail):
        entry = {"edge_class": str(edge_class), "reason": str(reason)}
        entry.update({key: value for key, value in detail.items()})
        self._failure_log.append(entry)

    # -- planning scene ---------------------------------------------------

    def _ensure_scene(self, scene, payload_id="b-payload"):
        """Idempotent scene diff for the edge's load state (fail closed).

        Carrying edges attach the payload at the real T_tcp_box through
        the production attach helper (DEFAULT_TOUCH_LINKS); unloaded
        retreat edges detach it. Released and placed boxes are world
        objects in the planning frame.
        """
        marker = json.dumps({
            "load_state": scene.load_state,
            "payload": vars(scene.payload) if scene.payload else None,
            "released_box": vars(scene.released_box)
            if scene.released_box else None,
            "placed": [vars(box) for box in scene.placed_boxes],
        }, sort_keys=True, default=str)
        if marker == self._scene_state_applied:
            return
        started = time.monotonic()
        world_boxes = tuple(scene.placed_boxes) + (
            (scene.released_box,) if scene.released_box else ())
        for box in world_boxes:
            if not self._scene.add_collision_box(
                    box.object_id, list(box.center), list(box.quaternion_xyzw),
                    list(box.size_whd), frame_id="world"):
                raise RuntimeError("scene add failed for %s" % box.object_id)
            self._call_counts["scene_apply"] += 1
        if scene.load_state == "carrying":
            if scene.payload is None:
                raise RuntimeError("carrying scene without a payload spec")
            from luggage_planning.planning_scene_client import (
                build_attach_scene,
            )

            attach_diff = build_attach_scene(
                payload_id,
                list(scene.payload.t_tcp_box_translation),
                list(scene.payload.t_tcp_box_quaternion_xyzw),
                list(scene.payload.dimensions_wdh),
                self._link)
            if not self._scene.apply_diff(attach_diff, timeout=5.0):
                raise RuntimeError("scene attach failed")
            self._call_counts["scene_apply"] += 1
        elif self._scene_state_applied is not None:
            if not self._scene.detach_and_remove(box_id=payload_id):
                raise RuntimeError("scene detach failed")
            self._call_counts["scene_apply"] += 1
        self._verify_scene(scene.load_state, payload_id)
        self._scene_state_applied = marker
        self._scene_dumps.append({
            "load_state": scene.load_state,
            "world_boxes": len(world_boxes),
            "wall_s": round(time.monotonic() - started, 6),
        })

    def _verify_scene(self, load_state, payload_id, attempts=10, delay_s=0.1):
        """Fail closed unless the live scene matches the requested state.

        The planning-scene monitor applies diffs asynchronously; a bounded
        retry absorbs the lag, and a mismatch that persists is a named
        scene_apply_error instead of a mysterious planner refusal.
        """
        from moveit_msgs.msg import PlanningSceneComponents

        for _attempt in range(max(1, int(attempts))):
            scene = self._scene.get_scene(
                PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS,
                timeout=2.0)
            if scene is not None:
                attached = [
                    obj.object.id
                    for obj in scene.robot_state.attached_collision_objects]
                if load_state == "carrying" and payload_id in attached:
                    return
                if load_state == "unloaded" and payload_id not in attached:
                    return
            time.sleep(float(delay_s))
        raise RuntimeError(
            "scene verification failed for %s (payload %s still %s)"
            % (load_state, payload_id,
               "attached" if load_state == "unloaded" else "missing"))

    # -- edge building ------------------------------------------------------

    def __call__(self, from_node, to_node, edge_class, candidate=None,
                 scene=None):
        return self.build_edge(from_node, to_node, edge_class,
                               candidate=candidate, scene=scene)

    def build_edge(self, from_node, to_node, edge_class, candidate=None,
                   scene=None):
        """Build one verified edge; raises RoutePlanningError on failure."""
        from luggage_planning.grid_path_search import RoutePlanningError

        klass = edge_class if isinstance(edge_class, EdgeClass) \
            else EdgeClass(str(edge_class))
        started = time.monotonic()
        waypoints_hint = None
        if candidate is not None and candidate.attributes.get("waypoints"):
            waypoints_hint = [
                (tuple(row["position"]), tuple(row["quaternion_xyzw"]))
                for row in candidate.attributes["waypoints"]]
        if scene is not None:
            self._ensure_scene(scene)
        try:
            if klass.planner == "joint":
                trajectory, outcome = self._plan_joint(from_node, to_node,
                                                       klass)
            else:
                trajectory, outcome = self._plan_cartesian(
                    from_node, to_node, klass, waypoints_hint)
        except RoutePlanningError:
            raise
        except Exception as err:  # scene/transport failures are named too
            self._record_failure(klass.value, "scene_apply_error",
                                 error=str(err))
            raise RoutePlanningError("EDGE_BUILD_FAILED",
                                     {"kind": "scene_apply_error",
                                      "error": str(err)})
        outcome.wall_s = round(time.monotonic() - started, 6)
        outcome.attempts = max(1, outcome.attempts)
        self._outcomes.append(outcome)
        attributes = {
            "edge_class": klass.value,
            "planner": klass.planner,
            "duration_ns": int(round(trajectory.time_from_start_ns[-1])),
            "joint_travel_rad": _joint_travel(trajectory),
            "fraction": outcome.fraction,
            "excursion_rad_per_m": outcome.excursion_rad_per_m,
            "wrist_margin_rad": outcome.wrist_margin_rad,
            "j6_rotation_rad": outcome.j6_rotation_rad,
            "moveit_error_code": outcome.moveit_error_code,
            "waypoints": _recorded_waypoints(
                klass, from_node, to_node, waypoints_hint),
            "reprobe": bool(candidate is not None),
        }
        from st09_contracts.graph import GraphEdge
        edge = GraphEdge(
            edge_id="e-%s-%s-%s" % (
                klass.value.lower(),
                _digest8(from_node.node_id + to_node.node_id),
                _digest8(str(attributes))),
            source_node_id=from_node.node_id,
            destination_node_id=to_node.node_id,
            load_state=from_node.load_state,
            static_identity="moveit-edge-v1",
            trajectory=trajectory,
            validation_revision=None,
            attributes=attributes)
        return edge

    # -- planners -----------------------------------------------------------

    def _plan_joint(self, from_node, to_node, klass):
        from luggage_planning.grid_path_search import RoutePlanningError
        from moveit_msgs.msg import Constraints, JointConstraint, RobotState
        from sensor_msgs.msg import JointState

        goal = self._move_group_type.Goal()
        goal.request.group_name = self._group
        goal.request.planner_id = self._planner_id
        goal.request.allowed_planning_time = self._allowed_planning_time
        goal.request.num_planning_attempts = self._num_planning_attempts
        goal.request.start_state.is_diff = True
        goal.request.start_state.joint_state = JointState()
        goal.request.start_state.joint_state.name = list(JOINT_NAMES)
        goal.request.start_state.joint_state.position = [
            float(v) for v in from_node.q]
        goal.planning_options.plan_only = True  # B never executes
        goal.planning_options.replan = False
        constraints = Constraints()
        constraints.name = "edge-%s" % klass.value.lower()
        for index, name in enumerate(JOINT_NAMES):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(to_node.q[index])
            joint.tolerance_above = 1e-4
            joint.tolerance_below = 1e-4
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints.append(constraints)
        self._call_counts["move_group"] += 1
        began = time.monotonic()
        result = self._send_action_goal(goal)
        self._call_durations["move_group"].append(
            round(time.monotonic() - began, 6))
        self._request_dumps.append({
            "planner": "joint", "edge_class": klass.value,
            "planner_id": self._planner_id,
            "from": from_node.node_id, "to": to_node.node_id,
            "error_code": int(result.error_code.val) if result else -1})
        trajectory = getattr(result, "planned_trajectory", None) if result \
            else None
        joint_trajectory = getattr(trajectory, "joint_trajectory", None) \
            if trajectory is not None else None
        if result is None or int(result.error_code.val) != 1 or \
                joint_trajectory is None or not joint_trajectory.points:
            self._record_failure(klass.value, "moveit_error",
                                 error_code=int(result.error_code.val)
                                 if result else -1)
            raise RoutePlanningError("EDGE_BUILD_FAILED", {
                "kind": "moveit_error",
                "error_code": int(result.error_code.val) if result else -1})
        timed = self._retime(joint_trajectory)
        outcome = EdgeOutcome(
            edge_class=klass.value, planner="joint", ok=True,
            moveit_error_code=int(result.error_code.val), attempts=1)
        return timed, outcome

    def _plan_cartesian(self, from_node, to_node, klass, waypoints_hint):
        from luggage_planning.grid_path_search import RoutePlanningError
        from moveit_msgs.msg import RobotState
        from geometry_msgs.msg import Pose, Point, Quaternion
        from sensor_msgs.msg import JointState

        poses, distance = (waypoints_hint, _path_length(waypoints_hint)) \
            if waypoints_hint else synthesize_waypoints(
                from_node.tcp_position, from_node.tcp_quaternion_xyzw,
                to_node.tcp_position, to_node.tcp_quaternion_xyzw, klass)
        request = self._cartesian_type.Request()
        request.header.frame_id = "world"
        request.group_name = self._group
        request.link_name = self._link
        request.max_step = self._cartesian_max_step
        request.jump_threshold = self._jump_threshold
        request.avoid_collisions = self._avoid_collisions
        start = RobotState()
        start.is_diff = True
        start.joint_state = JointState()
        start.joint_state.name = list(JOINT_NAMES)
        start.joint_state.position = [float(v) for v in from_node.q]
        request.start_state = start
        for position, quaternion in poses:
            pose = Pose()
            pose.position = Point(x=float(position[0]), y=float(position[1]),
                                  z=float(position[2]))
            pose.orientation = Quaternion(
                x=float(quaternion[0]), y=float(quaternion[1]),
                z=float(quaternion[2]), w=float(quaternion[3]))
            request.waypoints.append(pose)
        self._call_counts["cartesian"] += 1
        began = time.monotonic()
        response = self._send_cartesian_request(request)
        self._call_durations["cartesian"].append(
            round(time.monotonic() - began, 6))
        self._request_dumps.append({
            "planner": "cartesian", "edge_class": klass.value,
            "from": from_node.node_id, "to": to_node.node_id,
            "waypoints": len(poses),
            "fraction": round(float(response.fraction), 6)
            if response else None})
        if response is None or not response.solution.joint_trajectory.points:
            self._record_failure(klass.value, "moveit_error",
                                 fraction=None)
            raise RoutePlanningError("EDGE_BUILD_FAILED", {
                "kind": "moveit_error", "fraction": None})
        fraction = float(response.fraction)
        if fraction < CARTESIAN_MIN_FRACTION:
            self._record_failure(klass.value, "fraction_below_min",
                                 fraction=round(fraction, 6))
            raise RoutePlanningError("EDGE_BUILD_FAILED", {
                "kind": "fraction_below_min", "fraction": round(fraction, 6),
                "required": CARTESIAN_MIN_FRACTION})
        timed = self._retime(response.solution.joint_trajectory)
        excursion = _excursion(timed, distance)
        if excursion is not None and excursion["ratio"] > \
                CARTESIAN_EXCURSION_RAD_PER_M:
            self._record_failure(klass.value, "excursion_exceeded",
                                 ratio=round(excursion["ratio"], 4))
            raise RoutePlanningError("EDGE_BUILD_FAILED", {
                "kind": "excursion_exceeded",
                "ratio": round(excursion["ratio"], 4),
                "worst_joint": excursion["worst_joint"],
                "limit": CARTESIAN_EXCURSION_RAD_PER_M})
        outcome = EdgeOutcome(
            edge_class=klass.value, planner="cartesian", ok=True,
            fraction=round(fraction, 6),
            excursion_rad_per_m=round(excursion["ratio"], 6)
            if excursion else None,
            wrist_margin_rad=excursion.get("wrist_margin_rad") if excursion
            else None,
            j6_rotation_rad=excursion.get("j6_rotation_rad") if excursion
            else None,
            moveit_error_code=1, attempts=1, waypoints=len(poses))
        return timed, outcome

    # -- transports ---------------------------------------------------------

    def _send_action_goal(self, goal, timeout_sec=None):
        timeout = float(timeout_sec or self._allowed_planning_time + 3.0)
        future = self._action.send_goal_async(goal)
        handle = self._wait_future(future, 10.0)
        if handle is None:
            self._record_failure("transport", "goal_timeout")
            return None
        result_future = handle.get_result_async()
        result = self._wait_future(result_future, timeout)
        if result is None:
            handle.cancel_goal()
            self._record_failure("transport", "result_timeout")
            return None
        return result.result

    def _send_cartesian_request(self, request, timeout_sec=None):
        timeout = float(timeout_sec or self._allowed_planning_time + 3.0)
        future = self._call_async_cartesian(request)
        return self._wait_future(future, timeout)

    def _call_async_cartesian(self, request):
        return self._cartesian.call_async(request)

    def _wait_future(self, future, timeout_sec):
        import threading

        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=float(timeout_sec)):
            return None
        return future.result()

    # -- timing -------------------------------------------------------------

    def _retime(self, joint_trajectory):
        """Site rest-to-rest trapezoid over the geometric knots -> ns."""
        from st09_contracts.graph import TimedTrajectory
        from luggage_planning.trajectory_timing import (
            time_parameterize_cartesian)

        points = list(joint_trajectory.points)
        names = tuple(joint_trajectory.joint_names or JOINT_NAMES)
        if len(points) < 2:
            raise ValueError("retime needs at least two knots")
        timed = time_parameterize_cartesian(
            [list(point.positions) for point in points],
            self._velocity_scaling, self._acceleration_scaling)
        times_ns = []
        for value in timed.times_s:
            stamp = int(round(float(value) * 1e9))
            if times_ns and stamp <= times_ns[-1]:
                stamp = times_ns[-1] + 1  # strict monotonicity guard
            times_ns.append(stamp)
        return TimedTrajectory(
            joint_names=names,
            positions=tuple(tuple(float(v) for v in point.positions)
                            for point in points),
            velocities=tuple(tuple(float(v) for v in row)
                             for row in timed.velocities_rad),
            accelerations=tuple(tuple(float(v) for v in row)
                                for row in timed.accelerations_rad),
            time_from_start_ns=tuple(times_ns))


def _digest8(text):
    import hashlib

    return hashlib.sha256(str(text).encode("utf-8")).hexdigest()[:8]


def _recorded_waypoints(klass, from_node, to_node, waypoints_hint):
    """JSON-safe waypoint record: cartesian classes store the pose chain
    (a hint when re-probing a stored candidate); joint classes have none."""
    if klass.planner == "joint":
        return []
    poses = waypoints_hint or synthesize_waypoints(
        from_node.tcp_position, from_node.tcp_quaternion_xyzw,
        to_node.tcp_position, to_node.tcp_quaternion_xyzw, klass)[0]
    return [{"position": list(position), "quaternion_xyzw": list(quaternion)}
            for position, quaternion in poses]


def _joint_travel(trajectory):
    rows = trajectory.positions
    if len(rows) < 2:
        return 0.0
    return float(max(
        sum(abs(b[i] - a[i]) for a, b in zip(rows, rows[1:]))
        for i in range(len(rows[0]))))


def _path_length(poses):
    length = 0.0
    previous = None
    for position, _quaternion in poses:
        if previous is not None:
            length += math.sqrt(sum(
                (float(position[i]) - float(previous[i])) ** 2
                for i in range(3)))
        previous = position
    return length


def _excursion(trajectory, path_len_m):
    """Joint total variation per metre of cartesian travel (executor
    semantics: floor at 0.05 m; wrist/j6 measured, never refusing)."""
    from luggage_planning.joint_path import max_joint_step

    rows = trajectory.positions
    if len(rows) < 2 or path_len_m <= 1e-9:
        return None
    denominator = max(path_len_m, 0.05)
    variations = [
        sum(abs(b[index] - a[index]) for a, b in zip(rows, rows[1:]))
        for index in range(len(rows[0]))]
    worst = max(range(len(variations)), key=lambda i: variations[i])
    return {
        "worst_joint": JOINT_NAMES[worst],
        "variation_rad": variations[worst],
        "ratio": variations[worst] / denominator,
        "wrist_margin_rad": abs(float(rows[0][4])) % math.pi,
        "j6_rotation_rad": variations[5] if len(variations) > 5 else 0.0,
    }
