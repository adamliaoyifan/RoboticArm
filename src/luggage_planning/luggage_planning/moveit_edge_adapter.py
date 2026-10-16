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
import os
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple

#: Frozen judge values (a B test pins equality against
#: st09_contracts/resolved_planning_contract.json).
CARTESIAN_MIN_FRACTION = 0.95


def cartesian_failure_detail(from_q, to_q, from_xyz, to_xyz, fraction,
                             required, waypoint_count):
    """Refusal context for a short Cartesian path.

    ``failed_waypoint_index`` is the first requested waypoint the
    fraction did not reach. MoveIt reports a fraction, not an index.
    """
    count = max(int(waypoint_count), 0)
    span = max(count - 1, 0)
    index = int(math.floor(float(fraction) * span))
    return {
        "kind": "fraction_below_min",
        "fraction": round(float(fraction), 6),
        "required": float(required),
        "from_q": [round(float(v), 6) for v in from_q],
        "to_q": [round(float(v), 6) for v in to_q],
        "from_position": [round(float(v), 6) for v in from_xyz],
        "to_position": [round(float(v), 6) for v in to_xyz],
        "waypoint_count": count,
        "failed_waypoint_index": index,
    }
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
    """What one edge must be planned and certified against.

    ``placed_boxes`` is the request's existing cargo. Carrying and
    unloaded edges share that set. Release changes only the current
    payload: attached while carrying, a world box (``released_box``)
    after release. Edge building may still touch the live scene; the
    certifier rebuilds both load states from this record.
    """

    load_state: str  # "carrying" | "unloaded"
    payload: Optional[PayloadSpec] = None
    released_box: Optional[BoxSpec] = None
    placed_boxes: Tuple[BoxSpec, ...] = ()
    attach_touch_links: Tuple[str, ...] = ()


class OccupancyColumnError(ValueError):
    """Occupied cargo could not be turned into a world column.

    The name is the first token of the message (``OCCUPANCY_FLOOR_MISSING``,
    ``OCCUPANCY_FRAME_MISSING``, ``OCCUPANCY_HEIGHT_MISSING``).
    """


def scene_boxes_from_snapshot(snapshot, *, inflate_m=0.05,
                              cell_height_m=0.30) -> Tuple[BoxSpec, ...]:
    """Legacy coarse boxes in the snapshot's own XY numbers.

    This slab is not the cargo-map column. ``cell_height_m`` is a fixed
    stand-in, and the centers stay in the container-local grid. Route
    certification uses ``occupancy_boxes_from_surface`` instead, which
    requires the container-link floor and the world transform.
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


def _column_height_above_floor(surface, row, column):
    """Measured column top above the inner floor, or None when absent."""
    if "height_above_floor" in surface:
        value = surface["height_above_floor"][row][column]
        return None if value is None else float(value)
    if "surface_z_container" in surface and "floor_z_container" in surface:
        top = surface["surface_z_container"][row][column]
        if top is None:
            return None
        return float(top) - float(surface["floor_z_container"])
    if "height" in surface and "floor_z_container" in surface:
        # Legacy ``height`` is already above the inner floor. It is only
        # usable together with the container-link floor, never with the
        # legacy ``floor_z: 0`` datum.
        return float(surface["height"][row][column])
    return None


def occupancy_boxes_from_surface(surface, container_transform, *,
                                 inflate_m=0.05) -> Tuple[BoxSpec, ...]:
    """World-frame columns for cells the surface marks occupied.

    The grid XY is container-local. The column runs from
    ``floor_z_container`` up to that floor plus ``height_above_floor``
    (the map's top, not a fixed 0.30 m slab). ``container_transform``
    is world ← container_link. The returned centers are in world, so a
    later ``frame_id=world`` insert is the transformed pose.

    An occupied cell with no positive column top, or a grid that has
    occupied cells but no container-link floor or no world transform,
    fails closed. It does not invent a 0–0.30 m box.
    """
    from st09_contracts.attachment import compose_transform

    state = (surface or {}).get("state") or []
    occupied = [
        (row, column)
        for row, cells in enumerate(state)
        for column, name in enumerate(cells)
        if str(name) == "occupied"]
    if not occupied:
        return ()
    if not container_transform:
        raise OccupancyColumnError(
            "OCCUPANCY_FRAME_MISSING: occupied cells have no "
            "container_transform")
    if "floor_z_container" not in (surface or {}):
        raise OccupancyColumnError(
            "OCCUPANCY_FLOOR_MISSING: occupied cells have no "
            "floor_z_container")
    origin = (surface.get("origin_xy_container")
              or surface.get("origin_local"))
    if not origin or len(origin) < 2:
        raise OccupancyColumnError(
            "OCCUPANCY_FRAME_MISSING: occupied cells have no "
            "origin_xy_container")
    resolution = float(surface.get("resolution_m")
                       or surface.get("resolution"))
    floor = float(surface["floor_z_container"])
    translation = tuple(float(v) for v in container_transform["translation"])
    quaternion = tuple(
        float(v) for v in container_transform["quaternion_xyzw"])
    lateral = resolution + 2.0 * float(inflate_m)
    boxes = []
    for row, column in occupied:
        height = _column_height_above_floor(surface, row, column)
        if height is None or height <= 0.0:
            raise OccupancyColumnError(
                "OCCUPANCY_HEIGHT_MISSING: occupied cell %d,%d has no "
                "positive height_above_floor" % (row, column))
        local = (
            float(origin[0]) + (row + 0.5) * resolution,
            float(origin[1]) + (column + 0.5) * resolution,
            floor + 0.5 * height)
        world, world_q = compose_transform(
            translation, quaternion, local, (0.0, 0.0, 0.0, 1.0))
        boxes.append(BoxSpec(
            object_id="snapshot-cell-%d-%d" % (row, column),
            center=tuple(float(v) for v in world),
            size_whd=(lateral, lateral, height),
            quaternion_xyzw=tuple(float(v) for v in world_q)))
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
        self._scene_world_ids_added: list = []

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

    def planner_parameters(self):
        """Parameters that select a joint or Cartesian edge.

        ``ompl_rng_seed`` is ``unfixed``: RRTConnect has no seed in the
        planning request or in ``ompl_planning.yaml``. Cartesian edges do
        not use that generator.
        """
        return {
            "planner_id": self._planner_id,
            "allowed_planning_time_s": self._allowed_planning_time,
            "num_planning_attempts": self._num_planning_attempts,
            "cartesian_max_step_m": self._cartesian_max_step,
            "jump_threshold": self._jump_threshold,
            "avoid_collisions": self._avoid_collisions,
            "velocity_scaling": self._velocity_scaling,
            "acceleration_scaling": self._acceleration_scaling,
            "ompl_rng_seed": "unfixed",
        }

    def outcomes(self):
        return list(self._outcomes)

    def _record_failure(self, edge_class, reason, **detail):
        entry = {"edge_class": str(edge_class), "reason": str(reason)}
        entry.update({key: value for key, value in detail.items()})
        self._failure_log.append(entry)

    # -- planning scene ---------------------------------------------------

    def reset_scene(self, payload_id="b-payload"):
        """[ST09-I] leave the planning scene UNLOADED after a select.

        A select that fails after its entry planning (budget, no
        feasible route, refused certification) never reaches the retreat
        chain's unload diff, stranding the frozen payload ATTACHED in
        the live scene — the next motion in that scene collides with
        the phantom (live: a later pick's attach descent stopped at
        18.5% exactly where a phantom payload met the world box). The
        planning-added released/placed world boxes are planning
        artifacts too: a route that was planned but never physically
        released leaves them behind and the NEXT select certifies
        against a box that was never placed (live: the leftover
        released box sat exactly on the next cycle's slot line).
        Idempotent; the success path's real release re-adds its own
        world geometry through the vacuum/placement surfaces.
        """
        if self._scene is not None:
            self._scene.detach_and_remove(box_id=payload_id)
            for object_id in tuple(self._scene_world_ids_added):
                self._scene.remove_object(object_id)
        self._scene_world_ids_added = []
        self._scene_state_applied = None

    def _ensure_scene(self, scene, payload_id="b-payload"):
        """Idempotent scene diff for the edge's load state (fail closed).

        Carrying edges attach the payload at the real T_tcp_box through
        the production attach helper (DEFAULT_TOUCH_LINKS); unloaded
        retreat edges detach it. Released and placed boxes are world
        objects in the planning frame.
        """
        import hashlib

        marker = json.dumps({
            "load_state": scene.load_state,
            "payload": vars(scene.payload) if scene.payload else None,
            "released_box": vars(scene.released_box)
            if scene.released_box else None,
            "placed": [vars(box) for box in scene.placed_boxes],
        }, sort_keys=True, default=str)
        scene_digest = hashlib.sha256(marker.encode("utf-8")).hexdigest()
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
            self._scene_world_ids_added.append(str(box.object_id))
            self._call_counts["scene_apply"] += 1
        if scene.load_state == "carrying":
            if scene.payload is None:
                raise RuntimeError("carrying scene without a payload spec")
            from luggage_planning.planning_scene_client import (
                BOX_OBJECT_ID,
                build_attach_scene,
            )

            # ONE payload representation while planning: the frozen
            # measured record (b-payload). The live vacuum attach is
            # execution state — leaving it in produced a second,
            # misplaced payload copy in the planning scene and every
            # carrying edge refused on phantom collisions (live:
            # pedestal x pickup_box contacts at the world origin while
            # the arm stood over the platform).
            if not self._scene.detach_and_remove(box_id=BOX_OBJECT_ID):
                raise RuntimeError("live payload detach failed")
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
            "scene_digest": scene_digest,
            "payload_wdh": (
                [float(v) for v in scene.payload.dimensions_wdh]
                if scene.payload is not None else None),
            "payload_translation": (
                [float(v) for v in scene.payload.t_tcp_box_translation]
                if scene.payload is not None else None),
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

    def _portal_submission_scene(self):
        """Scene as MoveGroup is about to see it. Not a later query."""
        from moveit_msgs.msg import PlanningSceneComponents

        parts = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            | PlanningSceneComponents.TRANSFORMS
            | PlanningSceneComponents.OCTOMAP
            | PlanningSceneComponents.LINK_PADDING_AND_SCALING)
        return self._scene.get_scene(parts, timeout=3.0)

    def _keep_portal_failure(self, from_node, to_node, code, scene_msg,
                             constraints):
        """Keep the first PORTAL_TRANSIT failure of each goal signature.

        ``scene_msg`` was read before the MoveGroup call. A scene read
        after the call is recorded separately and is not that snapshot.
        """
        directory = os.environ.get("ST09_PORTAL_CAPTURE_DIR") or ""
        if not directory or scene_msg is None:
            return
        os.makedirs(directory, exist_ok=True)
        goal_q = tuple(round(float(v), 3) for v in to_node.q)
        signature = "%s:%s" % (int(code), ",".join("%.3f" % v for v in goal_q))
        seen = getattr(self, "_portal_signatures", None)
        if seen is None:
            seen = set()
            self._portal_signatures = seen
        if signature in seen or len(seen) >= 4:
            return
        seen.add(signature)
        from luggage_planning.planning_scene_client import (
            summarize_planning_scene,
        )

        summary = summarize_planning_scene(scene_msg)
        acm = scene_msg.allowed_collision_matrix
        enabled = 0
        names = [str(name) for name in acm.entry_names]
        for index, name in enumerate(names):
            if index >= len(acm.entry_values):
                break
            enabled += sum(1 for flag in acm.entry_values[index].enabled if flag)
        validity = self._goal_on_submission_scene(scene_msg, to_node.q)
        document = {
            "when": "before_move_group_call",
            "edge_class": "PORTAL_TRANSIT",
            "error_code": int(code),
            "signature": signature,
            "start_q": [float(v) for v in from_node.q],
            "goal_q": [float(v) for v in to_node.q],
            "goal_constraints": [{
                "joint_name": str(item.joint_name),
                "position": float(item.position),
                "tolerance_above": float(item.tolerance_above),
                "tolerance_below": float(item.tolerance_below),
            } for item in constraints.joint_constraints],
            "world_object_ids": summary.get("world_object_ids"),
            "world_objects": summary.get("world_objects"),
            "attached": summary.get("attached"),
            "acm": {
                "n_names": len(names),
                "n_enabled_flags": enabled,
            },
            "validity_on_submission_scene": validity,
        }
        path = os.path.join(
            directory, "portal_%02d.json" % (len(seen) - 1))
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(document, handle, indent=1, default=str)
            handle.write("\n")

    def _goal_on_submission_scene(self, scene_msg, goal_q):
        """Validate the goal joints against the pre-call scene copy."""
        from luggage_msgs.msg import RouteLegV2
        from luggage_msgs.srv import ValidateRouteV2
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        import threading

        if not hasattr(self, "_validity_client"):
            self._validity_client = self._node.create_client(
                ValidateRouteV2, "/validate_route_v2")
        client = self._validity_client
        if not client.wait_for_service(timeout_sec=3.0):
            return {"status": "not_evaluated", "reason": "service_unavailable"}
        from luggage_planning.moveit_state_validity import (
            FrozenSceneComposer,
        )
        from luggage_planning.planning_scene_client import acm_pair_allowed

        raw_allowed = acm_pair_allowed(
            scene_msg.allowed_collision_matrix,
            "elfin_base", "robot_pedestal")
        # The certifier's frozen scene allows this one bolted pair.
        # Judge the pre-call copy under that same matrix. This does not
        # read the scene again after the call.
        scene_msg.allowed_collision_matrix = (
            FrozenSceneComposer._composed_acm(
                FrozenSceneComposer, scene_msg.allowed_collision_matrix))
        scene_msg.is_diff = False
        request = ValidateRouteV2.Request()
        request.request_id = "portal-goal"
        request.route_digest = "portal-goal"
        request.scene_digest = "portal-goal"
        request.load_state_scenes = [scene_msg]
        request.max_sample_dt_ns = 0.0
        request.distance_stride = 1000
        request.budget_s = 2.0
        request.joint_velocity_limits = [0.0] * len(JOINT_NAMES)
        leg = RouteLegV2()
        leg.edge_id = "portal-goal"
        leg.edge_class = "PORTAL_TRANSIT"
        leg.carrying = True
        leg.load_state_index = 0
        leg.joint_names = list(JOINT_NAMES)
        leg.n_samples = 1
        leg.sample_times_ns = [0.0]
        leg.sample_positions_flat = [float(v) for v in goal_q]
        knots = JointTrajectory()
        knots.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in goal_q]
        knots.points = [point]
        leg.knots = knots
        request.legs = [leg]
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(8.0):
            return {"status": "not_evaluated", "reason": "timeout"}
        response = future.result()
        if response is None or not response.leg_results:
            return {"status": "not_evaluated", "reason": "empty",
                    "service_detail": "" if response is None else str(
                        response.detail)}
        result = response.leg_results[0]
        return {
            "status": "queried",
            "valid": bool(result.valid),
            "reason": str(result.reason),
            "detail": str(result.detail),
            "acm_rule": "certifier_composed",
            "structure_pair_allowed_before_compose": raw_allowed,
            "contacts": [
                {"body_1": str(item.body_1), "body_2": str(item.body_2)}
                for item in list(result.contacts)[:8]],
        }

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
        submission_scene = None
        if klass == EdgeClass.PORTAL_TRANSIT and os.environ.get(
                "ST09_PORTAL_CAPTURE_DIR"):
            submission_scene = self._portal_submission_scene()
        self._call_counts["move_group"] += 1
        began = time.monotonic()
        result = self._send_action_goal(goal)
        self._call_durations["move_group"].append(
            round(time.monotonic() - began, 6))
        self._request_dumps.append({
            "planner": "joint", "edge_class": klass.value,
            "planner_id": self._planner_id,
            "from": from_node.node_id, "to": to_node.node_id,
            "from_q": [round(float(v), 6) for v in from_node.q],
            "to_q": [round(float(v), 6) for v in to_node.q],
            "error_code": int(result.error_code.val) if result else -1})
        if submission_scene is not None and str(from_node.load_state) == "carrying":
            from luggage_planning.joint_boundary_capture import capture_boundary
            capture_index = getattr(self, "_carrying_capture_index", 0)
            if capture_boundary(os.environ.get("ST09_PORTAL_CAPTURE_DIR"),
                                capture_index, submission_scene, goal, result):
                self._carrying_capture_index = capture_index + 1
        trajectory = getattr(result, "planned_trajectory", None) if result \
            else None
        joint_trajectory = getattr(trajectory, "joint_trajectory", None) \
            if trajectory is not None else None
        if result is None or int(result.error_code.val) != 1 or \
                joint_trajectory is None or not joint_trajectory.points:
            # [ST09-I] request-level diagnostics: the refused edge's exact
            # endpoints, so a live "start in collision" names its state.
            try:
                self._node.get_logger().error(
                    "edge joint failed %s from=%s to=%s code=%s" % (
                        klass.value,
                        [round(float(v), 3) for v in from_node.q],
                        [round(float(v), 3) for v in to_node.q],
                        int(result.error_code.val) if result else -1))
            except Exception:  # noqa: BLE001 - diagnostics only
                pass
            code = int(result.error_code.val) if result else -1
            self._record_failure(klass.value, "moveit_error", error_code=code)
            if klass == EdgeClass.PORTAL_TRANSIT:
                self._keep_portal_failure(
                    from_node, to_node, code, submission_scene, constraints)
            raise RoutePlanningError("EDGE_BUILD_FAILED", {
                "kind": "moveit_error",
                "error_code": code})
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
        insert_capture_index = getattr(self, "_insert_capture_index", 0)
        capture_insert = (klass == EdgeClass.INSERT and insert_capture_index < 4
                          and os.environ.get("ST09_PORTAL_CAPTURE_DIR"))
        submission_scene = self._portal_submission_scene() if capture_insert else None
        self._call_counts["cartesian"] += 1
        began = time.monotonic()
        response = self._send_cartesian_request(request)
        if capture_insert:
            from luggage_planning.joint_boundary_capture import capture_insert_boundary
            if capture_insert_boundary(
                    os.environ.get("ST09_PORTAL_CAPTURE_DIR"), insert_capture_index,
                    submission_scene, request, response):
                self._insert_capture_index = insert_capture_index + 1
        self._call_durations["cartesian"].append(
            round(time.monotonic() - began, 6))
        self._request_dumps.append({
            "planner": "cartesian", "edge_class": klass.value,
            "from": from_node.node_id, "to": to_node.node_id,
            "from_q": [round(float(v), 6) for v in from_node.q],
            "to_q": [round(float(v), 6) for v in to_node.q],
            "from_position": [round(float(v), 6) for v in from_node.tcp_position],
            "to_position": [round(float(v), 6) for v in to_node.tcp_position],
            "waypoints": len(poses),
            "max_step_m": self._cartesian_max_step,
            "jump_threshold": self._jump_threshold,
            "avoid_collisions": self._avoid_collisions,
            "fraction": round(float(response.fraction), 6)
            if response else None})
        if response is None or not response.solution.joint_trajectory.points:
            self._record_failure(klass.value, "moveit_error",
                                 fraction=None)
            raise RoutePlanningError("EDGE_BUILD_FAILED", {
                "kind": "moveit_error", "fraction": None})
        fraction = float(response.fraction)
        if fraction < CARTESIAN_MIN_FRACTION:
            # [ST09-I] request-level diagnostics (same as the joint
            # planner): the refused edge's exact start state and goal.
            try:
                self._node.get_logger().error(
                    "edge cartesian failed %s from=%s to=%s fraction=%.3f"
                    % (klass.value,
                       [round(float(v), 3) for v in from_node.q],
                       [round(float(v), 3) for v in to_node.q],
                       fraction))
            except Exception:  # noqa: BLE001 - diagnostics only
                pass
            detail = cartesian_failure_detail(
                from_node.q, to_node.q, from_node.tcp_position,
                to_node.tcp_position, fraction, CARTESIAN_MIN_FRACTION,
                len(poses))
            self._record_failure(klass.value, "fraction_below_min", **detail)
            raise RoutePlanningError("EDGE_BUILD_FAILED", detail)
        # [ST09-I] unwrap re-seeded IK samples onto one revolute branch:
        # GetCartesianPath can answer with a joint flipped to its 2*pi
        # equivalent mid-path (bookkeeping, not motion). Unwrapped, the
        # excursion judge sees the true motion and the executor's
        # continuity gate never meets a phantom whole-turn jump.
        points = response.solution.joint_trajectory.points
        for index in range(1, len(points)):
            previous = points[index - 1].positions
            current = points[index].positions
            for joint in range(len(current)):
                turns = round(
                    (current[joint] - previous[joint])
                    / (2.0 * math.pi))
                if turns:
                    current[joint] = current[joint] - turns * 2.0 * math.pi
        timed = self._retime(response.solution.joint_trajectory)
        excursion = _excursion(timed, distance)
        if excursion is not None and excursion["ratio"] > \
                CARTESIAN_EXCURSION_RAD_PER_M:
            motion = {
                "kind": "excursion_exceeded",
                "ratio": round(float(excursion["ratio"]), 4),
                "worst_joint": excursion["worst_joint"],
                "limit": CARTESIAN_EXCURSION_RAD_PER_M,
                "path_length_m": excursion.get("path_length_m"),
                "end_delta_rad": excursion.get("end_delta_rad"),
                "cumulative_rad": excursion.get("cumulative_rad"),
                "max_step_rad": excursion.get("max_step_rad"),
                "start_q": excursion.get("start_q"),
                "end_q": excursion.get("end_q"),
                "from_ik_family": str(getattr(from_node, "ik_family", "")),
                "to_ik_family": str(getattr(to_node, "ik_family", "")),
                "from_q": [round(float(v), 6) for v in from_node.q],
                "to_q": [round(float(v), 6) for v in to_node.q],
            }
            self._record_failure(klass.value, "excursion_exceeded", **motion)
            raise RoutePlanningError("EDGE_BUILD_FAILED", motion)
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
    semantics: floor at 0.05 m; wrist/j6 measured, never refusing).

    [ST09-I] wrap-aware: GetCartesianPath samples can flip a revolute
    joint to its 2*pi equivalent between re-seeded IK samples; that is
    bookkeeping, not motion — measure angular distance on the circle
    (live: portal/approach traverses refused with ratios 6..510 whose
    "excursion" was exactly whole-turn flips)."""
    from luggage_planning.joint_path import max_joint_step

    from luggage_planning.joint_path import joint_motion_report

    report = joint_motion_report(trajectory.positions, path_len_m)
    if report is None:
        return None
    worst = int(report["worst_joint_index"])
    report["worst_joint"] = JOINT_NAMES[worst]
    report["wrist_margin_rad"] = abs(float(trajectory.positions[0][4])) % math.pi
    report["j6_rotation_rad"] = float(report["cumulative_rad"][5]) \
        if len(report["cumulative_rad"]) > 5 else 0.0
    return report
