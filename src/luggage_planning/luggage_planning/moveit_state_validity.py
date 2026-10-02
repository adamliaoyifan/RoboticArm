"""ST09-I: the real MoveIt validity adapter behind C's certifier.

``TrajectoryCertifier`` (ST09-C) samples every route edge at the frozen
temporal bound (<= 10 ms between samples) and asks one question per
sample: is this robot state valid, and what is its clearance? This
module answers through the live ``move_group``:

* Full-robot collision — a zero-length GetCartesianPath probe at the
  sampled joints with ``avoid_collisions`` on. MoveIt checks the whole
  robot (self-collision, static scene, attached payload, placed boxes);
  ``START_STATE_IN_COLLISION`` and any other error code fail the sample.
* Clearance — a conservative analytic distance between the payload's
  feature points (attached edges) or the TCP point (unloaded edges) and
  every world obstacle box, never turning UNKNOWN into free.

Method boundary (mirrored into ``CertificationResult.method_note`` by
the caller): the collision verdicts are discrete MoveIt state checks at
the certified sample spacing, not a mathematical continuity proof; the
clearance number is a point-sampled lower estimate over the payload
feature set. Missing or late MoveIt reports fail closed
(``ContractViolation``), never silently pass.
"""

from __future__ import division

import math
from typing import Callable, Dict, Mapping, Optional, Sequence, Tuple

from st09_contracts._base import ContractViolation

from luggage_planning.moveit_validity_adapter import (
    MoveItValidityAdapter,
    ValiditySample,
)
from luggage_planning.trajectory_certifier import MAX_CERT_SAMPLE_DT_NS

CANONICAL_JOINTS = tuple("elfin_joint%d" % i for i in range(1, 7))
PROBE_TIMEOUT_S = 0.5


def sample_states(trajectory) -> Tuple[Tuple[float, ...], ...]:
    """Interpolated joint states at the frozen certification spacing.

    Mirrors the certifier's subdivision rule exactly: for every knot
    pair, ``subdivisions = max(1, ceil(dt / 10 ms))`` and the interval
    contributes ``subdivisions + 1`` linear samples. A dedicated parity
    test pins this against the certifier's own iteration.
    """
    states = []
    positions = trajectory.positions
    times = trajectory.time_from_start_ns
    for left, right, t0, t1 in zip(positions, positions[1:], times,
                                   times[1:]):
        dt = t1 - t0
        subdivisions = max(1, int((dt + MAX_CERT_SAMPLE_DT_NS - 1)
                                  // MAX_CERT_SAMPLE_DT_NS))
        for step in range(subdivisions + 1):
            fraction = step / subdivisions
            states.append(tuple(
                a + (b - a) * fraction for a, b in zip(left, right)))
    if not states and positions:
        states.append(tuple(float(v) for v in positions[0]))
    return tuple(states)


def _rotate(quaternion_xyzw, vector):
    """Quaternion rotation of one vector (xyzw, normalized input)."""
    ux, uy, uz, qw = quaternion_xyzw
    vx, vy, vz = vector
    cx, cy, cz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    return (
        vx + 2.0 * (uy * cz - uz * cy + qw * cx),
        vy + 2.0 * (uz * cx - ux * cz + qw * cy),
        vz + 2.0 * (ux * cy - uy * cx + qw * cz),
    )


def point_box_distance(point, box) -> float:
    """World point to oriented-box distance; 0.0 when inside.

    ``box`` is a B ``BoxSpec``: world center, size, optional rotation.
    Pure arithmetic — no geometry library dependency.
    """
    quaternion = getattr(box, "quaternion_xyzw", (0.0, 0.0, 0.0, 1.0))
    # inverse rotation of (point - center) into the box frame
    inverse = (quaternion[0], quaternion[1], quaternion[2],
               -quaternion[3])
    delta = (point[0] - box.center[0], point[1] - box.center[1],
             point[2] - box.center[2])
    local = _rotate(inverse, delta)
    half = (box.size_whd[0] * 0.5, box.size_whd[1] * 0.5,
            box.size_whd[2] * 0.5)
    clamped = tuple(
        max(-half[i], min(half[i], local[i])) for i in range(3))
    if clamped == local:
        return 0.0
    return math.sqrt(sum((local[i] - clamped[i]) ** 2 for i in range(3)))


def payload_feature_points(tcp_position, tcp_quaternion, payload):
    """World-frame feature points of the attached payload.

    Corners plus face centres of the box placed at
    ``T_world_box = T_world_tcp * T_tcp_box``.
    """
    from st09_contracts.attachment import compose_transform

    t_world_box = compose_transform(
        tuple(float(v) for v in tcp_position),
        tuple(float(v) for v in tcp_quaternion),
        tuple(float(v) for v in payload.t_tcp_box_translation),
        tuple(float(v) for v in payload.t_tcp_box_quaternion_xyzw))
    center, quaternion = t_world_box[0], t_world_box[1]
    half = [0.5 * float(v) for v in payload.dimensions_wdh]
    points = [tuple(float(v) for v in center)]
    for axis in range(3):
        for sign in (-1.0, 1.0):
            offset = [0.0, 0.0, 0.0]
            offset[axis] = sign * half[axis]
            rotated = _rotate(quaternion, offset)
            points.append((center[0] + rotated[0], center[1] + rotated[1],
                           center[2] + rotated[2]))
    for a in (-1.0, 1.0):
        for b in (-1.0, 1.0):
            for c in (-1.0, 1.0):
                offset = (a * half[0], b * half[1], c * half[2])
                rotated = _rotate(quaternion, offset)
                points.append((center[0] + rotated[0],
                               center[1] + rotated[1],
                               center[2] + rotated[2]))
    return tuple(points)


class MoveItStateValidityAdapter(MoveItValidityAdapter):
    """Answer the certifier's per-sample validity questions via move_group.

    The producer loads the route's edges (with their B ``SceneState``
    per edge) before certification; ``validate_sample`` then applies the
    edge's planning-scene diff once and probes each sampled state.
    """

    METHOD_NOTE = (
        "moveit_state_validity: full-robot collision via zero-length "
        "GetCartesianPath probes at the certified sample spacing "
        "(discrete state checks, not a continuity proof); clearance is "
        "the analytic point-to-box distance over payload feature points "
        "and world obstacle boxes; missing reports fail closed")

    def __init__(self, node, *, fk, scene_client=None,
                 probe_timeout_s=PROBE_TIMEOUT_S, call_log=None):
        self._node = node
        self._fk = fk
        self._probe_timeout_s = float(probe_timeout_s)
        self._scene_client = scene_client
        self._cartesian = None
        self._edges: Dict[str, object] = {}
        self._scene_states: Dict[str, object] = {}
        self._samples: Dict[str, Tuple[Tuple[float, ...], ...]] = {}
        self._scene_applied_edge = None
        self.requests: list = call_log if call_log is not None else []

    # -- producer API ----------------------------------------------------

    def load_edges(self, edges, scene_states: Mapping):
        """Preload certified edges and their per-edge scene states."""
        for edge in edges:
            edge_id = str(edge.edge_id)
            self._edges[edge_id] = edge
            self._samples[edge_id] = sample_states(edge.trajectory)
            state = scene_states.get(edge_id)
            if state is None:
                raise ContractViolation(
                    "CERT_UNKNOWN_INPUT: no scene state for edge %r"
                    % edge_id)
            self._scene_states[edge_id] = state

    # -- certifier protocol ----------------------------------------------

    def validate_sample(self, edge_id: str, sample_index: int) -> ValiditySample:
        edge_id = str(edge_id)
        samples = self._samples.get(edge_id)
        if samples is None:
            raise ContractViolation(
                "CERT_UNKNOWN_INPUT: unknown edge %r" % edge_id)
        if not 0 <= int(sample_index) < len(samples):
            raise ContractViolation(
                "CERT_UNKNOWN_INPUT: sample %r out of range for edge %r"
                % (sample_index, edge_id))
        q = samples[int(sample_index)]
        self._ensure_scene(edge_id)
        valid, min_clearance = self._probe(edge_id, q)
        return ValiditySample(
            edge_id=edge_id,
            sample_index=int(sample_index),
            valid=bool(valid),
            min_clearance_m=float(min_clearance),
            contacts=())

    # -- internals ---------------------------------------------------------

    def _ensure_clients(self):
        if self._cartesian is None:
            from moveit_msgs.srv import GetCartesianPath

            self._cartesian = self._node.create_client(
                GetCartesianPath, "/compute_cartesian_path")

    def _ensure_scene(self, edge_id):
        if self._scene_applied_edge == edge_id:
            return
        self._apply_scene(self._scene_states[edge_id])
        self._scene_applied_edge = edge_id

    def _apply_scene(self, state):
        """Planning-scene diff for one edge: boxes + payload attach state."""
        if self._scene_client is None:
            raise ContractViolation(
                "CERT_UNKNOWN_INPUT: no planning scene client configured")
        from moveit_msgs.msg import (
            AttachedCollisionObject,
            CollisionObject,
            PlanningScene,
        )
        from shape_msgs.msg import SolidPrimitive

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = []
        scene.robot_state.attached_collision_objects = []
        for box in tuple(getattr(state, "placed_boxes", ()) or ()) + (
                (state.released_box, ) if getattr(
                    state, "released_box", None) else ()):
            collision = CollisionObject()
            collision.id = str(box.object_id)
            collision.header.frame_id = "world"
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                float(box.size_whd[0]), float(box.size_whd[1]),
                float(box.size_whd[2])]
            collision.primitives = [primitive]
            collision.primitive_poses = [_pose_message(box)]
            collision.operation = CollisionObject.ADD
            scene.world.collision_objects.append(collision)
        for box in tuple(getattr(state, "boxes", ()) or ()):
            collision = CollisionObject()
            collision.id = str(box.object_id)
            collision.header.frame_id = "world"
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                float(box.size_whd[0]), float(box.size_whd[1]),
                float(box.size_whd[2])]
            collision.primitive_poses = [_pose_message(box)]
            collision.operation = CollisionObject.ADD
            scene.world.collision_objects.append(collision)
        if getattr(state, "payload", None) is not None:
            attached = AttachedCollisionObject()
            attached.link_name = "suction_contact_frame"
            attached.object.id = "cert-payload"
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            payload = state.payload
            primitive.dimensions = [
                float(payload.dimensions_wdh[0]),
                float(payload.dimensions_wdh[1]),
                float(payload.dimensions_wdh[2])]
            attached.object.primitives = [primitive]
            attached.object.primitive_poses = [_pose_from_transform(
                payload.t_tcp_box_translation,
                payload.t_tcp_box_quaternion_xyzw)]
            attached.object.operation = CollisionObject.ADD
            scene.robot_state.attached_collision_objects.append(attached)
        ok, reason = self._scene_client.apply_diff(
            scene, timeout=self._probe_timeout_s)
        if not ok:
            raise ContractViolation(
                "CERT_UNKNOWN_INPUT: planning scene apply failed (%s)"
                % reason)
        self.requests.append({"kind": "scene", "edge": "scene"})

    def _probe(self, edge_id, q):
        """Zero-length cartesian probe at q: full-robot state validity.

        Blocking event wait (the motion-executor pattern): the hosting
        node runs under a MultiThreadedExecutor, so the response is
        completed by another executor thread while this one waits.
        """
        import threading

        from std_msgs.msg import Header

        self._ensure_clients()
        position, quaternion = self._fk(q)
        request = self._cartesian.srv_type.Request()
        request.header = Header(frame_id="world")
        request.start_state.joint_state.name = list(CANONICAL_JOINTS)
        request.start_state.joint_state.position = [float(v) for v in q]
        request.start_state.is_diff = True
        request.group_name = "elfin_arm"
        request.link_name = "suction_contact_frame"
        request.waypoints = [_pose_from_transform(position, quaternion)]
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        future = self._cartesian.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=self._probe_timeout_s):
            raise ContractViolation(
                "CERT_UNKNOWN_INPUT: cartesian probe timed out on edge %r"
                % edge_id)
        response = future.result()
        self.requests.append({
            "kind": "probe", "edge": edge_id,
            "error_code": int(response.error_code.val),
            "fraction": float(response.fraction)})
        valid = int(response.error_code.val) == 1  # MoveItErrorCodes.SUCCESS
        return valid, self.min_clearance(edge_id, q)

    def min_clearance(self, edge_id, q) -> float:
        """Analytic conservative clearance for one sampled state."""
        state = self._scene_states[edge_id]
        position, quaternion = self._fk(q)
        boxes = tuple(getattr(state, "boxes", ()) or ()) + tuple(
            getattr(state, "placed_boxes", ()) or ()) + (
            (state.released_box, ) if getattr(
                state, "released_box", None) else ())
        payload = getattr(state, "payload", None)
        if payload is not None:
            points = payload_feature_points(position, quaternion, payload)
        else:
            points = (tuple(float(v) for v in position),)
        distances = [point_box_distance(point, box)
                     for box in boxes for point in points]
        return min(distances) if distances else 1.0


def _pose_from_transform(translation, quaternion_xyzw):
    from geometry_msgs.msg import Pose

    pose = Pose()
    pose.position.x = float(translation[0])
    pose.position.y = float(translation[1])
    pose.position.z = float(translation[2])
    pose.orientation.x = float(quaternion_xyzw[0])
    pose.orientation.y = float(quaternion_xyzw[1])
    pose.orientation.z = float(quaternion_xyzw[2])
    pose.orientation.w = float(quaternion_xyzw[3])
    return pose


def _pose_message(box):
    return _pose_from_transform(
        box.center, getattr(box, "quaternion_xyzw",
                            (0.0, 0.0, 0.0, 1.0)))
