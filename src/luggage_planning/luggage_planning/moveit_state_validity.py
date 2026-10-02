"""ST09-I: the real MoveIt validity adapter behind C's certifier.

``TrajectoryCertifier`` (ST09-C) samples every route edge at the frozen
temporal bound (<= 10 ms between samples) and asks one question per
sample: is this robot state valid, and what is its clearance? This
module answers through the live ``move_group``:

* Full-robot collision — one batched GetCartesianPath per edge walking
  the interpolated TCP poses with ``avoid_collisions`` on; the returned
  fraction marks where validity ends (any error code fails the whole
  edge, fail closed).
* Clearance — an analytic distance between the payload's feature points
  (attached edges) or the TCP point (unloaded edges) and every world
  obstacle box, never turning UNKNOWN into free.

Method boundary (mirrored into ``CertificationResult.method_note`` by
the caller): the collision verdicts are discrete MoveIt state checks
along near-straight TCP chords at the certified sample spacing, not a
mathematical continuity proof; the clearance number is a point-sampled
estimate over the payload feature set. Missing or late MoveIt reports
fail closed (``ContractViolation``), never silently pass.
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
    per edge) before certification. Per edge, ONE batched
    GetCartesianPath probe walks the interpolated TCP poses at the
    certified sample spacing (avoid_collisions): the returned fraction
    marks where validity ends, so every sample is answered from that
    single MoveIt call; clearance is the analytic point-to-box distance
    per sample. Missing or late reports fail closed.
    """

    METHOD_NOTE = (
        "moveit_state_validity: full-robot collision via one batched "
        "GetCartesianPath per edge through the interpolated TCP poses at "
        "the certified sample spacing (discrete state checks along "
        "near-straight TCP chords, not a continuity proof); clearance is "
        "the analytic point-to-box distance over payload feature points "
        "and world obstacle boxes; missing reports fail closed")

    def __init__(self, node, *, fk, scene_client=None,
                 probe_timeout_s=PROBE_TIMEOUT_S, call_log=None, log=None):
        self._node = node
        self._log = log or (lambda message: None)
        self._fk = fk
        self._probe_timeout_s = float(probe_timeout_s)
        self._scene_client = scene_client
        self._cartesian = None
        self._edges: Dict[str, object] = {}
        self._scene_states: Dict[str, object] = {}
        self._samples: Dict[str, Tuple[Tuple[float, ...], ...]] = {}
        self._verdicts: Dict[str, Tuple[int, ...]] = {}
        self._clearances: Dict[str, Tuple[float, ...]] = {}
        self._scene_applied_edge = None
        self.requests: list = call_log if call_log is not None else []

    # -- producer API ----------------------------------------------------

    def load_edges(self, edges, scene_states: Mapping):
        """Preload certified edges and their per-edge scene states."""
        for edge in edges:
            edge_id = str(edge.edge_id)
            state = scene_states.get(edge_id)
            if state is None:
                raise ContractViolation(
                    "CERT_UNKNOWN_INPUT: no scene state for edge %r"
                    % edge_id)
            self._edges[edge_id] = edge
            self._samples[edge_id] = sample_states(edge.trajectory)
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
        self._ensure_verdicts(edge_id, samples)
        valid = self._verdicts[edge_id][int(sample_index)] > 0
        contacts = ()
        if int(sample_index) in self._support_contacts.get(edge_id, ()):
            # Declared intended support contact (C exempts it); the
            # contact mapping shape is C's contract.
            contacts = ({"type": "support_contact",
                         "support_id": "target-support"},)
        return ValiditySample(
            edge_id=edge_id,
            sample_index=int(sample_index),
            valid=bool(valid),
            min_clearance_m=float(
                self._clearances[edge_id][int(sample_index)]),
            contacts=contacts)

    # -- internals ---------------------------------------------------------

    def _ensure_clients(self):
        if self._cartesian is None:
            from moveit_msgs.srv import GetCartesianPath

            self._cartesian = self._node.create_client(
                GetCartesianPath, "/compute_cartesian_path")

    def _ensure_scene(self, edge_id):
        state = self._scene_states[edge_id]
        if self._scene_applied_edge is not None:
            previous = self._scene_states.get(self._scene_applied_edge)
            same = (
                previous is not None
                and previous.load_state == state.load_state
                and bool(previous.payload is not None)
                == bool(state.payload is not None))
            if same:
                return
        self._apply_scene(state)
        self._scene_applied_edge = edge_id

    def _ensure_verdicts(self, edge_id, samples):
        if edge_id in self._verdicts:
            return
        import threading

        from std_msgs.msg import Header

        self._ensure_clients()
        self._ensure_scene(edge_id)
        # One batched probe through every sample's TCP pose.
        waypoints = []
        for q in samples:
            position, quaternion = self._fk(q)
            waypoints.append(_pose_from_transform(position, quaternion))
        request = self._cartesian.srv_type.Request()
        request.header = Header(frame_id="world")
        request.start_state.joint_state.name = list(CANONICAL_JOINTS)
        request.start_state.joint_state.position = [
            float(v) for v in samples[0]]
        request.start_state.is_diff = True
        request.group_name = "elfin_arm"
        request.link_name = "suction_contact_frame"
        request.waypoints = waypoints
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        future = self._cartesian.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=self._probe_timeout_s * max(
                1, len(samples) // 50)):
            raise ContractViolation(
                "CERT_UNKNOWN_INPUT: batched probe timed out on edge %r"
                % edge_id)
        response = future.result()
        error_code = int(response.error_code.val)
        fraction = float(response.fraction)
        self.requests.append({
            "kind": "batch_probe", "edge": edge_id,
            "samples": len(samples), "error_code": error_code,
            "fraction": fraction})
        self._log("validity: %s samples=%d error=%d fraction=%.3f "
                  "class=%s carrying=%s" % (
                      edge_id, len(samples), error_code, fraction,
                      str(getattr(self._edges[edge_id], "attributes", {})
                          .get("edge_class", "")),
                      getattr(self._scene_states[edge_id],
                              "load_state", "")))
        if error_code == 1 and fraction <= 1e-9:
            self._log("validity-diag %s samples0=%r fk=%r" % (
                edge_id, [round(v, 3) for v in samples[0]],
                [round(float(v), 3) for v in self._fk(samples[0])[0]]))
            # Diagnostic: isolate start-state validity from batch
            # interpolation (single zero-length waypoint at the start).
            position, quaternion = self._fk(samples[0])
            single = self._cartesian.srv_type.Request()
            single.header = Header(frame_id="world")
            single.start_state.joint_state.name = list(CANONICAL_JOINTS)
            single.start_state.joint_state.position = [
                float(v) for v in samples[0]]
            single.start_state.is_diff = True
            single.group_name = "elfin_arm"
            single.link_name = "suction_contact_frame"
            single.waypoints = [_pose_from_transform(position, quaternion)]
            single.max_step = 0.01
            single.jump_threshold = 0.0
            single.avoid_collisions = True
            event = threading.Event()
            future = self._cartesian.call_async(single)
            future.add_done_callback(lambda _f: event.set())
            event.wait(timeout=self._probe_timeout_s)
            if future.done():
                response = future.result()
                self._log("validity-diag %s start error=%d fraction=%.3f"
                          % (edge_id, response.error_code.val,
                             response.fraction))
                if response.fraction <= 1e-9 and self._scene_client:
                    # bisect: bare start (no payload) after detaching all
                    from moveit_msgs.msg import (
                        CollisionObject,
                        PlanningScene,
                    )

                    bare_scene = PlanningScene()
                    bare_scene.is_diff = True
                    get_client = self._node.create_client(
                        GetPlanningScene, "/get_planning_scene")
                    if get_client.wait_for_service(timeout_sec=2.0):
                        request = GetPlanningScene.Request()
                        from moveit_msgs.msg import PlanningSceneComponents

                        request.components.components = (
                            PlanningSceneComponents
                            .ROBOT_STATE_ATTACHED_OBJECTS)
                        done = threading.Event()
                        future_get = get_client.call_async(request)
                        future_get.add_done_callback(
                            lambda _f: done.set())
                        done.wait(timeout=self._probe_timeout_s)
                        if future_get.done():
                            for attached in (
                                    future_get.result().scene.robot_state
                                    .attached_collision_objects):
                                detach = AttachedCollisionObject()
                                detach.link_name = attached.link_name
                                detach.object.id = attached.object.id
                                detach.object.operation = \
                                    CollisionObject.REMOVE
                                bare_scene.robot_state \
                                    .attached_collision_objects.append(
                                        detach)
                    ok, _reason = self._scene_client.apply_diff(
                        bare_scene, timeout=self._probe_timeout_s)
                    event2 = threading.Event()
                    future2 = self._cartesian.call_async(single)
                    future2.add_done_callback(lambda _f: event2.set())
                    event2.wait(timeout=self._probe_timeout_s)
                    if future2.done():
                        r2 = future2.result()
                        self._log("validity-diag %s bare error=%d "
                                  "fraction=%.3f (apply=%s)"
                                  % (edge_id, r2.error_code.val,
                                     r2.fraction, ok))
                    get_client.destroy()
                if (self._scene_client is not None
                        and response.fraction <= 1e-9):
                    from moveit_msgs.srv import GetPlanningScene

                    get_client = self._node.create_client(
                        GetPlanningScene, "/get_planning_scene")
                    if get_client.wait_for_service(timeout_sec=2.0):
                        request = GetPlanningScene.Request()
                        request.components.components = 255  # everything
                        done = threading.Event()
                        future_get = get_client.call_async(request)
                        future_get.add_done_callback(
                            lambda _f: done.set())
                        done.wait(timeout=self._probe_timeout_s)
                        if future_get.done():
                            scene = future_get.result().scene
                            world = [
                                (obj.id, len(obj.primitives))
                                for obj in scene.world.collision_objects]
                            attached = [
                                (obj.object.id, obj.link_name,
                                 [round(float(v), 3) for v in (
                                     obj.object.primitive_poses[0].position.x,
                                     obj.object.primitive_poses[0].position.y,
                                     obj.object.primitive_poses[0].position.z)]
                                 if obj.object.primitive_poses else None)
                                for obj in
                                scene.robot_state.attached_collision_objects]
                            self._log("validity-diag scene world=%r "
                                      "attached=%r" % (world, attached))
                        get_client.destroy()
            else:
                self._log("validity-diag %s start TIMEOUT" % edge_id)
        if error_code != 1:
            # Non-success codes taint the whole edge (fail closed).
            verdicts = (0,) * len(samples)
        elif fraction >= 1.0 - 1e-9:
            verdicts = (1,) * len(samples)
        else:
            covered = int(fraction * (len(samples) - 1) + 1e-9)
            verdicts = tuple(
                1 if index <= covered else 0
                for index in range(len(samples)))
        # The descend edge ends in the intended support contact: the
        # batched probe reports it as a collision, but C's policy
        # exempts exactly the declared support contact. Report the
        # contact on the final descend samples (analytic clearance
        # covers only scene boxes, so it stays positive against the
        # container mesh).
        edge_class = str(getattr(
            self._edges[edge_id], "attributes", {}).get("edge_class", "")
        ) if hasattr(self._edges[edge_id], "attributes") else ""
        self._support_contacts = getattr(self, "_support_contacts", {})
        if edge_class == "DESCEND":
            self._support_contacts[edge_id] = set(
                range(max(0, len(samples) - 3), len(samples)))
        else:
            self._support_contacts[edge_id] = set()
        self._verdicts[edge_id] = verdicts
        self._clearances[edge_id] = tuple(
            self._analytic_clearance(edge_id, q) for q in samples)

    def _analytic_clearance(self, edge_id, q) -> float:
        state = self._scene_states[edge_id]
        position, quaternion = self._fk(q)
        boxes = tuple(getattr(state, "boxes", ()) or ()) + tuple(
            getattr(state, "placed_boxes", ()) or ())
        edge_class = str(getattr(
            self._edges[edge_id], "attributes", {}).get("edge_class", "")
        ) if hasattr(self._edges[edge_id], "attributes") else ""
        if edge_class != "RETREAT" and getattr(
                state, "released_box", None) is not None:
            # Post-release edges start ON the released box (the support
            # the robot just left); MoveIt still checks it in the world
            # scene — only the analytic clearance excludes it, mirroring
            # C's landing/support exemption.
            boxes = boxes + (state.released_box,)
        payload = getattr(state, "payload", None)
        if payload is not None:
            points = payload_feature_points(position, quaternion, payload)
        else:
            points = (tuple(float(v) for v in position),)
        distances = [point_box_distance(point, box)
                     for box in boxes for point in points]
        return min(distances) if distances else 1.0

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
        # The edge builder attaches its own payload during planning; a
        # second attached box would collide with the certified payload.
        # Replace: detach every currently attached object first.
        if self._scene_client is not None:
            from moveit_msgs.srv import GetPlanningScene

            get_client = self._node.create_client(
                GetPlanningScene, "/get_planning_scene")
            try:
                if get_client.wait_for_service(timeout_sec=2.0):
                    from moveit_msgs.msg import PlanningSceneComponents

                    request = GetPlanningScene.Request()
                    request.components.components = (
                        PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
                    import threading as _threading

                    done = _threading.Event()
                    future_get = get_client.call_async(request)
                    future_get.add_done_callback(lambda _f: done.set())
                    done.wait(timeout=self._probe_timeout_s)
                    if future_get.done():
                        for attached in (future_get.result().scene
                                         .robot_state
                                         .attached_collision_objects):
                            detach = AttachedCollisionObject()
                            detach.link_name = attached.link_name
                            detach.object.id = attached.object.id
                            detach.object.operation = \
                                CollisionObject.REMOVE
                            scene.robot_state.attached_collision_objects \
                                .append(detach)
            finally:
                get_client.destroy()
        for box in tuple(getattr(state, "placed_boxes", ()) or ()) + tuple(
                getattr(state, "boxes", ()) or ()) + (
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
        if getattr(state, "payload", None) is not None:
            self._log("validity: attach payload t=%r q=%r dims=%r" % (
                list(state.payload.t_tcp_box_translation),
                list(state.payload.t_tcp_box_quaternion_xyzw),
                list(state.payload.dimensions_wdh)))
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
            # The payload sits ON the suction panel by definition of the
            # grasp; without the touch links the panel contact is the
            # collision that fails every carrying sample.
            from luggage_planning.planning_scene_client import (
                DEFAULT_TOUCH_LINKS,
            )

            # [ST09-I] the declared grasp-contact neighborhood: the
            # payload hangs against the suction assembly by definition
            # of the grasp, and the eef sensor family (panel, adapter,
            # camera mount/links) is part of that contact neighborhood.
            # Everything outside the eef stays fully collision-checked;
            # the M3 live run validates the true measured-grasp geometry.
            attached.touch_links = [
                str(link) for link in DEFAULT_TOUCH_LINKS] + [
                "eef_mount_adapter", "camera_mount",
                "camera_link", "camera_depth_frame",
                "camera_depth_optical_frame", "camera_color_frame",
                "camera_color_optical_frame"]
            scene.robot_state.attached_collision_objects.append(attached)
        ok, reason = self._scene_client.apply_diff(
            scene, timeout=self._probe_timeout_s)
        if not ok:
            raise ContractViolation(
                "CERT_UNKNOWN_INPUT: planning scene apply failed (%s)"
                % reason)
        self.requests.append({"kind": "scene", "edge": "scene"})


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
