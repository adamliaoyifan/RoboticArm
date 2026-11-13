"""ST09-I: the real MoveIt validity adapter behind C's certifier.

``TrajectoryCertifier`` (ST09-C) samples every route edge at the frozen
temporal bound (<= 10 ms between samples) and asks one question per
sample: is this robot state valid, and what is its clearance? This
module answers through the batched ``ValidateRouteV2`` service:

* The ORIGINAL joint states of the whole route are certified in ONE
  request — no GetCartesianPath, no IK, no re-planning, and no
  mutation of the live planning scene (the [ST09-I] frame bug that
  parked the certified payload inside the pedestal came from exactly
  that live-scene attach path; certification now composes a frozen
  private scene copy instead).
* The frozen scene carries the live world (container mesh, pedestal,
  placed boxes), the live ACM (SRDF defaults preserved — an empty ACM
  would make adjacent links collide), the payload attached through the
  shared link-relative ``build_attach_scene`` construction, and the
  DECLARED structure allowance (base plate resting on its stand),
  recorded separately from measured distances.
* Sampling positions follow the controller's actual interpolation
  (quintic Hermite when knot velocities+accelerations are present,
  cubic with velocities only, linear otherwise) at C's exact
  subdivision grid, knots included.
* The certified trajectory is CONTACT-FREE end to end ([ST09-I]
  release policy, 2026-10-02): the descend ends with the payload
  bottom RELEASE_DROP_M above the support and release is vacuum-off
  free fall, so no sample may touch anything beyond the declared
  allowances (the suction seal, the bolted base pair). Any contact is
  a real collision with the evidence attached; landed quality is
  measured post-release (err_xy, perceived-vs-actual z), never
  excused inside certification by a contact predicate.

Method boundary (mirrored into ``CertificationResult.method_note`` by
the caller): the verdicts are discrete MoveIt state checks at the
certified sample spacing under the controller's interpolation model —
not a mathematical continuity proof; distances are measured by MoveIt
against the frozen scene (exempt pairs excluded), NaN entries inherit
the nearest measured value. Missing, late or echo-mismatched service
reports fail closed (``ContractViolation``), never silently pass.
"""

from __future__ import division

import hashlib
import json
import os
import threading
from typing import Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from luggage_planning.planning_contract._base import ContractViolation

from luggage_planning.motion.moveit_validity_adapter import (
    MoveItValidityAdapter,
    ValiditySample,
)
from luggage_planning.motion.trajectory_certifier import MAX_CERT_SAMPLE_DT_NS

CANONICAL_JOINTS = tuple("elfin_joint%d" % i for i in range(1, 7))
PAYLOAD_OBJECT_ID = "cert-payload"
#: [ST09-I] the suction interface the payload physically seals against.
#: Deliberately NOT the camera/mount/eef neighbourhood: links outside
#: the seal stay fully collision-checked.
SUCTION_INTERFACE_TOUCH_LINKS = (
    "suction_panel", "suction_contact_frame")
#: Declared structure allowances of the frozen scene (the robot base
#: plate rests on its own pedestal stand — a bolted pair, not an
#: obstacle). Recorded separately from measured distances; never a
#: blanket relaxation.
DECLARED_STRUCTURE_PAIRS = (
    ("elfin_base", "robot_pedestal"),)
#: Support-object resolution order: the landing surface inside the
#: container. The mock chain's static scene sync names the container
#: ``airport_container_real``; production resolves the same way (the
#: mesh the placement targets), fail closed when absent.
SUPPORT_OBJECT_HINTS = ("airport_container_real",)
#: Load-state artifacts of the PLANNING phase: B's edge builder leaves
#: its released box (and any payload leftovers) in the live scene after
#: planning. The frozen base scene strips them — they belong to specific
#: legs' load states and the per-leg diffs add exactly the ones that
#: state declares (a carrying descend must never see the box it has not
#: released yet).
PLANNING_ARTIFACT_PREFIXES = ("released-", "route-released-box",
                              "cert-payload", "b-payload", "pickup_box")

SERVICE_NAME = "/validate_route_v2"
CERT_BUDGET_S = 2.0            # frozen certification_timeout_s
SERVICE_TIMEOUT_S = 5.0
#: Measured-distance cadence: ONE MoveIt distance query per leg, at its
#: first sample. Collision, joint-limit and constraint checks stay
#: PER-SAMPLE; only the informational distance measurement is limited
#: (a GLOBAL distance query against the container mesh costs ~150 ms on
#: this host, so a dense cadence alone exceeds the frozen 2 s budget;
#: the collision verdict itself is sub-millisecond per sample and never
#: strided). Unmeasured entries inherit the nearest measured value; the
#: cadence is declared in METHOD_NOTE.
DISTANCE_STRIDE = 1 << 30


# ---------------------------------------------------------------- sampling


def sample_grid(trajectory) -> List[float]:
    """C's subdivision grid, interval-major, endpoints included.

    Index-parity oracle: the certifier requests samples in exactly this
    order (``subdivisions + 1`` per interval), duplicate knot entries
    included, so the adapter must answer this precise index sequence.
    """
    times: List[float] = []
    stamps = [int(stamp) for stamp in trajectory.time_from_start_ns]
    for t0, t1 in zip(stamps, stamps[1:]):
        dt = t1 - t0
        subdivisions = max(1, int((dt + MAX_CERT_SAMPLE_DT_NS - 1)
                                  // MAX_CERT_SAMPLE_DT_NS))
        for step in range(subdivisions + 1):
            times.append(t0 + dt * (step / subdivisions))
    if not times and stamps:
        times.append(float(stamps[0]))
    return times


def _quintic_row(left, right, velocity_l, velocity_r, accel_l, accel_r,
                 s, dt):
    """Quintic Hermite position row at normalized ``s`` in [0, 1]."""
    s2 = s * s
    s3 = s2 * s
    s4 = s3 * s
    s5 = s4 * s
    h_left = 1 - 10 * s3 + 15 * s4 - 6 * s5
    h_right = 10 * s3 - 15 * s4 + 6 * s5
    h_vl = s - 6 * s3 + 8 * s4 - 3 * s5
    h_vr = -4 * s3 + 7 * s4 - 3 * s5
    h_al = 0.5 * (s2 - 3 * s3 + 3 * s4 - s5)
    h_ar = 0.5 * (s3 - 2 * s4 + s5)
    return tuple(
        left[j] * h_left + right[j] * h_right
        + velocity_l[j] * dt * h_vl + velocity_r[j] * dt * h_vr
        + accel_l[j] * dt * dt * h_al + accel_r[j] * dt * dt * h_ar
        for j in range(len(left)))


def _cubic_row(left, right, velocity_l, velocity_r, s, dt):
    """Cubic Hermite position row (positions + velocities only)."""
    s2 = s * s
    s3 = s2 * s
    h_left = 2 * s3 - 3 * s2 + 1
    h_right = -2 * s3 + 3 * s2
    h_vl = s3 - 2 * s2 + s
    h_vr = s3 - s2
    return tuple(
        left[j] * h_left + right[j] * h_right
        + velocity_l[j] * dt * h_vl + velocity_r[j] * dt * h_vr
        for j in range(len(left)))


def interpolated_positions(trajectory) -> Tuple[Tuple[float, ...], ...]:
    """Positions on C's grid under the controller's interpolation.

    Joint Trajectory Controller (humble, default ``splines``)
    interpolates each knot interval through the boundary derivatives
    the trajectory itself carries: quintic when velocities AND
    accelerations are present, cubic with velocities only, linear
    otherwise. Certifying linear samples of a spline-tracked leg would
    miss mid-segment excursions, so the same rule is mirrored here.
    """
    positions = trajectory.positions
    velocities = trajectory.velocities
    accelerations = trajectory.accelerations
    stamps = [int(stamp) for stamp in trajectory.time_from_start_ns]
    rows: List[Tuple[float, ...]] = []
    for index in range(len(positions) - 1):
        left, right = positions[index], positions[index + 1]
        t0, t1 = stamps[index], stamps[index + 1]
        dt_ns = t1 - t0
        dt = dt_ns / 1e9  # seconds: the derivative boundary terms
        subdivisions = max(1, int((dt_ns + MAX_CERT_SAMPLE_DT_NS - 1)
                                  // MAX_CERT_SAMPLE_DT_NS))
        has_velocity = len(velocities) > index + 1
        has_accel = len(accelerations) > index + 1
        velocity_l = list(velocities[index]) if has_velocity else None
        velocity_r = list(velocities[index + 1]) if has_velocity else None
        accel_l = list(accelerations[index]) if (
            has_accel and len(accelerations[index]) == len(left)) else None
        accel_r = list(accelerations[index + 1]) if (
            has_accel and len(accelerations[index + 1]) == len(right)) \
            else None
        for step in range(subdivisions + 1):
            s = step / subdivisions
            if velocity_l is not None and accel_l is not None \
                    and accel_r is not None:
                row = _quintic_row(left, right, velocity_l, velocity_r,
                                   accel_l, accel_r, s, dt)
            elif velocity_l is not None:
                row = _cubic_row(left, right, velocity_l, velocity_r, s,
                                 dt)
            else:
                row = tuple(a + (b - a) * s for a, b in zip(left, right))
            rows.append(row)
    if not rows and positions:
        rows.append(tuple(float(v) for v in positions[0]))
    return tuple(rows)


def sample_states(trajectory) -> Tuple[Tuple[float, ...], ...]:
    """C-parity sampled states (controller interpolation), public form.

    Kept as the module's sampling oracle: the parity test pins the
    index alignment against the certifier's own iteration.
    """
    return interpolated_positions(trajectory)


# ------------------------------------------------------- scene composing


def _scene_digest(scenes) -> str:
    """Digest every scene field, excluding nondeterministic CDR padding bytes."""
    from rosidl_runtime_py.convert import message_to_ordereddict

    digest = hashlib.sha256()
    for scene in scenes:
        material = json.dumps(message_to_ordereddict(scene), sort_keys=True,
                              separators=(",", ":"), allow_nan=False).encode("utf-8")
        digest.update(len(material).to_bytes(8, "big"))
        digest.update(material)
    return digest.hexdigest()


class FrozenSceneComposer(object):
    """Compose the private certification scenes from a live read.

    One frozen FULL scene per DISTINCT load state of the route (no
    diffs, no MoveIt object lifecycle semantics — the [ST09-I] finding:
    an attached-REMOVE detaches into the world at the release pose and
    the touch-link ACM entries LEAK, letting the arm sweep through the
    released box undetected):

    * base world: the live static scene (container mesh, pedestal,
      already-placed bodies) minus anything an edge build inserts
      (snapshot columns, the released box, the attached payload).
      Those artifacts are not part of the request scene;
    * carrying and unloaded scenes both add the request occupancy
      columns from ``SceneState.placed_boxes``;
    * carrying also attaches the payload; unloaded also adds the
      released box as a world object. Edge-build order cannot change
      either scene.
    """

    def __init__(self, scene_client, *, log=None):
        self._scene_client = scene_client
        self._log = log or (lambda message: None)

    def read_live(self, timeout_s=5.0):
        from moveit_msgs.msg import PlanningSceneComponents

        scene = self._scene_client.get_scene(
            PlanningSceneComponents.SCENE_SETTINGS
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS,
            timeout=timeout_s)
        if scene is None:
            raise ContractViolation(
                "CERT_SCENE_UNAVAILABLE: live scene read failed")
        return scene

    def compose(self, live, start_q, scene_states, edges):
        """Return (scenes, per-edge load-state indices, support id)."""
        from moveit_msgs.msg import PlanningScene

        support_id = self._resolve_support(live)
        base = self._base_scene(live, start_q)
        scenes = [base]
        indices = {}
        for edge in edges:
            state = scene_states[str(edge.edge_id)]
            key = self._state_key(state)
            if key in indices:
                continue
            payload = getattr(state, "payload", None)
            released = getattr(state, "released_box", None)
            placed = tuple(getattr(state, "placed_boxes", ()) or ())
            if payload is None and released is None and not placed:
                indices[key] = 0
                continue
            scene = self._copy_scene(base)
            for box in placed:
                _append_world_box(scene, box)
            if released is not None:
                _append_world_box(scene, released)
                scene.name = "st09-cert-frozen:%s" % str(released.object_id)
            if payload is not None:
                attach = _attach_payload_scene(payload)
                scene.robot_state.attached_collision_objects = \
                    attach.robot_state.attached_collision_objects
                scene.name = "st09-cert-frozen:carrying"
            scenes.append(scene)
            indices[key] = len(scenes) - 1
        return scenes, indices, support_id

    @staticmethod
    def _state_key(state):
        payload = getattr(state, "payload", None)
        released = getattr(state, "released_box", None)
        placed = tuple(
            (str(box.object_id), tuple(box.center), tuple(box.size_whd),
             tuple(box.quaternion_xyzw))
            for box in (getattr(state, "placed_boxes", ()) or ()))
        if payload is not None:
            return ("carrying", tuple(payload.t_tcp_box_translation),
                    tuple(payload.t_tcp_box_quaternion_xyzw),
                    tuple(payload.dimensions_wdh), placed)
        if released is not None:
            return ("released", str(released.object_id),
                    tuple(released.center), tuple(released.size_whd), placed)
        return ("base", placed)

    def _base_scene(self, live, start_q):
        from moveit_msgs.msg import PlanningScene

        base = PlanningScene()
        base.name = "st09-cert-frozen"
        base.is_diff = False
        base.world.collision_objects = [
            _copy_collision_object(obj)
            for obj in live.world.collision_objects
            if not _is_planning_artifact(str(obj.id))]
        base.allowed_collision_matrix = self._composed_acm(
            live.allowed_collision_matrix)
        base.robot_state.joint_state.name = list(CANONICAL_JOINTS)
        base.robot_state.joint_state.position = [
            float(v) for v in start_q]
        return base

    @staticmethod
    def _copy_scene(scene):
        from rclpy.serialization import (deserialize_message,
                                         serialize_message)
        from moveit_msgs.msg import PlanningScene as PlanningSceneMsg

        return deserialize_message(serialize_message(scene),
                                   PlanningSceneMsg)

    def _resolve_support(self, live):
        ids = [str(obj.id) for obj in live.world.collision_objects]
        for hint in SUPPORT_OBJECT_HINTS:
            if hint in ids:
                return hint
        for object_id in ids:
            if "container" in object_id.lower():
                return object_id
        raise ContractViolation(
            "CERT_SUPPORT_UNRESOLVED: no container/support object in "
            "the frozen scene (world=%r)" % (ids,))

    def _composed_acm(self, live_acm):
        from moveit_msgs.msg import AllowedCollisionEntry, \
            AllowedCollisionMatrix

        names = [str(name) for name in live_acm.entry_names]
        keep = [name for name in names if not _is_planning_artifact(name)]
        allowed = self._allowed_sets(live_acm)
        # internal symmetric matrix over `keep`, then the declared
        # structure pairs, then one clean emission
        matrix = {name: set() for name in keep}
        for name in keep:
            for other in keep:
                if other in allowed.get(name, ()) or \
                        name in allowed.get(other, ()):
                    matrix[name].add(other)
                    matrix[other].add(name)
        for first, second in DECLARED_STRUCTURE_PAIRS:
            for name in (first, second):
                if name not in matrix:
                    matrix[name] = set()
            matrix[first].add(second)
            matrix[second].add(first)
        acm = AllowedCollisionMatrix()
        acm.entry_names = list(matrix)
        for name in acm.entry_names:
            entry = AllowedCollisionEntry()
            entry.enabled = [bool(other in matrix[name])
                             for other in acm.entry_names]
            acm.entry_values.append(entry)
        return acm

    @staticmethod
    def _allowed_sets(live_acm):
        allowed: Dict[str, set] = {}
        names = [str(name) for name in live_acm.entry_names]
        for index, name in enumerate(names):
            if index < len(live_acm.entry_values):
                for other, flag in zip(
                        names, live_acm.entry_values[index].enabled):
                    if flag:
                        allowed.setdefault(name, set()).add(other)
        for name, value in zip(
                [str(n) for n in live_acm.default_entry_names],
                [bool(v) for v in live_acm.default_entry_values]):
            if value:
                allowed.setdefault(name, set())
        return allowed


PLANNING_ARTIFACT_IDS = ("route-released-box", "cert-payload",
                         "b-payload", "pickup_box")


def _is_planning_artifact(object_id):
    text = str(object_id)
    return (text in PLANNING_ARTIFACT_IDS
            or text.startswith("released-")
            or text.startswith("snapshot-cell-"))


def _append_world_box(scene, box):
    from moveit_msgs.msg import CollisionObject
    from luggage_planning.motion.planning_scene_client import build_collision_object

    scene.world.collision_objects.append(build_collision_object(
        str(box.object_id), list(box.center),
        list(getattr(box, "quaternion_xyzw", (0.0, 0.0, 0.0, 1.0))),
        list(box.size_whd), "world", CollisionObject.ADD))


def _attach_payload_scene(payload):
    """The carrying attach diff through the ONE shared construction."""
    from luggage_planning.motion.planning_scene_client import build_attach_scene

    return build_attach_scene(
        PAYLOAD_OBJECT_ID,
        list(payload.t_tcp_box_translation),
        list(payload.t_tcp_box_quaternion_xyzw),
        list(payload.dimensions_wdh),
        "suction_contact_frame",
        touch_links=SUCTION_INTERFACE_TOUCH_LINKS)


def _copy_collision_object(obj):
    """Deep copy by CDR round-trip (meshes preserved verbatim)."""
    from moveit_msgs.msg import CollisionObject
    from rclpy.serialization import deserialize_message, serialize_message

    return deserialize_message(serialize_message(obj), CollisionObject)


def _scene_inventory(scenes):
    """Object, triangle, ACM and octomap identity of each frozen scene."""
    import hashlib

    inventory = []
    for index, scene in enumerate(scenes or []):
        objects = []
        attached = [
            item.object for item in
            scene.robot_state.attached_collision_objects]
        for obj in list(scene.world.collision_objects) + attached:
            triangles = sum(len(mesh.triangles) for mesh in obj.meshes)
            objects.append({
                "id": str(obj.id),
                "triangles": int(triangles),
                "primitives": len(obj.primitives),
            })
        acm = scene.allowed_collision_matrix
        rows = [
            [bool(flag) for flag in entry.enabled]
            for entry in acm.entry_values]
        payload = json.dumps({
            "names": [str(name) for name in acm.entry_names],
            "values": rows,
        }, sort_keys=True)
        data = b""
        octomap = getattr(scene.world, "octomap", None)
        if octomap is not None:
            data = getattr(getattr(octomap, "octomap", None), "data", b"") \
                or b""
        inventory.append({
            "index": index,
            "objects": objects,
            "acm_digest": hashlib.sha256(
                payload.encode("utf-8")).hexdigest(),
            "acm_names": len(acm.entry_names),
            "octomap_bytes": len(data),
        })
    return inventory


# ------------------------------------------------------------ the adapter


class MoveItStateValidityAdapter(MoveItValidityAdapter):
    """Answer the certifier's per-sample questions via ValidateRouteV2.

    The producer loads the route's edges (with their B ``SceneState``
    per edge) before certification. The first ``validate_sample`` call
    composes the frozen scene, sends the WHOLE route's original sampled
    states in one request, and every later sample is answered from that
    single verdict set. The certified trajectory is contact-free: any
    reported contact fails the sample with the raw evidence attached.
    """

    METHOD_NOTE = (
        "moveit_state_validity: whole-route batched ValidateRouteV2 "
        "against a frozen private scene copy (live world + live ACM + "
        "declared structure pairs, payload attached through the shared "
        "link-relative construction); states sampled at the frozen "
        "10 ms bound under the controller's spline interpolation, knots "
        "included; per-sample MoveIt-native collision checks, distance "
        "measured once per leg at its first sample (a GLOBAL mesh "
        "distance query costs ~150 ms host-measured; unmeasured entries "
        "inherit the nearest measured value), no IK and no re-planning; the "
        "certified trajectory is contact-free (release gap before the "
        "landing), any contact beyond the declared allowances fails; "
        "missing/late/echo-mismatched reports fail closed")

    def __init__(self, node, *, fk, scene_client=None,
                 service_timeout_s=SERVICE_TIMEOUT_S,
                 budget_s=CERT_BUDGET_S,
                 velocity_limit_rad=1.57,
                 distance_stride=DISTANCE_STRIDE,
                 support_object_id=None,
                 call_log=None, log=None):
        self._node = node
        self._log = log or (lambda message: None)
        self._fk = fk
        self._scene_client = scene_client
        self._service_timeout_s = float(service_timeout_s)
        self._budget_s = float(budget_s)
        self._velocity_limit_rad = float(velocity_limit_rad)
        self._distance_stride = int(distance_stride)
        self._support_object_id = (
            str(support_object_id) if support_object_id else None)
        self._client = None
        self._edges: Dict[str, object] = {}
        self._scene_states: Dict[str, object] = {}
        self._samples: Dict[str, Tuple[Tuple[float, ...], ...]] = {}
        self._times: Dict[str, List[float]] = {}
        self._verdicts: Dict[str, Tuple[bool, ...]] = {}
        self._clearances: Dict[str, List[float]] = {}
        self._contacts: Dict[str, Dict[int, tuple]] = {}
        self._scene_digest_value: Optional[str] = None
        self._support_object_id_resolved: Optional[str] = None
        self.requests: list = call_log if call_log is not None else []

    # -- producer API ----------------------------------------------------

    def _state_key(self, edge_id):
        return FrozenSceneComposer._state_key(  # noqa: SLF001
            self._scene_states[edge_id])

    @property
    def support_object_id(self):
        """The support object the certification scene resolved."""
        return (self._support_object_id
                or self._support_object_id_resolved)

    @property
    def scene_digest(self):
        return self._scene_digest_value

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
            self._samples[edge_id] = interpolated_positions(
                edge.trajectory)
            self._times[edge_id] = sample_grid(edge.trajectory)
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
        self._ensure_verdicts()
        valid = self._verdicts[edge_id][int(sample_index)]
        contacts = self._contacts.get(edge_id, {}).get(int(sample_index),
                                                       ())
        return ValiditySample(
            edge_id=edge_id,
            sample_index=int(sample_index),
            valid=bool(valid),
            min_clearance_m=float(
                self._clearances[edge_id][int(sample_index)]),
            contacts=contacts)

    # -- internals ---------------------------------------------------------

    def _capture_dir(self):
        path = os.environ.get("ST09_I_CERT_CAPTURE_DIR") or ""
        if not path:
            return ""
        os.makedirs(path, exist_ok=True)
        return path

    def _capture_request(self, request):
        """Persist the exact ValidateRouteV2 request before the call."""
        import base64

        from rclpy.serialization import serialize_message

        dump_path = os.environ.get("ST09_I_DUMP_CERT")
        if dump_path:
            with open(dump_path, "w", encoding="utf-8") as handle:
                json.dump({
                    "scenes": [base64.b64encode(
                        serialize_message(scene)).decode()
                        for scene in request.load_state_scenes],
                    "legs": [base64.b64encode(
                        serialize_message(leg)).decode()
                        for leg in request.legs],
                }, handle)
        directory = self._capture_dir()
        if not directory:
            self._pending_capture = None
            return
        # Each certify() builds a new adapter, so the counter has to live
        # in the directory. Otherwise every route overwrites request 0.
        index = 0
        for name in os.listdir(directory):
            if not (name.startswith("cert_request_") and name.endswith(".json")):
                continue
            stem = name[len("cert_request_"):-len(".json")]
            if stem.isdigit():
                index = max(index, int(stem) + 1)
        document = {
            "index": index,
            "request_id": str(request.request_id),
            "budget_s": float(request.budget_s),
            "distance_stride": int(request.distance_stride),
            "scene_digest": str(request.scene_digest),
            "scenes": _scene_inventory(request.load_state_scenes),
            "legs": [{
                "edge_id": str(leg.edge_id),
                "edge_class": str(leg.edge_class),
                "carrying": bool(leg.carrying),
                "load_state_index": int(leg.load_state_index),
                "n_samples": int(leg.n_samples),
                "waypoints": len(leg.knots.points),
            } for leg in request.legs],
            "request_b64": base64.b64encode(
                serialize_message(request)).decode(),
        }
        for name in ("cert_request_%d.json" % index, "cert_request.json"):
            with open(os.path.join(directory, name), "w",
                      encoding="utf-8") as handle:
                json.dump(document, handle)
                handle.write("\n")
        self._pending_capture = index

    def _capture_response(self, response):
        directory = self._capture_dir()
        index = getattr(self, "_pending_capture", None)
        if not directory or index is None:
            return
        document = {
            "index": index,
            "result": int(response.result),
            "detail": str(response.detail),
            "all_valid": bool(response.all_valid),
            "model_sha256": str(response.model_sha256),
            "scene_digest": str(response.scene_digest),
            "service_time_ns": int(response.service_time_ns),
            "legs": [{
                "edge": str(leg.edge_id),
                "valid": bool(leg.valid),
                "reason": str(leg.reason),
                "n_samples": int(leg.n_samples),
                "first_invalid": int(leg.first_invalid_sample),
                "leg_time_ns": int(leg.leg_time_ns),
                "collision_time_ns": int(getattr(
                    leg, "collision_time_ns", 0) or 0),
                "distance_time_ns": int(getattr(
                    leg, "distance_time_ns", 0) or 0),
                "distance_queries": int(getattr(
                    leg, "distance_queries", 0) or 0),
            } for leg in response.leg_results],
        }
        for name in ("cert_response_%d.json" % index, "cert_response.json"):
            with open(os.path.join(directory, name), "w",
                      encoding="utf-8") as handle:
                json.dump(document, handle, indent=1)
                handle.write("\n")

    def _ensure_client(self):
        if self._client is None:
            from luggage_msgs.srv import ValidateRouteV2

            self._client = self._node.create_client(
                ValidateRouteV2, SERVICE_NAME)

    def _ensure_verdicts(self):
        if self._verdicts:
            return
        if not self._edges:
            raise ContractViolation(
                "CERT_UNKNOWN_INPUT: no edges loaded")
        self._ensure_client()
        if not self._client.wait_for_service(timeout_sec=2.0):
            raise ContractViolation(
                "CERT_SERVICE_UNAVAILABLE: %s not serving" % SERVICE_NAME)

        # certify in ROUTE order (insertion order)
        ordered = list(self._edges.values())
        start_q = self._samples[str(ordered[0].edge_id)][0]
        composer = FrozenSceneComposer(self._scene_client, log=self._log)
        live = composer.read_live()
        scenes, state_indices, resolved_support = composer.compose(
            live, start_q, self._scene_states, ordered)
        self._support_object_id_resolved = resolved_support
        self._state_indices = state_indices
        digest = _scene_digest(scenes)
        self._scene_digest_value = digest

        from luggage_msgs.srv import ValidateRouteV2

        request = ValidateRouteV2.Request()
        request.request_id = "cert-route"
        request.route_digest = digest
        request.scene_digest = digest
        request.load_state_scenes = scenes
        request.max_sample_dt_ns = float(MAX_CERT_SAMPLE_DT_NS)
        request.max_sample_step_m = 0.0
        request.distance_stride = self._distance_stride
        request.joint_velocity_limits = [
            self._velocity_limit_rad] * len(CANONICAL_JOINTS)
        request.joint_acceleration_limits = []
        request.budget_s = self._budget_s
        for edge in ordered:
            request.legs.append(self._leg_message(edge))

        event = threading.Event()
        self._capture_request(request)
        future = self._client.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=self._service_timeout_s):
            raise ContractViolation(
                "CERT_SERVICE_TIMEOUT: %s no answer within %.1f s"
                % (SERVICE_NAME, self._service_timeout_s))
        response = future.result()
        self._capture_response(response)
        self._absorb(response, request, ordered)

    def _leg_message(self, edge):
        from luggage_msgs.msg import RouteLegV2
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        edge_id = str(edge.edge_id)
        trajectory = edge.trajectory
        leg = RouteLegV2()
        leg.edge_id = edge_id
        leg.edge_class = str(getattr(edge, "attributes", {}).get(
            "edge_class", ""))
        leg.carrying = getattr(self._scene_states[edge_id],
                               "payload", None) is not None
        leg.load_state_index = int(self._state_indices[
            self._state_key(edge_id)])
        leg.joint_names = [str(n) for n in trajectory.joint_names]
        # strictly increasing dedupe of C's grid (duplicate knot entries
        # map back through self._index_map)
        times = self._times[edge_id]
        keep = [0]
        for index in range(1, len(times)):
            if times[index] > times[keep[-1]]:
                keep.append(index)
        self._index_map = getattr(self, "_index_map", {})
        self._index_map[edge_id] = keep
        leg.n_samples = len(keep)
        leg.sample_times_ns = [float(times[i]) for i in keep]
        leg.sample_positions_flat = [
            float(v) for i in keep
            for v in self._samples[edge_id][i]]
        knots = JointTrajectory()
        knots.joint_names = [str(n) for n in trajectory.joint_names]
        for position, velocity, accel, stamp in zip(
                trajectory.positions, trajectory.velocities,
                trajectory.accelerations, trajectory.time_from_start_ns):
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in position]
            point.velocities = [float(v) for v in velocity]
            point.accelerations = [float(v) for v in accel]
            point.time_from_start.sec = int(stamp) // 10**9
            point.time_from_start.nanosec = int(stamp) % 10**9
            knots.points.append(point)
        leg.knots = knots
        return leg

    def _absorb(self, response, request, ordered):
        results = {str(leg.edge_id): leg for leg in
                   response.leg_results}
        if response.route_digest != request.route_digest or \
                response.scene_digest != request.scene_digest:
            raise ContractViolation(
                "CERT_ECHO_MISMATCH: service answered a different "
                "route/scene digest")
        requested = {str(leg.edge_id): leg for leg in request.legs}
        self.requests.append({
            "kind": "validate_route", "result": int(response.result),
            "detail": str(response.detail),
            "all_valid": bool(response.all_valid),
            "model_sha256": str(response.model_sha256),
            "scene_digest": self._scene_digest_value,
            "support_object_id": self.support_object_id,
            "scenes": _scene_inventory(request.load_state_scenes),
            "legs": [{
                "edge": str(leg.edge_id), "valid": bool(leg.valid),
                "reason": str(leg.reason),
                "detail": str(leg.detail),
                "n_samples": int(leg.n_samples),
                "waypoints": len(
                    requested[str(leg.edge_id)].knots.points)
                if str(leg.edge_id) in requested else 0,
                "first_invalid": int(leg.first_invalid_sample),
                "leg_time_ns": int(leg.leg_time_ns),
                "collision_time_ns": int(getattr(
                    leg, "collision_time_ns", 0) or 0),
                "distance_time_ns": int(getattr(
                    leg, "distance_time_ns", 0) or 0),
                "distance_queries": int(getattr(
                    leg, "distance_queries", 0) or 0),
                "contacts": [{
                    "body_1": str(c.body_1), "body_2": str(c.body_2),
                    "depth": float(c.depth)}
                    for c in leg.contacts],
                "support_contacts": [{
                    "body_1": str(c.body_1), "body_2": str(c.body_2),
                    "depth": float(c.depth)}
                    for c in leg.support_contacts],
            } for leg in response.leg_results],
            "service_time_ns": int(response.service_time_ns),
        })
        if response.result != 0:
            raise ContractViolation(
                "CERT_SERVICE_REJECTED: result=%d detail=%s"
                % (int(response.result), response.detail))
        self._log("validity: route %s support=%s %s" % (
            "valid" if response.all_valid else "INVALID",
            self.support_object_id,
            "; ".join(
                "%s(%s%s)%s" % (
                    str(leg.edge_id)[-24:],
                    leg.reason or "ok",
                    " #%d" % int(leg.first_invalid_sample)
                    if not leg.valid else "",
                    (" {%s}" % " ".join(
                        "%s/%s d=%.3f" % (
                            c.body_1[:14], c.body_2[:14], c.depth)
                        for c in leg.contacts[:3]))
                    if leg.contacts else "")
                for leg in response.leg_results)))
        for edge in ordered:
            edge_id = str(edge.edge_id)
            leg_result = results.get(edge_id)
            if leg_result is None:
                raise ContractViolation(
                    "CERT_UNKNOWN_INPUT: service skipped edge %r"
                    % edge_id)
            samples = self._samples[edge_id]
            times = self._times[edge_id]
            keep = self._index_map[edge_id]
            forbidden_by_sample: Dict[int, list] = {}
            for contact in leg_result.contacts:
                forbidden_by_sample.setdefault(
                    int(contact.sample_index), []).append(contact)
            first_invalid = int(leg_result.first_invalid_sample)

            def raw_records(contacts):
                return tuple({
                    "type": "contact",
                    "body_1": str(c.body_1), "body_2": str(c.body_2),
                    "depth": float(c.depth),
                    "position": [float(c.position.x),
                                 float(c.position.y),
                                 float(c.position.z)],
                } for c in contacts)

            # per deduped sample (the service's sample indexing); the
            # certified trajectory is contact-free — any contact fails
            dedup_verdicts = []
            dedup_contacts: Dict[int, tuple] = {}
            for position in range(len(keep)):
                forbidden = forbidden_by_sample.get(position, [])
                if forbidden:
                    dedup_verdicts.append(False)
                    dedup_contacts[position] = raw_records(forbidden)
                    continue
                if not leg_result.valid and position >= first_invalid:
                    dedup_verdicts.append(False)
                    continue
                dedup_verdicts.append(True)

            # clearance: NaN entries inherit the nearest measured value
            measured = list(leg_result.min_distance_m)
            clearances: List[float] = []
            last_measured = float("inf")
            for position in range(len(keep)):
                value = measured[position] if position < len(measured) \
                    else float("nan")
                if value == value:
                    last_measured = value
                clearances.append(value if value == value
                                  else last_measured)

            # map back onto C's full index grid (duplicate knot entries
            # repeat the kept sample's verdict and clearance)
            verdicts = []
            full_clearances = []
            contacts: Dict[int, tuple] = {}
            position = 0
            for index in range(len(samples)):
                if position + 1 < len(keep) and times[index] >= \
                        times[keep[position + 1]] - 1e-9:
                    position += 1
                verdicts.append(dedup_verdicts[position])
                full_clearances.append(clearances[position])
                if position in dedup_contacts:
                    contacts[index] = dedup_contacts[position]
            self._verdicts[edge_id] = tuple(verdicts)
            self._contacts[edge_id] = contacts
            self._clearances[edge_id] = full_clearances
