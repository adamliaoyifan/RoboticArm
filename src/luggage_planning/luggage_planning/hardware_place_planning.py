"""Hardware place planning from controller-confirmed attachment and live state.

Only reads subscriptions/TF and calls planning services. It never executes a
route, releases vacuum, or writes occupancy or the shared planning scene.
"""
from copy import deepcopy
from dataclasses import dataclass
import json
import threading

from luggage_planning.placement_prior.contracts import Transform, finite_tuple
from luggage_planning.placement_prior.pick import transform_from_pose
from luggage_planning.typed_driver_adapter import grasp_record, tcp_record


@dataclass(frozen=True)
class MeasuredAttachment:
    box_id: str
    generation: int
    scene_revision: str
    stamp_ns: int
    box_wdh: tuple
    box_from_tcp: Transform
    geometry: object = None


def attachment_from_response(response, *, request_id, box_id, generation, scene_revision):
    if not response.success or not response.has_measured_attachment:
        raise ValueError('controller did not return a measured attachment')
    pose = response.measured_grasp
    identity = pose.identity
    if (identity.request_id != request_id or identity.scene_revision != scene_revision
            or identity.frame_id != 'suction_contact_frame' or not identity.stamp_valid
            or identity.stamp_ns <= 0 or pose.reference != 'suction_contact_frame'
            or pose.orientation_semantics != 'box' or not box_id or generation <= 0):
        raise ValueError('measured attachment identity mismatch')
    size = finite_tuple(response.attached_box_wdh, 3, 'attached dimensions')
    if min(size) <= 0:
        raise ValueError('attached dimensions must be positive')
    from luggage_planning.attachment_geometry_transport import geometry_from_message
    from st09_contracts.attachment import invert_transform
    geometry = geometry_from_message(response.conservative_geometry)
    box_from_tcp = transform_from_pose(pose.pose)
    tool_box = invert_transform(box_from_tcp.translation, box_from_tcp.quaternion_xyzw)
    if (tuple(size) != geometry.measured_dimensions_wdh
            or any(abs(a-b) > 1e-9 for a,b in zip(tool_box[0], geometry.t_tool_box_translation))
            or abs(sum(a*b for a,b in zip(tool_box[1], geometry.t_tool_box_quaternion_xyzw))) < 1-1e-9):
        raise ValueError('ATTACHMENT_GEOMETRY_BINDING_MISMATCH')
    return MeasuredAttachment(box_id, generation, scene_revision,
                              identity.stamp_ns, size, box_from_tcp, geometry)


def plan_attached_box(adapter, attachment, *, request_id, surface, inner_size,
                      start_tcp, start_joints, container_transform,
                      prior=None, selected_choice=None, timeout_sec=60.):
    """Keep the chosen box target, but use actual attachment for full Select."""
    if surface.identity.scene_revision != attachment.scene_revision:
        raise ValueError('placement map and attachment scene mismatch')
    if len(start_joints) != 6:
        raise ValueError('complete measured six-joint start required')
    if attachment.geometry is None:
        raise ValueError('ATTACHMENT_GEOMETRY_MISSING')
    finite_tuple(start_joints, 6, 'start joints')
    from luggage_planning.placement_prior.adoption import bound_target_slots
    bound = bound_target_slots(prior, selected_choice, surface, attachment.box_wdh,
                               attachment.scene_revision, request_id)
    if selected_choice is not None:
        slots, scores = bound
        # A pre-contact target remains binding, but must also be feasible for
        # the actual conservative attachment. Never silently move that target.
        placement = adapter.compute_placement(request_id=request_id,
            box_wdh=attachment.box_wdh, height_measured=True,
            conservative_geometry=attachment.geometry,
            scene_revision=attachment.scene_revision, geometry_hash=surface.geometry_hash,
            expected_map_revision=surface.map_revision, max_candidates=2000)
        from luggage_planning.placement_prior.adoption import _pose_equal
        candidates = [] if placement is None or not placement.success else (
            list(placement.candidates) or [placement.slot])
        if not any(_pose_equal(slots[0].pose, candidate.pose) for candidate in candidates):
            raise ValueError('BOUND_TARGET_NOT_IN_CONSERVATIVE_CANDIDATES')
    else:
        placement = adapter.compute_placement(request_id=request_id,
            box_wdh=attachment.box_wdh, height_measured=True,
            conservative_geometry=attachment.geometry,
            scene_revision=attachment.scene_revision, geometry_hash=surface.geometry_hash,
            expected_map_revision=surface.map_revision, max_candidates=2000)
        if placement is None or not placement.success:
            raise ValueError('no placement candidates from authoritative planner')
        slots = list(placement.candidates) or [placement.slot]
        scores = list(placement.candidate_scores) or [0.]
        # The placement service stamps its own request identity.
        request_id = slots[0].identity.request_id or request_id
    grasp = grasp_record(request_id, attachment.scene_revision,
        attachment.box_from_tcp.translation, attachment.box_from_tcp.quaternion_xyzw,
        stamp_ns=attachment.stamp_ns)
    start = dict(start_tcp, request_id=request_id)
    return adapter.select_place_path(request_id=request_id, box_wdh=attachment.box_wdh,
        conservative_geometry=attachment.geometry,
        slots=slots, slot_scores=scores, surface_message=surface, inner_size=inner_size,
        geometry_hash=surface.geometry_hash, map_revision=surface.map_revision,
        start_tcp=start, start_joints=start_joints, grasp=grasp,
        container_transform=container_transform, box_id=attachment.box_id,
        generation=attachment.generation, timeout_sec=timeout_sec, placement_prior=prior)


class HardwarePlacePlanner:
    """Live place planning with the common validity admission gate.

    [G1/I3] Planning needs the SAME entry+commit double check placement
    has: ``plan`` captures an admission token BEFORE the compute and the
    commit re-check presents exactly that token — a reset landing
    mid-computation (or an old cached surface surviving one) is rejected
    here even when the surface object happens to be unchanged. The gate
    is fed by this class's own validity/surface/joint subscriptions; the
    robot freshness bound is the existing ``max_state_age_s`` contract.
    """

    def __init__(self, node, adapter, *, group, max_state_age_s=1.,
                 validity_topic='/uld_occ_observer/validity',
                 validity_backend='observer_o'):
        import tf2_ros
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from luggage_msgs.msg import SurfaceContractV2
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from luggage_planning.placement_admission import AdmissionGate
        self._node, self._adapter = node, adapter
        self._max_age_ns = int(max_state_age_s * 1e9)
        self._lock = threading.Lock()
        self._joints = self._surface = self._geometry = None
        self._admission = AdmissionGate()
        self._validity_backend = str(validity_backend)
        self._rejected_maps = 0
        self._tf = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf, node)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._subscriptions = [
            node.create_subscription(JointState, '/joint_states', self._on_joints, 10, callback_group=group),
            node.create_subscription(SurfaceContractV2, '/luggage/cargo_map/surface_v2',
                                     self._on_surface, qos, callback_group=group),
            node.create_subscription(String, '/luggage/cargo_map/surface_2d',
                                     self._on_geometry, qos, callback_group=group),
            node.create_subscription(String, validity_topic,
                                     self._on_validity, qos, callback_group=group),
        ]

    def _on_validity(self, message):
        """Bind the gate to the producer's latched validity stream.

        Arrival re-stamped in this node's clock (one domain for the robot
        comparison); malformed payloads or epochs illegal for this
        deployment's backend are counted and dropped, never fatal.
        """
        from luggage_planning.placement_admission import (
            EpochFormatError, ValidityObservation)
        try:
            payload = json.loads(message.data)
            observation = ValidityObservation.from_json(
                payload, backend=self._validity_backend)
        except (ValueError, EpochFormatError):
            self._rejected_validity = getattr(self, '_rejected_validity', 0) + 1
            return
        arrival = ValidityObservation(
            state=observation.state, epoch=observation.epoch,
            generation=observation.generation, reason=observation.reason,
            reset_id=observation.reset_id,
            transition_mono_ns=self._node.get_clock().now().nanoseconds,
            map_revision=observation.map_revision,
            observed_revision=observation.observed_revision,
            geometry_hash=observation.geometry_hash,
            config_digest=observation.config_digest,
            clock_domain=observation.clock_domain,
            backend=observation.backend,
            producer_transition_mono_ns=(
                observation.producer_transition_mono_ns))
        self._admission.observe_validity(arrival)

    def _on_joints(self, message):
        from luggage_planning.placement_admission import RobotStateSample
        stamp = (int(message.header.stamp.sec) * 10**9
                 + int(message.header.stamp.nanosec))
        if stamp > 0:
            # [I2] Zero-stamp samples are never re-stamped to now.
            self._admission.observe_robot_state(RobotStateSample(
                stamp, 'joint_states', {'name': list(message.name)}))
        with self._lock:
            self._joints = message

    def _on_surface(self, message):
        from luggage_planning.placement_admission import MapObservation
        decision = self._admission.observe_map(MapObservation(
            epoch=str(getattr(message, 'epoch', '') or ''),
            generation=int(getattr(message, 'validity_generation', 0) or 0),
            map_revision=int(getattr(message, 'map_revision', 0) or 0),
            geometry_hash=str(getattr(message, 'geometry_hash', '') or ''),
            backend=self._validity_backend))
        if not decision.admitted:
            self._rejected_maps += 1
            return
        with self._lock:
            self._surface = message

    def _on_geometry(self, message):
        try:
            record = json.loads(message.data)
        except (TypeError, ValueError):
            record = None
        with self._lock:
            self._geometry = record

    def plan(self, attachment, *, request_id, prior=None, selected_choice=None):
        from rclpy.time import Time
        from luggage_description.container_geometry import normalize_descriptor
        with self._lock:
            joints, surface, geometry = deepcopy((self._joints, self._surface, self._geometry))
        if joints is None or surface is None or geometry is None:
            raise ValueError('live joint/OCC/geometry inputs unavailable')
        stamp = int(joints.header.stamp.sec)*10**9 + int(joints.header.stamp.nanosec)
        age = self._node.get_clock().now().nanoseconds - stamp
        if stamp <= 0 or not 0 <= age <= self._max_age_ns:
            raise ValueError('joint state is stale or from a different clock')
        # [G1/I3] Entry admission: the token is captured BEFORE the
        # compute; a closed gate (no VALID map for this identity, no
        # post-transition robot sample) refuses before any planning.
        entry = self._admission.admit_planning(
            now_mono_ns=self._node.get_clock().now().nanoseconds,
            robot_max_age_ns=self._max_age_ns)
        if not entry.admitted:
            raise ValueError('validity gate closed: %s' % entry.reason)
        token = entry.admission_token
        if (geometry.get('geometry_hash') != surface.geometry_hash
                or geometry.get('map_revision') != surface.map_revision
                or normalize_descriptor(geometry['geometry_descriptor']).geometry_hash != surface.geometry_hash):
            raise ValueError('OCC geometry metadata mismatch')
        positions = dict(zip(joints.name, joints.position))
        start_joints = [positions['elfin_joint%d' % index] for index in range(1,7)]
        when = Time(nanoseconds=stamp)
        tcp = self._tf.lookup_transform('container_link', 'suction_contact_frame', when).transform
        container = self._tf.lookup_transform('world', 'container_link', when).transform
        def components(transform):
            p, q = transform.translation, transform.rotation
            return (p.x,p.y,p.z), (q.x,q.y,q.z,q.w)
        tcp_xyz, tcp_q = components(tcp)
        container_xyz, container_q = components(container)
        result = plan_attached_box(self._adapter, attachment, request_id=request_id,
            surface=surface, inner_size=geometry['inner_size'],
            start_tcp=tcp_record(request_id, attachment.scene_revision, tcp_xyz, tcp_q),
            start_joints=start_joints,
            container_transform=('container_link','world',container_xyz,container_q),
            prior=prior, selected_choice=selected_choice)
        with self._lock:
            if self._surface != surface:
                raise ValueError('OCC changed during place planning')
        # [G1/I3] Commit re-check: present the token captured at entry —
        # a reset that landed mid-computation voids it, so an old plan
        # cannot commit even when the gate has since recovered.
        commit = self._admission.commit_plan(
            admission_token=token,
            now_mono_ns=self._node.get_clock().now().nanoseconds,
            robot_max_age_ns=self._max_age_ns)
        if not commit.admitted:
            raise ValueError('validity commit rejected during place '
                             'planning: %s' % commit.reason)
        return result
