"""Hardware place planning from controller-confirmed attachment and live state.

Only reads subscriptions/TF and calls planning services. It never executes a
route, releases vacuum, or writes occupancy or the shared planning scene.
"""
from copy import deepcopy
from dataclasses import dataclass
import json
import threading

from luggage_planning.placement_prior_contracts import Transform, finite_tuple
from luggage_planning.placement_prior_pick import transform_from_pose
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
    finite_tuple(start_joints, 6, 'start joints')
    from luggage_planning.placement_prior_adoption import bound_target_slots
    bound = bound_target_slots(prior, selected_choice, surface, attachment.box_wdh,
                               attachment.scene_revision, request_id)
    if selected_choice is not None:
        slots, scores = bound
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
    def __init__(self, node, adapter, *, group, max_state_age_s=1.):
        import tf2_ros
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from luggage_msgs.msg import SurfaceContractV2
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self._node, self._adapter = node, adapter
        self._max_age_ns = int(max_state_age_s * 1e9)
        self._lock = threading.Lock()
        self._joints = self._surface = self._geometry = None
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
        ]

    def _on_joints(self, message):
        with self._lock:
            self._joints = message

    def _on_surface(self, message):
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
        return result
