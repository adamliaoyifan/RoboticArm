"""Measured-envelope local endpoint transport for whole-operation preparation.

The existing native local checker owns its temporary payload identity. Remove
only the caller-associated pickup object, then submit the frozen envelope in
that checker's coordinates. Hints retain provenance but confer no validity.
Output stays untimed and must pass the separate whole-operation certifier.
"""
import hashlib
import math
import threading
import time

from luggage_planning.joint_operation.contracts import OPERATION_JOINTS
from luggage_planning.joint_operation.planner import (
    LocalEndpointEvidence, LocalJointBranch,
)
from luggage_planning.joint_operation.scenes import _reset_payload_contacts
from luggage_planning.placement_prior.contracts import Transform


LOCAL_PAYLOAD_ID = 'prior-local-box'  # owned by grasp_local_check_node, not a live scene ID


def _pose(transform):
    from geometry_msgs.msg import Pose
    result = Pose()
    result.position.x, result.position.y, result.position.z = transform.translation
    (result.orientation.x, result.orientation.y, result.orientation.z,
     result.orientation.w) = transform.quaternion_xyzw
    return result


def build_local_query(candidate, *, service, request_id, budget_s):
    """Translate nominal box/G to the already-conservative envelope once."""
    from moveit_msgs.msg import PlanningScene
    from rclpy.serialization import deserialize_message
    if not request_id or not math.isfinite(budget_s) or not 0 < budget_s <= 10:
        raise ValueError('request identity and bounded local budget required')
    scene = deserialize_message(candidate.scenes.pick.scene_cdr, PlanningScene)
    if (scene.is_diff or scene.robot_state.is_diff
            or scene.robot_state.attached_collision_objects):
        raise ValueError('full unloaded pickup phase required')
    if any(obj.id == LOCAL_PAYLOAD_ID for obj in scene.world.collision_objects):
        raise ValueError('native local payload identity conflicts with a world obstacle')
    owned_id = candidate.scenes.pickup_object_id
    scene.world.collision_objects = [obj for obj in scene.world.collision_objects if obj.id != owned_id]
    _reset_payload_contacts(scene, LOCAL_PAYLOAD_ID)
    query = service.Request()
    query.request_id = request_id+':'+candidate.candidate_id
    targets = tuple(getattr(candidate, name) for name in (
        'world_from_pick_box', 'world_from_pick_tool', 'world_from_release_box',
        'world_from_release_tool', 'pick_approach', 'pick_lift', 'place_entry', 'place_retreat'))
    material = (candidate.candidate_id, candidate.scenes.base_identity, candidate.scenes.digests,
                candidate.predicted_attachment.digest(), targets,
                candidate.pick_entry_seeds, candidate.place_entry_seeds)
    query.snapshot_id = hashlib.sha256(repr(material).encode()).hexdigest()
    query.model_sha256 = candidate.scenes.pick.model_hash
    query.scene = scene
    geometry = candidate.predicted_attachment
    envelope = geometry.conservative
    for name, nominal in (('pick_box_world', candidate.world_from_pick_box),
                          ('place_box_world', candidate.world_from_release_box)):
        xyz, quat = geometry.conservative_pose_at_box(nominal.translation, nominal.quaternion_xyzw)
        setattr(query, name, _pose(Transform(xyz, quat)))
    # The native checker computes tool = envelope_box * G, and attaches G^-1.
    # Nominal G would put the envelope centre at the wrong tool-relative offset.
    envelope_from_tool = Transform(envelope.t_tcp_box_translation,
                                    envelope.t_tcp_box_quaternion_xyzw).inverse()
    query.box_from_tcp = _pose(envelope_from_tool)
    for name, target in (('pick_approach_world', candidate.pick_approach),
                         ('pick_lift_world', candidate.pick_lift),
                         ('place_entry_world', candidate.place_entry),
                         ('place_retreat_world', candidate.place_retreat)):
        setattr(query, name, _pose(target))
    query.box_wdh = list(envelope.dimensions_wdh)
    query.pick_entry_seed_joints_flat = [q for seed in candidate.pick_entry_seeds for q in seed.joints]
    query.place_entry_seed_joints_flat = [q for seed in candidate.place_entry_seeds for q in seed.joints]
    query.budget_s = float(budget_s)
    return query


def _path_rows(path):
    if (tuple(path.joint_names) != OPERATION_JOINTS or path.header.frame_id
            or path.header.stamp.sec or path.header.stamp.nanosec
            or not 2 <= len(path.points) <= 50000):
        raise ValueError('invalid untimed local joint path')
    rows = []
    for point in path.points:
        row = tuple(float(q) for q in point.positions)
        if (len(row) != 6 or any(not math.isfinite(q) for q in row)
                or point.velocities or point.accelerations or point.effort
                or point.time_from_start.sec or point.time_from_start.nanosec):
            raise ValueError('geometric local path must contain finite positions only')
        if rows and max(abs(a-b) for a,b in zip(rows[-1],row)) > .01+1e-10:
            raise ValueError('undersampled local path or changed joint winding')
        rows.append(row)
    return tuple(rows)


def _branches(branches, hints, endpoint):
    if not 1 <= len(branches) <= 2:
        raise ValueError('one or two complete endpoint branches required')
    result, indices = [], set()
    for branch in branches:
        index = int(branch.seed_index)
        if not 0 <= index < 8 or index in indices:
            raise ValueError('invalid native local seed index')
        indices.add(index)
        provenance = ('hint:'+hints[index].seed_id if index < len(hints)
                      else 'generic-%d' % (index-len(hints)))
        result.append(LocalJointBranch(endpoint+':'+provenance,
            _path_rows(branch.incoming), _path_rows(branch.outgoing)))
    return tuple(result)


def endpoint_evidence(response, request, candidate):
    def result(complete, reason, pick=(), place=()):
        return LocalEndpointEvidence(candidate.candidate_id, request.model_sha256,
            candidate.scenes.digests, candidate.predicted_attachment.digest(), complete, reason, pick, place)
    if ((response.request_id, response.snapshot_id, response.model_sha256) !=
            (request.request_id, request.snapshot_id, request.model_sha256)):
        return result(False, 'LOCAL_PATH_IDENTITY_MISMATCH')
    if response.complete is not True or response.reason != 'LOCAL_PATHS_VALID':
        return result(False, str(response.reason))
    try:
        pick = _branches(response.pick_branches, candidate.pick_entry_seeds, 'pick')
        place = _branches(response.place_branches, candidate.place_entry_seeds, 'place')
    except (TypeError, ValueError) as error:
        return result(False, 'INVALID_LOCAL_PATH_WITNESS:'+str(error))
    return result(True, 'LOCAL_PATHS_VALID', pick, place)


class NativeOperationEndpointPort:
    """One in-flight query; recorder(kind, bytes) preserves exact captured CDR."""
    def __init__(self, node, *, max_query_s=2., recorder=None):
        from luggage_msgs.srv import CheckGraspLocalPaths
        if not math.isfinite(max_query_s) or not 0 < max_query_s <= 10:
            raise ValueError('max_query_s must be in (0,10]')
        self.service = CheckGraspLocalPaths
        self.client = node.create_client(self.service, '/placement_prior/check_local_paths')
        self.max_query_s, self.recorder, self.inflight = max_query_s, recorder, None

    def solve(self, candidate, *, request_id, deadline, cancelled):
        from rclpy.serialization import serialize_message
        def refused(reason):
            return LocalEndpointEvidence(candidate.candidate_id, candidate.scenes.pick.model_hash,
                candidate.scenes.digests, candidate.predicted_attachment.digest(), False, reason)
        def stopped():
            return cancelled() or time.monotonic() >= deadline
        if stopped():
            return refused('CANCELLED_OR_DEADLINE')
        if self.inflight is not None and not self.inflight.done():
            return refused('LOCAL_PATH_BACKEND_BUSY')
        if not self.client.service_is_ready():
            return refused('LOCAL_PATH_BACKEND_UNAVAILABLE')
        budget = min(self.max_query_s, deadline-time.monotonic())
        if budget <= 0:
            return refused('CANCELLED_OR_DEADLINE')
        query = build_local_query(candidate, service=self.service, request_id=request_id, budget_s=budget)
        if stopped():
            return refused('CANCELLED_OR_DEADLINE')
        query.budget_s = min(self.max_query_s, deadline-time.monotonic())
        if self.recorder:
            self.recorder('request', serialize_message(query))
        event = threading.Event()
        self.inflight = self.client.call_async(query)
        self.inflight.add_done_callback(lambda _: event.set())
        while not event.wait(min(.01, max(0., deadline-time.monotonic()))):
            if stopped():
                return refused('CANCELLED_OR_DEADLINE')
        try:
            response = self.inflight.result()
        except Exception as error:
            return refused('LOCAL_PATH_TRANSPORT_ERROR:'+type(error).__name__)
        if self.recorder:
            self.recorder('response', serialize_message(response))
        if stopped():
            return refused('CANCELLED_OR_DEADLINE')
        return endpoint_evidence(response, query, candidate)
