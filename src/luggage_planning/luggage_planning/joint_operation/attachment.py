"""Qualify a real seal receipt against the original stationary box observation.

This read-only adapter does not enable vacuum, execute motion or change a map.
The runtime calls it after its actual VacuumCommandV2 response. The result fixes
G once; actual-target and remaining-path native validation are still required.
"""
from dataclasses import dataclass
import hashlib
import json
import math
import time

from luggage_planning.attachment_provider import MeasuredPose
from luggage_planning.hardware_place_planning import attachment_from_response
from luggage_planning.joint_operation.actual_contact import RetainedContactObservation
from luggage_planning.joint_operation.contracts import admit_operation_plan
from luggage_planning.placement_prior.contracts import Transform
from st09_contracts.stationary_attachment import (
    StationaryAttachmentEvidence, measured_pose_digest,
)


@dataclass(frozen=True)
class QualifiedOperationAttachment:
    record: object
    geometry: object
    generation: int
    evidence: StationaryAttachmentEvidence


class StationaryOperationAttachmentPort:
    def __init__(self, provider, contacts, *, certificate_current, now_ns, recorder=None):
        self.provider, self.contacts = provider, contacts
        self.certificate_current, self.now_ns = certificate_current, now_ns
        self.recorder = recorder

    def qualify(self, plan, observation, detection, request, response, *, deadline, cancelled):
        from rclpy.serialization import serialize_message

        def current():
            if not math.isfinite(deadline) or cancelled() or time.monotonic() >= deadline:
                raise ValueError('CANCELLED_OR_ATTACHMENT_DEADLINE')
            if self.certificate_current(plan) is not True:
                raise ValueError('ATTACHMENT_PLAN_NOT_CURRENT')

        admit_operation_plan(plan, plan.identity, plan.plan_digest)
        current()
        if type(observation) is not RetainedContactObservation:
            raise ValueError('ORIGINAL_FROZEN_OBSERVATION_REQUIRED')
        identity, domain = plan.identity, observation.domain
        if Transform(domain.box_xyz_world, domain.box_quaternion_xyzw) != plan.world_from_pick_box:
            raise ValueError('ATTACHMENT_ORIGINAL_BOX_POSE_MISMATCH')
        from luggage_perception.ros_suction_domain import require_matching_detection
        from luggage_planning.placement_prior.pick import transform_from_pose
        from luggage_description.frame_contracts import validate_envelope
        from luggage_description.ros_frame_bindings.identity import identity_from_message
        validate_envelope(identity_from_message(request.box_pose.identity), identity.scene_revision)
        validate_envelope(identity_from_message(response.measured_grasp.identity), identity.scene_revision)
        require_matching_detection(domain, request.measured_box, generation=request.generation,
                                   contact_model_hash=identity.contact_model_hash)
        pose_identity = request.box_pose.identity
        if (not request.enable or request.request_id != identity.request_id
                or request.scene_revision != identity.scene_revision
                or request.box_id != identity.box_id or request.generation != identity.generation
                or pose_identity.request_id != identity.request_id
                or pose_identity.scene_revision != identity.scene_revision
                or pose_identity.frame_id != 'world' or not pose_identity.stamp_valid
                or pose_identity.stamp_ns != domain.stamp_ns
                or pose_identity.time_basis != 'acquisition'
                or response.measured_grasp.identity.time_basis != 'scene'
                or request.box_pose.reference != 'box_center'
                or request.box_pose.orientation_semantics != 'box_axis_pi'
                or transform_from_pose(request.box_pose.pose) != plan.world_from_pick_box):
            raise ValueError('STATIONARY_VACUUM_REQUEST_BINDING_MISMATCH')
        request_bytes = serialize_message(request)
        receipt_bytes = serialize_message(response)
        # DDS CDR alignment padding is not deterministic, even for the same
        # untouched Python message. Retain raw transport bytes for replay, but
        # identify a receipt by all its typed fields, including its ROS type.
        from rosidl_runtime_py.convert import message_to_ordereddict
        def field_bytes(message):
            return json.dumps(dict(ros_type=type(message).__module__ + '.' + type(message).__name__,
                fields=message_to_ordereddict(message)), sort_keys=True,
                separators=(',', ':'), allow_nan=False).encode()
        request_fields, response_fields = field_bytes(request), field_bytes(response)
        if self.recorder:
            self.recorder('stationary_vacuum_request', request_bytes)
            self.recorder('stationary_vacuum_response', receipt_bytes)
            self.recorder('stationary_vacuum_request_fields.json', request_fields)
            self.recorder('stationary_vacuum_response_fields.json', response_fields)
        measured = attachment_from_response(response, request_id=identity.request_id,
            box_id=identity.box_id, generation=identity.generation,
            scene_revision=identity.scene_revision)
        if measured.box_wdh != domain.box_wdh:
            raise ValueError('ATTACHMENT_ORIGINAL_DIMENSIONS_MISMATCH')
        # A stored token alone is not current perception-owner approval.
        retained = self.contacts.retain(domain, detection, request_id=identity.request_id,
            observation_id=identity.observation_id, expected_observation=observation,
            deadline=deadline, cancelled=cancelled)
        if not retained.retained:
            raise ValueError('ATTACHMENT_OBSERVATION_NOT_CURRENT:' + retained.reason)
        actual = self.contacts.check(observation, identity=identity, geometry=measured.geometry,
            deadline=deadline, cancelled=cancelled)
        if not actual.checked or not actual.accepted or actual.result is None:
            raise ValueError('ATTACHMENT_ACTUAL_FOOTPRINT_REFUSED:' + actual.reason)
        if (actual.result.binding_sha256 != observation.binding.digest()
                or actual.result.attachment_geometry_sha256 != measured.geometry.digest()):
            raise ValueError('ATTACHMENT_ACTUAL_EVIDENCE_BINDING_MISMATCH')
        # The world TCP is recovered from the controller's actual seal-time G,
        # not from a planned pick, a later carry TF or a re-stamped box pose.
        world_tcp = plan.world_from_pick_box.compose(measured.box_from_tcp)
        box_pose = MeasuredPose('world', domain.box_xyz_world, domain.box_quaternion_xyzw,
                                domain.stamp_ns)
        tcp_pose = MeasuredPose('suction_contact_frame', world_tcp.translation,
                                world_tcp.quaternion_xyzw, measured.stamp_ns)
        evidence = StationaryAttachmentEvidence(identity.request_id, identity.box_id,
            identity.generation, identity.scene_revision, domain.stamp_ns, measured.stamp_ns,
            measured_pose_digest(box_pose), measured_pose_digest(tcp_pose),
            observation.binding.digest(), actual.result.digest(),
            hashlib.sha256(request_fields).hexdigest(),
            hashlib.sha256(response_fields).hexdigest(), measured.geometry.digest())
        current()
        # Capture before freezing so evidence I/O failure cannot hide a record.
        if self.recorder:
            from dataclasses import asdict
            self.recorder('stationary_attachment_evidence',
                json.dumps(asdict(evidence), sort_keys=True).encode())
        current()
        record = self.provider.record_stationary_attachment(identity.request_id,
            box_pose, tcp_pose, evidence=evidence, conservative_geometry=measured.geometry,
            now_ns=self.now_ns(), cycle_id=identity.request_id)
        return QualifiedOperationAttachment(record, measured.geometry, identity.generation, evidence)
