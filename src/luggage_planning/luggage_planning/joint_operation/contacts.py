"""Consume perception-owned directed quality without reinterpreting its pose.

A quality receipt binds sampled support, not motion feasibility. The same
measured box-from-contact transform must reach both operation endpoints.
"""
from copy import deepcopy
from dataclasses import dataclass
import json
import math
import threading
import time

import numpy as np

from luggage_perception.directed_suction_quality import DirectedSuctionResult, QUALITY_METRICS
from luggage_perception.observed_contact_footprint import check_observed_contact_footprint
from luggage_perception.ros_suction_domain import require_matching_detection
from luggage_planning.placement_prior.contracts import GraspCandidate, Transform
from luggage_planning.waypoint_generator import DEFAULT_PICK_CLEARANCES


@dataclass(frozen=True)
class DirectedContact:
    query_id: str
    query_uv: tuple
    yaw_rad: float
    world_from_contact: Transform
    quality: DirectedSuctionResult


@dataclass(frozen=True)
class ContactBatchEvidence:
    complete: bool
    reason: str
    contacts: tuple = ()
    # Includes refused contacts for bounded-search diagnostics.
    verdicts: tuple = ()
    domain_sha256: str = ''
    quality_policy_sha256: str = ''
    samples_sha256: str = ''


def _sha256(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def contact_query(domain, detection, queries, model, *, budget_s, max_refinements=3,
                  expected_policy_sha256=''):
    """Snapshot the exact request; no cell-centre snapping or inferred support."""
    from luggage_msgs.srv import EvaluateDirectedSuctionV2
    require_matching_detection(domain, detection, generation=domain.generation,
                               contact_model_hash=model.identity_hash)
    if (model.model_version != domain.contact_model_version
            or model.contact_frame != 'suction_contact_frame'
            or not math.isfinite(budget_s) or not 0 < budget_s <= 10
            or type(max_refinements) is not int or not 1 <= max_refinements <= 8
            or not 1 <= len(queries) <= 8
            or (expected_policy_sha256 and not _sha256(expected_policy_sha256))):
        raise ValueError('matching physical model and bounded contact query required')
    ids = [query.query_id for query in queries]
    if (any(not value.strip() for value in ids) or len(set(ids)) != len(ids)
            or any(not all(math.isfinite(v) for v in (*q.center_uv_top_m, q.yaw_rad)) for q in queries)):
        raise ValueError('distinct finite directed queries required')
    request = EvaluateDirectedSuctionV2.Request()
    request.domain_sha256 = domain.digest()
    request.detection, request.generation = deepcopy(detection), domain.generation
    request.queries = deepcopy(list(queries))
    request.max_refinements, request.budget_s = max_refinements, float(budget_s)
    request.expected_quality_policy_sha256 = expected_policy_sha256
    return request


def contact_evidence(response, request, domain, model):
    """Admit an entire coherent batch, retaining only individually accepted poses."""
    def refused(reason):
        return ContactBatchEvidence(False, reason)
    try:
        require_matching_detection(domain, request.detection, generation=int(request.generation),
                                   contact_model_hash=model.identity_hash)
        if model.model_version != domain.contact_model_version or model.contact_frame != 'suction_contact_frame':
            return refused('CONTACT_QUALITY_MODEL_MISMATCH')
        if response.success is not True:
            return refused('CONTACT_QUALITY_REFUSED:'+response.reason)
        stamp_ns = int(response.header.stamp.sec)*10**9+int(response.header.stamp.nanosec)
        if (response.reason != 'QUALITY_BATCH_COMPLETE' or response.header.frame_id != 'world'
                or stamp_ns != domain.stamp_ns
                or response.domain_sha256 != request.domain_sha256
                or response.domain_sha256 != domain.digest()
                or not _sha256(response.quality_policy_sha256) or not _sha256(response.samples_sha256)
                or (request.expected_quality_policy_sha256 and
                    request.expected_quality_policy_sha256 != response.quality_policy_sha256)
                or not math.isfinite(response.elapsed_s) or not 0 <= response.elapsed_s < request.budget_s
                or len(response.contacts) != len(request.queries)):
            return refused('CONTACT_QUALITY_IDENTITY_MISMATCH')
        contacts, verdicts = [], []
        for wire, query in zip(response.contacts, request.queries):
            if (wire.query_id != query.query_id or tuple(wire.center_uv_top_m) != tuple(query.center_uv_top_m)
                    or wire.yaw_rad != query.yaw_rad
                    or not 0 <= wire.refinement_count <= request.max_refinements
                    or not _sha256(wire.evidence_sha256)):
                return refused('CONTACT_QUALITY_QUERY_MISMATCH')
            pose, metrics = (), ()
            if wire.accepted:
                if wire.reason != 'DIRECTED_CONTACT_QUALITY_OK' or wire.refinement_count < 1:
                    return refused('CONTACT_QUALITY_INVALID_ACCEPTANCE')
                if len(wire.quality_metrics_json) > 4096:
                    return refused('CONTACT_QUALITY_INVALID_METRICS')
                raw = json.loads(wire.quality_metrics_json)
                if (not isinstance(raw, dict) or set(raw) != set(QUALITY_METRICS)
                        or any(type(v) not in (int, float) or not math.isfinite(v) for v in raw.values())):
                    return refused('CONTACT_QUALITY_INVALID_METRICS')
                metrics = tuple((name, raw[name]) for name in QUALITY_METRICS)
                pose = tuple(wire.world_from_contact_frame)
            elif wire.quality_metrics_json or any(wire.world_from_contact_frame):
                return refused('CONTACT_QUALITY_REFUSAL_HAS_POSE')
            quality = DirectedSuctionResult(bool(wire.accepted), wire.reason, response.domain_sha256,
                response.quality_policy_sha256, response.samples_sha256, tuple(wire.center_uv_top_m),
                wire.yaw_rad, pose, metrics, int(wire.refinement_count), wire.evidence_sha256)
            if quality.evidence_digest() != wire.evidence_sha256:
                return refused('CONTACT_QUALITY_EVIDENCE_MISMATCH')
            verdicts.append((wire.query_id, bool(wire.accepted), wire.reason, wire.evidence_sha256))
            if wire.accepted:
                matrix = np.asarray(pose).reshape(4, 4)
                transform = Transform.from_matrix(matrix)
                # Numerical identity checks, not new physical quality limits.
                uv = np.asarray((domain.u_axis_world, domain.v_axis_world)) @ matrix[:3, 3]
                normal = -matrix[:3, 2]
                direction = (math.cos(query.yaw_rad)*np.asarray(domain.u_axis_world)
                             + math.sin(query.yaw_rad)*np.asarray(domain.v_axis_world))
                tangent = direction - np.dot(direction, normal)*normal
                if (np.dot(normal, domain.normal_world) <= 0 or np.linalg.norm(tangent) <= 1e-12
                        or not np.allclose(uv, query.center_uv_top_m, atol=1e-8, rtol=0)
                        or not np.allclose(matrix[:3, 0], tangent/np.linalg.norm(tangent), atol=1e-7, rtol=0)):
                    return refused('CONTACT_QUALITY_POSE_QUERY_MISMATCH')
                coverage = check_observed_contact_footprint(domain, model, matrix)
                if not coverage.contained:
                    return refused('CONTACT_QUALITY_SUPPORT_MISMATCH:'+coverage.reason)
                contacts.append(DirectedContact(wire.query_id, quality.query_uv, quality.yaw_rad, transform, quality))
        return ContactBatchEvidence(True, 'CONTACT_QUALITY_BATCH_VALID', tuple(contacts), tuple(verdicts),
            response.domain_sha256, response.quality_policy_sha256, response.samples_sha256)
    except (TypeError, ValueError, OverflowError, AttributeError) as error:
        return refused('INVALID_CONTACT_QUALITY_WITNESS:'+type(error).__name__)


def grasp_for_contact(domain, contact, *, observation_id, rank, pick_clearances=None):
    """Derive G once; all pick references retain the fitted tool orientation."""
    if (type(contact) is not DirectedContact or not contact.quality.accepted
            or contact.quality.domain_sha256 != domain.digest()
            or contact.quality.evidence_digest() != contact.quality.evidence_sha256
            or contact.query_uv != contact.quality.query_uv or contact.yaw_rad != contact.quality.yaw_rad
            or contact.world_from_contact != Transform.from_matrix(
                np.asarray(contact.quality.world_from_contact_frame).reshape(4, 4))):
        raise ValueError('measured directed contact receipt required')
    clearances = dict(DEFAULT_PICK_CLEARANCES)
    if pick_clearances is not None:
        clearances.update(pick_clearances)
    if clearances['attach'] != 0:
        raise ValueError('measured contact frame must reach the observed face')
    tool = contact.world_from_contact
    # +Z points into the face, so negative tool-Z is the measured outward normal.
    references = []
    for name in ('pre_grasp', 'approach', 'attach', 'pick_retreat'):
        distance = float(clearances[name])
        if not math.isfinite(distance) or distance < 0:
            raise ValueError('finite nonnegative pick clearance required')
        references.append(tool.compose(Transform((0., 0., -distance), (0., 0., 0., 1.))))
    world_box = Transform(domain.box_xyz_world, domain.box_quaternion_xyzw)
    return GraspCandidate(contact.query_id, world_box.inverse().compose(tool), observation_id,
                          domain.contact_model_hash, rank, tuple(references))


class DirectedContactPort:
    """Read-only ROS transport, one in-flight request and a shared deadline."""
    def __init__(self, node, *, recorder=None):
        from luggage_msgs.srv import EvaluateDirectedSuctionV2, EvaluateFrozenSuctionV2
        self.service = EvaluateDirectedSuctionV2
        self.client = node.create_client(self.service, '/luggage/evaluate_directed_suction_v2')
        self.frozen_service = EvaluateFrozenSuctionV2
        self.frozen_client = node.create_client(EvaluateFrozenSuctionV2, '/luggage/evaluate_frozen_suction_v2')
        self.recorder, self.inflight = recorder, None

    def evaluate(self, domain, detection, queries, model, *, deadline, cancelled,
                 max_refinements=3, expected_policy_sha256='',retained_observation=None):
        from rclpy.serialization import serialize_message
        def stopped():
            return cancelled() or time.monotonic() >= deadline
        def refused(reason):
            return ContactBatchEvidence(False, reason)
        if not math.isfinite(deadline):
            return refused('INVALID_CONTACT_QUALITY_DEADLINE')
        if stopped():
            return refused('CANCELLED_OR_DEADLINE')
        if self.inflight is not None and not self.inflight.done():
            return refused('CONTACT_QUALITY_BACKEND_BUSY')
        client = self.frozen_client if retained_observation is not None else self.client
        if not client.service_is_ready():
            return refused('CONTACT_QUALITY_BACKEND_UNAVAILABLE')
        try:
            # Keep physical geometry stable while the service is in flight.
            model = deepcopy(model)
            if retained_observation is not None:
                from luggage_planning.joint_operation.actual_contact import RetainedContactObservation
                if (type(retained_observation) is not RetainedContactObservation
                        or retained_observation.domain != domain
                        or retained_observation.binding.domain_sha256 != domain.digest()
                        or (expected_policy_sha256 and expected_policy_sha256 !=
                            retained_observation.binding.quality_policy_sha256)):
                    raise ValueError('FROZEN_DIRECTED_OBSERVATION_MISMATCH')
                expected_policy_sha256 = retained_observation.binding.quality_policy_sha256
            request = contact_query(domain, detection, queries, model,
                budget_s=min(10., deadline-time.monotonic()), max_refinements=max_refinements,
                expected_policy_sha256=expected_policy_sha256)
        except (TypeError, ValueError) as error:
            return refused('INVALID_CONTACT_QUALITY_INPUT:'+str(error))
        if stopped():
            return refused('CANCELLED_OR_DEADLINE')
        request.budget_s = min(10., deadline-time.monotonic())
        wire_request = request
        if retained_observation is not None:
            from luggage_perception.frozen_suction_service import binding_to_message
            wire_request = self.frozen_service.Request(binding=binding_to_message(retained_observation.binding),
                detection=request.detection,queries=request.queries,
                max_refinements=request.max_refinements,budget_s=request.budget_s)
        prefix = 'frozen_' if retained_observation is not None else ''
        if self.recorder:
            self.recorder(prefix+'request', serialize_message(wire_request))
        event = threading.Event()
        try:
            self.inflight = client.call_async(wire_request)
            self.inflight.add_done_callback(lambda _: event.set())
            while not event.wait(min(.01, max(0., deadline-time.monotonic()))):
                if stopped():
                    return refused('CANCELLED_OR_DEADLINE')
            response = self.inflight.result()
            if self.recorder:
                self.recorder(prefix+'response', serialize_message(response))
            if stopped():
                return refused('CANCELLED_OR_DEADLINE')
            result = contact_evidence(response, request, domain, model)
            return refused('CANCELLED_OR_DEADLINE') if stopped() else result
        except Exception as error:
            return refused('CONTACT_QUALITY_TRANSPORT_ERROR:'+type(error).__name__)
