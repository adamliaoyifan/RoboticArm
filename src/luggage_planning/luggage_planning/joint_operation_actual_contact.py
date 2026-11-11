"""Read-only transport of retained measured evidence and actual footprint checks.

Execution still owns actual attachment/seal validity, issuance/source liveness
and native collision validation. This port cannot authorize motion or release.
"""
from dataclasses import dataclass
import copy
import json
import math
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from luggage_perception.directed_suction_quality import QUALITY_METRICS
from luggage_perception.frozen_suction_observation import ActualFootprintResult
from luggage_perception.frozen_suction_service import binding_from_message, binding_to_message
from luggage_perception.ros_suction_domain import require_matching_detection
from luggage_planning.joint_operation_contracts import OperationIdentity
from luggage_planning.placement_prior_contracts import Transform
from st09_contracts.attachment_geometry import AttachmentGeometry


@dataclass(frozen=True)
class RetainedContactObservation:
    binding: object
    domain: object


@dataclass(frozen=True)
class ContactRetention:
    retained: bool
    reason: str
    observation: object = None


@dataclass(frozen=True)
class ActualContactEvidence:
    checked: bool
    accepted: bool
    reason: str
    result: object = None


def _matrix(transform):
    matrix = np.eye(4)
    matrix[:3,:3] = Rotation.from_quat(transform.quaternion_xyzw).as_matrix()
    matrix[:3,3] = transform.translation
    return matrix


class FrozenActualContactPort:
    def __init__(self, node, *, recorder=None):
        from luggage_msgs.srv import RetainSuctionObservationV2, CheckFrozenSuctionContactV2
        self.retain_service,self.check_service = RetainSuctionObservationV2,CheckFrozenSuctionContactV2
        self.retain_client = node.create_client(RetainSuctionObservationV2,'/luggage/retain_suction_observation_v2')
        self.check_client = node.create_client(CheckFrozenSuctionContactV2,'/luggage/check_frozen_suction_contact_v2')
        self.recorder,self._inflight = recorder,{}
        self._active = threading.Lock()

    def _call(self, client, request, *, deadline, cancelled, name):
        from rclpy.serialization import serialize_message
        def stopped():return cancelled() or time.monotonic() >= deadline
        if not math.isfinite(deadline) or stopped():
            raise ValueError('CANCELLED_OR_ACTUAL_CONTACT_DEADLINE')
        if any(not old.done() for old in self._inflight.values()):
            raise ValueError('FROZEN_CONTACT_BACKEND_BUSY')
        if not client.service_is_ready():
            raise ValueError('FROZEN_CONTACT_BACKEND_UNAVAILABLE')
        if self.recorder:self.recorder(name+'_request',serialize_message(request))
        event = threading.Event()
        future = self._inflight[client] = client.call_async(request)
        future.add_done_callback(lambda _:event.set())
        while not event.wait(min(.01,max(0.,deadline-time.monotonic()))):
            if stopped():raise ValueError('CANCELLED_OR_ACTUAL_CONTACT_DEADLINE')
        response = future.result()
        if self.recorder:self.recorder(name+'_response',serialize_message(response))
        if stopped():raise ValueError('CANCELLED_OR_ACTUAL_CONTACT_DEADLINE')
        return response

    def retain(self, domain, detection, *, request_id, observation_id,
               expected_policy_sha256='', deadline, cancelled):
        if not self._active.acquire(blocking=False):
            return ContactRetention(False,'FROZEN_CONTACT_PORT_BUSY')
        try:
            require_matching_detection(domain,detection,generation=domain.generation,
                contact_model_hash=domain.contact_model_hash)
            request = self.retain_service.Request(request_id=request_id,observation_id=observation_id,
                domain_sha256=domain.digest(),detection=copy.deepcopy(detection),generation=domain.generation,
                expected_quality_policy_sha256=expected_policy_sha256)
            response = self._call(self.retain_client,request,deadline=deadline,cancelled=cancelled,name='retain')
            if response.retained is not True:
                return ContactRetention(False,response.reason)
            binding = binding_from_message(response.binding)
            if ((binding.request_id,binding.observation_id,binding.box_id,binding.generation,binding.stamp_ns,
                    binding.contact_model_hash,binding.domain_sha256) !=
                    (request_id,observation_id,domain.box_id,domain.generation,domain.stamp_ns,
                     domain.contact_model_hash,domain.digest())
                    or (expected_policy_sha256 and expected_policy_sha256 != binding.quality_policy_sha256)
                    or not binding.retention_id
                    or any(len(value)!=64 or any(c not in '0123456789abcdef' for c in value)
                        for value in (binding.quality_policy_sha256,binding.samples_sha256))):
                raise ValueError('FROZEN_RETENTION_BINDING_MISMATCH')
            return ContactRetention(True,response.reason,RetainedContactObservation(binding,domain))
        except Exception as error:
            return ContactRetention(False,'FROZEN_RETENTION_REFUSED:'+type(error).__name__+':'+str(error))
        finally:self._active.release()

    def check(self, observation, *, identity, geometry, deadline, cancelled):
        if not self._active.acquire(blocking=False):
            return ActualContactEvidence(False,False,'FROZEN_CONTACT_PORT_BUSY')
        try:
            if (type(observation) is not RetainedContactObservation or type(identity) is not OperationIdentity
                    or type(geometry) is not AttachmentGeometry):
                raise ValueError('FROZEN_OBSERVATION_ACTUAL_IDENTITY_GEOMETRY_REQUIRED')
            binding,domain = observation.binding,observation.domain
            if ((identity.request_id,identity.observation_id,identity.box_id,identity.generation,
                    identity.observation_stamp_ns,identity.contact_model_hash) !=
                    (binding.request_id,binding.observation_id,binding.box_id,binding.generation,
                     binding.stamp_ns,binding.contact_model_hash)
                    or domain.digest() != binding.domain_sha256
                    or geometry.measured_dimensions_wdh != domain.box_wdh):
                raise ValueError('FROZEN_ACTUAL_CONTACT_IDENTITY_MISMATCH')
            geometry.__post_init__()
            relative = _matrix(Transform(geometry.t_tool_box_translation,
                geometry.t_tool_box_quaternion_xyzw).inverse())
            request = self.check_service.Request(binding=binding_to_message(binding),
                attachment_geometry_sha256=geometry.digest(),
                measured_box_wdh=list(geometry.measured_dimensions_wdh),
                box_from_contact_frame=list(relative.ravel()),
                budget_s=min(10.,max(0.,deadline-time.monotonic())))
            response = self._call(self.check_client,request,deadline=deadline,cancelled=cancelled,name='actual_contact')
            if response.checked is not True:
                return ActualContactEvidence(False,False,response.reason)
            if (binding_from_message(response.binding) != binding
                    or response.attachment_geometry_sha256 != geometry.digest()):
                raise ValueError('ACTUAL_CONTACT_RESPONSE_BINDING_MISMATCH')
            pose = np.asarray(response.world_from_contact_frame_at_observation).reshape(4,4)
            Transform.from_matrix(pose)
            expected = _matrix(Transform(domain.box_xyz_world,domain.box_quaternion_xyzw)) @ relative
            if not np.allclose(pose,expected,atol=1e-9,rtol=0):
                raise ValueError('ACTUAL_CONTACT_ACQUISITION_POSE_MISMATCH')
            metrics = json.loads(response.quality_metrics_json)
            if (not isinstance(metrics,dict) or set(metrics) not in (set(),set(QUALITY_METRICS))
                    or any(type(v) not in (int,float) or not math.isfinite(v) for v in metrics.values())
                    or (response.accepted and (response.reason!='FROZEN_ACTUAL_FOOTPRINT_QUALITY_OK'
                                              or set(metrics)!=set(QUALITY_METRICS)))):
                raise ValueError('ACTUAL_CONTACT_QUALITY_METRICS_INVALID')
            result = ActualFootprintResult(binding.digest(),geometry.digest(),geometry.measured_dimensions_wdh,
                tuple(relative.ravel()),response.accepted,response.reason,tuple(pose.ravel()),
                tuple((name,metrics[name]) for name in QUALITY_METRICS if name in metrics),
                response.plane_height_offset_m,response.normal_difference_rad)
            if result.digest() != response.evidence_sha256:
                raise ValueError('ACTUAL_CONTACT_EVIDENCE_DIGEST_MISMATCH')
            return ActualContactEvidence(True,result.accepted,result.reason,result)
        except Exception as error:
            return ActualContactEvidence(False,False,'FROZEN_ACTUAL_CONTACT_REFUSED:'+type(error).__name__+':'+str(error))
        finally:self._active.release()
