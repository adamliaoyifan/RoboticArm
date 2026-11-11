"""Typed retention/actual-footprint ports owned by the perception process."""
import json
import math
import time

from luggage_perception.directed_suction_quality import _digest
from luggage_perception.frozen_suction_observation import FrozenObservationBinding, FrozenSuctionObservations
from luggage_perception.ros_suction_domain import require_matching_detection


def binding_to_message(binding):
    from luggage_msgs.msg import FrozenSuctionObservationV2
    message = FrozenSuctionObservationV2()
    message.header.frame_id = 'world'
    message.header.stamp.sec,message.header.stamp.nanosec = divmod(binding.stamp_ns,10**9)
    for name in FrozenObservationBinding.__dataclass_fields__:
        if name != 'stamp_ns':setattr(message,name,getattr(binding,name))
    message.binding_sha256 = binding.digest()
    return message


def binding_from_message(message):
    if message.header.frame_id != 'world':
        raise ValueError('frozen acquisition world frame required')
    binding = FrozenObservationBinding(**{name:(
        int(message.header.stamp.sec)*10**9+int(message.header.stamp.nanosec)
        if name == 'stamp_ns' else getattr(message,name))
        for name in FrozenObservationBinding.__dataclass_fields__})
    if message.binding_sha256 != binding.digest():
        raise ValueError('frozen observation binding digest mismatch')
    return binding


class FrozenSuctionService:
    def __init__(self, store, *, context_provider, model_provider, clock=time.monotonic):
        self.store,self.context_provider,self.model_provider,self.clock = store,context_provider,model_provider,clock

    def retain(self, request, response):
        from luggage_msgs.msg import FrozenSuctionObservationV2
        response.retained,response.reason,response.binding = False,'FROZEN_OBSERVATION_UNAVAILABLE',FrozenSuctionObservationV2()
        try:
            context = self.context_provider()
            if context is None or request.domain_sha256 != context.domain_sha256:
                raise ValueError('CURRENT_MEASURED_ACQUISITION_MISMATCH')
            require_matching_detection(context.domain,request.detection,generation=int(request.generation),
                contact_model_hash=context.domain.contact_model_hash)
            if (_digest(self.model_provider().as_dict()) != context.policy_sha256
                    or (request.expected_quality_policy_sha256
                        and request.expected_quality_policy_sha256 != context.policy_sha256)):
                raise ValueError('CURRENT_CONTACT_POLICY_MISMATCH')
            binding = self.store.retain(request.request_id,request.observation_id,context)
            # Model can change during copying; never publish a stale policy pin.
            if _digest(self.model_provider().as_dict()) != binding.quality_policy_sha256:
                self.store.revoke(binding.request_id,binding.observation_id)
                raise ValueError('CURRENT_CONTACT_POLICY_CHANGED')
            response.binding = binding_to_message(binding)
            response.retained,response.reason = True,'MEASURED_OBSERVATION_RETAINED'
        except Exception as error:
            response.reason = 'FROZEN_OBSERVATION_REFUSED:'+type(error).__name__+':'+str(error)
        return response

    def check(self, request, response):
        from luggage_msgs.msg import FrozenSuctionObservationV2
        response.checked,response.accepted = False,False
        response.binding = FrozenSuctionObservationV2()
        response.reason = 'FROZEN_ACTUAL_CONTACT_UNAVAILABLE'
        response.attachment_geometry_sha256,response.evidence_sha256,response.quality_metrics_json = '','',''
        response.world_from_contact_frame_at_observation = [0.]*16
        response.plane_height_offset_m,response.normal_difference_rad = 0.,0.
        try:
            if not math.isfinite(request.budget_s) or not 0 < request.budget_s <= 10:
                raise ValueError('BOUNDED_ACTUAL_CONTACT_BUDGET_REQUIRED')
            deadline = self.clock()+request.budget_s
            binding = binding_from_message(request.binding)
            result = self.store.check(binding,
                attachment_geometry_sha256=request.attachment_geometry_sha256,
                measured_box_wdh=request.measured_box_wdh,
                box_from_contact_frame=request.box_from_contact_frame,
                current_model=self.model_provider(),deadline=deadline)
            if (self.clock() >= deadline or self.store.lookup(binding) is None
                    or _digest(self.model_provider().as_dict()) != binding.quality_policy_sha256):
                raise ValueError('FROZEN_ACTUAL_CONTACT_SOURCE_CHANGED_OR_DEADLINE')
            response.binding = binding_to_message(binding)
            response.attachment_geometry_sha256 = result.attachment_geometry_sha256
            if result.world_from_contact_frame_at_observation:
                response.world_from_contact_frame_at_observation = list(result.world_from_contact_frame_at_observation)
            response.quality_metrics_json = json.dumps(dict(result.metrics),sort_keys=True,allow_nan=False)
            response.plane_height_offset_m = result.plane_height_offset_m
            response.normal_difference_rad = result.normal_difference_rad
            response.evidence_sha256 = result.digest()
            response.checked,response.accepted,response.reason = True,result.accepted,result.reason
        except Exception as error:
            response.reason = 'FROZEN_ACTUAL_CONTACT_REFUSED:'+type(error).__name__+':'+str(error)
        return response


def register_frozen_suction_services(node, *, context_provider, model_provider):
    """Same registration used by the detector and isolated behavioral tests."""
    from luggage_msgs.srv import RetainSuctionObservationV2, CheckFrozenSuctionContactV2
    store = FrozenSuctionObservations(
        max_records=node.declare_parameter('frozen_suction_record_limit',16).value,
        max_bytes=node.declare_parameter('frozen_suction_byte_limit',64*1024*1024).value,
        lifetime_s=node.declare_parameter('frozen_suction_lifetime_s',300.).value)
    handler = FrozenSuctionService(store,context_provider=context_provider,model_provider=model_provider)
    # Keep service objects with their owner. These callbacks never execute motion.
    handler.services = (
        node.create_service(RetainSuctionObservationV2,'/luggage/retain_suction_observation_v2',handler.retain),
        node.create_service(CheckFrozenSuctionContactV2,'/luggage/check_frozen_suction_contact_v2',handler.check))
    return handler
