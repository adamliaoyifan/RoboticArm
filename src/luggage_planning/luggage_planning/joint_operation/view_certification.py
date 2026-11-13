"""Independently certify one timed empty-arm view path in a full private scene."""
from dataclasses import dataclass
import hashlib
import math
import time

from luggage_planning.joint_operation.certification import _valid_response, _wire_leg
from luggage_planning.joint_operation.rpc import OperationRpc
from luggage_planning.motion.moveit_state_validity import MAX_CERT_SAMPLE_DT_NS


@dataclass(frozen=True)
class ViewMotionCertificate:
    accepted: bool
    reason: str
    certificate_id: str = ''


class NativeViewCertificationPort:
    def __init__(self,node,model,*,velocity_limits,acceleration_limits,max_samples,recorder,rpc=None):
        from luggage_msgs.srv import ValidateRouteV2
        for limits in (velocity_limits,acceleration_limits):
            if len(limits)!=6 or any(not math.isfinite(v) or v<=0 for v in limits):
                raise ValueError('SIX_POSITIVE_VIEW_CERTIFICATION_LIMITS_REQUIRED')
        if type(max_samples) is not int or not 2<=max_samples<=50000:
            raise ValueError('BOUNDED_VIEW_CERTIFICATION_SAMPLES_REQUIRED')
        self.model,self.max_samples = model,max_samples
        self.velocity_limits,self.acceleration_limits = tuple(velocity_limits),tuple(acceleration_limits)
        self.service = ValidateRouteV2
        self.client = node.create_client(self.service,'/validate_route_v2')
        self.recorder = recorder
        self.rpc = rpc if rpc is not None else OperationRpc(self._record_message)

    def _record_message(self,name,message):
        from rclpy.serialization import serialize_message
        if self.recorder:self.recorder(name,message if isinstance(message,bytes) else serialize_message(message))

    def certify(self,snapshot,leg,route_digest,*,deadline,cancelled):
        from rclpy.serialization import serialize_message
        try:
            request,response=self.validate(snapshot,leg,route_digest,deadline=deadline,cancelled=cancelled)
        except (ValueError,RuntimeError,TimeoutError) as error:
            return ViewMotionCertificate(False,str(error))
        return ViewMotionCertificate(True,'NATIVE_VIEW_PATH_CHECKED',
            hashlib.sha256(serialize_message(request)+serialize_message(response)).hexdigest())

    def validate(self,snapshot,leg,route_digest,*,deadline,cancelled):
        """Return the exact independent request/response for the dispatch gate."""
        from moveit_msgs.msg import PlanningScene
        from rclpy.serialization import deserialize_message
        if cancelled() or time.monotonic()>=deadline:raise ValueError('CANCELLED_OR_DEADLINE')
        if snapshot.model_hash!=self.model.model_sha256:raise ValueError('VIEW_FK_MODEL_MISMATCH')
        # Bound interpolation work before allocating wire samples.
        count=sum(max(1,(b.time_from_start_ns-a.time_from_start_ns+MAX_CERT_SAMPLE_DT_NS-1)
            //MAX_CERT_SAMPLE_DT_NS)+1 for a,b in zip(leg.knots,leg.knots[1:]))
        if count>self.max_samples:raise ValueError('VIEW_CERTIFICATION_SAMPLE_CAP')
        request=self.service.Request(request_id=leg.trajectory_request_id,
            scene_revision=snapshot.scene_revision,route_digest=route_digest,
            scene_digest=hashlib.sha256(snapshot.scene_cdr).hexdigest(),
            load_state_scenes=[deserialize_message(snapshot.scene_cdr,PlanningScene)],
            max_sample_dt_ns=float(MAX_CERT_SAMPLE_DT_NS),max_sample_step_m=.01,
            joint_velocity_limits=list(self.velocity_limits),joint_acceleration_limits=list(self.acceleration_limits),
            distance_stride=2**30,budget_s=max(0.,deadline-time.monotonic()),legs=[_wire_leg(leg)])
        self.rpc.cancelled=cancelled
        response=self.rpc.call(self.client,request,deadline,'view_certification')
        error=_valid_response(response,request,self.model.model_sha256)
        if error:raise ValueError(error)
        return request,response
