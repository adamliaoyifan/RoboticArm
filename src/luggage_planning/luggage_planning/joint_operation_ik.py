"""Exact target IK proposals checked in a caller-owned full carrying scene."""
from dataclasses import dataclass
import hashlib
import math
import threading
import time

from luggage_planning.placement_prior_contracts import Transform, finite_tuple


@dataclass(frozen=True)
class SnapshotIKResult:
    complete: bool
    success: bool
    reason: str
    joints: tuple = ()


def ik_evidence(response, request, model):
    identity=('request_id','snapshot_id','model_sha256','scene_sha256')
    if (any(getattr(response,k)!=getattr(request,k) for k in identity)
            or request.model_sha256!=model.model_sha256):
        return SnapshotIKResult(False,False,'IK_IDENTITY_MISMATCH')
    if not math.isfinite(response.elapsed_s) or not 0<=response.elapsed_s<=request.budget_s:
        return SnapshotIKResult(False,False,'IK_ELAPSED_INVALID')
    if not response.success:
        if any(response.joints):return SnapshotIKResult(False,False,'REFUSED_IK_HAS_JOINTS')
        return SnapshotIKResult(bool(response.complete),False,str(response.reason))
    try:
        q=finite_tuple(response.joints,6,'refined IK')
        p=request.world_from_tool
        target=Transform((p.position.x,p.position.y,p.position.z),
            (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w))
        if (not response.complete or response.reason!='EXACT_TARGET_CHECKED'
                or not 1<=response.checked_states<=1000 or not model.matches(q,target)):
            raise ValueError('exact FK/full check receipt required')
    except (TypeError,ValueError):return SnapshotIKResult(False,False,'IK_WITNESS_INVALID')
    return SnapshotIKResult(True,True,response.reason,q)


class NativeSnapshotIKPort:
    def __init__(self,node,model,*,recorder=None,max_query_s=.2):
        from luggage_msgs.srv import RefineSnapshotToolTargetV2
        if not math.isfinite(max_query_s) or not 0<max_query_s<=10:
            raise ValueError('finite bounded IK query budget required')
        self.service=RefineSnapshotToolTargetV2
        self.client=node.create_client(self.service,'/joint_operation/refine_snapshot_tool_target')
        self.model,self.recorder,self.max_query_s=model,recorder,max_query_s
        self.inflight=None

    def refine(self,snapshot,target,seed,*,request_id,deadline,cancelled):
        from luggage_planning.joint_operation_scenes import _pose
        seed=finite_tuple(seed,6,'IK proposal seed')
        def stopped():return cancelled() or time.monotonic()>=deadline
        def refused(reason):return SnapshotIKResult(False,False,reason)
        if stopped():return refused('CANCELLED_OR_DEADLINE')
        if self.inflight is not None and not self.inflight.done():return refused('IK_BACKEND_BUSY')
        if not self.client.service_is_ready():return refused('IK_BACKEND_UNAVAILABLE')
        if not request_id.strip() or snapshot.model_hash!=self.model.model_sha256:
            return refused('IK_INPUT_IDENTITY_MISMATCH')
        request=self.service.Request(request_id=request_id,
            snapshot_id=hashlib.sha256(repr(snapshot.identity).encode()).hexdigest(),
            model_sha256=snapshot.model_hash,scene_sha256=hashlib.sha256(snapshot.scene_cdr).hexdigest(),
            scene_cdr=list(snapshot.scene_cdr),world_from_tool=_pose(target),seed_joints=list(seed),
            budget_s=min(self.max_query_s,deadline-time.monotonic()))
        if request.budget_s<=0:return refused('CANCELLED_OR_DEADLINE')
        if self.recorder:self.recorder('request',request)
        event=threading.Event();self.inflight=self.client.call_async(request)
        self.inflight.add_done_callback(lambda _:event.set())
        while not event.wait(min(.01,max(0.,deadline-time.monotonic()))):
            if stopped():return refused('CANCELLED_OR_DEADLINE')
        response=self.inflight.result()
        if self.recorder:self.recorder('response',response)
        if stopped():return refused('CANCELLED_OR_DEADLINE')
        return ik_evidence(response,request,self.model)
