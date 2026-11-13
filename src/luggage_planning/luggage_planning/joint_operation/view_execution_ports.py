"""Current-source and native admission for an empty pre-pick view.

Composes the whole-operation transport/model/O reader without changing its plan,
attachment or world state. Preparation provenance alone never permits dispatch.
"""
import hashlib
from types import SimpleNamespace

from luggage_description.frame_diagnostics import BoundaryContext
from luggage_description.ros_frame_bindings.poses import pose_from_message
from luggage_perception.ros_suction_domain import domain_from_message
from luggage_planning.joint_operation.adapters import operation_leg_from_message
from luggage_planning.joint_operation.map_source import map_source_from_message,OperationMapRefresh
from luggage_planning.joint_operation.view_certification import NativeViewCertificationPort
from luggage_planning.joint_operation.view_scene import pickup_view_scene
from luggage_planning.placement_prior.requests import admit_placement_targets
from luggage_planning.placement_prior.contracts import Transform
from luggage_planning.placement_prior.snapshot import build_private_snapshot,proposal_scene_hash


class PickupViewExecutionPorts:
    def __init__(self,parent,*,static_world_factory):
        from luggage_msgs.srv import PreparePickupViewV2,CheckPreparedPickupViewV2
        self.parent,self.node=parent,parent.node
        self.static_world_factory,self.static_world=static_world_factory,None
        self.base=parent.ports
        self.prepare_type,self.check_type=PreparePickupViewV2,CheckPreparedPickupViewV2
        self.prepare_client=self.node.create_client(PreparePickupViewV2,'/joint_operation/prepare_pickup_view')
        self.check_client=self.node.create_client(CheckPreparedPickupViewV2,'/joint_operation/check_pickup_view')
        native=self.base.native
        self.native=NativeViewCertificationPort(self.node,self.base.model,
            velocity_limits=native.velocity_limits,acceleration_limits=native.acceleration_limits,
            max_samples=native.max_samples,recorder=parent.record,rpc=self.base.rpc)

    def prepare(self,search,*,deadline,cancelled):
        import time
        if hasattr(self,'snapshot'):del self.snapshot
        sample=self.base.joints()
        request=self.prepare_type.Request()
        for field in search.get_fields_and_field_types():setattr(request,field,getattr(search,field))
        request.robot_state_stamp_valid=sample.stamp_valid
        request.robot_state_stamp_ns=sample.stamp_ns
        request.start_joints=list(sample.positions)
        request.budget_s=min(max(0.,deadline-time.monotonic()),
            float(self.node.get_parameter('max_preparation_budget_s').value))
        self.base.rpc.cancelled=cancelled
        response=self.base.rpc.call(self.prepare_client,request,deadline,'pickup_view_prepare')
        if not response.success:raise ValueError(response.reason)
        self.plan=response.plan
        if (self.plan.request_id,self.plan.task_request_id,self.plan.domain_sha256,self.plan.query_id)!= (
                search.request_id,search.task_request_id,search.domain_sha256,search.query_id):
            raise ValueError('PICKUP_VIEW_PREPARE_CORRELATION_MISMATCH')
        if self.plan.model_sha256!=self.base.model.model_sha256:
            raise ValueError('PICKUP_VIEW_MODEL_MISMATCH')
        self.source=map_source_from_message(self.plan.map_source)
        self.checked=self.check(deadline)
        self.domain=domain_from_message(self.checked.top_domain)
        if self.domain.digest()!=search.domain_sha256:
            raise ValueError('PICKUP_VIEW_RETAINED_DOMAIN_MISMATCH')
        wire=self.plan.leg
        context=BoundaryContext('pickup_view','joint_planner','joint_executor',
            self.plan.request_id,self.base.revision,'','','robot_state')
        self.leg=operation_leg_from_message(wire,context)
        return self.plan

    def check(self,deadline):
        import time
        budget=min(max(0.,deadline-time.monotonic()),
            float(self.node.get_parameter('max_preparation_budget_s').value))
        request=self.check_type.Request(plan=self.plan,budget_s=budget)
        result=self.base.rpc.call(self.check_client,request,deadline,'pickup_view_owner_check')
        if not result.current:raise ValueError(result.reason)
        return result

    def capture(self,deadline):
        """Latest O and complete static/pickup scene, with original target kept."""
        if self.static_world is None:self.static_world=self.static_world_factory()
        current=self.base.observer.capture(scene_revision=self.base.revision,request_id=self.plan.request_id,
            expected_epoch=self.source.epoch,expected_revision=None,deadline=deadline,require_latest=False)
        baseline=self.base.scene(deadline=deadline)
        self.static_world.admit(baseline)
        snapshot=build_private_snapshot(current.surface,baseline,self.base.model.model_sha256,
            self.base.container,collision_margin_m=self.base.margin,volume=current.volume,
            vertical_collision_margin_m=self.base.vertical_margin)
        refresh=OperationMapRefresh(self.source,current.source,hashlib.sha256(current.surface.record_json).hexdigest())
        identity=SimpleNamespace(scene_revision=self.source.scene_revision,geometry_hash=self.source.geometry_hash,
            map_revision=self.source.map_revision)
        if not refresh.admits(identity,snapshot):raise ValueError('PICKUP_VIEW_MAP_RESET_OR_REBOUND')
        placement=self.checked.probe.placement
        pose=pose_from_message(placement,'slot',BoundaryContext('PICKUP_VIEW_TARGET','prepare_owner',
            'joint_operation',self.plan.task_request_id,self.base.revision,'container_link','box_center','scene'))
        target=Transform(pose['position'],pose['quaternion_xyzw'])
        if admit_placement_targets(current.surface,self.domain.box_wdh,(target,),volume=current.volume)!=(True,):
            raise ValueError('PICKUP_VIEW_ORIGINAL_TARGET_REFUSED')
        derived=pickup_view_scene(snapshot,self.domain,self.base.pickup_id,admit_static_scene=self.static_world.admit)
        self.parent.record('pickup_view_current_scene',derived.scene_cdr)
        return current,baseline,derived

    def validate(self,leg,*,deadline,cancelled):
        self.checked=self.check(deadline)
        self.current_map,self.baseline,self.snapshot=self.capture(deadline)
        return self.native.validate(self.snapshot,leg,self.plan.route_digest,deadline=deadline,cancelled=cancelled)

    def current(self,deadline):
        checked=self.check(deadline)
        if (checked.frozen_observation!=self.checked.frozen_observation
                or checked.top_domain!=self.checked.top_domain
                or checked.checked_scene_cdr!=self.checked.checked_scene_cdr):
            return False
        # The first call is before native recertification; that call still
        # captures and checks the full latest scene. Later calls bind its source.
        if not hasattr(self,'snapshot'):return True
        return (proposal_scene_hash(self.base.scene(deadline=deadline))==proposal_scene_hash(self.baseline)
            and self.base.observer.planning_current(self.current_map,deadline=deadline))
