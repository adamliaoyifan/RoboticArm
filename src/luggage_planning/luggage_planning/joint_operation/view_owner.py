"""Read-only pickup-view preparation at the existing O/source ownership boundary.

Only owner-issued unresolved contacts are eligible. This adapter never executes,
updates a map, fills support or changes the six-leg operation interface.
"""
from dataclasses import asdict,replace
import json
import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.serialization import serialize_message

from luggage_description.frame_diagnostics import BoundaryContext
from luggage_description.ros_frame_bindings.joints import joints_to_message
from luggage_description.ros_frame_bindings.poses import pose_from_message
from luggage_mapping.observer import ObserverSnapshot
from luggage_perception.pickup_view_service import view_from_message
from luggage_planning.joint_operation.contact_observation import UnresolvedContactProbe
from luggage_planning.joint_operation.ik import NativeSnapshotIKPort
from luggage_planning.joint_operation.map_source import map_source_to_message
from luggage_planning.joint_operation.pickup_observation import assess_pickup_observation
from luggage_planning.joint_operation.view_certification import NativeViewCertificationPort
from luggage_planning.joint_operation.view_context import (
    IssuedPickupView, PickupViewContext, PickupViewContextStore,
)
from luggage_planning.joint_operation.view_motion import PickupViewMotionPreparation
from luggage_planning.joint_operation.view_proposals import PickupViewSampling,propose_pickup_views
from luggage_planning.placement_prior.contracts import Transform,finite_tuple
from luggage_planning.placement_prior.requests import admit_placement_targets
from luggage_planning.placement_prior.snapshot import build_private_snapshot,proposal_scene_hash


def pickup_view_plan_to_message(plan,task_request_id,source,model_sha256):
    from luggage_msgs.msg import JointOperationLegV2,PickupViewPlanV2
    leg=plan.leg
    context=BoundaryContext('pickup_view','joint_prepare','joint_executor',
        leg.trajectory_request_id,leg.trajectory_scene_revision,'','','robot_state')
    output=JointOperationLegV2(name=leg.name,phase=leg.phase,
        trajectory=joints_to_message(leg.trajectory_record(),context))
    return PickupViewPlanV2(request_id=plan.request_id,task_request_id=task_request_id,
        domain_sha256=plan.domain_sha256,view_sha256=plan.view_sha256,query_id=plan.query_id,
        model_sha256=model_sha256,map_source=map_source_to_message(source),leg=output,
        route_digest=plan.route_digest,certificate_id=plan.certificate_id)


class PickupViewPrepareOwner:
    def __init__(self,node,*,param,connections,velocity_scale,acceleration_scale,
                 velocity_limits,acceleration_limits,max_samples):
        from luggage_msgs.srv import GetPickupViewEvidenceV2,PreparePickupViewV2,CheckPreparedPickupViewV2
        self.node=node
        self.contexts=PickupViewContextStore(
            max_contexts=param('pickup_view_context_limit',16),
            max_bytes=param('pickup_view_context_byte_limit',16*1024*1024),
            lifetime_s=param('pickup_view_context_lifetime_s',60.))
        self.max_joint_age_ns=param('pickup_view_joint_max_age_ns',1_000_000_000)
        if type(self.max_joint_age_ns) is not int or self.max_joint_age_ns<=0:
            raise ValueError('POSITIVE_PICKUP_VIEW_JOINT_AGE_REQUIRED')
        self.evidence_service=GetPickupViewEvidenceV2
        self.evidence_client=node.create_client(GetPickupViewEvidenceV2,'/luggage/get_pickup_view_evidence_v2')
        certifier=NativeViewCertificationPort(node,node.model,velocity_limits=velocity_limits,
            acceleration_limits=acceleration_limits,max_samples=max_samples,
            recorder=lambda name,value:node._record('pickup_view_'+name,value))
        self.motion=PickupViewMotionPreparation(node.model,
            NativeSnapshotIKPort(node,node.model,recorder=lambda name,value:node._record('pickup_view_ik_'+name,value)),
            connections,certifier,velocity_scale=velocity_scale,acceleration_scale=acceleration_scale)
        self.service=node.create_service(PreparePickupViewV2,'/joint_operation/prepare_pickup_view',
            self.prepare,callback_group=ReentrantCallbackGroup())
        self.check_service=node.create_service(CheckPreparedPickupViewV2,'/joint_operation/check_pickup_view',
            self.check,callback_group=ReentrantCallbackGroup())

    def check(self,request,response):
        """Independent exact receipt and still-retained physical observation.

        No motion or renewed retention. Current full-scene/native/start checks
        remain the execution owner's responsibility.
        """
        from luggage_msgs.msg import PickupViewPlanV2
        from rclpy.serialization import deserialize_message
        from luggage_perception.frozen_suction_service import binding_to_message
        from luggage_perception.ros_suction_domain import domain_to_message
        response.current=False
        if not self.node._active.acquire(blocking=False):
            response.reason='OPERATION_PREPARATION_BUSY';return response
        try:
            if not self.node.budget.admits(request.budget_s):
                raise ValueError('PICKUP_VIEW_CHECK_BUDGET_INVALID')
            deadline=time.monotonic()+request.budget_s
            message=request.plan
            context=self.contexts.lookup(message.task_request_id,message.domain_sha256)
            receipt=self.contexts.issued(message.task_request_id,message.route_digest)
            if (context is None or receipt is None
                    or deserialize_message(receipt.message_cdr,PickupViewPlanV2)!=message):
                raise ValueError('EXACT_RETAINED_VIEW_REQUIRED')
            original=context.request()
            retained=self.node.observed_contacts.retain(context.domain,original.detection,
                request_id=original.request_id,observation_id=original.observation_id,
                expected_observation=context.observation,deadline=deadline,cancelled=lambda:False)
            if (not retained.retained or retained.observation.binding!=context.observation.binding
                    or self._acquisition(context,deadline).digest()!=receipt.view.digest()):
                raise ValueError('PICKUP_VIEW_OBSERVATION_REVOKED')
            response.detection=original.detection
            response.top_domain=domain_to_message(context.domain)
            response.frozen_observation=binding_to_message(context.observation.binding)
            response.probe=context.probe(message.query_id)
            response.checked_scene_cdr=list(receipt.snapshot.scene_cdr)
            response.baseline_scene_sha256=receipt.baseline_sha256
            if (self.contexts.lookup(message.task_request_id,message.domain_sha256) is not context
                    or self.contexts.issued(message.task_request_id,message.route_digest) is not receipt):
                raise ValueError('PICKUP_VIEW_CONTEXT_CHANGED_OR_EXPIRED')
            response.current=True;response.reason='RETAINED_PICKUP_VIEW_CURRENT'
        except Exception as error:
            response.reason='PICKUP_VIEW_CHECK_REFUSED:'+str(error)
        finally:self.node._active.release()
        return response

    def remember(self,request,response,domain,observation):
        """Called only after the ordinary prepare response has been captured."""
        if (self.node.observer is None or response.success or response.reason!='SEARCH_INCOMPLETE'
                or not response.unresolved_contacts):return
        try:
            context=PickupViewContext.capture(request,domain,observation,response.unresolved_contacts)
            self.contexts.remember(context)
            self.node._record('pickup_view_context_retained.json',json.dumps(dict(
                task_request_id=request.request_id,domain_sha256=domain.digest(),
                query_ids=[key for key,_ in context.probes_cdr])).encode())
        except Exception as error:
            self.contexts.forget(request.request_id)
            # This optional facility must not change the ordinary plan outcome.
            self.node.get_logger().warning('PICKUP_VIEW_CONTEXT_UNAVAILABLE:'+str(error))

    def _acquisition(self,context,deadline):
        original=context.request()
        request=self.evidence_service.Request(domain_sha256=context.domain.digest(),
            detection=original.detection,generation=context.domain.generation)
        result=self.node._call(self.evidence_client,request,deadline,'pickup_view_acquisition')
        if not result.success:raise ValueError('PICKUP_VIEW_ACQUISITION_UNAVAILABLE:'+result.reason)
        return view_from_message(result.view,context.domain)

    def _current(self,context,request,deadline):
        original=context.request()
        original.robot_state_stamp_valid=request.robot_state_stamp_valid
        original.robot_state_stamp_ns=request.robot_state_stamp_ns
        original.start_joints=request.start_joints
        self.node._current(original,frozen_domain=context.domain)
        now=self.node.get_clock().now().nanoseconds
        with self.node._state_lock:joints=self.node.joints
        if (not request.robot_state_stamp_valid or request.robot_state_stamp_ns>now
                or joints is None or not 0<=now-joints[0]<=self.max_joint_age_ns):
            raise ValueError('PICKUP_VIEW_START_AGE_REFUSED')
        checked=self.node.observed_contacts.retain(context.domain,original.detection,
            request_id=original.request_id,observation_id=original.observation_id,
            expected_observation=context.observation,deadline=deadline,cancelled=lambda:False)
        if not checked.retained or checked.observation.binding!=context.observation.binding:
            raise ValueError('PICKUP_VIEW_OBSERVATION_REVOKED')
        if self.contexts.lookup(context.task_request_id,context.domain.digest()) is not context:
            raise ValueError('PICKUP_VIEW_CONTEXT_CHANGED_OR_EXPIRED')

    def prepare(self,request,response):
        from luggage_msgs.msg import PickupViewPlanV2
        start=time.monotonic();trace=[]
        response.success,response.reason,response.plan=False,'PICKUP_VIEW_REFUSED',PickupViewPlanV2()
        response.trace_json,response.elapsed_s='[]',0.
        node=self.node
        if not node._active.acquire(blocking=False):
            response.reason='OPERATION_PREPARATION_BUSY';return response
        context=None
        try:
            node._record('pickup_view_prepare_request',request)
            if node.observer is None:raise ValueError('PICKUP_VIEW_REQUIRES_O_BACKEND')
            if (not node.budget.admits(request.budget_s) or not request.request_id.strip()
                    or not 1<=request.max_ik_seeds<=64):
                raise ValueError('PICKUP_VIEW_REQUEST_LIMITS_INVALID')
            finite_tuple(request.start_joints,6,'pickup view start')
            deadline=start+request.budget_s
            sampling=PickupViewSampling(tuple(request.tilts_rad),tuple(request.azimuths_rad),
                tuple(request.distance_scales),request.min_optical_z_m,request.max_optical_z_m,
                int(request.max_view_proposals))
            trace.append(('explicit_search',asdict(sampling),request.max_ik_seeds,request.budget_s))
            context=self.contexts.lookup(request.task_request_id,request.domain_sha256)
            probe=None if context is None else context.probe(request.query_id)
            if probe is None:raise ValueError('OWNED_UNRESOLVED_CONTACT_UNAVAILABLE')
            original=context.request();domain=context.domain
            self._current(context,request,deadline)
            view=self._acquisition(context,deadline)
            hypothesis=UnresolvedContactProbe(probe.query_id,probe.domain_sha256,probe.contact_model_hash,
                tuple(probe.center_uv_top_m),probe.yaw_rad,tuple(probe.footprint_uv_top_m),probe.unproven_cell_count)
            assessment=assess_pickup_observation(domain,node.preparation.model,hypothesis,view)
            trace.append(('observation_assessment',asdict(assessment)))
            if assessment.need is None:raise ValueError('NO_DECISION_RELEVANT_VIEW_GAP:'+assessment.reason)
            proposals=propose_pickup_views(domain,view,assessment.need,sampling)
            trace.append(('camera_proposals',[asdict(proposal) for proposal in proposals]))
            if not proposals:raise ValueError('NO_USEFUL_CAMERA_PROPOSAL')
            surface,volume,source=node._capture_map(original,deadline,latest=True)
            if (source is None or source.geometry_hash!=original.geometry_hash
                    or source.scene_revision!=node.revision or source.epoch!=original.expected_map_epoch):
                raise ValueError('PICKUP_VIEW_O_SOURCE_MISMATCH')
            captured=ObserverSnapshot(source,surface,volume)
            pose=pose_from_message(probe.placement,'slot',BoundaryContext(
                'PICKUP_VIEW_TARGET','prepare_owner','joint_operation',original.request_id,
                node.revision,'container_link','box_center','scene'))
            target=Transform(pose['position'],pose['quaternion_xyzw'])
            if admit_placement_targets(surface,domain.box_wdh,(target,),volume=volume)!=(True,):
                raise ValueError('ORIGINAL_PLACEMENT_NO_LONGER_ADMITTED')
            baseline=node._scene(deadline);baseline_hash=proposal_scene_hash(baseline)
            snapshot=build_private_snapshot(surface,baseline,node.model.model_sha256,node.container,
                collision_margin_m=node.margin,volume=volume,vertical_collision_margin_m=node.vertical_margin)
            def current(candidate):
                if candidate.identity!=snapshot.identity:return False
                scene=node._scene(deadline)
                self._current(context,request,deadline)
                fresh=self._acquisition(context,deadline)
                return (proposal_scene_hash(scene)==baseline_hash and fresh.digest()==view.digest()
                    and node.observer.planning_current(captured,deadline=deadline))
            seeds=tuple(dict.fromkeys((tuple(request.start_joints),tuple(original.start_joints))))[:request.max_ik_seeds]
            for index,proposal in enumerate(proposals):
                if time.monotonic()>=deadline:raise TimeoutError('PICKUP_VIEW_DEADLINE')
                result=self.motion.prepare(snapshot,domain,view,proposal,request.start_joints,
                    request_id=request.request_id,robot_state_stamp_ns=int(request.robot_state_stamp_ns),
                    pickup_object_id=node.pickup_id,ik_seeds=seeds,max_ik_seeds=int(request.max_ik_seeds),
                    admit_static_scene=node.static_world.admit,source_current=current,deadline=deadline)
                trace.append(('view_motion',index,result.reason,result.trace))
                if result.plan is None:continue
                output=pickup_view_plan_to_message(result.plan,original.request_id,source,node.model.model_sha256)
                derived=replace(snapshot,scene_cdr=result.plan.view_scene_cdr,
                    scene_content_hash='',proposal_scene_hash='')
                node._record('pickup_view_checked_scene',derived.scene_cdr)
                if derived.identity!=result.plan.view_snapshot_identity:
                    raise ValueError('PICKUP_VIEW_DERIVED_SCENE_CHANGED')
                receipt=IssuedPickupView(result.plan,source,derived,view,
                    bytes(serialize_message(output)),baseline_hash)
                # Capture and issue the exact proposed response before publication.
                response.plan=output;response.success=True;response.reason='PICKUP_VIEW_PREPARED'
                response.trace_json=json.dumps(trace,separators=(',',':'),allow_nan=False)
                response.elapsed_s=time.monotonic()-start
                node._record('pickup_view_prepare_response',response)
                if not current(snapshot):raise ValueError('PICKUP_VIEW_FINAL_SOURCE_CHANGED')
                self.contexts.issue(context,receipt)
                return response
            response.reason='PICKUP_VIEW_SEARCH_INCOMPLETE'
        except Exception as error:
            response.success=False;response.plan=PickupViewPlanV2()
            response.reason='PICKUP_VIEW_REFUSED:'+type(error).__name__+':'+str(error)
        finally:
            response.trace_json=json.dumps(trace,separators=(',',':'),allow_nan=False)
            response.elapsed_s=time.monotonic()-start
            try:
                if not response.success:node._record('pickup_view_prepare_response',response)
            except Exception as error:
                response.success=False;response.plan=PickupViewPlanV2()
                response.reason='PICKUP_VIEW_EVIDENCE_FAILED:'+type(error).__name__
            finally:node._active.release()
        return response
