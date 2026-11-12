"""Actual O, perception, MoveIt scene, vacuum and native ports for one operation.

Only owned payload scene diffs are written. O remains the observed-map owner;
this module has no O write, global planning or controller API.
"""
from dataclasses import replace
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time

from luggage_planning.attachment_provider import MeasuredAttachmentProvider
from luggage_planning.joint_operation.actual_contact import FrozenActualContactPort, RetainedContactObservation
from luggage_planning.joint_operation.attachment import StationaryOperationAttachmentPort
from luggage_planning.joint_operation.certification import EndpointModel
from luggage_planning.joint_operation.contracts import OperationIdentity, OPERATION_JOINTS
from luggage_planning.joint_operation.adapters import operation_plan_from_message
from luggage_planning.joint_operation.map_source import map_source_from_message, OperationMapRefresh
from luggage_planning.joint_operation.observer import ObserverConfiguration, ObserverSnapshotPort
from luggage_planning.joint_operation.revalidation import NativeOperationRevalidationPort
from luggage_planning.joint_operation.scenes import build_operation_scenes, _reset_payload_contacts
from luggage_planning.joint_operation.motion import MeasuredJointSample
from luggage_planning.placement_prior.contracts import Transform
from luggage_planning.placement_prior.snapshot import build_private_snapshot, proposal_scene_hash
from luggage_planning.placement_prior.requests import admit_placement_targets
from luggage_planning.trajectory_timing import cartesian_joint_limits
from luggage_planning.waypoint_generator import PLACE_RELEASE_GAP_M


class OperationRpc:
    """Single-flight RPC ownership extends through late-result cleanup."""
    def __init__(self, recorder):
        self.record, self.pending, self.consumed = recorder, {}, {}
        self.cancelled = lambda:False
        self.transport_uncertain = set()

    def unresolved(self):
        return bool(self.transport_uncertain) or any(not event.is_set() for event in self.consumed.values())

    def call(self, client, request, deadline, name, *, late_response=None):
        if self.cancelled() or time.monotonic() >= deadline:
            raise TimeoutError('OPERATION_CANCELLED_OR_DEADLINE')
        if client in self.transport_uncertain or (client in self.consumed and not self.consumed[client].is_set()):
            raise RuntimeError(name+'_BACKEND_BUSY')
        if not client.service_is_ready():raise RuntimeError(name+'_BACKEND_UNAVAILABLE')
        self.record(name+'_request', request)
        self.transport_uncertain.add(client)
        future = self.pending[client] = client.call_async(request)
        event, consumed, lock = threading.Event(), threading.Event(), threading.Lock()
        self.consumed[client] = consumed
        state = dict(abandoned=False, cleaned=False)
        def cleanup():
            with lock:
                if not state['abandoned'] or state['cleaned'] or not future.done():return
                state['cleaned'] = True
            try:
                response = future.result()
                self.record(name+'_late_response', response)
                if late_response is not None:late_response(response)
                self.transport_uncertain.discard(client)
            except Exception as error:
                self.record(name+'_cleanup_fault.json',json.dumps(dict(reason=str(error))).encode())
            finally:consumed.set()
        def completed(_):
            event.set()
            cleanup()
        future.add_done_callback(completed)
        returned = False
        try:
            while not event.wait(min(.01, max(0.,deadline-time.monotonic()))):
                if self.cancelled() or time.monotonic() >= deadline:
                    raise TimeoutError('OPERATION_CANCELLED_OR_DEADLINE')
            result = future.result()
            self.record(name+'_response',result)
            if self.cancelled() or time.monotonic() >= deadline:
                raise TimeoutError('OPERATION_CANCELLED_OR_DEADLINE')
            returned = True
            self.transport_uncertain.discard(client)
            return result
        finally:
            if returned:consumed.set()
            else:
                with lock:state['abandoned'] = True
                cleanup()


class OperationRuntimePorts:
    def __init__(self, node, *, record):
        from luggage_msgs.srv import CheckPreparedOperationV2, VacuumCommandV2
        from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
        self.node, self.record = node, record
        def param(name, default):
            return (node.get_parameter(name) if node.has_parameter(name)
                    else node.declare_parameter(name,default)).value
        self.revision = str(param('frame_scene_revision',''))
        self.model = EndpointModel(param('robot_description',''),param('robot_description_semantic',''),
            position_tolerance_m=param('endpoint_position_tolerance_m',.002),
            orientation_tolerance_rad=param('endpoint_orientation_tolerance_rad',.02))
        if not self.revision:raise ValueError('EXPLICIT_OPERATION_SCENE_REVISION_REQUIRED')
        self.container = Transform(param('container_translation',[float('nan')]*3),
                                   param('container_quaternion_xyzw',[float('nan')]*4))
        self.margin = float(param('collision_margin_m',.05))
        self.vertical_margin = float(param('vertical_collision_margin_m',self.margin))
        from luggage_planning.joint_operation.preparation_profile import (
            DEFAULT_MAX_PREPARATION_BUDGET_S, execution_observation_age_limit_ns)
        self.contact_admission_max_age_ns = execution_observation_age_limit_ns(
            float(param('max_preparation_budget_s',DEFAULT_MAX_PREPARATION_BUDGET_S)))
        self.start_tolerance = float(param('start_tolerance_rad',.01))
        self.slot_limit = int(param('geometric_slot_limit',256))
        self.pickup_id = str(param('pickup_object_id','pickup_box'))
        if (not math.isfinite(self.margin) or self.margin<0
                or not math.isfinite(self.vertical_margin) or self.vertical_margin<0
                or not 1<=self.slot_limit<=2048):
            raise ValueError('BOUNDED_OPERATION_GEOMETRY_POLICY_REQUIRED')
        self.rpc = OperationRpc(record)
        config = ObserverConfiguration.load(param('observer_geometry_file',''),param('observer_config_file',''))
        self.observer = ObserverSnapshotPort(node, configuration=config, call=self.rpc.call,
            prefix=param('observer_prefix','/uld_occ_observer'),
            max_age_ms=int(param('observer_max_age_ms',1000)),recorder=record)
        self.check_type, self.vacuum_type = CheckPreparedOperationV2, VacuumCommandV2
        self.check_client = node.create_client(CheckPreparedOperationV2,'/joint_operation/check_prepared')
        self.vacuum_client = node.create_client(VacuumCommandV2,'/vacuum/command_v2')
        self.scene_type, self.apply_type = GetPlanningScene, ApplyPlanningScene
        self.scene_client = node.create_client(GetPlanningScene,'/get_planning_scene')
        self.apply_client = node.create_client(ApplyPlanningScene,'/apply_planning_scene')
        self.contacts = FrozenActualContactPort(node,recorder=record)
        vmax,amax = cartesian_joint_limits(float(param('velocity_scale',.2)),float(param('acceleration_scale',.1)))
        self.native = NativeOperationRevalidationPort(node,self.model,velocity_limits=(vmax,)*6,
            acceleration_limits=(amax,)*6,release_drop_m=PLACE_RELEASE_GAP_M,
            start_tolerance_rad=self.start_tolerance,recorder=record)
        self.deadline, self.cancelled = 0., lambda:False
        self.plan = self.message = self.observation = self.original_source = None
        self.geometry = self.last_map = self.validated = self._last_scene = None
        self.owned_payload = False
        self.released = False
        self.finalized = False
        self.retryable_change = False

    def joints(self):
        # The host stores original sensor timestamps and canonical raw winding.
        with self.node._joint_lock:message = copy.deepcopy(self.node._joint_state)
        if message is None:raise ValueError('OPERATION_JOINTS_UNAVAILABLE')
        if len(message.name)!=len(message.position) or len(set(message.name))!=len(message.name):
            raise ValueError('OPERATION_JOINTS_MALFORMED')
        positions = dict(zip(message.name,message.position))
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        return MeasuredJointSample(tuple(positions[name] for name in OPERATION_JOINTS),stamp,stamp>0)

    def check_issued(self, plan):
        result = self.rpc.call(self.check_client,self.check_type.Request(plan=self.message),
                               self.deadline,'check_prepared')
        return (result.current is True and map_source_from_message(result.map_source)==self.original_source
                and result.frozen_observation.binding_sha256==self.observation.binding.digest())

    def open(self, goal, *, deadline, cancelled):
        from luggage_perception.ros_suction_domain import domain_from_message, require_matching_detection
        from luggage_perception.frozen_suction_service import binding_from_message
        self.deadline,self.cancelled = deadline,cancelled
        self.rpc.cancelled = cancelled
        self.message = copy.deepcopy(goal.plan)
        checked = self.rpc.call(self.check_client,self.check_type.Request(plan=self.message),deadline,'check_prepared')
        if checked.current is not True:raise ValueError('PREPARED_OWNER_REFUSED:'+checked.reason)
        identity = OperationIdentity(**{name:getattr(self.message,name) for name in OperationIdentity.__dataclass_fields__})
        self.plan = operation_plan_from_message(self.message,identity,self.message.plan_digest)
        if identity.scene_revision!=self.revision or identity.model_sha256!=self.model.model_sha256:
            raise ValueError('OPERATION_RUNTIME_MODEL_OR_SCENE_MISMATCH')
        domain = domain_from_message(goal.top_domain)
        require_matching_detection(domain,goal.detection,generation=identity.generation,
                                   contact_model_hash=identity.contact_model_hash)
        self.detection = copy.deepcopy(goal.detection)
        self.observation = RetainedContactObservation(binding_from_message(checked.frozen_observation),domain)
        self.original_source = map_source_from_message(checked.map_source)
        # The user-confirmed operation premise keeps all other cargo/static
        # geometry stationary while this one box moves. A wrist sensor can
        # temporarily look away from ULD. Bound retained map age by this action,
        # never restamp the acquisition or bypass content/epoch revalidation.
        now_ns = self.node.get_clock().now().nanoseconds
        map_stamp = (self.original_source.last_integrated_stamp_ns
                     or self.original_source.initialization_stamp_ns)
        if not 0 < map_stamp <= now_ns:
            raise ValueError('STATIONARY_MAP_ORIGINAL_STAMP_REQUIRED')
        initial_age_ns = now_ns - map_stamp
        retained_age_ns = initial_age_ns + math.ceil(goal.budget_s*1e9)
        self.observer.max_age_ms = math.ceil(retained_age_ns/1e6)
        self.record('stationary_map_scope.json',json.dumps(dict(
            premise='other_cargo_and_environment_stationary_during_one_operation',
            original_acquisition_or_initialization_stamp_ns=map_stamp,
            scope_open_stamp_ns=now_ns,max_retained_age_ms=self.observer.max_age_ms,
            wall_deadline=deadline,content_change='full_native_suffix_revalidation',
            physical_collision_detection_available=False),sort_keys=True).encode())
        # Preparation admitted a recent stationary observation, then consumed
        # its bounded watchdog. New-action age and journal replay retention use
        # that SAME composed window, measured from the original acquisition.
        self.provider = MeasuredAttachmentProvider(self.revision,identity.model_sha256,
            max_age_ns=self.contact_admission_max_age_ns)
        if not self.observation_current():raise ValueError('ORIGINAL_OPERATION_OBSERVATION_REFUSED')
        observation_age_ns = self.node.get_clock().now().nanoseconds-domain.stamp_ns
        retained_observation_age_ns = observation_age_ns + math.ceil(goal.budget_s*1e9)
        self.provider = MeasuredAttachmentProvider(self.revision,identity.model_sha256,
            max_age_ns=retained_observation_age_ns)
        self.record('stationary_contact_scope.json',json.dumps(dict(
            original_acquisition_stamp_ns=domain.stamp_ns,
            initial_admission_age_ns=observation_age_ns,
            initial_admission_max_age_ns=self.contact_admission_max_age_ns,
            max_retained_age_ns=retained_observation_age_ns,wall_deadline=deadline,
            premise='stationary_until_seal_then_fixed_actual_grasp',
            renews_acquisition=False),sort_keys=True).encode())
        self.attachment = StationaryOperationAttachmentPort(self.provider,self.contacts,
            certificate_current=self.check_issued,now_ns=lambda:self.node.get_clock().now().nanoseconds,
            recorder=self.record)
        self.geometry = self.last_map = self.validated = self._last_scene = None
        self.owned_payload,self.released,self.finalized = False,False,False
        self.backup_owned_objects = []
        if not self.observation_current():raise ValueError('ORIGINAL_OPERATION_OBSERVATION_REFUSED')
        self.capture()
        from luggage_planning.joint_operation.candidates import admit_pickup_association
        admit_pickup_association(self.snapshot,self.observation.domain,self.pickup_id)
        self.backup_owned_objects = copy.deepcopy([obj for obj in self._last_scene.world.collision_objects
                                                  if obj.id==self.pickup_id])
        self.apply_phase('pick',self.plan.predicted_attachment)
        return self.plan

    def observation_current(self):
        domain = self.observation.domain
        age = self.node.get_clock().now().nanoseconds-domain.stamp_ns
        if not 0<=age<=self.provider.max_age_ns:
            self.record('contact_age_refused.json',json.dumps(dict(
                original_stamp_ns=domain.stamp_ns,age_ns=age,
                max_age_ns=self.provider.max_age_ns),sort_keys=True).encode())
            return False
        retained = self.contacts.retain(domain,self.detection,request_id=self.plan.identity.request_id,
            observation_id=self.plan.identity.observation_id,expected_observation=self.observation,
            deadline=self.deadline,cancelled=self.cancelled)
        return retained.retained is True

    def scene(self):
        from moveit_msgs.msg import PlanningSceneComponents as C
        request = self.scene_type.Request()
        request.components.components = (C.SCENE_SETTINGS|C.ROBOT_STATE|C.ROBOT_STATE_ATTACHED_OBJECTS|
            C.WORLD_OBJECT_NAMES|C.WORLD_OBJECT_GEOMETRY|C.OCTOMAP|C.TRANSFORMS|
            C.ALLOWED_COLLISION_MATRIX|C.LINK_PADDING_AND_SCALING)
        scene = self.rpc.call(self.scene_client,request,self.deadline,'current_scene').scene
        if scene.is_diff or scene.robot_state.is_diff:raise ValueError('FULL_OPERATION_SCENE_REQUIRED')
        return scene

    def capture(self):
        current = self.observer.capture(scene_revision=self.revision,request_id=self.plan.identity.request_id,
            expected_epoch=self.original_source.epoch,expected_revision=None,
            deadline=self.deadline,require_latest=False)
        baseline = self.scene()
        snapshot = build_private_snapshot(current.surface,baseline,self.model.model_sha256,self.container,
                                          collision_margin_m=self.margin,volume=current.volume,
                                          vertical_collision_margin_m=self.vertical_margin)
        refresh = OperationMapRefresh(self.original_source,current.source,
                                      hashlib.sha256(current.surface.record_json).hexdigest())
        if not refresh.admits(self.plan.identity,snapshot):raise ValueError('OPERATION_MAP_SOURCE_RESET_OR_REBOUND')
        self.last_map,self.snapshot,self.refresh,self._last_scene = current,snapshot,refresh,baseline
        return current

    @staticmethod
    def same_collision_map(a,b):
        # Full classification and measured surface content, never revision alone.
        if (a.volume.resolution_m,a.volume.origin_container,a.volume.dimensions_xyz,a.volume.states,a.volume.active)!=\
                (b.volume.resolution_m,b.volume.origin_container,b.volume.dimensions_xyz,b.volume.states,b.volume.active):
            return False
        keys = ('floor_z_container','ceiling_z_container','resolution_m','origin_xy_container',
                'state','surface_z_container')
        left,right = a.surface.record(),b.surface.record()
        return all(left[k]==right[k] for k in keys)

    def source_current(self, snapshot, geometry):
        self.retryable_change=False
        if not self.check_issued(self.plan) or not self.observation_current():return False
        if geometry is not None and not self.released:
            self.provider.expire_stale(self.plan.identity.request_id,self.node.get_clock().now().nanoseconds)
            if self.provider.latest_record(self.plan.identity.request_id).evidence_digest!=self.actual.evidence.digest():return False
        checked = self.last_map
        current = self.observer.capture(scene_revision=self.revision,request_id=self.plan.identity.request_id,
            expected_epoch=self.original_source.epoch,expected_revision=None,
            deadline=self.deadline,require_latest=False)
        refresh = OperationMapRefresh(self.original_source,current.source,
                                      hashlib.sha256(current.surface.record_json).hexdigest())
        probe = replace(snapshot,map_revision=current.source.map_revision,
                        surface_content_hash=refresh.surface_record_sha256)
        source_admitted=refresh.admits(self.plan.identity,probe)
        map_equal=self.same_collision_map(checked,current)
        world_equal=proposal_scene_hash(self.scene())==proposal_scene_hash(self._last_scene)
        self.retryable_change=source_admitted and (not map_equal or not world_equal)
        self.record('current_source_verdict.json',json.dumps(dict(source_admitted=source_admitted,
            map_equal=map_equal,world_equal=world_equal,
            checked_revision=checked.source.map_revision,current_revision=current.source.map_revision,
            checked_volume_sha256=hashlib.sha256(checked.volume.states).hexdigest(),
            current_volume_sha256=hashlib.sha256(current.volume.states).hexdigest(),
            checked_occupied=checked.volume.states.count(2),current_occupied=current.volume.states.count(2)),sort_keys=True).encode())
        return source_admitted and map_equal and world_equal

    def settled_admission(self,snapshot,geometry,settled,**kwargs):
        if self.released:
            # Release follows the successful pre-release check. Native retreat
            # still includes both current O obstacles and the released world box.
            return self.released_target==settled
        target = self.container.inverse().compose(settled)
        return admit_placement_targets(self.last_map.surface,geometry.measured_dimensions_wdh,
            (target,),volume=self.last_map.volume)[0]

    def validate(self,index):
        self.retryable_change=False
        self.capture()
        geometry = None if index<2 else self.geometry
        def actual_contact(*args,**kwargs):
            result = self.contacts.check(self.observation,identity=self.plan.identity,geometry=geometry,
                deadline=self.deadline,cancelled=self.cancelled)
            return result.checked is True and result.accepted is True
        self.validated = self.native.validate(self.plan,expected_identity=self.plan.identity,
            expected_digest=self.plan.plan_digest,snapshot=self.snapshot,pickup_object_id=self.pickup_id,
            next_leg_index=index,actual_joints=self.joints().positions,geometry=geometry,
            certificate_current=self.check_issued,source_current=self.source_current,
            admit_actual_contact=actual_contact,admit_settled_box=self.settled_admission,
            deadline=self.deadline,cancelled=self.cancelled,map_refresh=self.refresh)
        self.record('native_suffix_verdict.json',json.dumps(dict(index=index,
            accepted=self.validated.receipt.accepted,reason=self.validated.receipt.reason,
            native_validation_id=self.validated.native_validation_id,
            scene_sha256=self.validated.current_scene_sha256,
            map_revision=self.last_map.source.map_revision),sort_keys=True).encode())
        return self.validated

    def apply_phase(self,phase,geometry):
        from moveit_msgs.msg import PlanningScene
        from rclpy.serialization import serialize_message,deserialize_message
        from luggage_planning.joint_operation.scenes import owned_payload_diff
        from luggage_planning.joint_operation.revalidation import settled_box_from_release
        live = self.scene()
        if any(item.object.id!=self.pickup_id for item in live.robot_state.attached_collision_objects):
            raise ValueError('UNEXPECTED_EXTERNAL_ATTACHMENT')
        baseline = copy.deepcopy(live)
        baseline.robot_state.attached_collision_objects=[]
        snapshot = replace(self.snapshot,scene_cdr=serialize_message(baseline))
        settled = (settled_box_from_release(self.model,self.plan,geometry,self.container,PLACE_RELEASE_GAP_M)
                   if phase=='retreat' else self.plan.world_from_settled_box)
        phases = build_operation_scenes(snapshot,pickup_object_id=self.pickup_id,geometry=geometry,
            world_from_pick_box=self.plan.world_from_pick_box,
            world_from_settled_box=settled)
        desired = deserialize_message(phases.for_phase(phase).scene_cdr,PlanningScene)
        diff = owned_payload_diff(live,desired,self.pickup_id)
        result = self.rpc.call(self.apply_client,self.apply_type.Request(scene=diff),self.deadline,'owned_payload_scene')
        if result.success is not True:raise ValueError('OWNED_PAYLOAD_SCENE_REFUSED')
        self.owned_payload=True
        self.last_owned_world = copy.deepcopy([obj for obj in desired.world.collision_objects if obj.id==self.pickup_id])

    def vacuum_request(self,enable):
        from luggage_planning.place_service import pose_message
        request = self.vacuum_type.Request(enable=enable,request_id=self.plan.identity.request_id,
            scene_revision=self.revision,box_id=self.plan.identity.box_id,generation=self.plan.identity.generation)
        if enable:
            request.measured_box=copy.deepcopy(self.detection)
            box = self.plan.world_from_pick_box
            request.box_pose=pose_message(dict(contract_version=2,kind='pose',frame_id='world',length_unit='m',
                stamp_ns=self.plan.identity.observation_stamp_ns,time_basis='acquisition',
                scene_revision=self.revision,request_id=self.plan.identity.request_id,
                reference='box_center',orientation_semantics='box_axis_pi',
                position=box.translation,quaternion_xyzw=box.quaternion_xyzw))
        return request

    def attach(self):
        request = self.vacuum_request(True)
        result = self.rpc.call(self.vacuum_client,request,self.deadline,'confirm_attachment')
        self.actual = self.attachment.qualify(self.plan,self.observation,self.detection,request,result,
                                            deadline=self.deadline,cancelled=self.cancelled)
        self.geometry=self.actual.geometry
        self.apply_phase('carry',self.geometry)
        return self.actual

    def release(self):
        request = self.vacuum_request(False)
        result = self.rpc.call(self.vacuum_client,request,self.deadline,'confirm_release')
        if result.success is not True:return False
        self.release_stamp_ns=self.node.get_clock().now().nanoseconds
        self.released_target=self.plan.world_from_settled_box
        self.released=True
        return True


    def finalize_world(self):
        """Retain released cargo under a unique ID, or restore untouched pickup.

        This is a collision-world bookkeeping transaction, never an O map write
        or an observed-placement claim. No attached load can enter this path.
        """
        if not self.owned_payload or self.finalized:return
        from moveit_msgs.msg import PlanningScene, CollisionObject
        live = self.scene()
        if live.robot_state.attached_collision_objects:
            raise ValueError('CANNOT_FINALIZE_ATTACHED_PAYLOAD')
        owned = [obj for obj in live.world.collision_objects if obj.id==self.pickup_id]
        from luggage_planning.joint_operation.scenes import same_payload_object
        if (len(owned)!=1 or len(self.last_owned_world)!=1
                or not same_payload_object(owned[0],self.last_owned_world[0])):
            raise ValueError('OWNED_WORLD_OBJECT_CHANGED')
        diff = PlanningScene(is_diff=True)
        diff.robot_state.is_diff=True
        diff.world.collision_objects=[CollisionObject(id=self.pickup_id,operation=CollisionObject.REMOVE)]
        if self.released:
            identifier='joint-operation-placed:'+self.plan.plan_digest
            if any(obj.id==identifier for obj in live.world.collision_objects):
                raise ValueError('RELEASED_WORLD_ID_ALREADY_OWNED')
            retained=copy.deepcopy(owned)
            for obj in retained:obj.id=identifier
            diff.world.collision_objects.extend(retained)
        else:diff.world.collision_objects.extend(self.backup_owned_objects)
        if not self.rpc.call(self.apply_client,self.apply_type.Request(scene=diff),
                             self.deadline,'finalize_owned_world').success:
            raise ValueError('FINALIZE_OWNED_WORLD_REFUSED')
        self.finalized=True

    def observed_target(self, captured):
        """Require actually occupied top columns at the fixed released target.

        All raster centres lying inside the physical top face must be observed
        occupied. Height tolerance is exactly one measured voxel (quantization).
        No centres => not evaluated, never a successful placement observation.
        """
        record=captured.surface.record()
        target=self.container.inverse().compose(self.plan.world_from_settled_box)
        from luggage_description.frame_transforms import rotate_vector
        normal=rotate_vector(target.quaternion_xyzw,(0.,0.,1.))
        if max(abs(a-b) for a,b in zip(normal,(0.,0.,1.)))>1e-9:
            raise ValueError('OBSERVED_RELEASE_TARGET_MUST_BE_UPRIGHT_IN_CONTAINER')
        width,depth,height=self.geometry.measured_dimensions_wdh
        top=target.translation[2]+height/2
        inverse=target.inverse()
        resolution=record['resolution_m']
        count=0
        for ix,row in enumerate(record['state']):
            for iy,state in enumerate(row):
                point=(record['origin_xy_container'][0]+(ix+.5)*resolution,
                       record['origin_xy_container'][1]+(iy+.5)*resolution,top)
                local=inverse.compose(Transform(point,(0.,0.,0.,1.))).translation
                if abs(local[0])<=width/2 and abs(local[1])<=depth/2:
                    count+=1
                    observed=record['surface_z_container'][ix][iy]
                    if state!='occupied' or observed is None or abs(observed-top)>resolution+1e-9:
                        return False,count
        return count>0,count

    def adopt_observed_release(self,settled_view_stamp_ns):
        if not self.released:raise ValueError('POST_RELEASE_OBSERVATION_WITHOUT_RELEASE')
        last_count=0
        while time.monotonic()<self.deadline and not self.cancelled():
            captured=self.capture()
            if (captured.source.last_integrated_stamp_ns>=settled_view_stamp_ns
                    and captured.source.last_integrated_stamp_ns>self.release_stamp_ns):
                accepted,last_count=self.observed_target(captured)
                self.record('observed_release.json',json.dumps(dict(accepted=accepted,
                    sample_columns=last_count,settled_view_stamp_ns=settled_view_stamp_ns,
                    release_stamp_ns=self.release_stamp_ns,
                    actual_acquisition_stamp_ns=captured.source.last_integrated_stamp_ns,
                    source=captured.source.__dict__),sort_keys=True).encode())
                if accepted:return captured
            time.sleep(.01)
        reason='POST_RELEASE_OBSERVATION_NOT_EVALUATED' if last_count==0 else 'POST_RELEASE_TARGET_NOT_OBSERVED'
        raise ValueError(reason)
