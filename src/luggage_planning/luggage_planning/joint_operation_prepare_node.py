"""Independent plan-only ROS entry; owned observations, read-only native ports."""
import hashlib
import json
import math
from pathlib import Path
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from luggage_msgs.msg import SuctionTopDomainV2, SurfaceContractV2
from luggage_msgs.srv import CheckPreparedOperationV2, GetCargoVolumeV2, PrepareJointOperationV2

from luggage_description.frame_diagnostics import BoundaryContext
from luggage_description.ros_frame_bindings.poses import pose_from_message
from luggage_description.ros_frame_bindings.surfaces import surface_from_message
from luggage_description.suction_contact_model import load_suction_contact_model
from luggage_perception.ros_suction_domain import domain_from_message, require_matching_detection
from luggage_planning.attachment_geometry_policy import load_attachment_geometry_policy
from luggage_planning.joint_operation_certification import EndpointModel, NativeOperationCertificationPort
from luggage_planning.joint_operation_connection import NativeSnapshotConnectionPort
from luggage_planning.joint_operation_contacts import DirectedContactPort
from luggage_planning.joint_operation_actual_contact import FrozenActualContactPort
from luggage_perception.frozen_suction_service import binding_to_message
from luggage_planning.joint_operation_contracts import OPERATION_JOINTS, OperationIdentity
from luggage_planning.joint_operation_adapters import operation_plan_from_message
from luggage_planning.joint_operation_issuance import IssuedOperationRegistry
from luggage_planning.joint_operation_map_source import map_source_to_message
from luggage_planning.joint_operation_observer import ObserverConfiguration, ObserverSnapshotPort
from luggage_planning.joint_operation_endpoints import NativeOperationEndpointPort
from luggage_planning.joint_operation_ik import NativeSnapshotIKPort
from luggage_planning.joint_operation_atlas import AtlasOperationHints, load_operation_atlas
from luggage_planning.joint_operation_planner import JointOperationPlanner
from luggage_planning.joint_operation_preparation import JointOperationPreparation, PreparationInputs
from luggage_planning.preparation_refresh import LatestMapFinalizer, RefreshCapture
from luggage_planning.pickup_collision import pickup_collision_aabb
from luggage_planning.placement_prior_contracts import PlacementCandidate, Transform
from luggage_planning.placement_prior_requests import enumerate_placements
from luggage_planning.placement_prior_snapshot import build_private_snapshot, proposal_scene_hash
from luggage_planning.placement_prior_surface import ConfirmedSurfaceStore
from luggage_planning.planning_scene_client import BOX_OBJECT_ID
from luggage_planning.trajectory_timing import cartesian_joint_limits
from luggage_planning.preparation_profile import (
    DEFAULT_MAX_PREPARATION_BUDGET_S, PreparationBudget, StageTimings,
)
from luggage_planning.waypoint_generator import PLACE_RELEASE_GAP_M


class JointOperationPrepareNode(Node):
    def __init__(self, **kwargs):
        super().__init__('joint_operation_prepare',**kwargs)
        def param(name,default): return self.declare_parameter(name,default).value
        self.budget = PreparationBudget(float(param('max_preparation_budget_s',
                                                   DEFAULT_MAX_PREPARATION_BUDGET_S)))
        self.refresh_attempts = param('preparation_refresh_attempts',2)
        if type(self.refresh_attempts) is not int or not 1 <= self.refresh_attempts <= 8:
            raise ValueError('BOUNDED_REFRESH_ATTEMPTS_REQUIRED')
        self.revision = param('frame_scene_revision','')
        xml,semantic = param('robot_description',''),param('robot_description_semantic','')
        if not self.revision or not xml or not semantic:
            raise ValueError('explicit scene revision, world-rooted model and SRDF required')
        self.container = Transform(param('container_translation',[float('nan')]*3),
                                   param('container_quaternion_xyzw',[float('nan')]*4))
        self.model = EndpointModel(xml,semantic,position_tolerance_m=param('endpoint_position_tolerance_m',.002),
                                  orientation_tolerance_rad=param('endpoint_orientation_tolerance_rad',.02))
        contact_path = param('suction_contact_model_config','')
        if not contact_path:
            from ament_index_python.packages import get_package_share_directory
            contact_path = str(Path(get_package_share_directory('luggage_description'))/'config/suction_contact_model.yaml')
        contact_model = load_suction_contact_model(contact_path)
        self.limits = load_attachment_geometry_policy(param('attachment_geometry_config',''))
        self.margin = float(param('collision_margin_m',.05))
        self.start_tolerance = float(param('start_tolerance_rad',.01))
        self.slot_limit = int(param('geometric_slot_limit',256))
        if (not math.isfinite(self.margin) or self.margin < 0 or not math.isfinite(self.start_tolerance)
                or self.start_tolerance < 0 or not 1 <= self.slot_limit <= 2048):
            raise ValueError('finite nonnegative margins and bounded geometry pool required')
        self.pickup_id = str(param('pickup_object_id',BOX_OBJECT_ID))
        self.store,self.domain,self.joints = ConfirmedSurfaceStore(),None,None
        self._active,self._state_lock = threading.Lock(),threading.Lock()
        self.issued = IssuedOperationRegistry(
            max_plans=param('issued_plan_limit',16),
            max_bytes=param('issued_plan_byte_limit',16*1024*1024),
            lifetime_s=param('issued_plan_lifetime_s',300.))
        self._inflight = {}
        self._rpc_completions = {}
        self._evidence = str(param('evidence_dir',''))
        self._evidence_seq,self._evidence_bytes = 0,0
        self._evidence_lock = threading.Lock()
        if self._evidence: Path(self._evidence).mkdir(parents=True,exist_ok=False)
        def recorder(port): return lambda kind,value:self._record(port+'_'+kind,value)
        contacts = DirectedContactPort(self,recorder=recorder('quality'))
        self.observed_contacts = FrozenActualContactPort(self,recorder=recorder('observation'))
        endpoints = NativeOperationEndpointPort(self,recorder=recorder('local'))
        connections = NativeSnapshotConnectionPort(self,recorder=recorder('connection'))
        velocity,acceleration = float(param('velocity_scale',.2)),float(param('acceleration_scale',.1))
        vmax,amax = cartesian_joint_limits(velocity,acceleration)
        certifier = NativeOperationCertificationPort(self,self.model,velocity_limits=(vmax,)*6,
            acceleration_limits=(amax,)*6,release_drop_m=PLACE_RELEASE_GAP_M,recorder=recorder('certification'))
        planner = JointOperationPlanner(endpoints,connections,certifier,velocity_scale=velocity,
            acceleration_scale=acceleration,certification_reserve_s=float(param('certification_reserve_s',5.)),
            max_candidates=8,max_branch_attempts=3,max_branch_attempts_per_candidate=2)
        atlas,atlas_reason = None,'ATLAS_UNAVAILABLE_ONLINE_IK'
        atlas_paths = [str(param(name,'')) for name in
            ('atlas_npz','atlas_yaml','atlas_qualification_file','atlas_scene_tf_config')]
        if any(atlas_paths):
            try:
                if not all(atlas_paths): raise ValueError('all four explicit atlas paths required')
                reader = load_operation_atlas(*atlas_paths,robot_description=xml,
                    model_sha256=self.model.model_sha256,scene_revision=self.revision,
                    world_from_container=self.container)
                atlas = AtlasOperationHints(reader,NativeSnapshotIKPort(self,self.model,
                    recorder=recorder('atlas_ik')),connections,scene_revision=self.revision,
                    model_sha256=self.model.model_sha256,
                    max_refinement_s=float(param('atlas_refinement_budget_s',1.)),
                    fallback_time_fraction=float(param('atlas_fallback_time_fraction',.5)))
            except (OSError,TypeError,ValueError,KeyError) as error:
                atlas_reason = 'ATLAS_REFUSED_ONLINE_IK:'+type(error).__name__+':'+str(error)
        self.preparation = JointOperationPreparation(contacts,planner,contact_model,self.limits,
            max_center_checks=int(param('max_center_checks',256)),atlas=atlas,
            atlas_unavailable_reason=atlas_reason,budget=self.budget)
        self.scene_client = self.create_client(GetPlanningScene,'/get_planning_scene')
        self.map_backend = str(param('map_backend','observer'))
        self.observer = None
        if self.map_backend == 'observer':
            configuration = ObserverConfiguration.load(param('observer_geometry_file',''),
                                                        param('observer_config_file',''))
            self.observer = ObserverSnapshotPort(self,configuration=configuration,call=self._call,
                prefix=str(param('observer_prefix','/uld_occ_observer')),
                max_age_ms=int(param('observer_max_age_ms',1000)),recorder=self._record)
        elif self.map_backend == 'grid_g':
            self.volume_client = self.create_client(GetCargoVolumeV2,'/cargo_map/get_volume_v2')
            self.map_parameters = self.create_client(GetParameters,param('map_parameter_service','/cargo_volume_mapper/get_parameters'))
            qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(SurfaceContractV2,'/luggage/cargo_map/surface_v2',self._surface,qos)
            self.create_subscription(String,'/luggage/cargo_map/surface_2d',self._confirmation,qos)
        else:
            raise ValueError('map_backend must be observer or explicit grid_g compatibility')
        # Detector publishes a sensor stream. A RELIABLE reader cannot receive
        # a BEST_EFFORT writer; frozen quality RPCs provide retained evidence.
        sensor_qos = QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(SuctionTopDomainV2,'/luggage/suction_top_domain_v2',self._domain,sensor_qos)
        self.create_subscription(JointState,'/joint_states',self._joints,10)
        self.create_service(PrepareJointOperationV2,'/joint_operation/prepare',self._prepare,
                            callback_group=ReentrantCallbackGroup())
        self.create_service(CheckPreparedOperationV2,'/joint_operation/check_prepared',self._check_prepared,
                            callback_group=ReentrantCallbackGroup())

    def _check_prepared(self,request,response):
        from luggage_msgs.msg import FrozenSuctionObservationV2
        response.current = False
        response.frozen_observation = FrozenSuctionObservationV2()
        response.map_source = map_source_to_message(None)
        response.reason = 'PLAN_NOT_CURRENTLY_ISSUED'
        try:
            issued = self.issued.lookup(request.plan.plan_digest)
            if issued is not None:
                # Expectations come from the owner, not the client's fields.
                plan = operation_plan_from_message(request.plan,issued.identity,issued.plan_digest)
                response.current = self.issued.current(plan)
                binding = self.issued.contact_binding(plan) if response.current else None
                if binding is not None:response.frozen_observation = binding_to_message(binding)
                if response.current:response.map_source = map_source_to_message(self.issued.map_source(plan))
                response.reason = 'OWNER_ISSUANCE_CURRENT' if response.current else 'PLAN_NOT_CURRENTLY_ISSUED'
        except Exception as error:
            response.reason = 'ISSUANCE_CHECK_REFUSED:'+type(error).__name__
        return response

    def _record(self,boundary,value):
        if not self._evidence: return
        with self._evidence_lock:
            self._record_owned(boundary,value)

    def _record_owned(self,boundary,value):
        raw = value if isinstance(value,bytes) else serialize_message(value)
        if self._evidence_bytes+len(raw) > 256*1024*1024:
            raise RuntimeError('OPERATION_EVIDENCE_BUDGET_EXHAUSTED')
        self._evidence_seq += 1;self._evidence_bytes += len(raw)
        name = '%05d.cdr'%self._evidence_seq
        (Path(self._evidence)/name).write_bytes(raw)
        with (Path(self._evidence)/'boundaries.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(file=name,boundary=boundary,bytes=len(raw),
                monotonic_ns=time.monotonic_ns(),sha256=hashlib.sha256(raw).hexdigest()))+'\n')

    def _surface(self,message):
        try:
            context = BoundaryContext('JOINT_OCC','cargo_volume_mapper','joint_operation',
                message.identity.request_id,self.revision,'container_link','','scene',message.geometry_hash,message.map_revision)
            self.store.update_typed(surface_from_message(message,context),serialize_message(message))
        except (TypeError,ValueError): self.store.invalidate()

    def _confirmation(self,message):
        try: self.store.update_confirmation(json.loads(message.data))
        except (TypeError,ValueError,KeyError): self.store.invalidate()

    def _domain(self,message):
        try: domain = domain_from_message(message)
        except (TypeError,ValueError): domain = None
        with self._state_lock: self.domain = domain

    def _joints(self,message):
        try:
            if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
                raise ValueError('ambiguous joint state')
            values = dict(zip(message.name,message.position))
            q = tuple(float(values[name]) for name in OPERATION_JOINTS)
            if not all(math.isfinite(v) for v in q): raise ValueError('finite joints required')
            stamp = int(message.header.stamp.sec)*10**9+int(message.header.stamp.nanosec)
            current = (stamp,q)
        except (TypeError,ValueError,KeyError): current = None
        with self._state_lock: self.joints = current

    def _current(self,request,*,frozen_domain=None):
        with self._state_lock: domain,joints = self.domain,self.joints
        if frozen_domain is not None:domain = frozen_domain
        if domain is None or joints is None: raise ValueError('OBSERVATION_OR_JOINTS_UNAVAILABLE')
        require_matching_detection(domain,request.detection,generation=int(request.generation),
                                   contact_model_hash=self.preparation.model.identity_hash)
        if (not request.robot_state_stamp_valid or joints[0] < request.robot_state_stamp_ns
                or any(abs(a-b)>self.start_tolerance for a,b in zip(joints[1],request.start_joints))):
            raise ValueError('CURRENT_START_STATE_MISMATCH')
        return domain

    def _call(self,client,request,deadline,boundary,*,late_response=None):
        if time.monotonic() >= deadline: raise TimeoutError('OPERATION_DEADLINE')
        old = self._inflight.get(client)
        completion = self._rpc_completions.get(client)
        if ((old is not None and not old.done()) or
                (completion is not None and not completion.is_set())):
            raise RuntimeError(boundary+'_BACKEND_BUSY')
        if not client.service_is_ready(): raise RuntimeError(boundary+'_BACKEND_UNAVAILABLE')
        self._record(boundary+'_request',request)
        event = threading.Event()
        future = self._inflight[client] = client.call_async(request)
        consumed = self._rpc_completions[client] = threading.Event()
        guard = threading.Lock()
        state = dict(abandoned=False,cleaned=False)
        def clean_late():
            with guard:
                if not state['abandoned'] or state['cleaned'] or not future.done():return
                state['cleaned'] = True
            try:
                result = future.result()
                if late_response is not None and result is not None:
                    try:self._record(boundary+'_late_response',result)
                    finally:late_response(result)
            except Exception as error:self.get_logger().error('LATE_RESPONSE_CLEANUP_FAILED:'+str(error))
            finally:consumed.set()
        def complete(_):
            event.set()
            clean_late()
        future.add_done_callback(complete)
        returned = False
        try:
            while not event.wait(min(.01,max(0.,deadline-time.monotonic()))):
                if time.monotonic() >= deadline: raise TimeoutError('OPERATION_DEADLINE')
            result = future.result()
            self._record(boundary+'_response',result)
            if time.monotonic() >= deadline: raise TimeoutError('OPERATION_DEADLINE')
            returned = True
            return result
        finally:
            if not returned:
                with guard:state['abandoned'] = True
                clean_late()
            else:consumed.set()

    def _capture_map(self,request,deadline,*,latest=False):
        if self.observer is not None:
            captured = self.observer.capture(scene_revision=self.revision,request_id=request.request_id,
                expected_epoch=request.expected_map_epoch,
                expected_revision=None if latest else request.expected_map_revision,
                deadline=deadline)
            return captured.surface,captured.volume,captured.source
        parameters = self._call(self.map_parameters,GetParameters.Request(names=['require_confirmed_map']),deadline,'map_parameters')
        if len(parameters.values)!=1 or parameters.values[0].type!=1:
            raise ValueError('MAP_CONFIRMATION_CONFIGURATION_UNAVAILABLE')
        self.store.set_confirmation_required(parameters.values[0].bool_value)
        confirmed = self.store.current()
        if confirmed is None:raise ValueError('NO_CONFIRMED_OCC_SNAPSHOT')
        return confirmed,None,None

    def _grid_g_volume(self,confirmed,deadline):
        record = confirmed.record()
        volume_request = GetCargoVolumeV2.Request(scene_revision=record['scene_revision'],geometry_hash=record['geometry_hash'],
            expected_map_revision=record['map_revision'],expected_epoch=str(record.get('epoch') or ''),
            expected_content_digest=str(record.get('content_digest') or ''))
        volume = self._call(self.volume_client,volume_request,deadline,'volume')
        if ((volume_request.expected_epoch and volume.epoch!=volume_request.expected_epoch) or
                (volume_request.expected_content_digest and volume.content_digest!=volume_request.expected_content_digest)):
            raise ValueError('VOLUME_SNAPSHOT_IDENTITY_MISMATCH')
        return volume

    def _scene(self,deadline):
        request = GetPlanningScene.Request()
        request.components.components = (PlanningSceneComponents.SCENE_SETTINGS|PlanningSceneComponents.ROBOT_STATE|
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS|PlanningSceneComponents.WORLD_OBJECT_NAMES|
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY|PlanningSceneComponents.OCTOMAP|PlanningSceneComponents.TRANSFORMS|
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX|PlanningSceneComponents.LINK_PADDING_AND_SCALING)
        return self._call(self.scene_client,request,deadline,'scene').scene

    def _placements(self,request,confirmed,volume):
        pool = enumerate_placements(confirmed,(request.detection.width,request.detection.depth,request.detection.height),
                                    self.slot_limit,volume=volume,margin=self.margin)
        if request.slots:
            result = []
            for index,slot in enumerate(request.slots):
                context = BoundaryContext('JOINT_SLOT','caller','joint_operation',slot.identity.request_id,
                    self.revision,'container_link','box_center','scene')
                record = pose_from_message(slot,'slot',context)
                target = Transform(record['position'],record['quaternion_xyzw'])
                def same_geometry(candidate):
                    pose = candidate.container_from_box
                    return (all(abs(a-b)<=1e-8 for a,b in zip(pose.translation,target.translation)) and
                        abs(sum(a*b for a,b in zip(pose.quaternion_xyzw,target.quaternion_xyzw)))>=1.-1e-12)
                if not any(same_geometry(candidate) for candidate in pool):
                    raise ValueError('REQUESTED_SLOT_NOT_IN_BOUNDED_GEOMETRIC_POOL')
                result.append(PlacementCandidate('requested-'+str(index),target))
            return tuple(result)
        # Shared retention puts one deep target per directed heading first,
        # then the shallow alternatives. Never fabricate a half turn here.
        return pool

    def _prepare(self,request,response):
        from luggage_msgs.msg import JointOperationPlanV2
        started = time.monotonic();response.success = False;response.plan = JointOperationPlanV2()
        issued_plan = None
        observed = None
        map_source = None
        response.trace_json,response.elapsed_s = '[]',0.
        if not self._active.acquire(blocking=False):
            response.reason = 'OPERATION_PREPARATION_BUSY';return response
        budget_value = request.budget_s if math.isfinite(request.budget_s) else str(request.budget_s)
        root_trace = [('preparation_budget',budget_value,self.budget.max_budget_s,
                       'diagnostic_watchdog_not_latency_target')]
        timing = StageTimings('root',root_trace,unit=request.request_id)
        def admit_request():
            if (not self.budget.admits(request.budget_s)
                    or not request.request_id or not request.observation_id or not 1 <= request.candidate_limit <= 8
                    or len(request.slots)>32
                    or request.expected_model_sha256 != self.model.model_sha256):
                raise ValueError('PREPARATION_INPUT_IDENTITY_MISMATCH')
            pickup_collision_aabb(request.detection)
        def sync_trace():
            # Keep the core trace separate so repeated synchronization after
            # issuance does not duplicate root stages in the returned response.
            response.trace_json = json.dumps(core_trace+root_trace,separators=(',',':'),allow_nan=False)
            response.elapsed_s = time.monotonic()-started
        core_trace = []
        try:
            self._record('prepare_request',request)
            timing.call('input_admission',admit_request)
            deadline = started+request.budget_s
            domain = timing.call('domain_admission',self._current,request)
            retained = timing.call('frozen_observation',self.observed_contacts.retain,domain,request.detection,
                request_id=request.request_id,observation_id=request.observation_id,
                deadline=deadline,cancelled=lambda:False)
            if not retained.retained:raise ValueError(retained.reason)
            observed = retained.observation
            confirmed,volume,map_source = timing.call('map_capture',self._capture_map,request,deadline)
            record = confirmed.record()
            if (record['geometry_hash'],record['map_revision'],record['scene_revision']) != (
                    request.geometry_hash,request.expected_map_revision,self.revision):
                raise ValueError('OCC_INPUT_IDENTITY_MISMATCH')
            baseline = timing.call('base_scene',self._scene,deadline)
            expected_world = proposal_scene_hash(baseline)
            if volume is None:
                volume = timing.call('g_volume_capture',self._grid_g_volume,confirmed,deadline)
            snapshot = timing.call('private_scene',build_private_snapshot,
                confirmed,baseline,self.model.model_sha256,self.container,
                                              collision_margin_m=self.margin,volume=volume)
            placements = timing.call('placement_candidates',self._placements,request,confirmed,volume)
            reference = Transform.from_matrix(self.model.urdf.tcp(list(request.start_joints),self.model.path))
            inputs = PreparationInputs(snapshot,domain,placements,reference,self.pickup_id,observed)
            def observation_current(limit):
                checked = self.observed_contacts.retain(domain,request.detection,
                    request_id=request.request_id,observation_id=request.observation_id,
                    expected_observation=observed,deadline=limit,cancelled=lambda:False)
                return (checked.retained and checked.observation.binding==observed.binding
                    and self._current(request,frozen_domain=domain).digest()==domain.digest())
            def revalidate(current,limit):
                # Check again after the asynchronous scene read: subscriptions
                # can advance the map/domain/joints while that read is pending.
                scene = self._scene(limit)
                observed_current = observation_current(limit)
                latest = self.store.current()
                map_current = (self.observer.current(map_source,deadline=limit) if self.observer is not None
                               else latest is not None and latest.identity==confirmed.identity)
                return (map_current
                    and observed_current
                    and proposal_scene_hash(scene)==expected_world)
            finalize = None
            if self.observer is not None:
                def capture_latest(limit):
                    surface,latest_volume,source = self._capture_map(request,limit,latest=True)
                    scene = self._scene(limit)
                    fresh_snapshot = build_private_snapshot(surface,scene,self.model.model_sha256,self.container,
                        collision_margin_m=self.margin,volume=latest_volume)
                    return RefreshCapture(surface,latest_volume,source,scene,fresh_snapshot)
                finalizer = LatestMapFinalizer(original_surface=confirmed,original_source=map_source,
                    original_snapshot=snapshot,baseline_hash=expected_world,capture=capture_latest,
                    read_baseline=self._scene,source_current=lambda source,limit:
                        self.observer.current(source,deadline=limit),observation_current=observation_current,
                    certification=self.preparation.planner.certification,attempts=self.refresh_attempts)
                def finalize(plan,candidate,current,limit):
                    nonlocal map_source
                    result = finalizer.finalize(plan,candidate,current,limit)
                    if result.plan is not None:map_source = result.map_source
                    return result
            kwargs = dict(deadline=deadline,revalidate=revalidate)
            if finalize is not None:kwargs['finalize'] = finalize
            timing.call('whole_preparation',self.preparation.prepare,request,response,inputs,**kwargs)
            if response.success:
                # Internal owner output, before publication; external check
                # requests instead decode against the retained owner identity.
                identity = OperationIdentity(**{name:getattr(response.plan,name)
                    for name in OperationIdentity.__dataclass_fields__})
                issued_plan = operation_plan_from_message(response.plan,identity,response.plan.plan_digest)
        except Exception as error:
            response.success = False;response.plan = JointOperationPlanV2()
            response.reason = 'PREPARATION_REFUSED:'+type(error).__name__+':'+str(error)
        finally:
            core_trace = json.loads(response.trace_json)
            sync_trace()
            try: self._record('prepare_response',response)
            except Exception as error:
                # Capture failure cannot leave a certified response active or
                # strand the ROS request by escaping from the service callback.
                response.success = False;response.plan = JointOperationPlanV2()
                response.reason = 'PREPARATION_EVIDENCE_FAILED:'+type(error).__name__
                self.get_logger().error(response.reason+':'+str(error))
            else:
                if response.success:
                    try:
                        timing.call('plan_issuance',self.issued.issue,issued_plan,
                                    contact_binding=observed.binding,map_source=map_source)
                    except Exception as error:
                        response.success = False;response.plan = JointOperationPlanV2()
                        response.reason = 'PREPARATION_ISSUANCE_FAILED:'+type(error).__name__
                        self.get_logger().error(response.reason+':'+str(error))
                        sync_trace()
                        try: self._record('issuance_failure_response',response)
                        except Exception: pass
            finally:
                sync_trace()
                self._active.release()
        return response


def main(args=None):
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = JointOperationPrepareNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try: executor.spin()
    finally:
        executor.shutdown();node.destroy_node();rclpy.shutdown()
