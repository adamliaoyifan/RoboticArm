#!/usr/bin/env python3
"""Read-only background placement prior service. Never applies a planning scene."""
import hashlib
import json
import threading
import time
from dataclasses import asdict

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.serialization import serialize_message, deserialize_message
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from luggage_msgs.msg import SurfaceContractV2
from luggage_msgs.srv import PreparePlacementPriorV2, GetCargoVolumeV2

from luggage_description.frame_diagnostics import BoundaryContext
from luggage_description.ros_frame_bindings.surfaces import surface_from_message
from luggage_planning.placement_prior_contracts import Transform
from luggage_planning.placement_prior_surface import ConfirmedSurfaceStore
from luggage_planning.placement_prior_snapshot import build_private_snapshot
from luggage_planning.placement_prior_collision_port import NativeToolCollisionPort
from luggage_planning.placement_prior_requests import decode_request, encode_choices
from luggage_planning.placement_prior_worker import PriorWorker
from luggage_planning.placement_prior_prefetch import PrefetchTemplate
from luggage_planning.placement_prior_schedule import SnapshotQuietGate


class PlacePriorNode(Node):
    def __init__(self):
        super().__init__('place_prior')
        self._scene_revision=self.declare_parameter('frame_scene_revision','').value
        xml=self.declare_parameter('robot_description','').value
        semantic=self.declare_parameter('robot_description_semantic','').value
        if not self._scene_revision or not xml or not semantic:
            raise ValueError('scene revision and complete model required')
        self._model_hash=hashlib.sha256((xml+semantic).encode()).hexdigest()
        translation=self.declare_parameter('container_translation',[float('nan')]*3).value
        rotation=self.declare_parameter('container_quaternion_xyzw',[float('nan')]*4).value
        self._container=Transform(translation,rotation)
        self._margin=float(self.declare_parameter('collision_margin_m',.05).value)
        self._fallback_yaw=float(self.declare_parameter('pick_fallback_yaw',0.).value)
        self._store=ConfirmedSurfaceStore()
        evidence_dir=self.declare_parameter('evidence_dir','').value
        self._recorder=None
        if evidence_dir:
            from luggage_planning.placement_prior_evidence import PriorEvidenceRecorder
            self._recorder=PriorEvidenceRecorder(evidence_dir)
        self._collision=NativeToolCollisionPort(self, recorder=self._record_collision)
        self._worker=PriorWorker(self._collision)
        self._active_snapshot=None
        self._last_request=None
        self._last_result=None
        self._lock=threading.Lock()
        self._foreground_active=False
        self._prefetched=None
        self._prefetch_attempt=None
        self._quiet_gate=SnapshotQuietGate(
            float(self.declare_parameter('prefetch_quiet_s',2.).value))
        enabled=self.declare_parameter('prefetch_enabled',False).value
        size=self.declare_parameter('prefetch_box_wdh',[0.,0.,0.]).value
        grasps=self.declare_parameter('prefetch_grasps_flat',[0.]*7).value
        self._template=PrefetchTemplate(size,grasps) if enabled else None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(SurfaceContractV2,'/luggage/cargo_map/surface_v2',self._on_surface,qos)
        self.create_subscription(String,'/luggage/cargo_map/surface_2d',self._on_confirmation,qos)
        self._get_volume=self.create_client(GetCargoVolumeV2,'/cargo_map/get_volume_v2')
        self._get_scene=self.create_client(GetPlanningScene,'/get_planning_scene')
        mapper=self.declare_parameter('map_parameter_service','/cargo_volume_mapper/get_parameters').value
        self._get_map_parameters=self.create_client(GetParameters,mapper)
        self._service_group=MutuallyExclusiveCallbackGroup()
        self._service=self.create_service(PreparePlacementPriorV2,'/placement_prior/prepare',
                                         self._prepare,callback_group=self._service_group)
        self._prefetch_group=MutuallyExclusiveCallbackGroup()
        self._prefetch_timer=self.create_timer(1.,self._prefetch_tick,
                                               callback_group=self._prefetch_group)

    def _invalidate_if_changed(self):
        confirmed=self._store.current()
        identity=confirmed.identity if confirmed else None
        self._quiet_gate.observe(identity,time.monotonic())
        if identity!=self._active_snapshot:
            self._active_snapshot=identity
            self._last_request=None;self._last_result=None
            self._prefetched=None
            self._prefetch_attempt=None
            self._worker.invalidate()

    def _prefetch_tick(self):
        if self._template is None:
            return
        if not (self._get_scene.service_is_ready()
                and self._get_map_parameters.service_is_ready() and self._collision.ready()):
            return
        try:
            params=self._call(self._get_map_parameters,
                             GetParameters.Request(names=['require_confirmed_map']),2.)
            if len(params.values)!=1 or params.values[0].type!=1:
                return
        except (RuntimeError,TimeoutError):
            return
        with self._lock:
            self._store.set_confirmation_required(params.values[0].bool_value)
            self._invalidate_if_changed()
            confirmed=self._store.current()
            if (confirmed is None or self._foreground_active
                    or self._prefetch_attempt==confirmed.identity
                    or not self._quiet_gate.ready(confirmed.identity,time.monotonic())):
                return
            self._prefetch_attempt=confirmed.identity
        response=self._prepare(self._template.request(confirmed),
                               PreparePlacementPriorV2.Response(),background=True)
        self.get_logger().info('template prefetch: %s (%.3fs)' %
                               (response.reason,response.elapsed_s))

    def _on_surface(self,message):
        try:
            context=BoundaryContext('PRIOR_OCC','cargo_volume_mapper','place_prior',
                message.identity.request_id,self._scene_revision,'container_link','',
                'scene',message.geometry_hash,message.map_revision)
            record=surface_from_message(message,context)
            with self._lock:
                self._store.update_typed(record,serialize_message(message))
                self._invalidate_if_changed()
        except Exception as exc:
            self.get_logger().warning('prior surface refused: '+str(exc))
            with self._lock:
                self._store.invalidate();self._invalidate_if_changed()

    def _on_confirmation(self,message):
        try:
            record=json.loads(message.data)
            with self._lock:
                self._store.update_confirmation(record)
                self._invalidate_if_changed()
        except Exception as exc:
            self.get_logger().warning('prior confirmation refused: '+str(exc))
            with self._lock:
                self._store.invalidate();self._invalidate_if_changed()

    @staticmethod
    def _call(client,request,timeout):
        if not client.service_is_ready():
            raise RuntimeError('read-only dependency unavailable')
        future=client.call_async(request);done=threading.Event()
        future.add_done_callback(lambda f:done.set())
        if not done.wait(timeout):
            raise TimeoutError('read-only dependency timeout')
        return future.result()

    def _record_collision(self, boundary, message):
        if self._recorder:
            self._recorder('collision_'+boundary, message)

    def _prepare(self,message,response,*,background=False):
        started=time.monotonic();response.success=False;response.measured=message.measured
        if not background:
            with self._lock:
                self._foreground_active=True
                self._worker.invalidate()
        try:
            if self._recorder:self._recorder('prepare_request',message)
            params=self._call(self._get_map_parameters,
                             GetParameters.Request(names=['require_confirmed_map']),2.)
            if len(params.values)!=1 or params.values[0].type!=1:
                raise ValueError('map confirmation configuration unavailable')
            # Read the owner setting on every admission; it may change at runtime.
            with self._lock:
                self._store.set_confirmation_required(params.values[0].bool_value)
                self._invalidate_if_changed()
                confirmed=self._store.current()
            if confirmed is None:raise ValueError('NO_CONFIRMED_OCC_SNAPSHOT')
            get_request=GetPlanningScene.Request()
            get_request.components.components=(PlanningSceneComponents.SCENE_SETTINGS |
                PlanningSceneComponents.ROBOT_STATE | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                PlanningSceneComponents.OCTOMAP | PlanningSceneComponents.TRANSFORMS |
                PlanningSceneComponents.ALLOWED_COLLISION_MATRIX | PlanningSceneComponents.LINK_PADDING_AND_SCALING)
            baseline=self._call(self._get_scene,get_request,2.).scene
            if self._recorder:self._recorder('baseline_scene',baseline)
            surface_record=confirmed.record()
            volume_request=GetCargoVolumeV2.Request(
                scene_revision=surface_record['scene_revision'],
                geometry_hash=surface_record['geometry_hash'],
                expected_map_revision=surface_record['map_revision'])
            if self._recorder:self._recorder('volume_request',volume_request)
            volume=self._call(self._get_volume,volume_request,2.)
            if self._recorder:self._recorder('volume_response',volume)
            snapshot=build_private_snapshot(confirmed,baseline,self._model_hash,self._container,
                                            collision_margin_m=self._margin,volume=volume)
            with self._lock:
                prefetched=self._prefetched
            request=decode_request(message,snapshot,confirmed,fallback_yaw=self._fallback_yaw,
                                   prefetched=prefetched if message.measured else None,
                                   volume=volume,margin=self._margin)
            if self._recorder:
                # Diagnostic eligibility/order, never an execution authorization.
                previous=prefetched[0] if prefetched is not None else None
                preferred=({c.placement.slot_id for c in prefetched[1].choices}
                           if prefetched is not None else set())
                self._recorder('proposal_context',String(data=json.dumps({
                    'request_id':request.request_id,'measured':request.measured,
                    'template_present':previous is not None,
                    'same_full_scene':bool(previous and previous.snapshot.identity==snapshot.identity),
                    'same_ordering_context':bool(previous and previous.snapshot.proposal_identity==snapshot.proposal_identity),
                    'same_dimensions':bool(previous and previous.box_wdh==request.box_wdh),
                    'explicit_slots':bool(message.slots),
                    'slot_order':[p.slot_id for p in request.placements],
                    'template_preferred_slots':sorted(preferred),
                })))
            with self._lock:
                if background and self._foreground_active:
                    raise ValueError('PREFETCH_SUPERSEDED_BY_FOREGROUND')
                current=self._store.current()
                if current is None or current.identity!=confirmed.identity:
                    raise ValueError('OCC_CHANGED_DURING_PREPARATION')
                cached=self._last_result if (self._last_request is not None and
                        self._last_request.reuse_key==request.reuse_key) else None
                # Atomic with the foreground-priority check above: a late
                # background submission must never supersede foreground work.
                generation=None if cached is not None else self._worker.submit(request)
            if cached is not None:
                result=cached;response.cache_hit=True
            else:
                result,error=self._worker.wait_result(generation,request.budget_s+.5)
                if error:raise RuntimeError(': '.join(error))
                if result is None:
                    self._worker.invalidate(generation=generation)
                    raise TimeoutError('PRIOR_DEADLINE')
            with self._lock:
                if background and self._foreground_active:
                    raise ValueError('PREFETCH_SUPERSEDED_BY_FOREGROUND')
                current=self._store.current()
                if current is None or current.identity!=confirmed.identity:
                    raise ValueError('OCC_CHANGED_DURING_PREPARATION')
                if result.choices:
                    self._last_request=request;self._last_result=result
                    if background:
                        self._prefetched=(request,result)
            response.snapshot_id=confirmed.identity
            response.geometry_hash=snapshot.geometry_hash;response.map_revision=snapshot.map_revision
            response.model_sha256=self._model_hash;response.box_wdh=list(request.box_wdh)
            response.surface=deserialize_message(snapshot.surface_cdr,SurfaceContractV2)
            response.choices=encode_choices(result,request)
            response.trace_json=json.dumps([asdict(row) for row in result.trace])
            response.success=bool(response.choices);response.reason=result.status
        except Exception as exc:
            response.reason=type(exc).__name__+': '+str(exc)
            response.choices=[]
        finally:
            if not background:
                with self._lock:
                    self._foreground_active=False
        response.elapsed_s=time.monotonic()-started
        if self._recorder:
            try:
                self._recorder('prepare_response',response)
            except Exception as exc:
                response.success=False
                response.choices=[]
                response.reason='PRIOR_EVIDENCE_INCOMPLETE: '+str(exc)
                self.get_logger().error(response.reason)
        return response

    def destroy_node(self):
        self._worker.close(timeout=1.)
        return super().destroy_node()


def main():
    rclpy.init();node=PlacePriorNode();executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:executor.spin()
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
