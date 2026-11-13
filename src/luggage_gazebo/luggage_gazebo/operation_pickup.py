"""Observed pickup preparation for the existing explicitly authorized cycle.

Owns observation joins, map leases and optional pre-pick recovery. Physics spawn
and complete-operation execution remain with the cycle driver. Online geometry
comes only from detector/domain/map owners, never the spawner response.
"""
import json
from pathlib import Path
import time

from rclpy.action import ActionClient
from luggage_msgs.action import ExecutePickupViewV2
from luggage_msgs.srv import DetectLuggage, PrepareJointOperationV2, ReprepareAfterPickupViewV2
from uld_occ_observer.srv import GetSnapshot, GetValidity, ReleaseSnapshot
from luggage_mapping.observer import admit_response, identity_record
from luggage_planning.joint_operation.pickup_recovery import (
    PickupPreparationAttempt, PickupPreparationRecovery, PickupRecoveryPorts, PickupRecoverySettings,
)
from luggage_gazebo.operation_readiness import wait_observed_detection


JOINTS = tuple('elfin_joint%d'%i for i in range(1, 7))


class ObservedPickupPreparation:
    def __init__(self, host, config_path=None):
        self.host = host
        self.settings = None
        self._recovery_task_id = self._recovery_deadline = None
        self.view_action = self.continuation = None
        if config_path:
            raw = Path(config_path).read_bytes()
            self.settings = PickupRecoverySettings.from_record(json.loads(raw))
            host.record('pickup_recovery_config.json', raw)
            if self.settings.max_extra_view_motions:
                self.view_action = ActionClient(host, ExecutePickupViewV2, '/joint_operation/execute_pickup_view')
                self.continuation = host.create_client(ReprepareAfterPickupViewV2,
                    '/joint_operation/continue_after_pickup_view')

    def observe(self, *, deadline=None, after_stamp_ns=None):
        host = self.host
        limit = time.monotonic()+30. if deadline is None else deadline
        detection, domain = wait_observed_detection(
            lambda remaining: host.call(host.detect, DetectLuggage.Request(), 'detect',
                timeout=remaining, deadline=limit), lambda: host.domain, deadline=limit,
            after_stamp_ns=after_stamp_ns)
        host.record('top_domain', domain)
        if host.pickup_capture is not None:
            host.pickup_capture_count += 1
            stamp = domain.header.stamp
            receipt = host.pickup_capture.write(
                host.folder/('pickup_acquisition_'+str(host.pickup_capture_count)),
                stamp.sec*10**9+stamp.nanosec, required_topics=host.pickup_capture_required)
            host.record('pickup_acquisition_receipt', json.dumps(receipt).encode())
            host.pickup_capture_active = False
        for key, message in host.images.items(): host.record('detection_'+key, message)
        return detection, domain

    def prepare_on_map(self, detection, domain, *, task_mode=None, deadline=None):
        host = self.host
        validity = host.call(host.validity, GetValidity.Request(), 'prepare_validity', deadline=deadline)
        # Pin actual repository bytes until the read-only owner has captured its
        # independent snapshot. This lease ends before any extra view motion.
        lease = host.call(host.snapshot, GetSnapshot.Request(expected_epoch=validity.epoch,
            exact_revision=False, max_age_ms=1000, allow_stale=False), 'prepare_snapshot', deadline=deadline)
        if not lease.handle: raise RuntimeError('MAP_SNAPSHOT:'+lease.status)
        try:
            admit_response(lease, identity_record(lease.identity), lease.validity.generation)
            return self.prepare_from_observation(detection, domain, lease.identity,
                                                  task_mode=task_mode, deadline=deadline)
        finally:
            # Resource cleanup is permitted after a planning deadline. It cannot
            # create a plan or another motion and does not renew recovery time.
            host.call(host.release_snapshot, ReleaseSnapshot.Request(handle=lease.handle), 'release_prepare_snapshot')

    def prepare_from_observation(self, detection, domain, identity, *, slots=(), task_mode=None, deadline=None):
        host = self.host
        joint = host.joint; host.record('prepare_joints', joint)
        positions = dict(zip(joint.name, joint.position))
        budget = float(host.launch['resolved']['operation_prepare_budget_s'])
        if deadline is not None: budget = min(budget, max(0., deadline-time.monotonic()))
        request = PrepareJointOperationV2.Request(request_id='observed-'+str(time.time_ns()),
            observation_id=domain.domain_sha256, detection=detection, generation=domain.generation,
            geometry_hash=identity.geometry_hash, expected_map_revision=identity.map_revision,
            expected_map_epoch=identity.epoch, expected_model_sha256=host.launch['model_sha256'],
            robot_state_stamp_valid=True,
            robot_state_stamp_ns=joint.header.stamp.sec*10**9+joint.header.stamp.nanosec,
            start_joints=[positions[name] for name in JOINTS], candidate_limit=8,
            placement_task_mode=task_mode or '', budget_s=budget)
        from luggage_planning.place.place_service import pose_message
        request.slots = [pose_message(dict(contract_version=2, kind='pose', frame_id='container_link', length_unit='m',
            stamp_ns=0, time_basis='scene', scene_revision=host.launch['scene_revision'], request_id=request.request_id,
            reference='box_center', orientation_semantics='box_axis_pi', position=slot.translation,
            quaternion_xyzw=slot.quaternion_xyzw)) for slot in slots]
        prepared = host.call(host.prepare, request, 'prepare', timeout=budget+10., deadline=deadline)
        trace = host.folder/'prepare_trace.json'
        if trace.exists(): trace = host.folder/('prepare_trace_'+request.request_id+'.json')
        trace.write_text(prepared.trace_json+'\n')
        return PickupPreparationAttempt(request, prepared, detection, domain)

    def execute_view(self, goal, deadline):
        host = self.host
        if host.pickup_capture is not None: host.pickup_capture_active = True
        return host.action(self.view_action, goal, 'pickup_extra_view',
                           max(0., deadline-time.monotonic()), deadline=deadline)

    def continue_after(self, request, deadline):
        return self.host.call(self.continuation, request, 'pickup_continue',
            timeout=max(0., deadline-time.monotonic()), deadline=deadline)

    def plan(self, *, task_mode=None):
        detection, domain = self.observe()
        initial = self.prepare_on_map(detection, domain, task_mode=task_mode)
        ports = PickupRecoveryPorts(self.execute_view,
            lambda stamp, deadline: self.observe(deadline=deadline, after_stamp_ns=stamp),
            self.continue_after,
            lambda detected, observed, deadline: self.prepare_on_map(detected, observed,
                task_mode=task_mode, deadline=deadline))
        task_id = self.host.pickup_task_id
        deadline = None
        if (self.settings is not None and self.settings.max_extra_view_motions
                and initial.observation_probes()):
            # This adapter spans pre-pick retries. Its monotonic bound also
            # limits ordinary fallback, which has no view-ledger RPC of its own.
            # The runtime ledger independently owns durable motion/count limits.
            if self._recovery_task_id != task_id:
                self._recovery_task_id = task_id
                self._recovery_deadline = time.monotonic()+self.settings.total_recovery_budget_s
                self.host.record('pickup_recovery_opened.json', json.dumps(dict(
                    task_id=task_id, deadline_monotonic=self._recovery_deadline)).encode())
            deadline = self._recovery_deadline
        workflow = PickupPreparationRecovery(self.settings, ports, record=self.host.record, clock=time.monotonic)
        return workflow.run(initial, pickup_task_id=task_id, deadline=deadline)
