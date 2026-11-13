"""Empty pre-pick view Action under the complete runtime's shared motion owner.

This stage ends at measured settling. It never treats motion as new observation,
associates boxes, fills support, changes vacuum or modifies a loaded grasp.
"""
import json
import math
import threading
import time
import traceback

from luggage_planning.joint_operation.contracts import OPERATION_JOINTS
from luggage_planning.joint_operation.view_budget import PickupViewBudgetLedger
from luggage_planning.joint_operation.view_execution_ports import PickupViewExecutionPorts
from luggage_planning.motion.settle_criterion import SettleTracker


class PickupViewExecution:
    def __init__(self,parent):
        from luggage_msgs.action import ExecutePickupViewV2
        from luggage_msgs.msg import VacuumState
        from rclpy.action import ActionServer,GoalResponse,CancelResponse
        from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
        from luggage_description.scene_tf_config_utils import load_scene_tf_config
        from luggage_planning.joint_operation.scene_admission import StaticWorldAdmission
        self.parent,self.node=parent,parent.node
        self.action=ExecutePickupViewV2
        self.owns_session=False
        self.dispatched_this_action=False;self.unexpected_load=False;self.load_checkpointed=False
        self.unexpected_load_sample=None
        self.phase='IDLE';self.task_id=''
        self.vacuum_lock=threading.Lock();self.vacuum=None
        self.ledger=PickupViewBudgetLedger(str(parent.journal.path)+'.pickup_views.json')
        scene_path=self.node.get_parameter('scene_tf_config').value
        mesh=(self.node.get_parameter('container_collision_mesh')
              if self.node.has_parameter('container_collision_mesh')
              else self.node.declare_parameter('container_collision_mesh','')).value
        self.ports=PickupViewExecutionPorts(parent,static_world_factory=lambda:StaticWorldAdmission(
            load_scene_tf_config(scene_path),parent.ports.container,mesh_path=mesh))
        # Volatile subscription does not consume an old transient-local sample.
        # Receipt time is transport freshness, not a sensor acquisition stamp.
        qos=QoSProfile(depth=10,durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE)
        self.subscription=self.node.create_subscription(VacuumState,'/vacuum/state',self.on_vacuum,qos,
            callback_group=self.node._group)
        def accept(goal):
            valid=(goal.pickup_task_id.strip() and math.isfinite(goal.budget_s)
                and 0<goal.budget_s<=parent.max_budget and math.isfinite(goal.total_recovery_budget_s)
                and goal.total_recovery_budget_s>0)
            return (GoalResponse.ACCEPT if valid and not parent._active.locked() and parent.session is None
                    else GoalResponse.REJECT)
        def cancel(handle):
            if parent.handle is handle:
                parent.owner.request_stop(handle);parent.motion.cancel()
            return CancelResponse.ACCEPT
        self.server=ActionServer(self.node,self.action,'/joint_operation/execute_pickup_view',
            goal_callback=accept,cancel_callback=cancel,execute_callback=self.execute,
            callback_group=self.node._group)

    def on_vacuum(self,message):
        with self.vacuum_lock:
            self.vacuum=(time.monotonic(),message.attached,message.vacuum_on)
            if self.owns_session and self.dispatched_this_action and (message.attached or message.vacuum_on):
                if not self.unexpected_load:self.unexpected_load_sample=self.vacuum
                self.unexpected_load=True

    def record_empty_state(self,stage):
        """Lossless consumed signal fields and receipt clock, not invented stamp."""
        with self.vacuum_lock:sample=self.vacuum;unexpected=self.unexpected_load_sample
        self.parent.record('pickup_view_empty_state.json',json.dumps(dict(stage=stage,
            receipt_monotonic_attached_vacuum_on=sample,now_monotonic_s=time.monotonic(),
            action_began_monotonic_s=self.began,source_acquisition_stamp_available=False,
            max_receipt_age_s=self.parent.motion.max_joint_age_ns/1e9,
            unexpected_load_sample=unexpected)).encode())

    def empty(self):
        with self.vacuum_lock:sample=self.vacuum
        return bool(sample is not None and sample[0]>=self.began
            and 0<=time.monotonic()-sample[0]<=self.parent.motion.max_joint_age_ns/1e9
            and sample[1:] == (False,False))

    def checkpoint(self,stage):
        self.phase=stage
        parent=self.parent
        count,elapsed=self.ledger.status(self.task_id)
        parent.journal.write(dict(kind='pickup_view',stage=stage,pickup_task_id=self.task_id,
            dispatched_motions=count,may_have_payload=self.unexpected_load,
            motion_owned=parent.owner.motion_lock.locked()))
        if self.unexpected_load:self.load_checkpointed=True
        parent.handle.publish_feedback(self.action.Feedback(pickup_task_id=self.task_id,
            stage=stage,dispatched_motions=count))
        parent.record('pickup_view_stage.json',json.dumps(dict(stage=stage,count=count,
            elapsed_s=elapsed,ros_stamp_ns=self.node.get_clock().now().nanoseconds)).encode())

    def charge_dispatch(self):
        self.record_empty_state('before_dispatch')
        self.checkpoint('CONTROLLER_DISPATCH_PENDING')
        self.ledger.dispatch(self.task_id)
        self.dispatched_this_action=True

    def settled(self,deadline,cancelled):
        tracker=SettleTracker(float(self.node.get_parameter('settle_vel_tol').value),
            float(self.node.get_parameter('settle_hold_time').value))
        began_ros=self.node.get_clock().now().nanoseconds
        began=None;last=-1
        stop=min(deadline,time.monotonic()+float(self.node.get_parameter('settle_timeout').value))
        while time.monotonic()<stop and not cancelled():
            sample=self.parent.ports.joints();now=self.node.get_clock().now().nanoseconds
            if (sample.stamp_valid and sample.stamp_ns>=began_ros and sample.stamp_ns>last
                    and 0<=now-sample.stamp_ns<=self.parent.motion.max_joint_age_ns):
                if began is None:began=sample.stamp_ns
                last=sample.stamp_ns
                tracker.update((last-began)/1e9,{},dict(zip(OPERATION_JOINTS,sample.positions)))
                self.parent.record('pickup_view_settle.json',json.dumps(dict(stamp_ns=last,
                    q=sample.positions,stable=tracker.settled_at is not None)).encode())
                if tracker.settled_at is not None:return last
            time.sleep(.01)
        raise ValueError('PICKUP_VIEW_SETTLE_NOT_CONFIRMED')

    def execute(self,handle):
        parent=self.parent
        result=self.action.Result(motion_terminal=True)
        if not parent._active.acquire(blocking=False):
            result.reason='OPERATION_BUSY';handle.abort();return result
        action_owned=False
        began=time.monotonic()
        try:
            if parent.session is not None:raise ValueError('OPERATION_UNRESOLVED')
            parent.session=parent.owner.begin('empty_pre_pick_view',request_key=handle)
            if parent.session is None:raise ValueError('SHARED_MOTION_BUSY')
            action_owned=True
            self.began=began;self.task_id=handle.request.pickup_task_id
            self.dispatched_this_action=False;self.unexpected_load=False;self.load_checkpointed=False
            self.unexpected_load_sample=None
            self.owns_session=True;parent.handle=handle
            parent.directory=parent.evidence_root/(str(time.monotonic_ns())+'_pickup_view')
            parent.directory.mkdir(exist_ok=False);parent.records=[];parent.bytes=0
            parent.record('pickup_view_execute_goal',handle.request)
            self.ledger.open(self.task_id,handle.request.preparation.domain_sha256,
                max_motions=int(handle.request.max_extra_view_motions),
                total_budget_s=handle.request.total_recovery_budget_s)
            deadline=self.began+min(handle.request.budget_s,self.ledger.remaining(self.task_id))
            cancelled=lambda:handle.is_cancel_requested
            self.checkpoint('WAIT_EMPTY_STATE')
            while not self.empty():
                if cancelled() or time.monotonic()>=deadline:
                    raise ValueError('PICKUP_VIEW_EMPTY_STATE_UNCONFIRMED')
                with self.vacuum_lock:sample=self.vacuum
                if sample is not None and sample[0]>=self.began and any(sample[1:]):
                    raise ValueError('PICKUP_VIEW_LOADED_OR_VACUUM_ON')
                time.sleep(.01)
            self.record_empty_state('empty_admission')
            def stopped():return cancelled() or not self.empty() or self.unexpected_load
            parent.ports.rpc.cancelled=stopped
            self.checkpoint('PREPARE_FROM_ACTUAL_JOINTS')
            result.plan=self.ports.prepare(handle.request.preparation,deadline=deadline,cancelled=stopped)
            self.checkpoint('RECERTIFY_AND_EXECUTE')
            outcome=parent.motion.execute_pickup_view(self.ports.leg,
                model_sha256=parent.ports.model.model_sha256,
                validate_current=lambda leg:self.ports.validate(leg,deadline=deadline,cancelled=stopped),
                source_current=lambda:not stopped() and self.ports.current(deadline),
                read_joints=parent.ports.joints,deadline=deadline,cancelled=stopped,
                on_dispatch=self.charge_dispatch)
            result.submitted,result.motion_terminal=outcome.submitted,outcome.terminal
            if not outcome.terminal:parent._uncertain=True
            if not outcome.success:raise ValueError(outcome.reason)
            self.checkpoint('SETTLING')
            result.settled_stamp_ns=self.settled(deadline,stopped)
            result.settled_stamp_valid=True;result.success=True;result.reason='PICKUP_VIEW_MOTION_SETTLED'
        except Exception as error:
            result.reason='PICKUP_VIEW_FAILED:'+str(error)
            if action_owned:
                try:
                    parent.record('pickup_view_failure.json',json.dumps(dict(
                        phase=self.phase,reason=result.reason,traceback=traceback.format_exc())).encode())
                except Exception as capture_error:
                    result.reason+=';FAILURE_CAPTURE_FAILED:'+str(capture_error)
        finally:
            try:
                if handle.request.pickup_task_id in self.ledger.rows:
                    result.dispatched_motions,result.recovery_elapsed_s=self.ledger.status(handle.request.pickup_task_id)
                if action_owned:
                    self.record_empty_state('action_terminal_or_hold')
                    result.may_have_payload=self.unexpected_load
                    if self.unexpected_load:
                        result.success=False;result.reason+=';UNEXPECTED_LOAD_RECONCILIATION_REQUIRED'
                    if parent.owner.motion_lock.locked() or parent.ports.rpc.unresolved() or self.unexpected_load:
                        parent._uncertain=True;result.motion_terminal=False
                        # A received controller terminal stays terminal even if
                        # unexpected load requires holding the task ownership.
                        if self.unexpected_load and not parent.owner.motion_lock.locked():
                            result.motion_terminal=True
                        self.checkpoint('UNRESOLVED')
                    else:self.release()
                    parent.record('pickup_view_execute_result',result)
                if result.success:handle.succeed()
                elif handle.is_cancel_requested and result.motion_terminal:handle.canceled()
                else:handle.abort()
            except Exception as error:
                parent._uncertain=True;result.success=False
                result.reason+=';FINALIZATION_UNRESOLVED:'+str(error)
                handle.abort()
            finally:parent._active.release()
        return result

    def release(self):
        parent=self.parent
        parent.journal.write(None)
        if not parent.owner.finish(parent.session):raise ValueError('PICKUP_VIEW_OWNER_UNRESOLVED')
        parent.session=None;parent._uncertain=False;self.owns_session=False;self.phase='IDLE'

    def poll_terminal(self):
        """Called under parent's operation lock; never run whole-plan cleanup."""
        parent=self.parent
        terminal=parent.motion.poll_terminal()
        if terminal is not None:
            parent.record('pickup_view_late_terminal.json',json.dumps(dict(success=terminal.success,
                terminal=terminal.terminal,reason=terminal.reason)).encode())
        if self.unexpected_load and not self.load_checkpointed:
            self.record_empty_state('late_unexpected_load')
            parent.journal.write(dict(kind='pickup_view',stage='UNEXPECTED_LOAD_RECONCILIATION_REQUIRED',
                pickup_task_id=self.task_id,may_have_payload=True,
                motion_owned=parent.owner.motion_lock.locked()))
            parent.record('pickup_view_unexpected_load.json',json.dumps(dict(pickup_task_id=self.task_id,
                automatic_loaded_recovery=False,operator_reconciliation_required=True)).encode())
            self.load_checkpointed=True
        if (not parent.owner.motion_lock.locked() and not parent.ports.rpc.unresolved()
                and not self.unexpected_load):self.release()
