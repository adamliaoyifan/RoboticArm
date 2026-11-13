"""Complete-operation Action adoption under the host's shared motion owner.

Original timed knots are the only motion payload. An unresolved controller/RPC
or carrying fault retains ownership. The durable journal also refuses dispatch
after a process restart with an unresolved operation; it never releases vacuum.
"""
import copy
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time

from luggage_planning.joint_operation.execution import JointOperationExecution
from luggage_planning.joint_operation.motion import StrictOperationMotionPort
from luggage_planning.joint_operation.map_source import map_source_to_message
from luggage_planning.joint_operation.runtime_ports import OperationRuntimePorts
from luggage_planning.motion.settle_criterion import SettleTracker
from luggage_planning.planning_contract.attachment import ATTACHMENT_MAX_AGE_NS


class OperationJournal:
    def __init__(self, path, *, capacity, max_acquisition_age_ns=ATTACHMENT_MAX_AGE_NS):
        if not path or type(capacity) is not int or capacity<1:
            raise ValueError('EXPLICIT_OPERATION_JOURNAL_AND_CAPACITY_REQUIRED')
        if type(max_acquisition_age_ns) is not int or max_acquisition_age_ns <= 0:
            raise ValueError('POSITIVE_JOURNAL_ACQUISITION_WINDOW_REQUIRED')
        self.max_acquisition_age_ns = max_acquisition_age_ns
        self.path,self.capacity = Path(path),capacity
        self.path.parent.mkdir(parents=True,exist_ok=True)
        self.value = json.loads(self.path.read_text()) if self.path.exists() else dict(active=None,used=[])
        if (set(self.value)!= {'active','used'} or not isinstance(self.value['used'],list)
                or len(self.value['used'])>capacity):raise ValueError('INVALID_OPERATION_JOURNAL')
        self.lock = threading.RLock()

    def write(self, active):
        with self.lock:
            self.value['active']=active
            raw=json.dumps(self.value,sort_keys=True,allow_nan=False).encode()
            temporary=self.path.with_name(self.path.name+'.pending')
            with temporary.open('wb') as stream:
                stream.write(raw);stream.flush();os.fsync(stream.fileno())
            os.replace(temporary,self.path)
            fd=os.open(str(self.path.parent),os.O_DIRECTORY)
            try:os.fsync(fd)
            finally:os.close(fd)

    def used(self,digest):
        return any(row['digest']==digest for row in self.value['used'])

    def finish(self,plan,now_ns):
        # Expiry uses original physical observation age, never storage TTL.
        used=[row for row in self.value['used'] if now_ns-row['stamp_ns']<=self.max_acquisition_age_ns]
        if not any(row['digest']==plan.plan_digest for row in used):
            if len(used)>=self.capacity:raise ValueError('OPERATION_JOURNAL_CAPACITY')
            used.append(dict(digest=plan.plan_digest,stamp_ns=plan.identity.observation_stamp_ns))
        self.value['used']=used
        self.write(None)


class JointOperationRuntime:
    def __init__(self,node):
        from rclpy.action import ActionServer,GoalResponse,CancelResponse
        from luggage_msgs.action import ExecuteJointOperationV2
        self.node,self.owner=node,node._motion_owner
        def param(name,default):
            return (node.get_parameter(name) if node.has_parameter(name)
                    else node.declare_parameter(name,default)).value
        from luggage_planning.joint_operation.preparation_profile import (
            DEFAULT_MAX_PREPARATION_BUDGET_S, execution_observation_age_limit_ns)
        contact_admission_age_ns=execution_observation_age_limit_ns(
            float(param('max_preparation_budget_s',DEFAULT_MAX_PREPARATION_BUDGET_S)))
        self.journal=OperationJournal(str(param('operation_state_file','')),
            capacity=int(param('operation_completed_limit',128)),
            max_acquisition_age_ns=contact_admission_age_ns)
        self.max_budget=float(param('operation_max_budget_s',600.))
        self.cleanup_budget=float(param('operation_cleanup_timeout_s',5.))
        self.refresh_attempts=int(param('operation_source_refresh_attempts',2))
        if not 1<=self.refresh_attempts<=8:raise ValueError('BOUNDED_SOURCE_REFRESH_ATTEMPTS_REQUIRED')
        if not all(math.isfinite(v) and v>0 for v in (self.max_budget,self.cleanup_budget)):
            raise ValueError('FINITE_OPERATION_BUDGETS_REQUIRED')
        self.evidence_root=Path(param('operation_evidence_dir',str(self.journal.path.parent/'operation_evidence')))
        self.evidence_root.mkdir(parents=True,exist_ok=True)
        self._record_lock=threading.RLock()
        self.directory=None;self.records=[];self.bytes=0
        self.evidence_limit=int(param('operation_evidence_limit_bytes',128*1024*1024))
        if self.evidence_limit<1:raise ValueError('POSITIVE_OPERATION_EVIDENCE_CAP_REQUIRED')
        from luggage_planning.joint_operation.path_cache import SuccessfulPathCache
        cache_directory=str(param('successful_path_cache_directory',
            str(Path.home()/'.ros'/'luggage_successful_paths')))
        self.path_cache=SuccessfulPathCache(cache_directory) if cache_directory else None
        self.ports=OperationRuntimePorts(node,record=self.record)
        self.motion=StrictOperationMotionPort(node,motion_lock=self.owner.motion_lock,
            start_tolerance_rad=self.ports.start_tolerance,
            end_tolerance_rad=float(param('operation_endpoint_joint_tolerance_rad',.01)),
            max_joint_age_ns=int(param('operation_joint_max_age_ns',1_000_000_000)),recorder=self.record)
        max_views=int(param('operation_post_place_views',0))
        if not 0<=max_views<=3:raise ValueError('BOUNDED_POST_PLACE_VIEW_COUNT_REQUIRED')
        self.explorer=None
        if max_views:
            from luggage_planning.joint_operation.post_place_explore import PostPlaceExplorer
            self.explorer=PostPlaceExplorer(node,self.ports,self.motion,max_views=max_views,
                refresh_attempts=self.refresh_attempts,settled=self.settled,
                checkpoint=self.checkpoint,record=self.record)
        self.machine=JointOperationExecution(start_tolerance_rad=self.ports.start_tolerance)
        self.session=self.handle=None
        self._active=threading.Lock()
        self._uncertain=False
        self._restored_unresolved=False
        self.pending_motion_command=None
        if self.journal.value['active'] is not None:
            self.session=self.owner.begin('unresolved_previous_joint_operation')
            self._uncertain=True
            self._restored_unresolved=True
        self.action=ExecuteJointOperationV2
        def accept(goal):
            return (GoalResponse.ACCEPT if math.isfinite(goal.budget_s) and 0<goal.budget_s<=self.max_budget
                    and not self._active.locked() and self.session is None else GoalResponse.REJECT)
        def cancel(handle):
            if self.handle is handle:
                self.owner.request_stop(handle)
                self.motion.cancel()
            return CancelResponse.ACCEPT
        self.server=ActionServer(node,ExecuteJointOperationV2,'/joint_operation/execute',
            goal_callback=accept,cancel_callback=cancel,execute_callback=self.execute,
            callback_group=node._group)
        self.timer=node.create_timer(.05,self.poll_terminal,callback_group=node._group)

    def record(self,name,value):
        from rclpy.serialization import serialize_message
        with self._record_lock:
            if self.directory is None:raise ValueError('OPERATION_EVIDENCE_NOT_OPEN')
            raw=value if isinstance(value,bytes) else serialize_message(value)
            stored=gzip.compress(raw,compresslevel=1,mtime=0)
            if self.bytes+len(stored)>self.evidence_limit:raise ValueError('OPERATION_EVIDENCE_CAP')
            file='%05d_'%len(self.records)+name.replace('/','_')+'.gz'
            (self.directory/file).write_bytes(stored)
            self.records.append(dict(file=file,sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw),
                encoding='gzip',stored_bytes=len(stored),stored_sha256=hashlib.sha256(stored).hexdigest()))
            self.bytes+=len(stored)
            (self.directory/'manifest.json').write_text(json.dumps(dict(records=self.records,
                bytes=self.bytes,phase=self.machine.state,may_have_payload=self.machine.may_have_payload,
                motion_owned=self.owner.motion_lock.locked()),sort_keys=True))

    def checkpoint(self,stage):
        plan=self.machine.plan
        self.journal.write(dict(stage=stage,plan_digest=plan.plan_digest if plan else self.handle.request.plan.plan_digest,
            may_have_payload=self.machine.may_have_payload,motion_owned=self.owner.motion_lock.locked()))
        feedback=self.action.Feedback(request_id=plan.identity.request_id if plan else self.handle.request.plan.request_id,
            stage=stage,next_leg_index=self.machine.next_leg_index)
        self.handle.publish_feedback(feedback)
        self.record('stage.json',json.dumps(dict(stage=stage,ros_stamp_ns=self.node.get_clock().now().nanoseconds,
            plan_digest=plan.plan_digest if plan else '',index=self.machine.next_leg_index,
            may_have_payload=self.machine.may_have_payload),sort_keys=True).encode())

    def settled(self):
        node=self.node
        tracker=SettleTracker(float(node.get_parameter('settle_vel_tol').value),
                              float(node.get_parameter('settle_hold_time').value))
        began_ros=node.get_clock().now().nanoseconds
        began=None;last=-1
        stop=min(self.ports.deadline,time.monotonic()+float(node.get_parameter('settle_timeout').value))
        while time.monotonic()<stop and not self.ports.cancelled():
            sample=self.ports.joints();now=node.get_clock().now().nanoseconds
            if (sample.stamp_ns>=began_ros and sample.stamp_ns>last
                    and 0<=now-sample.stamp_ns<=self.motion.max_joint_age_ns):
                if began is None:began=sample.stamp_ns
                last=sample.stamp_ns
                positions=dict(zip(self.machine.plan.legs[0].joint_names,sample.positions))
                tracker.update((sample.stamp_ns-began)/1e9,{},positions)
                self.record('settle_sample.json',json.dumps(dict(stamp_ns=sample.stamp_ns,q=sample.positions,
                    stable=tracker.settled_at is not None)).encode())
                if tracker.settled_at is not None:return last
            time.sleep(.01)
        raise ValueError('OPERATION_SETTLE_NOT_CONFIRMED')

    def validate_with_refresh(self,index,*,deadline,cancelled):
        """Recheck an unexecuted suffix when a valid source changes mid-check.

        Event effects (especially vacuum release) remain outside this loop.
        Collision/identity refusals are final; each retry captures and validates
        the complete suffix against the new source within the same action budget.
        """
        for attempt in range(self.refresh_attempts):
            checked=self.ports.validate(index)
            self.record('barrier_validation_attempt.json',json.dumps(dict(index=index,
                attempt=attempt,accepted=checked.receipt.accepted,
                reason=checked.receipt.reason,retryable_source_change=self.ports.retryable_change)).encode())
            if (checked.receipt.accepted or not self.ports.retryable_change
                    or cancelled() or time.monotonic()>=deadline):
                break
        return checked

    def execute(self,handle):
        result=self.action.Result(outcome='STOPPED',motion_terminal=True)
        if not self._active.acquire(blocking=False):
            handle.abort();result.reason='OPERATION_BUSY';return result
        plan=None;command=None
        try:
            if self.session is not None or self.journal.used(handle.request.plan.plan_digest):
                raise ValueError('OPERATION_UNRESOLVED_OR_ALREADY_CONSUMED')
            self.session=self.owner.begin('complete_joint_operation',request_key=handle)
            if self.session is None:raise ValueError('SHARED_MOTION_BUSY')
            self.handle=handle
            self.machine=JointOperationExecution(start_tolerance_rad=self.ports.start_tolerance)
            self.directory=self.evidence_root/(str(time.monotonic_ns())+'_'+handle.request.plan.plan_digest[:16])
            self.directory.mkdir(exist_ok=False);self.records=[];self.bytes=0
            self.record('execute_goal',handle.request)
            self.checkpoint('ADMITTING_PREPARED_PLAN')
            deadline=time.monotonic()+handle.request.budget_s
            cancelled=lambda:handle.is_cancel_requested
            plan=self.ports.open(handle.request,deadline=deadline,cancelled=cancelled)
            command=self.machine.start(plan,plan.identity,plan.plan_digest)
            while command is not None:
                self.checkpoint(command.kind)
                if cancelled() and command.kind!='cancel_motion':
                    command=self.machine.cancel()
                    continue
                kind=command.kind
                if kind.startswith('validate_'):
                    checked=self.validate_with_refresh(command.leg_index,deadline=deadline,cancelled=cancelled)
                    command=self.machine.validation_finished(command.token,checked.receipt)
                elif kind=='execute_leg':
                    # Every leg gets an actual fresh-scene suffix check inside
                    # the strict original-trajectory executor, including legs
                    # between the pure consumer's explicit event barriers.
                    for attempt in range(self.refresh_attempts):
                        motion=self.motion.execute(plan,command.leg_index,expected_identity=plan.identity,
                            expected_digest=plan.plan_digest,
                            geometry=None if command.leg_index<2 else self.ports.geometry,
                            validate_current=lambda p,i,**kwargs:self.ports.validate(i),
                            source_current=lambda p,i:self.ports.source_current(self.ports.snapshot,
                                None if i<2 else self.ports.geometry),read_joints=self.ports.joints,
                            deadline=deadline,cancelled=cancelled)
                        self.record('strict_motion_outcome.json',json.dumps(dict(index=command.leg_index,
                            attempt=attempt,success=motion.success,terminal=motion.terminal,
                            submitted=motion.submitted,reason=motion.reason),sort_keys=True).encode())
                        if (motion.terminal and not motion.submitted and self.ports.retryable_change
                                and attempt+1<self.refresh_attempts and not cancelled()):
                            self.ports.capture()
                            continue
                        break
                    if not motion.terminal:
                        self.pending_motion_command=command
                        self.machine.event_failed(command.token,motion.reason)
                        self._uncertain=True
                        result.motion_terminal=False
                        break
                    command=self.machine.motion_finished(command.token,success=motion.success)
                    if not motion.success:self.machine.reason=motion.reason
                elif kind=='confirm_attachment':
                    self.settled()
                    self.checkpoint('VACUUM_EFFECT_PENDING')
                    actual=self.ports.attach()
                    command=self.machine.attachment_confirmed(command.token,record=actual.record,
                        geometry=actual.geometry,generation=actual.generation)
                elif kind=='confirm_release':
                    self.settled()
                    self.checkpoint('RELEASE_EFFECT_PENDING')
                    command=self.machine.release_confirmed(command.token,success=self.ports.release())
                elif kind=='apply_release_and_validate_retreat':
                    self.ports.apply_phase('retreat',self.ports.geometry)
                    checked=self.validate_with_refresh(command.leg_index,deadline=deadline,cancelled=cancelled)
                    command=self.machine.validation_finished(command.token,checked.receipt)
                elif kind=='cancel_motion':
                    self.motion.cancel()
                    self._uncertain=True
                    result.motion_terminal=False
                    break
                else:raise ValueError('UNKNOWN_OPERATION_EFFECT:'+kind)
            if self.machine.state=='COMPLETE':
                view_stamp=self.settled()
                self.ports.finalize_world()
                observed=self.ports.adopt_observed_release(view_stamp,
                    explore=self.explorer.observe if self.explorer is not None else None)
                result.final_map_source=map_source_to_message(observed.source)
                result.success=True
                result.outcome='COMPLETE'
                result.reason='OBSERVED_OPERATION_COMPLETE'
            else:
                result.outcome=self.machine.state
                result.reason=self.machine.reason or 'OPERATION_NOT_COMPLETE'
        except Exception as error:
            result.reason=type(error).__name__+':'+str(error)
            if command is not None:self.machine.event_failed(command.token,result.reason)
            if self.ports.released:
                self.machine.may_have_payload=False
            self._uncertain = (self._uncertain or self.owner.motion_lock.locked() or self.ports.rpc.unresolved())
            result.outcome='CARRY_FAULT' if self.machine.may_have_payload else 'STOPPED'
            result.motion_terminal=not self.owner.motion_lock.locked()
        finally:
            self.finish_action(handle,result,plan)
        return result

    def finish_action(self,handle,result,plan):
        """Report terminal status even when cleanup or evidence storage fails.

        A storage failure retains the motion session for reconciliation. The
        callback mutex must still be released so cancellation and terminal
        polling remain possible; it does not discharge motion ownership.
        """
        try:
            result.may_have_payload=self.machine.may_have_payload
            if self.session is not None:
                retain=(result.may_have_payload or self._uncertain or self.ports.rpc.unresolved())
                if not retain:
                    try:
                        if plan is not None:
                            self.ports.deadline=max(self.ports.deadline,time.monotonic()+self.cleanup_budget)
                            self.ports.rpc.cancelled=lambda:False
                            self.ports.finalize_world()
                            self.journal.finish(plan,self.node.get_clock().now().nanoseconds)
                        else:self.journal.write(None)
                        if not self.owner.finish(self.session):
                            raise ValueError('SHARED_MOTION_OWNERSHIP_UNRESOLVED')
                        self.session=None
                    except Exception as error:
                        result.success=False;result.reason+=';CLEANUP_UNRESOLVED:'+str(error)
                        self._uncertain=True
                if self.session is not None:
                    self.journal.write(dict(plan_digest=handle.request.plan.plan_digest,
                        stage=result.outcome,may_have_payload=result.may_have_payload,
                        motion_owned=self.owner.motion_lock.locked(),reason=result.reason))
            if result.success and plan is not None and self.path_cache is not None:
                try:
                    promoted=self.path_cache.remember(plan,success=result.success,
                        motion_terminal=result.motion_terminal,may_have_payload=result.may_have_payload)
                    self.record('successful_path_promotion',json.dumps(dict(promoted=promoted,
                        plan_digest=plan.plan_digest)).encode())
                except (OSError,ValueError) as error:
                    # Persistence is optional; it cannot undo completed motion.
                    self.node.get_logger().warning('PATH_CACHE_WRITE_FAILED:'+str(error))
            if self.directory is not None:self.record('execute_result',result)
        except Exception as error:
            self._uncertain=True
            result.success=False
            result.reason+=';FINALIZATION_UNRESOLVED:'+str(error)
        finally:
            try:
                if result.success:handle.succeed()
                elif handle.is_cancel_requested and result.motion_terminal:handle.canceled()
                else:handle.abort()
            finally:
                self._active.release()

    def poll_terminal(self):
        if not self._active.acquire(blocking=False):return
        try:
            if (self.session is None or not self._uncertain
                    or self._restored_unresolved):return
            terminal=self.motion.poll_terminal()
            if terminal is not None:
                self.record('late_motion_terminal.json',json.dumps(dict(success=terminal.success,
                    reason=terminal.reason,terminal=terminal.terminal)).encode())
            if terminal is not None and self.pending_motion_command is not None:
                self.machine.motion_finished(self.pending_motion_command.token,success=False)
                self.pending_motion_command=None
            if (self.owner.motion_lock.locked() or self.ports.rpc.unresolved()
                    or self.machine.may_have_payload):return
            self.ports.deadline=time.monotonic()+self.cleanup_budget
            self.ports.rpc.cancelled=lambda:False
            self.ports.finalize_world()
            if self.machine.plan is not None:
                self.journal.finish(self.machine.plan,self.node.get_clock().now().nanoseconds)
            else:self.journal.write(None)
            if self.owner.finish(self.session):
                self.session=None
                self._uncertain=False
        except Exception as error:
            self.record('late_cleanup_fault.json',json.dumps(dict(reason=str(error))).encode())
        finally:self._active.release()

    def shutdown(self):
        self.motion.cancel()
        if self.session is not None:
            self.journal.write(dict(stage='SHUTDOWN_UNRESOLVED',
                may_have_payload=self.machine.may_have_payload,
                motion_owned=self.owner.motion_lock.locked()))
