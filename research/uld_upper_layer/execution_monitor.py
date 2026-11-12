"""Phase-aware physical monitors; no state mutation outside a private FCL scene.

Every accepted sample uses native physics-step joints/payload pose and checked
lower/contact stamps. Geometry remains separate from native contact evidence.
This monitor neither changes cargo poses nor supplies force/torque feedback.
"""
from collections import deque
from contextlib import nullcontext
from dataclasses import asdict,is_dataclass
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation
from observed_support import measure_observed_support
from contact_validation import validate_declared_contacts


def pose_error(reference,observed):
    a=np.asarray(reference).reshape(4,4);b=np.asarray(observed).reshape(4,4)
    return float(np.linalg.norm(b[:3,3]-a[:3,3])),float(np.rad2deg(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude()))


class ExecutionMonitor:
    def __init__(self,reader,scene,state,action,config,policy,initial,trace,panel_collision_name=None,
                 failure_capture=None,timing=None):
        self._reader=reader;self._scene=scene;self._state=state;self._action=action
        self._config=config;self._policy=policy;self._trace=trace
        self._failure_capture=failure_capture
        self._timing=timing
        self._baseline={b.name:b.container_T_body for b in initial.lower}
        self._initial_stamp=initial.stamp_s;self._started=time.monotonic()
        self._last=None;self._phase='initial';self._expected_attached=initial.attached
        self._contact_mode=bool(initial.attached);self._allow_support=True;self._detached_commands=initial.pose_command_count
        self._push_history=deque();self._ring=deque();self._failure=None
        self._support=None;self._sample=initial;self._contact_since=None
        self._panel_collision_name=panel_collision_name
        self._max_monitor_gap_s=0.;self._samples=0
        self._prior_attached=initial.attached
        self._lower_maxima={body.name:dict(translation_m=0.,rotation_deg=0.) for body in initial.lower}
        if initial.attached:
            if initial.tool_T_box_at_attach is None:
                raise ValueError('CONFIRMED_ATTACHMENT_RELATION_REQUIRED')
            expected=np.linalg.inv(np.array(action.box_T_tool).reshape(4,4))
            distance,angle=pose_error(expected,initial.tool_T_box_at_attach)
            if distance>config.contact_tolerance_m or angle>policy.attachment_rotation_error_deg:
                raise ValueError('INITIAL_ATTACHMENT_DIFFERS_FROM_PLAN')
        scene.set_payload(state.dimensions,'world',scene.world_container@np.array(initial.container_T_box).reshape(4,4),allow_support=True)

    def bind_held_action(self,record):
        """Rebind a newly scored action only at the monitored attached hold boundary."""
        from planning_types import action_from_record
        if self._phase!='planning_hold' or not self._sample.attached:
            raise ValueError('ACTION_REBIND_REQUIRES_ATTACHED_PLANNING_HOLD')
        action=action_from_record(record)
        distance,angle=pose_error(np.linalg.inv(np.array(action.box_T_tool).reshape(4,4)),
                                  self._sample.tool_T_box_at_attach)
        if distance>self._config.contact_tolerance_m or angle>self._policy.attachment_rotation_error_deg:
            raise ValueError('LIVE_ACTION_ATTACHMENT_CHANGED')
        self._action=action

    @property
    def sample(self):return self._sample

    @property
    def support(self):return self._support

    @property
    def failure(self):return self._failure

    @property
    def failure_stamp_s(self):
        return self._failure['stamp_s'] if self._failure else self._sample.stamp_s

    def _declared_contacts(self,sample,*,record_intended=True):
        return validate_declared_contacts(sample,self._state.dimensions,self._config,
            self._policy,self._allow_support,self._contact_mode,self._panel_collision_name,self._scene,
            record_intended=record_intended)

    def observe_native_contacts(self,row):
        """Check each raw native step without advancing the 50Hz scene monitor.

        Reuse the normalized reader and identical contact rule. This adds no
        contact-force feedback or commands. Failure retains the exact input.
        """
        if self._failure:return self._failure['reason']
        stamp=row.get('vacuum',{}).get('sim_time_s',self._sample.stamp_s)
        try:
            sample=self._reader.read(row)
            failure,_=self._declared_contacts(sample,record_intended=False)
        except (ValueError,KeyError,TypeError) as exc:
            return self._stop('PHYSICAL_OBSERVATION_INVALID',detail=str(exc),
                offending_stamp_s=stamp,invalid_observation=row)
        if failure:
            self._ring.append((sample.stamp_s,row))
            return self._stop(failure['reason'],offending_stamp_s=sample.stamp_s,
                invalid_observation=row,**{k:v for k,v in failure.items() if k!='reason'})
        return None

    def begin_phase(self,name,contact_mode=False,attached=False,allow_support=True):
        self._phase=name;self._contact_mode=contact_mode
        self._expected_attached=attached;self._allow_support=allow_support
        self._push_history.clear();self._contact_since=None
        self._trace.write(json.dumps(dict(event='phase',phase=name,stamp_s=self._sample.stamp_s,
            contact_mode=contact_mode,expected_attached=attached,allow_support=allow_support))+'\n');self._trace.flush()

    def confirm_detached(self,native):
        if native['attached']:raise ValueError('DETACH_NOT_CONFIRMED')
        self._detached_commands=int(native['pose_command_count'])

    def _stop(self,reason,**details):
        if self._failure is None:
            stamp=details.pop('offending_stamp_s',self._sample.stamp_s)
            self._failure=dict(reason=reason,phase=self._phase,stamp_s=stamp,
                               full_monitor_stamp_s=self._sample.stamp_s,**details)
            # Preserve the native ring at detection, before controller cancel
            # or formatting a potentially large invalid observation.
            if self._failure_capture:self._failure_capture(stamp)
            self._trace.write(json.dumps(dict(event='failure',**self._failure))+'\n');self._trace.flush()
        return self._failure['reason']

    def fail_execution(self,reason,**details):
        """Record controller/executor failures at the same observed phase boundary."""
        return self._stop(reason,**details)

    def observe(self,row):
        if self._failure:return self._failure['reason']
        try:sample=self._reader.read(row)
        except (ValueError,KeyError,TypeError) as exc:return self._stop('PHYSICAL_OBSERVATION_INVALID',detail=str(exc),invalid_observation=row)
        if not self._ring or sample.stamp_s>self._ring[-1][0]:self._ring.append((sample.stamp_s,row))
        while self._ring and sample.stamp_s-self._ring[0][0]>self._policy.trigger_pre_s:self._ring.popleft()
        if self._last is not None:
            delta=sample.stamp_s-self._last
            if delta < -1e-9:return self._stop('SIM_TIME_WENT_BACKWARDS')
            changed=sample.attached!=self._sample.attached or sample.processed_commands!=self._sample.processed_commands
            if delta < self._policy.monitor_period_s-1e-9 and not changed:return None
            self._max_monitor_gap_s=max(self._max_monitor_gap_s,delta)
            if delta>self._policy.monitor_max_gap_s+1e-9:return self._stop('MONITOR_RATE_INSUFFICIENT',gap_s=delta,offending_stamp_s=sample.stamp_s)
        self._last=sample.stamp_s;self._sample=sample;self._samples+=1
        if (sample.stamp_s-self._initial_stamp>self._policy.sim_budget_s or
                time.monotonic()-self._started>self._policy.wall_budget_s):return self._stop('PHYSICAL_BUDGET_EXHAUSTED')
        if self._expected_attached is not None and sample.attached!=self._expected_attached:
            return self._stop('ATTACHMENT_STATE_MISMATCH')
        if self._prior_attached and not sample.attached and self._expected_attached is None:
            self._detached_commands=sample.pose_command_count
        self._prior_attached=sample.attached
        if not sample.attached and sample.pose_command_count!=self._detached_commands:
            return self._stop('DETACHED_PAYLOAD_POSE_COMMAND')
        if not sample.attached and sample.payload_pose_command_component_present is True:
            return self._stop('DETACHED_PAYLOAD_POSE_COMMAND_COMPONENT_REMAINS')
        if not sample.attached and sample.attachment_joint_component_present is True:
            return self._stop('DETACHED_ATTACHMENT_CONSTRAINT_REMAINS')
        if sample.attached and sample.tool_T_box_at_attach is not None:
            actual=np.linalg.inv(np.array(sample.container_T_tool).reshape(4,4))@np.array(sample.container_T_box).reshape(4,4)
            distance,angle=pose_error(sample.tool_T_box_at_attach,actual)
            if distance>self._config.contact_tolerance_m or angle>self._policy.attachment_rotation_error_deg:
                return self._stop('ATTACHED_BOX_RELATION_ERROR',distance_m=distance,angle_deg=angle)
        for body in sample.lower:
            distance,angle=pose_error(self._baseline[body.name],body.container_T_body)
            self._lower_maxima[body.name]['translation_m']=max(self._lower_maxima[body.name]['translation_m'],distance)
            self._lower_maxima[body.name]['rotation_deg']=max(self._lower_maxima[body.name]['rotation_deg'],angle)
            if distance>self._policy.lower_translation_m or angle>self._policy.lower_rotation_deg:
                return self._stop('LOWER_CARGO_MOVED',body=body.name,distance_m=distance,angle_deg=angle)
            self._scene.update_world_transform(body.name,self._scene.world_container@np.array(body.container_T_body).reshape(4,4))
        self._scene.payload['world_pose']=self._scene.world_container@np.array(sample.container_T_box).reshape(4,4)
        self._scene.payload['allow_support']=self._allow_support
        failure,intended=self._declared_contacts(sample)
        if failure:return self._stop(failure['reason'],**{k:v for k,v in failure.items() if k!='reason'})
        with self._timing.stage('collision_check',sample.stamp_s) if self._timing else nullcontext():
            failure=self._scene.check_state(sample.q,self._config.collision_margin_m,self._contact_mode)
        if failure:return self._stop('OBSERVED_COLLISION_OR_CLEARANCE',diagnostic=failure)
        self._support=measure_observed_support(sample,self._state.dimensions,self._state.com_local,
            self._config.contact_tolerance_m,self._config.contact_penetration_m)
        if self._support.native_max_depth_m>self._config.contact_penetration_m:
            return self._stop('LOWER_CONTACT_OVERPENETRATION',depth_m=self._support.native_max_depth_m)
        if self._phase=='push':
            if not self.support_ready():
                return self._stop('PUSH_SUPPORT_BELOW_POLICY',
                    area_ratio=self._support.near_contact_area_ratio,
                    com_margin_m=self._support.com_margin_m,
                    native_lower_body_ids=self._support.native_lower_body_ids)
            reason=self._check_push(sample)
            if reason:return reason
        record=dict(event='observation',phase=self._phase,stamp_s=sample.stamp_s,q=sample.q,
            container_T_box=sample.container_T_box,container_T_tool=sample.container_T_tool,
            attached=sample.attached,pose_command_count=sample.pose_command_count,
            intended_contact_trace=intended,
            support=asdict(self._support),input_observation=(asdict(row) if is_dataclass(row) else row))
        with self._timing.stage('observation_write',sample.stamp_s) if self._timing else nullcontext():
            self._trace.write(json.dumps(record)+'\n');self._trace.flush()
        return None

    def _check_push(self,sample):
        start=np.array(self._action.handoff_box_pose).reshape(4,4)
        observed=np.array(sample.container_T_box).reshape(4,4)
        direction=np.asarray(self._action.push_direction);delta=observed[:3,3]-start[:3,3]
        lateral=float(np.linalg.norm(delta-direction*np.dot(delta,direction)))
        _,angle=pose_error(start,observed)
        if lateral>self._policy.push_lateral_error_m or angle>self._policy.push_rotation_error_deg:
            return self._stop('PUSH_REFERENCE_ERROR',lateral_m=lateral,angle_deg=angle)
        tool=np.array(sample.container_T_tool).reshape(4,4)
        self._push_history.append((sample.stamp_s,float(tool[:3,3]@direction),float(observed[:3,3]@direction)))
        while len(self._push_history)>1 and sample.stamp_s-self._push_history[1][0]>=self._policy.no_progress_window_s:
            self._push_history.popleft()
        old=self._push_history[0]
        if sample.stamp_s-old[0]>=self._policy.no_progress_window_s:
            tool_advance=self._push_history[-1][1]-old[1];box_advance=self._push_history[-1][2]-old[2]
            if tool_advance>=self._policy.no_progress_tool_advance_m and box_advance<self._policy.no_progress_box_advance_m:
                return self._stop('PUSH_NO_PROGRESS',tool_advance_m=tool_advance,box_advance_m=box_advance)
        return None

    def support_ready(self):
        support=self._support
        return bool(support and support.near_contact_area_ratio>=self._config.support_area_ratio_min and
            support.com_margin_m>=self._config.support_com_margin_m and support.native_lower_body_ids and
            support.native_max_depth_m<=self._config.contact_penetration_m)

    def tool_separated(self):
        # Recheck without a contact exemption. A completed retreat command
        # alone is not evidence that the plate stopped supporting the box.
        return self._scene.check_payload(contact_tool=False,margin=self._config.collision_margin_m) is None

    def save_trigger(self,path,tail):
        import gzip
        with gzip.open(path,'wt',compresslevel=3) as stream:
            for stamp,row in list(self._ring)+tail:
                stream.write(json.dumps(dict(stamp_s=stamp,observation=row))+'\n')

    def metrics(self):
        return dict(samples=self._samples,max_monitor_gap_s=self._max_monitor_gap_s,
                    lower_maxima={name:dict(values) for name,values in self._lower_maxima.items()},
                    failure=self._failure,physical_execution='not_certified_by_monitor_alone')
