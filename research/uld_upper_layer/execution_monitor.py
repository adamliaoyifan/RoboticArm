"""Phase-aware physical monitors; no state mutation outside a private FCL scene.

Every accepted sample uses native physics-step joints/payload pose and checked
lower/contact stamps. Geometry remains separate from native contact evidence.
This monitor neither changes cargo poses nor supplies force/torque feedback.
"""
from collections import deque
from dataclasses import asdict
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation
from observed_support import measure_observed_support


def pose_error(reference,observed):
    a=np.asarray(reference).reshape(4,4);b=np.asarray(observed).reshape(4,4)
    return float(np.linalg.norm(b[:3,3]-a[:3,3])),float(np.rad2deg(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude()))


class ExecutionMonitor:
    def __init__(self,reader,scene,state,action,config,policy,initial,trace,panel_collision_name=None):
        self._reader=reader;self._scene=scene;self._state=state;self._action=action
        self._config=config;self._policy=policy;self._trace=trace
        self._baseline={b.name:b.container_T_body for b in initial.lower}
        self._initial_stamp=initial.stamp_s;self._started=time.monotonic()
        self._last=None;self._phase='initial';self._expected_attached=False
        self._contact_mode=False;self._allow_support=True;self._detached_commands=initial.pose_command_count
        self._push_history=deque();self._ring=deque();self._failure=None
        self._support=None;self._sample=initial;self._contact_since=None
        self._panel_collision_name=panel_collision_name
        self._max_monitor_gap_s=0.;self._samples=0
        self._prior_attached=initial.attached
        scene.set_payload(state.dimensions,'world',scene.world_container@np.array(initial.container_T_box).reshape(4,4),allow_support=True)

    @property
    def sample(self):return self._sample

    @property
    def support(self):return self._support

    @property
    def failure(self):return self._failure

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
            self._failure=dict(reason=reason,phase=self._phase,stamp_s=self._sample.stamp_s,**details)
            self._trace.write(json.dumps(dict(event='failure',**self._failure))+'\n');self._trace.flush()
        return self._failure['reason']

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
            if distance>self._policy.lower_translation_m or angle>self._policy.lower_rotation_deg:
                return self._stop('LOWER_CARGO_MOVED',body=body.name,distance_m=distance,angle_deg=angle)
            self._scene.update_world_transform(body.name,self._scene.world_container@np.array(body.container_T_body).reshape(4,4))
        self._scene.payload['world_pose']=self._scene.world_container@np.array(sample.container_T_box).reshape(4,4)
        self._scene.payload['allow_support']=self._allow_support
        failure=self._scene.check_state(sample.q,self._config.collision_margin_m,self._contact_mode)
        if failure:return self._stop('OBSERVED_COLLISION_OR_CLEARANCE',diagnostic=failure)
        intended=[]
        for contact in sample.contacts:
            if contact.other_collision!=self._panel_collision_name:continue
            if not self._contact_mode:return self._stop('FORBIDDEN_NATIVE_PANEL_CONTACT')
            mode=self._scene.contact_mode(self._contact_mode)
            frame=np.array(sample.container_T_tool).reshape(4,4)
            if mode=='push_plate':frame=frame@np.array(self._scene.push_geometry.tool_T_push_surface).reshape(4,4)
            point=np.linalg.inv(frame)@np.r_[contact.point_container,1.]
            normal=np.array(contact.normal_toward_payload_container)
            from shapely.geometry import Point
            patch=self._scene.contact_polygon(mode).buffer(self._config.contact_tolerance_m)
            alignment=float(normal@frame[:3,2])
            if (not patch.covers(Point(point[:2])) or abs(point[2])>self._config.contact_tolerance_m or
                alignment<np.cos(np.deg2rad(self._policy.contact_normal_alignment_deg)) or
                contact.depth_m>self._config.contact_penetration_m):
                return self._stop('NATIVE_CONTACT_OUTSIDE_DECLARED_PANEL_PATCH',point_contact_frame=point[:3].tolist(),normal_alignment=alignment,depth_m=contact.depth_m)
            intended.append(asdict(contact))
        self._support=measure_observed_support(sample,self._state.dimensions,self._state.com_local,
            self._config.contact_tolerance_m,self._config.contact_penetration_m)
        if self._support.native_max_depth_m>self._config.contact_penetration_m:
            return self._stop('LOWER_CONTACT_OVERPENETRATION',depth_m=self._support.native_max_depth_m)
        if self._phase=='push':
            reason=self._check_push(sample)
            if reason:return reason
        record=dict(event='observation',phase=self._phase,stamp_s=sample.stamp_s,q=sample.q,
            container_T_box=sample.container_T_box,container_T_tool=sample.container_T_tool,
            attached=sample.attached,pose_command_count=sample.pose_command_count,
            intended_contact_trace=intended,
            support=asdict(self._support))
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
        with path.open('w') as stream:
            for stamp,row in list(self._ring)+tail:
                stream.write(json.dumps(dict(stamp_s=stamp,observation=row))+'\n')

    def metrics(self):
        return dict(samples=self._samples,max_monitor_gap_s=self._max_monitor_gap_s,
                    failure=self._failure,physical_execution='not_certified_by_monitor_alone')
