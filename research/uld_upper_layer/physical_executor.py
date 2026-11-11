"""Real arm-controller execution with explicit seal/release/push gates.

The input is a complete certified nominal plan. This adapter never sends a
payload pose or plans a missing stage while carrying the box. Completion here
is a nominal physics probe; robust-tube/calibration/campaign gates are separate.
"""
import time
import numpy as np
from controller_io import ArmController
from execution_monitor import pose_error


STAGES=('pick_pre','pick_contact','pick_lift','invert_and_approach','insert_up',
        'lower_to_support','release_down','withdraw','push_pre','push_contact','push','retreat')


class PhysicalExecutor:
    def __init__(self,runtime,monitor,observe,dimensions,config,policy):
        self._runtime=runtime;self._monitor=monitor;self._config=config;self._policy=policy
        self._controller=ArmController(runtime)
        self._observe=observe
        self._dimensions=tuple(dimensions)
        self._deadline=time.monotonic()+policy.wall_budget_s

    def _pump(self):
        import rclpy
        rclpy.spin_once(self._runtime.node,timeout_sec=self._policy.monitor_period_s)
        reason=self._observe(self._runtime.snapshot())
        if reason:raise RuntimeError(reason)
        if time.monotonic()>self._deadline:raise TimeoutError('PHYSICAL_EXECUTOR_WALL_BUDGET')

    def _dwell(self,predicate,duration):
        started=None;begin=self._monitor.sample.stamp_s
        while True:
            self._pump();stamp=self._monitor.sample.stamp_s
            if predicate():
                if started is None:started=stamp
                if stamp-started>=duration:return
            else:started=None
            if stamp-begin>self._policy.condition_wait_budget_s:
                raise TimeoutError('CONTACT_OR_STABILITY_DWELL_NOT_ESTABLISHED')

    def _seal_ready(self):
        sample=self._monitor.sample
        box=np.array(sample.container_T_box).reshape(4,4)
        relative=np.linalg.inv(box)@np.array(sample.container_T_tool).reshape(4,4)
        normal=relative[:3,2];axis=int(np.argmax(np.abs(normal)));sign=np.sign(normal[axis])
        gap=-sign*(relative[axis,3]+sign*self._dimensions[axis]/2)
        # Footprint validity is already a source-mesh scene check. Gap proves
        # the ideal seal reference actually reached the box, unlike a waiver
        # along the entire preceding contact approach.
        return abs(normal[axis])>=1-1e-7 and -self._config.contact_penetration_m<=gap<=self._config.contact_tolerance_m

    def _plate_contact(self):
        identity=self._runtime.panel_contact['collision_name']
        return any(contact.other_collision==identity and contact.depth_m<=self._config.contact_penetration_m
                   for contact in self._monitor.sample.contacts)

    def _stable(self,reference):
        distance,angle=pose_error(reference,self._monitor.sample.container_T_box)
        if distance>self._policy.stability_translation_m or angle>self._policy.stability_rotation_deg:
            raise RuntimeError('RELEASED_BOX_UNSTABLE')
        return self._monitor.tool_separated()

    def execute(self,plan):
        if not plan.get('valid') or tuple(stage['name'] for stage in plan['stages'])!=STAGES:
            raise ValueError('COMPLETE_NOMINAL_STAGE_CHAIN_REQUIRED')
        if not all(stage['certificate'].get('valid') for stage in plan['stages']):
            raise ValueError('UNPROVEN_NOMINAL_STAGE')
        action=plan['action'];detached_reference=None
        for stage in plan['stages']:
            name=stage['name'];attached=name in ('pick_lift','invert_and_approach','insert_up','lower_to_support')
            if name=='pick_lift':
                self._dwell(self._seal_ready,self._policy.contact_dwell_s)
                self._monitor.begin_phase('attach_transition','seal',None,True)
                self._runtime.toggle_vacuum(True,monitor=self._observe)
            if name=='release_down':
                self._dwell(self._monitor.support_ready,self._policy.contact_dwell_s)
                self._monitor.begin_phase('detach_transition','seal',None,True)
                state=self._runtime.toggle_vacuum(False,monitor=self._observe)
                self._monitor.confirm_detached(state)
                detached_reference=self._monitor.sample.container_T_box
            if name=='push_pre':
                self._dwell(lambda:self._stable(detached_reference),self._policy.stability_dwell_s)
            if name=='push':self._dwell(self._plate_contact,self._policy.contact_dwell_s)
            self._monitor.begin_phase(name,stage['contact_tool'],attached,
                name not in ('invert_and_approach',))
            self._controller.execute(stage['trajectory'],self._observe,
                                     max(0.,self._deadline-time.monotonic()))
            self._pump()
        distance,angle=pose_error(action['placement']['final_box_pose'],self._monitor.sample.container_T_box)
        if distance>self._policy.final_translation_m or angle>self._policy.final_rotation_deg:
            raise RuntimeError('FINAL_BOX_POSE_OUTSIDE_TOLERANCE')
        final_reference=self._monitor.sample.container_T_box
        self._dwell(lambda:self._stable(final_reference),self._policy.stability_dwell_s)
        return dict(status='nominal_physics_probe_pass',final_position_error_m=distance,final_rotation_error_deg=angle,
                    integration_acceptance='not_evaluated_robust_tube_calibration_and_campaign_pending')
