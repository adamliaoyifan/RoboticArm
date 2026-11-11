"""Monitor policy behaviour with injected observation/collision boundaries."""
from pathlib import Path
from types import SimpleNamespace
from dataclasses import replace
import io
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from execution_monitor import ExecutionMonitor
from execution_policy import ExecutionPolicy
from physical_observation import PhysicalSample,ObservedBody
from planning_types import UpperConfig
from geometry import pose


class MonitorContracts(unittest.TestCase):
    def create(self,policy=None):
        self.initial=PhysicalSample(1.,(0.,)*6,tuple(pose([0,0,1.125]).ravel()),tuple(np.eye(4).ravel()),
            (ObservedBody('lower_0',(1.,1.,1.),tuple(pose([0,0,.5]).ravel()),1.),),(),1.,False,0,0)
        scene=SimpleNamespace(world_container=np.eye(4),payload={},
             set_payload=lambda *args,**kw:None,update_world_transform=lambda *args:None,check_state=lambda *args:None)
        self.scene=scene
        action=SimpleNamespace(handoff_box_pose=self.initial.container_T_box,push_direction=(1.,0.,0.))
        return ExecutionMonitor(SimpleNamespace(read=lambda row:row),scene,
            SimpleNamespace(dimensions=(1.,1.,.25),com_local=(0,0,0)),action,
            UpperConfig.load(),policy or ExecutionPolicy.load(),self.initial,io.StringIO())

    def test_actual_collision_failure_is_not_eligible_for_later_scoring(self):
        monitor=self.create();self.scene.check_state=lambda *args:dict(reason='SELF_COLLISION',pair=['camera','arm'])
        self.assertEqual(monitor.observe(self.initial),'OBSERVED_COLLISION_OR_CLEARANCE')
        self.assertEqual(monitor.failure['diagnostic']['pair'],['camera','arm'])

    def test_missing_monitor_frequency_and_lower_motion_stop(self):
        monitor=self.create();self.assertIsNone(monitor.observe(self.initial))
        self.assertEqual(monitor.observe(replace(self.initial,stamp_s=1.03)),'MONITOR_RATE_INSUFFICIENT')
        monitor=self.create();body=replace(self.initial.lower[0],container_T_body=tuple(pose([.004,0,.5]).ravel()))
        self.assertEqual(monitor.observe(replace(self.initial,lower=(body,))),'LOWER_CARGO_MOVED')

    def test_external_policy_threshold_changes_observed_decision(self):
        body=ObservedBody('lower_0',(1.,1.,1.),tuple(pose([.004,0,.5]).ravel()),1.)
        monitor=self.create(replace(ExecutionPolicy.load(),lower_translation_m=.005))
        self.assertIsNone(monitor.observe(replace(self.initial,lower=(body,))))

    def test_attachment_release_counter_is_frozen_after_detachment(self):
        monitor=self.create();monitor.begin_phase('carry','seal',True,False)
        self.assertIsNone(monitor.observe(replace(self.initial,attached=True,pose_command_count=10)))
        monitor.begin_phase('detach','seal',None,True)
        self.assertIsNone(monitor.observe(replace(self.initial,stamp_s=1.01,pose_command_count=12)))
        monitor.confirm_detached(dict(attached=False,pose_command_count=12))
        monitor.begin_phase('withdraw',False,False,True)
        self.assertEqual(monitor.observe(replace(self.initial,stamp_s=1.02,pose_command_count=13)),
                         'DETACHED_PAYLOAD_POSE_COMMAND')
        monitor=self.create()
        self.assertEqual(monitor.observe(replace(self.initial,payload_pose_command_component_present=True)),
                         'DETACHED_PAYLOAD_POSE_COMMAND_COMPONENT_REMAINS')

    def test_push_stall_and_lateral_error_use_observed_box_motion(self):
        monitor=self.create();monitor.begin_phase('push','push_plate',False,True)
        for index in range(101):
            observed=replace(self.initial,stamp_s=1.+index*.01,
                container_T_tool=tuple(pose([index*.00006,0,0]).ravel()))
            reason=monitor.observe(observed)
            if reason:break
        self.assertEqual(reason,'PUSH_NO_PROGRESS')
        monitor=self.create();monitor.begin_phase('push','push_plate',False,True)
        observed=replace(self.initial,container_T_box=tuple(pose([0,.006,1.125]).ravel()))
        self.assertEqual(monitor.observe(observed),'PUSH_REFERENCE_ERROR')

    def test_batched_native_states_cover_poll_gap_but_missing_native_states_reject(self):
        from monitored_stream import NativeSampleMonitor
        for preserve,expected in ((True,None),(False,'MONITOR_RATE_INSUFFICIENT')):
            monitor=self.create();monitor.observe(self.initial)
            monitor._reader=SimpleNamespace(read=lambda row:replace(self.initial,
                stamp_s=row['vacuum']['sim_time_s']))
            native=[dict(sim_time_s=1.+i*.001,attached=False,processed_commands=0) for i in range(1,25)]
            if not preserve:native=native[-1:]
            runtime=SimpleNamespace(native_samples_after=lambda stamp:[r for r in native if r['sim_time_s']>stamp])
            callback=NativeSampleMonitor(runtime,monitor,ExecutionPolicy.load())
            self.assertEqual(callback(dict(vacuum=native[-1])),expected)
            if preserve:
                self.assertAlmostEqual(monitor.metrics()['max_monitor_gap_s'],.01)
                self.assertAlmostEqual(callback.metrics()['max_native_processing_backlog_s'],.014)

    def test_native_history_appended_after_snapshot_is_deferred(self):
        from monitored_stream import NativeSampleMonitor
        monitor=self.create();monitor.observe(self.initial)
        monitor._reader=SimpleNamespace(read=lambda row:replace(self.initial,stamp_s=row['vacuum']['sim_time_s']))
        native=[dict(sim_time_s=1.+i*.001,attached=False,processed_commands=0) for i in range(1,31)]
        runtime=SimpleNamespace(native_samples_after=lambda stamp:[r for r in native if r['sim_time_s']>stamp])
        callback=NativeSampleMonitor(runtime,monitor,ExecutionPolicy.load())
        self.assertIsNone(callback(dict(vacuum=native[23])))
        self.assertLessEqual(monitor.sample.stamp_s,1.024)
        self.assertIsNone(callback(dict(vacuum=native[-1])))


if __name__=='__main__':unittest.main()
