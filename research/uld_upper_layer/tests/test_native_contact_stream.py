"""Subperiod contact bursts must survive the slower full-scene sampling gate."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import io
import json
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from execution_monitor import ExecutionMonitor
from execution_policy import ExecutionPolicy
from planning_types import UpperConfig
from physical_observation import PhysicalSample,ObservedBody,ObservedContact
from monitored_stream import NativeSampleMonitor
from geometry import pose


class NativeContactStreamContracts(unittest.TestCase):
    def monitor(self,events,step=.001,period=.02,burst=None,panel=False):
        initial=PhysicalSample(1.,(0.,)*6,tuple(pose([0,0,1.125]).ravel()),tuple(np.eye(4).ravel()),
            (ObservedBody('lower_0',(1.,1.,1.),tuple(pose([0,0,.5]).ravel()),1.),),(),1.,False,0,0)
        policy=replace(ExecutionPolicy.load(),monitor_period_s=period)
        scene=SimpleNamespace(world_container=np.eye(4),payload={},set_payload=lambda *a,**k:None,
            update_world_transform=lambda *a:None,check_state=lambda *a:None)
        if panel:
            from shapely.geometry import box
            scene.contact_mode=lambda value:'push_plate'
            scene.push_geometry=SimpleNamespace(tool_T_push_surface=tuple(np.eye(4).ravel()))
            scene.contact_polygon=lambda mode:box(-.09,-.09,.09,.09)
        def read(row):
            if isinstance(row,PhysicalSample):return row
            stamp=row['vacuum']['sim_time_s'];contacts=burst(stamp) if burst else ()
            return replace(initial,stamp_s=stamp,contacts=contacts)
        trace=io.StringIO()
        monitor=ExecutionMonitor(SimpleNamespace(read=read),scene,
            SimpleNamespace(dimensions=(1,1,.25),com_local=(0,0,0)),
            SimpleNamespace(handoff_box_pose=initial.container_T_box,push_direction=(1,0,0)),
            UpperConfig.load(),policy,initial,trace,'panel' if panel else None)
        monitor.observe(initial)
        runtime=SimpleNamespace(native_step_s=step,native_samples_after=lambda stamp:[n for n in events if n['sim_time_s']>stamp])
        return monitor,NativeSampleMonitor(runtime,monitor,policy),trace

    @staticmethod
    def events(count,step=.001):
        return [dict(sim_time_s=1.+i*step,attached=False,processed_commands=0) for i in range(1,count+1)]

    def test_one_step_side_contact_between_full_checks_is_not_lost(self):
        for period in (.01,.02):
            events=self.events(25)
            contact=ObservedContact('lower_0::body::collision',(.5,0,1),(1,0,0),.000001)
            monitor,callback,trace=self.monitor(events,period=period,
                burst=lambda stamp:(contact,) if abs(stamp-1.005)<1e-9 else ())
            self.assertEqual(callback(dict(vacuum=events[-1])),'NATIVE_CONTACT_OUTSIDE_DECLARED_SUPPORT')
            self.assertAlmostEqual(monitor.failure_stamp_s,1.005)
            self.assertEqual(monitor.sample.stamp_s,1.)
            self.assertEqual(callback.metrics()['native_contact_samples'],5)
            failure=[json.loads(line) for line in trace.getvalue().splitlines() if json.loads(line)['event']=='failure'][0]
            self.assertEqual(failure['invalid_observation']['vacuum']['sim_time_s'],1.005)
            self.assertEqual(callback(dict(vacuum=events[-1])),'NATIVE_CONTACT_OUTSIDE_DECLARED_SUPPORT')
            self.assertEqual(callback.metrics()['native_contact_samples'],5)

    def test_one_step_panel_patch_violation_is_not_lost(self):
        events=self.events(25)
        contact=ObservedContact('panel',(.2,0,0),(0,0,1),.000001)
        monitor,callback,_=self.monitor(events,panel=True,
            burst=lambda stamp:(contact,) if abs(stamp-1.005)<1e-9 else ())
        monitor.begin_phase('push_contact','push_plate',False,True)
        self.assertEqual(callback(dict(vacuum=events[-1])),'NATIVE_CONTACT_OUTSIDE_DECLARED_PANEL_PATCH')
        self.assertAlmostEqual(monitor.failure_stamp_s,1.005)

    def test_valid_top_contacts_cover_all_steps_while_full_rate_remains_configurable(self):
        events=self.events(1000)
        contact=ObservedContact('lower_0::body::collision',(0,0,1),(0,0,1),.000001)
        for period,full_count in ((.01,101),(.02,51)):
            monitor,callback,_=self.monitor(events,period=period,burst=lambda stamp:(contact,))
            for index in range(49,1000,50):self.assertIsNone(callback(dict(vacuum=events[index])))
            self.assertEqual(callback.metrics()['native_contact_samples'],1000)
            self.assertEqual(monitor.metrics()['samples'],full_count)
            self.assertLessEqual(callback.metrics()['max_native_contact_gap_s'],.001000001)

    def test_missing_native_step_cannot_be_hidden_by_valid_full_check(self):
        for step in (.001,.002):
            events=self.events(10,step);del events[4]
            monitor,callback,_=self.monitor(events,step=step)
            self.assertEqual(callback(dict(vacuum=events[-1])),'NATIVE_CONTACT_STREAM_GAP')
            self.assertAlmostEqual(monitor.failure['gap_s'],2*step)
            self.assertAlmostEqual(monitor.failure['expected_step_s'],step)

    def test_newest_native_without_history_is_not_assumed_checked(self):
        monitor,callback,_=self.monitor([])
        newest=self.events(1)[0]
        self.assertEqual(callback(dict(vacuum=newest)),'NATIVE_CONTACT_STREAM_MISSING')


if __name__=='__main__':unittest.main()
