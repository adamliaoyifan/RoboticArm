"""Behavioural observation gates, including tilted faces and missing contacts."""
from pathlib import Path
from dataclasses import replace
import sys
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from geometry import pose
from physical_observation import ObservedBody,ObservedContact,PhysicalSample,ObservationReader
from observed_support import measure_observed_support


def sample(box=None,lower=None,contacts=()):
    if box is None:box=pose([0,0,1.125])
    if lower is None:lower=(ObservedBody('lower_0',(.5,1.,1.),tuple(pose([.25,0,.5]).ravel()),1.),)
    return PhysicalSample(1.,(0.,)*6,tuple(box.ravel()),tuple(np.eye(4).ravel()),lower,contacts,1.,False,0,0)


class ObservedSupportContracts(unittest.TestCase):
    def test_area_union_and_antitip_hull_are_distinct(self):
        lower=tuple(ObservedBody('lower_'+str(i),(.2,1,1),tuple(pose([x,0,.5]).ravel()),1.)
                    for i,x in enumerate((-.4,.4)))
        result=measure_observed_support(sample(lower=lower),(1,1,.25),(0,0,0),.001,.0005)
        self.assertAlmostEqual(result.near_contact_area_m2,.4)
        self.assertAlmostEqual(result.near_contact_area_ratio,.4)
        self.assertAlmostEqual(result.com_margin_m,.5)
        self.assertEqual(result.native_lower_body_ids,())
        self.assertIn('unavailable',result.pressure_area_status)

    def test_native_point_does_not_invent_pressure_area_and_overpenetration_survives(self):
        point=ObservedContact('lower_0::body::collision',(0,0,1),(0,0,1),.002)
        box=pose([0,0,1.123])
        result=measure_observed_support(sample(box=box,contacts=(point,)),(1,1,.25),(0,0,0),.001,.0005)
        self.assertEqual(result.near_contact_area_m2,0.)
        self.assertEqual(result.native_max_depth_m,.002)
        self.assertEqual(result.native_lower_body_ids,('lower_0',))
        self.assertTrue(result.native_contact_hull_wkt.startswith('POINT'))

    def test_tilt_is_clipped_in_3d_and_gap_excludes_support(self):
        horizontal=measure_observed_support(sample(),(1,1,.25),(0,0,0),.001,.0005)
        box=pose([0,0,1.125],Rotation.from_euler('y',.01).as_matrix())
        tilted=measure_observed_support(sample(box=box),(1,1,.25),(0,0,0),.001,.0005)
        self.assertGreater(tilted.near_contact_area_m2,0.)
        self.assertLess(tilted.near_contact_area_m2,horizontal.near_contact_area_m2)
        elevated=measure_observed_support(sample(box=pose([0,0,1.127])),(1,1,.25),(0,0,0),.001,.0005)
        self.assertEqual(elevated.near_contact_area_ratio,0.)


class ObservationReaderContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from assets import load
        from types import SimpleNamespace
        cls.assets=load(Path(__file__).resolve().parents[3])
        cls.snapshot=SimpleNamespace(occupied=())

    def row(self,first=True):
        state=dict(position=dict(x=0.,y=0.,z=1.),orientation=dict(w=1.))
        contact=dict(collision1=dict(name='payload::body::collision' if first else 'lower::body::collision'),
                     collision2=dict(name='lower::body::collision' if first else 'payload::body::collision'),
                     position=[dict(z=1.)],normal=[dict(z=1. if first else -1.)],depth=[.00001])
        return dict(vacuum=dict(ready=True,joint_positions_complete=True,q=[0.]*6,sim_time_s=2.,
                    payload_pose_world=state,link_pose_world=state,attached=False,pose_command_count=0,processed_commands=1),
                    clock=2.,contact_stamp=2.,contacts=dict(contact=[contact]),poses={},pose_stamps={})

    def test_normal_direction_is_payload_relative_and_coordinates_are_transformed(self):
        reader=ObservationReader(self.assets,self.snapshot,'payload',.1)
        a=reader.read(self.row());b=reader.read(self.row(False))
        self.assertEqual(a.contacts,b.contacts)
        expected=np.linalg.inv(self.assets['world_container'])@np.array([0,0,1,1])
        np.testing.assert_allclose(a.contacts[0].point_container,expected[:3])

    def test_stale_incomplete_and_nonfinite_streams_reject(self):
        reader=ObservationReader(self.assets,self.snapshot,'payload',.1)
        for change,reason in ((lambda row:row.update(clock=1.8),'STALE_PHYSICAL_STREAM'),
                              (lambda row:row.update(native_latest_stamp_s=2.11),'STALE_PHYSICAL_STREAM'),
                              (lambda row:row['vacuum'].update(joint_positions_complete=False),'NATIVE_PHYSICAL_STATE_INCOMPLETE'),
                              (lambda row:row['vacuum'].update(q=[float('nan')]*6),'INVALID_NATIVE_JOINTS_OR_STAMP'),
                              (lambda row:row['contacts']['contact'][0].update(depth=[]),'NATIVE_CONTACT_GEOMETRY_INCOMPLETE')):
            row=self.row();change(row)
            with self.assertRaisesRegex(ValueError,reason):reader.read(row)

    def test_empty_native_component_is_current_contact_evidence_not_topic_silence(self):
        reader=ObservationReader(self.assets,self.snapshot,'payload',.1)
        row=self.row();row['contact_stamp']=1.
        with self.assertRaisesRegex(ValueError,'STALE_PHYSICAL_STREAM'):reader.read(row)
        row['vacuum'].update(contacts_component_available=True,payload_contacts={})
        sample=reader.read(row)
        self.assertEqual(sample.contacts,())
        self.assertEqual(sample.contact_stamp_s,2.)
        row['vacuum']['contacts_component_available']=False
        with self.assertRaisesRegex(ValueError,'NATIVE_PHYSICAL_CONTACT_COMPONENT_UNAVAILABLE'):reader.read(row)


if __name__=='__main__':unittest.main()
