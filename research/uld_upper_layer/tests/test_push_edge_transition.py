"""Experimental seam mode admits only bounded, planned edge traversal."""
from pathlib import Path
from types import SimpleNamespace
from dataclasses import replace
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from geometry import pose
from planning_types import SurfacePatch,UpperConfig
from execution_policy import ExecutionPolicy
from push_edge_transition import PushEdgeTransition
from contact_validation import validate_declared_contacts
from physical_observation import ObservedContact,ObservedBody


class PushEdgeContracts(unittest.TestCase):
    def setUp(self):
        support=SimpleNamespace(source_ids=('old','next'))
        self.action=SimpleNamespace(handoff_box_pose=tuple(pose([0,0,1.125]).ravel()),
            placement=SimpleNamespace(final_box_pose=tuple(pose([.4,0,1.125]).ravel()),support=support),
            handoff_support=support,push_direction=(1.,0.,0.))
        self.policy=replace(ExecutionPolicy.load(),push_edge_transition_enabled=True)
        self.edge=PushEdgeTransition(self.action,(1,1,.25),
            (SurfacePatch('next',(.7,-.5,1.7,.5),1.),),self.policy)
        self.lower=ObservedBody('next',(1,1,1),tuple(pose([1.2,0,.5]).ravel()),1.)
        self.sample=SimpleNamespace(stamp_s=1.,attached=False,lower=(self.lower,),
            container_T_box=tuple(pose([.2,0,1.125]).ravel()),container_T_tool=tuple(np.eye(4).ravel()))
        self.contact=ObservedContact('next::body::collision',(.7,0,1),(-1,0,0),.00001)

    def classify(self,**changes):
        args=dict(box_pose=self.sample.container_T_box,support_id='next',
            support_pose=self.lower.container_T_body,bounds=[[-.5]*3,[.5]*3],
            point=self.contact.point_container,normal=self.contact.normal_toward_payload_container,
            depth=self.contact.depth_m,normal_toward_box=True)
        args.update(changes);return self.edge.classify(**args)

    def test_planned_edge_and_unsigned_fcl_order(self):
        self.assertTrue(self.classify())
        self.assertFalse(self.classify(normal=(1,0,0)))
        self.assertTrue(self.classify(normal=(1,0,0),normal_toward_box=False))

    def test_wrong_body_side_wall_depth_and_event_fail(self):
        for changes in (dict(support_id='wall'),dict(point=(.7,0,.99)),
                        dict(point=(.69,0,1)),dict(depth=.000051),
                        dict(box_pose=pose([.1,0,1.125]))):
            with self.subTest(changes=changes):self.assertFalse(self.classify(**changes))

    def check(self,contacts,**changes):
        sample=SimpleNamespace(**vars(self.sample));sample.contacts=contacts
        for k,v in changes.items():setattr(sample,k,v)
        return validate_declared_contacts(sample,(1,1,.25),UpperConfig.load(),self.policy,
            True,False,None,SimpleNamespace(),edge_transition=self.edge)[0]

    def test_edge_is_not_support_and_default_path_remains_strict(self):
        self.assertEqual(self.check((self.contact,))['reason'],'PUSH_EDGE_WITHOUT_STRICT_NATIVE_SUPPORT')
        top=replace(self.contact,normal_toward_payload_container=(0,0,1))
        self.assertIsNone(self.check((top,self.contact)))
        self.assertEqual(self.check((top,self.contact),attached=True)['reason'],
                         'NATIVE_CONTACT_OUTSIDE_DECLARED_SUPPORT')
        self.assertFalse(ExecutionPolicy.load().push_edge_transition_enabled)

    def test_recurrent_contact_cannot_reset_timeout(self):
        self.assertIsNone(self.edge.observe(1., [('next',.00001)]))
        self.assertIsNone(self.edge.observe(1.8, []))
        self.assertEqual(self.edge.observe(2.001,[('next',.00001)])['reason'],
                         'PUSH_EDGE_TRANSITION_TIMEOUT')


if __name__=='__main__':unittest.main()
