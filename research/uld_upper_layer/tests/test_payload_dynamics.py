"""Analytic payload pendulum tests for the source effort screen."""
from pathlib import Path
from types import SimpleNamespace
import sys,unittest,xml.etree.ElementTree as ET,time
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from payload_dynamics import PayloadGravityModel,screen_path
from planning_types import UpperConfig
from planning_pause import paused_for_planning


def pendulum(mass):
    root=ET.Element('robot');joints={}
    for i in range(6):
        name=f'elfin_joint{i+1}';j=ET.SubElement(root,'joint',name=name)
        ET.SubElement(j,'limit',effort='34');ET.SubElement(j,'dynamics',friction='15',damping='25')
        joints[name]=dict(parent=f'p{i}',xyz=np.zeros(3),rot=np.eye(3),axis=np.array([0,1,0]))
    u=SimpleNamespace(root=root,joints=joints,path_to=lambda x:[])
    return PayloadGravityModel(u,mass,(0,0,0),(0,0,-9.8),5)


class PayloadDynamics(unittest.TestCase):
    def test_mass_and_lever_change_admission_without_changing_actuator_limits(self):
        tf={f'p{i}':np.eye(4) for i in range(6)};box=np.eye(4);box[0,3]=.2
        small=pendulum(1).evaluate(tf,box,True);large=pendulum(8).evaluate(tf,box,True)
        self.assertTrue(small['valid']);self.assertFalse(large['valid'])
        self.assertAlmostEqual(abs(small['gravity_torque_nm'][0]),1.96)
        self.assertAlmostEqual(abs(large['gravity_torque_nm'][0]),15.68)
        self.assertTrue(pendulum(8).evaluate(tf,box,False)['valid'])
        box[0,3]=0.;box[2,3]=.2
        self.assertTrue(pendulum(8).evaluate(tf,box,True)['valid'])

    def test_edge_with_valid_endpoints_rejects_interior_gravity_peak(self):
        def transforms(q):
            t={f'p{i}':np.eye(4) for i in range(6)};box=np.eye(4)
            box[:3,3]=Rotation.from_euler('y',q[0]).apply([.2,0,0]);t['suction_contact_frame']=box
            return t
        scene=SimpleNamespace(payload_dynamics=pendulum(8),link_transforms=transforms,
            payload=dict(mode='attached',box_T_tool=np.eye(4)))
        result=screen_path(scene,[[-np.pi/2,0,0,0,0,0],[np.pi/2,0,0,0,0,0]],UpperConfig.load(),time.monotonic()+3)
        self.assertFalse(result['valid']);self.assertEqual(result['reason'],'QUASISTATIC_EFFORT_LIMIT')
        self.assertGreater(result['checks'],1)

    def test_planning_clock_restores_on_success_and_failure(self):
        calls=[];runtime=SimpleNamespace(set_physics_paused=calls.append)
        with paused_for_planning(runtime):pass
        self.assertEqual(calls,[True,False]);calls.clear()
        with self.assertRaisesRegex(RuntimeError,'failure'):
            with paused_for_planning(runtime):raise RuntimeError('failure')
        self.assertEqual(calls,[True,False])

if __name__=='__main__':unittest.main()
