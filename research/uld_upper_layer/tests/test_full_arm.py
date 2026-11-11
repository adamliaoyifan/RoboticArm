"""Source-mesh behavioural gates; optional isolated FCL environment required."""
from importlib.util import find_spec
from pathlib import Path
import sys
import tempfile
import json
import unittest
import numpy as np
import trimesh
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from assets import load
from geometry import pose
from ideal_snapshot import capture_fixture
from planning_types import UpperConfig
from scene import fixture


@unittest.skipUnless(find_spec('fcl'),'isolated python-fcl dependency not installed')
class FullArmContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from full_arm import FullArmScene
        cls.assets=load(Path(__file__).resolve().parents[3])
        cls.temp=tempfile.TemporaryDirectory()
        data=fixture(cls.assets)
        data['pickup_payload']=dict(name='upper_layer_payload',pose_world=[-1,0,.986,0,0,0])
        (Path(cls.temp.name)/'fixture.json').write_text(json.dumps(data))
        snapshot,_=capture_fixture(cls.temp.name,cls.assets,UpperConfig.load())
        cls.scene=FullArmScene(cls.assets,snapshot)

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()

    def test_source_robot_home_and_added_obstacle(self):
        scene=self.scene
        self.assertIsNone(scene.check_robot([0]*6,.005))
        tool=scene.transforms['suction_contact_frame']
        scene.add_world('test_forbidden_obstacle',trimesh.creation.box([.2,.2,.2]),tool)
        try:
            failure=scene.check_robot([0]*6)
            self.assertEqual(failure['reason'],'ROBOT_WORLD_COLLISION')
            self.assertIn('test_forbidden_obstacle',failure['pair'])
        finally:
            scene.remove_world('test_forbidden_obstacle')

    def test_fk_every_collision_link_matches_source_urdf(self):
        q=np.mean(self.scene.urdf.joint_limits(),axis=1)
        transforms=self.scene.link_transforms(q)
        for link,_,_ in self.scene.robot_parts.values():
            expected=self.scene.urdf.tcp(q,self.scene.urdf.path_to(link))
            np.testing.assert_allclose(transforms[link],expected,atol=1e-12)

    def test_out_of_bounds_not_collision_free(self):
        q=np.zeros(6);q[0]=self.scene.urdf.joint_limits()[0][1]+1
        self.assertEqual(self.scene.check_robot(q)['reason'],'JOINT_LIMIT')

    def test_source_point_radii_bound_all_robot_collision_vertices(self):
        from motion_bounds import SourceMotionBounds
        scene=self.scene;bounds=SourceMotionBounds(scene.urdf,scene.robot_parts)
        rng=np.random.default_rng(11);limits=np.array(scene.urdf.joint_limits())
        for _ in range(8):
            qa=rng.uniform(limits[:,0]+.2,limits[:,1]-.2)
            qb=qa+rng.uniform(-.15,.15,6)
            ta=scene.link_transforms(qa);tb=scene.link_transforms(qb)
            for name,(link,mesh,local) in scene.robot_parts.items():
                # Check every actual collision vertex, including fixed cameras.
                points=np.array(mesh.vertices)@local[:3,:3].T+local[:3,3]
                a=points@ta[link][:3,:3].T+ta[link][:3,3]
                b=points@tb[link][:3,:3].T+tb[link][:3,3]
                observed=float(np.linalg.norm(b-a,axis=1).max())
                self.assertLessEqual(observed,float(np.array(bounds.joint_point_radii(name))@np.abs(qb-qa))+1e-10,name)

    def test_attached_payload_offset_is_in_point_bound_and_detachment_removes_ancestry(self):
        from motion_bounds import SourceMotionBounds
        from scipy.spatial.transform import Rotation
        scene=self.scene;bounds=SourceMotionBounds(scene.urdf,scene.robot_parts)
        grasp=pose([.17,0,.125],Rotation.from_euler('x',np.pi).as_matrix())
        scene.set_payload((.55,.4,.25),'attached',np.eye(4),grasp)
        try:
            bounds.set_payload(scene.payload)
            rng=np.random.default_rng(13)
            tcp_box=np.linalg.inv(grasp)
            points=np.array(scene.payload['mesh'].vertices)@tcp_box[:3,:3].T+tcp_box[:3,3]
            for _ in range(8):
                qa=rng.uniform(-1,1,6);qb=qa+rng.uniform(-.1,.1,6)
                a=scene.urdf.tcp(qa,scene.urdf.path_to('suction_contact_frame'))
                b=scene.urdf.tcp(qb,scene.urdf.path_to('suction_contact_frame'))
                moved_a=points@a[:3,:3].T+a[:3,3];moved_b=points@b[:3,:3].T+b[:3,3]
                observed=float(np.linalg.norm(moved_b-moved_a,axis=1).max())
                self.assertLessEqual(observed,float(np.array(bounds.joint_point_radii('payload'))@np.abs(qb-qa))+1e-10)
            scene.set_payload((.55,.4,.25),'world',np.eye(4))
            bounds.set_payload(scene.payload)
            bounds.register_static('world_obstacle')
            self.assertEqual(bounds.relative_motion_bound(('payload','world_obstacle'),np.ones(6)),0.)
            self.assertGreater(bounds.relative_motion_bound(('payload','camera_link:0'),np.ones(6)),0.)
        finally:scene.set_payload((.55,.4,.25),'absent')

    def test_relative_mesh_distance_change_respects_source_bound(self):
        from motion_bounds import SourceMotionBounds
        scene=self.scene;bounds=SourceMotionBounds(scene.urdf,scene.robot_parts)
        pairs=[('elfin_link4:0','elfin_link6:0'),('elfin_link3:0','camera_link:0')]
        rng=np.random.default_rng(12)
        for _ in range(12):
            qa=rng.uniform(-1,1,6);qb=qa+rng.uniform(-.1,.1,6)
            measured=[]
            for q in (qa,qb):
                scene.set_joints(q)
                measured.append([max(0.,scene.pair_managers[a].min_distance_other(scene.pair_managers[b]))
                                 for a,b in pairs])
            for i,pair in enumerate(pairs):
                self.assertLessEqual(abs(measured[1][i]-measured[0][i]),
                    bounds.relative_motion_bound(pair,np.abs(qb-qa))+1e-9)
        with self.assertRaisesRegex(ValueError,'UNKNOWN_MOTION_BODY'):
            bounds.relative_motion_bound(('unregistered_body',pairs[0][0]),np.ones(6))
        weights=np.array(bounds.relative_motion_coefficients(pairs[0]))
        self.assertTrue(np.all(weights[:4]==0))
        self.assertTrue(np.any(weights[4:]>0))

    def test_common_ancestor_rotation_preserves_mesh_pair_distance(self):
        scene=self.scene;pair=('elfin_link4:0','elfin_link6:0')
        self.assertEqual(scene.relative_joint_indices(pair),{4,5})
        q=np.zeros(6);scene.set_joints(q)
        before=scene.pair_managers[pair[0]].min_distance_other(scene.pair_managers[pair[1]])
        q[0]=.4;scene.set_joints(q)
        after=scene.pair_managers[pair[0]].min_distance_other(scene.pair_managers[pair[1]])
        self.assertAlmostEqual(before,after,places=9)


if __name__=='__main__':unittest.main()
