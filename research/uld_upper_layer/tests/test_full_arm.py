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

    def test_common_ancestor_rotation_preserves_mesh_pair_distance(self):
        scene=self.scene;pair=('elfin_link4:0','elfin_link6:0')
        self.assertEqual(scene.relative_joint_indices(pair),{4,5})
        q=np.zeros(6);scene.set_joints(q)
        before=scene.pair_managers[pair[0]].min_distance_other(scene.pair_managers[pair[1]])
        q[0]=.4;scene.set_joints(q)
        after=scene.pair_managers[pair[0]].min_distance_other(scene.pair_managers[pair[1]])
        self.assertAlmostEqual(before,after,places=9)


if __name__=='__main__':unittest.main()
