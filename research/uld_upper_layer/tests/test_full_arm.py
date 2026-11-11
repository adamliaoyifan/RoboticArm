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

    def test_lazy_robot_minimum_matches_exhaustive_mesh_distances_not_search_minimum(self):
        from plan_clearance import FullArmDistanceSampler
        scene=self.scene;original=scene.minimum_clearance_m
        scene.set_payload((.55,.4,.25),'absent');scene.minimum_clearance_m=-123.
        try:
            result=FullArmDistanceSampler(scene,1e-6).evaluate(0,0.,np.zeros(6),False)
            self.assertIsNone(result.failure)
            scene.exact_distances=True
            self.assertIsNone(scene.check_state(np.zeros(6),scene.config.collision_margin_m))
            expected=min(d for d,pair in scene.sample_distances)
            self.assertAlmostEqual(result.robot_minimum_m,expected+1e-6,places=9)
            self.assertGreater(result.robot_minimum_m,0.)
            self.assertTrue(all(d>=expected-1e-6-1e-8 for d,pair,kind in result.distances))
        finally:
            scene.exact_distances=False;scene.minimum_clearance_m=original

    def test_source_fcl_bottom_corner_side_collision_is_not_support(self):
        scene=self.scene
        scene.add_world('lower_contract_probe',trimesh.creation.box([1,1,1]),pose([8,0,.5]))
        try:
            scene.set_payload((.55,.4,.25),'world',pose([8,0,1.125-.000001]),allow_support=True)
            self.assertIsNone(scene.check_payload(margin=0.))
            # The same bottom corner penetrates the lower body's vertical face
            # by 1um, versus 10um at its top. A bottom-point-only waiver accepts
            # this side jam; the normal-aware contract must reject it.
            scene.payload['world_pose']=pose([7.225+.000001,0,1.125-.00001])
            failure=scene.check_payload(margin=0.)
            self.assertEqual(failure['reason'],'PAYLOAD_WORLD_COLLISION')
            self.assertEqual(failure['support_diagnostic']['reason'],
                             'SUPPORT_CONTACT_NORMAL_NOT_TOP_BOTTOM')
        finally:
            scene.set_payload((.55,.4,.25),'absent')
            scene.remove_world('lower_contract_probe')

    def test_bounded_push_normal_contract_retains_patch_checks_and_strict_seal(self):
        from scipy.spatial.transform import Rotation
        from dataclasses import replace
        scene=self.scene;original=scene.config;scene.set_joints(np.zeros(6))
        frame=scene.contact_frame('push_plate')
        angle=original.contact_normal_alignment_deg/2
        box_pose=frame@pose([0,0,.125],Rotation.from_euler('x',angle,degrees=True).as_matrix())
        scene.set_payload((.55,.4,.25),'world',box_pose)
        try:
            self.assertTrue(scene.contact_patch_valid(box_pose,'push_plate'))
            self.assertFalse(scene.contact_patch_valid(box_pose,'seal'))
            scene.config=replace(original,contact_normal_alignment_deg=angle/2)
            self.assertFalse(scene.contact_patch_valid(box_pose,'push_plate'))
            scene.config=original
            outside=frame@pose([.4,0,.125],Rotation.from_euler('x',angle,degrees=True).as_matrix())
            self.assertFalse(scene.contact_patch_valid(outside,'push_plate'))
            tilted=frame@pose([0,0,.125],Rotation.from_euler('x',original.contact_normal_alignment_deg*1.1,degrees=True).as_matrix())
            self.assertFalse(scene.contact_patch_valid(tilted,'push_plate'))
        finally:
            scene.config=original;scene.set_payload((.55,.4,.25),'absent')

    def test_source_push_face_touches_payload_while_seal_reference_leaves_gap(self):
        from tool_contact_geometry import derive_push_surface
        from trimesh.collision import CollisionManager
        from shapely.geometry import Polygon,Point
        from types import SimpleNamespace
        from dataclasses import replace
        scene=self.scene;geometry=derive_push_surface(self.assets)
        scene.set_joints(np.zeros(6))
        tcp=scene.transforms['suction_contact_frame']
        relative=np.array(geometry.tool_T_push_surface).reshape(4,4)
        mesh=trimesh.creation.box([.55,.4,.25])
        def distance(box_pose):
            manager=CollisionManager();manager.add_object('payload',mesh,box_pose)
            return manager.min_distance_other(scene.pair_managers[geometry.collision_key])
        seal_box=tcp@pose([0,0,.125])
        plate_box=tcp@relative@pose([0,0,.125])
        self.assertLess(relative[2,3],-scene.config.contact_tolerance_m)
        self.assertAlmostEqual(distance(seal_box),-relative[2,3],places=7)
        self.assertLessEqual(abs(distance(plate_box)),geometry.cad_tolerance_m)
        polygon=Polygon(geometry.push_polygon_xy)
        self.assertFalse(polygon.covers(Point(polygon.bounds[0],polygon.bounds[1])))
        scene.set_payload((.55,.4,.25),'world',plate_box)
        try:
            self.assertTrue(scene.contact_patch_valid(plate_box,'push_plate'))
            self.assertFalse(scene.contact_patch_valid(plate_box,'seal'))
            outside=plate_box@pose([.4,0,0])
            self.assertFalse(scene.contact_patch_valid(outside,'push_plate'))
            with self.assertRaisesRegex(ValueError,'UNKNOWN_TOOL_CONTACT_MODE'):
                scene.contact_patch_valid(plate_box,'unregistered_contact')
            action=SimpleNamespace(push_contact_geometry_id=geometry.geometry_id,
                                   tool_T_push_surface=geometry.tool_T_push_surface)
            scene.validate_push_geometry(action)
            action.push_contact_geometry_id='old_geometry'
            with self.assertRaisesRegex(ValueError,'STALE_PUSH_CONTACT_GEOMETRY'):
                scene.validate_push_geometry(action)
        finally:scene.set_payload((.55,.4,.25),'absent')

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
