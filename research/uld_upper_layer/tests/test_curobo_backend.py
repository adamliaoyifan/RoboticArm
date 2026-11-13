"""Behavioural source-frame, native veto, dispatch and worker failure gates."""
from dataclasses import replace
from importlib.util import find_spec
from pathlib import Path
import json
import os
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
import numpy as np
import trimesh
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from planning_types import UpperConfig
from curobo_backend import admit
from curobo_client import CuroboClient
from curobo_scene import make_request,mesh_record
from test_path_contracts import ToyScene


class ConfigAndAdmission(unittest.TestCase):
    def test_hot_reload_changes_policy_and_revision(self):
        import yaml
        config=UpperConfig.load()
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'config.yml'
            source=Path(__file__).resolve().parents[1]/'upper_config.yaml'
            data=yaml.safe_load(source.read_text());p.write_text(yaml.safe_dump(data))
            before=UpperConfig.load(p)
            data.update(free_motion_backend='curobo_fast',curobo_e2e_root='/external/e2e')
            p.write_text(yaml.safe_dump(data));after=UpperConfig.load(p)
            self.assertEqual(before.free_motion_backend,'ompl')
            self.assertEqual(after.curobo_e2e_root,'/external/e2e');self.assertNotEqual(before.revision,after.revision)

    def test_invalid_options(self):
        for changes in (dict(free_motion_backend='fast'),dict(path_attempt_budget_s=0)):
            with self.subTest(changes=changes),self.assertRaises(ValueError):replace(UpperConfig.load(),**changes)

    def test_dispatch_includes_attached_seal_policy(self):
        from path_planning import joint_path
        config=replace(UpperConfig.load(),free_motion_backend='curobo_fast')
        with patch('curobo_backend.joint_path',return_value=('path','detail')) as call:
            self.assertEqual(joint_path('scene',[0]*6,[.1]*6,config,True),('path','detail'))
            self.assertTrue(call.call_args.args[-1])

    @unittest.skipUnless(find_spec('ompl'),'OMPL dependency required')
    def test_default_ompl_does_not_start_gpu(self):
        from path_planning import joint_path
        with patch('curobo_backend.joint_path',side_effect=AssertionError('GPU default regression')):
            q,detail=joint_path(ToyScene(),[-.5]*6,[.5]*6,UpperConfig.load())
            self.assertIsNotNone(q,detail)

    def test_worker_response_identity_is_verified(self):
        with tempfile.TemporaryDirectory() as directory:
            script=Path(directory)/'wrong-python'
            script.write_text('#!/usr/bin/python3\nimport os,sys,json\nf=int(sys.argv[sys.argv.index("--response-fd")+1])\nsys.stdin.readline()\nos.write(f,b\'{"identity":"wrong","success":true}\\n\')\n')
            script.chmod(0o755);client=CuroboClient(str(script))
            result=client.query(dict(identity='correct'),2.)
            self.assertFalse(result['success']);self.assertEqual(result['reason'],'GPU_RESPONSE_IDENTITY_MISMATCH')

    def test_float_endpoints_restored_and_recertified(self):
        start=[-.5]*6;goal=[.5]*6;path=np.array([start,goal])+1e-8
        q,detail=admit(ToyScene(),path,start,goal,UpperConfig.load(),False,time.monotonic()+10)
        self.assertEqual(q,[start,goal]);self.assertTrue(detail['certificate']['valid'])

    def test_native_intermediate_collision_overrides_gpu_success(self):
        q,detail=admit(ToyScene(True),[[-.5]*6,[.5]*6],[-.5]*6,[.5]*6,
                       UpperConfig.load(),False,time.monotonic()+10)
        self.assertIsNone(q);self.assertEqual(detail['reason'],'GPU_INTERVAL_REJECTED')

    def test_reject_bad_paths_and_limits(self):
        for path,reason in (([[0]*6],'INVALID_GPU_PATH'),([[float('nan')]*6,[.5]*6],'INVALID_GPU_PATH'),
            ([[-.4]*6,[.5]*6],'GPU_ENDPOINT_MISMATCH'),([[-.5]*6,[2]*6,[.5]*6],'GPU_JOINT_LIMIT')):
            with self.subTest(reason=reason):
                q,d=admit(ToyScene(),path,[-.5]*6,[.5]*6,UpperConfig.load(),False,time.monotonic()+10)
                self.assertIsNone(q);self.assertEqual(d['reason'],reason)

    def test_expired_admission_never_accepts(self):
        q,d=admit(ToyScene(),[[-.5]*6,[.5]*6],[-.5]*6,[.5]*6,
                  UpperConfig.load(),False,time.monotonic()-1)
        self.assertIsNone(q);self.assertEqual(d['reason'],'GPU_ADMISSION_BUDGET_EXHAUSTED')

    def test_exact_box_recognition_retains_hollow_mesh(self):
        box=trimesh.creation.box([.55,.4,.25])
        record=mesh_record(box,np.eye(4))
        np.testing.assert_allclose(record['size'],[.55,.4,.25])
        sphere=mesh_record(trimesh.creation.icosphere(subdivisions=1),np.eye(4))
        self.assertNotIn('size',sphere)

    def test_source_signature_binds_collision_bytes_and_fixed_tool(self):
        from curobo_scene import model_signature
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'part.stl';trimesh.creation.box().export(path)
            xml=f'<robot><link name="elfin_base_link"><collision><geometry><mesh filename="{path}"/></geometry></collision></link><link name="tcp"/><joint name="tool" type="fixed"><parent link="elfin_base_link"/><child link="tcp"/><origin xyz="0 0 1"/></joint></robot>'
            before=model_signature(xml)
            self.assertNotEqual(before,model_signature(xml.replace('0 0 1','0 0 2')))
            trimesh.creation.box([2,2,2]).export(path)
            self.assertNotEqual(before,model_signature(xml))

    def test_missing_reference_is_explicit_without_importing_cuda(self):
        from curobo_worker import Runtime
        with self.assertRaisesRegex(ValueError,'ELFIN_E2E_ROOT_REQUIRED'):
            request=dict(options=dict(e2e_root='/no/e2e'))
            from planning_types import digest
            request['identity']=digest(request)
            Runtime().query(request)

    def test_existing_session_owns_rrt_world_updates_and_context_reuse(self):
        from types import SimpleNamespace,ModuleType
        from unittest.mock import MagicMock
        from curobo_worker import Runtime
        from curobo_scene import model_signature
        from planning_types import digest
        # Real external schemas and profile, fake only the expensive GPU owner.
        package=os.environ.get('ELFIN_E2E_ROOT','')
        if not (Path(package)/'training/motion_constraints').is_dir():
            self.skipTest('external elfin_e2e not available')
        sys.path.insert(0,str(Path(package)/'training/motion_constraints'))
        from motion_constraints.evaluation.throughput import persistent
        from motion_constraints.evaluation.attachment_episode import robot
        from motion_constraints.evaluation.world_loop_v2 import snapshots
        with tempfile.TemporaryDirectory() as directory:
            xml='<robot><link name="elfin_base_link"/></robot>'
            urdf=Path(directory)/'robot.urdf';urdf.write_text(xml)
            manifest=Path(directory)/'manifest.json'
            manifest.write_text(json.dumps(dict(assets=dict(bundle='bundle',urdf=str(urdf),mesh_root=directory))))
            assets=SimpleNamespace(bundle=SimpleNamespace(content_sha256='verified-bundle'),joint_names=tuple(range(6)))
            session=MagicMock();session.assets=assets
            tensor=lambda a:SimpleNamespace(detach=lambda:SimpleNamespace(cpu=lambda:SimpleNamespace(numpy=lambda:np.asarray(a))))
            pose=SimpleNamespace(position=tensor([[0,0,0]]),quaternion=tensor([[1,0,0,0]]))
            session.planner.compute_kinematics.return_value=SimpleNamespace(tool_poses=SimpleNamespace(get_link_pose=lambda name:pose))
            result=object();session.planner.plan_cspace.return_value=result
            snap=SimpleNamespace(revision=1,scene=SimpleNamespace(revision='live-world'))
            torch=ModuleType('torch');torch.tensor=lambda q,**kwargs:np.asarray(q);torch.float32='float32'
            torch.cuda=SimpleNamespace(synchronize=lambda:None)
            types=ModuleType('curobo.types');types.JointState=SimpleNamespace(from_position=lambda q,**kwargs:q)
            request=dict(model=dict(urdf=xml,signature=digest(model_signature(xml)),joint_names=list(range(6))),
                options=dict(e2e_root=package,assets_manifest=str(manifest),margin_m=.002),
                world=dict(obstacle=dict(size=[.1]*3,pose=[0,0,0,1,0,0,0])),payload=None,
                start=[0]*6,goal=[.01]*6,source_fk=[np.eye(4).tolist()]*2)
            request['identity']=digest(request);runtime=Runtime()
            with patch.object(robot,'RobotAssets',return_value=assets),patch.object(persistent,'PersistentPlannerSession',return_value=session) as factory,patch.object(snapshots,'snapshot',return_value=snap) as snapshot_call,patch.dict(sys.modules,{'torch':torch,'curobo.types':types}),patch('motion_constraints.evaluation.throughput.results.successful_positions',return_value=(np.array([request['start'],request['goal']]),None,True)):
                first=runtime.query(request)
                request['world']['obstacle']['pose'][0]=.2
                request['identity']=digest({k:v for k,v in request.items() if k!='identity'})
                second=runtime.query(request)
                self.assertTrue(first['success']);self.assertTrue(first['cold']);self.assertFalse(second['cold'])
                self.assertEqual(factory.call_count,1);self.assertEqual(session.update.call_count,2)
                profile=factory.call_args.args[1]
                self.assertEqual(profile.graph_planner,'rrt');self.assertFalse(profile.inverse_dynamics)
                world=snapshot_call.call_args.args[1]
                self.assertAlmostEqual(world[0].transform[0][3],.2)
                np.testing.assert_allclose(session.planner.plan_cspace.call_args.args[0],[request['goal']])
                runtime.close();session.close.assert_called_once()

    def test_worker_missing_python_is_explicit(self):
        with self.assertRaises(FileNotFoundError):CuroboClient('/no/such/python')

    def test_partial_response_cannot_bypass_deadline(self):
        with tempfile.TemporaryDirectory() as directory:
            script=Path(directory)/'partial-python'
            script.write_text('#!/usr/bin/python3\nimport os,sys,time\nf=int(sys.argv[sys.argv.index("--response-fd")+1])\nsys.stdin.readline()\nos.write(f,b\'{"partial":\')\ntime.sleep(30)\n')
            script.chmod(0o755);client=CuroboClient(str(script))
            result=client.query(dict(identity='test'),.1)
            self.assertEqual(result['reason'],'GPU_WORKER_TIMEOUT')
            self.assertIsNotNone(client.process.poll())

    def test_worker_timeout_kills_private_process(self):
        # An executable that consumes input but never responds exercises the real
        # bounded IPC path without importing CUDA or changing a shared runtime.
        with tempfile.TemporaryDirectory() as directory:
            script=Path(directory)/'blocked-python'
            script.write_text('#!/usr/bin/python3\nimport time\ntime.sleep(30)\n');script.chmod(0o755)
            client=CuroboClient(str(script));result=client.query(dict(identity='test'),.02)
            self.assertFalse(result['success']);self.assertEqual(result['reason'],'GPU_WORKER_TIMEOUT')
            self.assertIsNotNone(client.process.poll());self.assertFalse(client.finalizer.alive)


@unittest.skipUnless(find_spec('fcl'),'source FCL environment required')
class SourceSceneExport(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from assets import load
        from scene import fixture
        from ideal_snapshot import capture_fixture
        from full_arm import FullArmScene
        cls.temp=tempfile.TemporaryDirectory();cls.config=UpperConfig.load()
        assets=load(Path(__file__).resolve().parents[3])
        data=fixture(assets);data['pickup_payload']=dict(name='upper_layer_payload',pose_world=[-1,0,.986,0,0,0])
        (Path(cls.temp.name)/'fixture.json').write_text(json.dumps(data))
        snapshot,_=capture_fixture(cls.temp.name,assets,cls.config)
        cls.scene=FullArmScene(assets,snapshot)

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()

    def test_world_all_objects_base_transform_and_immutable_scene(self):
        from scipy.spatial.transform import Rotation
        scene=self.scene;before=list(scene.world_parts)
        request=make_request(scene,[0]*6,[.01]*6,self.config)
        self.assertEqual(set(request['world']),set(before))
        base_world=np.linalg.inv(scene.urdf.tcp([0]*6,scene.urdf.path_to('elfin_base_link')))
        for name,(mesh,world) in scene.world_parts.items():
            expected=base_world@world;record=request['world'][name]
            np.testing.assert_allclose(record['pose'][:3],expected[:3,3],atol=1e-12)
            np.testing.assert_allclose(Rotation.from_quat(np.array(record['pose'][3:])[[1,2,3,0]]).as_matrix(),expected[:3,:3],atol=1e-12)
            self.assertEqual(len(record['faces']),len(mesh.faces))
        self.assertEqual(list(scene.world_parts),before)
        self.assertIn('signature',request['model'])
        self.assertEqual(request['frame_id'],'elfin_base_link')

    def test_existing_world_schema_accepts_full_source_scene(self):
        package=Path(os.environ.get('ELFIN_E2E_ROOT',''))/'training/motion_constraints'
        if not package.is_dir():self.skipTest('ELFIN_E2E_ROOT not set')
        sys.path.insert(0,str(package))
        from curobo_worker import external_world
        from motion_constraints.world import Mesh
        request=make_request(self.scene,[0]*6,[.01]*6,self.config)
        world=external_world(request['world'])
        self.assertEqual({o.object_id for o in world},set(self.scene.world_parts))
        meshes=[o for o in world if isinstance(o,Mesh)]
        self.assertTrue(meshes)
        hull=next(m for m in meshes if m.object_id=='uld_mesh')
        self.assertFalse(hull.solid)
        for mesh in meshes:
            self.assertEqual(len(mesh.faces),len(request['world'][mesh.object_id]['faces']))

    def test_attachment_transform_and_released_world_geometry(self):
        scene=self.scene;transform=np.eye(4);transform[:3,3]=[.1,.2,.3]
        scene.payload=dict(mode='attached',size=np.array([.5,.4,.3]),box_T_tool=transform,
                           mesh=trimesh.creation.box([.5,.4,.3]))
        try:
            request=make_request(scene,[0]*6,[.01]*6,self.config,True)
            np.testing.assert_allclose(request['payload']['tool_from_box'][:3],[-.1,-.2,-.3])
            old=request['identity'];scene.payload['size'][0]=.6
            self.assertNotEqual(old,make_request(scene,[0]*6,[.01]*6,self.config,True)['identity'])
            with self.assertRaises(ValueError):make_request(scene,[0]*6,[.01]*6,self.config)
            scene.payload.update(mode='world',world_pose=np.eye(4))
            request=make_request(scene,[0]*6,[.01]*6,self.config)
            self.assertIsNone(request['payload']);self.assertIn('released_payload',request['world'])
        finally:scene.payload=None


if __name__=='__main__':unittest.main()
