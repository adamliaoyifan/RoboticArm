"""Isolated IPC adapter to elfin_e2e's existing PersistentPlannerSession.

No robot fitting, custom planner, collision cost, or alternative scene owner lives
here. The external approved assets and pinned runtime remain elfin_e2e contracts.
"""
import argparse
import ctypes
import json
import os
from pathlib import Path
import signal
import tempfile
import time
import traceback




def external_world(records):
    """Use existing immutable world schemas; keep every source mesh triangle."""
    from motion_constraints.schema import Box
    from motion_constraints.world import Mesh
    from curobo_scene import pose_matrix
    return tuple(Box(name,tuple(m['size']),pose_matrix(m['pose'])) if 'size' in m else
                 Mesh(name,tuple(map(tuple,m['vertices'])),tuple(map(tuple,m['faces'])),pose_matrix(m['pose']),solid=m['solid'])
                 for name,m in records.items())


class Runtime:
    def __init__(self):
        self.session=None;self.directory=None;self.key=None

    def close(self):
        if self.session is not None:self.session.close()
        if self.directory is not None:self.directory.cleanup()
        self.session=None;self.directory=None;self.key=None

    def query(self,request):
        begin=time.monotonic()
        import sys
        import numpy as np
        from planning_types import digest
        from curobo_scene import model_signature
        if request['identity']!=digest({k:v for k,v in request.items() if k!='identity'}):
            raise ValueError('GPU_REQUEST_IDENTITY_MISMATCH')
        options=request['options']
        package=Path(options['e2e_root'])/'training/motion_constraints'
        if not (package/'motion_constraints').is_dir():raise ValueError('ELFIN_E2E_ROOT_REQUIRED')
        sys.path.insert(0,str(package))
        from motion_constraints.evaluation.attachment_episode.robot import RobotAssets
        from motion_constraints.evaluation.throughput.persistent import PersistentPlannerSession
        from motion_constraints.evaluation.throughput.profile import ExecutionProfile
        from motion_constraints.evaluation.throughput.capacity import PlannerCapacity
        from motion_constraints.evaluation.throughput.telemetry import Journal
        from motion_constraints.evaluation.throughput.native import BatchRequest
        from motion_constraints.evaluation.throughput.results import successful_positions
        from motion_constraints.evaluation.world_loop_v2.snapshots import snapshot
        from motion_constraints.schema import PayloadSpec,PlanningRequest
        from motion_constraints.world import Mesh
        from scipy.spatial.transform import Rotation
        manifest=json.loads(Path(options['assets_manifest']).read_text())['assets']
        from curobo_scene import pose_matrix
        assets=RobotAssets(manifest['bundle'],manifest['urdf'],manifest['mesh_root'])
        source=model_signature(request['model']['urdf'])
        if digest(source)!=request['model']['signature']:raise ValueError('GPU_REQUEST_SOURCE_CHANGED')
        if tuple(request['model']['joint_names'])!=tuple(assets.joint_names):raise ValueError('ELFIN_E2E_JOINT_ORDER_MISMATCH')
        if source!=model_signature(Path(manifest['urdf']).read_text(),manifest['mesh_root']):
            raise ValueError('ELFIN_E2E_SOURCE_MODEL_MISMATCH')
        world=external_world(request['world'])
        meshes=[m for m in world if isinstance(m,Mesh)]
        capacity=PlannerCapacity(cuboids=len(world)-len(meshes)+1,meshes=len(meshes),
                                 mesh_faces=max((len(m.faces) for m in meshes),default=0))
        key=digest(dict(model=request['model'],options=options,capacity=capacity.__dict__,
                        bundle=assets.bundle.content_sha256))
        cold=key!=self.key
        if cold:
            self.close();self.directory=tempfile.TemporaryDirectory(prefix='upper-e2e-')
            profile=ExecutionProfile(graph_planner='rrt',inverse_dynamics=False,enable_graph_attempt=0)
            self.session=PersistentPlannerSession(assets,profile,Path(self.directory.name)/'session',
                Journal(Path(self.directory.name)/'events.jsonl'),capacity=capacity)
            self.key=key
        # Use the session-bound sphere ordering for every contact snapshot.
        assets=self.session.assets
        payload=request['payload']
        payload=PayloadSpec('upper_payload',tuple(payload['size']),pose_matrix(payload['tool_from_box']),request['identity']) if payload else None
        snap=snapshot(assets,world,payload=payload,hard=options['margin_m'])
        item=BatchRequest('upper',snap,PlanningRequest(request['identity'],snap.revision,snap.scene.revision,
                          tuple(request['model']['joint_names']),tuple(request['start']),request['source_fk'][1]),self.directory.name)
        self.session.update([item])
        import torch
        from curobo.types import JointState
        def state(q):return JointState.from_position(torch.tensor([q],device='cuda:0',dtype=torch.float32),
                                                     joint_names=request['model']['joint_names'])
        errors=[]
        for q,expected in zip((request['start'],request['goal']),request['source_fk']):
            actual=self.session.planner.compute_kinematics(state(q)).tool_poses.get_link_pose('suction_contact_frame')
            p=actual.position.detach().cpu().numpy().reshape(-1,3)[0]
            quat=actual.quaternion.detach().cpu().numpy().reshape(-1,4)[0]
            rotation=Rotation.from_quat(quat[[1,2,3,0]]).as_matrix()
            angle=Rotation.from_matrix(rotation.T@np.asarray(expected)[:3,:3]).magnitude()
            position=float(np.linalg.norm(p-np.asarray(expected)[:3,3]))
            errors.append(dict(position_m=position,angle_rad=float(angle)))
            if position>1e-4 or angle>np.deg2rad(.01):raise ValueError('ELFIN_E2E_SOURCE_FK_MISMATCH')
        setup=time.monotonic()-begin;begin=time.monotonic()
        # Joint targets preserve the selected upper-stage IK branch. This is the
        # public planner already constructed and bound by PersistentPlannerSession.
        result=self.session.planner.plan_cspace(state(request['goal']),state(request['start']),
                                                 max_attempts=1,enable_graph_attempt=0)
        torch.cuda.synchronize()
        q,reason,success=successful_positions(result,0,6)
        return dict(success=q is not None,reason=reason,positions=q.tolist() if q is not None else None,
                    native_success=success,cold=cold,setup_s=setup,solve_s=time.monotonic()-begin,
                    fk_errors=errors,backend='elfin_e2e.PersistentPlannerSession.plan_cspace',identity=request['identity'])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--response-fd',type=int,required=True)
    parser.add_argument('--parent-pid',type=int,required=True);args=parser.parse_args()
    libc=ctypes.CDLL(None,use_errno=True)
    if libc.prctl(1,signal.SIGKILL,0,0,0)!=0:raise OSError(ctypes.get_errno(),'prctl')
    if os.getppid()!=args.parent_pid:return
    runtime=Runtime()
    import sys
    try:
        with os.fdopen(args.response_fd,'w',buffering=1) as out:
            for line in sys.stdin:
                request=json.loads(line)
                try:result=runtime.query(request)
                except Exception as exc:
                    runtime.close()
                    result=dict(success=False,reason=type(exc).__name__+':'+str(exc),
                        traceback=traceback.format_exc()[-12000:],identity=request.get('identity'))
                out.write(json.dumps(result,allow_nan=False)+'\n');out.flush()
    finally:runtime.close()


if __name__=='__main__':main()
