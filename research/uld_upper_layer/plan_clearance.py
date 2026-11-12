"""Reconstruct only accepted plan stages in a private source-mesh scene.

Exact queries resolve possible minima, not every distant pair. Broadphase values
remain conservative lower bounds. Robot and payload/environment scores stay
separate, and declared contact pairs retain the existing patch-level checks.
"""
import time
import numpy as np
from clearance_bounds import DistanceSample,bound_joint_path
from planning_types import digest
from support_geometry import measure_support
from push_seam_events import trace_push_seams


class FullArmDistanceSampler:
    def __init__(self,scene,numeric_error_m,box_motion=None):
        if not np.isfinite(numeric_error_m) or numeric_error_m<=0:
            raise ValueError('INVALID_NUMERIC_DISTANCE_ENCLOSURE')
        self.scene=scene;self.numeric_error_m=numeric_error_m;self.box_motion=box_motion

    def _exact(self,pair):
        scene=self.scene
        if 'payload' in pair:
            from trimesh.collision import CollisionManager
            payload=CollisionManager()
            payload.add_object('payload',scene.payload['mesh'],scene.payload_transform())
            other=next(name for name in pair if name!='payload')
            manager=scene.pair_managers.get(other) or scene.world_managers[other]
            return float(scene.observed_distance(payload,manager,pair))
        managers=[scene.pair_managers.get(name) or scene.world_managers[name] for name in pair]
        return float(scene.observed_distance(*managers,pair))

    def evaluate(self,segment,fraction,q,contact_mode):
        scene=self.scene
        if self.box_motion is not None:scene.payload['world_pose']=self.box_motion(segment,fraction)
        failure=scene.check_state(q,scene.config.collision_margin_m,contact_mode)
        if failure:return DistanceSample((),float('inf'),(),failure)
        error=self.numeric_error_m
        rows=[(float(distance)-error,tuple(pair),'robot' if any(n in scene.robot_parts for n in pair)
               else 'payload') for distance,pair in scene.sample_distances]
        robot_indices=sorted((i for i,r in enumerate(rows) if r[2]=='robot'),key=lambda i:rows[i][0])
        upper=float('inf');witness=()
        for index in robot_indices:
            lower,pair,kind=rows[index]
            if lower>=upper:break
            exact=self._exact(pair)
            if not np.isfinite(exact) or exact+error<lower:
                return DistanceSample((),upper,witness,dict(reason='INVALID_EXACT_DISTANCE',
                    pair=pair,lower_m=lower,exact_m=exact))
            if exact-error<scene.config.collision_margin_m-1e-10:
                return DistanceSample((),upper,witness,dict(reason='SCORE_ROBOT_CLEARANCE',pair=pair,distance_m=exact))
            rows[index]=(exact-error,pair,kind)
            if exact+error<upper:upper=exact+error;witness=pair
        return DistanceSample(tuple(rows),upper,witness)

    def motion_bound(self,pair,joint_delta,segment,low,high):
        result=self.scene.relative_motion_bound(pair,joint_delta)
        if self.box_motion is not None and 'payload' in pair:
            a=self.box_motion(segment,low);b=self.box_motion(segment,high)
            if not np.allclose(a[:3,:3],b[:3,:3],rtol=0,atol=1e-10):
                raise ValueError('SCORE_WORLD_PAYLOAD_MUST_HAVE_FIXED_ORIENTATION')
            # Within each q-linear segment, declared push progress is affine.
            result+=np.linalg.norm(a[:3,3]-b[:3,3])/2
        return float(result)


def plan_digest(plan):
    return digest(dict(action=plan['action'],start_attached=plan.get('start_attached',False),
        initial_q=plan.get('initial_q'),initial_box_pose=plan.get('initial_box_pose'),stages=[dict(
        {k:stage[k] for k in ('name','trajectory','certificate','payload_mode','contact_tool')},
        motion_kind=stage.get('motion_kind'),box_motion_model=stage.get('box_motion_model'),
        box_reference_container=stage.get('box_reference_container')) for stage in plan['stages']]))


def model_identity(scene):
    return digest(dict(assets=scene.assets['hashes'],meshes=scene.model_hashes))


def _push_motion(scene,stage,action):
    path=stage['trajectory']['positions'];chain=scene.urdf.path_to('suction_contact_frame')
    points=[scene.urdf.tcp(q,chain)[:3,3] for q in path]
    lengths=np.r_[0.,np.cumsum([np.linalg.norm(b-a) for a,b in zip(points,points[1:])])]
    if lengths[-1]<=0:raise ValueError('EMPTY_PUSH_REFERENCE')
    handoff=np.array(action['handoff_box_pose']).reshape(4,4)
    direction=np.asarray(action['push_direction'])
    def motion(segment,fraction):
        progress=(lengths[segment]+fraction*(lengths[segment+1]-lengths[segment]))/lengths[-1]
        box=handoff.copy();box[:3,3]+=direction*action['push_distance_m']*progress
        return scene.world_container@box
    return motion


def _support_trace(scene,state,action,config,deadline):
    handoff=np.array(action['handoff_box_pose']).reshape(4,4)
    final=np.array(action['placement']['final_box_pose']).reshape(4,4)
    if not np.allclose(handoff[:3,:3],final[:3,:3],rtol=0,atol=1e-10):
        return dict(valid=False,reason='UNSUPPORTED_ROTATING_PUSH_REFERENCE')
    distance=np.linalg.norm(final[:3,3]-handoff[:3,3])
    count=max(1,int(np.ceil(distance/config.sweep_step_m)));trace=[]
    for progress in np.linspace(0,1,count+1):
        if time.monotonic()>=deadline:return dict(valid=False,reason='SUPPORT_SCORE_BUDGET_EXHAUSTED')
        box=handoff.copy();box[:3,3]=(1-progress)*handoff[:3,3]+progress*final[:3,3]
        support=measure_support(box,state.dimensions,scene.snapshot.surfaces,config,state.com_local)
        trace.append(dict(progress=float(progress),area_ratio=support.contact_area_ratio,
                          com_margin_m=support.com_margin_m))
        if not support.accepted:return dict(valid=False,reason='PUSH_SUPPORT_POLICY',sample=trace[-1])
    return dict(valid=True,step_m=config.sweep_step_m,samples=trace,
        min_area_ratio=min(r['area_ratio'] for r in trace),min_com_margin_m=min(r['com_margin_m'] for r in trace),
        scope='nominal_support_sampled_at_configured_sweep_step_not_pressure_or_robust_tube')


def score_plan(scene,state,plan,config,policy,deadline,emit=None):
    from physical_executor import expected_stages
    started=time.monotonic()
    if (not plan.get('valid') or tuple(s['name'] for s in plan['stages'])!=expected_stages(plan) or
        not all(s['certificate'].get('valid') for s in plan['stages'])):
        return dict(valid=False,reason='COMPLETE_CERTIFIED_NOMINAL_PLAN_REQUIRED')
    action=plan['action']
    if action['placement']['snapshot_id']!=scene.snapshot.snapshot_id or action['placement']['config_revision']!=config.revision:
        return dict(valid=False,reason='SCORE_INPUT_IDENTITY_MISMATCH')
    support=_support_trace(scene,state,action,config,deadline)
    if not support['valid']:return support
    scores=[];previous=None
    for stage in plan['stages']:
        name=stage['name'];trajectory=stage['trajectory'];path=trajectory['positions']
        if (trajectory.get('interpolation')!='cubic_zero_velocity_knots' or
            not np.allclose(trajectory['velocities'],0.,rtol=0,atol=0)):
            return dict(valid=False,reason='SCORE_INTERPOLATION_NOT_SUPPORTED',stage=name)
        if previous is not None and not np.allclose(previous,path[0],rtol=0,atol=1e-8):
            return dict(valid=False,reason='DISCONNECTED_SCORE_STAGE',stage=name)
        previous=path[-1]
        box=(state.container_T_box if name in ('pick_pre','pick_contact') else
             action['placement']['final_box_pose'] if name=='retreat' else action['handoff_box_pose'])
        scene.set_payload(state.dimensions,stage['payload_mode'],
            scene.world_container@np.array(box).reshape(4,4),
            np.array(action['box_T_tool']).reshape(4,4),allow_support=name!='invert_and_approach')
        sampler=FullArmDistanceSampler(scene,policy.distance_numeric_error_m,
            _push_motion(scene,stage,action) if name=='push' else None)
        if emit:emit(dict(event='clearance_stage_begin',stage=name))
        result=bound_joint_path(path,lambda i,u,q:sampler.evaluate(i,u,q,stage['contact_tool']),
            sampler.motion_bound,config.collision_margin_m,policy.clearance_precision_m,deadline,
            (lambda row:emit(dict(row,stage=name))) if emit else None)
        if not result['valid']:return dict(valid=False,reason='PLAN_CLEARANCE_SCORE_REJECTED',stage=name,diagnostic=result)
        scores.append(dict(stage=name,**result))
    minimum=min(scores,key=lambda r:r['robot_lower_m'])
    payloads=[r for r in scores if r['payload_lower_m'] is not None]
    return dict(valid=True,action_id=action['action_id'],plan_digest=plan_digest(plan),
        snapshot_id=scene.snapshot.snapshot_id,config_revision=config.revision,
        clearance_policy_revision=policy.clearance_revision,model_identity=model_identity(scene),
        distance_numeric_error_m=policy.distance_numeric_error_m,
        robot_lower_m=minimum['robot_lower_m'],robot_upper_m=min(r['robot_upper_m'] for r in scores),
        robot_witness=dict(stage=minimum['stage'],**minimum['robot_lower_witness']),
        payload_lower_m=min((r['payload_lower_m'] for r in payloads),default=None),
        min_com_margin_m=support['min_com_margin_m'],min_support_area_ratio=support['min_area_ratio'],
        support_trace=support,stages=scores,elapsed_s=time.monotonic()-started,
        push_seam_events=trace_push_seams(action['handoff_box_pose'],
            action['placement']['final_box_pose'],state.dimensions,scene.snapshot.surfaces),
        scope='nominal_complete_path_clearance_only_robust_tube_and_physics_are_separate_gates')
