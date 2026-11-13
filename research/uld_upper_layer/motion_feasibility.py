#!/usr/bin/env python3
"""Bounded source-first ideal-input whole-arm experiment, no hardware target."""
import argparse
from dataclasses import asdict,replace
import json
import importlib.metadata
import os
from pathlib import Path
import shutil
import subprocess
import time
import traceback
import numpy as np
from assets import load,sha256
from candidates import generate_actions,matrix
from full_arm import FullArmScene
from ideal_snapshot import capture_fixture,from_observation
from planning_types import UpperConfig,action_from_record,digest
from runtime_io import RuntimeIO
from selection_policy import SelectionPolicy
from execution_policy import ExecutionPolicy
from plan_clearance import model_identity
from plan_selection import rank_nominal_plans

ROOT=Path(__file__).resolve().parents[2]
EVIDENCE=Path('/home/adamliao/work/elfin_humble_ws/docs/status/evidence')


def save(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-dir',type=Path,required=True)
    parser.add_argument('--evidence',type=Path,required=True)
    parser.add_argument('--observation',type=Path)
    parser.add_argument('--capture-only',action='store_true')
    parser.add_argument('--candidates',type=Path)
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--config',type=Path)
    parser.add_argument('--scene-config',type=Path)
    parser.add_argument('--selection-policy',type=Path)
    parser.add_argument('--execution-policy',type=Path)
    parser.add_argument('--plan-paths',action='store_true')
    args=parser.parse_args()
    output=args.evidence.resolve()
    if not output.is_relative_to(EVIDENCE):raise ValueError('primary evidence required')
    output.mkdir(parents=True,exist_ok=False)
    config=UpperConfig.load(args.config);selection_policy=SelectionPolicy.load(args.selection_policy)
    execution_policy=ExecutionPolicy.load(args.execution_policy);runtime=None
    source=output/'source';source.mkdir()
    hashes={}
    for p in Path(__file__).parent.glob('*.py'):
        shutil.copy2(p,source/p.name);hashes[p.name]=sha256(p)
    shutil.copy2(args.config or Path(__file__).with_name('upper_config.yaml'),source/'upper_config.yaml')
    hashes['upper_config.yaml']=sha256(source/'upper_config.yaml')
    shutil.copy2(args.selection_policy or Path(__file__).with_name('selection_policy.yaml'),source/'selection_policy.yaml')
    hashes['selection_policy.yaml']=sha256(source/'selection_policy.yaml')
    shutil.copy2(args.execution_policy or Path(__file__).with_name('execution_policy.yaml'),source/'execution_policy.yaml')
    hashes['execution_policy.yaml']=sha256(source/'execution_policy.yaml')
    if (ExecutionPolicy.load(source/'execution_policy.yaml')!=execution_policy or
        UpperConfig.load(source/'upper_config.yaml')!=config or
        SelectionPolicy.load(source/'selection_policy.yaml')!=selection_policy):
        raise ValueError('CONFIG_CHANGED_DURING_CAPTURE')
    for native in Path(__file__).with_name('native').glob('*'):
        shutil.copy2(native,source/native.name);hashes['native/'+native.name]=sha256(native)
    manifest=dict(started_wall=time.time(),revision=subprocess.check_output(
        ['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),source_hashes=hashes,
        config=asdict(config),selection_policy=asdict(selection_policy),
        selection_policy_revision=selection_policy.revision,
        execution_policy=asdict(execution_policy),execution_policy_revision=execution_policy.revision,
        seed=args.seed,command=list(os.sys.argv),capture_complete=False,replay_possible=False,
        dirty_count=len(subprocess.check_output(['git','-C',str(ROOT),'status','--porcelain'],text=True).splitlines()),
        dependency_versions={name:importlib.metadata.version(name) for name in ('python-fcl','ompl','numpy','scipy','shapely','trimesh')},
        profile='ideal_sim_snapshot_source_mesh_diagnostic',scored_units=0)
    save(output/'manifest.json',manifest)
    try:
        assets=load(ROOT,args.scene_config)
        if args.observation:
            observation=json.loads(args.observation.read_text())
        else:
            runtime=RuntimeIO(ROOT,output,args.scene_dir,scene_path=args.scene_config);runtime.start();observation=runtime.snapshot()
        save(output/'observation.json',observation)
        if args.capture_only:
            result=dict(stage='capture',lower_count=sum(n.startswith('lower_') for n in observation['poses']),
                        q=observation['q'],stamp=observation['pose_stamp'],capture_error=observation['capture_error'],
                        contact_messages=observation.get('contact_messages',0),
                        complete_contact_messages=observation.get('complete_contact_messages',0),
                        contact_fields_status='observed' if observation.get('complete_contact_messages',0)>0
                            else 'not_evaluated_missing_native_normal_or_depth')
            if result['lower_count']!=24 or result['q'] is None or result['capture_error']:
                raise RuntimeError('CAPTURE_INCOMPLETE')
            save(output/'summary.json',result);manifest['capture_complete']=True;manifest['replay_possible']=True
            print(json.dumps(result));return
        if runtime:
            runtime.stop();runtime=None # Offline planning does not occupy Gazebo.
        snapshot,state=capture_fixture(args.scene_dir,assets,config)
        snapshot,state=from_observation(snapshot,state,observation,assets)
        save(output/'inputs.json',dict(snapshot=asdict(snapshot),box=asdict(state)))
        planning_started=time.monotonic()
        planning_deadline=planning_started+config.search_budget_s
        scene=FullArmScene(assets,snapshot,config)
        save(output/'models.json',dict(hashes=scene.model_hashes,asset_hashes=assets['hashes'],self_disabled=[sorted(x) for x in scene.disabled],
            source_srdf_disabled=[sorted(x) for x in scene.source_disabled],
            self_policy='adjacent_or_fixed_assembly_only_no_sampled_never_pairs'))
        if args.candidates:
            previous=json.loads((args.candidates.parent/'manifest.json').read_text())
            for dependency in ('candidates.py','support_geometry.py','geometry.py','ideal_snapshot.py','assets.py','action.py','tool_contact_geometry.py','planning_types.py'):
                if previous['source_hashes'].get(dependency)!=hashes[dependency]:
                    raise ValueError('CANDIDATE_GEOMETRY_SOURCE_MISMATCH:'+dependency)
            previous_models=json.loads((args.candidates.parent/'models.json').read_text())
            if previous_models.get('asset_hashes')!=assets['hashes'] or previous_models.get('hashes')!=scene.model_hashes:
                raise ValueError('CANDIDATE_SOURCE_MODELS_MISMATCH')
            actions=[action_from_record(row) for row in json.loads(args.candidates.read_text())]
            old_config=previous['config'];new_config=asdict(config)
            changed=[key for key in new_config if key not in old_config or digest(new_config[key])!=digest(old_config[key])]
            if any(key not in ('search_budget_s','path_attempt_budget_s') for key in changed):
                raise ValueError('CANDIDATE_GEOMETRY_CONFIG_MISMATCH:'+str(changed))
            if any(a.placement.snapshot_id!=snapshot.snapshot_id or
                   a.placement.config_revision!=digest(old_config) for a in actions):
                raise ValueError('CANDIDATE_SNAPSHOT_OR_CONFIG_MISMATCH')
            if changed:
                rebound=[]
                for action in actions:
                    pid=digest(dict(pose=action.placement.final_box_pose,snapshot=snapshot.snapshot_id,
                                    config=config.revision))[:20]
                    placement=replace(action.placement,placement_id=pid,config_revision=config.revision)
                    aid=digest(dict(pid=pid,grasp=action.box_T_tool,
                                    handoff=action.handoff_box_pose,push=action.push_start_tool_pose,
                                    push_geometry=action.push_contact_geometry_id))[:20]
                    rebound.append(replace(action,action_id=aid,placement=placement))
                actions=rebound
            report=dict(reused_from=str(args.candidates),sha256=sha256(args.candidates),
                        rebound_search_only_fields=changed,
                        scope='geometry_only_full_paths_checked_again')
        else:
            with (output/'geometry_progress.jsonl').open('w') as progress:
                def geometry_emit(row):
                    row=dict(row)
                    if 'action' in row:row['action']=asdict(row['action'])
                    progress.write(json.dumps(row)+'\n');progress.flush()
                actions,report=generate_actions(assets,snapshot,state,config,geometry_emit,planning_deadline)
        save(output/'candidates.json',[asdict(a) for a in actions])
        save(output/'geometry_report.json',report)
        rng=np.random.default_rng(args.seed);limits=np.array(scene.urdf.joint_limits())
        seeds=[observation['q']]+list(rng.uniform(limits[:,0],limits[:,1],size=(config.ik_seed_count-1,6)))
        if args.plan_paths:
            from planning_worker import plan_bounded
            from action_screen import push_screen_key
            plans=[];attempted=set();screen_rejections={};reused_screens=0;start=planning_started
            selection=rank_nominal_plans(plans,snapshot.snapshot_id,config,selection_policy,model_identity(scene),execution_policy)
            with (output/'stage_boundaries.jsonl').open('w') as trace:
                def emit(row):
                    trace.write(json.dumps(row)+'\n');trace.flush()
                for action in actions:
                    if action.action_id in attempted:continue
                    remaining=config.search_budget_s-(time.monotonic()-start)
                    if remaining<=0:break
                    attempted.add(action.action_id)
                    emit(dict(event='action_input',action=asdict(action)))
                    screen_key=push_screen_key(snapshot,state,action,seeds,config)
                    if screen_key in screen_rejections:
                        reused_screens+=1
                        emit(dict(event='sampled_push_screen_reused',action_id=action.action_id,
                            source_action_id=screen_rejections[screen_key],key=screen_key,
                            exhaustive=False,valid=False))
                        continue
                    result=plan_bounded(ROOT,snapshot,state,action,observation['q'],seeds,config,selection_policy,execution_policy,
                        output/f'action_{len(attempted):04d}',remaining,args.seed,scene_path=args.scene_config)
                    emit(dict(event='action_output',action_id=action.action_id,
                              valid=result['valid'],reason=result.get('reason'),
                              diagnostic=result.get('diagnostic')))
                    if result.get('reason')=='NO_SAMPLED_PUSH_ENDPOINT_SOLUTION':
                        screen_rejections[screen_key]=action.action_id
                    if result['valid']:
                        plans.append(result)
                        selection=rank_nominal_plans(plans,snapshot.snapshot_id,config,selection_policy,model_identity(scene),execution_policy)
                        # Persist completed evidence before trying another action.
                        save(output/'plans.json',plans)
                        save(output/'selection.json',selection)
                    if selection['achieved']>=config.solution_count_target:break
            save(output/'plans.json',plans);save(output/'selection.json',selection)
            save(output/'summary.json',dict(stage='connected_path_search',
                attempted_actions=len(attempted),reused_sampled_push_rejections=reused_screens,
                complete_nominal_plans=len(plans),distinct_scored_nominal_plans=selection['achieved'],
                nominal_minimum_met=selection['nominal_minimum_met'],
                physical_execution='not_evaluated',search_elapsed_s=time.monotonic()-planning_started,
                search_budget_s=config.search_budget_s,termination=('SOLUTION_TARGET' if selection['achieved']>=config.solution_count_target
                    else 'BUDGET_EXHAUSTED' if time.monotonic()>=planning_deadline else 'CANDIDATES_EXHAUSTED')))
            manifest['capture_complete']=True;manifest['replay_possible']=True
            print((output/'summary.json').read_text());return
        checked=set();counts={};positive=[];start=planning_started
        with (output/'endpoint_attempts.jsonl').open('w') as stream:
            for action in actions:
                # Endpoint failure is reusable only for this exact pose/scene.
                target=tuple(np.round(action.handoff_tool_pose,8))
                if target in checked:continue
                remaining=planning_deadline-time.monotonic()
                if remaining<=0:break
                checked.add(target)
                solutions,attempt=scene.endpoint_solutions(action.handoff_tool_pose,seeds,
                    config.collision_margin_m,min(config.path_attempt_budget_s,remaining))
                row=dict(action_id=action.action_id,stage='handoff',solutions=solutions,**attempt)
                stream.write(json.dumps(row)+'\n');stream.flush()
                counts['handoff_checked']=counts.get('handoff_checked',0)+1
                if solutions:positive.append(dict(action=asdict(action),q_solutions=solutions))
                if len(positive)>=config.solution_count_target or time.monotonic()-start>=config.search_budget_s:break
        save(output/'endpoint_candidates.json',positive)
        save(output/'summary.json',dict(stage='endpoint_screen',**counts,
            collision_free_handoff_poses=len(positive),complete_plans=0,
            connected_path='not_evaluated',physical_execution='not_evaluated'))
        manifest['capture_complete']=True;manifest['replay_possible']=True
        print((output/'summary.json').read_text())
    except BaseException:
        (output/'error.txt').write_text(traceback.format_exc());raise
    finally:
        if runtime:
            save(output/'terminal_observation.json',runtime.snapshot())
            runtime.stop()
        manifest['ended_wall']=time.time();save(output/'manifest.json',manifest)


if __name__=='__main__':main()
