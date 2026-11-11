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
    parser.add_argument('--plan-paths',action='store_true')
    args=parser.parse_args()
    output=args.evidence.resolve()
    if not output.is_relative_to(EVIDENCE):raise ValueError('primary evidence required')
    output.mkdir(parents=True,exist_ok=False)
    config=UpperConfig.load(args.config);runtime=None
    source=output/'source';source.mkdir()
    hashes={}
    for p in Path(__file__).parent.glob('*.py'):
        shutil.copy2(p,source/p.name);hashes[p.name]=sha256(p)
    shutil.copy2(args.config or Path(__file__).with_name('upper_config.yaml'),source/'upper_config.yaml')
    for native in Path(__file__).with_name('native').glob('*'):
        shutil.copy2(native,source/native.name);hashes['native/'+native.name]=sha256(native)
    manifest=dict(started_wall=time.time(),revision=subprocess.check_output(
        ['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),source_hashes=hashes,
        config=asdict(config),seed=args.seed,command=list(os.sys.argv),capture_complete=False,replay_possible=False,
        dirty_count=len(subprocess.check_output(['git','-C',str(ROOT),'status','--porcelain'],text=True).splitlines()),
        dependency_versions={name:importlib.metadata.version(name) for name in ('python-fcl','ompl','numpy','scipy','shapely','trimesh')},
        profile='ideal_sim_snapshot_source_mesh_diagnostic',scored_units=0)
    save(output/'manifest.json',manifest)
    try:
        assets=load(ROOT)
        if args.observation:
            observation=json.loads(args.observation.read_text())
        else:
            runtime=RuntimeIO(ROOT,output,args.scene_dir);runtime.start();observation=runtime.snapshot()
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
        scene=FullArmScene(assets,snapshot,config)
        save(output/'models.json',dict(hashes=scene.model_hashes,self_disabled=[sorted(x) for x in scene.disabled],
            source_srdf_disabled=[sorted(x) for x in scene.source_disabled],
            self_policy='adjacent_or_fixed_assembly_only_no_sampled_never_pairs'))
        if args.candidates:
            previous=json.loads((args.candidates.parent/'manifest.json').read_text())
            for dependency in ('candidates.py','support_geometry.py','geometry.py','ideal_snapshot.py','assets.py','action.py'):
                if previous['source_hashes'][dependency]!=hashes[dependency]:
                    raise ValueError('CANDIDATE_GEOMETRY_SOURCE_MISMATCH:'+dependency)
            actions=[action_from_record(row) for row in json.loads(args.candidates.read_text())]
            old_config=previous['config'];new_config=asdict(config)
            changed=[key for key in new_config if digest(new_config[key])!=digest(old_config[key])]
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
                                    handoff=action.handoff_box_pose,push=action.push_start_tool_pose))[:20]
                    rebound.append(replace(action,action_id=aid,placement=placement))
                actions=rebound
            report=dict(reused_from=str(args.candidates),sha256=sha256(args.candidates),
                        rebound_search_only_fields=changed,
                        scope='geometry_only_full_paths_checked_again')
        else:
            actions,report=generate_actions(assets,snapshot,state,config)
        save(output/'candidates.json',[asdict(a) for a in actions])
        save(output/'geometry_report.json',report)
        rng=np.random.default_rng(args.seed);limits=np.array(scene.urdf.joint_limits())
        seeds=[observation['q']]+list(rng.uniform(limits[:,0],limits[:,1],size=(config.ik_seed_count-1,6)))
        if args.plan_paths:
            from planning_worker import plan_bounded
            plans=[];attempted=set();start=time.monotonic()
            with (output/'stage_boundaries.jsonl').open('w') as trace:
                def emit(row):
                    trace.write(json.dumps(row)+'\n');trace.flush()
                for action in actions:
                    if action.action_id in attempted:continue
                    remaining=config.search_budget_s-(time.monotonic()-start)
                    if remaining<=0:break
                    attempted.add(action.action_id)
                    emit(dict(event='action_input',action=asdict(action)))
                    result=plan_bounded(ROOT,snapshot,state,action,observation['q'],seeds,config,
                        output/f'action_{len(attempted):04d}',remaining,args.seed)
                    emit(dict(event='action_output',action_id=action.action_id,
                              valid=result['valid'],reason=result.get('reason'),
                              diagnostic=result.get('diagnostic')))
                    if result['valid']:plans.append(result)
                    if len(plans)>=config.solution_count_target:break
            save(output/'plans.json',plans)
            save(output/'summary.json',dict(stage='connected_path_search',
                attempted_actions=len(attempted),complete_nominal_plans=len(plans),
                physical_execution='not_evaluated'))
            manifest['capture_complete']=True;manifest['replay_possible']=True
            print((output/'summary.json').read_text());return
        checked=set();counts={};positive=[];start=time.monotonic()
        with (output/'endpoint_attempts.jsonl').open('w') as stream:
            for action in actions:
                # Endpoint failure is reusable only for this exact pose/scene.
                target=tuple(np.round(action.handoff_tool_pose,8))
                if target in checked:continue
                checked.add(target)
                solutions,attempt=scene.endpoint_solutions(action.handoff_tool_pose,seeds,
                    config.collision_margin_m,config.path_attempt_budget_s)
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
