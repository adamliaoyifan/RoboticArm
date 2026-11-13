#!/usr/bin/env python3
"""Request-driven upper placement planner. No robot or simulator commands.

Private scenes/workers die at the request boundary. Persisted plans are bound to
request, snapshot, model, policies and source revision; execution revalidates them.
"""
import argparse
from collections import Counter
import platform
from importlib.metadata import version
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import subprocess
import time
import numpy as np
from assets import load,sha256
from full_arm import FullArmScene
from placement_request import PlacementRequest,target_actions
from planning_types import (UpperConfig,BoxState,SceneSnapshot,OccupiedBox,SurfacePatch,digest)
from planning_worker import plan_bounded,atomic_json
from selection_policy import SelectionPolicy
from execution_policy import ExecutionPolicy
from plan_clearance import model_identity
from plan_selection import rank_nominal_plans
from upper_prior import (UpperPlacementPrior,PlacementGoal,PriorQueryBudget,
    apply_prior_to_actions,compose_seed_list)


def request_from_record(record):
    row=dict(record);box=dict(row.pop('box'));snapshot=dict(row.pop('snapshot'))
    for key in ('dimensions','container_T_box','com_local'):box[key]=tuple(box[key])
    snapshot['occupied']=tuple(OccupiedBox(r['name'],tuple(r['center']),tuple(r['size'])) for r in snapshot['occupied'])
    snapshot['surfaces']=tuple(SurfacePatch(r['source_id'],tuple(r['bounds_xy']),r['surface_z_container']) for r in snapshot['surfaces'])
    for key in ('grid_origin','grid_shape','observed_bounds'):snapshot[key]=tuple(snapshot[key])
    row['start_q']=tuple(row['start_q']);row['support_source_ids']=tuple(row['support_source_ids'])
    row['final_box_poses']=tuple(tuple(p) for p in row['final_box_poses'])
    if row.get('confirmed_box_T_tool') is not None:row['confirmed_box_T_tool']=tuple(row['confirmed_box_T_tool'])
    return PlacementRequest(box=BoxState(**box),snapshot=SceneSnapshot(**snapshot),**row)


class UpperPlacementPlanner:
    """Stateless public boundary. Each call reloads runtime YAML configuration."""
    def __init__(self,repo,scene_path,config_path=None,selection_path=None,execution_path=None):
        self._repo=Path(repo).resolve();self._scene_path=Path(scene_path).resolve()
        self._config_path=config_path;self._selection_path=selection_path;self._execution_path=execution_path
        common=Path(subprocess.check_output(['git','-C',str(self._repo),'rev-parse',
            '--path-format=absolute','--git-common-dir'],text=True).strip())
        self._evidence_root=common.parent/'docs/status/evidence'

    def plan(self,request,output,seed=0):
        output=Path(output).resolve()
        if not output.is_relative_to(self._evidence_root):raise ValueError('PRIMARY_EVIDENCE_REQUIRED')
        output.mkdir(parents=True,exist_ok=False);source=output/'source';source.mkdir()
        config=UpperConfig.load(self._config_path);selection=SelectionPolicy.load(self._selection_path)
        execution=ExecutionPolicy.load(self._execution_path)
        hashes={}
        for p in Path(__file__).parent.glob('*.py'):
            shutil.copy2(p,source/p.name);hashes[p.name]=sha256(p)
        for p in Path(__file__).with_name('native').iterdir():
            if p.is_file():shutil.copy2(p,source/p.name);hashes['native/'+p.name]=sha256(p)
        import yaml
        for name,obj in (('upper_config',config),('selection_policy',selection),('execution_policy',execution)):
            p=source/(name+'.yaml');p.write_text(yaml.safe_dump(asdict(obj)));hashes[p.name]=sha256(p)
        atomic_json(output/'request.json',asdict(request))
        started=time.monotonic();deadline=started+config.search_budget_s
        manifest=dict(revision=subprocess.check_output(['git','-C',str(self._repo),'rev-parse','HEAD'],text=True).strip(),
            dirty_count=len(subprocess.check_output(['git','-C',str(self._repo),'status','--porcelain'],text=True).splitlines()),
            started_wall=time.time(),request_id=request.request_id,request_revision=request.revision,
            source_hashes=hashes,config=asdict(config),selection_policy=asdict(selection),execution_policy=asdict(execution),
            execution_policy_revision=execution.revision,selection_policy_revision=selection.revision,
            scene_config=str(self._scene_path),scene_config_sha256=sha256(self._scene_path),seed=seed,
            prior_enabled=config.prior_enabled,prior_index_path=config.prior_index_path or None,
            python=platform.python_version(),dependency_versions={n:version(n) for n in
                ('numpy','scipy','python-fcl','ompl','trimesh','shapely')},
            capture_complete=False,replay_possible=False,scored_units=0,profile='ideal_snapshot_request_driven')
        atomic_json(output/'manifest.json',manifest)
        try:
            assets=load(self._repo,self._scene_path)
            if request.snapshot.geometry_hash!=assets['hull'].geometry_hash:raise ValueError('SNAPSHOT_HULL_MISMATCH')
            scene=FullArmScene(assets,request.snapshot,config)
            limits=np.array(scene.urdf.joint_limits())
            if np.any(np.array(request.start_q)<limits[:,0]) or np.any(np.array(request.start_q)>limits[:,1]):
                raise ValueError('START_JOINT_LIMIT')
            atomic_json(output/'inputs.json',dict(snapshot=asdict(request.snapshot),box=asdict(request.box)))
            atomic_json(output/'models.json',dict(hashes=scene.model_hashes,asset_hashes=assets['hashes']))
            with (output/'request_boundaries.jsonl').open('w') as trace:
                def emit(row):trace.write(json.dumps(row,allow_nan=False)+'\n');trace.flush()
                hints=None
                if config.prior_enabled:
                    prior=UpperPlacementPrior(assets,index_path=config.prior_index_path or None)
                    goals=tuple(PlacementGoal(f'goal_{index}',tuple(value))
                                for index,value in enumerate(request.final_box_poses))
                    prior_started=time.monotonic()
                    hints=prior.query(request,goals,PriorQueryBudget.from_config(config,execution))
                    atomic_json(output/'prior_query.json',hints.as_record())
                    emit(dict(event='prior_query',elapsed_s=time.monotonic()-prior_started,
                        fallback_required=hints.diagnostics_dict().get('fallback_required'),
                        index_status=hints.diagnostics_dict().get('index_status')))
                actions,report=target_actions(assets,request,config,emit,deadline)
                if hints is not None:
                    ordered,dropped=apply_prior_to_actions(actions,hints)
                    for row in dropped:emit(dict(event='prior_necessary_rejection',**row))
                    emit(dict(event='prior_order_applied',generated=len(actions),
                        kept=len(ordered),dropped=len(dropped)))
                    actions=ordered
                atomic_json(output/'candidates.json',[asdict(a) for a in actions]);atomic_json(output/'geometry_report.json',report)
                rng=np.random.default_rng(seed)
                random_seeds=rng.uniform(limits[:,0],limits[:,1],size=(config.ik_seed_count-1,6)).tolist()
                plans=[];attempts=0;ranking=None;rejections=Counter();common_failure=None
                atomic_json(output/'plans.json',plans)
                for action in actions:
                    remaining=deadline-time.monotonic()
                    if remaining<=0:break
                    attempts+=1
                    emit(dict(event='action_input',action_id=action.action_id,remaining_s=remaining))
                    seeds=compose_seed_list(request.start_q,config,hints,action,random_seeds)
                    result=plan_bounded(self._repo,request.snapshot,request.box,action,list(request.start_q),
                        seeds,config,selection,execution,output/f'action_{attempts:04d}',remaining,seed+attempts,
                        start_attached=request.start_attached,scene_path=self._scene_path)
                    emit(dict(event='action_output',action_id=action.action_id,valid=result.get('valid'),reason=result.get('reason')))
                    if not result.get('valid'):rejections[result.get('reason') or 'ACTION_PLANNING_REJECTED']+=1
                    if result.get('reason')=='INITIAL_STATE_INVALID':
                        common_failure='INITIAL_STATE_INVALID'
                        emit(dict(event='common_initial_state_rejected',diagnostic=result.get('diagnostic')))
                        break
                    if result.get('valid'):
                        result['request_revision']=request.revision
                        plans.append(result);atomic_json(output/'plans.json',plans)
                        ranking=rank_nominal_plans(plans,request.snapshot.snapshot_id,config,selection,model_identity(scene),execution)
                        atomic_json(output/'selection.json',ranking)
                        if ranking['achieved']>=config.solution_count_target:break
                if ranking is None:
                    ranking=rank_nominal_plans(plans,request.snapshot.snapshot_id,config,selection,model_identity(scene),execution)
                    atomic_json(output/'selection.json',ranking)
            result=dict(valid=ranking['nominal_minimum_met'],request_revision=request.revision,
                reason='COMPLETE_SCORED_PLANS' if ranking['nominal_minimum_met'] else
                    common_failure if common_failure else 'SEARCH_BUDGET_EXHAUSTED' if time.monotonic()>=deadline else 'NO_COMPLETE_PLAN_IN_ACTION_FAMILY',
                complete_plans=ranking['achieved'],attempted_actions=attempts,geometry_candidates=len(actions),
                selected_index=ranking['selected_index'],rejection_counts=dict(rejections),elapsed_s=time.monotonic()-started,
                physical_execution='not_evaluated',contact_uncertainty_admission='not_evaluated')
            atomic_json(output/'summary.json',result)
            manifest.update(capture_complete=True,replay_possible=True)
            return dict(result,plans=plans,selection=ranking)
        except BaseException:
            import traceback
            (output/'error.txt').write_text(traceback.format_exc());raise
        finally:
            manifest['finished_wall']=time.time()
            atomic_json(output/'manifest.json',manifest)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request',type=Path,required=True);parser.add_argument('--scene-config',type=Path,required=True)
    parser.add_argument('--evidence',type=Path,required=True);parser.add_argument('--config',type=Path)
    parser.add_argument('--selection-policy',type=Path);parser.add_argument('--execution-policy',type=Path)
    parser.add_argument('--seed',type=int,default=0);args=parser.parse_args()
    repo=Path(__file__).resolve().parents[2]
    planner=UpperPlacementPlanner(repo,args.scene_config,args.config,args.selection_policy,args.execution_policy)
    result=planner.plan(request_from_record(json.loads(args.request.read_text())),args.evidence,args.seed)
    print(json.dumps({k:v for k,v in result.items() if k not in ('plans','selection')}));return 0 if result['valid'] else 2


if __name__=='__main__':raise SystemExit(main())
