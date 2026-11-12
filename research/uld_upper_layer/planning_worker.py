"""Process boundary for bounded native collision/planning calls.

An interrupted native query can swallow a Python exception. This boundary keeps
its exact input durable and lets the parent enforce the existing search budget.
This is diagnostic instrumentation, not a mechanism for accepting partial paths.
"""
from dataclasses import asdict
import json
import multiprocessing
from pathlib import Path
import time
import traceback


def atomic_json(path,value):
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value,allow_nan=False)+'\n')
    temporary.replace(path)


def _worker(repo,snapshot,state,action,start_q,seeds,config,selection_policy,execution_policy,output,budget_s,seed,start_attached=False,scene_path=None):
    from assets import load
    from full_arm import FullArmScene
    from stage_planner import plan_action
    from plan_clearance import score_plan
    from free_box_validation import prepare_free_box_motion
    from ompl import util as ou
    ou.RNG.setSeed(seed+1)
    output=Path(output)
    deadline=time.monotonic()+budget_s
    try:
        assets=load(repo,scene_path)
        scene=FullArmScene(assets,snapshot,config)
        scene.query_observer=lambda row:atomic_json(output/'last_query.json',row)
        with (output/'stage_boundaries.jsonl').open('w') as stream:
            def emit(row):
                stream.write(json.dumps(row)+'\n');stream.flush()
                atomic_json(output/'last_boundary.json',row)
            result=plan_action(scene,state,action,start_q,seeds,config,emit,start_attached=start_attached)
            if result.get('valid'):
                # Preserve the complete nominal path even if scoring times out.
                # A nominal path without its completed score is never admitted.
                atomic_json(output/'nominal_result.json',result)
                prepared=prepare_free_box_motion(assets,snapshot,state,result,config,selection_policy,
                    execution_policy,deadline,emit,scene.query_observer)
                atomic_json(output/'envelope_result.json',prepared)
                if not prepared['valid']:
                    result=prepared['plan'];result['reason']='FREE_BOX_ENVELOPE_REJECTED'
                    result['valid']=False
                    result['envelope_diagnostic']=dict(reason=prepared['reason'],diagnostic=prepared.get('diagnostic'))
                else:
                    result=prepared['plan']
                    atomic_json(output/'envelope_prepared_result.json',result)
            if result.get('valid'):
                score_scene=FullArmScene(assets,snapshot,config)
                score_scene.query_observer=scene.query_observer
                with (output/'clearance_intervals.jsonl').open('w') as clearance_stream:
                    def score_emit(row):
                        clearance_stream.write(json.dumps(row)+'\n');clearance_stream.flush()
                        atomic_json(output/'last_boundary.json',row)
                    score=score_plan(score_scene,state,result,config,selection_policy,deadline,score_emit)
                result['clearance_score']=score
                result['valid']=bool(score.get('valid'))
                if not result['valid']:result['reason']='COMPLETE_PATH_CLEARANCE_REJECTED'
        if not result.get('valid') and not result.get('reason'):
            result['reason']=(result.get('diagnostic') or {}).get('reason','ACTION_PLANNING_REJECTED')
        result['search_observed_clearance_lower_bound_m']=scene.minimum_clearance_m
        result['clearance_score_status']=('complete' if result.get('clearance_score',{}).get('valid')
                                          else 'not_accepted')
        result['clearance_witness']=scene.clearance_witness
        atomic_json(output/'result.json',result)
    except BaseException:
        (output/'error.txt').write_text(traceback.format_exc())
        raise


def plan_bounded(repo,snapshot,state,action,start_q,seeds,config,selection_policy,execution_policy,output,budget_s,seed,start_attached=False,scene_path=None):
    output=Path(output);output.mkdir()
    atomic_json(output/'input.json',dict(snapshot=asdict(snapshot),box=asdict(state),
        action=asdict(action),start_q=start_q,seeds=[list(q) for q in seeds],
        config=asdict(config),selection_policy=asdict(selection_policy),execution_policy=asdict(execution_policy),budget_s=budget_s,seed=seed,start_attached=start_attached,scene_path=str(scene_path) if scene_path else None))
    process=multiprocessing.get_context('spawn').Process(target=_worker,
        args=(repo,snapshot,state,action,start_q,seeds,config,selection_policy,execution_policy,output,budget_s,seed,start_attached,scene_path))
    process.start();started=time.monotonic();abort=None
    try:
        while process.is_alive():
            process.join(timeout=.1)
            if time.monotonic()-started>budget_s:
                abort='SEARCH_BUDGET_EXHAUSTED';break
            query=output/'last_query.json'
            if query.exists():
                row=json.loads(query.read_text())
                if row['phase']=='begin' and time.time()-row['wall']>config.distance_query_timeout_s:
                    abort='NATIVE_DISTANCE_QUERY_TIMEOUT';break
    except BaseException:
        if process.is_alive():process.terminate();process.join(timeout=3)
        if process.is_alive():process.kill();process.join()
        atomic_json(output/'watchdog.json',dict(reason='PARENT_INTERRUPTED',exitcode=process.exitcode,
                    elapsed_s=time.monotonic()-started,physical_execution='not_evaluated'))
        raise
    if abort:
        process.terminate();process.join(timeout=3)
        if process.is_alive():process.kill();process.join()
    result_path=output/'result.json'
    if abort:
        result=dict(valid=False,reason=abort,exitcode=process.exitcode,
                    completed_result_path=str(result_path) if result_path.exists() else None)
    elif result_path.exists():result=json.loads(result_path.read_text())
    else:result=dict(valid=False,reason=abort or 'WORKER_FAILED',exitcode=process.exitcode)
    atomic_json(output/'watchdog.json',dict(reason=abort,exitcode=process.exitcode,
                elapsed_s=time.monotonic()-started,physical_execution='not_evaluated'))
    return result
