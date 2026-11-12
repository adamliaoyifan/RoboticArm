"""Plan from one live attached snapshot while observing the held robot.

The child process computes geometry/IK/paths; the parent continuously checks
native contacts and lower cargo. No robot or cargo command is issued here.
"""
from dataclasses import asdict,replace
import json,os,signal,subprocess,sys,time
from pathlib import Path
import numpy as np
from ideal_snapshot import from_observation
from request_planner import request_from_record
from plan_artifact import load_scored_plan
from execution_monitor import pose_error


def validate_held_start(request,sample,config,policy):
    joint_error=float(np.max(np.abs(np.array(request.start_q)-sample.q)))
    distance,angle=pose_error(request.box.container_T_box,sample.container_T_box)
    if (joint_error>policy.initial_joint_error_rad or distance>config.contact_penetration_m or
            angle>policy.initial_rotation_deg):
        raise ValueError('LIVE_START_CHANGED_DURING_PLANNING')
    return dict(joint_max_error_rad=joint_error,payload_translation_m=distance,payload_rotation_deg=angle)


def plan_while_held(runtime,monitor,observe,assets,snapshot,state,template_path,output,config,policy,
                    scene_path,seed=0):
    from planning_pause import paused_for_planning
    with paused_for_planning(runtime,observe):
        return _plan_while_held(runtime,monitor,observe,assets,snapshot,state,template_path,
                               output,config,policy,scene_path,seed)


def _plan_while_held(runtime,monitor,observe,assets,snapshot,state,template_path,output,config,policy,
                     scene_path,seed):
    import rclpy
    output=Path(output)
    template=request_from_record(json.loads(Path(template_path).read_text()))
    row=runtime.snapshot();snapshot,state=from_observation(snapshot,state,row,assets)
    if tuple(template.box.dimensions)!=state.dimensions:raise ValueError('LIVE_BOX_DIMENSIONS_DIFFER_FROM_REQUEST')
    if template.box.mass_kg is not None and template.box.mass_kg!=state.mass_kg:
        raise ValueError('LIVE_PAYLOAD_MASS_DIFFERS_FROM_REQUEST')
    sample=monitor.sample
    if not sample.attached or sample.tool_T_box_at_attach is None:
        raise ValueError('LIVE_PLANNING_REQUIRES_CONFIRMED_ATTACHMENT')
    request=replace(template,request_id=template.request_id+'-live',box=state,snapshot=snapshot,
        start_q=tuple(row['vacuum']['q']),start_attached=True,
        confirmed_box_T_tool=tuple(np.linalg.inv(np.array(sample.tool_T_box_at_attach).reshape(4,4)).ravel()))
    request_path=output/'live_request.json';request_path.write_text(json.dumps(asdict(request),indent=2)+'\n')
    destination=output/'live_planning';repo=Path(__file__).resolve().parents[2]
    import yaml
    from selection_policy import SelectionPolicy
    config_path=output/'live_upper_config.yaml';config_path.write_text(yaml.safe_dump(asdict(config)))
    selection_path=output/'live_selection_policy.yaml';selection_path.write_text(yaml.safe_dump(asdict(SelectionPolicy.load())))
    execution_path=output/'live_execution_policy.yaml';execution_path.write_text(yaml.safe_dump(asdict(policy)))
    command=[sys.executable,str(Path(__file__).with_name('request_planner.py')),
        '--request',str(request_path),'--scene-config',str(scene_path),
        '--config',str(config_path),
        '--selection-policy',str(selection_path),
        '--execution-policy',str(execution_path),
        '--evidence',str(destination),'--seed',str(seed)]
    monitor.begin_phase('planning_hold',True,True,False)
    deadline=time.monotonic()+config.search_budget_s+policy.condition_wait_budget_s
    with (output/'live_planning.log').open('w') as log:
        process=subprocess.Popen(command,cwd=repo,env=dict(os.environ,OPENBLAS_NUM_THREADS='1'),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while process.poll() is None:
                rclpy.spin_once(runtime.node,timeout_sec=policy.monitor_period_s)
                reason=observe(runtime.snapshot())
                if reason:raise RuntimeError(reason)
                validate_held_start(request,monitor.sample,config,policy)
                if time.monotonic()>deadline:raise TimeoutError('LIVE_PLANNING_DEADLINE')
            if process.returncode:raise RuntimeError('LIVE_PLANNER_FAILED:'+str(process.returncode))
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=3)
    loaded=load_scored_plan(destination,policy)
    validation=validate_held_start(request,monitor.sample,config,policy)
    (output/'live_start_revalidation.json').write_text(json.dumps(validation,indent=2)+'\n')
    monitor.bind_held_action(loaded['plan']['action'])
    return loaded['plan'],destination
