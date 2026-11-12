#!/usr/bin/env python3
"""Unscored actual-physics hold/nominal-action probe; no integration verdict.

All box motion after detachment must arise from the native physical simulator.
Input comes from its fresh ideal observation adapter. Replaying a diagnostic
plan requires source/model equality and explicit live initial-state validation.
"""
import argparse
from dataclasses import asdict,replace
import gzip
import json
from pathlib import Path
import shutil
import subprocess
import time
import traceback
import numpy as np
from assets import load,sha256
from candidates import generate_actions,matrix
from controller_io import ArmController
from execution_policy import ExecutionPolicy
from execution_monitor import ExecutionMonitor,pose_error
from physical_observation import ObservationReader
from physical_executor import PhysicalExecutor
from observed_collision import ObservedArmScene
from ideal_snapshot import capture_fixture,from_observation
from planning_types import UpperConfig,action_from_record,digest
from selection_policy import SelectionPolicy
from plan_selection import rank_nominal_plans
from runtime_io import RuntimeIO
from monitored_stream import NativeSampleMonitor

ROOT=Path(__file__).resolve().parents[2]
EVIDENCE=Path('/home/adamliao/work/elfin_humble_ws/docs/status/evidence')


def save(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-dir',type=Path,required=True)
    parser.add_argument('--evidence',type=Path,required=True)
    parser.add_argument('--planning-run',type=Path)
    parser.add_argument('--live-request',type=Path)
    parser.add_argument('--planning-seed',type=int,default=0)
    parser.add_argument('--replan-live',action='store_true',
        help='Use the existing request as a goal template; replan from the live attached state while monitoring hold')
    parser.add_argument('--policy',type=Path)
    parser.add_argument('--scene-config',type=Path)
    parser.add_argument('--config',type=Path)
    parser.add_argument('--initialize-pick',type=Path)
    parser.add_argument('--capture-after-pick',action='store_true')
    parser.add_argument('--planning-pause-rehearsal',action='store_true')
    parser.add_argument('--capture-rehearsal',action='store_true',
        help='Deliberate diagnostic failure after controller hold; never a physical action verdict')
    args=parser.parse_args();out=args.evidence.resolve()
    if args.planning_pause_rehearsal and (not args.initialize_pick or args.planning_run or args.live_request):
        parser.error('--planning-pause-rehearsal requires --initialize-pick and no placement input')
    if args.live_request:
        if args.planning_run or not args.scene_config or not args.config:
            parser.error('--live-request needs --scene-config and --config, without --planning-run')
        args.replan_live=True
    if args.replan_live and (not (args.planning_run or args.live_request) or not args.initialize_pick):
        parser.error('--replan-live requires --planning-run and --initialize-pick')
    if args.capture_after_pick and (not args.initialize_pick or args.planning_run):
        parser.error('--capture-after-pick needs --initialize-pick and no --planning-run')
    if args.capture_rehearsal and args.planning_run:parser.error('capture rehearsal does not execute a plan')
    if not out.is_relative_to(EVIDENCE):raise ValueError('PRIMARY_EVIDENCE_REQUIRED')
    policy=ExecutionPolicy.load(args.policy);config=UpperConfig.load(args.config);plan=None;selection=None;scene_path=args.scene_config
    if args.planning_run:
        from plan_artifact import load_scored_plan
        loaded=load_scored_plan(args.planning_run,policy)
        plan=loaded['plan'];config=loaded['config'];scene_path=loaded['scene_path']
        planning_manifest=loaded['manifest'];selection=loaded['selection'];selection_policy=loaded['selection_policy']
    fixture=json.loads((args.scene_dir/'fixture.json').read_text())
    if not fixture.get('ideal_vacuum'):raise ValueError('NATIVE_IDEAL_VACUUM_REQUIRED')
    for name,expected in fixture['source_hashes'].items():
        path=Path(name) if Path(name).is_absolute() else ROOT/name
        if sha256(path)!=expected:raise ValueError('FIXTURE_SOURCE_CHANGED:'+name)
    out.mkdir(parents=True,exist_ok=False);source=out/'source';source.mkdir()
    hashes={}
    for path in list(Path(__file__).parent.glob('*.py'))+list(Path(__file__).with_name('native').glob('*')):
        shutil.copy2(path,source/path.name);hashes[path.name]=sha256(path)
    save(source/'execution_policy.json',asdict(policy));save(source/'planning_config.json',asdict(config))
    if selection is not None:
        save(out/'selection.json',selection)
        save(source/'selection_policy.json',asdict(selection_policy))
    shutil.copy2(args.scene_dir/'fixture.json',out/'fixture.json')
    manifest=dict(scored_units=0,profile='unscored_native_physics_probe',
        acceptance_scope='nominal_only_robust_tube_release54_and_campaign_not_evaluated',
        revision=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
        dirty_count=len(subprocess.check_output(['git','-C',str(ROOT),'status','--porcelain'],text=True).splitlines()),
        source_hashes=hashes,started_wall=time.time(),command=list(__import__('sys').argv),
        capture_complete=False,replay_possible=False)
    save(out/'manifest.json',manifest)
    runtime=RuntimeIO(ROOT,out,args.scene_dir,capture_policy=policy,scene_path=scene_path);monitor=None;observe=None;trace=None;failed=False
    try:
        assets=load(ROOT,scene_path);runtime.start();row=runtime.snapshot();save(out/'initial_observation.json',row)
        if args.initialize_pick:
            from pick_initialization import initialize_pick
            row=initialize_pick(runtime,assets,args.scene_dir,args.initialize_pick,config,policy,out/'pick_initialization')
        if args.capture_after_pick:
            save(out/'observation.json',row)
            save(out/'summary.json',dict(status='post_pick_capture_pass',upper_placement='not_evaluated'))
            manifest['capture_complete']=True;manifest['replay_possible']=True
            return
        snapshot,state=capture_fixture(args.scene_dir,assets,config)
        snapshot,state=from_observation(snapshot,state,row,assets)
        save(out/'fresh_snapshot.json',dict(scene=asdict(snapshot),box=asdict(state)))
        scene=ObservedArmScene(assets,snapshot,config)
        if plan:
            if (sha256(args.planning_run/'source/execution_policy.yaml')!=planning_manifest['source_hashes'].get('execution_policy.yaml') or
                    planning_manifest.get('execution_policy_revision')!=policy.revision):
                raise ValueError('PLANNED_EXECUTION_POLICY_CHANGED')
            models=json.loads((args.planning_run/'models.json').read_text())
            if models.get('asset_hashes')!=assets['hashes'] or models['hashes']!=scene.model_hashes:
                raise ValueError('PLANNED_MODELS_CHANGED')
            action=action_from_record(plan['action']);scene.validate_push_geometry(action)
            old_input=None
            for path in sorted(args.planning_run.glob('action_*/input.json')):
                candidate=json.loads(path.read_text())
                if candidate['action']['action_id']==action.action_id:old_input=candidate;break
            if old_input is None:raise ValueError('PLANNED_INPUT_NOT_PRESERVED')
            if config.payload_effort_screen_enabled and old_input['box'].get('mass_kg')!=state.mass_kg:
                raise ValueError('LIVE_PAYLOAD_MASS_DIFFERS_FROM_PLAN')
            if tuple(old_input['box']['dimensions'])!=state.dimensions:
                raise ValueError('LIVE_BOX_DIMENSIONS_DIFFER_FROM_PLAN')
            errors=[]
            old={item['name']:item for item in old_input['snapshot']['occupied']}
            if set(old)!={body.name for body in snapshot.occupied}:raise ValueError('INITIAL_LOWER_SET_CHANGED')
            for body in snapshot.occupied:
                if tuple(old[body.name]['size'])!=body.size:raise ValueError('INITIAL_LOWER_SIZE_CHANGED')
                errors.append(float(np.linalg.norm(np.array(old[body.name]['center'])-body.center)))
            distance,angle=pose_error(old_input['box']['container_T_box'],state.container_T_box)
            joint_error=float(np.max(np.abs(np.array(old_input['start_q'])-row['vacuum']['q'])))
            validation=dict(lower_max_translation_m=max(errors),payload_translation_m=distance,
                            payload_rotation_deg=angle,joint_max_error_rad=joint_error,
                            original_snapshot_id=old_input['snapshot']['snapshot_id'],fresh_snapshot_id=snapshot.snapshot_id)
            save(out/'initial_state_revalidation.json',validation)
            if not args.replan_live and (max(errors+[distance])>config.contact_penetration_m or angle>policy.initial_rotation_deg or joint_error>policy.initial_joint_error_rad):
                raise ValueError('LIVE_INITIAL_STATE_DIFFERS_FROM_NOMINAL_PLAN')
            save(out/'nominal_plan.json',plan)
        elif args.initialize_pick:
            action=action_from_record(json.loads(args.initialize_pick.read_text())['action'])
        else:
            actions,_=generate_actions(assets,snapshot,state,replace(config,max_action_candidates=1))
            if not actions:raise ValueError('NO_GEOMETRIC_ACTION_FOR_MONITOR_HOLD')
            action=actions[0]
        if plan and plan.get('start_attached',False):
            if not row.get('vacuum',{}).get('attached',False):
                raise ValueError('ATTACHED_START_NEEDS_ALREADY_RUNNING_ATTACHED_RUNTIME')
        reader=ObservationReader(assets,snapshot,state.box_id,policy.observation_max_age_s)
        import rclpy
        deadline=time.monotonic()+policy.condition_wait_budget_s
        while True:
            rclpy.spin_once(runtime.node,timeout_sec=policy.monitor_period_s)
            try:initial=reader.read(runtime.snapshot());break
            except ValueError as exc:
                if not str(exc).startswith('STALE_PHYSICAL_STREAM') or time.monotonic()>deadline:raise
        trace=gzip.open(out/'monitor.jsonl.gz','wt',compresslevel=3)
        monitor=ExecutionMonitor(reader,scene,state,action,config,policy,initial,trace,
                                 panel_collision_name=runtime.panel_contact['collision_name'])
        observe=NativeSampleMonitor(runtime,monitor,policy)
        # Warm up the real observation boundary while holding the current arm.
        if initial.attached:
            # Retain the existing controller target while draining observations.
            # Re-commanding the measured loaded position changes its equilibrium
            # and would invalidate the just-revalidated post-pick start state.
            end_stamp=initial.stamp_s+policy.trigger_pre_s
            deadline=time.monotonic()+policy.condition_wait_budget_s
            while monitor.sample.stamp_s<end_stamp:
                rclpy.spin_once(runtime.node,timeout_sec=policy.monitor_period_s)
                reason=observe(runtime.snapshot())
                if reason:raise RuntimeError(reason)
                if time.monotonic()>deadline:raise TimeoutError('ATTACHED_OBSERVATION_WARMUP')
        else:
            q=list(initial.q);hold=dict(positions=[q,q],velocities=[[0.]*6,[0.]*6],time_from_start=[0.,policy.trigger_pre_s])
            ArmController(runtime).execute(hold,observe,policy.condition_wait_budget_s)
        if args.replan_live:
            from live_planning import plan_while_held
            plan,live_path=plan_while_held(runtime,monitor,observe,assets,snapshot,state,
                args.live_request or args.planning_run/'request.json',out,config,policy,
                scene_path,args.planning_seed)
            save(out/'nominal_plan.json',plan)
            save(out/'executed_planning_run.json',dict(path=str(live_path)))
        if args.capture_rehearsal:raise RuntimeError('DIAGNOSTIC_CAPTURE_REHEARSAL')
        if args.planning_pause_rehearsal:
            from planning_pause import rehearse_pause
            result=rehearse_pause(runtime,monitor,observe,policy)
        else:
            result=(PhysicalExecutor(runtime,monitor,observe,state.dimensions,config,policy).execute(plan) if plan else
                    dict(status='monitor_hold_pass',scope='native_observation_and_actual_controller_hold_only'))
        result['monitor']=monitor.metrics();result['stream']=observe.metrics();save(out/'summary.json',result)
        manifest['capture_complete']=True;manifest['replay_possible']=True
    except BaseException as exc:
        failed=True
        if monitor:
            monitor.fail_execution(str(exc) or type(exc).__name__,exception_type=type(exc).__name__)
            runtime.begin_failure_capture(monitor.failure_stamp_s)
        (out/'error.txt').write_text(traceback.format_exc())
        tail=[];wall_deadline=time.monotonic()+policy.condition_wait_budget_s
        first=runtime.snapshot().get('vacuum');stamp=first['sim_time_s'] if first else None
        if stamp is not None:
            if monitor is None:runtime.begin_failure_capture(stamp)
            import rclpy
            while time.monotonic()<wall_deadline:
                rclpy.spin_once(runtime.node,timeout_sec=policy.monitor_period_s)
                row=runtime.snapshot();native=row.get('vacuum')
                if native:
                    tail.append((native['sim_time_s'],row))
                    if native['sim_time_s']-stamp>=policy.trigger_post_s:break
        if monitor:monitor.save_trigger(out/'trigger_window.jsonl.gz',tail)
        save(out/'summary.json',dict(status='failed',monitor=monitor.metrics() if monitor else None,
                                    stream=observe.metrics() if observe else None,
                                    post_window_captured=bool(tail and tail[-1][0]-stamp>=policy.trigger_post_s)))
        manifest['capture_complete']=bool(tail and tail[-1][0]-stamp>=policy.trigger_post_s)
        manifest['replay_possible']=manifest['capture_complete']
    finally:
        if trace:trace.close()
        save(out/'native_capture_metrics.json',runtime.native_capture_metrics())
        save(out/'terminal_observation.json',runtime.snapshot())
        save(out/'manifest.json',manifest) # Failure observations are durable before teardown.
        runtime.stop()
        native_capture=runtime.native_capture_metrics()
        save(out/'native_capture_metrics.json',native_capture)
        if failed and runtime.vacuum_spec:
            manifest['capture_complete']=bool(manifest['capture_complete'] and native_capture and native_capture['trigger_complete'])
            manifest['replay_possible']=manifest['capture_complete']
        inventory={str(path.relative_to(out)):path.stat().st_size for path in out.rglob('*') if path.is_file()}
        manifest.update(ended_wall=time.time(),artifact_bytes=inventory,total_bytes=sum(inventory.values()))
        if manifest['total_bytes']>policy.max_case_bytes:
            manifest['capture_complete']=False;manifest['replay_possible']=False
            failed=True
            summary=json.loads((out/'summary.json').read_text()) if (out/'summary.json').exists() else {}
            summary.update(status='failed',capture_budget_error='CASE_CAPTURE_BUDGET_EXCEEDED',total_bytes=manifest['total_bytes'])
            save(out/'summary.json',summary)
        save(out/'manifest.json',manifest)
    if failed:raise SystemExit(1)


if __name__=='__main__':main()
