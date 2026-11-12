"""Unscored physical initializer for an already-picked placement experiment.

The test scene starts the arm at a source-IK seal pose, with the box resting on
its actual pickup platform. A real vacuum attachment and certified controller
lift produce the post-pick input. This does not validate the home-to-pick route.
No cargo pose or gravity command is ever sent.
"""
from dataclasses import asdict
import json,time
from pathlib import Path
import numpy as np
from assets import sha256
from planning_types import action_from_record
from candidates import matrix
from ideal_snapshot import capture_fixture,from_observation
from physical_observation import ObservationReader
from observed_collision import ObservedArmScene
from execution_monitor import ExecutionMonitor,pose_error
from monitored_stream import NativeSampleMonitor
from controller_io import ArmController


def initialize_pick(runtime,assets,scene_dir,record_path,config,policy,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    record=json.loads(Path(record_path).read_text())
    (output/'input.json').write_text(json.dumps(record,indent=2)+'\n')
    if record['config_revision']!=config.revision:raise ValueError('PICK_INITIALIZER_CONFIG_CHANGED')
    if record['asset_hashes']!=assets['hashes']:raise ValueError('PICK_INITIALIZER_ASSETS_CHANGED')
    for name,expected in record['planning_source_hashes'].items():
        if sha256(Path(__file__).with_name(name))!=expected:raise ValueError('PICK_INITIALIZER_SOURCE_CHANGED:'+name)
    if not record['certificate']['valid']:raise ValueError('UNCERTIFIED_PICK_INITIALIZER')
    trajectory=record['lift_trajectory']
    initial_q=np.array(record['initial_q'])
    if not np.allclose(trajectory['positions'][0],initial_q,rtol=0,atol=1e-10):
        raise ValueError('PICK_INITIALIZER_DISCONNECTED_START')
    action=action_from_record(record['action'])
    snapshot,state=capture_fixture(scene_dir,assets,config)
    scene=ObservedArmScene(assets,snapshot,config)
    import rclpy
    reader=ObservationReader(assets,snapshot,state.box_id,policy.observation_max_age_s)
    deadline=time.monotonic()+policy.condition_wait_budget_s
    while True:
        rclpy.spin_once(runtime.node,timeout_sec=policy.monitor_period_s)
        row=runtime.snapshot()
        try:initial=reader.read(row);break
        except ValueError as exc:
            if not str(exc).startswith('STALE_PHYSICAL_STREAM') or time.monotonic()>deadline:raise
    snapshot,state=from_observation(snapshot,state,row,assets)
    if initial.attached:raise ValueError('INITIALIZER_EXPECTS_UNATTACHED_PLATFORM_BOX')
    if np.max(np.abs(initial_q-initial.q))>policy.initial_joint_error_rad:
        raise ValueError('INITIALIZER_SEAL_JOINTS_MISMATCH')
    expected=matrix(initial.container_T_box)@matrix(action.box_T_tool)
    distance,angle=pose_error(expected,initial.container_T_tool)
    if distance>config.contact_tolerance_m or angle>policy.attachment_rotation_error_deg:
        raise ValueError('INITIALIZER_SEAL_RELATION_MISMATCH')
    with (output/'monitor.jsonl').open('w') as trace:
        monitor=ExecutionMonitor(reader,scene,state,action,config,policy,initial,trace,
            panel_collision_name=runtime.panel_contact['collision_name'],
            failure_capture=runtime.begin_failure_capture,timing=runtime.monitor_timing)
        observe=NativeSampleMonitor(runtime,monitor,policy)
        monitor.begin_phase('pick_contact',True,False,True)
        reason=observe(row)
        if reason:raise RuntimeError(reason)
        monitor.begin_phase('attach_transition',True,None,True)
        runtime.toggle_vacuum(True,monitor=observe)
        monitor.begin_phase('pick_lift',True,True,True)
        controller=ArmController(runtime,tracking_error_rad=policy.controller_tracking_error_rad,
            feedback_max_age_s=policy.observation_max_age_s)
        controller.execute(trajectory,observe,policy.wall_budget_s)
        monitor.begin_phase('held_after_pick',True,True,False)
        q=list(monitor.sample.q)
        hold=dict(positions=[q,q],velocities=[[0.]*6,[0.]*6],time_from_start=[0.,policy.stability_dwell_s])
        controller.execute(hold,observe,policy.condition_wait_budget_s)
        row=runtime.snapshot()
        summary=dict(status='post_pick_capture_pass',scope='real_seal_and_controller_lift_from_fixture_seal_pose',
            home_to_pick='not_evaluated',monitor=monitor.metrics(),stream=observe.metrics(),
            capture_complete=True)
        (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        (output/'observation.json').write_text(json.dumps(row,indent=2)+'\n')
        return row
