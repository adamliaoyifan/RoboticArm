#!/usr/bin/env python3
"""Own one exclusive observed-operation Gazebo rehearsal/cycle and its capture.

Run through production/run.sh. --start explicitly authorizes one simulated
operation; otherwise only startup readiness is checked. Never stops foreign sims.
"""
import argparse
import hashlib
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import tarfile
import time

from ament_index_python.packages import get_package_share_directory
from suction_input_evidence import configure_capture, audit_selected_inputs


ROOT = Path(__file__).resolve().parents[2]


def residuals():
    processes = subprocess.check_output(['ps','-eo','pid=,ppid=,args='],text=True)
    found = []
    for row in processes.splitlines():
        parts = row.strip().split(None,2)
        if len(parts)!=3 or int(parts[0])==os.getpid():continue
        command = parts[2]
        # Match executable forms, not a caller shell quoting this check.
        if command.startswith(('/bin/bash -c','bash -c','/bin/sh -c')):continue
        if ('/parameter_bridge ' in command or command.startswith('ign gazebo ')
                or command.startswith('gz sim ') or 'ruby /usr/bin/ign gazebo ' in command):
            found.append(row.strip())
    return found


def stop(process,timeout=15.):
    if process is None:return
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM)
            process.wait(timeout=5.)


def client_command(run_dir, evidence, *, start=False, three_box=False,
                   failure_hold_s=25., prepick_replans=0, view_poses=None, pickup_recovery_config=None):
    """One actual installed driver, with motion authorized only by start."""
    command=['ros2','run','luggage_gazebo','joint_operation_sim_driver.py',
             '--run-dir',str(run_dir),'--evidence',str(evidence)]
    if start:
        command.append('--start')
        if three_box:command.append('--three-box')
        if pickup_recovery_config:
            command.extend(['--pickup-recovery-config',str(pickup_recovery_config)])
        command.extend(['--failure-hold-s',str(failure_hold_s),
                        '--prepick-replans',str(prepick_replans)])
    if view_poses:command.extend(['--view-poses',str(view_poses)])
    return command


def freeze_pickup_recovery_config(source, folder):
    """Validate explicit policy once; driver consumes the exact archived bytes."""
    from luggage_planning.joint_operation.pickup_recovery import PickupRecoverySettings
    raw=Path(source).read_bytes()
    settings=PickupRecoverySettings.from_record(json.loads(raw))
    frozen=Path(folder)/'input_pickup_recovery.json'
    with frozen.open('xb') as stream:stream.write(raw)
    return frozen, dict(source=str(Path(source).resolve()),file=frozen.name,
        sha256=hashlib.sha256(raw).hexdigest(),max_extra_view_motions=settings.max_extra_view_motions,
        total_recovery_budget_s=settings.total_recovery_budget_s)


def client_timeout_s(*, start=False, three_box=False, prepick_replans=0):
    # Diagnostic action budgets, not a production latency requirement.
    return (3 if three_box else 1)*(900.+650.*prepick_replans) if start else 180.


def final_result(manifest, client_result, *, start, three_box):
    """Execution, evidence capture and teardown have independent failure facts."""
    result=client_result or int(bool(manifest.get('exception') or not manifest['capture_complete']))
    if manifest['residuals_after']:return 'TEARDOWN_INCOMPLETE',result or 1
    success=('THREE_BOX_PASS' if three_box else 'FULL_CYCLE_PASS') if start else 'STARTUP_READY'
    return (success if result==0 else 'INCOMPLETE'),result


def flush_sensor_snapshot(node, *, timeout_s=15.):
    """Flush through the existing discovery participant, not a short-lived CLI."""
    from rclpy.executors import SingleThreadedExecutor
    from rosbag2_interfaces.srv import Snapshot
    client=node.create_client(Snapshot,'/rosbag2_recorder/snapshot')
    executor=SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    started=time.monotonic()
    try:
        if not client.wait_for_service(timeout_sec=min(5.,timeout_s)):
            return dict(success=False,reason='SNAPSHOT_SERVICE_UNAVAILABLE')
        future=client.call_async(Snapshot.Request())
        while not future.done() and time.monotonic()-started < timeout_s:
            executor.spin_once(timeout_sec=min(.1,max(0.,timeout_s-(time.monotonic()-started))))
        if not future.done():return dict(success=False,reason='SNAPSHOT_RESPONSE_TIMEOUT')
        return dict(success=bool(future.result().success),reason='SNAPSHOT_RESPONSE',
                    elapsed_s=time.monotonic()-started)
    finally:
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1.)
        node.destroy_client(client)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence',required=True,type=Path)
    parser.add_argument('--semantic-deps',required=True,type=Path)
    parser.add_argument('--build-receipt',required=True,type=Path)
    parser.add_argument('--start',action='store_true')
    parser.add_argument('--three-box',action='store_true',help='Observed floor/floor/stack; requires --start')
    parser.add_argument('--failure-hold-s',default=25.,type=float)
    parser.add_argument('--prepick-replans',default=0,type=int,choices=(0,1,2))
    parser.add_argument('--capture-physics',action='store_true',help='Separate offline poses/contact ledger')
    parser.add_argument('--capture-suction-inputs',action='store_true',
        help='Optional exact selected depth/TF/top inputs for offline perception replay')
    parser.add_argument('--view-poses',type=Path,help='Explicit model/scene-bound diagnostic observation goals')
    parser.add_argument('--pickup-recovery-config',type=Path,
        help='Explicit count/time/sampling/association policy; archived and forwarded only to started driver')
    parser.add_argument('--domain',default=7,type=int)
    args = parser.parse_args()
    if args.three_box and not args.start:parser.error('--three-box requires --start')
    if not 0 < args.failure_hold_s <= 60:parser.error('failure hold must be within (0,60] seconds')
    folder = args.evidence.resolve();folder.mkdir(parents=True,exist_ok=False)
    env = dict(os.environ,ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',
               ELFIN_STARTUP_EVIDENCE_DIR=str(folder/'startup'),
               IGN_PARTITION='loading_'+str(os.getpid())+'_'+str(time.time_ns()))
    if args.capture_physics:env['ELFIN_BOX_CONTACT_TOPIC']='/loading/eval_contacts'
    suction_capture = configure_capture(env, folder, enabled=args.capture_suction_inputs)
    manifest = dict(started_at=datetime.now().astimezone().isoformat(),start_authorized=args.start,
        revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        dirty=subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).splitlines(),
        environment={key:env.get(key) for key in ('ROS_DOMAIN_ID','AMENT_PREFIX_PATH','PYTHONPATH')},
        capture_complete=False,replay_possible=False,stages=[],missing=[],
        suction_input_capture=suction_capture)
    def write():
        (folder/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    write()
    recovery_input=None
    if args.pickup_recovery_config:
        try:
            recovery_input,manifest['pickup_recovery_config']=freeze_pickup_recovery_config(
                args.pickup_recovery_config,folder)
        except (OSError,ValueError,TypeError,KeyError) as error:
            manifest.update(outcome='INVALID_PICKUP_RECOVERY_CONFIG',nodes_started=0,
                exception=type(error).__name__+':'+str(error),capture_complete=True,replay_possible=True)
            write();return 2
        write()
    view_input=None
    if args.view_poses:
        view_input=folder/'input_view_poses.json'
        view_input.write_bytes(args.view_poses.read_bytes())
        manifest['view_poses_source']=str(args.view_poses.resolve())
        write()
    occupied = residuals()
    if occupied:
        manifest.update(outcome='QUEUED_FOREIGN_SIM',foreign_processes=occupied,capture_complete=True,
                        replay_possible=True,nodes_started=0)
        write();print('Simulation slot occupied; no stack started. '+str(folder));return 75
    if not (args.semantic_deps/'DEPLOYED.json').is_file():
        manifest.update(outcome='DEPENDENCY_PREFIX_MISSING',nodes_started=0,capture_complete=True,
                        replay_possible=True)
        write();return 2
    # Preserve source bytes, including pre-existing shared untracked runtime
    # modules. A dirty hash alone is not a replay artifact.
    suffixes={'.py','.cpp','.h','.hpp','.yaml','.yml','.json','.xml','.msg','.srv',
              '.action','.xacro','.srdf','.sh','.stl','.STL','.dae','.obj','.npz','.sdf','.dsv'}
    receipt=json.loads(args.build_receipt.read_text())
    if not receipt.get('passed'):
        raise ValueError('PASSING_PRODUCTION_BUILD_RECEIPT_REQUIRED')
    changed=[name for name,digest in receipt['source_after'].items()
        if not (ROOT/name).is_file() or hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest]
    if changed:
        manifest.update(outcome='SOURCE_CHANGED_SINCE_BUILD',changed=changed);write();return 2
    manifest['build_receipt']=str(args.build_receipt.resolve())
    (folder/'build_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    with tarfile.open(folder/'runtime_sources.tar.gz','w:gz') as archive:
        bases=[ROOT/row.split()[1] for row in receipt['packages']]+[ROOT/'scripts/production']
        for base in bases:
            for path in sorted(base.rglob('*')):
                if (path.is_file() and path.suffix in suffixes and
                        not {'__pycache__','.pytest_cache','.git'} & set(path.parts)):
                    archive.add(path,arcname=str(path.relative_to(ROOT)),recursive=False)
    manifest['source_archive']='runtime_sources.tar.gz';write()
    pidfile = Path('/tmp/elfin_humble_sim.pid')
    if pidfile.exists():
        manifest.update(outcome='PIDFILE_REQUIRES_OWNER_REVIEW',pidfile=pidfile.read_text())
        write();return 75
    # One persistent discovery probe, no repeated ros2 CLI graph polling.
    import rclpy
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1')
    rclpy.init();probe=rclpy.create_node('operation_simulation_exclusivity_probe')
    until=time.monotonic()+2.
    while time.monotonic()<until:rclpy.spin_once(probe,timeout_sec=.1)
    prior_clock=probe.count_publishers('/clock')
    if prior_clock:
        manifest.update(outcome='PRIOR_CLOCK_PUBLISHER',prior_clock_publishers=prior_clock)
        write();probe.destroy_node();rclpy.shutdown();return 75
    profile=Path(get_package_share_directory('luggage_gazebo'))/'config/joint_operation.profile.yaml'
    command=['ros2','launch','luggage_gazebo','sim_world.launch.py',
             'profile_config:='+str(profile),'operation_run_dir:='+str(folder/'runtime'),
             'semantic_pydeps_dir:='+str(args.semantic_deps)]
    manifest['launch_command']=command;write()
    manifest.update(ign_partition=env['IGN_PARTITION'],three_box=args.three_box,
                    eval_contact_topic=env.get('ELFIN_BOX_CONTACT_TOPIC',''))
    launch=bag=client=physics=None
    result=1
    try:
        with (folder/'launch.log').open('w') as log:
            startup_deadline=time.monotonic()+180.
            launch=subprocess.Popen(command,env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            pidfile.write_text(str(launch.pid)+'\n');manifest['launch_pid']=launch.pid;write()
            # Launch materializes the model/owner identity before starting nodes.
            # A client cannot read that identity until the file is complete.
            config_ready=False
            config_deadline=time.monotonic()+45.
            while launch.poll() is None and time.monotonic()<config_deadline:
                try:
                    resolved=json.loads((folder/'runtime/launch.json').read_text())
                    config_ready=resolved.get('pipeline')=='joint_operation'
                except (FileNotFoundError,json.JSONDecodeError):
                    pass
                if config_ready:break
                time.sleep(.1)
            manifest['configuration_ready']=config_ready;write()
            if not config_ready:raise RuntimeError('LAUNCH_CONFIGURATION_NOT_READY')
            topics=['/camera/color/image_raw','/camera/depth/image_raw','/camera/depth/camera_info',
                '/luggage/preprocessed/camera/color/image','/luggage/preprocessed/camera/depth/image',
                '/luggage/preprocessed/camera/depth/camera_info','/tf','/tf_static','/joint_states',
                '/uld_occ_observer/receipts','/uld_occ_observer/validity','/luggage/suction_top_domain_v2',
                '/luggage/observation/status','/clock']
            bag_command=['ros2','bag','record','--snapshot-mode','--max-cache-size',str(512*1024*1024),
                '--use-sim-time','--output',str(folder/'sensor_window'),*topics]
            manifest['bag_command']=bag_command;write()
            with (folder/'bag.log').open('w') as baglog:
                bag=subprocess.Popen(bag_command,env=env,stdout=baglog,stderr=subprocess.STDOUT,start_new_session=True)
                def run_client(name,start=False):
                    nonlocal client
                    command=client_command(folder/'runtime',folder/name,start=start,
                        three_box=args.three_box,failure_hold_s=args.failure_hold_s,
                        prepick_replans=args.prepick_replans,view_poses=view_input,pickup_recovery_config=recovery_input)
                    row=dict(stage=name,command=command,begin=time.monotonic());manifest['stages'].append(row);write()
                    with (folder/(name+'.log')).open('w') as output:
                        client=subprocess.Popen(command,env=env,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
                        deadline=time.monotonic()+client_timeout_s(start=start,
                            three_box=args.three_box,prepick_replans=args.prepick_replans)
                        if not start:deadline=min(deadline,startup_deadline)
                        while client.poll() is None and launch.poll() is None and time.monotonic()<deadline:
                            time.sleep(.2)
                        if client.poll() is None:stop(client)
                    row.update(exit_code=client.returncode,elapsed_s=time.monotonic()-row['begin'])
                    write();return client.returncode
                result=run_client('readiness')
                def run_ready_work():
                    nonlocal physics
                    # Reference capture precedes cargo spawn/execution. It does
                    # not authorize a job without explicit --start.
                    if args.capture_physics:
                        physics_command=['/usr/bin/python3',str(ROOT/'scripts/production/physics_reference.py'),
                            '--run-dir',str(folder/'runtime'),'--output',str(folder/'reference'),
                            '--contact-topic',env['ELFIN_BOX_CONTACT_TOPIC']]
                        manifest['physics_command']=physics_command;write()
                        with (folder/'physics.log').open('w') as physics_log:
                            physics=subprocess.Popen(physics_command,env=env,stdout=physics_log,
                                stderr=subprocess.STDOUT,start_new_session=True)
                    return run_client('driver',True) if args.start else 0
                def record_startup_admission(admission):
                    (folder/'startup_admission.json').write_text(json.dumps(admission,indent=2)+'\n')
                    manifest['startup_admission']=admission;write()
                if result==0:
                    from luggage_gazebo.startup_admission import run_after_controller_probe
                    result,_=run_after_controller_probe(folder/'startup/controller_manager',launch,
                        deadline=startup_deadline,on_ready=run_ready_work,
                        on_admission=record_startup_admission)
                manifest['launch_exit_code']=launch.poll()
                # Flush the bounded pre-trigger cache before stopping sensors.
                snapshot=flush_sensor_snapshot(probe)
                (folder/'snapshot.json').write_text(json.dumps(snapshot,indent=2)+'\n')
                manifest['snapshot_exit_code']=0 if snapshot['success'] else 1
                if not snapshot['success']:manifest['missing'].append(snapshot['reason'])
                stop(bag);bag=None
                metadata=folder/'sensor_window/metadata.yaml'
                manifest['capture_complete']=metadata.is_file() and snapshot['success']
                if metadata.is_file():
                    import yaml
                    bag_info=yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
                    manifest['sensor_message_count']=bag_info['message_count']
                    manifest['sensor_topics']={row['topic_metadata']['name']:row['message_count']
                        for row in bag_info['topics_with_message_count']}
                    if launch.poll() is not None and not bag_info['message_count']:
                        manifest['sensor_capture']='not_applicable_before_sensor_start'
                    elif not bag_info['message_count']:
                        manifest['capture_complete']=False
                if not manifest['capture_complete']:manifest['missing'].append('nonempty bounded sensor bag')
                write()
    except Exception as error:
        result=1
        manifest['exception']=type(error).__name__+':'+str(error);write()
    finally:
        stop(client);stop(bag);stop(physics)
        if physics is not None:
            manifest['physics_exit_code']=physics.returncode
        if args.capture_suction_inputs:
            try:
                manifest['suction_input_capture']['audit'] = audit_selected_inputs(folder)
                if not manifest['suction_input_capture']['audit']['complete']:
                    manifest['capture_complete'] = False
                    manifest['missing'].append('exact selected suction inputs')
            except Exception as error:
                manifest['suction_input_capture']['audit_error'] = type(error).__name__+':'+str(error)
                manifest['capture_complete'] = False
                manifest['missing'].append('selected suction input audit')
        if launch is not None:
            teardown=subprocess.run([str(ROOT/'scripts/stop_sim.sh'),'--owned-pgid',str(launch.pid),
                '--pidfile',str(pidfile)],capture_output=True,text=True,timeout=45.)
            (folder/'teardown.log').write_text(teardown.stdout+teardown.stderr)
            manifest['teardown_exit_code']=teardown.returncode
            try:launch.wait(timeout=5.)
            except subprocess.TimeoutExpired:manifest['missing'].append('launch terminal result')
        manifest['residuals_after']=residuals()
        manifest['replay_possible']=manifest['capture_complete']
        manifest['outcome'],result=final_result(manifest,result,start=args.start,three_box=args.three_box)
        manifest['finished_at']=datetime.now().astimezone().isoformat();write()
        probe.destroy_node();rclpy.shutdown()
    print(json.dumps({key:manifest[key] for key in ('outcome','capture_complete','residuals_after')}))
    return result or (1 if manifest['residuals_after'] else 0)


if __name__=='__main__':
    raise SystemExit(main())
