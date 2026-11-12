#!/usr/bin/env python3
"""Own one exclusive observed-operation Gazebo rehearsal/cycle and its capture.

Run through production/run.sh. --start explicitly authorizes one simulated
operation; otherwise only startup readiness is checked. Never stops foreign sims.
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import tarfile
import time

from ament_index_python.packages import get_package_share_directory


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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence',required=True,type=Path)
    parser.add_argument('--semantic-deps',required=True,type=Path)
    parser.add_argument('--build-receipt',required=True,type=Path)
    parser.add_argument('--start',action='store_true')
    parser.add_argument('--domain',default=7,type=int)
    args = parser.parse_args()
    folder = args.evidence.resolve();folder.mkdir(parents=True,exist_ok=False)
    env = dict(os.environ,ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1')
    manifest = dict(started_at=datetime.now().astimezone().isoformat(),start_authorized=args.start,
        revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        dirty=subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).splitlines(),
        environment={key:env.get(key) for key in ('ROS_DOMAIN_ID','AMENT_PREFIX_PATH','PYTHONPATH')},
        capture_complete=False,replay_possible=False,stages=[],missing=[])
    def write():
        (folder/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
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
    import hashlib
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
    probe.destroy_node();rclpy.shutdown()
    if prior_clock:
        manifest.update(outcome='PRIOR_CLOCK_PUBLISHER',prior_clock_publishers=prior_clock)
        write();return 75
    profile=Path(get_package_share_directory('luggage_gazebo'))/'config/joint_operation.profile.yaml'
    command=['ros2','launch','luggage_gazebo','sim_world.launch.py',
             'profile_config:='+str(profile),'operation_run_dir:='+str(folder/'runtime'),
             'semantic_pydeps_dir:='+str(args.semantic_deps)]
    manifest['launch_command']=command;write()
    launch=bag=client=None
    result=1
    try:
        with (folder/'launch.log').open('w') as log:
            launch=subprocess.Popen(command,env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            pidfile.write_text(str(launch.pid)+'\n');manifest['launch_pid']=launch.pid;write()
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
                    command=['ros2','run','luggage_gazebo','joint_operation_sim_driver.py',
                        '--run-dir',str(folder/'runtime'),'--evidence',str(folder/name)]
                    if start:command.append('--start')
                    row=dict(stage=name,command=command,begin=time.monotonic());manifest['stages'].append(row);write()
                    with (folder/(name+'.log')).open('w') as output:
                        client=subprocess.Popen(command,env=env,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
                        deadline=time.monotonic()+(720. if start else 180.)
                        while client.poll() is None and launch.poll() is None and time.monotonic()<deadline:
                            time.sleep(.2)
                        if client.poll() is None:stop(client)
                    row.update(exit_code=client.returncode,elapsed_s=time.monotonic()-row['begin'])
                    write();return client.returncode
                result=run_client('readiness')
                if result==0 and args.start:result=run_client('driver',True)
                manifest['launch_exit_code']=launch.poll()
                # Flush the bounded pre-trigger cache before stopping sensors.
                snapshot=subprocess.run(['ros2','service','call','/rosbag2_recorder/snapshot',
                    'rosbag2_interfaces/srv/Snapshot','{}'],env=env,capture_output=True,text=True,timeout=15.)
                (folder/'snapshot.log').write_text(snapshot.stdout+snapshot.stderr)
                manifest['snapshot_exit_code']=snapshot.returncode
                stop(bag);bag=None
                metadata=folder/'sensor_window/metadata.yaml'
                manifest['capture_complete']=metadata.is_file()
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
        manifest['exception']=type(error).__name__+':'+str(error);write()
    finally:
        stop(client);stop(bag)
        if launch is not None:
            teardown=subprocess.run([str(ROOT/'scripts/stop_sim.sh'),'--owned-pgid',str(launch.pid),
                '--pidfile',str(pidfile)],capture_output=True,text=True,timeout=45.)
            (folder/'teardown.log').write_text(teardown.stdout+teardown.stderr)
            manifest['teardown_exit_code']=teardown.returncode
            try:launch.wait(timeout=5.)
            except subprocess.TimeoutExpired:manifest['missing'].append('launch terminal result')
        manifest['residuals_after']=residuals()
        manifest['replay_possible']=manifest['capture_complete']
        manifest['outcome']=('STARTUP_READY' if not args.start else 'FULL_CYCLE_PASS') if result==0 else 'INCOMPLETE'
        if manifest['residuals_after']:manifest['outcome']='TEARDOWN_INCOMPLETE'
        manifest['finished_at']=datetime.now().astimezone().isoformat();write()
    print(json.dumps({key:manifest[key] for key in ('outcome','capture_complete','residuals_after')}))
    return result or (1 if manifest['residuals_after'] else 0)


if __name__=='__main__':
    raise SystemExit(main())
