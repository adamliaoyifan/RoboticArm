"""Bounded single-stack production launch/readiness/cycle capture and teardown."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import zipfile

root = Path.cwd()
parser = argparse.ArgumentParser()
parser.add_argument('--name', required=True)
parser.add_argument('--contacts',action='store_true')
parser.add_argument('--probe-scenario',choices=['delete_reuse','retained_instances','physical_release','rigid_attachment'])
parser.add_argument('--three-box',action='store_true')
parser.add_argument('--cycle', action='store_true')
parser.add_argument('--build',default='build_01')
parser.add_argument('--failure-hold-s',type=float,default=0.)
parser.add_argument('--prepick-replans',type=int,default=0)
parser.add_argument('--capture-operation',action='store_true')
args = parser.parse_args()
folder = root/'docs/status/evidence/geometry_plan_reuse/20261113'/args.name
folder.mkdir(exist_ok=False)
live = subprocess.run(['pgrep','-af',r'(^|/)(ros2 launch luggage_gazebo|ign gazebo|gz sim|parameter_bridge)( |$)'],
                      capture_output=True,text=True)
if live.stdout.strip():
    (folder/'busy.txt').write_text(live.stdout)
    raise SystemExit('SIM_SLOT_BUSY')
env = dict(os.environ,ROS_DOMAIN_ID='7',ROS_LOCALHOST_ONLY='1',YOLO_OFFLINE='1',
           ULTRALYTICS_OFFLINE='1',YOLO_AUTOINSTALL='false',
           IGN_PARTITION='three_box_'+args.name+'_'+str(os.getpid()))
if args.contacts:env['ELFIN_BOX_CONTACT_TOPIC']='/three_box/physics_contacts'
manifest = dict(revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
    dirty=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines(),
    started=time.time(),ign_partition=env['IGN_PARTITION'],commands=[],stages=[],source_hashes={})
build=json.loads((folder.parent/args.build/'manifest.json').read_text())
hashes=build.get('source_after',{})
if not hashes:
    # Freeze the same package closure used by the production build.
    for package in ('luggage_description','luggage_msgs','luggage_planning','luggage_gazebo',
                    'luggage_perception','luggage_packing','luggage_mapping','uld_occ_observer'):
        for path in (root/'src'/package).rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix!='.pyc':
                hashes[str(path.relative_to(root))]=hashlib.sha256(path.read_bytes()).hexdigest()
base=folder.parent/'diagnostic_01/run_manifest.json'
base_hashes=json.loads(base.read_text())['source_hashes'] if base.is_file() else {}
if base_hashes:manifest['base_source_archive']='../diagnostic_01/effective_sources.zip'
with zipfile.ZipFile(folder/'effective_sources.zip','w',zipfile.ZIP_DEFLATED) as archive:
    for relative,digest in hashes.items():
        path=root/relative
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise SystemExit('SOURCE_CHANGED:'+relative)
        if base_hashes.get(relative)!=digest:archive.write(path,relative)
manifest['source_hashes']=hashes
def save():
    (folder/'run_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
launch=['ros2','launch','luggage_gazebo','sim_world.launch.py',
    'profile_config:='+str(root/'install-production/luggage_gazebo/share/luggage_gazebo/config/joint_operation.profile.yaml'),
    'operation_run_dir:='+str(folder/'run')]
manifest['commands'].append(launch)
manifest['eval_contact_topic']=env.get('ELFIN_BOX_CONTACT_TOPIC','')
save()
pidfile=folder/'launch.pid'
launch_log=(folder/'launch.log').open('w')
process=subprocess.Popen(launch,env=env,stdout=launch_log,stderr=subprocess.STDOUT,start_new_session=True)
pidfile.write_text(str(process.pid)+'\n')
manifest['launch_pid']=process.pid
try:
    deadline=time.monotonic()+90
    while not (folder/'run/launch.json').exists():
        if process.poll() is not None:raise RuntimeError('LAUNCH_EXIT:'+str(process.returncode))
        if time.monotonic()>deadline:raise TimeoutError('LAUNCH_MANIFEST')
        time.sleep(.2)
    for stage,start,timeout in [('ready',False,180),('cycle',True,(3 if args.three_box else 1)*(900+650*args.prepick_replans))]:
        if start and not args.cycle:break
        command=['ros2','run','luggage_gazebo','joint_operation_sim_driver.py',
            '--run-dir',str(folder/'run'),'--evidence',str(folder/stage)]
        if start:
            command.append('--start')
            if args.three_box:command.append('--three-box')
            if args.failure_hold_s:command.extend(['--failure-hold-s',str(args.failure_hold_s)])
            if args.prepick_replans:command.extend(['--prepick-replans',str(args.prepick_replans)])
            if args.capture_operation:command.append('--capture-operation')
        if start and args.probe_scenario:
            probe=root/('docs/status/evidence/three_box_observed/g2/rehearse_rigid_attachment.py' if args.probe_scenario=='rigid_attachment' else 'docs/status/evidence/three_box_observed/g2/rehearse_mesh_instances.py')
            command=['/usr/bin/python3',str(probe),'--run-dir',str(folder/'run'),
                '--evidence',str(folder/stage),'--scenario',args.probe_scenario]
            (folder/'probe_script.py').write_bytes(probe.read_bytes())
        manifest['commands'].append(command)
        began=time.monotonic()
        print('START',stage,flush=True)
        with (folder/(stage+'.log')).open('w') as output:
            result=subprocess.run(command,env=env,stdout=output,stderr=subprocess.STDOUT,timeout=timeout)
        manifest['stages'].append(dict(stage=stage,returncode=result.returncode,elapsed_s=time.monotonic()-began))
        save()
        print('FINISH',stage,manifest['stages'][-1],flush=True)
        if result.returncode:raise RuntimeError(stage+'_FAILED')
    manifest['outcome']='PASS'
except Exception as error:
    manifest['outcome']='INCOMPLETE'
    manifest['reason']=repr(error)
    print(manifest['reason'],flush=True)
finally:
    save()
    stop=subprocess.run(['scripts/stop_sim.sh','--owned-pgid',str(process.pid),'--pidfile',str(pidfile)],
                        env=env,capture_output=True,text=True,timeout=60)
    (folder/'teardown.log').write_text(stop.stdout+stop.stderr)
    manifest['teardown_returncode']=stop.returncode
    manifest['finished']=time.time()
    save()
    launch_log.close()
print(json.dumps({k:manifest.get(k) for k in ('outcome','reason','stages','teardown_returncode')}),flush=True)
