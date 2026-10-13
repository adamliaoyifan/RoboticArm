import json
import os
from pathlib import Path
import shlex
import subprocess
import time

out = Path(__file__).resolve().parent / 'matched-complete'
out.mkdir(exist_ok=False)
packages = ['luggage_description', 'luggage_planning', 'luggage_gazebo', 'luggage_packing']
roots = {
    'base': (Path('/tmp/elfin-frame-migration-st08'), Path('/tmp/elfin-frame-migration-st08/install/st08')),
    'candidate': (Path('/tmp/elfin-st09-g3'), Path('/tmp/elfin-st09-g3/install/continuation-full')),
}
clean = {k:v for k,v in os.environ.items() if k not in ('PYTHONPATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH')}
clean.update(ROS_DOMAIN_ID='7')
manifest = {'profiles': {}, 'commands': []}
for name, (ws, install) in roots.items():
    manifest['profiles'][name] = dict(workspace=str(ws), install=str(install),
        revision=subprocess.check_output(['git','-C',str(ws),'rev-parse','HEAD'],text=True).strip(),
        dirty=subprocess.check_output(['git','-C',str(ws),'status','--porcelain'],text=True))
    bootstrap = 'source /opt/ros/humble/setup.bash\nsource '+shlex.quote(str(install/'setup.bash'))+'\nexport PYTHONPATH='+shlex.quote(':'.join(str(ws/'src'/package) for package in packages+['luggage_perception']))+':"$PYTHONPATH"\nexec "$@"'
    env = dict(clean, ROS_LOG_DIR=str(out/(name+'-ros-logs')))
    commands = [('environment', ['python3','-c', 'import json,os,sys,luggage_msgs; print(json.dumps(dict(python=sys.version,environment={k:os.environ.get(k) for k in ("ROS_DOMAIN_ID","ROS_DISTRO","RMW_IMPLEMENTATION","AMENT_PREFIX_PATH","PYTHONPATH","PATH")},messages=luggage_msgs.__file__),sort_keys=True))'])]
    for package in packages:
        commands.extend([(package+'-nodes', ['python3','-m','pytest','--collect-only','-q','src/'+package+'/test']),
                         (package, ['python3','-m','pytest','-q','src/'+package+'/test','--tb=short'])])
    for label, argv in commands:
        start = time.monotonic()
        command = ['bash','-c',bootstrap,'matched',*argv]
        record = dict(profile=name, stage=label, command=command)
        with (out/(name+'-'+label+'.log')).open('x') as stream:
            try:
                result = subprocess.run(command,cwd=ws,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=180)
                record['exit_code'] = result.returncode
            except subprocess.TimeoutExpired:
                record['exit_code'] = 124
        record['elapsed_seconds'] = time.monotonic()-start
        manifest['commands'].append(record)
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        print(name, label, record['exit_code'], flush=True)
