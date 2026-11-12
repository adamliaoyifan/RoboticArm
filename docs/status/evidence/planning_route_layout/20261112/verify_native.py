"""Sequential isolated MoveIt regressions; no Gazebo or controller process."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

root=Path.cwd();ev=root/'docs/status/evidence/planning_route_layout/20261112'
bin_dir=root/'install-production/luggage_planning/lib/luggage_planning'
env=dict(os.environ,PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
    JOINT_NATIVE_ENDPOINT_EXECUTABLE=str(bin_dir/'grasp_local_check_node'),
    JOINT_NATIVE_CONNECTOR_EXECUTABLE=str(bin_dir/'snapshot_joint_path_node'),
    JOINT_NATIVE_CERTIFIER_EXECUTABLE=str(bin_dir/'validate_route_node'))
cases=[('aperture','test_joint_operation_aperture_native.py',3),
       ('prepare_g','test_joint_operation_preparation_native.py',16),
       ('observer_o','test_joint_operation_observer_native.py',10),
       ('connector','test_snapshot_joint_path_native.py',10),
       ('endpoints','test_joint_operation_endpoints_native.py',7),
       ('certification','test_joint_operation_certification_native.py',10)]
results=[]
for name,filename,expected in cases:
    env.update(JOINT_ENDPOINT_EVIDENCE_DIR=str(ev/('native_'+name)),
        JOINT_NATIVE_EVIDENCE_DIR=str(ev/('native_'+name+'_connector')),
        JOINT_CERTIFICATION_EVIDENCE_DIR=str(ev/('native_'+name+'_certification')))
    command=[sys.executable,'-c','import sys,pytest;sys.path.insert(0,"src/luggage_perception/test");sys.exit(pytest.main(sys.argv[1:]))',
        '-q','--junitxml='+str(ev/(name+'.xml')),'src/luggage_planning/test/'+filename]
    began=time.monotonic()
    with (ev/(name+'.log')).open('wb') as output:
        result=subprocess.run(command,env=env,stdout=output,stderr=subprocess.STDOUT,timeout=120)
    row=dict(stage=name,command=command,environment={k:v for k,v in env.items() if k.startswith('JOINT_')},
        expected_passed=expected,returncode=result.returncode,elapsed_s=time.monotonic()-began,
        log=name+'.log',junit=name+'.xml')
    results.append(row)
    (ev/'native_runs.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(row),flush=True)
    if result.returncode:raise SystemExit(result.returncode)
