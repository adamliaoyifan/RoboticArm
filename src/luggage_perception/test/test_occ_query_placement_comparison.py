"""Real CDR alignment noise must not be confused with a changed slot."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from rclpy.serialization import serialize_message
from luggage_msgs.srv import ComputePlacement


def test_full_values_match_despite_padding_but_reject_changed_slot(tmp_path):
    path=Path(__file__).resolve().parents[3]/'scripts/g3/occ_query_placement_comparison.py'
    spec=importlib.util.spec_from_file_location('placement_comparison',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    left,right=tmp_path/'baseline',tmp_path/'candidate';left.mkdir();right.mkdir()
    response=ComputePlacement.Response();response.success=True
    payload=serialize_message(response)
    changed_padding=bytearray(payload);changed_padding[16]^=128
    (left/'a.response.cdr').write_bytes(payload)
    (right/'a.response.cdr').write_bytes(changed_padding)
    comparison=module.compare_responses(left,right)
    assert comparison['equal'] and not comparison['wire_equal']
    response.slot.col=7
    (right/'a.response.cdr').write_bytes(serialize_message(response))
    comparison=module.compare_responses(left,right)
    assert not comparison['equal']
    assert comparison['rows'][0]['different_fields']==['slot']


@pytest.mark.parametrize('changed_slot', [False, True])
def test_child_decodes_real_messages_when_parent_cannot_import_ros(tmp_path, changed_slot):
    script=Path(__file__).resolve().parents[3]/'scripts/g3/occ_query_placement_comparison.py'
    left,right=tmp_path/'baseline values',tmp_path/'candidate values'
    left.mkdir();right.mkdir()
    response=ComputePlacement.Response();response.success=True
    original=serialize_message(response)
    (left/'a.response.cdr').write_bytes(original)
    if changed_slot:
        response.slot.col=7
    (right/'a.response.cdr').write_bytes(serialize_message(response))
    right_before=(right/'a.response.cdr').read_bytes()
    # Pass only dependency paths, not the parent's unrelated environment.
    keys=('PATH','PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH',
          'CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','ROS_DISTRO','ROS_VERSION',
          'ROS_PYTHON_VERSION')
    child_environment={key:os.environ[key] for key in keys if key in os.environ}
    environment_path=tmp_path/'child_environment.json'
    environment_path.write_text(json.dumps(child_environment))
    parent_code='''
import importlib.abc, importlib.util, json, sys
from pathlib import Path
class NoParentROS(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('luggage_msgs', 'rclpy', 'rosidl_runtime_py'):
            raise ModuleNotFoundError('ROS imports forbidden in parent')
sys.meta_path.insert(0, NoParentROS())
script,left,right,output,environment_path=sys.argv[1:]
spec=importlib.util.spec_from_file_location('comparison',script)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
result=module.compare_in_environment(left,right,output,script=script,
    environment=json.loads(Path(environment_path).read_text()))
print(json.dumps(result))
'''
    result=subprocess.run([sys.executable,'-c',parent_code,str(script),str(left),str(right),
        str(tmp_path/'comparison.json'),str(environment_path)],
        env={'PATH':os.environ.get('PATH',os.defpath)},cwd='/tmp',
        capture_output=True,text=True,timeout=30.)
    assert result.returncode==0,result.stderr
    comparison=json.loads(result.stdout)
    assert comparison['equal'] is not changed_slot
    assert comparison['samples']==1
    assert (left/'a.response.cdr').read_bytes()==original
    assert (right/'a.response.cdr').read_bytes()==right_before
    control=json.loads((tmp_path/'comparison.control.json').read_text())
    assert control['exit_code']==0 and not control['stderr']


def test_failed_child_preserves_control_and_never_overwrites_evidence(tmp_path):
    script=Path(__file__).resolve().parents[3]/'scripts/g3/occ_query_placement_comparison.py'
    spec=importlib.util.spec_from_file_location('comparison_failure',script)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    output=tmp_path/'comparison.json'
    with pytest.raises(RuntimeError,match='child failed'):
        module.compare_in_environment(tmp_path,tmp_path,output,
            script=tmp_path/'missing_reader.py',environment={'PATH':os.defpath})
    control=output.with_suffix('.control.json')
    retained=control.read_bytes()
    assert json.loads(retained)['exit_code']!=0
    assert json.loads(retained)['stderr']
    with pytest.raises(FileExistsError):
        module.compare_in_environment(tmp_path,tmp_path,output,
            script=script,environment={'PATH':os.defpath})
    assert control.read_bytes()==retained
