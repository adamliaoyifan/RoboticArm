"""Behavioural checks for installed-driver selection and reference capture."""
import gzip
import importlib.util
from pathlib import Path


def module(name):
    spec=importlib.util.spec_from_file_location(name,Path(__file__).with_name(name+'.py'))
    result=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def test_readiness_never_forwards_motion_or_cycle_flags():
    runner=module('simulation')
    command=runner.client_command('/run','/evidence',three_box=True,prepick_replans=2)
    assert command[:4]==['ros2','run','luggage_gazebo','joint_operation_sim_driver.py']
    assert '--start' not in command and '--three-box' not in command
    assert '--prepick-replans' not in command
    assert runner.client_timeout_s(start=False,three_box=True)==180.


def test_three_box_reaches_actual_driver_with_capture_and_full_budget():
    runner=module('simulation')
    command=runner.client_command('/run','/evidence',start=True,three_box=True,
        failure_hold_s=25.,prepick_replans=1,view_poses='/view.json')
    assert '--start' in command and '--three-box' in command
    assert command[command.index('--failure-hold-s')+1]=='25.0'
    assert command[command.index('--prepick-replans')+1]=='1'
    assert command[command.index('--view-poses')+1]=='/view.json'
    assert runner.client_timeout_s(start=True,three_box=True,prepick_replans=1)==4650.


def test_contacts_keep_original_acquisition_and_phase_examples(tmp_path):
    capture=module('physics_reference')
    ledger=capture.ContactLedger(tmp_path)
    raw=b'header { stamp { sec: 7 nsec: 12 } }\ncollision1 { name: "box::body" }\ncollision2 { name: "uld::floor" }'
    ledger.add(raw[:40],'carry')
    assert ledger.messages==0
    ledger.add(raw[40:]+b'\n\n','carry')
    ledger.add(raw+b'\n\n','released')
    rows=ledger.summary()['pairs']
    assert len(rows)==2 and rows[0]['first_stamp_ns']==7000000012
    assert rows[0]['phase']=='carry' and rows[1]['phase']=='released'
    assert gzip.decompress((tmp_path/rows[0]['example']).read_bytes())==raw


def test_actual_runtime_stage_is_used_for_reference_join(tmp_path):
    capture=module('physics_reference')
    directory=tmp_path/'execution'/'operation-1'
    directory.mkdir(parents=True)
    (directory/'leg_stage.json.gz').write_bytes(gzip.compress(b'{"next_leg_index": 4}'))
    (directory/'manifest.json').write_text(
        '{"phase":"EXECUTING","records":[{"file":"leg_stage.json.gz"}]}')
    assert capture.current_phase(tmp_path)=='operation-1:EXECUTING:4'


def test_capture_exception_after_successful_client_is_not_success():
    runner=module('simulation')
    manifest=dict(capture_complete=True,residuals_after=[],exception='snapshot RPC timeout')
    assert runner.final_result(manifest,0,start=True,three_box=True)==('INCOMPLETE',1)
    manifest.pop('exception')
    assert runner.final_result(manifest,0,start=True,three_box=True)==('THREE_BOX_PASS',0)
    manifest['capture_complete']=False
    assert runner.final_result(manifest,0,start=True,three_box=True)==('INCOMPLETE',1)
    manifest.update(capture_complete=True,residuals_after=['foreign-looking residual'])
    assert runner.final_result(manifest,0,start=True,three_box=True)==('TEARDOWN_INCOMPLETE',1)
