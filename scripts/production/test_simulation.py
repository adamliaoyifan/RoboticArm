"""Behavioural checks for installed-driver selection and reference capture."""
import gzip
import importlib.util
from pathlib import Path
import threading
import pytest
from pickup_continuation_fixtures import case_archive


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


def test_existing_discovery_node_receives_real_snapshot_response():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rosbag2_interfaces.srv import Snapshot
    runner=module('simulation')
    context=Context();rclpy.init(context=context,domain_id=177)
    server=rclpy.create_node('snapshot_owner_fixture',context=context)
    client=rclpy.create_node('existing_snapshot_discovery_probe',context=context)
    calls=[]
    accepted=[True]
    def capture(request,response):
        calls.append(request);response.success=accepted[0];return response
    service=server.create_service(Snapshot,'/rosbag2_recorder/snapshot',capture)
    executor=SingleThreadedExecutor(context=context);executor.add_node(server)
    worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
    try:
        assert runner.flush_sensor_snapshot(client,timeout_s=3.)['success']
        assert len(calls)==1
        accepted[0]=False
        assert not runner.flush_sensor_snapshot(client,timeout_s=3.)['success']
        assert len(calls)==2
    finally:
        executor.shutdown(timeout_sec=2.);worker.join(timeout=2.)
        server.destroy_service(service);server.destroy_node();client.destroy_node();rclpy.shutdown(context=context)
    assert not worker.is_alive()


@pytest.mark.parametrize('count',[0,2])
@pytest.mark.parametrize('start',[False,True])
def test_recovery_configuration_is_frozen_and_only_forwarded_to_authorized_driver(case_archive,count,start):
    from dataclasses import asdict
    import hashlib,json
    from test_pickup_recovery_cycle import settings
    runner=module('simulation')
    source=case_archive.directory/'operator_config.json'
    record=dict(schema='pickup_view_recovery_v1',**asdict(settings(count)))
    raw=(json.dumps(record,indent=2)+'\n').encode();source.write_bytes(raw)
    folder=case_archive.directory/'run';folder.mkdir()
    frozen,receipt=runner.freeze_pickup_recovery_config(source,folder)
    case_archive.save('config_receipt',receipt);case_archive.save('config_original_bytes',raw)
    source.write_text('{"changed_after_freeze":true}')
    command=runner.client_command('/run','/evidence',start=start,pickup_recovery_config=frozen)
    case_archive.save('constructed_driver_command',command)
    assert frozen.read_bytes()==raw and receipt['sha256']==hashlib.sha256(raw).hexdigest()
    assert receipt['max_extra_view_motions']==count
    if start:assert command[command.index('--pickup-recovery-config')+1]==str(frozen)
    else:assert '--pickup-recovery-config' not in command and '--start' not in command


@pytest.mark.parametrize('damage',['schema','missing_policy','boolean_count'])
def test_invalid_recovery_policy_never_freezes_an_implicit_production_default(case_archive,damage):
    from dataclasses import asdict
    import json
    from test_pickup_recovery_cycle import settings
    runner=module('simulation');record=dict(schema='pickup_view_recovery_v1',**asdict(settings()))
    if damage=='schema':record['schema']='unknown'
    elif damage=='missing_policy':record.pop('association_policy')
    else:record['max_extra_view_motions']=True
    source=case_archive.directory/'operator_config.json';source.write_text(json.dumps(record))
    folder=case_archive.directory/'run';folder.mkdir();case_archive.save('invalid_input',record)
    with pytest.raises((KeyError,TypeError,ValueError)):runner.freeze_pickup_recovery_config(source,folder)
    assert not (folder/'input_pickup_recovery.json').exists()
