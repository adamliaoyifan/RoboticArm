"""Optional simulation capture preserves actual pinned perception inputs."""
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from luggage_msgs.msg import DetectionFrame
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image

from luggage_perception.suction_input_capture import SuctionInputCapture
from test_suction_patch_evaluator import INTR, NadirRenderer, make_b1_case, make_model, run_evaluator
from suction_input_evidence import configure_capture, audit_selected_inputs


def test_enabled_capture_reaches_actual_child_without_changing_other_environment(tmp_path):
    env = dict(os.environ, UNRELATED_CAPTURE_TEST='preserved', ELFIN_SUCTION_INPUT_CAPTURE='/old')
    original = dict(env)
    assert not configure_capture(env, tmp_path, enabled=False)['managed']
    assert env == original
    receipt = configure_capture(env, tmp_path, enabled=True)
    assert receipt['managed'] and receipt['enabled']
    child = subprocess.check_output([sys.executable, '-c',
        'import json,os;print(json.dumps([os.environ["ELFIN_SUCTION_INPUT_CAPTURE"],os.environ["UNRELATED_CAPTURE_TEST"]]))'],
        env=env, text=True)
    assert json.loads(child) == [str(tmp_path/'suction_inputs'), 'preserved']
    assert {key: value for key,value in env.items() if key != 'ELFIN_SUCTION_INPUT_CAPTURE'} == {
        key: value for key,value in original.items() if key != 'ELFIN_SUCTION_INPUT_CAPTURE'}


def capture_fixture(tmp_path):
    case, _ = make_b1_case(NadirRenderer(), 0., 30., 1., 11, 1.)
    case.depth_mm[:] = case.depth_mm.astype(np.uint16)
    model = make_model()
    _, top, expected = run_evaluator(case, model)
    assert expected is not None and expected.accepted
    frame = DetectionFrame(instance_id=expected.instance_id, generation=expected.generation)
    stamp = 10**9
    frame.header.stamp.sec = 1
    frame.header.frame_id = 'world'
    image = Image(height=case.depth_mm.shape[0], width=case.depth_mm.shape[1], encoding='16UC1',
        step=case.depth_mm.shape[1]*2, data=case.depth_mm.astype(np.uint16).tobytes())
    image.header.stamp.sec = 1
    image.header.frame_id = 'camera_depth_optical_frame'
    region = np.zeros(case.depth_mm.shape, dtype=bool)
    x0,y0,x1,y1 = case.bbox
    region[max(0,y0):max(0,y1),max(0,x0):max(0,x1)] = True
    capture = SuctionInputCapture(tmp_path/'suction_inputs')
    capture.stage(case.depth_mm, INTR, case.mat4, top, region, model, expected, stamp_ns=stamp)
    assert capture.pin(expected.instance_id, expected.generation, stamp)['status'] == 'saved'
    directory = tmp_path/'driver/pickup_acquisition_1'
    directory.mkdir(parents=True)
    records = []
    for topic,message in (('/luggage/perception/detection_frame',frame),
                          ('/luggage/preprocessed/camera/depth/image',image)):
        raw = bytes(serialize_message(message))
        name = f'{len(records):04d}.cdr.gz'
        (directory/name).write_bytes(gzip.compress(raw))
        kind = type(message)
        records.append(dict(topic=topic, ros_type=kind.__module__+'.'+kind.__name__,file=name,
                            sha256=hashlib.sha256(raw).hexdigest(),acquisition_stamp_ns=stamp))
    (directory/'manifest.json').write_text(json.dumps(dict(records=records,
        acquisition_stamp_ns=stamp,selected_acquisition_complete=True)))
    return tmp_path/'suction_inputs/case-000', directory


def test_actual_staged_pinned_inputs_match_exact_selected_raw_acquisition(tmp_path):
    capture_fixture(tmp_path)
    result = audit_selected_inputs(tmp_path)
    assert result['status'] == 'COMPLETE' and result['complete']
    assert result['selected_count'] == result['captured_count'] == 1
    row = result['selected'][0]
    assert row['stamp_ns'] == 10**9 and row['depth_equal']
    assert row['instance_id'] == 'box-1' and row['generation'] == 7
    assert len(row['inputs_sha256']) == len(row['record_sha256']) == 64


@pytest.mark.parametrize('condition,reason',[
    ('stamp','EXACT_CONSUMED_INPUT_NOT_UNIQUE'),
    ('instance','EXACT_CONSUMED_INPUT_NOT_UNIQUE'),
    ('generation','EXACT_CONSUMED_INPUT_NOT_UNIQUE'),
    ('depth','SELECTED_DEPTH_DIFFERS_FROM_CONSUMED_INPUT'),
    ('missing_npz','FileNotFoundError'),
    ('missing_record','EXACT_CONSUMED_INPUT_NOT_UNIQUE'),
    ('corrupt_cdr','SELECTED_CDR_HASH_MISMATCH'),
    ('frame','CONSUMED_INPUT_METADATA_MISMATCH'),
    ('corrupt_record','Expecting property name'),
])
def test_missing_or_conflicting_capture_is_incomplete_evidence(tmp_path, condition, reason):
    case, directory = capture_fixture(tmp_path)
    record_path = case/'record.json'
    record = json.loads(record_path.read_text())
    if condition == 'stamp': record['stamp_ns'] += 1
    elif condition == 'instance': record['identity'][0] = 'different-box'
    elif condition == 'generation': record['identity'][1] += 1
    elif condition == 'frame': record['frame'] = 'wrong_frame'
    record_path.write_text(json.dumps(record))
    if condition == 'depth':
        with np.load(case/'inputs.npz', allow_pickle=False) as archive:
            arrays = {key: archive[key] for key in archive.files}
        arrays['depth_mm'] = np.zeros_like(arrays['depth_mm'])
        np.savez_compressed(case/'inputs.npz', **arrays)
    elif condition == 'missing_npz': (case/'inputs.npz').unlink()
    elif condition == 'missing_record': record_path.unlink()
    elif condition == 'corrupt_cdr': (directory/'0000.cdr.gz').write_bytes(gzip.compress(b'wrong'))
    elif condition == 'corrupt_record': record_path.write_text('{bad-json')
    result = audit_selected_inputs(tmp_path)
    assert result['status'] == 'INCOMPLETE' and not result['complete']
    assert any(reason in row['reason'] for row in result['selected']+result['invalid'] if 'reason' in row)


def test_no_selected_acquisition_never_claims_a_measured_capture(tmp_path):
    result = audit_selected_inputs(tmp_path)
    assert result['complete'] and result['status'] == 'NOT_EVALUATED'
    assert result['selected_count'] == 0


def test_real_runner_finally_tears_down_owned_stub_launch_when_audit_raises(tmp_path, monkeypatch):
    # Exercise runner main/finally, replacing only ROS/OS process boundaries.
    spec = importlib.util.spec_from_file_location('capture_simulation_runner', Path(__file__).with_name('simulation.py'))
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    root = tmp_path/'workspace'
    root.mkdir()
    semantic = tmp_path/'semantic'
    semantic.mkdir(); (semantic/'DEPLOYED.json').write_text('{}')
    receipt = tmp_path/'build.json'
    receipt.write_text(json.dumps(dict(passed=True, source_after={},packages=[])))
    output = tmp_path/'run'
    calls = []
    node = SimpleNamespace(count_publishers=lambda topic: 0,destroy_node=lambda: calls.append('destroy_node'))
    fake_ros = SimpleNamespace(init=lambda: None, create_node=lambda name: node,
        spin_once=lambda *a,**k: None, shutdown=lambda: calls.append('shutdown'))
    monkeypatch.setitem(sys.modules, 'rclpy', fake_ros)
    monkeypatch.setattr(runner, 'ROOT', root)
    monkeypatch.setattr(runner, 'Path', lambda path: tmp_path/'absent.pid' if str(path)=='/tmp/elfin_humble_sim.pid' else Path(path))
    monkeypatch.setattr(runner, 'get_package_share_directory', lambda package: str(root))
    monkeypatch.setattr(runner, 'residuals', lambda: [])
    ticks = iter(range(100))
    monkeypatch.setattr(runner.time, 'monotonic', lambda: float(next(ticks)))
    monkeypatch.setattr(runner.subprocess, 'check_output', lambda *a,**k: 'a076314\n')
    fake_launch = SimpleNamespace(pid=12345,poll=lambda: 1,returncode=1,wait=lambda **k: 1)
    def launch(command, **kwargs):
        calls.append(('launch_env', kwargs['env']['ELFIN_SUCTION_INPUT_CAPTURE']))
        return fake_launch
    monkeypatch.setattr(runner.subprocess, 'Popen', launch)
    def teardown(command, **kwargs):
        calls.append(('teardown', command))
        return SimpleNamespace(returncode=0,stdout='owned fixture stopped',stderr='')
    monkeypatch.setattr(runner.subprocess, 'run', teardown)
    monkeypatch.setattr(runner, 'audit_selected_inputs', lambda folder: (_ for _ in ()).throw(ValueError('audit fixture failure')))
    monkeypatch.setattr(sys, 'argv', ['simulation.py','--evidence',str(output),'--semantic-deps',str(semantic),
        '--build-receipt',str(receipt),'--capture-suction-inputs'])
    assert runner.main() == 1
    result = json.loads((output/'manifest.json').read_text())
    assert result['suction_input_capture']['audit_error'] == 'ValueError:audit fixture failure'
    assert ('launch_env', str(output/'suction_inputs')) in calls
    teardown_call = next(row for row in calls if isinstance(row, tuple) and row[0]=='teardown')
    assert teardown_call[1][teardown_call[1].index('--owned-pgid')+1] == '12345'
    assert result['teardown_exit_code'] == 0 and 'shutdown' in calls and 'destroy_node' in calls
