"""Behavioural runner freeze, identity, evidence and finite deadline checks."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from depth_dropout_diagnostic import (configure_fixture, validate_fixture_options,
    validate_fixture_binding, audit_fixture, record_fixture_audit)
from luggage_gazebo.depth_dropout_evidence import DepthDropoutEvidence
from pickup_continuation_fixtures import case_archive
from test_depth_dropout_fixture import config_record, event
from test_simulation import module


def test_fixture_exact_frozen_bytes_bound_to_actual_launch_model(case_archive):
    root=case_archive.directory;source=root/'operator.json'
    raw=json.dumps(config_record()).encode();source.write_bytes(raw)
    env={'UNRELATED':'kept'}
    receipt=configure_fixture(env,root,source);case_archive.save('receipt',receipt)
    source.write_text('{}')
    assert (root/receipt['file']).read_bytes()==raw
    assert receipt['sha256']==hashlib.sha256(raw).hexdigest()
    assert env['ELFIN_SIM_DEPTH_DROPOUT_CONFIG']==str(root/receipt['file'])
    resolved=dict(model_sha256=receipt['model_hash'],scene_revision=receipt['scene_revision'])
    validate_fixture_binding(receipt,resolved)
    for key in ('model_sha256','scene_revision'):
        with pytest.raises(ValueError):validate_fixture_binding(receipt,{**resolved,key:'different'})
    assert configure_fixture(env,root,None) is None and env=={'UNRELATED':'kept'}


@pytest.mark.parametrize('start,capture',[(False,False),(False,True),(True,False)])
def test_fault_requires_explicit_motion_and_input_capture(case_archive,start,capture):
    with pytest.raises(ValueError):validate_fixture_options(enabled=True,start=start,capture_suction_inputs=capture)
    validate_fixture_options(enabled=False,start=start,capture_suction_inputs=capture)
    output=case_archive.directory/'not_started'
    command=[sys.executable,str(Path(__file__).with_name('simulation.py')),
        '--evidence',str(output),'--semantic-deps','/not-used','--build-receipt','/not-used',
        '--depth-dropout-fixture','/not-used']
    if start:command.append('--start')
    if capture:command.append('--capture-suction-inputs')
    result=subprocess.run(command,text=True,capture_output=True)
    case_archive.save('cli',dict(command=command,returncode=result.returncode,stderr=result.stderr))
    assert result.returncode==2 and 'requires --start and --capture-suction-inputs' in result.stderr
    assert not output.exists()


@pytest.mark.parametrize('three_box',[False,True])
@pytest.mark.parametrize('count',[0,2])
def test_recovery_budget_counted_once_per_box(case_archive,three_box,count):
    runner=module('simulation');receipt=dict(max_extra_view_motions=count,total_recovery_budget_s=600.)
    value=runner.client_timeout_s(start=True,three_box=three_box,prepick_replans=1,recovery_receipt=receipt)
    case_archive.save('deadline',dict(receipt=receipt,three_box=three_box,actual=value))
    assert value==(3 if three_box else 1)*(1550.+(600. if count else 0.))
    assert runner.client_timeout_s(recovery_receipt=receipt,three_box=three_box)==180.


@pytest.mark.parametrize('corrupt',[False,True])
def test_audit_joins_pinned_actual_inputs_and_suction_identity(case_archive,corrupt):
    import numpy as np
    root=case_archive.directory;selection=root/'events.jsonl'
    capture=DepthDropoutEvidence(root/'depth_dropout',selection)
    before=np.full((4,4),1000,dtype='<u2');after=before.copy();after[0,0]=0
    capture.record(17,'camera_depth_optical_frame',before,after,
        dict(active=True,reason='ACTIVE_CONE',changed_pixels=1))
    selection.write_text(json.dumps(event(17))+'\n');capture.close()
    suction=root/'suction_inputs/case-000';suction.mkdir(parents=True)
    (suction/'record.json').write_text(json.dumps(dict(identity=['observed-box',2],stamp_ns=18 if corrupt else 17)))
    result=audit_fixture(root);case_archive.save('audit',result)
    assert result['complete'] is (not corrupt) and result['changed_selected_count']==1


def test_audit_failure_cannot_skip_owned_teardown(case_archive,monkeypatch):
    runner=module('simulation');calls=[]
    launch=SimpleNamespace(pid=987654,wait=lambda timeout:calls.append('launch_terminal'))
    def stop(command,**kwargs):
        calls.append('owned_stop');return SimpleNamespace(returncode=0,stdout='',stderr='')
    def broken_audit(folder):
        calls.append('audit');raise OSError('fixture-read-failure')
    monkeypatch.setattr(runner.subprocess,'run',stop)
    monkeypatch.setattr(runner,'record_fixture_audit',lambda manifest,folder:
        record_fixture_audit(manifest,folder,audit=broken_audit))
    manifest=dict(depth_dropout_fixture={},capture_complete=True,missing=[])
    runner.finish_owned_launch(launch,case_archive.directory/'pidfile',case_archive.directory,manifest)
    case_archive.save('manifest',manifest);case_archive.save('call_order',calls)
    assert calls==['owned_stop','launch_terminal','audit']
    assert not manifest['capture_complete'] and manifest['teardown_exit_code']==0
