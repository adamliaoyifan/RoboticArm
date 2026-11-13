"""Behavioral pre-operation admission with actual generated receipt bytes."""
import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest
from controller_manager_msgs.srv import ListControllers
from rclpy.serialization import serialize_message

from luggage_gazebo.startup_admission import run_after_controller_probe


class Clock:
    def __init__(self):self.value=0.;self.on_sleep=None
    def __call__(self):return self.value
    def sleep(self,seconds):
        self.value+=seconds
        if self.on_sleep:self.on_sleep()


@pytest.fixture
def case(request,tmp_path):
    root=Path(os.environ.get('STARTUP_ADMISSION_EVIDENCE',str(tmp_path)))/request.node.name
    root.mkdir(parents=True,exist_ok=False)
    folder=root/'probe';folder.mkdir()
    records=[]
    for name,message in [('request.cdr',ListControllers.Request()),('response.cdr',ListControllers.Response())]:
        raw=serialize_message(message);(folder/name).write_bytes(raw)
        records.append(dict(file=name,bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest()))
    receipt=dict(outcome=dict(ok=True,reason='CONTROLLER_MANAGER_RESPONDED',attempt_count=1,response_attempt=1),records=records)
    events=[];clock=Clock();state=SimpleNamespace(exit_code=None)
    data=SimpleNamespace(root=root,folder=folder,receipt=receipt,events=events,clock=clock,state=state,result=None)
    def write():
        (folder/'result.json').write_text(json.dumps(receipt))
    def run():
        def operation():events.append('operation');return 0
        result=run_after_controller_probe(folder,SimpleNamespace(poll=lambda:state.exit_code),
            deadline=.2,on_ready=operation,on_admission=lambda outcome:events.append('admission'),
            clock=clock,sleep=clock.sleep)
        data.result=result
        return result
    data.write,data.run=write,run
    yield data
    (root/'case.json').write_text(json.dumps(dict(result=data.result,events=events,
        clock=clock.value,launch_exit=state.exit_code,fixture_receipt=receipt),indent=2)+'\n')


def test_verified_response_invokes_job_once_after_admission(case):
    case.write();code,admission=case.run()
    assert code==0 and admission['ok']
    assert case.events==['admission','operation']


def test_delayed_complete_receipt_waits_and_preserves_caller_deadline(case):
    (case.folder/'result.json').write_text('{')
    def update():
        if case.clock.value>=.1:case.write()
    case.clock.on_sleep=update
    code,admission=case.run()
    assert code==0 and admission['ok'] and case.clock.value==.1
    assert case.events==['admission','operation']


@pytest.mark.parametrize('fault',['failed','missing_response','hash','wire','unmatched_attempt',
    'malformed_structure','missing','already_spent','terminal','terminal_while_waiting'])
def test_startup_refusal_never_dispatches_operation(case,fault):
    if fault=='failed':case.receipt['outcome']=dict(ok=False,reason='CONTROLLER_MANAGER_RESPONSE_TIMEOUT')
    elif fault=='missing_response':case.receipt['records']=case.receipt['records'][:1]
    elif fault=='hash':(case.folder/'response.cdr').write_bytes(b'changed')
    elif fault=='wire':
        raw=b'bad-wire';(case.folder/'response.cdr').write_bytes(raw)
        case.receipt['records'][1].update(bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())
    elif fault=='unmatched_attempt':case.receipt['outcome'].update(response_attempt=2,attempt_count=2)
    elif fault=='malformed_structure':case.receipt['outcome']=[]
    elif fault=='already_spent':case.clock.value=.19
    elif fault=='terminal':case.state.exit_code=0
    elif fault=='terminal_while_waiting':case.clock.on_sleep=lambda:setattr(case.state,'exit_code',1)
    if fault not in ('missing','already_spent','terminal_while_waiting'):case.write()
    code,admission=case.run()
    assert code==1 and not admission['ok']
    assert case.events==['admission'] and case.clock.value<=.2
