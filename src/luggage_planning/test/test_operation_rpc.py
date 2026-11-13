"""A timed-out RPC retains transport ownership until its late result is consumed."""
from concurrent.futures import Future
import json
import time

import pytest

from luggage_planning.joint_operation.rpc import OperationRpc


class Client:
    def __init__(self):self.future=Future();self.submissions=0
    def service_is_ready(self):return True
    def call_async(self,request):self.submissions+=1;return self.future


@pytest.mark.parametrize('late_error',[False,True])
def test_timeout_blocks_duplicate_submission_and_consumes_late_result_or_records_fault(late_error):
    records=[];cleaned=[];client=Client()
    rpc=OperationRpc(lambda name,value:records.append((name,value)))
    with pytest.raises(TimeoutError):
        rpc.call(client,'request',time.monotonic()+.02,'probe',late_response=cleaned.append)
    assert rpc.unresolved() and client.submissions==1
    with pytest.raises(RuntimeError,match='BACKEND_BUSY'):
        rpc.call(client,'duplicate',time.monotonic()+1.,'probe')
    assert client.submissions==1
    if late_error:
        client.future.set_exception(RuntimeError('constructed transport failure'))
        assert rpc.unresolved() and not cleaned
        name,raw=records[-1]
        assert name=='probe_cleanup_fault.json' and json.loads(raw)['reason']=='constructed transport failure'
    else:
        client.future.set_result('late-result')
        assert cleaned==['late-result'] and not rpc.unresolved()
        assert records[-1]==('probe_late_response','late-result')
    assert rpc.consumed[client].is_set()


def test_completed_result_is_returned_once_and_does_not_remain_inflight():
    records=[];client=Client();client.future.set_result('result')
    rpc=OperationRpc(lambda name,value:records.append((name,value)))
    result=rpc.call(client,'request',time.monotonic()+1.,'probe')
    assert result=='result' and not rpc.unresolved() and client.submissions==1
    assert records==[('probe_request','request'),('probe_response','result')]
