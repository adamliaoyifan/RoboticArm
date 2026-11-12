from types import SimpleNamespace
import json
from concurrent.futures import Future
from rclpy.serialization import deserialize_message
from luggage_msgs.srv import SelectPlacePathV2
from luggage_planning.typed_driver_adapter import TypedDriverAdapter
from luggage_planning.placement_prior.evidence import PriorEvidenceRecorder


def test_service_boundary_captures_exact_messages_before_caller_mutates(tmp_path):
    request=SelectPlacePathV2.Request();request.request_id='exact-select'
    request.start_joints=[.123456789123,0.,0.,0.,0.,0.]
    response=SelectPlacePathV2.Response();response.slot_index=7
    future=Future();future.set_result(response)
    calls=[]
    client=SimpleNamespace(srv_name='/select',wait_for_service=lambda **kw:True,
        call_async=lambda message: calls.append(message) or future)
    adapter=object.__new__(TypedDriverAdapter)
    adapter.runtime=SimpleNamespace(require_graph_ready=lambda:None)
    root=tmp_path/'boundaries'
    adapter.set_boundary_recorder(PriorEvidenceRecorder(root))
    returned=adapter._call(client,request,1.)
    returned.slot_index=99
    rows=[json.loads(line) for line in (root/'boundaries.jsonl').read_text().splitlines()]
    assert [row['boundary'] for row in rows]==['/select:request','/select:response']
    assert deserialize_message((root/rows[0]['file']).read_bytes(),SelectPlacePathV2.Request)==request
    assert deserialize_message((root/rows[1]['file']).read_bytes(),SelectPlacePathV2.Response).slot_index==7
    assert calls==[request]
