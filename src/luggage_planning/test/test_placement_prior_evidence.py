import json
import pytest
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from luggage_planning.placement_prior_evidence import PriorEvidenceRecorder


def test_exact_capture_and_budget(tmp_path):
    root=tmp_path/'capture'
    record=PriorEvidenceRecorder(root,max_bytes=64)
    message=String(data='exact boundary')
    record('test',message)
    row=json.loads((root/'boundaries.jsonl').read_text())
    assert deserialize_message((root/row['file']).read_bytes(),String)==message
    with pytest.raises(RuntimeError,match='BUDGET'):
        record('oversize',String(data='x'*65))
    assert len(list(root.glob('*.cdr')))==1
    with pytest.raises(FileExistsError):
        PriorEvidenceRecorder(root)
