"""Real CDR alignment noise must not be confused with a changed slot."""
import importlib.util
from pathlib import Path

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
