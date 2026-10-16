"""Exact bounded capture can reproduce ordinary footprint evaluation offline."""
import json
from types import SimpleNamespace

import numpy as np
import pytest

from luggage_description.suction_contact_model import ContactModel
from luggage_perception.suction_input_capture import SuctionInputCapture
from luggage_perception.suction_patch_evaluator import SuctionPatchEvaluator
from test_suction_patch_evaluator import INTR, NadirRenderer, make_b1_case, make_model, run_evaluator


def test_exact_capture_replays_candidates_and_preserves_limits(tmp_path):
    case, _ = make_b1_case(NadirRenderer(), 0., 30., 1., 11, 1.)
    model = make_model()
    _, top, expected = run_evaluator(case, model)
    assert expected is not None and expected.accepted
    region = np.zeros(case.depth_mm.shape, dtype=bool)
    x0,y0,x1,y1=case.bbox
    region[max(0,y0):max(0,y1),max(0,x0):max(0,x1)]=True
    capture = SuctionInputCapture(tmp_path, per_identity=1)
    assert capture.capture(case.depth_mm,INTR,case.mat4,top,region,model,expected,stamp_ns=1234567891)
    assert not capture.capture(case.depth_mm,INTR,case.mat4,top,region,model,expected,stamp_ns=1234567892)
    data=np.load(tmp_path/'case-000/inputs.npz')
    record=json.loads((tmp_path/'case-000/record.json').read_text())
    assert record['stamp_ns']==1234567891
    np.testing.assert_array_equal(data['depth_mm'],case.depth_mm)
    rebuilt_top=SimpleNamespace(**record['top'],plane_labels=data['plane_labels'])
    replay=SuctionPatchEvaluator(ContactModel(**record['contact_model']))
    replay.update(data['depth_mm'],SimpleNamespace(**record['intrinsics']),data['optical_to_world'],
        rebuilt_top,instance_region=data['instance_region'],stamp=record['stamp'],
        frame_id=record['frame'],instance_id=record['identity'][0],generation=record['identity'][1])
    actual=replay.copy_output()
    assert actual.accepted==expected.accepted
    assert actual.rejected==expected.rejected
    too_small=SuctionInputCapture(tmp_path/'small',max_case_bytes=1)
    with pytest.raises(ValueError,match='byte budget'):
        too_small.capture(case.depth_mm,INTR,case.mat4,top,region,model,expected,stamp_ns=1)
    assert list((tmp_path/'small').iterdir())==[]
