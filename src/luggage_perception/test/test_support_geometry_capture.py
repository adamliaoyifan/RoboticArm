import json
from types import SimpleNamespace
import numpy as np
import pytest
from luggage_perception.support_geometry_capture import SupportGeometryCapture
from luggage_perception.top_support_estimator import (
    TopSupportConfig,TopSurfaceEstimate,SupportPlaneEstimate,compose_box_geometry)


def result():
    top=TopSurfaceEstimate(np.array([.1,.2]),1.1,0.,.5,.3,.9)
    support=SupportPlaneEstimate(.86,residual=.001)
    return SimpleNamespace(height_valid=True,box=compose_box_geometry(top,support),support=support)


def test_exact_inputs_pinned_once_and_bounded(tmp_path):
    capture=SupportGeometryCapture(tmp_path,max_cases=1)
    raw=np.array([[.1,.2,.86],[.2,.3,.86000000001]],dtype=np.float64)
    fitted=result();before=fitted.box.height
    assert capture.capture(('box',1),42,raw,raw,TopSupportConfig(),fitted,[.86]*5)
    saved=np.load(tmp_path/'case-00/inputs.npz')
    assert np.array_equal(saved['raw'],raw)
    assert json.loads((tmp_path/'case-00/record.json').read_text())['stamp_ns']==42
    assert not capture.capture(('box',1),43,raw,raw,TopSupportConfig(),fitted,[])
    assert not capture.capture(('box',2),44,raw,raw,TopSupportConfig(),fitted,[])
    assert fitted.box.height==before


def test_byte_budget_and_missing_raw_refuse_capture(tmp_path):
    capture=SupportGeometryCapture(tmp_path,max_case_bytes=1)
    with pytest.raises(ValueError,match='byte budget'):
        capture.capture(('box',1),42,np.zeros((2,3)),np.zeros((2,3)),TopSupportConfig(),result(),[])
    assert not list(tmp_path.iterdir())


def test_failed_fit_requires_opt_in_and_preserves_status_and_history(tmp_path):
    fitted=result();fitted.height_valid=False
    fitted.support.reason='DETECT_SUPPORT_UNSTABLE'
    capture=SupportGeometryCapture(tmp_path,max_cases=1)
    points=np.array([[.1,.2,.86]],dtype=np.float64)
    history=[.86,.8601]
    assert not capture.capture(('box',1,42),42,points,points,TopSupportConfig(),fitted,history)
    assert capture.capture(('box',1,42),42,points,points,TopSupportConfig(),fitted,history,allow_invalid=True)
    saved=json.loads((tmp_path/'case-00/record.json').read_text())
    assert saved['height_valid'] is False
    assert saved['support']['reason']=='DETECT_SUPPORT_UNSTABLE'
    assert saved['support_history']==history
    assert not capture.capture(('box',1,43),43,points,points,TopSupportConfig(),fitted,history,allow_invalid=True)
    assert fitted.height_valid is False and history==[.86,.8601]
