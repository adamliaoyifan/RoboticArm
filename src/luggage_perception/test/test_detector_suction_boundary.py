"""The live detector delegates seal acceptance to the footprint evaluator."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import numpy as np


def test_detector_keeps_empty_evaluation_instead_of_minting_center(monkeypatch):
    import os
    path=Path(os.environ.get('ELFIN_TEST_DETECTOR_SCRIPT',
        Path(__file__).parents[1]/'scripts/luggage_detector_node.py'))
    spec=importlib.util.spec_from_file_location('detector_suction_boundary',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    depth=np.ones((4,4));transform=np.eye(4);mask=np.ones((4,4),dtype=bool)
    top=SimpleNamespace(reason='ok')
    output=SimpleNamespace(ok=True,accepted=(),rejected=('measured_patch_rejected',))
    calls=[]
    evaluator=SimpleNamespace(update=lambda *args,**kw:calls.append((args,kw)),copy_output=lambda:output)
    fake=SimpleNamespace(_pop_raw_depth=lambda key:((depth,SimpleNamespace(header=SimpleNamespace(
        frame_id='camera',stamp=SimpleNamespace(sec=1,nanosec=2)))),(1,1,1,1)),
        _peek_instance_mask=lambda key:mask,_select_instance_bbox=lambda msg:(0,0,4,4),
        _component_config=None,_dynamic_config=None,_tf_buffer=None,_world_frame='world',
        _timing={},_suction_evaluator=evaluator)
    monkeypatch.setattr(module,'isolate_depth_component',lambda *args,**kw:SimpleNamespace(ok=True,mask=mask))
    monkeypatch.setattr(module,'_lookup_transform_matrix',lambda *args:(transform,None))
    monkeypatch.setattr(module,'estimate_dynamic_top_surface',lambda *args,**kw:top)
    cloud=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=2)))
    yolo=SimpleNamespace(detections=[object()],instance_id='measured-box',generation=3)
    result,evaluation=module.LuggageDetector._estimate_dynamic_top(fake,cloud,yolo,object(),1.000000002)
    assert result is top and evaluation is output and not evaluation.accepted
    assert len(calls)==1
    args,kw=calls[0]
    assert args[0] is depth and args[2] is transform and args[3] is top
    assert kw['instance_id']=='measured-box' and kw['generation']==3
    assert kw['stamp']==1.000000002 and kw['instance_region'] is mask
    assert kw['acquisition_stamp_ns']==1_000_000_002
    assert kw['optical_frame_id']=='camera' and kw['region_source']=='instance_mask'
    assert fake._timing['suction_mode']=='validated_footprint'


def test_detection_reads_do_not_select_diagnostic_frames():
    import os
    path=Path(os.environ.get('ELFIN_TEST_DETECTOR_SCRIPT',
        Path(__file__).parents[1]/'scripts/luggage_detector_node.py'))
    spec=importlib.util.spec_from_file_location('detector_capture_selection',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    calls=[]
    def pin(*identity):
        calls.append(identity)
        return {'status':'exact_input_missing'}
    warnings=[]
    fake=SimpleNamespace(_stamp_age_sec=lambda stamp:0.,_cloud_max_age=1.,
        _use_semantic=True,_box_epoch_seen=True,_box_generation=7,
        _acquisition_unsettled=lambda stamp:False,
        _suction_input_capture=SimpleNamespace(pin=pin),_warn_throttled=warnings.append)
    box=object()
    frame=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=123,nanosec=456789123)),
        instance_id='selected-box',generation=7,pca_valid=True,pca_reason='ok',
        pca_confidence=.9,box=box)
    for _ in range(16):
        result,confidence=module.LuggageDetector._frame_to_detect(fake,frame)
        assert result is box and confidence==.9
    assert calls==[] and warnings==[]
    domain=SimpleNamespace(instance_id='selected-box',generation=7,stamp_ns=123456789123)
    module.LuggageDetector._capture_retained_suction_input(fake,domain)
    assert calls==[('selected-box',7,123456789123)]
    assert warnings and 'exact_input_missing' in warnings[0]
