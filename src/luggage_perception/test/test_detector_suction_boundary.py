"""The live detector delegates seal acceptance to the footprint evaluator."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import numpy as np


def test_detector_keeps_empty_evaluation_instead_of_minting_center(monkeypatch):
    path=Path(__file__).parents[1]/'scripts/luggage_detector_node.py'
    spec=importlib.util.spec_from_file_location('detector_suction_boundary',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    depth=np.ones((4,4));transform=np.eye(4);mask=np.ones((4,4),dtype=bool)
    top=SimpleNamespace(reason='ok')
    output=SimpleNamespace(ok=True,accepted=(),rejected=('measured_patch_rejected',))
    calls=[]
    evaluator=SimpleNamespace(update=lambda *args,**kw:calls.append((args,kw)),copy_output=lambda:output)
    fake=SimpleNamespace(_pop_raw_depth=lambda key:((depth,SimpleNamespace(header=SimpleNamespace(frame_id='camera'))),(1,1,1,1)),
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
    assert fake._timing['suction_mode']=='validated_footprint'
