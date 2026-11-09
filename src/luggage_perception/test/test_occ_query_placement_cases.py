"""Validate the intended path and genuine OCC/v2 fixture before ROS scoring."""
import importlib.util
import math
from pathlib import Path

import pytest
import yaml


@pytest.mark.parametrize('resolution',[.01,.02,.05])
@pytest.mark.parametrize('boxes',[0,5,20])
def test_real_snapshot_hit_and_forced_fallback_paths(resolution,boxes):
    import sys
    root=Path(__file__).resolve().parents[3]
    sys.path.insert(0,str(root/'src/luggage_packing'))
    from luggage_packing.placement_search_session import PlacementSearchSession
    from luggage_description.frame_contracts import validate_surface
    path=Path(__file__).with_name('occ_query_placement_cases.py')
    spec=importlib.util.spec_from_file_location('placement_cases',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    scene=yaml.safe_load((root/'src/luggage_description/config/scene_tf.yaml').read_text())
    for fallback in (False,True):
        store,geometry,points=module.create_case(scene,resolution,boxes,fallback)
        assert len(store.mapper._placed_boxes)==len(geometry)==boxes
        revision=store.identity.revision
        record=store.v2_record('scene','map-rev-'+str(revision))
        validate_surface(record,'scene',store.mapper.geometry_hash,revision)
        batch=PlacementSearchSession(store.v1_surface_dict(),module.BOX,
            [0.,math.pi/2,math.pi,-math.pi/2],budget_s=20.).start()
        assert batch['source']==('grid_fallback' if fallback else 'structural'),batch
        assert batch['candidates']
        assert any(candidate['feasible'] for candidate in batch['candidates'])
