import json
import pytest
from luggage_gazebo.depth_observation_targets import load_depth_targets


def test_accepts_bounded_scene_bound_camera_intents(tmp_path):
    p=tmp_path/'targets.json'
    p.write_text(json.dumps(dict(frame_id='world',scene_revision='scene',targets=[
        dict(position=[1.,2.,3.],quaternion_xyzw=[0.,0.,0.,1.])]*2)))
    assert load_depth_targets(p,'scene')==(((1.,2.,3.),(0.,0.,0.,1.)),)*2
    with pytest.raises(ValueError,match='mismatch'):load_depth_targets(p,'other')


@pytest.mark.parametrize('targets',[
    [],[dict(position=[0,0,0],quaternion_xyzw=[0,0,0,1])]*4,
    [dict(position=[float('nan'),0,0],quaternion_xyzw=[0,0,0,1])],
    [dict(position=[0,0,0],quaternion_xyzw=[0,0,0,2])]])
def test_refuses_invalid_camera_intents(tmp_path,targets):
    p=tmp_path/'targets.json';p.write_text(json.dumps(dict(frame_id='world',scene_revision='scene',targets=targets)))
    with pytest.raises(ValueError):load_depth_targets(p,'scene')
