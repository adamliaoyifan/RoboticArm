"""The sampled bounds MD5 and normalized operation hull SHA256 are distinct."""
from pathlib import Path
from types import SimpleNamespace
import hashlib
import time
import pytest
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from luggage_description.container_geometry import descriptor_from_scene_config
from luggage_description.scene_tf_config_utils import load_scene_tf_config,world_transform
from luggage_planning.joint_operation_atlas import load_operation_atlas,AtlasOperationHints
from luggage_planning.placement_prior_contracts import Transform


def binding():
    folder=Path(get_package_share_directory('luggage_planning'))/'data/reachability_atlas_deploy_update_gazebo'
    scene=Path(get_package_share_directory('luggage_description'))/'config/scene_tf_deploy_update.yaml'
    xml=(folder/'robot_description.xml').read_text();srdf=(folder/'robot_description_semantic.xml').read_text()
    config=load_scene_tf_config(str(scene));xyz,rpy=world_transform(config,'container_link')
    world=Transform(tuple(xyz),tuple(Rotation.from_euler('xyz',rpy).as_quat()))
    revision=hashlib.md5(scene.read_bytes()).hexdigest();model=hashlib.sha256((xml+srdf).encode()).hexdigest()
    reader=load_operation_atlas(folder/'s20_container_collision_aware.npz',
        folder/'s20_container_collision_aware.yaml',folder/'operation_atlas_qualification.json',scene,
        robot_description=xml,model_sha256=model,scene_revision=revision,world_from_container=world)
    return reader,world,revision,model,descriptor_from_scene_config(config).geometry_hash


@pytest.mark.parametrize('changed',['none','geometry','model','scene'])
def test_distinct_hash_contracts_allow_query_but_changed_operation_identity_refuses(changed):
    reader,world,revision,model,geometry=binding()
    assert reader.source.geometry_hash != geometry
    provider=AtlasOperationHints(reader,None,None,scene_revision=revision,model_sha256=model,
        operation_geometry_hash=geometry)
    scene=SimpleNamespace(scene_revision=revision,model_hash=model,geometry_hash=geometry,world_from_container=world)
    if changed=='geometry':scene.geometry_hash='different-hull'
    if changed=='model':scene.model_hash='different-model'
    if changed=='scene':scene.scene_revision='different-scene'
    down=Transform((0.,0.,0.),(0.,1.,0.,0.))
    box=world.compose(Transform((0.,0.,.8),(0.,0.,0.,1.)))
    candidate=SimpleNamespace(scenes=SimpleNamespace(carry=scene),world_from_pick_box=Transform((0.,0.,0.),(0.,0.,0.,1.)),
        world_from_pick_tool=down,world_from_settled_box=box,world_from_release_tool=box.compose(down))
    # An expired refinement budget isolates actual artifact queries from native
    # motion. No fake IK/collision pass can authorize a seed or trajectory.
    _,trace=provider.augment(candidate,request_id='geometry-regression',deadline=time.monotonic()-1.,
        cancelled=lambda:False,clock=time.monotonic)
    if changed=='none':
        assert any(row[0]=='atlas_contact' for row in trace)
        assert any(row[0]=='atlas_release' for row in trace)
        assert not any(row[0]=='atlas' and row[1]=='ATLAS_CURRENT_IDENTITY_MISMATCH' for row in trace)
    else:
        assert trace==( ('atlas','ATLAS_CURRENT_IDENTITY_MISMATCH'), )
