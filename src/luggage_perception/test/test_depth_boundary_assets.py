import hashlib
from pathlib import Path
import numpy as np
import pytest
from luggage_description.scene_tf_config_utils import load_scene_tf_config
from luggage_description.container_geometry import descriptor_from_scene_config
from luggage_description.stl_geometry import stl_arrays_from_bytes
from luggage_perception.depth_boundary_assets import load_boundary_assets

ROOT=Path(__file__).resolve().parents[2]
SCENE=ROOT/'luggage_description/config/scene_tf.yaml.example'
MESH=ROOT/'luggage_gazebo/models/airport_container_real/meshes/container_collision.stl'


def inputs():
    return dict(expected_scene_revision=hashlib.md5(SCENE.read_bytes()).hexdigest(),
        expected_geometry_hash=descriptor_from_scene_config(load_scene_tf_config(str(SCENE))).geometry_hash,
        expected_resource_sha256=hashlib.sha256(MESH.read_bytes()).hexdigest(),mesh_override=MESH)


def test_authoritative_mesh_bytes_and_canonical_geometry_are_bound():
    assets=load_boundary_assets(SCENE,**inputs())
    vertices,faces=stl_arrays_from_bytes(MESH.read_bytes())
    expected=hashlib.sha256(np.asarray(vertices[faces],dtype='<f8').tobytes()).hexdigest()
    assert assets.mesh.mesh_sha256==expected
    assert assets.resource_sha256==inputs()['expected_resource_sha256']
    hit=assets.mesh.first_hit((0.,.14,1.),(1.,.14,1.),deadline=1.,clock=lambda:0.)
    assert .764 < hit.distance_m < .766


@pytest.mark.parametrize('field',[
    'expected_scene_revision','expected_geometry_hash','expected_resource_sha256'])
def test_identity_mismatch_refuses_loading(field):
    values=inputs();values[field]='wrong'
    with pytest.raises(ValueError,match='identity mismatch'):
        load_boundary_assets(SCENE,**values)


def test_missing_resource_identity_is_not_self_authorized():
    values=inputs();values['expected_resource_sha256']=''
    with pytest.raises(ValueError,match='identities required'):
        load_boundary_assets(SCENE,**values)


def test_truncated_resource_refused_even_if_hash_is_supplied(tmp_path):
    path=tmp_path/'broken.stl';path.write_bytes(b'broken mesh')
    values=inputs();values.update(mesh_override=path,expected_resource_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    with pytest.raises(ValueError,match='too small'):
        load_boundary_assets(SCENE,**values)
