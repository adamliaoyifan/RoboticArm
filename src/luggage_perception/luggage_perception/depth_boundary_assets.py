"""Load scene-bound static container geometry without a planning dependency.

Resource identity must be supplied by the deployment owner. No current-world
query, Gazebo truth, automatic geometry adjustment, or map mutation occurs.
"""
from dataclasses import dataclass
import hashlib
from pathlib import Path
import numpy as np
import yaml
from luggage_description.scene_mesh_utils import container_collision_mesh_path
from luggage_description.container_geometry import descriptor_from_scene_config
from luggage_description.stl_geometry import stl_arrays_from_bytes
from luggage_perception.depth_boundary_mesh import StaticBoundaryMesh


@dataclass(frozen=True)
class BoundaryAssets:
    mesh: StaticBoundaryMesh
    resource_path: str
    resource_sha256: str
    geometry_hash: str
    scene_sha256: str


def _read_bounded(path, limit):
    with Path(path).open('rb') as stream:
        data = stream.read(limit+1)
    if len(data)>limit:
        raise ValueError('boundary asset exceeds byte budget')
    return data


def load_boundary_assets(scene_path, *, expected_scene_revision, expected_geometry_hash,
                         expected_resource_sha256, mesh_override=None):
    if not all((expected_scene_revision,expected_geometry_hash,expected_resource_sha256)):
        raise ValueError('deployment scene, geometry and resource identities required')
    scene_bytes = _read_bounded(scene_path,1024*1024)
    if hashlib.md5(scene_bytes).hexdigest()!=expected_scene_revision:
        raise ValueError('boundary scene identity mismatch')
    config = yaml.safe_load(scene_bytes)
    geometry = descriptor_from_scene_config(config)
    if geometry.geometry_hash!=expected_geometry_hash:
        raise ValueError('boundary usable geometry identity mismatch')
    path = Path(mesh_override or container_collision_mesh_path(config)).resolve()
    data = _read_bounded(path,8*1024*1024)
    resource_hash = hashlib.sha256(data).hexdigest()
    if resource_hash!=expected_resource_sha256:
        raise ValueError('boundary resource identity mismatch')
    vertices,faces = stl_arrays_from_bytes(data,str(path))
    triangles = np.asarray(vertices[faces],dtype='<f8',order='C')
    canonical_hash = hashlib.sha256(triangles.tobytes()).hexdigest()
    mesh = StaticBoundaryMesh(triangles,scene_revision=expected_scene_revision,
                              expected_sha256=canonical_hash)
    return BoundaryAssets(mesh,str(path),resource_hash,geometry.geometry_hash,
                          hashlib.sha256(scene_bytes).hexdigest())
