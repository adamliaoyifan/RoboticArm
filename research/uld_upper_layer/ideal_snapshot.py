"""Explicit ideal-simulation adapter; no production perception imports.

Fixture truth enters only this named experiment adapter. Surface Z remains exact
and separate from voxel upper-face heights. Unexported space is UNKNOWN.
"""
import json
from functools import lru_cache
from pathlib import Path
import numpy as np
from dataclasses import replace
from scipy.spatial.transform import Rotation
from geometry import axis_box, separated
from planning_types import BoxState, OccupiedBox, SceneSnapshot, SurfacePatch, digest


def capture_fixture(scene_dir, assets, config):
    data = json.loads((Path(scene_dir)/'fixture.json').read_text())
    if data['geometry_hash'] != assets['hull'].geometry_hash:
        raise ValueError('GEOMETRY_IDENTITY_MISMATCH')
    occupied, surfaces = [], []
    for row in data['boxes']:
        center, size = np.array(row['center_container_m']), np.array(row['size_m'])
        occupied.append(OccupiedBox(row['name'], tuple(center), tuple(size)))
        lo, hi = center-size/2, center+size/2
        surfaces.append(SurfacePatch(row['name'], (lo[0], lo[1], hi[0], hi[1]), hi[2]))
    h = assets['hull']
    # Includes the pickup platform and outside flip/withdrawal space. Static scene
    # meshes are a separate required planning-scene input, not silently empty.
    lo = np.array([-4., -4., -1.]); hi = np.array([4., 4., 4.])
    snapshot = SceneSnapshot(digest(data), h.geometry_hash, 'container_link',
        'ideal_sim_fixture', tuple(lo), tuple(int(v) for v in np.ceil((hi-lo)/config.grid_resolution_m)),
        config.grid_resolution_m, tuple(occupied), tuple(surfaces), tuple(lo)+tuple(hi),
        h.floor_z, h.ceiling_z)
    xyz = data['pickup_payload']['pose_world'][:3]
    pickup = np.linalg.inv(assets['world_container']) @ np.array([
        [1,0,0,xyz[0]], [0,1,0,xyz[1]], [0,0,1,xyz[2]], [0,0,0,1]])
    state = BoxState(data['pickup_payload']['name'], tuple(data['payload']['size_m']),
                     tuple(pickup.ravel()), (0.,0.,0.),mass_kg=float(data['payload']['mass_kg']))
    return snapshot, state


@lru_cache(maxsize=8)
def _occupied_arrays(occupied):
    centers=np.array([item.center for item in occupied],float).reshape(-1,3)
    halves=np.array([item.size for item in occupied],float).reshape(-1,3)/2
    centers.flags.writeable=False;halves.flags.writeable=False
    return centers,halves


def query_box(snapshot, body, clearance=0.):
    bounds = np.array(snapshot.observed_bounds).reshape(2,3)
    vertices = body.corners()
    if np.any(vertices < bounds[0]) or np.any(vertices > bounds[1]):
        return 'UNKNOWN', ()
    hits = []
    centers,halves=_occupied_arrays(snapshot.occupied)
    # A separating world-axis projection of enclosing AABBs is a sufficient
    # proof for the actual OBBs. Only overlapping/near bounds need 15-axis SAT.
    body_half=np.abs(body.axes)@body.half
    gaps=np.abs(centers-body.center)-halves-body_half
    possible=~np.any(gaps>=clearance-1e-9,axis=1)
    for index in np.flatnonzero(possible):
        item=snapshot.occupied[index];center=centers[index];half=halves[index]
        if not separated(body, axis_box(item.name, center-half, center+half), clearance):
            hits.append(item.name)
    return ('OCCUPIED' if hits else 'FREE'), tuple(hits)


def from_observation(snapshot, state, observation, assets):
    """Replace fixture initial conditions by observed, settled simulation poses."""
    def observed_matrix(name):
        row=observation['poses'].get(name)
        if row is None:raise ValueError('OBSERVATION_MISSING:'+name)
        p=row['position'];q=row['orientation']
        transform=np.eye(4)
        transform[:3,:3]=Rotation.from_quat([q.get('x',0),q.get('y',0),q.get('z',0),q.get('w',1)]).as_matrix()
        transform[:3,3]=[p.get('x',0),p.get('y',0),p.get('z',0)]
        return np.linalg.inv(assets['world_container'])@transform
    occupied=[];surfaces=[]
    for item in snapshot.occupied:
        transform=observed_matrix(item.name)
        angle=Rotation.from_matrix(transform[:3,:3]).magnitude()
        if angle>np.deg2rad(.1):raise ValueError('NON_COPLANAR_SUPPORT:'+item.name)
        center=transform[:3,3];half=np.array(item.size)/2
        occupied.append(replace(item,center=tuple(center)))
        surfaces.append(SurfacePatch(item.name,(center[0]-half[0],center[1]-half[1],
            center[0]+half[0],center[1]+half[1]),center[2]+half[2]))
    identity=digest(dict(fixture=snapshot.snapshot_id,poses=observation['poses'],
                         stamp=observation['pose_stamp']))
    return replace(snapshot,snapshot_id=identity,occupied=tuple(occupied),surfaces=tuple(surfaces)),\
           replace(state,container_T_box=tuple(observed_matrix(state.box_id).ravel()))
