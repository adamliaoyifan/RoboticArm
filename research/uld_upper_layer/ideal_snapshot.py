"""Explicit ideal-simulation adapter; no production perception imports.

Fixture truth enters only this named experiment adapter. Surface Z remains exact
and separate from voxel upper-face heights. Unexported space is UNKNOWN.
"""
import json
from pathlib import Path
import numpy as np
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
                     tuple(pickup.ravel()), (0.,0.,0.))
    return snapshot, state


def query_box(snapshot, body, clearance=0.):
    bounds = np.array(snapshot.observed_bounds).reshape(2,3)
    vertices = body.corners()
    if np.any(vertices < bounds[0]) or np.any(vertices > bounds[1]):
        return 'UNKNOWN', ()
    hits = []
    for item in snapshot.occupied:
        center, size = np.array(item.center), np.array(item.size)
        if not separated(body, axis_box(item.name, center-size/2, center+size/2), clearance):
            hits.append(item.name)
    return ('OCCUPIED' if hits else 'FREE'), tuple(hits)
