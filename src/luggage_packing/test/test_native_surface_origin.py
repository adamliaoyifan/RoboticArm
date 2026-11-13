"""Coordinate conformance at declared native origins, not a motion certificate."""
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np
import pytest

from luggage_description.container_geometry import cuboid_from_inner_size
from luggage_packing.selected_pose_admission import SelectedPoseAdmission
from luggage_packing.placement_query import placement_query
from luggage_packing.placement_solver import (
    generate_candidates, _footprint_edge_state, _grid_arrays, _edge_scan_grid,
)
from luggage_packing.placement_volume_query import KnownFreeVolume


@pytest.fixture
def archive(request, tmp_path):
    root = Path(os.environ.get('NATIVE_ORIGIN_EVIDENCE_DIR',str(tmp_path)))
    folder = root/hashlib.sha256(request.node.nodeid.encode()).hexdigest()[:16]
    folder.mkdir(parents=True,exist_ok=False)
    def write(data):
        def encode(value):
            if isinstance(value,bytes): return list(value)
            if isinstance(value,np.ndarray): return value.tolist()
            raise TypeError(type(value).__name__)
        raw = json.dumps(dict(case=request.node.nodeid,**data),default=encode).encode()
        (folder/'case.json').write_bytes(raw)
        (folder/'sha256.txt').write_text(hashlib.sha256(raw).hexdigest()+'\n')
    return write


def native_surface(resolution, *, divisible=False, explicit=True):
    extent = ((24.,32.) if divisible else (23.5,31.5))
    length,width = (v*resolution for v in extent)
    nx,ny = math.ceil(length/resolution),math.ceil(width/resolution)
    surface = dict(resolution=resolution,nx=nx,ny=ny,inner_size=[length,width,1.48],
        floor_z=0.,center_base=[0.,0.,.53+1.48/2],yaw=0.,
        height=[[0.]*ny for _ in range(nx)],state=[['free']*ny for _ in range(nx)],
        confidence=[['sensor']*ny for _ in range(nx)],
        geometry_descriptor=cuboid_from_inner_size((length,width,1.48),floor_z=.53).descriptor())
    origin = (-length/2,-width/2) if explicit else (-nx*resolution/2,-ny*resolution/2)
    if explicit: surface['origin_local'] = list(origin)
    return surface,origin


@pytest.mark.parametrize('resolution',[.01,.02])
@pytest.mark.parametrize('divisible',[False,True])
@pytest.mark.parametrize('explicit',[False,True])
@pytest.mark.parametrize('heading',[0,1,2,3])
@pytest.mark.parametrize('shift_cells',[(0.,0.),(.25,-.25),(.75,.25)])
def test_partial_support_area_uses_declared_origin(resolution,divisible,explicit,heading,shift_cells,archive):
    surface,origin = native_surface(resolution,divisible=divisible,explicit=explicit)
    cells = (3,2) if heading%2 else (2,3)
    ix,iy = 7,9
    for x in range(ix,ix+cells[0]):
        for y in range(iy,iy+cells[1]):
            surface['height'][x][y]=.26
            surface['state'][x][y]='occupied'
    center = (origin[0]+(ix+cells[0]/2+shift_cells[0])*resolution,
              origin[1]+(iy+cells[1]/2+shift_cells[1])*resolution,.53+.26+.1)
    size = (2*resolution,3*resolution,.2)
    gate = SelectedPoseAdmission(surface,size,volume_verifier=lambda _:None)
    detail = gate.explain(gate.candidate(center,heading*math.pi/2))
    expected_area = math.prod((cells[a]-abs(shift_cells[a]))*resolution for a in range(2))
    expected_ratio = expected_area/(cells[0]*cells[1]*resolution**2)
    archive(dict(surface=surface,size=size,center_container=center,heading=heading,
        volume_port='scripted_noop_surface_geometry_only',expected_area_m2=expected_area,
        expected_ratio=expected_ratio,result=detail))
    assert detail['supported_area_m2'] == pytest.approx(expected_area,abs=1e-12)
    assert detail['support_ratio'] == pytest.approx(expected_ratio,abs=1e-9)
    assert detail['accepted'] is (expected_ratio>=.6-1e-9)


@pytest.mark.parametrize('resolution',[.01,.02])
@pytest.mark.parametrize('state',['unknown','occupied'])
@pytest.mark.parametrize('shift_cells',[0.,.125])
def test_scalar_edge_scan_preserves_face_contact_and_actual_sliver(resolution,state,shift_cells,archive):
    surface,origin = native_surface(resolution)
    ix,iy = 7,9
    surface['state'][ix+1][iy]=state
    surface['height'][ix+1][iy]=.28 if state=='occupied' else 0.
    center = (origin[0]+(ix+.5+shift_cells)*resolution,origin[1]+(iy+.5)*resolution)
    result = _footprint_edge_state(surface,*center,resolution,resolution,.26,overlap_tol=0.)
    expected = (shift_cells>0. and state=='occupied',shift_cells>0. and state=='unknown')
    archive(dict(surface=surface,center_container=center,footprint=[resolution]*2,
                 bottom_z_floor_relative_container_m=.26,result=result,expected=expected))
    assert result==expected


@pytest.mark.parametrize('resolution',[.01,.02])
def test_generated_centres_and_vector_edge_scan_follow_explicit_origin(resolution,archive):
    surface,origin = native_surface(resolution)
    surface['height']=[[.26]*surface['ny'] for _ in range(surface['nx'])]
    # Shift the native grid within the hull to distinguish declaration from a
    # reconstructed hull/grid centre. This tests a pure consumer, not admission
    # of a changed producer descriptor at the ROS boundary.
    origin = (origin[0]+resolution/8,origin[1]-resolution/8)
    surface['origin_local']=list(origin)
    ix,iy=7,9
    surface['state'][ix+1][iy]='unknown';surface['height'][ix+1][iy]=0.
    size=(resolution,resolution,.2)
    rows=generate_candidates(surface,size,allowed_yaws=[0.],params=dict(top_n=2000,keep_rejected=2000))
    target=next(r for r in rows if r['ix0']==ix and r['iy0']==iy)
    expected=[origin[0]+(ix+.5)*resolution,origin[1]+(iy+.5)*resolution]
    archive(dict(surface=surface,size=size,result=target,expected_center_xy_container=expected))
    assert target['center_container'][:2]==pytest.approx(expected,abs=1e-12)
    assert target['feasible'] and target['confidence_ratio']==1.


@pytest.mark.parametrize('resolution',[.01,.02])
@pytest.mark.parametrize('state',[0,2])
@pytest.mark.parametrize('shift_cells',[0.,.125])
def test_same_source_xyz_volume_keeps_positive_sliver_obstacles(resolution,state,shift_cells,archive):
    surface,origin=native_surface(resolution)
    nx,ny=surface['nx'],surface['ny'];nz=round(.6/resolution)
    ix,iy=7,9
    surface['height']=[[.26]*ny for _ in range(nx)]
    surface['state'][ix+1][iy]='unknown' if state==0 else 'occupied'
    surface['height'][ix+1][iy]=0. if state==0 else .28
    states=np.ones((nz,ny,nx),dtype=np.uint8)
    states[:,iy,ix+1]=state
    evidence=dict(resolution=resolution,nx=nx,ny=ny,nz=nz,origin_xy=origin,
                  floor_z=.53,states=states.tobytes(),overlap_tol=0.)
    size=(resolution,resolution,.2)
    center=(origin[0]+(ix+.5+shift_cells)*resolution,origin[1]+(iy+.5)*resolution,.53+.26+.1)
    gate=SelectedPoseAdmission(surface,size,volume_verifier=KnownFreeVolume(evidence))
    candidate=gate.candidate(center,0.);detail=gate.explain(candidate)
    volume_reason=KnownFreeVolume(evidence)(candidate)
    archive(dict(surface=surface,evidence=evidence,candidate=candidate,result=detail,volume_reason=volume_reason))
    assert detail['accepted'] is (shift_cells==0.)
    assert (volume_reason is None) is (shift_cells==0.)


@pytest.mark.parametrize('resolution',[.01,.02])
def test_shared_query_admits_native_face_but_not_unknown_neighbour(resolution,archive):
    surface,origin=native_surface(resolution)
    nx,ny=surface['nx'],surface['ny'];nz=round(.6/resolution)
    surface['height']=[[.26]*ny for _ in range(nx)]
    ix,iy=7,9;surface['state'][ix+1][iy]='unknown';surface['height'][ix+1][iy]=0.
    states=np.ones((nz,ny,nx),dtype=np.uint8);states[:,iy,ix+1]=0
    evidence=dict(resolution=resolution,nx=nx,ny=ny,nz=nz,origin_xy=origin,
                  floor_z=.53,states=states.tobytes(),overlap_tol=0.)
    result=placement_query(surface,(resolution,resolution,.2),allowed_yaws=[0.],
        volume_verifier=KnownFreeVolume(evidence))
    own=next(r for r in result['candidates'] if r['ix0']==ix and r['iy0']==iy)
    archive(dict(surface=surface,evidence=evidence,result=own))
    assert own['feasible'] and own['volume_verified']


@pytest.mark.parametrize('resolution',[.01,.02])
@pytest.mark.parametrize('heading',[0,1,2,3])
def test_translating_declared_grid_and_target_preserves_support(resolution,heading,archive):
    surface,origin=native_surface(resolution)
    surface['height'][7][9]=.26
    size=(resolution,resolution,.2)
    results=[]
    for dx,dy in [(0.,0.),(resolution/8,-resolution/8)]:
        surface['origin_local']=[origin[0]+dx,origin[1]+dy]
        center=(origin[0]+dx+7.5*resolution,origin[1]+dy+9.5*resolution,.53+.26+.1)
        gate=SelectedPoseAdmission(surface,size,volume_verifier=lambda _:None)
        results.append(gate.explain(gate.candidate(center,heading*math.pi/2)))
    archive(dict(surface=surface,size=size,results=results,
                 volume_port='scripted_noop_surface_geometry_only'))
    assert all(r['accepted'] for r in results)
    assert results[0]['support_ratio']==pytest.approx(results[1]['support_ratio'],abs=1e-9)
    assert results[0]['supported_area_m2']==pytest.approx(results[1]['supported_area_m2'],abs=1e-12)


@pytest.mark.parametrize('resolution',[.01,.02])
@pytest.mark.parametrize('explicit',[False,True])
@pytest.mark.parametrize('heading',[0,1,2,3])
def test_scalar_and_batch_edge_scans_agree_at_all_window_positions(resolution,explicit,heading,archive):
    surface,origin=native_surface(resolution,explicit=explicit)
    rng=np.random.default_rng(419)
    state=rng.choice(['free','occupied','unknown'],(surface['nx'],surface['ny']))
    height=np.where(state=='occupied',rng.choice([.1,.2,.3],state.shape),0.)
    surface['state']=state.tolist();surface['height']=height.tolist()
    footprint=(3.5*resolution,2.25*resolution) if heading%2 else (2.25*resolution,3.5*resolution)
    cells=tuple(math.ceil(v/resolution) for v in footprint)
    peak=np.lib.stride_tricks.sliding_window_view(height,cells).max(axis=(2,3))
    candidate_origin=origin if explicit else tuple(-v/2 for v in surface['inner_size'][:2])
    arrays=_grid_arrays(surface)
    batch=_edge_scan_grid(arrays,peak,surface['nx'],surface['ny'],resolution,
        surface['inner_size'][0]/2,surface['inner_size'][1]/2,*footprint,*cells,1,0.,
        grid_origin_xy_container=origin,candidate_origin_xy_container=candidate_origin)
    scalar=[np.zeros(peak.shape,dtype=bool),np.zeros(peak.shape,dtype=bool)]
    for i in range(peak.shape[0]):
        for j in range(peak.shape[1]):
            xy=(candidate_origin[0]+(i+cells[0]/2)*resolution,
                candidate_origin[1]+(j+cells[1]/2)*resolution)
            values=_footprint_edge_state(surface,*xy,*footprint,float(peak[i,j]),overlap_tol=0.)
            for k,value in enumerate(values):scalar[k][i,j]=value
    archive(dict(surface=surface,footprint=footprint,candidate_origin_xy_container=candidate_origin,
                 batch=list(batch),scalar=scalar))
    assert all(np.array_equal(a,b) for a,b in zip(batch,scalar))
