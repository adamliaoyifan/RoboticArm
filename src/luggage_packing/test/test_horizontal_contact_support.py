"""Native contact conformance, separate from physical subvoxel flatness."""
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np
import pytest

from luggage_description.container_geometry import cuboid_from_inner_size
from luggage_packing.placement_query import placement_query
from luggage_packing.selected_pose_admission import SelectedPoseAdmission
from luggage_packing.placement_volume_query import KnownFreeVolume


@pytest.fixture
def record_case(request, tmp_path):
    root = Path(os.environ.get('CONTACT_SUPPORT_EVIDENCE_DIR', str(tmp_path)))
    folder = root / hashlib.sha256(request.node.nodeid.encode()).hexdigest()[:16]
    folder.mkdir(parents=True, exist_ok=False)

    def record(data):
        raw = json.dumps(dict(case=request.node.nodeid, **data), allow_nan=False).encode()
        (folder / 'case.json').write_bytes(raw)
        (folder / 'sha256.txt').write_text(hashlib.sha256(raw).hexdigest() + '\n')
    return record


def contact_case(resolution, heading, case):
    nx, ny, floor, bottom = 24, 32, .53, .26
    size = (10*resolution, 8*resolution, .1)
    cells = (8, 10) if heading % 2 else (10, 8)
    ix, iy = 7, 9
    origin = (-nx*resolution/2, -ny*resolution/2)
    surface = dict(resolution=resolution, nx=nx, ny=ny,
        inner_size=[nx*resolution, ny*resolution, 1.48], floor_z=0.,
        origin_local=list(origin), center_base=[0., 0., floor+.74], yaw=0.,
        height=[[0.]*ny for _ in range(nx)], state=[['free']*ny for _ in range(nx)],
        confidence=[['sensor']*ny for _ in range(nx)],
        geometry_descriptor=cuboid_from_inner_size(
            (nx*resolution, ny*resolution, 1.48), floor_z=floor).descriptor())
    heights, states = np.asarray(surface['height']), np.asarray(surface['state'], dtype=object)
    patch = heights[ix:ix+cells[0], iy:iy+cells[1]]
    state_patch = states[ix:ix+cells[0], iy:iy+cells[1]]
    patch[:] = bottom
    ratio = 1.
    reason = None
    if case == 'lower_cell':
        patch[0, 0] -= .04
        ratio = 79/80
    elif case == 'central_hole':
        patch[cells[0]//2-2:cells[0]//2+2, cells[1]//2-2:cells[1]//2+2] = .22
        ratio = .8
    elif case == 'broad_ledge':
        patch[:cells[0]//2+1, :] = .22
        ratio = (cells[0]-cells[0]//2-1)/cells[0]
        reason = 'insufficient_support'
    elif case == 'sparse_corners':
        patch[:] = .22
        patch[0, 0] = patch[-1, 0] = patch[0, -1] = patch[-1, -1] = bottom
        ratio, reason = 4/80, 'insufficient_support'
    elif case == 'unsupported_height':
        patch[:] = .22
        ratio, reason = 0., 'insufficient_support'
    elif case == 'ramp':
        patch[:] = np.linspace(.20, bottom, cells[0])[:, None]
        ratio, reason = 1/cells[0], 'insufficient_support'
    elif case == 'higher_cell':
        patch[0, 0] += .02
        reason = 'edge_occupied_overlap'
    elif case == 'unknown_cell':
        state_patch[0, 0] = 'unknown'
        reason = 'unknown_above_floor'
    elif case == 'all_unknown':
        state_patch[:] = 'unknown'
        reason = 'unknown_above_floor'
    elif case != 'flat':
        raise ValueError(case)
    surface.update(height=heights.tolist(), state=states.tolist())
    center = (origin[0]+(ix+cells[0]/2)*resolution,
              origin[1]+(iy+cells[1]/2)*resolution, floor+bottom+.05)
    return surface, size, center, ratio, reason


@pytest.mark.parametrize('resolution', [.01, .02])
@pytest.mark.parametrize('heading', [0, 1, 2, 3])
@pytest.mark.parametrize('case', ['flat', 'lower_cell', 'central_hole', 'broad_ledge',
    'sparse_corners', 'unsupported_height', 'ramp', 'higher_cell', 'unknown_cell', 'all_unknown'])
def test_horizontal_contact_cases(resolution, heading, case, record_case):
    surface, size, center, ratio, expected = contact_case(resolution, heading, case)
    gate = SelectedPoseAdmission(surface, size, volume_verifier=lambda _: None)
    candidate = gate.candidate(center, heading*math.pi/2)
    detail = gate.explain(candidate)
    record_case(dict(surface=surface, candidate=candidate, expected_reason=expected,
        expected_contact_ratio=ratio, result=detail,
        volume_port='scripted_noop_surface_geometry_only'))
    assert gate(candidate) == detail['reason'] == expected
    if case not in ('higher_cell', 'unknown_cell', 'all_unknown'):
        assert detail['support_ratio'] == pytest.approx(ratio, abs=1e-9)
        assert detail['supported_area_m2'] == pytest.approx(ratio*math.prod(size[:2]), abs=1e-12)
    if expected is None:
        assert detail['centre_in_support_polygon']
        assert detail['physical_flatness'] == 'not_established_by_native_grid'


@pytest.mark.parametrize('resolution', [.01, .02])
@pytest.mark.parametrize('heading', [0, 1, 2, 3])
@pytest.mark.parametrize('surrounds_centre', [False, True])
def test_low_configured_area_requires_actual_support_polygon(resolution, heading, surrounds_centre, record_case):
    surface, size, center, _, _ = contact_case(resolution, heading, 'sparse_corners')
    heights = np.asarray(surface['height'])
    cells = (8, 10) if heading % 2 else (10, 8)
    patch = heights[7:7+cells[0], 9:9+cells[1]]
    if not surrounds_centre:
        patch[:] = .22
        patch[-2:, :] = .26
    surface['height'] = heights.tolist()
    gate = SelectedPoseAdmission(surface, size, volume_verifier=lambda _: None,
                                 params={'min_support_ratio': .04})
    candidate = gate.candidate(center, heading*math.pi/2)
    detail = gate.explain(candidate)
    expected = None if surrounds_centre else 'support_centre_outside_polygon'
    record_case(dict(surface=surface, candidate=candidate, result=detail,
        expected_reason=expected, params={'min_support_ratio': .04},
        volume_port='scripted_noop_surface_geometry_only'))
    assert detail['reason'] == expected
    assert detail['centre_in_support_polygon'] is surrounds_centre


def volume_for(surface):
    resolution = surface['resolution']
    nx, ny = surface['nx'], surface['ny']
    nz = math.ceil(1.48/resolution)
    z = np.arange(nz)*resolution
    heights = np.asarray(surface['height'])
    states = np.where(z[:, None, None] < heights.T[None, :, :]-1e-9, 2, 1).astype(np.uint8)
    return dict(resolution=resolution, nx=nx, ny=ny, nz=nz,
                origin_xy=surface['origin_local'], floor_z=.53,
                states=states.tobytes(), overlap_tol=0.)


@pytest.mark.parametrize('resolution', [.01, .02])
@pytest.mark.parametrize('heading', [0, 1, 2, 3])
def test_common_query_scores_and_admits_same_contact_as_selected_target(resolution, heading, record_case):
    surface, size, center, ratio, _ = contact_case(resolution, heading, 'lower_cell')
    surface['confidence'][0][0] = 'initial_empty'
    evidence = volume_for(surface)
    query = placement_query(surface, size, allowed_yaws=[heading*math.pi/2],
        params={'top_n': 10000, 'keep_rejected': 10000},
        volume_verifier=KnownFreeVolume(evidence))
    candidate = next(c for c in query['candidates'] if c['ix0']==7 and c['iy0']==9)
    gate = SelectedPoseAdmission(surface, size, volume_verifier=KnownFreeVolume(evidence))
    detail = gate.explain(gate.candidate(center, heading*math.pi/2))
    record_case(dict(surface=surface, candidate=candidate, selected_target=detail,
        volume=dict(evidence, states=list(evidence['states']))))
    assert candidate['feasible'] and candidate['volume_verified'] and detail['accepted']
    assert candidate['support_source'] == 'sensor'
    assert candidate['support_score'] == pytest.approx(round(ratio, 4))
    assert candidate['detail']['support']['support_ratio'] == pytest.approx(detail['support_ratio'])
