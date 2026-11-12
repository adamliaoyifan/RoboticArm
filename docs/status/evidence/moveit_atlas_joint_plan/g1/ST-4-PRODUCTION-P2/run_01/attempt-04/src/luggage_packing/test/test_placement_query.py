"""Behavior of the shared production query, including strict volume admission."""
import json
import math
import os
from pathlib import Path
import random
import time

import pytest

from luggage_description.container_geometry import cuboid_from_inner_size
from luggage_packing.placement_query import placement_query
from luggage_packing.placement_solver import volume_state_in_evidence
from luggage_packing.placement_volume_query import KnownFreeVolume


def observed_map(*, resolution=.02, floor=.53):
    side, height = .4, .4
    n, nz = round(side / resolution), round(height / resolution)
    surface = dict(resolution=resolution, nx=n, ny=n, inner_size=[side, side, height],
        floor_z=0., center_base=[0., 0., floor + height / 2], yaw=0.,
        state=[['free'] * n for _ in range(n)], height=[[0.] * n for _ in range(n)],
        confidence=[['sensor'] * n for _ in range(n)],
        geometry_descriptor=cuboid_from_inner_size((side, side, height), floor_z=floor).descriptor())
    evidence = dict(resolution=resolution, nx=n, ny=n, nz=nz, origin_xy=[-.2, -.2],
                    floor_z=floor, states=[1] * (n * n * nz), overlap_tol=0.)
    return surface, evidence


@pytest.mark.parametrize('size', [(.12, .08, .1), (.1, .1, .1)])
@pytest.mark.parametrize('limit', [4, 8])
def test_four_directed_headings_survive_identical_footprints_and_pool_limits(size, limit):
    surface, evidence = observed_map()
    result = placement_query(surface, size, params=dict(top_n=limit),
                             volume_verifier=KnownFreeVolume(evidence))
    candidates = [c for c in result['candidates'] if c['feasible']]
    assert len(candidates) == limit
    assert {round(c['heading_theta'] / (math.pi / 2)) % 4 for c in candidates} == {0, 1, 2, 3}
    assert all(c['volume_verified'] for c in candidates)
    if limit == 8:
        assert max(c['depth_m'] for c in candidates[4:]) < min(c['depth_m'] for c in candidates[:4])


@pytest.mark.parametrize('resolution', [.01, .02])
@pytest.mark.parametrize('floor', [-.17, .53])
def test_absolute_origin_is_independent_of_legacy_height_zero(resolution, floor):
    surface, evidence = observed_map(resolution=resolution, floor=floor)
    result = placement_query(surface, (.1, .1, .1), params=dict(top_n=4),
                             volume_verifier=KnownFreeVolume(evidence))
    assert result['success']
    assert all(c['center_container'][2] == pytest.approx(floor + .05)
               for c in result['candidates'] if c['feasible'])


@pytest.mark.parametrize('defect', ['missing', 'short', 'unknown', 'occupied', 'inactive'])
def test_floor_prior_never_authorizes_unseen_volume(defect):
    surface, evidence = observed_map()
    surface['state'] = [['unknown'] * surface['ny'] for _ in range(surface['nx'])]
    if defect == 'missing':
        verifier = None
    else:
        if defect == 'short': evidence['states'] = evidence['states'][:-1]
        if defect == 'unknown': evidence['states'] = [0] * len(evidence['states'])
        if defect == 'occupied': evidence['states'] = [2] * len(evidence['states'])
        if defect == 'inactive': evidence['active'] = [False] * len(evidence['states'])
        verifier = KnownFreeVolume(evidence)
    result = placement_query(surface, (.1, .1, .1), volume_verifier=verifier)
    assert not result['success']
    assert not any(c['volume_verified'] for c in result['candidates'])
    assert result['reason_code'] != 'BIN_FULL'


def test_face_touching_is_allowed_but_positive_penetration_is_rejected():
    surface, evidence = observed_map()
    # Occupy the left half up to 20cm. A final box may touch x=0 exactly.
    n = evidence['nx']
    for z in range(10):
        for y in range(n):
            for x in range(n // 2): evidence['states'][x+n*(y+n*z)] = 2
    verify = KnownFreeVolume(evidence)
    touching = dict(center_container=[.05, 0., .58], footprint=[.1, .1], size=[.1, .1, .1])
    assert verify(touching) is None
    penetrating = dict(touching, center_container=[.049, 0., .58])
    assert verify(penetrating) == 'volume_occupied'
    # The measured box ledger and the voxel proof both distinguish touching
    # from penetration; the default layout rule adds no five-centimeter gap.
    placed = [(-.2, -.2, 0., 0., .2, .2)]
    result = placement_query(surface, (.1, .1, .1), placed_aabbs=placed,
                             volume_verifier=verify, params=dict(top_n=1000, hull_margin=0.))
    assert result['success']
    assert any(c['center_container'][0] == pytest.approx(.05)
               for c in result['candidates'] if c['feasible'])
    assert result['admission_policy']['nominal_side_gap_m'] == 0.


def test_blocked_straight_corridor_and_center_atlas_are_hints_not_exclusion():
    surface, evidence = observed_map()
    wall = [(-.1, -.2, 0., -.08, .2, .2)]
    # Isolate corridor policy from the measured-volume gate: wall occupies
    # only this hypothetical straight approach, not the chosen final volume.
    def center_prior(candidates):
        for c in candidates:
            if c['feasible']:
                c.update(feasible=False, reason='atlas_unreachable')
        return candidates, {'unreachable': len(candidates)}
    result = placement_query(surface, (.08, .08, .1), placed_aabbs=wall,
        volume_verifier=KnownFreeVolume(evidence), atlas_annotate=center_prior)
    assert result['success'] and result['atlas']['authority'] == 'ranking_only'
    assert any(c['feasible'] and c['detail'].get('straight_corridor_blocked')
               and c['detail'].get('center_atlas_rejection_hint') == 'atlas_unreachable'
               for c in result['candidates'])
    assert not any(c['feasible'] and c['reason'] != 'ok' for c in result['candidates'])


def test_integral_volume_matches_zero_erosion_scalar_reference():
    rng = random.Random(20261112)
    _, evidence = observed_map(resolution=.02)
    # A mixture of entirely free areas, occupied and unknown cells produces
    # both positive and negative cases; the reference remains independent.
    for i in range(len(evidence['states']) // 2, len(evidence['states'])):
        evidence['states'][i] = rng.choices([0, 1, 2], [1, 98, 1])[0]
    began = time.perf_counter(); verifier = KnownFreeVolume(evidence)
    build_s = time.perf_counter() - began
    candidates = []
    for _ in range(1200):
        lower = [rng.randrange(-1, 19) / 50 for _ in range(3)]
        size = [rng.randrange(1, 6) / 50 for _ in range(3)]
        center = [lower[i] + size[i]/2 + (-.2 if i < 2 else .53) for i in range(3)]
        candidates.append(dict(center_container=center, footprint=size[:2], size=size))
    began = time.perf_counter(); actual = [verifier(c) for c in candidates]
    query_s = time.perf_counter() - began
    began = time.perf_counter()
    expected = [volume_state_in_evidence(evidence, c['center_container'], c['footprint'],
        c['center_container'][2]-c['size'][2]/2, c['center_container'][2]+c['size'][2]/2)
        for c in candidates]
    reference_s = time.perf_counter() - began
    assert [v is None for v in actual] == [v is None for v in expected]
    accepted = sum(v is None for v in actual)
    assert 0 < accepted < len(actual)
    target = os.environ.get('P2_EVIDENCE_ROOT')
    if target:
        Path(target, 'volume-parity.json').write_text(json.dumps(dict(seed=20261112,
            samples=len(actual), accepted=accepted, prefix_build_s=build_s,
            query_s=query_s, scalar_reference_s=reference_s,
            prefix_shape=list(verifier.unknown.shape),
            prefix_bytes=verifier.unknown.nbytes+verifier.occupied.nbytes,
            admission_mismatches=0), indent=2)+'\n')
