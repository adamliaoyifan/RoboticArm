"""Explain existing physical-boundary refusals without changing their decisions."""
import json

import pytest

from test_placement_query import observed_map
from luggage_packing.selected_pose_admission import SelectedPoseAdmission
from luggage_packing.placement_volume_query import KnownFreeVolume


@pytest.mark.parametrize('resolution', [.01, .02])
@pytest.mark.parametrize('case,expected', [
    ('flat', None), ('lower_ledge', 'support_tilt_unsupported'),
    ('ramp', 'support_tilt_unsupported'), ('support', 'insufficient_support'),
    ('unknown_support', 'unknown_above_floor'), ('overlap', 'edge_occupied_overlap'),
    ('hull', 'outside_hull'), ('unknown_volume', 'volume_unknown'),
    ('occupied_volume', 'volume_occupied'),
])
def test_explanation_preserves_boundary_decision(resolution, case, expected):
    surface, evidence = observed_map(resolution=resolution)
    n = surface['nx']
    surface['height'] = [[.1]*n for _ in range(n)]
    target = (0., 0., .68)
    if case == 'lower_ledge':
        surface['height'][n//2][n//2] = .06
    elif case == 'ramp':
        surface['height'] = [[.06+.04*i/(n-1)]*n for i in range(n)]
    elif case == 'support':
        surface['height'] = [[0.]*n for _ in range(n)]
    elif case == 'unknown_support':
        surface['state'] = [['unknown']*n for _ in range(n)]
    elif case == 'overlap':
        surface['height'][n//2][n//2] = .12
    elif case == 'hull':
        target = (.19, 0., .68)
    elif case in ('unknown_volume', 'occupied_volume'):
        evidence['states'] = [0 if case=='unknown_volume' else 2]*len(evidence['states'])
    gate = SelectedPoseAdmission(surface, (.12, .10, .1),
                                volume_verifier=KnownFreeVolume(evidence))
    candidate = gate.candidate(target, 0.)
    assert gate(candidate) == expected
    detail = gate.explain(candidate)
    assert detail['reason'] == expected
    assert detail['accepted'] is (expected is None)
    assert detail['center_container'] == list(target)
    assert detail['bottom_z_floor_relative_container_m'] == pytest.approx(.1)
    assert len(json.dumps(detail, allow_nan=False)) < 2048
    if case in ('flat', 'lower_ledge'):
        assert detail['support_ratio'] == pytest.approx(1.)
        assert detail['min_support_ratio'] == .6
        assert detail['support_band_m'] == .05
        assert detail['support_height_span_m'] == pytest.approx(0. if case=='flat' else .04)
        assert detail['support_tilt_ratio'] == pytest.approx(0. if case=='flat' else .04/.12)
    if case=='lower_ledge':
        assert detail['support_tilt_ratio'] > detail['max_support_tilt_ratio']
        assert 'volume_checked' not in detail  # the rejected branch never reaches volume


@pytest.mark.parametrize('yaw', [0., 1.5707963267948966, 3.141592653589793, -1.5707963267948966])
def test_directed_heading_is_retained_in_diagnostic(yaw):
    surface, evidence = observed_map()
    gate = SelectedPoseAdmission(surface, (.12,.1,.1),volume_verifier=KnownFreeVolume(evidence))
    detail = gate.explain(gate.candidate((0.,0.,.58),yaw))
    assert detail['accepted']
    assert detail['heading_container_rad'] == yaw
