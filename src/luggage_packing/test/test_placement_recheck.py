"""Source-resolution proof reuse, without guessed uncertainty parameters."""
import copy
from dataclasses import replace
import math
from types import SimpleNamespace

import pytest

from luggage_packing.placement_recheck import recheck_placement


def fixture():
    surface = dict(geometry_hash='hull', scene_revision='scene', frame_id='container_link',
        resolution_m=.1, origin_xy_container=[-.5, -.5], floor_z_container=.53,
        ceiling_z_container=1.53, state=[['free']*10 for _ in range(10)],
        surface_z_container=[[.53]*10 for _ in range(10)], confidence=[[1.]*10 for _ in range(10)])
    volume = SimpleNamespace(resolution_m=.1, origin_container=(-.5, -.5, .53),
        dimensions_xyz=(10, 10, 10), states=b'\1'*1000, active=(True,)*1000)
    return surface, copy.deepcopy(surface), volume


def check(before, after, volume, **kwargs):
    args = dict(center_container=(0., 0., .63), box_size=(.2, .4, .2), yaw_rad=0.)
    args.update(kwargs)
    return recheck_placement(before, after, volume, **args)


@pytest.mark.parametrize('yaw', [0., math.pi/2, math.pi, -math.pi/2, 3*math.pi/2])
def test_remote_support_change_allowed_at_all_directed_headings(yaw):
    before, after, volume = fixture()
    after['confidence'][0][0] = .7
    assert check(before, after, volume, yaw_rad=yaw) is None


@pytest.mark.parametrize('field,value', [('state', 'unknown'), ('surface_z_container', .54), ('confidence', .99)])
def test_any_local_support_change_refuses(field, value):
    before, after, volume = fixture()
    after[field][4][4] = value
    assert check(before, after, volume) == 'REFRESH_PLACEMENT_SUPPORT_CHANGED'


def test_positive_edge_sliver_checked_but_exact_face_not_crossed():
    before, after, volume = fixture()
    after['confidence'][6][4] = .9
    assert check(before, after, volume) is None
    assert check(before, after, volume, center_container=(.00001, 0., .63)) == 'REFRESH_PLACEMENT_SUPPORT_CHANGED'


@pytest.mark.parametrize('state,active,reason', [(0, True, 'UNKNOWN'), (2, True, 'OCCUPIED'), (1, False, 'UNKNOWN')])
def test_complete_nominal_box_volume_must_remain_observed_free(state, active, reason):
    before, after, volume = fixture()
    cells, mask = bytearray(volume.states), list(volume.active)
    cells[4+10*4], mask[4+10*4] = state, active
    volume.states, volume.active = bytes(cells), tuple(mask)
    assert check(before, after, volume) == 'REFRESH_PLACEMENT_VOLUME_' + reason


@pytest.mark.parametrize('key,value', [('resolution_m', .02), ('origin_xy_container', [0., 0.]),
    ('floor_z_container', 0.), ('ceiling_z_container', 2.), ('geometry_hash', 'new'), ('scene_revision', 'new')])
def test_changed_grid_or_hull_not_reinterpreted(key, value):
    before, after, volume = fixture()
    after[key] = value
    assert check(before, after, volume) == 'REFRESH_PLACEMENT_GRID_CHANGED'


def test_volume_grid_mismatch_or_non_quarter_turn_refuses():
    before, after, volume = fixture()
    assert check(before, after, None) == 'REFRESH_PLACEMENT_VOLUME_GRID_MISMATCH'
    volume.origin_container = (-.5, -.5, 0.)
    assert check(before, after, volume) == 'REFRESH_PLACEMENT_VOLUME_GRID_MISMATCH'
    assert check(before, after, volume, yaw_rad=.3) == 'REFRESH_PLACEMENT_GEOMETRY_INVALID'


def test_quarter_turn_swaps_non_square_support_footprint():
    before, after, volume = fixture()
    after['confidence'][3][4] = .9
    assert check(before, after, volume, yaw_rad=0.) is None
    assert check(before, after, volume, yaw_rad=math.pi/2) == 'REFRESH_PLACEMENT_SUPPORT_CHANGED'
