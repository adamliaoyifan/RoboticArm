"""Replay input joins reject later maps and same-labelled changed geometry."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location('extract_prior_surface',
    Path(__file__).resolve().parents[1] / 'scripts/placement_prior/extract_replay_surface.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def pair():
    record = dict(map_revision=21, geometry_hash='fixture', height_datum='container_origin',
        frame_id='container_link', floor_z_container=.5, ceiling_z_container=1.5,
        origin_xy_container=[-.05, -.1], resolution_m=.1,
        surface_z_container=[[.7, None]], state=[['occupied', 'unknown']],
        confidence=[['sensor', 'unknown']])
    surface = dict(map_revision=21, geometry_hash='fixture', geometry_descriptor={'fixture': True},
        resolution=.1, nx=1, ny=2, inner_size=[.1, .2, 1.], origin_local=[-.05, -.1],
        state=[['occupied', 'unknown']], confidence=[['sensor', 'none']], height=[[.2, 0.]],
        map_source='measured', acquisition_stamp_ns=123, placed_count=1, cargo_observed=True,
        occupancy_confirmed=True, pending_changes=[], change_event='')
    stats = SimpleNamespace(success=True, map_revision=21, geometry_hash='fixture',
        last_acquisition_stamp_ns=123, map_source='measured', placed_count=1, cargo_observed=True)
    return surface, record, stats


def test_exact_join_ignores_unknown_height_but_preserves_known_cells():
    surface, record, stats = pair()
    module.check_legacy(surface, record, stats, [.1, .2, 1.])


@pytest.mark.parametrize('field,value', [
    ('map_revision', 24), ('geometry_hash', 'other'), ('origin_local', [0., 0.]),
    ('height', [[.3, 0.]]), ('height', [[float('nan'), 0.]]),
    ('confidence', [['geometry', 'none']]), ('acquisition_stamp_ns', 124),
    ('occupancy_confirmed', False), ('pending_changes', ['release']),
    ('change_event', 'box_removed'),
])
def test_reject_mismatched_or_pending_publication(field, value):
    surface, record, stats = pair()
    surface[field] = value
    with pytest.raises(ValueError):
        module.check_legacy(surface, record, stats, [.1, .2, 1.])
