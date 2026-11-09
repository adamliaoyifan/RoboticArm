"""Incremental status/frontier compared with the retained full-grid oracle."""
import json
from pathlib import Path

import numpy as np
import pytest

from luggage_perception.cargo_grid_projection import occupancy_counts
from luggage_perception.cargo_query_indexes import QueryIndexOwner
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.cargo_voxel_delta import VoxelDeltaBuilder
from luggage_perception.occupancy_map_store import OccupancyMapStore, MapUpdate, UpdateKind


def assert_equivalent(m, owner):
    summary = owner.freeze(m, ('test', m.map_revision, '')).as_dict()
    total, inactive, unknown, free, occupied, labels = occupancy_counts(m)
    assert [summary[k] for k in ('total_voxels', 'inactive_count', 'unknown_count',
                                'free_count', 'occupied_count')] == [total, inactive, unknown, free, occupied]
    assert summary['label_distribution'] == {str(k): v for k, v in labels.items()}
    legacy = m._frontier_indices_full()
    assert owner.legacy_frontier_indices(m) == legacy
    assert summary['frontier_count'] == len(legacy)


@pytest.mark.parametrize('seed', range(20))
def test_500_mutations_per_seed(seed):
    rng = np.random.default_rng(seed)
    m = CargoVolumeMapper((.12, .12, .12), (0, 0, .06), 0, .02,
                          max_raycast_points=4)
    owner = QueryIndexOwner()
    owner.initialize(m)
    for transaction in range(500):
        m._voxel_delta = VoxelDeltaBuilder(m)
        action = int(rng.integers(9))
        voxel = rng.integers(0, 6, size=3)
        point = m._local_to_world(*m._voxel_center_local(*voxel))
        try:
            if action < 3:
                m.mark_occupied_voxel(*voxel, label=int(rng.integers(4)), instance_id=int(rng.integers(5)))
            elif action < 5:
                m.mark_free_world(*point)
            elif action == 5:
                m.integrate_points([point], origin=(0, 0, .2), labels=[2], instance_ids=[3])
            elif action == 6:
                m.mark_placed_box((0, 0, .06), (.06, .06, .06))
            elif action == 7:
                m.unmark_placed_box((0, 0, .06), (.06, .06, .06))
            else:
                m.reset(preserve_placed=bool(transaction % 2))
            delta = m._voxel_delta.finalize()
            owner.apply_delta(m, delta)
            assert_equivalent(m, owner)
        except Exception:
            # A deterministic seed+transaction fully regenerates this input;
            # preserve the first failing boundary rather than blindly rerun.
            root = Path('docs/status/evidence/occ_owner_b1/occ_query_index_g2_20261025/s1_failures')
            root.mkdir(parents=True, exist_ok=True)
            (root / ('seed_%02d_txn_%04d.json' % (seed, transaction))).write_text(json.dumps({
                'seed': seed, 'transaction': transaction, 'action': action,
                'replay': 'python3 -m pytest -q src/luggage_perception/test/test_cargo_query_indexes.py -k "500_mutations and [%d]"' % seed,
                'capture_complete': True, 'replay_possible': True}) + '\n')
            raise


def test_single_cell_frontier_work_and_zero_read_scan(monkeypatch):
    m = CargoVolumeMapper((.12, .12, .12), (0, 0, .06), 0, .02)
    s = OccupancyMapStore(m, epoch='test')
    m.mark_occupied_voxel(3, 3, 3, label=2)
    s._snapshot = s._build_snapshot()
    assert s._query_indexes.work['frontier_evaluated'] == 7
    assert s.summary().visited_voxels == 0
    old = s.summary()
    import luggage_perception.cargo_volume_mapper as mapper_module
    monkeypatch.setattr(mapper_module, 'occupancy_counts', lambda *_: pytest.fail('hot read scanned'))
    monkeypatch.setattr(m, '_frontier_indices_full', lambda: pytest.fail('hot read enumerated'))
    for _ in range(100):
        assert m.stats() == old.as_dict()
        assert m.frontier_count() == 6
    exported = old.as_dict()
    exported['label_distribution']['2'] = 900
    assert old.as_dict()['label_distribution'] == {'2': 1}


def test_summary_identity_immutable_and_rejections_do_not_publish():
    m = CargoVolumeMapper((.12, .12, .12), (0, 0, .06), 0, .02)
    s = OccupancyMapStore(m, epoch='test')
    old_bundle = s.status_bundle()
    invalid = MapUpdate(operation_id='', kind=UpdateKind.RESET)
    assert not s.apply_update(invalid).accepted
    assert s.status_bundle() is old_bundle
    s.apply_update(MapUpdate(operation_id='reset', kind=UpdateKind.RESET))
    snapshot, summary = s.status_bundle()
    assert snapshot.identity == summary.identity
    assert old_bundle[0].identity.epoch == 'test'
    assert snapshot.identity.epoch != 'test'
    with pytest.raises(TypeError):
        summary.values['free_count'] = 100


def test_index_failure_preserves_commit_and_reset_recovers(monkeypatch):
    m = CargoVolumeMapper((.12, .12, .12), (0, 0, .06), 0, .02)
    s = OccupancyMapStore(m, epoch='test')
    def unavailable(*_):
        raise RuntimeError('injected index fault')
    monkeypatch.setattr(s._query_indexes, 'apply_delta', unavailable)
    result = s.apply_update(MapUpdate(operation_id='box', kind=UpdateKind.ADD_BOX_V1,
                                     box=((0, 0, .06), (.06, .06, .06), 0)))
    assert result.accepted and result.snapshot.placed_count == 1
    assert m.commit_ledger()
    assert s.status_bundle()[1] is None
    with pytest.raises(RuntimeError, match='QUERY_INDEX_UNAVAILABLE'):
        s.summary()
    assert s.apply_update(MapUpdate(operation_id='reset', kind=UpdateKind.RESET)).accepted
    assert s.summary().identity == s.identity
