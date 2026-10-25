"""Independent native-grid lookup, seam, mutation and ownership checks."""
import numpy as np
import pytest

from luggage_perception.cargo_canonical_evidence import CanonicalEvidenceBlocks
from luggage_perception.cargo_fact_digest import CanonicalFactDigest
from luggage_perception.cargo_voxel_delta import EVIDENCE_DTYPE, capture_evidence
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.cargo_map_query import QueryError
from luggage_perception.occupancy_map_store import MapUpdate, OccupancyMapStore, UpdateKind


def initialize(shape, seed):
    rng = np.random.default_rng(seed)
    expected = np.empty(np.prod(shape), dtype=EVIDENCE_DTYPE)
    expected.view('u1')[:] = rng.integers(0, 256, expected.nbytes, dtype='u1')
    storage = CanonicalEvidenceBlocks(shape)
    nx, ny, nz = shape
    block_indices = []
    for z in range(0, nz, 16):
        for y in range(0, ny, 16):
            for x in range(0, nx, 16):
                indices = np.asarray([i + nx*(j + ny*k)
                    for k in range(z, min(z+16, nz))
                    for j in range(y, min(y+16, ny))
                    for i in range(x, min(x+16, nx))])
                storage.initialize_block(len(block_indices), expected[indices])
                block_indices.append(indices)
    return rng, expected, storage, block_indices


@pytest.mark.parametrize('seed', range(20))
def test_native_grid_prior_facts_and_roots_across_partial_blocks(seed):
    rng, expected, storage, blocks = initialize((17, 18, 19), seed)
    raw = np.dtype(('V', EVIDENCE_DTYPE.itemsize))
    # This global grid oracle knows no block offsets or storage layout.
    for transaction in range(50):
        reader = storage.bind_prior_reader()
        indices = np.unique(np.r_[rng.integers(0, len(expected), 100), 0, 15, 16, len(expected)-1])
        prior = reader(indices)
        assert prior.tobytes() == expected[indices].tobytes()
        prior.view('u1')[:] = 0
        assert reader(indices).tobytes() == expected[indices].tobytes()
        changed = expected[indices].copy()
        changed.view('u1')[:] = rng.integers(0, 256, changed.nbytes, dtype='u1')
        ids, offsets = storage.patch(indices, changed)
        expected.view(raw)[indices] = changed.view(raw)
        for bid in np.unique(ids):
            storage.rehash(int(bid), offsets[ids == bid])
        with pytest.raises(RuntimeError, match='STALE_PRIOR_EVIDENCE'):
            reader(indices)
        current = storage.bind_prior_reader()
        assert current(indices).tobytes() == expected[indices].tobytes()
        if transaction % 10 == 0:
            for bid, keys in enumerate(blocks):
                actual = storage.block_bytes(bid)
                assert actual.readonly and bytes(actual) == expected[keys].tobytes()
                rebuilt = CanonicalFactDigest.build(expected[keys].tobytes())
                assert storage.rehash(bid, np.empty(0, dtype=int)) == rebuilt
    assert storage.byte_size() > len(expected)*EVIDENCE_DTYPE.itemsize
    assert storage.byte_size() < len(blocks)*16**3*EVIDENCE_DTYPE.itemsize


def test_invalid_record_counts_and_indices_cannot_broadcast_or_wrap():
    _, _, storage, _ = initialize((2, 3, 4), 0)
    reader = storage.bind_prior_reader()
    with pytest.raises(IndexError):
        reader(np.asarray([-1]))
    with pytest.raises(IndexError):
        reader(np.asarray([24]))
    with pytest.raises(ValueError):
        storage.patch(np.asarray([0, 1]), np.zeros(1, dtype=EVIDENCE_DTYPE))


def test_deferred_first_old_capture_matches_primary_before_repeated_mutations():
    mapper = CargoVolumeMapper((.2, .2, .2), (0, 0, .1), 0, .02)
    store = OccupancyMapStore(mapper, epoch='prior')
    prior = capture_evidence(mapper, np.arange(1000))
    mapper.mark_occupied_voxel(1, 2, 3, label=4, instance_id=7)
    mapper.mark_occupied_voxel(1, 2, 3, label=5, instance_id=8)
    mapper.mark_free_world(*mapper._local_to_world(*mapper._voxel_center_local(7, 8, 9)))
    delta = mapper._voxel_delta.finalize()
    assert delta.before.tobytes() == prior[delta.indices].tobytes()
    assert delta.after.tobytes() == capture_evidence(mapper, delta.indices).tobytes()
    store._snapshot = store._build_snapshot()
    assert store.summary().visited_voxels == 0


def test_binding_failure_keeps_physical_fact_and_reset_recovers(monkeypatch):
    mapper = CargoVolumeMapper((.2, .2, .2), (0, 0, .1), 0, .02)
    store = OccupancyMapStore(mapper, epoch='prior-fault')
    def broken():
        raise RuntimeError('injected binding failure')
    monkeypatch.setattr(store._query_indexes, 'bind_prior_evidence', broken)
    result = store.apply_update(MapUpdate(operation_id='release', kind=UpdateKind.ADD_BOX_V1,
        box=((0., 0., .05), (.04, .04, .04), 0.)))
    assert result.accepted and len(mapper._placed_boxes) == 1
    with pytest.raises(QueryError, match='QUERY_INDEX_UNAVAILABLE'):
        store.acquire_query_snapshot()
    assert store.apply_update(MapUpdate(operation_id='reset', kind=UpdateKind.RESET)).accepted
    with store.acquire_query_snapshot() as snapshot:
        assert snapshot.summary().status == 'OK'
