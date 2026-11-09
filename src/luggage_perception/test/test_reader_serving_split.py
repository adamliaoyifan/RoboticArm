"""Reader/serving split (B representative finding, 2026-10-29).

Preconditions verified here before any concurrency was added:
- single replica updater: enforced, not assumed (POLL_SINGLE_WRITER);
- atomic full-state publish: one immutable bundle per swap -- a reader
  holding `replica.applied` sees a self-consistent triple;
- fixed query handles + recycle protection: unchanged registry
  semantics (covered by the session-contract tests; here we assert the
  facade serves correct pinned data WHILE the poller swaps).
"""

import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))),
    'src', 'luggage_perception'))

import numpy as np

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)
from luggage_perception.cargo_map_query_mode import wire_store_export
from luggage_perception.cargo_query_replica import (
    QueryReplica, ReplicaState)
from luggage_perception.cargo_map_read_facade import QueryMapReadFacade


class PairReplica(QueryReplica):
    """Test double: only the pair consistency of applied bundles."""

    def __init__(self):
        QueryReplica.__init__(self, root='/nonexistent')


def test_applied_bundle_pair_consistency_under_concurrent_swaps():
    replica = PairReplica()
    stop = threading.Event()
    errors = []

    def swapper():
        index = 0
        while not stop.is_set():
            index += 1
            # the REAL apply path swaps the whole bundle in ONE
            # assignment; a reader that takes .applied once must
            # never see a mixed pair
            replica._swap(snapshot='snapshot-%d' % index,
                          manifest={'tag': index, 'target_identity':
                                    ['e', index, 'd%d' % index]},
                          state=ReplicaState.READY)

    def reader():
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            applied = replica.applied
            if applied.snapshot is None:
                continue
            tag = applied.manifest['tag']
            if applied.snapshot != 'snapshot-%d' % tag:
                errors.append((applied.snapshot, tag))
                return

    threads = [threading.Thread(target=swapper),
               threading.Thread(target=reader),
               threading.Thread(target=reader)]
    for thread in threads:
        thread.start()
    time.sleep(1.2)
    stop.set()
    for thread in threads:
        thread.join(5)
    assert not errors, errors[:3]


def test_single_writer_guard_enforced_not_assumed():
    replica = PairReplica()
    spans = []
    replica.start_polling(interval_s=0.02, on_poll=lambda *a: spans.append(a))
    try:
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and not spans:
            time.sleep(0.01)
        assert spans, 'poller never ran'
        try:
            replica.poll()
            raise AssertionError('external poll accepted while the '
                                 'poller owns polling')
        except RuntimeError as error:
            assert 'POLL_SINGLE_WRITER' in str(error)
    finally:
        assert replica.stop_polling(5)
    # after stop, external polling is legal again
    replica.poll()


def _real_pair(tmp, boxes=2):
    root = os.path.join(tmp, 'root')
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0, 0.1)
    store = OccupancyMapStore(mapper, epoch='split')
    hook = wire_store_export(store, root)
    for index in range(boxes):
        store.apply_update(MapUpdate(
            operation_id='b%d' % index, kind=UpdateKind.ADD_BOX_V1,
            box=((0.0, 0.1 * index, mapper._hull.floor_z + .11 + .2),
                 (.2, .2, .15), 0.), box_id='box-%d' % index))
    store._snapshot = store._build_snapshot()
    hook.wait_pending(30)
    return store, hook, mapper, root


def test_facade_serves_pinned_correctly_while_poller_applies(tmp_path):
    """End-to-end: the poller thread applies new generations while the
    MAIN thread serves queries; every pinned read resolves to the data
    of the identity it was acquired on (fixed handles + recycle
    protection under the split)."""
    import tempfile
    import shutil
    tmp = tempfile.mkdtemp(dir=str(tmp_path))
    try:
        store, hook, mapper, root = _real_pair(tmp)
        replica = QueryReplica(root)
        facade = QueryMapReadFacade(replica)
        replica.start_polling(interval_s=0.01)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            applied = replica.applied
            if (str(applied.state) == str(ReplicaState.READY)
                    and applied.snapshot is not None):
                break
            time.sleep(0.01)
        assert replica.state == ReplicaState.READY, replica.last_error

        # pinned read survives concurrent swaps with exact data
        handle = facade.acquire_query_snapshot('CURRENT')
        pinned_identity = tuple(map(str, handle.identity))
        pinned_values = handle.summary().values
        rng = np.random.default_rng(5)
        for index in range(4):
            store.apply_update(MapUpdate(
                operation_id='v%d' % index,
                kind=UpdateKind.INTEGRATE_VIEW,
                expected_snapshot=store.identity,
                geometry_hash=mapper.geometry_hash,
                points=rng.uniform(-0.3, 0.3, (2000, 3)),
                origin=(0., 0., 1.4),
                minimum_z=mapper._hull.floor_z + .05,
                stamp_key=(index + 1, 0),
                acquisition_stamp_ns=index + 1, settled=True,
                schema_version=1))
            store._snapshot = store._build_snapshot()
            hook.wait_pending(30)
            time.sleep(0.15)          # let the poller apply
            # the PINNED handle resolves its own generation data or a
            # NAMED expiry once retention (two generations) evicts it
            # -- never another generation's data
            try:
                again = facade.resolve(handle._token, pinned_identity)
                assert again is not None
                assert again.summary().values == pinned_values
            except Exception as error:
                assert 'SNAPSHOT_EXPIRED' in repr(error) or \
                    'SNAPSHOT_IDENTITY_MISMATCH' in repr(error), error
            # CURRENT tracks the applied head, never a torn pair
            current = facade.acquire_query_snapshot('CURRENT')
            current_identity = tuple(map(str, current.identity))
            current.close()
            assert current_identity != pinned_identity or index < 3
        handle.close()
        replica.stop_polling(5)
        hook.close()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_default_single_threaded_behavior_unchanged(tmp_path):
    """Without start_polling the replica is exactly the old object:
    external poll() drives everything, bundles behave like fields."""
    import tempfile
    import shutil
    tmp = tempfile.mkdtemp(dir=str(tmp_path))
    try:
        store, hook, mapper, root = _real_pair(tmp)
        replica = QueryReplica(root)
        assert replica.poll() in (ReplicaState.SYNCING,
                                  ReplicaState.READY,
                                  ReplicaState.RESYNC_REQUIRED)
        replica.state = ReplicaState.UNAVAILABLE        # setter path
        assert replica.state == ReplicaState.UNAVAILABLE
        assert replica.applied.snapshot is replica.snapshot
        hook.close()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
