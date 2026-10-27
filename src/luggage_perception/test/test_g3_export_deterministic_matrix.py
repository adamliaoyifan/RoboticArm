"""G4-A gen3 M2 deterministic matrix (plan d87c5731 section 5.2):
ledger accounting, failure recovery without new writes, world-reset
segregation, close/offer races, retry stays DELTA. Real store and
export roots; failures injected by first-fail wrappers, never sleeps.
"""

import threading
import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from luggage_perception.cargo_query_export import QueryExportPublisher
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)


def make_store():
    mapper = CargoVolumeMapper((0.4, 0.4, 0.4), (0, 0, 0.2), 0, 0.025)
    return OccupancyMapStore(mapper, epoch='g3-matrix')


def commit(store, index, box_id):
    result = store.apply_update(MapUpdate(
        operation_id='m%d' % index, kind=UpdateKind.ADD_BOX_V1,
        box=((0, 0, 0.05), (0.05, 0.05, 0.05), 0.), box_id=box_id))
    assert result.accepted
    store._snapshot = store._build_snapshot()
    with store.acquire_query_snapshot() as handle:
        return handle._data()


def settled_publisher(root, store):
    publisher = QueryExportPublisher(root, async_mode=True)
    publisher.offer_committed(commit(store, 0, 'seed'))
    assert publisher.wait_pending(10)
    return publisher


class Blocker:
    """Blocks the FIRST publish inside the encoder until released;
    later publishes pass straight through."""

    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()
        self.passed = 0

    def __call__(self, snapshot, dirty, provenance, full=False,
                 heal_hint=None, commit_ns=None):
        self.passed += 1
        if self.passed == 1:
            self.entered.set()
            self.release.wait(10)
        return {'status': 'OK', 'seq': 10 + self.passed}


def test_ledger_exact_and_coalesced_barrier(tmp_path):
    """g1 in flight; g2 queued; g3 coalesces into g2. One publish per
    generation: g1 -> exact_visible(seq1); the merged g2+g3 item ->
    g2 coalesced_into(seq2), g3 exact_visible(seq2)."""
    store = make_store()
    blocker = Blocker()
    with patch.object(QueryExportPublisher, '_publish', blocker), \
            patch.object(QueryExportPublisher, '_head_loop_tick',
                         lambda self: None):
        publisher = QueryExportPublisher(str(tmp_path), async_mode=True)
        data1 = commit(store, 1, 'b1')
        data2 = commit(store, 2, 'b2')
        data3 = commit(store, 3, 'b3')
        publisher.offer_committed(data1, {1})
        assert blocker.entered.wait(5)
        publisher.offer_committed(data2, {2})
        third = publisher.offer_committed(data3, {3})
        assert third['status'] == 'COALESCED'
        # second publish also blocks; release both
        blocker.release.set()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if publisher._in_flight is None and not publisher._queued:
                break
            time.sleep(0.005)
    ledger = publisher.ledger_snapshot()
    statuses = {e['ledger_id']: e['status'] for e in ledger}
    assert statuses[1] == 'exact_visible'
    assert statuses[2] == 'coalesced_into'
    assert statuses[3] == 'exact_visible'
    seqs = {e['ledger_id']: e['published_seq'] for e in ledger}
    assert seqs[2] == seqs[3] and seqs[1] != seqs[2]


class FlakyPublish:
    """Fails the first N real publishes, then delegates to the real
    implementation; records every successful result."""

    def __init__(self, real, fail_count):
        self.real = real
        self.fail_count = fail_count
        self.calls = 0
        self.results = []

    def __call__(self, snapshot, dirty, provenance, full=False,
                 heal_hint=None, commit_ns=None):
        self.calls += 1
        if self.calls <= self.fail_count:
            raise OSError('injected export failure %d' % self.calls)
        result = self.real(snapshot, dirty, provenance, full,
                           heal_hint=heal_hint, commit_ns=commit_ns)
        self.results.append(result)
        return result


def test_encode_failure_recovers_without_new_writes(tmp_path):
    """The final accepted fact must become visible even when no new
    write arrives after the failure: bounded retry, dirty preserved,
    retry stays DELTA, ledger exact_visible, error cleared."""
    store = make_store()
    publisher = settled_publisher(str(tmp_path), store)
    real = QueryExportPublisher._publish.__get__(publisher)
    flaky = FlakyPublish(real, fail_count=1)
    data2 = commit(store, 2, 'b2')
    with patch.object(publisher, '_publish', flaky):
        publisher.offer_committed(data2, {2})
        assert publisher.wait_pending(10), 'retry must recover'
    assert len(flaky.results) == 1
    assert flaky.results[0]['kind'] == 'DELTA'   # no FULL bypass
    ledger = publisher.ledger_snapshot()
    assert [e['status'] for e in ledger][-1] == 'exact_visible'
    assert publisher._encoder_error == ''
    publisher.close()


def test_failure_then_newer_offer_merges(tmp_path):
    """A pending failure folds into the next committed state: ONE
    eventual publish carries both facts; the failed one is
    coalesced_into, not lost and not re-claimed as exact."""
    store = make_store()
    publisher = settled_publisher(str(tmp_path), store)
    real = QueryExportPublisher._publish.__get__(publisher)
    flaky = FlakyPublish(real, fail_count=1)
    data2 = commit(store, 2, 'b2')
    data3 = commit(store, 3, 'b3')
    with patch.object(publisher, '_publish', flaky):
        publisher.offer_committed(data2, {2})
        while publisher._failure is None and publisher._in_flight is None:
            time.sleep(0.005)                    # reach the failed state
        publisher.offer_committed(data3, {3})    # merges into failure
        assert publisher.wait_pending(10)
    assert len(flaky.results) == 1
    ledger = publisher.ledger_snapshot()
    statuses = {e['ledger_id']: e['status'] for e in ledger}
    assert statuses[2] == 'coalesced_into'
    assert statuses[3] == 'exact_visible'
    publisher.close()


def test_ledger_timeout_derived_at_read(tmp_path):
    """Pending facts older than the timeout read back as 'timeout';
    the stored status itself stays pending (read-time derivation)."""
    store = make_store()
    publisher = settled_publisher(str(tmp_path), store)
    entry = {'ledger_id': 1, 'identity': None,
             'commit_monotonic_ns': publisher._clock_ns() - 5_000_000_000,
             'status': 'pending', 'published_seq': None}
    with patch.object(publisher, '_ledger', [entry]):
        view = publisher.ledger_snapshot(timeout_s=2.0)
    assert view[0]['status'] == 'timeout'
    assert entry['status'] == 'pending'          # storage untouched
    publisher.close()


def test_world_reset_never_merges_and_marks_not_delivered(tmp_path):
    blocker = Blocker()
    with patch.object(QueryExportPublisher, '_publish', blocker), \
            patch.object(QueryExportPublisher, '_head_loop_tick',
                         lambda self: None):
        publisher = QueryExportPublisher(str(tmp_path), async_mode=True)
        old_a = SimpleNamespace(identity=('epoch-a', 1, 'x', 'y', 2))
        old_b = SimpleNamespace(identity=('epoch-a', 2, 'x', 'y', 3))
        new_world = SimpleNamespace(identity=('epoch-b', 1, 'x', 'y', 2))
        publisher.offer_committed(old_a, {1})
        assert blocker.entered.wait(5)           # old-a in flight
        publisher.offer_committed(old_b, {2})    # old-b queued (same world)
        result = publisher.offer_committed(new_world, {3})
        assert result['status'] == 'WORLD_RESET'
        blocker.release.set()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if publisher._in_flight is None and not publisher._queued:
                break
            time.sleep(0.005)
    ledger = publisher.ledger_snapshot()
    statuses = {e['ledger_id']: e['status'] for e in ledger}
    assert statuses[1] == 'exact_visible'
    assert statuses[2] == 'superseded_by_reset'  # dropped, NOT delivered
    assert statuses[3] == 'exact_visible'


def test_close_rejects_new_offers_named(tmp_path):
    store = make_store()
    publisher = settled_publisher(str(tmp_path), store)
    assert publisher.close(timeout_s=5) is True
    data = commit(store, 9, 'b9')
    result = publisher.offer_committed(data, {9})
    assert result['status'] == 'EXPORT_CLOSING'
    assert publisher.ledger_snapshot()[-1]['status'] == 'export_failed'
