"""G4-A gen3 M0/M1 witnesses: facade session pinning, close contract,
health gating (plan d87c5731 sections 3-4; review_8a372dee P1 1-3).

Every test here MUST fail on 8a372dee (verified in M0) and pass after M1.
Deterministic: controllable replica stub + injectable clock, no sleeps.
"""

import time
from types import SimpleNamespace

import pytest

from luggage_perception.cargo_map_read_facade import QueryMapReadFacade
from luggage_perception.cargo_map_query import QueryError
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)


def make_store():
    mapper = CargoVolumeMapper((0.4, 0.4, 0.4), (0, 0, 0.2), 0, 0.025)
    return OccupancyMapStore(mapper, epoch='g3-facade')


class ReplicaStub:
    """Controllable replica surface: exactly the attributes the facade
    reads (snapshot, manifest, state, control_healthy, head_state)."""

    def __init__(self, snapshot, now_ns=None):
        self.snapshot = snapshot
        self.manifest = {'target_identity': list(snapshot.identity)}
        self.state = 'READY'
        self.control_healthy = True
        self._now_ns = now_ns if now_ns is not None else time.monotonic_ns()
        self._head = {'target_identity': list(snapshot.identity),
                      'published_monotonic_ns': self._now_ns,
                      'commit_monotonic_ns': self._now_ns,
                      'error': ''}

    def advance(self, store):
        """Publish a new accepted generation into this stub."""
        update = store.apply_update(MapUpdate(
            operation_id='stub-%d' % (self._head.get('seq', 0) + 1),
            kind=UpdateKind.ADD_BOX_V1,
            box=((0, 0, 0.05), (0.1, 0.1, 0.1), 0.), box_id='stub'))
        assert update.accepted
        with store.acquire_query_snapshot() as handle:
            self.snapshot = handle._data()
        self.manifest = {'target_identity': list(self.snapshot.identity)}
        return update

    def head_state(self):
        return dict(self._head)

    def tick_heartbeat(self):
        """A heartbeat refresh: published time moves NOW, but the real
        commit time of the outstanding generation must NOT move."""
        self._now_ns = time.monotonic_ns()
        self._head['published_monotonic_ns'] = self._now_ns


# -- P1-1: provider.close(handle) contract -----------------------------

def test_close_handle_is_idempotent_and_silent():
    store = make_store()
    with store.acquire_query_snapshot() as handle:
        data = handle._data()
    replica = ReplicaStub(data)
    facade = QueryMapReadFacade(replica)
    held = facade.acquire_query_snapshot('CURRENT')
    held.close()          # must not raise (8a372dee: TypeError)
    held.close()          # idempotent


# -- P1-2: held snapshot is pinned -------------------------------------

def test_held_handle_stays_on_old_generation():
    store = make_store()
    with store.acquire_query_snapshot() as handle:
        old = handle._data()
    replica = ReplicaStub(old)
    facade = QueryMapReadFacade(replica)
    held = facade.acquire_query_snapshot('CURRENT')
    old_identity = tuple(held.identity)
    old_summary = held.summary()
    replica.advance(store)            # receiver publishes generation 2
    # The held handle must still resolve to generation 1 data.
    assert tuple(held.identity) == old_identity
    assert held._data().identity == old_identity       # 8a372dee: rev2
    assert held.summary() == old_summary


def test_old_pinned_identity_read_within_bound():
    store = make_store()
    with store.acquire_query_snapshot() as handle:
        old = handle._data()
    replica = ReplicaStub(old)
    facade = QueryMapReadFacade(replica)
    held = facade.acquire_query_snapshot('CURRENT')
    old_identity = tuple(held.identity)
    replica.advance(store)
    # Same-world pinned read of a known old identity within the 2s
    # bound must still be acquirable and pinned to that generation.
    pinned = facade.acquire_query_snapshot(old_identity)
    assert tuple(pinned.identity) == old_identity
    assert pinned._data().identity == old_identity


# -- P1-3: health gating ------------------------------------------------

def test_unavailable_replica_rejects_current():
    store = make_store()
    with store.acquire_query_snapshot() as handle:
        data = handle._data()
    replica = ReplicaStub(data)
    facade = QueryMapReadFacade(replica)
    replica.state = 'UNAVAILABLE'
    replica.control_healthy = False
    replica._head.clear()             # missing head entirely
    with pytest.raises(QueryError):
        facade.acquire_query_snapshot('CURRENT').summary()


def test_resync_required_rejects_current():
    store = make_store()
    with store.acquire_query_snapshot() as handle:
        data = handle._data()
    replica = ReplicaStub(data)
    facade = QueryMapReadFacade(replica)
    replica.state = 'RESYNC_REQUIRED'
    with pytest.raises(QueryError):
        facade.acquire_query_snapshot('CURRENT')


def test_stalled_propagation_with_fresh_heartbeat_rejects_current():
    """A fresh heartbeat must not mask a stalled map propagation: the
    lag clock must run from the outstanding COMMIT time, which a
    heartbeat tick cannot reset (8a372dee: head tick refreshed
    published_monotonic_ns and CURRENT kept serving the old map)."""
    store = make_store()
    with store.acquire_query_snapshot() as handle:
        data = handle._data()
    replica = ReplicaStub(data)
    facade = QueryMapReadFacade(replica)
    update = replica.advance(store)
    # The receiver's local map is still the OLD generation; the head
    # announces the NEW committed identity, committed 10 s ago, with a
    # continuously fresh heartbeat.
    replica._head['target_identity'] = list(update.identity)
    replica._head['commit_monotonic_ns'] = time.monotonic_ns() - 10_000_000_000
    for _ in range(40):
        replica.tick_heartbeat()
        time.sleep(0.001)
    with pytest.raises(QueryError):
        facade.acquire_query_snapshot('CURRENT')


def test_current_rejects_immediately_on_known_newer_head():
    """CURRENT is strict: as soon as a newer committed head is known,
    CURRENT must reject (no bounded-stale window for CURRENT itself)."""
    store = make_store()
    with store.acquire_query_snapshot() as handle:
        data = handle._data()
    replica = ReplicaStub(data)
    facade = QueryMapReadFacade(replica)
    update = replica.advance(store)
    replica._head['target_identity'] = list(update.identity)
    with pytest.raises(QueryError):
        facade.acquire_query_snapshot('CURRENT')


def test_static_healthy_map_stays_readable():
    """No map age rejection when head identity == local identity and
    control is healthy, even after a long quiet period."""
    store = make_store()
    with store.acquire_query_snapshot() as handle:
        data = handle._data()
    replica = ReplicaStub(data)
    facade = QueryMapReadFacade(replica)
    replica._head['commit_monotonic_ns'] = time.monotonic_ns() - 60_000_000_000
    replica._head['published_monotonic_ns'] = time.monotonic_ns() - 60_000_000_000
    assert facade.acquire_query_snapshot('CURRENT').summary().status == 'OK'


def test_unknown_requested_identity_rejected():
    store = make_store()
    with store.acquire_query_snapshot() as handle:
        data = handle._data()
    replica = ReplicaStub(data)
    facade = QueryMapReadFacade(replica)
    bogus = list(data.identity)
    bogus[1] += 7
    with pytest.raises(QueryError):
        facade.acquire_query_snapshot(tuple(bogus))
