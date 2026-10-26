"""G4-A C3: ROS-free read facade over the replica (isolated mode).

Named failures, never pseudo-healthy stale maps (user 2026-10-26):
heartbeat freshness != map freshness -- the facade compares the latest
committed head identity against the local READY identity, checks the
export error surface and the lag bound; a fresh heartbeat cannot mask
a stalled export. use_current serves the local newest COMPLETE
generation only when control is healthy and no newer head is already
known past the bound.
"""

import time

from luggage_perception.cargo_map_query import MapQuerySnapshot, QueryError


class QueryMapReadFacade:
    """Read-only facade over the replica implementing the store-facing
    query API surface MapQueryService needs (acquire_query_snapshot).

    Named failures (never a silently stale map):
    - no complete generation yet / receiver resyncing -> QUERY_INDEX_UNAVAILABLE;
    - known-newer committed head (lagging) -> REPLICA_LAGGING;
    - expected identity not held -> SNAPSHOT_IDENTITY_MISMATCH
      (existing semantics; never waits, never falls back to older).
    """

    def __init__(self, replica, lag_bound_s=2.0):
        self._replica = replica
        self._lag_bound_s = float(lag_bound_s)

    # MapQueryService calls acquire_query_snapshot(expected, token=...)
    def acquire_query_snapshot(self, expected_identity=None, *, token=''):
        import time

        from luggage_perception.cargo_map_query import MapQuerySnapshot, QueryError
        replica = self._replica
        if replica.snapshot is None or replica.manifest is None:
            raise QueryError('QUERY_INDEX_UNAVAILABLE')
        head = replica.head_state()
        if head.get('error'):
            error = str(head['error'])
            raise QueryError(
                error if error.startswith('REPLICA_')
                else 'REPLICA_' + error)
        # Heartbeat freshness is necessary but NOT sufficient: compare
        # the latest committed identity against the local READY one.
        committed = head.get('target_identity')
        local = list(replica.manifest['target_identity'])
        if committed is not None and list(committed) != local:
            age_s = (time.monotonic_ns()
                     - int(head.get('published_monotonic_ns', 0))) / 1e9
            if age_s > self._lag_bound_s:
                raise QueryError('REPLICA_LAGGING')
            # within the propagation window: serve the local complete
            # generation, echoing its exact identity (bounded-stale
            # read, protocol section 5) -- never an older generation.
        if expected_identity is not None and expected_identity != 'CURRENT':
            want = tuple(expected_identity)
            actual = tuple(local)
            if want != actual and want != actual[:3]:
                raise QueryError('SNAPSHOT_IDENTITY_MISMATCH')
        # Token sessions are receiver-local (cursor secret is the
        # replica's own); single-shot queries use the default token.
        return MapQuerySnapshot(replica.snapshot.identity,
                                replica.snapshot.grid, '', self)

    # The facade is read-only: the store's write services are not
    # present on this node by construction; these methods exist only
    # so MapQueryService's release/close path stays identical.
    def resolve(self, token, identity):
        return self._replica.snapshot

    def close(self):
        return None
