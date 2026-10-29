"""G4-A gen3 read facade over the replica (isolated mode), plan
d87c5731 sections 3-4 / review_8a372dee P1 1-3.

Session ownership REUSES the store's QuerySnapshotRegistry (token
secret, two-generation retention, 5 s expiry, bounded backpressure):
a handle resolves to exactly the generation it was acquired on --
never a mutable latest pointer. Health gating separates four clocks
(owner commit, published, receiver READY, control heartbeat): a fresh
heartbeat can never reset the propagation lag of an outstanding
commit. CURRENT is strict (a known newer committed head rejects);
pinned reads keep the original same-world 2 s bounded-stale window.
"""

import time
from types import MappingProxyType

from luggage_perception.cargo_map_query import QueryError
from luggage_perception.cargo_map_query_adapter import QuerySnapshotRegistry


class QueryMapReadFacade:
    """Read-only facade implementing the MapQueryService surface
    (acquire_query_snapshot / resolve / close) over one replica.

    Named failures (never a silently stale map):
    - no complete generation / no head / stale control -> named reject;
    - persistent UNAVAILABLE / RESYNC_REQUIRED -> named reject;
    - known-newer committed head: CURRENT -> REPLICA_LAGGING (strict,
      no bounded-stale window for CURRENT itself);
    - pinned identity outside the same-world 2 s bound, cross-world,
      or not retained -> SNAPSHOT_EXPIRED / SNAPSHOT_IDENTITY_MISMATCH.
    """

    def __init__(self, replica, lag_bound_s=2.0, control_bound_s=2.0,
                 clock_ns=time.monotonic_ns):
        self._replica = replica
        self._lag_bound_s = float(lag_bound_s)
        # Control loss and propagation lag are SEPARATE clocks (plan
        # section 4): the control bound measures heartbeat freshness
        # directly from head.json; the lag bound measures the
        # outstanding-commit window for pinned bounded-stale reads.
        self._control_bound_s = float(control_bound_s)
        self._clock_ns = clock_ns
        self._registry = QuerySnapshotRegistry(clock_ns=clock_ns)
        self._published_identity = None

    # -- health gate (plan section 4) ----------------------------------

    def _check_control(self, now_ns):
        """Returns (head, committed_identity). Raises a named
        QueryError when any new query must fail closed."""
        replica = self._replica
        # ONE bundle read: a self-consistent (state, snapshot, manifest)
        # triple even while the single poller swaps generations
        applied = replica.applied
        if applied.snapshot is None or applied.manifest is None:
            raise QueryError('QUERY_INDEX_UNAVAILABLE')
        state = str(applied.state or '')
        if state in ('UNAVAILABLE', 'RESYNC_REQUIRED'):
            raise QueryError('REPLICA_' + state)
        head = replica.head_state()
        if callable(head):
            head = head()
        if not isinstance(head, dict) or not head:
            raise QueryError('REPLICA_NO_HEAD')
        committed = head.get('target_identity')
        if committed is not None and (not isinstance(committed, list)
                                      or len(committed) < 3):
            raise QueryError('REPLICA_HEAD_MALFORMED')
        if head.get('error'):
            error = str(head['error'])
            raise QueryError(
                error if error.startswith('REPLICA_')
                else 'REPLICA_' + error)
        try:
            heartbeat_ns = int(head.get('published_monotonic_ns', 0) or 0)
        except (TypeError, ValueError):
            raise QueryError('REPLICA_HEAD_MALFORMED') from None
        # Control freshness is computed HERE from the head heartbeat
        # (same-host monotonic clock), never from a cached replica
        # attribute: nothing in the replica's poll loop maintains
        # control_healthy on its own.
        if not heartbeat_ns or now_ns - heartbeat_ns > int(
                self._control_bound_s * 1e9):
            raise QueryError('REPLICA_CONTROL_STALE')
        return head, committed

    def _sync_generations(self):
        """Feed the newest local COMPLETE generation into the bounded
        registry. Called with the registry's short lock NOT held; the
        registry keeps the previous same-epoch generation (two
        retained) and retires sessions of evicted generations."""
        applied = self._replica.applied
        snapshot = applied.snapshot
        if snapshot is None or applied.manifest is None:
            return
        identity = tuple(snapshot.identity)
        if identity == self._published_identity:
            return
        if self._registry.publish(snapshot):
            self._published_identity = identity

    def read_health(self):
        """Immutable control/identity/propagation view (plan 3.1):
        diagnostic only -- acquire re-checks everything under the
        live state, so this snapshot never authorizes a query."""
        now_ns = self._clock_ns()
        replica = self._replica
        head = replica.head_state()
        if callable(head):
            head = head()
        if not isinstance(head, dict):
            head = {}
        applied = replica.applied
        local = (list(applied.manifest['target_identity'])
                 if applied.manifest is not None else None)
        heartbeat_ns = int(head.get('published_monotonic_ns', 0) or 0)
        commit_ns = int(head.get('commit_monotonic_ns', 0) or 0)
        return MappingProxyType(dict(
            state=str(getattr(replica, 'state', '') or ''),
            control_healthy=bool(
                getattr(replica, 'control_healthy', False)),
            local_identity=local,
            head_identity=(head.get('target_identity')
                           if head else None),
            head_error=str(head.get('error', '') or ''),
            heartbeat_age_s=((now_ns - heartbeat_ns) / 1e9
                             if heartbeat_ns else None),
            commit_age_s=((now_ns - commit_ns) / 1e9
                          if commit_ns else None),
            lag_bound_s=self._lag_bound_s,
        ))

    # -- MapQueryService surface ---------------------------------------

    def acquire_query_snapshot(self, expected_identity=None, *, token=''):
        now_ns = self._clock_ns()
        if token:
            # A resumed session is a new request on a pinned identity:
            # fail-closed control checks apply, but the strict CURRENT
            # rejection does not (the caller pinned deliberately).
            self._check_control(now_ns)
            self._sync_generations()
            return self._registry.resume(token, expected_identity)
        head, committed = self._check_control(now_ns)
        self._sync_generations()
        local = tuple(self._registry_identity())
        is_current = (expected_identity is None
                      or (isinstance(expected_identity, str)
                          and expected_identity == 'CURRENT'))
        if is_current:
            # Heartbeat freshness is necessary but NOT sufficient: a
            # known newer committed head rejects CURRENT outright
            # (strict read contract; the commit time is the lag origin
            # and no heartbeat refresh can extend it).
            if committed is not None and list(committed) != list(local):
                raise QueryError('REPLICA_LAGGING')
            return self._registry.open(expected_identity)
        # Pinned identity: newest, or a retained predecessor within the
        # same-world 2 s bounded-stale window measured from the CURRENT
        # head's commit time (not the heartbeat).
        want = tuple(expected_identity)
        if want == local or want == local[:3]:
            return self._registry.open(expected_identity)
        same_world = bool(local) and want[0] == local[0]
        try:
            commit_ns = int(head.get('commit_monotonic_ns', 0) or 0)
        except (TypeError, ValueError):
            raise QueryError('REPLICA_HEAD_MALFORMED') from None
        within_bound = (same_world and commit_ns
                        and now_ns - commit_ns
                        <= int(self._lag_bound_s * 1e9))
        if not within_bound:
            raise QueryError('SNAPSHOT_EXPIRED')
        return self._registry.open_retained(expected_identity)

    def _registry_identity(self):
        # The registry was just synced to the replica's manifest
        # identity; reuse it (the registry exposes no identity list).
        return tuple(self._replica.applied.manifest['target_identity'])

    # The facade is read-only: the store's write services are not
    # present on this node by construction.
    def resolve(self, token, identity):
        """Return ONLY the generation this session was bound to."""
        return self._registry.resolve(token, identity)

    def close(self, snapshot):
        """Release one session (idempotent; never touches facts)."""
        self._registry.close(snapshot)
