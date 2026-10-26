"""G4-A C3: isolated read-only query replica ROS node.

Registers the four typed v2 read services (query cells / distance /
surface patch / summary) from the read facade over the replica state
machine -- and NOTHING else. No mapper import, no write path, no OCC
fusion. The mapper process keeps its legacy get_stats / observation /
write services and its runtime peer; in isolated mode it must NOT
register these four service names (single-provider check is a C3
behavior gate).

Liveness semantics (user 2026-10-26, frozen): a fresh HEARTBEAT does
not imply a fresh MAP. The facade compares the latest committed
identity (head) against the local READY identity, checks the export
error surface and the lag bound; a stale heart beat cannot mask a
stalled export. use_current serves the local newest COMPLETE
generation only when control is healthy and no newer head is already
known -- otherwise a named error, never a silently stale map.
"""

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from luggage_perception.cargo_map_query_service import MapQueryService
from luggage_perception.cargo_query_replica import QueryReplica


class CargoMapQueryReplicaNode(Node):
    """Owns the four typed query services in isolated mode."""

    def __init__(self, export_root, poll_period_s=0.025,
                 lag_bound_s=2.0):
        super().__init__('cargo_map_query_replica')
        self._replica = QueryReplica(export_root)
        self._poll_period_s = float(poll_period_s)
        self._lag_bound_s = float(lag_bound_s)
        self._facade = QueryMapReadFacade(self._replica,
                                          lag_bound_s=self._lag_bound_s)
        MapQueryService(self, self._facade, ReentrantCallbackGroup())
        self._poll_timer = self.create_timer(
            self._poll_period_s, self._poll)
        self._heartbeat = self.create_timer(0.5, self._heartbeat_tick)

    def _poll(self):
        try:
            self._replica.poll()
        except Exception:  # noqa: BLE001 - fail closed, keep polling
            self.get_logger().warn('replica poll failed; state=%s'
                                   % self._replica.state, throttle_duration_sec=1.0)

    def _heartbeat_tick(self):
        self._replica.refresh_control_freshness(self._lag_bound_s)


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
            raise QueryError('REPLICA_EXPORT_' + str(head['error']))
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
