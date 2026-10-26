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
from luggage_perception.cargo_map_read_facade import QueryMapReadFacade
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


