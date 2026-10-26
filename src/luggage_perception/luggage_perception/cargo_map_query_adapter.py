"""Bounded query handle lifecycle. No primary-map reference or write authority.

Store alone publishes completed data. Opening/resolving handles only uses the
short registry lock, never the OCC transaction lock. Typed ROS adaptation is
added on top of this registry, keeping the core independent of rclpy.
"""
from collections import OrderedDict
from types import MappingProxyType
import secrets
import threading
import time

from luggage_perception.cargo_map_query import MapQuerySnapshot, QueryError


class _NullCtx:
    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False


MAX_RETAINED_BYTES = 512*1024*1024
TOKEN_LIFETIME_NS = 5_000_000_000
MAX_SESSIONS = 128


class QuerySnapshotRegistry:
    def __init__(self, clock_ns=time.monotonic_ns):
        self._clock_ns = clock_ns
        self._lock = threading.Lock()
        # [G3 P0 diagnostic] Optional lock wait/hold span collector;
        # zero effect unless set.
        self.span_collector = None
        self._generations = ()
        self._sessions = OrderedDict()
        self._error = 'QUERY_INDEX_UNAVAILABLE'
        self._retained_bytes = 0
        self._requested_bytes = 0

    def publish(self, data, publish_status=None):
        # [G3 P0 diagnostic] lock wait/hold spans when a collector is
        # set; the no-collector path keeps the original with-block.
        collector = self.span_collector
        if collector is None:
            with self._lock:
                return self._publish_body(data, publish_status)
        t0 = self._clock_ns()
        self._lock.acquire()
        wait_ns = self._clock_ns() - t0
        t1 = self._clock_ns()
        try:
            return self._publish_body(data, publish_status)
        finally:
            self._lock.release()
            collector.append({'operation': 'publish', 'wait_ns': wait_ns,
                              'hold_ns': self._clock_ns() - t1})

    def _publish_body(self, data, publish_status=None):
            generations = (data,)
            if self._generations and self._generations[0].identity.epoch == data.identity.epoch:
                previous = tuple(d for d in self._generations if d.identity != data.identity)
                generations += previous[:1]
            seen = set()
            # Owner frontier bitmap and bounded session/container overhead.
            size = (128*1024 + data.owner_evidence_bytes
                    + data.grid.shape[0]*data.grid.shape[1]*data.grid.shape[2])
            for generation in generations:
                # Retained legacy 2D projection + summary/reverse-map metadata.
                size += (generation.grid.shape[0]*generation.grid.shape[1]*384
                         + 256*(len(generation.sensor_blocks)+len(generation.box_blocks)
                                + len(generation.summary_values['label_distribution'])))
                for block in generation.blocks:
                    if id(block) not in seen:
                        size += block.byte_size()
                        seen.add(id(block))
                        layout = block.distance_layout
                        if id(layout) not in seen:
                            size += layout.byte_size()
                            seen.add(id(layout))
                if generation.surface_projection is not None:
                    for tile in generation.surface_projection.tiles:
                        if id(tile) not in seen:
                            size += 1024 + sum(a.nbytes for a in tile.fields.values())
                            seen.add(id(tile))
                if generation.distance_index is not None and id(generation.distance_index) not in seen:
                    size += generation.distance_index.byte_size()
                    seen.add(id(generation.distance_index))
            self._requested_bytes = size
            if size > MAX_RETAINED_BYTES:
                self._error = 'RESOURCE_LIMIT'
                if publish_status is not None:
                    publish_status()
                return False
            self._retained_bytes = size
            valid = {d.identity for d in generations}
            self._sessions = OrderedDict((k, v) for k, v in self._sessions.items() if v[0] in valid)
            self._generations = generations
            self._error = ''
            if publish_status is not None:
                publish_status()
            return True

    def unavailable(self):
        with self._lock:
            self._generations = ()
            self._sessions.clear()
            self._error = 'QUERY_INDEX_UNAVAILABLE'
            self._retained_bytes = 0

    def open(self, expected_identity=None):
        collector = self.span_collector
        if collector is None:
            with self._lock:
                return self._open_body(expected_identity)
        t0 = self._clock_ns()
        self._lock.acquire()
        wait_ns = self._clock_ns() - t0
        t1 = self._clock_ns()
        try:
            return self._open_body(expected_identity)
        finally:
            self._lock.release()
            collector.append({'operation': 'open', 'wait_ns': wait_ns,
                              'hold_ns': self._clock_ns() - t1})

    def _open_body(self, expected_identity=None):
        now = self._clock_ns()
        if True:
            self._expire_locked(now)
            if self._error:
                raise QueryError(self._error)
            data = self._generations[0]
            is_current = isinstance(expected_identity, str) and expected_identity == 'CURRENT'
            if expected_identity is not None and not is_current:
                # Legacy 3-leg identity remains accepted, but a supplied
                # Full voxel identity must match all five versioned legs.
                actual = tuple(data.identity)
                try:
                    want = tuple(expected_identity)
                    valid = (len(want) in (3, 5) and isinstance(want[0], str)
                             and type(want[1]) is int and isinstance(want[2], str))
                    if valid and len(want) == 5:
                        valid = isinstance(want[3], str) and type(want[4]) is int
                    matches = valid and want in (actual, actual[:3])
                except TypeError:
                    matches = False
                if not matches:
                    raise QueryError('SNAPSHOT_IDENTITY_MISMATCH')
            if len(self._sessions) >= MAX_SESSIONS:
                raise QueryError('RESOURCE_LIMIT')
            token = secrets.token_hex(16)
            self._sessions[token] = (data.identity, now+TOKEN_LIFETIME_NS)
            return MapQuerySnapshot(data.identity, data.grid, token, self)

    def resolve(self, token, identity):
        collector = self.span_collector
        if collector is None:
            with self._lock:
                return self._resolve_body(token, identity)
        t0 = self._clock_ns()
        self._lock.acquire()
        wait_ns = self._clock_ns() - t0
        t1 = self._clock_ns()
        try:
            return self._resolve_body(token, identity)
        finally:
            self._lock.release()
            collector.append({'operation': 'resolve', 'wait_ns': wait_ns,
                              'hold_ns': self._clock_ns() - t1})

    def _resolve_body(self, token, identity):
        if True:
            now = self._clock_ns()
            self._expire_locked(now)
            session = self._sessions.get(token)
            if session is None or session[0] != identity:
                raise QueryError('SNAPSHOT_EXPIRED')
            for data in self._generations:
                if data.identity == identity:
                    return data
            raise QueryError('SNAPSHOT_EXPIRED')

    def resume(self, token, expected_identity):
        """Continue a typed-service handle; never extends its expiry."""
        try:
            expected = tuple(expected_identity)
        except TypeError:
            raise QueryError('SNAPSHOT_IDENTITY_MISMATCH') from None
        if len(expected) != 5:
            raise QueryError('SNAPSHOT_IDENTITY_MISMATCH')
        with self._lock:
            self._expire_locked(self._clock_ns())
            session = self._sessions.get(token)
            if session is None:
                raise QueryError('SNAPSHOT_EXPIRED')
            if tuple(session[0]) != expected:
                raise QueryError('SNAPSHOT_IDENTITY_MISMATCH')
            for data in self._generations:
                if data.identity == session[0]:
                    return MapQuerySnapshot(data.identity, data.grid, token, self)
            raise QueryError('SNAPSHOT_EXPIRED')

    def lookup_since(self, current_identity, since_identity):
        """Resolve a retained predecessor without substituting CURRENT."""
        with self._lock:
            try:
                wanted = tuple(since_identity)
            except TypeError:
                raise QueryError('SNAPSHOT_IDENTITY_MISMATCH') from None
            if len(wanted) != 5 or wanted[0] != current_identity.epoch:
                raise QueryError('SNAPSHOT_IDENTITY_MISMATCH')
            current_index = next((i for i, d in enumerate(self._generations)
                                  if d.identity == current_identity), None)
            previous_index = next((i for i, d in enumerate(self._generations)
                                   if tuple(d.identity) == wanted), None)
            if current_index is None or previous_index is None:
                raise QueryError('SNAPSHOT_EXPIRED')
            if previous_index < current_index:
                raise QueryError('SNAPSHOT_IDENTITY_MISMATCH')
            return self._generations[previous_index]

    def close(self, snapshot):
        with self._lock:
            self._sessions.pop(snapshot._token, None)

    def resource_summary(self):
        with self._lock:
            self._expire_locked(self._clock_ns())
            return MappingProxyType(dict(retained_generations=len(self._generations),
                sessions=len(self._sessions), conservative_retained_bytes=self._retained_bytes,
                requested_publication_bytes=self._requested_bytes,
                budget_bytes=MAX_RETAINED_BYTES, index_error=self._error))

    def _expire_locked(self, now):
        for token in tuple(self._sessions):
            if self._sessions[token][1] <= now:
                del self._sessions[token]
