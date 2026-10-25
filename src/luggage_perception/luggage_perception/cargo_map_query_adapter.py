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


MAX_RETAINED_BYTES = 512*1024*1024
TOKEN_LIFETIME_NS = 5_000_000_000
MAX_SESSIONS = 128


class QuerySnapshotRegistry:
    def __init__(self, clock_ns=time.monotonic_ns):
        self._clock_ns = clock_ns
        self._lock = threading.Lock()
        self._generations = ()
        self._sessions = OrderedDict()
        self._error = 'QUERY_INDEX_UNAVAILABLE'
        self._retained_bytes = 0
        self._requested_bytes = 0

    def publish(self, data, publish_status=None):
        with self._lock:
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
        now = self._clock_ns()
        with self._lock:
            self._expire_locked(now)
            if self._error:
                raise QueryError(self._error)
            data = self._generations[0]
            is_current = isinstance(expected_identity, str) and expected_identity == 'CURRENT'
            if expected_identity is not None and not is_current:
                # Legacy 3-leg identity remains accepted, but a supplied
                # full voxel identity must match all five schema-1 legs.
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
        with self._lock:
            now = self._clock_ns()
            self._expire_locked(now)
            session = self._sessions.get(token)
            if session is None or session[0] != identity:
                raise QueryError('SNAPSHOT_EXPIRED')
            for data in self._generations:
                if data.identity == identity:
                    return data
            raise QueryError('SNAPSHOT_EXPIRED')

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
