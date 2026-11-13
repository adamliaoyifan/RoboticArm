"""Bounded handoff of already-published suction acquisitions, without fusion.

Each entry retains its original domain and optional immutable quality samples.
Count, bytes and lifetime bound transport resources; consumers still own physical
freshness and identity admission. Reading never renews the entry's lifetime.
"""
from collections import OrderedDict
from dataclasses import asdict, dataclass
import json
import math
import threading
import time

from luggage_perception.observed_suction_domain import ObservedSuctionDomain


@dataclass(frozen=True)
class _Acquisition:
    domain: ObservedSuctionDomain
    context: object
    expires_at: float
    size_bytes: int


class SuctionAcquisitionHistory:
    def __init__(self, *, max_records=16, max_bytes=64*1024*1024,
                 lifetime_s=60., clock=time.monotonic):
        if (type(max_records) is not int or max_records < 1
                or type(max_bytes) is not int or max_bytes < 1
                or not math.isfinite(lifetime_s) or lifetime_s <= 0):
            raise ValueError('POSITIVE_SUCTION_HANDOFF_RESOURCE_LIMITS_REQUIRED')
        self.max_records, self.max_bytes = max_records, max_bytes
        self.lifetime_s, self.clock = float(lifetime_s), clock
        self._rows = OrderedDict()
        self._identity, self._bytes = None, 0
        self._lock = threading.RLock()
        self._last_clear = None

    def clear(self, reason='EXPLICIT_REVOCATION'):
        with self._lock:
            self._last_clear = dict(reason=reason,monotonic=self.clock(),
                discarded=[self.acquisition_key(row.domain) for row in self._rows.values()])
            self._rows.clear()
            self._identity, self._bytes = None, 0

    @staticmethod
    def acquisition_key(domain):
        return (domain.box_id, domain.generation, domain.stamp_ns,
                domain.frame_id, domain.contact_model_hash)

    def _expire(self, now):
        for key, row in tuple(self._rows.items()):
            if now >= row.expires_at:
                self._bytes -= self._rows.pop(key).size_bytes

    def remember(self, domain, *, context=None):
        if type(domain) is not ObservedSuctionDomain:
            raise ValueError('TYPED_OBSERVED_SUCTION_ACQUISITION_REQUIRED')
        digest = domain.digest()
        if context is not None and context.domain_sha256 != digest:
            raise ValueError('SUCTION_HANDOFF_SAMPLE_IDENTITY_MISMATCH')
        metadata = asdict(domain)
        metadata.pop('support')
        size = len(domain.support) + len(json.dumps(metadata).encode()) + 1024
        if context is not None:
            size += sum(getattr(value, 'nbytes', 0)
                        for value in vars(context._height_map).values())
            size += len(json.dumps(context._model.as_dict()).encode())
        identity = (domain.box_id, domain.instance_id, domain.generation,
                    domain.contact_model_hash)
        with self._lock:
            now = self.clock()
            self._expire(now)
            if identity != self._identity:
                self.clear('INSTANCE_OR_MODEL_CHANGED')
                self._identity = identity
            for old_key, row in self._rows.items():
                if self.acquisition_key(row.domain) == self.acquisition_key(domain):
                    if old_key != digest or (context is not None and row.context is not None
                            and context.samples_sha256 != row.context.samples_sha256):
                        self.clear('CONFLICTING_ACQUISITION')
                        raise ValueError('CONFLICTING_SUCTION_ACQUISITION')
                    return  # Duplicate publication does not renew retention.
            if size > self.max_bytes:
                return  # No retained handoff; never substitute another frame.
            while self._rows and (len(self._rows) >= self.max_records
                                  or self._bytes + size > self.max_bytes):
                self._bytes -= self._rows.popitem(last=False)[1].size_bytes
            self._rows[digest] = _Acquisition(domain, context, now+self.lifetime_s, size)
            self._bytes += size

    def domain_for_acquisition(self, key):
        with self._lock:
            self._expire(self.clock())
            return next((row.domain for row in self._rows.values()
                         if self.acquisition_key(row.domain) == key), None)

    def context_for_digest(self, digest):
        with self._lock:
            self._expire(self.clock())
            row = self._rows.get(digest)
            return row.context if row is not None else None

    def diagnostics(self):
        """Metadata only, for an unavailable-handoff boundary dump."""
        with self._lock:
            return dict(records=[dict(key=self.acquisition_key(row.domain),digest=digest)
                for digest,row in self._rows.items()],bytes=self._bytes,last_clear=self._last_clear)
