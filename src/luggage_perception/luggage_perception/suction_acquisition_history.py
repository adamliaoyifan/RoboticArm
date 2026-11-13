"""Bounded handoff of already-published suction acquisitions, without fusion.

Each entry retains its original domain and optional immutable quality samples.
Count, bytes and lifetime bound transport resources; consumers still own physical
freshness and identity admission. Reading never renews the entry's lifetime.
"""
from collections import OrderedDict
from dataclasses import asdict, dataclass, replace
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
    view_sha256: str = ''


class SuctionAcquisitionHistory:
    def __init__(self, *, max_records=16, max_bytes=64*1024*1024,
                 lifetime_s=60., clock=time.monotonic, max_view_bytes=16*1024*1024):
        if (type(max_records) is not int or max_records < 1
                or type(max_bytes) is not int or max_bytes < 1
                or not math.isfinite(lifetime_s) or lifetime_s <= 0
                or type(max_view_bytes) is not int or max_view_bytes<0):
            raise ValueError('POSITIVE_SUCTION_HANDOFF_RESOURCE_LIMITS_REQUIRED')
        self.max_records, self.max_bytes = max_records, max_bytes
        self.lifetime_s, self.clock = float(lifetime_s), clock
        self.max_view_bytes,self._view_bytes=max_view_bytes,0
        self._rows = OrderedDict()
        self._identity, self._bytes = None, 0
        self._lock = threading.RLock()
        self._last_clear = None
        self._last_view_conflict = None

    def clear(self, reason='EXPLICIT_REVOCATION'):
        with self._lock:
            self._last_clear = dict(reason=reason,monotonic=self.clock(),
                discarded=[self.acquisition_key(row.domain) for row in self._rows.values()])
            self._rows.clear()
            self._identity, self._bytes = None, 0
            self._view_bytes=0

    @staticmethod
    def acquisition_key(domain):
        return (domain.box_id, domain.generation, domain.stamp_ns,
                domain.frame_id, domain.contact_model_hash)

    def _expire(self, now):
        for key, row in tuple(self._rows.items()):
            if now >= row.expires_at:
                self._remove(key)

    @staticmethod
    def _view(context):
        return getattr(context,'pickup_view',None)

    def _remove(self, key):
        row=self._rows.pop(key)
        self._bytes-=row.size_bytes
        view=self._view(row.context)
        if view is not None:self._view_bytes-=view.size_bytes

    def _admit_optional_view(self, context):
        """Camera pressure drops camera inputs, never ordinary quality rows."""
        view=self._view(context)
        if view is None:return context
        if view.size_bytes>self.max_view_bytes:
            return context.without_pickup_view('PICKUP_VIEW_RESOURCE_LIMIT')
        for key,row in tuple(self._rows.items()):
            if self._view_bytes+view.size_bytes<=self.max_view_bytes:break
            previous=self._view(row.context)
            if previous is not None:
                self._view_bytes-=previous.size_bytes
                self._rows[key]=replace(row,context=row.context.without_pickup_view('PICKUP_VIEW_RESOURCE_EVICTED'))
        self._view_bytes+=view.size_bytes
        return context

    def remember(self, domain, *, context=None):
        if type(domain) is not ObservedSuctionDomain:
            raise ValueError('TYPED_OBSERVED_SUCTION_ACQUISITION_REQUIRED')
        digest = domain.digest()
        view=self._view(context)
        view_sha256=view.digest() if view is not None else ''
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
                    if view_sha256 and row.view_sha256 and view_sha256!=row.view_sha256:
                        previous=self._view(row.context)
                        if previous is not None:self._view_bytes-=previous.size_bytes
                        # Optional camera disagreement cannot invalidate matching
                        # quality samples. Refuse both camera versions instead.
                        self._rows[old_key]=replace(row,context=row.context.without_pickup_view(
                            'PICKUP_VIEW_CONFLICTING_ACQUISITION'))
                        self._last_view_conflict=dict(domain_sha256=digest,monotonic=now,
                            original_view_sha256=row.view_sha256,conflicting_view_sha256=view_sha256)
                    return  # Duplicate publication does not renew retention.
            if size > self.max_bytes:
                return  # No retained handoff; never substitute another frame.
            while self._rows and (len(self._rows) >= self.max_records
                                  or self._bytes + size > self.max_bytes):
                self._remove(next(iter(self._rows)))
            context=self._admit_optional_view(context)
            self._rows[digest] = _Acquisition(domain, context, now+self.lifetime_s, size,view_sha256)
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
                for digest,row in self._rows.items()],bytes=self._bytes,
                view_bytes=self._view_bytes,max_view_bytes=self.max_view_bytes,
                last_view_conflict=self._last_view_conflict,last_clear=self._last_clear)
