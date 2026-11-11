"""Bounded, process-local provenance of plans issued by the preparation owner.

Only the preparation owner writes this registry after complete certification
and final response capture. A lookup is not a collision or freshness proof;
execution still needs current source, attachment and native path admission.
"""
from collections import OrderedDict
from dataclasses import asdict, dataclass
import json
import math
import threading
import time

from luggage_planning.joint_operation_contracts import JointOperationPlan, admit_operation_plan
from luggage_perception.frozen_suction_observation import FrozenObservationBinding
from luggage_planning.joint_operation_map_source import OperationMapSource


@dataclass(frozen=True)
class _IssuedPlan:
    plan: JointOperationPlan
    expires_at: float
    size_bytes: int
    contact_binding: object = None
    map_source: object = None


class IssuedOperationRegistry:
    def __init__(self, *, max_plans, max_bytes, lifetime_s, clock=time.monotonic):
        if (type(max_plans) is not int or max_plans < 1
                or type(max_bytes) is not int or max_bytes < 1
                or type(lifetime_s) not in (float, int)
                or not math.isfinite(lifetime_s) or lifetime_s <= 0):
            raise ValueError('positive explicit issuance retention limits required')
        self.max_plans, self.max_bytes = max_plans, max_bytes
        self.lifetime_s, self.clock = float(lifetime_s), clock
        self._entries, self._bytes = OrderedDict(), 0
        self._lock = threading.Lock()

    def _now(self):
        now = self.clock()
        if not math.isfinite(now):
            raise ValueError('finite monotonic issuance clock required')
        return now

    def _remove(self, digest):
        self._bytes -= self._entries.pop(digest).size_bytes

    def _expire(self, now):
        for digest, entry in tuple(self._entries.items()):
            if now >= entry.expires_at:
                self._remove(digest)

    def issue(self, plan, *, contact_binding=None, map_source=None):
        """Owner-only publication; callers cannot issue through the check RPC.

        Repeating an identical publication does not extend its lifetime. A new
        successful plan replaces the prior plan for that request and box.
        Oldest issuance is evicted at a resource limit, never silently trusted.
        """
        admit_operation_plan(plan, plan.identity, plan.plan_digest)
        size = len(json.dumps(asdict(plan), sort_keys=True, separators=(',', ':'),
                              allow_nan=False).encode())
        if contact_binding is not None:
            identity = plan.identity
            if (type(contact_binding) is not FrozenObservationBinding
                    or (identity.request_id,identity.observation_id,identity.box_id,identity.generation,
                        identity.observation_stamp_ns,identity.contact_model_hash) !=
                       (contact_binding.request_id,contact_binding.observation_id,contact_binding.box_id,
                        contact_binding.generation,contact_binding.stamp_ns,contact_binding.contact_model_hash)):
                raise ValueError('ISSUED_CONTACT_BINDING_IDENTITY_MISMATCH')
            size += len(json.dumps(asdict(contact_binding),sort_keys=True,separators=(',',':'),
                                   allow_nan=False).encode())
        if map_source is not None:
            if (type(map_source) is not OperationMapSource
                    or (map_source.scene_revision,map_source.geometry_hash,map_source.map_revision) !=
                       (plan.identity.scene_revision,plan.identity.geometry_hash,plan.identity.map_revision)):
                raise ValueError('ISSUED_MAP_SOURCE_IDENTITY_MISMATCH')
            size += len(json.dumps(asdict(map_source),sort_keys=True,separators=(',',':')).encode())
        if size > self.max_bytes:
            raise ValueError('ISSUED_PLAN_EXCEEDS_BYTE_LIMIT')
        with self._lock:
            now = self._now()
            self._expire(now)
            if plan.plan_digest in self._entries:
                if self._entries[plan.plan_digest].contact_binding != contact_binding:
                    raise ValueError('ISSUED_CONTACT_BINDING_REBIND_REFUSED')
                if self._entries[plan.plan_digest].map_source != map_source:
                    raise ValueError('ISSUED_MAP_SOURCE_REBIND_REFUSED')
                return
            key = (plan.identity.request_id, plan.identity.box_id)
            previous = [(digest, entry) for digest, entry in self._entries.items()
                        if (entry.plan.identity.request_id, entry.plan.identity.box_id) == key]
            if any(entry.plan.identity.generation > plan.identity.generation
                   for _, entry in previous):
                raise ValueError('OLDER_ISSUED_GENERATION')
            for digest, _ in previous:
                self._remove(digest)
            while (len(self._entries) >= self.max_plans
                   or self._bytes + size > self.max_bytes):
                self._remove(next(iter(self._entries)))
            self._entries[plan.plan_digest] = _IssuedPlan(plan, now + self.lifetime_s, size,contact_binding,map_source)
            self._bytes += size

    def lookup(self, digest):
        """Return the independently retained plan, without renewing retention."""
        with self._lock:
            self._expire(self._now())
            entry = self._entries.get(digest)
            return None if entry is None else entry.plan

    def current(self, plan):
        issued = self.lookup(plan.plan_digest)
        if issued is None:
            return False
        try:
            admit_operation_plan(plan, issued.identity, issued.plan_digest)
        except (TypeError, ValueError):
            return False
        # Digest validation can take time; expiry or replacement during it
        # cannot leave a stale positive admission.
        return self.lookup(plan.plan_digest) is issued

    def contact_binding(self, plan):
        """Independent owner provenance, never a client-provided replacement."""
        if not self.current(plan):
            return None
        with self._lock:
            self._expire(self._now())
            entry = self._entries.get(plan.plan_digest)
            return entry.contact_binding if entry is not None and entry.plan == plan else None

    def map_source(self, plan):
        """Original source from the owner; never infer it from caller labels."""
        if not self.current(plan):
            return None
        with self._lock:
            self._expire(self._now())
            entry = self._entries.get(plan.plan_digest)
            return entry.map_source if entry is not None and entry.plan == plan else None
