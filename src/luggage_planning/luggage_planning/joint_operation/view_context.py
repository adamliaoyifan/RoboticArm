"""Bounded owner provenance for failed contacts and plan-only pickup views.

ROS inputs are retained as immutable CDR. Reads do not renew either this resource
lifetime or the perception owner's independent frozen-observation lifetime.
"""
from collections import OrderedDict
from dataclasses import dataclass, replace
import math
import threading
import time


@dataclass(frozen=True)
class PickupViewContext:
    task_request_id: str
    request_cdr: bytes
    domain: object
    observation: object
    probes_cdr: tuple

    @classmethod
    def capture(cls, request, domain, observation, probes):
        from rclpy.serialization import serialize_message
        from luggage_perception.ros_suction_domain import require_matching_detection
        require_matching_detection(domain,request.detection,generation=request.generation,
            contact_model_hash=domain.contact_model_hash)
        binding=observation.binding
        if (binding.request_id,binding.observation_id,binding.box_id,binding.generation,
                binding.stamp_ns,binding.domain_sha256,binding.contact_model_hash) != (
                request.request_id,request.observation_id,domain.box_id,domain.generation,
                domain.stamp_ns,domain.digest(),domain.contact_model_hash):
            raise ValueError('VIEW_CONTEXT_OBSERVATION_MISMATCH')
        rows=tuple((probe.query_id,bytes(serialize_message(probe))) for probe in probes)
        if (not request.request_id or not rows or len({key for key,_ in rows})!=len(rows)
                or any(not probe.query_id or probe.domain_sha256!=domain.digest()
                    or probe.contact_model_hash!=domain.contact_model_hash
                    or probe.placement.identity.request_id!=request.request_id for probe in probes)):
            raise ValueError('VIEW_CONTEXT_PROBE_MISMATCH')
        return cls(request.request_id,bytes(serialize_message(request)),domain,observation,rows)

    def request(self):
        from rclpy.serialization import deserialize_message
        from luggage_msgs.srv import PrepareJointOperationV2
        return deserialize_message(self.request_cdr,PrepareJointOperationV2.Request)

    def probe(self, query_id):
        from rclpy.serialization import deserialize_message
        from luggage_msgs.msg import SuctionSupportProbeV2
        raw=next((raw for key,raw in self.probes_cdr if key==query_id),None)
        return None if raw is None else deserialize_message(raw,SuctionSupportProbeV2)

    @property
    def size_bytes(self):
        # Immutable domain/observation records are small except support bytes;
        # repr includes all metadata and byte values (conservative resource use).
        return len(self.request_cdr)+sum(len(raw) for _,raw in self.probes_cdr)+len(
            repr((self.domain,self.observation)).encode())


@dataclass(frozen=True)
class IssuedPickupView:
    plan: object
    map_source: object
    snapshot: object
    view: object
    message_cdr: bytes
    baseline_sha256: str

    @property
    def size_bytes(self):
        a=self.view.acquisition
        return (len(self.message_cdr)+len(self.snapshot.scene_cdr)+len(a.depth_mm)
            +len(a.instance_region)+len(repr(self.plan).encode()))


@dataclass(frozen=True)
class _Entry:
    context: PickupViewContext
    expires_at: float
    issued: object = None

    @property
    def size_bytes(self):
        return self.context.size_bytes+(self.issued.size_bytes if self.issued is not None else 0)


class PickupViewContextStore:
    """One current context/optional issued view per task, with bounded retention."""
    def __init__(self,*,max_contexts,max_bytes,lifetime_s,clock=time.monotonic):
        if (type(max_contexts) is not int or max_contexts<1 or type(max_bytes) is not int or max_bytes<1
                or not math.isfinite(lifetime_s) or lifetime_s<=0):
            raise ValueError('EXPLICIT_VIEW_CONTEXT_RETENTION_LIMITS_REQUIRED')
        self.max_contexts,self.max_bytes,self.lifetime_s=max_contexts,max_bytes,lifetime_s
        self.clock,self.entries,self.lock=clock,OrderedDict(),threading.RLock()

    def _expire(self):
        now=self.clock()
        if not math.isfinite(now):raise ValueError('FINITE_VIEW_RETENTION_CLOCK_REQUIRED')
        for key,row in tuple(self.entries.items()):
            if now>=row.expires_at:del self.entries[key]
        return now

    def forget(self,task_request_id):
        with self.lock:self.entries.pop(task_request_id,None)

    def remember(self,context):
        if type(context) is not PickupViewContext:raise ValueError('OWNED_VIEW_CONTEXT_REQUIRED')
        with self.lock:
            now=self._expire();old=self.entries.get(context.task_request_id)
            if old is not None:
                if old.context!=context:raise ValueError('VIEW_CONTEXT_REBIND_REFUSED')
                return  # Identical retry is not a renewal.
            row=_Entry(context,now+self.lifetime_s)
            if row.size_bytes>self.max_bytes:raise ValueError('VIEW_CONTEXT_BYTE_LIMIT')
            while (len(self.entries)>=self.max_contexts
                    or sum(e.size_bytes for e in self.entries.values())+row.size_bytes>self.max_bytes):
                self.entries.popitem(last=False)
            self.entries[context.task_request_id]=row

    def lookup(self,task_request_id,domain_sha256):
        with self.lock:
            self._expire();row=self.entries.get(task_request_id)
            return (row.context if row is not None
                and row.context.domain.digest()==domain_sha256 else None)

    def issue(self,context,issued):
        if type(issued) is not IssuedPickupView:raise ValueError('OWNED_PICKUP_VIEW_RECEIPT_REQUIRED')
        with self.lock:
            self._expire();row=self.entries.get(context.task_request_id)
            if row is None or row.context is not context:raise ValueError('VIEW_CONTEXT_CHANGED_OR_EXPIRED')
            updated=replace(row,issued=issued)
            if updated.size_bytes>self.max_bytes:raise ValueError('ISSUED_VIEW_BYTE_LIMIT')
            while sum(e.size_bytes for key,e in self.entries.items()
                    if key!=context.task_request_id)+updated.size_bytes>self.max_bytes:
                key=next(key for key in self.entries if key!=context.task_request_id)
                del self.entries[key]
            self.entries[context.task_request_id]=updated

    def issued(self,task_request_id,route_digest):
        with self.lock:
            self._expire();row=self.entries.get(task_request_id)
            receipt=None if row is None else row.issued
            return receipt if receipt is not None and receipt.plan.route_digest==route_digest else None
