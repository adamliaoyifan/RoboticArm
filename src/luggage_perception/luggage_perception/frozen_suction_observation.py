"""Bounded ownership of one acquisition and its actual-footprint surface check.

Only observed support and the existing surface-quality policy are evaluated.
The result does not establish a physical seal, attachment freshness or motion
permission. Those require the execution owner's actual vacuum/provider gates.
There are no guessed missing-depth, contact-height or contact-angle tolerances.
"""
from collections import OrderedDict
from dataclasses import asdict, dataclass, fields
import math
import json
import threading
import time
import uuid

import numpy as np
from scipy.spatial.transform import Rotation

from luggage_perception.directed_suction_quality import (
    DirectedSuctionContext, QUALITY_METRICS, _digest,
)
from luggage_perception.observed_contact_footprint import check_observed_contact_footprint
from luggage_perception.suction_patch_evaluator import SuctionPatchEvaluator


@dataclass(frozen=True)
class FrozenObservationBinding:
    request_id: str
    observation_id: str
    retention_id: str
    box_id: str
    generation: int
    stamp_ns: int
    contact_model_hash: str
    domain_sha256: str
    quality_policy_sha256: str
    samples_sha256: str

    def digest(self):
        return _digest(asdict(self))


@dataclass(frozen=True)
class ActualFootprintResult:
    binding_sha256: str
    attachment_geometry_sha256: str
    measured_box_wdh: tuple
    box_from_contact_frame: tuple
    accepted: bool
    reason: str
    world_from_contact_frame_at_observation: tuple = ()
    metrics: tuple = ()
    # Diagnostics only, not new contact tolerances or a physical-seal verdict.
    plane_height_offset_m: float = 0.
    normal_difference_rad: float = 0.

    def digest(self):
        return _digest(asdict(self))


@dataclass(frozen=True)
class _Retained:
    binding: FrozenObservationBinding
    context: DirectedSuctionContext
    expires_at: float
    size_bytes: int


class FrozenSuctionObservations:
    """Perception owner only. Reads do not renew lifetime or change acquisition.

    Limits bound memory/retention; they do not represent physical freshness.
    Explicit revocation and execution's identity/model/observation checks remain
    required. New acquisition publication does not mutate a retained context.
    """
    def __init__(self, *, max_records, max_bytes, lifetime_s, clock=time.monotonic):
        if (type(max_records) is not int or max_records < 1
                or type(max_bytes) is not int or max_bytes < 1
                or type(lifetime_s) not in (int,float)
                or not math.isfinite(lifetime_s) or lifetime_s <= 0):
            raise ValueError('explicit positive frozen observation resource limits required')
        self.max_records, self.max_bytes, self.lifetime_s = max_records, max_bytes, lifetime_s
        self.clock, self._entries, self._bytes = clock, OrderedDict(), 0
        self._lock = threading.Lock()

    def _now(self):
        now = self.clock()
        if not math.isfinite(now):
            raise ValueError('finite monotonic observation clock required')
        return now

    def _remove(self, key):
        self._bytes -= self._entries.pop(key).size_bytes

    def retain(self, request_id, observation_id, context):
        if (not isinstance(request_id,str) or not request_id.strip()
                or not isinstance(observation_id,str) or not observation_id.strip()
                or type(context) is not DirectedSuctionContext):
            raise ValueError('request/observation identity and measured context required')
        # Own the sampling buffers and model even if a caller retains its input.
        owned = DirectedSuctionContext(context.domain,context._height_map,context._model)
        if (owned.domain_sha256,owned.policy_sha256,owned.samples_sha256) != (
                context.domain_sha256,context.policy_sha256,context.samples_sha256):
            raise ValueError('MEASURED_CONTEXT_CHANGED')
        metadata = asdict(owned.domain)
        metadata.pop('support')
        size = len(owned.domain.support) + len(json.dumps({
            'domain':metadata,'model':owned._model.as_dict(),
            'request_id':request_id,'observation_id':observation_id},
            sort_keys=True,separators=(',',':'),allow_nan=False).encode())
        size += sum(getattr(owned._height_map,f.name).nbytes for f in fields(owned._height_map)
                    if isinstance(getattr(owned._height_map,f.name),np.ndarray))
        if size > self.max_bytes:
            raise ValueError('FROZEN_OBSERVATION_EXCEEDS_BYTE_LIMIT')
        key = request_id,observation_id
        with self._lock:
            now = self._now()
            old = self._entries.get(key)
            if old is not None:
                if now >= old.expires_at:
                    raise ValueError('FROZEN_OBSERVATION_EXPIRED')
                if (old.context.domain_sha256,old.context.policy_sha256,old.context.samples_sha256) != (
                        owned.domain_sha256,owned.policy_sha256,owned.samples_sha256):
                    raise ValueError('FROZEN_OBSERVATION_REBIND_REFUSED')
                return old.binding
            for other,entry in tuple(self._entries.items()):
                if now >= entry.expires_at:
                    self._remove(other)
            while self._entries and (len(self._entries) >= self.max_records or self._bytes+size > self.max_bytes):
                self._remove(next(iter(self._entries)))
            domain = owned.domain
            binding = FrozenObservationBinding(request_id,observation_id,uuid.uuid4().hex,
                domain.box_id,domain.generation,domain.stamp_ns,domain.contact_model_hash,
                owned.domain_sha256,owned.policy_sha256,owned.samples_sha256)
            self._entries[key] = _Retained(binding,owned,now+self.lifetime_s,size)
            self._bytes += size
            return binding

    def lookup(self, binding):
        if type(binding) is not FrozenObservationBinding:
            return None
        with self._lock:
            entry = self._entries.get((binding.request_id,binding.observation_id))
            if entry is None or entry.binding != binding or self._now() >= entry.expires_at:
                return None
            return entry.context

    def revoke(self, request_id, observation_id):
        with self._lock:
            key = request_id,observation_id
            if key in self._entries:
                self._remove(key)

    def check(self, binding, *, attachment_geometry_sha256, measured_box_wdh,
              box_from_contact_frame, current_model, deadline):
        """Recheck the measured rectangle against the frozen measured samples.

        box_from_contact_frame is the actual measured rigid attachment inverse,
        in the same body axes as the original measured box. Compose it with the
        acquisition box pose, never a live wrist-camera/world transform.
        """
        context = self.lookup(binding)
        if context is None:
            raise ValueError('FROZEN_OBSERVATION_NOT_CURRENT')
        if (_digest(current_model.as_dict()) != binding.quality_policy_sha256
                or current_model.identity_hash != binding.contact_model_hash):
            raise ValueError('FROZEN_CONTACT_MODEL_OR_POLICY_CHANGED')
        dimensions = tuple(float(v) for v in measured_box_wdh)
        if dimensions != context.domain.box_wdh:
            raise ValueError('FROZEN_MEASURED_BOX_MODEL_CHANGED')
        if (not isinstance(attachment_geometry_sha256,str) or len(attachment_geometry_sha256) != 64
                or any(c not in '0123456789abcdef' for c in attachment_geometry_sha256)):
            raise ValueError('ACTUAL_GEOMETRY_DIGEST_REQUIRED')
        actual = np.asarray(box_from_contact_frame,dtype=float).reshape(4,4)
        # check_observed_contact_footprint below validates the resulting SE(3).
        observed_box = np.eye(4)
        observed_box[:3,:3] = Rotation.from_quat(context.domain.box_quaternion_xyzw).as_matrix()
        observed_box[:3,3] = context.domain.box_xyz_world
        pose = observed_box @ actual
        if not math.isfinite(deadline):
            raise ValueError('FINITE_ACTUAL_CONTACT_DEADLINE_REQUIRED')

        def result(accepted, reason, metrics=()):
            value = ActualFootprintResult(binding.digest(),attachment_geometry_sha256,dimensions,
                tuple(float(v) for v in actual.ravel()),accepted,reason,
                tuple(float(v) for v in pose.ravel()),metrics,
                float(np.asarray(context.domain.normal_world) @ pose[:3,3]
                    + context.domain.plane_offset_world_m),
                float(np.arccos(np.clip(np.asarray(context.domain.normal_world) @ -pose[:3,2],-1.,1.))))
            if self.lookup(binding) is not context:
                return ActualFootprintResult(binding.digest(),attachment_geometry_sha256,dimensions,
                    tuple(float(v) for v in actual.ravel()),False,'FROZEN_OBSERVATION_INVALIDATED')
            return value

        coverage = check_observed_contact_footprint(context.domain,context._model,pose)
        if self._now() >= deadline:
            return result(False,'ACTUAL_CONTACT_DEADLINE')
        if not coverage.contained:
            return result(False,coverage.reason)
        if np.asarray(context.domain.normal_world) @ -pose[:3,2] <= 0:
            return result(False,'ACTUAL_CONTACT_WRONG_TOP_FACE_DIRECTION')
        window = context._window(pose)
        local,mask,half = context._sample_window(window)
        uv = (np.asarray((context.domain.u_axis_world,context.domain.v_axis_world)) @ pose[:3,3])
        sec,ns = divmod(context.domain.stamp_ns,10**9)
        records,rejects = SuctionPatchEvaluator(context._model)._evaluate_batch(local,
            np.array([half[0]]),np.array([half[1]]),np.zeros(1),float(sec)+float(ns)*1e-9,
            'world',context.domain.instance_id,context.domain.generation,window_half_shape=half,
            footprint_mask=mask[None],centers_uv=np.array([uv]))
        if self._now() >= deadline:
            return result(False,'ACTUAL_CONTACT_DEADLINE')
        if not records:
            return result(False,rejects[0][2] if rejects else 'ACTUAL_CONTACT_NO_QUALITY_SAMPLES')
        metrics = tuple((name,getattr(records[0],name)) for name in QUALITY_METRICS)
        # Keep measured pose. The fitted quality-kernel normal never repairs it.
        return result(True,'FROZEN_ACTUAL_FOOTPRINT_QUALITY_OK',metrics)
