#!/usr/bin/env python3
"""OccupancyMapStore: the sole transactional owner of the cargo map (B1).

The store wraps one :class:`CargoVolumeMapper` (unchanged voxel semantics)
plus one :class:`PendingConfirmation` and exposes a single mutating entry
point, :meth:`apply_update`. Every ROS service on the mapper node is an
adapter that builds a :class:`MapUpdate` and calls it; nothing else may
mutate the map while the store owns it.

Transaction contract (Plan B spec):
- Idempotency: the same ``operation_id`` with the same content retries
  exactly once (:attr:`UpdateStatus.ALREADY_PROCESSED`); the same id with
  different content is refused (``OPERATION_ID_CONFLICT``).
- Premise: a v2 command declares the snapshot identity it was planned
  against, leg by leg (epoch / revision / content digest; an undeclared
  leg is an empty string and is not checked — B2 makes full identity
  mandatory). Any declared-but-mismatched leg is refused with the current
  identity in the result so the caller can re-read and re-check; the
  store never rebases an update onto the latest revision.
- Facts (release/removal/disturb/container pose) are never silently
  dropped on a stale premise: with the confirmation gate on they are
  recorded as pending changes and the revision advances
  (``LATE_FACT_RECORDED_PENDING``); with the gate off there is nothing to
  record, so the stale premise is refused instead (fail closed).
- Evidence: production confirmations need linkable evidence checked
  server-side (verify-anchor tolerance finiteness/negativity/ceiling/
  delta, measured provenance vocabulary).
- Snapshot: each accepted transaction publishes one immutable
  :class:`MapSnapshot` whose ``content_digest`` covers the planning-relevant
  content AND the confirmation state; identical content re-observed yields
  an identical digest, so routes are not invalidated by re-observation.

ROS-free. ``clock_ns`` is injected (the node passes its ROS clock; tests
pass a controllable one). No cross-restart continuity: a restart is a new
epoch and retried pre-restart updates fail with ``EPOCH_MISMATCH``.
"""

import threading
import time
import uuid
from collections import OrderedDict
from dataclasses import dataclass, field
from enum import Enum
from typing import NamedTuple, Optional

from luggage_perception.cargo_view_integration import (
    MAP_MEASURED,
    MAP_RELEASE_INTEGRATED,
    MAP_STATIC_EMPTY,
    REASON_CONTENT_DIGEST_MISMATCH,
    REASON_EPOCH_MISMATCH,
    REASON_EVIDENCE_ANCHOR_MISMATCH,
    REASON_EVIDENCE_MISSING,
    REASON_EVIDENCE_NOT_FINITE,
    REASON_GEOMETRY_HASH_MISMATCH,
    REASON_INVALID_PAYLOAD,
    REASON_LATE_FACT_RECORDED_PENDING,
    REASON_NO_MATCHING_BOX,
    REASON_OPERATION_ID_CONFLICT,
    REASON_REVISION_MISMATCH,
    REASON_STAMP_ALREADY_INTEGRATED,
    apply_integrate,
    release_identity,
    validate_integrate_request,
)
from luggage_perception.cargo_volume_mapper import verify_anchor_delta_m
from luggage_perception.occupancy_confirmation import PendingConfirmation


#: Server-side ceiling on caller-supplied verify tolerances (moved from the
#: node): the anchor gate exists to catch slot-vs-reality divergence
#: (witness_02: 0.65 m phantom; live verify noise 0.033-0.038 m).
VERIFY_ANCHOR_TOLERANCE_MAX_M = 0.30

#: Declared measured-commit provenance vocabulary (moved from the node).
V2_MEASURED_SOURCES = ("verify_placed_box",)

#: Kinds whose payload is a FACT about the world, not a command against a
#: planned snapshot. Facts survive stale premises as pending changes.
FACT_KINDS = frozenset((
    "REMOVE_BOX_V1", "REMOVE_BOX_V2", "NOTE_EVENT", "CONTAINER_POSE"))

#: Fact kinds that need no ``event`` field (their event name is fixed).
_FIXED_FACT_EVENTS = {
    "REMOVE_BOX_V1": "box_removed",
    "REMOVE_BOX_V2": "box_removed",
    "CONTAINER_POSE": "container_pose_changed",
}

#: Default idempotency ledger capacity (FIFO eviction; eviction is safe
#: because ADD/REMOVE are duplicate-safe via ledger matching and INTEGRATE
#: dedups on the cloud stamp key — both properties are pinned by tests).
PROCESSED_CAP = 256


def _digest_of(*parts):
    """Lazy st09_contracts digest (same availability as the planning side)."""
    from st09_contracts._base import digest_of
    return digest_of(*parts)


def _norm_height(value):
    """Height digest input: quantized, -0.0 collapsed, None preserved."""
    if value is None:
        return None
    return round(float(value), 6) + 0.0


class SnapshotIdentity(NamedTuple):
    epoch: str
    revision: int
    content_digest: str


class UpdateStatus(str, Enum):
    ACCEPTED = "accepted"
    ALREADY_PROCESSED = "already_processed"
    RECORDED_PENDING = "recorded_pending"
    REJECTED = "rejected"


class UpdateKind(str, Enum):
    ADD_BOX_V1 = "ADD_BOX_V1"
    ADD_BOX_V2 = "ADD_BOX_V2"
    REMOVE_BOX_V1 = "REMOVE_BOX_V1"
    REMOVE_BOX_V2 = "REMOVE_BOX_V2"
    INTEGRATE_VIEW = "INTEGRATE_VIEW"
    RESET = "RESET"
    NOTE_EVENT = "NOTE_EVENT"
    CONTAINER_POSE = "CONTAINER_POSE"


@dataclass(frozen=True)
class MapUpdate:
    operation_id: str
    kind: UpdateKind
    expected_snapshot: Optional[SnapshotIdentity] = None
    geometry_hash: str = ""
    # Box payloads (container-frame, already normalized by the adapter):
    box: Optional[tuple] = None            # (center, size, yaw)
    measured_source: str = ""
    verified_center: Optional[tuple] = None
    verify_tolerance_m: float = 0.0
    # Integrate payload (container-frame points, TF applied by the adapter):
    points: object = None
    origin: Optional[tuple] = None
    stamp_key: Optional[tuple] = None
    acquisition_stamp_ns: int = 0
    settled: bool = False
    schema_version: int = 0
    pose_at_confirm: Optional[dict] = None
    # Fact payloads:
    event: str = ""
    event_stamp_ns: int = 0

    def content_fingerprint(self):
        """Digest over the canonical content of this update.

        ``points`` is fingerprinted through its stamp key and origin, not
        its raw values: a literal retry of the same view carries the same
        stamp, and a different cloud under one operation id is caught by
        the stamp-key dedup anyway.
        """
        return _digest_of(
            self.kind.value,
            self.expected_snapshot,
            self.geometry_hash,
            None if self.box is None else (
                tuple(float(v) for v in self.box[0]),
                tuple(float(v) for v in self.box[1]),
                float(self.box[2])),
            self.measured_source,
            None if self.verified_center is None else
            tuple(float(v) for v in self.verified_center),
            float(self.verify_tolerance_m),
            self.stamp_key,
            None if self.origin is None else
            tuple(float(v) for v in self.origin),
            int(self.acquisition_stamp_ns),
            bool(self.settled),
            int(self.schema_version),
            self.event,
            int(self.event_stamp_ns),
        )


@dataclass(frozen=True)
class MapSnapshot:
    """Immutable projection of the map at one identity.

    Carries the 2.5D projection (hundreds of cells), never the voxel
    grid. All sequences are tuples; the dataclass is frozen. Built exactly
    once per accepted transaction, inside the store lock.
    """
    identity: SnapshotIdentity
    geometry_hash: str
    resolution: float
    nx: int
    ny: int
    inner_size: tuple
    center_base: tuple
    yaw: float
    # Column projections (row-major tuples-of-tuples, [ix][iy]):
    height: tuple
    state: tuple
    confidence: tuple
    clearance: tuple
    semantic_label: tuple
    surface_z_container: tuple
    contract_confidence: tuple
    height_above_floor: tuple
    known_ratio: tuple
    origin_local: tuple
    # Placed ledger (detached, digest-relevant):
    ledger: tuple
    # Confirmation (PendingConfirmation.surface_fields() projection):
    occupancy_confirmed: bool
    change_event: str
    change_token: int
    pending_changes: tuple
    last_settle: dict = field(default_factory=dict)
    # Acquisition identity (provenance; excluded from the digest):
    acquisition_stamp_ns: int = 0
    map_source: str = MAP_STATIC_EMPTY
    placed_count: int = 0
    cargo_observed: bool = False

    @property
    def revision(self):
        return self.identity.revision


@dataclass(frozen=True)
class UpdateResult:
    status: UpdateStatus
    reason: str
    identity: SnapshotIdentity
    snapshot: MapSnapshot
    details: dict = field(default_factory=dict)

    @property
    def accepted(self):
        return self.status is UpdateStatus.ACCEPTED


class OccupancyMapStore(object):
    """Sole write owner of one cargo map. See module docstring."""

    def __init__(self, mapper, *, require_confirmation=False, clock_ns=None,
                 epoch=None, processed_cap=PROCESSED_CAP):
        self._mapper = mapper
        self._pending = PendingConfirmation(bool(require_confirmation))
        self._clock_ns = clock_ns or time.time_ns
        self._txn_lock = threading.RLock()
        self._epoch = str(epoch) if epoch else uuid.uuid4().hex
        self._processed = OrderedDict()
        self._processed_cap = max(1, int(processed_cap))
        # State moved under the transaction lock from the node:
        self._integrated = set()
        self._last_acquisition_ns = 0
        self._map_source = MAP_STATIC_EMPTY
        self._cargo_observed = False
        self._pose_at_confirm = None
        self._snapshot = self._build_snapshot()

    # ------------------------------------------------------------------
    # Reads

    @property
    def mapper(self):
        return self._mapper

    @property
    def epoch(self):
        return self._epoch

    @property
    def map_revision(self):
        return self._mapper.map_revision

    @property
    def identity(self):
        return self._snapshot.identity

    def snapshot(self):
        """Current immutable snapshot (rebuilt only by transactions)."""
        return self._snapshot

    # ------------------------------------------------------------------
    # The one mutating entry point

    def apply_update(self, update):
        if not isinstance(update, MapUpdate):
            raise TypeError("apply_update takes a MapUpdate")
        with self._txn_lock:
            return self._apply_locked(update)

    def _apply_locked(self, update):
        # 1. IDEMPOTENCY ---------------------------------------------
        key = str(update.operation_id or "")
        if not key:
            return self._reject(REASON_OPERATION_ID_CONFLICT,
                                "operation_id is required")
        stored = self._processed.get(key)
        if stored is not None:
            fingerprint, identity, status, reason = stored
            if fingerprint != update.content_fingerprint():
                return self._reject(
                    REASON_OPERATION_ID_CONFLICT,
                    "operation_id %r reused with different content" % key)
            # Exactly-once replay: the write already happened (whatever the
            # map has been through since, including a reset — a retried
            # pre-reset write must NOT re-apply onto the fresh epoch). The
            # verdict is ALREADY_PROCESSED against the CURRENT identity;
            # the original outcome rides in details.
            return UpdateResult(
                status=UpdateStatus.ALREADY_PROCESSED, reason="",
                identity=self._snapshot.identity,
                snapshot=self._snapshot,
                details={"operation_id": key, "replay": True,
                         "original_status": status.value,
                         "original_reason": reason,
                         "original_identity": identity})

        # 2. PREMISE ---------------------------------------------------
        current = self._snapshot.identity
        is_fact = update.kind.value in FACT_KINDS
        if not is_fact:
            reason = self._premise_violation(update, current)
            if reason is not None:
                return self._reject(reason, self._premise_message(
                    reason, update, current))
        else:
            want = str(update.geometry_hash or "")
            if want and want != self._mapper.geometry_hash:
                return self._reject(
                    REASON_GEOMETRY_HASH_MISMATCH,
                    "fact against a foreign hull (%r)" % want)
            if (update.expected_snapshot is not None
                    and update.expected_snapshot != current):
                # A late fact is never silently dropped when the
                # confirmation gate can hold it: record it as pending so no
                # consumer can keep claiming "confirmed, unchanged".
                event = self._fact_event_name(update)
                if event is None:
                    return self._reject(
                        REASON_EVIDENCE_MISSING,
                        "fact kind %s needs an event name"
                        % update.kind.value)
                token = self._pending.record(
                    event, update.event_stamp_ns or self._clock_ns())
                if not token:
                    # Confirmation gate off: nothing can be recorded, so a
                    # stale-premise fact fails closed instead of applying
                    # blind (pre-B1 _on_remove_v2 behavior).
                    return self._reject(
                        REASON_REVISION_MISMATCH,
                        self._premise_message(
                            REASON_REVISION_MISMATCH, update, current))
                self._mapper.note_external_change()
                self._snapshot = self._build_snapshot()
                self._remember(key, update, UpdateStatus.RECORDED_PENDING,
                               REASON_LATE_FACT_RECORDED_PENDING)
                return UpdateResult(
                    status=UpdateStatus.RECORDED_PENDING,
                    reason=REASON_LATE_FACT_RECORDED_PENDING,
                    identity=self._snapshot.identity,
                    snapshot=self._snapshot,
                    details={"operation_id": key,
                             "expected": update.expected_snapshot,
                             "current": self._snapshot.identity})

        # 3. EVIDENCE --------------------------------------------------
        anchor = None
        token_at_start = None
        if update.kind is UpdateKind.ADD_BOX_V2:
            anchor = self._check_add_v2_evidence(update)
            if isinstance(anchor, str):
                return self._reject(anchor, self._evidence_message(
                    anchor, update))
        elif update.kind is UpdateKind.INTEGRATE_VIEW:
            reason = validate_integrate_request(
                update.settled, update.schema_version, update.geometry_hash,
                current.revision, self._mapper.geometry_hash,
                self._mapper.map_revision)
            if reason is not None:
                return self._reject(reason, "integrate premise failed")
            if tuple(update.stamp_key or ()) in self._integrated:
                return self._reject(
                    REASON_STAMP_ALREADY_INTEGRATED,
                    "cloud stamp %r already integrated" % (update.stamp_key,))
            # Read BEFORE any mutation: an integration already running when
            # a change arrived must not clear that change.
            token_at_start = self._pending.begin()

        # 4. MUTATION + state transitions ------------------------------
        details = {"operation_id": key}
        try:
            rejection = self._mutate_locked(
                update, anchor, token_at_start, details)
        except ValueError as error:
            # The mapper validates before it mutates (finite/positive
            # size, zero-voxel rasterization), so a refusal leaves the
            # map untouched; surface it as a transaction rejection, never
            # as an exception through the adapter boundary.
            rejection = (REASON_INVALID_PAYLOAD, str(error))
        if rejection is not None:
            return self._reject(*rejection)

        # 5. SNAPSHOT + idempotency ledger -----------------------------
        self._snapshot = self._build_snapshot()
        self._remember(key, update, UpdateStatus.ACCEPTED, "")
        return UpdateResult(
            status=UpdateStatus.ACCEPTED, reason="",
            identity=self._snapshot.identity, snapshot=self._snapshot,
            details=details)

    def _mutate_locked(self, update, anchor, token_at_start, details):
        """Step 4 of the transaction: mutate + state transitions.

        Returns None on success, or a ``(reason, message)`` rejection
        tuple. ValueError from the mapper's payload validation is
        converted to INVALID_PAYLOAD by the caller.
        """
        if update.kind in (UpdateKind.ADD_BOX_V1, UpdateKind.ADD_BOX_V2):
            center, size, yaw = update.box
            added = self._mapper.mark_placed_box(
                list(center), list(size), yaw=float(yaw),
                measured_source=(update.measured_source or None),
                verify_anchor=anchor)
            details["committed_now"] = bool(added)
            self._adopt_release(bool(added), update.box)
        elif update.kind in (UpdateKind.REMOVE_BOX_V1,
                             UpdateKind.REMOVE_BOX_V2):
            center, size, yaw = update.box
            removed = self._mapper.unmark_placed_box(
                list(center), list(size), yaw=float(yaw))
            if not removed:
                return (REASON_NO_MATCHING_BOX,
                        "no committed box matches the removal")
            details["removed"] = True
            self._adopt_after_removal()
        elif update.kind is UpdateKind.INTEGRATE_VIEW:
            outcome = apply_integrate(self._mapper, update.points,
                                      update.origin)
            if not outcome["ok"]:
                # Nothing mutated: the operation id is NOT consumed, so an
                # I/O-level retry with a fresh cloud stays meaningful.
                return (outcome["reason_code"] or "VIEW_EMPTY",
                        "integration produced no change")
            self._integrated.add(tuple(update.stamp_key or ()))
            self._last_acquisition_ns = int(update.acquisition_stamp_ns or 0)
            self._map_source = MAP_MEASURED
            self._cargo_observed = True
            if update.pose_at_confirm is not None:
                self._pose_at_confirm = dict(update.pose_at_confirm)
            cleared = self._pending.settle(
                token_at_start, update.acquisition_stamp_ns,
                outcome.get("observed_points"), bool(update.settled))
            details.update({
                "n_finite": outcome.get("n_finite"),
                "n_kept": outcome.get("n_kept"),
                "latency_sec": outcome.get("latency_sec"),
                "cleared_tokens": list(cleared or ()),
            })
        elif update.kind is UpdateKind.RESET:
            self._mapper.reset()
            self._pending.clear()
            self._integrated = set()
            self._last_acquisition_ns = 0
            self._map_source = MAP_STATIC_EMPTY
            self._cargo_observed = False
            self._pose_at_confirm = None
            self._epoch = uuid.uuid4().hex
            details["epoch_rotated"] = True
        elif update.kind in (UpdateKind.NOTE_EVENT, UpdateKind.CONTAINER_POSE):
            event = self._fact_event_name(update)
            if event is None:
                return (REASON_EVIDENCE_MISSING,
                        "fact kind %s needs an event name"
                        % update.kind.value)
            token = self._pending.record(
                event, update.event_stamp_ns or self._clock_ns())
            details["recorded"] = bool(token)
            if token:
                # A real confirmation transition with no voxel mutation:
                # advance the revision so the rebuilt snapshot is
                # distinguishable from a stale one.
                self._mapper.note_external_change()
        else:  # pragma: no cover - enum exhaustiveness guard
            return ("UNKNOWN_KIND", str(update.kind))
        return None

    # ------------------------------------------------------------------
    # Premise / evidence helpers

    @staticmethod
    def _premise_violation(update, current):
        """Leg-by-leg premise check; undeclared legs are not checked.

        The revision leg is always required for command kinds (0 vs a live
        map is a mismatch, exactly like the node's ledger guard today).
        """
        expected = update.expected_snapshot
        if expected is None:
            return None
        if expected.epoch and expected.epoch != current.epoch:
            return REASON_EPOCH_MISMATCH
        if expected.revision != current.revision:
            return REASON_REVISION_MISMATCH
        if (expected.content_digest
                and expected.content_digest != current.content_digest):
            return REASON_CONTENT_DIGEST_MISMATCH
        return None

    @staticmethod
    def _premise_message(reason, update, current):
        return "premise %s does not match current %s (%s)" % (
            update.expected_snapshot, current, reason)

    def _check_add_v2_evidence(self, update):
        """Server-side verify-anchor gate. Returns the anchor dict on
        success or a reason-code string on refusal."""
        tolerance = float(update.verify_tolerance_m or 0.0)
        if tolerance != tolerance or tolerance in (float("inf"),
                                                  float("-inf")):
            return REASON_EVIDENCE_NOT_FINITE
        if tolerance < 0.0:
            # 0 is the ONLY un-anchored compat mode; negatives refused
            # (a negative tolerance used to fall through the `> 0` gate).
            return REASON_EVIDENCE_NOT_FINITE
        if tolerance > VERIFY_ANCHOR_TOLERANCE_MAX_M:
            return REASON_EVIDENCE_ANCHOR_MISMATCH
        if str(update.measured_source or "") not in V2_MEASURED_SOURCES:
            return REASON_EVIDENCE_MISSING
        if tolerance == 0.0:
            return None  # legacy un-anchored commit
        center, _size, _yaw = update.box
        verified = update.verified_center
        if verified is None:
            return REASON_EVIDENCE_MISSING
        verified = tuple(float(v) for v in verified)
        if not all(v == v and abs(v) != float("inf") for v in verified):
            return REASON_EVIDENCE_NOT_FINITE
        delta = verify_anchor_delta_m(list(center), list(verified))
        if delta is None or delta > tolerance:
            return REASON_EVIDENCE_ANCHOR_MISMATCH
        return {
            "request_id": str(update.operation_id),
            "delta_m": float(delta),
            "tolerance_m": tolerance,
        }

    @staticmethod
    def _evidence_message(reason, update):
        if reason == REASON_EVIDENCE_NOT_FINITE:
            return ("verify_tolerance_m must be finite and non-negative "
                    "(got %r); 0 is the un-anchored legacy mode"
                    % update.verify_tolerance_m)
        if reason == REASON_EVIDENCE_ANCHOR_MISMATCH:
            return ("verify tolerance %r outside the mapper gate (anchor "
                    "delta exceeds it or the ceiling %.3f)"
                    % (update.verify_tolerance_m,
                       VERIFY_ANCHOR_TOLERANCE_MAX_M))
        return "add_placed_box_v2 evidence incomplete (%r)" % (
            update.measured_source,)

    @staticmethod
    def _fact_event_name(update):
        """Map a fact kind to the PendingConfirmation event vocabulary."""
        fixed = _FIXED_FACT_EVENTS.get(update.kind.value)
        if fixed is not None:
            return fixed
        return str(update.event or "") or None

    # ------------------------------------------------------------------
    # Adoption logic (ported verbatim from the node, same semantics)

    def _adopt_release(self, committed_now, footprint):
        """A placed box leaves the static-empty identity.

        Confirmation gate on: a new commit records a pending change whose
        ``stamp_ns`` is the commit clock and never touches the acquisition
        stamp (a map with no sensor acquisition keeps stamp <= 0). Gate
        off: a new commit stamps the clock; a duplicate keeps the existing
        release stamp.
        """
        if len(self._mapper.commit_ledger()) <= 0:
            return
        previous_source = self._map_source
        now_ns = self._clock_ns()
        stamp, source = release_identity(now_ns)
        self._map_source = source
        if self._pending.require_confirmation:
            if committed_now:
                self._pending.record("box_placed", now_ns, footprint)
            return
        if committed_now or int(self._last_acquisition_ns) <= 0 \
                or previous_source != MAP_RELEASE_INTEGRATED:
            self._last_acquisition_ns = int(stamp)

    def _adopt_after_removal(self):
        placed = len(self._mapper.commit_ledger())
        self._pending.record("box_removed", self._clock_ns())
        if placed > 0:
            self._map_source = MAP_RELEASE_INTEGRATED
            return
        if self._cargo_observed:
            self._map_source = MAP_MEASURED
            return
        self._map_source = MAP_STATIC_EMPTY
        self._last_acquisition_ns = 0
        self._pending.clear()

    # ------------------------------------------------------------------
    # Bookkeeping

    def _reject(self, reason, message):
        return UpdateResult(
            status=UpdateStatus.REJECTED, reason=reason,
            identity=self._snapshot.identity, snapshot=self._snapshot,
            details={"message": message})

    def _remember(self, key, update, status, reason):
        self._processed[key] = (
            update.content_fingerprint(), self._snapshot.identity,
            status, reason)
        while len(self._processed) > self._processed_cap:
            self._processed.popitem(last=False)

    # ------------------------------------------------------------------
    # Snapshot construction + digest

    def _build_snapshot(self):
        base = self._mapper.surface_map_2d()
        contract = self._mapper.surface_contract_v2("snap", "snap")
        fields = self._pending.surface_fields()
        ledger = tuple(
            (tuple(r["center"]), tuple(r["size"]), float(r["yaw"]),
             str(r.get("measured_source") or ""),
             r.get("verify_anchor"))
            for r in self._mapper.commit_ledger())
        snapshot = MapSnapshot(
            identity=None,  # set below once the digest exists
            geometry_hash=str(base.get("geometry_hash") or ""),
            resolution=float(base.get("resolution")),
            nx=int(base.get("nx")),
            ny=int(base.get("ny")),
            inner_size=tuple(float(v) for v in base.get("inner_size")),
            center_base=tuple(float(v) for v in base.get("center_base")),
            yaw=float(base.get("yaw") or 0.0),
            height=_freeze(base.get("height")),
            state=_freeze(base.get("state")),
            confidence=_freeze(base.get("confidence")),
            clearance=_freeze(base.get("clearance")),
            semantic_label=_freeze(base.get("semantic_label")),
            surface_z_container=_freeze_norm(
                contract.get("surface_z_container")),
            contract_confidence=_freeze(contract.get("confidence")),
            height_above_floor=_freeze_norm(
                contract.get("height_above_floor")),
            known_ratio=_freeze_flat(base.get("known_ratio")),
            origin_local=tuple(float(v) for v in base.get("origin_local")),
            ledger=ledger,
            occupancy_confirmed=bool(fields["occupancy_confirmed"]),
            change_event=str(fields["change_event"] or ""),
            change_token=int(fields["change_token"]),
            pending_changes=tuple(
                (c["event"], int(c["token"]))
                for c in fields["pending_changes"]),
            last_settle=dict(fields.get("last_settle") or {}),
            acquisition_stamp_ns=int(self._last_acquisition_ns),
            map_source=str(self._map_source),
            placed_count=len(ledger),
            cargo_observed=bool(self._cargo_observed),
        )
        digest = _content_digest(snapshot)
        identity = SnapshotIdentity(self._epoch, self._mapper.map_revision,
                                    digest)
        # frozen dataclass: the one deferred field is set via the
        # low-level hook; every other field was set at construction.
        object.__setattr__(snapshot, "identity", identity)
        return snapshot


def _freeze(rows):
    """tuple-of-tuples for a row-major [ix][iy] projection."""
    if rows is None:
        return ()
    return tuple(tuple(row) for row in rows)


def _freeze_norm(rows):
    """Nested projection with per-cell height normalization (digest-stable)."""
    if rows is None:
        return ()
    return tuple(tuple(_norm_height(v) for v in row) for row in rows)


def _freeze_flat(values):
    if values is None:
        return ()
    return tuple(values)


def _content_digest(snapshot):
    """Digest over planning-relevant content AND confirmation state.

    Excludes revision/epoch/stamps/request ids/commit order: identical
    content re-observed keeps the digest (routes survive re-observation)
    and two disjoint commits landing in either order converge, while any
    column state/confidence/height change, ledger geometry change, or
    confirmation transition changes it. Provenance fields
    (measured_source, verify_anchor) are excluded — same geometry is the
    same planning content.
    """
    columns = tuple(
        (snapshot.state[ix][iy],
         snapshot.confidence[ix][iy],
         _norm_height(snapshot.height[ix][iy]))
        for ix in range(snapshot.nx) for iy in range(snapshot.ny))
    ledger_geometry = tuple(sorted(
        (center, size, yaw)
        for center, size, yaw, _source, _anchor in snapshot.ledger))
    confirmation = (
        snapshot.occupancy_confirmed,
        snapshot.change_event,
        snapshot.pending_changes,
    )
    return _digest_of(
        snapshot.geometry_hash, snapshot.nx, snapshot.ny,
        snapshot.resolution, snapshot.inner_size,
        columns, ledger_geometry, confirmation)
