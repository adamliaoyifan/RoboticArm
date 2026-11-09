#!/usr/bin/env python3
"""MapValidityLedger: the reset-validity lifecycle owned by the map store.

The G-side analogue of the OctoMap observer's ``ValidityBoard``
(``src/uld_occ_observer/src/validity.cpp``): a ROS-free state machine whose
only writer is :class:`~luggage_perception.occupancy_map_store.OccupancyMapStore`
(every hook below runs under the store's transaction lock — the store calls
them from inside ``apply_update``; nothing else may). The node only converts
the ledger's frozen records to topics/services.

State machine (state words deliberately identical to the observer's wire
vocabulary so the common consumer maps them 1:1):

- boot                     -> BUILDING(epoch, generation=1)
- reset accepted           -> INVALID(new epoch, gen+1) then
                              BUILDING(new epoch, gen+1) — both events land
                              atomically inside the one reset transaction
                              (the observer lands them across two calls; the
                              generation counting matches either way).
- first accepted integrate -> premises recorded (new_epoch_observed,
                              last_integrated_stamp_ns)
- any accepted transaction -> BUILDING->VALID(generation+1) iff
                              epoch matches AND an observation was accepted
                              in this epoch AND the snapshot passes the
                              explicit integrity predicate AND
                              now - last_integrated_stamp_ns <= max_age_ns.
                              ``known_cells > 0`` alone is NOT the integrity
                              predicate (reviews 2026-11-05 point 8).
- once VALID              -> stays VALID until the next reset; freshness
                              afterwards is the consumer's contract.

Identity is lossless end to end: the G epoch is the store's UUID string,
carried verbatim on every wire surface. Nothing here truncates, hashes or
re-numbers it (the rejected 31-bit scheme is explicitly not reused).

Timestamps: ``transition_mono_ns`` is a monotonic-domain stamp (injected
``mono_clock_ns``, default ``time.monotonic_ns``) used for event ordering;
``transition_stamp_ns`` is the same transition in the store's clock domain
(the node's ROS clock). The two are recorded together, never compared to
each other (I2: ROS time and monotonic are not mixed).

ROS-free. ``clock_ns`` is injected by the store.
"""

import json
import time
from dataclasses import dataclass, field
from typing import Callable, Optional, Tuple

#: Wire state words (identical to uld_occ_observer's ValidityBoard).
STATE_INVALID = "INVALID"
STATE_BUILDING = "BUILDING"
STATE_VALID = "VALID"

#: BUILDING->VALID freshness premise: the snapshot's last integrated
#: acquisition stamp must be within this bound of now (same clock domain;
#: mirrors the observer's ``validity_max_age_ms`` default of 1000).
VALIDITY_MAX_AGE_NS = 1_000_000_000

#: Latched transition log depth (mirrors the observer's kMaxEvents so a
#: reconnecting consumer can replay recent history from the status topic).
LIFECYCLE_EVENTS_CAP = 64

#: Backend tag bound into every identity comparison downstream (I2: G and O
#: epochs must never be interchangeable even when their strings collide).
BACKEND = "grid_g"

#: Clock domain tag (the store's clock is the node's ROS clock).
CLOCK_DOMAIN = "ros"


@dataclass(frozen=True)
class ValidityEvent:
    """One latched transition (replayable history for late subscribers)."""

    generation: int
    state: str
    epoch: str
    reason: str
    reset_id: str
    transition_mono_ns: int


@dataclass(frozen=True)
class ValidityRecord:
    """The current lifecycle state, one consistent frozen value.

    Field names mirror the observer's ``validity_json`` payload (epoch is
    the G UUID string, not the observer's uint64) plus ``backend`` and the
    ROS-domain ``transition_stamp_ns`` twin.
    """

    state: str
    epoch: str
    generation: int
    reason: str
    reset_id: str
    transition_mono_ns: int
    transition_stamp_ns: int
    map_revision: int
    observed_revision: int
    last_integrated_stamp_ns: int
    geometry_hash: str
    config_digest: str
    clock_domain: str
    backend: str = BACKEND

    def as_dict(self):
        return {
            "state": self.state,
            "epoch": self.epoch,
            "generation": self.generation,
            "reason": self.reason,
            "reset_id": self.reset_id,
            "transition_mono_ns": self.transition_mono_ns,
            "transition_stamp_ns": self.transition_stamp_ns,
            "map_revision": self.map_revision,
            "observed_revision": self.observed_revision,
            "last_integrated_stamp_ns": self.last_integrated_stamp_ns,
            "geometry_hash": self.geometry_hash,
            "config_digest": self.config_digest,
            "clock_domain": self.clock_domain,
            "backend": self.backend,
        }

    def as_json(self):
        return json.dumps(self.as_dict(), sort_keys=True)


class MapValidityLedger(object):
    """Sole-owner validity ledger; every method runs under the store lock."""

    def __init__(self, clock_ns, *, mono_clock_ns=None, backend=BACKEND,
                 max_age_ns=VALIDITY_MAX_AGE_NS,
                 events_cap=LIFECYCLE_EVENTS_CAP):
        self._clock_ns = clock_ns
        self._mono_clock_ns = mono_clock_ns or time.monotonic_ns
        self._backend = str(backend or BACKEND)
        self._max_age_ns = int(max_age_ns)
        self._events_cap = max(1, int(events_cap))
        self._events = []
        # Current record; identity fields are empty until an observation
        # promotes them (a BUILDING map has nothing to pin yet).
        self._record = ValidityRecord(
            state=STATE_INVALID, epoch="", generation=0, reason="",
            reset_id="", transition_mono_ns=0, transition_stamp_ns=0,
            map_revision=0, observed_revision=0,
            last_integrated_stamp_ns=0, geometry_hash="", config_digest="",
            clock_domain=CLOCK_DOMAIN, backend=self._backend)
        # Premises for BUILDING->VALID:
        self._new_epoch_observed = False
        # Store-clock stamp of the epoch's birth (the reset transaction);
        # diagnostic only — authoritative old-update rejection is the
        # store's entry-epoch transaction check, never a stamp comparison.
        self._epoch_birth_stamp_ns = 0

    # ------------------------------------------------------------------
    # Reads (also called under the store lock; records are frozen)

    @property
    def record(self):
        return self._record

    @property
    def generation(self):
        return self._record.generation

    @property
    def state(self):
        return self._record.state

    def events(self):
        return tuple(self._events)

    def events_json(self):
        return json.dumps(
            [{"generation": e.generation, "state": e.state, "epoch": e.epoch,
              "reason": e.reason, "reset_id": e.reset_id,
              "transition_mono_ns": e.transition_mono_ns}
             for e in self._events], sort_keys=True)

    # ------------------------------------------------------------------
    # Store hooks (private to the store; all under the transaction lock)

    def _emit(self, state, epoch, reason, reset_id):
        generation = self._record.generation + 1
        mono = int(self._mono_clock_ns())
        stamp = int(self._clock_ns())
        self._record = ValidityRecord(
            state=state, epoch=epoch, generation=generation,
            reason=reason, reset_id=reset_id,
            transition_mono_ns=mono, transition_stamp_ns=stamp,
            map_revision=self._record.map_revision
            if state == STATE_VALID else 0,
            observed_revision=self._record.observed_revision
            if state == STATE_VALID else 0,
            last_integrated_stamp_ns=self._record.last_integrated_stamp_ns
            if state == STATE_VALID else 0,
            geometry_hash=self._record.geometry_hash
            if state == STATE_VALID else "",
            config_digest=self._record.config_digest
            if state == STATE_VALID else "",
            clock_domain=CLOCK_DOMAIN, backend=self._backend)
        self._events.append(ValidityEvent(
            generation=generation, state=state, epoch=epoch, reason=reason,
            reset_id=reset_id, transition_mono_ns=mono))
        while len(self._events) > self._events_cap:
            self._events.pop(0)
        return generation

    def on_boot(self, epoch):
        """Fresh process: BUILDING on a new epoch, generation 1."""
        self._record = ValidityRecord(
            state=STATE_INVALID, epoch="", generation=0, reason="",
            reset_id="", transition_mono_ns=0, transition_stamp_ns=0,
            map_revision=0, observed_revision=0,
            last_integrated_stamp_ns=0, geometry_hash="", config_digest="",
            clock_domain=CLOCK_DOMAIN, backend=self._backend)
        self._new_epoch_observed = False
        self._epoch_birth_stamp_ns = int(self._clock_ns())
        self._emit(STATE_BUILDING, str(epoch), "boot", "")

    def note_reset(self, old_epoch, new_epoch, reset_id, reason):
        """The reset transaction committed: invalidate, then rebuild.

        Emits INVALID(new epoch) followed by BUILDING(new epoch) — the two
        observer transitions, atomically. The generation advances twice, so
        the latched counting matches the observer's commit_reset +
        begin_building pair. Any rebuild premise from the previous epoch is
        dropped (``BUILDING中再次不同reset使前一次重建失效``).
        """
        self._new_epoch_observed = False
        self._epoch_birth_stamp_ns = int(self._clock_ns())
        self._emit(STATE_INVALID, str(new_epoch), str(reason or "explicit"),
                   str(reset_id or ""))
        self._emit(STATE_BUILDING, str(new_epoch), "store_ready",
                   str(reset_id or ""))

    def note_observation(self, identity, acquisition_stamp_ns, now_ns=None):
        """An integrate was ACCEPTED in this epoch (store step 5).

        Records the BUILDING->VALID premises. Only the first accepted
        observation flips ``new_epoch_observed``; later observations still
        refresh ``last_integrated_stamp_ns`` on the record. ``identity`` is
        the freshly built snapshot identity (epoch/revision/digest).
        """
        if identity.epoch != self._record.epoch:
            # Never possible via the store (same lock); guard stays so a
            # future caller cannot corrupt the ledger silently.
            return
        self._new_epoch_observed = True
        stamp = int(acquisition_stamp_ns or 0)
        if stamp > 0:
            self._record = ValidityRecord(
                **{**self._record.as_dict(),
                   "last_integrated_stamp_ns": stamp})
        self._record = ValidityRecord(
            **{**self._record.as_dict(),
               "map_revision": int(identity.revision),
               "observed_revision": int(identity.revision)})

    def observation_diagnostic(self, acquisition_stamp_ns):
        """Diagnostic only: stamp predates this epoch's birth?

        Not a rejection rule — authoritative old-update rejection is the
        store's entry-captured epoch check inside the transaction (same
        stamps, clock rollback and queued requests are covered there; a
        stamp comparison alone is not).
        """
        stamp = int(acquisition_stamp_ns or 0)
        if stamp and self._epoch_birth_stamp_ns:
            return {"stamp_before_epoch_birth":
                    stamp < self._epoch_birth_stamp_ns}
        return {"stamp_before_epoch_birth": None}

    def evaluate_valid(self, identity, geometry_hash, now_ns,
                       integrity_check):
        """Try BUILDING->VALID after an accepted transaction.

        Premises (all must hold; each failure keeps BUILDING):
        1. current state is BUILDING and ``identity.epoch`` is this epoch;
        2. an observation was accepted in this epoch (never a floor prior,
           an empty receipt or a message that merely arrived);
        3. ``integrity_check()`` — the store's explicit snapshot-integrity
           predicate — passes (identity legs complete + query indexes
           rebuilt + projections render);
        4. ``last_integrated_stamp_ns > 0`` and
           ``now_ns - last_integrated_stamp_ns <= max_age_ns`` (same clock
           domain; mirrors the observer's try_publish_valid bound).

        Returns the new state.
        """
        if self._record.state != STATE_BUILDING:
            return self._record.state
        if identity.epoch != self._record.epoch:
            return self._record.state
        if not self._new_epoch_observed:
            return self._record.state
        last = int(self._record.last_integrated_stamp_ns or 0)
        if last <= 0:
            return self._record.state
        if int(now_ns) - last > self._max_age_ns:
            return self._record.state
        if not integrity_check():
            return self._record.state
        self._emit(STATE_VALID, self._record.epoch, "observed",
                   self._record.reset_id)
        # Carry the pinning identity onto the VALID record.
        self._record = ValidityRecord(
            **{**self._record.as_dict(),
               "map_revision": int(identity.revision),
               "observed_revision": int(identity.revision),
               "last_integrated_stamp_ns": last,
               "geometry_hash": str(geometry_hash or ""),
               "config_digest": ""})
        return self._record.state
