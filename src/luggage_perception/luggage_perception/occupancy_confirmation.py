#!/usr/bin/env python3
"""Pending occupancy changes and the one transition that confirms them.

ROS-free. The cargo mapper owns one ``PendingConfirmation``.

A change event appends a record with a monotonic token, the clock time of
the change and, for a placed box, its committed footprint. The map is
confirmed when no record is pending.

Contract scope. Only a placed box can be confirmed. A record is cleared by
``settle`` for an integration that:

1. began after the change (``token <= token_at_start``), so an integration
   that was already running when a new change arrived cannot clear it;
2. used a frame acquired strictly after the change;
3. was settled; and
4. itself observed the new box: at least ``MIN_EVIDENCE_POINTS`` of that
   view's own points lie on the upper half of the committed box. The
   accumulated map is never consulted, so an old sensor cell inside the
   footprint, or the box underneath in a stack, is not evidence.

This proves the box is present at its committed pose. It does not
measure the box's full geometry.

Removal, disturbance, hand edits and container pose changes have no
observation rule yet (they would need "observed free" over the affected
region). They stay pending, fail closed, until ``clear`` (reset or return
to the static empty container). They are recorded but not confirmable.

With ``require_confirmation`` False nothing is ever recorded and the map
is always confirmed. That is the default for flows that never integrate a
view after a placement.
"""

import threading

from luggage_perception.cargo_view_integration import (
    MIN_EVIDENCE_POINTS,
    NON_INVALIDATING_EVENTS,
    OCCUPANCY_CHANGE_EVENTS,
    new_box_evidence_points,
)


class PendingConfirmation(object):

    def __init__(self, require_confirmation):
        self._require = bool(require_confirmation)
        self._lock = threading.Lock()
        self._token = 0
        self._changes = []
        self._last_settle = {}

    @property
    def require_confirmation(self):
        return self._require

    def record(self, event, stamp_ns, footprint=None):
        """Register a change; return its token (0 when the gate is off).

        ``footprint`` is ``(center, size, yaw)`` in the container frame.
        """
        if event in NON_INVALIDATING_EVENTS:
            return 0
        if event not in OCCUPANCY_CHANGE_EVENTS:
            raise ValueError("unknown occupancy event %r" % (event,))
        if not self._require:
            return 0
        with self._lock:
            self._token += 1
            self._changes.append({
                "token": self._token,
                "event": str(event),
                "stamp_ns": int(stamp_ns or 0),
                "footprint": None if footprint is None else (
                    [float(v) for v in footprint[0]],
                    [float(v) for v in footprint[1]],
                    float(footprint[2])),
            })
            return self._token

    def begin(self):
        """Token to carry through one integration (read before it starts)."""
        with self._lock:
            return self._token

    def settle(self, token_at_start, frame_stamp_ns, observed_points,
               settled):
        """Clear what this integration confirms; return the cleared tokens.

        ``observed_points`` are the points this integration itself
        observed, in the container frame.
        """
        frame = int(frame_stamp_ns or 0)
        cleared = []
        evidence = {}
        with self._lock:
            for change in list(self._changes):
                if change["token"] > int(token_at_start):
                    continue
                if not settled or frame <= 0 or frame <= change["stamp_ns"]:
                    continue
                footprint = change["footprint"]
                if footprint is None:
                    continue  # no observation rule for this event yet
                count = new_box_evidence_points(
                    observed_points, footprint[0], footprint[1],
                    footprint[2])
                evidence[change["token"]] = count
                if count < MIN_EVIDENCE_POINTS:
                    continue
                self._changes.remove(change)
                cleared.append(change["token"])
            # Boundary trace for the evidence bundle: what the last
            # integration could and could not confirm, and why.
            self._last_settle = {
                "token_at_start": int(token_at_start),
                "frame_stamp_ns": frame,
                "settled": bool(settled),
                "evidence_points": evidence,
                "min_evidence_points": int(MIN_EVIDENCE_POINTS),
                "cleared": list(cleared),
            }
        return cleared

    def settle_observed_free(self, surface):
        """Clear removal records whose footprint a fresh view observed FREE.

        [B2] Positive free evidence only: every footprint column must be
        state ``free`` in the post-integration grid (a miss raycast
        through the region). Occluded or unknown columns keep the record
        pending — "did not see" is not free-space evidence. Returns the
        cleared tokens.
        """
        from luggage_perception.cargo_view_integration import (
            footprint_free_coverage,
            rotated_rect_aabb,
        )
        cleared = []
        evidence = {}
        with self._lock:
            for change in list(self._changes):
                if change["event"] != "box_removed":
                    continue
                footprint = change["footprint"]
                if footprint is None:
                    continue
                center, size, yaw = footprint
                aabb = rotated_rect_aabb(
                    (center[0], center[1]),
                    float(size[0]), float(size[1]), float(yaw))
                fraction, free_count, total = footprint_free_coverage(
                    surface, aabb)
                evidence[change["token"]] = {
                    "free_columns": free_count,
                    "total_columns": total,
                }
                if total > 0 and free_count == total:
                    self._changes.remove(change)
                    cleared.append(change["token"])
            if cleared or evidence:
                record = dict(self._last_settle)
                record["observed_free"] = {
                    "evidence": evidence,
                    "cleared": list(cleared),
                }
                self._last_settle = record
        return cleared

    def settle_matching(self, event, center, size, yaw, tolerance=0.10,
                        evidence=None):
        """Clear pending records of ``event`` whose footprint matches the
        given box (center within ``tolerance``).

        [release-failure contract] Used when a measurement itself is the
        confirming evidence (an accepted, anchored commit proves the box
        occupies its commanded pose — the anchor IS the observation), so
        a ``box_released`` fact resolves without waiting for a camera
        view. ``evidence`` rides into ``last_settle`` for the audit trail.
        """
        cleared = []
        with self._lock:
            for change in list(self._changes):
                if change["event"] != str(event):
                    continue
                footprint = change["footprint"]
                if footprint is None:
                    continue
                delta = sum(
                    (float(center[axis]) - float(footprint[0][axis])) ** 2
                    for axis in range(3)) ** 0.5
                if delta > float(tolerance):
                    continue
                self._changes.remove(change)
                cleared.append(change["token"])
            if cleared:
                record = dict(self._last_settle)
                record["settle_matching"] = {
                    "event": str(event),
                    "cleared": list(cleared),
                    "evidence": dict(evidence or {}),
                }
                self._last_settle = record
        return cleared

    def clear(self):
        with self._lock:
            self._changes = []

    @property
    def confirmed(self):
        with self._lock:
            return not self._changes

    def surface_fields(self):
        """Fields the mapper publishes on its surface."""
        with self._lock:
            pending = [dict(c) for c in self._changes]
            token = self._token
            last_settle = dict(self._last_settle)
        return {
            "occupancy_confirmed": not pending,
            "change_event": pending[-1]["event"] if pending else "",
            "change_token": token,
            "last_settle": last_settle,
            "pending_changes": [
                {"token": c["token"], "event": c["event"],
                 "stamp_ns": c["stamp_ns"],
                 "confirmable": c["footprint"] is not None}
                for c in pending],
        }
