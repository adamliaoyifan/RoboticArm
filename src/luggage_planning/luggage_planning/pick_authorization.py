#!/usr/bin/env python3
"""ROS-free pick authorization: measured FULL_3D or wait / fail closed.

TOP_ONLY is a valid DetectLuggage result (cargo present, height unknown).
It never authorizes a grasp, a pickup MoveIt AABB, or ComputePlacement.
Configured and catalog height sources never authorize either.

``hold_track`` (and the ``DETECT_SUPPORT_STAMP_MISMATCH`` gate that names
it) is continuity output from an earlier cargo cloud. It is not a fresh
measurement, so it cannot authorize a pick and it does not consume the
attempt budget. The wait still ends at ``max_elapsed_sec`` with
``DETECT_FULL_GEOMETRY_REQUIRED``.

No ROS, no numpy. Elapsed budget is supplied by the caller so tests can
drive a fake clock. Drivers treat REOBSERVE as TERMINAL until named
recovery poses exist (ACTIVE-VIEW-2 follow-on).
"""

from __future__ import division

import re
from dataclasses import dataclass, field

# Match luggage_msgs/DetectedLuggage.msg (kept numeric so this module
# stays ROS-free).
HEIGHT_SOURCE_UNAVAILABLE = 0
HEIGHT_SOURCE_MEASURED_SUPPORT = 1
HEIGHT_SOURCE_CONFIGURED_SUPPORT = 2
HEIGHT_SOURCE_CATALOG_PRIOR = 3

AUTHORIZE = "AUTHORIZE"
WAIT = "WAIT"
REOBSERVE = "REOBSERVE"
TERMINAL = "TERMINAL"

DETECT_FULL_GEOMETRY_REQUIRED = "DETECT_FULL_GEOMETRY_REQUIRED"
DETECT_FAILED = "DETECT_FAILED"
WAIT_FOR_FULL_3D = "WAIT_FOR_FULL_3D"

HOLD_TRACK_GATE = "hold_track"
HOLD_TRACK_SUPPORT = "DETECT_SUPPORT_STAMP_MISMATCH"

# The wait is the elapsed window. Samples are one detect plus one per
# wait period, so the attempt cap must not close the window early:
# 5 attempts at 0.5 s ended at 2 s while max_elapsed_sec was already 5.
DEFAULT_WAIT_PERIOD_SEC = 0.5
DEFAULT_MAX_ELAPSED_SEC = 5.0
DEFAULT_MAX_ATTEMPTS = int(round(
    DEFAULT_MAX_ELAPSED_SEC / DEFAULT_WAIT_PERIOD_SEC)) + 1

_FORBIDDEN_SOURCES = (
    HEIGHT_SOURCE_CONFIGURED_SUPPORT,
    HEIGHT_SOURCE_CATALOG_PRIOR,
)


@dataclass(frozen=True)
class AuthorizationConfig:
    max_attempts: int = DEFAULT_MAX_ATTEMPTS
    max_elapsed_sec: float = DEFAULT_MAX_ELAPSED_SEC
    wait_period_sec: float = DEFAULT_WAIT_PERIOD_SEC

    def __post_init__(self):
        if int(self.max_attempts) < 1:
            raise ValueError("max_attempts must be >= 1")
        if float(self.max_elapsed_sec) <= 0.0:
            raise ValueError("max_elapsed_sec must be > 0")
        if float(self.wait_period_sec) < 0.0:
            raise ValueError("wait_period_sec must be >= 0")


@dataclass(frozen=True)
class AuthorizationDecision:
    action: str
    reason: str
    attempts: int
    elapsed_sec: float

    def to_dict(self):
        return {
            "action": self.action,
            "reason": self.reason,
            "attempts": int(self.attempts),
            "elapsed_sec": float(self.elapsed_sec),
        }


@dataclass(frozen=True)
class AuthorizationLoopResult:
    decision: AuthorizationDecision
    box: object
    detect_ok: bool
    detect_reason: str
    trace: tuple = field(default_factory=tuple)
    attempts: int = 0
    hold_track_skipped: int = 0
    first_measure_latency_sec: object = None
    last_gate: str = ""
    last_measure_age_sec: object = None

    @property
    def authorized(self):
        return self.decision.action == AUTHORIZE

    def to_dict(self):
        return {
            "decision": self.decision.to_dict(),
            "detect_ok": bool(self.detect_ok),
            "detect_reason": str(self.detect_reason or ""),
            "authorized": bool(self.authorized),
            "attempts": int(self.attempts),
            "hold_track_skipped": int(self.hold_track_skipped),
            "first_measure_latency_sec": self.first_measure_latency_sec,
            "last_gate": str(self.last_gate or ""),
            "last_measure_age_sec": self.last_measure_age_sec,
            "trace": [dict(row) for row in self.trace],
        }


def _height_source(box):
    if box is None:
        return HEIGHT_SOURCE_UNAVAILABLE
    try:
        return int(getattr(box, "height_source", HEIGHT_SOURCE_UNAVAILABLE) or 0)
    except (TypeError, ValueError):
        return HEIGHT_SOURCE_UNAVAILABLE


def _diag_fields(text):
    """``key=value`` pairs from a DetectLuggage message. ``n/a`` is absent."""
    found = {}
    for key, value in re.findall(r"([A-Za-z_]+)=([^,\)]*)", str(text or "")):
        token = value.strip()
        if token in ("", "n/a", "None"):
            continue
        found[key] = token
    return found


def _attr_or_diag(box, attr, diag, key):
    value = str(getattr(box, attr, "") or "")
    if value in ("", "n/a", "None"):
        value = str(diag.get(key) or "")
    if value in ("", "n/a", "None"):
        return ""
    return value


def observation_gate(box, detect_reason=""):
    """Gate label for one detect sample. ``hold_track`` is continuity."""
    diag = _diag_fields(detect_reason)
    gate = _attr_or_diag(box, "support_gate", diag, "gate")
    source = _attr_or_diag(box, "pca_source", diag, "source")
    if not source:
        source = _attr_or_diag(box, "source", diag, "pca_source")
    support = _attr_or_diag(box, "support_reason", diag, "support")
    if source == HOLD_TRACK_GATE or gate == HOLD_TRACK_GATE:
        return HOLD_TRACK_GATE
    if support == HOLD_TRACK_SUPPORT:
        return HOLD_TRACK_GATE
    if gate:
        return gate
    if source:
        return source
    return support


def is_continuity_hold(box, detect_reason=""):
    """True when this sample is a held cargo cloud, not a new measurement."""
    return observation_gate(box, detect_reason) == HOLD_TRACK_GATE


def is_fresh_measure(detect_ok, box, detect_reason=""):
    """A non-hold sample the estimator actually measured this acquisition."""
    if not detect_ok or box is None or is_continuity_hold(box, detect_reason):
        return False
    source = _attr_or_diag(
        box, "pca_source", _diag_fields(detect_reason), "source")
    if not source:
        source = str(getattr(box, "source", "") or "")
    if source:
        return source == "measure"
    return bool(getattr(box, "height_valid", False))


def authorized_pick_boundary(pick_fields, *, authorized, detect_reason,
                             observation_gate_name, selected_frame,
                             scene_size):
    """Join the authorizing sample to the size that entered the scene.

    ``pick_fields`` is the detection authorization returned. ``selected_frame``
    is the detector window row marked selected, including that frame's own
    ``size_wdh`` and ``pca_source``. ``scene_size`` is the AABB taken from
    the same message. This records whether those three agree. It does not
    decide why a size is wrong, and it does not replace a size with a catalog
    value.
    """
    size = [float(value) for value in (pick_fields.get("size_wdh") or [])]
    scene = [float(value) for value in (scene_size or [])]
    frame = dict(selected_frame or {})
    frame_size = frame.get("size_wdh")
    stamp = pick_fields.get("stamp_sec")
    frame_stamp = frame.get("stamp")

    def _same(left, right):
        if not isinstance(left, list) or not isinstance(right, list):
            return None
        if len(left) != 3 or len(right) != 3:
            return None
        return all(abs(float(left[i]) - float(right[i])) <= 1e-6
                   for i in range(3))

    stamp_matches = None
    if stamp is not None and frame_stamp is not None:
        stamp_matches = abs(float(stamp) - float(frame_stamp)) < 1e-3
    return {
        "authorized": bool(authorized),
        "detect_reason": str(detect_reason or ""),
        "observation_gate": str(observation_gate_name or ""),
        "pick": dict(pick_fields),
        "selected_frame": frame,
        "scene_size_wdh": scene,
        "scene_size_matches_pick": _same(size, scene),
        "selected_frame_size_matches_pick": _same(
            size, list(frame_size) if isinstance(frame_size, list) else None),
        "selected_frame_stamp_matches_pick": stamp_matches,
        "authorizing_message_has_pca_source": bool(
            pick_fields.get("pca_source")),
    }


def pick_authorized(box, detect_reason=""):
    """True only for a measured top plus measured support plane.

    Continuity output never qualifies, even if height fields were copied
    forward from an older measure.
    """
    if box is None or is_continuity_hold(box, detect_reason):
        return False
    return (
        bool(getattr(box, "top_surface_valid", False))
        and bool(getattr(box, "height_valid", False))
        and _height_source(box) == HEIGHT_SOURCE_MEASURED_SUPPORT
    )


class PickAuthorizationPolicy:
    """Decide AUTHORIZE / WAIT / REOBSERVE / TERMINAL for one detect result."""

    def __init__(self, config=None):
        self.config = config or AuthorizationConfig()

    def _budget_left(self, scored_attempts, elapsed_sec):
        cfg = self.config
        return (int(scored_attempts) < int(cfg.max_attempts)
                and float(elapsed_sec) < float(cfg.max_elapsed_sec))

    def evaluate(self, detect_ok, box, elapsed_sec, attempts,
                 detect_reason="", scored_attempts=None):
        elapsed = float(elapsed_sec)
        n = int(attempts)
        if n < 1:
            raise ValueError("attempts must be >= 1")
        # Continuity frames are not failure samples. Callers that omit
        # scored_attempts keep the historical "every call counts" budget.
        if scored_attempts is None:
            scored = n if not is_continuity_hold(box, detect_reason) else n - 1
        else:
            scored = int(scored_attempts)
        if pick_authorized(box, detect_reason):
            return AuthorizationDecision(AUTHORIZE, "ok", n, elapsed)
        if (box is not None
                and not is_continuity_hold(box, detect_reason)
                and _height_source(box) in _FORBIDDEN_SOURCES):
            return AuthorizationDecision(
                TERMINAL, DETECT_FULL_GEOMETRY_REQUIRED, n, elapsed)
        if self._budget_left(scored, elapsed):
            return AuthorizationDecision(WAIT, WAIT_FOR_FULL_3D, n, elapsed)
        if bool(detect_ok) and box is not None:
            return AuthorizationDecision(
                REOBSERVE, DETECT_FULL_GEOMETRY_REQUIRED, n, elapsed)
        reason = str(detect_reason or "") or DETECT_FAILED
        return AuthorizationDecision(TERMINAL, reason, n, elapsed)


def run_authorization_loop(detect_fn, sleep_fn, monotonic_fn, policy=None):
    """Call *detect_fn* until AUTHORIZE or a terminal action.

    *detect_fn()* returns ``(detect_ok, box, detect_reason)``.
    *sleep_fn(seconds)* and *monotonic_fn()* are injected so tests do not
    wait on the wall clock. Drivers pass ``time.sleep`` / ``time.monotonic``.
    """
    policy = policy or PickAuthorizationPolicy()
    t0 = float(monotonic_fn())
    trace = []
    detect_ok = False
    box = None
    detect_reason = ""
    attempts = 0
    scored = 0
    skipped = 0
    first_measure = None
    last_measure_elapsed = None
    last_gate = ""
    while True:
        detect_ok, box, detect_reason = detect_fn()
        attempts += 1
        elapsed = float(monotonic_fn()) - t0
        held = is_continuity_hold(box, detect_reason)
        gate = observation_gate(box, detect_reason)
        if gate:
            last_gate = gate
        if held:
            skipped += 1
        else:
            scored += 1
        if is_fresh_measure(detect_ok, box, detect_reason):
            if first_measure is None:
                first_measure = elapsed
            last_measure_elapsed = elapsed
        decision = policy.evaluate(
            detect_ok, box, elapsed, attempts, detect_reason,
            scored_attempts=scored if not held else scored)
        trace.append(decision.to_dict())
        if decision.action == WAIT:
            sleep_fn(float(policy.config.wait_period_sec))
            continue
        age = None
        if last_measure_elapsed is not None:
            age = float(elapsed) - float(last_measure_elapsed)
        return AuthorizationLoopResult(
            decision=decision,
            box=box,
            detect_ok=bool(detect_ok),
            detect_reason=str(detect_reason or ""),
            trace=tuple(trace),
            attempts=int(attempts),
            hold_track_skipped=int(skipped),
            first_measure_latency_sec=first_measure,
            last_gate=str(last_gate or ""),
            last_measure_age_sec=age,
        )
