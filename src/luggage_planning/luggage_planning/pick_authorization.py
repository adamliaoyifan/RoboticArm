#!/usr/bin/env python3
"""ROS-free pick authorization: measured FULL_3D or wait / fail closed.

TOP_ONLY is a valid DetectLuggage result (cargo present, height unknown).
It never authorizes a grasp, a pickup MoveIt AABB, or ComputePlacement.
Configured and catalog height sources never authorize either.

No ROS, no numpy. Elapsed budget is supplied by the caller so tests can
drive a fake clock. Drivers treat REOBSERVE as TERMINAL until named
recovery poses exist (ACTIVE-VIEW-2 follow-on).
"""

from __future__ import division

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

# Detector estimate_retry_count=4 at 0.25 s plus settle margin. Stays
# well under typical DetectLuggage timeouts (15-20 s).
DEFAULT_MAX_ATTEMPTS = 5
DEFAULT_MAX_ELAPSED_SEC = 5.0
DEFAULT_WAIT_PERIOD_SEC = 0.5

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

    @property
    def authorized(self):
        return self.decision.action == AUTHORIZE

    def to_dict(self):
        return {
            "decision": self.decision.to_dict(),
            "detect_ok": bool(self.detect_ok),
            "detect_reason": str(self.detect_reason or ""),
            "authorized": bool(self.authorized),
            "trace": [dict(row) for row in self.trace],
        }


def _height_source(box):
    if box is None:
        return HEIGHT_SOURCE_UNAVAILABLE
    try:
        return int(getattr(box, "height_source", HEIGHT_SOURCE_UNAVAILABLE) or 0)
    except (TypeError, ValueError):
        return HEIGHT_SOURCE_UNAVAILABLE


def pick_authorized(box):
    """True only for a measured top plus measured support plane."""
    if box is None:
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

    def _budget_left(self, attempts, elapsed_sec):
        cfg = self.config
        return (int(attempts) < int(cfg.max_attempts)
                and float(elapsed_sec) < float(cfg.max_elapsed_sec))

    def evaluate(self, detect_ok, box, elapsed_sec, attempts,
                 detect_reason=""):
        elapsed = float(elapsed_sec)
        n = int(attempts)
        if n < 1:
            raise ValueError("attempts must be >= 1")
        if pick_authorized(box):
            return AuthorizationDecision(AUTHORIZE, "ok", n, elapsed)
        if box is not None and _height_source(box) in _FORBIDDEN_SOURCES:
            return AuthorizationDecision(
                TERMINAL, DETECT_FULL_GEOMETRY_REQUIRED, n, elapsed)
        if self._budget_left(n, elapsed):
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
    while True:
        detect_ok, box, detect_reason = detect_fn()
        attempts += 1
        elapsed = float(monotonic_fn()) - t0
        decision = policy.evaluate(
            detect_ok, box, elapsed, attempts, detect_reason)
        trace.append(decision.to_dict())
        if decision.action == WAIT:
            sleep_fn(float(policy.config.wait_period_sec))
            continue
        return AuthorizationLoopResult(
            decision=decision,
            box=box,
            detect_ok=bool(detect_ok),
            detect_reason=str(detect_reason or ""),
            trace=tuple(trace),
        )
