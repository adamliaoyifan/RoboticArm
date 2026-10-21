"""Single-transit observation policy, separate from placement/control policy.

Only explicitly installed by the diagnostic entry point. Never releases the
payload, clears the scene, changes gates, or submits an extra holding goal.
"""
import os
import time

HOLD_NS = 60 * 10**9


def _capture_trigger_status(reason):
    """Notify the joint-command capture of a diagnostic outcome event.

    Uses the unified ctl trigger protocol (same numbering/idempotency as the
    manual service). Observation only: a failure here is reported in the
    trial extras and never blocks the diagnostic exit path. Inert unless
    JOINT_CAPTURE_DIR/JOINT_CAPTURE_MODE select an active capture.
    """
    run_dir = os.environ.get("JOINT_CAPTURE_DIR", "").strip()
    mode = os.environ.get("JOINT_CAPTURE_MODE", "").strip()
    if not run_dir or mode not in ("armed", "window"):
        return "inactive"
    try:
        from joint_command_capture.backends import (allocate_event_id,
                                                    write_trigger)
        write_trigger(run_dir, allocate_event_id(run_dir), reason)
        return "delivered"
    except Exception as exc:  # diagnostic side-channel only; never block exit
        return f"failed:{type(exc).__name__}"


class TransitCaptureExit:
    def __init__(self, *, healthy, now_ns, sleep=time.sleep,
                 monotonic=time.monotonic, hold_wall_limit=300.):
        self.healthy, self.now_ns, self.sleep = healthy, now_ns, sleep
        self.monotonic, self.hold_wall_limit = monotonic, hold_wall_limit
        self.attempts = 0

    def before(self, name):
        if self.attempts:
            return 'DIAGNOSTIC_NEXT_LEG_REFUSED'
        if name != 'transit':
            return 'DIAGNOSTIC_FIRST_LEG_NOT_TRANSIT'
        if not self.healthy():
            return 'DIAGNOSTIC_CAPTURE_UNHEALTHY'
        self.attempts += 1
        return ''

    def finish(self, driver, trial, ok, code, extras):
        if not ok:
            trial.extras['capture_trigger'] = _capture_trigger_status(
                f'leg_fail:{code}')
            return driver._abort(trial, code, True, extra=extras)
        start, wall = self.now_ns(), self.monotonic()
        last = start
        trial.extras['diagnostic_hold'] = dict(start_ns=start, required_ns=HOLD_NS,
                                              submitted_legs=self.attempts)
        driver._set_state('DIAGNOSTIC_HOLD')
        reason = ''
        while True:
            now = self.now_ns()
            held, payload_reason = driver._carrying_vacuum_holds()
            if not self.healthy():
                reason = 'DIAGNOSTIC_CAPTURE_UNHEALTHY'
            elif not held:
                reason = payload_reason
            elif now < last:
                reason = 'DIAGNOSTIC_CLOCK_BACKWARDS'
            elif self.monotonic() - wall >= self.hold_wall_limit:
                reason = 'DIAGNOSTIC_HOLD_CLOCK_TIMEOUT'
            trial.extras['diagnostic_hold'].update(end_ns=now, elapsed_ns=now-start,
                                                   reason=reason)
            if reason:
                trial.extras['capture_trigger'] = _capture_trigger_status(
                    f'hold_fail:{reason}')
                return driver._abort(trial, reason, True, extra=extras)
            if now - start >= HOLD_NS:
                break
            last = now
            self.sleep(.02)
        trial.extras['capture_trigger'] = _capture_trigger_status(
            'diagnostic_complete_carrying')
        # Explicit diagnostic state, not a placement success or commit.
        driver._set_state('DIAGNOSTIC_COMPLETE_CARRYING')
        trial.place_state = 'DIAGNOSTIC_COMPLETE_CARRYING'
        trial.extras['diagnostic_complete'] = True
        driver._dump_place(trial, extra=extras)
        return trial
