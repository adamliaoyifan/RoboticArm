#!/usr/bin/env python3
"""Manual step gate for the active-loading state machine.

The orchestrator owns the pipeline. A front-end that wants to walk the pipeline
one state at a time must not re-implement the states -- that is what produced
the two divergent state machines this module exists to remove. Instead the
front-end holds a gate that the orchestrator consults before every state.

In ``auto`` mode ``wait()`` returns immediately and unconditionally, so an
automated acceptance run behaves exactly as it did before the gate existed.

No ROS imports: the gate is a plain state machine so it can be unit tested
without a roscore.
"""

from __future__ import division

import threading

AUTO = "auto"
MANUAL = "manual"
IDLE_STATE = "Idle"

STEP = "step"
RUN = "run"
RUN_TO = "run_to"
PAUSE = "pause"
ABORT = "abort"
TAINT = "taint"
STATUS = "status"

COMMANDS = (STEP, RUN, RUN_TO, PAUSE, ABORT, TAINT, STATUS)

# How long wait() blocks before re-checking should_abort. Keeps a paused
# orchestrator responsive to Ctrl-C / rospy shutdown.
_POLL_SEC = 0.2


class StepGate(object):
    """Decides whether the orchestrator may execute the next state."""

    def __init__(self, mode=AUTO, breakpoints=None, should_abort=None,
                 poll_sec=_POLL_SEC):
        self._mode = MANUAL if str(mode).lower() == MANUAL else AUTO
        self._breakpoints = set(breakpoints or [])
        self._should_abort = should_abort or (lambda: False)
        self._poll_sec = float(poll_sec)
        self._cv = threading.Condition()

        self._paused = False
        self._paused_state = ""
        self._aborted = False
        self._pause_requested = False
        self._run_free = False
        self._run_to = ""
        self._steps_remaining = 0
        # Grants exactly the state the gate is currently paused at, so
        # releasing a breakpoint does not immediately re-trip it.
        self._released_state = None
        self._probe_touched = False
        self._taint_reasons = []
        self._executed = 0

    @property
    def mode(self):
        return self._mode

    @property
    def probe_touched(self):
        with self._cv:
            return self._probe_touched

    @property
    def taint_reasons(self):
        with self._cv:
            return list(self._taint_reasons)

    def wait(self, state):
        """Block until ``state`` may run. Returns the state, or Idle on abort.

        Returning ``Idle`` lets the caller fall out of its loop through the
        normal terminal path, so an abort can never leave a half-committed
        placement behind.
        """
        if self._mode != MANUAL:
            return state
        with self._cv:
            # A release is only valid for the state it was granted at.
            if self._released_state is not None and (
                    self._released_state != state):
                self._released_state = None
            while True:
                if self._aborted or self._should_abort():
                    self._paused = False
                    return IDLE_STATE
                if self._may_run(state):
                    self._consume(state)
                    self._paused = False
                    self._paused_state = state
                    self._executed += 1
                    return state
                self._paused = True
                self._paused_state = state
                self._cv.wait(self._poll_sec)

    def _may_run(self, state):
        if self._released_state == state:
            return True
        if self._pause_requested:
            return False
        if state in self._breakpoints:
            return False
        if self._run_to and state == self._run_to:
            return False
        if self._steps_remaining > 0:
            return True
        return self._run_free

    def _consume(self, state):
        if self._released_state == state:
            self._released_state = None
        elif self._steps_remaining > 0:
            self._steps_remaining -= 1

    def command(self, command, target_state="", breakpoints=None,
                clear_breakpoints=False, reason=""):
        """Apply a control command. Returns (success, message)."""
        command = str(command or "").strip().lower()
        if command not in COMMANDS:
            return False, "unknown command: %s" % command
        with self._cv:
            if clear_breakpoints:
                self._breakpoints.clear()
            if breakpoints:
                self._breakpoints = set(breakpoints)

            if command == STATUS:
                message = "mode=%s paused=%s" % (self._mode, self._paused)
            elif command == TAINT:
                # Valid in any mode: an out-of-band service call taints the
                # session whether or not anybody is stepping.
                self._probe_touched = True
                if reason:
                    self._taint_reasons.append(str(reason))
                message = "session marked as probe-touched"
            elif self._mode != MANUAL:
                return False, (
                    "orchestrator is in %s mode; start it with "
                    "run_mode:=manual to step it" % self._mode)
            elif command == STEP:
                self._pause_requested = False
                self._run_free = False
                self._run_to = ""
                if self._paused:
                    self._released_state = self._paused_state
                    self._steps_remaining = 0
                else:
                    self._steps_remaining = 1
                message = "step granted"
            elif command == RUN:
                self._pause_requested = False
                self._run_free = True
                self._run_to = ""
                self._steps_remaining = 0
                if self._paused:
                    self._released_state = self._paused_state
                message = "running until a breakpoint or Idle"
            elif command == RUN_TO:
                target = str(target_state or "").strip()
                if not target:
                    return False, "run_to needs target_state"
                self._pause_requested = False
                self._run_free = True
                self._run_to = target
                self._steps_remaining = 0
                if self._paused:
                    self._released_state = self._paused_state
                message = "running to %s" % target
            elif command == PAUSE:
                self._pause_requested = True
                self._run_free = False
                self._run_to = ""
                self._steps_remaining = 0
                self._released_state = None
                message = "will pause before the next state"
            else:  # ABORT
                self._aborted = True
                message = "aborting at the next state boundary"
            self._cv.notify_all()
            return True, message

    def snapshot(self):
        with self._cv:
            return {
                "mode": self._mode,
                "paused": self._paused,
                "paused_state": self._paused_state,
                "aborted": self._aborted,
                "run_free": self._run_free,
                "run_to": self._run_to,
                "steps_remaining": self._steps_remaining,
                "breakpoints": sorted(self._breakpoints),
                "probe_touched": self._probe_touched,
                "taint_reasons": list(self._taint_reasons),
                "states_executed": self._executed,
            }
