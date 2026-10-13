#!/usr/bin/env python3
"""plan_motion should_abort cancel chain (ST09-I S5.7-6 评审点 3 / G0-4).

The vacuum admission window closes the driver-check -> controller-commit
gap; this cancel chain closes BOTH the commit -> result gap (payload
loss mid-execution: cancel, WAIT for the controller's terminal receipt,
named CARRY_FAULT_PAYLOAD_LOST refusal — a receipt that never lands is
cancel_status=TIMEOUT, never a resume) AND the send -> acceptance gap
([G0-4] payload loss while pending: a late-arriving acceptance is
resolved and CANCELED, a late server rejection is named, and a goal
that never resolves is a named TIMEOUT — "future not done" is never
treated as "server did not accept"). Fail-closed either way, zero
controller submissions after the refusal.
"""
import os
import sys

import pytest

_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
for entry in ("src/luggage_planning",):
    path = os.path.join(_ROOT, entry)
    if path not in sys.path:
        sys.path.append(path)

pytest.importorskip("luggage_msgs.action")

from luggage_planning.typed_driver_adapter import (  # noqa: E402
    TypedDriverAdapter,
)


class _Runtime(object):
    def require_graph_ready(self):
        return None


class _PendingFuture(object):
    """A goal-accept future that never completes."""

    def done(self):
        return False


class _AcceptedFuture(object):
    def __init__(self, handle):
        self._handle = handle

    def done(self):
        return True

    def result(self):
        return self._handle


class _LateAcceptFuture(object):
    """[G0-4] A goal future that resolves a few polls AFTER the abort
    decision: the server accepted late — without the cancel chain the
    handle would execute after the caller walked away."""

    def __init__(self, handle, resolve_after=2):
        self._handle = handle
        self._remaining = int(resolve_after)

    def done(self):
        if self._remaining > 0:
            self._remaining -= 1
            return False
        return True

    def result(self):
        return self._handle


class _RejectedHandle(object):
    """The server's late answer is a refusal: nothing to run."""

    accepted = False
    cancel_calls = 0

    def cancel_goal(self):
        self.cancel_calls += 1


class _ResultFuture(object):
    """Completes only when the fake controller issues its terminal status."""

    def __init__(self):
        self._outcome = None

    def done(self):
        return self._outcome is not None

    def finish(self, outcome):
        self._outcome = outcome

    def result(self):
        return self._outcome


class _Handle(object):
    accepted = True

    def __init__(self, result_future, cancel_outcome):
        self._result_future = result_future
        self._cancel_outcome = cancel_outcome
        self.cancel_calls = 0

    def get_result_async(self):
        return self._result_future

    def cancel_goal(self):
        self.cancel_calls += 1
        if self._cancel_outcome is not None:
            self._result_future.finish(self._cancel_outcome)


class _PlanClient(object):
    def __init__(self, goal_future, handle):
        self._goal_future = goal_future
        self._handle = handle
        self.sent = []

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal):
        self.sent.append(goal)
        return self._goal_future


def _adapter(client):
    adapter = TypedDriverAdapter.__new__(TypedDriverAdapter)
    adapter.runtime = _Runtime()
    adapter._plan = client
    return adapter


def _outcome(status, message="canceled by client"):
    from types import SimpleNamespace

    return SimpleNamespace(
        status=int(status),
        result=SimpleNamespace(success=False, message=message))


def _segment():
    """A real (empty) MotionSegmentV2 — the goal setter type-checks."""
    from luggage_msgs.msg import MotionSegmentV2

    return MotionSegmentV2()


def test_goal_wait_abort_cancels_the_late_accepted_goal():
    """[G0-4] Abort while pending, acceptance arrives LATE: the adapter
    resolves the handle, cancels it, consumes the terminal receipt, and
    the named refusal carries cancel_status — the late acceptance never
    executes, no subsequent leg starts."""
    result_future = _ResultFuture()
    handle = _Handle(result_future, cancel_outcome=_outcome(5))
    client = _PlanClient(_LateAcceptFuture(handle), handle)
    adapter = _adapter(client)
    ok, message, result = adapter.plan_motion(
        _segment(), 5.0, should_abort=lambda: True)
    assert not ok
    assert message == "CARRY_FAULT_PAYLOAD_LOST cancel_status=CANCELED"
    assert handle.cancel_calls == 1
    assert result is not None and not result.success
    assert client.sent, "the goal future was polled"


def test_goal_wait_abort_late_server_rejection_is_named():
    """[G0-4] The server's late answer is a refusal: the goal never
    runs, nothing to cancel, the refusal names the rejection."""
    handle = _RejectedHandle()
    client = _PlanClient(_LateAcceptFuture(handle), handle)
    adapter = _adapter(client)
    ok, message, result = adapter.plan_motion(
        _segment(), 5.0, should_abort=lambda: True)
    assert not ok and result is None
    assert message == "CARRY_FAULT_PAYLOAD_LOST server_rejected"
    assert handle.cancel_calls == 0


def test_goal_wait_abort_unresolved_goal_is_named_timeout():
    """[G0-4] A goal that never resolves (server dead mid-handshake) is
    NOT silently treated as never-sent: the adapter holds to the
    caller's deadline and returns the named TIMEOUT refusal."""
    handle = _Handle(_ResultFuture(), cancel_outcome=None)
    client = _PlanClient(_PendingFuture(), handle)
    adapter = _adapter(client)
    ok, message, result = adapter.plan_motion(
        _segment(), 0.3, should_abort=lambda: True)
    assert not ok and result is None
    assert message == (
        "CARRY_FAULT_PAYLOAD_LOST cancel_status=TIMEOUT goal_unresolved")
    assert handle.cancel_calls == 0
    assert client.sent, "the goal future was polled"


def test_result_wait_abort_cancels_and_consumes_receipt():
    """Abort mid-execution: cancel_goal is called, the controller's
    terminal receipt is consumed, and the refusal carries cancel_status
    (rclpy goal status 5 = CANCELED)."""
    result_future = _ResultFuture()
    handle = _Handle(result_future, cancel_outcome=_outcome(5))
    client = _PlanClient(_AcceptedFuture(handle), handle)
    adapter = _adapter(client)
    polls = {"n": 0}

    def should_abort():
        polls["n"] += 1
        return polls["n"] > 1

    ok, message, result = adapter.plan_motion(
        _segment(), 5.0, should_abort=should_abort)
    assert not ok
    assert message == "CARRY_FAULT_PAYLOAD_LOST cancel_status=CANCELED"
    assert handle.cancel_calls == 1
    assert result is not None and not result.success


def test_result_wait_abort_without_receipt_stays_closed():
    """A receipt that never lands is cancel_status=TIMEOUT: the refusal
    stands (fail-closed), never a resume."""
    result_future = _ResultFuture()
    handle = _Handle(result_future, cancel_outcome=None)
    client = _PlanClient(_AcceptedFuture(handle), handle)
    adapter = _adapter(client)
    ok, message, result = adapter.plan_motion(
        _segment(), 0.4, should_abort=lambda: True)
    assert not ok and result is None
    assert message == "CARRY_FAULT_PAYLOAD_LOST cancel_status=TIMEOUT"
    assert handle.cancel_calls == 1


def test_abort_status_aborted_is_named_too():
    """A controller that ABORTS on cancel (status 6) is still a
    consumed terminal receipt — the refusal names it."""
    result_future = _ResultFuture()
    handle = _Handle(result_future, cancel_outcome=_outcome(6))
    client = _PlanClient(_AcceptedFuture(handle), handle)
    adapter = _adapter(client)
    ok, message, _result = adapter.plan_motion(
        _segment(), 5.0, should_abort=lambda: True)
    assert not ok
    assert message == "CARRY_FAULT_PAYLOAD_LOST cancel_status=ABORTED"


def test_without_abort_the_chain_is_unchanged():
    """No should_abort: the pre-existing wait semantics hold (result
    consumed on completion, no cancel)."""
    result_future = _ResultFuture()
    result_future.finish(_outcome(4, "done"))
    handle = _Handle(result_future, cancel_outcome=None)
    client = _PlanClient(_AcceptedFuture(handle), handle)
    adapter = _adapter(client)
    ok, message, result = adapter.plan_motion(_segment(), 5.0)
    # outcome status 4 = SUCCEEDED but the business result.success is
    # False here — the adapter reports the business verdict verbatim.
    assert not ok and result is result_future.result().result
    assert handle.cancel_calls == 0
    assert message == "done"
