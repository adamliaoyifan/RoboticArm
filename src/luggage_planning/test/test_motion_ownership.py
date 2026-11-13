"""Ownership behavior under concurrent requests and uncertain transports."""
from concurrent.futures import Future
import threading
from types import SimpleNamespace

import pytest

from luggage_planning.motion.motion_ownership import MotionOwnership, OwnedActionClient, run_legacy_motion


class Controller:
    def __init__(self, *, accepted=True):
        self.acceptance = Future()
        self.result = Future()
        self.accepted = accepted
        self.cancels = 0
        self.requests = []

    def send_goal_async(self, goal, **kwargs):
        self.requests.append((goal, kwargs))
        return self.acceptance

    def accept(self):
        self.acceptance.set_result(self)

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancels += 1
        return Future()  # Even an unreturned cancel ack is not terminal.

    def finish(self, status=4):
        self.result.set_result(SimpleNamespace(status=status))


def dispatch(owner, controller):
    return OwnedActionClient(controller, owner, '/controller').send_goal_async(object())


def assert_held(owner):
    assert owner.motion_lock.locked()
    assert owner.begin('second legacy', legacy=True) is None
    assert owner.begin('second operation') is None


def test_concurrent_callback_admission_and_terminal_release():
    owner = MotionOwnership()
    entered, leave = threading.Event(), threading.Event()
    handle = SimpleNamespace(abort=lambda: None)
    def callback(_):
        entered.set()
        assert leave.wait(2.)
        return SimpleNamespace(success=True)
    outputs = []
    thread = threading.Thread(target=lambda: outputs.append(
        run_legacy_motion(owner, handle, SimpleNamespace, 'first', callback)))
    thread.start()
    try:
        assert entered.wait(1.)
        other = run_legacy_motion(owner, handle, SimpleNamespace, 'second', callback)
        assert not other.success and other.message == 'SHARED_MOTION_BUSY'
    finally:
        leave.set()
        thread.join(2.)
    assert not thread.is_alive() and outputs[0].success
    assert owner.snapshot()['active'] is None and not owner.motion_lock.locked()


@pytest.mark.parametrize('late', [False, True])
@pytest.mark.parametrize('status', [4, 5, 6])
def test_timeout_and_cancel_ack_hold_until_real_terminal(late, status):
    owner, controller = MotionOwnership(), Controller()
    session = owner.begin('legacy', legacy=True)
    accepted = dispatch(owner, controller)
    if not late:
        controller.accept()
        assert accepted.result().accepted
    assert not owner.finish(session)
    assert_held(owner)
    if late:
        controller.accept()
    assert controller.cancels == 1
    assert not owner.finish(session) and controller.cancels == 1
    assert_held(owner)
    controller.finish(status)
    assert accepted.result().get_result_async().result().status == status
    assert owner.snapshot()['active'] is None and not owner.motion_lock.locked()
    assert owner.begin('next', legacy=True) is not None


@pytest.mark.parametrize('late', [False, True])
def test_rejection_is_a_terminal_non_submission(late):
    owner, controller = MotionOwnership(), Controller(accepted=False)
    session = owner.begin('legacy', legacy=True)
    accepted = dispatch(owner, controller)
    if late:
        assert not owner.finish(session)
    controller.accept()
    assert accepted.result().accepted is False
    owner.finish(session)
    assert not owner.motion_lock.locked() and owner.snapshot()['active'] is None
    assert controller.cancels == 0


@pytest.mark.parametrize('fault', ['send', 'acceptance', 'result', 'cancel', 'nonterminal', 'no_response'])
def test_uncertainty_has_no_force_clear_or_fallback(fault):
    owner, controller = MotionOwnership(), Controller()
    session = owner.begin('legacy', legacy=True)
    if fault == 'send':
        def send(*args, **kwargs):
            raise RuntimeError('unknown whether request left transport')
        controller.send_goal_async = send
        with pytest.raises(RuntimeError):
            dispatch(owner, controller)
    else:
        accepted = dispatch(owner, controller)
        if fault == 'acceptance':
            controller.acceptance.set_exception(RuntimeError('acceptance unavailable'))
            with pytest.raises(RuntimeError):
                accepted.result()
        elif fault != 'no_response':
            controller.accept()
            if fault == 'cancel':
                def cancel():
                    raise RuntimeError('unknown cancel transport')
                controller.cancel_goal_async = cancel
            elif fault == 'nonterminal':
                controller.finish(status=2)
                with pytest.raises(ValueError):
                    accepted.result().get_result_async().result()
            else:
                controller.result.set_exception(RuntimeError('result unavailable'))
    with pytest.raises(RuntimeError, match='MOTION_SESSION_NOT_DISPATCHABLE'):
        dispatch(owner, Controller())
    assert not owner.finish(session)
    assert_held(owner)
    assert not owner.finish(session)  # Repeated finish cannot clear uncertainty.
    assert owner.snapshot()['active']['closing'] is True


def test_terminal_is_consumed_before_callback_can_send_the_next_original_goal():
    owner, first, second = MotionOwnership(), Controller(), Controller()
    session = owner.begin('legacy', legacy=True)
    first_goal, next_goal = object(), object()
    public = OwnedActionClient(first, owner, 'first').send_goal_async(first_goal)
    first.accept()
    followups = []
    public.result().get_result_async().add_done_callback(lambda _: followups.append(
        OwnedActionClient(second, owner, 'second').send_goal_async(next_goal)))
    first.finish()
    assert first.requests[0][0] is first_goal and second.requests[0][0] is next_goal
    assert len(followups) == 1
    second.accept()
    second.finish()
    assert owner.finish(session)


def test_unowned_send_never_reaches_transport_and_pending_send_blocks_fallback():
    owner, controller = MotionOwnership(), Controller()
    with pytest.raises(RuntimeError):
        dispatch(owner, controller)
    assert controller.requests == []
    owner.begin('legacy', legacy=True)
    dispatch(owner, controller)
    with pytest.raises(RuntimeError):
        dispatch(owner, controller)
    assert len(controller.requests) == 1


def test_operation_session_excludes_legacy_across_event_barriers_and_uncertain_strict_motion():
    owner = MotionOwnership()
    session = owner.begin('bound operation')
    assert not owner.motion_lock.locked()  # Between controller legs/barriers.
    assert owner.begin('legacy', legacy=True) is None
    assert owner.motion_lock.acquire(blocking=False)
    assert not owner.finish(session)
    assert_held(owner)
    owner.motion_lock.release()  # Represents strict port consuming terminal.
    assert owner.finish(session)
    new = owner.begin('new', legacy=True)
    assert new is not None
    assert not owner.finish(session)  # Old completion cannot discharge new owner.
    assert_held(owner)
    assert owner.finish(new)


def test_callback_exception_after_send_still_retains_and_cancels_motion():
    owner, controller = MotionOwnership(), Controller()
    def callback(_):
        dispatch(owner, controller)
        controller.accept()
        raise RuntimeError('caller lost its result')
    with pytest.raises(RuntimeError):
        run_legacy_motion(owner, None, SimpleNamespace, 'legacy', callback)
    assert controller.cancels == 1
    assert_held(owner)
    controller.finish()
    assert not owner.motion_lock.locked()


@pytest.mark.parametrize('late', [False, True])
def test_correlated_stop_blocks_fallback_but_retains_running_caller_after_terminal(late):
    owner, controller, key = MotionOwnership(), Controller(), object()
    session = owner.begin('legacy', legacy=True, request_key=key)
    accepted = dispatch(owner, controller)
    if not late:
        controller.accept()
    assert not owner.request_stop(object())
    assert controller.cancels == 0
    assert owner.request_stop(key)
    if late:
        controller.accept()
    assert controller.cancels == 1
    with pytest.raises(RuntimeError):
        dispatch(owner, Controller())
    controller.finish(5)
    assert accepted.result().get_result_async().result().status == 5
    assert_held(owner)
    assert owner.finish(session)
    next_session = owner.begin('new', legacy=True, request_key=object())
    assert not owner.request_stop(key)
    assert owner.snapshot()['active']['stopping'] is False
    assert owner.finish(next_session)


def test_cancel_before_callback_never_enters_backend():
    owner, calls, cancelled = MotionOwnership(), [], []
    handle = SimpleNamespace(is_cancel_requested=True, canceled=lambda: cancelled.append(True))
    response = run_legacy_motion(owner, handle, SimpleNamespace, 'already cancelled',
        lambda _: calls.append(True))
    assert response.success is False and response.message == 'CANCELLED_BEFORE_DISPATCH'
    assert calls == [] and cancelled == [True] and not owner.motion_lock.locked()


def test_journal_storage_is_bounded_without_clearing_motion():
    owner = MotionOwnership(journal_capacity=3)
    session = owner.begin('legacy', legacy=True)
    for i in range(8):
        assert owner.begin(str(i)) is None
    assert len(owner.snapshot()['events']) == 3
    assert_held(owner)
    assert owner.finish(session)
