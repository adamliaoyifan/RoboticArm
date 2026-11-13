"""Process-local ownership shared by every motion entry in one motion node.

A session spans planning, controller work and event barriers. ``motion_lock``
is the separate, non-reentrant submission lock used by StrictOperationMotionPort.
Legacy callbacks hold both. A complete-operation owner holds a session across
its barriers and uses the submission lock per leg. Unknown transport never
establishes stopped motion and there is intentionally no force-clear operation.
"""
from collections import deque
from concurrent.futures import Future
from dataclasses import dataclass, field
import threading
import time


_TERMINAL_STATUSES = frozenset((4, 5, 6))  # ROS SUCCEEDED, CANCELED, ABORTED


@dataclass(eq=False)
class MotionSession:
    sequence: int
    label: str
    legacy: bool
    request_key: object = None
    closing: bool = False
    stopping: bool = False
    transports: dict = field(default_factory=dict)


@dataclass
class _Transport:
    endpoint: str
    accepted_future: object = field(default_factory=Future)
    public_result: object = field(default_factory=Future)
    handle: object = None
    result_future: object = None
    cancel_sent: bool = False
    uncertainty: str = ''


class MotionOwnership:
    """One active session; callbacks may only discharge their own transport.

    ``journal_capacity`` bounds diagnostic storage, not motion time or safety.
    Successful finish of a session is only ownership discharge. Native motion,
    current-source, seal and endpoint checks remain the respective owners' work.
    """

    def __init__(self, *, journal_capacity=128):
        if type(journal_capacity) is not int or journal_capacity < 1:
            raise ValueError('positive journal_capacity required')
        self.motion_lock = threading.Lock()
        self._mutex = threading.RLock()
        self._active = None
        self._sequence = 0
        self._transport_sequence = 0
        self._journal = deque(maxlen=journal_capacity)

    def _event(self, kind, **fields):
        self._journal.append(dict(kind=kind, monotonic_ns=time.monotonic_ns(), **fields))

    def snapshot(self):
        with self._mutex:
            session = self._active
            return dict(active=None if session is None else dict(
                sequence=session.sequence, label=session.label, legacy=session.legacy,
                closing=session.closing, stopping=session.stopping, pending=[dict(sequence=key,
                    endpoint=value.endpoint, uncertainty=value.uncertainty,
                    cancel_sent=value.cancel_sent) for key, value in session.transports.items()]),
                motion_locked=self.motion_lock.locked(), events=list(self._journal))

    def begin(self, label, *, legacy=False, request_key=None):
        if not isinstance(label, str) or not label or type(legacy) is not bool:
            raise ValueError('explicit session label and boolean legacy required')
        with self._mutex:
            if self._active is not None or self.motion_lock.locked():
                self._event('busy', label=label)
                return None
            if legacy and not self.motion_lock.acquire(blocking=False):
                return None
            self._sequence += 1
            session = MotionSession(self._sequence, label, legacy, request_key=request_key)
            self._active = session
            self._event('begin', session=session.sequence, label=label, legacy=legacy)
            return session

    def request_stop(self, request_key):
        """Cancel only the correlated active action, without discharging it.

        The same in-process ServerGoalHandle is supplied by the node's execute
        and cancel callbacks. A cancel of another/old goal cannot stop a newer
        session. Stopping blocks dispatch; only handler completion sets closing.
        A terminal inner goal cannot release a still-running caller.
        """
        with self._mutex:
            session = self._active
            if request_key is None or session is None or session.request_key is not request_key:
                return False
            session.stopping = True
            self._event('stop', session=session.sequence)
            for transport in list(session.transports.values()):
                self._cancel(transport)
            return True

    def finish(self, session):
        """Close dispatch; release only after tracked/strict motion is terminal.

        Legacy transports discharge asynchronously. A complete-operation caller
        must finish again after its StrictOperationMotionPort consumes terminal
        status and releases motion_lock. It must not abandon the session on a
        timeout or a vacuum/transport exception.
        """
        with self._mutex:
            if session is not self._active:
                return False
            session.closing = True
            for transport in list(session.transports.values()):
                self._cancel(transport)
            return self._release_if_finished(session)

    def _release_if_finished(self, session):
        if session is not self._active or not session.closing or session.transports:
            return False
        if session.legacy:
            self.motion_lock.release()
        elif self.motion_lock.locked():
            return False
        self._event('finish', session=session.sequence)
        self._active = None
        return True

    def _cancel(self, transport):
        if transport.handle is None or transport.cancel_sent:
            return
        transport.cancel_sent = True
        try:
            # Acknowledgement is not terminal; only the result callback removes
            # this transport. A failed cancel send still retains ownership.
            transport.handle.cancel_goal_async()
        except Exception as error:
            transport.uncertainty = 'CANCEL_TRANSPORT_ERROR:' + type(error).__name__

    def _accepted(self, session, sequence, future):
        with self._mutex:
            transport = session.transports.get(sequence)
            if session is not self._active or transport is None:
                return
            try:
                handle = future.result()
                if handle.accepted is False:
                    self._terminal(session, sequence, 'rejected')
                    transport.accepted_future.set_result(handle)
                    return
                if handle.accepted is not True:
                    raise ValueError('boolean acceptance required')
                transport.handle = handle
                # The owner independently observes terminal status even when a
                # timed-out legacy handler has stopped waiting for its result.
                result = transport.result_future = handle.get_result_async()
                result.add_done_callback(lambda done: self._result(session, sequence, done))
                if session.closing or session.stopping:
                    self._cancel(transport)
                transport.accepted_future.set_result(_OwnedGoalHandle(handle, transport.public_result))
            except Exception as error:
                transport.uncertainty = 'ACCEPTANCE_TRANSPORT_ERROR:' + type(error).__name__
                self._event('uncertain', session=session.sequence, transport=sequence,
                            reason=transport.uncertainty)
                if session.closing or session.stopping:
                    self._cancel(transport)
                if not transport.accepted_future.done():
                    transport.accepted_future.set_exception(error)

    def _result(self, session, sequence, future):
        with self._mutex:
            transport = session.transports.get(sequence)
            if session is not self._active or transport is None:
                return
            try:
                wrapped = future.result()
                status = wrapped.status
                if type(status) is not int or status not in _TERMINAL_STATUSES:
                    raise ValueError('terminal action status required')
                self._terminal(session, sequence, status)
                transport.public_result.set_result(wrapped)
            except Exception as error:
                transport.uncertainty = 'RESULT_TRANSPORT_ERROR:' + type(error).__name__
                self._event('uncertain', session=session.sequence, transport=sequence,
                            reason=transport.uncertainty)
                self._cancel(transport)
                if not transport.public_result.done():
                    transport.public_result.set_exception(error)

    def _terminal(self, session, sequence, status):
        session.transports.pop(sequence)
        self._event('terminal', session=session.sequence, transport=sequence, status=status)
        self._release_if_finished(session)

    def send_goal_async(self, client, endpoint, *args, **kwargs):
        """Explicit legacy transport adapter; track before any goal can leave.

        One transport at a time also blocks an old fallback after unknown result
        or acceptance. The current legacy handler may send another goal after a
        real terminal result; successful handler return then closes its session.
        """
        with self._mutex:
            session = self._active
            if (session is None or not session.legacy or session.closing or session.stopping
                    or session.transports):
                raise RuntimeError('MOTION_SESSION_NOT_DISPATCHABLE')
            self._transport_sequence += 1
            sequence = self._transport_sequence
            transport = _Transport(str(endpoint))
            session.transports[sequence] = transport
            self._event('dispatch', session=session.sequence, transport=sequence, endpoint=str(endpoint))
            try:
                future = client.send_goal_async(*args, **kwargs)
                future.add_done_callback(lambda done: self._accepted(session, sequence, done))
                return transport.accepted_future
            except Exception as error:
                # A send exception cannot prove the request never left DDS.
                transport.uncertainty = 'SEND_TRANSPORT_ERROR:' + type(error).__name__
                self._event('uncertain', session=session.sequence, transport=sequence,
                            reason=transport.uncertainty)
                raise


class _OwnedGoalHandle:
    """Expose results only after ownership consumes the same terminal receipt.

    The owner obtains exactly one raw result future. Sharing its public future
    avoids duplicate result RPCs and executor callback ordering races where a
    handler starts its next leg before the owner observes the previous result.
    """

    def __init__(self, handle, result_future):
        self._handle, self._result_future = handle, result_future

    @property
    def accepted(self):
        return self._handle.accepted

    @property
    def goal_id(self):
        return self._handle.goal_id

    def get_result_async(self):
        return self._result_future

    def cancel_goal(self):
        return self._handle.cancel_goal()

    def cancel_goal_async(self):
        return self._handle.cancel_goal_async()


class OwnedActionClient:
    """Narrow explicit wrapper for the existing legacy action clients."""

    def __init__(self, client, owner, endpoint):
        self.client, self.owner, self.endpoint = client, owner, str(endpoint)

    def wait_for_server(self, *args, **kwargs):
        return self.client.wait_for_server(*args, **kwargs)

    def server_is_ready(self):
        return self.client.server_is_ready()

    def send_goal_async(self, *args, **kwargs):
        return self.owner.send_goal_async(self.client, self.endpoint, *args, **kwargs)

    def destroy(self):
        return self.client.destroy()


def run_legacy_motion(owner, goal_handle, result_type, label, callback):
    """Node boundary used by all three real legacy execute callbacks."""
    session = owner.begin(label, legacy=True, request_key=goal_handle)
    if session is None:
        result = result_type()
        result.success, result.message = False, 'SHARED_MOTION_BUSY'
        goal_handle.abort()
        return result
    try:
        if getattr(goal_handle, 'is_cancel_requested', False):
            owner.request_stop(goal_handle)
            result = result_type()
            result.success, result.message = False, 'CANCELLED_BEFORE_DISPATCH'
            goal_handle.canceled()
            return result
        return callback(goal_handle)
    finally:
        owner.finish(session)
