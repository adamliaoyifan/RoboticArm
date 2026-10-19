"""Single bounded background worker; newest submission replaces pending work.

One running job, one pending job, one result. Cancellation is cooperative and
collision ports MUST honor the request deadline. No ROS callbacks block waiting
for completion. This object owns neither occupancy nor a shared planning scene.
"""
from threading import Condition, Event, Thread

from luggage_planning.placement_prior import compute_prior


class PriorWorker:
    def __init__(self, collision, compute=compute_prior):
        self._collision = collision
        self._compute = compute
        self._condition = Condition()
        self._pending = None
        self._cancel = None
        self._generation = 0
        self._result = None
        self._error = None
        self._closed = False
        self._thread = Thread(target=self._run, name='placement-prior', daemon=True)
        self._thread.start()

    def submit(self, request):
        with self._condition:
            if self._closed:
                raise RuntimeError('prior worker closed')
            self._generation += 1
            if self._cancel:
                self._cancel.set()
            self._result = None
            self._error = None
            self._pending = (self._generation, request)
            self._condition.notify_all()
            return self._generation

    def invalidate(self, generation=None):
        with self._condition:
            if generation is not None and generation != self._generation:
                return False
            self._generation += 1
            self._pending = None
            self._result = None
            self._error = None
            if self._cancel:
                self._cancel.set()
            self._condition.notify_all()
            return True

    def poll(self):
        with self._condition:
            return self._generation, self._result, self._error

    def wait_result(self, generation, timeout):
        """Bounded service-side wait; supersession returns immediately."""
        with self._condition:
            self._condition.wait_for(
                lambda: self._closed or generation != self._generation
                or self._result is not None or self._error is not None,
                timeout=timeout)
            if generation != self._generation or self._closed:
                return None, ('Cancelled', 'prior job superseded')
            return self._result, self._error

    def close(self, timeout=1.0):
        with self._condition:
            self._closed = True
            self._pending = None
            if self._cancel:
                self._cancel.set()
            self._condition.notify_all()
        self._thread.join(timeout)
        return not self._thread.is_alive()

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._pending is not None)
                if self._closed:
                    return
                generation, request = self._pending
                self._pending = None
                self._cancel = Event()
                cancel = self._cancel
            try:
                result = self._compute(request, self._collision, cancelled=cancel.is_set)
                error = None
            except Exception as exc:
                result = None
                error = (type(exc).__name__, str(exc))
            with self._condition:
                if generation == self._generation and not self._closed:
                    self._result = result
                    self._error = error
                self._condition.notify_all()
