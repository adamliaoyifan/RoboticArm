"""Single-flight operation RPC with late-result ownership and capture."""
import threading
import time
import json


class OperationRpc:
    """Single-flight RPC ownership extends through late-result cleanup."""
    def __init__(self, recorder):
        self.record, self.pending, self.consumed = recorder, {}, {}
        self.cancelled = lambda:False
        self.transport_uncertain = set()

    def unresolved(self):
        return bool(self.transport_uncertain) or any(not event.is_set() for event in self.consumed.values())

    def call(self, client, request, deadline, name, *, late_response=None):
        if self.cancelled() or time.monotonic() >= deadline:
            raise TimeoutError('OPERATION_CANCELLED_OR_DEADLINE')
        if client in self.transport_uncertain or (client in self.consumed and not self.consumed[client].is_set()):
            raise RuntimeError(name+'_BACKEND_BUSY')
        if not client.service_is_ready():raise RuntimeError(name+'_BACKEND_UNAVAILABLE')
        self.record(name+'_request', request)
        self.transport_uncertain.add(client)
        future = self.pending[client] = client.call_async(request)
        event, consumed, lock = threading.Event(), threading.Event(), threading.Lock()
        self.consumed[client] = consumed
        state = dict(abandoned=False, cleaned=False)
        def cleanup():
            with lock:
                if not state['abandoned'] or state['cleaned'] or not future.done():return
                state['cleaned'] = True
            try:
                response = future.result()
                self.record(name+'_late_response', response)
                if late_response is not None:late_response(response)
                self.transport_uncertain.discard(client)
            except Exception as error:
                self.record(name+'_cleanup_fault.json',json.dumps(dict(reason=str(error))).encode())
            finally:consumed.set()
        def completed(_):
            event.set()
            cleanup()
        future.add_done_callback(completed)
        returned = False
        try:
            while not event.wait(min(.01, max(0.,deadline-time.monotonic()))):
                if self.cancelled() or time.monotonic() >= deadline:
                    raise TimeoutError('OPERATION_CANCELLED_OR_DEADLINE')
            result = future.result()
            self.record(name+'_response',result)
            if self.cancelled() or time.monotonic() >= deadline:
                raise TimeoutError('OPERATION_CANCELLED_OR_DEADLINE')
            returned = True
            self.transport_uncertain.discard(client)
            return result
        finally:
            if returned:consumed.set()
            else:
                with lock:state['abandoned'] = True
                cleanup()

