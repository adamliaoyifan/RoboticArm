"""Acknowledged sensor self-filter barriers around vacuum physical effects.

PENDING discards complete clouds. A failed post-seal measurement or handshake
keeps that state; it never declares a held payload absent to resume mapping.
"""
import threading
import time

from luggage_msgs.srv import SetObserverPayloadMask


class ObserverPayloadMaskBarrier:
    def __init__(self, node, callback_group, tool_frame, timeout_s=5.):
        self.node, self.tool_frame, self.timeout_s = node, tool_frame, timeout_s
        self.client = node.create_client(SetObserverPayloadMask,
            '/luggage/observation/set_payload_mask', callback_group=callback_group)
        self.lock = threading.RLock()
        self.initialized = False
        self.timer = node.create_timer(.5, self._initialize)

    def _send(self, state, operation_id, geometry=None):
        request = SetObserverPayloadMask.Request(state=state,
            operation_id=operation_id, tool_frame=self.tool_frame)
        if geometry is not None:
            request.geometry = geometry
        if not self.client.wait_for_service(timeout_sec=self.timeout_s):
            return False
        future = self.client.call_async(request)
        deadline = time.monotonic() + self.timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.01)  # Response runs in the existing reentrant executor.
        accepted = bool(future.done() and future.result() and future.result().accepted)
        self.node.get_logger().info('observer_payload_mask state=%s operation=%s accepted=%s'
                                    % (state, operation_id, accepted))
        return accepted

    def _initialize(self):
        if not self.lock.acquire(blocking=False):
            return
        try:
            if not self.initialized and self.client.service_is_ready():
                # Explicit joint-operation launch starts unloaded. This does not
                # infer NONE from an absent or expired attachment observation.
                self.initialized = self._send(0, 'joint-operation-unloaded-startup')
            if self.initialized:
                self.timer.cancel()
        finally:
            self.lock.release()

    def transition(self, state, operation_id, geometry=None):
        with self.lock:
            if not self.initialized:
                self.initialized = self._send(0, 'joint-operation-unloaded-startup')
                if not self.initialized:
                    return False
                self.timer.cancel()
            return self._send(state, operation_id, geometry)
