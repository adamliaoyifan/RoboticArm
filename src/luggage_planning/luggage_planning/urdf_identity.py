"""Live robot_description hash for atlas identity.

The atlas builder stores the MD5 of the expanded URDF string on
``/robot_state_publisher``. This watcher reads that same parameter. It
never blocks a callback: a timer starts the request, and ``poll`` observes
the result. Call ``poll`` at the start of a request so a completed future
is visible before the timer fires again.
"""

from __future__ import division

from luggage_planning.atlas_runtime import compute_urdf_hash


class UrdfIdentity(object):
    def __init__(self, node):
        self._node = node
        self.hash = ""
        self.ready = False
        self.reason = "pending"
        self._client = None
        self._future = None
        self._timer = None

    def start(self):
        """Begin polling. A local non-empty robot_description wins."""
        if self.ready:
            return
        if self._node.has_parameter("robot_description"):
            text = str(self._node.get_parameter("robot_description").value or "")
            if text:
                self._accept(text)
                return
        from rcl_interfaces.srv import GetParameters
        self._client = self._node.create_client(
            GetParameters, "/robot_state_publisher/get_parameters")
        self._timer = self._node.create_timer(0.5, self.poll)

    def poll(self):
        """One non-blocking step. Safe from a callback on any executor."""
        if self.ready or self._client is None:
            return
        if self._future is None:
            if not self._client.service_is_ready():
                self.reason = "robot_description_unavailable"
                return
            from rcl_interfaces.srv import GetParameters
            request = GetParameters.Request()
            request.names = ["robot_description"]
            self._future = self._client.call_async(request)
            self.reason = "pending"
            return
        if not self._future.done():
            return
        result = self._future.result()
        text = ""
        if result is not None and result.values:
            text = str(result.values[0].string_value or "")
        if not text:
            self.reason = "robot_description_empty"
            self._future = None
            return
        self._accept(text)
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _accept(self, text):
        self.hash = compute_urdf_hash(text)
        self.ready = bool(self.hash)
        self.reason = "ok" if self.ready else "robot_description_empty"
