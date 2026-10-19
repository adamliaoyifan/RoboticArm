"""ROS transport to the private native rigid-tool checker, never MoveIt apply.

Call from the background worker while the node executor services futures. One
in-flight call is retained even after caller cancellation: another task cannot
pile up service work behind an abandoned request.
"""
import hashlib
import threading
import time

from luggage_planning.placement_prior import CollisionVerdict


class NativeToolCollisionPort:
    def __init__(self, node, *, payload_id='prior-payload', recorder=None):
        from luggage_msgs.srv import CheckPlacementToolV2
        self._service = CheckPlacementToolV2
        self._client = node.create_client(CheckPlacementToolV2, '/placement_prior/check_tool')
        from luggage_planning.placement_prior_local_paths import NativeLocalPathPort
        self._local = NativeLocalPathPort(node, recorder=recorder)
        self._payload_id = str(payload_id)
        self._recorder = recorder
        self._inflight = None
        self._counter = 0
        # One private decoded scene, shared read-only by sequential requests.
        # Match complete immutable bytes, not map revision or snapshot labels:
        # geometry/ACM changes with reused labels must still invalidate it.
        self._scene_cdr = None
        self._decoded_scene = None

    def ready(self):
        return self._client.service_is_ready() and self._local.client.service_is_ready()

    def check_local(self, request, placement, grasp, *, deadline, cancelled):
        return self._local.check(request, placement, grasp, deadline=deadline, cancelled=cancelled)

    def check(self, snapshot, world_from_tcp, world_from_box, box_wdh, *, deadline, cancelled):
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import PlanningScene
        from rclpy.serialization import deserialize_message
        if cancelled() or time.monotonic() >= deadline:
            return CollisionVerdict(False, 'CANCELLED_OR_DEADLINE')
        if self._inflight is not None and not self._inflight.done():
            return CollisionVerdict(False, 'BACKEND_BUSY')
        if not self._client.service_is_ready():
            return CollisionVerdict(False, 'BACKEND_UNAVAILABLE')
        def pose(transform):
            result = Pose()
            result.position.x, result.position.y, result.position.z = transform.translation
            (result.orientation.x, result.orientation.y,
             result.orientation.z, result.orientation.w) = transform.quaternion_xyzw
            return result
        self._counter += 1
        request = self._service.Request()
        request.request_id = 'prior-tool-%d' % self._counter
        request.snapshot_id = hashlib.sha256(repr(snapshot.identity).encode()).hexdigest()
        request.model_sha256 = snapshot.model_hash
        request.frame_id = 'world'
        request.tcp_link = 'suction_contact_frame'
        request.payload_id = self._payload_id
        if self._scene_cdr != snapshot.scene_cdr:
            decoded = deserialize_message(snapshot.scene_cdr, PlanningScene)
            self._decoded_scene = decoded
            self._scene_cdr = snapshot.scene_cdr
        request.scene = self._decoded_scene
        request.tcp_poses = [pose(world_from_tcp)]
        request.box_poses = [pose(world_from_box)]
        request.box_wdh = list(box_wdh)
        request.budget_s = min(10., max(.000001, deadline-time.monotonic()))
        ready = threading.Event()
        if self._recorder:
            self._recorder('request', request)
        self._inflight = self._client.call_async(request)
        self._inflight.add_done_callback(lambda future: ready.set())
        while not ready.wait(min(.01, max(0., deadline-time.monotonic()))):
            if cancelled() or time.monotonic() >= deadline:
                return CollisionVerdict(False, 'CANCELLED_OR_DEADLINE')
        response = self._inflight.result()
        if self._recorder:
            self._recorder('response', response)
        if cancelled() or time.monotonic() >= deadline:
            return CollisionVerdict(False, 'CANCELLED_OR_DEADLINE')
        if (response.request_id != request.request_id or
                response.snapshot_id != request.snapshot_id or
                response.model_sha256 != snapshot.model_hash):
            return CollisionVerdict(False, 'BACKEND_IDENTITY_MISMATCH')
        if not response.complete or len(response.valid) != 1:
            return CollisionVerdict(False, response.reason or 'INCOMPLETE_NATIVE_CHECK')
        contacts = tuple((c.body_1, c.body_2) for c in response.contacts)
        return CollisionVerdict(bool(response.valid[0]),
                                'CLEAR' if response.valid[0] else 'COLLISION', contacts)
