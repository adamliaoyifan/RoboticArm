"""Plan-only ROS port for private-snapshot native joint connections.

Geometric results are endpoint/scene-bound seeds for the whole-chain certifier.
They have no timing, no execution authorization and no actual-attachment claim.
"""
from dataclasses import dataclass
import hashlib
import math
import threading
import time

from luggage_description.frame_contracts import vector
from luggage_planning.joint_operation_contracts import OPERATION_JOINTS


@dataclass(frozen=True)
class GeometricConnection:
    success: bool
    complete: bool
    reason: str
    positions: tuple = ()
    checked_states: int = 0


def validate_connection_response(response, *, request_id, snapshot_id,
                                 model_sha256, scene_sha256, start, goal,
                                 max_joint_step_rad=.01):
    """Validate received binding before timing/assembling it into any plan."""
    expected = (request_id, snapshot_id, model_sha256, scene_sha256)
    actual = (response.request_id, response.snapshot_id,
              response.model_sha256, response.scene_sha256)
    if actual != expected:
        return GeometricConnection(False, False, "CONNECTION_IDENTITY_MISMATCH")
    path = response.geometric_path
    if not response.success:
        if path.points or path.joint_names:
            return GeometricConnection(False, False, "REFUSED_CONNECTION_HAS_PATH")
        return GeometricConnection(False, bool(response.complete), str(response.reason))
    if not response.complete or tuple(path.joint_names) != OPERATION_JOINTS:
        return GeometricConnection(False, False, "INVALID_CONNECTION_PATH")
    if (path.header.frame_id or path.header.stamp.sec or path.header.stamp.nanosec
            or not 2 <= len(path.points) <= 50000):
        return GeometricConnection(False, False, "INVALID_CONNECTION_PATH")
    positions = []
    for point in path.points:
        q = tuple(float(v) for v in point.positions)
        if (len(q) != 6 or not all(math.isfinite(v) for v in q)
                or point.velocities or point.accelerations or point.effort
                or point.time_from_start.sec or point.time_from_start.nanosec):
            return GeometricConnection(False, False, "INVALID_GEOMETRIC_CONNECTION")
        if positions and max(abs(a-b) for a, b in zip(positions[-1], q)) > max_joint_step_rad + 1e-10:
            return GeometricConnection(False, False, "UNDERSAMPLED_CONNECTION")
        positions.append(q)
    if (max(abs(a-b) for a, b in zip(positions[0], start)) > 1e-8
            or max(abs(a-b) for a, b in zip(positions[-1], goal)) > 1e-8):
        return GeometricConnection(False, False, "CONNECTION_ENDPOINT_MISMATCH")
    if response.checked_states < len(positions):
        return GeometricConnection(False, False, "INCOMPLETE_CONNECTION_CHECKS")
    return GeometricConnection(True, True, str(response.reason), tuple(positions),
                               int(response.checked_states))


class NativeSnapshotConnectionPort:
    def __init__(self, node, *, recorder=None, max_edge_s=2.):
        from luggage_msgs.srv import PlanSnapshotJointPathV2
        if not math.isfinite(max_edge_s) or not 0 < max_edge_s <= 10:
            raise ValueError("max_edge_s must be in (0,10]")
        self.service = PlanSnapshotJointPathV2
        self.client = node.create_client(self.service, "/joint_operation/plan_connection")
        self.recorder = recorder
        self.max_edge_s = float(max_edge_s)
        self.inflight = None

    def connect(self, snapshot, start, goal, *, request_id, deadline, cancelled, path_constraints=None):
        start = vector(start, 6, "connection_start")
        goal = vector(goal, 6, "connection_goal")
        if not isinstance(request_id, str) or not request_id.strip():
            raise ValueError("connection request_id required")
        if cancelled() or time.monotonic() >= deadline:
            return GeometricConnection(False, False, "CANCELLED_OR_DEADLINE")
        if self.inflight is not None and not self.inflight.done():
            return GeometricConnection(False, False, "CONNECTION_BACKEND_BUSY")
        if not self.client.service_is_ready():
            return GeometricConnection(False, False, "CONNECTION_BACKEND_UNAVAILABLE")
        request = self.service.Request()
        request.request_id = request_id
        request.snapshot_id = hashlib.sha256(repr(snapshot.identity).encode()).hexdigest()
        request.model_sha256 = snapshot.model_hash
        request.scene_sha256 = hashlib.sha256(snapshot.scene_cdr).hexdigest()
        request.scene_cdr = list(snapshot.scene_cdr)
        request.start_joints, request.goal_joints = list(start), list(goal)
        if path_constraints is not None:
            request.path_constraints = path_constraints
        request.budget_s = min(self.max_edge_s, deadline-time.monotonic())
        if request.budget_s <= 0:
            return GeometricConnection(False, False, "CANCELLED_OR_DEADLINE")
        if self.recorder:
            self.recorder("request", request)
        ready = threading.Event()
        self.inflight = self.client.call_async(request)
        self.inflight.add_done_callback(lambda future: ready.set())
        while not ready.wait(min(.01, max(0., deadline-time.monotonic()))):
            if cancelled() or time.monotonic() >= deadline:
                return GeometricConnection(False, False, "CANCELLED_OR_DEADLINE")
        response = self.inflight.result()
        if self.recorder:
            self.recorder("response", response)
        if cancelled() or time.monotonic() >= deadline:
            return GeometricConnection(False, False, "CANCELLED_OR_DEADLINE")
        return validate_connection_response(response, request_id=request_id,
            snapshot_id=request.snapshot_id, model_sha256=request.model_sha256,
            scene_sha256=request.scene_sha256, start=start, goal=goal)
