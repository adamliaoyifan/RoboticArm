"""client — the T1 chain client: one rclpy node per chain graph.

Owns the /clock master (sim time drives ONLY fixture scenario time), the
fixture inputs (TF + PointCloud2 for G's real subscription intake,
IntegrateObservation for O's service intake, JointState for the robot
freshness contract), the reset/status/placement service clients, and the
planner observables (last_result) used as absorb barriers. Deadlines are
REAL monotonic.
"""
from __future__ import annotations

import json
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .ledger import mono_ns

_CLOCK_HZ = 50.0


def _points_to_cloud(points, frame_id, stamp_sec, stamp_ns):
    """Optical-frame xyz float32 PointCloud2 (the mapper's intake shape)."""
    from builtin_interfaces.msg import Time as TimeMsg
    from std_msgs.msg import Header
    step = 12
    data = bytearray()
    for x, y, z in points:
        data += struct.pack("<fff", float(x), float(y), float(z))
    return PointCloud2(
        header=Header(
            stamp=TimeMsg(sec=int(stamp_sec), nanosec=int(stamp_ns)),
            frame_id=str(frame_id)),
        height=1, width=len(points), is_bigendian=False, point_step=step,
        row_step=step * max(len(points), 1), is_dense=True,
        fields=[
            PointField(name="x", offset=0, datatype=PointField.FLOAT32,
                       count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32,
                       count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32,
                       count=1),
        ],
        data=bytes(data))


class ChainClient(Node):
    """Drives one backend chain; created AFTER the chain processes boot."""

    def __init__(self, backend, *, sim=False):
        super().__init__("t1_client_%s" % backend.lower())
        self.backend = backend
        self.sim = sim
        self._last_result = {}
        self._validity_seen = {}
        latched = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # /clock master: T1 owns fixture scenario time exclusively.
        self._clock_pub = self.create_publisher(
            __import__("rosgraph_msgs.msg", fromlist=["Clock"]).Clock,
            "/clock", 10)
        self._tf_static = StaticTransformBroadcaster(self)
        self._tf = TransformBroadcaster(self)
        self._cloud_pub = self.create_publisher(
            PointCloud2, "/luggage/semantic/cargo_points_untracked",
            QoSProfile(depth=12,
                       reliability=ReliabilityPolicy.BEST_EFFORT))
        self._joint_pub = self.create_publisher(JointState, "/joint_states",
                                                10)
        self._validity_sub = self.create_subscription(
            String,
            "/uld_occ_observer/validity" if backend == "O"
            else "/luggage/cargo_map/validity",
            self._on_validity, latched)
        self._surface_map = {}
        self._surface_sub = self.create_subscription(
            String, "/luggage/cargo_map/surface_2d",
            self._on_surface_map, latched)
        self._last_sub = self.create_subscription(
            String, "/placement_planner/last_result",
            self._on_last_result, latched)
        self._chain_clients = {}
        self._make_clients(backend)

    # -- service clients ------------------------------------------------
    def _make_clients(self, backend):
        from luggage_msgs.srv import ComputePlacement, ComputePlacementV2
        if backend == "G":
            from luggage_msgs.srv import (GetCargoValidity,
                                          IntegrateCargoView,
                                          ResetCargoMapV2)
            self._chain_clients["integrate"] = self.create_client(
                IntegrateCargoView, "/cargo_map/integrate_cargo_view")
            self._chain_clients["reset"] = self.create_client(
                ResetCargoMapV2, "/cargo_map/reset_v2")
            self._chain_clients["validity"] = self.create_client(
                GetCargoValidity, "/cargo_map/get_validity")
        else:
            from uld_occ_observer.srv import (GetValidity,
                                              IntegrateObservation,
                                              ResetObservation)
            self._chain_clients["integrate"] = None  # O integrates per-frame
            self._chain_clients["reset"] = self.create_client(
                ResetObservation, "/uld_occ_observer/reset_observation")
            self._chain_clients["validity"] = self.create_client(
                GetValidity, "/uld_occ_observer/get_validity")
            self._chain_clients["integrate_observation"] = self.create_client(
                IntegrateObservation, "/uld_occ_observer/"
                                      "integrate_observation")
        self._chain_clients["placement"] = self.create_client(
            ComputePlacement, "/placement_planner/compute_placement")
        self._chain_clients["placement_v2"] = self.create_client(
            ComputePlacementV2, "/placement_planner/compute_placement_v2")

    #: Services every chain must expose. placement_v2 exists only in the
    #: planner's v2 surface mode (frame_scene_revision set) — the product
    #: creates no V2 service otherwise, so it is NOT required here.
    REQUIRED_SERVICES = ("reset", "validity", "placement")

    def wait_services(self, timeout_s=20.0):
        deadline = time.monotonic() + timeout_s
        names = [n for n in self.REQUIRED_SERVICES
                 if self._chain_clients.get(n) is not None]
        if self.backend == "O":
            names.append("integrate_observation")
        else:
            names.append("integrate")
        pending = list(names)
        while time.monotonic() < deadline and pending:
            pending = [name for name in pending
                       if not self._chain_clients[name].wait_for_service(
                           timeout_sec=0.2)]
        return not pending, pending

    def wait_placement_v2(self, timeout_s=10.0):
        """V2 service presence is a v2-mode deployment property."""
        client = self._chain_clients.get("placement_v2")
        if client is None:
            return False
        return client.wait_for_service(timeout_sec=timeout_s)

    def _call(self, name, request, timeout_s=30.0):
        """Synchronous service call with a REAL monotonic deadline (the
        contract's fixed 30s request budget)."""
        client = self._chain_clients[name]
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return {"error": "TIMEOUT", "deadline_s": timeout_s}
        return future.result()

    # -- fixture inputs ---------------------------------------------------
    def tick(self, sim_ns):
        """Advance /clock to sim_ns (fixture scenario time only)."""
        from rosgraph_msgs.msg import Clock
        message = Clock()
        message.clock.sec = int(sim_ns // 1_000_000_000)
        message.clock.nanosec = int(sim_ns % 1_000_000_000)
        self._clock_pub.publish(message)

    def start_clock(self, sim_start_ns, rate_hz=50.0, wall_ratio=1.0):
        """Continuous /clock master: sim time advances with real time.

        The producers' age premises (observer validity_max_age, planner
        robot max age) are judged in the sim domain, so scenario time must
        keep flowing between injected events — a stopped clock would age
        every VALID out. ``wall_ratio`` scales the advance rate (tests may
        run scenario time faster than the wall clock).
        """
        import threading
        # Exactly ONE clock master: an older thread would publish a
        # conflicting (earlier) sim time and drag consumers' now()
        # backwards — staleness probes depend on monotone scenario time.
        for stop in getattr(self, "_clock_stops", []):
            stop.set()
        self._clock_state = {"sim_ns": int(sim_start_ns),
                             "wall": time.monotonic(),
                             "ratio": float(wall_ratio)}
        self._clock_stop = threading.Event()
        self._clock_stops = [self._clock_stop]

        def _run():
            period = 1.0 / rate_hz
            while not self._clock_stop.wait(period):
                now_wall = time.monotonic()
                state = self._clock_state
                state["sim_ns"] += int(
                    (now_wall - state["wall"]) * 1e9 * state["ratio"])
                state["wall"] = now_wall
                self.tick(state["sim_ns"])

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()
        return self._clock_stop

    def start_robot_stream(self, interval_s=0.10, stamp_offset_ns=50_000_000):
        """Continuous frozen robot fixture: fresh joint_states at the
        real subscription entry, stamped just behind the running scenario
        clock (the production robot publishes a sequence, not one shot —
        a solve that outlives one sample's 500ms window is normal)."""
        import threading
        stop = threading.Event()
        # Every stream ever started is tracked: restarting never leaves
        # an older thread publishing (stale-robot subprobes depend on
        # ALL streams being stoppable).
        self._robot_streams = getattr(self, "_robot_streams", [])
        self._robot_streams.append(stop)

        def _run():
            while not stop.wait(interval_s):
                try:
                    now = self.clock_now_ns()
                    self.publish_joints(now - stamp_offset_ns, spin=False)
                except Exception:  # noqa: BLE001 - stream must survive
                    pass

        threading.Thread(target=_run, daemon=True).start()
        return stop

    def stop_robot_streams(self):
        for stop in getattr(self, "_robot_streams", []):
            stop.set()
        self._robot_streams = []

    def clock_now_ns(self):
        state = getattr(self, "_clock_state", None)
        if state is None:
            raise RuntimeError("continuous clock not started")
        now_wall = time.monotonic()
        return int(state["sim_ns"] + (now_wall - state["wall"]) * 1e9
                   * state["ratio"])

    @staticmethod
    def _matrix_to_quaternion(rotation):
        """3x3 row-major rotation matrix (fixture form) -> quaternion."""
        if isinstance(rotation, dict):
            return (float(rotation.get("x", 0.0)),
                    float(rotation.get("y", 0.0)),
                    float(rotation.get("z", 0.0)),
                    float(rotation.get("w", 1.0)))
        m = [float(v) for v in rotation]
        if len(m) != 9:
            raise ValueError("rotation must be a dict or 3x3 matrix")
        r00, r01, r02, r10, r11, r12, r20, r21, r22 = m
        trace = r00 + r11 + r22
        if trace > 0.0:
            scale = (trace + 1.0) ** 0.5 * 2.0
            quat = ((r21 - r12) / scale, (r02 - r20) / scale,
                    (r10 - r01) / scale, 0.25 * scale)
        elif r00 > r11 and r00 > r22:
            scale = (1.0 + r00 - r11 - r22) ** 0.5 * 2.0
            quat = (0.25 * scale, (r01 + r10) / scale,
                    (r02 + r20) / scale, (r21 - r12) / scale)
        elif r11 > r22:
            scale = (1.0 + r11 - r00 - r22) ** 0.5 * 2.0
            quat = ((r01 + r10) / scale, 0.25 * scale,
                    (r12 + r21) / scale, (r02 - r20) / scale)
        else:
            scale = (1.0 + r22 - r00 - r11) ** 0.5 * 2.0
            quat = ((r02 + r20) / scale, (r12 + r21) / scale,
                    0.25 * scale, (r10 - r01) / scale)
        norm = sum(v * v for v in quat) ** 0.5
        return tuple(v / norm for v in quat)

    def broadcast_transform(self, transform_dict, stamp_ns, static=False,
                            frame_id=None, child_frame_id="container_link"):
        """One geometry_msgs/TransformStamped from a fixture record.

        Fixture form: translation [x, y, z] + 3x3 row-major rotation
        matrix (converted to a quaternion here, reversibly).
        """
        from geometry_msgs.msg import TransformStamped
        message = TransformStamped()
        message.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        message.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        message.header.frame_id = str(
            frame_id or transform_dict.get("header", {}).get(
                "frame_id", "optical_frame"))
        message.child_frame_id = str(
            transform_dict.get("child_frame_id", child_frame_id))
        translation = transform_dict.get("translation", [0.0, 0.0, 0.0])
        if isinstance(translation, dict):
            translation = [translation.get("x", 0.0),
                           translation.get("y", 0.0),
                           translation.get("z", 0.0)]
        message.transform.translation.x = float(translation[0])
        message.transform.translation.y = float(translation[1])
        message.transform.translation.z = float(translation[2])
        qx, qy, qz, qw = self._matrix_to_quaternion(
            transform_dict.get("rotation", [0.0, 0.0, 0.0, 1.0]))
        message.transform.rotation.x = qx
        message.transform.rotation.y = qy
        message.transform.rotation.z = qz
        message.transform.rotation.w = qw
        if static:
            self._tf_static.sendTransform(message)
        else:
            self._tf.sendTransform(message)

    def publish_cloud(self, points, frame_id, stamp_ns, spin=3):
        cloud = _points_to_cloud(points, frame_id,
                                 stamp_ns // 1_000_000_000,
                                 stamp_ns % 1_000_000_000)
        self._cloud_pub.publish(cloud)
        for _ in range(spin):
            rclpy.spin_once(self, timeout_sec=0.02)

    def publish_joints(self, stamp_ns, zero_stamp=False, names=None,
                       spin=True):
        message = JointState()
        if not zero_stamp:
            message.header.stamp.sec = int(stamp_ns // 1_000_000_000)
            message.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        message.name = list(names or ["elfin_joint%d" % i
                                      for i in range(1, 7)])
        message.position = [0.0] * len(message.name)
        self._joint_pub.publish(message)
        if spin:
            # Single-threaded callers only: the executor is not reentrant
            # (the robot stream thread publishes with spin=False).
            rclpy.spin_once(self, timeout_sec=0.05)

    # -- observations -----------------------------------------------------
    def integrate_frame(self, frame_doc, *, observation_suffix="",
                        timeout_s=30.0):
        """One fixture observation through the REAL intake.

        G: publish the cloud on the production topic (freshness-windowed
        subscription), then trigger the production integrate service.
        O: IntegrateObservation with the document's points + transform
        (replay-unique stamps/ids, the frozen replay rule).
        """
        points = frame_doc.get("points_m") or []
        stamp_ns = int(frame_doc.get("stamp_ns", 0) or 0)
        result = {}
        if self.backend == "G":
            self.publish_cloud(points, str(
                frame_doc.get("optical_frame", "optical_frame")), stamp_ns)
            from luggage_msgs.srv import GetCargoMapStats, IntegrateCargoView
            if "stats" not in self._chain_clients:
                self._chain_clients["stats"] = self.create_client(
                    GetCargoMapStats, "/cargo_map/get_stats")
                self._chain_clients["stats"].wait_for_service(
                    timeout_sec=5.0)
            # The product integrate contract pins geometry+revision (no
            # defaults): read the mapper's own stats first, then pin.
            stats = self._call("stats", GetCargoMapStats.Request(), 10.0)
            request = IntegrateCargoView.Request()
            request.schema_version = 1
            request.view_request_id = str(
                frame_doc.get("observation_id", "")) + observation_suffix
            request.source_acquisition_stamp.sec = int(
                stamp_ns // 1_000_000_000)
            request.source_acquisition_stamp.nanosec = int(
                stamp_ns % 1_000_000_000)
            request.settled = True
            request.frame_count = 1
            request.geometry_hash = str(getattr(
                stats, "geometry_hash", "") or "")
            request.expected_map_revision = int(getattr(
                stats, "map_revision", 0) or 0)
            result = self._call("integrate", request, timeout_s)
        else:
            from uld_occ_observer.srv import IntegrateObservation
            request = IntegrateObservation.Request()
            request.sensor_id = str(frame_doc.get("sensor_id", "d555"))
            request.observation_id = str(
                frame_doc.get("observation_id", "")) + observation_suffix
            request.clock_domain = "ros"
            # The observer pins epochs AND geometry on integrate: read
            # the CURRENT epoch from its own status service (never
            # assume 1) and carry the fixture document's geometry hash.
            status = self.get_validity()
            request.epoch = int(getattr(status, "epoch", 1) or 1)
            request.geometry_hash = str(frame_doc.get("geometry_hash", "")
                                        or "")
            translation = frame_doc.get("transform", {}).get(
                "translation", [0.0, 0.0, 0.0])
            qx, qy, qz, qw = self._matrix_to_quaternion(
                frame_doc.get("transform", {}).get(
                    "rotation", [0.0, 0.0, 0.0, 1.0]))
            request.uld_from_optical.transform.translation.x = float(translation[0])
            request.uld_from_optical.transform.translation.y = float(translation[1])
            request.uld_from_optical.transform.translation.z = float(translation[2])
            request.uld_from_optical.transform.rotation.x = qx
            request.uld_from_optical.transform.rotation.y = qy
            request.uld_from_optical.transform.rotation.z = qz
            request.uld_from_optical.transform.rotation.w = qw
            request.uld_from_optical.header.stamp.sec = int(
                stamp_ns // 1_000_000_000)
            request.uld_from_optical.header.stamp.nanosec = int(
                stamp_ns % 1_000_000_000)
            # uld_from_optical convention (ros_adapters): the header
            # says container_link -> optical_frame, the body carries the
            # optical->ULD motion (fixture yaml verbatim).
            request.uld_from_optical.header.frame_id = "container_link"
            request.uld_from_optical.child_frame_id = str(
                frame_doc.get("optical_frame", "optical_frame"))
            request.cloud = _points_to_cloud(
                points, request.uld_from_optical.child_frame_id,
                stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000)
            result = self._call("integrate_observation", request,
                                timeout_s)
        return result

    def reset(self, *, reset_id, expected_epoch="", reason="",
              timeout_s=30.0):
        if self.backend == "G":
            from luggage_msgs.srv import ResetCargoMapV2
            request = ResetCargoMapV2.Request()
            request.reset_id = str(reset_id)
            request.expected_epoch = str(expected_epoch)
            request.reason = str(reason)
        else:
            from uld_occ_observer.srv import ResetObservation
            request = ResetObservation.Request()
            request.reset_id = str(reset_id)
            request.reason = str(reason)
            # The observer REQUIRES expected_epoch == current (the store
            # throws EPOCH_MISMATCH otherwise): pin the live epoch.
            status = self.get_validity()
            current = int(getattr(status, "epoch", 1) or 1)
            request.expected_epoch = int(expected_epoch) if \
                expected_epoch else current
        return self._call("reset", request, timeout_s)

    def get_validity(self, timeout_s=10.0):
        """Raw status response (field names identical on both backends)."""
        backend = self.backend
        if backend == "G":
            from luggage_msgs.srv import GetCargoValidity
            request = GetCargoValidity.Request()
        else:
            from uld_occ_observer.srv import GetValidity
            request = GetValidity.Request()
        return self._call("validity", request, timeout_s)

    def validity_record(self):
        """Last observed validity payload (dict) from the stream."""
        return self._validity_seen

    # -- placement --------------------------------------------------------
    def place_v1(self, box_whd, *, geometry_hash="", placed=(),
                 max_candidates=8, timeout_s=30.0):
        from luggage_msgs.msg import DetectedLuggage
        from luggage_msgs.srv import ComputePlacement
        request = ComputePlacement.Request()
        request.box.width = float(box_whd[0])
        request.box.depth = float(box_whd[1])
        request.box.height = float(box_whd[2])
        request.box.height_valid = True
        request.box.height_source = 1  # MEASURED_SUPPORT
        request.geometry_hash = str(geometry_hash)
        request.max_candidates = int(max_candidates)
        return self._call("placement", request, timeout_s)

    # -- observables (absorb barriers) -------------------------------------
    def _on_validity(self, msg):
        try:
            self._validity_seen = json.loads(msg.data)
        except ValueError:
            pass

    def _on_surface_map(self, msg):
        try:
            self._surface_map = json.loads(msg.data)
        except ValueError:
            pass

    @property
    def surface_map(self):
        return self._surface_map

    def _on_last_result(self, msg):
        try:
            self._last_result = json.loads(msg.data)
        except ValueError:
            pass

    @property
    def last_result(self):
        return self._last_result

    def wait_until(self, predicate, timeout_s=10.0, spin_period=0.05):
        """REAL-monotonic bounded absorb barrier on a client observable."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=spin_period)
            try:
                if predicate():
                    return True
            except Exception:  # noqa: BLE001 - keep polling
                pass
        return False

    def wait_validity(self, state, timeout_s=10.0):
        return self.wait_until(
            lambda: (self._validity_seen or {}).get("state") == state,
            timeout_s)
