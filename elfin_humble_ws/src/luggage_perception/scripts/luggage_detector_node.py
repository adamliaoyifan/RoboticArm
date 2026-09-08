#!/usr/bin/env python3
"""Luggage detector: platform-free box estimation from RGB-D data.

ROS 2 port of the noetic luggage_detector_node. Subscribes to the semantic
cargo cloud and the preprocessed raw depth cloud, joins them by exact
acquisition stamp, runs the platform-free top/support estimators
(``luggage_perception.top_support_estimator`` via the gating pipeline in
``luggage_perception.platform_free_pipeline``), and serves
``detect_luggage``.

Platform-free height contract (docs/plans/platform_free_height_eng_todo.md):

- top surface, XY, yaw, width, depth are measured from the cargo cloud;
- height/center-Z are valid (FULL_3D) only when the local support plane
  was measured from the *same* acquisition stamp with settled geometry;
  otherwise the result is TOP_ONLY with an explicit reason;
- a catalog width/depth match may populate a numeric prior height with
  ``height_valid=false``;
- no GT fallback: ``GetCurrentBox``/spawner state never enters the online
  estimate (eval drivers do their own GT comparison).
"""

from __future__ import division

import json
import math
import threading
import time
from collections import OrderedDict

import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.exceptions import ParameterUninitializedException
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)

from geometry_msgs.msg import Point, Pose, Quaternion
from luggage_msgs.msg import DetectedLuggage, DetectionFrame, YoloDetections
from luggage_msgs.srv import DetectLuggage
from luggage_perception import ros_message_adapters as adapters
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
from std_msgs.msg import String
import tf2_ros

from luggage_description.box_catalog_utils import (
    box_catalog_entries,
    load_box_catalog,
)
from luggage_description.scene_tf_config_utils import (
    load_scene_tf_config,
    pickup_source_in_world,
    resolve_scene_tf_config_path,
)
from luggage_perception.detection_temporal_gate import (
    SuitcaseViewWait,
    should_retry_estimate,
)
from luggage_perception.cargo_instance_tracker import parse_current_box_payload
from luggage_perception.detection_frame_join import (
    ExactStampJoin,
    empty_cargo_pca_fields,
    pca_fields_from_failure,
    stamp_key,
)
from luggage_perception.locked_stamp_window import LockedStampWindow
from luggage_perception.motion_stability_filter import (
    detection_replay_fields,
)
from luggage_perception.platform_free_pipeline import (
    GATE_SUPPORT_REASONS,
    GeometryStatusGate,
    PlatformFreeDetector,
)
from luggage_perception.top_support_estimator import (
    GEOMETRY_FULL_3D,
    GEOMETRY_TOP_ONLY,
    TopSupportConfig,
)


def _transform_points_to_world(tf_buffer, points, source_frame, target_frame,
                               stamp, wall_timeout_sec=0.5,
                               poll_sec=0.02):
    """Rigid-body transform an (N,3) array into the target frame.

    The lookup retries a zero-timeout query on a *wall-clock* deadline:
    tf2 timeouts run on the node clock, and when the simulation clock
    stalls (gz_ros2_control failure) a sim-time timeout never expires —
    concurrent lookups then block forever and starve the executor
    (observed as the recurring launch-context detector wedge).
    """
    import time as _time
    deadline = _time.monotonic() + float(wall_timeout_sec)
    tf_msg = None
    err = None
    while True:
        try:
            tf_msg = tf_buffer.lookup_transform(
                target_frame, source_frame, stamp,
                rclpy.duration.Duration(seconds=0))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            err = str(exc)
        if _time.monotonic() >= deadline:
            break
        _time.sleep(poll_sec)
    if tf_msg is None:
        return None, err

    t = tf_msg.transform.translation
    r = tf_msg.transform.rotation
    qx, qy, qz, qw = r.x, r.y, r.z, r.w
    rot = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])
    trans = np.array([t.x, t.y, t.z])
    return points.dot(rot.T) + trans, None


class LuggageDetector(Node):

    def __init__(self):
        super().__init__("luggage_detector")
        self._group = ReentrantCallbackGroup()

        self.declare_parameter("scene_tf_config", "")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("use_semantic", False)
        self.declare_parameter("roi_margin", 0.5)
        self.declare_parameter("cloud_max_age_sec", 1.0)
        self.declare_parameter("catalog_match_tolerance", 0.08)
        self.declare_parameter("min_points", 50)
        self.declare_parameter("min_confidence", 0.70)
        # --- Platform-free geometry (E2/E3) ---
        # Pickup workspace center XY; empty (unset) -> scene_tf
        # pickup_source XY (measured static workspace geometry, allowed).
        # Declared as an empty DOUBLE_ARRAY so "unset" is distinct from
        # the legitimate value [0.0, 0.0].
        self.declare_parameter("workspace_center_xy",
                               Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("workspace_half_extents",
                               Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("min_luggage_height", 0.15)
        self.declare_parameter("max_luggage_height", 0.60)
        self.declare_parameter("support_inner_margin", 0.03)
        self.declare_parameter("support_outer_margin", 0.18)
        self.declare_parameter("min_support_points", 80)
        self.declare_parameter("normal_tolerance_deg", 5.0)
        self.declare_parameter("ransac_dist_thresh", 0.008)
        self.declare_parameter("min_support_sides", 2)
        self.declare_parameter("stability_window", 5)
        self.declare_parameter("stability_max_z_spread", 0.015)
        # auto | configured | auto_then_configured | top_only
        self.declare_parameter("support_mode", "auto")
        # PF-R3: bounded same-stamp status evidence buffer.
        self.declare_parameter("geometry_status_buffer_maxlen", 16)
        # Optional configured support Z. Empty string means omitted
        # (valid configuration; the auto estimator never sees it).
        self.declare_parameter("platform_z", "")
        self.declare_parameter(
            "cargo_cloud_topic", "/luggage/semantic/cargo_points")
        self.declare_parameter(
            "depth_topic", "/luggage/preprocessed/camera/depth/points")
        # Empty means use the cloud header frame_id. The preprocessor publishes
        # optical-frame points; override only if a consumer still sees raw gz
        # clouds labelled optical but stored in camera_link.
        self.declare_parameter("cloud_data_frame", "")
        # Uniform-grid downsample inside estimate_box, applied after the
        # ROI/platform band crop. 0.01 m is statistically equivalent for
        # plane/rectangle estimation while shrinking the fitting stages'
        # input ~20x (cargo clouds ~110k -> ~5k points); 0 disables.
        # Supported range <= 0.02 m (confidence is count-based; see the
        # estimator docstring).
        self.declare_parameter("voxel_size", 0.01)
        self.declare_parameter("estimate_retry_count", 4)
        self.declare_parameter("estimate_retry_period_sec", 0.25)
        # RGB MAD wait before PCA. Kept in tree but off: meas-vs-image
        # overlay alignment is the check that matters. 0 disables.
        self.declare_parameter("suitcase_update_timeout_sec", 0.0)
        self.declare_parameter("suitcase_update_mad", 10.0)
        self.declare_parameter("suitcase_stable_mad", 4.0)
        self.declare_parameter("suitcase_stable_frames", 2)
        self.declare_parameter(
            "color_topic", "/luggage/preprocessed/camera/color/image")
        self.declare_parameter("current_box_topic", "/luggage/current_box")
        self.declare_parameter(
            "preprocessor_status_topic", "/luggage/preprocessed/status")
        self.declare_parameter(
            "filter_stats_topic", "/semantic_point_filter/stats_json")
        self.declare_parameter(
            "yolo_topic", "/luggage/semantic/yolo_detections")
        self.declare_parameter(
            "detection_frame_topic", "/luggage/perception/detection_frame")
        self.declare_parameter("stream_stats_topic", "~/stream_stats_json")
        self.declare_parameter("join_buffer_maxlen", 10)

        scene_cfg_path = self.get_parameter("scene_tf_config").value
        if not scene_cfg_path:
            scene_cfg_path = resolve_scene_tf_config_path()
        scene_config = load_scene_tf_config(scene_cfg_path)
        self._source_xyz, _ = pickup_source_in_world(scene_config)

        catalog_config = load_box_catalog(scene_config=scene_config)
        self._catalog_entries = box_catalog_entries(catalog_config)

        self._world_frame = self.get_parameter("world_frame").value
        self._use_semantic = bool(self.get_parameter("use_semantic").value)
        self._cloud_data_frame = self.get_parameter("cloud_data_frame").value
        self._voxel_size = float(self.get_parameter("voxel_size").value)
        self._estimate_retry_count = max(
            0, int(self.get_parameter("estimate_retry_count").value))
        self._estimate_retry_period = max(
            0.0, float(self.get_parameter("estimate_retry_period_sec").value))
        self._suitcase_update_timeout = max(
            0.0, float(self.get_parameter("suitcase_update_timeout_sec").value))
        self._view_wait = SuitcaseViewWait(
            update_mad=float(self.get_parameter("suitcase_update_mad").value),
            stable_mad=float(self.get_parameter("suitcase_stable_mad").value),
            stable_frames=int(self.get_parameter("suitcase_stable_frames").value),
        )
        self._view_lock = threading.Lock()
        self._latest_rgb = None
        self._timing = {}
        self._roi_margin = float(self.get_parameter("roi_margin").value)
        self._cloud_max_age = float(self.get_parameter("cloud_max_age_sec").value)
        self._catalog_tol = float(
            self.get_parameter("catalog_match_tolerance").value)
        self._min_points = int(self.get_parameter("min_points").value)
        self._min_confidence = float(self.get_parameter("min_confidence").value)

        # Platform-free geometry pipeline (E2). The pickup workspace
        # defaults to the scene pickup XY (static workspace geometry) with
        # the ROI margin as half extents; pickup_source.z never enters.
        # Parameters declared as a bare DOUBLE_ARRAY start uninitialized,
        # so reading .value raises until they are set.
        def _ws_param(name):
            try:
                return list(self.get_parameter(name).value or [])
            except ParameterUninitializedException:
                return []
        ws_center = _ws_param("workspace_center_xy")
        if not ws_center:
            ws_center = [self._source_xyz[0], self._source_xyz[1]]
        ws_half = _ws_param("workspace_half_extents")
        if not ws_half:
            ws_half = [self._roi_margin, self._roi_margin]
        platform_z_raw = str(self.get_parameter("platform_z").value).strip()
        try:
            self._platform_z = (
                float(platform_z_raw) if platform_z_raw else None)
        except ValueError:
            self._platform_z = None
        self._support_mode = str(self.get_parameter("support_mode").value)
        self._pipeline = PlatformFreeDetector(
            config=TopSupportConfig(
                workspace_center_xy=ws_center,
                workspace_half_extents=ws_half,
                min_luggage_height=float(
                    self.get_parameter("min_luggage_height").value),
                max_luggage_height=float(
                    self.get_parameter("max_luggage_height").value),
                voxel_size=self._voxel_size,
                min_top_points=self._min_points,
                support_inner_margin=float(
                    self.get_parameter("support_inner_margin").value),
                support_outer_margin=float(
                    self.get_parameter("support_outer_margin").value),
                min_support_points=int(
                    self.get_parameter("min_support_points").value),
                ransac_dist_thresh=float(
                    self.get_parameter("ransac_dist_thresh").value),
                normal_tolerance_deg=float(
                    self.get_parameter("normal_tolerance_deg").value),
                min_support_sides=int(
                    self.get_parameter("min_support_sides").value),
            ),
            support_mode=self._support_mode,
            catalog_entries=self._catalog_entries,
            catalog_tolerance=self._catalog_tol,
            stability_window=int(
                self.get_parameter("stability_window").value),
            stability_max_z_spread=float(
                self.get_parameter("stability_max_z_spread").value),
        )
        # Bounded raw-depth world-point buffer keyed by exact stamp.
        self._raw_buffer = OrderedDict()
        self._raw_buffer_maxlen = max(
            4, int(self.get_parameter("join_buffer_maxlen").value))
        self._raw_lock = threading.Lock()
        self._raw_counts = {
            "raw_received": 0,
            "raw_evicted": 0,
            "raw_lookup_hit": 0,
            "raw_lookup_miss": 0,
            "raw_lookup_empty": 0,
            "raw_lookup_decode_fail": 0,
            "raw_lookup_tf_fail": 0,
        }
        self._last_raw_lookup = {}
        # PF-R3 rework: same-acquisition status join. Status evidence is
        # buffered by its exact (sec, nanosec) primary stamp; only the
        # entry for this cloud's acquisition can authorize support
        # fitting (N-1 or out-of-order evidence fails closed).
        self._geometry_gate = GeometryStatusGate(
            maxlen=max(4, int(self.get_parameter(
                "geometry_status_buffer_maxlen").value)))
        self._last_failure_reason = "not_run"
        self._last_cloud_stamp_sec = None
        self._status = {"payload": None}
        self._filter_stats = None
        self._box_epoch_seen = False
        self._box_id = ""
        self._box_generation = 0
        self._join = ExactStampJoin(
            maxlen=max(1, int(self.get_parameter("join_buffer_maxlen").value)))
        self._frame_window = LockedStampWindow(
            maxlen=max(1, int(self.get_parameter("join_buffer_maxlen").value)))
        self._frame_seq = 0
        self._frame_lock = threading.Lock()

        # Transient local replaces the ROS 1 latched diagnostics publisher.
        transient = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._diag_pub = self.create_publisher(
            String, "~/diagnostics_json", transient)
        self._stream_stats_pub = self.create_publisher(
            String, self.get_parameter("stream_stats_topic").value, transient)
        self._latest_pub = self.create_publisher(
            String, "/luggage/perception/detection/latest", transient)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._cloud_lock = threading.Lock()
        self._latest_cloud = None
        self._latest_stamp = None
        self._latest_frame = None

        stream_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        topic = (
            self.get_parameter("cargo_cloud_topic").value
            if self._use_semantic
            else self.get_parameter("depth_topic").value
        )
        self.create_subscription(
            PointCloud2, topic, self._cloud_cb, stream_qos,
            callback_group=self._group)
        self.create_subscription(
            YoloDetections, self.get_parameter("yolo_topic").value,
            self._yolo_cb, stream_qos, callback_group=self._group)
        # Raw depth cloud feeds the local support fit (exact-stamp keyed).
        # On the raw (non-semantic) path it is the same topic as _cloud_cb;
        # the buffer handles the duplicate insert idempotently.
        self.create_subscription(
            PointCloud2, self.get_parameter("depth_topic").value,
            self._raw_cloud_cb, stream_qos, callback_group=self._group)
        self._frame_pub = self.create_publisher(
            DetectionFrame,
            self.get_parameter("detection_frame_topic").value,
            stream_qos)
        self.get_logger().info("luggage_detector subscribing to %s" % topic)

        image_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            Image, self.get_parameter("color_topic").value,
            self._rgb_cb, image_qos, callback_group=self._group)
        self.create_subscription(
            String, self.get_parameter("current_box_topic").value,
            self._on_current_box, transient, callback_group=self._group)
        self.create_subscription(
            String, self.get_parameter("preprocessor_status_topic").value,
            self._on_status, transient, callback_group=self._group)
        self.create_subscription(
            String, self.get_parameter("filter_stats_topic").value,
            self._on_filter_stats, transient, callback_group=self._group)

        self.create_service(
            DetectLuggage, "/luggage_detector/detect_luggage",
            self.handle_detect, callback_group=self._group)
        if not self._use_semantic:
            # PF-R2 fail-closed: an unsegmented raw depth cloud is not a
            # luggage observation. The node stays up (motion/vacuum
            # workflows keep their infrastructure) but every detection
            # fails with DETECT_CARGO_SEGMENTATION_REQUIRED until the
            # semantic chain is enabled.
            self.get_logger().error(
                "luggage_detector: use_semantic=false rejects ALL "
                "detections with DETECT_CARGO_SEGMENTATION_REQUIRED "
                "(support_mode=%s): raw depth is not segmented cargo; "
                "launch with use_semantic:=true"
                % self._support_mode)
        self.get_logger().info(
            "luggage_detector ready (platform-free, semantic=%s, "
            "support_mode=%s, platform_z=%s, retries=%d period=%.2fs "
            "suitcase_wait=%.1fs)"
            % (self._use_semantic, self._support_mode,
               ("%.3f" % self._platform_z) if self._platform_z is not None
               else "omitted",
               self._estimate_retry_count,
               self._estimate_retry_period, self._suitcase_update_timeout))

    def _cloud_cb(self, msg):
        with self._cloud_lock:
            self._latest_cloud = msg
            self._latest_stamp = msg.header.stamp
            self._latest_frame = msg.header.frame_id
        key = stamp_key(msg.header.stamp)
        if key is None:
            return
        if self._use_semantic:
            pair = self._join.push_right(key, msg)
            if pair is not None:
                self._emit_joined(pair[0], pair[1])
            return
        self._emit_joined(self._empty_yolo_for_cloud(msg), msg)

    def _yolo_cb(self, msg):
        key = stamp_key(msg.header.stamp)
        if key is None:
            return
        pair = self._join.push_left(key, msg)
        if pair is not None:
            self._emit_joined(pair[0], pair[1])

    def _raw_cloud_cb(self, msg):
        """Buffer the raw depth cloud message by exact stamp (PF-R6 opt 1).

        Decoding + TFing every raw cloud (~250k points at 4 Hz) competed
        with the join/geometry callbacks on the same executor — the
        dominant yolo->frame latency in the PF-R6 baseline. The message
        is already in memory from the subscription, so buffering it is
        free; the transform happens lazily, once, only for stamps whose
        cargo cloud actually joins (see _pop_raw_world_with_retry).
        """
        key = stamp_key(msg.header.stamp)
        if key is None:
            return
        with self._raw_lock:
            self._raw_counts["raw_received"] += 1
            self._raw_buffer[key] = msg
            while len(self._raw_buffer) > self._raw_buffer_maxlen:
                self._raw_buffer.popitem(last=False)
                self._raw_counts["raw_evicted"] += 1

    def _pop_raw_world_with_retry(self, key, attempts=3, period_sec=0.02):
        """Exact-stamp raw world points, transformed on demand (PF-R6).

        The raw cloud is buffered lazily as a message by
        ``_raw_cloud_cb``; it is decoded + transformed here once, only
        for stamps whose cargo cloud joined (the dominant PF-R6 baseline
        cost was transforming every raw cloud at 4 Hz on the shared
        executor). The bounded same-stamp re-check covers the executor
        ordering race and never accepts a different stamp — no fusion
        semantics change.
        """
        start = time.monotonic()
        last = max(1, attempts)
        lookup = {
            "raw_lookup_key": list(key) if key is not None else None,
            "raw_lookup_attempts": 0,
            "raw_lookup_status": "miss",
            "raw_lookup_wait_ms": 0.0,
            "raw_buffer_len": 0,
            "raw_buffer_maxlen": int(self._raw_buffer_maxlen),
        }
        for attempt in range(last):
            msg = None
            with self._raw_lock:
                lookup["raw_lookup_attempts"] = attempt + 1
                lookup["raw_buffer_len"] = len(self._raw_buffer)
                msg = self._raw_buffer.get(key)
            if msg is not None:
                pts = adapters.cloud_points_from_msg(msg)
                if pts is None:
                    lookup["raw_lookup_status"] = "decode_fail"
                    with self._raw_lock:
                        self._raw_counts["raw_lookup_decode_fail"] += 1
                    break
                pts = pts[np.isfinite(pts).all(axis=1)]
                if not len(pts):
                    lookup["raw_lookup_status"] = "empty"
                    with self._raw_lock:
                        self._raw_counts["raw_lookup_empty"] += 1
                    break
                stamp_time = rclpy.time.Time.from_msg(msg.header.stamp)
                source_frame = self._cloud_data_frame or msg.header.frame_id
                pts_world, _err = _transform_points_to_world(
                    self._tf_buffer, pts, source_frame,
                    self._world_frame, stamp_time)
                if pts_world is not None:
                    lookup["raw_lookup_status"] = "hit"
                    lookup["raw_lookup_wait_ms"] = (
                        time.monotonic() - start) * 1000.0
                    with self._raw_lock:
                        self._raw_counts["raw_lookup_hit"] += 1
                        self._last_raw_lookup = dict(lookup)
                    return pts_world
                lookup["raw_lookup_status"] = "tf_fail"
                with self._raw_lock:
                    self._raw_counts["raw_lookup_tf_fail"] += 1
                break
            if attempt + 1 < last:
                time.sleep(period_sec)
        lookup["raw_lookup_wait_ms"] = (time.monotonic() - start) * 1000.0
        if lookup["raw_lookup_status"] == "miss":
            with self._raw_lock:
                self._raw_counts["raw_lookup_miss"] += 1
        with self._raw_lock:
            self._last_raw_lookup = dict(lookup)
        return None

    def _empty_yolo_for_cloud(self, cloud_msg):
        msg = YoloDetections()
        msg.header = cloud_msg.header
        msg.generation = int(self._box_generation)
        msg.instance_id = str(self._box_id)
        return msg

    def _rgb_cb(self, msg):
        image = adapters.image_array_from_msg(msg)
        if image is None:
            return
        with self._view_lock:
            self._latest_rgb = image

    def _on_current_box(self, msg):
        box_id, generation = parse_current_box_payload(msg.data)
        self._box_epoch_seen = True
        self._box_id = box_id
        self._box_generation = generation
        self._join.clear()
        self._frame_window.clear()
        with self._raw_lock:
            self._raw_buffer.clear()
        # New luggage instance: support-Z history must not leak across boxes.
        self._pipeline.reset()
        with self._view_lock:
            rgb = self._latest_rgb
            changed = self._view_wait.note_box_id(box_id, rgb)
        if changed and box_id:
            self.get_logger().info(
                "luggage_detector: waiting for suitcase RGB update (%s)"
                % box_id)

    def _on_filter_stats(self, msg):
        if not msg.data:
            return
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(data, dict):
            self._filter_stats = data

    def _on_status(self, msg):
        self._status["payload"] = msg.data
        # PF-R3: buffer the payload by its stamped acquisition.
        # Unparseable JSON is stored as the raw string so the gate
        # reports status_malformed rather than status_missing.
        if msg.data:
            try:
                payload = json.loads(msg.data)
            except (TypeError, ValueError):
                payload = msg.data  # non-dict -> status_malformed
        else:
            payload = None
        self._geometry_gate.update(payload)

    def _status_data(self):
        payload = self._status.get("payload")
        if not payload:
            return None
        try:
            data = json.loads(payload)
        except (TypeError, ValueError):
            return None
        return data if isinstance(data, dict) else None

    def _instance_gate_reason(self):
        """None when DetectLuggage may run. Skip when no current_box (real robot)."""
        if not self._use_semantic or not self._box_epoch_seen:
            return None
        if not self._box_id:
            return "DETECT_STALE_INSTANCE"
        stats = self._filter_stats
        if not stats:
            return "DETECT_STALE_INSTANCE"
        try:
            gen = int(stats.get("generation") or 0)
            raw = stats.get("last_cargo_n_points")
            if raw is None:
                raw = stats.get("n_points", 0)
            n_points = int(raw if raw is not None else 0)
        except (TypeError, ValueError):
            return "DETECT_STALE_INSTANCE"
        if gen != int(self._box_generation):
            return "DETECT_STALE_INSTANCE"
        if n_points <= 0:
            return "DETECT_NO_CLOUD"
        return None

    def _wait_instance_ready(self):
        """Block until tracker generation matches current_box, or timeout."""
        if not self._use_semantic or not self._box_epoch_seen:
            return True
        timeout = max(
            0.5,
            (1.0 + float(self._estimate_retry_count))
            * float(self._estimate_retry_period))
        deadline = time.time() + timeout
        while time.time() < deadline:
            reason = self._instance_gate_reason()
            if reason is None:
                return True
            self._last_failure_reason = reason
            time.sleep(0.02)
        if self._instance_gate_reason() is None:
            return True
        return False

    def _wait_suitcase_view(self):
        """Block until RGB leaves the pre-spawn view, or timeout.

        Returns True when detect may run. Timeout is a hard miss: the pixels
        are still the previous suitcase, so GT fallback would hide that.
        """
        if self._suitcase_update_timeout <= 0.0:
            return True
        with self._view_lock:
            if not self._view_wait.pending:
                return True
            if self._latest_rgb is None:
                # Raw-depth launches may have no preprocessor RGB.
                return True
        deadline = time.time() + self._suitcase_update_timeout
        while time.time() < deadline:
            with self._view_lock:
                rgb = self._latest_rgb
                self._view_wait.set_baseline_if_missing(rgb)
                ready = self._view_wait.observe(rgb)
            if ready:
                prev = self._cloud_stamp_key()
                remaining = deadline - time.time()
                self._wait_newer_cloud(prev, max(0.0, min(0.5, remaining)))
                return True
            time.sleep(0.02)
        return False

    def _cloud_stamp_key(self):
        with self._cloud_lock:
            stamp = self._latest_stamp
        if stamp is None:
            return None
        return (int(stamp.sec), int(stamp.nanosec))

    def _wait_newer_cloud(self, prev_key, timeout):
        """Block until cargo/depth stamp changes, or *timeout* elapses."""
        if timeout <= 0.0:
            return False
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            time.sleep(0.02)
            key = self._cloud_stamp_key()
            if key is not None and key != prev_key:
                return True
        return False

    def _estimate_with_retries(self):
        attempts = 1 + int(self._estimate_retry_count)
        detected, confidence = None, 0.0
        for attempt in range(attempts):
            prev_key = self._cloud_stamp_key()
            detected, confidence = self._estimate_from_cloud()
            if detected is not None:
                if attempt:
                    self.get_logger().info(
                        "luggage_detector: estimate ok on retry %d/%d"
                        % (attempt, self._estimate_retry_count))
                return detected, confidence
            if attempt + 1 >= attempts:
                break
            if not should_retry_estimate(self._last_failure_reason):
                break
            self._wait_newer_cloud(prev_key, self._estimate_retry_period)
        return detected, confidence

    # ------------------------------------------------------------------
    # Perception path
    # ------------------------------------------------------------------

    def _warn_throttled(self, msg):
        # Replaces rospy.logwarn_throttle; 5 s window on the node clock.
        now = self.get_clock().now().nanoseconds
        last = getattr(self, "_last_warn_ns", 0)
        if now - last > 5e9:
            self.get_logger().warning(msg)
            self._last_warn_ns = now

    def _pca_source_label(self, n_points):
        if int(n_points) <= 0:
            return "empty"
        stats = self._filter_stats or {}
        source = str(stats.get("source") or "")
        if source in ("measure", "hold_track", "empty"):
            return source
        return "measure"

    def _geometry_ok(self):
        """Legacy helper: settled flag for diagnostics (None = absent)."""
        data = self._status_data()
        if not data:
            return None
        flags = data.get("flags")
        if isinstance(flags, dict) and "geometry_ok" in flags:
            return bool(flags["geometry_ok"])
        return None

    def _evaluate_geometry_gate(self, stamp):
        """PF-R3 same-acquisition status validation -> (ok, gate_reason)."""
        return self._geometry_gate.evaluate(
            int(stamp.sec), cloud_stamp_nanosec=int(stamp.nanosec))

    def _support_fields(self, result):
        """Map a PipelineResult onto DetectionFrame support diagnostics."""
        support = result.support
        if support is None:
            gate = result.support_gate or "no_top"
            return {
                "support_valid": False,
                "support_reason": GATE_SUPPORT_REASONS.get(
                    gate, "DETECT_SUPPORT_UNOBSERVABLE"),
                "support_gate": gate,
                "support_z": float("nan"),
                "support_confidence": 0.0,
                "support_residual": float("nan"),
                "support_side_coverage": 0.0,
                "support_inliers": 0,
            }
        return {
            "support_valid": bool(support.reason == "ok"),
            "support_reason": str(support.reason),
            "support_gate": result.support_gate,
            "support_z": float(support.support_z),
            "support_confidence": float(support.confidence),
            "support_residual": float(support.residual),
            "support_side_coverage": float(support.side_coverage),
            "support_inliers": int(support.inlier_count),
        }

    def _detected_from_result(self, result, cloud_msg, confidence):
        """Build a DetectedLuggage honoring the E0 validity contract.

        - ``top_surface_pose.z`` is the measured pickup contact Z;
        - ``pose.position.z``/``height`` are measurements only when
          ``height_valid`` (else pose.z falls back to the top Z and the
          numeric height stays whatever prior produced it);
        - ``header`` carries the acquisition stamp/frame.
        """
        box = result.box
        top = box.top
        height_valid = bool(result.height_valid)
        if height_valid and box.center_xyz is not None:
            center_z = float(box.center_xyz[2])
        else:
            center_z = float(top.top_z)
        msg = DetectedLuggage()
        msg.id = "detected_box"
        msg.width = float(box.width)
        msg.depth = float(box.depth)
        msg.height = float(box.height)
        msg.yaw_valid = bool(top.yaw_valid)
        msg.aspect_ratio = float(top.aspect_ratio)
        msg.pose = Pose(
            position=Point(
                x=float(top.center_xy[0]),
                y=float(top.center_xy[1]),
                z=center_z,
            ),
            orientation=Quaternion(
                z=float(math.sin(top.yaw * 0.5)),
                w=float(math.cos(top.yaw * 0.5)),
            ),
        )
        msg.header = Header(
            stamp=cloud_msg.header.stamp, frame_id=self._world_frame)
        msg.top_surface_pose = Pose(
            position=Point(
                x=float(top.center_xy[0]),
                y=float(top.center_xy[1]),
                z=float(top.top_z),
            ),
            orientation=Quaternion(
                z=float(math.sin(top.yaw * 0.5)),
                w=float(math.cos(top.yaw * 0.5)),
            ),
        )
        msg.top_surface_valid = True
        msg.top_surface_confidence = float(confidence)
        msg.height_valid = height_valid
        msg.height_confidence = float(
            result.support.confidence if (
                height_valid and result.support is not None) else 0.0)
        msg.height_source = int(result.height_source)
        return msg

    def _pca_from_cloud_msg(self, cloud_msg):
        """Platform-free fit from one cargo/depth acquisition.

        Returns ``(pca_fields, DetectedLuggage or None, support_fields)``.
        Always returns fields so the stream can publish invalid frames.
        """
        stamp = cloud_msg.header.stamp
        frame = cloud_msg.header.frame_id
        stamp_time = rclpy.time.Time.from_msg(stamp)
        self._timing = {}
        _t0 = time.monotonic()
        pts_camera = adapters.cloud_points_from_msg(cloud_msg)
        if pts_camera is None:
            return (pca_fields_from_failure(
                "DETECT_CLOUD_DECODE_FAILED", 0, "empty"),
                None, self._support_fields_empty())
        pts_camera = pts_camera[np.isfinite(pts_camera).all(axis=1)]
        self._timing["read_ms"] = (time.monotonic() - _t0) * 1000.0
        n_points = int(len(pts_camera))
        source = self._pca_source_label(n_points)
        if n_points <= 0:
            return (empty_cargo_pca_fields(0), None,
                    self._support_fields_empty())
        if n_points < self._min_points:
            return (pca_fields_from_failure(
                "DETECT_TOO_FEW_POINTS", n_points, source),
                None, self._support_fields_empty())

        _t0 = time.monotonic()
        source_frame = self._cloud_data_frame or frame
        pts_world, tf_err = _transform_points_to_world(
            self._tf_buffer, pts_camera, source_frame,
            self._world_frame, stamp_time,
        )
        if pts_world is None:
            return (pca_fields_from_failure(
                "DETECT_TF_FAILED", n_points, source),
                None, self._support_fields_empty())
        self._timing["tf_ms"] = (time.monotonic() - _t0) * 1000.0
        centroid = tuple(float(v) for v in pts_world.mean(axis=0))

        raw_world = self._pop_raw_world_with_retry(stamp_key(stamp))
        # PF-R3: the support fit runs only when same-acquisition status
        # evidence exists for this exact stamp. N-1, out-of-order,
        # missing, malformed, or not-settled status yields TOP_ONLY with
        # its own machine reason — never a measured height.
        geometry_ok, geometry_gate_reason = self._evaluate_geometry_gate(stamp)
        cloud_stamp_sec = float(adapters.stamp_to_sec(stamp))
        _t0 = time.monotonic()
        result = self._pipeline.update(
            pts_world, raw_world,
            source=source,
            geometry_ok=geometry_ok,
            geometry_gate_reason=geometry_gate_reason or None,
            platform_z=self._platform_z,
            stamp_sec=cloud_stamp_sec,
            cargo_segmented=self._use_semantic)
        self._timing["geometry_ms"] = (time.monotonic() - _t0) * 1000.0
        self._timing["pipeline"] = dict(result.timing)
        support_fields = self._support_fields(result)

        if not result.top_valid:
            return (pca_fields_from_failure(
                result.top_reason, n_points, source, centroid),
                None, support_fields)
        top = result.box.top
        if float(top.confidence) < self._min_confidence:
            fields = pca_fields_from_failure(
                "DETECT_LOW_CONFIDENCE", n_points, source,
                (float(top.center_xy[0]), float(top.center_xy[1]),
                 float(top.top_z)))
            fields["pca_confidence"] = float(top.confidence)
            return fields, None, support_fields
        fields = {
            "pca_valid": True,
            "pca_reason": "ok",
            "pca_source": source,
            "pca_confidence": float(top.confidence),
            "n_cargo_points": n_points,
            "centroid": centroid,
        }
        return (fields,
                self._detected_from_result(
                    result, cloud_msg, float(top.confidence)),
                support_fields)

    def _support_fields_empty(self):
        return {
            "support_valid": False,
            "support_reason": "DETECT_SUPPORT_UNOBSERVABLE",
            "support_gate": "no_top",
            "support_z": float("nan"),
            "support_confidence": 0.0,
            "support_residual": float("nan"),
            "support_side_coverage": 0.0,
            "support_inliers": 0,
        }

    def _make_detection_frame(self, yolo_msg, cloud_msg, fields, box,
                              support):
        msg = DetectionFrame()
        msg.header.stamp = cloud_msg.header.stamp
        msg.header.frame_id = self._world_frame
        msg.yolo_optical_frame = (
            yolo_msg.header.frame_id or cloud_msg.header.frame_id)
        with self._frame_lock:
            msg.frame_seq = self._frame_seq
            self._frame_seq += 1
        msg.generation = int(yolo_msg.generation)
        msg.instance_id = str(yolo_msg.instance_id)
        msg.yolo = list(yolo_msg.detections)
        msg.pca_valid = bool(fields["pca_valid"])
        msg.pca_reason = str(fields["pca_reason"])
        msg.pca_source = str(fields["pca_source"])
        msg.pca_confidence = float(fields["pca_confidence"])
        msg.n_cargo_points = int(fields["n_cargo_points"])
        cx, cy, cz = fields["centroid"]
        msg.centroid = Point(x=float(cx), y=float(cy), z=float(cz))
        # --- Support diagnostics (E0 contract) ---
        msg.support_valid = bool(support["support_valid"])
        msg.support_reason = str(support["support_reason"])
        msg.support_z = float(support["support_z"])
        msg.support_confidence = float(support["support_confidence"])
        msg.support_residual = float(support["support_residual"])
        msg.support_side_coverage = float(support["support_side_coverage"])
        msg.support_inliers = int(support["support_inliers"])
        msg.geometry_level = int(
            GEOMETRY_FULL_3D if (
                box is not None and box.height_valid)
            else GEOMETRY_TOP_ONLY)
        if box is not None and fields["pca_valid"]:
            msg.box = box
        return msg

    def _emit_joined(self, yolo_msg, cloud_msg):
        fields, box, support = self._pca_from_cloud_msg(cloud_msg)
        if not fields["pca_valid"]:
            reason = fields["pca_reason"]
            if reason in (
                    "DETECT_CLOUD_DECODE_FAILED",
                    "DETECT_TF_FAILED",
                    "DETECT_TOO_FEW_POINTS"):
                self._warn_throttled(
                    "luggage_detector: stream %s (n=%d)"
                    % (reason, fields["n_cargo_points"]))
        frame = self._make_detection_frame(
            yolo_msg, cloud_msg, fields, box, support)
        self._frame_pub.publish(frame)
        self._frame_window.push(
            adapters.stamp_to_sec(cloud_msg.header.stamp), frame)
        self._publish_stream_stats(frame, fields, support, yolo_msg, cloud_msg)

    def _publish_stream_stats(self, frame, fields, support, yolo_msg, cloud_msg):
        with self._raw_lock:
            raw_counts = dict(self._raw_counts)
            raw_lookup = dict(self._last_raw_lookup)
            raw_buffer_len = len(self._raw_buffer)
        record = {
            "stamp": (
                float(cloud_msg.header.stamp.sec)
                + 1e-9 * float(cloud_msg.header.stamp.nanosec)),
            "stamp_key": [
                int(cloud_msg.header.stamp.sec),
                int(cloud_msg.header.stamp.nanosec),
            ],
            "frame_seq": int(frame.frame_seq),
            "generation": int(frame.generation),
            "instance_id": str(frame.instance_id),
            "yolo_count": int(len(yolo_msg.detections)),
            "pca_valid": bool(fields["pca_valid"]),
            "pca_reason": str(fields["pca_reason"]),
            "pca_source": str(fields["pca_source"]),
            "pca_confidence": float(fields["pca_confidence"]),
            "n_cargo_points": int(fields["n_cargo_points"]),
            "geometry_level": int(frame.geometry_level),
            "support_valid": bool(support["support_valid"]),
            "support_reason": str(support["support_reason"]),
            "support_gate": str(support["support_gate"]),
            "support_inliers": int(support["support_inliers"]),
            "timing_ms": dict(self._timing),
            "raw_buffer_len": int(raw_buffer_len),
            "raw_buffer_maxlen": int(self._raw_buffer_maxlen),
            "raw_counts": raw_counts,
            "raw_lookup": raw_lookup,
            "filter_stats": self._filter_stats or {},
        }
        self._stream_stats_pub.publish(
            String(data=json.dumps(record, sort_keys=True)))

    def _wait_newer_frame(self, prev_stamp, timeout):
        if timeout <= 0.0:
            return False
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            time.sleep(0.02)
            hit = self._frame_window.latest()
            if hit is not None and hit[0] != prev_stamp:
                return True
        return False

    def _frame_to_detect(self, frame):
        stamp_time = rclpy.time.Time.from_msg(frame.header.stamp)
        age = (self.get_clock().now() - stamp_time).nanoseconds / 1e9
        self._last_cloud_stamp_sec = (
            float(frame.header.stamp.sec)
            + 1e-9 * float(frame.header.stamp.nanosec))
        if age > self._cloud_max_age:
            self._last_failure_reason = "DETECT_STALE_CLOUD"
            return None, 0.0
        if self._use_semantic and self._box_epoch_seen:
            if int(frame.generation) != int(self._box_generation):
                self._last_failure_reason = "DETECT_STALE_INSTANCE"
                return None, 0.0
        if not frame.pca_valid:
            self._last_failure_reason = str(
                frame.pca_reason or "DETECT_ESTIMATION_FAILED")
            return None, float(frame.pca_confidence)
        self._last_failure_reason = "ok"
        return frame.box, float(frame.pca_confidence)

    def _detect_from_window_with_retries(self):
        attempts = 1 + int(self._estimate_retry_count)
        detected, confidence = None, 0.0
        prev_stamp = None
        for attempt in range(attempts):
            hit = self._frame_window.latest()
            if hit is None:
                self._last_failure_reason = "DETECT_NO_CLOUD"
            else:
                prev_stamp, frame = hit
                detected, confidence = self._frame_to_detect(frame)
                if detected is not None:
                    if attempt:
                        self.get_logger().info(
                            "luggage_detector: window ok on retry %d/%d"
                            % (attempt, self._estimate_retry_count))
                    return detected, confidence
            if attempt + 1 >= attempts:
                break
            if not should_retry_estimate(self._last_failure_reason):
                break
            self._wait_newer_frame(prev_stamp, self._estimate_retry_period)
        return detected, confidence

    def _estimate_from_cloud(self):
        """Run the full perception pipeline on the latest cloud.

        Used when the DetectionFrame window is empty (raw-depth path).
        """
        with self._cloud_lock:
            cloud_msg = self._latest_cloud
            stamp = self._latest_stamp
        self._last_cloud_stamp_sec = None
        if stamp is not None:
            self._last_cloud_stamp_sec = (
                float(stamp.sec) + 1e-9 * float(stamp.nanosec))

        if cloud_msg is None:
            self._last_failure_reason = "DETECT_NO_CLOUD"
            self._warn_throttled("luggage_detector: no point cloud received yet")
            return None, 0.0

        stamp_time = rclpy.time.Time.from_msg(stamp)
        age = (self.get_clock().now() - stamp_time).nanoseconds / 1e9
        if age > self._cloud_max_age:
            self._last_failure_reason = "DETECT_STALE_CLOUD"
            self._warn_throttled(
                "luggage_detector: cloud too old (%.2fs)" % age)
            return None, 0.0

        fields, box, support = self._pca_from_cloud_msg(cloud_msg)
        self._last_failure_reason = (
            "ok" if fields["pca_valid"] else fields["pca_reason"])
        if fields["pca_valid"]:
            t = self._timing
            self.get_logger().info(
                "detect timing: read=%.1fms tf=%.1fms geometry=%.1fms"
                % (t.get("read_ms", -1), t.get("tf_ms", -1),
                   t.get("geometry_ms", -1)))
            self.get_logger().info(
                "luggage_detector: estimated box '%s' top_z=%.3f "
                "xy=(%.3f, %.3f) size=(%.3f, %.3f) conf=%.2f yaw_valid=%s "
                "height_valid=%s height_source=%d support_z=%s reason=%s"
                % (box.id, box.top_surface_pose.position.z,
                   box.pose.position.x, box.pose.position.y,
                   box.width, box.depth,
                   fields["pca_confidence"], box.yaw_valid,
                   box.height_valid, box.height_source,
                   ("%.3f" % support["support_z"])
                   if support["support_z"] == support["support_z"] else "nan",
                   support["support_reason"]))
            return box, fields["pca_confidence"]
        self._warn_throttled(
            "luggage_detector: %s" % self._last_failure_reason)
        return None, fields["pca_confidence"]

    def _publish_diagnostics(self, source, success, confidence, reason,
                             detected=None):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        ctype = self.get_clock().clock_type
        clock_name = ctype.name if hasattr(ctype, "name") else str(ctype)
        record = {
            "stamp": now_sec,
            "source": source,
            "success": bool(success),
            "confidence": float(confidence),
            "reason": str(reason),
        }
        record.update(detection_replay_fields(
            self._status_data(),
            self._last_cloud_stamp_sec,
            now_sec,
            clock_name,
        ))
        record["box_generation"] = int(self._box_generation)
        record["box_id"] = str(self._box_id)
        stats = self._filter_stats or {}
        record["cargo_generation"] = int(stats.get("generation") or 0)
        record["cargo_n_points"] = int(
            stats.get("last_cargo_n_points", stats.get("n_points", 0)) or 0)
        record["cargo_source"] = str(stats.get("source") or "")
        record["support_mode"] = self._support_mode
        if detected is not None:
            record["detected"] = {
                "id": detected.id,
                "position": [
                    detected.pose.position.x,
                    detected.pose.position.y,
                    detected.pose.position.z,
                ],
                "orientation": [
                    detected.pose.orientation.x,
                    detected.pose.orientation.y,
                    detected.pose.orientation.z,
                    detected.pose.orientation.w,
                ],
                "size": [detected.width, detected.depth, detected.height],
                "yaw_valid": bool(getattr(detected, "yaw_valid", False)),
                "aspect_ratio": float(getattr(detected, "aspect_ratio", 0.0)),
                "acquisition_stamp": (
                    detected.header.stamp.sec
                    + 1e-9 * detected.header.stamp.nanosec),
                "acquisition_frame": str(detected.header.frame_id),
                "top_surface_z": float(
                    detected.top_surface_pose.position.z),
                "top_surface_valid": bool(detected.top_surface_valid),
                "height_valid": bool(detected.height_valid),
                "height_source": int(detected.height_source),
            }
        payload = String(data=json.dumps(record, sort_keys=True))
        self._diag_pub.publish(payload)
        self._latest_pub.publish(payload)

    # ------------------------------------------------------------------
    # Service handler
    # ------------------------------------------------------------------

    def handle_detect(self, _req, _response):
        response = DetectLuggage.Response()
        try:
            if not self._wait_suitcase_view():
                self._last_failure_reason = "DETECT_SUITCASE_NOT_UPDATED"
                self._publish_diagnostics(
                    "perception", False, 0.0, self._last_failure_reason)
                response.luggage = []
                response.success = False
                response.message = self._last_failure_reason
                self.get_logger().warning(
                    "luggage_detector: suitcase RGB did not update "
                    "within %.1fs" % self._suitcase_update_timeout)
                return response
            if not self._wait_instance_ready():
                reason = self._last_failure_reason or "DETECT_STALE_INSTANCE"
                self._last_failure_reason = reason
                self._publish_diagnostics(
                    "perception", False, 0.0, self._last_failure_reason)
                response.luggage = []
                response.success = False
                response.message = self._last_failure_reason
                self.get_logger().warning(
                    "luggage_detector: cargo instance not ready (%s)"
                    % self._last_failure_reason)
                return response
            detected, confidence = self._detect_from_window_with_retries()
            if detected is None and not self._use_semantic:
                detected, confidence = self._estimate_with_retries()
        except Exception as exc:  # noqa: BLE001 - service boundary
            self.get_logger().error("detect_luggage handler failed: %s" % exc)
            response.luggage = []
            response.success = False
            response.message = "DETECT_HANDLER_ERROR: %s" % exc
            return response

        if detected is not None:
            self._publish_diagnostics(
                "perception", True, confidence, "ok", detected)
            response.luggage = [detected]
            response.success = True
            response.message = (
                "perception estimate (conf=%.2f, height_valid=%s)"
                % (confidence, detected.height_valid))
            return response

        self.get_logger().warning(
            "luggage_detector: strict perception failed (%s)"
            % self._last_failure_reason)
        self._publish_diagnostics(
            "perception", False, confidence, self._last_failure_reason)
        response.luggage = []
        response.success = False
        response.message = self._last_failure_reason
        return response


def main(argv=None):
    rclpy.init(args=argv)
    node = LuggageDetector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
