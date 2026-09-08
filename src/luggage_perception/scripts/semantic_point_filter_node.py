#!/usr/bin/env python3
"""Semantic point filter node (ROS 2 Humble port).

Routes the preprocessed depth cloud through the semantic label mask:
pixels labelled cargo go to ``/luggage/semantic/cargo_points`` (the
detector's input), obstacle labels to ``/luggage/semantic/obstacle_points``.

This is deliberately much thinner than the ROS 1 node: frame conversion,
inf filtering and slop-based pairing all live in the preprocessor now. The
only alignment logic here is an **exact** ``(sec, nanosec)`` stamp join
between the cloud and the mask - both inherit the preprocessor's RGB
primary stamp, so the keys match by construction. Approximate-time pairing
is not allowed downstream of the preprocessor
(docs/architecture/sensor_data_pipeline.md).

``fallback_to_raw`` from the ROS 1 node is deliberately gone: an
unfiltered cloud must never be published as cargo geometry (acceptance
criterion). No mask -> no cargo cloud, reason recorded in stats.

Intrinsics come from the live ``camera_info`` (preferred; in simulation the
preprocessor feeds both slots from the single gz camera_info, which is
self-consistent with ``extrinsics_source: identity``). A realsense yaml is
only an offline fallback for ``extrinsics_source: config`` on real
hardware.
"""

from __future__ import division

import json
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from luggage_perception import ros_message_adapters as adapters
from luggage_perception.cargo_instance_tracker import (
    SOURCE_EMPTY,
    SOURCE_MEASURE,
    CargoInstanceTracker,
    parse_current_box_payload,
    rotation_from_xyzw,
    transform_points,
    xyz_array,
)
from luggage_perception.luggage_box_estimator import voxel_downsample
from luggage_perception.semantic_point_filter import (
    CameraIntrinsics,
    DepthToColorExtrinsics,
    JoinStampTracker,
    SemanticPointFilter,
)


def _stamp_key(msg):
    return (msg.header.stamp.sec, msg.header.stamp.nanosec)


def _stamp_to_tf_time(stamp):
    """ROS stamp message -> rclpy Time for stamped TF lookups (PF-R3)."""
    return rclpy.time.Time(
        seconds=int(stamp.sec), nanoseconds=int(stamp.nanosec))


class SemanticPointFilterNode(Node):

    def __init__(self):
        super().__init__("semantic_point_filter")
        defaults = {
            "input.cloud": "/luggage/preprocessed/camera/depth/points",
            "input.mask": "/luggage/semantic/mask",
            "input.instance_mask": "/luggage/semantic/instance_mask",
            "input.camera_info": "/luggage/preprocessed/camera/color/camera_info",
            "output.cargo_points": "/luggage/semantic/cargo_points",
            "output.obstacle_points": "/luggage/semantic/obstacle_points",
            "output.stats": "~/stats_json",
            "output.cargo_voxel_size": 0.0,
            "cargo_labels": [2],
            "obstacle_labels": [2, 4],
            # "identity" (gz: color and depth are one sensor) | "config"
            # (real D435: depth_to_color from the realsense yaml).
            "extrinsics_source": "identity",
            "realsense_extrinsics_config": "",
            "buffer_maxlen": 10,
            # PF-R9 B5: stats serialization moved off the per-callback path
            # onto a timer (0 keeps the legacy per-call behaviour for unit
            # tests).
            "stats_publish_hz": 1.0,
            "world_frame": "world",
            "associate_radius_m": 0.15,
            "current_box_topic": "/luggage/current_box",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._cargo_labels = [
            int(v) for v in self.get_parameter("cargo_labels").value]
        self._obstacle_labels = [
            int(v) for v in self.get_parameter("obstacle_labels").value]
        self._cargo_voxel_size = max(
            0.0, float(self.get_parameter("output.cargo_voxel_size").value))
        self._buffer_maxlen = max(2, int(self.get_parameter("buffer_maxlen").value))
        self._world_frame = str(self.get_parameter("world_frame").value)
        stats_hz = float(self.get_parameter("stats_publish_hz").value)
        self._stats_publish_interval_sec = (
            0.0 if stats_hz <= 0.0 else 1.0 / stats_hz)
        self._stats_dirty = False

        extrinsics = self._load_extrinsics()
        self._intrinsics = None          # from live camera_info
        self._extrinsics = extrinsics
        self._filter = None              # built once intrinsics arrive
        self._join_stamps = JoinStampTracker()
        self._tracker = CargoInstanceTracker(
            associate_radius_m=float(
                self.get_parameter("associate_radius_m").value))
        self._lock = threading.RLock()
        # Sensor callbacks are exclusive so 307k-point joins cannot pile up
        # (Reentrant + MultiThreadedExecutor previously spawned ~70 threads
        # and froze /stats_json). current_box is a *different* exclusive
        # group so epoch reset can run while a join is in numpy.
        self._group = MutuallyExclusiveCallbackGroup()
        self._box_group = MutuallyExclusiveCallbackGroup()
        self._tf_buffer = Buffer()
        # Humble TransformListener(spin_thread=True) does add_node(node) on a
        # dedicated executor. That cannot be *this* node (main() also adds it
        # to MultiThreadedExecutor → "Node already added to an executor").
        # A sidecar node is the Humble-equivalent of the plan's spin_thread.
        self._tf_node = Node("semantic_point_filter_tf")
        self._tf_listener = TransformListener(
            self._tf_buffer, self._tf_node, spin_thread=True)
        self._last_cloud_stamp = None
        self._last_cloud_frame = "camera_depth_optical_frame"

        # Exact-stamp join buffers (bounded, oldest evicted first).
        self._clouds = {}
        self._masks = {}
        self._instances = {}
        self._counts = {"cloud": 0, "mask": 0, "instance": 0,
                        "joined": 0, "processed": 0, "no_intrinsics": 0,
                        "mask_decode_fail": 0, "cloud_decode_fail": 0,
                        "epoch_reset": 0, "tf_miss": 0,
                        "cloud_waiting_for_mask": 0,
                        "mask_waiting_for_cloud": 0,
                        "cloud_buffer_evicted": 0,
                        "mask_buffer_evicted": 0,
                        "instance_buffer_evicted": 0,
                        "stale_cloud_dropped": 0,
                        "stale_mask_dropped": 0,
                        "stale_instance_dropped": 0,
                        "obstacle_publish_count": 0,
                        "obstacle_publish_skipped": 0}
        self._last_stage_ms = {}

        sensor_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        # camera_info from the preprocessor is also best-effort sensor data.
        stats_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._cargo_pub = self.create_publisher(
            PointCloud2, self.get_parameter("output.cargo_points").value,
            sensor_qos)
        self._obstacle_pub = self.create_publisher(
            PointCloud2, self.get_parameter("output.obstacle_points").value,
            sensor_qos)
        self._stats_pub = self.create_publisher(
            String, self.get_parameter("output.stats").value, stats_qos)

        self.create_subscription(
            PointCloud2, self.get_parameter("input.cloud").value,
            self._on_cloud, sensor_qos, callback_group=self._group)
        self.create_subscription(
            Image, self.get_parameter("input.mask").value, self._on_mask,
            sensor_qos, callback_group=self._group)
        self.create_subscription(
            Image, self.get_parameter("input.instance_mask").value,
            self._on_instance, sensor_qos, callback_group=self._group)
        self.create_subscription(
            CameraInfo, self.get_parameter("input.camera_info").value,
            self._on_camera_info, sensor_qos, callback_group=self._group)
        self.create_subscription(
            String, self.get_parameter("current_box_topic").value,
            self._on_current_box, stats_qos, callback_group=self._box_group)

        if self._stats_publish_interval_sec > 0.0:
            # PF-R9 B5: serialize stats off the sensor-callback path.
            self.create_timer(
                self._stats_publish_interval_sec, self._on_stats_timer)

        self.get_logger().info(
            "semantic_point_filter ready (extrinsics=%s, cargo_labels=%s)"
            % (self.get_parameter("extrinsics_source").value, self._cargo_labels))

    def _on_stats_timer(self):
        if not self._stats_dirty:
            return
        self._stats_dirty = False
        with self._lock:
            self._publish_stats_now()

    # ------------------------------------------------------------------
    # Config
    # ------------------------------------------------------------------

    def _load_extrinsics(self):
        source = str(self.get_parameter("extrinsics_source").value)
        if source == "identity":
            return DepthToColorExtrinsics.identity()
        if source == "config":
            path = str(self.get_parameter("realsense_extrinsics_config").value)
            if not path:
                raise RuntimeError(
                    "extrinsics_source=config requires realsense_extrinsics_config")
            import yaml
            with open(path, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle)
            section = data.get("camera", {}).get("extrinsics", {})
            return DepthToColorExtrinsics.from_dict(
                section.get("depth_to_color", {}))
        raise RuntimeError("unknown extrinsics_source %r" % source)

    # ------------------------------------------------------------------
    # Subscriptions + exact-stamp join
    # ------------------------------------------------------------------

    def _on_camera_info(self, msg):
        frame = adapters.camera_info_frame_from_msg(msg)
        intr = CameraIntrinsics(
            frame.fx, frame.fy, frame.cx, frame.cy, frame.width, frame.height,
            distortion_coeffs=list(frame.distortion_coeffs),
            distortion_model=frame.distortion_model,
        )
        with self._lock:
            if self._intrinsics is None or (
                    intr.fx, intr.fy, intr.cx, intr.cy) != (
                    self._intrinsics.fx, self._intrinsics.fy,
                    self._intrinsics.cx, self._intrinsics.cy):
                self._filter = SemanticPointFilter(
                    intr, intr, self._extrinsics,
                    self._cargo_labels, self._obstacle_labels)
            self._intrinsics = intr

    def _on_cloud(self, msg):
        key = _stamp_key(msg)
        with self._lock:
            self._counts["cloud"] += 1
            self._last_cloud_stamp = msg.header.stamp
            self._last_cloud_frame = msg.header.frame_id or self._last_cloud_frame
            self._join_stamps.note_cloud(adapters.stamp_to_sec(msg.header.stamp))
            self._store(self._clouds, key, msg, "cloud_buffer_evicted")
            joined = self._take_newest_join()
            if joined is None:
                if key not in self._masks:
                    self._counts["cloud_waiting_for_mask"] += 1
                    self._join_stamps.note_cloud_waiting_for_mask()
                self._publish_stats()
                return
        self._process_joined(*joined)

    def _on_mask(self, msg):
        key = _stamp_key(msg)
        with self._lock:
            self._counts["mask"] += 1
            self._join_stamps.note_mask(adapters.stamp_to_sec(msg.header.stamp))
            self._store(self._masks, key, msg, "mask_buffer_evicted")
            joined = self._take_newest_join()
            if joined is None:
                if key not in self._clouds:
                    self._counts["mask_waiting_for_cloud"] += 1
                    self._join_stamps.note_mask_waiting_for_cloud()
                self._publish_stats()
                return
        self._process_joined(*joined)

    def _on_instance(self, msg):
        with self._lock:
            self._counts["instance"] += 1
            self._store(
                self._instances, _stamp_key(msg), msg,
                "instance_buffer_evicted")

    def _on_current_box(self, msg):
        box_id, generation = parse_current_box_payload(msg.data)
        with self._lock:
            if not self._tracker.set_epoch(generation, box_id):
                return
            self._counts["epoch_reset"] += 1
            self._join_stamps.note_epoch(generation, box_id)
            stamp = self._last_cloud_stamp
            if stamp is None:
                stamp = self.get_clock().now().to_msg()
            self._publish_cargo(
                [], stamp, self._last_cloud_frame, n_points=0)
            # Epoch resets are rare; publish immediately, not on the timer.
            self._publish_stats_now()

    def _store(self, buffer_, key, msg, evict_counter):
        buffer_[key] = msg
        while len(buffer_) > self._buffer_maxlen:
            oldest = min(buffer_)
            del buffer_[oldest]
            self._counts[evict_counter] += 1
            if oldest == key:
                break

    def _lookup_rt(self, target, source, stamp):
        """TF at the acquisition stamp (PF-R3). Latest-TF fallback is
        forbidden: a missing historical transform is an explicit miss, so
        the frame is dropped rather than transformed with a pose the
        robot no longer holds. The retry is bounded by a *wall-clock*
        deadline: a tf2 sim-time timeout never expires when the
        simulation clock stalls, which wedged the join callback."""
        import time as _time
        if not target or not source:
            return None
        deadline = _time.monotonic() + 0.05
        while True:
            try:
                tf_msg = self._tf_buffer.lookup_transform(
                    target, source,
                    _stamp_to_tf_time(stamp),
                    rclpy.duration.Duration(seconds=0))
                break
            except TransformException:
                tf_msg = None
            if _time.monotonic() >= deadline:
                return None
            _time.sleep(0.01)
        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation
        rot = rotation_from_xyzw(r.x, r.y, r.z, r.w)
        trans = (float(t.x), float(t.y), float(t.z))
        return rot, trans

    def _to_world(self, points_camera, camera_frame, stamp):
        rt = self._lookup_rt(self._world_frame, camera_frame, stamp)
        if rt is None:
            return None
        rot, trans = rt
        return transform_points(points_camera, rot, trans)

    def _to_camera(self, points_world, camera_frame, stamp):
        rt = self._lookup_rt(camera_frame, self._world_frame, stamp)
        if rt is None:
            return None
        rot, trans = rt
        return transform_points(points_world, rot, trans)

    def _publish_cargo(self, points_xyz, stamp, frame_id, n_points=None):
        xyz = xyz_array(points_xyz)
        if self._cargo_voxel_size > 0.0 and xyz.shape[0]:
            xyz = voxel_downsample(xyz, self._cargo_voxel_size)
        n = int(xyz.shape[0] if n_points is None else n_points)
        self._cargo_pub.publish(adapters.cloud_msg_from_points(
            xyz, stamp, frame_id or self._last_cloud_frame))
        with self._lock:
            self._join_stamps.note_join(
                adapters.stamp_to_sec(stamp), n)

    def _take_join(self, key):
        """Pop a stamp-matched pair. Caller holds ``_lock``. None if incomplete."""
        if key not in self._clouds or key not in self._masks:
            return None
        self._counts["joined"] += 1
        return (
            self._clouds.pop(key),
            self._masks.pop(key),
            self._instances.pop(key, None),
            self._filter,
        )

    def _take_newest_join(self):
        """Process only the newest exact-stamp pair; drop older buffered frames.

        Exclusive-group callbacks queue FIFO. Processing every 307k-point
        join in that queue stalled cargo for ~25 s (CARGO_NOT_READY with
        frozen ``cloud`` counts).
        """
        keys = set(self._clouds) & set(self._masks)
        if not keys:
            return None
        key = max(keys)
        joined = self._take_join(key)
        stale_cloud = len(self._clouds)
        stale_mask = len(self._masks)
        stale_instance = len(self._instances)
        self._counts["stale_cloud_dropped"] += stale_cloud
        self._counts["stale_mask_dropped"] += stale_mask
        self._counts["stale_instance_dropped"] += stale_instance
        self._join_stamps.note_stale_drop(
            stale_cloud + stale_mask + stale_instance)
        self._clouds.clear()
        self._masks.clear()
        self._instances.clear()
        return joined

    def _process_joined(self, cloud_msg, mask_msg, instance_msg, filt):
        """Decode + numpy off the lock so current_box can reset the epoch."""
        start = time.monotonic()
        label_map = adapters.image_array_from_msg(mask_msg)
        after_mask = time.monotonic()
        if label_map is None:
            with self._lock:
                self._last_stage_ms = {
                    "mask_decode_ms": (after_mask - start) * 1000.0,
                    "process_total_ms": (after_mask - start) * 1000.0,
                }
                self._counts["mask_decode_fail"] += 1
                self._warn_throttled(
                    "dropping mask with encoding %s" % mask_msg.encoding)
                self._publish_stats()
            return
        points = adapters.cloud_points_from_msg(cloud_msg)
        after_cloud = time.monotonic()
        if points is None:
            with self._lock:
                self._last_stage_ms = {
                    "mask_decode_ms": (after_mask - start) * 1000.0,
                    "cloud_decode_ms": (after_cloud - after_mask) * 1000.0,
                    "process_total_ms": (after_cloud - start) * 1000.0,
                }
                self._counts["cloud_decode_fail"] += 1
                self._warn_throttled("dropping cloud with unsupported layout")
                self._publish_stats()
            return
        if filt is None:
            with self._lock:
                self._counts["no_intrinsics"] += 1
                self._warn_throttled("no camera_info yet; dropping joined pair")
                self._publish_stats()
            return

        # Instance ids are not consumed downstream; the 5-tuple Python loop
        # over ~100k cargo points stalled the join callback for seconds.
        del instance_msg
        cargo, obstacle = filt.filter_points(points, label_map, None)
        after_filter = time.monotonic()

        stamp = cloud_msg.header.stamp
        frame_id = cloud_msg.header.frame_id or self._last_cloud_frame
        camera_pts = xyz_array(cargo)
        world_pts = None
        tf_miss = False
        before_tf = time.monotonic()
        if camera_pts.shape[0]:
            world_pts = self._to_world(camera_pts, frame_id, stamp)
            tf_miss = world_pts is None
        after_tf = time.monotonic()

        with self._lock:
            self._counts["processed"] += 1
            self._last_cloud_stamp = stamp
            self._last_cloud_frame = frame_id
            if camera_pts.shape[0] and tf_miss:
                self._counts["tf_miss"] += 1
                source = self._tracker.note_tf_miss(
                    adapters.stamp_to_sec(stamp))
            elif camera_pts.shape[0]:
                source = self._tracker.observe(
                    adapters.stamp_to_sec(stamp), world_pts)
            else:
                source = self._tracker.observe(
                    adapters.stamp_to_sec(stamp), camera_pts)
            frozen_empty = (
                self._tracker.generation > 0
                and not self._tracker.instance_id)
            tracked = self._tracker.points_world
            if tracked is not None:
                tracked = tracked.copy()
            after_track = time.monotonic()

        before_publish = time.monotonic()
        if source == SOURCE_MEASURE:
            self._publish_cargo(camera_pts, stamp, frame_id)
        elif frozen_empty or (source == SOURCE_EMPTY and camera_pts.shape[0] == 0):
            self._publish_cargo([], stamp, frame_id, n_points=0)
        elif source == SOURCE_EMPTY and camera_pts.shape[0]:
            self._publish_cargo(camera_pts, stamp, frame_id)
        else:
            cam = (
                self._to_camera(tracked, frame_id, stamp)
                if tracked is not None else None)
            if cam is not None:
                self._publish_cargo(cam, stamp, frame_id)
            elif tracked is not None and len(tracked):
                self._publish_cargo(tracked, stamp, self._world_frame)
            else:
                self._publish_cargo([], stamp, frame_id, n_points=0)

        if obstacle is not None and len(obstacle) and self._has_obstacle_subscribers():
            self._obstacle_pub.publish(adapters.cloud_msg_from_points(
                obstacle, stamp, frame_id))
            with self._lock:
                self._counts["obstacle_publish_count"] += 1
        elif obstacle is not None and len(obstacle):
            with self._lock:
                self._counts["obstacle_publish_skipped"] += 1
        after_publish = time.monotonic()
        with self._lock:
            self._last_stage_ms = {
                "mask_decode_ms": (after_mask - start) * 1000.0,
                "cloud_decode_ms": (after_cloud - after_mask) * 1000.0,
                "filter_ms": (after_filter - after_cloud) * 1000.0,
                "to_world_ms": (after_tf - before_tf) * 1000.0,
                "tracker_ms": (after_track - after_tf) * 1000.0,
                "publish_ms": (after_publish - before_publish) * 1000.0,
                "process_total_ms": (after_publish - start) * 1000.0,
                "input_points": int(len(points)),
                "cargo_points": int(camera_pts.shape[0]),
                "obstacle_points": int(len(obstacle) if obstacle is not None else 0),
            }
            self._publish_stats()

    def _publish_stats(self):
        """Mark stats dirty; the timer serializes (PF-R9 B5 throttle).

        _publish_stats used to run json.dumps(sort_keys=True) on a nested
        dict on EVERY cloud/mask callback (~60-70 Hz) inside the same
        exclusive callback group as the join; now the timer publishes at
        stats_publish_hz (0 keeps the legacy per-call behaviour, used by
        unit tests that assert per-event records).
        """
        if self._stats_publish_interval_sec <= 0.0:
            self._publish_stats_now()
            return
        self._stats_dirty = True

    def _publish_stats_now(self):
        record = dict(self._filter.last_stats) if self._filter is not None else {}
        record.update(self._counts)
        record.update(self._join_stamps.as_dict())
        record.update(self._tracker.as_dict())
        record["buffer_maxlen"] = int(self._buffer_maxlen)
        record["cargo_voxel_size"] = float(self._cargo_voxel_size)
        record["buffer_occupancy"] = {
            "cloud": len(self._clouds),
            "mask": len(self._masks),
            "instance": len(self._instances),
            "exact_join_candidates": len(set(self._clouds) & set(self._masks)),
        }
        record["stage_ms"] = dict(self._last_stage_ms)
        now_sec = adapters.stamp_to_sec(self.get_clock().now().to_msg())
        if self._last_cloud_stamp is not None:
            last = adapters.stamp_to_sec(self._last_cloud_stamp)
            record["executor_lag_sec"] = float(now_sec) - float(last)
        else:
            record["executor_lag_sec"] = None
        self._stats_pub.publish(String(data=json.dumps(record, sort_keys=True)))

    def _has_obstacle_subscribers(self):
        try:
            return self._obstacle_pub.get_subscription_count() > 0
        except AttributeError:
            return True

    def _warn_throttled(self, text):
        now = self.get_clock().now().nanoseconds
        last = getattr(self, "_warn_ns", 0)
        if now - last > 5e9:
            self.get_logger().warning(text)
            self._warn_ns = now


    def shutdown_tf(self):
        listener = getattr(self, "_tf_listener", None)
        executor = getattr(listener, "executor", None) if listener else None
        if executor is not None:
            executor.shutdown()
            thread = getattr(listener, "dedicated_listener_thread", None)
            if thread is not None:
                thread.join(timeout=2.0)
        tf_node = getattr(self, "_tf_node", None)
        if tf_node is not None:
            tf_node.destroy_node()
            self._tf_node = None


def main(argv=None):
    rclpy.init(args=argv)
    node = SemanticPointFilterNode()
    # Two exclusive groups (sensors + current_box) plus spare; do not default
    # to cpu_count, which previously exploded the join callback pileup.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        node.shutdown_tf()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
