#!/usr/bin/env python3
"""ROS node: RGB semantic segmentation.

Subscribes to the camera color image, runs the configured segmenter backend,
and publishes a per-pixel label map plus a colorized visualization. When ML
dependencies (torch, ultralytics) are missing the node still runs and emits
an all-background mask so downstream consumers stay alive.
"""

from __future__ import division

import os
import sys
import threading

import rospy
import rospkg
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
PERC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_perception"), "scripts")
for path in (DESC_SCRIPTS, PERC_SCRIPTS):
    if path not in sys.path:
        sys.path.insert(0, path)

from semantic_segmenter import (  # noqa: E402
    DEFAULT_LABEL_NAMES,
    build_segmenter,
    colorize_label_map,
    draw_detections_overlay,
)


def _serialize_stats(stats):
    """Coerce a segmenter stats dict into a ROS-param-safe structure.

    ROS parameter server requires all dict keys to be strings and all values
    to be bool/int/float/str/list/dict (no numpy scalars). ``label_counts`` is
    keyed by integer label id, and counts/inference_ms may be numpy scalars —
    both crash ``rospy.set_param`` with "dictionary key must be string".
    """
    if not isinstance(stats, dict):
        return stats
    out = {}
    for key, value in stats.items():
        skey = str(key)
        if isinstance(value, dict):
            # label_counts: use the human label name when known, else str(id).
            sub = {}
            for k, v in value.items():
                name = DEFAULT_LABEL_NAMES.get(k, str(k))
                sub[name] = _coerce_scalar(v)
            out[skey] = sub
        elif isinstance(value, (list, tuple)):
            out[skey] = [_coerce_scalar(v) for v in value]
        else:
            out[skey] = _coerce_scalar(value)
    return out


def _coerce_scalar(value):
    """Best-effort cast of numpy scalars / numbers to plain python types."""
    if hasattr(value, "item"):  # numpy scalar
        try:
            return value.item()
        except Exception:
            pass
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float, str)):
        return value
    try:
        return float(value)
    except (TypeError, ValueError):
        return str(value)



class SemanticSegmenterNode:
    def __init__(self):
        self._config_path = rospy.get_param(
            "~config",
            os.path.join(
                rospkg.RosPack().get_path("luggage_perception"),
                "config", "semantic_segmenter.yaml",
            ),
        )
        config = self._load_config(self._config_path)
        config["backend"] = rospy.get_param("~backend", config.get("backend", "stub"))
        config["device"] = rospy.get_param("~device", config.get("device", "cpu"))
        config["confidence_threshold"] = float(
            rospy.get_param("~confidence_threshold",
                            config.get("confidence_threshold", 0.25))
        )
        if rospy.has_param("~prompts"):
            config["prompts"] = rospy.get_param("~prompts")

        self._bridge = CvBridge()
        self._segmenter = build_segmenter(config)
        self._enabled = bool(rospy.get_param("~enabled", True))
        self._publish_viz = bool(rospy.get_param("~publish_viz", True))
        # RGB overlay with detection bboxes/masks/labels drawn on the camera
        # image - subscribe in RViz (rviz/Image) on /luggage/semantic/overlay
        # to confirm the backend is actually detecting objects. Independent of
        # the colorized label-map viz above.
        self._publish_overlay = bool(rospy.get_param("~publish_overlay", True))
        # INFO-level detection summary is throttled by this period so it stays
        # visible by default without spamming at frame rate. State transitions
        # (0 <-> N detections) log immediately because each branch throttles
        # on its own call site.
        self._det_log_period = float(
            rospy.get_param("~detection_log_period", 2.0)
        )
        self._max_rate_hz = float(rospy.get_param("~max_rate_hz", 0.0))
        self._last_publish_time = rospy.Time(0)
        self._lock = threading.Lock()
        self._latest_image = None

        self._mask_pub = rospy.Publisher(
            "/luggage/semantic/mask", Image, queue_size=1, latch=False
        )
        self._instance_mask_pub = rospy.Publisher(
            "/luggage/semantic/instance_mask", Image, queue_size=1, latch=False
        )
        self._viz_pub = None
        if self._publish_viz:
            self._viz_pub = rospy.Publisher(
                "/luggage/semantic/viz", Image, queue_size=1, latch=False
            )
        self._overlay_pub = None
        if self._publish_overlay:
            self._overlay_pub = rospy.Publisher(
                "/luggage/semantic/overlay", Image, queue_size=1, latch=False
            )

        self._info_pub = rospy.Publisher(
            "/luggage/semantic/camera_info", CameraInfo, queue_size=1, latch=True
        )

        color_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        info_topic = rospy.get_param("~color_info_topic", "/camera/color/camera_info")
        self._info_sub = rospy.Subscriber(
            info_topic, CameraInfo, self._on_info, queue_size=1
        )
        self._image_sub = rospy.Subscriber(
            color_topic, Image, self._on_image, queue_size=1, buff_size=2 ** 24
        )

        rospy.loginfo(
            "semantic_segmenter ready: backend=%s prompts=%d conf=%.2f device=%s",
            self._segmenter.last_stats.get("backend", "?"),
            len(self._segmenter.prompts),
            self._segmenter.confidence_threshold,
            config.get("device", "cpu"),
        )

    @staticmethod
    def _load_config(path):
        try:
            with open(path, "r") as handle:
                data = yaml.safe_load(handle) or {}
        except (IOError, OSError) as exc:
            rospy.logwarn("semantic_segmenter: cannot load config %s: %s", path, exc)
            return {}
        return data.get("semantic", data)

    def _on_info(self, msg):
        # Re-publish color camera_info under the semantic namespace once.
        self._info_pub.publish(msg)

    def _on_image(self, msg):
        if not self._enabled:
            return
        if self._max_rate_hz > 0.0:
            now = rospy.Time.now()
            elapsed = (now - self._last_publish_time).to_sec()
            if elapsed < 1.0 / self._max_rate_hz:
                return
            self._last_publish_time = now

        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "semantic_segmenter: cv_bridge decode failed: %s", exc)
            return

        with self._lock:
            try:
                label_map, detections = self._segmenter.segment(cv_image)
            except Exception as exc:
                rospy.logerr_throttle(5.0, "semantic_segmenter inference failed: %s", exc)
                return
            stats = self._segmenter.last_stats
            inst_map = self._segmenter.instance_map

        mask_msg = self._bridge.cv2_to_imgmsg(label_map, encoding="mono8")
        mask_msg.header = msg.header
        self._mask_pub.publish(mask_msg)

        if inst_map is not None:
            inst_msg = self._bridge.cv2_to_imgmsg(inst_map, encoding="mono16")
            inst_msg.header = msg.header
            self._instance_mask_pub.publish(inst_msg)

        if self._viz_pub is not None:
            viz_bgr = colorize_label_map(label_map)
            viz_msg = self._bridge.cv2_to_imgmsg(viz_bgr, encoding="bgr8")
            viz_msg.header = msg.header
            self._viz_pub.publish(viz_msg)

        if self._overlay_pub is not None:
            try:
                overlay_bgr = draw_detections_overlay(cv_image, detections)
                overlay_msg = self._bridge.cv2_to_imgmsg(
                    overlay_bgr, encoding="bgr8")
                overlay_msg.header = msg.header
                self._overlay_pub.publish(overlay_msg)
            except Exception as exc:
                rospy.logwarn_throttle(
                    5.0, "semantic_segmenter: overlay draw failed: %s", exc)

        rospy.set_param("/luggage/semantic/segmenter_stats", _serialize_stats(stats))
        rospy.set_param("/luggage/semantic/detection_count", len(detections))

        det_param = []
        for d in detections:
            det_param.append({
                "label": int(d["label"]),
                "prompt": str(d["prompt"]),
                "confidence": float(d["confidence"]),
                "bbox": [int(v) for v in d["bbox"]],
                "instance_id": int(d.get("instance_id", 0)),
            })
        rospy.set_param("/luggage/semantic/detections", det_param)

        self._log_detections(detections, stats)
        rospy.logdebug(
            "semantic_segmenter: %d detections, %dms",
            len(detections), stats.get("inference_ms", 0.0),
        )

    def _log_detections(self, detections, stats):
        """Throttled INFO summary of what the backend detected this frame.

        Answers "is the model actually outputting results?" at a glance: the
        per-detection line shows label/prompt/confidence/bbox, and the
        zero-detection branch names the backend + confidence threshold so a
        silent stub or over-high threshold is obvious.
        """
        count = len(detections)
        summary = "%d detection%s in %.1fms" % (
            count, "" if count == 1 else "s",
            float(stats.get("inference_ms", 0.0) or 0.0),
        )
        if count:
            parts = []
            for d in detections:
                name = DEFAULT_LABEL_NAMES.get(int(d["label"]), str(d["label"]))
                x1, y1, x2, y2 = (int(v) for v in d["bbox"])
                parts.append("%s/%s %.2f (%d,%d,%d,%d)" % (
                    name, str(d["prompt"]), float(d["confidence"]),
                    x1, y1, x2, y2))
            rospy.loginfo_throttle(
                self._det_log_period,
                "semantic_segmenter: %s: %s", summary, "; ".join(parts),
            )
        else:
            rospy.loginfo_throttle(
                self._det_log_period,
                "semantic_segmenter: %s (backend=%s conf=%.2f) - no objects detected",
                summary,
                self._segmenter.last_stats.get("backend", "?"),
                self._segmenter.confidence_threshold,
            )


def main():
    rospy.init_node("semantic_segmenter")
    SemanticSegmenterNode()
    rospy.spin()


if __name__ == "__main__":
    main()
