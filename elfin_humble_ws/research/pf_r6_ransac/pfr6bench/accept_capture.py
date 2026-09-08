#!/usr/bin/env python3
"""PF-R6 acceptance failure-case capture (research-only ROS node).

Runs alongside the perception stack and records, per acquisition stamp:
- the detection frame outcome (pca_reason, n_cargo_points, support fields)
- the same-stamp YOLO detections (prompt/label/confidence/bbox)
- the same-stamp semantic mask label distribution
- an RGB snapshot for failing/suspicious frames (pca_reason != ok or
  cargo points below --cargo-threshold)

Everything lands in --out (outside Git for images, jsonl for all rows).
Nothing is published; no production node is touched.
"""

from __future__ import division

import argparse
import json
import os
import threading
import time
from collections import OrderedDict, deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from luggage_msgs.msg import DetectionFrame, YoloDetections

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover
    CvBridge = None

NAMES = {0: "background", 1: "container_wall", 2: "cargo",
         3: "robot_arm", 4: "unknown"}


class AcceptCapture(Node):

    def __init__(self, args):
        super().__init__("pf_r6_accept_capture")
        self.args = args
        os.makedirs(os.path.join(args.out, "images"), exist_ok=True)
        be = QoSProfile(depth=10,
                        reliability=ReliabilityPolicy.BEST_EFFORT)
        self.det_frames = deque(maxlen=60)
        self.yolo = OrderedDict()
        self.masks = OrderedDict()
        self.rgbs = OrderedDict()
        self.lock = threading.Lock()
        self.rows = []
        self.n_saved = 0
        self.bridge = CvBridge() if CvBridge else None
        self.create_subscription(DetectionFrame,
                                 "/luggage/perception/detection_frame",
                                 self._det_cb, be)
        self.create_subscription(YoloDetections,
                                 "/luggage/semantic/yolo_detections",
                                 self._yolo_cb, be)
        self.create_subscription(Image, "/luggage/semantic/mask",
                                 self._mask_cb, be)
        self.create_subscription(
            Image, "/luggage/preprocessed/camera/color/image",
            self._rgb_cb, be)

    def _key(self, header):
        return (header.stamp.sec, header.stamp.nanosec)

    def _yolo_cb(self, msg):
        with self.lock:
            self.yolo[self._key(msg.header)] = {
                "backend": msg.backend,
                "detections": [
                    {"label": int(d.label), "prompt": d.prompt,
                     "conf": round(float(d.confidence), 4),
                     "bbox": [int(v) for v in d.bbox],
                     "held": bool(d.held)}
                    for d in msg.detections],
            }
            while len(self.yolo) > 24:
                self.yolo.popitem(last=False)

    def _mask_cb(self, msg):
        a = np.frombuffer(msg.data, dtype=np.uint8)[
            :msg.width * msg.height]
        vals, cnts = np.unique(a, return_counts=True)
        dist = {NAMES.get(int(v), int(v)): int(c)
                for v, c in zip(vals, cnts)}
        with self.lock:
            self.masks[self._key(msg.header)] = dist
            while len(self.masks) > 24:
                self.masks.popitem(last=False)

    def _rgb_cb(self, msg):
        # Manual decode: cv_bridge on this host is built against numpy1
        # and raises _ARRAY_API errors with the installed numpy 2.x.
        try:
            n = msg.width * msg.height
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            enc = msg.encoding.lower()
            if enc in ("rgb8", "bgr8"):
                img = raw[:n * 3].reshape(msg.height, msg.width, 3)
                if enc == "rgb8":
                    img = img[:, :, ::-1]  # to BGR for imwrite
            elif enc == "mono8":
                img = raw[:n].reshape(msg.height, msg.width)
            else:
                return
            img = img.copy()
        except Exception:
            return
        with self.lock:
            self.rgbs[self._key(msg.header)] = img
            while len(self.rgbs) > 6:
                self.rgbs.popitem(last=False)

    def _det_cb(self, msg):
        key = self._key(msg.header)
        with self.lock:
            yolo = self.yolo.get(key)
            mask = self.masks.get(key)
            rgb = self.rgbs.get(key)
        n_cargo = int(msg.n_cargo_points)
        failed = (msg.pca_reason != "ok") or (
            n_cargo < self.args.cargo_threshold)
        row = {
            "t": time.time(),
            "stamp": "%d.%09d" % key,
            "pca_reason": msg.pca_reason,
            "n_cargo_points": n_cargo,
            "support_valid": bool(msg.support_valid),
            "support_reason": msg.support_reason,
            "geometry_level": int(msg.geometry_level),
            "yolo": yolo,
            "mask_labels": mask,
        }
        self.rows.append(row)
        if failed and yolo is not None:
            self.n_saved += 1
            fid = "fail_%03d_%s" % (self.n_saved, row["stamp"])
            with open(os.path.join(
                    self.args.out, "failed_cases.jsonl"), "a") as fh:
                fh.write(json.dumps(row) + "\n")
            if rgb is not None and self.n_saved <= self.args.max_images:
                out_png = os.path.join(
                    self.args.out, "images", "%s.png" % fid)
                try:
                    import cv2
                    cv2.imwrite(out_png, rgb,
                                [int(cv2.IMWRITE_PNG_COMPRESSION), 6])
                except Exception as exc:
                    self.get_logger().warn("image save failed: %s" % exc)
            self.get_logger().info(
                "FAILCASE %s pca=%s cargo=%d yolo=%s mask=%s"
                % (fid, msg.pca_reason, n_cargo,
                   None if yolo is None else [
                       (d["prompt"], d["conf"]) for d in yolo["detections"]],
                   mask))

    def dump_rows(self):
        with open(os.path.join(self.args.out, "all_frames.jsonl"),
                  "w") as fh:
            for r in self.rows:
                fh.write(json.dumps(r) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=180.0)
    ap.add_argument("--cargo-threshold", type=int, default=10000)
    ap.add_argument("--max-images", type=int, default=60)
    args = ap.parse_args()
    rclpy.init()
    node = AcceptCapture(args)
    exe = rclpy.executors.SingleThreadedExecutor()
    exe.add_node(node)
    spin = threading.Thread(target=exe.spin, daemon=True)
    spin.start()
    t0 = time.time()
    try:
        while time.time() - t0 < args.duration:
            time.sleep(1.0)
    finally:
        node.dump_rows()
        node.get_logger().info(
            "done: %d frames, %d failcases -> %s"
            % (len(node.rows), node.n_saved, args.out))
        exe.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
