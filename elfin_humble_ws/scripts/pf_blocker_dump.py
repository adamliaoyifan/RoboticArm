#!/usr/bin/env python3
"""Dump per-trial visuals for the PF-R5 blocker investigation.

Subscribes the preprocessed RGB, YOLO detections, and DetectionFrame
stream. For every spawn trial (driven by /luggage/current_box id
changes) it records:
  - raw RGB PNG (exactly as the segmenter receives it);
  - annotated PNG: YOLO boxes (label/conf), cargo point count,
    pca_reason/support_reason/geometry_level of the matching frame;
  - trial JSON sidecar (GT-independent telemetry only).

Written for human inspection: this script never interprets the images.
"""
import json
import os
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from luggage_msgs.msg import DetectionFrame, YoloDetections
from sensor_msgs.msg import Image
from std_msgs.msg import String


class BlkDump(Node):
    def __init__(self, out_root):
        super().__init__("pf_blocker_dump")
        self.out_root = out_root
        os.makedirs(out_root, exist_ok=True)
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self._rgb = None
        self._rgb_stamp = None
        self._yolo = deque(maxlen=10)
        self._frames = deque(maxlen=40)
        self._trial = -1
        self._box_id = ""
        self._saved = 0
        self.create_subscription(
            Image, "/luggage/preprocessed/camera/color/image",
            self._rgb_cb, qos)
        self.create_subscription(
            YoloDetections, "/luggage/semantic/yolo_detections",
            self._yolo_cb, qos)
        self.create_subscription(
            DetectionFrame, "/luggage/perception/detection_frame",
            self._frame_cb, qos)
        self.create_subscription(
            String, "/luggage/current_box", self._box_cb, qos)

    def _rgb_cb(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        w, h = msg.width, msg.height
        if msg.encoding == "rgb8":
            arr = arr.reshape(h, w, 3)
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif msg.encoding == "bgr8":
            arr = arr.reshape(h, w, 3)
        else:
            arr = None
        self._rgb = arr
        self._rgb_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def _yolo_cb(self, msg):
        self._yolo.append((msg.header.stamp.sec, msg.header.stamp.nanosec,
                           [(int(d.label), round(float(d.confidence), 3),
                             [int(v) for v in d.bbox])
                            for d in msg.detections]))

    def _frame_cb(self, msg):
        self._frames.append({
            "stamp": (msg.header.stamp.sec, msg.header.stamp.nanosec),
            "pca_valid": bool(msg.pca_valid),
            "pca_reason": str(msg.pca_reason),
            "pca_source": str(msg.pca_source),
            "n_cargo_points": int(msg.n_cargo_points),
            "support_reason": str(msg.support_reason),
            "geometry_level": int(msg.geometry_level),
            "instance_id": str(msg.instance_id),
        })

    def _box_cb(self, msg):
        box_id = ""
        try:
            data = json.loads(msg.data)
            box_id = str(data.get("id", ""))
        except (TypeError, ValueError):
            return
        if box_id and box_id != self._box_id:
            if self._box_id:
                self._flush()
            self._box_id = box_id
            self._trial += 1
            self._trial_t0 = time.time()

    def _flush(self):
        if self._trial < 0 or not self._frames:
            return
        frames = [f for f in self._frames
                  if f["instance_id"] == self._box_id]
        if not frames:
            frames = list(self._frames)
        reasons = {f["pca_reason"] for f in frames}
        n_cargo = [f["n_cargo_points"] for f in frames]
        verdict = "FAIL" if any(
            f["pca_reason"] == "DETECT_TOP_UNOBSERVABLE" for f in frames
        ) else ("OK" if any(f["pca_valid"] for f in frames) else "NONE")
        trial_dir = os.path.join(
            self.out_root,
            "trial_%02d_%s_%s" % (self._trial, verdict, self._box_id))
        os.makedirs(trial_dir, exist_ok=True)
        # Save the latest RGB (raw) + annotated.
        if self._rgb is not None:
            cv2.imwrite(os.path.join(trial_dir, "rgb_raw.png"), self._rgb)
            ann = self._rgb.copy()
            yolo = list(self._yolo)[-1]
            for label, conf, bbox in yolo[2]:
                x1, y1, x2, y2 = bbox
                color = (0, 255, 0) if conf > 0.1 else (0, 165, 255)
                cv2.rectangle(ann, (x1, y1), (x2, y2), color, 2)
                cv2.putText(ann, "L%d %.3f" % (label, conf),
                            (x1, max(12, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
            summary = "%s %s | cargo %d..%d | %s" % (
                verdict, self._box_id,
                min(n_cargo) if n_cargo else 0,
                max(n_cargo) if n_cargo else 0,
                ",".join(sorted(reasons))[:60])
            cv2.putText(ann, summary, (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(ann, summary, (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv2.imwrite(os.path.join(trial_dir, "rgb_annotated.png"), ann)
        with open(os.path.join(trial_dir, "trial.json"), "w") as fh:
            json.dump({
                "trial": self._trial,
                "box_id": self._box_id,
                "verdict": verdict,
                "pca_reasons": sorted(reasons),
                "n_cargo_points": n_cargo,
                "frames": frames[-12:],
                "yolo_last": list(self._yolo)[-3:],
                "rgb_stamp": self._rgb_stamp,
            }, fh, indent=2)
        self._saved += 1
        print("saved %s (%s, cargo %s)" % (
            trial_dir, verdict,
            "%d..%d" % (min(n_cargo), max(n_cargo)) if n_cargo else "-"))

    def finish(self):
        self._flush()


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--duration", type=float, default=600.0)
    args = parser.parse_args()
    rclpy.init()
    node = BlkDump(args.out)
    t0 = time.time()
    try:
        while rclpy.ok() and time.time() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.finish()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
