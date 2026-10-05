#!/usr/bin/env python3
"""One-shot SelectPlacePath service smoke against a live stack.

Subscribes the latched /luggage/cargo_map/surface_2d, waits for the
service, issues one request with two synthetic map-frame slots and prints
a verdict JSON. Exercised the production service path for the first time
(2026-09-29): before the service-chain fix every winner crashed the
callback (fixed-slots _joint_knots) and cartesian responses carried a
default zero target_pose.
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from geometry_msgs.msg import Pose
from luggage_msgs.msg import DetectedLuggage, SlotSpec
from luggage_msgs.srv import SelectPlacePath


class Probe(Node):
    def __init__(self):
        super().__init__("select_place_path_probe")
        self._surface = None
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, "/luggage/cargo_map/surface_2d", self._on_surface, qos)
        self._cli = self.create_client(SelectPlacePath,
                                       "/trajectory_selector/select_place_path")

    def _on_surface(self, msg):
        try:
            surface = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(surface, dict) and "height" in surface:
            self._surface = surface

    def run(self, timeout=20.0):
        deadline = time.time() + timeout
        while time.time() < deadline and (
                self._surface is None or not self._cli.wait_for_service(
                    timeout_sec=1.0)):
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._surface is None or not self._cli.service_is_ready():
            return {"ok": False, "error": "surface or service unavailable"}
        req = SelectPlacePath.Request()
        req.request_id = "probe-%d" % int(time.time())
        req.box.width = 0.55
        req.box.depth = 0.40
        req.box.height = 0.25
        for xy in ((0.2, -0.6), (0.4, -0.3)):
            slot = SlotSpec()
            slot.width, slot.depth, slot.height = 0.55, 0.40, 0.25
            slot.place_pose = Pose()
            slot.place_pose.position.x = float(xy[0])
            slot.place_pose.position.y = float(xy[1])
            slot.place_pose.position.z = 0.53 + 0.125
            slot.place_pose.orientation.w = 1.0
            req.candidates.append(slot)
        req.candidate_scores = [0.8, 0.6]
        req.geometry_hash = str(self._surface.get("geometry_hash") or "")
        req.map_revision = int(self._surface.get("map_revision") or 0)
        req.start_pose.position.x = -0.745
        req.start_pose.position.y = -0.27
        req.start_pose.position.z = 1.65
        future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        resp = future.result()
        if resp is None:
            return {"ok": False, "error": "service call timeout"}
        out = {
            "ok": bool(resp.success),
            "reason_code": str(resp.reason_code),
            "path_source": str(resp.path_source),
            "slot_index": int(resp.slot_index),
            "segments": [],
        }
        for seg in resp.carry_segments:
            out["segments"].append({
                "name": str(seg.name),
                "type": str(seg.type),
                "n_waypoints": len(seg.waypoints),
                "target_z": round(float(seg.target_pose.position.z), 4),
                "target_x": round(float(seg.target_pose.position.x), 4),
                "n_joint_waypoints": len(seg.joint_waypoints),
            })
        # The old bug shipped empty waypoints + zero target_pose on
        # cartesian responses; a healthy response must be non-zero.
        for seg in out["segments"]:
            if seg["type"] == "cartesian" and seg["target_z"] == 0.0:
                out["ok"] = False
                out["error"] = "zero target_pose leaked through"
        return out


def main():
    rclpy.init()
    node = Probe()
    try:
        print(json.dumps(node.run(), indent=1, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
