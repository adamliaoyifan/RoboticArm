#!/usr/bin/env python3
"""Hardware pick: observe -> DetectLuggage -> BuildMotionSequence -> PlanMotion.

Vacuum attach after the attach segment (box DO0/DO1, wait DI0). Default
holds suction after retreat so you can see the box; Ctrl-C or --release
drops it.

Person on e-stop. Does not start Gazebo. Does not spawn a simulated box.

Typical order after hardware_pick.launch.py is up:

  ros2 run luggage_planning hardware_pick_driver.py --detect-only
  ros2 run luggage_planning hardware_pick_driver.py --plan-only
  ros2 run luggage_planning hardware_pick_driver.py
  ros2 run luggage_planning hardware_pick_driver.py --skip-observe
      # detect+pick from the current joints; do not drive pickup_observe
"""

from __future__ import division

import argparse
import signal
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from luggage_msgs.action import GoToRobotPose, PlanMotion
from luggage_msgs.msg import VacuumState
from luggage_msgs.srv import BuildMotionSequence, DetectLuggage, VacuumCommand


class HardwarePickDriver(Node):

    def __init__(self, args):
        super().__init__("hardware_pick_driver")
        self._args = args
        self._group = ReentrantCallbackGroup()
        self._vacuum_state = {}
        self._scene_box = None
        self.create_subscription(
            VacuumState, "/vacuum/state", self._on_vacuum, 10,
            callback_group=self._group)
        self._detect = self.create_client(
            DetectLuggage, "/luggage_detector/detect_luggage",
            callback_group=self._group)
        self._build = self.create_client(
            BuildMotionSequence, "/waypoint_generator/build_motion_sequence",
            callback_group=self._group)
        self._vacuum = self.create_client(
            VacuumCommand, "/vacuum/command", callback_group=self._group)
        self._goto = ActionClient(
            self, GoToRobotPose, "/motion_planner/go_to_robot_pose",
            callback_group=self._group)
        self._plan = ActionClient(
            self, PlanMotion, "/motion_planner/plan_motion",
            callback_group=self._group)
        self._scene = None
        self._release_on_exit = False

    def _scene_client(self):
        if self._scene is None:
            from luggage_planning.planning_scene_client import PlanningSceneClient
            self._scene = PlanningSceneClient(self, callback_group=self._group)
        return self._scene

    def _on_vacuum(self, msg):
        self._vacuum_state = {
            "attached": bool(msg.attached),
            "vacuum_on": bool(msg.vacuum_on),
            "fail_reason": msg.fail_reason,
        }

    def _call_srv(self, client, request, timeout):
        if not client.wait_for_service(timeout_sec=min(5.0, timeout)):
            return None
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout):
            return None
        return future.result()

    def _send_action(self, client, goal, timeout, name):
        if not client.wait_for_server(timeout_sec=5.0):
            return False, "%s server missing" % name, None
        event = threading.Event()
        send = client.send_goal_async(goal)
        send.add_done_callback(lambda _f: event.set())
        if not event.wait(10.0):
            return False, "%s goal send timeout" % name, None
        handle = send.result()
        if handle is None or not handle.accepted:
            return False, "%s rejected" % name, None
        event.clear()
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout):
            return False, "%s result timeout" % name, None
        wrapped = result_fut.result()
        result = wrapped.result
        status = wrapped.status
        ok = status == GoalStatus.STATUS_SUCCEEDED and bool(result.success)
        return ok, str(result.message), result

    def vacuum(self, enable, timeout=20.0):
        req = VacuumCommand.Request()
        req.enable = bool(enable)
        response = self._call_srv(self._vacuum, req, timeout)
        if response is None:
            return False, "vacuum timeout"
        return bool(response.success), str(response.message)

    def goto_observe(self):
        goal = GoToRobotPose.Goal()
        goal.pose_name = self._args.observe_pose
        return self._send_action(
            self._goto, goal, self._args.goto_timeout, "GoToRobotPose")

    def detect_once(self):
        return self._call_srv(
            self._detect, DetectLuggage.Request(), self._args.detect_timeout)

    def detect(self):
        last = None
        attempts = max(1, int(self._args.detect_retries))
        for attempt in range(attempts):
            last = self.detect_once()
            if last is not None and last.success and last.luggage:
                return last
            msg = last.message if last else "timeout"
            self.get_logger().warn(
                "detect attempt %d/%d: %s" % (attempt + 1, attempts, msg))
            if attempt + 1 < attempts:
                time.sleep(2.0)
        return last

    def wait_graph(self):
        timeout = float(self._args.ready_timeout)
        deadline = time.monotonic() + timeout
        needed = [
            (self._detect, "DetectLuggage"),
            (self._build, "BuildMotionSequence"),
        ]
        if (self._args.use_vacuum
                and not self._args.detect_only
                and not self._args.plan_only):
            needed.append((self._vacuum, "VacuumCommand"))
        for client, name in needed:
            remain = max(0.5, deadline - time.monotonic())
            if not client.wait_for_service(timeout_sec=remain):
                return False, "%s missing after %.0fs" % (name, timeout)
        remain = max(0.5, deadline - time.monotonic())
        if not self._args.skip_observe:
            if not self._goto.wait_for_server(timeout_sec=remain):
                return False, "GoToRobotPose missing after %.0fs" % timeout
        if self._args.detect_only or self._args.plan_only:
            return True, "observe/detect graph ready"
        remain = max(0.5, deadline - time.monotonic())
        if not self._plan.wait_for_server(timeout_sec=remain):
            return False, "PlanMotion missing after %.0fs" % timeout
        return True, "pick graph ready"

    @staticmethod
    def _box_geom(box):
        xyz = [box.pose.position.x, box.pose.position.y, box.pose.position.z]
        quat = [box.pose.orientation.x, box.pose.orientation.y,
                box.pose.orientation.z, box.pose.orientation.w]
        size = [box.width, box.depth, box.height]
        return xyz, quat, size

    def add_scene_box(self, box):
        scene = self._scene_client()
        if not scene.wait_ready(timeout_sec=5.0):
            return False, "apply_planning_scene unavailable"
        xyz, quat, size = self._box_geom(box)
        ok, message = scene.add_pickup_box(xyz, quat, size)
        if ok:
            self._scene_box = (xyz, quat, size)
        return ok, message

    def set_pickup_touch(self, allowed):
        from luggage_planning.planning_scene_client import (
            BOX_OBJECT_ID,
            DEFAULT_TOUCH_LINKS,
        )
        pairs = [(BOX_OBJECT_ID, link) for link in DEFAULT_TOUCH_LINKS]
        return self._scene_client().set_acm_pairs(
            pairs, bool(allowed), verify=False)

    def attach_scene_box(self):
        if self._scene_box is None:
            return False, "no scene box"
        xyz, quat, size = self._scene_box
        return self._scene_client().attach_pickup_box(
            "pickup_box", xyz, quat, size)

    def remove_scene_box(self):
        if self._scene_box is None:
            return True, "no scene box"
        ok, message = self._scene_client().detach_and_remove()
        self._scene_box = None
        return ok, message

    def run(self):
        self.get_logger().warn(
            "hardware pick: e-stop person required; vacuum uses box DO0/DO1")
        ready, ready_msg = self.wait_graph()
        if not ready:
            self.get_logger().error("graph not ready: %s" % ready_msg)
            return 1
        self.get_logger().info(ready_msg)

        if self._args.skip_observe:
            self.get_logger().warn(
                "skip-observe: detect/pick from current joints, no %s"
                % self._args.observe_pose)
        else:
            ok, message, _ = self.goto_observe()
            if not ok:
                self.get_logger().error("observe failed: %s" % message)
                return 2
            self.get_logger().info(
                "at %s: %s" % (self._args.observe_pose, message))
        time.sleep(self._args.settle_sec)

        detect = self.detect()
        if detect is None or not detect.success or not detect.luggage:
            msg = detect.message if detect else "timeout"
            self.get_logger().error("detect failed: %s" % msg)
            return 3
        box = detect.luggage[0]
        pos = box.pose.position
        top = box.top_surface_pose.position
        self.get_logger().info(
            "detect id=%s xyz=(%.3f,%.3f,%.3f) size=%.3fx%.3fx%.3f "
            "top_z=%.3f valid=%s/%s src=%s"
            % (box.id, pos.x, pos.y, pos.z, box.width, box.depth, box.height,
               top.z, box.top_surface_valid, box.height_valid, box.height_source))
        if self._args.detect_only:
            return 0
        if not box.top_surface_valid:
            self.get_logger().error("top_surface_valid=false; refusing motion")
            return 3

        req = BuildMotionSequence.Request()
        req.phase = "pick"
        req.pick = box
        built = self._call_srv(self._build, req, 15.0)
        if built is None or not built.success:
            self.get_logger().error(
                "build failed: %s" % (built.message if built else "timeout"))
            return 4
        self.get_logger().info(
            "built %d segments: %s"
            % (len(built.segments), [s.name for s in built.segments]))
        for segment in built.segments:
            p = segment.target_pose.position
            self.get_logger().info(
                "  %s type=%s xyz=(%.3f,%.3f,%.3f)"
                % (segment.name, segment.type, p.x, p.y, p.z))
        if self._args.plan_only:
            return 0

        if self._args.use_vacuum and not self._args.no_scene:
            scene_ok, scene_msg = self.add_scene_box(box)
            self.get_logger().info("scene add pickup_box: %s (%s)" % (
                scene_ok, scene_msg))
            if not scene_ok:
                self.get_logger().error("refusing motion without pickup_box")
                return 4

        for segment in built.segments:
            if (self._scene_box is not None
                    and segment.name in ("approach", "attach", "pick_retreat")):
                touch_ok, touch_msg = self.set_pickup_touch(True)
                self.get_logger().info(
                    "acm pickup touch: %s (%s)" % (touch_ok, touch_msg))
            goal = PlanMotion.Goal()
            goal.segment = segment
            ok, message, _res = self._send_action(
                self._plan, goal, self._args.plan_timeout,
                "PlanMotion:%s" % segment.name)
            if not ok:
                self.get_logger().error(
                    "segment %s failed: %s" % (segment.name, message))
                if self._vacuum_state.get("vacuum_on") or self._vacuum_state.get("attached"):
                    self.vacuum(False)
                return 5
            self.get_logger().info("segment %s ok: %s" % (segment.name, message))
            if segment.name == "attach" and self._args.use_vacuum:
                vac_ok, vac_msg = self.vacuum(True, timeout=20.0)
                self.get_logger().info("vacuum attach: %s (%s)" % (vac_ok, vac_msg))
                if not vac_ok:
                    return 6
                self._release_on_exit = True
                if self._scene_box is not None:
                    att_ok, att_msg = self.attach_scene_box()
                    self.get_logger().info(
                        "scene attach pickup_box: %s (%s)" % (att_ok, att_msg))

        state = dict(self._vacuum_state)
        self.get_logger().info("after pick vacuum=%s" % state)
        if self._args.use_vacuum and self._args.release:
            vac_ok, vac_msg = self.vacuum(False)
            self.get_logger().info("vacuum release: %s (%s)" % (vac_ok, vac_msg))
            self._release_on_exit = False
            self.remove_scene_box()
        elif self._args.use_vacuum:
            self.get_logger().warn(
                "holding suction (DI0). rerun with --release or Ctrl-C to drop")
        return 0

    def shutdown_vacuum(self):
        if self._release_on_exit:
            self.get_logger().warn("releasing vacuum on exit")
            self.vacuum(False)
            self.remove_scene_box()
            self._release_on_exit = False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--observe-pose", default="pickup_observe")
    parser.add_argument(
        "--skip-observe", action="store_true",
        help="Do not GoToRobotPose(pickup_observe); detect and pick in place.")
    parser.add_argument("--detect-only", action="store_true")
    parser.add_argument(
        "--plan-only", action="store_true",
        help="Detect and print pick waypoints; do not execute PlanMotion.")
    parser.add_argument("--no-vacuum", dest="use_vacuum", action="store_false")
    parser.add_argument(
        "--no-scene", action="store_true",
        help="Do not add the detected box to MoveIt (debug).")
    parser.add_argument(
        "--release", action="store_true",
        help="Drop the box after pick_retreat (default holds suction).")
    parser.add_argument("--settle-sec", type=float, default=2.0)
    parser.add_argument("--goto-timeout", type=float, default=90.0)
    parser.add_argument("--detect-timeout", type=float, default=20.0)
    parser.add_argument("--detect-retries", type=int, default=5)
    parser.add_argument("--plan-timeout", type=float, default=90.0)
    parser.add_argument("--ready-timeout", type=float, default=90.0)
    parser.set_defaults(use_vacuum=True)
    args = parser.parse_args(args=argv)

    rclpy.init()
    node = HardwarePickDriver(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    def _stop(*_unused):
        node.shutdown_vacuum()
        executor.shutdown()

    signal.signal(signal.SIGINT, _stop)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        code = node.run()
    finally:
        node.shutdown_vacuum()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
