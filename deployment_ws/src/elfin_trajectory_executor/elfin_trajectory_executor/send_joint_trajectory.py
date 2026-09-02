"""CLI: send one FJT goal and wait until the executor says it is done.

Safe default on a real arm: a small delta from the *current* joints.
Do not use the mock home/goal poses from the Gazebo MVP client.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .execution_contract import DEFAULT_ACTION_NAME
from .fjt_client import FollowJointTrajectoryClient, JOINT_NAMES


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Send a FollowJointTrajectory goal to the hardware executor and "
            "block until the action result. Prints READY_FOR_NEXT on success."
        )
    )
    parser.add_argument(
        "--action-name",
        default=DEFAULT_ACTION_NAME,
        help="FJT action (must match the executor).",
    )
    parser.add_argument(
        "--delta-deg",
        type=float,
        default=2.0,
        help="Rotate this many degrees on --joint from the current pose.",
    )
    parser.add_argument(
        "--joint",
        type=int,
        default=1,
        choices=range(1, 7),
        help="1-based joint index for --delta-deg (default: 1).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="time_from_start of the goal waypoint, seconds.",
    )
    parser.add_argument(
        "--and-back",
        action="store_true",
        help="After the delta move succeeds, return to the start pose.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the target joints and exit without sending a goal.",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)
    rclpy.init(args=[])
    node = rclpy.create_node("send_joint_trajectory")
    client = FollowJointTrajectoryClient(node, action_name=args.action_name)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=False)
    spinner.start()

    exit_code = 1
    try:
        client.wait_ready()
        start = client.current_positions()
        target = list(start)
        target[args.joint - 1] = start[args.joint - 1] + math.radians(args.delta_deg)
        node.get_logger().info(
            "current=%s  target=%s  action=%s"
            % (_fmt(start), _fmt(target), args.action_name)
        )
        if args.dry_run:
            node.get_logger().info("dry-run: not sending a goal")
            exit_code = 0
            return exit_code

        first = client.execute_positions(target, args.duration)
        node.get_logger().info(first.message)
        if not first.ready_for_next:
            return 1

        if args.and_back:
            back = client.execute_positions(start, args.duration)
            node.get_logger().info(back.message)
            if not back.ready_for_next:
                return 1

        node.get_logger().info("READY_FOR_NEXT")
        exit_code = 0
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        executor.shutdown()
        spinner.join(timeout=5.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.try_shutdown()
        except Exception:
            pass



def _fmt(positions):
    return ["%s=%.4f" % (n, v) for n, v in zip(JOINT_NAMES, positions)]


if __name__ == "__main__":
    sys.exit(main())
