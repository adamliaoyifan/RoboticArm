#!/usr/bin/env python3
"""Capture-only diagnostic wiring; never starts simulation or a motion client."""
import argparse
import rclpy
from rclpy.executors import MultiThreadedExecutor
from luggage_msgs.srv import ValidateRouteV2
from luggage_planning.diagnostic_action_capture import CaptureNode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', required=True)
    parser.add_argument('--run-id', required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = CaptureNode(args.directory, args.run_id)
    node.bridge_service(ValidateRouteV2, '/validate_route_v2')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        health = node.close_capture()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if not health['recorder_io_complete']:
            raise SystemExit('CAPTURE_INCOMPLETE: ' + str(health))


if __name__ == '__main__':
    main()
