#!/usr/bin/env python3
"""Wake an idle original executor through its existing parameter service.

Private Python signals alone do not interrupt rcl_wait. This read-only empty
parameter request wakes it so a pending capture/stop signal can run. Only
used after scoring; no new product callback, timer, thread or endpoint.
"""
import argparse
import os


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--node',required=True)
    args=parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1')
    import rclpy
    from rcl_interfaces.srv import GetParameters
    rclpy.init();node=rclpy.create_node('occ_g2_control_wake')
    try:
        client=node.create_client(GetParameters,'/'+args.node.strip('/')+'/get_parameters')
        if not client.wait_for_service(timeout_sec=2.):return 1
        future=client.call_async(GetParameters.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=2.)
        return int(not future.done() or future.exception() is not None)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
