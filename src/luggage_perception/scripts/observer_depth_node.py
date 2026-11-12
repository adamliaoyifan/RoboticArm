#!/usr/bin/env python3
"""Thin ROS adapter for already preprocessed optical depth to the sole O owner."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, JointState, PointCloud2
from std_msgs.msg import String

from luggage_perception.observer_depth import SettledDepthRays


class ObserverDepthNode(Node):
    def __init__(self):
        super().__init__('observer_depth')
        self.rays = SettledDepthRays(stride=self.declare_parameter('pixel_stride',4).value)
        self.publisher = self.create_publisher(PointCloud2,'/luggage/observation/optical_points',10)
        self.status = self.create_publisher(String,'/luggage/observation/status',10)
        self.create_subscription(Image,'/luggage/preprocessed/camera/depth/image',
            lambda msg:self.rays.store('image',msg),qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/luggage/preprocessed/camera/depth/camera_info',
            lambda msg:self.rays.store('info',msg),qos_profile_sensor_data)
        self.create_subscription(JointState,'/joint_states',self.rays.joints,qos_profile_sensor_data)
        self.create_timer(.2,self.update)

    def update(self):
        try:
            cloud, reason = self.rays.next_cloud(self.get_clock().now().nanoseconds/1e9)
            if cloud is not None:
                self.publisher.publish(cloud)
        except (ValueError, TypeError) as error:
            reason = 'DEPTH_REFUSED:'+str(error)
        self.status.publish(String(data=reason))


def main():
    rclpy.init()
    node = ObserverDepthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
