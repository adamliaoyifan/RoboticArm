#!/usr/bin/env python3
"""Forward stationary physical-frame LiDAR scans to the sole O writer."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import String
from luggage_perception.observer_lidar import SettledLidarClouds


class ObserverLidarNode(Node):
    def __init__(self):
        super().__init__('observer_lidar')
        frame = self.declare_parameter('lidar_frame', 'livox_frame').value
        topic = self.declare_parameter('lidar_topic', '/livox/lidar').value
        self.clouds = SettledLidarClouds(frame=frame)
        self.publisher = self.create_publisher(PointCloud2, '/luggage/observation/lidar_points', 10)
        self.status = self.create_publisher(String, '/luggage/observation/lidar_status', 10)
        self.create_subscription(JointState, '/joint_states', self.clouds.joints, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, topic, self.update, qos_profile_sensor_data)

    def update(self, message):
        cloud, reason = self.clouds.admit(message, self.get_clock().now().nanoseconds / 1e9)
        if cloud is not None:
            self.publisher.publish(cloud)
        self.status.publish(String(data=reason))


def main():
    rclpy.init()
    node = ObserverLidarNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
