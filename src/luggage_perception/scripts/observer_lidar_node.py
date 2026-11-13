#!/usr/bin/env python3
"""Forward stationary physical-frame LiDAR scans to the sole O writer."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String
from luggage_perception.observer_lidar import SettledLidarClouds


class ObserverLidarNode(Node):
    def __init__(self):
        super().__init__('observer_lidar')
        frame = self.declare_parameter('lidar_frame', 'livox_frame').value
        topic = self.declare_parameter('lidar_topic', '/livox/lidar').value
        self.clouds = SettledLidarClouds(frame=frame)
        calibration = str(self.declare_parameter('structure_calibration_file','').value)
        geometry_hash = str(self.declare_parameter('structure_geometry_hash','').value)
        mount_id = str(self.declare_parameter('structure_sensor_mount_id','').value)
        self.structure = None
        if calibration:
            from luggage_perception.lidar_structure import StructureAssociation
            self.structure = StructureAssociation(json.loads(Path(calibration).read_text()),
                geometry_hash=geometry_hash,sensor_frame=frame,sensor_mount_id=mount_id)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)

        self.publisher = self.create_publisher(PointCloud2, '/luggage/observation/lidar_points', 10)
        self.status = self.create_publisher(String, '/luggage/observation/lidar_status', 10)
        self.create_subscription(JointState, '/joint_states', self.clouds.joints, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, topic, self.update, qos_profile_sensor_data)

    def update(self, message):
        cloud, reason = self.clouds.admit(message, self.get_clock().now().nanoseconds / 1e9)
        if cloud is not None:
            if self.structure is not None:
                try:
                    cloud, count = self._structure_terminals(cloud)
                    reason += ':STRUCTURE_TERMINALS='+str(count)+':'+self.structure.calibration_id
                except Exception as error:
                    # A missing acquisition transform cannot authorize filtering.
                    reason += ':STRUCTURE_UNQUALIFIED_RETAIN_HITS:'+type(error).__name__
            self.publisher.publish(cloud)
        self.status.publish(String(data=reason))

    def _structure_terminals(self, cloud):
        import copy
        if cloud.is_bigendian or any(f.name=='structure_terminal' for f in cloud.fields):
            raise ValueError('INVALID_STRUCTURE_CLOUD_LAYOUT')
        tf = self.tf_buffer.lookup_transform('container_link',cloud.header.frame_id,
                                            rclpy.time.Time.from_msg(cloud.header.stamp))
        t,q = tf.transform.translation,tf.transform.rotation
        rotation = Rotation.from_quat((q.x,q.y,q.z,q.w)).as_matrix()
        raw = point_cloud2.read_points(cloud,field_names=('x','y','z'),skip_nans=False)
        points = np.column_stack([raw[name] for name in ('x','y','z')])
        origin = np.array((t.x,t.y,t.z))
        labels = self.structure.classify(points@rotation.T+origin,origin)
        output = copy.deepcopy(cloud)
        offset = cloud.point_step
        output.fields.append(PointField(name='structure_terminal',offset=offset,
                                       datatype=PointField.UINT8,count=1))
        output.point_step += 1
        output.row_step = output.width*output.point_step
        packed = bytearray()
        for i,label in enumerate(labels):
            start = (i//cloud.width)*cloud.row_step+(i%cloud.width)*cloud.point_step
            packed.extend(cloud.data[start:start+cloud.point_step])
            packed.append(int(label))
        output.data = bytes(packed)
        return output,int(labels.sum())


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
