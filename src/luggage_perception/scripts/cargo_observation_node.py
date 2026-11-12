#!/usr/bin/env python3
"""Explicit read-only acquisition service; no launch enables it by default."""
import functools

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from luggage_msgs.srv import GetCargoObservationV2

from luggage_perception.cargo_observation_service import (
    CargoObservationBuffer, CargoObservationService,
)


class CargoObservationNode(Node):
    def __init__(self, **kwargs):
        super().__init__('cargo_observation', **kwargs)
        # Resource/age limits, not geometric association or identity thresholds.
        defaults = dict(sensor_id='wrist_rgbd', sampling_stride=1,
            max_frames=8,max_buffer_bytes=64*1024*1024,max_pixels=2_000_000,max_age_ms=1000,
            rgb_topic='/luggage/preprocessed/camera/color/image',
            depth_topic='/luggage/preprocessed/camera/depth/image',
            camera_info_topic='/luggage/preprocessed/camera/depth/camera_info',
            instance_mask_topic='/luggage/semantic/instance_mask')
        for name,value in defaults.items():
            self.declare_parameter(name,value)
        values = {name:self.get_parameter(name).value for name in defaults}
        self.buffer = CargoObservationBuffer(max_frames=values['max_frames'],
            max_bytes=values['max_buffer_bytes'],max_pixels=values['max_pixels'],
            max_age_ns=int(values['max_age_ms'])*1_000_000)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.port = CargoObservationService(buffer=self.buffer,sensor_id=values['sensor_id'],
            stride=values['sampling_stride'],now_ns=lambda:self.get_clock().now().nanoseconds,
            lookup_tf=self._lookup_tf)
        image_qos = QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_ = [self.create_subscription(message_type,values[topic],
            functools.partial(self._store,kind),image_qos) for kind,message_type,topic in (
                ('rgb',Image,'rgb_topic'),('depth',Image,'depth_topic'),
                ('mask',Image,'instance_mask_topic'),('calibration',CameraInfo,'camera_info_topic'))]
        self.service_ = self.create_service(GetCargoObservationV2,
            '~/get_observation',self.port.handle)

    def _lookup_tf(self, optical_frame, stamp_ns):
        # Exact acquisition lookup, with no latest-TF fallback or robot motion.
        return self.tf_buffer.lookup_transform('container_link',optical_frame,
                                              Time(nanoseconds=stamp_ns))

    def _store(self, kind, message):
        try:
            self.buffer.add(kind,message)
        except ValueError as error:
            self.get_logger().warning('Observation source refused: '+str(error),throttle_duration_sec=1.)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CargoObservationNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
