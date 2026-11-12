"""Stationary XYZ scan admission; never rename frames or invent missing rays."""
from luggage_perception.motion_stability_filter import MotionStabilityGate


class SettledLidarClouds:
    def __init__(self, *, frame, max_age_s=1.0):
        if not frame or max_age_s <= 0:
            raise ValueError('EXPLICIT_LIDAR_FRAME_AND_AGE_REQUIRED')
        self.frame, self.max_age_s = frame, max_age_s
        self.gate = MotionStabilityGate(joint_names=['elfin_joint%d' % i for i in range(1, 7)])
        self.last_stamp_ns = 0

    def joints(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        self.gate.update(message.name, message.position, message.velocity, stamp=stamp, now=stamp)

    def admit(self, cloud, now_s):
        stamp = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
        if cloud.header.frame_id != self.frame:
            return None, 'LIDAR_FRAME_MISMATCH'
        if not {'x', 'y', 'z'}.issubset({f.name for f in cloud.fields}):
            return None, 'LIDAR_XYZ_REQUIRED'
        if stamp <= self.last_stamp_ns or not 0 <= now_s - stamp / 1e9 <= self.max_age_s:
            return None, 'NO_FRESH_LIDAR'
        if not self.gate.accepts_cloud(stamp / 1e9, now_s):
            return None, 'ROBOT_NOT_SETTLED'
        self.last_stamp_ns = stamp
        return cloud, 'OBSERVED_LIDAR_XYZ'
