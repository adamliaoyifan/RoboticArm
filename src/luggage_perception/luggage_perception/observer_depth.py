"""Convert an already-paired preprocessed, stationary RGB-D view to O rays.

No geometry inference, scene lookup or semantic selection occurs here. Invalid
or moving input yields no points. TF at the acquisition stamp belongs to O.
"""
from collections import OrderedDict

from luggage_perception.container_depth_observer import depth_cloud, stamp_ns
from luggage_perception.motion_stability_filter import MotionStabilityGate


class SettledDepthRays:
    def __init__(self, *, stride=4, max_age_s=1.0):
        if type(stride) is not int or stride < 1 or max_age_s <= 0:
            raise ValueError('positive depth stride and age required')
        self.stride, self.max_age_s = stride, max_age_s
        self.images, self.infos = OrderedDict(), OrderedDict()
        self.gate = MotionStabilityGate(joint_names=['elfin_joint%d'%i for i in range(1,7)])
        self.last_stamp = 0

    def store(self, kind, message):
        buffer = self.images if kind == 'image' else self.infos
        buffer[stamp_ns(message)] = message
        while len(buffer) > 8:
            buffer.popitem(last=False)

    def joints(self, message):
        stamp = stamp_ns(message)/1e9
        self.gate.update(message.name, message.position, message.velocity, stamp=stamp, now=stamp)

    def next_cloud(self, now_s):
        for stamp in sorted(self.images.keys() & self.infos.keys(), reverse=True):
            if stamp <= self.last_stamp or not 0 <= now_s-stamp/1e9 <= self.max_age_s:
                continue
            if not self.gate.accepts_cloud(stamp/1e9, now_s):
                return None, 'ROBOT_NOT_SETTLED'
            image, info = self.images[stamp], self.infos[stamp]
            # Fail closed before advancing the consumed stamp on malformed input.
            cloud = depth_cloud(image, info, stride=self.stride)
            self.last_stamp = stamp
            return cloud, 'OBSERVED_OPTICAL_RAYS'
        return None, 'NO_FRESH_PAIRED_DEPTH'
