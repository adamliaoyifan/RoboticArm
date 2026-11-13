"""Private source collision scene for routing around an uncertain free box.

Only fixed world-box routing is supported. Intended contact uncertainty is
still unproved; this class cannot admit a whole physical action. Its tightened
distance rows feed the existing source joint-motion interval validator, rather
than merely checking RRT waypoints or inflating a visual-only object.
"""
import numpy as np
from full_arm import FullArmScene
from payload_tube import PayloadEnvelopeSampler


class FreeBoxEnvelopeScene(FullArmScene):
    def __init__(self, assets, snapshot, config, numeric_error_m, translation_m, angle_rad):
        super().__init__(assets, snapshot, config)
        self.envelope = PayloadEnvelopeSampler(self, numeric_error_m, translation_m, angle_rad)

    def set_payload(self, size, mode, world_pose=None, box_T_tool=None, allow_support=False):
        if mode == 'world' and world_pose is None:
            raise ValueError('FIXED_WORLD_BOX_POSE_REQUIRED')
        super().set_payload(size, mode, world_pose, box_T_tool, allow_support)
        self._fixed_world_payload = ((np.array(size).copy(), np.array(world_pose).copy())
                                     if mode == 'world' else None)

    def check_state(self, q, margin, contact_tool=False):
        if self.payload is None or self.payload['mode'] != 'world':
            raise ValueError('FIXED_WORLD_BOX_REQUIRED_FOR_ENVELOPE_ROUTING')
        size, pose = self._fixed_world_payload
        if (not np.array_equal(size, self.payload['size']) or
                not np.array_equal(pose, self.payload_transform())):
            raise ValueError('MOVING_WORLD_BOX_INVALIDATES_ENVELOPE_ROUTING_BOUND')
        failure = super().check_state(q, margin, contact_tool)
        if failure:
            return failure
        nominal = [(float(distance) - self.envelope.numeric_error_m, tuple(pair),
                    'payload' if 'payload' in pair else 'robot')
                   for distance, pair in self.sample_distances]
        try:
            enclosed = self.envelope.enclose_current_distances(nominal, contact_tool)
        except ValueError as error:
            return dict(reason='ENVELOPE_PAIR_INVENTORY_INVALID', diagnostic=str(error), q=list(q))
        if not enclosed:
            return dict(reason='ENVELOPE_MISSING_DISTANCE_EVIDENCE')
        for lower, pair in enclosed:
            if lower < margin:
                return dict(reason='PAYLOAD_ENVELOPE_MARGIN_UNPROVED', pair=pair,
                            lower_m=lower, required_m=margin, q=list(q))
        # Relative source motion bounds remain valid: the uncertain box set is
        # fixed in world and only the robot side moves over each joint segment.
        self.sample_distances = [(d, p) for d, p in self.sample_distances if 'payload' not in p]
        self.sample_distances.extend(enclosed)
        self.sample_clearance_m = min(d for d, _ in self.sample_distances)
        return None
