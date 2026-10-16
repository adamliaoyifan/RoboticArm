"""Explicit hypothetical template and safe geometric proposal reuse."""
from luggage_planning.placement_prior_contracts import Transform, finite_tuple


class PrefetchTemplate:
    def __init__(self, box_wdh, grasps_flat):
        self.box_wdh = finite_tuple(box_wdh, 3, 'template dimensions')
        if min(self.box_wdh) <= 0:
            raise ValueError('template dimensions must be positive')
        if not grasps_flat or len(grasps_flat) % 7 or len(grasps_flat) > 32 * 7:
            raise ValueError('template needs 1..32 explicit xyz/xyzw grasps')
        self.grasps = tuple(Transform(grasps_flat[i:i+3], grasps_flat[i+3:i+7])
                            for i in range(0, len(grasps_flat), 7))

    def request(self, confirmed):
        from geometry_msgs.msg import Pose
        from luggage_msgs.srv import PreparePlacementPriorV2
        record = confirmed.record()
        request = PreparePlacementPriorV2.Request()
        request.request_id = 'template-prefetch-' + confirmed.identity
        request.measured = False
        request.template_box_wdh = list(self.box_wdh)
        for grasp in self.grasps:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = grasp.translation
            (pose.orientation.x, pose.orientation.y, pose.orientation.z,
             pose.orientation.w) = grasp.quaternion_xyzw
            request.template_box_from_tcp.append(pose)
        request.geometry_hash = record['geometry_hash']
        request.expected_map_revision = record['map_revision']
        request.max_slots = 8
        request.candidate_limit = 32
        request.budget_s = 10.
        return request


def reusable_placements(prefetched, snapshot, dimensions, max_slots):
    """Reuse geometry/order only; never reuse template collision authorization.

    Keep rejected and untested placements as fallbacks. A changed measured size
    requires fresh geometric generation instead of translating an old slot.
    """
    if prefetched is None:
        return ()
    request, result = prefetched
    if (request.measured or request.snapshot.identity != snapshot.identity
            or request.box_wdh != tuple(dimensions)
            or len(request.placements) < max_slots):
        return ()
    preferred = {choice.placement.slot_id for choice in result.choices}
    return tuple(sorted(request.placements,
                        key=lambda slot: slot.slot_id not in preferred))[:max_slots]


def rank_regenerated_placements(prefetched, snapshot, placements):
    """Apply template preferences to fresh, dimension-checked candidates only.

    Slot IDs identify grid anchors and headings. A different payload size may
    retain an anchor but change its height or invalidate it entirely. Never
    insert a missing template slot or copy its pose into the current request.
    Full measured-grasp collision checks still follow this ordering hint.
    """
    current = tuple(placements)
    if prefetched is None:
        return current
    request, result = prefetched
    if request.measured or request.snapshot.identity != snapshot.identity:
        return current
    preferred = {choice.placement.slot_id for choice in result.choices}
    return tuple(sorted(current, key=lambda slot: slot.slot_id not in preferred))
