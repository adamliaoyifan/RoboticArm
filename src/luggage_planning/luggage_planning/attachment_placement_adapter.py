"""Convert capacity-solver envelope centres into nominal box targets.

This adapter transforms an Attachment output; it never creates margins or
changes the input response. The solver receives conservative dimensions.
"""
from copy import deepcopy


def nominal_placement_response(response, geometry):
    if response is None or not response.success:
        return response
    def convert(source):
        slot = deepcopy(source)
        p, q = slot.pose.position, slot.pose.orientation
        position, quaternion = geometry.box_pose_at_conservative(
            (p.x,p.y,p.z), (q.x,q.y,q.z,q.w))
        p.x, p.y, p.z = position
        q.x, q.y, q.z, q.w = quaternion
        return slot
    result = deepcopy(response)
    result.slot = convert(response.slot)
    result.candidates = [convert(slot) for slot in response.candidates]
    return result
