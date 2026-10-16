"""Use a prior only to order existing Select candidates; never skip certification.

All caller candidates remain present, all coordinates and actual attachment/start
fields remain untouched. A changed grasp/map/dimension makes this an ordinary
Select. Return an index map so caller-visible slot indices keep their meaning.
"""
import math
from copy import deepcopy


def bound_target_slots(prior, choice, surface, box_wdh, scene_revision, request_id):
    """Resolve a chosen target without changing it after pickup.

    None means no target was bound. A stale bound target is an error, not
    permission to silently select a different placement for that pickup.
    """
    if choice is None:
        return None
    if (prior is None or not prior.success or not prior.measured
            or prior.map_revision != surface.map_revision
            or prior.geometry_hash != surface.geometry_hash
            or prior.surface != surface or tuple(prior.box_wdh) != tuple(box_wdh)
            or choice not in prior.choices
            or choice.slot.identity.scene_revision != scene_revision
            or choice.slot.identity.frame_id != 'container_link'
            or choice.slot.reference != 'box_center'):
        raise ValueError('selected prior target is stale or unbound')
    slot = deepcopy(choice.slot)
    slot.identity.request_id = request_id
    return [slot], [0.]


def _pose_equal(left, right):
    a,b=left.position,right.position
    if not all(math.isfinite(v) for v in (a.x,a.y,a.z,b.x,b.y,b.z)):
        return False
    if max(abs(a.x-b.x),abs(a.y-b.y),abs(a.z-b.z))>1e-9:
        return False
    x,y=left.orientation,right.orientation
    q=(x.x,x.y,x.z,x.w);r=(y.x,y.y,y.z,y.w)
    if not all(math.isfinite(v) for v in q+r):return False
    return min(max(abs(a-b) for a,b in zip(q,r)),
               max(abs(a+b) for a,b in zip(q,r)))<=1e-9


def order_select_candidates(request, prior):
    original=tuple(range(len(request.slots)))
    if prior is None or not prior.success or not prior.measured:
        return original
    if (prior.geometry_hash!=request.geometry_hash or prior.map_revision!=request.map_revision or
            tuple(prior.box_wdh)!=tuple(request.box_wdh) or len(request.slot_scores)!=len(request.slots)):
        return original
    if (prior.surface.identity.scene_revision!=request.surface.identity.scene_revision or
            prior.surface!=request.surface):
        return original
    chosen=[]
    for choice in prior.choices:
        if (choice.grasp.identity.scene_revision!=request.grasp.identity.scene_revision or
                choice.grasp.reference!=request.grasp.reference or
                choice.grasp.orientation_semantics!=request.grasp.orientation_semantics or
                not _pose_equal(choice.grasp.pose,request.grasp.pose)):
            continue
        for index,slot in enumerate(request.slots):
            if index in chosen:continue
            if (slot.identity.scene_revision==choice.slot.identity.scene_revision and
                    slot.identity.frame_id==choice.slot.identity.frame_id and
                    slot.reference==choice.slot.reference and
                    slot.orientation_semantics==choice.slot.orientation_semantics and
                    _pose_equal(slot.pose,choice.slot.pose)):
                chosen.append(index)
    order=tuple(chosen+[index for index in original if index not in chosen])
    if order!=original:
        slots=list(request.slots);scores=list(request.slot_scores)
        request.slots=[slots[index] for index in order]
        request.slot_scores=[scores[index] for index in order]
    return order
