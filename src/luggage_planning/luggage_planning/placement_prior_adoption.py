"""Use a prior only to order existing Select candidates; never skip certification.

All caller candidates remain present, all coordinates and actual attachment/start
fields remain untouched. A changed grasp/map/dimension makes this an ordinary
Select. Return an index map so caller-visible slot indices keep their meaning.
"""
import math


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
