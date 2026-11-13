"""Rigid bottom/top contact contract, independent of contact-force feedback.

A point near the payload bottom is insufficient: a vertical obstacle can touch
that same corner. Both material faces and the contact normal must agree.
Transforms, points and normals passed together must use the same frame.
"""
import numpy as np


def support_contact_failure(box_pose, box_size, support_pose, support_bounds,
                            point, normal, gap_m, penetration_m, alignment_deg,
                            depth_m, *, normal_toward_box):
    box=np.asarray(box_pose).reshape(4,4)
    support=np.asarray(support_pose).reshape(4,4)
    bounds=np.asarray(support_bounds).reshape(2,3)
    point=np.asarray(point);normal=np.asarray(normal)
    bottom_sign=-1 if box[2,2]>=0 else 1
    box_up=-bottom_sign*box[:3,2]
    support_up=support[:3,2]
    box_local=np.linalg.inv(box)@np.r_[point,1.]
    support_local=np.linalg.inv(support)@np.r_[point,1.]
    box_alignment=float(normal@box_up)
    support_alignment=float(normal@support_up)
    if not normal_toward_box:
        # FCL mesh contact orientation depends on object order. Native contacts
        # have a known direction and must retain the signed check instead.
        box_alignment=abs(box_alignment)
        support_alignment=abs(support_alignment)
    minimum=float(np.cos(np.deg2rad(alignment_deg)))
    detail=dict(point_box=box_local[:3].tolist(),point_support=support_local[:3].tolist(),
                normal_box_alignment=box_alignment,normal_support_alignment=support_alignment,
                depth_m=float(depth_m))
    if support_up[2]<=0 or min(box_alignment,support_alignment)<minimum:
        return dict(reason='SUPPORT_CONTACT_NORMAL_NOT_TOP_BOTTOM',**detail)
    if abs(box_local[2]-bottom_sign*np.asarray(box_size)[2]/2)>gap_m:
        return dict(reason='SUPPORT_CONTACT_NOT_ON_BOX_BOTTOM',**detail)
    if (abs(support_local[2]-bounds[1,2])>gap_m or
        np.any(support_local[:2]<bounds[0,:2]-gap_m) or
        np.any(support_local[:2]>bounds[1,:2]+gap_m)):
        return dict(reason='SUPPORT_CONTACT_NOT_ON_SUPPORT_TOP',**detail)
    if abs(depth_m)>penetration_m:
        return dict(reason='SUPPORT_CONTACT_OVERPENETRATION',**detail)
    return None
