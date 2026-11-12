"""One declared lower/panel contact rule for fast and full observation checks.

Inputs are normalized container-absolute observations. The geometry provider is
read only: contact_mode, contact_polygon and push_geometry use source tool CAD.
Other world contacts remain subject to the full source-scene collision monitor.
"""
from dataclasses import asdict
import numpy as np
from shapely import intersects_xy
from support_contact import support_contact_failure


def validate_declared_contacts(sample,dimensions,config,policy,allow_support,
                               contact_mode,panel_collision_name,geometry,*,record_intended=True,edge_transition=None):
    intended=[]
    edge_contacts=[];strict_support_count=0
    lower_by_collision={body.name+'::body::collision':body for body in sample.lower}
    for contact in sample.contacts:
        lower=lower_by_collision.get(contact.other_collision)
        if lower is None:continue
        half=np.asarray(lower.dimensions)/2
        diagnostic=support_contact_failure(sample.container_T_box,dimensions,
            lower.container_T_body,[-half,half],contact.point_container,
            contact.normal_toward_payload_container,config.contact_tolerance_m,
            config.contact_penetration_m,min(policy.contact_normal_alignment_deg,
                config.contact_normal_alignment_deg),contact.depth_m,normal_toward_box=True)
        if diagnostic is None:strict_support_count+=1
        elif allow_support and edge_transition and not sample.attached and edge_transition.classify(
                sample.container_T_box,lower.name,lower.container_T_body,[-half,half],
                contact.point_container,contact.normal_toward_payload_container,contact.depth_m,
                normal_toward_box=True):
            edge_contacts.append((lower.name,contact.depth_m));continue
        if not allow_support or diagnostic:
            return dict(reason='NATIVE_CONTACT_OUTSIDE_DECLARED_SUPPORT',
                        body=lower.name,diagnostic=diagnostic),intended
    if edge_contacts:
        if not strict_support_count:
            return dict(reason='PUSH_EDGE_WITHOUT_STRICT_NATIVE_SUPPORT'),intended
        failure=edge_transition.observe(sample.stamp_s,edge_contacts)
        if failure:return failure,intended
    panel_contacts=[contact for contact in sample.contacts if contact.other_collision==panel_collision_name]
    if not panel_contacts:return None,intended
    if not contact_mode:return dict(reason='FORBIDDEN_NATIVE_PANEL_CONTACT'),intended
    mode=geometry.contact_mode(contact_mode)
    frame=np.array(sample.container_T_tool).reshape(4,4)
    if mode=='push_plate':frame=frame@np.array(geometry.push_geometry.tool_T_push_surface).reshape(4,4)
    inverse=np.linalg.inv(frame)
    points=np.asarray([c.point_container for c in panel_contacts])@inverse[:3,:3].T+inverse[:3,3]
    alignment=np.asarray([c.normal_toward_payload_container for c in panel_contacts])@frame[:3,2]
    depth=np.asarray([c.depth_m for c in panel_contacts])
    # Each native mesh manifold may contain many triangle contact points. Build
    # the same source polygon once and test every point, without downsampling.
    patch=geometry.contact_polygon(mode).buffer(config.contact_tolerance_m)
    inside=intersects_xy(patch,points[:,0],points[:,1])
    bad=(~inside | (np.abs(points[:,2])>config.contact_tolerance_m) |
         (alignment<np.cos(np.deg2rad(min(policy.contact_normal_alignment_deg,
                                          config.contact_normal_alignment_deg)))) |
         (depth>config.contact_penetration_m))
    if np.any(bad):
        index=int(np.flatnonzero(bad)[0])
        return dict(reason='NATIVE_CONTACT_OUTSIDE_DECLARED_PANEL_PATCH',
            point_contact_frame=points[index].tolist(),normal_alignment=float(alignment[index]),
            depth_m=float(depth[index])),intended
    if record_intended:intended=[asdict(contact) for contact in panel_contacts]
    return None,intended
