"""Geometry-derived retreat/reorientation for a free tool outside the ULD.

This is a search route, not a shortcut around collision certification. Each leg
uses seed-continuous Cartesian IK; the caller certifies the combined timed path.
No new contact mode or cargo movement is introduced.
"""
import numpy as np
from path_planning import cartesian_path


def outside_reorientation(scene,start_q,target,config):
    if scene.payload['mode']!='world':
        return None,dict(reason='FREE_TOOL_ROUTE_REQUIRES_WORLD_PAYLOAD')
    current=scene.container_world@scene.urdf.tcp(start_q,scene.urdf.path_to('suction_contact_frame'))
    aperture=np.array(scene.assets['scene']['container']['opening']['aperture']['corners'])
    radius=max(np.linalg.norm(p) for body in scene.assets['tool'] for p in body.corners())
    outside_x=float(aperture[:,0].min()-radius-config.outside_aperture_margin_m)
    escape=current.copy();escape[0,3]=min(current[0,3],target[0,3],outside_x)
    oriented=escape.copy();oriented[:3,:3]=target[:3,:3]
    aligned=target.copy();aligned[0,3]=escape[0,3]
    path=[list(start_q)];legs=[]
    for name,pose in [('separate_from_opening',escape),('orient_outside',oriented),
                      ('align_outside',aligned),('approach_push_pre',target)]:
        segment,detail=cartesian_path(scene,path[-1],pose,config,False)
        legs.append(dict(leg=name,target_container=pose.tolist(),diagnostic=detail))
        if segment is None:return None,dict(reason='OUTSIDE_ROUTE_REJECTED',legs=legs)
        path.extend(segment[1:])
    return path,dict(reason='OUTSIDE_REORIENTATION_CONNECTED',outside_x_container=outside_x,
        tool_bounding_radius_m=float(radius),legs=legs)
