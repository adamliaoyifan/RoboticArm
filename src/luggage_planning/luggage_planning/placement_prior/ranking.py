"""Cheap measured-volume ranking before spending bounded native tool checks."""
import math
import numpy as np


def rank_payload_clearance(candidates, box_wdh, volume, margin, *, hull=None,
                           geometry=None, release_gap_m=0., vertical_margin=None,
                           approach_clearance_m=0.):
    """Prioritize clear payload AABBs, retaining solver order within each group.

    Input volume has already passed snapshot admission. This does not authorize
    any grasp or reject a requested target; native tool checks still decide.
    Blocked candidates remain as fallback since an AABB can overestimate an
    oriented box. Include the existing vertical approach sweep when supplied;
    a clear release pose alone does not imply a clear descent. Prefix sums bound
    each candidate query to eight array reads. No obstacle or candidate is removed.
    """
    nx, ny, nz = map(int, volume.dimensions_xyz)
    states = (np.frombuffer(volume.states,dtype=np.uint8) if isinstance(volume.states,bytes)
              else np.asarray(volume.states)).reshape(nz, ny, nx)
    active = np.asarray(volume.active, dtype=bool).reshape(nz, ny, nx)
    prefix = np.pad((active & (states != 1)).astype(np.int32), ((1, 0),) * 3)
    for axis in range(3):
        prefix = prefix.cumsum(axis=axis)
    origin = np.asarray(volume.origin_container)
    resolution = float(volume.resolution_m)
    maximum = np.array([nx, ny, nz])
    ranked = []
    for index, candidate in enumerate(candidates):
        yaw = float(candidate['heading_theta'])
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        center = np.asarray(candidate['center_container'])
        dimensions = box_wdh
        if geometry is not None:
            quat=(0.,0.,math.sin(yaw/2),math.cos(yaw/2))
            center,_=geometry.conservative_pose_at_box(center,quat)
            center=np.asarray(center)
            dimensions=geometry.conservative.dimensions_wdh
        center=center+np.array([0.,0.,release_gap_m+approach_clearance_m/2])
        width, depth, height = dimensions
        mz=margin if vertical_margin is None else vertical_margin
        half = np.array([(c*width+s*depth)/2+margin,
                         (s*width+c*depth)/2+margin, height/2+mz+approach_clearance_m/2])
        outside_hull=False
        if hull is not None:
            from luggage_description.container_geometry import contains_oriented_box
            outside_hull=any(not contains_oriented_box(hull,
                center+np.array([0.,0.,offset]),dimensions,yaw,margin=margin,floor_margin=0.)
                for offset in (-approach_clearance_m/2,approach_clearance_m/2))
        lower = np.maximum(0, np.minimum(maximum, np.floor((center-half-origin)/resolution).astype(int)))
        upper = np.maximum(0, np.minimum(maximum, np.ceil((center+half-origin)/resolution).astype(int)))
        x0, y0, z0 = lower; x1, y1, z1 = upper
        count = (prefix[z1,y1,x1]-prefix[z0,y1,x1]-prefix[z1,y0,x1]-prefix[z1,y1,x0]
                 +prefix[z0,y0,x1]+prefix[z0,y1,x0]+prefix[z1,y0,x0]-prefix[z0,y0,x0])
        ranked.append((outside_hull, bool(count), index, candidate))
    return [row[3] for row in sorted(ranked, key=lambda row: row[:3])]
