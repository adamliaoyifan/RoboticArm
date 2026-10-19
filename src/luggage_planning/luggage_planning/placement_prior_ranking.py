"""Cheap measured-volume ranking before spending bounded native tool checks."""
import math
import numpy as np


def rank_payload_clearance(candidates, box_wdh, volume, margin):
    """Prioritize clear payload AABBs, retaining solver order within each group.

    Input volume has already passed snapshot admission. This does not authorize
    any grasp or reject a requested target; native tool checks still decide.
    Blocked candidates remain as fallback since an AABB can overestimate an
    oriented box. Prefix sums bound each candidate query to eight array reads.
    """
    nx, ny, nz = map(int, volume.dimensions_xyz)
    states = np.asarray(volume.states).reshape(nz, ny, nx)
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
        width, depth, height = box_wdh
        half = np.array([(c*width+s*depth)/2, (s*width+c*depth)/2, height/2]) + margin
        center = np.asarray(candidate['center_container'])
        lower = np.maximum(0, np.minimum(maximum, np.floor((center-half-origin)/resolution).astype(int)))
        upper = np.maximum(0, np.minimum(maximum, np.ceil((center+half-origin)/resolution).astype(int)))
        x0, y0, z0 = lower; x1, y1, z1 = upper
        count = (prefix[z1,y1,x1]-prefix[z0,y1,x1]-prefix[z1,y0,x1]-prefix[z1,y1,x0]
                 +prefix[z0,y0,x1]+prefix[z0,y1,x0]+prefix[z1,y0,x0]-prefix[z0,y0,x0])
        ranked.append((bool(count), index, candidate))
    return [row[2] for row in sorted(ranked, key=lambda row: row[:2])]
