#!/usr/bin/env python3
"""Size-uncertainty margins and committed-box overlap checks (no ROS).

Perception underestimates a partially visible top face: E17 seeds 2 and 3 came
in 0.10 and 0.05 m under the true width/depth. Underestimating is the dangerous
direction. The map and the free-space model would believe the box is smaller
than it is, plan the next box into the overlap, and physically lift it.

The margin is therefore applied asymmetrically by purpose:

  * footprint (width/depth) grows symmetrically -- either side may be the
    under-measured one, and a larger reserved footprint only costs空间;
  * height grows upward only, with the center raised by half the margin, so the
    box bottom stays on its support while the recorded top rises;
  * the grasp is NOT inflated. Enlarging the box there would push the suction
    cup away from the measured center, which is the one number the estimator is
    good at.
"""

from __future__ import division


def inflate_size(size, xy_margin, z_margin):
    """Conservative box extents for collision / map / free-space bookkeeping."""
    width, depth, height = [float(v) for v in size]
    return [
        width + 2.0 * float(xy_margin),
        depth + 2.0 * float(xy_margin),
        height + float(z_margin),
    ]


def inflated_center_z(center_z, z_margin):
    """Center Z after growing the box upward only (bottom stays put)."""
    return float(center_z) + 0.5 * float(z_margin)


def box_aabb(center, size):
    """(x0, y0, z0, x1, y1, z1) for an axis-aligned box about ``center``."""
    cx, cy, cz = [float(v) for v in center]
    half = [float(v) * 0.5 for v in size]
    return (cx - half[0], cy - half[1], cz - half[2],
            cx + half[0], cy + half[1], cz + half[2])


def aabb_overlap_volume(a, b):
    """Intersection volume of two AABBs; 0.0 when they only touch."""
    dx = min(a[3], b[3]) - max(a[0], b[0])
    dy = min(a[4], b[4]) - max(a[1], b[1])
    dz = min(a[5], b[5]) - max(a[2], b[2])
    if dx <= 0.0 or dy <= 0.0 or dz <= 0.0:
        return 0.0
    return dx * dy * dz


def find_overlaps(boxes, tolerance=1e-6):
    """Pairs of ``boxes`` that intersect by more than ``tolerance`` volume.

    ``boxes`` is a list of ``(label, aabb)``. Returns
    ``[(label_a, label_b, volume), ...]``.
    """
    overlaps = []
    for i in range(len(boxes)):
        label_a, aabb_a = boxes[i]
        for j in range(i + 1, len(boxes)):
            label_b, aabb_b = boxes[j]
            volume = aabb_overlap_volume(aabb_a, aabb_b)
            if volume > tolerance:
                overlaps.append((label_a, label_b, volume))
    return overlaps
