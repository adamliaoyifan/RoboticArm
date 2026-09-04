#!/usr/bin/env python3
"""EMS (Maximal Empty Space) incremental maintenance + regularity metric.

P2 of the online-packing redesign (design §5.2). Pure Python + numpy, no
ROS/scipy. An EMS is a maximal axis-aligned empty cuboid inside the container
that does not intersect any placed box. After each placement the intersected
EMSs are split along the 6 faces and contained sub-spaces are eliminated.

EMS is the direct input to:
  - candidate generation (EMS corners, §3.1);
  - the "do not block deep EMS" hard constraint (§5.5);
  - the ``ems_regularity`` proxy for V̂ (§5.7 degraded path).

Coordinates are floor-relative (X/Y in [-inner/2, +inner/2], Z in [0, inner_h])
to match ``FreeSpaceModel.H``.
"""

from __future__ import division


class EMS(object):
    """Maximal Empty Space list with incremental split + containment elimination."""

    def __init__(self, inner_size, min_useful_edge=0.25, max_count=64):
        self.inner_l, self.inner_w, self.inner_h = [float(v) for v in inner_size]
        self.min_edge = float(min_useful_edge)
        self.max_count = int(max_count)
        self.spaces = [self._initial_space()]

    def _initial_space(self):
        return (-self.inner_l * 0.5, -self.inner_w * 0.5, 0.0,
                self.inner_l * 0.5, self.inner_w * 0.5, self.inner_h)

    def place(self, box):
        """Split EMSs intersecting ``box`` (x0,y0,z0,x1,y1,z1); eliminate contained."""
        new = []
        for space in self.spaces:
            new.extend(self._split(space, box))
        self.spaces = self._eliminate_contained(new)
        return list(self.spaces)

    def ems_after(self, box):
        """Return a NEW EMS with ``box`` placed (non-mutating). For what-if
        checks like ``blocks_deep_space`` without disturbing the live model."""
        import copy
        clone = copy.copy(self)
        clone.spaces = list(self.spaces)
        clone.place(box)
        return clone

    def _split(self, space, box):
        sx0, sy0, sz0, sx1, sy1, sz1 = space
        bx0, by0, bz0, bx1, by1, bz1 = box
        ix0, iy0, iz0 = max(sx0, bx0), max(sy0, by0), max(sz0, bz0)
        ix1, iy1, iz1 = min(sx1, bx1), min(sy1, by1), min(sz1, bz1)
        if ix1 <= ix0 or iy1 <= iy0 or iz1 <= iz0:
            return [space]  # no intersection -> unchanged
        # Clip the +/-y and +/-z splits to the intersection's x/y range so the
        # 6 sub-spaces exactly tile the original minus the box (§5.2).
        cx0, cx1 = ix0, ix1
        cy0, cy1 = iy0, iy1
        subs = [
            (sx0, sy0, sz0, bx0, sy1, sz1),                     # x-
            (bx1, sy0, sz0, sx1, sy1, sz1),                     # x+
            (cx0, sy0, sz0, cx1, by0, sz1),                     # y-
            (cx0, by1, sz0, cx1, sy1, sz1),                     # y+
            (cx0, cy0, sz0, cx1, cy1, bz0),                     # z-
            (cx0, cy0, bz1, cx1, cy1, sz1),                     # z+
        ]
        return [s for s in subs if self._useful(s)]

    def _useful(self, s):
        return (s[3] - s[0] >= self.min_edge and
                s[4] - s[1] >= self.min_edge and
                s[5] - s[2] >= self.min_edge)

    def _eliminate_contained(self, spaces):
        keep = []
        n = len(spaces)
        for i in range(n):
            s = spaces[i]
            contained = False
            for j in range(n):
                if i == j:
                    continue
                t = spaces[j]
                if (s[0] >= t[0] - 1e-9 and s[1] >= t[1] - 1e-9 and s[2] >= t[2] - 1e-9 and
                        s[3] <= t[3] + 1e-9 and s[4] <= t[4] + 1e-9 and s[5] <= t[5] + 1e-9):
                    contained = True
                    break
            if not contained:
                keep.append(s)
        # Bound count; drop smallest when over max.
        if len(keep) > self.max_count:
            keep.sort(key=volume, reverse=True)
            keep = keep[:self.max_count]
        return keep

    @staticmethod
    def intersects_box(space, box):
        sx0, sy0, sz0, sx1, sy1, sz1 = space
        bx0, by0, bz0, bx1, by1, bz1 = box
        return (sx0 < bx1 and sx1 > bx0 and
                sy0 < by1 and sy1 > by0 and
                sz0 < bz1 and sz1 > bz0)

    def regularity(self):
        """§5.7 ``ems_regularity`` proxy in [0, 1]: large + concentrated + balanced."""
        if not self.spaces:
            return 0.0
        vols = sorted((volume(s) for s in self.spaces), reverse=True)
        v_ref = self.inner_l * self.inner_w * self.inner_h
        max_ratio = vols[0] / v_ref
        top3_share = sum(vols[:3]) / max(1e-9, sum(vols))
        largest = max(self.spaces, key=volume)
        lwh = [largest[3] - largest[0], largest[4] - largest[1], largest[5] - largest[2]]
        balance = 1.0 - (max(lwh) - min(lwh)) / max(max(lwh), 1e-9)
        return 0.5 * max_ratio + 0.3 * top3_share + 0.2 * balance


def volume(space):
    """Volume of an EMS 6-tuple (clamped non-negative)."""
    return (max(0.0, space[3] - space[0]) *
            max(0.0, space[4] - space[1]) *
            max(0.0, space[5] - space[2]))
