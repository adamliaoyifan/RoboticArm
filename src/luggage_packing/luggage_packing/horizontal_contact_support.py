"""Horizontal support on native upper faces, without inferring physical flatness.

Heights and requested bottom are floor-relative. XY uses the admitted producer
origin. Numeric equality is arithmetic handling, not a sensor-error allowance.
"""
from collections import OrderedDict
import math

import numpy as np

from luggage_packing.surface_grid import surface_origin_xy_container

HEIGHT_NUMERIC_EPSILON_M = 1e-9


class HorizontalContactSupport:
    """Shared support geometry for proposal scoring and continuous target checks.

    Prefix integrals retain exact partial-cell area. The bounded layer cache is
    per immutable surface snapshot; no cross-map or cross-request state exists.
    Returned dictionaries are read-only by contract so adjacent heading checks
    can reuse the last geometric result.
    """

    def __init__(self, surface):
        self.resolution = float(surface['resolution'])
        self.heights = np.asarray(surface['height'], dtype=float)
        self.unknown = np.asarray(surface['state']) == 'unknown'
        self.shape = (int(surface['nx']), int(surface['ny']))
        self.origin_xy_container = surface_origin_xy_container(surface,
            legacy_origin_xy_container=tuple(-n*self.resolution/2 for n in self.shape))
        self.unknown_prefix = self._prefix(self.unknown)
        self._layers = OrderedDict()
        self._last_key = self._last_result = None

    @staticmethod
    def _prefix(mask):
        return np.pad(np.asarray(mask, dtype=float), ((1, 0), (1, 0))).cumsum(0).cumsum(1)

    @staticmethod
    def _count(prefix, first, stop):
        x0, y0 = first
        x1, y1 = stop
        return int(prefix[x1, y1]-prefix[x0, y1]-prefix[x1, y0]+prefix[x0, y0])

    @staticmethod
    def _area(prefix, lower, upper):
        """Integral in cell units over a continuous axis-aligned rectangle."""
        def primitive(x, y):
            nx, ny = prefix.shape[0]-1, prefix.shape[1]-1
            x, y = min(max(x, 0.), nx), min(max(y, 0.), ny)
            ix, iy = min(math.floor(x), nx-1), min(math.floor(y), ny-1)
            fx, fy = x-ix, y-iy
            return ((1-fx)*(1-fy)*prefix[ix, iy] + fx*(1-fy)*prefix[ix+1, iy]
                + (1-fx)*fy*prefix[ix, iy+1] + fx*fy*prefix[ix+1, iy+1])
        x0, y0 = lower
        x1, y1 = upper
        return primitive(x1, y1)-primitive(x0, y1)-primitive(x1, y0)+primitive(x0, y0)

    def _layer(self, bottom):
        if bottom not in self._layers:
            contact = np.abs(self.heights-bottom) <= HEIGHT_NUMERIC_EPSILON_M
            # The physical floor can support a box without observed surface
            # points. This does not authorize unknown XYZ volume above it.
            if abs(bottom) > HEIGHT_NUMERIC_EPSILON_M:
                contact &= ~self.unknown
            self._layers[bottom] = (contact, self._prefix(contact),
                self._prefix(self.heights > bottom+HEIGHT_NUMERIC_EPSILON_M))
            if len(self._layers) > 32:
                self._layers.popitem(last=False)
        self._layers.move_to_end(bottom)
        return self._layers[bottom]

    def evaluate(self, center_xy_container, footprint, bottom, min_support_ratio):
        key = (tuple(center_xy_container), tuple(footprint), float(bottom), float(min_support_ratio))
        if key != self._last_key:
            self._last_result = self._evaluate(*key)
            self._last_key = key
        return self._last_result

    def _evaluate(self, center, footprint, bottom, min_ratio):
        lower = [(center[a]-footprint[a]/2-self.origin_xy_container[a])/self.resolution
                 for a in range(2)]
        upper = [(center[a]+footprint[a]/2-self.origin_xy_container[a])/self.resolution
                 for a in range(2)]
        first = [math.floor(v+1e-9) for v in lower]
        stop = [math.ceil(v-1e-9) for v in upper]
        result = dict(support_model='native_horizontal_contact_v1',
            contact_height_numeric_epsilon_m=HEIGHT_NUMERIC_EPSILON_M,
            physical_flatness='not_established_by_native_grid',
            centre_in_support_polygon=None, centre_support_method='not_evaluated',
            min_support_ratio=min_ratio, reason=None)
        if any(first[a] < 0 or stop[a] > self.shape[a] for a in range(2)):
            result['reason'] = 'outside_hull'
            return result
        contact, prefix, penetration = self._layer(bottom)
        penetrating = self._count(penetration, first, stop)
        unknown = self._count(self.unknown_prefix, first, stop)
        total = math.prod(upper[a]-lower[a] for a in range(2))
        area = self._area(prefix, lower, upper)
        ratio = min(1., max(0., area/total))
        result.update(footprint_area_m2=total*self.resolution**2,
            supported_area_m2=area*self.resolution**2, support_ratio=ratio,
            penetration_columns=penetrating, unknown_columns=unknown)
        if penetrating:
            result['reason'] = 'edge_occupied_overlap'
        elif unknown and abs(bottom) > HEIGHT_NUMERIC_EPSILON_M:
            result['reason'] = 'unknown_above_floor'
        elif ratio < min_ratio-1e-9:
            result['reason'] = 'insufficient_support'
        else:
            # A line through the centre divides this rectangle into equal
            # areas. More than half its area cannot lie wholly on one side;
            # therefore the centre lies in the contact convex hull. This
            # avoids constructing a polygon for the default 0.6 threshold.
            if ratio > .5+1e-9:
                inside, method = True, 'greater_than_half_footprint_area'
            else:
                inside = self._centre_in_contact_polygon(contact, first, stop, lower, upper)
                method = 'clipped_contact_polygon'
            result.update(centre_in_support_polygon=inside, centre_support_method=method)
            if not inside:
                result['reason'] = 'support_centre_outside_polygon'
        return result

    @staticmethod
    def _centre_in_contact_polygon(contact, first, stop, lower, upper):
        """Convex hull of actual clipped patches for lower configured thresholds.

        Work in native cell coordinates; the cross-product tolerance is scaled
        by edge length so it represents a positional floating-point tolerance.
        The centre cell itself need not be in contact (e.g. a bounded hole).
        """
        points = []
        for ix, iy in np.argwhere(contact[first[0]:stop[0], first[1]:stop[1]]):
            x, y = int(ix)+first[0], int(iy)+first[1]
            x0, y0 = max(lower[0], x), max(lower[1], y)
            x1, y1 = min(upper[0], x+1), min(upper[1], y+1)
            if x1 > x0 and y1 > y0:
                points.extend(((x0, y0), (x0, y1), (x1, y0), (x1, y1)))
        points = sorted(set(points))
        if len(points) < 3:
            return False

        def cross(a, b, c):
            return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])

        def half_hull(ordered):
            hull = []
            for point in ordered:
                while len(hull) >= 2 and cross(hull[-2], hull[-1], point) <= 0.:
                    hull.pop()
                hull.append(point)
            return hull

        polygon = half_hull(points)[:-1]+half_hull(reversed(points))[:-1]
        center = tuple((lo+hi)/2 for lo, hi in zip(lower, upper))
        return all(cross(a, b, center) >= -1e-9*math.dist(a, b)
                   for a, b in zip(polygon, polygon[1:]+polygon[:1]))
