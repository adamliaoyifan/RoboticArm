"""Synthetic directed atlas for offline ST09-B tests (no ROS, no npz).

Implements exactly the public surface the Section B builders touch:
``query``/``resolution``/``grid_size``/``meta``/``contract_record``/
``yaw_bins`` (v2 directed semantics over 2pi, both pi twins distinct).
Cells are keyed by (ix, iy, iz, yaw-bin index); a cell without seeds for
the queried bin models an uncovered orientation.
"""

from __future__ import division

import math
from collections import namedtuple

from luggage_planning.atlas.frame_contracts import DIRECTED_YAW_BINS

#: Status values mirroring reachability_atlas semantics (>= MARGINAL == 2).
TOY_GOOD = 3
TOY_UNKNOWN = 0

ToyQuery = namedtuple(
    "ToyQuery",
    "status reason indices transit_seeds contact_seeds opening_connected "
    "hard_reject_safe")


def _bin_index(yaw, bins):
    best, best_err = 0, float("inf")
    for position, bin_yaw in enumerate(bins):
        err = abs((float(yaw) - bin_yaw + math.pi) % (2.0 * math.pi) - math.pi)
        if err < best_err:
            best_err, best = err, position
    return best


class ToyAtlas(object):
    """Seeded cells addressed by container-frame xyz and directed yaw bin."""

    def __init__(self, origin=(0.0, 0.0, 0.0), resolution=0.15, size=(4, 4, 4)):
        self._origin = tuple(float(v) for v in origin)
        self.resolution = float(resolution)
        self.grid_size = (int(size[0]), int(size[1]), int(size[2]),
                          len(DIRECTED_YAW_BINS))
        self._seeds = {}  # (ix, iy, iz, ibin) -> {"transit": [...], "contact": [...]}

    @property
    def meta(self):
        return {
            "grid": {"origin": list(self._origin)},
            "container": {"opening_side": "negative_x"},
            "contract": {"yaw_period_rad": 2.0 * math.pi},
        }

    @property
    def contract_record(self):
        return {"yaw_period_rad": 2.0 * math.pi}

    @property
    def yaw_bins(self):
        return list(DIRECTED_YAW_BINS)

    def add_seeds(self, cell, yaw, seeds, seed_set="transit"):
        """Attach seeds for one cell at the bin nearest ``yaw``."""
        index = (int(cell[0]), int(cell[1]), int(cell[2]),
                 _bin_index(yaw, DIRECTED_YAW_BINS))
        bucket = self._seeds.setdefault(index, {"transit": [], "contact": []})
        bucket[seed_set].extend(
            [float(v) for v in row] for row in seeds)

    def _cell_of(self, x, y, z):
        for axis, value in enumerate((x, y, z)):
            cells = self.grid_size[axis]
            position = int(math.floor(
                (float(value) - self._origin[axis]) / self.resolution))
            if position < 0 or position >= cells:
                return None
        return tuple(
            int(math.floor(
                (float(value) - self._origin[axis]) / self.resolution))
            for axis, value in enumerate((x, y, z)))

    def query(self, x, y, z, yaw=0.0, yaw_tolerance=0.1):
        cell = self._cell_of(x, y, z)
        if cell is None:
            return ToyQuery(TOY_UNKNOWN, "out_of_bounds", None, [], [], False, False)
        key = cell + (_bin_index(yaw, DIRECTED_YAW_BINS),)
        bucket = self._seeds.get(key, {"transit": [], "contact": []})
        seeded = bool(bucket["transit"] or bucket["contact"])
        return ToyQuery(
            status=TOY_GOOD if seeded else TOY_UNKNOWN,
            reason="" if seeded else "no_seeds",
            indices=key,
            transit_seeds=[list(row) for row in bucket["transit"]],
            contact_seeds=[list(row) for row in bucket["contact"]],
            opening_connected=seeded,
            hard_reject_safe=False)
