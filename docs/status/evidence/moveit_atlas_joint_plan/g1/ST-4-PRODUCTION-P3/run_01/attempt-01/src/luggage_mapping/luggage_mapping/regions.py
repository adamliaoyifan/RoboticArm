"""Exact native-grid coverage, independent of the O/G transport.

These are read results, not snapshot admission or motion certificates. Callers
must first admit the owner's immutable identity. A block retains the original
grid and explicit covered bounds; it is deliberately not a full-volume DTO.
"""
from dataclasses import dataclass
import math

from luggage_description.cargo_grid_contract import MAX_VOLUME_CELLS


def _cell_tuple(value):
    if (type(value) is not tuple or len(value) != 3
            or any(type(v) is not int for v in value)):
        raise ValueError('REGION_INTEGER_CELL_COORDINATES_REQUIRED')


@dataclass(frozen=True)
class NativeGrid:
    resolution_m: float
    origin_container: tuple
    dimensions_xyz: tuple

    def __post_init__(self):
        _cell_tuple(self.dimensions_xyz)
        if (type(self.origin_container) is not tuple or len(self.origin_container) != 3
                or not all(math.isfinite(v) for v in self.origin_container)
                or not math.isfinite(self.resolution_m) or self.resolution_m <= 0
                or min(self.dimensions_xyz) <= 0
                or math.prod(self.dimensions_xyz) > MAX_VOLUME_CELLS):
            raise ValueError('REGION_INVALID_NATIVE_GRID')

    def full_region(self):
        return CellRegion((0, 0, 0), self.dimensions_xyz)


@dataclass(frozen=True)
class CellRegion:
    """Half-open native cell bounds, including inactive cells in the AABB."""
    begin: tuple
    end: tuple

    def __post_init__(self):
        _cell_tuple(self.begin)
        _cell_tuple(self.end)
        if any(a < 0 or a >= b for a, b in zip(self.begin, self.end)):
            raise ValueError('REGION_EMPTY_OR_NEGATIVE_BOUNDS')

    @property
    def shape(self):
        return tuple(b-a for a, b in zip(self.begin, self.end))

    @property
    def count(self):
        return math.prod(self.shape)

    def validate_grid(self, grid):
        if any(b > n for b, n in zip(self.end, grid.dimensions_xyz)):
            raise ValueError('REGION_OUTSIDE_NATIVE_GRID')


@dataclass(frozen=True)
class CellBlock:
    grid: NativeGrid
    region: CellRegion
    identity: bytes
    states: bytes
    active: bytes

    def __post_init__(self):
        if type(self.grid) is not NativeGrid or type(self.region) is not CellRegion:
            raise ValueError('REGION_TYPED_GEOMETRY_REQUIRED')
        self.region.validate_grid(self.grid)
        if (type(self.identity) is not bytes or not self.identity
                or type(self.states) is not bytes or type(self.active) is not bytes
                or len(self.states) != self.region.count or len(self.active) != self.region.count
                or any(v not in (0, 1, 2) for v in self.states)
                or any(v not in (0, 1) for v in self.active)
                or any(s != 0 and not a for s, a in zip(self.states, self.active))):
            raise ValueError('REGION_INVALID_IDENTITY_OR_CELLS')

    def cell(self, index):
        """Read a global cell index. Outside coverage is never an implicit free."""
        _cell_tuple(index)
        if any(i < a or i >= b for i, a, b in zip(index, self.region.begin, self.region.end)):
            raise ValueError('REGION_NOT_COVERED')
        x, y, z = (i-a for i, a in zip(index, self.region.begin))
        nx, ny, _ = self.region.shape
        offset = x+nx*(y+ny*z)
        return self.states[offset], bool(self.active[offset])


def block_from_volume(volume, region, *, identity):
    """Slice an already identity-admitted immutable O/G volume; no RPC savings.

    The caller supplies the admitted identity as canonical bytes, including the
    producer and all snapshot legs. This function does not invent provenance or
    cache across identities. Inactive cells normalize to unknown, never free.
    """
    grid = NativeGrid(volume.resolution_m, tuple(volume.origin_container),
                      tuple(volume.dimensions_xyz))
    region.validate_grid(grid)
    count = math.prod(grid.dimensions_xyz)
    if len(volume.states) != count or len(volume.active) != count:
        raise ValueError('REGION_INCOMPLETE_SOURCE_VOLUME')
    nx, ny, _ = grid.dimensions_xyz
    states, active = bytearray(), bytearray()
    x0, y0, z0 = region.begin
    x1, y1, z1 = region.end
    for z in range(z0, z1):
        for y in range(y0, y1):
            for x in range(x0, x1):
                index = x+nx*(y+ny*z)
                state, enabled = volume.states[index], volume.active[index]
                if state not in (0, 1, 2) or type(enabled) is not bool:
                    raise ValueError('REGION_INVALID_SOURCE_CELL')
                states.append(state if enabled else 0)
                active.append(enabled)
    return CellBlock(grid, region, identity, bytes(states), bytes(active))
