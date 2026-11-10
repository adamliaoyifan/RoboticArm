"""Shared, ROS-free cargo-grid geometry and allocation/transport bounds."""
from dataclasses import dataclass
import math

# One bound for owner allocation, exported volumes and downstream admission.
MAX_VOLUME_CELLS = 8_000_000


@dataclass(frozen=True)
class CargoGridSpec:
    resolution_m: float
    dimensions_xyz: tuple

    @classmethod
    def create(cls, inner_size, resolution_m):
        size = tuple(float(v) for v in inner_size)
        resolution = float(resolution_m)
        if (len(size) != 3 or not all(math.isfinite(v) and v > 0 for v in size)
                or not math.isfinite(resolution) or resolution <= 0):
            raise ValueError('cargo grid requires finite positive dimensions/resolution')
        if any(v / resolution > MAX_VOLUME_CELLS for v in size):
            raise ValueError("cargo grid exceeds volume cell budget before allocation")
        # Preserve the historical ceil convention, including partial edge cells.
        dimensions = tuple(max(1, math.ceil(v / resolution)) for v in size)
        if math.prod(dimensions) > MAX_VOLUME_CELLS:
            raise ValueError('cargo grid exceeds volume cell budget before allocation')
        return cls(resolution, dimensions)


def deployment_resolution(value):
    result = float(value)
    if not math.isfinite(result) or not 0.005 <= result <= 0.05:
        raise ValueError('cargo grid resolution must be finite in [0.005, 0.05] m')
    return result
