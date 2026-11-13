"""Native surface coordinates, with explicit compatibility for old v1 inputs."""
import math


def surface_origin_xy_container(surface, *, legacy_origin_xy_container):
    """Preserve the admitted producer origin instead of recentering ceil cells.

    The v2 adapter stores ``origin_xy_container`` as ``origin_local``. Older v1
    dictionaries omitted it; callers name their historical fallback explicitly
    because old candidate generation and cell-coverage helpers used different
    conventions. A malformed declaration is never replaced by that fallback.
    """
    origin = surface['origin_local'] if 'origin_local' in surface else legacy_origin_xy_container
    try:
        x,y = origin
        result = float(x),float(y)
        if not all(map(math.isfinite,result)):
            raise ValueError()
        return result
    except (TypeError,ValueError,OverflowError) as error:
        raise ValueError('finite container XY surface origin required') from error
