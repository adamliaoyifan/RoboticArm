import math
import pytest
from luggage_planning.routing.staging_candidates import staging_positions


def test_candidates_follow_container_axes_and_keep_baseline():
    points = staging_positions((1, 2, 3), (0, 0, math.sqrt(.5), math.sqrt(.5)))
    assert len(points) == 6
    assert points[0] == (1, 2, 3)
    assert points[1] == pytest.approx((1, 2.15, 3))
    assert points[3] == pytest.approx((.85, 2, 3))
    assert points[-1] == (1, 2, 3.2)


def test_tilted_container_does_not_lower_transit_floor():
    points = staging_positions((1, 2, 3), (0, math.sqrt(.5), 0, math.sqrt(.5)))
    assert all(p[2] >= 3 - 1e-9 for p in points)
    assert 1 < len(points) <= 6
    assert staging_positions(None, (0, 0, 0, 1)) == ()
