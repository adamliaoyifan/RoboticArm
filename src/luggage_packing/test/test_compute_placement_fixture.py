import json
from pathlib import Path

from luggage_packing.placement_solver import (
    aabb_overlap,
    candidate_aabb,
    placement_constraint_reason,
    solve_placement,
)


FIXTURE = json.loads(
    (Path(__file__).parent / "fixtures" / "compute_placement_cases.json")
    .read_text(encoding="utf-8"))
PARAMS = {"top_n": 100, "keep_rejected": 100}


def _solve(name):
    case = FIXTURE["cases"][name]
    surface = FIXTURE[case["surface"]]
    bounds = case.get("hull_bounds")

    def hull_contains(point):
        return all(
            bounds[axis] - 1e-9 <= point[axis] <= bounds[axis + 3] + 1e-9
            for axis in range(3))

    validator = lambda candidate: placement_constraint_reason(
        candidate,
        surface["inner_size"][2],
        placed_aabbs=case.get("placed", []),
        aperture_y=case.get("aperture_y"),
        hull_contains=hull_contains if bounds else None,
        inner_size=surface["inner_size"] if case.get("smallest_size") else None,
        smallest_size=case.get("smallest_size"),
    )
    return case, surface, solve_placement(
        surface, case["box"], allowed_yaws=[0.0], params=PARAMS,
        candidate_validator=validator)


def test_empty_container_fixture_returns_floor_slot():
    case, _surface, result = _solve("empty")
    assert result["success"]
    assert result["map_revision"] == 7
    assert result["selected"]["support_source"] == "floor_prior"
    assert abs(result["selected"]["center_base"][2] - case["box"][2] / 2) < 1e-9


def test_existing_box_fixture_selects_non_overlapping_slot():
    case, surface, result = _solve("existing")
    assert result["success"]
    selected = candidate_aabb(result["selected"], surface["inner_size"][2])
    assert not any(aabb_overlap(selected, placed) for placed in case["placed"])


def test_exact_boundary_fixture_is_accepted():
    _case, surface, result = _solve("boundary")
    assert result["success"]
    actual = candidate_aabb(result["selected"], surface["inner_size"][2])
    expected = (-0.3, -0.2, 0.0, 0.3, 0.2, 0.2)
    assert all(abs(got - want) < 1e-9
               for got, want in zip(actual, expected))


def test_aperture_fixture_filters_before_selection():
    _case, _surface, result = _solve("aperture")
    assert result["success"]
    assert result["reject_histogram"]["outside_aperture"] > 0
    assert -0.1 - 1e-9 <= result["selected"]["center_local"][1] <= 0.1 + 1e-9


def test_hull_fixture_filters_before_selection():
    _case, surface, result = _solve("hull")
    assert result["success"]
    assert result["reject_histogram"]["outside_hull"] > 0
    assert candidate_aabb(result["selected"], surface["inner_size"][2])[0] >= -0.2


def test_stale_map_overlap_fixture_fails_closed():
    _case, _surface, result = _solve("overlap")
    assert not result["success"]
    assert result["reject_histogram"]["overlap"] > 0
    assert result["message"].startswith("BIN_FULL no_candidate")


def test_corridor_fixture_records_blocked_candidates():
    _case, _surface, result = _solve("corridor")
    assert result["reject_histogram"]["corridor_blocked"] > 0


def test_oversized_fixture_has_no_candidate():
    _case, _surface, result = _solve("no_candidate")
    assert not result["success"]
    assert result["candidates"] == []
    assert result["message"] == "BIN_FULL no_candidate"
