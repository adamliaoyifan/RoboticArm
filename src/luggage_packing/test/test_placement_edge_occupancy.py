"""Edge-occupancy regression (adjudication 2026-11-08).

The ceil-sized window cells are anchored to the hull edge while the map
grid anchors at -(nx/2)*resolution: on a 75x99 grid over a 1.49x1.97
hull the box's real extent spills 5mm past the last window cell into
the adjacent column. The conservatively-occupied wall column was never
read by window stats, and the solver returned floor-level candidates
whose footprint overlaps it — exactly the run07 K01 candidates the
independent oracle rejected. These tests pin the footprint-coverage
fix and its two no-false-reject controls.
"""
from __future__ import annotations

import math

import pytest

try:
    from luggage_packing.luggage_packing import placement_solver as ps
except ImportError:  # ament-flattened install
    from luggage_packing import placement_solver as ps

RES = 0.02
NX, NY = 75, 99          # grid 1.50 x 1.98 over a 1.49 x 1.97 hull
HX, HY = 1.49 / 2.0, 1.97 / 2.0
FLOOR = 0.53
CEIL = 2.01
INNER_H = CEIL - FLOOR
BOX = [0.30, 0.20, 0.20]


def _map(state_fn):
    """surface_map_2d v1 dict with grid anchored at -(nx/2)*res and a
    cuboid geometry descriptor (hull legality must be active)."""
    return {
        "resolution": RES, "nx": NX, "ny": NY,
        "inner_size": [1.49, 1.97, INNER_H],
        "floor_z": 0.0,
        "center_base": [0.0, 0.0, FLOOR + INNER_H / 2.0],
        "yaw": 0.0,
        "height": [[(1.48 if state_fn(ix, iy) == "occupied" else 0.0)
                    for iy in range(NY)] for ix in range(NX)],
        "state": [[state_fn(ix, iy) for iy in range(NY)]
                  for ix in range(NX)],
        "confidence": [["sensor" if state_fn(ix, iy) in ("occupied", "free")
                        else "none" for iy in range(NY)] for ix in range(NX)],
        # the deployment chamfer hull (user ruling 2026-11-08: the real
        # chain is chamfer-only; the cuboid fixture was deleted)
        "geometry_descriptor": {
            "length": 1.49, "width": 1.97, "ceiling_z": CEIL,
            "floor_z": FLOOR, "frame_id": "container_link",
            "schema_version": 1,
            "chamfer": {"floor_y": 0.55, "side": "positive_y",
                        "wall_y": 0.985, "wall_z": 0.9},
            "geometry_hash": "cb41f55f243786d50ab30997291083ab"
                             "40467507abcf78a00e55da77ffc00856",
        },
    }


def _x_of(ix):
    return -NX * RES / 2.0 + ix * RES


def _interior(ix, iy):
    """A column is interior only when its WHOLE square lies inside the
    hull — a boundary column (square extends past the hull edge) is the
    conservative wall representation and reads occupied."""
    return (_x_of(ix) >= -HX - 1e-9 and _x_of(ix) + RES <= HX + 1e-9
            and -NY * RES / 2.0 + iy * RES >= -HY - 1e-9
            and -NY * RES / 2.0 + (iy + 1) * RES <= HY + 1e-9)


def _feasible(surface, box=BOX, yaws=(math.pi / 2.0,)):
    params = dict(ps._default_params())
    params["top_n"] = 8000   # the default 8 truncates before deep windows
    result = ps.generate_candidates(surface, box, allowed_yaws=list(yaws),
                                    params=params)
    candidates = result if isinstance(result, list) else result["candidates"]
    return [c for c in candidates if c.get("feasible")]


def test_wall_sliver_within_tolerance_deeper_rejects():
    """OCC overlap tolerance (user ruling 2026-11-08 night): with the
    exact-face hull margin and the boundary columns occupied full
    height, a footprint clipping up to one cell (2cm) into the occupied
    wall column is ACCEPTABLE; deeper than the tolerance rejects. The
    default (auto) margin keeps candidates a full cell away regardless."""
    surface = _map(
        lambda ix, iy: "occupied" if not _interior(ix, iy) else "free")
    # exact-face margin so candidates can actually reach the wall column
    params = dict(ps._default_params())
    params["top_n"] = 8000
    params["hull_margin"] = 0.0
    result = ps.generate_candidates(surface, BOX,
                                    allowed_yaws=[math.pi / 2.0],
                                    params=params)
    candidates = result if isinstance(result, list) else result["candidates"]
    feasible = [c for c in candidates if c.get("feasible")]
    assert feasible, "interior placements must remain available"
    for candidate in feasible:
        fw = candidate["footprint"][0]
        x1 = candidate["center_container"][0] + fw / 2.0
        x0 = candidate["center_container"][0] - fw / 2.0
        # wall column 74 spans [0.73, 0.75]; a clip up to 2cm is legal
        assert x1 <= 0.73 + RES + 1e-9, (
            "footprint clips deeper than one cell into the wall column", x1)
        assert x0 >= -0.73 - RES - 1e-9
        fd = candidate["footprint"][1]
        y1 = candidate["center_container"][1] + fd / 2.0
        y0 = candidate["center_container"][1] - fd / 2.0
        assert y1 <= 0.965 + RES + 1e-9 and y0 >= -0.965 - RES - 1e-9


def test_stacking_on_observed_cargo_still_legal():
    """No false reject: a box resting ON an observed cargo block (its
    top inside the support-contact band, full window support) must
    stay feasible — the edge scan only rejects occupied tops ABOVE the
    box bottom outside the band."""
    cargo = {(ix, iy) for ix in range(28, 33) for iy in range(38, 43)}

    def states(ix, iy):
        if (ix, iy) in cargo:
            return "occupied"
        return "free"

    surface = _map(states)
    for (ix, iy) in cargo:
        surface["height"][ix][iy] = 0.20
    feasible = _feasible(surface, box=[0.10, 0.10, 0.10],
                         yaws=(0.0,))
    assert feasible, "stacking on observed cargo must stay feasible"
    on_top = [c for c in feasible
              if c["center_container"][2] > FLOOR + 0.20 + 0.05 - 1e-6]
    assert on_top, "the on-top placement must be among the answers"


def test_unknown_edge_sliver_never_claims_known_free():
    """An unknown sliver under the footprint routes the candidate to
    the floor-prior answer with confidence 0 — never a >= 0.5 claim."""
    def states(ix, iy):
        if ix >= 40:
            return "unknown"       # unobserved half of the container
        if not _interior(ix, iy):
            return "occupied"
        return "free"

    surface = _map(states)
    result = ps.generate_candidates(surface, BOX,
                                    allowed_yaws=[math.pi / 2.0])
    candidates = result if isinstance(result, list) else result["candidates"]
    for candidate in candidates:
        fw = candidate["footprint"][0]
        x1 = candidate["center_container"][0] + fw / 2.0
        if candidate.get("feasible"):
            assert candidate["confidence_ratio"] < 0.5 or x1 <= _x_of(40), (
                "known-free claim over an unknown footprint is forbidden")
    prior = [c for c in candidates
             if c.get("feasible") and c["support_source"] == "floor_prior"]
    assert prior, "floor-prior answers over unknown ground must appear"
    assert all(c["confidence_ratio"] == 0.0 for c in prior)


def test_contact_band_is_gap_side_only():
    """OCC overlap band (user ruling 2026-11-08 night, superseding the
    earlier any-depth penetration rule): an occupied top 3cm ABOVE the
    box bottom exceeds the one-cell tolerance and must reject; 1cm
    above is an acceptable clip and passes; a top AT the bottom is face
    contact; 4cm below is the gap band and stays legal. An explicit
    occ_overlap_tolerance=0.0 restores the strict any-depth rule."""
    nx, ny, res = 75, 99, 0.02
    ox = -nx * res / 2.0

    def surface_with(top):
        height = [[0.0] * ny for _ in range(nx)]
        state = [["free"] * ny for _ in range(nx)]
        for ix in range(30, 46):          # under the 0.3x0.2 footprint
            for iy in range(40, 50):
                state[ix][iy] = "occupied"
                height[ix][iy] = top
        return {"resolution": res, "nx": nx, "ny": ny,
                "height": height, "state": state}

    # footprint centered at x=0.15, y=0.08 -> cells 30..45 in x
    surface = surface_with(0.23)
    overlaps, _ = ps._footprint_edge_state(
        surface, 0.15, 0.08, 0.30, 0.20, box_bottom=0.20)
    assert overlaps, "3cm above the bottom exceeds the 2cm tolerance"
    surface = surface_with(0.21)
    overlaps, _ = ps._footprint_edge_state(
        surface, 0.15, 0.08, 0.30, 0.20, box_bottom=0.20)
    assert not overlaps, "1cm clip is within the one-cell tolerance"
    overlaps, _ = ps._footprint_edge_state(
        surface, 0.15, 0.08, 0.30, 0.20, box_bottom=0.20,
        overlap_tol=0.0)
    assert overlaps, "explicit occ_overlap_tolerance=0.0 restores strict"
    surface = surface_with(0.20)
    overlaps, _ = ps._footprint_edge_state(
        surface, 0.15, 0.08, 0.30, 0.20, box_bottom=0.20)
    assert not overlaps, "face contact (top == bottom) stays legal"
    surface = surface_with(0.16)
    overlaps, _ = ps._footprint_edge_state(
        surface, 0.15, 0.08, 0.30, 0.20, box_bottom=0.20)
    assert not overlaps, "gap within the support tolerance stays legal"
    surface = surface_with(0.2 + 1e-12)
    overlaps, _ = ps._footprint_edge_state(
        surface, 0.15, 0.08, 0.30, 0.20, box_bottom=0.20)
    assert not overlaps, "numeric-representation dust is not penetration"


def test_default_hull_margin_is_one_cell():
    """Default wall margin = one map cell (2026-11-08 user directive).

    The map's conservative boundary column reaches up to one cell inside
    the exact wall face, so an exact-face margin can still slide a
    footprint into map-occupied ring cells. Default params keep the
    deepest footprint at least wall - resolution inside (quantized to
    the window grid: 0.715 here); an explicit hull_margin=0.0 restores
    the exact-face behavior (0.735, touching)."""
    surface = _map(lambda ix, iy: "free")

    feasible = _feasible(surface)   # default params (auto margin)
    assert feasible
    x1 = max(c["center_container"][0] + c["footprint"][0] / 2.0
             for c in feasible)
    assert x1 <= HX - RES + 1e-9, (
        "default margin must keep the footprint one cell inside the wall", x1)

    params = dict(ps._default_params())
    params["top_n"] = 8000
    params["hull_margin"] = 0.0
    result = ps.generate_candidates(surface, BOX,
                                    allowed_yaws=[math.pi / 2.0],
                                    params=params)
    candidates = result if isinstance(result, list) else result["candidates"]
    exact = [c for c in candidates if c.get("feasible")]
    x1_exact = max(c["center_container"][0] + c["footprint"][0] / 2.0
                   for c in exact)
    assert abs(x1_exact - 0.735) < 1e-9, (
        "explicit hull_margin=0.0 restores exact-face touching", x1_exact)
