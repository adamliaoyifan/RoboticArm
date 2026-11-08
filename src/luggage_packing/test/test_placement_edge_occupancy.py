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
        "geometry_descriptor": {
            "length": 1.49, "width": 1.97, "ceiling_z": CEIL,
            "floor_z": FLOOR, "frame_id": "container_link",
            "schema_version": 1,
            # verbatim from the run07 cuboid payload — normalize_descriptor
            # recomputes and compares the hash
            "geometry_hash": "30a9914e50249d6abb42fcd6e861a977a"
                             "83986c728525cc8d1345f942ad48a4a",
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


def test_wall_sliver_candidates_rejected():
    """The regression: boundary columns occupied full height (the map's
    conservative wall representation). No feasible candidate's actual
    footprint may overlap them — the 5mm spill past the last window
    cell is a collision, and interior placements must still appear."""
    surface = _map(
        lambda ix, iy: "occupied" if not _interior(ix, iy) else "free")
    feasible = _feasible(surface)
    assert feasible, "interior placements must remain available"
    for candidate in feasible:
        fw = candidate["footprint"][0]
        x1 = candidate["center_container"][0] + fw / 2.0
        x0 = candidate["center_container"][0] - fw / 2.0
        # the boundary column starts at the first grid line past the hull
        assert x1 <= HX + 1e-9, (
            "footprint spills into the occupied wall column", x1)
        assert x0 >= -HX - 1e-9
        fd = candidate["footprint"][1]
        y1 = candidate["center_container"][1] + fd / 2.0
        y0 = candidate["center_container"][1] - fd / 2.0
        assert y1 <= HY + 1e-9 and y0 >= -HY - 1e-9


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
