#!/usr/bin/env python3
"""Atlas hook on solve_placement: hard-reject, prior re-rank, dump identity."""

import os
import sys
from collections import namedtuple

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(TEST_DIR, "..")))
sys.path.insert(0, os.path.normpath(os.path.join(TEST_DIR, "..", "..")))

from luggage_packing.placement_reachability import (  # noqa: E402
    NEUTRAL_PRIOR,
    make_atlas_annotate,
    solver_contact_point,
)
from luggage_packing.placement_solver import solve_placement  # noqa: E402

BOX = [0.2, 0.2, 0.2]
PARAMS = {"top_n": 100, "keep_rejected": 100}


def surface(height=0.0, inner_h=0.5, nx=6, ny=4, resolution=0.1):
    # ST05: the volume center sits above the container floor the hook
    # declares (0.53), keeping the fixture datum-consistent with
    # make_atlas_annotate(floor_z=0.53).
    return {
        "map_revision": 1,
        "resolution": resolution,
        "nx": nx, "ny": ny,
        "inner_size": [nx * resolution, ny * resolution, inner_h],
        "floor_z": 0.0,
        "center_base": [0.0, 0.0, 0.53 + inner_h * 0.5],
        "yaw": 0.0,
        "height": [[height] * ny for _ in range(nx)],
        "state": [["free"] * ny for _ in range(nx)],
        "confidence": [["sensor"] * ny for _ in range(nx)],
    }


_StubResult = namedtuple(
    "_StubResult", ("status", "reason", "hard_reject_safe"))


class StubAtlas(object):
    """Duck-typed atlas: status by (x, y) sign, hard-reject opt-in.

    ``axis`` selects which coordinate the stub keys on. The default keeps
    the historical x-split (right half reachable, left half unknown);
    ``axis="y"`` splits along +Y/-Y for tests that need the atlas to
    separate candidates the deep-first ordering key does not (ST09-A S1:
    selection orders by support layer, then depth — both x-aligned here —
    so an x-keyed prior can no longer *reorder*; a y-keyed one still can).
    """

    yaw_bins = (0.0, 1.5707963267948966)

    def __init__(self, reject=False, axis="x"):
        self.reject = reject
        self.axis = axis
        self.queries = []

    def query(self, x, y, z, yaw):
        self.queries.append((round(x, 3), round(y, 3), round(z, 3), yaw))
        coordinate = x if self.axis == "x" else y
        # positive side reachable, negative side unknown (or unreachable)
        if coordinate >= 0.0:
            return _StubResult(3, "reachable", False)
        if self.reject:
            return _StubResult(1, "unreachable", True)
        return _StubResult(0, "unknown", False)


def candidates_for(surface_map, box=BOX):
    result = solve_placement(surface_map, box, allowed_yaws=[0.0],
                             params=PARAMS)
    return result["candidates"]


def test_solver_contact_point_matches_aabb_convention():
    inner_h, floor_z, box_h = 0.5, 0.53, 0.25
    candidate = {
        "center_local": [0.1, -0.2, 0.05],
        "size": [0.3, 0.3, box_h],
    }
    x, y, z = solver_contact_point(candidate, floor_z, inner_h)
    # peak recovered from the AABB convention: z_local + inner_h/2 - box_h/2
    assert x == 0.1 and y == -0.2
    peak = 0.05 + inner_h / 2.0 - box_h / 2.0
    assert z == floor_z + peak + box_h


def test_hook_records_status_and_queries_at_contact():
    atlas = StubAtlas()
    hook = make_atlas_annotate(atlas, 0.53, 2.01)
    candidates, report = hook(candidates_for(surface()))
    assert report["atlas"] and report["n_candidates"] == len(candidates)
    assert report["status_histogram"]["reachable"] > 0
    for candidate in candidates:
        assert candidate["atlas_status"] in (0, 3)
        assert candidate["atlas_status_name"] in ("reachable", "unknown")
        assert "reachability_prior" in candidate
    # contact z is container-absolute: floor + peak + box height
    x, y, z, _yaw = atlas.queries[0]
    assert z > 0.53


def test_hook_hard_reject_flips_feasible_and_histogram():
    atlas = StubAtlas(reject=True)
    hook = make_atlas_annotate(atlas, 0.53, 2.01, hard_reject=True)
    result = solve_placement(
        surface(), BOX, allowed_yaws=[0.0], params=PARAMS,
        atlas_annotate=hook)
    rejected = [c for c in result["candidates"]
                if c.get("reason") == "atlas_unreachable"]
    assert rejected
    assert result["reject_histogram"].get("atlas_unreachable") == len(rejected)
    assert all(not c["feasible"] for c in rejected)
    # a feasible candidate survives on the reachable side
    assert result["success"]
    assert result["atlas"]["n_hard_rejected"] == len(rejected)
    assert result["selected"]["atlas_status"] == 3


def test_hook_prior_reorders_selection():
    # ST09-A S1: selection follows the frozen deep-first key (support layer,
    # depth, route clearance, score) — both are x-aligned on this fixture,
    # so an x-keyed prior can no longer change the winner. A y-keyed atlas
    # separates candidates the ordering key does not, and the weighted
    # prior must then flip the winner through the score term: the base
    # confidence spread favours the -Y rows, the prior favours +Y.
    atlas = StubAtlas(axis="y")  # +Y half reachable, -Y half unknown
    hook = make_atlas_annotate(atlas, 0.53, 2.01, score_weight=5.0)
    split = surface()
    resolution = split["resolution"]
    for iy in range(split["ny"]):
        cell_y = -0.2 + (iy + 0.5) * resolution
        confidence = "sensor" if cell_y < 0.0 else "geometry"
        for ix in range(split["nx"]):
            split["confidence"][ix][iy] = confidence
    plain = solve_placement(split, BOX, allowed_yaws=[0.0], params=PARAMS)
    ranked = solve_placement(
        split, BOX, allowed_yaws=[0.0], params=PARAMS,
        atlas_annotate=hook)
    assert plain["selected"]["center_local"][1] < 0.0
    assert ranked["selected"]["center_local"] != \
        plain["selected"]["center_local"]
    assert ranked["selected"]["center_local"][1] >= 0.0
    assert ranked["selected"]["atlas_status"] == 3
    assert ranked["selected"]["score"] == round(
        ranked["selected"]["score"], 4)


def test_hook_dump_only_changes_nothing_but_annotates():
    atlas = StubAtlas(reject=True)
    baseline = solve_placement(
        surface(), BOX, allowed_yaws=[0.0], params=PARAMS)
    hooked = solve_placement(
        surface(), BOX, allowed_yaws=[0.0], params=PARAMS,
        atlas_annotate=make_atlas_annotate(
            atlas, 0.53, 2.01, hard_reject=True, dump_only=True))
    assert hooked["selected"]["center_local"] == \
        baseline["selected"]["center_local"]
    assert hooked["reject_histogram"] == baseline["reject_histogram"]
    assert hooked["atlas"]["n_hard_rejected"] == 0
    assert all("atlas_status" in c
               for c in hooked["candidates"])


def test_hook_no_atlas_is_neutral():
    hook = make_atlas_annotate(None, 0.53, 2.01)
    baseline = solve_placement(
        surface(), BOX, allowed_yaws=[0.0], params=PARAMS)
    hooked = solve_placement(
        surface(), BOX, allowed_yaws=[0.0], params=PARAMS,
        atlas_annotate=hook)
    assert hooked["selected"]["center_local"] == \
        baseline["selected"]["center_local"]
    assert hooked["atlas"]["atlas"] is False
    assert all(c["reachability_prior"] == NEUTRAL_PRIOR
               for c in hooked["candidates"])


def test_hook_none_keeps_result_shape():
    result = solve_placement(
        surface(), BOX, allowed_yaws=[0.0], params=PARAMS)
    assert result["atlas"] is None
