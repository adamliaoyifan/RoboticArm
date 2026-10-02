"""ST09-A acceptance: the 16-case deep-placement matrix plus the named
starvation outcomes.

2 yaw x 2 scene x 4 deep-region states on the production chamfered hull:
same support layer -> deeper legal slot must win; deep lost to occupancy or
corridor -> legal shallower slot; deep unobserved -> floor-prior deep slot
with an observation requirement, never a faked free/sensor slot. Dedicated
cases pin OBSERVATION_REQUIRED (starved map), BIN_FULL (ceiling-stacked
cargo: observation cannot conjure space) and PLACE_CANDIDATE_EXHAUSTED
(corridor policy) apart.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from helpers import (
    BOX_WDH,
    PARAMS_FULL,
    YAWS,
    chamfered_surface,
    hull_from_descriptor,
    surface_height_fn,
    surface_state_fn,
    validator_with,
)

from luggage_packing.placement_solver import (
    CODE_BIN_FULL,
    CODE_OBSERVATION_REQUIRED,
    CODE_PLACE_CANDIDATE_EXHAUSTED,
    solve_placement,
)

HULL = hull_from_descriptor()
INNER_SIZE = [1.49, 1.97, 1.48]
SMALLEST = [0.35, 0.25, 0.20]

# Placed-box scene: a carryon in the south-west shallow area (floor-relative
# AABB, container XY).
CARRYON_AABB = (-0.70, -0.90, 0.0, -0.30, -0.60, 0.28)
# Deep-region blocker: spans the deep third.
DEEP_BLOCKER_AABB = (0.20, -0.985, 0.0, 0.745, 0.60, 0.30)
# Corridor wall: full hull Y span at the carry z band, mid-lane. Shallow
# candidates in front of it keep a free corridor; everything behind is
# walled off.
CORRIDOR_WALL_AABB = (-0.05, -0.985, 0.0, 0.00, 0.60, 0.30)


def deep_state_surface(state):
    """Fresh observed floor with the deep third (ix >= 20) in ``state``."""
    regions = {
        "free": [(20, 30, 0, 40, "free")],
        "occupied": [(20, 30, 0, 40, "occupied")],
        "unknown": [(20, 30, 0, 40, "unknown")],
    }[state]
    return chamfered_surface(states=surface_state_fn(regions))


def solve(surface, yaw, scene, extra=()):
    placed = list(extra)
    if scene == "carryon":
        placed.append(CARRYON_AABB)
    validator = None
    if placed:
        validator = validator_with(
            placed_aabbs=placed, inner_size=INNER_SIZE,
            smallest_size=SMALLEST, hull=HULL, inner_h=INNER_SIZE[2])
    return solve_placement(
        surface, list(BOX_WDH), allowed_yaws=[yaw], params=PARAMS_FULL,
        candidate_validator=validator)


class TestDeepMatrix(object):
    def test_16_cases(self):
        for yaw in YAWS:
            for scene in ("fresh", "carryon"):
                label = "yaw=%.2f scene=%s" % (yaw, scene)

                result = solve(deep_state_surface("free"), yaw, scene)
                assert result["success"] is True, label + " deep_free"
                assert result["selected"]["depth_m"] > 0.9, label + " deep_free"
                assert result["selected"]["support_source"] == "sensor", (
                    label + " deep_free")

                result = solve(
                    deep_state_surface("free"), yaw, scene,
                    extra=[DEEP_BLOCKER_AABB])
                assert result["success"] is True, label + " deep_occupied"
                selected = result["selected"]
                # the deep region is genuinely lost: the winner sits entirely
                # in front of the blocker's x-range (0.20 .. 0.745)
                front_x = (
                    selected["center_container"][0]
                    + 0.5 * selected["footprint"][0])
                assert front_x <= 0.205, (
                    label + " deep_occupied: winner overlaps the blocker")
                assert selected["verdict"] == "legal", label + " deep_occupied"

                result = solve(
                    deep_state_surface("free"), yaw, scene,
                    extra=[CORRIDOR_WALL_AABB])
                assert result["success"] is True, label + " cannot_retreat"
                selected = result["selected"]
                # everything behind the wall (x > -0.05) is unreachable; the
                # winner must sit in front of it
                front_x = (
                    selected["center_container"][0]
                    + 0.5 * selected["footprint"][0])
                assert front_x <= -0.045, (
                    label + " cannot_retreat: winner sits behind the wall")
                assert selected["verdict"] == "legal", label + " cannot_retreat"

                result = solve(deep_state_surface("unknown"), yaw, scene)
                assert result["success"] is True, label + " deep_unknown"
                selected = result["selected"]
                # deep-first still prefers the deep region, and the
                # unobserved support stays floor_prior (never faked as
                # observed free), with an observation requirement.
                assert selected["depth_m"] > 0.9, label + " deep_unknown"
                assert selected["support_source"] == "floor_prior", (
                    label + " deep_unknown")
                # partial observed cells at the region boundary are honest;
                # a fully-observed slot would read 1.0 and support_source
                # sensor — the unobserved majority must remain visible.
                assert selected["confidence_ratio"] < 1.0, (
                    label + " deep_unknown")
                requirement = result["observation_requirement"]
                assert requirement is not None, label + " deep_unknown"
                assert requirement["missing_cells"], label + " deep_unknown"


class TestNamedOutcomes(object):
    def test_starved_map_is_observation_required_not_bin_full(self):
        """Everything unobserved above floor: no candidate is decidable, but
        occupied testimony is absent, so a bounded observation pass could
        open legal floor slots -> OBSERVATION_REQUIRED with a requirement
        over the best hypothetical window, never BIN_FULL."""
        surface = chamfered_surface(
            states=surface_state_fn([(0, 30, 0, 40, "unknown")]),
            heights=surface_height_fn([(0, 30, 0, 40, 0.10)]))
        result = solve_placement(surface, list(BOX_WDH), params=PARAMS_FULL)
        assert result["success"] is False
        assert result["reason_code"] == CODE_OBSERVATION_REQUIRED
        assert result["reject_histogram"].get("unknown_above_floor", 0) > 0
        requirement = result["observation_requirement"]
        assert requirement is not None
        assert requirement["missing_cells"]
        assert result["hypothetical_legal_windows"]

    def test_ceiling_stacked_cargo_is_bin_full(self):
        """Cargo stacked to the ceiling everywhere: no headroom anywhere and
        occupied columns under every footprint — observing anything cannot
        conjure space, so this is a genuine capacity failure -> BIN_FULL."""
        surface = chamfered_surface(
            states=surface_state_fn([(0, 30, 0, 40, "occupied")]),
            heights=surface_height_fn([(0, 30, 0, 40, 1.30)]))
        result = solve_placement(surface, list(BOX_WDH), params=PARAMS_FULL)
        assert result["success"] is False
        assert result["reason_code"] == CODE_BIN_FULL
        assert result["capacity_feasible_count"] == 0
        assert result["observation_requirement"] is None
        assert not result["hypothetical_legal_windows"]

    def test_corridor_only_failure_is_candidate_exhausted_not_bin_full(self):
        """A full-span wall at the opening walls off every corridor: zero
        feasible, but capacity remains (corridor is policy) ->
        PLACE_CANDIDATE_EXHAUSTED, never BIN_FULL."""
        surface = chamfered_surface()
        wall = (-0.70, -0.985, 0.0, -0.65, 0.60, 0.30)
        result = solve_placement(
            surface, list(BOX_WDH), params=PARAMS_FULL,
            candidate_validator=validator_with(
                placed_aabbs=[wall], inner_size=INNER_SIZE,
                smallest_size=SMALLEST, hull=HULL, inner_h=INNER_SIZE[2]))
        assert result["success"] is False
        assert result["reason_code"] == CODE_PLACE_CANDIDATE_EXHAUSTED
        assert result["capacity_feasible_count"] > 0

    def test_deep_preference_is_layer_first(self):
        """Same deep region available at floor and on a stacked support:
        the floor layer wins even though the stack is deeper (support layer
        sorts before depth)."""
        surface = chamfered_surface()
        nx, ny = surface["nx"], surface["ny"]
        # A tall support tower in the deepest column band: any window on it
        # sits at layer 0.30.
        for ix in range(nx - 6, nx):
            for iy in range(ny):
                surface["state"][ix][iy] = "occupied"
                surface["height"][ix][iy] = 0.30
        result = solve_placement(surface, list(BOX_WDH), params=PARAMS_FULL)
        assert result["success"] is True
        assert result["selected"]["support_layer_z"] == 0.0
        assert result["selected"]["depth_m"] > 0.5
