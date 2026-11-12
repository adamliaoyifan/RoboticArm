"""Corridor lattice: kinematic gates and candidate fallback, no MoveIt."""

from __future__ import division

import math
import re
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT / "src" / "luggage_planning") not in sys.path:
    sys.path.append(str(ROOT / "src" / "luggage_planning"))

from luggage_planning.routing.corridor_lattice import (  # noqa: E402
    BRANCH_DISTANCE_RAD,
    EXCURSION_MIN_PATH_M,
    LINE_MAX_JUMP_RAD,
    LINE_STEP_M,
    WRIST_SINGULARITY_MARGIN_RAD,
    CorridorRequest,
    PreparedCorridor,
    branch_seeds,
    corridor_layers,
    kinematic_queries,
    prepare_corridor,
    search_candidates,
    search_lazy,
    search_prepared,
    solve_corridor,
    step_reject,
)
from luggage_planning.routing.grid_search import (  # noqa: E402
    DESCEND_BRANCH_TOL_RAD,
    ROUTE_TOTAL_WALL_S,
)
from luggage_planning.routing.moveit_edge_adapter import (  # noqa: E402
    CARTESIAN_EXCURSION_RAD_PER_M,
    CARTESIAN_MIN_FRACTION,
)

ANCHORS = {
    "portal": (0.00, 0.0, 0.10),
    "slot_approach": (0.02, 0.0, 0.10),
    "slot_upper": (0.04, 0.0, 0.08),
    "slot": (0.04, 0.0, 0.06),
}
LIMITS = tuple((-6.0, 6.0) for _ in range(6))
ORIENTATION = (0.0, 1.0, 0.0, 0.0)
START = (0.2, -0.5, 0.4, -1.0, 1.2, 0.3)


def _literal(path, name):
    text = Path(path).read_text(encoding="utf-8")
    match = re.search(
        r"^%s\s*=\s*([0-9]+(?:\.[0-9]+)?)" % re.escape(name),
        text, re.M)
    assert match, name
    return float(match.group(1))


def _smooth(position, _orientation, _seed):
    return [0.2, -0.5 + position[0], 0.4, -1.0, 1.2, 0.3], {}


def _request(anchors=None, candidate_id="c0", start=START):
    return CorridorRequest(
        anchors=dict(anchors or ANCHORS),
        orientation=ORIENTATION,
        start_q=start,
        joint_limits=LIMITS,
        candidate_id=candidate_id)


def _allow(_joints, _load):
    return True


def test_constants_are_the_existing_ones():
    matrix = ROOT / "scripts" / "st09_i" / "route_matrix_eval.py"
    executor = (ROOT / "src" / "luggage_planning" / "luggage_planning"
                / "motion_executor.py")
    builder = (ROOT / "src" / "luggage_planning" / "scripts"
               / "reachability_atlas_builder.py")
    assert LINE_STEP_M == _literal(matrix, "LINE_STEP_M")
    assert LINE_MAX_JUMP_RAD == _literal(matrix, "LINE_MAX_JUMP_RAD")
    assert WRIST_SINGULARITY_MARGIN_RAD == _literal(
        executor, "_WRIST_SINGULARITY_MARGIN_RAD")
    assert EXCURSION_MIN_PATH_M == _literal(
        executor, "_CARTESIAN_EXCURSION_MIN_PATH_M")
    assert CARTESIAN_EXCURSION_RAD_PER_M == 3.0
    assert CARTESIAN_MIN_FRACTION == 0.95
    assert DESCEND_BRANCH_TOL_RAD == 1.0
    assert ROUTE_TOTAL_WALL_S == 60.0
    assert BRANCH_DISTANCE_RAD == 0.25
    assert 'get_param("~branch_distance", 0.25)' in builder.read_text(
        encoding="utf-8")
    lattice = (ROOT / "src" / "luggage_planning" / "luggage_planning"
               / "corridor_lattice.py").read_text(encoding="utf-8")
    assert "CARTESIAN_MIN_FRACTION" not in lattice


def test_eight_branch_seeds_keep_start_and_atlas_order():
    seeds = branch_seeds(START, atlas_seeds=((0.1, 0.2, 0.3, 0.4, 0.5, 0.6),))
    assert seeds[0][0] == "start"
    assert seeds[0][1] == START
    assert seeds[-1][0] == "atlas_0"
    assert len(seeds) == 9
    assert branch_seeds(START) == branch_seeds(START)


def test_chain_is_deterministic_and_covers_entry_and_retreat():
    loads = []

    def collision(joints, load):
        loads.append(load)
        return True

    first = solve_corridor(_request(), _smooth, collision)
    second = solve_corridor(_request(), _smooth, _allow)
    assert first.ok and first.reason == ""
    assert first.chain == second.chain
    assert first.layer_names == second.layer_names
    names = set(first.layer_names)
    for name in ("portal", "slot_approach", "slot_upper", "slot",
                 "slot_upper_retreat", "slot_approach_retreat",
                 "portal_retreat"):
        assert name in names
    assert "carrying" in first.edge_loads
    assert "unloaded" in first.edge_loads
    switch = first.edge_loads.index("unloaded")
    assert set(first.edge_loads[:switch]) == {"carrying"}
    assert set(first.edge_loads[switch:]) == {"unloaded"}
    assert loads == list(first.edge_loads)
    assert first.n_collision_calls == len(first.edge_loads)
    assert first.max_step_rad <= LINE_MAX_JUMP_RAD
    assert first.max_step_rad <= DESCEND_BRANCH_TOL_RAD
    assert first.within_budget
    assert first.wall_time_s >= 0.0
    assert first.n_vertices >= len(first.layer_names)
    assert first.n_edges >= len(first.chain) - 1


def test_over_budget_keeps_the_chain_and_flags_the_clock():
    result = solve_corridor(
        _request(), _smooth, _allow, budget_s=-1.0)
    assert result.ok
    assert result.within_budget is False


def test_pi_jump_has_no_chain():
    def flipped(position, _orientation, _seed):
        base = [0.2, -0.5, 0.4, -1.0, 1.2, 0.3]
        if abs(position[0]) > 1e-9:
            base[0] += math.pi
        return base, {}

    result = solve_corridor(_request(), flipped, _allow)
    assert result.ok is False
    assert result.reason == "LATTICE_NO_CHAIN"
    assert step_reject(
        (0.2, -0.5, 0.4, -1.0, 1.2, 0.3),
        (0.2 + math.pi, -0.5, 0.4, -1.0, 1.2, 0.3),
        ANCHORS["portal"], ANCHORS["slot_approach"]) == "jump"


def test_excursion_wrist_and_collision_reject_on_their_own_gates():
    near = (0.2, -0.5, 0.4, -1.0, 1.2, 0.3)
    excursion = list(near)
    excursion[1] += 0.2
    assert step_reject(
        near, excursion, (0.0, 0.0, 0.0), (0.02, 0.0, 0.0)) == "excursion"
    singular = list(near)
    singular[4] = 0.1
    assert step_reject(
        singular, singular, (0.0, 0.0, 0.0), (0.02, 0.0, 0.0)) == "wrist"

    def block(_joints, _load):
        return False

    result = solve_corridor(_request(), _smooth, block)
    assert result.reason == "LATTICE_NO_CHAIN"
    assert result.n_collision_calls > 0


def test_joint_limit_empties_the_layer():
    def outside(_position, _orientation, _seed):
        return [10.0, 0.0, 0.0, 0.0, 1.2, 0.0], {}

    result = solve_corridor(_request(), outside, _allow)
    assert result.reason == "LATTICE_LAYER_EMPTY:portal"


def test_empty_layer_names_the_slot_and_the_next_candidate_runs():
    def missing_slot(position, orientation, seed):
        if abs(position[2] - 0.06) < 1e-9:
            return None, {}
        return _smooth(position, orientation, seed)

    blocked = _request(candidate_id="blocked")
    clear_anchors = dict(ANCHORS)
    clear_anchors["slot"] = (0.04, 0.0, 0.05)
    clear = _request(clear_anchors, candidate_id="clear")
    first = solve_corridor(blocked, missing_slot, _allow)
    assert first.reason == "LATTICE_LAYER_EMPTY:slot"
    winner, rows = search_candidates(
        (blocked, clear), missing_slot, _allow)
    assert [row.candidate_id for row in rows] == ["blocked", "clear"]
    assert rows[0].ok is False
    assert winner is not None and winner.ok
    assert winner.candidate_index == 1
    assert winner.candidate_id == "clear"


def test_collision_argument_is_required():
    try:
        solve_corridor(_request(), _smooth, None)
    except ValueError as exc:
        assert "collision" in str(exc)
    else:
        raise AssertionError("missing collision check was accepted")


# -- Generation 2: equivalent performance ---------------------------------

LANE_EVEN = (0.0, 0.26, 0.52)
LANE_ODD = (0.13, 0.39)


def _lane_corridor(extra_flip_layer=None):
    """Prepared layers whose lanes cross every step (many equal costs)."""
    layers, release = corridor_layers(ANCHORS)
    solutions = []
    for layer in range(len(layers)):
        values = LANE_EVEN if layer % 2 == 0 else LANE_ODD
        row = [(0.2 + value, -0.5, 0.4, -1.0, 1.2, 0.3) for value in values]
        if layer == extra_flip_layer:
            row.append((0.2 + math.pi, -0.5, 0.4, -1.0, 1.2, 0.3))
        row.sort(key=lambda joints: tuple(round(v, 6) for v in joints))
        solutions.append(tuple(row))
    return PreparedCorridor(
        layers=layers, solutions=tuple(solutions), release=release,
        n_vertices=sum(len(row) for row in solutions), reason="",
        candidate_id="lanes")


def _lazy_validate(mask, seen):
    def validate(states):
        seen.extend(states)
        return {state: mask[state] for state in states}
    return validate


def test_lazy_equals_full_mask_on_random_masks():
    import random

    prepared = _lane_corridor()
    queries = kinematic_queries(prepared)
    assert len(queries) > len(prepared.layers)
    outcomes = {"chain": 0, "none": 0}
    for trial in range(300):
        rng = random.Random(1000 + trial)
        rate = (0.0, 0.15, 0.35, 0.6)[trial % 4]
        mask = {state: rng.random() >= rate for state in queries}
        full = search_prepared(
            prepared, lambda joints, load: mask[(joints, load)])
        seen = []
        lazy, verdicts = search_lazy(
            prepared, _lazy_validate(mask, seen))
        assert lazy.ok == full.ok
        assert lazy.reason == full.reason
        assert lazy.chain == full.chain
        assert set(seen) <= set(queries)
        assert len(seen) == len(set(seen))
        for state, clear in verdicts.items():
            assert clear == mask[state]
        outcomes["chain" if full.ok else "none"] += 1
    assert outcomes["chain"] > 20 and outcomes["none"] > 20


def test_lazy_stops_at_the_first_clear_chain():
    prepared = _lane_corridor()
    queries = kinematic_queries(prepared)
    seen = []
    result, _verdicts = search_lazy(
        prepared, _lazy_validate({state: True for state in queries}, seen))
    assert result.ok
    chain_states = {
        (joints, load)
        for joints, load in zip(result.chain[1:], result.edge_loads)}
    assert set(seen) == chain_states
    assert len(seen) < len(queries)


def test_pruned_state_never_reaches_collision():
    flip_layer = 5
    prepared = _lane_corridor(extra_flip_layer=flip_layer)
    flipped = next(
        joints for joints in prepared.solutions[flip_layer]
        if joints[0] > 3.0)
    queries = kinematic_queries(prepared)
    assert all(joints != flipped for joints, _load in queries)
    full_calls = []

    def collision(joints, load):
        full_calls.append(joints)
        return True

    search_prepared(prepared, collision)
    assert flipped not in full_calls
    seen = []
    search_lazy(prepared, _lazy_validate(
        {state: True for state in queries}, seen))
    assert all(joints != flipped for joints, _load in seen)


def test_exact_ik_cache_keeps_every_layer():
    plain_stats, cached_stats = {}, {}
    plain = prepare_corridor(_request(), _smooth, stats=plain_stats)
    cached = prepare_corridor(
        _request(), _smooth, cache_ik=True, stats=cached_stats)
    assert plain.solutions == cached.solutions
    assert plain.layers == cached.layers
    assert plain_stats["cache_hits"] == 0
    assert cached_stats["cache_hits"] > 0
    assert (cached_stats["refine_calls"] + cached_stats["cache_hits"]
            == plain_stats["refine_calls"])


def test_lazy_reports_an_empty_layer_without_collision():
    def missing_slot(position, orientation, seed):
        if abs(position[2] - 0.06) < 1e-9:
            return None, {}
        return _smooth(position, orientation, seed)

    prepared = prepare_corridor(_request(), missing_slot)
    seen = []
    result, verdicts = search_lazy(prepared, _lazy_validate({}, seen))
    assert result.reason == "LATTICE_LAYER_EMPTY:slot"
    assert seen == [] and verdicts == {}
    prepared = prepare_corridor(_request(), missing_slot)
    assert prepared.attempts[-1]["layer"] == "slot"
    assert prepared.attempts[-1]["seeds"] > 0
    assert prepared.attempts[-1]["raw_ik_ok"] == 0
    assert prepared.attempts[-1]["ik_fail"] == prepared.attempts[-1]["seeds"]


def test_wrist_margin_argument_does_not_change_the_default_gate():
    singular = (0.2, -0.5, 0.4, -1.0, 0.1, 0.3)
    ends = ((0.0, 0.0, 0.0), (0.02, 0.0, 0.0))
    assert step_reject(singular, singular, *ends) == "wrist"
    assert step_reject(singular, singular, *ends, wrist_margin=0.35) == "wrist"
    assert step_reject(singular, singular, *ends, wrist_margin=0.05) == ""


def test_anchor_variants_change_one_family_and_keep_the_slot():
    from luggage_planning.routing.producer import (
        RegionAnchorPlan,
        corridor_anchor_variants,
    )

    plan = RegionAnchorPlan(
        portal=(0.0, 0.0, 1.0),
        slot_approach=(0.2, 0.0, 1.0),
        slot_upper=(0.2, 0.0, 0.6),
        slot=(0.2, 0.0, 0.3),
        exit=(0.2, 0.0, 1.2),
        depth_m=0.2)
    rows = corridor_anchor_variants(plan, (0.0, 0.0, 0.0, 1.0))
    names = [name for name, _anchors in rows]
    assert names[0] == "baseline"
    assert "portal_height_+0.10" in names
    assert "approach_lateral_+0.08" in names
    assert "slot_upper_higher" in names
    by_name = dict(rows)
    assert by_name["baseline"]["slot"] == (0.2, 0.0, 0.3)
    assert by_name["portal_height_+0.10"]["portal"][2] == 1.10
    assert by_name["portal_height_+0.10"]["slot_approach"] == (0.2, 0.0, 1.0)
    lateral = by_name["approach_lateral_+0.08"]
    assert lateral["slot"] == (0.2, 0.0, 0.3)
    assert lateral["slot_upper"] == (0.2, 0.0, 0.6)
    assert abs(lateral["slot_approach"][1] - 0.08) < 1e-9
    assert abs(lateral["portal"][1] - 0.08) < 1e-9
    higher = by_name["slot_upper_higher"]["slot_upper"]
    assert abs(higher[0] - 0.2) < 1e-9 and abs(higher[1]) < 1e-9
    assert higher[2] > 0.6
