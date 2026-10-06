"""Layered IK graph for the in-container place corridor.

The polyline is the existing anchor chain and its reverse:

    portal -> slot_approach -> slot_upper -> slot
        -> slot_upper -> slot_approach -> portal

Home and staging stay on the MoveIt joint plans outside this graph.
Each waypoint keeps every distinct solution of a fixed seed set (the
start joints, eight shoulder/elbow/wrist flips of those joints, and the
atlas seeds). A solution is also tracked from the previous waypoint so
a branch found there survives a 2 cm step. Edges exist only when the
existing jump, excursion, branch, and wrist limits all pass and the
injected collision check accepts the arrival state.

Entry edges are checked with the payload attached. The slot layer is
the release: edges that leave it are checked unloaded, with the placed
box as an obstacle in the caller's scene. This module does not plan
those scenes and does not execute.

A missing layer is ``LATTICE_LAYER_EMPTY:<name>``. A layer set with no
path through it is ``LATTICE_NO_CHAIN``. Seed order, dedup, and the
layer walk are fixed, so the same input yields the same joint chain.
"""

from __future__ import division

import math
import time
from dataclasses import dataclass, field
from typing import Callable, List, Sequence, Tuple

from luggage_planning.grid_path_search import (
    DESCEND_BRANCH_TOL_RAD,
    ROUTE_TOTAL_WALL_S,
    _wrap_rad,
)
from luggage_planning.moveit_edge_adapter import (
    CARTESIAN_EXCURSION_RAD_PER_M,
)
from luggage_planning.reachability_wavefront import joint_branch_distance

#: Paired with each other in scripts/st09_i/route_matrix_eval.py.
#: MoveIt's Cartesian jump_threshold is 0.0 there (disabled), so this
#: pair is the per-step limit the lattice reads. The unit test pins
#: these literals to that file.
LINE_STEP_M = 0.02
LINE_MAX_JUMP_RAD = 0.3

#: motion_executor._WRIST_SINGULARITY_MARGIN_RAD. That module imports
#: ROS, so the number is pinned by the unit test instead of imported.
WRIST_SINGULARITY_MARGIN_RAD = 0.50
WRIST_JOINT_INDEX = 4

#: motion_executor._CARTESIAN_EXCURSION_MIN_PATH_M. Short edges are
#: judged against this floor, matching cartesian_excursion.
EXCURSION_MIN_PATH_M = 0.05

#: reachability_atlas_builder ``~branch_distance`` default. Two refined
#: solutions closer than this are one branch.
BRANCH_DISTANCE_RAD = 0.25

ENTRY_ANCHORS = ("portal", "slot_approach", "slot_upper", "slot")
RETREAT_ANCHORS = ("slot_upper", "slot_approach", "portal")

Layer = Tuple[str, Tuple[float, float, float]]
JointVector = Tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class LatticeResult(object):
    """One candidate's corridor search."""

    ok: bool
    reason: str
    chain: Tuple[JointVector, ...] = ()
    layer_names: Tuple[str, ...] = ()
    edge_loads: Tuple[str, ...] = ()
    wall_time_s: float = 0.0
    n_vertices: int = 0
    n_edges: int = 0
    n_collision_calls: int = 0
    within_budget: bool = False
    max_step_rad: float = 0.0
    candidate_index: int = -1
    candidate_id: str = ""


@dataclass
class CorridorRequest(object):
    """Geometry and seeds for one placement candidate."""

    anchors: dict
    orientation: Sequence[float]
    start_q: Sequence[float]
    joint_limits: Sequence[Tuple[float, float]]
    atlas_seeds: Sequence[Sequence[float]] = field(default_factory=tuple)
    candidate_id: str = ""


def branch_seeds(start_q, atlas_seeds=()):
    """Start, eight shoulder/elbow/wrist flips, then atlas seeds.

    Shoulder is joint 1 shifted by 0 or -pi. Elbow is the sign of
    joint 3. Wrist flip adds pi to joints 4 and 6 and negates joint 5.
    Identical seeds are dropped, first label kept. Order is fixed.
    """
    base = tuple(float(v) for v in start_q)
    rows = [("start", base)]
    for shoulder_index, shoulder in enumerate((0.0, -math.pi)):
        for elbow_index, elbow_sign in enumerate((1.0, -1.0)):
            for wrist_index, wrist_flip in enumerate((False, True)):
                joints = list(base)
                joints[0] = base[0] + shoulder
                joints[2] = abs(base[2]) * elbow_sign
                if wrist_flip:
                    joints[3] = base[3] + math.pi
                    joints[4] = -base[4]
                    joints[5] = base[5] + math.pi
                rows.append((
                    "branch_s%d_e%d_w%d" % (
                        shoulder_index, elbow_index, wrist_index),
                    tuple(joints),
                ))
    for index, seed in enumerate(atlas_seeds or ()):
        rows.append((
            "atlas_%d" % index,
            tuple(float(v) for v in seed),
        ))
    seen = set()
    ordered = []
    for label, joints in rows:
        key = tuple(round(v, 6) for v in joints)
        if key in seen:
            continue
        seen.add(key)
        ordered.append((label, joints))
    return tuple(ordered)


def corridor_layers(anchors, step_m=LINE_STEP_M):
    """Sample the entry polyline and the reverse retreat.

    Returns ``(layers, release_index)``. ``release_index`` is the slot
    layer. Edges into it carry the payload; edges out of it do not.
    Endpoint samples use the anchor coordinates, not the interpolated
    float, so a repeated run builds the same positions.
    """
    missing = [name for name in ENTRY_ANCHORS if name not in anchors]
    if missing:
        raise ValueError("corridor anchors missing %s" % ",".join(missing))
    named = [(name, anchors[name]) for name in ENTRY_ANCHORS]
    named.extend(
        ("%s_retreat" % name, anchors[name]) for name in RETREAT_ANCHORS)
    layers: List[Layer] = []
    for name, point in named:
        point = tuple(float(v) for v in point)
        if not layers:
            layers.append((name, point))
            continue
        start = layers[-1][1]
        distance = _distance(start, point)
        if distance <= 1e-9:
            layers.append((name, point))
            continue
        steps = max(1, int(math.ceil((distance / float(step_m)) - 1e-12)))
        for index in range(1, steps + 1):
            if index == steps:
                layers.append((name, point))
                continue
            scale = index / float(steps)
            sample = tuple(
                start[axis] + (point[axis] - start[axis]) * scale
                for axis in range(3))
            layers.append(("%s_step_%03d" % (name, index), sample))
    release = next(
        index for index, (name, _point) in enumerate(layers)
        if name == "slot")
    return tuple(layers), release


def step_reject(q_from, q_to, xyz_from, xyz_to, wrist_margin=None):
    """First kinematic gate this step fails, or ``""``.

    Order is the plan's order: per-step jump, excursion, branch
    tolerance, wrist margin. Collision is applied by the caller after
    this returns empty. Jump uses the wrapped per-joint delta; excursion
    uses the raw total variation, matching ``cartesian_excursion``.
    ``wrist_margin`` defaults to ``WRIST_SINGULARITY_MARGIN_RAD``. A
    smaller value is a diagnostic ablation and is not the frozen gate.
    """
    margin = (WRIST_SINGULARITY_MARGIN_RAD if wrist_margin is None
              else float(wrist_margin))
    wrapped = [_wrap_rad(float(b) - float(a)) for a, b in zip(q_from, q_to)]
    if max(abs(delta) for delta in wrapped) > LINE_MAX_JUMP_RAD:
        return "jump"
    raw = [abs(float(b) - float(a)) for a, b in zip(q_from, q_to)]
    length = _distance(xyz_from, xyz_to)
    denominator = max(float(length), EXCURSION_MIN_PATH_M)
    if (max(raw) / denominator) > CARTESIAN_EXCURSION_RAD_PER_M:
        return "excursion"
    if max(abs(delta) for delta in wrapped) > DESCEND_BRANCH_TOL_RAD:
        return "branch"
    if not _wrist_ok(q_from, q_to, margin=margin):
        return "wrist"
    return ""


def layer_family(name):
    """Anchor family of a sampled layer name.

    ``slot_approach_step_012`` and ``slot_approach_retreat`` both report
    ``slot_approach``. The retreat suffix is kept only as ``retreat``.
    """
    text = str(name or "")
    retreat = text.endswith("_retreat") or "_retreat_step_" in text
    head = text.split("_step_", 1)[0]
    if head.endswith("_retreat"):
        head = head[: -len("_retreat")]
    return head, retreat


def has_kinematic_chain(prepared, wrist_margin=None):
    """True when some joint chain passes every kinematic gate.

    Collision is treated as clear. An empty layer is not a chain.
    """
    return kinematic_chain_joints(prepared, wrist_margin=wrist_margin) is not None


def gate_counts(prepared, wrist_margin=None):
    """Kinematic-gate histogram over every consecutive solution pair.

    Collision is not included. An empty layer has no pairs at the
    missing waypoint; the caller reads ``prepared.reason`` for that.
    """
    counts = {"jump": 0, "excursion": 0, "branch": 0, "wrist": 0,
              "pass": 0}
    if prepared.reason:
        return counts
    layers = prepared.layers
    for layer in range(len(layers) - 1):
        _name, xyz_from = layers[layer]
        _next, xyz_to = layers[layer + 1]
        for joints in prepared.solutions[layer]:
            for nxt in prepared.solutions[layer + 1]:
                reason = step_reject(
                    joints, nxt, xyz_from, xyz_to,
                    wrist_margin=wrist_margin)
                counts[reason or "pass"] += 1
    return counts


@dataclass
class PreparedCorridor(object):
    """IK layers for one candidate, before collision."""

    layers: Tuple[Layer, ...]
    solutions: Tuple[Tuple[JointVector, ...], ...]
    release: int
    n_vertices: int
    reason: str
    candidate_id: str
    #: One IK-attempt record per solved layer, plus the empty layer when
    #: the corridor stops. Edge gates are not in these records.
    attempts: Tuple[dict, ...] = ()


class _RefineMeter(object):
    """Counts underlying refine calls; optionally memoizes exact repeats.

    The key is the exact (position, orientation, seed) floats. It is not
    rounded: a rounded key would hand IK a different input. An exception
    is not cached, so a retry behaves as the uncached call would.
    """

    def __init__(self, refine, cache, stats):
        self._refine = refine
        self._cache = {} if cache else None
        self._stats = stats

    def __call__(self, position, orientation, seed):
        key = None
        if self._cache is not None:
            key = (tuple(position), tuple(orientation),
                   tuple(float(v) for v in seed))
            if key in self._cache:
                self._stats["cache_hits"] += 1
                return self._cache[key]
        self._stats["refine_calls"] += 1
        value = self._refine(position, orientation, seed)
        if key is not None:
            self._cache[key] = value
        return value


def prepare_corridor(request: CorridorRequest, refine: Callable,
                     step_m=LINE_STEP_M, cache_ik=False,
                     stats=None) -> PreparedCorridor:
    """Solve every waypoint. No collision calls.

    ``stats`` (a dict) receives ``refine_calls``, ``cache_hits``, and
    ``ik_s``. ``cache_ik`` memoizes exact repeats only.
    """
    meter_stats = {"refine_calls": 0, "cache_hits": 0}
    refine = _RefineMeter(refine, cache_ik, meter_stats)
    started = time.perf_counter()
    try:
        return _prepare(request, refine, step_m)
    finally:
        if stats is not None:
            stats.update(meter_stats)
            stats["ik_s"] = time.perf_counter() - started


def _prepare(request, refine, step_m):
    layers, release = corridor_layers(request.anchors, step_m)
    orientation = tuple(float(v) for v in request.orientation)
    limits = tuple(
        (float(lo), float(hi)) for lo, hi in request.joint_limits)
    fixed = branch_seeds(request.start_q, request.atlas_seeds)
    solutions: List[Tuple[JointVector, ...]] = []
    previous: Tuple[JointVector, ...] = ()
    vertices = 0
    candidate_id = str(request.candidate_id or "")
    attempts = []
    for name, position in layers:
        seeds = _layer_seeds(fixed, previous)
        attempt = {}
        found = _solutions_at(
            position, orientation, seeds, refine, limits, attempt=attempt)
        attempt["layer"] = name
        family, retreat = layer_family(name)
        attempt["layer_family"] = family
        attempt["retreat"] = retreat
        attempts.append(attempt)
        if not found:
            return PreparedCorridor(
                layers=tuple(layers[:len(solutions) + 1]),
                solutions=tuple(solutions),
                release=release, n_vertices=vertices,
                reason="LATTICE_LAYER_EMPTY:%s" % name,
                candidate_id=candidate_id,
                attempts=tuple(attempts))
        solutions.append(found)
        vertices += len(found)
        previous = found
    return PreparedCorridor(
        layers=layers, solutions=tuple(solutions), release=release,
        n_vertices=vertices, reason="", candidate_id=candidate_id,
        attempts=tuple(attempts))


def kinematic_queries(prepared: PreparedCorridor):
    """Arrival states the search will collision-check, in walk order.

    The joint tuples are the layer objects, so a later lookup by those
    same tuples hits. One entry per distinct (configuration, load).
    """
    if prepared.reason:
        return ()
    seen = set()
    rows = []
    layers = prepared.layers
    for layer in range(len(layers) - 1):
        _name, xyz_from = layers[layer]
        _next, xyz_to = layers[layer + 1]
        load = "carrying" if layer < prepared.release else "unloaded"
        for joints in prepared.solutions[layer]:
            for nxt in prepared.solutions[layer + 1]:
                if step_reject(joints, nxt, xyz_from, xyz_to):
                    continue
                key = (nxt, load)
                if key in seen:
                    continue
                seen.add(key)
                rows.append(key)
    return tuple(rows)


def search_prepared(prepared: PreparedCorridor, collision: Callable,
                    budget_s=ROUTE_TOTAL_WALL_S, started=None,
                    wrist_margin=None) -> LatticeResult:
    """Walk a prepared layer graph. ``started`` is a perf_counter value."""
    if collision is None:
        raise ValueError("corridor collision check is required")
    if started is None:
        started = time.perf_counter()
    if prepared.reason:
        return _finish(
            LatticeResult(
                ok=False, reason=prepared.reason,
                n_vertices=prepared.n_vertices,
                candidate_id=prepared.candidate_id),
            started, budget_s)
    chain, loads, edges, calls, reason = _search(
        prepared.layers, prepared.solutions, prepared.release, collision,
        wrist_margin=wrist_margin)
    max_step = 0.0
    if chain:
        max_step = max(
            max(abs(_wrap_rad(float(b) - float(a)))
                for a, b in zip(left, right))
            for left, right in zip(chain, chain[1:]))
    return _finish(
        LatticeResult(
            ok=bool(chain) and reason == "",
            reason=reason,
            chain=chain,
            layer_names=(tuple(name for name, _point in prepared.layers)
                         if chain else ()),
            edge_loads=loads,
            n_vertices=prepared.n_vertices,
            n_edges=edges,
            n_collision_calls=calls,
            max_step_rad=max_step,
            candidate_id=prepared.candidate_id),
        started, budget_s)


@dataclass(frozen=True)
class KinematicGraph(object):
    """Edges that pass every kinematic gate, before collision.

    ``edges[layer]`` holds ``(index, follow, step_cost)`` in the same
    order ``_search`` visits them, so the lazy walk updates the same
    nodes in the same order.
    """

    edges: Tuple[Tuple[Tuple[int, int, float], ...], ...]
    loads: Tuple[str, ...]
    n_edges: int


def kinematic_graph(prepared: PreparedCorridor,
                    wrist_margin=None) -> KinematicGraph:
    layers = prepared.layers
    rows = []
    loads = []
    total = 0
    for layer in range(len(layers) - 1):
        _name, xyz_from = layers[layer]
        _next, xyz_to = layers[layer + 1]
        loads.append("carrying" if layer < prepared.release else "unloaded")
        edges = []
        for index, joints in enumerate(prepared.solutions[layer]):
            for follow, nxt in enumerate(prepared.solutions[layer + 1]):
                if step_reject(joints, nxt, xyz_from, xyz_to,
                               wrist_margin=wrist_margin):
                    continue
                step = sum(
                    abs(_wrap_rad(float(b) - float(a)))
                    for a, b in zip(joints, nxt))
                edges.append((index, follow, step))
        total += len(edges)
        rows.append(tuple(edges))
    return KinematicGraph(
        edges=tuple(rows), loads=tuple(loads), n_edges=total)


def _optimistic_path(prepared, graph, blocked):
    """``_search``'s DP with unknown states treated as clear."""
    best = {
        (0, index): (0.0, (index,))
        for index in range(len(prepared.solutions[0]))
    }
    for layer, edges in enumerate(graph.edges):
        load = graph.loads[layer]
        following = prepared.solutions[layer + 1]
        for index, follow, step in edges:
            slot = best.get((layer, index))
            if slot is None:
                continue
            if (following[follow], load) in blocked:
                continue
            cost, path = slot
            new_cost = cost + step
            new_path = path + (follow,)
            previous = best.get((layer + 1, follow))
            if previous is None or new_cost < previous[0] - 1e-12 \
                    or (abs(new_cost - previous[0]) <= 1e-12
                        and new_path < previous[1]):
                best[(layer + 1, follow)] = (new_cost, new_path)
    last = len(prepared.layers) - 1
    winners = [
        best[(last, index)] for index in range(len(prepared.solutions[last]))
        if (last, index) in best]
    if not winners:
        return None
    return min(winners, key=lambda item: (item[0], item[1]))[1]


def kinematic_chain_joints(prepared, wrist_margin=None):
    """Joint chain that passes every kinematic gate, or None.

    Collision is treated as clear. The chain is the same optimistic
    path ``search_lazy`` starts from.
    """
    if prepared.reason or not prepared.solutions:
        return None
    graph = kinematic_graph(prepared, wrist_margin=wrist_margin)
    path = _optimistic_path(prepared, graph, set())
    if path is None:
        return None
    return tuple(
        prepared.solutions[layer][index]
        for layer, index in enumerate(path))


def search_lazy(prepared: PreparedCorridor, validate: Callable,
                budget_s=ROUTE_TOTAL_WALL_S, started=None, stats=None,
                wrist_margin=None):
    """Lazy collision search. Returns ``(LatticeResult, verdicts)``.

    ``validate(states)`` takes ``[(joints, load), ...]`` and returns a
    dict with a bool for every state it was given. Only states on the
    current cheapest optimistic chain are sent. A chain whose states
    all pass is the result; a failed state is blocked and the DP runs
    again. The returned chain equals ``search_prepared`` on the full
    mask: the optimistic graph contains the true graph, the costs and
    the tie-break are the same.
    """
    if validate is None:
        raise ValueError("corridor collision check is required")
    if started is None:
        started = time.perf_counter()
    stats = stats if stats is not None else {}
    verdicts = {}
    if prepared.reason:
        stats.update(graph_build_s=0.0, search_s=0.0, collision_s=0.0,
                     lazy_iterations=0, states_checked=0)
        return _finish(
            LatticeResult(
                ok=False, reason=prepared.reason,
                n_vertices=prepared.n_vertices,
                candidate_id=prepared.candidate_id),
            started, budget_s), verdicts
    mark = time.perf_counter()
    graph = kinematic_graph(prepared, wrist_margin=wrist_margin)
    stats["graph_build_s"] = time.perf_counter() - mark
    blocked = set()
    search_s = 0.0
    collision_s = 0.0
    iterations = 0
    path = None
    while True:
        iterations += 1
        mark = time.perf_counter()
        path = _optimistic_path(prepared, graph, blocked)
        search_s += time.perf_counter() - mark
        if path is None:
            break
        states = [
            (prepared.solutions[layer + 1][follow], graph.loads[layer])
            for layer, follow in enumerate(path[1:])]
        unknown = []
        for state in states:
            if state not in verdicts and state not in unknown:
                unknown.append(state)
        if not unknown:
            break
        mark = time.perf_counter()
        answered = validate(unknown)
        collision_s += time.perf_counter() - mark
        for state in unknown:
            clear = bool(answered.get(state, False))
            verdicts[state] = clear
            if not clear:
                blocked.add(state)
        if all(verdicts[state] for state in states):
            break
    stats.update(search_s=search_s, collision_s=collision_s,
                 lazy_iterations=iterations, states_checked=len(verdicts))
    if path is None:
        return _finish(
            LatticeResult(
                ok=False, reason="LATTICE_NO_CHAIN",
                n_vertices=prepared.n_vertices,
                n_edges=graph.n_edges,
                n_collision_calls=len(verdicts),
                candidate_id=prepared.candidate_id),
            started, budget_s), verdicts
    chain = tuple(prepared.solutions[layer][index]
                  for layer, index in enumerate(path))
    max_step = max(
        max(abs(_wrap_rad(float(b) - float(a)))
            for a, b in zip(left, right))
        for left, right in zip(chain, chain[1:]))
    return _finish(
        LatticeResult(
            ok=True, reason="", chain=chain,
            layer_names=tuple(name for name, _point in prepared.layers),
            edge_loads=graph.loads,
            n_vertices=prepared.n_vertices,
            n_edges=graph.n_edges,
            n_collision_calls=len(verdicts),
            max_step_rad=max_step,
            candidate_id=prepared.candidate_id),
        started, budget_s), verdicts


def solve_corridor(request: CorridorRequest, refine: Callable,
                   collision: Callable, budget_s=ROUTE_TOTAL_WALL_S,
                   step_m=LINE_STEP_M, wrist_margin=None) -> LatticeResult:
    """Search one candidate. ``collision(q, load_state)`` must be set.

    ``load_state`` is ``"carrying"`` through the slot and ``"unloaded"``
    after it. The clock covers IK, edge checks, and the layer walk.
    """
    if collision is None:
        raise ValueError("corridor collision check is required")
    started = time.perf_counter()
    prepared = prepare_corridor(request, refine, step_m)
    return search_prepared(
        prepared, collision, budget_s, started, wrist_margin=wrist_margin)


def search_candidates(requests: Sequence[CorridorRequest], refine: Callable,
                      collision: Callable, budget_s=ROUTE_TOTAL_WALL_S,
                      step_m=LINE_STEP_M):
    """Try candidates in order. A miss does not stop the list.

    The first complete chain is returned. Later candidates are not
    searched after that, matching the route producer. The row list
    includes the miss that caused the next candidate to run.
    """
    rows = []
    winner = None
    for index, request in enumerate(requests):
        result = solve_corridor(
            request, refine, collision, budget_s=budget_s, step_m=step_m)
        result = LatticeResult(
            **{**result.__dict__, "candidate_index": index})
        rows.append(result)
        if result.ok and winner is None:
            winner = result
            break
    return winner, tuple(rows)


def _layer_seeds(fixed, previous):
    rows = [
        ("continue_%d" % index, tuple(float(v) for v in joints))
        for index, joints in enumerate(previous or ())]
    rows.extend(fixed)
    seen = set()
    ordered = []
    for label, joints in rows:
        key = tuple(round(float(v), 6) for v in joints)
        if key in seen:
            continue
        seen.add(key)
        ordered.append((label, joints))
    return tuple(ordered)


def _solutions_at(position, orientation, seeds, refine, limits, attempt=None):
    found: List[JointVector] = []
    counts = {
        "seeds": 0,
        "raw_ik_ok": 0,
        "ik_fail": 0,
        "joint_limit_reject": 0,
        "branch_reject": 0,
    }
    for _label, seed in seeds:
        counts["seeds"] += 1
        try:
            solved, _info = refine(position, orientation, list(seed))
        except Exception:  # noqa: BLE001 - one seed must not abort the layer
            solved = None
        if solved is None:
            counts["ik_fail"] += 1
            continue
        joints = tuple(float(v) for v in solved)
        counts["raw_ik_ok"] += 1
        if len(joints) != 6 or not _inside(joints, limits):
            counts["joint_limit_reject"] += 1
            continue
        if any(joint_branch_distance(joints, prior) < BRANCH_DISTANCE_RAD
               for prior in found):
            counts["branch_reject"] += 1
            continue
        found.append(joints)
    found.sort(key=lambda joints: tuple(round(v, 6) for v in joints))
    if attempt is not None:
        attempt.update(counts)
        attempt["kept"] = len(found)
    return tuple(found)


def boundary_counts(prepared, wrist_margin=None):
    """Per-boundary node and kinematic-edge counts.

    Collision is not included. The first boundary with zero passing
    edges is the kinematic disconnect; an empty layer has no boundary
    into itself because that layer has no nodes.
    """
    rows = []
    solved = len(prepared.solutions)
    for layer in range(max(0, solved - 1)):
        name_from, xyz_from = prepared.layers[layer]
        name_to, xyz_to = prepared.layers[layer + 1]
        counts = {"jump": 0, "excursion": 0, "branch": 0, "wrist": 0,
                  "pass": 0}
        for joints in prepared.solutions[layer]:
            for nxt in prepared.solutions[layer + 1]:
                reason = step_reject(
                    joints, nxt, xyz_from, xyz_to,
                    wrist_margin=wrist_margin)
                counts[reason or "pass"] += 1
        family_from, _retreat_from = layer_family(name_from)
        family_to, retreat_to = layer_family(name_to)
        release = int(prepared.release)
        if layer + 1 < release:
            load = "carrying"
        elif layer + 1 == release:
            load = "carrying"
        else:
            load = "unloaded"
        rows.append({
            "from_layer": name_from,
            "to_layer": name_to,
            "from_family": family_from,
            "to_family": family_to,
            "retreat": retreat_to,
            "load": load,
            "nodes_from": len(prepared.solutions[layer]),
            "nodes_to": len(prepared.solutions[layer + 1]),
            "edges": counts,
        })
    disconnect = next(
        (row for row in rows if int(row["edges"]["pass"]) == 0), None)
    return rows, disconnect


def _search(layers, solutions, release, collision, wrist_margin=None):
    """Deterministic shortest joint-travel chain across the layers.

    Equal cost keeps the lexicographically smaller solution-index path,
    and solution indices are sorted joint order, so the winner does not
    depend on seed attempt order.
    """
    first = solutions[0]
    best = {
        (0, index): (0.0, (index,))
        for index in range(len(first))
    }
    accepted = 0
    calls = 0
    for layer in range(len(layers) - 1):
        _name, xyz_from = layers[layer]
        _next_name, xyz_to = layers[layer + 1]
        load = "carrying" if layer < release else "unloaded"
        for index, joints in enumerate(solutions[layer]):
            slot = best.get((layer, index))
            if slot is None:
                continue
            cost, path = slot
            for follow, nxt in enumerate(solutions[layer + 1]):
                if step_reject(joints, nxt, xyz_from, xyz_to,
                               wrist_margin=wrist_margin):
                    continue
                calls += 1
                try:
                    clear = bool(collision(nxt, load))
                except Exception:  # noqa: BLE001 - a check error is a block
                    clear = False
                if not clear:
                    continue
                accepted += 1
                step = sum(
                    abs(_wrap_rad(float(b) - float(a)))
                    for a, b in zip(joints, nxt))
                new_cost = cost + step
                new_path = path + (follow,)
                previous = best.get((layer + 1, follow))
                if previous is None or new_cost < previous[0] - 1e-12 \
                        or (abs(new_cost - previous[0]) <= 1e-12
                            and new_path < previous[1]):
                    best[(layer + 1, follow)] = (new_cost, new_path)
    last = len(layers) - 1
    winners = [
        best[(last, index)] for index in range(len(solutions[last]))
        if (last, index) in best]
    if not winners:
        return (), (), accepted, calls, "LATTICE_NO_CHAIN"
    _cost, path = min(winners, key=lambda item: (item[0], item[1]))
    chain = tuple(solutions[layer][index]
                  for layer, index in enumerate(path))
    loads = tuple(
        "carrying" if layer < release else "unloaded"
        for layer in range(len(layers) - 1))
    return chain, loads, accepted, calls, ""


def _wrist_ok(q_from, q_to, margin=WRIST_SINGULARITY_MARGIN_RAD,
              index=WRIST_JOINT_INDEX):
    track = (float(q_from[index]), float(q_to[index]))
    closest = min(
        abs(value - round(value / math.pi) * math.pi) for value in track)
    crosses = math.floor(track[0] / math.pi) != math.floor(track[1] / math.pi)
    return (not crosses) and closest >= float(margin)


def _inside(joints, limits):
    for value, (lower, upper) in zip(joints, limits):
        if value < lower - 1e-8 or value > upper + 1e-8:
            return False
    return True


def _distance(left, right):
    return math.sqrt(sum(
        (float(a) - float(b)) ** 2 for a, b in zip(left, right)))


def _finish(result: LatticeResult, started: float,
            budget_s: float) -> LatticeResult:
    elapsed = time.perf_counter() - started
    return LatticeResult(
        **{
            **result.__dict__,
            "wall_time_s": elapsed,
            "within_budget": elapsed <= float(budget_s),
        })
