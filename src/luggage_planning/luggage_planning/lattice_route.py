"""Corridor-lattice route planner for RouteProducer.

Same protocol as ``GraphRoutePlanner.plan_route``: it returns a
``PlannedRoute`` whose edges are in ``graph.edges``, so certification,
map revalidation and leg records run unchanged.

Route shape (regions are the producer's anchor plan):

* entry, carrying: home -> [staging ->] portal is MoveIt joint
  ``PORTAL_TRANSIT``; portal -> slot_approach ``TRAVERSE``,
  -> slot_upper ``INSERT``, -> slot ``DESCEND`` come from the lattice chain;
* retreat, unloaded: slot -> slot_upper -> slot_approach -> portal are
  lattice ``RETREAT`` edges; portal -> [staging ->] home is MoveIt joint
  ``RETURN``.

Lattice edges are the chain's own joint states retimed with the site
rest-to-rest profile, the same retime MoveIt Cartesian edges get. The
lattice gates (jump, excursion, branch, wrist) already ran on every step.
The certifier still checks the dense interpolated trajectory, so a
collision between two clear knots is still a refusal.

Failures raise ``RoutePlanningError`` with a named ``candidate_failure``
(``LATTICE_NO_CHAIN``, ``LATTICE_LAYER_EMPTY``, ``LATTICE_OVER_BUDGET``,
``LATTICE_COLLISION_SERVICE``), so the producer does not redraw it.
"""

from __future__ import division

import hashlib
import time
from typing import Callable, Dict, Mapping, Optional, Sequence

from luggage_planning.corridor_lattice import (
    ENTRY_ANCHORS,
    RETREAT_ANCHORS,
    CorridorRequest,
    gate_counts,
    prepare_corridor,
    search_lazy,
)

LATTICE_EDGE_IDENTITY = "corridor-lattice-v1"

#: (first layer, last layer, edge class, destination region, load).
LATTICE_SEGMENTS = (
    ("portal", "slot_approach", "TRAVERSE", "slot_approach", "carrying"),
    ("slot_approach", "slot_upper", "INSERT", "slot_upper", "carrying"),
    ("slot_upper", "slot", "DESCEND", "slot", "carrying"),
    ("slot", "slot_upper_retreat", "RETREAT", "slot_upper", "unloaded"),
    ("slot_upper_retreat", "slot_approach_retreat", "RETREAT",
     "slot_approach", "unloaded"),
    ("slot_approach_retreat", "portal_retreat", "RETREAT", "portal",
     "unloaded"),
)

#: One ValidateRouteV2 call answers every state in a group until the
#: first invalid one; the rest is re-sent. This bounds a pathological
#: group to one call per invalid state.
SERVICE_TIMEOUT_S = 5.0
SERVICE_BUDGET_S = 5.0
SAMPLE_SPACING_NS = 1.0e8
JOINTS = tuple("elfin_joint%d" % i for i in range(1, 7))


class LatticeCheckError(RuntimeError):
    """The collision service did not answer; the candidate fails closed."""


class LatticeStateChecker(object):
    """Collision verdict per ``(joints, load)`` through ValidateRouteV2.

    Scenes come from the certifier's ``FrozenSceneComposer`` and the
    producer's certification ``SceneState`` per load, so a lattice
    verdict and a certification verdict at the same state use the same
    scene. Each state is one sample of a one-knot leg; samples sit
    ``SAMPLE_SPACING_NS`` apart with no velocity limits, so only the
    state itself is judged. One distance query per call (stride 1e9).
    """

    def __init__(self, node, scene_client, scene_states: Mapping,
                 start_q, service_timeout_s=SERVICE_TIMEOUT_S,
                 distance_stride=10 ** 9):
        self._node = node
        self._scene_client = scene_client
        self._scene_states = dict(scene_states)
        self._start_q = tuple(float(v) for v in start_q)
        self._timeout_s = float(service_timeout_s)
        self._distance_stride = max(1, int(distance_stride))
        self._client = None
        self._scenes = None
        self._pending_live = None
        self.prepared_digest = None
        self._indices = None
        self._distances = []
        self.collision_rejected = 0
        self.collision_by_load = {"carrying": 0, "unloaded": 0}
        self.min_collision_distance_m = None
        self.first_block = None
        self.stats = {"service_calls": 0, "service_time_s": 0.0,
                      "states": 0}

    def live_scene_digest(self):
        """Digest of the live scene this checker will judge against.

        The scene read for the digest is kept and is the one ``_prepare``
        composes from, so digest and judged scene are the same object. A
        checker that already prepared against another scene reports
        ``prepared_digest`` so the caller can build a new one.
        """
        from luggage_planning.anchor_payload_gate import live_scene_digest
        from luggage_planning.moveit_state_validity import FrozenSceneComposer

        live = FrozenSceneComposer(self._scene_client).read_live()
        digest = live_scene_digest(live)
        if self._scenes is None:
            self._pending_live = live
            self.prepared_digest = digest
        return digest

    def _prepare(self):
        if self._scenes is not None:
            return
        from luggage_msgs.srv import ValidateRouteV2

        from luggage_planning.moveit_state_validity import (
            SERVICE_NAME,
            FrozenSceneComposer,
        )

        class _Leg(object):
            def __init__(self, edge_id):
                self.edge_id = edge_id

        composer = FrozenSceneComposer(self._scene_client)
        live = getattr(self, "_pending_live", None) or composer.read_live()
        legs = [_Leg("lattice-%s" % load) for load in self._scene_states]
        states = {
            "lattice-%s" % load: state
            for load, state in self._scene_states.items()}
        scenes, indices, _support = composer.compose(
            live, self._start_q, states, legs)
        self._scenes = scenes
        self._indices = {
            load: indices[FrozenSceneComposer._state_key(state)]  # noqa: SLF001
            for load, state in self._scene_states.items()}
        self._client = self._node.create_client(ValidateRouteV2, SERVICE_NAME)
        if not self._client.wait_for_service(timeout_sec=2.0):
            raise LatticeCheckError("%s not serving" % SERVICE_NAME)

    def _call(self, group, load):
        import threading

        from luggage_msgs.msg import RouteLegV2
        from luggage_msgs.srv import ValidateRouteV2
        from trajectory_msgs.msg import JointTrajectoryPoint

        leg = RouteLegV2()
        leg.edge_id = "lattice-%s-%d" % (load, len(group))
        leg.edge_class = "TRAVERSE" if load == "carrying" else "RETREAT"
        leg.carrying = load == "carrying"
        leg.load_state_index = int(self._indices[load])
        leg.joint_names = list(JOINTS)
        leg.n_samples = len(group)
        leg.sample_times_ns = [
            float(i) * SAMPLE_SPACING_NS for i in range(len(group))]
        leg.sample_positions_flat = [
            float(v) for joints in group for v in joints]
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in group[0]]
        leg.knots.joint_names = list(JOINTS)
        leg.knots.points = [point]
        request = ValidateRouteV2.Request()
        request.request_id = "corridor-lattice"
        request.route_digest = "corridor-lattice"
        request.scene_digest = "corridor-lattice"
        request.load_state_scenes = list(self._scenes)
        request.max_sample_dt_ns = 0.0
        request.max_sample_step_m = 0.0
        request.distance_stride = int(self._distance_stride)
        request.budget_s = SERVICE_BUDGET_S
        request.legs = [leg]
        event = threading.Event()
        future = self._client.call_async(request)
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(self._timeout_s):
            raise LatticeCheckError("validate_route_v2 timeout")
        response = future.result()
        self.stats["service_calls"] += 1
        if response is None or int(response.result) != 0:
            raise LatticeCheckError("validate_route_v2 rejected: %s" % (
                "" if response is None else response.detail))
        self.stats["service_time_s"] += \
            float(response.service_time_ns) / 1e9
        leg = response.leg_results[0]
        for value in getattr(leg, "min_distance_m", ()) or ():
            number = float(value)
            if number == number:  # not NaN
                self._distances.append(number)
        if not leg.valid and str(getattr(leg, "reason", "") or "") == "COLLISION":
            self._distances.append(0.0)
            self.collision_rejected += 1
            self.collision_by_load[load] = (
                self.collision_by_load.get(load, 0) + 1)
            if self.first_block is None:
                contacts = []
                seen = set()
                for contact in getattr(leg, "contacts", ()) or ():
                    pair = (str(contact.body_1), str(contact.body_2))
                    if pair in seen:
                        continue
                    seen.add(pair)
                    contacts.append({
                        "body_1": pair[0],
                        "body_2": pair[1],
                        "depth_m": round(float(contact.depth), 6),
                    })
                    if len(contacts) >= 8:
                        break
                index = int(getattr(leg, "first_invalid_sample", 0) or 0)
                joints = None
                if 0 <= index < len(group):
                    joints = [round(float(v), 6) for v in group[index]]
                self.first_block = {
                    "reason": "COLLISION",
                    "load": load,
                    "joints": joints,
                    "contacts": contacts,
                }
        if self._distances:
            self.min_collision_distance_m = min(self._distances)
        return leg

    def validate(self, states):
        """``[(joints, load), ...]`` -> ``{(joints, load): bool}``."""
        self._prepare()
        verdicts = {}
        for load in ("carrying", "unloaded"):
            pending = [[joints for joints, state_load in states
                        if state_load == load]]
            while pending and pending[-1]:
                group = pending.pop()
                result = self._call(group, load)
                if result.valid:
                    for joints in group:
                        verdicts[(joints, load)] = True
                    continue
                index = int(result.first_invalid_sample)
                if not 0 <= index < len(group):
                    raise LatticeCheckError(
                        "invalid leg without a sample: %s" % result.reason)
                for joints in group[:index]:
                    verdicts[(joints, load)] = True
                verdicts[(group[index], load)] = False
                if index + 1 < len(group):
                    pending.append(group[index + 1:])
        self.stats["states"] += len(verdicts)
        return verdicts


def _chain_sigma_min(fk, chain):
    """Minimum singular value of the geometric Jacobian along ``chain``.

    Empty input returns None. At most eight states are sampled, including
    the ends, so a long lattice chain does not dominate the route budget.
    This is a diagnostic. It is not a refuse gate.
    """
    rows = [tuple(float(v) for v in joints) for joints in (chain or ())]
    if not rows:
        return None
    if len(rows) > 8:
        last = len(rows) - 1
        picked = []
        for index in range(8):
            slot = int(round(index * last / 7.0))
            if not picked or picked[-1] != slot:
                picked.append(slot)
        rows = [rows[index] for index in picked]
    values = [_tcp_sigma_min(fk, joints) for joints in rows]
    values = [value for value in values if value is not None]
    if not values:
        return None
    return round(min(values), 6)


def _tcp_sigma_min(fk, joints, delta=1e-6):
    import numpy as np
    from scipy.spatial.transform import Rotation

    base = [float(v) for v in joints]
    try:
        pos0, quat0 = fk(base)
    except Exception:  # noqa: BLE001 - a diagnostic must not fail the route
        return None
    rot0 = Rotation.from_quat([float(v) for v in quat0])
    jacobian = np.zeros((6, 6))
    origin = np.asarray(pos0, dtype=float)
    for index in range(6):
        bumped = list(base)
        bumped[index] += float(delta)
        try:
            pos, quat = fk(bumped)
        except Exception:  # noqa: BLE001
            return None
        jacobian[:3, index] = (np.asarray(pos, dtype=float) - origin) / delta
        rot = Rotation.from_quat([float(v) for v in quat])
        jacobian[3:, index] = (rot0.inv() * rot).as_rotvec() / float(delta)
    singular = np.linalg.svd(jacobian, compute_uv=False)
    return float(singular.min())


def _digest8(text):
    return hashlib.sha256(str(text).encode("utf-8")).hexdigest()[:8]


def retime_positions(positions, velocity_scaling, acceleration_scaling):
    """Site rest-to-rest profile over joint states -> TimedTrajectory."""
    from st09_contracts.graph import TimedTrajectory

    from luggage_planning.trajectory_timing import (
        time_parameterize_cartesian,
    )

    rows = [[float(v) for v in joints] for joints in positions]
    if len(rows) < 2:
        raise ValueError("lattice edge needs at least two states")
    timed = time_parameterize_cartesian(
        rows, velocity_scaling, acceleration_scaling)
    times_ns = []
    for value in timed.times_s:
        stamp = int(round(float(value) * 1e9))
        if times_ns and stamp <= times_ns[-1]:
            stamp = times_ns[-1] + 1
        times_ns.append(stamp)
    return TimedTrajectory(
        joint_names=JOINTS,
        positions=tuple(tuple(row) for row in rows),
        velocities=tuple(tuple(float(v) for v in row)
                         for row in timed.velocities_rad),
        accelerations=tuple(tuple(float(v) for v in row)
                            for row in timed.accelerations_rad),
        time_from_start_ns=tuple(times_ns))


def chain_segments(layer_names, chain):
    """Split the chain at the anchor layers named in LATTICE_SEGMENTS.

    Returns ``[(edge_class, region, load, states, names)]``; a segment
    whose states do not move (coincident anchors) is dropped.
    """
    index_of = {name: index for index, name in enumerate(layer_names)}
    missing = [name for name in ENTRY_ANCHORS if name not in index_of]
    missing += ["%s_retreat" % name for name in RETREAT_ANCHORS
                if "%s_retreat" % name not in index_of]
    if missing:
        raise ValueError("lattice chain lacks layers %s" % missing)
    segments = []
    for first, last, edge_class, region, load in LATTICE_SEGMENTS:
        start, stop = index_of[first], index_of[last]
        states = tuple(chain[start:stop + 1])
        moving = any(
            abs(float(b) - float(a)) > 1e-12
            for left, right in zip(states, states[1:])
            for a, b in zip(left, right))
        if len(states) < 2 or not moving:
            continue
        segments.append((edge_class, region, load, states,
                         tuple(layer_names[start:stop + 1])))
    return segments


class LatticeRoutePlanner(object):
    """One candidate's lattice route. See the module docstring."""

    def __init__(self, graph, edge_builder, *, home_q, anchors: Mapping,
                 orientation, joint_limits, refine: Callable,
                 fk: Callable, checker_factory: Callable,
                 check_scene_states: Mapping, build_scene_states: Mapping,
                 t_frame_container, q_frame_container,
                 staging_q: Sequence = (), staging_unloaded_q: Sequence = (),
                 atlas_seeds: Sequence = (), candidate_id="",
                 velocity_scaling=None, acceleration_scaling=None,
                 clock_s: Optional[Callable] = None, log=None,
                 wrist_margin=None, anchor_sets=None):
        from luggage_planning.moveit_edge_adapter import (
            MOVEIT_ACCELERATION_SCALING,
            MOVEIT_VELOCITY_SCALING,
        )

        self.graph = graph
        self.edge_builder = edge_builder
        self.home_q = tuple(float(v) for v in home_q)
        self.anchors = {name: tuple(float(v) for v in point)
                        for name, point in anchors.items()}
        self.orientation = tuple(float(v) for v in orientation)
        self.joint_limits = tuple(
            (float(lo), float(hi)) for lo, hi in joint_limits)
        self.refine = refine
        self.fk = fk
        self.checker_factory = checker_factory
        self.check_scene_states = dict(check_scene_states)
        self.build_scene_states = dict(build_scene_states)
        self.t_frame_container = tuple(float(v) for v in t_frame_container)
        self.q_frame_container = tuple(float(v) for v in q_frame_container)
        self.staging_q = [tuple(float(v) for v in q) for q in staging_q]
        self.staging_unloaded_q = [
            tuple(float(v) for v in q) for q in staging_unloaded_q]
        self.atlas_seeds = tuple(
            tuple(float(v) for v in seed) for seed in atlas_seeds)
        self.candidate_id = str(candidate_id)
        self.velocity_scaling = float(
            MOVEIT_VELOCITY_SCALING if velocity_scaling is None
            else velocity_scaling)
        self.acceleration_scaling = float(
            MOVEIT_ACCELERATION_SCALING if acceleration_scaling is None
            else acceleration_scaling)
        self._clock_s = clock_s or time.perf_counter
        self._log = log or (lambda message: None)
        self.wrist_margin = wrist_margin
        self.anchor_sets = tuple(anchor_sets or ())
        self.trace = None
        self.last_result = None
        self.last_stats: Dict[str, object] = {}
        self.last_budget_snapshot = None

    # -- lattice -----------------------------------------------------------

    def _lattice(self, budget_s, started):
        from luggage_planning.grid_path_search import RoutePlanningError

        if self.last_result is not None:
            return self.last_result
        sets = self.anchor_sets or (("baseline", self.anchors),)
        stats = self.last_stats
        stats["wrist_margin_rad"] = (
            None if self.wrist_margin is None else float(self.wrist_margin))
        attempts = []
        checker = None
        failures = []
        result = None

        def validate(states):
            try:
                return checker.validate(states)
            except LatticeCheckError as exc:
                failures.append(str(exc))
                return {}

        for label, anchors in sets:
            if self._clock_s() - started > budget_s:
                break
            self.anchors = {
                name: tuple(float(v) for v in point)
                for name, point in anchors.items()}
            request = CorridorRequest(
                anchors=self.anchors, orientation=self.orientation,
                start_q=self.home_q, joint_limits=self.joint_limits,
                atlas_seeds=self.atlas_seeds, candidate_id=self.candidate_id)
            variant_stats = {}
            prepared = prepare_corridor(
                request, self.refine, cache_ik=True, stats=variant_stats)
            if checker is None and not prepared.reason:
                checker = self.checker_factory(
                    scene_states=self.check_scene_states, start_q=self.home_q)
            counts = gate_counts(prepared, wrist_margin=self.wrist_margin)
            if prepared.reason:
                from luggage_planning.corridor_lattice import LatticeResult
                elapsed = self._clock_s() - started
                result = LatticeResult(
                    ok=False, reason=prepared.reason,
                    n_vertices=prepared.n_vertices,
                    candidate_id=prepared.candidate_id,
                    wall_time_s=elapsed,
                    within_budget=elapsed <= float(budget_s))
            else:
                result, _verdicts = search_lazy(
                    prepared, validate, budget_s, started, variant_stats,
                    wrist_margin=self.wrist_margin)
            sigma = _chain_sigma_min(self.fk, result.chain)
            attempts.append({
                "variant": str(label),
                "reason": result.reason or "chain",
                "n_vertices": result.n_vertices,
                "n_collision_calls": result.n_collision_calls,
                "gate_counts": counts,
                "ik_s": round(float(variant_stats.get("ik_s") or 0.0), 3),
                "min_sigma": sigma,
                "wall_s": round(result.wall_time_s, 3),
            })
            self._log("lattice: %s %s variant=%s vertices=%d checked=%d"
                      " wall=%.2fs" % (
                          self.candidate_id, result.reason or "chain",
                          label, result.n_vertices, result.n_collision_calls,
                          result.wall_time_s))
            if failures:
                break
            if result.ok and result.within_budget:
                break
        stats["request"] = {
            "anchors": {name: [round(v, 9) for v in point]
                        for name, point in self.anchors.items()},
            "orientation": [round(v, 9) for v in self.orientation],
            "start_q": [round(v, 9) for v in self.home_q],
            "atlas_seeds": [[round(v, 9) for v in seed]
                            for seed in self.atlas_seeds],
            "anchor_attempts": attempts,
        }
        stats["anchor_attempts"] = attempts
        if checker is not None:
            stats.update(getattr(checker, "stats", {}) or {})
            stats["min_collision_distance_m"] = getattr(
                checker, "min_collision_distance_m", None)
            stats["collision_rejected"] = int(getattr(
                checker, "collision_rejected", 0))
            stats["first_block"] = getattr(checker, "first_block", None)
            stats["collision_by_load"] = dict(getattr(
                checker, "collision_by_load", {}) or {})
        if result is None:
            from luggage_planning.corridor_lattice import LatticeResult
            result = LatticeResult(
                ok=False, reason="LATTICE_OVER_BUDGET",
                candidate_id=self.candidate_id, within_budget=False)
        sigmas = [row["min_sigma"] for row in attempts
                  if row.get("min_sigma") is not None]
        stats.update(lattice_reason=result.reason,
                     lattice_wall_s=round(result.wall_time_s, 4),
                     n_vertices=result.n_vertices,
                     n_states_checked=result.n_collision_calls,
                     min_sigma=(min(sigmas) if sigmas else None))
        if failures:
            raise RoutePlanningError("EDGE_BUILD_FAILED", {
                "candidate_failure": "LATTICE_COLLISION_SERVICE",
                "error": failures[0][:300], "lattice": dict(stats)})
        if not result.ok:
            name = result.reason.split(":", 1)[0]
            raise RoutePlanningError("NO_FEASIBLE_ROUTE", {
                "candidate_failure": name,
                "lattice_reason": result.reason,
                "lattice": dict(stats)})
        if not result.within_budget:
            raise RoutePlanningError("PLANNING_BUDGET_EXHAUSTED", {
                "candidate_failure": "LATTICE_OVER_BUDGET",
                "lattice": dict(stats)})
        self.last_result = result
        return result

    # -- graph -------------------------------------------------------------

    def _node(self, q, region, load_state, family):
        position, quaternion = self.fk(q)
        return self.graph.register_node(
            q, position, quaternion, region, load_state, family)

    def _add(self, edge):
        existing = self.graph.edges.get(edge.edge_id)
        if existing is not None:
            return existing
        return self.graph.add_edge(edge)

    def _lattice_edge(self, source, destination, edge_class, states, names):
        from st09_contracts.graph import GraphEdge

        trajectory = retime_positions(
            states, self.velocity_scaling, self.acceleration_scaling)
        travel = sum(
            abs(float(b) - float(a))
            for left, right in zip(states, states[1:])
            for a, b in zip(left, right))
        attributes = {
            "edge_class": edge_class,
            "planner": "corridor_lattice",
            "duration_ns": int(trajectory.time_from_start_ns[-1]),
            "joint_travel_rad": round(travel, 6),
            "fraction": 1.0,
            "waypoints": [],
            "lattice_layers": [names[0], names[-1]],
            "lattice_states": len(states),
            "reprobe": False,
        }
        return self._add(GraphEdge(
            edge_id="e-%s-lattice-%s-%s" % (
                edge_class.lower(),
                _digest8(source.node_id + destination.node_id),
                _digest8(trajectory.digest())),
            source_node_id=source.node_id,
            destination_node_id=destination.node_id,
            load_state=source.load_state,
            static_identity=LATTICE_EDGE_IDENTITY,
            trajectory=trajectory,
            attributes=attributes))

    def _joint_chain(self, waypoints, edge_class, scene, deadline):
        """MoveIt joint edges through ``waypoints`` (nodes, in order)."""
        from luggage_planning.grid_path_search import RoutePlanningError

        edges = []
        for source, destination in zip(waypoints, waypoints[1:]):
            if self._clock_s() > deadline:
                raise RoutePlanningError("PLANNING_BUDGET_EXHAUSTED", {
                    "stage": "lattice_joint_%s" % edge_class.lower()})
            edges.append(self._add(self.edge_builder(
                source, destination, edge_class, scene=scene)))
        return edges

    def _with_staging(self, ends, staging, edge_class, scene, deadline,
                      load_state):
        """Joint edges between ``ends``, via a staging node when one exists.

        Each staging solution is tried in order; the first that builds
        both joint edges is used. Without staging the edge is direct.
        """
        from luggage_planning.grid_path_search import RoutePlanningError

        if not staging:
            return self._joint_chain(list(ends), edge_class, scene, deadline)
        last = None
        for q in staging:
            middle = self._node(q, "staging", load_state, "anchor")
            try:
                return self._joint_chain(
                    [ends[0], middle, ends[1]], edge_class, scene, deadline)
            except RoutePlanningError as exc:
                if exc.code == "PLANNING_BUDGET_EXHAUSTED":
                    raise
                last = exc
        raise last

    # -- protocol ----------------------------------------------------------

    def plan_route(self, snapshot, attachment, candidate, targets,
                   budget_ns):
        from st09_contracts._base import digest_of
        from st09_contracts.graph import LoadState, PlannedRoute

        from luggage_planning.grid_path_search import RoutePlanningError
        from luggage_planning.reachable_graph import GraphRoutePlanner

        started = self._clock_s()
        budget_s = max(0.0, float(budget_ns) * 1e-9)
        if budget_s <= 0.0:
            raise RoutePlanningError("PLANNING_BUDGET_EXHAUSTED", {
                "budget_ns": float(budget_ns)})
        deadline = started + budget_s
        result = self._lattice(budget_s, started)
        loads = {"carrying": LoadState.CARRYING,
                 "unloaded": LoadState.UNLOADED}
        segments = chain_segments(result.layer_names, result.chain)
        entry_lattice, retreat_lattice = [], []
        first = {}
        last = {}
        for edge_class, region, load, states, names in segments:
            load_state = loads[load]
            source = self._node(
                states[0], names[0].replace("_retreat", ""), load_state,
                "lattice")
            destination = self._node(
                states[-1], region, load_state, "lattice")
            edge = self._lattice_edge(
                source, destination, edge_class, states, names)
            first.setdefault(load, source)
            last[load] = destination
            (entry_lattice if load == "carrying"
             else retreat_lattice).append(edge)
        if "carrying" not in first or "unloaded" not in first:
            raise RoutePlanningError("NO_FEASIBLE_ROUTE", {
                "candidate_failure": "LATTICE_NO_CHAIN",
                "detail": "lattice chain has no carrying or no retreat motion"})
        carrying_scene = self.build_scene_states["carrying"]
        unloaded_scene = self.build_scene_states["unloaded"]
        home_carrying = self._node(
            self.home_q, "home", LoadState.CARRYING, "start")
        home_unloaded = self._node(
            self.home_q, "home", LoadState.UNLOADED, "start")
        transit = self._with_staging(
            (home_carrying, first["carrying"]), self.staging_q,
            "PORTAL_TRANSIT", carrying_scene, deadline,
            LoadState.CARRYING)
        returning = self._with_staging(
            (last["unloaded"], home_unloaded), self.staging_unloaded_q,
            "RETURN", unloaded_scene, deadline,
            LoadState.UNLOADED)
        entry = tuple(transit) + tuple(entry_lattice)
        retreat = tuple(retreat_lattice) + tuple(returning)
        edges = entry + retreat
        payload = GraphRoutePlanner._payload_identity(None, attachment)
        node_path = tuple(edge.source_node_id for edge in edges) + (
            edges[-1].destination_node_id,)
        self.last_budget_snapshot = {
            "planner": "corridor_lattice",
            "budget_s": round(budget_s, 3),
            "elapsed_s": round(self._clock_s() - started, 3),
            "lattice": dict(self.last_stats),
        }
        return PlannedRoute(
            route_id="route-lattice-%s" % digest_of(
                attachment.request_id, candidate.candidate_id,
                node_path)[:12],
            request_id=attachment.request_id,
            snapshot_digest=snapshot.digest(),
            payload_digest=payload.digest(),
            attachment_digest=attachment.digest(),
            candidate_id=candidate.candidate_id,
            entry_edge_ids=tuple(edge.edge_id for edge in entry),
            retreat_edge_ids=tuple(edge.edge_id for edge in retreat),
            edge_digests=tuple(edge.digest() for edge in edges),
            trajectory_digest=digest_of(
                *(edge.trajectory.digest() for edge in edges)),
            planning_revision="L:%s" % self.graph.digest()[:12])
