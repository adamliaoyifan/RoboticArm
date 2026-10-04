#!/usr/bin/env python3
"""Offline scenario matrix for state-conditioned place-route search.

Runs ``RouteSearch`` on the real offline arm model (``offline_ik``
refine + FK, deploy scene) for a fixed matrix of payloads, attachment
offsets, start joints, occupancy, headings, slots and atlas on/off. Two
planners are compared on identical inputs:

- ``baseline``: the 08d25cf search (pair-rank ordering only; no frontier
  continuation, no branch ladders, no failure-class cap);
- ``state``: frontier continuation + branch ladders + failure-class cap.

Edges are built offline, not by MoveIt:

- cartesian classes: a straight TCP line in ``LINE_STEP_M`` steps, each
  refined from the previous step's q (continuous); any IK miss or a
  per-step joint jump above ``LINE_MAX_JUMP_RAD`` fails the edge;
- joint classes (PORTAL_TRANSIT, RETURN): linear joint interpolation
  standing in for OMPL.

Collision is a proxy: the carried payload (oriented box from FK and the
attachment transform) against the seven-face hull through the -X aperture
and the placed boxes. Arm links are not checked here; that remains
MoveIt certification's job. Timing is offline wall time, not MoveIt time.

Case classes:

- ``feasible``: a reference search (60 s budget, no cap, extra seeds)
  found a full route;
- ``invalid``: constructed so that any route must collide or the goal
  cannot exist (overlap with a placed box, box below the floor, box
  through the +Y wall);
- ``undetermined``: neither; reported, never scored.

The dev/held-out split is a sha1 of the case id, frozen in
``split.json`` before any planner runs.

``MATRIX_LINEAGE`` names the planner commits a new run is descended from.
The historical ``route_matrix_20261004`` manifest stays at its original
``e5c9a87`` revision; do not rewrite it as a later commit's result.
"""
import argparse
import hashlib
import json
import math
import os
import random
import subprocess
import sys
import time

import numpy as np
import yaml

from luggage_description.container_geometry import (
    contains_point_through_aperture,
    descriptor_from_scene_config,
    oriented_box_corners,
)
from luggage_planning import grid_path_search as gps
from luggage_planning.grid_path_search import (
    DEFAULT_RETREAT_HOPS,
    RouteBudget,
    RouteBudgetPolicy,
    RouteHop,
    RoutePlanningError,
    RouteSearch,
    descend_pair_rank,
    hops_from_table,
)
from luggage_planning.offline_ik import load_urdf, refine_to_pose
from luggage_planning.reachability_atlas import ReachabilityAtlas
from luggage_planning.reachable_graph import (
    GraphRoutePlanner,
    RouteGraph,
    make_offline_fk,
)
from luggage_planning.route_producer import (
    TOOL_DOWN_SEED_Q,
    anchor_plan,
    tool_down_branch_seeds,
)
from st09_contracts.fixtures import make_timed_trajectory
from st09_contracts.graph import GraphEdge, LoadState

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

#: Planner commits that later matrix runs must name. Values are full hashes.
#: ``route_matrix_20261004/manifest.json`` is historical and is not updated.
MATRIX_LINEAGE = {
    "descend_pair_baseline": "f9ba0e84aebb0b39979871aee3ebf1306c040232",
    "container_drawing_rule": "bdb9394307d81a44496f7baa7b5e7bdf983a5a67",
    "candidate_scope": "dd6d48eaec68965b1e94d4a9df68346ace1b4957",
}
SCENE = os.path.join(
    ROOT, "install-st09/luggage_description/share/luggage_description/"
    "config/scene_tf_deploy.yaml")
ATLAS_DIR = os.path.join(
    ROOT, "src/luggage_planning/data/reachability_atlas_deploy")
RUN02_CANDIDATES = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3/"
    "g3_three_box_run02_20261004/dumps/box_01_paths/candidates.json")

LINE_STEP_M = 0.02
LINE_MAX_JUMP_RAD = 0.3
JOINT_STEP_RAD = 0.05
CONTINUITY_MAX_STEP_RAD = 0.1
CARTESIAN_MIN_FRACTION = 0.95
TRANSIT_CLEARANCE_Z = 0.86
HELD_OUT_FRACTION = 0.3
#: New held-out draws use this salt. The 2026-10-04 held-out ids are forced
#: to dev so that split cannot accept a later planner.
SPLIT_SALT = "st09-matrix-g2-20261005"
SLOT_LIMIT = 16
#: One failure still clears Wilson 95% lower bound 0.90 at n >= 53.
TARGET_HELD_OUT_FEASIBLE = 53
TOOL_DOWN_QUAT = (0.0, 1.0, 0.0, 0.0)
PAYLOADS = {
    "carryon": (0.55, 0.40, 0.25),
    "standard": (0.70, 0.45, 0.28),
    "large": (0.80, 0.50, 0.32),
}
#: Attachment offsets (TCP in box xy). Nominal centre, and the suction
#: placement tolerance corner.
OFFSETS = {"nominal": (0.0, 0.0), "offset_tol": (0.03, -0.02)}
HEADINGS = {"h0": 0.0, "hpi": math.pi}


# -- geometry -------------------------------------------------------------

def _yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _wrap(delta):
    return (float(delta) + math.pi) % (2.0 * math.pi) - math.pi


class Scene(object):
    """Deploy scene: container pose, hull, opening datum."""

    def __init__(self, path):
        with open(path, "r", encoding="utf-8") as handle:
            self.config = yaml.safe_load(handle)
        self.geometry = descriptor_from_scene_config(self.config)
        tf = next(item for item in self.config["static_transforms"]
                  if item["child"] == "container_link")
        self.t = np.array(tf["translation"], dtype=float)
        self.yaw = float(tf["rotation_rpy"][2])
        self.q = _yaw_quat(self.yaw)
        self.r = _rot(self.q)
        aperture = self.config["container"]["opening"]["aperture"]["corners"]
        self.ap_y = (min(c[1] for c in aperture), max(c[1] for c in aperture))
        self.ap_z = (min(c[2] for c in aperture), max(c[2] for c in aperture))
        opening = next(item for item in self.config["static_transforms"]
                       if item["child"] == "container_opening_frame")
        self.opening_world = tuple(self.to_world(opening["translation"]))
        self.inward_world = tuple(self.r.dot([1.0, 0.0, 0.0]))

    def to_world(self, p):
        return self.t + self.r.dot(np.asarray(p, dtype=float))

    def to_container(self, p):
        return self.r.T.dot(np.asarray(p, dtype=float) - self.t)


def _aabb(corners):
    arr = np.asarray(corners)
    return arr.min(axis=0), arr.max(axis=0)


def _overlap(a, b, eps=1e-4):
    return bool(np.all(a[0] < b[1] - eps) and np.all(b[0] < a[1] - eps))


class Payload(object):
    """Carried box from TCP pose: centre = tcp + R_tcp * t_tcp_box."""

    def __init__(self, size, offset_xy):
        self.size = tuple(size)
        # Tool z points into the box (tool-down); TCP sits on the top face.
        self.t_tcp_box = np.array(
            [-offset_xy[0], -offset_xy[1], 0.5 * size[2]])

    def world_box(self, tcp_pos, tcp_quat):
        r = _rot(tcp_quat)
        centre = np.asarray(tcp_pos) + r.dot(self.t_tcp_box)
        # Box x axis is the tool x axis projected on the floor.
        yaw = math.atan2(r[1, 0], r[0, 0])
        return centre, yaw

    def tcp_for_box(self, centre_world, box_yaw_world):
        tcp_quat = _qmul(_yaw_quat(box_yaw_world), TOOL_DOWN_QUAT)
        r = _rot(tcp_quat)
        tcp = np.asarray(centre_world) - r.dot(self.t_tcp_box)
        return tuple(float(v) for v in tcp), tcp_quat


class Collision(object):
    """Payload-vs-hull/placed-box proxy in container coordinates."""

    def __init__(self, scene, placed):
        self.scene = scene
        self.placed = [_aabb(oriented_box_corners(c, s, y))
                       for c, s, y in placed]

    def payload_ok(self, payload, tcp_pos, tcp_quat):
        centre_w, yaw_w = payload.world_box(tcp_pos, tcp_quat)
        centre = self.scene.to_container(centre_w)
        yaw = yaw_w - self.scene.yaw
        corners = oriented_box_corners(centre, payload.size, yaw)
        geo = self.scene.geometry
        xs = [c[0] for c in corners]
        if max(xs) > -geo.half_x:  # some part inside the door plane
            if min(xs) < -geo.half_x:  # straddles the aperture
                for _x, y, z in corners:
                    if not (self.scene.ap_y[0] <= y <= self.scene.ap_y[1]
                            and self.scene.ap_z[0] <= z <= self.scene.ap_z[1]):
                        return False, "aperture"
            for corner in corners:
                if corner[0] > -geo.half_x and \
                        not contains_point_through_aperture(geo, corner):
                    return False, "hull"
        box = _aabb(corners)
        for index, other in enumerate(self.placed):
            if _overlap(box, other):
                return False, "placed_box_%d" % index
        return True, ""


# -- offline edge builder --------------------------------------------------

def _slerp(a, b, t):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b, dot = -b, -dot
    if dot > 0.9995:
        out = a + t * (b - a)
        return tuple(out / np.linalg.norm(out))
    theta = math.acos(dot)
    out = (math.sin((1 - t) * theta) * a + math.sin(t * theta) * b) \
        / math.sin(theta)
    return tuple(out)


class OfflineEdgeBuilder(object):
    JOINT = ("PORTAL_TRANSIT", "RETURN")

    def __init__(self, urdf, tcp, fk, collision, payload):
        self.urdf = urdf
        self.tcp = tcp
        self.fk = fk
        self.collision = collision
        self.payload = payload
        self.calls = []

    def _fail(self, edge_class, why, **extra):
        detail = {"why": why}
        detail.update(extra)
        self.calls[-1]["why"] = why
        raise RoutePlanningError("EDGE_BUILD_FAILED", detail)

    def _check(self, q, carrying):
        if not carrying:
            return True, ""
        pos, quat = self.fk(q)
        return self.collision.payload_ok(self.payload, pos, quat)

    def __call__(self, from_node, to_node, edge_class, candidate=None,
                 scene=None):
        carrying = (scene or {}).get("load_state") == "carrying"
        record = {"from": from_node.node_id, "to": to_node.node_id,
                  "edge_class": edge_class, "ok": False}
        self.calls.append(record)
        if edge_class in self.JOINT:
            knots = self._joint_line(from_node.q, to_node.q, carrying, record)
        else:
            knots = self._cartesian_line(from_node, to_node, edge_class,
                                         carrying, record)
        record["ok"] = True
        travel = sum(max(abs(b - a) for a, b in zip(p, n))
                     for p, n in zip(knots, knots[1:]))
        n = len(knots)
        trajectory = type(make_timed_trajectory())(
            joint_names=tuple("elfin_joint%d" % i for i in range(1, 7)),
            positions=tuple(tuple(float(v) for v in k) for k in knots),
            velocities=tuple((0.0,) * 6 for _ in range(n)),
            accelerations=tuple((0.0,) * 6 for _ in range(n)),
            time_from_start_ns=tuple(int(i * 1e8) for i in range(n)))
        return GraphEdge(
            edge_id="e-off-%04d" % len(self.calls),
            source_node_id=from_node.node_id,
            destination_node_id=to_node.node_id,
            load_state=from_node.load_state,
            static_identity="matrix",
            trajectory=trajectory,
            validation_revision=None,
            attributes={"edge_class": edge_class,
                        "duration_ns": int(max(1.0, travel) * 1e9),
                        "joint_travel_rad": float(travel)})

    def _joint_line(self, qa, qb, carrying, record):
        qa = np.asarray(qa, dtype=float)
        qb = np.asarray(qb, dtype=float)
        steps = max(1, int(math.ceil(np.max(np.abs(qb - qa)) / JOINT_STEP_RAD)))
        knots = []
        for i in range(steps + 1):
            q = qa + (qb - qa) * (float(i) / steps)
            ok, why = self._check(q, carrying)
            if not ok:
                record["why"] = why
                self._fail("joint", "collision:" + why, step=i)
            knots.append(tuple(q))
        return knots

    def _cartesian_line(self, from_node, to_node, edge_class, carrying,
                        record):
        start_pos, start_quat = self.fk(from_node.q)
        goal_pos = np.asarray(to_node.tcp_position, dtype=float)
        if edge_class in ("TRAVERSE", "RETREAT"):
            goal_quat = start_quat
        else:
            goal_quat = tuple(to_node.tcp_quaternion_xyzw)
        dist = float(np.linalg.norm(goal_pos - np.asarray(start_pos)))
        steps = max(4, int(math.ceil(dist / LINE_STEP_M)))
        prev = list(from_node.q)
        knots = [tuple(prev)]
        max_jump = 0.0
        for i in range(1, steps + 1):
            t = float(i) / steps
            pos = np.asarray(start_pos) + (goal_pos - np.asarray(start_pos)) * t
            quat = _slerp(start_quat, goal_quat, t)
            q, _info = refine_to_pose(self.urdf, self.tcp, pos, quat, prev,
                                      pos_tol=2e-3, rot_tol=2e-2)
            if q is None:
                record["fraction"] = (i - 1) / float(steps)
                self._fail(edge_class, "ik", fraction=record["fraction"])
            jump = max(abs(_wrap(b - a)) for a, b in zip(prev, q))
            max_jump = max(max_jump, jump)
            if jump > LINE_MAX_JUMP_RAD:
                record["fraction"] = (i - 1) / float(steps)
                self._fail(edge_class, "jump", fraction=record["fraction"],
                           jump=jump)
            ok, why = self._check(q, carrying)
            if not ok:
                record["fraction"] = (i - 1) / float(steps)
                self._fail(edge_class, "collision:" + why,
                           fraction=record["fraction"])
            prev = list(q)
            knots.append(tuple(prev))
        record["fraction"] = 1.0
        record["max_step_rad"] = max_jump
        return knots


# -- matrix ---------------------------------------------------------------

def _candidate_slots(scene, size, placed, heading):
    """Hull-legal, unoccupied, supported box centres in container frame."""
    geo = scene.geometry
    supports = [(geo.floor_z, None)]
    for c, s, _y in placed:
        supports.append((c[2] + 0.5 * s[2], (c, s)))
    slots = []
    for z_support, on_box in supports:
        cz = z_support + 0.5 * size[2] + 0.005
        for cx in np.arange(-geo.half_x + 0.3, geo.half_x - 0.2, 0.15):
            for cy in np.arange(-geo.half_y + 0.25, geo.half_y - 0.2, 0.15):
                centre = (float(cx), float(cy), float(cz))
                corners = oriented_box_corners(centre, size, heading)
                if not all(contains_point_through_aperture(geo, c)
                           for c in corners):
                    continue
                # Aperture Y-shadow (the packing solver's gate): the box
                # must fit through the -X opening along its own y line.
                if min(c[1] for c in corners) < scene.ap_y[0] + 0.01 or \
                        max(c[1] for c in corners) > scene.ap_y[1] - 0.01:
                    continue
                if on_box is not None:
                    bc, bs = on_box
                    if abs(cx - bc[0]) > 0.5 * bs[0] - 0.15 or \
                            abs(cy - bc[1]) > 0.5 * bs[1] - 0.1:
                        continue
                box = _aabb(corners)
                if any(_overlap(box, _aabb(oriented_box_corners(c, s, y)))
                       for c, s, y in placed):
                    continue
                slots.append(centre)
    return slots


def _slot_key(centre):
    return tuple(round(float(v), 3) for v in centre)


def _grid_slot_name(centre):
    return "s%05d_%05d_%05d" % tuple(int(round(float(v) * 1000)) for v in centre)


def _pick_slots(slots, limit=SLOT_LIMIT):
    """Deterministic spread, then a stable fill up to ``limit`` centres.

    The first four names stay door / deep / wall_y / high so the original
    case ids still exist. Extra centres are named from their rounded
    coordinates and sorted, so the pool can grow without reshuffling.
    """
    if not slots:
        return []
    picks = (
        ("door", min(slots, key=lambda c: (c[0], c[1], c[2]))),
        ("deep", max(slots, key=lambda c: (c[0], -c[1], -c[2]))),
        ("wall_y", max(slots, key=lambda c: (c[1], -c[0], -c[2]))),
        ("high", max(slots, key=lambda c: (c[2], -c[0], c[1]))),
    )
    out = []
    seen = set()
    for name, centre in picks:
        key = _slot_key(centre)
        if key not in seen:
            seen.add(key)
            out.append((name, centre))
    for centre in sorted(slots, key=_slot_key):
        if len(out) >= int(limit):
            break
        key = _slot_key(centre)
        if key in seen:
            continue
        seen.add(key)
        out.append((_grid_slot_name(centre), centre))
    return out


def load_retired_held_out(path):
    """Case ids whose 2026-10-04 split was held_out. Empty if absent."""
    if not path or not os.path.isfile(path):
        return set()
    with open(path, "r", encoding="utf-8") as handle:
        split = json.load(handle)
    return {case_id for case_id, part in split.items() if part == "held_out"}


def assign_split(case_id, retired_held_out):
    """Dev/held-out bit. Retired held-out ids stay dev."""
    if case_id in retired_held_out:
        return "dev"
    digest = int(hashlib.sha1(
        (SPLIT_SALT + "|" + case_id).encode()).hexdigest()[:8], 16)
    if (digest % 1000) < HELD_OUT_FRACTION * 1000:
        return "held_out"
    return "dev"


def _occupancies(scene):
    geo = scene.geometry
    run02_box1 = scene.to_container((-1.505, 0.585, 0.6536))
    box1 = (tuple(float(v) for v in run02_box1), PAYLOADS["carryon"],
            0.0)
    back = (geo.half_x - 0.30, -geo.half_y + 0.25,
            geo.floor_z + 0.5 * PAYLOADS["standard"][2] + 0.005)
    mid = (geo.half_x - 0.30, -geo.half_y + 0.75,
           geo.floor_z + 0.5 * PAYLOADS["carryon"][2] + 0.005)
    return {
        "occ0": [],
        "occ1": [box1],
        "occ3": [box1, (back, PAYLOADS["standard"], 0.0),
                 (mid, PAYLOADS["carryon"], 0.0)],
    }


def _starts(urdf, tcp):
    base, _ = refine_to_pose(urdf, tcp, (1.0, 0.0, 1.26), TOOL_DOWN_QUAT,
                             list(TOOL_DOWN_SEED_Q))
    yawed_quat = _qmul(_yaw_quat(1.0), TOOL_DOWN_QUAT)
    yawed, _ = refine_to_pose(urdf, tcp, (1.0, 0.0, 1.26), yawed_quat,
                              list(base))
    far, _ = refine_to_pose(urdf, tcp, (0.75, -0.55, 1.35), TOOL_DOWN_QUAT,
                            list(base))
    starts = {"pick_tooldown": base, "pick_yaw1": yawed, "pick_far": far}
    return {k: tuple(float(x) for x in v) for k, v in starts.items()
            if v is not None}


def _invalid_cases(scene, occupancy):
    geo = scene.geometry
    box1 = occupancy["occ1"][0]
    size = PAYLOADS["standard"]
    return [
        ("overlap_placed", (box1[0][0], box1[0][1], box1[0][2] + 0.05),
         "occ1"),
        ("below_floor", (0.0, -0.3, geo.floor_z - 0.10), "occ0"),
        ("through_y_wall", (0.0, geo.half_y - 0.5 * size[1] + 0.20,
                            geo.floor_z + 0.5 * size[2] + 0.005), "occ0"),
    ]


def build_cases(scene, starts, retired_held_out=()):
    occupancy = _occupancies(scene)
    cases = []
    for payload_name, size in PAYLOADS.items():
        for occ_name, placed in occupancy.items():
            for heading_name, heading in HEADINGS.items():
                slots = _pick_slots(
                    _candidate_slots(scene, size, placed, heading))
                for slot_name, centre in slots:
                    for offset_name in OFFSETS:
                        for start_name in starts:
                            cases.append({
                                "payload": payload_name,
                                "offset": offset_name,
                                "start": start_name,
                                "occupancy": occ_name,
                                "heading": heading_name,
                                "slot": slot_name,
                                "centre_container": list(centre),
                                "kind": "generated",
                            })
    for name, centre, occ_name in _invalid_cases(scene, occupancy):
        for heading_name in HEADINGS:
            cases.append({
                "payload": "standard", "offset": "nominal",
                "start": "pick_tooldown", "occupancy": occ_name,
                "heading": heading_name, "slot": name,
                "centre_container": list(centre), "kind": "invalid"})
    cases.append({
        "payload": "standard", "offset": "nominal",
        "start": "pick_tooldown", "occupancy": "occ1", "heading": "run02",
        "slot": "slot64_run02", "centre_container": None,
        # Live Run02 corridor: hover z 0.9553 matches the floor corridor.
        "corridor": "floor", "kind": "regression"})
    for case in cases:
        case["id"] = "|".join(str(case[k]) for k in (
            "payload", "offset", "start", "occupancy", "heading", "slot"))
        case["split"] = assign_split(case["id"], set(retired_held_out))
    return cases, occupancy


def _slot64_target():
    with open(RUN02_CANDIDATES, "r", encoding="utf-8") as handle:
        row = next(item for item in json.load(handle)["rows"]
                   if item.get("slot_index") == 64)
    pos, quat = row["ik_refine"][0]["target_world"]
    return tuple(pos), tuple(quat)


class Atlases(object):
    def __init__(self):
        self.cache = {}

    def get(self, size):
        key = "%.2fx%.2fx%.2f" % size
        if key not in self.cache:
            stem = os.path.join(
                ATLAS_DIR, "s20_container_collision_aware_payload_%s" % key)
            self.cache[key] = ReachabilityAtlas.load(stem + ".npz",
                                                     stem + ".yaml")
        return self.cache[key]


def run_case(case, ctx, mode, atlas_on, *, budget_s=None, extra_seeds=0,
             capture_first_descend=False):
    scene, urdf, tcp, fk = ctx["scene"], ctx["urdf"], ctx["tcp"], ctx["fk"]
    size = PAYLOADS[case["payload"]]
    payload = Payload(size, OFFSETS[case["offset"]])
    placed = ctx["occupancy"][case["occupancy"]]
    start_q = ctx["starts"][case["start"]]
    if case["slot"] == "slot64_run02":
        goal_pos, goal_quat = _slot64_target()
    else:
        heading = HEADINGS[case["heading"]]
        centre_w = scene.to_world(case["centre_container"])
        goal_pos, goal_quat = payload.tcp_for_box(
            centre_w, scene.yaw + heading)
    surface = max([scene.to_world((0, 0, scene.geometry.floor_z))[2]] + [
        scene.to_world((c[0], c[1], c[2] + 0.5 * s[2]))[2]
        for c, s, _y in placed if case.get("corridor") != "floor"])
    plan = anchor_plan(goal_pos, scene.q, size[2],
                       corridor_surface_max=surface,
                       opening=scene.opening_world,
                       inward_normal=scene.inward_world,
                       transit_clearance_z=TRANSIT_CLEARANCE_Z)
    graph = RouteGraph(static_identity="matrix")
    for load in (LoadState.CARRYING, LoadState.UNLOADED):
        graph.register_node(start_q, fk(start_q)[0], goal_quat, "home",
                            load, "start")

    def refine(pos, quat, seed):
        # Same tolerances as trajectory_selector_node's production refine.
        return refine_to_pose(urdf, tcp, pos, quat, list(seed),
                              pos_tol=2e-3, rot_tol=2e-2)

    rows = [("portal", plan.portal, LoadState.CARRYING),
            ("slot_approach", plan.slot_approach, LoadState.CARRYING),
            ("slot_upper", plan.slot_upper, LoadState.CARRYING),
            ("exit", plan.exit, LoadState.UNLOADED)]
    entry_table = [("portal", "PORTAL_TRANSIT"),
                   ("slot_approach", "TRAVERSE"),
                   ("slot_upper", "INSERT"), ("slot", "DESCEND")]
    if plan.staging is not None:
        rows.insert(0, ("staging", plan.staging, LoadState.CARRYING))
        entry_table = [("staging", "PORTAL_TRANSIT"),
                       ("portal", "TRAVERSE")] + entry_table[1:]
    rng = random.Random(case["id"])
    seeds = [start_q]
    seeds.extend(q for _label, q in tool_down_branch_seeds())
    lower = [lo for lo, _ in urdf.joint_limits()]
    upper = [hi for _, hi in urdf.joint_limits()]
    for _ in range(extra_seeds):
        seeds.append(tuple(rng.uniform(lo, hi) for lo, hi in
                           zip(lower, upper)))
    for region, position, load in rows:
        for seed in seeds:
            q, _ = refine(position, goal_quat, seed)
            if q is not None and graph.node_for_q(str(load), q) is None:
                graph.register_node(tuple(q), tuple(position), goal_quat,
                                    region, load, "anchor")
    goal_seed_list = [n.q for n in graph.nodes_by_region("slot_upper")]
    atlas_seed_count = 0
    if atlas_on:
        contact_c = scene.to_container(goal_pos)
        r = _rot(goal_quat)
        tool_yaw = math.atan2(r[1, 0], r[0, 0]) - scene.yaw
        result = ctx["atlases"].get(size).query(
            float(contact_c[0]), float(contact_c[1]), float(contact_c[2]),
            yaw=tool_yaw % (2 * math.pi))
        atlas_seeds = [tuple(float(v) for v in s)
                       for s in (result.contact_seeds or [])]
        atlas_seed_count = len(atlas_seeds)
        goal_seed_list += atlas_seeds
    goal_seed_list += seeds
    goal_nodes = []
    for seed in goal_seed_list:
        q, _ = refine(goal_pos, goal_quat, seed)
        if q is None:
            continue
        q = tuple(float(v) for v in q)
        if graph.node_for_q("carrying", q) is not None:
            continue
        node = graph.register_node(q, tuple(goal_pos), goal_quat, "slot",
                                   LoadState.CARRYING, "goal")
        graph.register_node(q, tuple(goal_pos), goal_quat, "slot",
                            LoadState.UNLOADED, "goal")
        goal_nodes.append(node)
    builder = OfflineEdgeBuilder(urdf, tcp, fk, Collision(scene, placed),
                                 payload)
    entry_hops = hops_from_table(tuple(entry_table), "carrying")
    policy = RouteBudgetPolicy(total_wall_s=budget_s) if budget_s \
        else RouteBudgetPolicy()
    budget = RouteBudget(policy=policy)
    started = time.monotonic()
    saved_cap = gps.MAX_SAME_FAILURE_PER_GOAL
    if mode == "state":
        planner = GraphRoutePlanner(graph, builder,
                                    home_node_id=graph.node_for_q(
                                        "carrying", start_q).node_id,
                                    refine=refine, entry_hops=entry_hops,
                                    region_poses={
                                        name: p for name, p, _load in rows})
        planner._register_branch_ladders(goal_nodes)
        search = RouteSearch(graph, builder, budget,
                             continuation=planner._frontier_continuation())
    else:
        search = RouteSearch(graph, builder, budget, branch_lock=False)
    if mode == "baseline" or budget_s:
        gps.MAX_SAME_FAILURE_PER_GOAL = 10 ** 6
    result = {"case": case["id"], "mode": mode, "atlas": atlas_on,
              "goal_nodes": len(goal_nodes),
              "atlas_seed_count": atlas_seed_count}
    try:
        if not goal_nodes:
            raise RoutePlanningError("NO_FEASIBLE_ROUTE",
                                     {"detail": "no IK goal",
                                      "search_outcome": gps.SEARCH_INCOMPLETE})
        entry, retreat, _goal = search.best_full_route(
            graph.node_for_q("carrying", start_q).node_id, goal_nodes,
            entry_hops=entry_hops,
            retreat_hops=hops_from_table(DEFAULT_RETREAT_HOPS, "unloaded"),
            scene_carrying={"load_state": "carrying"},
            scene_unloaded={"load_state": "unloaded"})
        result.update(_route_checks(entry, retreat, builder, fk))
        result["success"] = True
        result["reason"] = ""
        result["search_outcome"] = "FOUND"
    except RoutePlanningError as err:
        result["success"] = False
        result["reason"] = err.code
        result["search_outcome"] = err.search_outcome
        result["t2"] = {
            "goal_pos": list(goal_pos), "goal_quat": list(goal_quat),
            "start_q": list(start_q), "placed": placed,
            "anchors": {name: list(p) for name, p, _l in rows},
            "nodes": [[n.region, str(n.load_state), list(n.q), n.ik_family]
                      for n in graph.nodes.values()],
            "edge_calls": builder.calls[-60:],
            "coverage": err.detail.get("coverage"),
        }
    finally:
        gps.MAX_SAME_FAILURE_PER_GOAL = saved_cap
    result["wall_s"] = round(time.monotonic() - started, 4)
    result["edges_built"] = len(builder.calls)
    descends = [c for c in builder.calls if c["edge_class"] == "DESCEND"]
    if capture_first_descend and descends:
        first = descends[0]
        result["first_descend"] = {
            "fraction": first.get("fraction"),
            "rank": list(descend_pair_rank(graph.node(first["from"]).q,
                                           graph.node(first["to"]).q)),
            "ok": first["ok"]}
    return result


def _route_checks(entry, retreat, builder, fk):
    """Continuity, branch flips, and an independent collision re-check."""
    max_step = 0.0
    flips = 0
    collisions = 0
    for edge in entry.edges:
        klass = edge.attributes["edge_class"]
        knots = edge.trajectory.positions
        if klass in ("INSERT", "DESCEND"):
            for a, b in zip(knots, knots[1:]):
                max_step = max(max_step, max(
                    abs(_wrap(y - x)) for x, y in zip(a, b)))
            if descend_pair_rank(knots[0], knots[-1])[0] != 0:
                flips += 1
        for a, b in zip(knots, knots[1:]):
            for t in (0.25, 0.5, 0.75, 1.0):
                q = [x + (y - x) * t for x, y in zip(a, b)]
                ok, _why = builder._check(q, True)
                if not ok:
                    collisions += 1
    return {"constrained_max_step_rad": round(max_step, 5),
            "constrained_branch_flips": flips,
            "proxy_collisions": collisions,
            "entry_edges": len(entry.edges),
            "retreat_edges": len(retreat.edges)}


def wilson_lower(successes, n, z=1.96):
    if n == 0:
        return None
    p = successes / float(n)
    denom = 1 + z * z / n
    centre = p + z * z / (2 * n)
    margin = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (centre - margin) / denom


def _percentile(values, q):
    if not values:
        return None
    return float(np.percentile(values, q))


#: Wilson 95% lower bound >= 0.90 is unattainable below 35 cases even at
#: 100% success, so fewer feasible held-out cases is not_evaluated.
MIN_FEASIBLE_CASES = 35


def summarize(results, cases):
    """Rates are per CASE: a case passes only if every repetition passed.

    Offline repetitions are deterministic in outcome; they measure timing
    and must not inflate the Wilson sample size.
    """
    by_id = {c["id"]: c for c in cases}
    out = {}
    for mode in ("baseline", "state"):
        for atlas_on in (True, False):
            rows = [r for r in results if r["mode"] == mode
                    and r["atlas"] == atlas_on and not r.get("reference")]
            held = [r for r in rows if by_id[r["case"]]["split"] == "held_out"]
            feasible = [r for r in held
                        if by_id[r["case"]].get("class") == "feasible"]
            # Invalid cases are constructed, not tuned on: all are scored.
            invalid = [r for r in rows
                       if by_id[r["case"]]["kind"] == "invalid"]
            per_case = {}
            for r in feasible:
                per_case.setdefault(r["case"], []).append(r["success"])
            ok = sum(1 for flags in per_case.values() if all(flags))
            n_cases = len(per_case)
            wall = [r["wall_s"] for r in feasible]
            good = [r for r in feasible if r["success"]]
            out["%s/atlas_%s" % (mode, "on" if atlas_on else "off")] = {
                "held_out_feasible_cases": n_cases,
                "held_out_feasible_runs": len(feasible),
                "reps_outcome_consistent": all(
                    len(set(flags)) == 1 for flags in per_case.values()),
                "success": ok,
                "success_rate": (ok / float(n_cases)) if n_cases else None,
                "wilson95_lower": wilson_lower(ok, n_cases),
                "failed_cases": sorted(c for c, flags in per_case.items()
                                       if not all(flags)),
                "over_budget_runs": sum(1 for r in feasible
                                        if r["wall_s"] > 10.0),
                "wall_p50_s": _percentile(wall, 50),
                "wall_p95_s": _percentile(wall, 95),
                "edges_p50": _percentile([r["edges_built"] for r in feasible],
                                         50),
                "edges_p95": _percentile([r["edges_built"] for r in feasible],
                                         95),
                "invalid_runs": len(invalid),
                "invalid_false_success": sum(1 for r in invalid
                                             if r["success"]),
                "invalid_outcomes": sorted({r["search_outcome"]
                                            for r in invalid}),
                "constrained_max_step_rad": max(
                    [r["constrained_max_step_rad"] for r in good] or [None],
                    key=lambda v: -1 if v is None else v),
                "constrained_branch_flips": sum(
                    r["constrained_branch_flips"] for r in good),
                "proxy_collisions": sum(r["proxy_collisions"] for r in good),
                "failure_outcomes": _count(
                    [r["search_outcome"] for r in feasible
                     if not r["success"]]),
            }
    return out


def _count(values):
    out = {}
    for value in values:
        out[value] = out.get(value, 0) + 1
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--reps", type=int, default=3)
    parser.add_argument("--limit", type=int, default=0,
                        help="first N generated cases only (smoke)")
    parser.add_argument("--reference-budget-s", type=float, default=60.0)
    parser.add_argument("--reference-extra-seeds", type=int, default=8)
    parser.add_argument("--resummarize", action="store_true",
                        help="recompute summary.json from --out/trace.jsonl")
    parser.add_argument("--reference-only", action="store_true",
                        help="classify feasible/undetermined and write the "
                             "freeze, then stop before baseline/state")
    parser.add_argument("--score-frozen", action="store_true",
                        help="score baseline and state on an existing "
                             "reference trace; does not rewrite it")
    parser.add_argument("--score-split", choices=("dev", "held_out"),
                        default=None,
                        help="score one split into its own trace; "
                             "held_out is the acceptance run")
    parser.add_argument(
        "--retired-held-out",
        default=os.path.join(
            ROOT, "docs", "status", "evidence", "frame_migration",
            "ST09-parallel-I-s5", "route_matrix_20261004", "split.json"),
        help="split.json whose held_out ids are forced to dev")
    args = parser.parse_args()
    if args.resummarize:
        resummarize(args.out)
        return
    if args.score_frozen:
        score_frozen(args)
        return
    os.makedirs(args.out, exist_ok=True)
    started = time.time()
    scene = Scene(SCENE)
    urdf = load_urdf(scene_tf_config=SCENE)
    tcp = urdf.path_to("suction_contact_frame")
    fk = make_offline_fk(urdf, tcp)
    starts = _starts(urdf, tcp)
    retired = load_retired_held_out(args.retired_held_out)
    cases, occupancy = build_cases(scene, starts, retired)
    if args.limit:
        generated = [c for c in cases if c["kind"] == "generated"]
        keep = set(c["id"] for c in generated[:args.limit])
        cases = [c for c in cases if c["kind"] != "generated"
                 or c["id"] in keep]
    with open(os.path.join(args.out, "split.json"), "w") as handle:
        json.dump({c["id"]: c["split"] for c in cases}, handle, indent=0)
    ctx = {"scene": scene, "urdf": urdf, "tcp": tcp, "fk": fk,
           "starts": starts, "occupancy": occupancy, "atlases": Atlases()}
    trace_path = os.path.join(args.out, "trace.jsonl")
    t2_dir = os.path.join(args.out, "t2")
    os.makedirs(t2_dir, exist_ok=True)
    results = []
    with open(trace_path, "w") as trace:
        def emit(row):
            t2 = row.pop("t2", None)
            if t2 is not None and not row.get("reference"):
                name = hashlib.sha1(("%s|%s|%s|%s" % (
                    row["case"], row["mode"], row["atlas"],
                    row.get("rep", 0))).encode()).hexdigest()[:16]
                path = os.path.join(t2_dir, name + ".json")
                with open(path, "w") as handle:
                    json.dump(dict(t2, case=row["case"], mode=row["mode"],
                                   atlas=row["atlas"]), handle, default=str)
                row["t2_bundle"] = os.path.relpath(path, args.out)
            trace.write(json.dumps(row, default=str) + "\n")
            trace.flush()
            results.append(row)

        for index, case in enumerate(cases):
            if case["kind"] == "generated":
                ref = run_case(case, ctx, "state", True,
                               budget_s=args.reference_budget_s,
                               extra_seeds=args.reference_extra_seeds)
                ref["reference"] = True
                case["class"] = "feasible" if ref["success"] \
                    else "undetermined"
                emit(ref)
            elif case["kind"] == "regression":
                # Gated on its first DESCEND only; the full route sits on
                # the reach boundary and has no live ground truth.
                case["class"] = "regression"
            else:
                case["class"] = "invalid"
            if args.reference_only:
                print("[%d/%d] %s %s" % (index + 1, len(cases), case["id"],
                                         case["class"]), flush=True)
                continue
            for atlas_on in (True, False):
                reps = args.reps if case["split"] == "held_out" else 1
                for rep in range(reps):
                    for mode in ("baseline", "state"):
                        if mode == "baseline" and rep > 0:
                            continue
                        row = run_case(
                            case, ctx, mode, atlas_on,
                            capture_first_descend=(
                                case["kind"] == "regression"))
                        row["rep"] = rep
                        emit(row)
            print("[%d/%d] %s %s" % (index + 1, len(cases), case["id"],
                                     case["class"]), flush=True)
    if args.reference_only:
        freeze = write_reference_freeze(args, cases, started)
        raise SystemExit(0 if freeze["ready_for_scored_run"] else 2)
    summary = summarize(results, cases)
    regression = [r for r in results if r["case"].endswith("slot64_run02")
                  and r["mode"] == "state"]
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                            capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "status", "--porcelain"], cwd=ROOT,
                           capture_output=True, text=True).stdout
    sizes = sum(os.path.getsize(os.path.join(dp, f))
                for dp, _dn, fn in os.walk(args.out) for f in fn)
    manifest = {
        "commit": commit,
        "dirty_count": len([l for l in dirty.splitlines() if l.strip()]),
        "lineage": dict(MATRIX_LINEAGE),
        "command": " ".join(sys.argv),
        "started": started, "ended": time.time(),
        "scene": os.path.relpath(SCENE, ROOT),
        "cases": len(cases),
        "case_classes": _count([c["class"] for c in cases]),
        "split": _count([c["split"] for c in cases]),
        "reps_held_out": args.reps,
        "reference": {"budget_s": args.reference_budget_s,
                      "extra_seeds": args.reference_extra_seeds},
        "edge_model": {
            "cartesian": "straight TCP line, %.3f m steps, continuous "
                         "refine, jump > %.2f rad fails" % (
                             LINE_STEP_M, LINE_MAX_JUMP_RAD),
            "joint": "linear joint interpolation, %.2f rad steps" %
                     JOINT_STEP_RAD,
            "collision": "payload vs 7-face hull through aperture and "
                         "placed boxes; arm links not checked",
            "timing": "offline wall time, not MoveIt",
        },
        "budgets": {"route_total_wall_s": gps.ROUTE_TOTAL_WALL_S,
                    "single_new_edge_wall_s": gps.SINGLE_NEW_EDGE_WALL_S,
                    "max_edge_repair_attempts": gps.MAX_EDGE_REPAIR_ATTEMPTS,
                    "max_same_failure_per_goal":
                        gps.MAX_SAME_FAILURE_PER_GOAL},
        "artifacts": ["split.json", "trace.jsonl", "t2/", "summary.json"],
        "size_bytes": sizes,
        "capture_complete": True,
        "replay_possible": True,
        "replay_command": "python3 scripts/st09_i/route_matrix_eval.py "
                          "--out <dir>  (deterministic inputs; see split)",
    }
    gate = _gate(summary, regression)
    with open(os.path.join(args.out, "summary.json"), "w") as handle:
        json.dump({"summary": summary, "regression_slot64": regression,
                   "gate": gate}, handle, indent=1, default=str)
    with open(os.path.join(args.out, "manifest.json"), "w") as handle:
        json.dump(manifest, handle, indent=1)
    print(json.dumps({"summary": summary, "gate": gate}, indent=1,
                     default=str))
    raise SystemExit(0 if gate["pass"] else 1)


def load_reference_rows(trace_path):
    """Reference classifications already written by ``--reference-only``."""
    rows = []
    classes = {}
    with open(trace_path, "r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            row = json.loads(line)
            if not row.get("reference"):
                continue
            rows.append(row)
            classes[row["case"]] = ("feasible" if row.get("success")
                                    else "undetermined")
    return rows, classes


def apply_reference_classes(cases, classes):
    """Label each generated case from the frozen reference trace."""
    missing = []
    for case in cases:
        if case["kind"] == "generated":
            label = classes.get(case["id"])
            if label is None:
                missing.append(case["id"])
            else:
                case["class"] = label
        elif case["kind"] == "regression":
            case["class"] = "regression"
        else:
            case["class"] = "invalid"
    if missing:
        raise SystemExit("reference trace missing %d cases, first %s" % (
            len(missing), missing[0]))


def case_is_scored(case):
    """Reference-undetermined generated cases are not an acceptance sample.

    The reference search already failed them. They are excluded from the
    Wilson denominator, and running the 10 s planner on them does not
    change the gate.
    """
    return not (case.get("kind") == "generated"
                and case.get("class") == "undetermined")


def score_frozen(args):
    """Baseline and state on the frozen reference labels.

    Reads ``trace.jsonl`` and writes ``score_trace.jsonl``. The reference
    file is not opened for write.
    """
    trace_path = os.path.join(args.out, "trace.jsonl")
    if not os.path.isfile(trace_path):
        raise SystemExit("no reference trace at %s" % trace_path)
    started = time.time()
    scene = Scene(SCENE)
    urdf = load_urdf(scene_tf_config=SCENE)
    tcp = urdf.path_to("suction_contact_frame")
    fk = make_offline_fk(urdf, tcp)
    starts = _starts(urdf, tcp)
    retired = load_retired_held_out(args.retired_held_out)
    cases, occupancy = build_cases(scene, starts, retired)
    reference_rows, classes = load_reference_rows(trace_path)
    apply_reference_classes(cases, classes)
    held_feasible = sum(
        1 for case in cases
        if case["split"] == "held_out" and case.get("class") == "feasible")
    if held_feasible < MIN_FEASIBLE_CASES:
        print(json.dumps({
            "held_out_feasible": held_feasible,
            "outcome": "not_evaluated",
            "reason": "below minimum feasible held-out",
        }))
        raise SystemExit(2)
    ctx = {"scene": scene, "urdf": urdf, "tcp": tcp, "fk": fk,
           "starts": starts, "occupancy": occupancy, "atlases": Atlases()}
    split_name = getattr(args, "score_split", None)
    score_name = ("score_%s_trace.jsonl" % split_name) if split_name \
        else "score_trace.jsonl"
    summary_name = ("summary_%s.json" % split_name) if split_name \
        else "summary.json"
    manifest_name = ("score_%s_manifest.json" % split_name) if split_name \
        else "score_manifest.json"
    score_path = os.path.join(args.out, score_name)
    t2_dir = os.path.join(args.out, "t2")
    os.makedirs(t2_dir, exist_ok=True)
    results = list(reference_rows)
    with open(score_path, "w", encoding="utf-8") as trace:
        def emit(row):
            t2 = row.pop("t2", None)
            if t2 is not None:
                name = hashlib.sha1(("%s|%s|%s|%s" % (
                    row["case"], row["mode"], row["atlas"],
                    row.get("rep", 0))).encode()).hexdigest()[:16]
                path = os.path.join(t2_dir, name + ".json")
                with open(path, "w", encoding="utf-8") as handle:
                    json.dump(dict(t2, case=row["case"], mode=row["mode"],
                                   atlas=row["atlas"]), handle, default=str)
                row["t2_bundle"] = os.path.relpath(path, args.out)
            trace.write(json.dumps(row, default=str) + "\n")
            trace.flush()
            results.append(row)

        for index, case in enumerate(cases):
            if not case_is_scored(case):
                continue
            for atlas_on in (True, False):
                reps = args.reps if case["split"] == "held_out" else 1
                for rep in range(reps):
                    for mode in ("baseline", "state"):
                        if mode == "baseline" and rep > 0:
                            continue
                        row = run_case(
                            case, ctx, mode, atlas_on,
                            capture_first_descend=(
                                case["kind"] == "regression"))
                        row["rep"] = rep
                        emit(row)
            print("[%d/%d] %s %s" % (index + 1, len(cases), case["id"],
                                     case.get("class")), flush=True)
    summary = summarize(results, cases)
    regression = [r for r in results if r["case"].endswith("slot64_run02")
                  and r["mode"] == "state" and not r.get("reference")]
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT,
        capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(
        ["git", "status", "--porcelain"], cwd=ROOT,
        capture_output=True, text=True).stdout
    manifest = {
        "commit": commit,
        "dirty_count": len([line for line in dirty.splitlines() if line.strip()]),
        "lineage": dict(MATRIX_LINEAGE),
        "planner_revision": MATRIX_LINEAGE["candidate_scope"],
        "command": " ".join(sys.argv),
        "started": started,
        "ended": time.time(),
        "cases": len(cases),
        "case_classes": _count([c.get("class") for c in cases]),
        "split": _count([c["split"] for c in cases]),
        "reps_held_out": args.reps,
        "held_out_feasible": held_feasible,
        "skipped_reference_undetermined": True,
        "score_trace": "score_trace.jsonl",
        "reference_trace": "trace.jsonl",
        "capture_complete": True,
        "replay_possible": True,
    }
    gate = _gate(summary, regression)
    with open(os.path.join(args.out, summary_name), "w") as handle:
        json.dump({"summary": summary, "regression_slot64": regression,
                   "gate": gate}, handle, indent=1, default=str)
    with open(os.path.join(args.out, manifest_name), "w") as handle:
        json.dump(manifest, handle, indent=1)
    print(json.dumps({"summary": summary, "gate": gate}, indent=1,
                     default=str))
    raise SystemExit(0 if gate["pass"] else 1)


def write_reference_freeze(args, cases, started):
    """Record the frozen split after reference classification.

    The scored baseline/state run must read this directory's ``split.json``.
    Held-out rows are not a tuning set.
    """
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT,
        capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(
        ["git", "status", "--porcelain"], cwd=ROOT,
        capture_output=True, text=True).stdout
    generated = [c for c in cases if c["kind"] == "generated"]
    held_feasible = [
        c["id"] for c in generated
        if c["split"] == "held_out" and c.get("class") == "feasible"]
    dev_feasible = [
        c["id"] for c in generated
        if c["split"] == "dev" and c.get("class") == "feasible"]
    freeze = {
        "schema": "st09.route_matrix_freeze/1",
        "planner_revision": MATRIX_LINEAGE["candidate_scope"],
        "harness_commit": commit,
        "dirty_count": len([line for line in dirty.splitlines() if line.strip()]),
        "lineage": dict(MATRIX_LINEAGE),
        "split_salt": SPLIT_SALT,
        "retired_held_out": os.path.relpath(args.retired_held_out, ROOT)
        if args.retired_held_out else "",
        "reference": {
            "budget_s": args.reference_budget_s,
            "extra_seeds": args.reference_extra_seeds,
        },
        "command": " ".join(sys.argv),
        "started": started,
        "ended": time.time(),
        "target_held_out_feasible": TARGET_HELD_OUT_FEASIBLE,
        "held_out_feasible": len(held_feasible),
        "dev_feasible": len(dev_feasible),
        "case_classes": _count([c.get("class") for c in cases]),
        "split": _count([c["split"] for c in cases]),
        "ready_for_scored_run": len(held_feasible) >= TARGET_HELD_OUT_FEASIBLE,
        "held_out_feasible_ids": held_feasible,
        "dev_feasible_ids": dev_feasible,
    }
    with open(os.path.join(args.out, "freeze.json"), "w") as handle:
        json.dump(freeze, handle, indent=1)
    manifest = {
        "schema": "st09.route_matrix_manifest/1",
        "commit": commit,
        "dirty_count": freeze["dirty_count"],
        "lineage": dict(MATRIX_LINEAGE),
        "planner_revision": freeze["planner_revision"],
        "command": freeze["command"],
        "split_salt": SPLIT_SALT,
        "reference": dict(freeze["reference"]),
        "case_ids": [c["id"] for c in cases],
        "split": {c["id"]: c["split"] for c in cases},
        "case_classes": freeze["case_classes"],
        "held_out_feasible": freeze["held_out_feasible"],
        "dev_feasible": freeze["dev_feasible"],
        "ready_for_scored_run": freeze["ready_for_scored_run"],
        "capture_complete": True,
    }
    with open(os.path.join(args.out, "manifest.json"), "w") as handle:
        json.dump(manifest, handle, indent=1)
    print(json.dumps({
        "held_out_feasible": freeze["held_out_feasible"],
        "dev_feasible": freeze["dev_feasible"],
        "ready_for_scored_run": freeze["ready_for_scored_run"],
        "target": TARGET_HELD_OUT_FEASIBLE,
    }), flush=True)
    return freeze


def resummarize(out_dir):
    """Rebuild summary.json from a finished run without re-planning."""
    with open(os.path.join(out_dir, "split.json")) as handle:
        split = json.load(handle)
    with open(os.path.join(out_dir, "trace.jsonl")) as handle:
        results = [json.loads(line) for line in handle]
    reference = {r["case"]: r["success"] for r in results
                 if r.get("reference")}
    cases = []
    for case_id, part in split.items():
        slot = case_id.split("|")[-1]
        if slot == "slot64_run02":
            kind, klass = "regression", "regression"
        elif slot in ("overlap_placed", "below_floor", "through_y_wall"):
            kind, klass = "invalid", "invalid"
        else:
            kind = "generated"
            klass = "feasible" if reference.get(case_id) else "undetermined"
        cases.append({"id": case_id, "split": part, "kind": kind,
                      "class": klass})
    summary = summarize(results, cases)
    regression = [r for r in results if r["case"].endswith("slot64_run02")
                  and r["mode"] == "state"]
    gate = _gate(summary, regression)
    with open(os.path.join(out_dir, "summary.json"), "w") as handle:
        json.dump({"summary": summary, "regression_slot64": regression,
                   "gate": gate, "case_classes": _count(
                       [c["class"] for c in cases])},
                  handle, indent=1, default=str)
    print(json.dumps({"summary": summary, "gate": gate}, indent=1,
                     default=str))


def _gate(summary, regression):
    checks = {}
    for atlas in ("on", "off"):
        row = summary["state/atlas_%s" % atlas]
        n = row["held_out_feasible_cases"]
        point_ok = row["success_rate"] is not None and \
            row["success_rate"] >= 0.95 and row["over_budget_runs"] == 0
        if n == 0:
            verdict = "not_evaluated"
        elif not point_ok:
            verdict = False  # the point estimate alone already fails
        elif n < MIN_FEASIBLE_CASES:
            verdict = "not_evaluated"
        else:
            verdict = bool(row["wilson95_lower"] >= 0.90)
        checks["feasible_rate_atlas_%s" % atlas] = verdict
        checks["invalid_no_false_success_atlas_%s" % atlas] = (
            "not_evaluated" if row["invalid_runs"] == 0 else
            row["invalid_false_success"] == 0)
        checks["continuity_atlas_%s" % atlas] = bool(
            (row["constrained_max_step_rad"] or 0.0)
            <= CONTINUITY_MAX_STEP_RAD
            and row["constrained_branch_flips"] == 0
            and row["proxy_collisions"] == 0)
        checks["p95_within_budget_atlas_%s" % atlas] = (
            row["wall_p95_s"] is None or row["wall_p95_s"] <= 10.0)
    first = [r.get("first_descend") for r in regression
             if r.get("first_descend")]
    checks["slot64_first_descend_fraction"] = bool(first) and all(
        (f["fraction"] or 0.0) >= CARTESIAN_MIN_FRACTION for f in first)
    passed = all(v is True for v in checks.values())
    return {"checks": checks, "pass": passed}


if __name__ == "__main__":
    main()
