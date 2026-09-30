#!/usr/bin/env python3
"""Unified place-path selector: grid search plus geometric variants.

Collision, missing IK, and an empty feasible set are rejections, not large
finite costs. ``mode: legacy`` delegates to ``PlacePathPlanner``.
"""

from __future__ import division

import hashlib
import json
import math
import os

from luggage_planning.occupancy_place_paths import (
    DEFAULT_WEIGHTS,
    OccupancySnapshot,
    PLACE_PATH_INFEASIBLE,
    clip01,
    load_selector_weights,
    path_length,
)
from luggage_planning.path_candidates import (
    GeometricVariantGenerator,
    GridPathGenerator,
)
from luggage_planning.place_path_planner import PlacePathPlanner


SELECTOR_DEFAULTS = {
    "mode": "unified",
    "w_placement": 1.0,
    "w_dimension": 1.0,
    "w_safety": 1.5,
    "w_efficiency": 0.5,
    "safety_ref_m": 0.10,
    "placement_score_ref": 1.0,
    "joint_travel_ref_rad": 3.0,
    "max_joint_step_rad": 1.2,
    "high_z_gain": 8.0,
    "diversity_penalty": 3.0,
    "overlap_max": 0.7,
    "probe_budget": 3,
    "hash_policy": "off",
    # Five families: direct, clear_top, clear_headroom, corridor, offset.
    "max_candidates_per_slot": 5,
    "cartesian_min_fraction": 0.95,
    # Arm-ceiling cost: prefilter penalises descent-while-translating legs
    # (they ride the wrist band on the lid); the optional arm_check callable
    # replaces it with the measured wrist/elbow clearance on the top-K rows.
    "w_arm": 1.0,
    "arm_margin_ref_m": 0.10,
    "arm_min_margin_m": 0.0,
    "arm_check_top_k": 4,
    "arm_radius_m": 0.08,
    "inflate_m": 0.05,
}

_METHOD_RANK = {
    "grid_short": 0,
    "grid_high": 1,
    "grid_clear": 2,
    "direct": 3,
    "clear_top": 4,
    "clear_headroom": 5,
}


def selector_config_path(module_file=None):
    here = os.path.dirname(os.path.abspath(module_file or __file__))
    path = os.path.normpath(os.path.join(
        here, "..", "config", "trajectory_selector.yaml"))
    if os.path.isfile(path):
        return path
    return None


def load_selector_config(path=None):
    """Merge trajectory-selector YAML over the sweep defaults."""
    cfg = dict(DEFAULT_WEIGHTS)
    cfg.update(SELECTOR_DEFAULTS)
    if path is None:
        path = selector_config_path()
    if not path or not os.path.isfile(path):
        return cfg
    try:
        import yaml
    except ImportError:
        return cfg
    with open(path, "r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle) or {}
    if not isinstance(loaded, dict):
        return cfg
    for key, value in loaded.items():
        if key in cfg and isinstance(cfg[key], bool):
            cfg[key] = bool(value)
        elif key in cfg and isinstance(cfg[key], int) and not isinstance(cfg[key], bool):
            cfg[key] = int(value)
        elif key in cfg and isinstance(cfg[key], float):
            cfg[key] = float(value)
        else:
            cfg[key] = value
    return cfg


def lateral_margin(snapshot, waypoints, payload_section):
    """XY gap from the polyline to occupied columns, minus half the section.

    This is independent of vertical clearance: a high path over a narrow
    corridor still pays the dimension term.
    """
    if snapshot is None or not waypoints:
        return float("inf")
    half = 0.5 * abs(float(payload_section))
    best = float("inf")
    for point in waypoints:
        nearest = float("inf")
        for ix in range(snapshot.nx):
            for iy in range(snapshot.ny):
                if not snapshot.obstacle[ix][iy]:
                    continue
                aabb = snapshot.cell_aabb_xy(ix, iy)
                dx = max(0.0, aabb[0] - point[0], point[0] - aabb[2])
                dy = max(0.0, aabb[1] - point[1], point[1] - aabb[3])
                nearest = min(nearest, math.hypot(dx, dy) if dx or dy else 0.0)
        if nearest == float("inf"):
            return float("inf")
        best = min(best, nearest - half)
    return best


def arm_prefilter(waypoints):
    """Zero-cost stand-in for the arm-ceiling cost before any IK.

    A leg that descends while translating (|dz| and horizontal extent both
    significant) drives the wrist/elbow band across the lid at door-crossing
    depth -- the 2026-09-27 box3 failure shape. Axis-aligned legs (corridor)
    score 0. Returns a cost in [0, 1].
    """
    if not waypoints or len(waypoints) < 2:
        return 0.0
    worst = 0.0
    for a, b in zip(waypoints, waypoints[1:]):
        dz = abs(float(b[2]) - float(a[2]))
        xy = math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
        if dz > 0.05 and xy > 0.10:
            worst = max(worst, min(1.0, dz / 0.4))
    return worst


def joint_travel(seeds):
    if not seeds or len(seeds) < 2:
        return 0.0
    from luggage_planning.joint_path import max_joint_step
    total = 0.0
    for left, right in zip(seeds, seeds[1:]):
        total += max_joint_step(left, right)
    return total


def score_row(row, cfg, payload_section):
    """Write cost terms. Infeasible rows keep null costs."""
    row["selected"] = False
    if not row.get("feasible"):
        for key in ("C_place", "C_dim", "C_safe", "C_arm", "C_eff", "cost"):
            row[key] = None
        return None
    ref = float(cfg["placement_score_ref"])
    c_place = 1.0 - clip01(float(row.get("slot_score", 0.0)) / max(ref, 1e-6))
    gap = row.get("free_gap_m")
    safety_ref = float(cfg["safety_ref_m"])
    if gap is None:
        c_dim = 0.0
    elif float(gap) <= 0.0:
        c_dim = 1.0
    else:
        c_dim = 1.0 - clip01(float(gap) / max(safety_ref, 1e-6))
    c_safe = 1.0 - clip01(float(row.get("min_clearance", 0.0)) / max(safety_ref, 1e-6))
    margin = row.get("arm_margin_m")
    if margin is None:
        c_arm = arm_prefilter(row.get("waypoints") or [])
    elif float(margin) < float(cfg.get("arm_min_margin_m", 0.0)):
        c_arm = 1.0
    else:
        c_arm = 1.0 - clip01(
            float(margin) / max(float(cfg.get("arm_margin_ref_m", 0.10)), 1e-6))
    waypoints = row.get("waypoints") or []
    if len(waypoints) < 2:
        detour = 0.0
    else:
        straight = max(1e-6, path_length([waypoints[0], waypoints[-1]]))
        detour = max(0.0, path_length(waypoints) / straight - 1.0)
    fraction = row.get("cartesian_fraction")
    frac_cost = 0.5 if fraction is None else 1.0 - clip01(float(fraction))
    travel = joint_travel(row.get("joint_seeds"))
    joint_cost = clip01(travel / max(float(cfg["joint_travel_ref_rad"]), 1e-6))
    c_eff = (detour + frac_cost + joint_cost) / 3.0
    cost = (
        float(cfg["w_placement"]) * c_place
        + float(cfg["w_dimension"]) * c_dim
        + float(cfg["w_safety"]) * c_safe
        + float(cfg.get("w_arm", 1.0)) * c_arm
        + float(cfg["w_efficiency"]) * c_eff)
    row["C_place"] = round(c_place, 4)
    row["C_dim"] = round(c_dim, 4)
    row["C_safe"] = round(c_safe, 4)
    row["C_arm"] = round(c_arm, 4)
    row["C_eff"] = round(c_eff, 4)
    row["cost"] = round(cost, 4)
    return row["cost"]


def _path_hash(row):
    payload = {
        "method": row.get("method"),
        "slot": row.get("slot_index"),
        "waypoints": [[round(float(v), 4) for v in point] for point in (row.get("waypoints") or [])],
    }
    raw = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(raw.encode("utf-8")).hexdigest()[:16]


def _sort_key(row):
    return (
        float(row.get("cost") if row.get("cost") is not None else 1e9),
        int(_METHOD_RANK.get(row.get("method"), 9)),
        int(row.get("slot_index") or 0),
        str(row.get("path_hash") or ""),
    )


def _apply_probe_gate(row, probe_fn, min_fraction):
    if probe_fn is None and row.get("source") == "grid" and row.get("joint_seeds"):
        row["ik_ok"] = True
        row["cartesian_fraction"] = 1.0
        return
    if probe_fn is None:
        return
    probe_fn(row)
    if row.get("ik_ok") is None or row.get("cartesian_fraction") is None:
        row["feasible"] = False
        row["reason"] = row.get("reason") or "PLACE_PROBE_UNAVAILABLE"
        return
    if row.get("ik_ok") is False:
        row["feasible"] = False
        row["reason"] = row.get("reason") or "ik_failed"
        return
    fraction = row.get("cartesian_fraction")
    if (fraction is not None and float(fraction) < float(min_fraction)
            and not row.get("ompl_ok")):
        row["feasible"] = False
        row["reason"] = row.get("reason") or ("cartesian_fraction_%.3f" % float(fraction))


class TrajectorySelector(object):
    def __init__(self, config=None):
        self.config = dict(SELECTOR_DEFAULTS)
        self.config.update(DEFAULT_WEIGHTS)
        if config:
            self.config.update(config)
        self.last_diagnostics = {}

    @classmethod
    def from_yaml(cls, path=None):
        return cls(load_selector_config(path))

    def select(self, surface, slots, portal, payload_wdh, atlas=None,
               expected_hash=None, expected_revision=None,
               opening_tangent=None, probe_fn=None, yaw=0.0,
               start_joints=None, scene_tf_hash=None, urdf_hash=None,
               excluded_ids=None, arm_check=None):
        """Return ``(winner, rows, reason, snapshot)``."""
        if str(self.config.get("mode") or "unified") == "legacy":
            return self._legacy(
                surface, slots, portal, payload_wdh, expected_hash,
                expected_revision, opening_tangent, probe_fn, yaw)
        cfg = self.config
        snapshot = OccupancySnapshot.from_surface_2d(
            surface, expected_hash=expected_hash,
            expected_revision=expected_revision,
            inflate_m=float(cfg["inflate_m"]))
        section = max(abs(float(payload_wdh[0])), abs(float(payload_wdh[1])))
        geometric = GeometricVariantGenerator(
            cfg["max_candidates_per_slot"], cfg["arm_radius_m"], cfg)
        grid = GridPathGenerator(
            cfg["max_joint_step_rad"], cfg["high_z_gain"],
            cfg["diversity_penalty"], cfg["overlap_max"], cfg["arm_radius_m"])
        excluded = set(excluded_ids or [])
        rows = []
        diagnostics = {"slots": [], "probe_calls": 0}
        for slot in slots:
            rows.extend(geometric.rows(
                snapshot, slot, portal, payload_wdh, opening_tangent, yaw))
            grid_rows, diag = grid.rows(
                atlas, snapshot, slot, portal, payload_wdh, yaw,
                scene_tf_hash=scene_tf_hash, urdf_hash=urdf_hash,
                policy=cfg.get("hash_policy"))
            diagnostics["slots"].append(diag)
            rows.extend(grid_rows)
        kept = []
        for row in rows:
            row["path_hash"] = _path_hash(row)
            gap = lateral_margin(snapshot, row["waypoints"], section)
            row["free_gap_m"] = None if gap == float("inf") else float(gap)
            if row["path_hash"] in excluded:
                row["feasible"] = False
                row["reason"] = "excluded"
            score_row(row, cfg, section)
            kept.append(row)
        ranked = sorted(
            [row for row in kept if row.get("feasible")], key=_sort_key)
        passed = []
        probe_passes = 0
        budget = int(cfg["probe_budget"])
        for row in ranked:
            if row.get("source") == "grid":
                _apply_probe_gate(row, None, cfg["cartesian_min_fraction"])
                score_row(row, cfg, section)
                passed.append(row)
                continue
            if probe_fn is not None and probe_passes >= budget:
                row["feasible"] = False
                row["reason"] = "probe_budget_deferred"
                score_row(row, cfg, section)
                continue
            before = row.get("feasible")
            _apply_probe_gate(row, probe_fn, cfg["cartesian_min_fraction"])
            if probe_fn is not None:
                diagnostics["probe_calls"] += 1
            score_row(row, cfg, section)
            if row.get("feasible"):
                probe_passes += 1
                passed.append(row)
            elif before:
                pass
        winner = None
        reason = ""
        if passed and callable(arm_check):
            # Top-K measured wrist/elbow clearance replaces the prefilter
            # cost on the real contenders (offline IK walk, ~40 ms/row).
            checked = 0
            limit = int(cfg.get("arm_check_top_k", 4))
            for row in sorted(passed, key=_sort_key):
                if checked >= limit:
                    break
                checked += 1
                try:
                    margin = arm_check(row.get("waypoints") or [],
                                       float(row.get("yaw") or 0.0))
                except Exception:  # noqa: BLE001 - arm check is advisory
                    margin = None
                if margin is None:
                    continue
                row["arm_margin_m"] = round(float(margin), 4)
                if margin < float(cfg.get("arm_min_margin_m", 0.0)):
                    row["feasible"] = False
                    row["reason"] = "arm_ceiling"
                score_row(row, cfg, section)
            passed = [row for row in passed if row.get("feasible")]
        if passed:
            winner = min(passed, key=_sort_key)
            winner["selected"] = True
            winner["path_source"] = winner.get("source")
            winner["segments"] = _carry_segments(
                winner, portal, start_joints, cfg["max_joint_step_rad"])
            winner["selection_id"] = _selection_id(
                surface, winner)
        else:
            reason = PLACE_PATH_INFEASIBLE
        self.last_diagnostics = diagnostics
        if winner is not None:
            winner["diagnostics"] = diagnostics
        return winner, kept, reason, snapshot

    def _legacy(self, surface, slots, portal, payload_wdh, expected_hash,
                expected_revision, opening_tangent, probe_fn, yaw):
        planner = PlacePathPlanner(
            weights=self.config,
            inflate_m=self.config.get("inflate_m"),
            arm_radius=self.config.get("arm_radius_m"),
            max_candidates_per_slot=self.config.get("max_candidates_per_slot"),
            cartesian_min_fraction=self.config.get("cartesian_min_fraction"))
        winner, rows, reason, snapshot = planner.plan(
            surface, slots, portal, payload_wdh,
            expected_hash=expected_hash, expected_revision=expected_revision,
            opening_tangent=opening_tangent, probe_fn=probe_fn, yaw=yaw)
        if winner is not None:
            winner["source"] = "geometric"
            winner["path_source"] = "legacy"
            winner["path_hash"] = _path_hash(winner)
            winner["segments"] = {
                "transit": _cartesian_spec(winner.get("waypoints")),
                "traverse": _cartesian_spec(winner.get("waypoints")),
            }
            winner["selection_id"] = _selection_id(surface, winner)
        self.last_diagnostics = {"mode": "legacy"}
        return winner, rows, reason, snapshot


def make_arm_ceiling_checker(seed_joints, ceiling_z, frame_convert=None,
                              link_names=("elfin_link3", "elfin_link4",
                                          "elfin_link5"),
                              sample_step=0.50, footprint=None):
    """Build ``arm_check(waypoints, yaw) -> margin_m`` via offline IK walk.

    The selector's sweep cannot see the arm body vs the container lid, so a
    winning diagonal can ride the wrist band over the rim (2026-09-27 box3:
    link z 2.02-2.06 vs lid 2.01). This checker walks the polyline with the
    deterministic refiner and returns ``ceiling_z - max(link z)`` in the FK
    frame, or ``None`` when the walk cannot converge (those rows keep the
    prefilter cost and stay MoveIt's to judge). ``frame_convert`` maps
    map-frame waypoints into the URDF/world frame. ``sample_step`` stays at
    waypoint-hop scale (~0.5 m): finer resampling lands intermediate points
    on reach-boundary saddles where the local solver stalls even though the
    corner-to-corner hops converge (2026-09-29 replay measurement).
    """
    state = {"urdf": None, "tcp": None, "links": None}

    def checker(waypoints, yaw=0.0):
        if not waypoints or not seed_joints:
            return None
        try:
            from luggage_planning.offline_ik import (
                load_urdf, refine_to_pose)
        except ImportError:
            return None
        if state["urdf"] is None:
            urdf = load_urdf()
            state["urdf"] = urdf
            state["tcp"] = urdf.path_to("suction_contact_frame")
            state["links"] = [urdf.path_to(name) for name in link_names]
        urdf, tcp, links = state["urdf"], state["tcp"], state["links"]
        convert = frame_convert or (lambda p: p)
        half = 0.5 * float(yaw)
        quat = (math.cos(half), math.sin(half), 0.0, 0.0)
        pts = []
        # Seed the walk at the CURRENT end-effector pose (FK of the live
        # joints): the first polyline corner can be a large reconfiguration
        # away (platform -> inside the door) and a cold refine to it diverges,
        # while stepping from where the arm actually is converges. Samples
        # up to the first converted waypoint keep the CURRENT orientation;
        # the slot yaw only has to hold from that corner on.
        start_fk = urdf.tcp([float(v) for v in seed_joints], state["tcp"])
        rot = start_fk[:3, :3]
        trace = max(-1.0, min(3.0, rot[0, 0] + rot[1, 1] + rot[2, 2]))
        w = math.sqrt(max(0.0, 1.0 + trace)) * 0.5
        carry_quat = (
            (rot[2, 1] - rot[1, 2]) / (4 * w) if w > 1e-9 else 0.0,
            (rot[0, 2] - rot[2, 0]) / (4 * w) if w > 1e-9 else 0.0,
            (rot[1, 0] - rot[0, 1]) / (4 * w) if w > 1e-9 else 0.0,
            w)
        first_world = convert(waypoints[0])
        ee = tuple(float(v) for v in start_fk[:3, 3])
        pts.append((ee, False))
        cross = math.sqrt(sum((first_world[i] - ee[i]) ** 2 for i in range(3)))
        for k in range(1, max(1, int(round(cross / float(sample_step)))) + 1):
            blend = k / max(1, int(round(cross / float(sample_step))))
            pts.append((tuple(ee[i] + (first_world[i] - ee[i]) * blend
                              for i in range(3)), False))
        for a, b in zip(waypoints, waypoints[1:]):
            a, b = convert(a), convert(b)
            seg = math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))
            steps = max(1, int(round(seg / float(sample_step))))
            for k in range(steps):
                pts.append((tuple(a[i] + (b[i] - a[i]) * k / steps
                                  for i in range(3)), True))
        pts.append((convert(waypoints[-1]), True))
        current = [float(v) for v in seed_joints]
        zmax = -1e9
        visited = 0
        skipped = 0
        for point, use_yaw in pts:
            target_quat = quat if use_yaw else carry_quat
            sol, _info = refine_to_pose(urdf, tcp, point, target_quat,
                                        seed=current)
            if sol is None:
                # A sample on a reach-boundary saddle stalls the local
                # solver even though corner hops converge; skip it and keep
                # the walk going. Too many skips => no measurement.
                skipped += 1
                if skipped > max(1, len(pts) // 3):
                    return None
                continue
            current = sol
            visited += 1
            for path in links:
                tf = urdf.tcp(sol, path)
                if footprint is not None:
                    # The lid exists only over the container footprint:
                    # link height outside it (over the pedestal/platform)
                    # is legitimate and must not count against the margin.
                    x, y = float(tf[0, 3]), float(tf[1, 3])
                    if not (footprint[0] <= x <= footprint[1]
                            and footprint[2] <= y <= footprint[3]):
                        continue
                zmax = max(zmax, float(tf[2, 3]))
        if visited < 2:
            return None
        return float(ceiling_z) - zmax

    return checker


def _cartesian_spec(waypoints, allow_ompl=True):
    return {
        "type": "cartesian",
        "waypoints": list(waypoints or []),
        "joint_knots": [],
        "allow_ompl_fallback": bool(allow_ompl),
    }


def _carry_segments(row, portal, start_joints, continuity):
    from luggage_planning.joint_path import max_joint_step
    waypoints = list(row.get("waypoints") or [])
    seeds = list(row.get("joint_seeds") or [])
    if row.get("source") != "grid" or not seeds:
        spec = _cartesian_spec(waypoints, allow_ompl=True)
        return {"transit": spec, "traverse": spec}
    traverse = {
        "type": "joint_path",
        "waypoints": waypoints,
        "joint_knots": seeds,
        "allow_ompl_fallback": False,
    }
    entry = waypoints[0] if waypoints else tuple(portal)
    portal_t = tuple(float(v) for v in portal)
    if _dist_ok(portal_t, entry):
        transit_wps = [portal_t, tuple(entry)]
    else:
        transit_wps = [tuple(entry)]
    knots = []
    if start_joints and max_joint_step(start_joints, seeds[0]) <= float(continuity) + 1e-9:
        knots = [[float(v) for v in start_joints], list(seeds[0])]
    if knots:
        transit = {
            "type": "joint_path",
            "waypoints": transit_wps,
            "joint_knots": knots,
            "allow_ompl_fallback": False,
        }
    else:
        transit = _cartesian_spec(transit_wps, allow_ompl=False)
    return {"transit": transit, "traverse": traverse}


def _dist_ok(a, b):
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3))) > 1e-3


def _selection_id(surface, winner):
    payload = {
        "geometry_hash": str((surface or {}).get("geometry_hash") or ""),
        "map_revision": (surface or {}).get("map_revision"),
        "path_hash": winner.get("path_hash"),
        "slot": winner.get("slot_index"),
    }
    raw = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(raw.encode("utf-8")).hexdigest()[:16]
