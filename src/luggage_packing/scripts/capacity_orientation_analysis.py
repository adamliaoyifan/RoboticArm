#!/usr/bin/env python3
"""Offline placement-capacity and box-orientation analysis (no ROS / Gazebo).

Answers two questions with the production placement stack, entirely offline
(the Gazebo slot is busy with other sampling and must not be touched):

  1. capacity    -- with current settings (2026-09-24 adaptive-lift payload
     atlases, production scorer, atlas hook hard_reject + w_reachability),
     how many boxes fit before solve_placement returns no feasible
     candidate, per mix (pure carryon / standard / large / catalog mix)?
  2. orientation -- "横着放" decomposed:
       (a) yaw-90 footprint swap (0.55x0.40 <-> 0.40x0.55): already
           supported by the solver and the atlas yaw bins; yaw-ablation
           measures its marginal value.
       (b) roll onto a side (vertical dim becomes D or W): NOT producible by
           the tool-down top-face pick chain and not modeled by any payload
           atlas variant; roll-whatif brackets the capacity between an
           optimistic (runtime file resolution, underestimates the vertical
           extent) and a pessimistic (strict UNKNOWN/UNREACHABLE reject)
           reachability view, with the atlas-free geometry ceiling above.

Production parity is the point: solve_placement + make_atlas_annotate +
placement_constraint_verdict are wired exactly like placement_planner_node
(floor 0.53 container-frame into the hook while the surface map stays
floor-relative; floor-prior surface map; validator closure). The scene
config is scene_tf.yaml.example because its md5 matches the atlas meta
scene_tf_hash (the non-example file differs only in comments but hashes
differently).

`calibrate` hard-asserts the parity against the 2026-09-24_1640 smoke1 slot
before anything else runs; a known production fact it encodes: the placed
slot was atlas_status=1 (UNREACHABLE) with neighbor_confidence 0.5 < 0.9,
so hard_reject_safe was False -- production "placeable" means NOT
hard_reject_safe, and the atlas is currently barely binding at the
placement layer.

Usage:
    python3 capacity_orientation_analysis.py calibrate
    python3 capacity_orientation_analysis.py all [--seeds 3] [--max-boxes 60]
"""

from __future__ import division

import os

# Single-threaded BLAS before numpy arrives (via the atlas loader): the FK/IK
# math here is tiny-matrix and otherwise spins every core for nothing.
for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS",
             "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ[_var] = "1"

import argparse  # noqa: E402
import datetime  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import sys  # noqa: E402

import yaml  # noqa: E402

_REPO = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
for _p in (
        os.path.join(_REPO, "src", "luggage_packing"),
        os.path.join(_REPO, "src", "luggage_planning"),
        os.path.join(_REPO, "src", "luggage_description"),
        os.path.join(_REPO, "src", "luggage_packing", "test", "harness"),
):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from luggage_packing.placement_solver import (  # noqa: E402
    candidate_aabb,
    generate_candidates,
    placement_constraint_verdict,
    solve_placement,
)
from luggage_packing.placement_reachability import make_atlas_annotate  # noqa: E402
from luggage_planning.atlas.runtime import (  # noqa: E402
    apply_query_semantics,
    compute_scene_tf_hash,
    load_atlas_runtime,
    load_runtime_atlas,
)
from luggage_description.container_geometry import (  # noqa: E402
    descriptor_from_scene_config,
    y_max_at_z,
)
from packing_sequences import (  # noqa: E402
    catalog_entries,
    generate_sequence,
    load_catalog,
)

DEFAULT_SCENE_CONFIG = os.path.join(
    _REPO, "src", "luggage_description", "config", "scene_tf.yaml.example")
DEFAULT_ATLAS_CONFIG = os.path.join(
    _REPO, "src", "luggage_planning", "config", "atlas_runtime.yaml")

# Parity anchors, all verified against placement_planner_node + atlas meta.
FLOOR_Z = 0.53
INNER_SIZE = [1.49, 1.97, 1.48]
APERTURE_Y = (-0.928, 0.398)
HULL_MARGIN = 0.0
SCENE_HASH_PREFIX = "b40c85ae"
HULL_HASH_PREFIX = "cb41f55f"

# The 2026-09-24_1640_newatlas_smoke1 placed slot (suite.json +
# dumps/box_00_paths/candidates.json): measured box size, winning suction
# waypoint in container_link, and the atlas verdict on it. RAW documents the
# iz1 floor false negative (cell centre 0.725 clips the slab); SEMANTIC is
# the query-layer fix (floor re-anchor to the first centre-clearing cell).
SMOKE1_SIZE = [0.4970396641980876, 0.35838427204770823, 0.2471675444924447]
SMOKE1_QUERY = (-0.495, -0.735, 0.7772, 0.0)
SMOKE1_EXPECT_RAW = {"status": 1, "cell": (1, 1, 1, 0),
                     "hard_reject_safe": False}
SMOKE1_EXPECT_SEM = {"min_status": 2, "cell": (1, 1, 2, 0)}

# placement_planner_node request parity: node params (:109-110, :166-177).
ALLOWED_YAWS = [0.0, math.pi / 2.0, math.pi, -math.pi / 2.0]
SOLVER_PARAMS = {
    "clearance_margin": 0.03,
    "min_support_ratio": 0.6,
    "top_n": 2000,
    "keep_rejected": 200,
}

CATALOG = [
    ("carryon", [0.55, 0.40, 0.25]),
    ("standard", [0.70, 0.45, 0.28]),
    ("large", [0.80, 0.50, 0.32]),
]

ATLAS_MODES = ("production", "strict", "none")


# ----------------------------------------------------------------------
# Scene context (ROS-free replacement of the node's scene_tf_config_utils)

class SceneContext(object):
    def __init__(self, scene_config_path, atlas_config_path):
        with open(scene_config_path, "r") as handle:
            self.scene = yaml.safe_load(handle)
        self.hull = descriptor_from_scene_config(self.scene)
        inner = self.scene["container"]["inner"]
        self.inner_size = [
            float(inner["length"]), float(inner["width"]),
            float(inner["ceiling_z"]) - float(inner["floor_z"])]
        self.floor_z = float(inner["floor_z"])
        corners = self.scene["container"]["opening"]["aperture"]["corners"]
        ys = [float(corner[1]) for corner in corners]
        self.aperture_y = (min(ys), max(ys))
        self.scene_hash = compute_scene_tf_hash(scene_config_path)
        self.atlas_cfg = load_atlas_runtime(atlas_config_path)
        self.scene_config_path = scene_config_path
        self.atlas_config_path = atlas_config_path

    def check_constants(self):
        """Assert the numbers this analysis hardcodes match the scene yaml."""
        checks = {
            "floor_z": (self.floor_z, FLOOR_Z),
            "inner_size": (self.inner_size, INNER_SIZE),
            "aperture_y": (self.aperture_y, APERTURE_Y),
        }
        problems = []
        for name, (got, want) in sorted(checks.items()):
            got_r = [_round(v) for v in _flatten(got)]
            want_r = [_round(v) for v in _flatten(want)]
            if got_r != want_r:
                problems.append("%s: got %s want %s" % (name, got_r, want_r))
        if not str(self.scene_hash).startswith(SCENE_HASH_PREFIX):
            problems.append("scene hash %s != %s..."
                            % (self.scene_hash, SCENE_HASH_PREFIX))
        if not str(self.hull.geometry_hash).startswith(HULL_HASH_PREFIX):
            problems.append("hull hash %s != %s..."
                            % (self.hull.geometry_hash, HULL_HASH_PREFIX))
        return problems


def _flatten(value):
    if isinstance(value, (list, tuple)):
        out = []
        for item in value:
            out.extend(_flatten(item))
        return out
    return [value]


def _round(value):
    return round(float(value), 4)


# ----------------------------------------------------------------------
# Atlas store: production resolution per box size + the strict bracket.

class StrictAtlas(object):
    """Pessimistic wrapper: UNKNOWN/UNREACHABLE cells become hard rejects.

    Offline stand-in for the MoveIt filter that normally gets the final
    word. Production keeps those candidates (hard_reject_safe is False for
    low-confidence cells -- the smoke1 slot proves it); strict closes that
    escape hatch, so capacity under it is a lower bracket.
    """

    def __init__(self, inner):
        self._inner = inner

    @property
    def yaw_bins(self):
        return self._inner.yaw_bins

    def query(self, x, y, z, yaw=0.0, yaw_tolerance=None):
        result = self._inner.query(x, y, z, yaw, yaw_tolerance)
        if int(result.status) in (0, 1):
            # QueryResult is a namedtuple: rebuild with the flipped flag.
            return result._replace(hard_reject_safe=True)
        return result


class AtlasStore(object):
    """Per-size atlas handles through the production resolution order."""

    def __init__(self, ctx):
        self._cfg = ctx.atlas_cfg
        self._hash = ctx.scene_hash
        self._cache = {}

    def get(self, size, mode):
        if mode == "none":
            return None, {"mode": "none", "npz": None}
        key = (tuple(_round(v) for v in size), mode)
        if key not in self._cache:
            atlas, diag = load_runtime_atlas(
                self._cfg, box_size=list(key[0]), scene_tf_hash=self._hash)
            diag = dict(diag)
            diag["npz_basename"] = (
                os.path.basename(diag["npz"]) if diag.get("npz") else None)
            # "current settings" include the runtime query semantics (floor
            # re-anchoring + boundary promotion), same wrap as both nodes.
            if atlas is not None:
                atlas = apply_query_semantics(
                    atlas, self._cfg, floor_z=FLOOR_Z,
                    box_height=float(key[0][2]))
                diag["query_semantics"] = True
            if mode == "strict" and atlas is not None:
                atlas = StrictAtlas(atlas)
                diag["strict"] = True
            self._cache[key] = (atlas, diag)
        return self._cache[key]


# ----------------------------------------------------------------------
# Greedy capacity loop (placement_planner_node parity, offline).

def floor_prior_surface(ctx):
    """Replicate the node's _floor_prior_surface (floor-relative heights)."""
    res = 0.05
    inner_l, inner_w, inner_h = ctx.inner_size
    nx = max(1, int(round(inner_l / res)))
    ny = max(1, int(round(inner_w / res)))
    return {
        "geometry_hash": str(ctx.hull.geometry_hash),
        "geometry_descriptor": dict(ctx.hull.descriptor()),
        "resolution": res,
        "nx": nx, "ny": ny,
        "inner_size": [inner_l, inner_w, inner_h],
        "floor_z": 0.0,
        "center_base": [0.0, 0.0, ctx.floor_z + 0.5 * inner_h],
        "yaw": 0.0,
        "height": [[0.0] * ny for _ in range(nx)],
        "state": [["unknown"] * ny for _ in range(nx)],
        "clearance": [[inner_h] * ny for _ in range(nx)],
        "known_ratio": [[0.0] * ny for _ in range(nx)],
        "confidence": [["none"] * ny for _ in range(nx)],
    }


def make_validator(ctx, placed_aabbs, smallest_size):
    """Node _constraint_verdict parity (:399-428), offline."""
    hull = ctx.hull
    floor_z = ctx.floor_z

    def hull_contains_floor_relative(point):
        # Lateral clearance only: a floor-resting box touches the floor by
        # design, so the margin applies to walls/ceiling, never the floor.
        x, y, z_rel = point
        z = z_rel + floor_z
        if not (float(hull.floor_z) - 1e-9 <= z
                <= float(hull.ceiling_z) - HULL_MARGIN):
            return False
        if not (-hull.half_x + HULL_MARGIN <= x <= hull.half_x - HULL_MARGIN):
            return False
        if y < -hull.half_y + HULL_MARGIN:
            return False
        return y <= y_max_at_z(hull, z, margin=HULL_MARGIN) + 1e-9

    def validator(candidate):
        return placement_constraint_verdict(
            candidate,
            ctx.inner_size[2],
            placed_aabbs=placed_aabbs,
            aperture_y=ctx.aperture_y,
            hull_contains=hull_contains_floor_relative,
            inner_size=ctx.inner_size,
            smallest_size=smallest_size,
            hull=hull,
            floor_z=ctx.floor_z,
        )

    return validator


def make_hook(ctx, atlas):
    placement = ctx.atlas_cfg.get("placement") or {}
    return make_atlas_annotate(
        atlas,
        ctx.floor_z,
        ctx.inner_size[2],
        score_weight=float(placement.get("w_reachability", 0.3)),
        hard_reject=bool(placement.get("hard_reject", True)),
        dump_only=False,
        query_offset=ctx.atlas_cfg.get("query_offset"),
    )


def apply_placement(surface, selected, inflation):
    """Raise the height map under the footprint (free_space_model indexing)."""
    inner_h = float(surface["inner_size"][2])
    inner_l = float(surface["inner_size"][0])
    inner_w = float(surface["inner_size"][1])
    res = float(surface["resolution"])
    nx, ny = int(surface["nx"]), int(surface["ny"])
    fl, fw = [float(v) for v in selected["footprint"]]
    bh = float(selected["size"][2])
    lx, ly = float(selected["center_local"][0]), float(selected["center_local"][1])
    peak = float(selected["center_local"][2]) + inner_h * 0.5 - bh * 0.5
    top = peak + bh + float(inflation)
    ix0 = max(0, int((lx - fl * 0.5 + inner_l * 0.5) / res))
    ix1 = min(nx, int(math.ceil((lx + fl * 0.5 + inner_l * 0.5) / res)))
    iy0 = max(0, int((ly - fw * 0.5 + inner_w * 0.5) / res))
    iy1 = min(ny, int(math.ceil((ly + fw * 0.5 + inner_w * 0.5) / res)))
    for ix in range(ix0, ix1):
        for iy in range(iy0, iy1):
            if surface["height"][ix][iy] < top:
                surface["height"][ix][iy] = top
            surface["state"][ix][iy] = "occupied"
            surface["confidence"][ix][iy] = "sensor"
    return peak, top


def run_capacity(ctx, store, sequence, atlas_mode="production", yaws=None,
                 height_inflation=0.0, skip_on_fail=False):
    """Greedy sequential placement until solve_placement fails.

    Returns {"placed": [records], "failure": record_or_None}. Every record
    carries the atlas verdict for the selected slot, so the report can show
    how binding the atlas actually was.
    """
    surface = floor_prior_surface(ctx)
    inner_h = ctx.inner_size[2]
    placed_aabbs = []
    records = []
    failure = None
    yaws = ALLOWED_YAWS if yaws is None else list(yaws)
    for idx, raw_size in enumerate(sequence):
        size = [float(v) for v in raw_size]
        atlas, diag = store.get(size, atlas_mode)
        hook = make_hook(ctx, atlas)
        validator = make_validator(ctx, placed_aabbs, size)
        result = solve_placement(
            surface, size, allowed_yaws=yaws, params=dict(SOLVER_PARAMS),
            candidate_validator=validator, atlas_annotate=hook)
        if not result["success"]:
            failure = {
                "index": idx,
                "size": [_round(v) for v in size],
                "reason_code": result["reason_code"],
                "message": result["message"],
                "reject_histogram": result["reject_histogram"],
                "candidates_enumerated": result["candidates_enumerated"],
                "capacity_feasible_count": result["capacity_feasible_count"],
                "atlas": result["atlas"],
                "atlas_npz": diag.get("npz_basename"),
            }
            if not skip_on_fail:
                break
            continue
        selected = result["selected"]
        peak, top = apply_placement(surface, selected, height_inflation)
        placed_aabbs.append(candidate_aabb(selected, inner_h))
        records.append({
            "index": idx,
            "size": [_round(v) for v in size],
            "box_yaw": _round(selected["box_yaw"]),
            "center_local": [_round(v) for v in selected["center_local"]],
            "peak": _round(peak),
            "top": _round(top),
            "support_source": selected.get("support_source"),
            "score": selected.get("score"),
            "support_score": selected.get("support_score"),
            "atlas_status": selected.get("atlas_status"),
            "atlas_status_name": selected.get("atlas_status_name"),
            "atlas_contact_z": selected.get("atlas_contact_z"),
            "reachability_prior": selected.get("reachability_prior"),
            "n_feasible": sum(
                1 for c in result["candidates"] if c.get("feasible")),
            "candidates_enumerated": result["candidates_enumerated"],
            "capacity_feasible_count": result["capacity_feasible_count"],
        })
    return {"placed": records, "failure": failure}


def summarize_run(run):
    """Per-run aggregates reused by the report."""
    placed = run["placed"]
    n = len(placed)
    floor_n = sum(1 for r in placed if r["support_source"] == "floor_prior")
    layers = {}
    for r in placed:
        key = _round(r["peak"])
        layers[key] = layers.get(key, 0) + 1
    status_hist = {}
    for r in placed:
        key = str(r["atlas_status_name"])
        status_hist[key] = status_hist.get(key, 0) + 1
    yaw90 = sum(1 for r in placed if abs(r["box_yaw"]) > 1.0)
    out = {
        "n_placed": n,
        "floor_placed": floor_n,
        "stacked_placed": n - floor_n,
        "peak_histogram": dict(sorted(layers.items())),
        "atlas_status_histogram": status_hist,
        "selected_yaw90_share": _round(yaw90 / n) if n else None,
    }
    if run["failure"] is not None:
        out["failure"] = {
            "at_box": run["failure"]["index"],
            "reason_code": run["failure"]["reason_code"],
            "reject_histogram": run["failure"]["reject_histogram"],
            "capacity_feasible_count": run["failure"]["capacity_feasible_count"],
            "candidates_enumerated": run["failure"]["candidates_enumerated"],
        }
    else:
        out["failure"] = None
    return out


# ----------------------------------------------------------------------
# Subcommands.

def cmd_calibrate(ctx, store, out_dir):
    """Hard parity assertions against smoke1 + scene constants."""
    report = {"checks": [], "ok": True}

    def check(name, ok, detail):
        report["checks"].append(
            {"name": name, "ok": bool(ok), "detail": detail})
        if not ok:
            report["ok"] = False

    problems = ctx.check_constants()
    check("scene_constants", not problems, problems or "matched")

    atlas_raw, raw_diag = load_runtime_atlas(
        ctx.atlas_cfg, box_size=SMOKE1_SIZE, scene_tf_hash=ctx.scene_hash)
    check("atlas_resolved", atlas_raw is not None,
          {"npz": os.path.basename(raw_diag.get("npz") or ""),
           "reason": raw_diag.get("reason")})
    raw = atlas_raw.query(*SMOKE1_QUERY)
    check("smoke1_raw_query_documents_artifact",
          int(raw.status) == SMOKE1_EXPECT_RAW["status"]
          and tuple(raw.indices) == SMOKE1_EXPECT_RAW["cell"],
          {"status": int(raw.status), "indices": [int(v) for v in raw.indices],
           "neighbor_confidence": getattr(raw, "neighbor_confidence", None),
           "hard_reject_safe": bool(raw.hard_reject_safe)})
    sem = apply_query_semantics(
        atlas_raw, ctx.atlas_cfg, floor_z=FLOOR_Z,
        box_height=float(SMOKE1_SIZE[2]))
    got = sem.query(*SMOKE1_QUERY)
    check("smoke1_semantic_query_fixed",
          int(got.status) >= SMOKE1_EXPECT_SEM["min_status"]
          and tuple(got.indices) == SMOKE1_EXPECT_SEM["cell"],
          {"status": int(got.status), "reason": str(got.reason),
           "indices": [int(v) for v in got.indices]})

    surface = floor_prior_surface(ctx)
    atlas, diag = store.get(SMOKE1_SIZE, "production")
    result = solve_placement(
        surface, SMOKE1_SIZE, allowed_yaws=ALLOWED_YAWS,
        params=dict(SOLVER_PARAMS),
        candidate_validator=make_validator(ctx, [], SMOKE1_SIZE),
        atlas_annotate=make_hook(ctx, atlas))
    contact_ok = False
    if result["success"]:
        selected = result["selected"]
        contact_z = float(selected.get("atlas_contact_z") or -1.0)
        contact_ok = abs(contact_z - SMOKE1_QUERY[2]) <= 0.075
        check("pipeline_floor_prior",
              selected.get("support_source") == "floor_prior",
              {"support_source": selected.get("support_source")})
        check("pipeline_contact_z", contact_ok,
              {"atlas_contact_z": contact_z,
               "expected": SMOKE1_QUERY[2], "tol": 0.075})
        check("pipeline_no_hard_reject",
              (result["atlas"] or {}).get("n_hard_rejected") == 0,
              {"atlas_report": result["atlas"]})
    else:
        check("pipeline_floor_prior", False,
              {"reason_code": result["reason_code"],
               "message": result["message"]})

    path = os.path.join(out_dir, "calibrate.json")
    with open(path, "w") as handle:
        json.dump(report, handle, indent=1, sort_keys=True)
    return report


def cmd_capacity(ctx, store, args, out_dir):
    mixes = []
    for name, size in CATALOG:
        mixes.append(("%s_only" % name, [list(size)] * args.max_boxes))
    for seed in range(args.seeds):
        seq = generate_sequence(seed, args.max_boxes, size_mode="catalog")
        mixes.append(("catalog_mix_seed%d" % seed, seq))

    runs = {}
    # Production bracket for every mix. Timing stays out of the dump so the
    # artifact is byte-reproducible.
    for label, seq in mixes:
        run = run_capacity(ctx, store, seq, atlas_mode="production",
                           height_inflation=args.height_inflation)
        runs["production/%s" % label] = dict(run, summary=summarize_run(run))
    # Negative/ceiling controls on carryon: strict <= production <= none.
    carryon_seq = [list(CATALOG[0][1])] * args.max_boxes
    for mode in ("strict", "none"):
        run = run_capacity(ctx, store, carryon_seq, atlas_mode=mode,
                           height_inflation=args.height_inflation)
        runs["%s/carryon_only" % mode] = dict(run, summary=summarize_run(run))
    return runs


def cmd_yaw_ablation(ctx, store, args, out_dir):
    out = {"capacity": {}, "empty_container": {}, "atlas_per_yaw": {}}
    mixes = []
    for name, size in CATALOG:
        mixes.append(("%s_only" % name, [list(size)] * args.max_boxes))
    for seed in range(args.seeds):
        seq = generate_sequence(seed, args.max_boxes, size_mode="catalog")
        mixes.append(("catalog_mix_seed%d" % seed, seq))

    for label, seq in mixes:
        for tag, yaws in (("yaw0", [0.0]), ("yaw0+90", [0.0, math.pi / 2.0])):
            run = run_capacity(ctx, store, seq, atlas_mode="production",
                               yaws=yaws, height_inflation=args.height_inflation)
            out["capacity"]["%s/%s" % (label, tag)] = summarize_run(run)

    # Empty-container per-yaw candidate counts, untruncated.
    surface = floor_prior_surface(ctx)
    params = dict(SOLVER_PARAMS)
    params.update({"top_n": 4000, "keep_rejected": 4000})
    for name, size in CATALOG:
        for tag, yaw in (("yaw0", 0.0), ("yaw90", math.pi / 2.0)):
            totals = {}
            cands = generate_candidates(
                surface, size, allowed_yaws=[yaw], params=params,
                candidate_validator=make_validator(ctx, [], size),
                totals=totals)
            out["empty_container"]["%s/%s" % (name, tag)] = {
                "n_enumerated": totals.get("n_enumerated"),
                "n_capacity_feasible": totals.get("n_capacity_feasible"),
                "n_feasible": sum(1 for c in cands if c.get("feasible")),
            }

    # Atlas-side coverage per yaw bin, straight from the resolved npz.
    import numpy as np
    for name, size in CATALOG:
        atlas, diag = store.get(size, "production")
        if atlas is None or not diag.get("npz"):
            continue
        with np.load(diag["npz"]) as data:
            status = data["status"]
        per_yaw = [
            int((status[..., k] >= 2).sum()) for k in range(status.shape[-1])]
        out["atlas_per_yaw"][name] = {
            "npz": diag["npz_basename"],
            "reachable_or_marginal_per_yaw_bin": per_yaw,
        }
    return out


def cmd_roll_whatif(ctx, store, args, out_dir):
    """Geometry what-if for side-rolled boxes, three reachability brackets.

    Permutations of [W, D, H]: upright keeps H vertical; "d_up" and "w_up"
    roll the box so D or W is vertical. The solver/validators are size-driven
    so nothing else changes. The roll is NOT producible by the current
    tool-down top-face pick chain -- these numbers say whether it would be
    worth building that chain (side-face attach + rolled atlas variants).
    """
    out = {}
    for name, size in CATALOG:
        w, d, h = size
        variants = {
            "upright_h%d" % int(h * 100): [w, d, h],
            "rolled_d%d" % int(d * 100): [w, h, d],
            "rolled_w%d" % int(w * 100): [d, h, w],
        }
        for vname, vsize in sorted(variants.items()):
            for mode in ATLAS_MODES:
                atlas, diag = store.get(vsize, mode)
                run = run_capacity(
                    ctx, store, [list(vsize)] * args.max_boxes,
                    atlas_mode=mode,
                    height_inflation=args.height_inflation)
                summary = summarize_run(run)
                summary["atlas_npz"] = diag.get("npz_basename")
                out["%s/%s/%s" % (name, vname, mode)] = summary
    return out


# ----------------------------------------------------------------------
# Report.

def _failure_line(summary):
    if not summary or summary.get("failure") is None:
        return "sequence exhausted"
    f = summary["failure"]
    top = sorted((f.get("reject_histogram") or {}).items(),
                 key=lambda kv: -kv[1])[:2]
    gates = ", ".join("%s=%d" % kv for kv in top) if top else "no rejects kept"
    return "%s @box %d (capacity_feasible %d/%d; %s)" % (
        f["reason_code"], f["at_box"], f["capacity_feasible_count"],
        f["candidates_enumerated"], gates)


def build_report(ctx, cal, cap, yaw, roll, args):
    lines = []
    add = lines.append
    add("# Placement capacity + orientation analysis (offline)")
    add("")
    add("Generated by `src/luggage_packing/scripts/capacity_orientation_analysis.py` "
        "on scene `%s`, atlas config `%s`."
        % (os.path.basename(ctx.scene_config_path),
           os.path.basename(ctx.atlas_config_path)))
    add("")
    add("Production parity: `solve_placement` + atlas hook (floor_z %.2f, "
        "hard_reject=%s, w_reachability=%s) + constraint validator, "
        "floor-prior 0.05 m surface."
        % (ctx.floor_z,
           bool((ctx.atlas_cfg.get("placement") or {}).get("hard_reject", True)),
           (ctx.atlas_cfg.get("placement") or {}).get("w_reachability", 0.3)))
    add("")
    add("## Calibration")
    add("")
    for check in cal["checks"]:
        add("- %s **%s**: %s" % (
            "PASS" if check["ok"] else "FAIL", check["name"],
            json.dumps(check["detail"], sort_keys=True)))
    add("")
    add("## Capacity (current settings, per mix)")
    add("")
    add("| mix | boxes placed | floor/stacked | binding at exhaustion |")
    add("|---|---|---|---|")
    for key in sorted(k for k in cap if k.startswith("production/")):
        s = cap[key]["summary"]
        add("| %s | %d | %d/%d | %s |" % (
            key[len("production/"):], s["n_placed"], s["floor_placed"],
            s["stacked_placed"], _failure_line(s)))
    add("")
    add("Controls (carryon): strict %d <= production %d <= none %d."
        % (cap["strict/carryon_only"]["summary"]["n_placed"],
           cap["production/carryon_only"]["summary"]["n_placed"],
           cap["none/carryon_only"]["summary"]["n_placed"]))
    add("")
    add("## Yaw-90 marginal value (already supported in production)")
    add("")
    add("| mix | boxes @ yaw0 | boxes @ yaw0+90 | selected yaw90 share |")
    add("|---|---|---|---|")
    labels = sorted({key.rsplit("/", 1)[0] for key in yaw["capacity"]})
    for label in labels:
        a = yaw["capacity"].get("%s/yaw0" % label)
        b = yaw["capacity"].get("%s/yaw0+90" % label)
        share = b.get("selected_yaw90_share") if b else None
        add("| %s | %s | %s | %s |" % (
            label,
            a["n_placed"] if a else "-",
            b["n_placed"] if b else "-",
            share if share is not None else "-"))
    add("")
    add("Empty-container candidates per yaw and atlas per-yaw-bin coverage:")
    add("")
    add("| size | yaw0 feas/enumerated | yaw90 feas/enumerated | atlas bins {2,3} |")
    add("|---|---|---|---|")
    for name, _size in CATALOG:
        e0 = yaw["empty_container"].get("%s/yaw0" % name)
        e90 = yaw["empty_container"].get("%s/yaw90" % name)
        ay = yaw["atlas_per_yaw"].get(name, {}).get(
            "reachable_or_marginal_per_yaw_bin")
        add("| %s | %s/%s | %s/%s | %s |" % (
            name,
            e0["n_feasible"] if e0 else "-",
            e0["n_enumerated"] if e0 else "-",
            e90["n_feasible"] if e90 else "-",
            e90["n_enumerated"] if e90 else "-",
            ay or "-"))
    add("")
    add("## Roll what-if (NOT producible today: tool-down top-face pick)")
    add("")
    add("| variant | naive | strict | geometry ceiling |")
    add("|---|---|---|---|")
    for name, _size in CATALOG:
        w, d, h = _size
        for vname in ("upright_h%d" % int(h * 100),
                      "rolled_d%d" % int(d * 100),
                      "rolled_w%d" % int(w * 100)):
            row = []
            for mode in ("production", "strict", "none"):
                key = "%s/%s/%s" % (name, vname, mode)
                row.append(roll[key]["n_placed"] if key in roll else "-")
            add("| %s/%s | %s | %s | %s |" % (name, vname, row[0], row[1], row[2]))
    add("")
    add("`naive` = the atlas file the runtime would resolve for the rolled "
        "size (falls back to the largest variant, which underestimates the "
        "vertical extent -> optimistic). `strict` = every UNKNOWN/"
        "UNREACHABLE cell hard-rejected (pessimistic). `geometry ceiling` = "
        "no atlas at all.")
    add("")
    add("## Parity approximations")
    add("")
    add("- Perception idealized: exact box-top heights, full-footprint "
        "coverage, `occupied/sensor` cells; live mapper has partial coverage "
        "and COVERAGE_LOW failures. Sensitivity: `--height-inflation %s`."
        % args.height_inflation)
    add("- No motion stack: the atlas is the only reachability proxy here "
        "and (calibration shows) barely binding at the placement layer -- "
        "production `placeable` means NOT `hard_reject_safe`. This capacity "
        "is a placement-policy upper bound on the full chain; the sim's "
        "transit `outside_hull` sweep can still reject planned paths.")
    add("- Floor prior 30x39 cells vs the live mapper's 30x40 (<= 1 Y row).")
    add("- Aperture gate is the z-independent Y shadow from the -X door "
        "(production behavior, conservative for deep slots).")
    add("- The old B4 replay (~9.95 items, continuous sizes, pre-rebuild "
        "atlas; its yaml was removed in the 2026-09-24 stale-data cleanup) "
        "used a different scorer: order-of-magnitude context only, not an "
        "anchor.")
    return "\n".join(lines) + "\n"


# ----------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "command", choices=("calibrate", "capacity", "yaw-ablation",
                            "roll-whatif", "all"))
    parser.add_argument("--scene-config", default=DEFAULT_SCENE_CONFIG)
    parser.add_argument("--atlas-config", default=DEFAULT_ATLAS_CONFIG)
    parser.add_argument("--out-dir", default=None)
    parser.add_argument("--seeds", type=int, default=3)
    parser.add_argument("--max-boxes", type=int, default=60)
    parser.add_argument("--height-inflation", type=float, default=0.0)
    parser.add_argument(
        "--skip-on-fail", action="store_true",
        help="keep trying after the first infeasible box (secondary metric)")
    args = parser.parse_args(argv)

    out_dir = args.out_dir
    if out_dir is None:
        stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M")
        out_dir = os.path.join(
            _REPO, "docs", "status", "evidence", "capacity_orientation",
            "%s_run1" % stamp)
    os.makedirs(out_dir, exist_ok=True)

    ctx = SceneContext(args.scene_config, args.atlas_config)
    store = AtlasStore(ctx)

    if args.command == "calibrate":
        report = cmd_calibrate(ctx, store, out_dir)
        for check in report["checks"]:
            print("%s %-28s %s" % (
                "PASS" if check["ok"] else "FAIL", check["name"],
                json.dumps(check["detail"], sort_keys=True)[:120]))
        print("calibrate: %s (%s)" % (
            "OK" if report["ok"] else "FAILED",
            os.path.join(out_dir, "calibrate.json")))
        return 0 if report["ok"] else 1

    if args.command in ("capacity", "all"):
        cal = cmd_calibrate(ctx, store, out_dir)
        if not cal["ok"]:
            print("calibration failed; refusing to score (see calibrate.json)")
            return 1
        cap = cmd_capacity(ctx, store, args, out_dir)
        if args.command == "capacity":
            results = {"calibrate": cal, "capacity": cap}
        else:
            yaw = cmd_yaw_ablation(ctx, store, args, out_dir)
            roll = cmd_roll_whatif(ctx, store, args, out_dir)
            results = {"calibrate": cal, "capacity": cap, "yaw": yaw,
                       "roll": roll}
    else:
        cal = cmd_calibrate(ctx, store, out_dir)
        if not cal["ok"]:
            print("calibration failed; refusing to score (see calibrate.json)")
            return 1
        cap = None
        if args.command == "yaw-ablation":
            yaw = cmd_yaw_ablation(ctx, store, args, out_dir)
            roll = None
        else:
            yaw = None
            roll = cmd_roll_whatif(ctx, store, args, out_dir)
        results = {"calibrate": cal, "capacity": cap, "yaw": yaw, "roll": roll}

    with open(os.path.join(out_dir, "results.yaml"), "w") as handle:
        yaml.safe_dump(results, handle, sort_keys=True)
    report = build_report(ctx, results["calibrate"],
                          results["capacity"] or {}, results["yaw"] or {},
                          results["roll"] or {}, args)
    with open(os.path.join(out_dir, "report.md"), "w") as handle:
        handle.write(report)
    print("wrote %s" % out_dir)
    if results["capacity"]:
        for key in sorted(results["capacity"]):
            s = results["capacity"][key]["summary"]
            print("%-36s placed=%-3d %s" % (key, s["n_placed"],
                                            _failure_line(s)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
