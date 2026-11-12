#!/usr/bin/env python3
"""Fixed-pose audit of the offline matrix atlas query yaw.

The builder stores directed tool yaw in ``container_link``
(``atlas_frame_contracts.tool_target_in_base``). Production recovers that
yaw with ``tool_down_yaw`` minus the container yaw in world
(``motion_planner_node._container_tool_yaw``). The matrix does the same
subtraction on the goal quaternion it actually builds.

This probe does not change planning. It reports, for one pose per directed
bin:

- the yaw the matrix passes to ``ReachabilityAtlas.query``;
- the production recovery of that same quaternion;
- the builder yaw that reproduces that tool-down rotation;
- the world yaw, which is a different directed bin when the container yaw
  in world is pi.
"""
import json
import math
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _entry in (
        _ROOT,
        os.path.join(_ROOT, "src", "luggage_planning"),
        os.path.join(_ROOT, "src", "luggage_description"),
        os.path.join(_ROOT, "scripts", "st09_i")):
    if _entry not in sys.path:
        sys.path.insert(0, _entry)

from luggage_planning.atlas.frame_contracts import (  # noqa: E402
    tool_target_in_base,
    wrapped_yaw_error,
)
from luggage_planning.waypoint_generator import tool_down_yaw  # noqa: E402
from route_matrix_eval import (  # noqa: E402
    TOOL_DOWN_QUAT,
    _qmul,
    _rot,
    _yaw_quat,
)

DIRECTED_BINS = (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)


def _wrap_pi(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def matrix_goal_quat(box_yaw_world):
    """Goal orientation ``route_matrix_eval`` builds for a world box yaw."""
    return _qmul(_yaw_quat(float(box_yaw_world)), TOOL_DOWN_QUAT)


def matrix_query_yaw(box_yaw_world, scene_yaw):
    """Container-frame yaw passed to ``atlas.query`` for that goal."""
    rotation = _rot(matrix_goal_quat(box_yaw_world))
    world_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return world_yaw - float(scene_yaw), world_yaw


def production_query_yaw(quaternion, scene_yaw):
    """``tool_down_yaw(x, y) - container_yaw_in_world``."""
    world_yaw = tool_down_yaw(quaternion[0], quaternion[1])
    return world_yaw - float(scene_yaw), world_yaw


def builder_yaw_for_same_rotation(container_yaw, scene_yaw):
    """Yaw argument whose builder pose matches ``container_yaw``.

    Returns the recovered container yaw. It equals ``container_yaw`` when
    the builder's tool-down convention is inverted correctly.
    """
    _position, quaternion = tool_target_in_base(
        0.0, 0.0, 1.0, float(container_yaw), (0.0, 0.0, 0.0),
        (0.0, 0.0, float(scene_yaw)))
    recovered, _world = production_query_yaw(quaternion, scene_yaw)
    return recovered


def fixed_pose_report(scene_yaw):
    """One row per directed bin at a fixed container mounting."""
    rows = []
    for bin_yaw in DIRECTED_BINS:
        # The matrix heading is this bin. The quaternion, not the heading
        # label, is what the query consumes.
        box_yaw_world = float(scene_yaw) + float(bin_yaw)
        quaternion = matrix_goal_quat(box_yaw_world)
        matrix_yaw, world_yaw = matrix_query_yaw(box_yaw_world, scene_yaw)
        production_yaw, production_world = production_query_yaw(
            quaternion, scene_yaw)
        builder_yaw = builder_yaw_for_same_rotation(matrix_yaw, scene_yaw)
        rows.append({
            "heading_parameter_rad": bin_yaw,
            "matrix_query_yaw_rad": matrix_yaw,
            "production_query_yaw_rad": production_yaw,
            "builder_recovered_yaw_rad": builder_yaw,
            "world_tool_yaw_rad": world_yaw,
            "production_world_yaw_rad": production_world,
            "matrix_matches_production": wrapped_yaw_error(
                matrix_yaw, production_yaw) < 1e-9,
            "matrix_matches_builder": wrapped_yaw_error(
                matrix_yaw, builder_yaw) < 1e-9,
            "world_yaw_is_other_bin": abs(
                wrapped_yaw_error(world_yaw, matrix_yaw) - math.pi) < 1e-6,
            "heading_parameter_is_other_bin": abs(
                wrapped_yaw_error(bin_yaw, _wrap_pi(matrix_yaw)) - math.pi)
                < 1e-6,
        })
    return {
        "scene_yaw_rad": float(scene_yaw),
        "query_frame": "container_link tool yaw",
        "matrix_tool_down_xyzw": list(TOOL_DOWN_QUAT),
        "builder_tool_down_xyzw": [1.0, 0.0, 0.0, 0.0],
        "convention_mismatch": not all(
            row["matrix_matches_production"] and row["matrix_matches_builder"]
            and row["world_yaw_is_other_bin"]
            for row in rows),
        "rows": rows,
    }


def atlas_cell_report(scene_yaw, atlas_stem):
    """Seed counts at one interior cell for container yaw versus world yaw."""
    from luggage_planning.atlas.model import ReachabilityAtlas

    atlas = ReachabilityAtlas.load(atlas_stem + ".npz", atlas_stem + ".yaml")
    origin = tuple(float(v) for v in atlas.meta["grid"]["origin"])
    resolution = float(atlas.meta["grid"]["resolution_xyz"])
    # Cell (0, 2, 2) is reachable on every directed bin for the 0.70 payload.
    index = (0, 2, 2)
    point = tuple(
        origin[i] + (index[i] + 0.5) * resolution for i in range(3))
    matrix_yaw, world_yaw = matrix_query_yaw(scene_yaw + 0.0, scene_yaw)
    counts = {}
    for name, yaw in (
            ("matrix_heading0", matrix_yaw),
            ("world_tool_yaw", world_yaw),
            ("builder_bin_0", 0.0),
            ("builder_bin_pi", math.pi)):
        result = atlas.query(point[0], point[1], point[2], yaw)
        counts[name] = {
            "yaw_rad": float(yaw),
            "status": result.reason,
            "yaw_index": None if result.indices is None else int(
                result.indices[3]),
            "contact_seeds": len(result.contact_seeds or []),
        }
    return {
        "atlas_stem": os.path.basename(atlas_stem),
        "container_base_rpy": list(
            atlas.meta.get("container", {}).get("base_rpy") or []),
        "yaw_semantics": atlas.meta["grid"].get("yaw_semantics"),
        "cell_index": list(index),
        "cell_xyz": list(point),
        "queries": counts,
    }


def main():
    scene_yaw = math.pi
    report = fixed_pose_report(scene_yaw)
    stem = os.path.join(
        _ROOT, "src", "luggage_planning", "data", "reachability_atlas",
        "s20_container_collision_aware_payload_0.70x0.45x0.28")
    if os.path.isfile(stem + ".npz"):
        report["atlas_cell"] = atlas_cell_report(scene_yaw, stem)
    json.dump(report, sys.stdout, indent=1)
    sys.stdout.write("\n")
    return 1 if report["convention_mismatch"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
