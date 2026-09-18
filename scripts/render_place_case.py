#!/usr/bin/env python3
"""Render a saved place-only case dump as a figure.

Reconstructs the scene from the preserved boundary artifacts only
(`surface_at_request.json`, `fixture_install.json`, `case_manifest.json`,
`placement_last_result.json`), so a placement failure can be inspected without
relaunching Gazebo. See `.cursor/rules/sim-lifecycle.mdc`.

Usage:
  scripts/render_place_case.py --case DUMP_DIR [--out FILE]
"""

import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

import luggage_gazebo.place_only_fixture as fx  # noqa: E402
from luggage_description.scene_tf_config_utils import (  # noqa: E402
    container_inner_floor_z,
    container_opening_aperture_corners_in_container,
    default_scene_tf_config_path,
    load_scene_tf_config,
)

REASON_COLORS = {
    "ok": "#2ca02c",
    "overlap": "#d62728",
    "insufficient_clearance": "#ff7f0e",
    "outside_hull": "#8c564b",
    "outside_aperture": "#1f77b4",
    "corridor_blocked": "#9467bd",
    "unknown_above_floor": "#7f7f7f",
    "insufficient_support": "#e377c2",
}


def load_case(case_dir):
    def read(name):
        path = os.path.join(case_dir, name)
        return json.load(open(path, encoding="utf-8")) if os.path.exists(path) else None

    return {
        "surface": read("surface_at_request.json"),
        "install": read("fixture_install.json") or [],
        "manifest": read("case_manifest.json") or {},
        "last_result": read("placement_last_result.json") or {},
        "classification": read("failure_classification.json") or {},
    }


def hull_floor_polygon(ctx, z_rel):
    """Container cross-section at one floor-relative height."""
    half_x, half_y = ctx["inner_l"] * 0.5, ctx["inner_w"] * 0.5
    y_max = ctx["y_max_at_z"](z_rel)
    return [(-half_x, -half_y), (half_x, -half_y), (half_x, y_max),
            (-half_x, y_max)]


def draw_top_view(ax, ctx, data, aabbs):
    half_x, half_y = ctx["inner_l"] * 0.5, ctx["inner_w"] * 0.5
    ax.add_patch(Polygon(hull_floor_polygon(ctx, 0.0), closed=True,
                         fill=False, lw=2, edgecolor="black",
                         label="hull at floor"))
    for aabb in aabbs:
        ax.add_patch(Rectangle(
            (aabb[0], aabb[1]), aabb[3] - aabb[0], aabb[4] - aabb[1],
            facecolor="#d62728", alpha=0.35, edgecolor="#d62728", lw=1.5))
    size = data["manifest"].get("cargo_size_wdh") or [0.0, 0.0, 0.0]
    ax.add_patch(Rectangle(
        (-half_x - size[0] - 0.12, -size[1] * 0.5), size[0], size[1],
        facecolor="#2ca02c", alpha=0.6, edgecolor="#2ca02c",
        label="cargo footprint"))
    ax.annotate("cargo\n%.2f x %.2f m" % (size[0], size[1]),
                (-half_x - size[0] * 0.5 - 0.12, -size[1] * 0.5 - 0.10),
                ha="center", va="top", fontsize=8)
    ax.annotate("opening (-X)", (-half_x, half_y + 0.05), fontsize=8)
    ax.set_xlim(-half_x - size[0] - 0.25, half_x + 0.15)
    ax.set_ylim(-half_y - 0.15, half_y + 0.2)
    ax.set_aspect("equal")
    ax.set_xlabel("container x [m]")
    ax.set_ylabel("container y [m]")
    ax.set_title("top view: committed fixture vs cargo")
    ax.legend(loc="lower right", fontsize=7)


def draw_side_view(ax, ctx, data, aabbs):
    half_x = ctx["inner_l"] * 0.5
    inner_h = ctx["inner_h"]
    ax.add_patch(Rectangle((-half_x, 0.0), ctx["inner_l"], inner_h,
                           fill=False, lw=2, edgecolor="black"))
    peak = 0.0
    for aabb in aabbs:
        ax.add_patch(Rectangle(
            (aabb[0], aabb[2]), aabb[3] - aabb[0], aabb[5] - aabb[2],
            facecolor="#d62728", alpha=0.35, edgecolor="#d62728", lw=1.5))
        peak = max(peak, aabb[5])
    size = data["manifest"].get("cargo_size_wdh") or [0.0, 0.0, 0.0]
    box_h = float(size[2])
    ax.add_patch(Rectangle((-half_x * 0.6, peak), size[0], box_h,
                           facecolor="#2ca02c", alpha=0.5,
                           edgecolor="#2ca02c", lw=1.5, linestyle="--"))
    clearance = inner_h - (peak + box_h)
    ax.annotate(
        "stacked cargo leaves %.0f mm\n(clearance_margin 30 mm)"
        % (clearance * 1000.0),
        (-half_x * 0.6 + size[0] + 0.05, peak + box_h * 0.5), fontsize=8,
        va="center")
    ax.axhline(inner_h, color="black", lw=1)
    ax.annotate("ceiling (inner_h %.2f m)" % inner_h, (-half_x + 0.02,
                inner_h + 0.02), fontsize=8)
    ax.set_xlim(-half_x - 0.1, half_x + 0.6)
    ax.set_ylim(-0.05, inner_h + 0.2)
    ax.set_xlabel("container x [m]")
    ax.set_ylabel("height above floor [m]")
    ax.set_title("side view: cargo stacked on the highest committed surface")


def draw_height_map(ax, data):
    surface = data["surface"]
    heights = surface["height"]
    res = float(surface["resolution"])
    inner = surface["inner_size"]
    extent = [-inner[0] * 0.5, inner[0] * 0.5, -inner[1] * 0.5, inner[1] * 0.5]
    grid = [[heights[ix][iy] for ix in range(surface["nx"])]
            for iy in range(surface["ny"])]
    image = ax.imshow(grid, origin="lower", extent=extent, aspect="equal",
                      cmap="viridis")
    plt.colorbar(image, ax=ax, fraction=0.046, label="surface height [m]")
    ax.set_xlabel("container x [m]")
    ax.set_ylabel("container y [m]")
    ax.set_title("cargo map as consumed (rev %s, %.0f mm cells)"
                 % (surface.get("map_revision"), res * 1000.0))


def support_z(ctx, candidate):
    """Floor-relative height the candidate rests on."""
    return (float(candidate["center_local"][2]) + ctx["inner_h"] * 0.5
            - float(candidate["size"][2]) * 0.5)


def draw_candidates(ax, ctx, data, aperture_y=None):
    candidates = data["last_result"].get("candidates") or []
    by_reason = {}
    for cand in candidates:
        by_reason.setdefault(str(cand.get("reason")), []).append(cand)
    for reason, group in sorted(by_reason.items()):
        supports = sorted({round(support_z(ctx, c), 2) for c in group})
        ax.scatter([c["center_local"][0] for c in group],
                   [c["center_local"][1] for c in group], s=14,
                   color=REASON_COLORS.get(reason, "#333333"),
                   label="%s (%d, support z=%s)"
                         % (reason, len(group),
                            "/".join("%.2f" % s for s in supports)))
    ax.add_patch(Polygon(hull_floor_polygon(ctx, 0.0), closed=True,
                         fill=False, lw=1.5, edgecolor="black",
                         label="hull at floor"))
    # The +Y chamfer opens up with height, so a floor-level window beyond the
    # solid outline is outside the hull even though the space above it is free.
    chamfer_top = max(0.0, float(ctx["hull"].chamfer.wall_z) - ctx["floor_z"]) \
        if getattr(ctx["hull"], "chamfer", None) else 0.0
    if chamfer_top > 0.0:
        ax.add_patch(Polygon(hull_floor_polygon(ctx, chamfer_top), closed=True,
                             fill=False, lw=1.2, linestyle="--",
                             edgecolor="#555555",
                             label="hull above chamfer (z>=%.2f m)"
                                   % chamfer_top))
    if aperture_y:
        for bound in aperture_y:
            ax.axhline(bound, color="#1f77b4", lw=1.0, linestyle=":")
        ax.annotate("aperture y span", (0.0, aperture_y[1] + 0.02),
                    color="#1f77b4", fontsize=7, ha="center")
    ax.set_aspect("equal")
    ax.set_xlabel("container x [m]")
    ax.set_ylabel("container y [m]")
    result = data["last_result"]
    ax.set_title("retained candidates by reject reason\n"
                 "reason_code=%s  capacity_feasible=%s"
                 % (result.get("reason_code"),
                    result.get("n_capacity_feasible")))
    ax.legend(loc="lower right", fontsize=7, framealpha=0.9)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", required=True, help="case dump directory")
    parser.add_argument("--out", default="", help="output PNG path")
    parser.add_argument("--scene", default="", help="scene_tf config path")
    args = parser.parse_args()

    data = load_case(args.case)
    if not data["surface"]:
        raise SystemExit("no surface_at_request.json in %s" % args.case)

    scene = load_scene_tf_config(args.scene or default_scene_tf_config_path())
    ctx = fx.hull_context(fx.hull_from_scene_config(scene),
                          container_inner_floor_z(scene))
    saved_hash = str(data["surface"].get("geometry_hash") or "")
    if saved_hash != ctx["geometry_hash"]:
        raise SystemExit(
            "geometry_hash mismatch: dump %s, scene %s. Render with the scene "
            "config the case ran against." % (saved_hash, ctx["geometry_hash"]))

    aabbs = [fx.FixtureBox(tuple(entry["record"]["center"]),
                           tuple(entry["record"]["size"]),
                           entry["record"]["yaw"]).aabb()
             for entry in data["install"] if entry.get("record")]

    try:
        corners = container_opening_aperture_corners_in_container(scene, 0.0)
        ys = [float(corner[1]) for corner in corners]
        aperture_y = (min(ys), max(ys))
    except (KeyError, TypeError, ValueError):
        aperture_y = None

    figure, axes = plt.subplots(2, 2, figsize=(14, 11))
    draw_top_view(axes[0][0], ctx, data, aabbs)
    draw_side_view(axes[0][1], ctx, data, aabbs)
    draw_height_map(axes[1][0], data)
    draw_candidates(axes[1][1], ctx, data, aperture_y)

    manifest = data["manifest"]
    verdict = (data["classification"].get("verdict") or {})
    figure.suptitle(
        "%s  %s  |  product %s  |  independent capacity test %s  |  hash %s"
        % (manifest.get("case_id", "?"), manifest.get("cargo_id", "?"),
           data["last_result"].get("reason_code") or "success (slot returned)",
           "agrees" if verdict.get("product_agrees")
           else ("disagrees" if verdict else "not run (no failure)"),
           saved_hash[:12]),
        fontsize=11)
    figure.tight_layout(rect=(0, 0, 1, 0.97))

    out = args.out or os.path.join(args.case, "scene.png")
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    figure.savefig(out, dpi=130)
    print("wrote %s" % out)


if __name__ == "__main__":
    main()
