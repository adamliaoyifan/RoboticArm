#!/usr/bin/env python3
"""Frozen COMMON GEOMETRIC BASELINE for the A0/A'/B control (2026-10-19).

Arbitrated: the baseline is the CURRENT YAML's full 7-face inner cavity
(±0.985, chamfer, floor 0.53, aperture, wall thickness from YAML outer)
— a SIMULATION-PRODUCT baseline choice, NOT a claim that the YAML is
closer to the physical container (that needs mechanical drawings or
measurement). The 8k CAD is retained as a separate asset with its
deviations recorded; it is not truth this round.

Single source of truth: scene_tf_deploy_update.yaml is READ DIRECTLY
(no retyped numbers). Wall thickness = YAML outer − YAML inner (both
defined; nothing inferred).

7 faces: floor, ceiling, -Y wall, +Y vertical (to wall_z), +Y chamfer
slope, ±X pentagon ends; aperture = rect hole in the -X pentagon.
"""
import os

import yaml

_SCENE = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))),
    "src", "luggage_description", "config", "scene_tf_deploy_update.yaml")


def load_baseline(scene_path=_SCENE):
    cfg = yaml.safe_load(open(scene_path))
    cont = cfg["container"]
    inner, outer, cham, opening = (cont["inner"], cont["outer"],
                                   cont["inner"]["chamfer"],
                                   cont["opening"])
    ap = opening["aperture"]["corners"]
    ys = [c[1] for c in ap]
    zs = [c[2] for c in ap]
    xs = [c[0] for c in ap]
    base = {
        "source": os.path.realpath(scene_path),
        # Inner cavity (container_link).
        "x_lo": -inner["length"] / 2, "x_hi": inner["length"] / 2,
        "y_lo": -inner["width"] / 2, "y_hi": inner["width"] / 2,
        "floor_z": float(inner["floor_z"]),
        "ceiling_z": float(inner["ceiling_z"]),
        # 7-face hull: +Y side clips above wall_z from y_hi back to
        # floor_y at the ceiling (cavity = cuboid minus triangular
        # prism along +Y).
        "chamfer_floor_y": float(cham["floor_y"]),
        "chamfer_wall_z": float(cham["wall_z"]),
        # Aperture (rect in the -X pentagon).
        "aperture": {
            "x": float(min(xs)),
            "y_lo": float(min(ys)), "y_hi": float(max(ys)),
            "z_lo": float(min(zs)), "z_hi": float(max(zs)),
        },
        # Outer shell (wall thickness = outer - inner, YAML-defined).
        "o_x_lo": -outer["length"] / 2, "o_x_hi": outer["length"] / 2,
        "o_y_lo": -outer["width"] / 2, "o_y_hi": outer["width"] / 2,
        "o_z_lo": 0.0, "o_z_hi": float(outer["height"]),
    }
    assert base["aperture"]["x"] <= base["x_lo"] + 1e-9, \
        "aperture must pierce the -X inner face"
    return base


def panels(b):
    """The wall decomposition shared by A' (triangles) and B (SDF
    primitives): axis-aligned boxes + one rotated chamfer wedge.

    Returns a list of dicts: {"name", "kind": "box"|"wedge",
    box: (center, size) | wedge: (plane point, slope, x-span, thickness)}.
    Every panel's inner face lies EXACTLY on a baseline plane.
    """
    x_lo, x_hi = b["x_lo"], b["x_hi"]
    y_lo, y_hi = b["y_lo"], b["y_hi"]
    fz, cz = b["floor_z"], b["ceiling_z"]
    ox_lo, ox_hi = b["o_x_lo"], b["o_x_hi"]
    oy_lo, oy_hi = b["o_y_lo"], b["o_y_hi"]
    oz_lo, oz_hi = b["o_z_lo"], b["o_z_hi"]
    ap = b["aperture"]
    wall_z, floor_y = b["chamfer_wall_z"], b["chamfer_floor_y"]

    def box(name, lo, hi):
        center = [(lo[i] + hi[i]) / 2 for i in range(3)]
        size = [hi[i] - lo[i] for i in range(3)]
        return {"name": name, "kind": "box", "center": center,
                "size": size}

    panels_ = [
        box("floor", (ox_lo, oy_lo, oz_lo), (ox_hi, oy_hi, fz)),
        box("ceiling", (ox_lo, oy_lo, cz), (ox_hi, oy_hi, oz_hi)),
        box("wall_neg_y", (ox_lo, oy_lo, oz_lo), (ox_hi, y_lo, oz_hi)),
        # +Y vertical wall ABOVE the chamfer (full width from wall_z up).
        box("wall_pos_y_vertical", (ox_lo, y_hi, wall_z),
            (ox_hi, oy_hi, oz_hi)),
        # -X face: aperture frame. The +Y-side pillar above the chamfer
        # is a plain box; the sloped zone z[floor_z, wall_z] is carried
        # by the -X end wedge.
        box("sill", (ox_lo, oy_lo, oz_lo),
            (x_lo, oy_hi, ap["z_lo"])),
        box("lintel", (ox_lo, ap["y_lo"], ap["z_hi"]),
            (x_lo, oy_hi, oz_hi)),
        box("pillar_neg_y", (ox_lo, oy_lo, ap["z_lo"]),
            (x_lo, ap["y_lo"], ap["z_hi"])),
        box("pillar_pos_y", (ox_lo, ap["y_hi"], wall_z),
            (x_lo, y_hi, ap["z_hi"])),
        # +X end wall: band below the chamfer start (clipped at
        # floor_y) + full-width band above it; the sloped corner is the
        # +X end wedge.
        box("end_pos_x_band", (x_hi, oy_lo, fz),
            (ox_hi, floor_y, wall_z)),
        box("end_pos_x_upper", (x_hi, oy_lo, wall_z),
            (ox_hi, oy_hi, oz_hi)),
    ]
    # Chamfer wedge (+Y lower wall): inner face on the slope plane
    # through (y=floor_y, z=floor_z) and (y=y_hi, z=wall_z), full x.
    slope_dy = y_hi - floor_y
    slope_dz = wall_z - fz
    panels_.append({
        "name": "chamfer_wedge", "kind": "wedge",
        "point": (0.0, floor_y, fz),
        "slope": (slope_dy, slope_dz),
        "x_span": (ox_lo, ox_hi),
        "outer_y": oy_hi, "floor_z": fz,
    })
    # End wedges: the triangular wall corner at BOTH end walls in the
    # chamfer zone — triangle (floor_y, floor_z), (y_hi, floor_z),
    # (y_hi, wall_z) in (y, z).
    panels_.append({
        "name": "end_wedge_neg_x", "kind": "end_wedge",
        "x_span": (ox_lo, x_lo), "outer_y": oy_hi,
        "floor_z": fz, "point": (0.0, floor_y, fz),
        "slope": (slope_dy, slope_dz),
    })
    panels_.append({
        "name": "end_wedge_pos_x", "kind": "end_wedge",
        "x_span": (x_hi, ox_hi), "outer_y": oy_hi,
        "floor_z": fz, "point": (0.0, floor_y, fz),
        "slope": (slope_dy, slope_dz),
    })
    return panels_


def chamfer_y(b, z):
    """Inner +Y boundary at height z — BOTTOM chamfer per the frozen
    YAML (authoritative docstring: floor_y is the usable +Y limit ON
    THE CARGO FLOOR; wall_z is where the cut meets the full wall; the
    LOWER +Y edge is the slanted face). Narrow at the floor, full
    width from wall_z up."""
    fz, wz = b["floor_z"], b["chamfer_wall_z"]
    if z >= wz:
        return b["y_hi"]
    if z <= fz:
        return b["chamfer_floor_y"]
    t = (z - fz) / (wz - fz)
    return b["chamfer_floor_y"] + t * (b["y_hi"] - b["chamfer_floor_y"])


def inside_cavity(b, p, eps=0.0):
    """Point strictly inside the 7-face cavity (aperture not carved —
    the aperture tunnel is part of the -X panel, not the cavity)."""
    x, y, z = p
    return (b["x_lo"] + eps < x < b["x_hi"] - eps
            and b["y_lo"] + eps < y < chamfer_y(b, z) - eps
            and b["floor_z"] + eps < z < b["ceiling_z"] - eps)


if __name__ == "__main__":
    import json
    print(json.dumps(load_baseline(), indent=1, sort_keys=True))
