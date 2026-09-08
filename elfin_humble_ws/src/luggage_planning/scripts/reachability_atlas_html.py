#!/usr/bin/env python3
"""ROS-free HTML viewer for reachability atlas .npz files.

The checked-in RViz tool (reachability_atlas_viz.py) needs rospy. This
script dumps a Plotly page so Humble can inspect the same grids.

Usage:
  python3 src/luggage_planning/scripts/reachability_atlas_html.py
  python3 src/luggage_planning/scripts/reachability_atlas_html.py \\
      --atlas src/luggage_planning/data/reachability_atlas/s20_container_collision_aware.npz \\
      --scene-tf src/luggage_description/config/scene_tf.yaml \\
      --out /tmp/atlas.html
"""

from __future__ import division

import argparse
import json
import os
import sys

import numpy as np
import yaml

_DESC_SRC = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "luggage_description"))
if _DESC_SRC not in sys.path:
    sys.path.insert(0, _DESC_SRC)

from luggage_description.container_geometry import (  # noqa: E402
    descriptor_from_scene_config,
    yz_polygon,
)

STATUS_NAMES = {0: "unknown", 1: "unreachable", 2: "marginal", 3: "reachable"}
STATUS_COLORS = {
    "reachable": "#2ca02c",
    "marginal": "#ffbf00",
    "unreachable": "#d62728",
    "unknown": "#888888",
}


def _load_one(npz_path):
    meta_path = npz_path[:-4] + ".yaml" if npz_path.endswith(".npz") else npz_path + ".yaml"
    data = np.load(npz_path, allow_pickle=True)
    with open(meta_path, "r") as handle:
        meta = yaml.safe_load(handle)
    grid = meta["grid"]
    origin = [float(v) for v in grid["origin"]]
    res = float(grid["resolution_xyz"])
    yaw_bins = [float(v) for v in grid["yaw_bins"]]
    status = np.asarray(data["status"])
    opening = np.asarray(data["opening_connected"]) if "opening_connected" in data.files else None
    nx, ny, nz, nyaw = status.shape
    points = []
    for ix in range(nx):
        for iy in range(ny):
            for iz in range(nz):
                for iyaw in range(nyaw):
                    code = int(status[ix, iy, iz, iyaw])
                    points.append({
                        "x": origin[0] + (ix + 0.5) * res,
                        "y": origin[1] + (iy + 0.5) * res,
                        "z": origin[2] + (iz + 0.5) * res,
                        "yaw": yaw_bins[iyaw],
                        "status": STATUS_NAMES[code],
                        "opening": bool(opening[ix, iy, iz, iyaw]) if opening is not None else True,
                    })
    payload = meta.get("payload") or {}
    label = os.path.basename(npz_path)
    if payload.get("enabled"):
        size = payload.get("size") or []
        label = "payload %sx%sx%s" % tuple(size)
    else:
        label = "empty load"
    return {
        "file": os.path.basename(npz_path),
        "label": label,
        "stats": meta.get("stats", {}),
        "resolution": res,
        "yaw_bins": yaw_bins,
        "payload": payload,
        "points": points,
    }


def _default_scene_tf():
    return os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "luggage_description",
        "config", "scene_tf.yaml"))


def _aabb_edges(hx, hy, z0, z1):
    corners = [
        (-hx, -hy, z0), (hx, -hy, z0), (hx, hy, z0), (-hx, hy, z0),
        (-hx, -hy, z1), (hx, -hy, z1), (hx, hy, z1), (-hx, hy, z1),
    ]
    pairs = (
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    )
    return [(corners[a], corners[b]) for a, b in pairs]


def _edges_to_xyz(edges):
    xs, ys, zs = [], [], []
    for start, end in edges:
        xs.extend([float(start[0]), float(end[0]), None])
        ys.extend([float(start[1]), float(end[1]), None])
        zs.extend([float(start[2]), float(end[2]), None])
    return {"x": xs, "y": ys, "z": zs}


def _inner_hull_edges(geometry):
    hx = geometry.half_x
    polygon = yz_polygon(geometry)
    minus = [(-hx, y, z) for y, z in polygon]
    plus = [(hx, y, z) for y, z in polygon]
    edges = []
    count = len(polygon)
    for index in range(count):
        nxt = (index + 1) % count
        edges.append((minus[index], minus[nxt]))
        edges.append((plus[index], plus[nxt]))
        edges.append((minus[index], plus[index]))
    return edges


def load_container_outline(scene_tf_path):
    """Wireframes in container_link: 7-face inner hull, outer AABB, aperture."""
    with open(scene_tf_path, "r") as handle:
        config = yaml.safe_load(handle)
    geometry = descriptor_from_scene_config(config)
    container = config.get("container") or {}
    outer = container.get("outer") or {}
    aperture = (((container.get("opening") or {}).get("aperture") or {}).get("corners")
                or [])
    aperture_edges = []
    if len(aperture) == 4:
        corners = [[float(v) for v in corner] for corner in aperture]
        aperture_edges = [
            (corners[i], corners[(i + 1) % 4]) for i in range(4)
        ]
    outer_l = float(outer.get("length", geometry.length))
    outer_w = float(outer.get("width", geometry.width))
    outer_h = float(outer.get("height", geometry.ceiling_z))
    return {
        "scene_tf": os.path.basename(scene_tf_path),
        "geometry_hash": geometry.geometry_hash,
        "inner": _edges_to_xyz(_inner_hull_edges(geometry)),
        "outer": _edges_to_xyz(_aabb_edges(
            0.5 * outer_l, 0.5 * outer_w, 0.0, outer_h)),
        "aperture": _edges_to_xyz(aperture_edges),
    }


def _discover(atlas_dir):
    names = [
        "s20_container_collision_aware.npz",
        "s20_container_collision_aware_payload_0.55x0.40x0.25.npz",
        "s20_container_collision_aware_payload_0.70x0.45x0.28.npz",
        "s20_container_collision_aware_payload_0.80x0.50x0.32.npz",
    ]
    found = []
    for name in names:
        path = os.path.join(atlas_dir, name)
        if os.path.isfile(path):
            found.append(path)
    return found


def _html(atlases, hull, skip_unknown):
    payload = json.dumps(atlases)
    hull_js = json.dumps(hull)
    skip = "true" if skip_unknown else "false"
    return """<!DOCTYPE html>
<html><head>
<meta charset="utf-8"/>
<title>Reachability atlas</title>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
<style>
body { font-family: sans-serif; margin: 12px; }
.row { display: flex; gap: 12px; flex-wrap: wrap; align-items: center; }
#plot { width: 100%%; height: 80vh; }
code { background: #f3f3f3; padding: 1px 4px; }
</style>
</head><body>
<h2>S20 container reachability atlas</h2>
<p>Grid is in <code>container_link</code>. Blue = 7-face inner hull (chamfered
usable volume), gray = outer AABB, cyan = door aperture. Green/yellow/red/gray
spheres are atlas cells. Sampled 2026-08-11, 0.15 m, yaw 0 / 90 deg.</p>
<div class="row">
  <label>Atlas <select id="atlas"></select></label>
  <label>Yaw <select id="yaw">
    <option value="all">both</option>
    <option value="0">0 deg</option>
    <option value="90">90 deg</option>
  </select></label>
  <label><input type="checkbox" id="opening"/> opening-connected only</label>
  <label><input type="checkbox" id="unknown"/> show UNKNOWN</label>
  <label><input type="checkbox" id="hull" checked/> inner hull</label>
  <label><input type="checkbox" id="outer" checked/> outer AABB</label>
  <label><input type="checkbox" id="door" checked/> aperture</label>
  <span id="stats"></span>
</div>
<div id="plot"></div>
<script>
const ATLASES = %s;
const HULL = %s;
const SKIP_UNKNOWN_DEFAULT = %s;
const COLORS = {
  reachable: "#2ca02c",
  marginal: "#ffbf00",
  unreachable: "#d62728",
  unknown: "#888888"
};
const sel = document.getElementById("atlas");
ATLASES.forEach((a, i) => {
  const o = document.createElement("option");
  o.value = i;
  o.textContent = a.label + "  rate=" + ((a.stats.reachability_rate || 0).toFixed(3));
  sel.appendChild(o);
});
document.getElementById("unknown").checked = !SKIP_UNKNOWN_DEFAULT;
function draw() {
  const atlas = ATLASES[Number(sel.value)];
  const yawMode = document.getElementById("yaw").value;
  const openingOnly = document.getElementById("opening").checked;
  const showUnknown = document.getElementById("unknown").checked;
  const by = {reachable:[], marginal:[], unreachable:[], unknown:[]};
  atlas.points.forEach(p => {
    if (openingOnly && !p.opening) return;
    if (yawMode === "0" && Math.abs(p.yaw) > 0.2) return;
    if (yawMode === "90" && Math.abs(p.yaw - Math.PI/2) > 0.2) return;
    if (p.status === "unknown" && !showUnknown) return;
    by[p.status].push(p);
  });
  document.getElementById("stats").textContent =
    atlas.file + "  shown " +
    Object.keys(by).map(k => k[0] + "=" + by[k].length).join(" ");
  const traces = Object.keys(by).map(k => ({
    type: "scatter3d",
    mode: "markers",
    name: k,
    x: by[k].map(p => p.x),
    y: by[k].map(p => p.y),
    z: by[k].map(p => p.z),
    marker: {size: 4, color: COLORS[k], opacity: k === "unknown" ? 0.15 : 0.7},
    hovertemplate: k + " x=%%{x:.2f} y=%%{y:.2f} z=%%{z:.2f}<extra></extra>"
  }));
  function lineTrace(kind, name, color, width) {
    const g = HULL[kind];
    if (!g || !g.x || !g.x.length) return null;
    return {
      type: "scatter3d",
      mode: "lines",
      name: name,
      x: g.x, y: g.y, z: g.z,
      line: {color: color, width: width},
      hoverinfo: "name"
    };
  }
  if (document.getElementById("hull").checked) {
    const t = lineTrace("inner", "inner hull", "#1f77b4", 6);
    if (t) traces.push(t);
  }
  if (document.getElementById("outer").checked) {
    const t = lineTrace("outer", "outer AABB", "#555555", 3);
    if (t) traces.push(t);
  }
  if (document.getElementById("door").checked) {
    const t = lineTrace("aperture", "aperture", "#17becf", 7);
    if (t) traces.push(t);
  }
  Plotly.react("plot", traces, {
    scene: {
      xaxis: {title: "container X (m)"},
      yaxis: {title: "container Y (m)"},
      zaxis: {title: "container Z (m)"},
      aspectmode: "data"
    },
    margin: {l: 0, r: 0, t: 20, b: 0},
    legend: {orientation: "h"}
  }, {responsive: true});
}
["atlas","yaw","opening","unknown","hull","outer","door"].forEach(id =>
  document.getElementById(id).addEventListener("change", draw));
draw();
</script>
</body></html>
""" % (payload, hull_js, skip)


def main():
    root = os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "data", "reachability_atlas"))
    parser = argparse.ArgumentParser()
    parser.add_argument("--atlas-dir", default=root)
    parser.add_argument("--atlas", action="append", default=[])
    parser.add_argument("--scene-tf", default=_default_scene_tf())
    parser.add_argument("--out", default="/tmp/reachability_atlas.html")
    parser.add_argument("--skip-unknown", action="store_true", default=True)
    parser.add_argument("--keep-unknown", action="store_true")
    args = parser.parse_args()
    paths = list(args.atlas) or _discover(args.atlas_dir)
    if not paths:
        raise SystemExit("no atlas .npz found under %s" % args.atlas_dir)
    atlases = [_load_one(path) for path in paths]
    hull = load_container_outline(args.scene_tf)
    skip_unknown = not args.keep_unknown
    out = os.path.abspath(args.out)
    with open(out, "w") as handle:
        handle.write(_html(atlases, hull, skip_unknown))
    print("wrote %s (%d atlases)" % (out, len(atlases)))
    print("  hull %s hash=%s inner_pts=%d outer_pts=%d aperture_pts=%d" % (
        hull["scene_tf"],
        hull["geometry_hash"][:12],
        len(hull["inner"]["x"]),
        len(hull["outer"]["x"]),
        len(hull["aperture"]["x"])))
    for atlas in atlases:
        stats = atlas["stats"]
        print("  %s  reachable=%s marginal=%s rate=%s" % (
            atlas["file"],
            stats.get("reachable_cells"),
            stats.get("marginal_cells"),
            stats.get("reachability_rate")))


if __name__ == "__main__":
    main()
