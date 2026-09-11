#!/usr/bin/env python3
"""Interactive 3D view of LRF-P1 clouds vs GT / baseline / fusion / learned boxes.

Rebuilds the same seed-7 observations as run_spike.py and writes a Plotly HTML
page you can rotate in a browser. Not a Gazebo/RViz recording.
"""

from __future__ import division

import argparse
import os
import sys
import webbrowser

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

ROOT_HINTS = (
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")),
)

DEFAULT_IDS = (
    "b6b2e836cd488ce9",  # worst learned heavy
    "d5b1b8fdf8fabbc2",  # second-worst heavy
    "unoccluded",        # first test unoccluded control
    "light",             # first test light (not the unoccluded control)
)

COLORS = {
    "gt": "#2ca02c",
    "baseline": "#1f77b4",
    "fusion": "#9467bd",
    "learned": "#d62728",
    "points": "#7f7f7f",
}


def _prepare_path(workspace):
    for path in (
        os.path.join(workspace, "src", "luggage_description"),
        os.path.join(workspace, "src", "luggage_perception"),
        workspace,
    ):
        if path not in sys.path:
            sys.path.insert(0, path)


def _gt_vector(gt):
    c = gt["center_xyz"]
    return np.array([
        gt["width"], gt["depth"], gt["height"],
        c[0], c[1], c[2], gt["yaw"],
    ], dtype=np.float64)


def _base_vector(box, gt):
    if box is None:
        c = gt["center_xyz"]
        return np.array([0.0, 0.0, 0.0, c[0], c[1], c[2], 0.0])
    c = box["center_xyz"]
    return np.array([
        box["width"], box["depth"], box["height"],
        c[0], c[1], c[2], box["yaw"],
    ], dtype=np.float64)


def box_wire_xyz(box):
    if not box:
        return None
    w, d, h = float(box["width"]), float(box["depth"]), float(box["height"])
    cx, cy, cz = [float(v) for v in box["center_xyz"]]
    yaw = float(box["yaw"])
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    signs = (
        (-0.5, -0.5, -0.5), (0.5, -0.5, -0.5), (0.5, 0.5, -0.5), (-0.5, 0.5, -0.5),
        (-0.5, -0.5, 0.5), (0.5, -0.5, 0.5), (0.5, 0.5, 0.5), (-0.5, 0.5, 0.5),
    )
    corners = np.array(signs) * np.array([w, d, h])
    corners = corners.dot(rot.T)
    corners[:, 0] += cx
    corners[:, 1] += cy
    corners[:, 2] += cz
    edges = (
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    )
    xs, ys, zs = [], [], []
    for a, b in edges:
        xs.extend([corners[a, 0], corners[b, 0], None])
        ys.extend([corners[a, 1], corners[b, 1], None])
        zs.extend([corners[a, 2], corners[b, 2], None])
    return xs, ys, zs


def add_box(fig, row, col, box, name, color, showlegend):
    wire = box_wire_xyz(box)
    if wire is None:
        return
    xs, ys, zs = wire
    fig.add_trace(
        go.Scatter3d(
            x=xs, y=ys, z=zs, mode="lines",
            line=dict(color=color, width=6),
            name=name, legendgroup=name, showlegend=showlegend,
            hoverinfo="name",
        ),
        row=row, col=col,
    )


def add_cloud(fig, row, col, points, showlegend):
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or len(pts) == 0:
        return
    if len(pts) > 1200:
        idx = np.linspace(0, len(pts) - 1, 1200).astype(int)
        pts = pts[idx]
    fig.add_trace(
        go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
            mode="markers",
            marker=dict(size=2.4, color=COLORS["points"], opacity=0.55),
            name="observed points",
            legendgroup="observed points",
            showlegend=showlegend,
            hoverinfo="skip",
        ),
        row=row, col=col,
    )


def resolve_ids(packed, requested):
    by_id = {item["eval"]["sample_id"]: item for item in packed}
    chosen = []
    used = set()
    for token in requested:
        token = str(token).strip()
        if not token:
            continue
        if token in by_id:
            chosen.append(by_id[token])
            used.add(id(by_id[token]))
            continue
        if token == "unoccluded":
            for item in packed:
                if item["eval"]["split"] != "test":
                    continue
                if item["eval"]["unoccluded_control"] and id(item) not in used:
                    chosen.append(item)
                    used.add(id(item))
                    break
            else:
                raise SystemExit("no test unoccluded_control sample")
            continue
        if token in ("light", "medium", "heavy"):
            for item in packed:
                ev = item["eval"]
                if ev["split"] != "test" or ev["occlusion_bucket"] != token:
                    continue
                if token == "light" and ev["unoccluded_control"]:
                    continue
                if id(item) not in used:
                    chosen.append(item)
                    used.add(id(item))
                    break
            else:
                raise SystemExit("no unused test %s sample" % token)
            continue
        raise SystemExit("unknown sample id or preset: %s" % token)
    if not chosen:
        raise SystemExit("no samples selected")
    return chosen


def title_for(item, scores):
    ev = item["eval"]
    parts = [
        ev["sample_id"],
        ev["mesh_id"],
        ev["occlusion_bucket"],
    ]
    if ev.get("unoccluded_control"):
        parts.append("unoccluded")
    learned = scores.get("learned")
    if learned and learned.get("composite") is not None:
        parts.append("learned composite=%.3f" % learned["composite"])
    return " | ".join(parts)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default=ROOT_HINTS[0])
    parser.add_argument("--out", default="/tmp/lrf_p1_viz.html")
    parser.add_argument(
        "--ids",
        default=",".join(DEFAULT_IDS),
        help="Comma-separated sample_ids, or presets: unoccluded,light,medium,heavy",
    )
    parser.add_argument("--open", action="store_true", help="Open the HTML after writing")
    args = parser.parse_args(argv)
    workspace = os.path.abspath(args.workspace)
    _prepare_path(workspace)

    from research.lrf_p1 import metrics as M
    from research.lrf_p1.baseline_adapter import estimate_from_observation, fuse_views
    from research.lrf_p1.candidate import ResidualEnsemble, infer
    from research.lrf_p1.features import cloud_features
    from research.lrf_p1.observe import build_sample, enumerate_specs

    gen_rng = np.random.RandomState(7)
    packed = []
    for spec in enumerate_specs():
        obs, ev, extra = build_sample(workspace, spec, gen_rng)
        packed.append({"obs": obs, "eval": ev, "extra_views": extra})

    for item in packed:
        item["base"] = estimate_from_observation(item["obs"])
        views = [item["obs"]] + [
            {
                "points": cloud,
                "roi_center_xy": item["obs"].get("roi_center_xy"),
                "frame_id": "world",
                "stamp": 0.0,
            }
            for cloud in item["extra_views"]
        ]
        item["fuse"] = fuse_views(views)
        item["feat"] = cloud_features(item["obs"], item["base"].get("box"))

    train = [item for item in packed if item["eval"]["split"] == "train"]
    features, targets = [], []
    for item in train:
        residual = _gt_vector(item["eval"]["gt"]) - _base_vector(
            item["base"].get("box"), item["eval"]["gt"])
        features.append(item["feat"])
        targets.append(residual)
    ens = ResidualEnsemble(n_models=5, alpha=2.0, seeds=(1, 2, 3, 4, 5))
    ens.fit(features, targets)

    requested = [tok.strip() for tok in args.ids.split(",") if tok.strip()]
    chosen = resolve_ids(packed, requested)

    n = len(chosen)
    cols = 2 if n > 1 else 1
    rows = int(np.ceil(n / float(cols)))
    subplot_titles = []
    scored = []
    for item in chosen:
        learned = infer(item["obs"], ens)
        scores = {
            "baseline": {
                "composite": M.normalized_composite(
                    M.component_errors(item["base"].get("box"), item["eval"]["gt"]),
                    item["eval"]["gt"],
                ) if item["base"].get("ok") else None,
            },
            "fusion": {
                "composite": M.normalized_composite(
                    M.component_errors(item["fuse"].get("box"), item["eval"]["gt"]),
                    item["eval"]["gt"],
                ) if item["fuse"].get("ok") else None,
            },
            "learned": {
                "composite": M.normalized_composite(
                    M.component_errors(learned.get("box"), item["eval"]["gt"]),
                    item["eval"]["gt"],
                ) if learned.get("ok") else None,
            },
        }
        scored.append((item, learned, scores))
        subplot_titles.append(title_for(item, scores))

    fig = make_subplots(
        rows=rows, cols=cols,
        specs=[[{"type": "scene"}] * cols for _ in range(rows)],
        subplot_titles=subplot_titles,
        vertical_spacing=0.08,
        horizontal_spacing=0.04,
    )
    for idx, (item, learned, scores) in enumerate(scored):
        row = idx // cols + 1
        col = idx % cols + 1
        first = idx == 0
        add_cloud(fig, row, col, item["obs"]["points"], showlegend=first)
        add_box(fig, row, col, item["eval"]["gt"], "GT observable box", COLORS["gt"], first)
        add_box(fig, row, col, item["base"].get("box"), "baseline estimate_box", COLORS["baseline"], first)
        add_box(fig, row, col, item["fuse"].get("box"), "measured multi-view", COLORS["fusion"], first)
        add_box(fig, row, col, learned.get("box"), "learned residual", COLORS["learned"], first)
        scene_name = "scene" if idx == 0 else "scene%d" % (idx + 1)
        fig.update_layout({
            scene_name: dict(
                aspectmode="data",
                xaxis_title="x (m)",
                yaxis_title="y (m)",
                zaxis_title="z (m)",
            )
        })

    fig.update_layout(
        title=(
            "LRF-P1 offline visualization (not Gazebo). "
            "Gray = observed points. Green = GT. Blue = baseline. "
            "Purple = extra-view fusion. Red = learned residual. "
            "Drag to rotate."
        ),
        height=520 * rows,
        width=1500,
        legend=dict(orientation="h", y=1.08),
        margin=dict(l=0, r=0, t=90, b=0),
    )
    out = os.path.abspath(args.out)
    fig.write_html(out, include_plotlyjs=True, full_html=True)
    print("wrote", out)
    for item, learned, scores in scored:
        print("%s  %s  %s  baseline=%s  fusion=%s  learned=%s" % (
            item["eval"]["sample_id"],
            item["eval"]["mesh_id"],
            item["eval"]["occlusion_bucket"],
            scores["baseline"]["composite"],
            scores["fusion"]["composite"],
            scores["learned"]["composite"],
        ))
    if args.open:
        webbrowser.open("file://%s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
