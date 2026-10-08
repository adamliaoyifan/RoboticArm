#!/usr/bin/env python3
"""Project the unbound cargo cloud onto its RGB and mark the long-edge points."""
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

INK, INK2 = "#0b0b0b", "#52514e"
BLUE, CRIT = "#2a78d6", "#d03b3b"
LABELS = {0: "background", 1: "container", 2: "cargo", 3: "robot"}


def _load(directory):
    directory = Path(directory)
    meta = json.loads((directory / "frame.json").read_text())
    arrays = np.load(directory / "arrays.npz")
    return meta, arrays


def _project(cloud, intr):
    z = cloud[:, 2]
    ok = np.isfinite(z) & (z > 0.05)
    u = np.full(len(cloud), np.nan)
    v = np.full(len(cloud), np.nan)
    u[ok] = intr["fx"] * cloud[ok, 0] / z[ok] + intr["cx"]
    v[ok] = intr["fy"] * cloud[ok, 1] / z[ok] + intr["cy"]
    return u, v, ok


def _sample(image, u, v):
    if image is None:
        return None
    ui = np.rint(u).astype(np.int32)
    vi = np.rint(v).astype(np.int32)
    inside = (
        np.isfinite(u) & np.isfinite(v)
        & (ui >= 0) & (vi >= 0)
        & (ui < image.shape[1]) & (vi < image.shape[0]))
    out = np.full(len(u), -1, dtype=np.int32)
    out[inside] = image[vi[inside], ui[inside]]
    return out


def main():
    anomaly_dir = sys.argv[1]
    control_dir = sys.argv[2]
    out_png = sys.argv[3]
    meta_a, arr_a = _load(anomaly_dir)
    _meta_c, arr_c = _load(control_dir)
    intr = (meta_a.get("components") or {}).get("intrinsics", {}).get("value")
    if intr is None or "cloud" not in arr_a.files or "rgb" not in arr_a.files:
        json.dump({
            "status": "projection_incomplete",
            "anomaly": str(anomaly_dir),
            "has_intrinsics": intr is not None,
            "arrays": list(arr_a.files),
        }, open(out_png + ".json", "w"), indent=2)
        print("projection_incomplete")
        return 2
    anomaly = arr_a["cloud"].astype(np.float64)
    control = arr_c["cloud"].astype(np.float64) if "cloud" in arr_c.files else None
    center = anomaly.mean(0)
    _, _, vt = np.linalg.svd(anomaly - center, full_matrices=False)
    axis = vt[0]
    coord_a = (anomaly - center) @ axis
    if control is not None and len(control):
        coord_c = (control - center) @ axis
        lo, hi = float(coord_c.min()), float(coord_c.max())
    else:
        lo, hi = float(np.percentile(coord_a, 5)), float(np.percentile(coord_a, 95))
    margin = 0.02
    extend = (coord_a < lo - margin) | (coord_a > hi + margin)
    u, v, ok = _project(anomaly, intr)
    extend = extend & ok
    label = _sample(arr_a["label_map"] if "label_map" in arr_a.files else None, u, v)
    inst = _sample(
        arr_a["instance_mask"] if "instance_mask" in arr_a.files else None, u, v)
    rgb = arr_a["rgb"]
    fig, ax = plt.subplots(figsize=(8.8, 5.4), dpi=140)
    ax.imshow(rgb)
    keep = ok & ~extend
    ax.scatter(u[keep], v[keep], s=2, c=BLUE, linewidths=0, label="inside control extent")
    ax.scatter(u[extend], v[extend], s=6, c=CRIT, linewidths=0, label="extends the long edge")
    ax.set_xlim(0, rgb.shape[1])
    ax.set_ylim(rgb.shape[0], 0)
    ax.set_axis_off()
    ax.set_title(
        "Unbound cargo points on the pickup view  %d extend the long edge"
        % int(extend.sum()),
        loc="left", color=INK, fontsize=11)
    ax.legend(frameon=False, loc="lower right", fontsize=8, labelcolor=INK)
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)

    def _hist(values):
        if values is None:
            return None
        sel = values[extend]
        keys, counts = np.unique(sel, return_counts=True)
        return {
            (LABELS.get(int(k), str(int(k)))): int(c)
            for k, c in zip(keys, counts)}

    report = {
        "status": "projected",
        "anomaly_stamp_ns": meta_a.get("stamp_ns"),
        "control_stamp_ns": _meta_c.get("stamp_ns"),
        "anomaly_points": int(len(anomaly)),
        "extending_points": int(extend.sum()),
        "control_axis_range_m": [lo, hi],
        "semantic_labels_of_extending_points": _hist(label),
        "instance_ids_of_extending_points": _hist(inst),
        "anomaly_components": {
            name: body.get("present")
            for name, body in (meta_a.get("components") or {}).items()
            if isinstance(body, dict)},
    }
    json.dump(report, open(out_png + ".json", "w"), indent=2)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
