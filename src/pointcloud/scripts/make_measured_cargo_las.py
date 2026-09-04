#!/usr/bin/env python3
"""Synthetic cropped cargo LAS for pipeline testing."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

try:
    import laspy
except ImportError as exc:
    raise SystemExit("pip install -r requirements.txt") from exc


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", type=Path, required=True)
    args = parser.parse_args()
    rng = np.random.default_rng(7)

    def rand_box(low, high, n):
        return np.column_stack([rng.uniform(low[i], high[i], n) for i in range(3)])

    def wall_face(x0, x1, y0, y1, z0, z1, n):
        return rand_box([x0, y0, z0], [x1, y1, z1], n)

    ground = rand_box([-1.5, -1.5, -0.02], [1.5, 1.5, 0.02], 2000)
    legs = []
    for cx, cy in [(-1.0, -0.8), (1.0, -0.8), (-1.0, 0.8), (1.0, 0.8)]:
        legs.append(rand_box([cx - 0.08, cy - 0.08, 0.0], [cx + 0.08, cy + 0.08, 0.12], 600))
    legs = np.vstack(legs)
    floor = rand_box([-1.2, -1.0, 0.12], [1.2, 1.0, 0.17], 4000)
    back = wall_face(-1.2, 1.2, -1.0, -0.95, 0.17, 2.2, 5000)
    front_l = wall_face(-1.2, -0.55, 0.95, 1.0, 0.17, 2.2, 2500)
    front_r = wall_face(0.55, 1.2, 0.95, 1.0, 0.17, 2.2, 2500)
    lintel = wall_face(-0.55, 0.55, 0.95, 1.0, 2.05, 2.2, 1500)
    left = wall_face(-1.2, -1.15, -0.95, 0.95, 0.17, 2.15, 4500)
    right = wall_face(1.15, 1.2, -0.95, 0.95, 0.17, 2.15, 4500)
    roof = wall_face(-1.2, 1.2, -1.0, 1.0, 2.15, 2.2, 4000)
    xyz = np.vstack((ground, legs, floor, back, front_l, front_r, lintel, left, right, roof))
    rgb = np.tile([0.3, 0.4, 0.5], (xyz.shape[0], 1))

    las = laspy.create(point_format=3, file_version="1.4")
    las.x, las.y, las.z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    las.red = (rgb[:, 0] * 65535).astype(np.uint16)
    las.green = (rgb[:, 1] * 65535).astype(np.uint16)
    las.blue = (rgb[:, 2] * 65535).astype(np.uint16)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    las.write(str(args.output))
    print(f"Wrote {xyz.shape[0]} points -> {args.output}")


if __name__ == "__main__":
    main()
