#!/usr/bin/env python3
"""Generate a small synthetic room LAS with an embedded Cargo box for pipeline testing."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

try:
    import laspy
except ImportError as exc:
    raise SystemExit("pip install -r requirements.txt") from exc


def _box_points(
    center: np.ndarray,
    size: np.ndarray,
    n: int,
    rng: np.random.Generator,
) -> np.ndarray:
    half = size / 2
    pts = []
    for axis in range(3):
        for sign in (-1, 1):
            face = center.copy()
            face[axis] += sign * half[axis]
            tang = [i for i in range(3) if i != axis]
            u = rng.uniform(-half[tang[0]], half[tang[0]], n // 6)
            v = rng.uniform(-half[tang[1]], half[tang[1]], n // 6)
            block = np.tile(face, (len(u), 1))
            block[:, tang[0]] += u
            block[:, tang[1]] += v
            pts.append(block)
    return np.vstack(pts)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--cargo-center", type=float, nargs=3, default=[3.0, 0.0, 0.0])
    args = parser.parse_args()

    rng = np.random.default_rng(42)
    def _rand_box(low, high, n):
        return np.column_stack(
            [rng.uniform(low[i], high[i], n) for i in range(3)]
        )

    floor = _rand_box([-1, -3, 0], [8, 3, 0], 8000)
    wall_x = _rand_box([7.5, -3, 0], [8, 3, 3], 3000)
    wall_y = _rand_box([-1, 2.8, 0], [8, 3, 3], 3000)
    cargo = _box_points(
        np.asarray(args.cargo_center),
        np.array([2.4, 2.0, 2.2]),
        12000,
        rng,
    )
    xyz = np.vstack((floor, wall_x, wall_y, cargo))
    intensity = np.concatenate(
        [
            rng.integers(100, 300, floor.shape[0]),
            rng.integers(100, 300, wall_x.shape[0] + wall_y.shape[0]),
            rng.integers(800, 1200, cargo.shape[0]),
        ]
    )
    rgb = np.zeros((xyz.shape[0], 3))
    rgb[: floor.shape[0]] = [0.6, 0.55, 0.5]
    rgb[floor.shape[0] : floor.shape[0] + wall_x.shape[0] + wall_y.shape[0]] = [
        0.9,
        0.9,
        0.85,
    ]
    rgb[-cargo.shape[0] :] = [0.2, 0.35, 0.55]

    las = laspy.create(point_format=3, file_version="1.4")
    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]
    las.intensity = intensity.astype(np.uint16)
    las.red = (rgb[:, 0] * 65535).astype(np.uint16)
    las.green = (rgb[:, 1] * 65535).astype(np.uint16)
    las.blue = (rgb[:, 2] * 65535).astype(np.uint16)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    las.write(str(args.output))
    print(f"Wrote {xyz.shape[0]} points -> {args.output}")
    print(f"Suggested cargo.center: {args.cargo_center}")


if __name__ == "__main__":
    main()
