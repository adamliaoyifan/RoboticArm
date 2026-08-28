#!/usr/bin/env python3
"""Extract horizontal support surfaces from a binary or ASCII STL.

This is intentionally dependency-free so the E12 floor evidence can be
reproduced both on the host and in the ROS Noetic container.
"""
from __future__ import division

import argparse
import hashlib
import math
import os
import struct

import yaml


def _triangle_area(a, b, c):
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    cx = uy * vz - uz * vy
    cy = uz * vx - ux * vz
    cz = ux * vy - uy * vx
    return 0.5 * math.sqrt(cx * cx + cy * cy + cz * cz), (cx, cy, cz)


def _binary_triangles(path):
    with open(path, "rb") as stream:
        stream.read(80)
        raw = stream.read(4)
        if len(raw) != 4:
            raise ValueError("truncated binary STL header")
        count = struct.unpack("<I", raw)[0]
        for _ in range(count):
            record = stream.read(50)
            if len(record) != 50:
                raise ValueError("truncated binary STL triangle")
            values = struct.unpack("<12fH", record)
            yield (
                values[3:6],
                values[6:9],
                values[9:12],
            )


def _ascii_triangles(path):
    vertices = []
    with open(path, "r", encoding="utf-8", errors="ignore") as stream:
        for line in stream:
            words = line.strip().split()
            if len(words) == 4 and words[0].lower() == "vertex":
                vertices.append(tuple(float(v) for v in words[1:]))
                if len(vertices) == 3:
                    yield tuple(vertices)
                    vertices = []


def _is_binary_stl(path):
    size = os.path.getsize(path)
    if size < 84:
        return False
    with open(path, "rb") as stream:
        stream.seek(80)
        raw = stream.read(4)
    if len(raw) != 4:
        return False
    return 84 + struct.unpack("<I", raw)[0] * 50 == size


def analyze_floor_surfaces(path, horizontal_cos=0.995, z_bin=0.005,
                           floor_search_min=0.40, floor_search_max=0.65):
    triangles = _binary_triangles(path) if _is_binary_stl(path) else _ascii_triangles(path)
    levels = {}
    total_triangles = 0
    for triangle in triangles:
        total_triangles += 1
        area, cross = _triangle_area(*triangle)
        norm = math.sqrt(sum(v * v for v in cross))
        if norm <= 1e-12 or abs(cross[2]) / norm < horizontal_cos:
            continue
        z = sum(vertex[2] for vertex in triangle) / 3.0
        key = round(z / z_bin) * z_bin
        entry = levels.setdefault(key, {
            "area": 0.0,
            "triangle_count": 0,
            "x_min": float("inf"),
            "x_max": float("-inf"),
            "y_min": float("inf"),
            "y_max": float("-inf"),
            "z_min": float("inf"),
            "z_max": float("-inf"),
        })
        entry["area"] += area
        entry["triangle_count"] += 1
        for x, y, vertex_z in triangle:
            entry["x_min"] = min(entry["x_min"], x)
            entry["x_max"] = max(entry["x_max"], x)
            entry["y_min"] = min(entry["y_min"], y)
            entry["y_max"] = max(entry["y_max"], y)
            entry["z_min"] = min(entry["z_min"], vertex_z)
            entry["z_max"] = max(entry["z_max"], vertex_z)

    rows = []
    for z, entry in sorted(levels.items()):
        row = {"z": round(z, 6)}
        row.update({
            "area": round(entry["area"], 6),
            "triangle_count": entry["triangle_count"],
            "x_range": [round(entry["x_min"], 6), round(entry["x_max"], 6)],
            "y_range": [round(entry["y_min"], 6), round(entry["y_max"], 6)],
            "z_range": [round(entry["z_min"], 6), round(entry["z_max"], 6)],
        })
        rows.append(row)

    floor_rows = [
        row for row in rows
        if floor_search_min <= row["z"] <= floor_search_max
    ]
    support = max(floor_rows, key=lambda row: row["area"]) if floor_rows else None
    with open(path, "rb") as stream:
        digest = hashlib.sha256(stream.read()).hexdigest()
    return {
        "schema_version": 1,
        "mesh_path": path,
        "mesh_sha256": digest,
        "triangle_count": total_triangles,
        "horizontal_cos_threshold": horizontal_cos,
        "z_bin": z_bin,
        "horizontal_levels": rows,
        "floor_search_range": [floor_search_min, floor_search_max],
        "floor_candidate": support,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mesh")
    parser.add_argument("--output", default="")
    parser.add_argument("--horizontal-cos", type=float, default=0.995)
    parser.add_argument("--z-bin", type=float, default=0.005)
    parser.add_argument("--floor-search-min", type=float, default=0.40)
    parser.add_argument("--floor-search-max", type=float, default=0.65)
    args = parser.parse_args()
    result = analyze_floor_surfaces(
        os.path.abspath(args.mesh),
        horizontal_cos=args.horizontal_cos,
        z_bin=args.z_bin,
        floor_search_min=args.floor_search_min,
        floor_search_max=args.floor_search_max,
    )
    text = yaml.safe_dump(result, default_flow_style=False, sort_keys=False)
    if args.output:
        with open(args.output, "w") as stream:
            stream.write(text)
    else:
        print(text)


if __name__ == "__main__":
    main()
