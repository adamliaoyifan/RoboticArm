"""gen_sfree — deterministic S-free scene generator (seed 20261105).

Contract §4: when the reused 30k fixtures cannot guarantee a probe-box
placement region on OBSERVED known-free, a deterministic sensor-ray
generator builds the scene (source/params/payloads frozen, no point
duplication). Coverage analysis (commit 085e785d) showed every run793
30k combo has zero observed-free columns, so this generator is the
sanctioned S-free path.

Scene model (container frame, cuboid + chamfer):
- Empty container, floor at ``floor_z``, usable z from ``floor_z + 0.06``
  (above the fixture integrate_min_z) to the ceiling.
- One camera grid ON the door plane (x = -door_offset): 3 y positions x
  3 z positions = 9 frames, exactly like a scanning session. Each frame
  sends a dense ray fan onto the far interior surface: the back wall
  plane for most targets, the chamfer plane where it applies (chamfer
  hull only). Endpoints are wall HITS; every interior cell a ray crosses
  becomes observed-free — with 9 origins the fan stack crosses every
  column at every usable height, so whole columns classify free (the
  surface rule requires NO unknown cell in the column).
- Deterministic: no RNG in the geometry; seed 20261105 fixes the point
  ordering/subsample so payloads are byte-stable.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

SEED = 20261105

#: Fixture-lineage sensor pose convention: optical->container rotation
#: (row-major) and door-plane origin grid.
ROTATION = [0, 0, 1, -1, 0, 0, 0, -1, 0]
ORIGINS_YZ = [(-0.8, 0.75), (0.0, 0.75), (0.8, 0.75),
              (-0.8, 1.20), (0.0, 1.20), (0.8, 1.20),
              (-0.8, 1.70), (0.0, 1.70), (0.8, 1.70),
              # coverage proof round (adjudication 2026-11-08): the top
              # band above every origin stayed unobserved with 3 z rows —
              # a fourth row near the ceiling closes the column knowledge
              (-0.8, 1.95), (0.0, 1.95), (0.8, 1.95)]
DOOR_X = -1.15
TARGET_STEP = 0.02
WALL_EPS = 0.005

#: The observed support: one low box near the container center. Its top
#: (floor + 0.20 m) sits above the fixture integrate_min_z line, so the
#: top surface is OBSERVABLE — a probe box on it is the genuine
#: known-free positive (floor-level columns can never be observed under
#: the min_z policy; first-box-on-floor is always floor-prior).
SUPPORT_BOX = {"center": (0.10, -0.30), "size": (0.90, 0.60, 0.20)}


def _matrix(rotation):
    return [[float(rotation[0]), float(rotation[1]), float(rotation[2])],
            [float(rotation[3]), float(rotation[4]), float(rotation[5])],
            [float(rotation[6]), float(rotation[7]), float(rotation[8])]]


def _transpose(m):
    return [[m[r][c] for r in range(3)] for c in range(3)]


def _matvec(m, v):
    return [sum(m[r][c] * v[c] for c in range(3)) for r in range(3)]


def _sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _box_hit(box, floor, origin, target):
    """First intersection of the door->target ray with the support box,
    or None. Slab method, container frame."""
    cx, cy = box["center"]
    bw, bd, bh = box["size"]
    lo = [cx - bw / 2.0, cy - bd / 2.0, floor]
    hi = [cx + bw / 2.0, cy + bd / 2.0, floor + bh]
    direction = _sub(target, origin)
    t_enter, t_exit = 0.0, 1.0
    for axis in range(3):
        if abs(direction[axis]) < 1e-12:
            if not (lo[axis] - 1e-9 <= origin[axis] <= hi[axis] + 1e-9):
                return None
            continue
        t1 = (lo[axis] - origin[axis]) / direction[axis]
        t2 = (hi[axis] - origin[axis]) / direction[axis]
        if t1 > t2:
            t1, t2 = t2, t1
        t_enter = max(t_enter, t1)
        t_exit = min(t_exit, t2)
        if t_enter > t_exit:
            return None
    if t_enter <= 0.0 or t_enter > 1.0:
        return None
    return [origin[i] + t_enter * direction[i] for i in range(3)]


def _chamfer_hit(hull, target, origin):
    """Where a door->target ray crosses the chamfer plane (if before the
    back wall). Chamfer: on positive_y, wall recedes from wall_y at
    z<=wall_z to floor_y at the ceiling."""
    cham = hull.get("chamfer")
    if not cham or cham.get("side") != "positive_y":
        return None
    half_l = float(hull["length"]) / 2.0
    ceiling = float(hull["ceiling_z"])
    wall_z = float(cham["wall_z"])
    wall_y = float(cham["wall_y"])
    floor_y = float(cham["floor_y"])
    direction = _sub(target, origin)
    # plane through (half_l-eps', y=w(y varies with z), ...) — model the
    # chamfer as a slanted plane through points (y=wall_y, z=wall_z) and
    # (y=floor_y, z=ceiling) at x=half_l (the wall plane).
    # Plane normal in (y, z): n = (ceiling - wall_z, wall_y - floor_y)
    ny = ceiling - wall_z
    nz = wall_y - floor_y
    d = ny * wall_y + nz * wall_z
    # solve origin + s*dir on the plane, with x reaching the wall
    s_x = (half_l - WALL_EPS - origin[0]) / direction[0]
    point = [origin[i] + s_x * direction[i] for i in range(3)]
    on_plane = ny * point[1] + nz * point[2]
    if point[1] >= wall_y - 1e-9 and point[2] >= wall_z - 1e-9:
        # inside the chamfer band: slide the hit onto the slanted plane
        # by bisecting along the ray toward the door until on_plane.
        lo, hi = 0.0, s_x
        for _ in range(40):
            mid = (lo + hi) / 2.0
            p = [origin[i] + mid * direction[i] for i in range(3)]
            if ny * p[1] + nz * p[2] > d:
                hi = mid
            else:
                lo = mid
        s = (lo + hi) / 2.0
        return [origin[i] + s * direction[i] for i in range(3)]
    return None


def generate_frames(hull, *, origins_yz=ORIGINS_YZ, step=TARGET_STEP):
    """Deterministic fixture documents (the run793 observation schema)."""
    floor = float(hull["floor_z"])
    ceiling = float(hull["ceiling_z"])
    half_w = float(hull["width"]) / 2.0
    half_l = float(hull["length"]) / 2.0
    z_min = floor + 0.06
    z_max = ceiling - 0.02
    wall_x = half_l - WALL_EPS
    rotation = _matrix(ROTATION)
    rotation_t = _transpose(rotation)

    frames = []
    for index, (oy, oz) in enumerate(origins_yz):
        origin = [DOOR_X, oy, oz]
        points_optical = []
        y = -half_w + WALL_EPS
        while y <= half_w - WALL_EPS + 1e-9:
            z = z_min
            while z <= z_max + 1e-9:
                target = [wall_x, y, z]
                hit = _box_hit(SUPPORT_BOX, floor, origin, target)
                if hit is None:
                    hit = _chamfer_hit(hull, target, origin)
                world = hit or target
                optical = _matvec(rotation_t, _sub(world, origin))
                points_optical.append(
                    [round(optical[0], 6), round(optical[1], 6),
                     round(optical[2], 6)])
                z += step
            y += step
        # Dense sampling ON the support box's visible surfaces: the wall
        # fan hits the top only at grazing incidence, so the top is
        # explicitly scanned (a real session aims at the load surface).
        cx, cy = SUPPORT_BOX["center"]
        bw, bd, bh = SUPPORT_BOX["size"]
        top_z = floor + bh
        bx = cx - bw / 2.0 + 0.01
        while bx <= cx + bw / 2.0 - 0.01 + 1e-9:
            by = cy - bd / 2.0 + 0.01
            while by <= cy + bd / 2.0 - 0.01 + 1e-9:
                world = [bx, by, top_z - 0.002]
                optical = _matvec(rotation_t, _sub(world, origin))
                points_optical.append(
                    [round(optical[0], 6), round(optical[1], 6),
                     round(optical[2], 6)])
                by += step
            bx += step
        # deterministic ordering: seed-tagged shuffle keeps payloads
        # byte-stable without favoring any spatial pattern.
        import random
        rng = random.Random(SEED + index)
        rng.shuffle(points_optical)
        frames.append({
            "sensor_id": "cam0",
            "observation_id": "sfree%d-%s-%d" % (
                SEED, hull.get("geometry_hash", "")[:8], index),
            "optical_frame": "d555_color_optical_frame",
            "clock_domain": "ros",
            "geometry_hash": str(hull["geometry_hash"]),
            "epoch": 1,
            "stamp_ns": (index + 1) * 1_000_000_000,
            "points_m": points_optical,
            "transform": {
                "stamp_ns": (index + 1) * 1_000_000_000,
                "translation": [origin[0], origin[1], origin[2]],
                "rotation": list(ROTATION),
            },
        })
    return frames


def write_frames(frames, out_dir):
    """Freeze payloads as YAML observation documents + a manifest."""
    import hashlib
    import yaml
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    manifest = {"seed": SEED, "frames": []}
    for frame in frames:
        name = "%s.yaml" % frame["observation_id"]
        text = yaml.safe_dump(frame, sort_keys=True)
        (out / name).write_text(text)
        manifest["frames"].append({
            "path": str(out / name),
            "sha256": hashlib.sha256((out / name).read_bytes()).hexdigest(),
            "n_points": len(frame["points_m"]),
        })
    (out / "MANIFEST.json").write_text(json.dumps(manifest, indent=1))
    return manifest


def main():
    import argparse
    import yaml
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", required=True,
                        help="geometry export yaml (descriptor + hash)")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    document = yaml.safe_load(open(args.geometry).read())
    hull = json.loads(document["descriptor_json"])
    frames = generate_frames(hull)
    manifest = write_frames(frames, args.out)
    print("sfree: %d frames, %d points/frame -> %s"
          % (len(frames), manifest["frames"][0]["n_points"], args.out))


if __name__ == "__main__":
    main()
