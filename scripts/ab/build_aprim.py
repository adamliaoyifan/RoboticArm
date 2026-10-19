#!/usr/bin/env python3
"""Generate A' (baseline trimesh) and B (baseline primitives) for the
A0/A'/B control from the FROZEN COMMON BASELINE (design_baseline.py).

A' = panel triangles (each panel a closed trimesh, concatenated). B = the
same panels as SDF <box> elements; the chamfer wall is an exact
trapezoid prism in A' and ONE rotated box in B whose inner face lies
exactly on the slope plane. Both assets' inner faces lie on the baseline
planes. A manifest (inputs, parameters, hashes, runtime load paths) is
written per the frozen pre-check 3. A0 is untouched.
"""
import argparse
import hashlib
import json
import os
import shutil
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from design_baseline import load_baseline, panels, inside_cavity  # noqa: E402


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def read_mu(a_model_dir):
    for line in open(os.path.join(a_model_dir, "model.sdf")):
        if "<mu>" in line:
            return line.split("<mu>")[1].split("</mu>")[0].strip()
    return "0.6"


def wedge_trimesh(w, b):
    """Exact trapezoid prism for the chamfer wall (A') — BOTTOM chamfer:
    slope from (floor_y, floor_z) to (y_hi, wall_z)."""
    y_hi, wall_z = b["y_hi"], b["chamfer_wall_z"]
    floor_y, fz = b["chamfer_floor_y"], b["floor_z"]
    oy = b["o_y_hi"]
    x0, x1 = w["x_span"]
    cross = [(floor_y, fz), (y_hi, wall_z), (oy, wall_z), (oy, fz)]
    return _prism(x0, x1, cross)


def end_wedge_trimesh(w, b):
    """Exact triangle prism for the end-wall corner in the chamfer zone
    (A'): triangle (floor_y, floor_z), (y_hi, floor_z), (y_hi, wall_z)."""
    y_hi, wall_z = b["y_hi"], b["chamfer_wall_z"]
    floor_y, fz = b["chamfer_floor_y"], b["floor_z"]
    x0, x1 = w["x_span"]
    return _prism(x0, x1, [(floor_y, fz), (y_hi, wall_z),
                           (y_hi, fz)])


def _prism(x0, x1, cross):
    n = len(cross)
    verts = [[x0, y, z] for (y, z) in cross] + \
            [[x1, y, z] for (y, z) in cross]
    faces = []
    cap0 = [(0, (i + 1) % n, (i + 2) % n) for i in range(n - 2)]
    faces.extend([(a, b_, c) for (a, b_, c) in cap0])
    faces.extend([(n + a, n + c, n + b_) for (a, b_, c) in cap0])
    for i in range(n):
        j = (i + 1) % n
        faces.append((i, j, n + j))
        faces.append((i, n + j, n + i))
    mesh = trimesh.Trimesh(vertices=verts, faces=faces)
    if mesh.volume < 0:
        mesh.invert()
    return mesh


def wedge_box(w, b):
    """B's chamfer wedge as ONE rotated box: inner face exactly on the
    slope plane (bottom chamfer: (floor_y, floor_z) -> (y_hi, wall_z)),
    corners kept inside the outer +Y face and the subfloor solid."""
    y_hi, wall_z = b["y_hi"], b["chamfer_wall_z"]
    floor_y, fz = b["chamfer_floor_y"], b["floor_z"]
    dy, dz = y_hi - floor_y, wall_z - fz
    slope_len = (dy * dy + dz * dz) ** 0.5
    # Outward normal (into the wall): the slope direction is
    # (dy, dz)/L, so the normal is (dz, -dy)/L (perpendicular!).
    ny, nz = dz / slope_len, -dy / slope_len
    roll = float(np.arctan2(nz, ny))
    # Thickness bound: the upper outer corner stays within o_y_hi.
    t_max = 2.0 * (b["o_y_hi"] - y_hi) / max(ny, 1e-6)
    t = min(0.7 * t_max, 0.09)
    mid = np.array([0.0, (floor_y + y_hi) / 2, (fz + wall_z) / 2])
    center = mid + np.array([0.0, ny, nz]) * (t / 2)
    return {"name": "%s_box" % w["name"],
            "center": [float(center[0]), float(center[1]), float(center[2])],
            "size": [w["x_span"][1] - w["x_span"][0], t, slope_len],
            "roll": roll}


VISUAL_BLOCK = """      <visual name="visual">
        <geometry><mesh>
          <uri>model://%(name)s/meshes/container_visual.stl</uri>
          <scale>1 1 1</scale>
        </mesh></geometry>
        <material>
          <ambient>0.25 0.35 0.45 1</ambient>
          <diffuse>0.35 0.45 0.55 1</diffuse>
        </material>
      </visual>"""

MODEL_CONFIG = ('<?xml version="1.0"?>\n<model>\n  <name>%s</name>\n'
                '  <version>1.0</version>\n  <sdf version="1.6">'
                'model.sdf</sdf>\n</model>\n')


def build_aprime(b, out_dir, a_model_dir):
    parts = []
    for p in panels(b):
        if p["kind"] == "box":
            m = trimesh.creation.box(extents=p["size"])
            m.apply_translation(p["center"])
        elif p["kind"] == "wedge":
            m = wedge_trimesh(p, b)
        else:
            m = end_wedge_trimesh(p, b)
        parts.append(m)
    mesh = trimesh.util.concatenate(parts)
    os.makedirs(os.path.join(out_dir, "meshes"), exist_ok=True)
    stl = os.path.join(out_dir, "meshes", "container_collision.stl")
    mesh.export(stl)
    shutil.copy(os.path.join(a_model_dir, "meshes", "container_visual.stl"),
                os.path.join(out_dir, "meshes", "container_visual.stl"))
    mu = read_mu(a_model_dir)
    name = "airport_container_real_aprime"
    open(os.path.join(out_dir, "model.sdf"), "w").write(
        MESH_SDF % {"name": name, "mu": mu,
                    "visual": VISUAL_BLOCK % {"name": name}})
    open(os.path.join(out_dir, "model.config"), "w").write(
        MODEL_CONFIG % name)
    return {"tris": int(len(mesh.faces)),
            "watertight": bool(mesh.is_watertight),
            "volume_m3": round(float(mesh.volume), 4),
            "stl_sha256": sha256(stl)}


MESH_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="%(name)s">
    <static>true</static>
    <link name="container_link">
      <pose>0 0 0 0 0 0</pose>
%(visual)s
      <collision name="collision_mesh">
        <geometry><mesh>
          <uri>model://%(name)s/meshes/container_collision.stl</uri>
          <scale>1 1 1</scale>
        </mesh></geometry>
        <surface><friction><ode><mu>%(mu)s</mu><mu2>%(mu)s</mu2></ode>
</friction></surface>
      </collision>
    </link>
  </model>
</sdf>
"""


def build_b(b, out_dir, a_model_dir):
    boxes = []
    for p in panels(b):
        if p["kind"] == "box":
            boxes.append({"name": p["name"], "center": p["center"],
                          "size": p["size"], "roll": 0.0})
        else:
            # wedge / end_wedge: one rotated box with its inner face
            # exactly on the chamfer slope plane.
            boxes.append(wedge_box(p, b))
    os.makedirs(os.path.join(out_dir, "meshes"), exist_ok=True)
    shutil.copy(os.path.join(a_model_dir, "meshes", "container_visual.stl"),
                os.path.join(out_dir, "meshes", "container_visual.stl"))
    mu = read_mu(a_model_dir)
    name = "airport_container_real_bprim"
    sdf = ['<?xml version="1.0"?>', '<sdf version="1.6">',
           '  <model name="%s">' % name,
           '    <static>true</static>',
           '    <link name="container_link">',
           '      <pose>0 0 0 0 0 0</pose>',
           VISUAL_BLOCK % {"name": name}]
    for bxx in boxes:
        pose = list(bxx["center"]) + [bxx["roll"], 0.0, 0.0]
        sdf.append(
            '      <collision name="collision_%s">'
            '<pose>%s</pose>'
            '<geometry><box><size>%s</size></box></geometry>'
            '<surface><friction><ode><mu>%s</mu><mu2>%s</mu2></ode>'
            '</friction></surface></collision>' % (
                bxx["name"],
                " ".join("%.9f" % v for v in pose),
                " ".join("%.9f" % v for v in bxx["size"]), mu, mu))
    sdf += ['    </link>', '  </model>', '</sdf>', '']
    path = os.path.join(out_dir, "model.sdf")
    open(path, "w").write("\n".join(sdf))
    open(os.path.join(out_dir, "model.config"), "w").write(
        MODEL_CONFIG % name)
    return {"boxes": [{k: v for k, v in bx.items()}
                      for bx in boxes], "sdf_sha256": sha256(path)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--a-model-dir", required=True)
    parser.add_argument("--out-root", default="src/luggage_gazebo/models")
    parser.add_argument("--check-release", nargs=5, metavar=(
        "X", "Y", "Z", "HALF_X", "HALF_Y"))
    args = parser.parse_args()

    b = load_baseline()
    if args.check_release:
        x, y, z, hx, hy = map(float, args.check_release)
        corners = [(x + sx * hx, y + sy * hy, z) for sx in (-1, 1)
                   for sy in (-1, 1)]
        valid = all(inside_cavity(b, c) for c in corners)
        print("release footprint valid under baseline:", valid)
        return

    aprime_dir = os.path.join(args.out_root, "airport_container_real_aprime")
    bprim_dir = os.path.join(args.out_root, "airport_container_real_bprim")
    a_info = build_aprime(b, aprime_dir, args.a_model_dir)
    b_info = build_b(b, bprim_dir, args.a_model_dir)
    manifest = {
        "baseline_source": b["source"],
        "baseline": b,
        "baseline_yaml_sha256": sha256(b["source"]),
        "a_model_dir": os.path.realpath(args.a_model_dir),
        "a0_stl_sha256": sha256(os.path.join(
            args.a_model_dir, "meshes", "container_collision_physics.stl")),
        "aprime": a_info, "bprim": b_info,
        "runtime": {
            "resource_share": "install/luggage_gazebo/share/"
                              "luggage_gazebo/models",
            "a0_loads": "model://airport_container_real "
                        "(container_collision_physics.stl) — untouched",
            "note": "A'/B load their model:// dirs after colcon install; "
                    "the experiment swaps the spawned model name only.",
        },
    }
    out = os.path.join(args.out_root, "ab_generation_manifest.json")
    with open(out, "w") as handle:
        json.dump(manifest, handle, indent=2)
    print("A' tris=%(tris)s watertight=%(watertight)s vol=%(volume_m3)s"
          % a_info)
    print("B boxes=%d" % len(b_info["boxes"]))
    print("manifest:", out)


if __name__ == "__main__":
    main()
