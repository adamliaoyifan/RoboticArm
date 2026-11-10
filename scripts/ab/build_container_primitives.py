#!/usr/bin/env python3
"""Build representation-B: primitive-box collision for the real container.

A/B single-variable control (frozen contract 2026-10-19). A = the live
airport_container_real single trimesh collision (2k-decimated physics
STL; measured: axis-aligned box cavity, NO chamfer, leaky front face).
B = box primitives fitted to A's MEASURED inner planes (median of
inward-facing vertices), with the designed aperture (semantic rect) cut
into the -X face. Wall slabs extend to the STL bbox so outer bounds
match. Same visual mesh, same friction.

Findings baked into the fit (2026-10-19 survey of the physics STL):
  cavity x [-0.775, +0.765], y [-1.040, +0.970],
  floor_z 0.5285, ceiling_z 2.117, mu 0.6,
  world placement pure yaw (level floor - gravity tilt excluded).
"""
import argparse
import json
import os
import shutil

import numpy as np
import open3d as o3d

CAVITY = {"x_lo": -0.775, "x_hi": 0.765, "y_lo": -1.040, "y_hi": 0.970,
          "floor_z": 0.5285, "ceiling_z": 2.117}
BBOX = {"x_lo": -0.7895, "x_hi": 0.774, "y_lo": -1.0464, "y_hi": 0.9746,
        "z_lo": 0.0119, "z_hi": 2.1178}
APERTURE = {"y_lo": -0.928, "y_hi": 0.398, "z_lo": 0.597, "z_hi": 2.009}


def verify_against_stl(stl_path, margin=0.004):
    """Re-derive the cavity planes from the STL and confirm the baked
    constants still hold (the builder must not drift from A)."""
    mesh = o3d.io.read_triangle_mesh(stl_path)
    v = np.asarray(mesh.vertices)
    n = np.asarray(mesh.vertex_normals)
    c = v.mean(axis=0)
    inw = (n * (c - v)).sum(axis=1) > 0
    vi, ni = v[inw], n[inw]

    def plane(dirvec, coord, lo, hi):
        sel = ni.dot(dirvec) / (np.linalg.norm(dirvec) + 1e-9) > 0.9
        vals = vi[sel][:, coord]
        vals = vals[(vals > lo) & (vals < hi)]
        return float(np.median(vals)) if len(vals) else None

    checks = {
        "floor_z": plane([0, 0, 1], 2, 0.3, 1.5),
        "ceiling_z": plane([0, 0, -1], 2, 1.5, 2.2),
        # Cavity-facing normals: -y wall faces +y; +y wall faces -y;
        # -x face faces +x; +x wall faces -x (verified on the STL).
        "y_lo_wall": plane([0, 1, 0], 1, -1.15, -0.95),
        "y_hi_wall": plane([0, -1, 0], 1, 0.9, 1.1),
        "x_lo_face": plane([1, 0, 0], 0, -0.85, -0.7),
        "x_hi_wall": plane([-1, 0, 0], 0, 0.7, 0.85),
    }
    expect = {"floor_z": CAVITY["floor_z"],
              "ceiling_z": CAVITY["ceiling_z"],
              "y_lo_wall": CAVITY["y_lo"], "y_hi_wall": CAVITY["y_hi"],
              "x_lo_face": CAVITY["x_lo"], "x_hi_wall": CAVITY["x_hi"]}
    report = {}
    ok = True
    for key, measured in checks.items():
        if measured is None:
            report[key] = {"measured": None, "ok": False}
            ok = False
            continue
        delta = abs(measured - expect[key])
        good = delta <= margin
        ok = ok and good
        report[key] = {"measured": round(measured, 5),
                       "expected": expect[key],
                       "delta_mm": round(delta * 1000, 2), "ok": good}
    return ok, report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stl", required=True)
    parser.add_argument("--a-model-dir", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--report", default="")
    parser.add_argument("--force", action="store_true",
                        help="emit even if the STL drift check fails")
    args = parser.parse_args()

    ok, drift = verify_against_stl(args.stl)
    cav = dict(CAVITY)
    if not ok:
        if not args.force:
            raise SystemExit(
                "A's STL no longer matches the baked fit (%s); re-fit "
                "before building B" % json.dumps(drift))
        for key in ("floor_z", "ceiling_z"):
            if drift[key]["measured"] is not None:
                cav[key] = drift[key]["measured"]

    x_lo, x_hi = cav["x_lo"], cav["x_hi"]
    y_lo, y_hi = cav["y_lo"], cav["y_hi"]
    fz, cz = cav["floor_z"], cav["ceiling_z"]

    boxes = []

    def box(name, cx, cy, cz_, sx, sy, sz):
        boxes.append({"name": name,
                     "pose": [cx, cy, cz_, 0.0, 0.0, 0.0],
                     "size": [sx, sy, sz]})

    # Floor: cavity footprint extended to the shell bbox, slab bottom at
    # the STL bbox floor.
    box("floor",
        (BBOX["x_lo"] + BBOX["x_hi"]) / 2,
        (BBOX["y_lo"] + BBOX["y_hi"]) / 2,
        (BBOX["z_lo"] + fz) / 2,
        BBOX["x_hi"] - BBOX["x_lo"], BBOX["y_hi"] - BBOX["y_lo"],
        fz - BBOX["z_lo"])
    # Ceiling.
    box("ceiling",
        (BBOX["x_lo"] + BBOX["x_hi"]) / 2,
        (BBOX["y_lo"] + BBOX["y_hi"]) / 2,
        (cz + BBOX["z_hi"]) / 2,
        BBOX["x_hi"] - BBOX["x_lo"], BBOX["y_hi"] - BBOX["y_lo"],
        BBOX["z_hi"] - cz)
    # +X back wall, -Y wall, +Y wall.
    box("wall_pos_x", (x_hi + BBOX["x_hi"]) / 2,
        (BBOX["y_lo"] + BBOX["y_hi"]) / 2, (BBOX["z_lo"] + cz) / 2,
        BBOX["x_hi"] - x_hi, BBOX["y_hi"] - BBOX["y_lo"],
        cz - BBOX["z_lo"])
    box("wall_neg_y", (BBOX["x_lo"] + BBOX["x_hi"]) / 2,
        (BBOX["y_lo"] + y_lo) / 2, (BBOX["z_lo"] + cz) / 2,
        BBOX["x_hi"] - BBOX["x_lo"], y_lo - BBOX["y_lo"],
        cz - BBOX["z_lo"])
    box("wall_pos_y", (BBOX["x_lo"] + BBOX["x_hi"]) / 2,
        (y_hi + BBOX["y_hi"]) / 2, (BBOX["z_lo"] + cz) / 2,
        BBOX["x_hi"] - BBOX["x_lo"], BBOX["y_hi"] - y_hi,
        cz - BBOX["z_lo"])
    # -X face around the aperture: sill, lintel, two pillars.
    face_lo, face_hi = BBOX["x_lo"], x_lo
    ft = face_hi - face_lo
    box("sill", (face_lo + face_hi) / 2,
        (APERTURE["y_lo"] + APERTURE["y_hi"]) / 2, (fz + APERTURE["z_lo"]) / 2,
        ft, APERTURE["y_hi"] - APERTURE["y_lo"], APERTURE["z_lo"] - fz)
    box("lintel", (face_lo + face_hi) / 2,
        (APERTURE["y_lo"] + APERTURE["y_hi"]) / 2,
        (APERTURE["z_hi"] + cz) / 2,
        ft, APERTURE["y_hi"] - APERTURE["y_lo"], cz - APERTURE["z_hi"])
    box("pillar_neg_y", (face_lo + face_hi) / 2,
        (y_lo + APERTURE["y_lo"]) / 2, (fz + cz) / 2,
        ft, APERTURE["y_lo"] - y_lo, cz - fz)
    box("pillar_pos_y", (face_lo + face_hi) / 2,
        (APERTURE["y_hi"] + y_hi) / 2, (fz + cz) / 2,
        ft, y_hi - APERTURE["y_hi"], cz - fz)

    os.makedirs(os.path.join(args.out, "meshes"), exist_ok=True)
    shutil.copy(os.path.join(args.a_model_dir, "meshes",
                            "container_visual.stl"),
                os.path.join(args.out, "meshes", "container_visual.stl"))
    a_sdf = open(os.path.join(args.a_model_dir, "model.sdf")).read()
    mu = "0.6"
    for line in a_sdf.splitlines():
        if "<mu>" in line:
            mu = line.split("<mu>")[1].split("</mu>")[0].strip()
            break

    sdf = ['<?xml version="1.0"?>', '<sdf version="1.6">',
           '  <model name="airport_container_real_prim">',
           '    <static>true</static>',
           '    <link name="container_link">',
           '      <pose>0 0 0 0 0 0</pose>',
           '      <visual name="visual">',
           '        <geometry><mesh>',
           '          <uri>model://airport_container_real_prim/meshes/'
           'container_visual.stl</uri>',
           '          <scale>1 1 1</scale>',
           '        </mesh></geometry>',
           '        <material>',
           '          <ambient>0.25 0.35 0.45 1</ambient>',
           '          <diffuse>0.35 0.45 0.55 1</diffuse>',
           '        </material>',
           '      </visual>']
    for b in boxes:
        sdf.append(
            '      <collision name="collision_%s">'
            '<pose>%s</pose>'
            '<geometry><box><size>%s</size></box></geometry>'
            '<surface><friction><ode><mu>%s</mu><mu2>%s</mu2></ode>'
            '</friction></surface></collision>' % (
                b["name"],
                " ".join("%.9f" % v for v in b["pose"]),
                " ".join("%.9f" % v for v in b["size"]), mu, mu))
    sdf += ['    </link>', '  </model>', '</sdf>', '']
    open(os.path.join(args.out, "model.sdf"), "w").write("\n".join(sdf))
    open(os.path.join(args.out, "model.config"), "w").write(
        '<?xml version="1.0"?>\n<model>\n  <name>'
        'airport_container_real_prim</name>\n  <version>1.0</version>\n'
        '  <sdf version="1.6">model.sdf</sdf>\n'
        '  <description>Representation B: axis-aligned primitive '
        "collision fitted to A's measured physics-STL inner boundary "
        '(A/B control)</description>\n</model>\n')

    report = {"stl_drift_check": drift, "drift_ok": ok,
              "cavity": cav, "bbox": BBOX, "aperture": APERTURE,
              "mu": mu, "boxes": boxes}
    out = args.report or os.path.join(args.out, "fit_report.json")
    with open(out, "w") as handle:
        json.dump(report, handle, indent=2)
    print("drift_ok=%s boxes=%d mu=%s -> %s" % (ok, len(boxes), mu,
                                                args.out))


if __name__ == "__main__":
    main()
