#!/usr/bin/env python3
"""Unify the box's three surfaces into ONE box frame (offline, 2026-10-19).

User-directed bounded task after gate3_aprime_r5/r6: the perception
surface (lid plateau), the grasp/contact surface (suction plane) and
the physical collision surface (full-AABB primitive) of the SAME box
must be listed in one coordinate frame, with the recorded sizes and
poses of r5/r6, before any side is chosen for the fix. No compensation
is applied here; this script only MEASURES the correspondence.

Inputs are all recorded artifacts: the sized STL, the release SDF dump,
the trial verify poses, the tf_trace suction plane and the contact
stream's S20 manifolds.
"""
import argparse
import collections
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

WS = "/tmp/elfin-master-pack-baseline-g3"
EVID = ("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
        "master_pack_recovery/2026-10-14_g3")
MODELS = (WS + "/install/luggage_gazebo/share/luggage_gazebo/models")
sys.path.insert(0, WS + "/src/luggage_description")
from luggage_description.suitcase_visual import (  # noqa: E402
    mesh_observable_reference)


def stl_vertices(path):
    """Vertices via the package's own STL reader (single source)."""
    from luggage_description.suitcase_visual import iter_stl_vertices
    return [(x, y, z) for x, y, z in iter_stl_vertices(path)]


def struct_unpack(b):
    import struct
    return struct.unpack("<12f", b)


def mesh_stats(stl_path):
    """AABB, lid plateau, and the material ABOVE the plateau."""
    verts = stl_vertices(stl_path)
    zs = [v[2] for v in verts]
    zmin, zmax = min(zs), max(zs)
    w, d, lid_off, full_h = mesh_observable_reference(stl_path)
    # mesh_observable_reference is vertex-histogram based; recompute the
    # plateau z here in the STL frame for the frame-unification table.
    band = max(0.25 * (zmax - zmin), 0.002)
    top = [(x, y, z) for x, y, z in verts if z >= zmax - band]
    bins = collections.Counter(int(z / 0.001) for _x, _y, z in top)
    modal_key = max(bins, key=lambda k: bins[k])
    modal_z = modal_key * 0.001 + 0.0005
    above = [(x, y, z) for x, y, z in verts if z > modal_z + 0.001]
    above_xy = None
    if above:
        xs = [p[0] for p in above]
        ys = [p[1] for p in above]
        above_xy = [min(xs), min(ys), max(xs), max(ys)]
    return {
        "stl": os.path.basename(stl_path),
        "n_vertices": len(verts),
        "z_min": round(zmin, 5), "z_max": round(zmax, 5),
        "full_h": round(full_h, 5),
        "lid_plateau_z": round(modal_z, 5),
        "lid_offset_from_crown": round(zmax - modal_z, 5),
        "observable_w": round(w, 5), "observable_d": round(d, 5),
        "vertices_above_plateau": len(above),
        "fraction_above_plateau": round(len(above) / len(verts), 4),
        "above_plateau_xy_bbox": ([round(v, 4) for v in above_xy]
                                  if above_xy else None),
    }


def sdf_collision(path):
    root = ET.parse(path).getroot()
    for col in root.iter("collision"):
        box = col.find("geometry/box/size")
        if box is not None:
            mask = col.find("surface/contact/collide_bitmask")
            return {
                "size": [round(float(v), 5) for v in box.text.split()],
                "collide_bitmask": (mask.text if mask is not None
                                    else "255(default)"),
                "friction_mu": (col.find("surface/friction/ode/mu").text
                                if col.find("surface/friction/ode/mu")
                                is not None else None),
            }
    return None


def tf_suction_at(path, label):
    for line in open(path):
        d = json.loads(line)
        if d.get("label") == label:
            return d.get("suction"), d.get("t_ros")
    return None, None


def s20_contacts(path, box_id):
    """[(t_ns, lump, positions)] for S20-panel-lump x box contacts.

    Protobuf-text state machine: `sec:`/`nsec:` update the running
    stamp; inside a `contact {` block, `name:` lines accumulate
    collision names and x:/y:/z: triplets accumulate positions.
    """
    out = []
    t_ns = None
    in_contact = False
    names, xs = [], []
    xyz = []

    def flush():
        nonlocal in_contact, names, xs
        if in_contact and any("S20" in n for n in names) \
                and any(box_id in n for n in names):
            lump = next(n for n in names if "S20" in n)
            points = [xs[i:i + 3] for i in range(0, len(xs) - 2, 3)]
            out.append((t_ns, lump.split("lump__")[-1], points))
        in_contact = False
        names, xs = [], []

    for raw in open(path, encoding="utf-8", errors="replace"):
        line = raw.strip()
        if line.startswith("contact {"):
            flush()
            in_contact = True
            continue
        m = re.match(r"sec:\s*(\d+)$", line)
        if m:
            t_ns = int(m.group(1)) * 1_000_000_000
            continue
        m = re.match(r"nsec:\s*(\d+)$", line)
        if m:
            if t_ns is None:
                t_ns = 0
            t_ns += int(m.group(1))
            continue
        if not in_contact:
            continue
        if line.startswith("name:"):
            names.append(line.split('"')[1])
            continue
        for axis in ("x", "y", "z"):
            m = re.match(r"%s:\s*(-?\d+\.?\d*(?:e-?\d+)?)$" % axis, line)
            if m:
                xs.append(float(m.group(1)))
                break
    flush()
    return out


def analyze_run(run, box_seq, box_id, detach_sim, suction_label):
    base = os.path.join(EVID, run)
    tier = "inch20"
    stl = os.path.join(MODELS, "suitcase_loafbrr_%s" % tier,
                       "meshes", "suitcase.stl")
    trial = json.load(open(
        base + "/dumps/box_%02d_ok/trial.json" % box_seq))
    v = (trial.get("extras") or {}).get("verify") or {}
    dwell = (trial.get("extras") or {}).get("release_dwell") or {}
    suction, t_suc = tf_suction_at(
        base + "/dumps/box_%02d_ok/tf_trace.jsonl" % box_seq,
        suction_label)
    box_at_release = v.get("box_pose_at_release")
    half = 0.110  # from the release SDF below; assert-checked
    rel = sdf_collision(base + "/sdf/release.sdf")
    assert rel and abs(rel["size"][2] / 2 - half) < 1e-9, rel
    center_z = box_at_release[2]
    report = {
        "run": run,
        "box_collision": rel,
        "mesh": mesh_stats(stl),
        "release_frame_z": {
            "box_center": round(center_z, 5),
            "collision_top (=center+0.110)": round(center_z + half, 5),
            "suction_contact_plane (tf %s @ %.3f)" % (
                suction_label, t_suc or 0):
                round(suction[2], 5) if suction else None,
        },
        "deltas_at_release": {},
        "s20_contacts": {},
    }
    # Lid plateau in world: bottom-shared ⇒ plateau = center + half −
    # lid_offset(STL-frame: crown−plateau). Use the mesh's own numbers.
    mesh = report["mesh"]
    lid_off = mesh["lid_offset_from_crown"]
    plateau_z = center_z + half - lid_off
    report["release_frame_z"]["lid_plateau (bottom-shared)"] = round(
        plateau_z, 5)
    suction_z = suction[2] if suction else None
    report["deltas_at_release"] = {
        "suction_plane_above_lid_plateau":
            round(suction_z - plateau_z, 5) if suction_z else None,
        "suction_plane_below_collision_top":
            round(suction_z - (center_z + half), 5) if suction_z else None,
        "collision_top_above_lid_plateau": round(lid_off, 5),
    }
    contacts = s20_contacts(base + "/contact_stream.txt", box_id)
    if contacts:
        ts = [t for t, _l, _p in contacts]
        lumps = collections.Counter(l for _t, l, _p in contacts)
        zs = [p[2] for _t, _l, ps in contacts for p in ps if len(p) == 3]
        report["s20_contacts"] = {
            "n": len(contacts),
            "t_span_ms": [round((min(ts) - detach_sim * 1e9) / 1e6, 1),
                          round((max(ts) - detach_sim * 1e9) / 1e6, 1)],
            "lumps": dict(lumps),
            "manifold_z_planes": sorted({round(z, 4) for z in zs}),
            "manifold_z_vs_collision_top": sorted({
                round(z - (center_z + half), 4) for z in zs}),
        }
    report["dwell_end_pose"] = dwell.get("box_pose_at_dwell_end")
    report["verify"] = {k: v.get(k) for k in (
        "err_xy", "err_z", "drift", "fall_m",
        "box_pose_after_drift_wait", "expected_pose_world")}
    return report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    runs = [
        ("gate3_aprime_r5", 0, "pickup_box_0001", 102.439,
         "after_descend"),
        ("gate3_aprime_r6", 0, "pickup_box_0001", 102.138,
         "after_descend"),
    ]
    reports = []
    for run, seq, box_id, detach, label in runs:
        try:
            reports.append(analyze_run(run, seq, box_id, detach, label))
        except (OSError, KeyError, AssertionError) as exc:
            reports.append({"run": run, "error": repr(exc)})
    text = json.dumps(reports, indent=1)
    print(text)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            handle.write(text + "\n")


if __name__ == "__main__":
    main()
