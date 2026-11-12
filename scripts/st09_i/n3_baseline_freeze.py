#!/usr/bin/env python3
"""Freeze the n=3 baseline without touching earlier evidence directories."""

from __future__ import division

import hashlib
import json
import os
import shutil
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
HERE = os.path.dirname(os.path.abspath(__file__))
for _path in (
        HERE,
        os.path.join(ROOT, "src", "luggage_planning"),
        os.path.join(ROOT, "src", "luggage_description"),
        os.path.join(ROOT, "st09_contracts"),
):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import placement_robustness as scan  # noqa: E402

OUT = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_n3_baseline_freeze_20261005")
PRIOR = (
    "g3_placement_robustness_20261005",
    "g3_corridor_ablation_20261005",
)


def _sha(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git(args):
    return subprocess.check_output(
        ["git", *args], cwd=ROOT, text=True).strip()


def main():
    parent = os.path.dirname(OUT)
    for name in PRIOR:
        prior = os.path.join(parent, name)
        if not os.path.isdir(prior):
            raise SystemExit("refusing to freeze; missing prior evidence %s" % prior)
    if os.path.exists(OUT):
        raise SystemExit("freeze directory already exists: %s" % OUT)
    os.makedirs(OUT)
    scan_src = os.path.join(scan.OUT, "scan_list.json")
    shutil.copy2(scan_src, os.path.join(OUT, "scan_list.json"))
    pins = scan.assert_safety_pins()
    points = json.load(open(scan_src, encoding="utf-8"))["points"]
    sizes = {}
    for point in points:
        sizes.setdefault(point["box"], point["size"])
    grasps = {
        name: {
            "position": list(scan._grasp(size)[0]),  # noqa: SLF001
            "quaternion_xyzw": list(scan._grasp(size)[1]),  # noqa: SLF001
        }
        for name, size in sizes.items()
    }
    identity_files = [
        "src/luggage_description/config/scene_tf_deploy.yaml",
        "src/elfin_description/urdf/S20.urdf.xacro",
        "src/luggage_planning/config/atlas_runtime_deploy_sim.yaml",
        "st09_contracts/resolved_planning_contract.json",
        "src/luggage_planning/luggage_planning/routing/corridor_lattice.py",
        "src/luggage_planning/luggage_planning/routing/lattice.py",
        "src/luggage_planning/luggage_planning/routing/producer.py",
    ]
    atlas_dir = os.path.join(
        ROOT, "src/luggage_planning/data/reachability_atlas_deploy_sim")
    atlas_files = sorted(
        name for name in os.listdir(atlas_dir)
        if name.endswith((".npz", ".yaml", ".xml")))
    manifest = {
        "commit": _git(["rev-parse", "HEAD"]),
        "branch": _git(["rev-parse", "--abbrev-ref", "HEAD"]),
        "dirty": _git(["status", "--porcelain"]).splitlines(),
        "scene_sha256": _sha(os.path.join(
            ROOT, "src/luggage_description/config/scene_tf_deploy.yaml")),
        "urdf_xacro_sha256": _sha(os.path.join(
            ROOT, "src/elfin_description/urdf/S20.urdf.xacro")),
        "atlas_config_sha256": _sha(os.path.join(
            ROOT, "src/luggage_planning/config/atlas_runtime_deploy_sim.yaml")),
        "atlas_dir": "src/luggage_planning/data/reachability_atlas_deploy_sim",
        "atlas_files": {
            name: _sha(os.path.join(atlas_dir, name)) for name in atlas_files},
        "source_sha256": {
            path: _sha(os.path.join(ROOT, path)) for path in identity_files},
        "safety_pins": pins,
        "boxes_m": sizes,
        "start_joints": list(scan.CYCLE_START_Q),
        "grasp": grasps,
        "scan_list_sha256": _sha(scan_src),
        "scan_list_copy": "scan_list.json",
        "n_points": len(points),
        "prior_evidence_untouched": list(PRIOR),
        "positive_control": {
            "slot": 656,
            "size_m": [0.50, 0.36, 0.25],
            "note": "Historical detected size. Not in the 48-point nominal list.",
        },
    }
    with open(os.path.join(OUT, "baseline_freeze.json"), "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print(json.dumps({
        "out": OUT,
        "commit": manifest["commit"],
        "dirty": len(manifest["dirty"]),
        "points": len(points),
        "boxes": sizes,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
