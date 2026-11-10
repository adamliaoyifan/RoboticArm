#!/usr/bin/env python3
"""Freeze baseline A step records. Refuses to run if product files differ from A."""

import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
BASELINE = "a34472d0682f35172d1fc80a0cfb654a0b8ddc16"
YAW_DEFAULT = (
    "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
    "occ_structure_review_20261102/yaw_scalar_vectorized.json")
PRODUCT_PREFIXES = (
    "src/luggage_perception/luggage_perception/",
    "src/luggage_perception/scripts/",
)


def git(*args):
    return subprocess.check_output(["git", "-C", str(ROOT), *args], text=True).strip()


def product_hashes():
    files = git("ls-tree", "-r", "--name-only", BASELINE).splitlines()
    selected = [path for path in files if path.startswith(PRODUCT_PREFIXES)]
    dirty = git("diff", "--name-only", BASELINE, "--", *PRODUCT_PREFIXES)
    if dirty:
        raise SystemExit("product files differ from %s:\n%s" % (BASELINE, dirty))
    hashes = {}
    for path in selected:
        hashes[path] = git("rev-parse", "%s:%s" % (BASELINE, path))
    return hashes


def save_step(directory, index, step):
    directory.mkdir(parents=True, exist_ok=True)
    meta_path = directory / ("step_%04d.json" % index)
    text = json.dumps(step.meta, sort_keys=True, separators=(",", ":"))
    meta_path.write_text(text)
    if step.payload:
        np.savez(directory / ("step_%04d.npz" % index), **step.payload)


def save_group(root, name, steps):
    folder = root / name
    for index, step in enumerate(steps):
        save_step(folder, index, step)
    (folder / "count.txt").write_text(str(len(steps)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--yaw", default=YAW_DEFAULT)
    args = parser.parse_args()
    sys.path.insert(0, str(ROOT))
    sys.path.insert(0, str(ROOT / "src/luggage_perception/test"))
    sys.path.insert(0, str(ROOT / "src/luggage_perception"))
    sys.path.insert(0, str(ROOT / "src/luggage_description"))
    from occ_structure_sequences import (
        assert_m1_threshold_coverage,
        sequence_m1,
        sequence_m2,
        sequence_m3,
        sequence_m4_grid,
        sequence_m4_m7_depth,
        sequence_m5,
        sequence_m6,
        sequence_m8,
        sequence_store,
    )

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    hashes = product_hashes()
    m1 = sequence_m1()
    coverage = assert_m1_threshold_coverage(m1)
    if not coverage["ok"]:
        (out / "m1_coverage.json").write_text(json.dumps(coverage, indent=2))
        raise SystemExit("M1 threshold coverage failed: %s" % coverage)
    save_group(out, "M1", m1)
    save_group(out, "M2", sequence_m2())
    save_group(out, "M3", sequence_m3())
    for name, steps in sequence_m4_grid().items():
        save_group(out, "M4-" + name, steps)
    for name, steps in sequence_m4_m7_depth().items():
        save_group(out, "M7-" + name, steps)
    for name, steps in sequence_m5().items():
        save_group(out, "M5-" + name, steps)
    for name, steps in sequence_m6().items():
        save_group(out, "M6-" + name, steps)
    m8_steps, saved = sequence_m8(args.yaw)
    save_group(out, "M8", m8_steps)
    import binascii
    saved_vec = binascii.unhexlify(saved["vectorized"]["grid"]["hex"])
    saved_sca = binascii.unhexlify(saved["scalar"]["grid"]["hex"])
    if m8_steps[0].payload["grid"].tobytes() != saved_vec:
        raise SystemExit("M8 vectorized grid does not match the frozen yaw file")
    if m8_steps[1].payload["grid"].tobytes() != saved_sca:
        raise SystemExit("M8 scalar reference does not match the frozen yaw file")
    differ = [int(i) for i in saved["different_grid_indices"]]
    vec = m8_steps[0].payload["grid"]
    sca = m8_steps[1].payload["grid"]
    found = [int(i) for i in range(len(vec)) if int(vec[i]) != int(sca[i])]
    if found != differ:
        raise SystemExit("M8 yaw diff positions %s != %s" % (found, differ))
    for name, steps in sequence_store().items():
        save_group(out, name, steps)
    (out / "m1_coverage.json").write_text(json.dumps(coverage, sort_keys=True, indent=2))
    manifest = {
        "baseline": BASELINE,
        "head": git("rev-parse", "HEAD"),
        "python": platform.python_version(),
        "numpy": np.__version__,
        "platform": platform.platform(),
        "yaw_sha256": hashlib.sha256(Path(args.yaw).read_bytes()).hexdigest(),
        "product_blob_hashes": hashes,
        "importer": os.path.realpath(sys.modules["luggage_perception.cargo_volume_mapper"].__file__),
    }
    (out / "MANIFEST.json").write_text(json.dumps(manifest, indent=2, sort_keys=True))
    command = "PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:src/luggage_perception/test:src/luggage_perception:src/luggage_description python3 scripts/occ_structure/freeze_baseline.py --out %s --yaw %s\n" % (args.out, args.yaw)
    (out / "COMMANDS.sh").write_text(command)
    print("froze", out)


if __name__ == "__main__":
    main()
