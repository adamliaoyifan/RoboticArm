#!/usr/bin/env python3
"""Replay the fixed sequences and require byte-identical step records."""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def load_npz(path):
    if not path.exists():
        return None
    with np.load(path, allow_pickle=False) as data:
        return {key: data[key] for key in data.files}


def mismatch(expected, actual):
    if expected.keys() != actual.keys():
        return "keys %s vs %s" % (sorted(expected), sorted(actual))
    for key in sorted(expected):
        left, right = expected[key], actual[key]
        if left.shape != right.shape or left.dtype != right.dtype:
            return "%s shape/dtype %s %s vs %s %s" % (
                key, left.shape, left.dtype, right.shape, right.dtype)
        if left.tobytes() != right.tobytes():
            return "%s bytes differ" % key
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--expected", required=True)
    parser.add_argument("--yaw", required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(ROOT))
    sys.path.insert(0, str(ROOT / "src/luggage_perception/test"))
    sys.path.insert(0, str(ROOT / "src/luggage_perception"))
    sys.path.insert(0, str(ROOT / "src/luggage_description"))
    from occ_structure_sequences import (
        sequence_m1, sequence_m2, sequence_m3, sequence_m4_grid,
        sequence_m4_m7_depth, sequence_m5, sequence_m6, sequence_m8,
        sequence_store,
    )
    groups = {
        "M1": sequence_m1(),
        "M2": sequence_m2(),
        "M3": sequence_m3(),
    }
    for name, steps in sequence_m4_grid().items():
        groups["M4-" + name] = steps
    for name, steps in sequence_m4_m7_depth().items():
        groups["M7-" + name] = steps
    for name, steps in sequence_m5().items():
        groups["M5-" + name] = steps
    for name, steps in sequence_m6().items():
        groups["M6-" + name] = steps
    groups["M8"] = sequence_m8(args.yaw)[0]
    groups.update(sequence_store())
    expected_root = Path(args.expected)
    failures = []
    for name, steps in groups.items():
        folder = expected_root / name
        count = int((folder / "count.txt").read_text())
        if count != len(steps):
            failures.append("%s count %s != %s" % (name, count, len(steps)))
            continue
        for index, step in enumerate(steps):
            meta_path = folder / ("step_%04d.json" % index)
            saved = meta_path.read_text()
            actual = json.dumps(step.meta, sort_keys=True, separators=(",", ":"))
            if saved != actual:
                failures.append("%s step %d meta differs" % (name, index))
                (folder / ("step_%04d.actual.json" % index)).write_text(actual)
                break
            saved_npz = load_npz(folder / ("step_%04d.npz" % index))
            if saved_npz is None:
                if step.payload:
                    failures.append("%s step %d missing npz" % (name, index))
                continue
            reason = mismatch(saved_npz, step.payload)
            if reason:
                failures.append("%s step %d %s" % (name, index, reason))
                break
    if failures:
        print("\n".join(failures))
        raise SystemExit(1)
    print("compare ok", len(groups))


if __name__ == "__main__":
    main()
