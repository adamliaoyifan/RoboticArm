#!/usr/bin/env python3
"""Emit one runtime-interface proof bound to the current tree.

Recomputes the interface digest (id..checks, matching
scripts/check_frame_contracts.py) and the sha256 of every row source plus
the shared contract modules, so a proof can only be written against the
tree it was measured on. Output is exclusive-create: refreshing means a new
evidence directory, never an in-place edit.
"""
from __future__ import division

import argparse
import hashlib
import json
from pathlib import Path

CONTRACT_SOURCES = (
    "src/luggage_description/luggage_description/frame_contracts.py",
    "src/luggage_description/luggage_description/frame_transforms.py",
)


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def interface_digest(row):
    fields = ("id", "producer", "consumer", "transport", "scope", "target",
              "sources", "checks")
    data = json.dumps({key: row[key] for key in fields}, sort_keys=True,
                      separators=(",", ":"))
    return hashlib.sha256(data.encode("utf-8")).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ws", required=True, type=Path)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--command", required=True)
    parser.add_argument("--sample-count", required=True, type=int)
    parser.add_argument("--observation", action="append", required=True)
    parser.add_argument("--kind", default="runtime_boundary",
                        choices=("runtime_boundary", "isolation_check"))
    parser.add_argument("--prior", default="")
    parser.add_argument("--test-evidence", default="")
    parser.add_argument("--limits", default="")
    parser.add_argument("--boundary-trace", default="")
    args = parser.parse_args()

    inventory = json.loads(
        (args.ws / "docs/architecture/frame_interfaces.json").read_text())
    row = next((entry for entry in inventory["interfaces"]
                if entry.get("id") == args.interface), None)
    if row is None:
        raise SystemExit("unknown interface %s" % args.interface)
    if args.sample_count <= 0:
        raise SystemExit("sample_count must be positive")
    sources = dict.fromkeys([*row["sources"], *CONTRACT_SOURCES])
    source_sha256 = {path: sha256_file(args.ws / path) for path in sources}
    proof = {
        "interface_id": row["id"],
        "kind": args.kind,
        "status": "pass",
        "contract_version": 2,
        "command": args.command,
        "observations": list(args.observation),
        "sample_count": args.sample_count,
        "interface_contract_sha256": interface_digest(row),
        "source_sha256": source_sha256,
    }
    if args.prior:
        proof["prior_evidence"] = args.prior
    if args.test_evidence:
        proof["test_evidence"] = args.test_evidence
    if args.limits:
        proof["limits"] = args.limits
    if args.boundary_trace:
        proof["boundary_trace"] = args.boundary_trace
    args.out_dir.mkdir(parents=True, exist_ok=True)
    target = args.out_dir / ("runtime_%s.json" % row["id"])
    with open(target, "x", encoding="utf-8") as stream:
        json.dump(proof, stream, indent=1, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(target)


if __name__ == "__main__":
    main()
