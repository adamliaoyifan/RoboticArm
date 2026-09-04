#!/usr/bin/env python3
"""Validate rebuilt atlases against persisted E12 floor-contact evidence."""
import argparse
import os
import sys

import yaml

SCRIPTS = os.path.dirname(os.path.abspath(__file__))
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from reachability_atlas import ReachabilityAtlas, STATUS_NAMES  # noqa: E402


def validate(atlas_prefixes, evidence_path):
    with open(evidence_path, "r") as stream:
        evidence = yaml.safe_load(stream) or {}
    rows = [row for row in evidence.get("results", []) if row.get("ik_solved")]
    results = []
    for prefix in atlas_prefixes:
        atlas = ReachabilityAtlas.load(prefix + ".npz", prefix + ".yaml")
        queries = []
        for row in rows:
            result = atlas.query(
                row["container_x"], row["container_y"], row["contact_z"], 0.0)
            queries.append({
                "box": row["box"],
                "ix": row["ix"],
                "contact_z": row["contact_z"],
                "expected_state_valid": row["state_valid"],
                "atlas_status": STATUS_NAMES[result.status],
                "atlas_indices": list(result.indices) if result.indices else None,
            })
        results.append({
            "atlas": prefix,
            "grid": atlas.meta["grid"],
            "container": atlas.meta["container"],
            "queries": queries,
            "summary": {
                "query_count": len(queries),
                "reachable_or_marginal": sum(
                    q["atlas_status"] in ("reachable", "marginal")
                    for q in queries),
                "unexpected_rejects": sum(
                    q["expected_state_valid"]
                    and q["atlas_status"] in ("unreachable", "unknown")
                    for q in queries),
            },
        })
    return {
        "schema_version": 1,
        "evidence_path": evidence_path,
        "atlases": results,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("atlas_prefix", nargs="+")
    args = parser.parse_args()
    result = validate(args.atlas_prefix, args.evidence)
    with open(args.output, "w") as stream:
        yaml.safe_dump(result, stream, default_flow_style=False, sort_keys=False)
    bad = sum(
        atlas["summary"]["unexpected_rejects"]
        for atlas in result["atlases"])
    print("validated %d atlases; unexpected_rejects=%d" % (
        len(result["atlases"]), bad))
    raise SystemExit(1 if bad else 0)


if __name__ == "__main__":
    main()
