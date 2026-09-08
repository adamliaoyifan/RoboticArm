#!/usr/bin/env python3
"""PF-A2 CLI: validate a Gate 5 bag manifest. Never claims accuracy."""

from __future__ import division

import argparse
import json
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "src", "luggage_perception"))

from luggage_perception.eval.gate5_bag_readiness import (  # noqa: E402
    BAG_GATE5_ACCURACY_NOT_CLAIMED,
    check_manifest,
    dump_report,
    load_manifest,
    mutate_fixture,
    valid_fixture_manifest,
)


def main():
    parser = argparse.ArgumentParser(
        description="Gate 5 rosbag readiness checker (PF-A2).")
    parser.add_argument("--manifest", default="", help="manifest JSON path")
    parser.add_argument(
        "--write-valid-fixture", default="",
        help="write the synthetic valid fixture to this path and exit")
    parser.add_argument(
        "--write-fixture", default="",
        help="kind for mutate_fixture (with --out)")
    parser.add_argument("--out", default="", help="write the report JSON")
    parser.add_argument(
        "--claim-accuracy", action="store_true",
        help="attempt to claim Gate 5 accuracy (always rejected)")
    args = parser.parse_args()

    if args.write_valid_fixture:
        path = args.write_valid_fixture
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "w") as handle:
            json.dump(valid_fixture_manifest(), handle, indent=2, sort_keys=True)
            handle.write("\n")
        return 0

    if args.write_fixture:
        if not args.out:
            sys.stderr.write("--write-fixture needs --out for the manifest\n")
            return 2
        directory = os.path.dirname(args.out)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(args.out, "w") as handle:
            json.dump(
                mutate_fixture(args.write_fixture), handle,
                indent=2, sort_keys=True)
            handle.write("\n")
        return 0

    if not args.manifest:
        sys.stderr.write("need --manifest or --write-valid-fixture\n")
        return 2
    manifest = load_manifest(args.manifest)
    report = check_manifest(manifest, claim_accuracy=args.claim_accuracy)
    if args.out:
        dump_report(report, args.out)
    else:
        sys.stdout.write(json.dumps(report, indent=2, sort_keys=True) + "\n")
    if args.claim_accuracy or BAG_GATE5_ACCURACY_NOT_CLAIMED in report["reasons"]:
        # Accuracy claim is a hard failure even if other checks would pass.
        return 1
    return 0 if report["ready"] else 1


if __name__ == "__main__":
    sys.exit(main())
