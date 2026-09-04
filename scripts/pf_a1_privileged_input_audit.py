#!/usr/bin/env python3
"""PF-A1 CLI: scan the workspace and write the privileged-input inventory."""

from __future__ import division

import argparse
import json
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "src", "luggage_perception"))

from luggage_perception.eval.pf_a1_static_audit import (  # noqa: E402
    audit_tree,
    write_report,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root", default=ROOT,
        help="workspace root (default: repository root)")
    parser.add_argument("--out", default="", help="write JSON report here")
    args = parser.parse_args()
    report = audit_tree(os.path.abspath(args.root))
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.out:
        write_report(report, args.out)
    else:
        sys.stdout.write(text + "\n")
    if not report["height_geometry_pass"]:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
