#!/usr/bin/env python3
"""Classify every record in an ST09_PORTAL_CAPTURE_DIR (see portal_goal_classifier)."""
import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "luggage_planning"))
from luggage_planning.place.portal_goal_classifier import (  # noqa: E402
    classify_portal_record, urdf_joint_limits)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("capture_dir")
    parser.add_argument("--urdf", required=True)
    args = parser.parse_args()
    limits = urdf_joint_limits(open(args.urdf).read())
    out = []
    for path in sorted(glob.glob(os.path.join(args.capture_dir, "portal_[0-9][0-9].json"))):
        record = json.load(open(path))
        verdict = classify_portal_record(record, limits)
        verdict["record"] = os.path.basename(path)
        out.append(verdict)
    target = os.path.join(args.capture_dir, "portal_classification.json")
    json.dump({"urdf": args.urdf, "limits": limits, "verdicts": out},
              open(target, "w"), indent=1)
    for item in out:
        print(item["record"], item["category"])
    print("records=%d" % len(out))


if __name__ == "__main__":
    main()
