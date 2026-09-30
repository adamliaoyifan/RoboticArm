#!/usr/bin/env python3
"""Evaluate a joined two-box record against an explicitly frozen input manifest."""
import argparse
import json
from pathlib import Path

from luggage_gazebo.integration_acceptance import extract_acceptance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", required=True, type=Path)
    parser.add_argument("--frozen", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    # Reserve output before parsing so malformed/truncated inputs leave evidence.
    with args.output.open("x", encoding="utf-8") as stream:
        try:
            evidence = json.loads(args.evidence.read_text())
            frozen = json.loads(args.frozen.read_text())
            result = extract_acceptance(evidence, frozen)
        except (ValueError, KeyError, TypeError, AttributeError, OSError) as error:
            result = {"status": "not_evaluated", "missing": [str(error)], "failures": [], "sample_count": 0}
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return 0 if result["status"] == "pass" else 2


if __name__ == "__main__":
    raise SystemExit(main())
