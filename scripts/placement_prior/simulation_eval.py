#!/usr/bin/env python3
"""Evaluate preserved prior cycle artifacts without changing original evidence.

This reports individual unscored motion/OCC coverage, not the six-run F3 gate.
"""
import argparse
import json
from pathlib import Path
from luggage_gazebo.prior_functional_evidence import summarize_prior_cycle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    reports = [summarize_prior_cycle(json.loads(line))
               for line in args.trials.read_text().splitlines() if line.strip()]
    result = {"source": str(args.trials), "cases": reports,
              "f3_campaign": "not_evaluated", "performance": "not_evaluated"}
    with args.output.open("x") as handle:
        json.dump(result, handle, indent=2)
        handle.write("\n")
    print(json.dumps({"cases": len(reports),
                      "functional_cycles_passed": sum(r["functional_cycle_passed"] for r in reports),
                      "f3_campaign": "not_evaluated"}))
    return 0 if reports and all(r["functional_cycle_passed"] for r in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())
