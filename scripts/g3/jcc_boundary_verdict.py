#!/usr/bin/env python3
"""Boundary localization verdict for one joint-command capture run.

Usage:
  jcc_boundary_verdict.py RUN_DIR [--controller-state JSONL]
      [--joint-index N] [--pos-tol RAD] [--vel-tol RAD_S]

Prints the verdict (jtc_output | plugin_write | physical_response |
jtc_interface_unseparated | insufficient_evidence) plus the measured L2xL3
coverage and per-boundary statistics as JSON. Per-cycle samples only; no
interpolation between knots (campaign lesson).
"""
from __future__ import annotations

import argparse
import json
import sys

from joint_command_capture.analysis import analyze_run


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir")
    parser.add_argument("--controller-state", default=None,
                        help="controller_state JSONL (JTC desired rows)")
    parser.add_argument("--joint-index", type=int, default=0)
    parser.add_argument("--pos-tol", type=float, default=0.01)
    parser.add_argument("--vel-tol", type=float, default=0.05)
    args = parser.parse_args()
    verdict = analyze_run(
        args.run_dir,
        controller_state_jsonl=args.controller_state,
        joint_index=args.joint_index,
        pos_tol_rad=args.pos_tol,
        vel_tol_rad_s=args.vel_tol)
    print(json.dumps({
        "verdict": verdict.verdict,
        "boundary": verdict.boundary,
        "first_deviation_sim_ns": verdict.first_deviation_sim_ns,
        "details": verdict.details,
    }, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
