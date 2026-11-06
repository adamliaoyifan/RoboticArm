#!/usr/bin/env python3
"""occ_consumer_compat — OCC-GRID-CONSUMER-COMPAT G1 tool T1.

Contract CLI (docs/plans/OCC_GRID_CONSUMER_COMPAT_G1_20261105.md §7):

  inventory  freeze the consumer/matrix/policy/fixture inventory + EXPECTED
  preflight  <=120s (real monotonic) readiness rehearsal of one chain
  selftest   judge self-test on a real healthy rehearsal archive
  freeze     freeze G/C1/T1 SHAs + install manifests + dirty check
  verify     run V1 (10 states x 2 backends x 2 hulls x consumers) and
             V2 (12 deterministic races x 2 backends) against real chains
  judge      A0-A5 from the archive on disk (never summary self-reports,
             never /tmp worktrees or the network)
  replay     re-run one archived case and verify same-case same-category

Exit 0 means THIS subcommand's goal completed; verify/judge exit 0 only
when every required gate is PASS. BLOCKED/NOT_EVALUATED exit non-zero with
the full result written.
"""
import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from occ_compat import cli  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="occ_consumer_compat", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name, help_text, needs in (
            ("inventory", "freeze consumer/matrix/policy/fixture inventory",
             ("manifest", "out")),
            ("preflight", "readiness rehearsal (<=120s real monotonic)",
             ("manifest", "out")),
            ("selftest", "judge self-test on a healthy rehearsal archive",
             ("manifest", "out")),
            ("freeze", "freeze SHAs + install manifests + dirty check",
             ("manifest", "out")),
            ("verify", "run V1/V2 matrices", ("freeze", "matrix", "out")),
            ("judge", "A0-A5 verdict from an archive", ("archive", "out")),
            ("replay", "replay one archived case", (
                "archive", "case", "out"))):
        subparser = sub.add_parser(name, help=help_text)
        for arg in needs:
            subparser.add_argument("--" + arg, required=True)
        if name == "verify":
            subparser.add_argument(
                "--cases", default="",
                help="comma-separated case ids for the targeted G2 "
                     "supplement; empty runs the full matrix")
        if name == "preflight":
            subparser.add_argument("--backend", default="G",
                                   choices=["G", "O"])
        if name == "judge":
            subparser.add_argument(
                "--selftest", default=None,
                help="SELFTEST_RESULT.json from this tree (gate A1)")
            subparser.add_argument(
                "--freeze", default=None,
                help="FREEZE.json the run was produced under (gate A0)")
            subparser.add_argument(
                "--evidence-root", default=None,
                help="campaign evidence root (A5 regression + replay)")
            subparser.add_argument(
                "--a0-verify", default=None,
                help="A0_VERIFY.json from occ_compat.a0_verify (gate A0)")
        subparser.set_defaults(func=getattr(cli, "cmd_" + name))
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
