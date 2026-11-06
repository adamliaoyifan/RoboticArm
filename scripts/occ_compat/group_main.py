"""group_main — one matrix group in its own process (formal entry).

The formal run drives every group as a SUBPROCESS: a fresh interpreter
means a fresh rclpy lifecycle (repeated init/shutdown in one interpreter
degrades into InvalidHandle teardowns, which once leaked whole chains
onto the shared default domain and contaminated every later group), a
unique ROS domain per group (any leak is fenced), and PDEATHSIG cleans
the chain if this process dies. The child writes GROUP_RESULT.json; its
exit code reflects driver completion, not the verdict.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", required=True, choices=["V1", "V2"])
    parser.add_argument("--backend", required=True, choices=["G", "O"])
    parser.add_argument("--hull", required=True, choices=["cuboid", "chamfer"])
    parser.add_argument("--freeze", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--domain", required=True, type=int)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args(argv)

    from . import chains, runner
    chains.ChainProcess.domain = str(args.domain)
    # The client node lives in THIS interpreter: rclpy reads the domain
    # from the environment at participant creation, so the child must
    # export the SAME domain it assigns to the spawned chain processes
    # (otherwise the chain fences itself away from its own client).
    import os
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    freeze = json.loads(Path(args.freeze).read_text())
    manifest = freeze["manifest"]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    if args.matrix == "V1":
        results = runner.run_v1_group(
            args.backend, args.hull, manifest, out, args.run_id)
    else:
        from . import v2_races
        results = v2_races.run_v2_group(
            args.backend, args.hull, manifest, out, args.run_id)
    json.dump({"group": "%s-%s-%s" % (args.matrix, args.backend, args.hull),
               "results": results, "domain": args.domain},
              open(out / "GROUP_RESULT.json", "w"), indent=1, default=str)
    return 0


if __name__ == "__main__":
    sys.exit(main())
