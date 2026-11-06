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
    parser.add_argument("--cases", default="",
                        help="comma-separated case ids (G2 targeted "
                             "supplement); empty runs the full matrix")
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
        selected = [c for c in args.cases.split(",") if c.strip()]
        results = v2_races.run_v2_group(
            args.backend, args.hull, manifest, out, args.run_id,
            cases=selected or None)
    _write_probes(out)
    json.dump({"group": "%s-%s-%s" % (args.matrix, args.backend, args.hull),
               "results": results, "domain": args.domain},
              open(out / "GROUP_RESULT.json", "w"), indent=1, default=str)
    return 0


def _write_probes(out):
    """Runtime identity record (G2 §3.1), written at group END so lazily
    imported case modules are covered: hash every campaign-relevant
    module THIS interpreter actually imported, plus every executable the
    chain spawned (recorded by ChainProcess at spawn time). a0_verify
    binds these probes against the freeze manifest — undeclared loaded
    paths and post-freeze byte drift both fail A0."""
    import hashlib
    probes = []
    for name, module in sorted(sys.modules.items()):
        if module is None:
            continue
        if not module.__dict__.get("__name__", "").startswith(
                ("luggage_", "uld_occ", "occ_compat")):
            continue
        origin = getattr(module, "__file__", None)
        if not origin:
            continue
        path = Path(origin)
        if not path.is_file() or path.suffix != ".py":
            continue
        probes.append({
            "kind": "module",
            "module": name,
            "path": str(path.resolve()),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        })
    exe_paths = sorted(out.glob("**/EXEC_PROBES.jsonl"))
    for exe_path in exe_paths:
        for line in exe_path.read_text().splitlines():
            if line.strip():
                probes.append(json.loads(line))
    (out / "MODULE_PROBES.json").write_text(json.dumps(
        {"schema": "module-probes-v1", "probes": probes}, indent=1))


if __name__ == "__main__":
    sys.exit(main())
