#!/usr/bin/env python3
"""Fail-close validator for physical closed-loop phase evidence bundles."""
from __future__ import division

import argparse
import json
import os

import yaml


REQUIRED_MANIFEST_FIELDS = (
    "schema_version",
    "phase",
    "run_id",
    "status",
    "hypothesis",
    "git_sha",
    "dirty_diff_sha256",
    "docker_image",
    "config_hashes",
    "command",
    "seeds",
    "started_at",
    "required_evidence",
    "hard_gates",
)

# Schema 1 bundles record only the outcome. Schema 2 additionally requires the
# *process*: which alternatives were tried and the numbers that ruled them out.
# Without it the reasoning behind a tuned constant lives only in someone's
# terminal scrollback and has to be re-derived from scratch.
PROCESS_SCHEMA_VERSION = 2
REQUIRED_PROCESS_ARTIFACTS = (
    "process.md",     # chronological hypothesis -> action -> observation -> conclusion
    "ablations.yaml",  # machine-readable sweep / ablation / isolation results
)


def validate_bundle(bundle_dir, allow_running=False):
    manifest_path = os.path.join(bundle_dir, "manifest.yaml")
    errors = []
    if not os.path.isfile(manifest_path):
        return {
            "passed": False,
            "errors": ["manifest.yaml missing"],
            "bundle_dir": bundle_dir,
        }
    with open(manifest_path, "r") as stream:
        manifest = yaml.safe_load(stream) or {}
    for field in REQUIRED_MANIFEST_FIELDS:
        if field not in manifest:
            errors.append("manifest field missing: %s" % field)
    status = str(manifest.get("status", ""))
    if status not in ("running", "passed", "failed"):
        errors.append("invalid status: %s" % status)
    if status == "running" and not allow_running:
        errors.append("phase is still running")
    for relative_path in manifest.get("required_evidence", []):
        if not os.path.isfile(os.path.join(bundle_dir, relative_path)):
            errors.append("required evidence missing: %s" % relative_path)
    gates = manifest.get("hard_gates", {})
    if status == "passed":
        if not manifest.get("ended_at"):
            errors.append("passed phase missing ended_at")
        if not gates:
            errors.append("passed phase has no hard gates")
        for name, value in gates.items():
            if value is not True:
                errors.append("hard gate not true: %s=%r" % (name, value))
        for required in ("metrics.yaml", "result.yaml"):
            if not os.path.isfile(os.path.join(bundle_dir, required)):
                errors.append("passed phase missing %s" % required)
    if status == "failed":
        if not os.path.isfile(os.path.join(bundle_dir, "failure.yaml")):
            errors.append("failed phase missing failure.yaml")
    try:
        schema_version = int(manifest.get("schema_version", 1))
    except (TypeError, ValueError):
        schema_version = 1
    if schema_version >= PROCESS_SCHEMA_VERSION and status in ("passed", "failed"):
        for artifact in REQUIRED_PROCESS_ARTIFACTS:
            path = os.path.join(bundle_dir, artifact)
            if not os.path.isfile(path):
                errors.append("missing process artifact: %s" % artifact)
            elif os.path.getsize(path) == 0:
                errors.append("empty process artifact: %s" % artifact)
    return {
        "passed": not errors,
        "errors": errors,
        "bundle_dir": bundle_dir,
        "phase": manifest.get("phase"),
        "run_id": manifest.get("run_id"),
        "status": status,
        "hard_gates": gates,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bundle_dir")
    parser.add_argument("--allow-running", action="store_true")
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    result = validate_bundle(args.bundle_dir, args.allow_running)
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w") as stream:
            stream.write(text + "\n")
    print(text)
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
