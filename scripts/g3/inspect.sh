#!/usr/bin/env bash
# usage: inspect.sh <run-directory>
# Compute the G3 gate observations from a run directory's artifacts.
# Read-only: every number printed comes from files on disk.
set -euo pipefail
RUN="${1:?run directory}"
python3 - "$RUN" <<'PY'
import glob
import json
import os
import sys

run = sys.argv[1]


def load(path):
    with open(os.path.join(run, path)) as handle:
        return json.load(handle)


def maybe(path):
    try:
        return load(path)
    except (OSError, ValueError):
        return None


report = {"run": run}

# G3-0 identity/readiness -----------------------------------------------
manifest = maybe("manifest.json")
readiness = maybe("readiness.json")
if manifest:
    report["identity"] = {
        "head": manifest.get("head"),
        "base_revision": manifest.get("base_revision"),
        "dirty_lines": manifest.get("dirty_lines"),
        "driver": manifest.get("driver"),
    }
if readiness:
    report["g3_0"] = {
        "clock_publishers": readiness.get("clock_publishers"),
        "camera_samples": readiness.get("camera_samples"),
        "camera_rate_hz": readiness.get("camera_rate_hz"),
        "camera_stamp_increasing": readiness.get("camera_stamp_increasing"),
        "inference_samples": readiness.get("inference_samples"),
        "inference_rate_hz": readiness.get("inference_rate_hz"),
        "inference_stamp_increasing": readiness.get("inference_stamp_increasing"),
        "pass": (readiness.get("clock_publishers") == 1
                 and (readiness.get("camera_samples") or 0) >= 10
                 and readiness.get("camera_stamp_increasing")
                 and (readiness.get("inference_samples") or 0) >= 10
                 and readiness.get("inference_stamp_increasing")),
    }

# driver outcome ----------------------------------------------------------
summary = maybe("summary.json")
suite = maybe("suite.json")
ledger_lines = []
ledger_path = os.path.join(run, "ledger.jsonl")
if os.path.exists(ledger_path):
    with open(ledger_path) as handle:
        ledger_lines = [json.loads(line) for line in handle if line.strip()]
trial = None
for record in ledger_lines:
    if record.get("event") in ("trial", "box_result", "case_result"):
        trial = record
boxes = [r for r in ledger_lines if r.get("event") == "box_result"]
report["driver"] = {
    "summary": summary,
    "n_ledger": len(ledger_lines),
    "boxes": boxes,
}

# capture health ---------------------------------------------------------
def dir_counts(name):
    return sorted(
        (os.path.basename(p), len(os.listdir(p)))
        for p in glob.glob(os.path.join(run, name)))


report["captures"] = {
    "portal": dir_counts("portal_capture"),
    "select": dir_counts("select_capture"),
    "cert": dir_counts("cert_capture"),
    "boundary_dumps": dir_counts("dumps/boundaries"),
}

# perception / map handoff evidence --------------------------------------
dumps = sorted(glob.glob(os.path.join(run, "dumps", "box_*")))
per_run = []
for dump in dumps:
    entry = {"dump": os.path.basename(dump)}
    pb = maybe(os.path.relpath(os.path.join(dump, "perception_boundary.json"), run))
    if pb:
        entry["n_detection_frames"] = len(pb.get("detection_frames") or [])
        entry["classification"] = (pb.get("classification") or {}).get("outcome")
    surf = maybe(os.path.relpath(os.path.join(dump, "surface_2d.json"), run))
    if surf:
        heights = surf.get("height") or []
        occupied = sum(1 for row in heights for cell in row
                       if isinstance(cell, (int, float)) and cell > 0)
        entry["occupied_columns"] = occupied
        entry["max_height"] = max(
            (cell for row in heights for cell in row
             if isinstance(cell, (int, float))), default=None)
    commit = maybe(os.path.relpath(os.path.join(dump, "commit.json"), run))
    if commit:
        entry["commit"] = {k: commit.get(k) for k in
                           ("success", "revision", "map_revision", "geometry_hash")
                           if k in commit}
    per_run.append(entry)
report["boxes_evidence"] = per_run

with open(os.path.join(run, "gate_observations.json"), "w") as handle:
    json.dump(report, handle, indent=1)
    handle.write("\n")
print(json.dumps(report, indent=1))
PY
