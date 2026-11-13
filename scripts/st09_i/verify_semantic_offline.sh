#!/usr/bin/env bash
# [ST09-I S5.7-6] Offline isolation proof for the semantic assets.
#
# The FINAL positive case (user lock 2026-10-03):
#   PYTHONNOUSERSITE=1 + fresh HOME + dead network + source/vendor
#   hidden -> backend construction succeeds from ALLOWED prefixes
#   only. If this still raised ImportError it would prove fail-closed,
#   not that the candidate can run.
#
# Proves, in ONE run:
#   1. the user site never satisfies an import (disabled + fresh HOME):
#      torch/ultralytics resolve from the candidate-owned frozen
#      prefix, luggage_perception from the install overlay, and any
#      remaining import from the system distribution is allowed;
#   2. versions + RECORD-based tree digests match the acceptance
#      manifest's semantic_assets.python_deps;
#   3. with the source-tree vendor and ~/.cache/clip hidden, nothing
#      reads or writes them;
#   4. dead proxies: zero install/download attempts (any pip/ultralytics
#      fetch would fail fast; require_backend turns construction
#      failure into a named error, never a silent stub);
#   5. the runtime-reported load paths sit under allowed prefixes and
#      the weights/vendor sha256s match the manifest;
#   6. torch CUDA initializes from the prefix's own cuda runtime.
#
# Usage: scripts/st09_i/verify_semantic_offline.sh --out DIR [--ws WS]
set -uo pipefail

OUT=""
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    --ws) WS="$2"; shift 2 ;;
    *) echo "unknown arg $1" >&2; exit 2 ;;
  esac
done
[[ -n "$OUT" ]] || { echo "--out required (evidence dir)" >&2; exit 2; }
mkdir -p "$OUT"

OVERLAY="$WS/install-st09"
SHARE="$OVERLAY/luggage_perception/share/luggage_perception"
VENDOR_SRC="$WS/src/luggage_perception/vendor"
PYDEPS="$WS/pydeps-frozen"

[[ -f "$PYDEPS/DEPLOYED.json" ]] || {
  echo "frozen python deps prefix absent at $PYDEPS; run scripts/st09_i/deploy_python_deps.sh" >&2
  exit 2
}

# -- hide the mutable sources for the whole proof -------------------------
if [[ -d "$VENDOR_SRC/clip_pkg" ]]; then
  mv "$VENDOR_SRC/clip_pkg" "$VENDOR_SRC/clip_pkg.hidden"
  HIDDEN=1
else
  HIDDEN=0
fi
CACHE_DIR="$HOME/.cache/clip"
CACHE_HIDDEN=0
if [[ -e "$CACHE_DIR" ]]; then
  mv "$CACHE_DIR" "$CACHE_DIR.hidden-proof"
  CACHE_HIDDEN=1
fi
restore() {
  if [[ "$HIDDEN" -eq 1 ]]; then
    mv "$VENDOR_SRC/clip_pkg.hidden" "$VENDOR_SRC/clip_pkg"
  fi
  if [[ "$CACHE_HIDDEN" -eq 1 ]]; then
    mv "$CACHE_DIR.hidden-proof" "$CACHE_DIR"
  fi
}
trap restore EXIT

NVIDIA_LIBS=$(find "$PYDEPS/nvidia" -type d -name lib 2>/dev/null | tr '\n' ':')
FRESH_HOME="$(mktemp -d /tmp/sem_offline_home.XXXXXX)"

# -- the final positive case: user site OFF, fresh HOME, dead network -----
set +u
source /opt/ros/humble/setup.bash
source "$OVERLAY/setup.bash"
set -u
export HOME="$FRESH_HOME"
export PYTHONNOUSERSITE=1
export PYTHONPATH="$PYDEPS${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$NVIDIA_LIBS${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export HTTP_PROXY="http://127.0.0.1:9" HTTPS_PROXY="http://127.0.0.1:9"
export ALL_PROXY="http://127.0.0.1:9" all_proxy="http://127.0.0.1:9"
export NO_PROXY="" no_proxy=""
export YOLO_AUTOUPDATE="false"
unset LUGGAGE_CLIP_VENDOR_DIR || true
export PYTHONPATH="${PYTHONPATH//"$WS\/src"/}"

python3 - "$OUT" "$OVERLAY" "$SHARE" "$PYDEPS" <<'PYEOF'
import importlib
import importlib.metadata as md
import json
import os
import sys

out_dir, overlay, share, pydeps = sys.argv[1:5]
record = {"source_vendor_hidden": True, "dead_proxy": True,
          "pythonnousersite": True, "fresh_home": True}
ALLOWED = (overlay, pydeps, "/usr", "/opt/ros")

def _where(name):
    module = importlib.import_module(name)
    return str(getattr(module, "__file__", "")) or \
        str(os.path.dirname(module.__path__[0]))

# Provenance: the package from the overlay; the backend stack from the
# frozen prefix; anything else only from the system distribution.
record["module_path"] = _where("luggage_perception.semantic_segmenter")
assert record["module_path"].startswith(overlay), record["module_path"]
frozen = {}
for name in ("torch", "ultralytics", "sympy"):
    path = _where(name)
    frozen[name] = {"path": path,
                    "version": importlib.metadata.version(name)}
    assert path.startswith(pydeps), (name, path)
record["frozen_prefix_modules"] = frozen
record["user_site_disabled"] = __import__("site").ENABLE_USER_SITE is False

from luggage_perception.semantic_segmenter import build_segmenter

weights = os.path.join(share, "models", "yolov8s-world.pt")
# [ST09-I S5.7-6] construction on cuda — the formal sim runs
# device:=cuda; a prefix that imports but cannot drive the GPU is not
# a positive proof (torch.cuda availability is asserted below).
segmenter = build_segmenter({
    "backend": "bbox_fill",
    "prompts": ["suitcase"],
    "model_name": weights,
    "device": "cuda",
    "require_backend": "bbox_fill",
})
stats = getattr(segmenter, "_last_stats", {})
record["backend"] = stats.get("backend", "")
record["model_path"] = stats.get("model_path", "")
record["clip_vendor"] = stats.get("clip_vendor", "")
assert record["backend"].startswith("bbox_fill:"), record
for key in ("model_path", "clip_vendor"):
    assert record[key], "stats missing %s" % key
    assert record[key].startswith(overlay), (key, record[key])
record["cache_clip_absent"] = not os.path.exists(
    os.path.join(os.path.expanduser("~"), ".cache", "clip"))
assert record["cache_clip_absent"]

# [ST09-I S5.7-6] one minimal INFERENCE — the model must run, not
# merely load (segment(rgb_image) -> (label_map uint8 HxW,
# detections); semantic_segmenter.py BboxFillSegmenter).
import numpy as _np

label_map, detections = segmenter.segment(
    _np.zeros((96, 96, 3), dtype=_np.uint8))
assert label_map is not None and list(label_map.shape) == [96, 96], (
    getattr(label_map, "shape", None))
stats = getattr(segmenter, "_last_stats", {})
record["inference_probe"] = {
    "label_map_shape": list(label_map.shape),
    "detections": len(detections or []),
    "backend": stats.get("backend", ""),
}
assert record["inference_probe"]["backend"].startswith("bbox_fill:")

# CUDA must initialize from the prefix's own runtime (the formal sim
# runs device:=cuda; a prefix that imports but cannot drive the GPU is
# not a positive proof).
import torch

record["torch_cuda_available"] = bool(torch.cuda.is_available())
assert record["torch_cuda_available"], "torch.cuda unavailable from prefix"

import hashlib

with open(weights, "rb") as handle:
    record["weights_sha256"] = hashlib.sha256(handle.read()).hexdigest()

# Frozen-prefix integrity: every recorded dist still matches its
# RECORD-based tree digest from the acceptance manifest.
with open(os.path.join(pydeps, "DEPLOYED.json")) as handle:
    deployed = {d["name"]: d for d in json.load(handle)["distributions"]}

def _dist_info(name):
    needle = name.replace("-", "_").lower() + "-"
    for entry in os.listdir(pydeps):
        if entry.lower().startswith(needle) and (
                entry.endswith(".dist-info") or entry.endswith(".egg-info")):
            return os.path.join(pydeps, entry)
    raise AssertionError("dist-info not found for %s" % name)

for name, want in sorted(deployed.items()):
    dist = md.Distribution.at(_dist_info(name))
    digest = hashlib.sha256()
    files = 0
    for item in (dist.files or []):
        path = os.path.normpath(os.path.join(pydeps, str(item)))
        if not os.path.isfile(path):
            continue
        rel = os.path.relpath(path, pydeps)
        with open(path, "rb") as handle:
            digest.update(hashlib.sha256(("%-96s" % rel).encode()).digest())
            digest.update(hashlib.sha256(handle.read()).digest())
        files += 1
    assert files == want["files"], (name, files, want["files"])
    got = digest.hexdigest()
    assert got == want["tree_sha256"], (name, got, want["tree_sha256"])
record["frozen_prefix_dists_verified"] = len(deployed)

with open(os.path.join(out_dir, "semantic_offline_proof.json"), "w") as handle:
    json.dump(record, handle, indent=2, sort_keys=True)
print(json.dumps(record, indent=2, sort_keys=True))
PYEOF
rc=$?
restore
trap - EXIT
if [[ $rc -ne 0 ]]; then
  echo "semantic offline isolation proof FAILED (rc=$rc)" >&2
  exit $rc
fi

# -- manifest hash match (same pipeline the manifest recorded) -------------
vendor_tree=$(cd "$SHARE/vendor" && LC_ALL=C find clip_pkg deps clip_models -type f \
  | LC_ALL=C sort | xargs sha256sum | sha256sum | cut -d' ' -f1)
python3 - "$WS" "$OUT" "$vendor_tree" "$PYDEPS" <<'PYEOF'
import json
import os
import sys

ws, out_dir, vendor_tree, pydeps = sys.argv[1:5]
with open(os.path.join(ws, "src", "luggage_planning", "config", "planning_contract", "st09_acceptance_manifest.json")) as h:
    frozen = json.load(h)["semantic_assets"]
with open(os.path.join(out_dir, "semantic_offline_proof.json")) as h:
    proof = json.load(h)
assert proof["weights_sha256"] == frozen["yolov8s_world_pt"]["sha256"], \
    "weights sha256 drifted from the acceptance manifest"
assert vendor_tree == frozen["clip_vendor"]["tree_sha256"], \
    "vendor tree digest drifted from the acceptance manifest"
with open(os.path.join(pydeps, "DEPLOYED.json")) as h:
    deployed = json.load(h)["distributions"]
manifest_deps = frozen["python_deps"]["distributions"]
assert deployed == manifest_deps, \
    "frozen prefix deployment drifted from the acceptance manifest"

# [ST09-I S5.7-6 评审点 4] whole-prefix root digest — VERIFY_ROOT_DIGEST=1
# (freeze_python_prefix.sh, S6, and replay/R runners). Recomputed with
# the exact recorded semantics (LC_ALL=C sorted relative paths, sha256
# per file, sha256 of the sha256sum stream) against the manifest's
# freeze record. The record lives OUTSIDE the prefix — recompute is
# stable forever (the freeze script's no-self-reference rule).
if os.environ.get("VERIFY_ROOT_DIGEST") == "1":
    import hashlib as _hl

    freeze = frozen["python_deps"].get("freeze") or {}
    if not freeze.get("root_sha256"):
        raise AssertionError(
            "FREEZE_RECORD_MISSING: semantic_assets.python_deps.freeze"
            ".root_sha256 absent — run scripts/st09_i/"
            "freeze_python_prefix.sh")
    entries = []
    for root, _dirs, names in os.walk(pydeps):
        for name in names:
            path = os.path.join(root, name)
            rel = "./" + os.path.relpath(path, pydeps)
            with open(path, "rb") as fh:
                entries.append((rel, _hl.sha256(fh.read()).hexdigest()))
    digest = _hl.sha256()
    for rel, file_hash in sorted(entries):
        digest.update(("%s  %s\n" % (file_hash, rel)).encode())
    got = digest.hexdigest()
    assert got == freeze["root_sha256"], (
        "root digest mismatch: %s != %s" % (got, freeze["root_sha256"]))
    proof.update({"root_digest_verified": True,
                  "root_digest_files": len(entries)})
proof.update({"manifest_match": True,
              "vendor_tree_sha256": vendor_tree})
with open(os.path.join(out_dir, "semantic_offline_proof.json"), "w") as h:
    json.dump(proof, h, indent=2, sort_keys=True)
print(json.dumps({"manifest_match": True,
                  "vendor_tree_sha256": vendor_tree,
                  "frozen_dists": len(deployed)}, indent=2))
PYEOF
rc2=$?
if [[ $rc2 -ne 0 ]]; then
    echo "semantic offline manifest/root-digest proof FAILED (rc=$rc2)" >&2
    exit $rc2
fi
echo "semantic offline isolation proof PASSED"
