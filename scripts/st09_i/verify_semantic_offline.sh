#!/usr/bin/env bash
# [ST09-I S5.7-6] Offline isolation proof for the semantic assets.
#
# Proves, in ONE run, everything the frozen-asset rule requires:
#   1. with the SOURCE-tree vendor hidden, the backend still constructs
#      (everything model/vendor-related resolves from the install
#      overlay package share);
#   2. with ~/.cache/clip hidden, nothing reads or writes it;
#   3. with dead proxies (no network reachable), zero install/download
#      attempts happen (any pip/ultralytics fetch would fail fast and
#      the construction would abort — no silent stub fallback either:
#      require_backend turns construction failure into a named error);
#   4. the runtime-reported load paths (model_path / clip_vendor stats)
#      sit under the candidate install overlay;
#   5. the sha256s match the acceptance manifest's semantic_assets;
#   6. the pinned interpreter-side dependencies (ultralytics/torch in
#      the user site) are RECORDED — they are the explicit pinned
#      system dependencies, everything else comes from the overlay.
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

# -- hide the source-tree vendor for the whole proof ----------------------
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

# -- dead proxies: any network fetch fails fast and loudly ---------------
export HTTP_PROXY="http://127.0.0.1:9" HTTPS_PROXY="http://127.0.0.1:9"
export ALL_PROXY="http://127.0.0.1:9" all_proxy="http://127.0.0.1:9"
export NO_PROXY="" no_proxy=""
export YOLO_AUTOUPDATE="false"
unset LUGGAGE_CLIP_VENDOR_DIR || true

set +u
source /opt/ros/humble/setup.bash
source "$OVERLAY/setup.bash"
set -u
# The overlay setup must be the ONLY repo python path source: scrub any
# residual src-tree entries a caller's profile may have injected.
export PYTHONPATH="${PYTHONPATH//"$WS\/src"/}"

python3 - "$OUT" "$OVERLAY" "$SHARE" <<'PYEOF'
import importlib
import json
import os
import sys

out_dir, overlay, share = sys.argv[1], sys.argv[2], sys.argv[3]
record = {"source_vendor_hidden": True, "dead_proxy": True}

# Provenance: the module itself must come from the overlay.
mod = importlib.import_module("luggage_perception.semantic_segmenter")
record["module_path"] = os.path.abspath(mod.__file__)
assert overlay in record["module_path"], record["module_path"]

from luggage_perception.semantic_segmenter import build_segmenter

weights = os.path.join(share, "models", "yolov8s-world.pt")
segmenter = build_segmenter({
    "backend": "bbox_fill",
    "prompts": ["suitcase"],
    "model_name": weights,
    "device": "cpu",
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

# Pinned interpreter-side dependencies: user-site packages the backend
# stack needs. They are OUTSIDE the overlay by design (too heavy to
# vendor); the record makes them explicit pinned system dependencies.
pinned = {}
for name in ("ultralytics", "torch", "cv2", "numpy"):
    try:
        dep = importlib.import_module(name)
        pinned[name] = {
            "version": str(getattr(dep, "__version__", "unknown")),
            "path": os.path.abspath(dep.__file__),
        }
    except ImportError as exc:
        raise AssertionError("pinned dependency %s missing: %s"
                             % (name, exc))
record["pinned_system_dependencies"] = pinned

import hashlib
with open(weights, "rb") as handle:
    record["weights_sha256"] = hashlib.sha256(handle.read()).hexdigest()

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
python3 - "$WS" "$OUT" "$vendor_tree" <<'PYEOF'
import json
import os
import sys

ws, out_dir, vendor_tree = sys.argv[1], sys.argv[2], sys.argv[3]
with open(os.path.join(ws, "st09_contracts", "st09_acceptance_manifest.json")) as h:
    frozen = json.load(h)["semantic_assets"]
with open(os.path.join(out_dir, "semantic_offline_proof.json")) as h:
    proof = json.load(h)
assert proof["weights_sha256"] == frozen["yolov8s_world_pt"]["sha256"], \
    "weights sha256 drifted from the acceptance manifest"
assert vendor_tree == frozen["clip_vendor"]["tree_sha256"], \
    "vendor tree digest drifted from the acceptance manifest"
with open(os.path.join(out_dir, "semantic_offline_proof.json"), "w") as h:
    proof["manifest_match"] = True
    proof["vendor_tree_sha256"] = vendor_tree
    json.dump(proof, h, indent=2, sort_keys=True)
print(json.dumps({"manifest_match": True,
                  "vendor_tree_sha256": vendor_tree}, indent=2))
PYEOF
echo "semantic offline isolation proof PASSED"
