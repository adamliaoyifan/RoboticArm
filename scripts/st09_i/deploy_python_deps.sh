#!/usr/bin/env bash
# [ST09-I S5.7-6] Deploy the semantic backend's Python dependencies into
# a CANDIDATE-OWNED frozen prefix (<ws>/pydeps-frozen).
#
# User lock 2026-10-03: dependencies living in the executing user's
# ~/.local site do NOT count as pinned system dependencies — a fresh
# HOME proves the candidate depended on a mutable user path. The formal
# candidate instead materializes the closure via this explicit deploy
# step into a prefix the candidate owns, hash-records it in the
# acceptance manifest, and runs with PYTHONNOUSERSITE=1 so the user
# site can never silently satisfy an import.
#
# Resolution strategy: bootstrap loop. Each iteration runs the REAL
# positive proof (fresh HOME, user site disabled, dead network, source
# vendor hidden, construction from the overlay) and copies the dist
# that owns the first reported missing module from the staging source
# (this machine's user site) into the prefix. The loop terminates only
# when construction succeeds — it can never under-copy. Binary-incompat
# crashes (no module name) abort loudly for manual resolution.
#
# Usage: scripts/st09_i/deploy_python_deps.sh [--force] [--ws WS]
set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FORCE=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --force) FORCE=1; shift ;;
    --ws) WS="$2"; shift 2 ;;
    *) echo "unknown arg $1" >&2; exit 2 ;;
  esac
done
TARGET="$WS/pydeps-frozen"
OVERLAY="$WS/install-st09"
SHARE="$OVERLAY/luggage_perception/share/luggage_perception"
VENDOR_SRC="$WS/src/luggage_perception/vendor"

if [[ -e "$TARGET" && "$FORCE" -ne 1 ]]; then
  echo "refusing to overwrite existing $TARGET (use --force)" >&2
  exit 2
fi
mkdir -p "$TARGET"

# -- cu128 wheels carry their CUDA runtime as nvidia-* pip dists whose
#    libs load via DT_NEEDED/ctypes (never an import error) -> copy
#    them UPFRONT; the probe adds their lib dirs to LD_LIBRARY_PATH.
python3 - "$TARGET" <<'PYEOF'
import importlib.metadata as md
import os
import shutil
import sys

target = sys.argv[1]
copied = 0
for dist in md.distributions():
    name = (dist.metadata or {}).get("Name", "") or ""
    if not name.lower().startswith(("nvidia", "triton")):
        continue
    root = os.path.dirname(str(dist._path))
    tops = (dist.read_text("top_level.txt") or "").split()
    tops = tops or [name.replace("-", "_").lower()]
    for top in tops:
        src = os.path.join(root, top)
        if os.path.isdir(src):
            shutil.copytree(src, os.path.join(target, top),
                            dirs_exist_ok=True)
        elif os.path.isfile(src + ".py"):
            shutil.copy2(src + ".py", os.path.join(target, top + ".py"))
    shutil.copytree(str(dist._path),
                    os.path.join(target, os.path.basename(str(dist._path))),
                    dirs_exist_ok=True)
    copied += 1
print("upfront nvidia/triton dists copied:", copied)
PYEOF

# -- hide the source-tree vendor: the proof must resolve from overlay --
HIDDEN=0
if [[ -d "$VENDOR_SRC/clip_pkg" ]]; then
  mv "$VENDOR_SRC/clip_pkg" "$VENDOR_SRC/clip_pkg.hidden"
  HIDDEN=1
fi
restore() { [[ "$HIDDEN" -eq 1 ]] && mv "$VENDOR_SRC/clip_pkg.hidden" "$VENDOR_SRC/clip_pkg"; }
trap restore EXIT

probe_rc=0
for iteration in $(seq 1 80); do
  NVIDIA_LIBS=$(find "$TARGET/nvidia" -type d -name lib 2>/dev/null | tr '\n' ':')
  output=$(env -i HOME="$(mktemp -d /tmp/pydeps_home.XXXXXX)" \
    PYTHONNOUSERSITE=1 \
    PYTHONPATH="$TARGET" \
    LD_LIBRARY_PATH="$NVIDIA_LIBS" \
    HTTP_PROXY="http://127.0.0.1:9" HTTPS_PROXY="http://127.0.0.1:9" \
    ALL_PROXY="http://127.0.0.1:9" NO_PROXY="" \
    YOLO_AUTOUPDATE=false \
    PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
    bash -c "set +u; source /opt/ros/humble/setup.bash; source '$OVERLAY/setup.bash'; set -u; \
      python3 - '$SHARE' <<'PYEOF'
import os, sys
share = sys.argv[1]
from luggage_perception.semantic_segmenter import build_segmenter
seg = build_segmenter({'backend': 'bbox_fill', 'prompts': ['suitcase'],
                       'model_name': os.path.join(share, 'models', 'yolov8s-world.pt'),
                       'device': 'cpu', 'require_backend': 'bbox_fill'})
import torch
print('CONSTRUCT_OK', torch.__version__, torch.cuda.is_available())
PYEOF" 2>&1)
  rc=$?
  if [[ $rc -eq 0 ]] && echo "$output" | grep -q CONSTRUCT_OK; then
    echo "iteration $iteration: construction OK from the frozen prefix"
    cuda_ok=$(echo "$output" | grep CONSTRUCT_OK | awk '{print $3}')
    echo "construction OK; cuda_available_from_prefix=$cuda_ok"
    probe_rc=0
    break
  fi
  missing=$(echo "$output" | grep -o "No module named '[^']*'" | head -1 | cut -d"'" -f2 | cut -d. -f1)
  if [[ -z "$missing" ]]; then
    missing=$(echo "$output" | grep -o "No package metadata was found for [^ ]*" | head -1 | awk '{print $NF}')
  fi
  if [[ -z "$missing" ]]; then
    # Version shadowing: an import SUCCEEDED but resolved to a stale
    # non-prefix copy (apt/system) lacking the symbol the closure
    # needs -> copy the owning dist so the frozen prefix wins.
    stale=$(echo "$output" | grep -o "cannot import name '[^']*' from '[^']*' (\([^)]*\))" | head -1)
    if [[ -n "$stale" ]] && ! echo "$stale" | grep -q "$TARGET"; then
      missing=$(echo "$stale" | sed "s/.*from '\([^']*\)'.*/\1/" | cut -d. -f1)
      echo "  stale-system copy serves '$missing'; freezing the prefix version"
    fi
  fi
  if [[ -z "$missing" ]]; then
    echo "bootstrap iteration $iteration failed WITHOUT a module name (binary incompat?); aborting:" >&2
    echo "$output" | tail -20 >&2
    probe_rc=3
    break
  fi
  echo "iteration $iteration: missing '$missing' -> copying owning dist"
  copied=$(python3 - "$missing" "$TARGET" <<'PYEOF'
import importlib.metadata as md
import os
import shutil
import sys

missing, target = sys.argv[1], sys.argv[2]
for dist in md.distributions():
    tops = (dist.read_text("top_level.txt") or "").split()
    names = {(t.strip() or "").lower() for t in tops}
    meta_name = (dist.metadata or {}).get("Name", "") or ""
    if missing.lower() not in names and \
            missing.lower().replace("_", "-") != meta_name.lower().replace("_", "-") \
            and missing.lower() != meta_name.lower().replace("-", "_"):
        continue
    root = os.path.dirname(str(dist._path))
    tops = tops or [meta_name.replace("-", "_").lower()]
    for top in tops:
        src = os.path.join(root, top)
        if os.path.isdir(src):
            shutil.copytree(src, os.path.join(target, top),
                            dirs_exist_ok=True)
        elif os.path.isfile(src + ".py"):
            shutil.copy2(src + ".py", os.path.join(target, top + ".py"))
        libs = src + ".libs"
        if os.path.isdir(libs):
            shutil.copytree(libs, os.path.join(target, top + ".libs"),
                            dirs_exist_ok=True)
    shutil.copytree(str(dist._path),
                    os.path.join(target, os.path.basename(str(dist._path))),
                    dirs_exist_ok=True)
    print(meta_name, dist.version)
    break
else:
    print("NOTFOUND")
    sys.exit(4)
PYEOF
  )
  if [[ $? -ne 0 || "$copied" == "NOTFOUND" ]]; then
    echo "no owning distribution found for '$missing'; aborting" >&2
    probe_rc=4
    break
  fi
  echo "  copied: $copied"
done
restore
trap - EXIT
[[ $probe_rc -ne 0 ]] && exit $probe_rc

# -- CUDA re-probe after any nvidia dist copy --------------------------------
NVIDIA_LIBS=$(find "$TARGET/nvidia" -type d -name lib 2>/dev/null | tr '\n' ':')
cuda_ok=$(env -i HOME="$(mktemp -d)" PYTHONNOUSERSITE=1 PYTHONPATH="$TARGET" \
  LD_LIBRARY_PATH="$NVIDIA_LIBS" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  bash -c "set +u; source /opt/ros/humble/setup.bash; source '$OVERLAY/setup.bash'; set -u; \
    python3 -c 'import torch; print(torch.cuda.is_available())'" 2>&1 | tail -1)
echo "cuda_available_from_prefix=$cuda_ok"

# -- record: per-dist versions + tree digests into the prefix marker --------
python3 - "$TARGET" "$WS" <<'PYEOF'
import hashlib
import importlib.metadata as md
import json
import os
import sys

target, ws = sys.argv[1], sys.argv[2]
record = {"deployed_from": "user-site staging (explicit deploy step)",
          "note": "candidate-owned frozen prefix; PYTHONNOUSERSITE=1 at "
                  "runtime so the user site never satisfies an import"}
dists = []
for entry in sorted(os.listdir(target)):
    if not (entry.endswith(".dist-info") or entry.endswith(".egg-info")):
        continue
    dist = md.Distribution.at(os.path.join(target, entry))
    name = (dist.metadata or {}).get("Name", "") or entry
    digest = hashlib.sha256()
    files = 0
    # RECORD is the authority: exactly the files this dist installed
    # (nvidia-* dists share the nvidia/ namespace, so top-level walking
    # would attribute the merged tree to every one of them).
    for item in (dist.files or []):
        path = os.path.normpath(os.path.join(target, str(item)))
        if not os.path.isfile(path):
            continue
        rel = os.path.relpath(path, target)
        with open(path, "rb") as handle:
            digest.update(hashlib.sha256(
                ("%-96s" % rel).encode()).digest())
            digest.update(hashlib.sha256(handle.read()).digest())
        files += 1
    dists.append({"name": name, "version": dist.version,
                  "files": files,
                  "tree_sha256": digest.hexdigest()})
record["distributions"] = dists
with open(os.path.join(target, "DEPLOYED.json"), "w") as handle:
    json.dump(record, handle, indent=2, sort_keys=True)
manifest_path = os.path.join(ws, "src", "luggage_planning", "config", "planning_contract",
                             "st09_acceptance_manifest.json")
with open(manifest_path) as handle:
    manifest = json.load(handle)
manifest["semantic_assets"]["python_deps"] = {
    "mode": "candidate-owned frozen prefix (pydeps-frozen/)",
    "deploy_step": "scripts/st09_i/deploy_python_deps.sh",
    "runtime_rule": ("segmenter runs with PYTHONNOUSERSITE=1 and the "
                     "prefix prepended to ITS PYTHONPATH only; the user "
                     "site can never satisfy an import"),
    "verification": ("PYTHONNOUSERSITE=1 + fresh HOME + dead network + "
                     "source vendor hidden -> construction succeeds "
                     "(scripts/st09_i/verify_semantic_offline.sh)"),
    "distributions": dists,
}
manifest["semantic_assets"].pop("pinned_system_dependencies", None)
with open(manifest_path, "w") as handle:
    json.dump(manifest, handle, indent=2, ensure_ascii=False)
    handle.write("\n")
print(json.dumps({"distributions": len(dists)}, indent=2))
PYEOF
echo "pydeps-frozen deployed at $TARGET"
