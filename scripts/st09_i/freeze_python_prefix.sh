#!/usr/bin/env bash
# [ST09-I S5.7-6 / S6] Freeze the candidate-owned python deps prefix as a
# CONTENT-ADDRESSED deployment artifact and record it in the acceptance
# manifest (semantic_assets.python_deps.freeze).
#
# Discipline (user lock 2026-10-03, 评审点 4):
#   * The freeze record (root digest included) lives ONLY in the runtime
#     manifest OUTSIDE the prefix; no freeze digest is ever written back
#     into any file under pydeps-frozen/ (DEPLOYED.json stays exactly as
#     the deploy bootstrap wrote it) — no self-reference, the root digest
#     recomputes stably forever.
#   * The root digest covers EVERY file in the prefix (DEPLOYED.json
#     included), LC_ALL=C sorted relative paths, sha256 per file, sha256
#     of the stream.
#   * The script self-checks: manifest path outside the prefix, and two
#     consecutive root-digest computations byte-identical.
#   * GATE: after writing the record, VERIFY_ROOT_DIGEST=1
#     verify_semantic_offline.sh must pass (cuda construction + minimal
#     inference + whole-prefix digest recomputed against the record this
#     script just wrote — the round-trip proof).
#
# Usage: scripts/st09_i/freeze_python_prefix.sh [--ws WS]
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ws) WS="$(cd "$2" && pwd)"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done
TARGET="$WS/pydeps-frozen"
MANIFEST="$WS/st09_contracts/st09_acceptance_manifest.json"

[[ -f "$TARGET/DEPLOYED.json" ]] || {
    echo "frozen python deps prefix absent at $TARGET; run scripts/st09_i/deploy_python_deps.sh" >&2
    exit 2
}
[[ -f "$MANIFEST" ]] || {
    echo "acceptance manifest absent at $MANIFEST" >&2
    exit 2
}

# -- anti-self-reference precondition --------------------------------------
case "$MANIFEST" in
    "$TARGET"/*) echo "manifest must live OUTSIDE the prefix" >&2; exit 2 ;;
esac

root_digest_command='cd pydeps-frozen && LC_ALL=C find . -type f | LC_ALL=C sort | xargs sha256sum | sha256sum'

compute_root_sha256() {
    (cd "$TARGET" && LC_ALL=C find . -type f | LC_ALL=C sort \
        | xargs sha256sum | sha256sum | cut -d' ' -f1)
}

ROOT_A="$(compute_root_sha256)"
ROOT_B="$(compute_root_sha256)"
if [[ "$ROOT_A" != "$ROOT_B" ]]; then
    echo "root digest unstable ($ROOT_A vs $ROOT_B) — self-reference suspected" >&2
    exit 3
fi
ROOT_SHA256="$ROOT_A"

DEPLOYED_SHA256="$(sha256sum "$TARGET/DEPLOYED.json" | cut -d' ' -f1)"

# -- ABI / platform / cuda under the frozen prefix -------------------------
NVIDIA_LIBS="$(LC_ALL=C find "$TARGET" -type d -name lib 2>/dev/null | while read -r d; do
    case "$d" in
        *nvidia*) printf '%s:' "$d" ;;
    esac
done)"
export PYTHONNOUSERSITE=1
export PYTHONPATH="$TARGET"
export LD_LIBRARY_PATH="$NVIDIA_LIBS${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

PREFIX_FACTS="$(python3 - "$WS" <<'PYEOF'
import importlib
import json
import os
import platform
import sys
import sysconfig

ws = sys.argv[1]
facts = {}
facts["python_abi"] = sysconfig.get_config_var("EXT_SUFFIX") or ""
facts["platform"] = "%s-%s" % (platform.machine(), "-".join(platform.libc_ver()))
module_paths = {}
for name in ("torch", "torchvision", "ultralytics"):
    try:
        module = importlib.import_module(name)
        module_paths[name] = str(getattr(module, "__file__", "")
                                 or "%s/" % module.__path__[0])
    except Exception as error:  # noqa: BLE001 - record, do not guess
        module_paths[name] = "unavailable: %r" % (error,)
# clip is NOT a prefix module: it is the overlay's vendored package
# (share/luggage_perception/vendor/clip_pkg) — probe it there, exactly
# where the offline proof loads it from.
vendor = os.path.join(
    ws, "install-st09", "luggage_perception", "share", "luggage_perception",
    "vendor", "clip_pkg")
if os.path.isdir(vendor):
    sys.path.insert(0, vendor)
try:
    module = importlib.import_module("clip")
    module_paths["clip"] = str(getattr(module, "__file__", "")
                               or "%s/" % module.__path__[0])
except Exception as error:  # noqa: BLE001 - record, do not guess
    module_paths["clip"] = "unavailable: %r" % (error,)
facts["module_paths"] = module_paths
try:
    import torch

    facts["cuda_runtime"] = str(torch.version.cuda or "")
except Exception as error:  # noqa: BLE001
    facts["cuda_runtime"] = "unavailable: %r" % (error,)
print(json.dumps(facts))
PYEOF
)"

OBSERVED_DRIVER="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 || true)"
OBSERVED_DRIVER="${OBSERVED_DRIVER:-unavailable}"

# -- archive (gitignored large asset) --------------------------------------
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="$WS/docs/status/evidence/freeze/python_prefix/$STAMP"
mkdir -p "$OUT"
tar -C "$WS" -cf "$OUT/pydeps-frozen.tar" pydeps-frozen
ARCHIVE_SHA256="$(sha256sum "$OUT/pydeps-frozen.tar" | cut -d' ' -f1)"
ARCHIVE_BYTES="$(stat -c%s "$OUT/pydeps-frozen.tar")"

# -- manifest record (json edit, same pattern deploy_python_deps.sh uses) --
python3 - "$MANIFEST" "$ROOT_SHA256" "$DEPLOYED_SHA256" "$PREFIX_FACTS" \
    "$OBSERVED_DRIVER" "$OUT" "$ARCHIVE_SHA256" "$ARCHIVE_BYTES" \
    "$root_digest_command" <<'PYEOF'
import datetime
import json
import sys

manifest_path, root_sha, deployed_sha, facts_json, observed_driver, \
    out_dir, archive_sha, archive_bytes, digest_command = sys.argv[1:10]
facts = json.loads(facts_json)
with open(manifest_path, encoding="utf-8") as handle:
    manifest = json.load(handle)
assets = manifest.setdefault("semantic_assets", {})
python_deps = assets.setdefault("python_deps", {})
python_deps["freeze"] = {
    "root_sha256": root_sha,
    "tree_digest_command": digest_command,
    "python_abi": facts["python_abi"],
    "platform": facts["platform"],
    "cuda_runtime": facts["cuda_runtime"],
    "cuda_min_driver_linux": {
        "floor": "525.60.13",
        "observed": observed_driver,
    },
    "module_paths": facts["module_paths"],
    "deployed_json_sha256": deployed_sha,
    "archive": {
        "format": "tar",
        "path": out_dir,
        "sha256": archive_sha,
        "bytes": int(archive_bytes),
        "gitignored": True,
    },
    "named_residuals": {
        "urdf_hash_separation": {
            "status": "residual",
            "note": ("launch runtime identity urdf_sha256 embeds the spawn "
                     "initial joints; sim pose files pin SRDF + scene "
                     "revision + joint order and leave urdf EMPTY. "
                     "Convergence: kinematic_urdf_sha256 (normalized, "
                     "initial joints excluded) + initial_joint_state_digest "
                     "+ srdf_sha256 + joint order. Never claim a URDF full "
                     "match until separated."),
        },
    },
    "frozen_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
}
python_deps["runtime_rule"] = (
    "R takes the S6-frozen EXACT prefix (never a bootstrap re-run): verify "
    "the archive sha256, extract, recompute the whole-prefix root digest "
    "with the recorded file list and LC_ALL=C sort, and verify every "
    "dist's RECORD tree digest — all three layers must pass before "
    "PYTHONNOUSERSITE=1 offline start.")
with open(manifest_path, "w", encoding="utf-8") as handle:
    json.dump(manifest, handle, indent=2, sort_keys=True)
    handle.write("\n")
print(json.dumps(python_deps["freeze"], indent=2, sort_keys=True))
PYEOF

# -- GATE: the record this script just wrote must recompute ----------------
VERIFY_OUT="$(mktemp -d /tmp/freeze_gate.XXXXXX)"
trap 'rm -rf "$VERIFY_OUT"' EXIT
if ! VERIFY_ROOT_DIGEST=1 bash "$WS/scripts/st09_i/verify_semantic_offline.sh" \
        --out "$VERIFY_OUT" --ws "$WS"; then
    echo "freeze gate FAILED; the manifest record is NOT certified" >&2
    exit 4
fi

echo "freeze complete: root=$ROOT_SHA256 archive=$OUT gate=passed"
