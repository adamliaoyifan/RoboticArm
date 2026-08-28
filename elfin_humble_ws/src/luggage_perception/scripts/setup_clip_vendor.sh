#!/usr/bin/env bash
# Populate luggage_perception/vendor/ with CLIP (package + deps + ViT-B/32
# weights) so the YOLO-World semantic backends run offline, without
# ultralytics' slow `pip install git+.../CLIP.git` auto-install.
#
# Run once on a host with network access to GitHub + PyPI + Azure CDN. The
# resulting vendor/ folder is committed-ignored but lives in the repo tree, so
# it is picked up by whatever container mounts the workspace. Re-run anytime to
# refresh. Override the interpreter with $PYTHON (default python3).
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR="$PKG_DIR/vendor"
PY="${PYTHON:-python3}"

mkdir -p "$VENDOR/clip_pkg" "$VENDOR/deps" "$VENDOR/clip_models"

echo "[1/3] Cloning ultralytics/CLIP -> $VENDOR/clip_pkg"
rm -rf "$VENDOR/clip_pkg"
git clone --depth 1 https://github.com/ultralytics/CLIP.git "$VENDOR/clip_pkg"

echo "[2/3] Installing ftfy + regex (+ wcwidth) -> $VENDOR/deps"
rm -rf "$VENDOR/deps"
"$PY" -m pip install --target="$VENDOR/deps" ftfy regex

echo "[3/3] Downloading CLIP ViT-B/32 weights -> $VENDOR/clip_models"
URL=https://openaipublic.azureedge.net/clip/models/40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af/ViT-B-32.pt
EXPECT=40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af
curl -fsSL -o "$VENDOR/clip_models/ViT-B-32.pt" "$URL"
sha="$(sha256sum "$VENDOR/clip_models/ViT-B-32.pt" | cut -d' ' -f1)"
if [ "$sha" != "$EXPECT" ]; then
  echo "ERROR: ViT-B-32.pt sha256 mismatch (got $sha, want $EXPECT)" >&2
  exit 1
fi

echo "OK: CLIP vendor populated at $VENDOR"
echo "  clip_pkg/   - ultralytics/CLIP python package (import clip)"
echo "  deps/       - ftfy, regex, wcwidth (CLIP runtime deps)"
echo "  clip_models/ViT-B-32.pt  - $(du -h "$VENDOR/clip_models/ViT-B-32.pt" | cut -f1) weights"
