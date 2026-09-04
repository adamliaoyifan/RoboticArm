#!/usr/bin/env bash
# Download yolov8s-world.pt into the perception package root so CMake can
# install it to share/luggage_perception/models.
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$PKG_DIR/yolov8s-world.pt"
URL="${YOLO_WORLD_URL:-https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8s-world.pt}"

if [[ -f "$DEST" ]]; then
  echo "already present: $DEST ($(du -h "$DEST" | cut -f1))"
  exit 0
fi

echo "downloading $URL -> $DEST"
curl -fL --retry 3 -o "$DEST.partial" "$URL"
mv "$DEST.partial" "$DEST"
echo "OK: $DEST ($(du -h "$DEST" | cut -f1))"
echo "rebuild luggage_perception so share/models is updated"
