#!/usr/bin/env bash
# Optional CloudCompare CLI helper. Install: sudo apt install cloudcompare
set -euo pipefail

INPUT="${1:?Usage: $0 room.las [output_dir]}"
OUT_DIR="${2:-./output_cc}"
mkdir -p "$OUT_DIR"

if ! command -v CloudCompare >/dev/null 2>&1; then
  echo "CloudCompare not installed. Run: sudo apt install cloudcompare"
  echo "Or use: python3 scripts/extract_cargo_from_room.py"
  exit 1
fi

# Opens cloud for manual crop in GUI; batch crop flags vary by CC version.
CloudCompare "$INPUT" &
echo "CloudCompare opened. Use Segment/Crop Box (~2.8x2.4x2.6 m) then File->Save to:"
echo "  $OUT_DIR"
