#!/usr/bin/env bash
# Convert CloudCompare native BIN (CCB2) to LAS for measure_cargo_from_las.py
# Usage: convert_cloudcompare_bin.sh input.bin|box.las output.las
set -euo pipefail

INPUT="${1:?Usage: $0 input.bin|box.las output.las}"
OUTPUT="${2:?Usage: $0 input.bin|box.las output.las}"

if ! command -v CloudCompare >/dev/null 2>&1; then
  echo "CloudCompare not found. Install: sudo apt install cloudcompare"
  echo "Or export LAS/PLY manually from CloudCompare GUI."
  exit 1
fi

if [[ ! -f "$INPUT" ]]; then
  echo "Input not found: $INPUT"
  exit 1
fi

mkdir -p "$(dirname "$OUTPUT")"
CloudCompare -SILENT -AUTO_SAVE OFF -O "$INPUT" -C_EXPORT_FMT LAS -SAVE_CLOUDS FILE "$OUTPUT"
if [[ ! -f "$OUTPUT" ]]; then
  echo "Conversion failed: $OUTPUT not created"
  exit 1
fi
echo "Wrote $OUTPUT"
