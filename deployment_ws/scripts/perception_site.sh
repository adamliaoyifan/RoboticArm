#!/usr/bin/env bash
# 实机代码 (site / real-cell). Not part of the Gazebo simulation stack.
# Wrapper: same implementation as src/luggage_perception/scripts/perception_site.sh
set -euo pipefail
DEPLOY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "$DEPLOY/.." && pwd)"
SRC="$REPO/src/luggage_perception/scripts/perception_site.sh"
if [[ ! -f "$SRC" ]]; then
  echo "missing $SRC" >&2
  exit 1
fi
exec "$SRC" "$@"
