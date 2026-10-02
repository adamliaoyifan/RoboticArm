#!/usr/bin/env bash
# Preserve the caller's explicit workspace/install selection; no source overlay.
set -euo pipefail
exec python3 "$(dirname "${BASH_SOURCE[0]}")/frame_migration_st09_verify.py" "$@"
