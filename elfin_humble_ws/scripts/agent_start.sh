#!/usr/bin/env bash
# Claim one owner-assigned subtask after all declared dependencies pass.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/agent_mailbox.py" start "$@"

usage() {
  cat <<'USAGE'
Usage:
  scripts/agent_start.sh --thread FILE --role ROLE --agent AGENT \
    --model MODEL --cli CLI

The helper verifies concrete ownership and dependency completion, then appends
one Claim event with an ISO-8601 started_at timestamp. Set AGENT_COORD_ROOT
when running from a satellite worktree.
USAGE
}

metadata_value() {
  local file="$1"
  local key="$2"
  sed -n "s/^- ${key}: //p" "$file" | head -n 1
}

dependency_passed() {
  local parent="$1"
  local dependency="$2"
  local candidate

  shopt -s nullglob
  for candidate in docs/agents/discuss/*.md; do
    [[ "$(metadata_value "$candidate" parent)" == "$parent" ]] || continue
    [[ "$(metadata_value "$candidate" subtask)" == "$dependency" ]] || continue
    [[ "$(metadata_value "$candidate" status)" == "done" ]] || continue
    grep -q '^## Result -- ' "$candidate" || continue
    grep -q '^- outcome: pass$' "$candidate" || continue
    shopt -u nullglob
    return 0
  done
  shopt -u nullglob
  return 1
}

THREAD=""
ROLE=""
AGENT=""
MODEL=""
CLI=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --thread) THREAD="${2:-}"; shift 2 ;;
    --role) ROLE="${2:-}"; shift 2 ;;
    --agent) AGENT="${2:-}"; shift 2 ;;
    --model) MODEL="${2:-}"; shift 2 ;;
    --cli) CLI="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown arg: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "$THREAD" && -n "$ROLE" && -n "$AGENT" && -n "$MODEL" && \
   -n "$CLI" ]] || {
  usage >&2
  exit 2
}
[[ "$THREAD" != */* && "$THREAD" == *.md ]] || {
  echo "--thread must be a discuss filename" >&2
  exit 2
}

ROOT="${AGENT_COORD_ROOT:-}"
if [[ -z "$ROOT" ]]; then
  ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
else
  echo "AGENT_COORD_ROOT is set: mailbox root -> $ROOT" >&2
  if [[ ! -f "$ROOT/docs/agents/README.md" || ! -d "$ROOT/src/luggage_gazebo" ]] \
      && [[ "${AGENT_MAILBOX_ALLOW_ANY_ROOT:-0}" != "1" ]]; then
    echo "refusing: $ROOT does not look like the primary workspace" >&2
    exit 1
  fi
fi
cd "$ROOT"

THREAD_PATH="docs/agents/discuss/$THREAD"
OPEN_FILE="docs/agents/discuss/OPEN.md"
[[ -f "$THREAD_PATH" && -f "$OPEN_FILE" ]] || {
  echo "missing thread or mailbox" >&2
  exit 1
}

LOCK_FILE="${AGENT_MAILBOX_LOCK:-/tmp/elfin_humble_agents_open.lock}"
if command -v flock >/dev/null 2>&1; then
  exec 9>"$LOCK_FILE"
  flock 9
fi

STATUS="$(metadata_value "$THREAD_PATH" status)"
KIND="$(metadata_value "$THREAD_PATH" kind)"
PARENT="$(metadata_value "$THREAD_PATH" parent)"
DEPENDS_ON="$(metadata_value "$THREAD_PATH" depends_on)"
TO_ROLE="$(metadata_value "$THREAD_PATH" to_role)"
TO_AGENT="$(metadata_value "$THREAD_PATH" to_agent)"
TO_MODEL="$(metadata_value "$THREAD_PATH" to_model)"

[[ "$STATUS" == "open" ]] || {
  echo "thread is not open: $STATUS" >&2
  exit 1
}
[[ "$KIND" =~ ^(subtask|integration|regression)$ ]] || {
  echo "thread is not runnable owner work: $KIND" >&2
  exit 1
}
[[ "$TO_ROLE" == "$ROLE" || "$TO_ROLE" == "any" ]] || {
  echo "role mismatch: assigned to $TO_ROLE" >&2
  exit 1
}
[[ "$TO_AGENT" == "$AGENT" && "$TO_MODEL" == "$MODEL" ]] || {
  echo "owner mismatch: assigned to $TO_AGENT/$TO_MODEL" >&2
  exit 1
}
grep -Fq "| $THREAD |" "$OPEN_FILE" || {
  echo "thread has no open mailbox row: $THREAD" >&2
  exit 1
}

if grep -q '^- started_at: ' "$THREAD_PATH"; then
  echo "already started $THREAD"
  exit 0
fi

if [[ "$DEPENDS_ON" != "none" ]]; then
  IFS=',' read -r -a dependencies <<< "$DEPENDS_ON"
  for dependency in "${dependencies[@]}"; do
    dependency="${dependency#"${dependency%%[![:space:]]*}"}"
    dependency="${dependency%"${dependency##*[![:space:]]}"}"
    dependency_passed "$PARENT" "$dependency" || {
      echo "dependency is not complete: $PARENT/$dependency" >&2
      exit 1
    }
  done
fi

HUMAN_TIME="$(date '+%Y-%m-%d %H:%M')"
ISO_TIME="$(date --iso-8601=seconds)"
{
  printf '## Claim -- %s/%s -- %s -- %s/%s\n\n' \
    "$ROLE" "$AGENT" "$HUMAN_TIME" "$CLI" "$MODEL"
  printf -- '- started_at: %s\n\n' "$ISO_TIME"
} >> "$THREAD_PATH"

echo "started $THREAD at $ISO_TIME"
