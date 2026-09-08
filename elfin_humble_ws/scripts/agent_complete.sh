#!/usr/bin/env bash
# Close one owner-assigned subtask after implementation and tests pass.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/agent_mailbox.py" complete "$@"

usage() {
  cat <<'USAGE'
Usage:
  scripts/agent_complete.sh --thread FILE --role ROLE --agent AGENT \
    --model MODEL --cli CLI --outcome pass|blocked --summary TEXT [options]

Options:
  --revision REV   Required for pass; must resolve to a Git commit.
  --tests TEXT     Required for pass; one-line test result.
  --started-at ISO Migration-only start time when no Claim event exists.
  --evidence PATH  Evidence pointer. May be repeated.
  -h, --help       Show this help.

Set AGENT_COORD_ROOT when running from a satellite worktree.
USAGE
}

trim_cell() {
  local value="$*"
  value="${value//$'\n'/ }"
  value="${value//|/\\|}"
  printf '%s' "$value"
}

metadata_value() {
  local file="$1"
  local key="$2"
  sed -n "s/^- ${key}: //p" "$file" | head -n 1
}

THREAD=""
ROLE=""
AGENT=""
MODEL=""
CLI=""
OUTCOME=""
SUMMARY=""
REVISION=""
TESTS=""
STARTED_AT=""
EVIDENCE=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --thread) THREAD="${2:-}"; shift 2 ;;
    --role) ROLE="${2:-}"; shift 2 ;;
    --agent) AGENT="${2:-}"; shift 2 ;;
    --model) MODEL="${2:-}"; shift 2 ;;
    --cli) CLI="${2:-}"; shift 2 ;;
    --outcome) OUTCOME="${2:-}"; shift 2 ;;
    --summary) SUMMARY="${2:-}"; shift 2 ;;
    --revision) REVISION="${2:-}"; shift 2 ;;
    --tests) TESTS="${2:-}"; shift 2 ;;
    --started-at) STARTED_AT="${2:-}"; shift 2 ;;
    --evidence) EVIDENCE+=("${2:-}"); shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown arg: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "$THREAD" && -n "$ROLE" && -n "$AGENT" && -n "$MODEL" && \
   -n "$CLI" && -n "$OUTCOME" && -n "$SUMMARY" ]] || {
  usage >&2
  exit 2
}
[[ "$THREAD" != */* && "$THREAD" == *.md ]] || {
  echo "--thread must be a discuss filename" >&2
  exit 2
}
[[ "$OUTCOME" =~ ^(pass|blocked)$ ]] || {
  echo "invalid --outcome: $OUTCOME" >&2
  exit 2
}
if [[ "$OUTCOME" == "pass" && ( -z "$REVISION" || -z "$TESTS" ) ]]; then
  echo "--outcome pass requires --revision and --tests" >&2
  exit 2
fi

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

KIND="$(metadata_value "$THREAD_PATH" kind)"
STATUS="$(metadata_value "$THREAD_PATH" status)"
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

if ! grep -q '^- started_at: ' "$THREAD_PATH"; then
  [[ -n "$STARTED_AT" ]] || {
    echo "thread has no Claim event; run agent_start.sh before work" >&2
    echo "or use --started-at ISO only to migrate work already in progress" >&2
    exit 1
  }
  date -d "$STARTED_AT" >/dev/null 2>&1 || {
    echo "invalid --started-at timestamp: $STARTED_AT" >&2
    exit 2
  }
  HUMAN_TIME="$(date '+%Y-%m-%d %H:%M')"
  {
    printf '## Claim -- %s/%s -- %s -- %s/%s\n\n' \
      "$ROLE" "$AGENT" "$HUMAN_TIME" "$CLI" "$MODEL"
    printf -- '- started_at: %s\n\n' "$STARTED_AT"
  } >> "$THREAD_PATH"
fi

RESOLVED_REVISION="$REVISION"
if [[ "$OUTCOME" == "pass" ]]; then
  RESOLVED_REVISION="$(git rev-parse --verify "${REVISION}^{commit}" 2>/dev/null)" || {
    echo "revision does not resolve to a Git commit: $REVISION" >&2
    exit 1
  }
fi

HUMAN_TIME="$(date '+%Y-%m-%d %H:%M')"
ISO_TIME="$(date --iso-8601=seconds)"
{
  printf '## Result -- %s/%s -- %s -- %s/%s\n\n' \
    "$ROLE" "$AGENT" "$HUMAN_TIME" "$CLI" "$MODEL"
  printf -- '- outcome: %s\n' "$OUTCOME"
  printf -- '- completed_at: %s\n' "$ISO_TIME"
  [[ -n "$RESOLVED_REVISION" ]] &&
    printf -- '- revision: %s\n' "$RESOLVED_REVISION"
  [[ -n "$TESTS" ]] && printf -- '- tests: %s\n' "$(trim_cell "$TESTS")"
  printf -- '- summary: %s\n' "$(trim_cell "$SUMMARY")"
  for pointer in "${EVIDENCE[@]}"; do
    printf -- '- evidence: %s\n' "$pointer"
  done
  printf '\n'
} >> "$THREAD_PATH"

if [[ "$OUTCOME" == "pass" ]]; then
  thread_tmp="$(mktemp)"
  while IFS= read -r line; do
    if [[ "$line" == "- status:"* ]]; then
      printf -- '- status: done\n' >> "$thread_tmp"
    else
      printf '%s\n' "$line" >> "$thread_tmp"
    fi
  done < "$THREAD_PATH"
  mv "$thread_tmp" "$THREAD_PATH"

  open_tmp="$(mktemp)"
  while IFS= read -r line; do
    [[ "$line" == *"| $THREAD |"* ]] || printf '%s\n' "$line" >> "$open_tmp"
  done < "$OPEN_FILE"
  mv "$open_tmp" "$OPEN_FILE"
  echo "completed $THREAD at $RESOLVED_REVISION"
else
  echo "left $THREAD open as blocked"
fi
