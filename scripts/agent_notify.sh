#!/usr/bin/env bash
# Create a discuss thread entry and mailbox row for cross-agent notification.
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/agent_notify.sh --to ROLE --from ROLE --cli CLI --slug SLUG --question TEXT [options]

Options:
  --body TEXT       Thread post body. Defaults to --question.
  --pointer PATH    Add one pointer. May be repeated.
  --thread FILE     Append to an existing discuss thread filename.
  --title TEXT      Thread title for a new thread.
  -h, --help        Show this help.

Roles:
  reviews, eng, test, discuss, any

Set AGENT_COORD_ROOT to write the mailbox in the primary workspace when
running from a satellite git worktree.
USAGE
}

trim_cell() {
  local value="$*"
  value="${value//$'\n'/ }"
  value="${value//|/\\|}"
  printf '%s' "$value"
}

valid_to_role() {
  [[ "$1" =~ ^(reviews|eng|test|discuss|any)$ ]]
}

valid_from_role() {
  [[ "$1" =~ ^(reviews|eng|test|discuss)$ ]]
}

safe_slug() {
  printf '%s' "$1" | tr '[:upper:]' '[:lower:]' | tr -cs '[:alnum:]_-' '-'
}

next_question_id() {
  local open_file="$1"
  local today prefix max line raw_id suffix

  today="$(date +%Y%m%d)"
  prefix="Q-$today-"
  max=0

  while IFS= read -r line; do
    [[ "$line" == \|* ]] || continue
    [[ "$line" == "| id "* || "$line" == "|---"* ]] && continue
    raw_id="${line#|}"
    raw_id="${raw_id%%|*}"
    raw_id="${raw_id// /}"
    [[ "$raw_id" == "$prefix"* ]] || continue
    suffix="${raw_id#"$prefix"}"
    [[ "$suffix" =~ ^[0-9]+$ ]] || continue
    (( suffix > max )) && max="$suffix"
  done < "$open_file"

  printf '%s%s' "$prefix" "$((max + 1))"
}

TO=""
FROM=""
CLI=""
SLUG=""
QUESTION=""
BODY=""
TITLE=""
THREAD=""
POINTERS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --to) TO="${2:-}"; shift 2 ;;
    --from) FROM="${2:-}"; shift 2 ;;
    --cli) CLI="${2:-}"; shift 2 ;;
    --slug) SLUG="${2:-}"; shift 2 ;;
    --question) QUESTION="${2:-}"; shift 2 ;;
    --body) BODY="${2:-}"; shift 2 ;;
    --pointer) POINTERS+=("${2:-}"); shift 2 ;;
    --thread) THREAD="${2:-}"; shift 2 ;;
    --title) TITLE="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "unknown arg: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -n "$TO" && -n "$FROM" && -n "$CLI" && -n "$QUESTION" ]] || {
  usage >&2
  exit 2
}
[[ -n "$SLUG" || -n "$THREAD" ]] || {
  echo "missing --slug or --thread" >&2
  exit 2
}
valid_to_role "$TO" || {
  echo "invalid --to role: $TO" >&2
  exit 2
}
valid_from_role "$FROM" || {
  echo "invalid --from role: $FROM" >&2
  exit 2
}

ROOT="${AGENT_COORD_ROOT:-}"
if [[ -z "$ROOT" ]]; then
  ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
fi
cd "$ROOT"

DISCUSS_DIR="docs/agents/discuss"
OPEN_FILE="$DISCUSS_DIR/OPEN.md"
[[ -d "$DISCUSS_DIR" ]] || {
  echo "missing $DISCUSS_DIR" >&2
  exit 1
}
[[ -f "$OPEN_FILE" ]] || {
  echo "missing $OPEN_FILE" >&2
  exit 1
}

LOCK_FILE="${AGENT_MAILBOX_LOCK:-/tmp/elfin_humble_agents_open.lock}"
if command -v flock >/dev/null 2>&1; then
  exec 9>"$LOCK_FILE"
  flock 9
else
  echo "warning: flock not found; updating mailbox without a process lock" >&2
fi

STAMP="$(date +%Y-%m-%d_%H%M)"
DATE="$(date +%Y-%m-%d)"
HUMAN_TIME="$(date '+%Y-%m-%d %H:%M')"
if [[ -z "$THREAD" ]]; then
  THREAD="${STAMP}_$(safe_slug "$SLUG").md"
fi
THREAD_PATH="$DISCUSS_DIR/$THREAD"
BODY="${BODY:-$QUESTION}"
TITLE="${TITLE:-$SLUG}"

if [[ "$THREAD" == */* || "$THREAD" != *.md ]]; then
  echo "--thread must be a filename under $DISCUSS_DIR ending in .md" >&2
  exit 2
fi

if [[ ! -f "$THREAD_PATH" ]]; then
  {
    printf '# %s -- %s\n\n' "$DATE" "$TITLE"
    printf -- '- status: open\n'
    printf -- '- to: %s\n\n' "$TO"
  } > "$THREAD_PATH"
fi

{
  printf '## Post -- %s -- %s -- %s\n\n' "$FROM" "$HUMAN_TIME" "$CLI"
  printf '%s\n\n' "$BODY"
  if [[ ${#POINTERS[@]} -gt 0 ]]; then
    printf '## Pointers\n\n'
    for pointer in "${POINTERS[@]}"; do
      printf -- '- `%s`\n' "$pointer"
    done
    printf '\n'
  fi
  printf '## Open\n\n'
  printf -- '- %s\n\n' "$QUESTION"
} >> "$THREAD_PATH"

if grep -F "| $THREAD |" "$OPEN_FILE" >/dev/null 2>&1; then
  echo "thread already listed in $OPEN_FILE: $THREAD" >&2
  exit 1
fi

ID="$(next_question_id "$OPEN_FILE")"
printf '| %s | %s | %s | %s | %s | %s |\n' \
  "$ID" "$TO" "$FROM" "$(trim_cell "$CLI")" "$THREAD" "$(trim_cell "$QUESTION")" >> "$OPEN_FILE"

echo "notified $TO as $ID via $THREAD"
