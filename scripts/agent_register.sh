#!/usr/bin/env bash
# Register or refresh a running CLI session for scheduler visibility.
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/agent_register.sh --id ID --role ROLE --agent AGENT --model MODEL \
    --cli CLI [options]

Options:
  --session SESSION        CLI session id/name. Defaults to n/a.
  --worktree PATH         Workspace path. Defaults to current directory.
  --state idle|busy|down  Defaults to idle.
  --capabilities LIST     Comma-separated capabilities. Defaults to file.
                          Use queue only when a supported live adapter exists.
  --heartbeat ISO         Defaults to current time.
  --registry FILE         Defaults to docs/agents/RUNTIME.md.
  -h, --help              Show this help.
USAGE
}

trim_cell() {
  local value="$*"
  value="${value//$'\n'/ }"
  value="${value//|/\\|}"
  printf '%s' "$value"
}

trim() {
  local value="$*"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "$value"
}

parse_first_cell() {
  local line="$1"
  line="${line#|}"
  line="${line%%|*}"
  trim "$line"
}

header() {
  printf '| id | role | agent | model | cli | session | worktree | state | capabilities | heartbeat |\n'
}

separator() {
  printf '|---|---|---|---|---|---|---|---|---|---|\n'
}

ID=""
ROLE=""
AGENT=""
MODEL=""
CLI=""
SESSION="n/a"
WORKTREE=""
STATE="idle"
CAPABILITIES="file"
HEARTBEAT=""
REGISTRY="docs/agents/RUNTIME.md"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --id) ID="${2:-}"; shift 2 ;;
    --role) ROLE="${2:-}"; shift 2 ;;
    --agent) AGENT="${2:-}"; shift 2 ;;
    --model) MODEL="${2:-}"; shift 2 ;;
    --cli) CLI="${2:-}"; shift 2 ;;
    --session) SESSION="${2:-}"; shift 2 ;;
    --worktree) WORKTREE="${2:-}"; shift 2 ;;
    --state) STATE="${2:-}"; shift 2 ;;
    --capabilities) CAPABILITIES="${2:-}"; shift 2 ;;
    --heartbeat) HEARTBEAT="${2:-}"; shift 2 ;;
    --registry) REGISTRY="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown arg: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "$ID" && -n "$ROLE" && -n "$AGENT" && -n "$MODEL" && -n "$CLI" ]] || {
  usage >&2
  exit 2
}
[[ "$ROLE" =~ ^(reviews|eng|test|discuss|any)$ ]] || {
  echo "invalid role: $ROLE" >&2
  exit 2
}
[[ "$STATE" =~ ^(idle|busy|down)$ ]] || {
  echo "invalid state: $STATE" >&2
  exit 2
}
[[ "$ID" =~ ^[[:alnum:]_.@/-]+$ ]] || {
  echo "invalid id: $ID" >&2
  exit 2
}

ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
cd "$ROOT"
WORKTREE="${WORKTREE:-$ROOT}"
HEARTBEAT="${HEARTBEAT:-$(date --iso-8601=seconds)}"

mkdir -p "$(dirname "$REGISTRY")"
if [[ ! -f "$REGISTRY" ]]; then
  {
    printf '# Agent runtime registry\n\n'
    header
    separator
  } > "$REGISTRY"
fi

LOCK_FILE="${AGENT_RUNTIME_LOCK:-/tmp/elfin_humble_agents_runtime.lock}"
if command -v flock >/dev/null 2>&1; then
  exec 9>"$LOCK_FILE"
  flock 9
fi

ROW="$(printf '| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |' \
  "$(trim_cell "$ID")" "$(trim_cell "$ROLE")" "$(trim_cell "$AGENT")" \
  "$(trim_cell "$MODEL")" "$(trim_cell "$CLI")" "$(trim_cell "$SESSION")" \
  "$(trim_cell "$WORKTREE")" "$(trim_cell "$STATE")" \
  "$(trim_cell "$CAPABILITIES")" "$(trim_cell "$HEARTBEAT")")"

tmp="$(mktemp)"
found=0
seen_header=0
seen_separator=0
declare -a prefix=()
declare -a rows=()
declare -a suffix=()
while IFS= read -r line; do
  if [[ "$line" == "$(header)" ]]; then
    seen_header=1
  elif [[ "$seen_header" -eq 1 && "$line" == "$(separator)" ]]; then
    seen_separator=1
  elif [[ "$seen_separator" -eq 1 && "$line" == \|* && \
          "$(parse_first_cell "$line")" =~ ^[[:alnum:]_.@/-]+$ ]]; then
    if [[ "$(parse_first_cell "$line")" == "$ID" ]]; then
      rows+=("$ROW")
      found=1
    else
      rows+=("$line")
    fi
  elif [[ "$seen_header" -eq 0 ]]; then
    prefix+=("$line")
  else
    suffix+=("$line")
  fi
done < "$REGISTRY"

if [[ "$found" -eq 0 ]]; then
  rows+=("$ROW")
fi

printf '%s\n' "${prefix[@]}" >> "$tmp"
header >> "$tmp"
separator >> "$tmp"
printf '%s\n' "${rows[@]}" >> "$tmp"
printf '%s\n' "${suffix[@]}" >> "$tmp"

mv "$tmp" "$REGISTRY"
echo "registered $ID as $ROLE/$AGENT/$MODEL via $CLI ($STATE, $CAPABILITIES)"
