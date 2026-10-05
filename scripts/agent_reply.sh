#!/usr/bin/env bash
# Append a Reply section to a discuss thread under the mailbox lock.
#
# A Reply is an append-only event: it must never edit thread metadata (the
# scheduler treats a runnable thread's header as the source of truth) and it
# must never create or update an OPEN.md row. This helper productizes the
# manual flock + append workaround.
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/agent_reply.sh --thread FILE --from ROLE --cli CLI [options]

Options:
  --from ROLE         Sender role: reviews, eng, test, or discuss.
  --from-agent AGENT  Sender agent id. Defaults to --cli.
  --from-model MODEL  Sender model id. Defaults to unknown.
  --agent AGENT       Alias for --from-agent.
  --model MODEL       Alias for --from-model.
  --cli CLI           Sending CLI.
  --thread FILE       Discuss thread filename under docs/agents/discuss.
  --body TEXT         Reply body. Defaults to --request.
  --request TEXT      Single-line reply text. Alias for --body.
  --pointer PATH      Add one pointer. May be repeated.
  -h, --help          Show this help.

Appends a `## Reply -- ...` section under the mailbox lock. Never edits thread
metadata and never touches OPEN.md, so it is safe on runnable threads: Claim
events, scope, and dependency metadata stay intact.

Set AGENT_COORD_ROOT to write the mailbox in the primary workspace when
running from a satellite git worktree.
USAGE
}

trim() {
  local value="$*"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "$value"
}

valid_from_role() {
  [[ "$1" =~ ^(reviews|eng|test|discuss)$ ]]
}

valid_agent_id() {
  [[ "$1" =~ ^[[:alnum:]_.@/-]+$ ]]
}

normalize_role() {
  case "$1" in
    reviewers) printf 'reviews' ;;
    *) printf '%s' "$1" ;;
  esac
}

FROM=""
FROM_AGENT=""
FROM_MODEL="unknown"
CLI=""
THREAD=""
BODY=""
REQUEST=""
POINTERS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --from) FROM="${2:-}"; shift 2 ;;
    --from-agent|--agent) FROM_AGENT="${2:-}"; shift 2 ;;
    --from-model|--model) FROM_MODEL="${2:-}"; shift 2 ;;
    --cli) CLI="${2:-}"; shift 2 ;;
    --thread) THREAD="${2:-}"; shift 2 ;;
    --body) BODY="${2:-}"; shift 2 ;;
    --request) REQUEST="${2:-}"; shift 2 ;;
    --pointer) POINTERS+=("${2:-}"); shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "unknown arg: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -n "$FROM" && -n "$CLI" && -n "$THREAD" && ( -n "$BODY" || -n "$REQUEST" ) ]] || {
  usage >&2
  exit 2
}
FROM="$(normalize_role "$FROM")"
valid_from_role "$FROM" || {
  echo "invalid --from role: $FROM" >&2
  exit 2
}
FROM_AGENT="${FROM_AGENT:-$CLI}"
valid_agent_id "$FROM_AGENT" || {
  echo "invalid --from-agent: $FROM_AGENT" >&2
  exit 2
}
valid_agent_id "$FROM_MODEL" || {
  echo "invalid --from-model: $FROM_MODEL" >&2
  exit 2
}
BODY="${BODY:-$REQUEST}"

ROOT="${AGENT_COORD_ROOT:-}"
if [[ -z "$ROOT" ]]; then
  ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
else
  # Same guard as agent_notify.sh: a leftover exported AGENT_COORD_ROOT must
  # not silently retarget the mailbox.
  echo "AGENT_COORD_ROOT is set: mailbox root -> $ROOT" >&2
  if [[ ! -f "$ROOT/docs/agents/README.md" || ! -d "$ROOT/src/luggage_gazebo" ]] \
      && [[ "${AGENT_MAILBOX_ALLOW_ANY_ROOT:-0}" != "1" ]]; then
    echo "refusing: $ROOT does not look like the primary workspace" \
         "(missing docs/agents/README.md or src/luggage_gazebo)." \
         "Unset AGENT_COORD_ROOT or set AGENT_MAILBOX_ALLOW_ANY_ROOT=1" \
         "for a deliberate sandbox." >&2
    exit 1
  fi
fi
cd "$ROOT"

DISCUSS_DIR="docs/agents/discuss"
THREAD_PATH="$DISCUSS_DIR/$THREAD"
[[ -f "$THREAD_PATH" ]] || {
  echo "missing thread: $THREAD_PATH" >&2
  exit 1
}

LOCK_FILE="${AGENT_MAILBOX_LOCK:-/tmp/elfin_humble_agents_open.lock}"
if command -v flock >/dev/null 2>&1; then
  exec 9>"$LOCK_FILE"
  flock 9
else
  echo "warning: flock not found; appending reply without a process lock" >&2
fi

HUMAN_TIME="$(date '+%Y-%m-%d %H:%M')"

{
  printf '## Reply -- %s/%s -- %s -- %s/%s\n\n' \
    "$FROM" "$FROM_AGENT" "$HUMAN_TIME" "$CLI" "$FROM_MODEL"
  printf '%s\n\n' "$BODY"
  if [[ ${#POINTERS[@]} -gt 0 ]]; then
    printf '## Pointers\n\n'
    for pointer in "${POINTERS[@]}"; do
      printf -- '- `%s`\n' "$pointer"
    done
    printf '\n'
  fi
} >> "$THREAD_PATH"

echo "replied to $THREAD as $FROM/$FROM_AGENT via $CLI/$FROM_MODEL"
