#!/usr/bin/env bash
# Create a discuss thread entry and mailbox row for cross-agent notification.
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/agent_notify.sh --to ROLE --from ROLE --cli CLI --slug SLUG --request TEXT [options]

Options:
  --kind KIND          question, consensus, subtask, integration, or regression.
                       Defaults to question.
  --parent TASK        Unified parent task id. Defaults to n/a.
  --subtask ID         Executable subtask id. Defaults to n/a.
  --depends-on IDS     Comma-separated prerequisite subtask/mailbox ids, or none.
                       Defaults to none.
  --base-revision REV  Commit, branch, tag, or isolated worktree revision.
                       Defaults to n/a.
  --generation N       Runnable task generation. Required for new runnable work.
  --plan-revision REV  Exact approved plan commit for new runnable work.
  --dispatch-ready yes|no
                       Runnable dispatch gate. Defaults to no for runnable work.
  --checkpoint ID      Deprecated alias for --subtask.
  --revision REV       Deprecated alias for --base-revision.
  --to-agent AGENT    Concrete target agent. Defaults to any.
  --to-model MODEL    Concrete target model. Defaults to any.
  --from-agent AGENT  Concrete sender agent. Defaults to --cli.
  --from-model MODEL  Concrete sender model. Defaults to unknown.
  --agent AGENT       Alias for --from-agent.
  --model MODEL       Alias for --from-model.
  --request TEXT     One-line requested action.
  --question TEXT    Backward-compatible alias for --request.
  --body TEXT       Thread post body. Defaults to --request.
  --pointer PATH    Add one pointer. May be repeated.
  --thread FILE     Append to an existing discuss thread filename. The thread's
                    kind must match --kind; a runnable thread additionally
                    refuses any in-place change to protected metadata. Never
                    send a question on a runnable thread -- use a separate
                    thread or scripts/agent_reply.sh.
  --title TEXT      Thread title for a new thread.
  -h, --help        Show this help.

Roles:
  reviews, eng, test, discuss, any
  reviewers is accepted as an alias for reviews.

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

trim() {
  local value="$*"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "$value"
}

valid_to_role() {
  [[ "$1" =~ ^(reviews|eng|test|discuss|any)$ ]]
}

valid_from_role() {
  [[ "$1" =~ ^(reviews|eng|test|discuss)$ ]]
}

valid_agent_id() {
  [[ "$1" =~ ^[[:alnum:]_.@/-]+$ ]]
}

valid_model_id() {
  [[ "$1" =~ ^[[:alnum:]_.@/-]+$ ]]
}

valid_kind() {
  [[ "$1" =~ ^(question|consensus|subtask|integration|regression|task|checkpoint)$ ]]
}

valid_route_token() {
  [[ "$1" =~ ^[[:alnum:]_.@:/-]+$ ]]
}

valid_dependency_list() {
  [[ "$1" =~ ^[[:alnum:]_.@:/,-]+$ ]]
}

normalize_role() {
  case "$1" in
    reviewers) printf 'reviews' ;;
    *) printf '%s' "$1" ;;
  esac
}

safe_slug() {
  printf '%s' "$1" | tr '[:upper:]' '[:lower:]' | tr -cs '[:alnum:]_-' '-'
}

thread_meta_value() {
  local file="$1" key="$2"
  awk -v key="$key" '
    /^## / { exit }
    index($0, "- " key ": ") == 1 {
      print substr($0, length("- " key ": ") + 1)
      exit
    }
  ' "$file"
}

max_id_suffix() {
  local prefix="$1" raw_id suffix max=0
  while IFS= read -r raw_id; do
    [[ "$raw_id" == "$prefix"* ]] || continue
    suffix="${raw_id#"$prefix"}"
    [[ "$suffix" =~ ^[0-9]+$ ]] || continue
    (( suffix > max )) && max="$suffix"
  done
  printf '%s' "$max"
}

# Ids must be unique for the whole mailbox history, not just the live rows:
# completed rows are removed from OPEN.md, so the allocator also consults the
# append-only ledger. Both reads happen under the mailbox lock.
next_question_id() {
  local open_file="$1" ledger_file="$2"
  local today prefix max line raw_id

  today="$(date +%Y%m%d)"
  prefix="Q-$today-"

  max="$(
    {
      while IFS= read -r line; do
        [[ "$line" == \|* ]] || continue
        [[ "$line" == "| id "* || "$line" == "|---"* ]] && continue
        raw_id="${line#|}"
        raw_id="${raw_id%%|*}"
        raw_id="${raw_id// /}"
        printf '%s\n' "$raw_id"
      done < "$open_file"
      if [[ -f "$ledger_file" ]]; then
        grep -oE 'Q-[0-9]{8}-[0-9]+' "$ledger_file" || true
      fi
    } | max_id_suffix "$prefix"
  )"

  printf '%s%s' "$prefix" "$((max + 1))"
}

open_header() {
  printf '| id | kind | parent | subtask | depends_on | revision | to_role | to_agent | to_model | from_role | from_agent | from_model | cli | thread | request | generation | plan_revision |\n'
}

open_separator() {
  printf '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n'
}

parse_first_cell() {
  local line="$1"
  line="${line#|}"
  line="${line%%|*}"
  trim "$line"
}

upsert_open_row() {
  local open_file="$1"
  local id="$2"
  local row="$3"
  local thread="$4"
  local tmp line existing_id found updated_row

  tmp="$(mktemp)"
  found=0
  while IFS= read -r line; do
    if [[ "$line" == "| id "* ]]; then
      open_header >> "$tmp"
    elif [[ "$line" == "|---"* ]]; then
      open_separator >> "$tmp"
    elif [[ "$line" == \|* && "$line" == *"| $thread |"* ]]; then
      existing_id="$(parse_first_cell "$line")"
      updated_row="| $existing_id |${row#| $id |}"
      printf '%s\n' "$updated_row" >> "$tmp"
      found=1
    else
      printf '%s\n' "$line" >> "$tmp"
    fi
  done < "$open_file"

  if [[ "$found" -eq 0 ]]; then
    printf '%s\n' "$row" >> "$tmp"
  fi

  mv "$tmp" "$open_file"
  [[ "$found" -eq 0 ]]
}

set_thread_target() {
  local file="$1"
  local to_role="$2"
  local to_agent="$3"
  local to_model="$4"
  local kind="$5"
  local parent="$6"
  local subtask="$7"
  local depends_on="$8"
  local revision="$9"
  local generation="${10}"
  local plan_revision="${11}"
  local dispatch_ready="${12}"
  local tmp line state saw_role saw_agent saw_model saw_kind saw_parent
  local saw_subtask saw_depends_on saw_revision saw_consensus saw_generation
  local saw_plan_revision saw_dispatch_ready

  tmp="$(mktemp)"
  state="preamble"
  saw_role=0
  saw_agent=0
  saw_model=0
  saw_kind=0
  saw_parent=0
  saw_subtask=0
  saw_depends_on=0
  saw_revision=0
  saw_consensus=0
  saw_generation=0
  saw_plan_revision=0
  saw_dispatch_ready=0

  while IFS= read -r line; do
    if [[ "$state" == "preamble" && "$line" != "- "* ]]; then
      printf '%s\n' "$line" >> "$tmp"
      continue
    fi

    if [[ "$state" == "preamble" ]]; then
      state="meta"
    fi

    if [[ "$state" == "meta" && "$line" == "- to_role:"* ]]; then
      printf -- '- to_role: %s\n' "$to_role" >> "$tmp"
      saw_role=1
    elif [[ "$state" == "meta" && "$line" == "- to_agent:"* ]]; then
      printf -- '- to_agent: %s\n' "$to_agent" >> "$tmp"
      saw_agent=1
    elif [[ "$state" == "meta" && "$line" == "- to_model:"* ]]; then
      printf -- '- to_model: %s\n' "$to_model" >> "$tmp"
      saw_model=1
    elif [[ "$state" == "meta" && "$line" == "- kind:"* ]]; then
      printf -- '- kind: %s\n' "$kind" >> "$tmp"
      saw_kind=1
    elif [[ "$state" == "meta" && "$line" == "- parent:"* ]]; then
      printf -- '- parent: %s\n' "$parent" >> "$tmp"
      saw_parent=1
    elif [[ "$state" == "meta" && "$line" == "- subtask:"* ]]; then
      printf -- '- subtask: %s\n' "$subtask" >> "$tmp"
      saw_subtask=1
    elif [[ "$state" == "meta" && "$line" == "- depends_on:"* ]]; then
      printf -- '- depends_on: %s\n' "$depends_on" >> "$tmp"
      saw_depends_on=1
    elif [[ "$state" == "meta" && "$line" == "- checkpoint:"* ]]; then
      continue
    elif [[ "$state" == "meta" && "$line" == "- revision:"* ]]; then
      printf -- '- revision: %s\n' "$revision" >> "$tmp"
      saw_revision=1
    elif [[ "$state" == "meta" && "$line" == "- generation:"* ]]; then
      [[ -n "$generation" ]] && printf -- '- generation: %s\n' "$generation" >> "$tmp"
      saw_generation=1
    elif [[ "$state" == "meta" && "$line" == "- plan_revision:"* ]]; then
      [[ -n "$plan_revision" ]] && printf -- '- plan_revision: %s\n' "$plan_revision" >> "$tmp"
      saw_plan_revision=1
    elif [[ "$state" == "meta" && "$line" == "- dispatch_ready:"* ]]; then
      [[ -n "$dispatch_ready" ]] && printf -- '- dispatch_ready: %s\n' "$dispatch_ready" >> "$tmp"
      saw_dispatch_ready=1
    elif [[ "$state" == "meta" && "$line" == "- consensus:"* ]]; then
      printf '%s\n' "$line" >> "$tmp"
      saw_consensus=1
    elif [[ "$state" == "meta" && "$line" == "- to:"* ]]; then
      continue
    elif [[ "$state" == "meta" && -z "$line" ]]; then
      [[ "$saw_role" -eq 1 ]] || printf -- '- to_role: %s\n' "$to_role" >> "$tmp"
      [[ "$saw_agent" -eq 1 ]] || printf -- '- to_agent: %s\n' "$to_agent" >> "$tmp"
      [[ "$saw_model" -eq 1 ]] || printf -- '- to_model: %s\n' "$to_model" >> "$tmp"
      [[ "$saw_kind" -eq 1 ]] || printf -- '- kind: %s\n' "$kind" >> "$tmp"
      [[ "$saw_parent" -eq 1 ]] || printf -- '- parent: %s\n' "$parent" >> "$tmp"
      [[ "$saw_subtask" -eq 1 ]] || printf -- '- subtask: %s\n' "$subtask" >> "$tmp"
      [[ "$saw_depends_on" -eq 1 ]] || printf -- '- depends_on: %s\n' "$depends_on" >> "$tmp"
      [[ "$saw_revision" -eq 1 ]] || printf -- '- revision: %s\n' "$revision" >> "$tmp"
      if [[ "$kind" =~ ^(subtask|integration|regression)$ ]]; then
        [[ "$saw_generation" -eq 1 ]] || printf -- '- generation: %s\n' "$generation" >> "$tmp"
        [[ "$saw_plan_revision" -eq 1 ]] || printf -- '- plan_revision: %s\n' "$plan_revision" >> "$tmp"
        [[ "$saw_dispatch_ready" -eq 1 ]] || printf -- '- dispatch_ready: %s\n' "$dispatch_ready" >> "$tmp"
      fi
      if [[ "$kind" == "consensus" && "$saw_consensus" -eq 0 ]]; then
        printf -- '- consensus: open\n' >> "$tmp"
      fi
      printf '\n' >> "$tmp"
      state="body"
    else
      printf '%s\n' "$line" >> "$tmp"
    fi
  done < "$file"

  mv "$tmp" "$file"
}

# Reusing --thread on runnable work must never rewrite the metadata the
# scheduler treats as the source of truth. Every field set_thread_target would
# write is compared against the live thread header; only the reviews dispatch
# flip (dispatch_ready no -> yes) may differ.
guard_thread_metadata() {
  local file="$1"
  local pairs=("$TO" "$TO_AGENT" "$TO_MODEL" "$KIND" "$PARENT" "$SUBTASK"
               "$DEPENDS_ON" "$REVISION" "$GENERATION" "$PLAN_REVISION"
               "$DISPATCH_READY")
  local keys=(to_role to_agent to_model kind parent subtask depends_on revision
              generation plan_revision dispatch_ready)
  local changes="" index key value current

  for index in "${!keys[@]}"; do
    key="${keys[$index]}"
    value="${pairs[$index]}"
    current="$(thread_meta_value "$file" "$key")"
    [[ "$current" == "$value" ]] && continue
    if [[ "$key" == "dispatch_ready" && "$value" == "yes" && "$FROM" == "reviews" ]] \
        && [[ -z "$current" || "$current" == "no" ]]; then
      continue
    fi
    changes+="$key '$current' -> '$value'; "
  done

  if [[ -n "$changes" ]]; then
    echo "refusing: reusing --thread would change protected metadata of $THREAD" >&2
    echo "changed: $changes" >&2
    echo "owner/scope/dependencies/revisions must not change in place; dispatch a" >&2
    echo "higher-generation thread instead, or append a Reply via scripts/agent_reply.sh" >&2
    exit 1
  fi
}

# Cross-field dispatch rules. Runs after the existing-thread defaulting step so
# that reusing a runnable thread can inherit generation/plan_revision/
# dispatch_ready from its own metadata instead of being forced to restate them.
validate_cross_fields() {
  if [[ "$KIND" =~ ^(subtask|integration|regression)$ && \
        ( "$PARENT" == "n/a" || "$SUBTASK" == "n/a" || "$REVISION" == "n/a" ) ]]; then
    echo "--kind $KIND requires --parent, --subtask, and --base-revision" >&2
    exit 2
  fi
  if [[ "$KIND" =~ ^(subtask|integration|regression)$ && \
        ( -z "$GENERATION" || -z "$PLAN_REVISION" ) ]]; then
    echo "--kind $KIND requires --generation and --plan-revision" >&2
    exit 2
  fi
  if [[ "$KIND" =~ ^(subtask|integration|regression)$ && -z "$DISPATCH_READY" ]]; then
    DISPATCH_READY="no"
  fi
  if [[ "$KIND" =~ ^(subtask|integration|regression)$ && \
        ( "$TO_AGENT" == "any" || "$TO_MODEL" == "any" ) ]]; then
    echo "--kind $KIND requires concrete --to-agent and --to-model values" >&2
    exit 2
  fi
  if [[ "$KIND" == "consensus" && \
        ( "$PARENT" == "n/a" || "$REVISION" == "n/a" ) ]]; then
    echo "--kind consensus requires --parent and --base-revision" >&2
    exit 2
  fi
  if [[ "$KIND" == "consensus" && \
        ( "$TO_AGENT" != codex* || "$TO_AGENT" == "$FROM_AGENT" ) ]]; then
    echo "--kind consensus requires a distinct Codex --to-agent" >&2
    exit 2
  fi
  if [[ "$KIND" == "consensus" && \
        ( "$FROM" != "reviews" || "$TO" != "reviews" ) ]]; then
    echo "--kind consensus must be a reviews-to-reviews Codex consultation" >&2
    exit 2
  fi
  if [[ "$KIND" =~ ^(subtask|integration)$ && "$FROM" != "reviews" ]]; then
    echo "--kind $KIND must be dispatched by reviews" >&2
    exit 2
  fi
  if [[ "$KIND" =~ ^(task|checkpoint)$ && \
        ( "$SUBTASK" == "n/a" || "$REVISION" == "n/a" ) ]]; then
    echo "legacy --kind $KIND requires --subtask/--checkpoint and --base-revision/--revision" >&2
    exit 2
  fi
}

TO=""
TO_AGENT="any"
TO_MODEL="any"
KIND="question"
KIND_SET=0
PARENT="n/a"
SUBTASK="n/a"
DEPENDS_ON="none"
REVISION="n/a"
GENERATION=""
PLAN_REVISION=""
DISPATCH_READY=""
DISPATCH_READY_SET=0
FROM=""
FROM_AGENT=""
FROM_MODEL="unknown"
CLI=""
SLUG=""
QUESTION=""
BODY=""
TITLE=""
THREAD=""
POINTERS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --kind) KIND="${2:-}"; KIND_SET=1; shift 2 ;;
    --parent) PARENT="${2:-}"; shift 2 ;;
    --subtask|--checkpoint) SUBTASK="${2:-}"; shift 2 ;;
    --depends-on) DEPENDS_ON="${2:-}"; shift 2 ;;
    --base-revision|--revision) REVISION="${2:-}"; shift 2 ;;
    --generation) GENERATION="${2:-}"; shift 2 ;;
    --plan-revision) PLAN_REVISION="${2:-}"; shift 2 ;;
    --dispatch-ready) DISPATCH_READY="${2:-}"; DISPATCH_READY_SET=1; shift 2 ;;
    --to) TO="${2:-}"; shift 2 ;;
    --to-agent) TO_AGENT="${2:-}"; shift 2 ;;
    --to-model) TO_MODEL="${2:-}"; shift 2 ;;
    --from) FROM="${2:-}"; shift 2 ;;
    --from-agent|--agent) FROM_AGENT="${2:-}"; shift 2 ;;
    --from-model|--model) FROM_MODEL="${2:-}"; shift 2 ;;
    --cli) CLI="${2:-}"; shift 2 ;;
    --slug) SLUG="${2:-}"; shift 2 ;;
    --request|--question) QUESTION="${2:-}"; shift 2 ;;
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
if [[ -n "$THREAD" && "$KIND_SET" -eq 0 ]]; then
  echo "reusing --thread requires explicit --kind" >&2
  exit 2
fi
TO="$(normalize_role "$TO")"
FROM="$(normalize_role "$FROM")"
valid_to_role "$TO" || {
  echo "invalid --to role: $TO" >&2
  exit 2
}
valid_from_role "$FROM" || {
  echo "invalid --from role: $FROM" >&2
  exit 2
}
valid_kind "$KIND" || {
  echo "invalid --kind: $KIND" >&2
  exit 2
}
valid_route_token "$PARENT" || {
  echo "invalid --parent: $PARENT" >&2
  exit 2
}
valid_route_token "$SUBTASK" || {
  echo "invalid --subtask: $SUBTASK" >&2
  exit 2
}
valid_dependency_list "$DEPENDS_ON" || {
  echo "invalid --depends-on: $DEPENDS_ON" >&2
  exit 2
}
valid_route_token "$REVISION" || {
  echo "invalid --base-revision: $REVISION" >&2
  exit 2
}
if [[ -n "$GENERATION" && ! "$GENERATION" =~ ^[1-9][0-9]*$ ]]; then
  echo "invalid --generation: $GENERATION" >&2
  exit 2
fi
if [[ -n "$PLAN_REVISION" ]]; then
  valid_route_token "$PLAN_REVISION" || {
    echo "invalid --plan-revision: $PLAN_REVISION" >&2
    exit 2
  }
fi
if [[ -n "$DISPATCH_READY" && ! "$DISPATCH_READY" =~ ^(yes|no)$ ]]; then
  echo "invalid --dispatch-ready: $DISPATCH_READY" >&2
  exit 2
fi
FROM_AGENT="${FROM_AGENT:-$CLI}"
valid_agent_id "$TO_AGENT" || {
  echo "invalid --to-agent: $TO_AGENT" >&2
  exit 2
}
valid_agent_id "$FROM_AGENT" || {
  echo "invalid --from-agent: $FROM_AGENT" >&2
  exit 2
}
valid_model_id "$TO_MODEL" || {
  echo "invalid --to-model: $TO_MODEL" >&2
  exit 2
}
valid_model_id "$FROM_MODEL" || {
  echo "invalid --from-model: $FROM_MODEL" >&2
  exit 2
}

ROOT="${AGENT_COORD_ROOT:-}"
if [[ -z "$ROOT" ]]; then
  ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
else
  # A leftover exported AGENT_COORD_ROOT (e.g. a sandbox path in /tmp) must
  # not silently retarget the mailbox. Require the coordination root to look
  # like this workspace; opt out explicitly for deliberate sandboxes.
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
OPEN_FILE="$DISCUSS_DIR/OPEN.md"
LEDGER_FILE="docs/agents/QID_LEDGER.md"
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

EXISTING_KIND=""
if [[ -f "$THREAD_PATH" ]]; then
  EXISTING_KIND="$(thread_meta_value "$THREAD_PATH" kind)"
  if [[ -z "$EXISTING_KIND" ]]; then
    echo "refusing: $THREAD has no kind metadata; create a new thread instead" >&2
    exit 1
  fi
  if [[ "$KIND" != "$EXISTING_KIND" ]]; then
    echo "refusing: --kind $KIND does not match existing thread kind $EXISTING_KIND in $THREAD" >&2
    echo "never reuse a runnable thread to send a question; create a separate" >&2
    echo "question thread or append a Reply via scripts/agent_reply.sh" >&2
    exit 1
  fi
  if [[ "$EXISTING_KIND" =~ ^(subtask|integration|regression)$ ]]; then
    # Reuse may inherit lifecycle fields from the thread metadata instead of
    # restating them; anything explicitly passed is still compared below so a
    # silent in-place change remains impossible.
    [[ -n "$GENERATION" ]] || GENERATION="$(thread_meta_value "$THREAD_PATH" generation)"
    [[ -n "$PLAN_REVISION" ]] || PLAN_REVISION="$(thread_meta_value "$THREAD_PATH" plan_revision)"
    if [[ "$DISPATCH_READY_SET" -eq 0 ]]; then
      DISPATCH_READY="$(thread_meta_value "$THREAD_PATH" dispatch_ready)"
    fi
  fi
fi

validate_cross_fields

if [[ -f "$THREAD_PATH" && "$EXISTING_KIND" =~ ^(subtask|integration|regression)$ ]]; then
  guard_thread_metadata "$THREAD_PATH"
fi

if [[ ! -f "$THREAD_PATH" ]]; then
  {
    printf '# %s -- %s\n\n' "$DATE" "$TITLE"
    printf -- '- status: open\n'
    printf -- '- to_role: %s\n' "$TO"
    printf -- '- to_agent: %s\n' "$TO_AGENT"
    printf -- '- to_model: %s\n' "$TO_MODEL"
    printf -- '- kind: %s\n' "$KIND"
    printf -- '- parent: %s\n' "$PARENT"
    printf -- '- subtask: %s\n' "$SUBTASK"
    printf -- '- depends_on: %s\n' "$DEPENDS_ON"
    printf -- '- revision: %s\n' "$REVISION"
    if [[ "$KIND" =~ ^(subtask|integration|regression)$ ]]; then
      printf -- '- generation: %s\n' "$GENERATION"
      printf -- '- plan_revision: %s\n' "$PLAN_REVISION"
      printf -- '- dispatch_ready: %s\n' "$DISPATCH_READY"
    fi
    if [[ "$KIND" == "consensus" ]]; then
      printf -- '- consensus: open\n'
    fi
    printf '\n'
  } > "$THREAD_PATH"
fi
set_thread_target "$THREAD_PATH" "$TO" "$TO_AGENT" "$TO_MODEL" \
  "$KIND" "$PARENT" "$SUBTASK" "$DEPENDS_ON" "$REVISION" \
  "$GENERATION" "$PLAN_REVISION" "$DISPATCH_READY"

{
  printf '## Post -- %s/%s -- %s -- %s/%s\n\n' \
    "$FROM" "$FROM_AGENT" "$HUMAN_TIME" "$CLI" "$FROM_MODEL"
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

# An existing row keeps its id (no allocation, no ledger entry); only genuinely
# new rows allocate, so the ledger records every id exactly once.
ROW_EXISTS=0
ID=""
while IFS= read -r row_line; do
  [[ "$row_line" == \|* ]] || continue
  [[ "$row_line" == "| id "* || "$row_line" == "|---"* ]] && continue
  if [[ "$row_line" == *"| $THREAD |"* ]]; then
    ROW_EXISTS=1
    ID="$(parse_first_cell "$row_line")"
    break
  fi
done < "$OPEN_FILE"

if [[ "$ROW_EXISTS" -eq 0 ]]; then
  ID="$(next_question_id "$OPEN_FILE" "$LEDGER_FILE")"
  if [[ ! -f "$LEDGER_FILE" ]]; then
    {
      printf '# Mailbox Q-id allocation ledger\n\n'
      printf 'Append-only record of every mailbox id allocated by agent_notify.sh.\n'
      printf 'The allocator takes max+1 over this ledger plus the live OPEN.md\n'
      printf 'rows, so ids of completed and removed rows are never reused.\n'
    } > "$LEDGER_FILE"
  fi
  {
    printf '\n## %s\n\n' "$(date +%Y-%m-%d)"
    printf -- '- %s\n' "$ID"
  } >> "$LEDGER_FILE"
fi

ROW="$(printf '| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |' \
  "$ID" "$KIND" "$(trim_cell "$PARENT")" "$(trim_cell "$SUBTASK")" \
  "$(trim_cell "$DEPENDS_ON")" "$(trim_cell "$REVISION")" "$TO" \
  "$TO_AGENT" "$TO_MODEL" "$FROM" "$FROM_AGENT" "$FROM_MODEL" \
  "$(trim_cell "$CLI")" "$THREAD" "$(trim_cell "$QUESTION")")"
if [[ "$KIND" =~ ^(subtask|integration|regression)$ ]]; then
  ROW="${ROW% |} | $(trim_cell "$GENERATION") | $(trim_cell "$PLAN_REVISION") |"
else
  ROW="${ROW% |} |  |  |"
fi

if upsert_open_row "$OPEN_FILE" "$ID" "$ROW" "$THREAD"; then
  echo "notified $TO/$TO_AGENT/$TO_MODEL for $KIND/$PARENT/$SUBTASK at $REVISION as $ID via $THREAD"
else
  echo "updated $TO/$TO_AGENT/$TO_MODEL for $KIND/$PARENT/$SUBTASK at $REVISION via $THREAD"
fi
