#!/usr/bin/env bash
# Concurrency and metadata-safety tests for mailbox notifications and replies:
# N parallel questions/replies must produce zero id reuse and zero thread
# frontmatter corruption, and question-kind notifications must never rewrite a
# runnable thread's metadata.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
TMP_ROOT="$(mktemp -d /tmp/agent-mailbox-concurrency.XXXXXX)"
trap 'rm -rf "$TMP_ROOT"' EXIT

mkdir -p "$TMP_ROOT/docs/agents/discuss" "$TMP_ROOT/docs/agents" "$TMP_ROOT/src/luggage_gazebo"
cd "$TMP_ROOT"
git init -q
git config user.email concurrency@example.invalid
git config user.name "Agent Concurrency"
touch docs/agents/README.md src/luggage_gazebo/.keep
git add docs/agents/README.md src/luggage_gazebo/.keep
git commit -q -m baseline
REVISION="$(git rev-parse HEAD)"
TODAY="$(date +%Y%m%d)"

cat > docs/agents/discuss/OPEN.md <<EOF
# Open cross-agent work

| id | kind | parent | subtask | depends_on | revision | to_role | to_agent | to_model | from_role | from_agent | from_model | cli | thread | request | generation | plan_revision |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Q-$TODAY-2 | question | n/a | n/a | none | n/a | reviews | codex | gpt-5 | eng | claude | glm-5.3 | claude | seed-question.md | seed live row |  |  |
EOF

cat > docs/agents/discuss/seed-question.md <<EOF
# $TODAY -- Seed question

- status: open
- to_role: reviews
- to_agent: codex
- to_model: gpt-5
- kind: question
- parent: n/a
- subtask: n/a
- depends_on: none
- revision: n/a

## Post -- eng/claude -- 2026-10-01 00:00 -- claude/glm-5.3

seed
EOF

# Q-$TODAY-5 was completed and its row removed; only the ledger remembers it.
cat > docs/agents/QID_LEDGER.md <<EOF
# Mailbox Q-id allocation ledger

## $(date +%Y-%m-%d)

- Q-$TODAY-5
EOF

notify() {
  AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$TMP_ROOT" \
    AGENT_MAILBOX_LOCK="$TMP_ROOT/mailbox.lock" \
    "$ROOT/scripts/agent_notify.sh" "$@"
}

reply() {
  AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$TMP_ROOT" \
    AGENT_MAILBOX_LOCK="$TMP_ROOT/mailbox.lock" \
    "$ROOT/scripts/agent_reply.sh" "$@"
}

metadata_block() {
  awk '/^## /{exit} {print}' "$1"
}

data_row_count() {
  grep -c '^| Q-' docs/agents/discuss/OPEN.md
}

row_ids() {
  grep '^| Q-' docs/agents/discuss/OPEN.md | while IFS= read -r line; do
    line="${line#|}"
    printf '%s\n' "${line%%|*}"
  done
}

assert_eq() {
  [[ "$1" == "$2" ]] || {
    echo "FAIL: $3: '$1' != '$2'" >&2
    exit 1
  }
}

# 1. The allocator must skip ids that only exist in the ledger.
notify --to reviews --from eng --from-agent claude --from-model glm-5.3 \
  --cli claude --slug reuse-check --request "must allocate above ledger max" \
  >/tmp/conc-reuse.log 2>&1 || {
    echo "FAIL: reuse-check notify failed" >&2
    cat /tmp/conc-reuse.log >&2
    exit 1
  }
grep -q "as Q-$TODAY-6 via" /tmp/conc-reuse.log || {
  echo "FAIL: allocator reused a ledger-only id (wanted Q-$TODAY-6)" >&2
  cat /tmp/conc-reuse.log >&2
  exit 1
}
grep -q -- "- Q-$TODAY-6" docs/agents/QID_LEDGER.md

# 2. Parallel new questions: unique ids, intact table, intact thread headers.
pids=()
for i in 1 2 3 4 5 6 7 8; do
  (
    notify --to reviews --from eng --from-agent claude --from-model glm-5.3 \
      --cli claude --slug "par-$i" --request "parallel question $i"
  ) >"/tmp/conc-par-$i.log" 2>&1 &
  pids+=("$!")
done
for pid in "${pids[@]}"; do
  wait "$pid" || {
    echo "FAIL: parallel notify pid $pid failed" >&2
    exit 1
  }
done
assert_eq "$(data_row_count)" "10" "row count after parallel questions"
duplicates="$(row_ids | sort | uniq -d)"
assert_eq "$duplicates" "" "duplicate row ids"
assert_eq "$(grep -oE 'Q-[0-9]{8}-[0-9]+' docs/agents/QID_LEDGER.md | sort -u | wc -l)" "10" \
  "ledger unique id count"
assert_eq "$(grep -c '^| id |' docs/agents/discuss/OPEN.md)" "1" "header row count"
for i in 1 2 3 4 5 6 7 8; do
  thread="$(ls docs/agents/discuss/*par-$i.md)"
  grep -q '^- status: open$' "$thread"
  grep -q '^- kind: question$' "$thread"
  grep -q '^## Post -- eng/claude' "$thread"
done

# 3. Runnable thread protection.
notify --kind subtask --parent CONC --subtask R-1 --depends-on none \
  --base-revision "$REVISION" --generation 1 --plan-revision "$REVISION" \
  --dispatch-ready no --to eng --to-agent claude --to-model glm-5.3 \
  --from reviews --from-agent codex-reviews-main --from-model gpt-5 --cli codex \
  --slug conc-runnable --request "runnable thread under test" \
  >/tmp/conc-runnable.log 2>&1 || {
    echo "FAIL: runnable dispatch failed" >&2
    cat /tmp/conc-runnable.log >&2
    exit 1
  }
RUNNABLE="$(basename "$(ls docs/agents/discuss/*conc-runnable.md)")"
RUNNABLE_PATH="docs/agents/discuss/$RUNNABLE"

# 3a. A question must never ride a runnable thread.
before="$(sha256sum "$RUNNABLE_PATH" | cut -d' ' -f1)"
if notify --kind question --thread "$RUNNABLE" --to reviews --from eng \
    --from-agent claude --from-model glm-5.3 --cli claude --slug x \
    --request "question on runnable" >/tmp/conc-q-on-r.log 2>&1; then
  echo "FAIL: question accepted on a runnable thread" >&2
  exit 1
fi
grep -q 'does not match existing thread kind' /tmp/conc-q-on-r.log
assert_eq "$(sha256sum "$RUNNABLE_PATH" | cut -d' ' -f1)" "$before" \
  "runnable thread changed by refused question"

# 3b. Same-kind reuse may not change protected scope metadata.
if notify --kind subtask --thread "$RUNNABLE" --parent CONC --subtask R-2 \
    --depends-on none --base-revision "$REVISION" --generation 1 \
    --plan-revision "$REVISION" --to eng --to-agent claude --to-model glm-5.3 \
    --from reviews --from-agent codex-reviews-main --from-model gpt-5 --cli codex \
    --slug x --request "scope drift" >/tmp/conc-scope.log 2>&1; then
  echo "FAIL: in-place scope change accepted" >&2
  exit 1
fi
grep -q 'would change protected metadata' /tmp/conc-scope.log
assert_eq "$(sha256sum "$RUNNABLE_PATH" | cut -d' ' -f1)" "$before" \
  "runnable thread changed by refused scope edit"

# 3c. Same-kind context post without restating lifecycle fields inherits them.
notify --kind subtask --thread "$RUNNABLE" --parent CONC --subtask R-1 \
  --depends-on none --base-revision "$REVISION" --to eng --to-agent claude \
  --to-model glm-5.3 --from reviews --from-agent codex-reviews-main \
  --from-model gpt-5 --cli codex --slug x \
  --request "context without restating generation" >/tmp/conc-context.log 2>&1 || {
    echo "FAIL: inherited context post rejected" >&2
    cat /tmp/conc-context.log >&2
    exit 1
  }
grep -q '^- generation: 1$' "$RUNNABLE_PATH"
grep -q '^- plan_revision: '"$REVISION"'$' "$RUNNABLE_PATH"
grep -q '^- dispatch_ready: no$' "$RUNNABLE_PATH"
[[ "$(grep -c '^## Post -- ' "$RUNNABLE_PATH")" -eq 2 ]]

# 3d. Reviews may flip dispatch_ready no -> yes on identical metadata.
notify --kind subtask --thread "$RUNNABLE" --parent CONC --subtask R-1 \
  --depends-on none --base-revision "$REVISION" --generation 1 \
  --plan-revision "$REVISION" --dispatch-ready yes --to eng --to-agent claude \
  --to-model glm-5.3 --from reviews --from-agent codex-reviews-main \
  --from-model gpt-5 --cli codex --slug x \
  --request "dispatch ready" >/tmp/conc-ready.log 2>&1 || {
    echo "FAIL: reviews dispatch flip rejected" >&2
    cat /tmp/conc-ready.log >&2
    exit 1
  }
grep -q '^- dispatch_ready: yes$' "$RUNNABLE_PATH"

# 3e. The flip must not walk back, even from reviews.
dispatched_meta="$(metadata_block "$RUNNABLE_PATH" | sha256sum | cut -d' ' -f1)"
if notify --kind subtask --thread "$RUNNABLE" --parent CONC --subtask R-1 \
    --depends-on none --base-revision "$REVISION" --generation 1 \
    --plan-revision "$REVISION" --dispatch-ready no --to eng --to-agent claude \
    --to-model glm-5.3 --from reviews --from-agent codex-reviews-main \
    --from-model gpt-5 --cli codex --slug x \
    --request "un-dispatch" >/tmp/conc-unready.log 2>&1; then
  echo "FAIL: dispatch_ready yes -> no accepted" >&2
  exit 1
fi
grep -q 'would change protected metadata' /tmp/conc-unready.log
assert_eq "$(metadata_block "$RUNNABLE_PATH" | sha256sum | cut -d' ' -f1)" \
  "$dispatched_meta" "metadata changed by refused un-dispatch"

# 3f. agent_reply.sh appends without touching metadata.
reply --thread "$RUNNABLE" --from eng --from-agent claude --from-model glm-5.3 \
  --cli claude --body "reply zero" --pointer docs/agents/QID_LEDGER.md \
  >/tmp/conc-reply0.log || {
    echo "FAIL: reply rejected" >&2
    exit 1
  }
grep -q '^## Reply -- eng/claude -- .* -- claude/glm-5.3$' "$RUNNABLE_PATH"
grep -q '^## Pointers$' "$RUNNABLE_PATH"
assert_eq "$(metadata_block "$RUNNABLE_PATH" | sha256sum | cut -d' ' -f1)" \
  "$dispatched_meta" "metadata changed by reply"

# 3g. A question thread also refuses kind drift.
QUESTION_THREAD="$(basename "$(ls docs/agents/discuss/*reuse-check.md)")"
before_q="$(sha256sum "docs/agents/discuss/$QUESTION_THREAD" | cut -d' ' -f1)"
if notify --kind consensus --thread "$QUESTION_THREAD" --to reviews \
    --from reviews --from-agent codex --from-model gpt-5 --cli codex --slug x \
    --request "kind drift" >/tmp/conc-kind-drift.log 2>&1; then
  echo "FAIL: kind drift accepted on question thread" >&2
  exit 1
fi
grep -q 'does not match existing thread kind' /tmp/conc-kind-drift.log
assert_eq "$(sha256sum "docs/agents/discuss/$QUESTION_THREAD" | cut -d' ' -f1)" \
  "$before_q" "question thread changed by refused kind drift"

# 4. Mixed concurrency: parallel replies and new questions in one burst.
rows_before="$(data_row_count)"
pids=()
for i in 1 2 3 4 5 6; do
  (
    reply --thread "$RUNNABLE" --from test --from-agent poller \
      --from-model glm-5.3 --cli claude --body "parallel reply body $i"
  ) >"/tmp/conc-mixed-reply-$i.log" 2>&1 &
  pids+=("$!")
done
for i in 1 2 3 4; do
  (
    notify --to reviews --from eng --from-agent claude --from-model glm-5.3 \
      --cli claude --slug "mixed-$i" --request "mixed question $i"
  ) >"/tmp/conc-mixed-q-$i.log" 2>&1 &
  pids+=("$!")
done
for pid in "${pids[@]}"; do
  wait "$pid" || {
    echo "FAIL: mixed concurrency pid $pid failed" >&2
    exit 1
  }
done
assert_eq "$(data_row_count)" "$((rows_before + 4))" "row count after mixed burst"
assert_eq "$(row_ids | sort | uniq -d)" "" "duplicate ids after mixed burst"
assert_eq "$(grep -oE 'Q-[0-9]{8}-[0-9]+' docs/agents/QID_LEDGER.md | sort -u | wc -l)" \
  "$((rows_before + 4))" "ledger covers every live id"
[[ "$(grep -c '^## Reply -- ' "$RUNNABLE_PATH")" -eq 7 ]]
for i in 1 2 3 4 5 6; do
  [[ "$(grep -c "parallel reply body $i" "$RUNNABLE_PATH")" -eq 1 ]] || {
    echo "FAIL: reply body $i lost or duplicated" >&2
    exit 1
  }
done
assert_eq "$(metadata_block "$RUNNABLE_PATH" | sha256sum | cut -d' ' -f1)" \
  "$dispatched_meta" "runnable metadata corrupted during mixed burst"
assert_eq "$(grep -c '^| id |' docs/agents/discuss/OPEN.md)" "1" \
  "mailbox header intact after mixed burst"

echo "agent mailbox concurrency tests passed"
