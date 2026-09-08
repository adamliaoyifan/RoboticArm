#!/usr/bin/env bash
# Focused MPF-1 generation/plan-revision lifecycle tests in isolated repos.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
TMP_ROOT="$(mktemp -d /tmp/agent-mailbox-freshness.XXXXXX)"
trap 'rm -rf "$TMP_ROOT"' EXIT

mkdir -p "$TMP_ROOT/docs/agents/discuss" "$TMP_ROOT/docs/agents" "$TMP_ROOT/src/luggage_gazebo"
cd "$TMP_ROOT"
git init -q
git config user.email freshness@example.invalid
git config user.name "Agent Freshness"
touch docs/agents/README.md src/luggage_gazebo/.keep approved-plan.md
git add docs/agents/README.md src/luggage_gazebo/.keep approved-plan.md
git commit -q -m baseline
REVISION="$(git rev-parse HEAD)"

write_open() {
  cat > docs/agents/discuss/OPEN.md <<EOF
# Open cross-agent work

| id | kind | parent | subtask | depends_on | revision | to_role | to_agent | to_model | from_role | from_agent | from_model | cli | thread | request | generation | plan_revision |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
$*
EOF
}

append_open_row() {
  printf '%s\n' "$1" >> docs/agents/discuss/OPEN.md
}

thread() {
  local file="$1" status="$2" parent="$3" subtask="$4" depends="$5" generation="$6"
  cat > "docs/agents/discuss/$file" <<EOF
# test -- $subtask g$generation

- status: $status
- to_role: eng
- to_agent: codex
- to_model: gpt-5
- kind: subtask
- parent: $parent
- subtask: $subtask
- depends_on: $depends
- revision: $REVISION
- generation: $generation
- plan_revision: $REVISION

## Post -- reviews/codex -- 2026-09-05 00:00 -- codex/gpt-5

test
EOF
}

run_start() {
  AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$TMP_ROOT" \
    "$ROOT/scripts/agent_start.sh" \
    --thread "$1" --role eng --agent codex --model gpt-5 --cli codex
}

run_complete() {
  AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$TMP_ROOT" \
    "$ROOT/scripts/agent_complete.sh" \
    --thread "$1" --role eng --agent codex --model gpt-5 --cli codex \
    --outcome pass --revision "$REVISION" --tests "freshness test pass" \
    --summary "closed"
}

run_transition() {
  AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$TMP_ROOT" \
    python3 "$ROOT/scripts/agent_mailbox.py" transition "$@"
}

run_ack() {
  AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$TMP_ROOT" \
    python3 "$ROOT/scripts/agent_mailbox.py" stop-ack "$@"
}

# New runnable notifications must carry generation and plan_revision.
write_open ""
if AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$TMP_ROOT" \
    "$ROOT/scripts/agent_notify.sh" --kind subtask --parent NOTIFY \
    --subtask A --depends-on none --base-revision "$REVISION" --to eng \
    --to-agent codex --to-model gpt-5 --from reviews --from-agent codex \
    --from-model gpt-5 --cli codex --slug notify-missing \
    --request "missing generation" >/tmp/notify-missing.log 2>&1; then
  echo "FAIL: runnable notify accepted missing generation/plan_revision" >&2
  exit 1
fi
grep -q 'requires --generation and --plan-revision' /tmp/notify-missing.log
AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$TMP_ROOT" \
  "$ROOT/scripts/agent_notify.sh" --kind subtask --parent NOTIFY \
  --subtask A --depends-on none --base-revision "$REVISION" --to eng \
  --to-agent codex --to-model gpt-5 --from reviews --from-agent codex \
  --from-model gpt-5 --cli codex --slug notify-ok \
  --request "generation present" --generation 1 --plan-revision "$REVISION" \
  >/tmp/notify-ok.log
grep -q '| request | generation | plan_revision |' docs/agents/discuss/OPEN.md
grep -q "| 1 | $REVISION |" docs/agents/discuss/OPEN.md
grep -q '^- generation: 1$' docs/agents/discuss/*notify-ok.md
grep -q "^- plan_revision: $REVISION$" docs/agents/discuss/*notify-ok.md
grep -q '^- dispatch_ready: no$' docs/agents/discuss/*notify-ok.md
notify_thread="$(basename "$(ls docs/agents/discuss/*notify-ok.md)")"
if run_start "$notify_thread" >/tmp/notify-draft-start.log 2>&1; then
  echo "FAIL: draft runnable started before dispatch_ready" >&2
  exit 1
fi
grep -q 'thread is not dispatch_ready' /tmp/notify-draft-start.log
AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$TMP_ROOT" \
  "$ROOT/scripts/agent_notify.sh" --kind subtask --parent NOTIFY \
  --subtask A --depends-on none --base-revision "$REVISION" --to eng \
  --to-agent codex --to-model gpt-5 --from reviews --from-agent codex \
  --from-model gpt-5 --cli codex --thread "$notify_thread" \
  --request "reviewers dispatch ready" --generation 1 \
  --plan-revision "$REVISION" --dispatch-ready yes >/tmp/notify-ready.log
grep -q '^- dispatch_ready: yes$' "docs/agents/discuss/$notify_thread"
run_start "$notify_thread" >/tmp/notify-ready-start.log

# Row/thread generation mismatch fails closed before claim.
write_open "| Q-M | subtask | MISMATCH | A | none | $REVISION | eng | codex | gpt-5 | reviews | codex | gpt-5 | codex | mismatch-a-g1.md | mismatch | 2 | $REVISION |"
thread mismatch-a-g1.md open MISMATCH A none 1
if run_start mismatch-a-g1.md >/tmp/mismatch-start.log 2>&1; then
  echo "FAIL: row/thread generation mismatch started" >&2
  exit 1
fi
grep -q 'row/thread generation or plan_revision mismatch' /tmp/mismatch-start.log

# Normal generated task records generation and plan revision, then closes.
write_open "| Q-1 | subtask | NORMAL | A | none | $REVISION | eng | codex | gpt-5 | reviews | codex | gpt-5 | codex | normal-a-g1.md | normal | 1 | $REVISION |"
thread normal-a-g1.md open NORMAL A none 1
run_start normal-a-g1.md >/tmp/normal-start.log
grep -q '^- claimed_generation: 1$' docs/agents/discuss/normal-a-g1.md
grep -q "^- claimed_plan_revision: $REVISION$" docs/agents/discuss/normal-a-g1.md
run_complete normal-a-g1.md >/tmp/normal-complete.log
grep -q '^- status: done$' docs/agents/discuss/normal-a-g1.md
! grep -q 'normal-a-g1.md' docs/agents/discuss/OPEN.md

# Claimed generation 1 cannot pass after generation 2 supersedes it.
write_open "| Q-2 | subtask | SUPER | A | none | $REVISION | eng | codex | gpt-5 | reviews | codex | gpt-5 | codex | super-a-g1.md | old | 1 | $REVISION |"
thread super-a-g1.md open SUPER A none 1
run_start super-a-g1.md >/tmp/super-start.log
append_open_row "| Q-3 | subtask | SUPER | A | none | $REVISION | eng | codex | gpt-5 | reviews | codex | gpt-5 | codex | super-a-g2.md | new | 2 | $REVISION |"
thread super-a-g2.md open SUPER A none 2
run_transition --thread super-a-g1.md --state superseded --replacement super-a-g2.md \
  --reason replan --role reviews --agent codex --model gpt-5 --cli codex >/tmp/super-transition.log
if run_complete super-a-g1.md >/tmp/super-complete.log 2>&1; then
  echo "FAIL: stale superseded generation completed" >&2
  exit 1
fi
! grep -q '^- outcome: pass$' docs/agents/discuss/super-a-g1.md
grep -q 'super-a-g1.md' docs/agents/discuss/OPEN.md
run_ack --thread super-a-g1.md --reason seen --role eng --agent codex --model gpt-5 --cli codex >/tmp/super-ack.log
! grep -q 'super-a-g1.md' docs/agents/discuss/OPEN.md

# Cancelled claimed work cannot pass and remains until owner ack.
write_open "| Q-4 | subtask | CANCEL | A | none | $REVISION | eng | codex | gpt-5 | reviews | codex | gpt-5 | codex | cancel-a-g1.md | cancel | 1 | $REVISION |"
thread cancel-a-g1.md open CANCEL A none 1
run_start cancel-a-g1.md >/tmp/cancel-start.log
run_transition --thread cancel-a-g1.md --state cancelled \
  --reason stop --role reviews --agent codex --model gpt-5 --cli codex >/tmp/cancel-transition.log
if run_complete cancel-a-g1.md >/tmp/cancel-complete.log 2>&1; then
  echo "FAIL: cancelled generation completed" >&2
  exit 1
fi
grep -q 'cancel-a-g1.md' docs/agents/discuss/OPEN.md
run_ack --thread cancel-a-g1.md --role eng --agent codex --model gpt-5 --cli codex >/tmp/cancel-ack.log
! grep -q 'cancel-a-g1.md' docs/agents/discuss/OPEN.md

# A downstream claim against A1 goes stale when A2 supersedes the passing A1.
write_open "| Q-5 | subtask | DEPS | A | none | $REVISION | eng | codex | gpt-5 | reviews | codex | gpt-5 | codex | deps-a-g1.md | a1 | 1 | $REVISION |"
append_open_row "| Q-6 | subtask | DEPS | B | A | $REVISION | eng | codex | gpt-5 | reviews | codex | gpt-5 | codex | deps-b-g1.md | b1 | 1 | $REVISION |"
thread deps-a-g1.md open DEPS A none 1
thread deps-b-g1.md open DEPS B A 1
run_start deps-a-g1.md >/tmp/deps-a-start.log
run_complete deps-a-g1.md >/tmp/deps-a-complete.log
run_start deps-b-g1.md >/tmp/deps-b-start.log
grep -q '^- claimed_dependencies: A=1$' docs/agents/discuss/deps-b-g1.md
append_open_row "| Q-7 | subtask | DEPS | A | none | $REVISION | eng | codex | gpt-5 | reviews | codex | gpt-5 | codex | deps-a-g2.md | a2 | 2 | $REVISION |"
thread deps-a-g2.md open DEPS A none 2
run_transition --thread deps-a-g1.md --state superseded --replacement deps-a-g2.md \
  --reason upstream-replan --role reviews --agent codex --model gpt-5 --cli codex >/tmp/deps-supersede.log
if run_complete deps-b-g1.md >/tmp/deps-b-complete.log 2>&1; then
  echo "FAIL: downstream completed with stale dependency generation" >&2
  exit 1
fi
grep -q 'claimed dependency generation is stale: A' /tmp/deps-b-complete.log

# Concurrent terminal transitions are serialized by the mailbox lock.
write_open "| Q-8 | subtask | CONCUR | A | none | $REVISION | eng | codex | gpt-5 | reviews | codex | gpt-5 | codex | concur-a-g1.md | concurrent | 1 | $REVISION |"
thread concur-a-g1.md open CONCUR A none 1
(
  run_transition --thread concur-a-g1.md --state cancelled --reason first \
    --role reviews --agent codex --model gpt-5 --cli codex
) >/tmp/concur-one.log 2>&1 &
pid_one=$!
(
  run_transition --thread concur-a-g1.md --state cancelled --reason second \
    --role reviews --agent codex --model gpt-5 --cli codex
) >/tmp/concur-two.log 2>&1 &
pid_two=$!
ok=0
fail=0
wait "$pid_one" && ok=$((ok + 1)) || fail=$((fail + 1))
wait "$pid_two" && ok=$((ok + 1)) || fail=$((fail + 1))
[[ "$ok" -eq 1 && "$fail" -eq 1 ]] || {
  echo "FAIL: concurrent cancel did not serialize one success/one failure" >&2
  exit 1
}
grep -q '^- status: cancelled$' docs/agents/discuss/concur-a-g1.md

# Positive legacy migration is dry-run capable and idempotent.
MIG_ROOT="$(mktemp -d /tmp/agent-mailbox-migrate.XXXXXX)"
mkdir -p "$MIG_ROOT/docs/agents/discuss" "$MIG_ROOT/docs/agents" "$MIG_ROOT/src/luggage_gazebo"
cd "$MIG_ROOT"
git init -q
git config user.email freshness@example.invalid
git config user.name "Agent Freshness"
touch docs/agents/README.md src/luggage_gazebo/.keep approved-plan.md
git add docs/agents/README.md src/luggage_gazebo/.keep approved-plan.md
git commit -q -m baseline
MIG_REV="$(git rev-parse HEAD)"
cat > docs/agents/discuss/OPEN.md <<EOF
# Open cross-agent work

| id | kind | parent | subtask | depends_on | revision | to_role | to_agent | to_model | from_role | from_agent | from_model | cli | thread | request |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Q-MIG | subtask | MIG | A | none | $MIG_REV | eng | codex | gpt-5 | reviews | codex | gpt-5 | codex | mig-a.md | migrate |
EOF
cat > docs/agents/discuss/mig-a.md <<EOF
# migrate

- status: open
- to_role: eng
- to_agent: codex
- to_model: gpt-5
- kind: subtask
- parent: MIG
- subtask: A
- depends_on: none
- revision: $MIG_REV

## Post -- reviews/codex -- 2026-09-05 00:00 -- codex/gpt-5

migrate
EOF
AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$MIG_ROOT" \
  python3 "$ROOT/scripts/agent_mailbox.py" migrate --plan-revision "$MIG_REV" \
  --dry-run >/tmp/migrate-dry-run.log
! grep -q '^- generation:' docs/agents/discuss/mig-a.md
AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$MIG_ROOT" \
  python3 "$ROOT/scripts/agent_mailbox.py" migrate --plan-revision "$MIG_REV" \
  >/tmp/migrate-run.log
grep -q '| request | generation | plan_revision |' docs/agents/discuss/OPEN.md
grep -q '^- generation: 1$' docs/agents/discuss/mig-a.md
grep -q "^- plan_revision: $MIG_REV$" docs/agents/discuss/mig-a.md
AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$MIG_ROOT" \
  python3 "$ROOT/scripts/agent_mailbox.py" migrate --plan-revision "$MIG_REV" \
  >/tmp/migrate-idempotent.log
grep -q 'migrated 0 runnable thread' /tmp/migrate-idempotent.log
rm -rf "$MIG_ROOT"

# Migration refuses ambiguous duplicate legacy lineages.
AMBIG_ROOT="$(mktemp -d /tmp/agent-mailbox-ambiguous.XXXXXX)"
mkdir -p "$AMBIG_ROOT/docs/agents/discuss" "$AMBIG_ROOT/docs/agents" "$AMBIG_ROOT/src/luggage_gazebo"
cd "$AMBIG_ROOT"
git init -q
git config user.email freshness@example.invalid
git config user.name "Agent Freshness"
touch docs/agents/README.md src/luggage_gazebo/.keep approved-plan.md
git add docs/agents/README.md src/luggage_gazebo/.keep approved-plan.md
git commit -q -m baseline
AMBIG_REV="$(git rev-parse HEAD)"
cat > docs/agents/discuss/OPEN.md <<EOF
# Open cross-agent work

| id | kind | parent | subtask | depends_on | revision | to_role | to_agent | to_model | from_role | from_agent | from_model | cli | thread | request |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
EOF
for file in legacy-one.md legacy-two.md; do
  cat > "docs/agents/discuss/$file" <<EOF
# legacy duplicate

- status: open
- to_role: eng
- to_agent: codex
- to_model: gpt-5
- kind: subtask
- parent: AMBIG
- subtask: A
- depends_on: none
- revision: $AMBIG_REV

## Post -- reviews/codex -- 2026-09-05 00:00 -- codex/gpt-5

legacy
EOF
done
if AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$AMBIG_ROOT" \
    python3 "$ROOT/scripts/agent_mailbox.py" migrate --plan-revision "$AMBIG_REV" \
    --dry-run >/tmp/ambig-migrate.log 2>&1; then
  echo "FAIL: ambiguous duplicate migration succeeded" >&2
  exit 1
fi
grep -q 'ambiguous duplicate lineage' /tmp/ambig-migrate.log
rm -rf "$AMBIG_ROOT"

echo "agent mailbox freshness tests passed"
