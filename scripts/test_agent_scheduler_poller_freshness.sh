#!/usr/bin/env bash
# Focused MPF-2 scheduler/poller freshness tests in an isolated temp repo.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
TMP_ROOT="$(mktemp -d /tmp/agent-scheduler-poller.XXXXXX)"
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
HEARTBEAT="$(date --iso-8601=seconds)"

cat > docs/agents/RUNTIME.md <<EOF
# Agent runtime registry

| id | role | agent | model | cli | session | worktree | state | capabilities | heartbeat |
|---|---|---|---|---|---|---|---|---|---|
| codex-live | eng | codex | gpt-5 | codex | session-live | $TMP_ROOT | idle | queue | $HEARTBEAT |
| cursor-file | eng | cursor | opus5 | cursor | session-file | $TMP_ROOT | idle | file | $HEARTBEAT |
EOF

write_open() {
  cat > docs/agents/discuss/OPEN.md <<EOF
# Open cross-agent work

| id | kind | parent | subtask | depends_on | revision | to_role | to_agent | to_model | from_role | from_agent | from_model | cli | thread | request | generation | plan_revision |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
$*
EOF
}

thread() {
  local file="$1" status="$2" parent="$3" subtask="$4" depends="$5" generation="$6" agent="${7:-codex}" model="${8:-gpt-5}"
  cat > "docs/agents/discuss/$file" <<EOF
# test -- $subtask g$generation

- status: $status
- to_role: eng
- to_agent: $agent
- to_model: $model
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

append_claim() {
  cat >> "docs/agents/discuss/$1" <<EOF

## Claim -- eng/$2 -- 2026-09-05 00:01 -- codex/$3

- started_at: 2026-09-05T00:01:00+08:00
- claimed_generation: $4
- claimed_plan_revision: $REVISION
- claimed_dependencies: none
EOF
}

append_result_pass() {
  cat >> "docs/agents/discuss/$1" <<EOF

## Result -- eng/codex -- 2026-09-05 00:02 -- codex/gpt-5

- outcome: pass
- completed_at: 2026-09-05T00:02:00+08:00
- revision: $REVISION
- tests: fixture
- summary: fixture
EOF
}

append_superseded() {
  cat >> "docs/agents/discuss/$1" <<EOF

## Superseded -- reviews/codex -- 2026-09-05 00:03 -- codex/gpt-5

- transitioned_at: 2026-09-05T00:03:00+08:00
- old_generation: $2
- replacement: $3
- reason: replan
EOF
}

run_ack() {
  AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$TMP_ROOT" \
    python3 "$ROOT/scripts/agent_mailbox.py" stop-ack "$@"
}

# Draft runnable rows are visible but not claimable until reviewers mark ready.
write_open "| Q-D | subtask | DRAFT | A | none | $REVISION | eng | codex | gpt-5 | reviews | codex | gpt-5 | codex | draft-a-g1.md | draft | 1 | $REVISION |"
thread draft-a-g1.md open DRAFT A none 1
printf '%s\n' '- dispatch_ready: no' > /tmp/dispatch-ready-line.txt
tmp_thread="$(mktemp)"
awk 'BEGIN{inserted=0} /^## / && !inserted {print "- dispatch_ready: no"; inserted=1} {print}' \
  docs/agents/discuss/draft-a-g1.md > "$tmp_thread"
mv "$tmp_thread" docs/agents/discuss/draft-a-g1.md
python3 "$ROOT/scripts/agent_poll_self.py" --agent codex --model gpt-5 \
  --cli codex --claim-next --json --queue-file /tmp/mpf2-poll-draft.json \
  >/tmp/mpf2-poll-draft.out
python3 - <<'PY'
import json
data = json.load(open("/tmp/mpf2-poll-draft.out", encoding="utf-8"))
item = next(row for row in data["mine"] if row["thread"] == "draft-a-g1.md")
assert item["action"] == "wait", item
assert item["reason"] == "waiting for reviewers dispatch_ready", item
assert "claimed" not in data, data
PY
! grep -q '^## Claim --' docs/agents/discuss/draft-a-g1.md
python3 "$ROOT/scripts/agent_scheduler.py" --json \
  --registry docs/agents/RUNTIME.md --leases-dir docs/agents/discuss/leases \
  >/tmp/mpf2-scheduler-draft.json
python3 - <<'PY'
import json
rows = json.load(open("/tmp/mpf2-scheduler-draft.json", encoding="utf-8"))
item = next(row for row in rows if row["thread"] == "draft-a-g1.md")
assert item["action"] == "wait", item
assert item["reason"] == "waiting for reviewers dispatch_ready", item
PY

# Stop items appear before replacements and block auto-claim.
write_open "| Q-1 | subtask | STOP | A | none | $REVISION | eng | codex | gpt-5 | reviews | codex | gpt-5 | codex | stop-a-g1.md | old | 1 | $REVISION |
| Q-2 | subtask | STOP | A | none | $REVISION | eng | codex | gpt-5 | reviews | codex | gpt-5 | codex | stop-a-g2.md | new | 2 | $REVISION |"
thread stop-a-g1.md superseded STOP A none 1
thread stop-a-g2.md open STOP A none 2
append_claim stop-a-g1.md codex gpt-5 1
append_superseded stop-a-g1.md 1 stop-a-g2.md
python3 "$ROOT/scripts/agent_poll_self.py" --agent codex --model gpt-5 \
  --cli codex --claim-next --json --queue-file /tmp/mpf2-poll-stop.json \
  >/tmp/mpf2-poll-stop.out
python3 - <<'PY'
import json
data = json.load(open("/tmp/mpf2-poll-stop.out", encoding="utf-8"))
assert data["next"]["action"] == "stop", data["next"]
assert data["next"]["id"] == "Q-1", data["next"]
assert "replacement stop-a-g2.md" in data["next"]["reason"], data["next"]
assert "claimed" not in data, data
PY
! grep -q '^## Claim --' docs/agents/discuss/stop-a-g2.md

python3 "$ROOT/scripts/agent_scheduler.py" --json \
  --registry docs/agents/RUNTIME.md --leases-dir docs/agents/discuss/leases \
  >/tmp/mpf2-scheduler-stop.json
python3 - <<'PY'
import json
rows = json.load(open("/tmp/mpf2-scheduler-stop.json", encoding="utf-8"))
old = next(row for row in rows if row["thread"] == "stop-a-g1.md")
new = next(row for row in rows if row["thread"] == "stop-a-g2.md")
assert old["action"] == "stop", old
assert "replacement stop-a-g2.md" in old["reason"], old
assert new["action"] == "wait", new
assert new["reason"] == "waiting for owner stop acknowledgement", new
PY

mkdir -p docs/agents/discuss/leases
cat > docs/agents/discuss/leases/stop-stop-a-g1.md.lease.md <<EOF
# Scheduler lease -- stop-a-g1.md

- status: leased
- lease_until: 2999-01-01T00:00:00+00:00
EOF
python3 "$ROOT/scripts/agent_scheduler.py" --json \
  --registry docs/agents/RUNTIME.md --leases-dir docs/agents/discuss/leases \
  >/tmp/mpf2-scheduler-stop-leased.json
python3 - <<'PY'
import json
rows = json.load(open("/tmp/mpf2-scheduler-stop-leased.json", encoding="utf-8"))
old = next(row for row in rows if row["thread"] == "stop-a-g1.md")
assert old["action"] == "skip", old
assert old["reason"].startswith("stop notice leased until"), old
PY
rm docs/agents/discuss/leases/stop-stop-a-g1.md.lease.md

run_ack --thread stop-a-g1.md --role eng --agent codex --model gpt-5 \
  --cli codex >/tmp/mpf2-stop-ack.log
python3 "$ROOT/scripts/agent_poll_self.py" --agent codex --model gpt-5 \
  --cli codex --json --queue-file /tmp/mpf2-poll-after-ack.json \
  >/tmp/mpf2-poll-after-ack.out
python3 - <<'PY'
import json
data = json.load(open("/tmp/mpf2-poll-after-ack.out", encoding="utf-8"))
assert data["next"]["id"] == "Q-2", data["next"]
assert data["next"]["action"] == "ready", data["next"]
PY

# File-only owners see stop-file-only through scheduler.
write_open "| Q-3 | subtask | FILESTOP | A | none | $REVISION | eng | cursor | opus5 | reviews | codex | gpt-5 | codex | file-stop-a-g1.md | old | 1 | $REVISION |"
thread file-stop-a-g1.md cancelled FILESTOP A none 1 cursor opus5
append_claim file-stop-a-g1.md cursor opus5 1
python3 "$ROOT/scripts/agent_scheduler.py" --json \
  --registry docs/agents/RUNTIME.md --leases-dir docs/agents/discuss/leases \
  >/tmp/mpf2-scheduler-file-stop.json
python3 - <<'PY'
import json
rows = json.load(open("/tmp/mpf2-scheduler-file-stop.json", encoding="utf-8"))
item = next(row for row in rows if row["thread"] == "file-stop-a-g1.md")
assert item["action"] == "stop-file-only", item
assert "registered session has no supported live queue adapter" in item["reason"], item
PY

# Dependency readiness uses the highest dependency generation, not an older pass.
write_open "| Q-4 | subtask | DEPHIGH | B | A | $REVISION | eng | codex | gpt-5 | reviews | codex | gpt-5 | codex | dep-b-g1.md | b | 1 | $REVISION |"
thread dep-a-g1.md done DEPHIGH A none 1
append_claim dep-a-g1.md codex gpt-5 1
append_result_pass dep-a-g1.md
thread dep-a-g2.md open DEPHIGH A none 2
thread dep-b-g1.md open DEPHIGH B A 1
python3 "$ROOT/scripts/agent_scheduler.py" --json \
  --registry docs/agents/RUNTIME.md --leases-dir docs/agents/discuss/leases \
  >/tmp/mpf2-scheduler-dep.json
python3 - <<'PY'
import json
rows = json.load(open("/tmp/mpf2-scheduler-dep.json", encoding="utf-8"))
item = next(row for row in rows if row["thread"] == "dep-b-g1.md")
assert item["action"] == "wait", item
assert item["reason"] == "waiting for A", item
PY
python3 "$ROOT/scripts/agent_poll_self.py" --agent codex --model gpt-5 \
  --cli codex --json --queue-file /tmp/mpf2-poll-dep.json \
  >/tmp/mpf2-poll-dep.out
python3 - <<'PY'
import json
data = json.load(open("/tmp/mpf2-poll-dep.out", encoding="utf-8"))
item = next(row for row in data["mine"] if row["thread"] == "dep-b-g1.md")
assert item["action"] == "wait", item
assert item["reason"] == "waiting for A", item
PY

echo "agent scheduler/poller freshness tests passed"
