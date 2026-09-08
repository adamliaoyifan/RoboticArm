#!/usr/bin/env bash
# Smoke-test the file-based agent owner lifecycle in an isolated temp repo.
set -euo pipefail

TMP_ROOT="$(mktemp -d /tmp/agent-lifecycle-smoke.XXXXXX)"
trap 'rm -rf "$TMP_ROOT"' EXIT

mkdir -p "$TMP_ROOT/docs/agents/discuss" "$TMP_ROOT/docs/agents" "$TMP_ROOT/src/luggage_gazebo" \
  "$TMP_ROOT/scripts"
cp scripts/agent_start.sh "$TMP_ROOT/scripts.agent_start.sh"
cp scripts/agent_complete.sh "$TMP_ROOT/scripts.agent_complete.sh"
cp scripts/agent_mailbox.py "$TMP_ROOT/agent_mailbox.py"
cp scripts/agent_flow_metrics.py "$TMP_ROOT/scripts.agent_flow_metrics.py"
cp scripts/agent_register.sh "$TMP_ROOT/scripts.agent_register.sh"
cp scripts/agent_scheduler.py "$TMP_ROOT/scripts.agent_scheduler.py"
cp scripts/agent_poll_self.py "$TMP_ROOT/scripts/agent_poll_self.py"
cp scripts/agent_scheduler.py "$TMP_ROOT/scripts/agent_scheduler.py"
cp scripts/agent_register.sh "$TMP_ROOT/scripts/agent_register.sh"
chmod +x "$TMP_ROOT/scripts/agent_poll_self.py"

cd "$TMP_ROOT"
git init -q
git config user.email smoke@example.invalid
git config user.name "Agent Smoke"
touch docs/agents/README.md src/luggage_gazebo/.keep
git add docs/agents/README.md src/luggage_gazebo/.keep
git commit -q -m baseline
REVISION="$(git rev-parse HEAD)"
HEARTBEAT="$(date --iso-8601=seconds)"

cat > docs/agents/discuss/OPEN.md <<EOF
# Open cross-agent work

| id | kind | parent | subtask | depends_on | revision | to_role | to_agent | to_model | from_role | from_agent | from_model | cli | thread | request |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Q-20260904-1 | subtask | SMOKE | ST-1 | none | $REVISION | eng | claude | glm-5.3 | reviews | codex | gpt-5 | codex | 2026-09-04_0001_smoke-st1.md | close ST-1 |
| Q-20260904-2 | subtask | SMOKE | ST-2 | ST-1 | $REVISION | eng | cursor | opus5 | reviews | codex | gpt-5 | codex | 2026-09-04_0002_smoke-st2.md | close ST-2 |
EOF

cat > docs/agents/discuss/2026-09-04_0001_smoke-st1.md <<EOF
# 2026-09-04 -- Smoke ST-1

- status: open
- to_role: eng
- to_agent: claude
- to_model: glm-5.3
- kind: subtask
- parent: SMOKE
- subtask: ST-1
- depends_on: none
- revision: $REVISION

## Post -- reviews/codex -- 2026-09-04 00:01 -- codex/gpt-5

close ST-1
EOF

cat > docs/agents/discuss/2026-09-04_0002_smoke-st2.md <<EOF
# 2026-09-04 -- Smoke ST-2

- status: open
- to_role: eng
- to_agent: cursor
- to_model: opus5
- kind: subtask
- parent: SMOKE
- subtask: ST-2
- depends_on: ST-1
- revision: $REVISION

## Post -- reviews/codex -- 2026-09-04 00:02 -- codex/gpt-5

close ST-2
EOF

AGENT_RUNTIME_LOCK="$TMP_ROOT/runtime.lock" \
  "$TMP_ROOT/scripts.agent_register.sh" \
  --id smoke-claude --role eng --agent claude --model glm-5.3 \
  --cli codex --session smoke-session --worktree "$TMP_ROOT" \
  --capabilities queue --heartbeat "$HEARTBEAT" >/tmp/register-claude.log

AGENT_RUNTIME_LOCK="$TMP_ROOT/runtime.lock" \
  "$TMP_ROOT/scripts.agent_register.sh" \
  --id smoke-cursor --role eng --agent cursor --model opus5 \
  --cli cursor --session smoke-session-2 --worktree "$TMP_ROOT" \
  --capabilities file --heartbeat "$HEARTBEAT" >/tmp/register-cursor.log

python3 "$TMP_ROOT/scripts.agent_scheduler.py" --json >/tmp/scheduler-before.json
grep -q '"thread": "2026-09-04_0001_smoke-st1.md"' /tmp/scheduler-before.json
grep -q '"action": "queue"' /tmp/scheduler-before.json
grep -q '"thread": "2026-09-04_0002_smoke-st2.md"' /tmp/scheduler-before.json
grep -q '"action": "wait"' /tmp/scheduler-before.json

python3 "$TMP_ROOT/scripts/agent_poll_self.py" --agent cursor --model opus5 \
  --json --queue-file /tmp/agent-poll-cursor-before.json >/tmp/poll-cursor-before.json
grep -q '"id": "Q-20260904-2"' /tmp/poll-cursor-before.json
grep -q '"action": "wait"' /tmp/poll-cursor-before.json
! grep -q '"id": "Q-20260904-1"' /tmp/poll-cursor-before.json

python3 "$TMP_ROOT/scripts/agent_poll_self.py" --agent claude --model glm-5.3 \
  --json --queue-file /tmp/agent-poll-claude-before.json >/tmp/poll-claude-before.json
python3 - <<'PY'
import json
data = json.load(open("/tmp/poll-claude-before.json", encoding="utf-8"))
assert data["next"]["id"] == "Q-20260904-1", data["next"]
assert data["next"]["action"] == "ready", data["next"]
assert data["queue"] == ["Q-20260904-1"], data["queue"]
PY

if AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$TMP_ROOT" \
    "$TMP_ROOT/scripts.agent_start.sh" \
    --thread 2026-09-04_0002_smoke-st2.md \
    --role eng --agent cursor --model opus5 --cli cursor >/tmp/st2-before.log 2>&1; then
  echo "FAIL: dependent ST-2 started before ST-1 passed" >&2
  exit 1
fi

if AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$TMP_ROOT" \
    "$TMP_ROOT/scripts.agent_start.sh" \
    --thread 2026-09-04_0001_smoke-st1.md \
    --role eng --agent cursor --model opus5 --cli cursor >/tmp/st1-wrong-owner.log 2>&1; then
  echo "FAIL: wrong owner started ST-1" >&2
  exit 1
fi

AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$TMP_ROOT" \
  "$TMP_ROOT/scripts.agent_start.sh" \
  --thread 2026-09-04_0001_smoke-st1.md \
  --role eng --agent claude --model glm-5.3 --cli claude >/tmp/st1-start.log

AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$TMP_ROOT" \
  "$TMP_ROOT/scripts.agent_complete.sh" \
  --thread 2026-09-04_0001_smoke-st1.md \
  --role eng --agent claude --model glm-5.3 --cli claude \
  --outcome pass --revision "$REVISION" --tests "smoke unit pass" \
  --summary "ST-1 smoke closed" >/tmp/st1-complete.log

python3 "$TMP_ROOT/scripts/agent_poll_self.py" --agent cursor --model opus5 \
  --json --queue-file /tmp/agent-poll-cursor-after.json >/tmp/poll-cursor-after.json
python3 - <<'PY'
import json
data = json.load(open("/tmp/poll-cursor-after.json", encoding="utf-8"))
assert data["next"]["id"] == "Q-20260904-2", data["next"]
assert data["next"]["action"] == "ready", data["next"]
assert data["queue"] == ["Q-20260904-2"], data["queue"]
PY

AGENT_MAILBOX_ALLOW_ANY_ROOT=1 AGENT_COORD_ROOT="$TMP_ROOT" \
  "$TMP_ROOT/scripts.agent_start.sh" \
  --thread 2026-09-04_0002_smoke-st2.md \
  --role eng --agent cursor --model opus5 --cli cursor >/tmp/st2-start.log

python3 "$TMP_ROOT/scripts.agent_scheduler.py" --json >/tmp/scheduler-after.json
grep -q '"thread": "2026-09-04_0002_smoke-st2.md"' /tmp/scheduler-after.json
grep -q '"action": "skip"' /tmp/scheduler-after.json
grep -q '"reason": "already claimed"' /tmp/scheduler-after.json

python3 "$TMP_ROOT/scripts.agent_flow_metrics.py" --parent SMOKE --threads-dir docs/agents/discuss --json >/tmp/agent-lifecycle-smoke-metrics.json

grep -q '"completed": 1' /tmp/agent-lifecycle-smoke-metrics.json
grep -q '"subtask": "ST-1"' /tmp/agent-lifecycle-smoke-metrics.json
grep -q '"subtask": "ST-2"' /tmp/agent-lifecycle-smoke-metrics.json
grep -q '2026-09-04_0002_smoke-st2.md' docs/agents/discuss/OPEN.md
! grep -q '2026-09-04_0001_smoke-st1.md' docs/agents/discuss/OPEN.md

echo "agent lifecycle smoke passed"
