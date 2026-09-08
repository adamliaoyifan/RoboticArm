#!/usr/bin/env bash
# Validate the shared CLI-agent contract and note hygiene.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
cd "$ROOT"

failures=0

fail() {
  echo "FAIL: $*" >&2
  failures=$((failures + 1))
}

trim() {
  local value="$*"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "$value"
}

require_file() {
  [[ -f "$1" ]] || fail "missing required file: $1"
}

require_executable() {
  [[ -x "$1" ]] || fail "not executable: $1"
}

check_required_files() {
  local file
  for file in \
    AGENTS.md \
    docs/agents/README.md \
    docs/agents/FORMAT.md \
    docs/agents/RUNTIME.md \
    docs/agents/discuss/OPEN.md \
    docs/agents/reviews/README.md \
    docs/agents/eng/README.md \
    docs/agents/test/README.md \
    .cursor/rules/agent-logs.mdc \
    .cursor/rules/container-geometry.mdc \
    .cursor/rules/sim-lifecycle.mdc \
    .cursor/rules/perception-data-pipeline.mdc \
    .cursor/rules/ros2-node-structure.mdc \
    .cursor/rules/sensor-frames-and-timing.mdc \
    docs/architecture/README.md \
    scripts/agent_start.sh \
    scripts/agent_complete.sh \
    scripts/agent_mailbox.py \
    scripts/agent_flow_metrics.py \
    scripts/agent_notify.sh \
    scripts/agent_register.sh \
    scripts/agent_scheduler.py \
    scripts/agent_poll_self.py \
    scripts/poll_eng_completed.py \
    scripts/test_agent_lifecycle_smoke.sh \
    scripts/test_agent_mailbox_freshness.sh \
    scripts/test_agent_scheduler_poller_freshness.sh \
    scripts/eng_checkpoint_gates.json \
    scripts/stop_sim.sh; do
    require_file "$file"
  done
}

check_git_baseline() {
  git rev-parse --is-inside-work-tree >/dev/null 2>&1 ||
    fail "workspace is not a git repository"
  git rev-parse --verify --quiet HEAD >/dev/null ||
    fail "git repository has no baseline commit"
}

check_open_mailbox() {
  local open_file="docs/agents/discuss/OPEN.md"
  local line id kind parent subtask depends_on revision to_role to_agent
  local to_model from_role from_agent from_model cli thread request generation
  local plan_revision has_generation_columns

  [[ -f "$open_file" ]] || return
  grep -Eq '^\| id \| kind \| parent \| subtask \| depends_on \| revision \| to_role \| to_agent \| to_model \| from_role \| from_agent \| from_model \| cli \| thread \| request( \| generation \| plan_revision)? \|$' "$open_file" ||
    fail "$open_file is missing the expected table header"
  has_generation_columns=0
  grep -q '| request | generation | plan_revision |' "$open_file" &&
    has_generation_columns=1

  while IFS= read -r line; do
    [[ "$line" == \|* ]] || continue
    [[ "$line" == "| id "* ]] && continue
    [[ "$line" == "|---"* ]] && continue

    line="${line#|}"
    line="${line%|}"
    IFS='|' read -r id kind parent subtask depends_on revision to_role to_agent to_model from_role from_agent from_model cli thread request generation plan_revision <<< "$line"
    id="$(trim "$id")"
    kind="$(trim "$kind")"
    parent="$(trim "$parent")"
    subtask="$(trim "$subtask")"
    depends_on="$(trim "$depends_on")"
    revision="$(trim "$revision")"
    to_role="$(trim "$to_role")"
    to_agent="$(trim "$to_agent")"
    to_model="$(trim "$to_model")"
    from_role="$(trim "$from_role")"
    from_agent="$(trim "$from_agent")"
    from_model="$(trim "$from_model")"
    cli="$(trim "$cli")"
    thread="$(trim "$thread")"
    request="$(trim "$request")"
    generation="$(trim "${generation:-}")"
    plan_revision="$(trim "${plan_revision:-}")"

    [[ "$id" =~ ^Q-[0-9]{8}-[0-9]+$ ]] ||
      fail "$open_file has invalid id: $id"
    [[ "$kind" =~ ^(question|consensus|subtask|integration|regression|task|checkpoint)$ ]] ||
      fail "$open_file row $id has invalid kind: $kind"
    [[ -n "$parent" ]] ||
      fail "$open_file row $id has empty parent"
    [[ -n "$subtask" ]] ||
      fail "$open_file row $id has empty subtask"
    [[ -n "$depends_on" ]] ||
      fail "$open_file row $id has empty depends_on"
    [[ -n "$revision" ]] ||
      fail "$open_file row $id has empty revision"
    if [[ "$kind" =~ ^(subtask|integration|regression)$ ]]; then
      [[ "$parent" != "n/a" ]] ||
        fail "$open_file $kind row $id has no parent task"
      [[ "$subtask" != "n/a" ]] ||
        fail "$open_file $kind row $id has no subtask id"
      [[ "$revision" != "n/a" ]] ||
        fail "$open_file $kind row $id has no revision"
      [[ "$to_agent" != "any" ]] ||
        fail "$open_file $kind row $id has no concrete target agent"
      [[ "$to_model" != "any" ]] ||
        fail "$open_file $kind row $id has no concrete target model"
      if [[ "$has_generation_columns" -eq 1 &&
          ( -n "$generation" || -n "$plan_revision" ) ]]; then
        [[ "$generation" =~ ^[1-9][0-9]*$ ]] ||
          fail "$open_file $kind row $id has invalid generation: $generation"
        [[ -n "$plan_revision" ]] ||
          fail "$open_file $kind row $id has empty plan_revision"
      fi
    fi
    if [[ "$kind" == "consensus" ]]; then
      [[ "$parent" != "n/a" ]] ||
        fail "$open_file consensus row $id has no parent task"
      [[ "$revision" != "n/a" ]] ||
        fail "$open_file consensus row $id has no revision"
      [[ "$to_agent" == codex* ]] ||
        fail "$open_file consensus row $id does not target Codex"
      [[ "$to_agent" != "$from_agent" ]] ||
        fail "$open_file consensus row $id targets its own sender"
      [[ "$from_role" == "reviews" && "$to_role" == "reviews" ]] ||
        fail "$open_file consensus row $id is not reviews-to-reviews"
    fi
    if [[ "$kind" =~ ^(subtask|integration)$ ]]; then
      [[ "$from_role" == "reviews" ]] ||
        fail "$open_file $kind row $id was not dispatched by reviews"
    fi
    [[ "$to_role" =~ ^(reviews|eng|test|discuss|any)$ ]] ||
      fail "$open_file row $id has invalid to_role: $to_role"
    [[ -n "$to_agent" ]] ||
      fail "$open_file row $id has empty to_agent"
    [[ -n "$to_model" ]] ||
      fail "$open_file row $id has empty to_model"
    [[ "$from_role" =~ ^(reviews|eng|test|discuss)$ ]] ||
      fail "$open_file row $id has invalid from_role: $from_role"
    [[ -n "$from_agent" ]] ||
      fail "$open_file row $id has empty from_agent"
    [[ -n "$from_model" ]] ||
      fail "$open_file row $id has empty from_model"
    [[ -n "$cli" ]] || fail "$open_file row $id has empty cli"
    [[ -n "$request" ]] || fail "$open_file row $id has empty request"
    [[ -f "docs/agents/discuss/$thread" ]] ||
      fail "$open_file row $id points at missing thread: $thread"
  done < "$open_file"
}

check_runtime_registry() {
  local registry="docs/agents/RUNTIME.md"
  local line id role agent model cli session worktree state capabilities heartbeat

  [[ -f "$registry" ]] || return
  grep -q '^| id | role | agent | model | cli | session | worktree | state | capabilities | heartbeat |$' "$registry" ||
    fail "$registry is missing the expected table header"

  while IFS= read -r line; do
    [[ "$line" == \|* ]] || continue
    [[ "$line" == "| id "* ]] && continue
    [[ "$line" == "|---"* ]] && continue

    line="${line#|}"
    line="${line%|}"
    IFS='|' read -r id role agent model cli session worktree state capabilities heartbeat <<< "$line"
    id="$(trim "$id")"
    role="$(trim "$role")"
    agent="$(trim "$agent")"
    model="$(trim "$model")"
    cli="$(trim "$cli")"
    session="$(trim "$session")"
    worktree="$(trim "$worktree")"
    state="$(trim "$state")"
    capabilities="$(trim "$capabilities")"
    heartbeat="$(trim "$heartbeat")"

    [[ "$id" =~ ^[[:alnum:]_.@/-]+$ ]] ||
      fail "$registry has invalid id: $id"
    [[ "$role" =~ ^(reviews|eng|test|discuss|any)$ ]] ||
      fail "$registry row $id has invalid role: $role"
    [[ -n "$agent" ]] || fail "$registry row $id has empty agent"
    [[ -n "$model" ]] || fail "$registry row $id has empty model"
    [[ -n "$cli" ]] || fail "$registry row $id has empty cli"
    [[ -n "$session" ]] || fail "$registry row $id has empty session"
    [[ -n "$worktree" ]] || fail "$registry row $id has empty worktree"
    [[ "$state" =~ ^(idle|busy|down)$ ]] ||
      fail "$registry row $id has invalid state: $state"
    [[ -n "$capabilities" ]] ||
      fail "$registry row $id has empty capabilities"
    [[ -n "$heartbeat" ]] ||
      fail "$registry row $id has empty heartbeat"
  done < "$registry"
}

check_role_notes() {
  local role file base

  shopt -s nullglob
  for role in reviews eng test; do
    for file in "docs/agents/$role"/*.md; do
      base="$(basename "$file")"
      [[ "$base" == "README.md" ]] && continue

      [[ "$file" =~ ^docs/agents/$role/[0-9]{4}-[0-9]{2}-[0-9]{2}_[0-9]{4}_.+\.md$ ]] ||
        fail "$file does not match note filename convention"
      grep -q "^- role: $role$" "$file" ||
        fail "$file is missing '- role: $role'"
      grep -Eq '^- agent: .+$' "$file" ||
        fail "$file is missing '- agent: ...'"
      grep -Eq '^- model: .+$' "$file" ||
        fail "$file is missing '- model: ...'"
      grep -Eq '^- cli: .+$' "$file" ||
        fail "$file is missing '- cli: ...'"
      grep -Eq '^- status: (done|open)$' "$file" ||
        fail "$file is missing '- status: done|open'"
      grep -q '^## Summary$' "$file" ||
        fail "$file is missing '## Summary'"
      grep -q '^## Pointers$' "$file" ||
        fail "$file is missing '## Pointers'"
      if grep -Eq '^- parent: .+$' "$file"; then
        grep -Eq '^- subtask: .+$' "$file" ||
          fail "$file has parent but is missing '- subtask: ...'"
        if [[ "$role" == "eng" ]]; then
          grep -Eq '^- base_revision: .+$' "$file" ||
            fail "$file has parent but is missing '- base_revision: ...'"
          grep -Eq '^- started_at: .+$' "$file" ||
            fail "$file has parent but is missing '- started_at: ...'"
          grep -Eq '^- completed_at: .+$' "$file" ||
            fail "$file has parent but is missing '- completed_at: ...'"
          grep -q '^## Requirement$' "$file" ||
            fail "$file has parent but is missing '## Requirement'"
          grep -q '^## Result$' "$file" ||
            fail "$file has parent but is missing '## Result'"
        fi
        if [[ "$role" == "test" ]]; then
          grep -Eq '^- revision: .+$' "$file" ||
            fail "$file has parent but is missing '- revision: ...'"
        fi
      fi
    done
  done
  shopt -u nullglob
}

check_discuss_threads() {
  local file base

  shopt -s nullglob
  for file in docs/agents/discuss/*.md; do
    base="$(basename "$file")"
    [[ "$base" == "README.md" || "$base" == "OPEN.md" ]] && continue

    [[ "$file" =~ ^docs/agents/discuss/[0-9]{4}-[0-9]{2}-[0-9]{2}_[0-9]{4}_.+\.md$ ]] ||
      fail "$file does not match discuss filename convention"
    grep -Eq '^- status: (open|done|superseded|cancelled)$' "$file" ||
      fail "$file is missing '- status: open|done|superseded|cancelled'"
    if grep -Eq '^- to_role: ' "$file"; then
      grep -Eq '^- to_agent: .+$' "$file" ||
        fail "$file has to_role but is missing '- to_agent: ...'"
      grep -Eq '^- to_model: .+$' "$file" ||
        fail "$file has to_role but is missing '- to_model: ...'"
    fi
    if grep -Eq '^- kind: ' "$file"; then
      grep -Eq '^- kind: (question|consensus|subtask|integration|regression|task|checkpoint)$' "$file" ||
        fail "$file has invalid '- kind: ...'"
      grep -Eq '^- parent: .+$' "$file" ||
        fail "$file has kind but is missing '- parent: ...'"
      grep -Eq '^- subtask: .+$' "$file" ||
        fail "$file has kind but is missing '- subtask: ...'"
      grep -Eq '^- depends_on: .+$' "$file" ||
        fail "$file has kind but is missing '- depends_on: ...'"
      grep -Eq '^- revision: .+$' "$file" ||
        fail "$file has kind but is missing '- revision: ...'"
      if grep -q '^- kind: consensus$' "$file"; then
        grep -Eq '^- consensus: (open|reached)$' "$file" ||
          fail "$file consensus thread is missing open|reached state"
        if grep -q '^- status: done$' "$file"; then
          grep -q '^- consensus: reached$' "$file" ||
            fail "$file closed consensus without '- consensus: reached'"
        fi
      fi
      if grep -Eq '^- kind: (subtask|integration|regression)$' "$file" &&
          grep -q '^- status: done$' "$file"; then
        grep -q '^## Result -- ' "$file" ||
          fail "$file closed runnable work without a Result event"
        grep -q '^- outcome: pass$' "$file" ||
          fail "$file closed runnable work without '- outcome: pass'"
      fi
    fi
    if grep -Eq '^- role: ' "$file"; then
      grep -Eq '^- agent: .+$' "$file" ||
        fail "$file has role but is missing '- agent: ...'"
      grep -Eq '^- model: .+$' "$file" ||
        fail "$file has role but is missing '- model: ...'"
      grep -Eq '^- cli: .+$' "$file" ||
        fail "$file has role but is missing '- cli: ...'"
    fi
    grep -Eq '^## (Post|Summary)' "$file" ||
      fail "$file is missing a post or summary section"
  done
  shopt -u nullglob
}

check_no_agent_artifacts() {
  local artifacts
  artifacts="$(find docs/agents -type f \
    \( -name '*.jsonl' -o -name '*.log' -o -name '*.png' -o -name '*.ply' -o -name '*.txt' \) \
    -print)"
  [[ -z "$artifacts" ]] || fail "raw artifacts found under docs/agents: $artifacts"
}

check_architecture_rules() {
  grep -q 'docs/agents/README.md' .cursor/rules/agent-logs.mdc ||
    fail ".cursor/rules/agent-logs.mdc should point at docs/agents/README.md"
  grep -q 'docs/architecture/perception_architecture.md' .cursor/rules/ros2-node-structure.mdc ||
    fail ".cursor/rules/ros2-node-structure.mdc should point at perception_architecture.md"
  grep -q 'docs/architecture/sensor_data_pipeline.md' .cursor/rules/perception-data-pipeline.mdc ||
    fail ".cursor/rules/perception-data-pipeline.mdc should point at sensor_data_pipeline.md"
  grep -q 'docs/architecture/motion_compensation.md' .cursor/rules/sensor-frames-and-timing.mdc ||
    fail ".cursor/rules/sensor-frames-and-timing.mdc should point at motion_compensation.md"
  grep -q 'docs/architecture/container_geometry.md' .cursor/rules/container-geometry.mdc ||
    fail ".cursor/rules/container-geometry.mdc should point at container_geometry.md"
}

check_executables() {
  require_executable scripts/agent_complete.sh
  require_executable scripts/agent_mailbox.py
  require_executable scripts/agent_start.sh
  require_executable scripts/agent_flow_metrics.py
  require_executable scripts/agent_notify.sh
  require_executable scripts/agent_register.sh
  require_executable scripts/agent_scheduler.py
  require_executable scripts/agent_poll_self.py
  require_executable scripts/poll_eng_completed.py
  require_executable scripts/test_agent_lifecycle_smoke.sh
  require_executable scripts/test_agent_mailbox_freshness.sh
  require_executable scripts/test_agent_scheduler_poller_freshness.sh
  require_executable scripts/stop_sim.sh
  require_executable scripts/check_agent_contract.sh
}

check_required_files
check_git_baseline
check_open_mailbox
check_runtime_registry
check_role_notes
check_discuss_threads
check_no_agent_artifacts
check_architecture_rules
check_executables

if (( failures > 0 )); then
  echo "agent contract check failed: $failures issue(s)" >&2
  exit 1
fi

echo "agent contract check passed"
