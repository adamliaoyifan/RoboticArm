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
    docs/agents/discuss/OPEN.md \
    docs/agents/reviews/README.md \
    docs/agents/eng/README.md \
    docs/agents/test/README.md \
    .cursor/rules/agent-logs.mdc \
    .cursor/rules/sim-lifecycle.mdc \
    .cursor/rules/perception-data-pipeline.mdc \
    .cursor/rules/ros2-node-structure.mdc \
    .cursor/rules/sensor-frames-and-timing.mdc \
    docs/architecture/README.md \
    scripts/agent_notify.sh \
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
  local line id to from cli thread question

  [[ -f "$open_file" ]] || return
  grep -q '^| id | to | from | cli | thread | question |$' "$open_file" ||
    fail "$open_file is missing the expected table header"

  while IFS= read -r line; do
    [[ "$line" == \|* ]] || continue
    [[ "$line" == "| id "* ]] && continue
    [[ "$line" == "|---"* ]] && continue

    line="${line#|}"
    line="${line%|}"
    IFS='|' read -r id to from cli thread question <<< "$line"
    id="$(trim "$id")"
    to="$(trim "$to")"
    from="$(trim "$from")"
    cli="$(trim "$cli")"
    thread="$(trim "$thread")"
    question="$(trim "$question")"

    [[ "$id" =~ ^Q-[0-9]{8}-[0-9]+$ ]] ||
      fail "$open_file has invalid id: $id"
    [[ "$to" =~ ^(reviews|eng|test|discuss|any)$ ]] ||
      fail "$open_file row $id has invalid to role: $to"
    [[ "$from" =~ ^(reviews|eng|test|discuss)$ ]] ||
      fail "$open_file row $id has invalid from role: $from"
    [[ -n "$cli" ]] || fail "$open_file row $id has empty cli"
    [[ -n "$question" ]] || fail "$open_file row $id has empty question"
    [[ -f "docs/agents/discuss/$thread" ]] ||
      fail "$open_file row $id points at missing thread: $thread"
  done < "$open_file"
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
      grep -Eq '^- cli: .+$' "$file" ||
        fail "$file is missing '- cli: ...'"
      grep -Eq '^- status: (done|open)$' "$file" ||
        fail "$file is missing '- status: done|open'"
      grep -q '^## Summary$' "$file" ||
        fail "$file is missing '## Summary'"
      grep -q '^## Pointers$' "$file" ||
        fail "$file is missing '## Pointers'"
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
    grep -Eq '^- status: (open|done)$' "$file" ||
      fail "$file is missing '- status: open|done'"
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
}

check_executables() {
  require_executable scripts/agent_notify.sh
  require_executable scripts/stop_sim.sh
  require_executable scripts/check_agent_contract.sh
}

check_required_files
check_git_baseline
check_open_mailbox
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
