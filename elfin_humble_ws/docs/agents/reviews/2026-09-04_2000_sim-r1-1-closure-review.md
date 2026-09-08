# SIM-R1-1 Closure Review

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done
- parent: SIM-R1-20260904
- subtask: SIM-R1-1
- revision: e40df9801dcaae39ddd445bc7238a29673751a0b

## Summary

Revision `e40df98` builds and its submitted Gate R1 tests pass, but closure is
rejected because the implemented contracts violate existing production
requirements under restart, asynchronous completion, nested mutable data, and
failure recovery. The canonical SIM-R1-1 thread is reopened; SIM-R1-2 remains
blocked. These are corrections to approved behavior, not new scope or weaker
acceptance.

## Findings

### F1 - Critical: operation completions are not correlated

`OperatorEvent` and `operation_succeeded()` carry no operation/request ID.
Any delayed or unrelated success event advances the current state; in
`COMMIT_AND_VERIFY`, it can commit a synthetic box identity. Effects may also
contain multiple ordered service calls while the state changes before either
result is correlated. Add coordinator-owned operation IDs and pending-operation
state; accept success/failure only for the exact pending effect. Carry the real
detected/placed box identity through commit and prove retries cannot double
count it.

### F2 - High: pickup authorization IDs repeat after restart

`_enter_wait_pickup_ready()` derives IDs only from a sequence that resets to
zero, so every process starts with `pickup-000001`. A delayed operator/UI event
from the previous session can authorize the new session. Inject and retain a
fresh session ID at explicit Start and include it in every request ID; reject
cross-session events.

### F3 - High: WAIT_PICKUP_READY does not guarantee pick observe

Pre-pick failures call `_enter_wait_pickup_ready()` directly. Failures after a
motion attempt can therefore publish a new pickup prompt without a successful
correlated `GoToRobotPose(pick_observe_pose)`. Recovery paths enter
`RETURN_PICK_OBSERVE` but do not consistently emit that action. Make every path
to `WAIT_PICKUP_READY` pass through and correlate successful return-to-observe.

### F4 - High: exploration contracts are only shallowly immutable

`_freeze_mapping()` freezes top-level key/value pairs while retaining nested
dict/list references; `_plain_mapping()` returns those same references. Source
mutation and mutation of `to_dict()` output both alter contract state. Deeply
freeze/copy supported plain values and reject unsupported/non-serializable
objects. Add nested mutation tests for context, snapshot, candidate, proposal,
outcome, and effect payloads.

### F5 - Medium: acquisition stamps lose exact identity

`ExplorationSnapshot` and `ViewOutcome` store acquisition stamps as float even
though the ROS interfaces use `(sec,nanosec)` and integration requires exact
frame correlation. Use a validated plain integer sec/nanosec value type and
test adjacent nanosecond stamps as distinct.

## Acceptance

- Stale, duplicate, wrong-operation, and out-of-order completion events cannot
  advance state, execute release, or commit a box.
- Real box identity is committed exactly once; attempts and synthetic sequence
  numbers are not placement identity.
- Request IDs are unique across process sessions and cross-session pickup-ready
  events fail closed.
- Every entry into `WAIT_PICKUP_READY` follows a successful correlated return
  to `pick_observe_pose`, including failure and recovery paths.
- All contract inputs and outputs are deeply immutable/defensively copied and
  restricted to serializable plain values.
- Acquisition timestamps retain exact integer sec/nanosec identity.
- Existing seven tests remain green and focused regressions cover each finding.
- `luggage_msgs` and `luggage_planning` build, ROS-free imports pass, agent
  contract passes, and the same canonical thread receives the replacement
  passing Result revision.

## Pointers

- `docs/plans/sim_r1_production_orchestrator_exploration.md`
- `docs/architecture/production_orchestration.md`
- `src/luggage_planning/luggage_planning/orchestration_contracts.py`
- `src/luggage_planning/luggage_planning/exploration_contracts.py`
- `src/luggage_planning/test/test_sim_r1_contracts.py`
- `docs/agents/discuss/2026-09-04_1809_sim-r1-1-contracts.md`

## Open

- `codex/gpt-5.5` must remediate F1-F5 and close SIM-R1-1 before SIM-R1-2 is
  dispatched.
