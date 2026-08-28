# Phase 7 process log — manual panel / orchestrator parity

Chronological. Hypothesis, what was done, what was observed, what was rejected.

## 1. What the audit found

The panel was not missing features. It was a **second implementation** of the
orchestrator state machine, and the two had drifted. Comparing the two files
state by state produced nine behavioural differences, all in the panel's favour
of "simpler":

| State | Orchestrator | Panel |
|---|---|---|
| DetectPickupBox | pickup observe pose, retries, plausibility gate, `sync_detected_pickup_box` | one attempt, **falls back to spawner ground truth**, `sync_pickup_box` |
| ExecPick | vacuum at `attach`, post-pick staging pose | vacuum at `retreat`, no staging |
| PlanPlace | re-runs placement with the payload attached | uses the pre-pick estimate |
| ExecPlace | backup candidate on pre-insertion failures; release at `descend` + settle | no backup; release after every segment |
| UpdateOccupancy | verify → **finalize** → margin → overlap → map → reconcile → commit | `add_placed` then **`clear_current_box`** |

The last one is the bug that invalidated E16 and E17: the panel deletes the
Gazebo model it just placed.

## 2. Rejected: share the gate logic between two state machines

The obvious smaller change is to extract the gates into modules both files
import. Rejected: it leaves two state machines, so ordering bugs — which is
what six of the nine differences actually are — remain expressible. Sharing a
`commit_slot()` helper does not stop a panel from calling it after retreat
instead of before.

## 3. Rejected: make the panel a pure front end with no direct calls at all

Cleaner still, but it removes the one thing the panel is genuinely good for:
poking a single service while debugging (go to a pose, clear the octomap, read
map stats). Instead the probe tab stays, is labelled non-evidence in the UI, and
**taints the session** on first use.

## 4. Chosen: a step gate in front of the state machine

The constraint that drove the design: the automated acceptance path must not
change at all. So the state bodies were not touched and `run()` was not split
into per-state functions — that refactor would have had to rewrite every
`continue` in a 500-line `elif` chain.

The whole change to `run()` is three lines at the top of the loop body:

```python
state = self._await_step_permission(state)
if state == "Idle":
    break
```

In auto mode `_await_step_permission` returns before consulting any control
state, which the test asserts by checking the gate executed zero states after a
full automatic run.

## 5. Abort had to return Idle, not raise

First sketch had abort raise out of `run()`. That can abort **between** the
scene write and the map write, leaving exactly the half-committed state the
whole commit chain exists to prevent. Returning `Idle` drops the loop into its
normal terminal path instead, so an abort can only take effect on a state
boundary. Test: abort while paused before `ExecPlace`, assert `add_placed` and
`_remap_placed_box` were never called.

## 6. The harness had no producer

While wiring the panel it became clear that `active_loading_bag_harness.py`
consumes six record kinds across thirteen gates, but nothing in the tree ever
wrote them — the only records were hand-authored fixtures in its own unit test.
Adding `untainted_session` here makes fourteen. The
orchestrator logged one kind (`PLACEMENT_COMMIT`) to the console and the matrix
scraped it back out with a regex.

So the recorder writes all six. Two details that only showed up in practice:

* `cycle_sec` was missing from the log record entirely, so the `cycle_budget`
  gate had never been evaluated on a real run. It is measured spawn → commit.
* Failure records are emitted from `publish_status` rather than from ~30 call
  sites, because Idle is both the failure sink and the normal end of a run.
  That needed an explicit list of terminal-success messages; anything else that
  lands in Idle is a failure. A message with no leading uppercase token is
  recorded as `UNCLASSIFIED` rather than being folded into a neighbouring class.

## 7. Parity is checked at the state machine, not only by eye

The end-to-end version of this gate needs a human stepping a Gazebo run. That
is worth doing but it is not a regression guard. So the same fixture is driven
twice through the real `run()` — once automatically, once one state at a time by
a thread that clicks Step whenever the gate reports paused — and the two event
files are compared after dropping timestamps, durations and `run_mode`.

`run_mode` has to be in that dropped set: it is the one field that differs
between the two runs by construction, and proving they agree despite it is the
point.

## 8. The guard against re-drifting is source level

A reviewer will not catch the panel growing a `plan_motion` call back. So the
check is mechanical: parse the panel, find every string literal that looks like
a ROS name, and reject any that names an executing or committing service unless
it sits inside a `_probe*` method. The checker is also run against a synthetic
violating snippet, because a guard nobody has seen fail is a guard nobody should
trust.

The same test compares the panel's fallback state list against the
orchestrator's `STATES` tuple by parsing both files, so the two cannot drift
even in the branch where the import fails.

## 9. Real run: wiring works, EX1 still blocks

One headless single-box run, seed 0, warn level:

* events file written, 4 records
* console log and events file agree on placed count (`evidence_consistent`)
* run stops at `Cartesian descend release settle failed after hold 3.01s/0.0318`

That is P0-EX EX1, unchanged, at the same magnitude phase 5 saw (0.0313 /
0.0302 against a 0.03 tolerance). Which is the expected result — this change was
not supposed to move it — but it also means the three gates that need a
committing run stay open, and the bundle stays `running` rather than `passed`.
