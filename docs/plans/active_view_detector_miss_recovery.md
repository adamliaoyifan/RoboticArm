# ACTIVE-VIEW-1 detector-miss active viewpoint recovery

- status: approved
- parent: ACTIVE-VIEW-RECOVERY-20260915
- subtask: ACTIVE-VIEW-1
- owner: eng/claude/glm-5.3
- base_revision: afe851efa1df14c4acedeacabd0f609d4e5b4d48
- simulator: forbidden for this subtask

## Goal

Implement and document the ROS-free decision and candidate-selection slice for
pre-pick recovery when measured evidence indicates that cargo is present but
the current view cannot produce a planner-ready detection. The slice proposes
safe next views; it does not execute motion, change detector acceptance floors,
attach a payload, or certify the live robot path.

`ACTIVE-VIEW-1` is deliberately offline so it can run while PF-R7 generation 5
owns the simulator. ROS/action wiring, MoveIt execution, live simulation, and
hardware verification are deferred to a later `ACTIVE-VIEW-2` generation after
this contract passes review.

## Architecture boundary

- The policy is plain Python and ROS-free. It consumes immutable, serializable
  measured summaries and returns declarative recovery decisions/candidates.
- Online policy input must not contain Gazebo entity state, GT boxes, GT pose,
  eval-only IoU, fixture names, or a `sim_mode` branch.
- Production cargo acceptance remains `confidence >= 0.20`; the diagnostic
  presence-hint floor is `0.05` and must never create a semantic mask, cargo
  cloud, pick proposal, or success result.
- Recovery is legal only before payload attachment, with vacuum off, in the
  pre-pick detect/recovery state, after the current view is measured settled.
- The policy may rank candidates but never authorizes or executes motion. The
  later node/coordinator layer owns exact-stamp TF, geometry identity, IK,
  collision, joint-limit, cancellation, settle, and motion-result gates.
- Reuse `exploration_contracts` for immutable context/snapshot/proposal/outcome
  values, `constrained_view_planner` for admissibility/coverage ranking, and
  `cargo_nbv_planner` for bounded visited-candidate behavior. Small adapters or
  extensions are allowed; copying those modules into a parallel implementation
  is not.
- Do not cherry-pick the complete `3dca6f9` SIM-R1-2 change. It is a useful
  reference, but remains a separately owned unmerged work item.

## Input summary and health gate

Each evaluated observation window contains exactly three settled D555 frames
whose acquisition stamps span at most `0.50 s`. The plain input records:

- state, payload-attached and vacuum-commanded flags;
- exact acquisition stamps, frame ID, camera model ID, geometry hash and map
  revision;
- RGB/depth heartbeat, aligned-depth status, exact-stamp TF availability and
  measured robot-settled status;
- accepted production cargo detections at the unchanged `0.20` floor;
- diagnostic cargo hints produced at `0.05 <= confidence < 0.20`, including
  bbox, label and confidence, without semantic/pick authority;
- depth-foreground component summaries in the configured pickup workspace;
- cargo-cloud/geometry status, including point count, top-surface validity and
  `FULL_3D` readiness.

The window is healthy only when all three frames have the expected frame,
camera model and geometry identity; monotonically increasing stamps; RGB and
aligned depth present; exact-stamp TF present; and the robot settled. Missing,
stale, mismatched, non-finite or ambiguous fields return
`RECOVERY_EVIDENCE_INVALID` and must not propose a view.

## View-contains-cargo criterion

Declare `cargo_present` only when at least two of the three healthy frames meet
at least one of these measured, production-safe signals:

1. A diagnostic cargo hint has `0.05 <= confidence < 0.20`, bbox centre inside
   the configured pickup workspace projection, bbox width and height each at
   least `16 px`, and bbox area between `0.5%` and `80%` of the image; or
2. A single depth-foreground component inside the pickup workspace has at
   least `500` aligned valid-depth pixels, projected area between `0.5%` and
   `80%` of the image, and measured height above its local support plane in
   `[0.03 m, 0.80 m]`.

The two agreeing frames must spatially associate at bbox/component IoU
`>= 0.30`. A clean/empty view, one-frame transient, workspace-external signal,
robot-only component, or unsupported component is not evidence of cargo and
must return `NO_CARGO_EVIDENCE` with no candidate.

## Incomplete-observation criterion

An otherwise healthy `cargo_present` window is incomplete when either:

- all three frames have zero accepted production cargo detections; or
- an accepted detection exists but no planner-ready geometry is produced and
  at least one of these measured defects holds: bbox border margin `< 8 px`,
  valid aligned-depth ratio inside the associated bbox `< 0.80`, cargo cloud
  point count `< 500`, `top_surface_valid=false`, or support mode is not
  `FULL_3D`.

An accepted detection with planner-ready `FULL_3D` geometry is immediate
`RECOVERY_SUCCEEDED`, not a reason to move. Cargo presence without one of the
incomplete conditions is `RECOVERY_NOT_REQUIRED`.

## Candidate adjustment policy

Use a configurable candidate library expressed relative to the calibrated
`pickup_observe` camera pose and a measured look-at point. The default offline
matrix is:

| Candidate | Camera translation from nominal | Look adjustment |
|---|---|---|
| `lateral_left` | `(0.00, +0.10, 0.00) m` | point at measured cargo centroid |
| `lateral_right` | `(0.00, -0.10, 0.00) m` | point at measured cargo centroid |
| `raised_center` | `(0.00, 0.00, +0.08) m` | point at measured cargo centroid |
| `yaw_left` | `(0.00, 0.00, 0.00) m` | orbit/look yaw `+12 deg` |
| `yaw_right` | `(0.00, 0.00, 0.00) m` | orbit/look yaw `-12 deg` |

Reject non-finite, duplicate, already visited, workspace-external, camera
envelope violating, or configuration-invalid candidates before ranking. Rank
remaining candidates deterministically by frontier/visibility coverage,
expected observation gain, motion distance, and repeat cost; ties resolve by
the table order above. A candidate carries its ID, camera pose, score terms,
source stamps, geometry hash and map revision so the later coordinator can
fail closed before motion.

The policy must not synthesize a joint trajectory. Candidate feasibility is an
explicit `unknown` until the later coordinator records the IK/collision/motion
gate outcome through `ViewOutcome`.

## Termination

Stop and return exactly one machine-readable reason when any condition occurs:

- `RECOVERY_SUCCEEDED`: unchanged production detection plus planner-ready
  `FULL_3D` geometry;
- `RECOVERY_NOT_REQUIRED` or `NO_CARGO_EVIDENCE`: no recovery motion allowed;
- `RECOVERY_EVIDENCE_INVALID`: health/identity/stamp failure;
- `RECOVERY_STATE_INVALID`: state left pre-pick, payload attached, vacuum on,
  cancellation, emergency stop, or operator abort;
- `RECOVERY_BUDGET_EXHAUSTED`: three executed recovery views or `15.0 s` from
  the first recovery decision;
- `RECOVERY_NO_GAIN`: two consecutive integrated views each improve best cargo
  confidence by `< 0.02` and bbox valid-depth ratio by `< 0.05`;
- `RECOVERY_CANDIDATES_EXHAUSTED`: no unused candidate remains.

Budget/exhaustion/no-gain are safe pre-pick failures: they must not emit pick,
vacuum or placement authority. Returning to nominal `pickup_observe` is a later
coordinator effect and is not executed by this policy.

## Workload and acceptance

Claude owns implementation, focused tests, failure repair, commit evidence and
closure. Work in a satellite worktree/branch created from the exact base
revision because Cursor is editing `src/` for PF-R7; keep mailbox edits in the
primary workspace.

Required workload:

1. Pure classification matrix: planner-ready detection, low-confidence hint,
   depth-only persistent component, empty view, one-frame transient,
   workspace-external hint, border-truncated detection, low valid depth,
   insufficient cargo cloud, non-FULL_3D geometry, stale/non-monotonic stamp,
   missing exact TF, identity mismatch, carrying/vacuum-on, cancellation and
   non-finite input. Assert the exact terminal reason and zero motion authority
   for every fail-closed case.
2. Candidate matrix: all five defaults, deterministic tie order, invalid and
   visited rejection, no duplicates, three-view cap, elapsed-time cap,
   two-view no-gain stop, outcome correlation, and reset between sessions.
3. Saved PF-R7 replay: `standard_00/01/02` must not start recovery because a
   production proposal exists; `standard_03/04/05` must classify as
   cargo-present/incomplete from measured inputs and propose a bounded first
   view. Offline GT/IoU may label expected results in the harness only and must
   be rejected if inserted into the policy input.
4. Contract compatibility: existing SIM-R1 exploration-contract tests remain
   green and serialized outputs contain schema ID, decision reason, source
   stamps, geometry hash, map revision, candidate ID and score terms.

Required commands, all without Gazebo:

```bash
PYTHONPATH=src/luggage_planning python3 -m pytest -q src/luggage_planning/test/test_active_view_recovery.py
PYTHONPATH=src/luggage_planning python3 -m pytest -q src/luggage_planning/test/test_active_view_recovery.py
PYTHONPATH=src/luggage_planning python3 -m pytest -q src/luggage_planning/test/test_active_view_recovery.py
PYTHONPATH=src/luggage_planning python3 -m pytest -q src/luggage_planning/test/test_sim_r1_contracts.py
python3 -m py_compile src/luggage_planning/luggage_planning/active_view_recovery.py
```

All three focused runs must pass with identical decision/candidate fixtures;
the contract regression must have zero failures. The owner must also run
`git diff --check` on the task branch.

## Artifacts and completion

- Commit the ROS-free implementation, tests, bounded replay fixture/adapter and
  configuration example on the task branch; do not merge to `master`.
- Write the owner eng note and append Claim/Result to the discuss thread using
  the lifecycle helpers. A passing Result points to the exact clean task commit.
- Write `docs/status/evidence/active_view/2026-09-15_active-view-1/summary.json`
  with base/commit, dirty-state, test counts, per-case terminal reason and
  candidate sequence. Store a bounded failed-case input/output bundle there
  before any rerun if a test or replay case fails.
- `ACTIVE-VIEW-1` passes only when every workload row above passes on the exact
  committed revision. It does not authorize live motion or close PF-R7.

## Deferred integration

`ACTIVE-VIEW-2` will be separately reviewed after this result and PF-R7 G5. It
will own perception diagnostic-hint publication, ROS/action orchestration,
MoveIt hard gates, measured settle, return-to-observe behavior, one bounded
sim campaign and later hardware verification.
