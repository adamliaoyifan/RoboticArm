# 2026-11-12 -- Planning route package layout

- status: approved
- authorization: User requests grouping files belonging to each planning route.
- parent: PLANNING-ROUTE-LAYOUT-20261112
- subtask: PACKAGE-LAYOUT
- generation: 1
- owner: eng/codex/gpt-6
- base_revision: dfbcc39fa8c5e58defc46d7fdb33f3f30bb38e35

## Scope and ownership

One owner implements and verifies the complete mechanical migration. Group72
currently flat modules into `joint_operation` (29), `placement_prior` (19),
`routing` (14), and `atlas` (10). Shorten redundant filename prefixes inside each
package. Keep ROS services, launch names, executable names, algorithms, config
values, and active route selection unchanged. Keep shared geometry, attachment,
observation and motion interfaces outside route-specific packages. Add a route
entry/dependency index at the package root and link it from architecture.

Update active repository Python imports, dynamic module strings and actual source
path references. Historical plans, discussion and evidence remain immutable.
Do not add a second copy, import hook, broad path fallback or renamed version of
an algorithm. Internal Python module paths change explicitly; ROS entry wrappers
remain stable. A compatibility shim is not needed for repository callers once
all those callers are migrated.

The workspace contains concurrent tracked changes and untracked runtime modules.
Snapshot all touched working bytes and HEAD bytes before migration. Transform both
independently: stage only transformed HEAD content for previously tracked files;
preserve pre-existing dirty/untracked implementation content in the working tree
at its new location. Do not absorb concurrent algorithms into this refactor commit.
Record exact dirty input hashes for working-tree validation; do not claim a clean
checkout is certified when it depends on existing untracked implementation.

## Acceptance

Environment: ROS Humble, existing production prefix, current shared source state.
No Gazebo/controller launch, no scored product cycle. Freeze selected case list
and exact old-to-new module mapping before edits. Run each deterministic workload
once before/after migration (AND across cases, no stochastic reliability claim).

1. AST inverse-normalization of every changed Python source and moved module must
   reproduce its frozen pre-migration AST exactly, except explicit package/docs/
   install-check wiring. No planning behavior, margins, ordering or defaults change.
2. Active source/import scan must find zero remaining old qualified module imports
   for the72 migrated names. All72 new modules compile/import in the installed
   prefix, resolved source bytes equal installed bytes; no old installed flat
   module may mask an import. Missing untracked dependencies are explicit evidence.
3. Baseline workload: non-native joint-operation and placement-prior tests, atlas
   proposal/runtime tests, route-producer factory, direct-first and aperture tests.
   Freeze file list in evidence. Require zero new failed/error case IDs relative
   to baseline, identical collected case IDs, and all new layout checks passing.
   Existing failures/skips are named and remain unvalidated, not counted as pass.
4. Build all13 production packages, zero source changes during build. Installed
   checker passes. Native MoveIt aperture (3), preparation G (16), O observer (10),
   connector (10), endpoints (7), certification (10) regressions pass in fresh,
   isolated domains173/174/175, preserving requests/responses and shutdown proof.
5. Stable ROS wrapper names and launch argument defaults are unchanged. Whole
   Gazebo, observation coverage, real hardware and latency remain not_evaluated.

## Evidence and procedure

Root: `docs/status/evidence/planning_route_layout/20261112/`.
T0: frozen map/test list, HEAD/worktree hashes, dirty state, exact commands, counts,
timings and verdict. T1: per-file import/AST preservation and installed-origin
results. T2: failed pytest JUnit/log plus frozen input bytes; native failures also
include request/response CDR, model/config and bounded node logs before retry.
Capture one failed stage boundary; no continuous simulator or sensor dump.
Max100MiB per native fixture,512MiB total; original bundles remain immutable.
Replay commands and provenance manifest accompany results. Behavioral failures
after migration are repaired only when demonstrated to be caused by the migration.
