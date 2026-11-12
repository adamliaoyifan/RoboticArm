# 2026-11-12 -- P3 source-resolution regional transport foundation

- status: approved
- dispatch_ready: yes
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-PRODUCTION-P3
- generation: 1
- owner: codex/gpt-6
- base_revision: 5ea0132b3c5f107241661d6d2382490fe33aa2e7
- authorization: Continue approved convergence; explicit spatial paging question and baseline-first latency answer.

## Scope and decisions

Implement a narrow ROS-free native-grid region/block contract in luggage_mapping,
used by O full-volume capture and explicit leased region reads. Integer cell
bounds are half-open, validated against original grid geometry; cursor starts at
zero separately for each region. Preserve resolution, container-absolute origin,
x-fastest ordering, unknown/inactive states and immutable snapshot identity.
Provide an identical block representation for slicing an already admitted frozen
O/G volume; G slicing saves no initial whole-volume RPC. Blocks expose coverage
explicitly and cannot masquerade as a complete volume. No cache or cross-revision
stitching is introduced. No arbitrary radius, coarsening, free filling or fixed
64-page limit. O page sizes/progress counts follow the exact region and owner limit.

This unit establishes real transport and shared coverage primitives, not the
later two-stage candidate ROI scheduler or regional-only native certification.
Current production capture still reads full surface and full volume under one
lease; full-model motion certification still requires complete coverage. Record
this limit explicitly. No changes to production latency policy in this unit:
user chose successful full-model baseline with stage timings before thresholds.
P4 must replace the unexplained 10-second ceiling with explicit configured budget.

## Acceptance

Environment: system Python/Humble through production wrapper, isolated source
runner. Deterministic contract coverage only, not an operation reliability gate.
AND over all fixture cases; one run per case, zero skipped or failed selected cases.

1. Region contracts: offset non-cubic region, full grid, 1/2cm sources, positive
   and negative absolute origins; exact x-fastest scalar parity for admitted O/G
   blocks. Reject empty/out-of-grid/fractional/bool bounds, malformed state/active
   arrays, absent identity and nonfinite/nonpositive geometry. Region never changes
   the underlying grid. Out-of-region lookup explicitly refuses coverage.
2. Real ROS ReadCells request/response types with a deterministic transport double:
   offset region spans multiple pages, same region bounds on every page, cursor
   zero then exact progress; one unchanged caller deadline. Reject changed identity,
   generation, stale/invalid response, short/long/empty/nonprogressing page,
   inconsistent done and invalid states. No partial block returned on failure.
3. Full 149x197x148 source grid produces exactly 4,344,244 cells in 67 owner pages
   at limit65,536, with no old 64-page refusal. Record count, page bounds, wall time
   and fixture hash. This is transport-fixture evidence, not measured live throughput.
4. Existing ObserverSnapshotPort full capture calls the same region reader and
   retains complete surface/volume agreement and mandatory lease release. Preserve
   focused observer conversion/release, common placement consumers, source launch
   and volume tests. Exact selected file list recorded before execution.
5. Rebuild scripts/production/build.sh into the one production prefix; all closure
   packages and installed-origin/content checks pass, including new region module.
   Diff check owned code/docs clean. No simulation, controllers or native processes.

## Evidence and revision rule

Root docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PRODUCTION-P3/run_01/.
Freeze exact base, dirty listing, source/fixture inputs before runs. Shared unrelated
changes remain unstaged. Preserve every failed test log/JUnit and fixture source
before repair/retry. Build artifact admission uses before/after hash identity.

| Boundary | T1 trace | Failure trigger | T2 payload | Window/budget | Replay |
|---|---|---|---|---|---|
| Grid -> region | original geometry, bounds, identity | invalid admission | exact fixture/test source, assertion | one fixture <=8MiB | named pytest case |
| Lease -> pages -> block | bounds, cursor, count, identity, deadline | malformed/incomplete page | deterministic source+seed, response recipe, traceback | one request <=16MiB | named pytest case |
| Full capture -> consumer | source and coverage counts | changed verdict/release | frozen test inputs, logs, JUnit | one test <=16MiB | source runner |
| Build/install | hashes, package status, timings | nonzero/mismatch | logs, receipt, input manifest | <=32MiB | production build/check |

Total retained <=128MiB. No raw repetitive cell dumps when exact deterministic
recipe is lossless. Capture health: complete inputs, command, log/JUnit, manifest
and hashes before rerun. Teardown not_needed; no external runtime started.
