# 2026-11-12 -- P2 shared placement admission and directed candidates

- status: approved
- dispatch_ready: yes
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-PRODUCTION-P2
- generation: 1
- owner: codex/gpt-6
- base_revision: ea19c48c6f9cd9e0fa049a6ed3b3f2b6ba02dfff
- authorization: User execution instruction, four headings, matching map source,
  final side contact without penetration, and corridor-ranking-only answer.

## Scope and decisions

One ROS-free packing query for ComputePlacement and complete prepare. Read
container-absolute voxel origin from the admitted 3D source (never the legacy
floor-relative surface zero). Preserve 0/90/180/270 directed targets through
retention and wire output. A box footprint may reuse geometric computation, but
distinct headings must survive. Stop reconstructing missing headings in prepare.
Same-source known-free proof covers the entire candidate volume, including
floor-prior supported candidates; missing/short/unknown/inactive evidence refuses.
Use integral-volume queries for bounded per-candidate work, with no occupied-cell
erosion in the new shared production query. Face contact is not penetration.
Final nominal side gap defaults to zero as authorized; uncertainty/carry/release
policies remain separate and unchanged. Geometric hull/support/overlap are hard
gates. Straight insertion corridor and box-center atlas evidence are ranking
information, not proof that every legal offset grasp is impossible. Actual grasp
atlas seeds and full-model native trajectories retain their existing authority.
Do not inject requested planned boxes as measured occupancy in the shared path.
Existing explicitly selected offline/structural compatibility paths remain
identified; no claim that those are the new default operation client.

## Subtask and acceptance

| ID | Owner | Depends | Base | Scope | Required tests | Commit evidence |
|---|---|---|---|---|---|---|
| ST-4-PRODUCTION-P2 | codex/gpt-6 | ST-4-PRODUCTION-P1 | ea19c48c | Above | Below | Pending |

Environment: system Python + Humble through production/run.sh, explicit test-only
source runner, no nodes/controllers/Gazebo. One deterministic fixture matrix;
AND over all cases measures contract coverage, not operation reliability.

1. Frozen map-owner callback reproducer changes from refusal to authorization
   for known-free volume at absolute floor .6m; occupied and unknown correct
   absolute layers still refuse. Source-resolution assertion remains enforced.
2. Shared-query tests: asymmetric and square boxes, all four headings, bounded
   retention including top_n=4, positive/negative absolute floors, 1/2cm map
   sources, face touching versus positive overlap, blocked straight corridor
   retained with ranking diagnostic, missing/short/unknown/inactive 3D refusal.
3. Deterministic integral verifier versus scalar zero-tolerance reference on
   random rectangular queries, including exact boundaries and uncovered boxes:
   every admission verdict matches; no unmeasured speedup claim. Record counts,
   timings, seed, source/fixture hashes and memory shape. Occupied and unknown
   reason precedence may differ when both occur; both must reject.
4. Actual Compute callback and prepare candidate consumer tests prove same
   candidate geometry/heading/volume admission; directed slot serialization
   retains heading and prepare uses balanced depth fallback per heading without
   inventing additional flips. Caller planned-box fallback is refused when no
   corresponding measured ledger is available.
5. Run focused shared query + solver + volume + map-owner + prior request +
   complete preparation tests via `scripts/production/run.sh python3
   scripts/production/test_source.py -q <named files> --junitxml=<evidence>`.
   Select exact existing preparation fixtures after discovery, record command.
   Every selected regression must pass; preserve failures before repair/rerun.
6. Rebuild with `scripts/production/build.sh --evidence <root>/build`, then
   `scripts/production/run.sh python3 scripts/production/check_install.py`:
   all closure packages build, installed-origin/content checks pass. Diff check
   on changed code/docs excluding immutable raw evidence has no whitespace errors.

## Evidence and revision rule

Root: docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PRODUCTION-P2/run_01/.
Freeze selected inputs and exact revision/dirty listing before tests. Preserve
existing unrelated dirty changes, stage only owned paths. Already-reproduced
datum failure is retained under P1/run_01/datum-regression.{log,xml} and copied
or linked in this manifest; do not rewrite P1's bounded result.

| Boundary | Trace | Trigger | Preserved input/output | Window/budget | Replay |
|---|---|---|---|---|---|
| Map projection -> verifier | origin/resolution/dimensions/identity | wrong verdict | lossless fixture, test/JUnit, stack | one callback, <=16MiB | source runner exact test |
| Shared query -> candidates | heading/count/reason/timing | invariant/test failure | fixture+params, retained candidates, source hashes | one query, <=16MiB | focused query test |
| Consumer -> requested slot | target/candidate IDs, refusal | mismatch | request/fixture and output/exception | one request, <=16MiB | consumer test |
| Build/install | package summary/hash/timing | nonzero/mismatch | logs, receipt, before/after input hashes | <=32MiB | build/check command |

Total retained <=128MiB, no raw whole-run capture. Capture health requires
complete logs, JUnit and replay source before rerun. No statistical motion metric
is scored; whole-model simulation and real ULD baseline remain later integration
gates. Teardown not_needed (no processes launched beyond tests/build).
