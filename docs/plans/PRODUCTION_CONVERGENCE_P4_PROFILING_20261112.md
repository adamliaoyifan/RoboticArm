# 2026-11-12 -- Complete preparation budget and stage profiling

- status: approved
- dispatch_ready: yes
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-PRODUCTION-P4-PROFILING
- generation: 1
- owner: codex/gpt-6
- base_revision: c47e7d3430ba4ac25984219cadfc5f91bf708b24
- authorization: User chooses complete-model successful baseline plus stage timing before production latency thresholds.

## Scope

Remove duplicated unexplained10s admission caps from root/core preparation.
Declare a finite positive max_preparation_budget_s ROS parameter, default300s as
an explicit diagnostic watchdog only (not an SLA or a successful baseline).
Each request still provides its own finite positive budget within that policy;
one monotonic deadline covers acquisition, candidates, planning and revalidation.
Expose override in prepare launch. Core and node share the admitted cap. Native
planner reservations, per-port limits, search caps and all collision gates remain.

Record bounded stage timing through existing trace_json/result.trace; no new ROS
interface/version. Root: admission, frozen observation, map, base scene, G volume
if needed, private scene, placement, whole preparation, issuance. Inner preparation:
proposal/quality, bound candidate, atlas augmentation, revalidation. Native planner:
local paths, align, carry, time parametrization, complete-chain certification.
Every timing has scope/name, elapsed seconds and completed/error outcome; attempted
failed stages remain observable. Do not add nested stage durations as a total.
Full elapsed_s remains root total. No motion or vacuum/map writes are introduced.
This prerequisite is not the complete P4 default client/launch migration or P5.

## Acceptance

Environment system Python/Humble production wrapper with source runner. One
AND over deterministic fixtures measures contract coverage, not cycle reliability.
Every selected case passes, zero skips; preserve failures before repair/rerun.

1. Shared budget: positive values above10 accepted within configured cap; zero,
   negative, NaN, infinity and above-cap refused; invalid configured cap rejected
   at startup/construction. Root and core agree under same configuration. Expired
   deadline returns inactive plan, with no trajectory or side effects.
2. Real root callback fixture at budget30 reaches domain admission with cap60;
   cap20 refuses before domain lookup. Exceptions still emit root failed stage
   trace, preserve total elapsed and mandatory lock release/evidence response.
3. Deterministic fake clock verifies stage duration/outcome for success, returned
   refusal, raised error and nonoverlap semantics; no hidden sleeping/timing gate.
4. Existing pure planner positive branch recovery/certification/timeout/refusal
   fixtures preserve verdict and report attempted local/connection/certification
   stage durations. Same chosen contacts, winding and trajectory bytes unaffected.
5. Focused tests: new preparation profiling tests plus joint operation planner,
   contracts, contacts, candidates, observer, bindings and placement consumers.
   Exact file list/command in evidence. No native external process or Gazebo.
6. Rebuild production closure; installed origin/hash checks include new module.
   Full package build and all installed checks pass; owned diff whitespace clean.

## Evidence

Root docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PRODUCTION-P4-PROFILING/run_01/.
Freeze revision/dirty/source inputs and exact fixtures before tests. Changes remain
separate from existing dirty/untracked runtime work. Stage only owned deltas.

| Boundary | T1 trace | Trigger | Preserved payload | Window/budget | Replay |
|---|---|---|---|---|---|
| Budget -> root/core | cap/request/deadline/reason | mismatch/timeout | fixture, trace, exception, source | one request <=16MiB | named pytest case |
| Planner stages | stage/outcome/seconds | verdict or timer mismatch | deterministic port recipe, trace, JUnit | one candidate <=16MiB | named planner case |
| Build/install | hashes/package status | failure | logs/receipt/frozen input hashes | <=32MiB | production build/check |

Total<=128MiB; lossless deterministic fixture recipes replace repetitive wire dumps.
Capture health requires complete source/command/log/JUnit and replay pointers before
rerun. No statistical claim; no simulation launch. Teardown not_needed.
