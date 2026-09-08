# Platform-free PF-R5 Closure Plan

Date: 2026-09-05

- status: approved
- parent: PFH-R5-CLOSURE-20260905
- engineering_consensus: reached
- implementation_owner: `claude/glm-5.3`

## Objective

Close the gap between the passing PF-R5 online algorithm result at `c5921d5`
and the still-incomplete acceptance contract. Preserve run8 as the original,
unrelaxed Gate 4 accuracy result while making eval truth fail closed and
producing the missing exact-revision evidence.

This work is eval/spawner and evidence hardening. It must not change online
perception, planning, geometry thresholds, Gate 4 scoring, or hardware-path
inputs.

## Fixed Decisions

1. PF-R5 remains `algorithm-pass/acceptance-pending` until both subtasks pass.
2. Mesh-observable GT never falls back to catalog dimensions.
3. The original `top_surface_rate >= 0.95` remains authoritative; run8 passed
   it at `0.9606` without using a detection split or waiver.
4. Run8 remains the 30-trial accuracy evidence if the valid mesh reference
   values, evaluator scoring, and online detection path remain unchanged.
5. A new 30-trial run is mandatory if any of those three valid paths change.
6. PF-R6 profiling may continue independently, but PF-R6 formal closure and
   PF-R7 release remain blocked until this plan closes.
7. `platform_free_height_gate4_revision.md` is a proposal pending explicit user
   confirmation; this closure does not depend on it.
8. The user approved a risk-based dispatch exception for this bounded
   remediation: because PF-R5A/PF-R5B implement existing hard gates without
   changing architecture, online behavior, or thresholds, engineering
   consensus with the owner is sufficient and an additional Codex consensus is
   not required.

## Subtasks

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| PF-R5A | `claude/glm-5.3` | none | Fail-closed mesh-observable GT and deterministic asset identity | Invalid/unavailable STL fails spawn/eval explicitly; no box state or catalog-substituted GT is published; all six valid assets retain pinned observable values | Missing, truncated, malformed/non-binary, unknown tier, six positive assets, deterministic hash/version tests, affected package regression |
| PF-R5B | `claude/glm-5.3` | PF-R5A | Exact-revision focused acceptance, evidence completion, PF-A3 reconciliation, canonical close | Every focused hard gate passes on one clean commit; evidence contract is complete; rerun boundary is mechanically checked; PF-R5 status is reconciled without weakening Gate 4 | Positive smoke per tier, raw-only and no-box controls, online-truth audit, parameter dump, lifecycle teardown, contract/diff checks; conditional 30-trial rerun only when triggered |

## PF-R5A Requirements

- `_observable_reference` raises a stable explicit error when the STL cannot be
  loaded or validated.
- `handle_spawn_next` reports failure without publishing current box state,
  `GetCurrentBox` dimensions, or `/pickup_box_spawner/size_eval` data for the
  failed instance.
- Tests cover missing file, truncated file, malformed/non-binary input, and
  unknown visual/tier resolution.
- Pin all six sized-STL observable outputs. Prove values match the run8-era
  valid-path computation within strict numeric tolerance.
- Record SHA-256 for each STL and version the observable-reference parameters,
  including `top_band_frac` and Z-bin behavior. Parameter/version changes must
  be visible in evidence instead of silently changing GT.
- Keep all changes eval-side. Online nodes must not import or consume the asset
  hash, GT reference, `GetCurrentBox`, or `size_eval` payload.

## PF-R5B Evidence

Store focused artifacts under a new
`docs/status/evidence/platform_free_height/<run_id>/` directory:

- exact command, profile, code/worktree identity, and dirty-file count;
- complete relevant `ros2 param dump` output;
- raw-only negative control where every frame is
  `DETECT_CARGO_SEGMENTATION_REQUIRED` with zero valid top/full geometry;
- no-box negative control with zero scene/robot false-positive valid geometry;
- static and runtime evidence that online nodes do not read `GetCurrentBox`,
  `/pickup_box_spawner/size_eval`, or other eval truth;
- one positive mesh smoke per size tier proving reported GT equals the pinned
  STL-derived reference and includes asset/reference-version identity;
- `scripts/stop_sim.sh` teardown with zero residual ROS/Gazebo processes.

Before reusing run8, compare all six valid reference values, evaluator source,
and online detection files against the run8-era implementation. Record the
comparison. If any valid path differs, run a fresh clean 30-trial matrix and
pass every original Gate 4 limit.

## Closeout

PF-R5B closes only when:

- PF-R5A has a passing Result at an exact commit;
- all focused evidence above passes at the integrated exact commit;
- PF-A3 `gt_readiness=blocked` is explicitly reconciled to pass;
- the canonical PF-R5 thread points to the new evidence and states that run8
  passed the original gate without relaxation;
- no unsupported `user ruling` language remains in the Gate 4 proposal;
- an eng role note records commands, tests, revisions, and evidence pointers.

## Consensus And Dispatch Decision

Engineering consensus with `claude/glm-5.3` is recorded in
`docs/agents/discuss/2026-09-04_1453_pfh-remediation-r6.md`. On 2026-09-05 the
user explicitly cancelled the additional Codex review and authorized dispatch
under the risk-based exception above. The approved plan must still receive an
exact Git revision. Dispatch PF-R5A first; PF-R5B remains waiting on PF-R5A.
