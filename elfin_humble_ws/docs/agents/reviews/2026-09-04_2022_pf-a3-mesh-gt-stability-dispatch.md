# PF-A3 Mesh-observable GT Stability Dispatch

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done
- parent: PFH-REMEDIATION-20260904
- subtask: PF-A3
- revision: 408f6d5dd9aeb8536377f0f14d06155be7a904d5

## Summary

`PF-A3` is an independent eval-truth audit assigned to `cursor/grok-4.6` while
PF-R5 validation is still open. It determines whether the STL-derived
mesh-observable reference is independent of the online estimator, reproducible
from identified assets, invariant to irrelevant STL representation changes,
and fail-closed when its reference source is unavailable. It does not rerun or
close PF-R5 and must not modify the GT or perception implementation.

## Scope

Primary code and assets under audit:

- `src/luggage_description/luggage_description/suitcase_visual.py`
- `src/luggage_gazebo/scripts/pickup_box_spawner_node.py`
- `src/luggage_description/test/test_suitcase_visual.py`
- six sized suitcase STLs under `src/luggage_gazebo/models/`
- `scripts/platform_free_height_gate4_eval.py`
- online consumers under `src/luggage_perception/` and `src/luggage_planning/`

Do not edit production source, tests, model assets, launch/config files, or the
PF-R5 evidence. Audit helpers live only in the new evidence directory. No ROS
graph or simulator is required.

## GT Definition Under Audit

For a sized binary STL, the current definition is:

- top band: vertices within `max(0.25 * full_height, 2 * 0.001 m)` of AABB top;
- lid plane: median Z in the densest `1 mm` Z bin in that band;
- observable width/depth: XY span of vertices within `1 mm` of the lid plane;
- lid offset: AABB top Z minus lid-plane Z;
- observable height: full AABB height minus lid offset;
- top pose: spawn AABB top minus lid offset;
- center Z: observable top Z minus observable height divided by two.

The audit must verify this actual behavior and flag any ambiguous tie-breaking,
silent fallback, unit, axis, or asset-version assumption.

## Execution Commands

Use the same immutable-snapshot pattern as EXP-A1:

```bash
set -euo pipefail
export SOURCE_ROOT=/home/adamliao/work/elfin_humble_ws
export BASE_REV=408f6d5dd9aeb8536377f0f14d06155be7a904d5
export RUN_ID="$(date +%Y-%m-%d_%H%M%S)_pf-a3"
export EVIDENCE="$SOURCE_ROOT/docs/status/evidence/platform_free_height/$RUN_ID"
export AUDIT_ROOT="$(mktemp -d /tmp/elfin_pf_a3_408f6d5.XXXXXX)"
mkdir -p "$EVIDENCE"
git -C "$SOURCE_ROOT" archive "$BASE_REV" | tar -x -C "$AUDIT_ROOT"
test "$(git -C "$SOURCE_ROOT" rev-parse "$BASE_REV")" = "$BASE_REV"
printf '%s\n' "$BASE_REV" > "$EVIDENCE/base_revision.txt"
printf '%s\n' "$AUDIT_ROOT" > "$EVIDENCE/audit_root.txt"
cd "$AUDIT_ROOT"
export PYTHONPATH="$AUDIT_ROOT/src/luggage_description:$AUDIT_ROOT/src/luggage_perception"
```

Run the existing focused tests:

```bash
set -o pipefail
python3 -m pytest -q \
  src/luggage_description/test/test_suitcase_visual.py \
  2>&1 | tee "$EVIDENCE/focused_pytest.log"
```

Record source/consumer boundaries and asset identity:

```bash
sha256sum \
  src/luggage_gazebo/models/suitcase_loafbrr_{small,medium,large}/meshes/suitcase.stl \
  src/luggage_gazebo/models/suitcase_vintage_{small,medium,large}/meshes/suitcase.stl \
  > "$EVIDENCE/stl_sha256.txt"

rg -n \
  'mesh_observable_reference|_observable_reference|_gt_size|lid_offset|top_surface_pose' \
  src/luggage_description src/luggage_gazebo scripts/platform_free_height_gate4_eval.py \
  > "$EVIDENCE/gt_producer_scan.txt"

rg -n \
  'GetCurrentBox|SpawnNextBox|pickup_box_spawner|mesh_observable_reference|ground[_ -]?truth|model_states' \
  src/luggage_perception src/luggage_planning src/luggage_packing \
  > "$EVIDENCE/online_gt_scan.txt" || true
```

Create `$EVIDENCE/pf_a3_gt_probe.py` and run it twice in fresh processes:

```bash
PYTHONHASHSEED=1 python3 "$EVIDENCE/pf_a3_gt_probe.py" \
  --repo "$AUDIT_ROOT" --out "$EVIDENCE/probe_seed1.json"
PYTHONHASHSEED=987654 python3 "$EVIDENCE/pf_a3_gt_probe.py" \
  --repo "$AUDIT_ROOT" --out "$EVIDENCE/probe_seed987654.json"
cmp "$EVIDENCE/probe_seed1.json" "$EVIDENCE/probe_seed987654.json"
```

The probe must use an independently written binary-STL parser/oracle for the
cross-check; calling `mesh_observable_reference()` twice is not an independent
oracle. It must cover all six sized STLs and execute these checks:

1. Record triangle count, AABB, SHA-256, visual ID, tier, algorithm parameters,
   returned width/depth/full height/lid offset/observable height, and all
   invariants in machine-readable JSON.
2. Repeat each calculation at least ten times and in the two fresh hash-seed
   processes. Results must be exactly stable after canonical JSON encoding.
3. Copy each STL to a different path. The result must agree within `1e-9 m`.
4. Deterministically reverse and randomly shuffle binary STL triangle records
   without changing vertex geometry. The result must agree within `1e-9 m`.
   A tie dependent on triangle insertion order is a blocker.
5. Translate every vertex by a representable fixed XYZ offset in a temporary
   STL. Width, depth, lid offset, and height must agree within `1e-6 m`.
6. Compare the production result with the independent oracle. Each scalar must
   agree within `1e-6 m`; the oracle must define deterministic modal-bin tie
   handling explicitly.
7. Verify `top_z - center_z == observable_height / 2` and that the observable
   bottom matches the spawned AABB bottom within `1e-6 m` for representative
   poses and yaw values. Yaw must not change the scalar dimensions/top Z.
8. Exercise missing, truncated, malformed, and non-binary STL inputs. Classify
   whether the eval GT fails closed or silently substitutes catalog geometry.
   Silent substitution in an acceptance run is `BLOCKED`, even when a warning
   is logged.
9. Trace `GetCurrentBox` from producer to Gate 4 evaluator and prove that no
   online perception/planning algorithm consumes it. Any runtime path from
   spawner/eval truth into detection, geometry, pick, or placement is `FAIL`.
10. Record sensitivity for `top_band_frac` values `0.20, 0.25, 0.30` and
    `z_bin` values `0.00075, 0.001, 0.00125`. This is diagnostic, not a relaxed
    oracle. Any change over `1 mm` in lid offset or `5 mm` in width/depth must
    be reported as a versioning risk.

Finish with:

```bash
cd "$SOURCE_ROOT"
git diff --check
scripts/check_agent_contract.sh
git status --short > "$EVIDENCE/primary_status_after.txt"
```

## Acceptance

- Evidence identifies the exact source revision and SHA-256 of all six STL
  inputs. A catalog label alone is not a reproducible GT identity.
- `probe_seed1.json` and `probe_seed987654.json` contain all ten checks and use
  `PASS`, `FAIL`, `BLOCKED`, or `NOT_APPLICABLE` with concrete evidence.
- Independence is established separately for algorithm independence, dataflow
  isolation, asset identity, and evaluator-only access.
- Stability is established separately for repeatability, process hash seed,
  file path, triangle order, translation, independent-oracle agreement, and
  parameter sensitivity.
- Missing/corrupt asset behavior is explicitly tested. The audit may not infer
  fail-closed behavior from source inspection alone.
- No online or GT production code is changed and PF-R5 is neither accepted nor
  closed by this task.
- A test role note reports `audit_outcome` and `gt_readiness` separately. If
  any invariance, independent-oracle, fail-closed, or online-isolation hard
  check fails, `gt_readiness=blocked` even though a complete audit may close
  with `audit_outcome=pass`.
- Close the canonical thread with `scripts/agent_complete.sh` and exact
  evidence pointers. Send blockers to `reviews` in that same thread; do not
  open a competing PF-R5 thread.

## Risks

- The current modal-bin selection may be sensitive to equal-count bin insertion
  order; triangle-order permutation is required to settle this.
- The spawner currently appears to warn and use catalog dimensions when the STL
  reference cannot be loaded. The runtime test must determine whether this can
  contaminate an acceptance run.
- The current GT is observable-surface truth, not collision-AABB truth. Both
  identities must remain distinct in evidence and field naming.

## Consensus

- Codex agent: existing PFH remediation consensus
- Thread: `docs/agents/discuss/2026-09-04_1453_pfh-remediation-r5.md`
- Result: reached for mesh-observable GT use; this user-requested audit validates
  the definition without changing it

## Pointers

- `docs/plans/platform_free_height_remediation.md`
- `docs/plans/platform_free_height_test_plan.md`
- `docs/agents/reviews/2026-09-04_1943_pfr5-vintage-segmenter-decision.md`
- `docs/agents/discuss/2026-09-04_2022_pf-a3-mesh-gt-stability-audit.md`

## Open

- None. Dispatch is authorized by the user.
