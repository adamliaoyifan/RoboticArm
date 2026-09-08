# 2026-09-05 15:50 — PF-R5A GT fail-closed + deterministic asset identity

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude-code
- status: done
- parent: PFH-R5-CLOSURE-20260905
- subtask: PF-R5A
- base_revision: 28370bc（approved plan）
- started_at: 2026-09-05T15:36:21+08:00
- completed_at: 2026-09-05T15:50:00+08:00

## Requirement

Approved plan `docs/plans/platform_free_height_closure.md` @ 28370bc
PF-R5A: remove the catalog-dimension fallback from mesh-observable GT;
invalid/unavailable STL must fail spawn/eval explicitly with no box
state or size_eval publication; deterministic asset/reference identity
(sha256 + parameter version); eval-side only.

## Summary

Implemented and committed at `24060c7`. The PF-A3 check-8 defect
(silent catalog substitution when the sized STL cannot be loaded) is
removed: `resolve_observable_reference` (new pure resolver in
`luggage_description.suitcase_visual`) raises `MeshReferenceError` for
unknown tier, missing asset, truncated file, non-binary or
non-finite/degenerate geometry; the spawner resolves the reference
BEFORE any world mutation or state publication and rejects the spawn
with `MESH_REFERENCE_UNAVAILABLE: <reason>`.

## Changed

- `src/luggage_description/luggage_description/suitcase_visual.py`:
  `MeshReferenceError`, `OBSERVABLE_REFERENCE_VERSION` (pinned
  semantics string: top_band_frac=0.25, z_bin=0.001,
  lid=densest-bin-median, extent=plateau_band), `stl_sha256`,
  `resolve_observable_reference` (finite/degenerate validation
  included — malformed STL bytes can decode to NaN floats that would
  otherwise poison GT silently).
- `src/luggage_gazebo/scripts/pickup_box_spawner_node.py`:
  `_observable_reference` delegates to the resolver, no fallback;
  `handle_spawn_next` catches `MeshReferenceError` → failed response,
  no `_current_box/_current_model/_current_ref` assignment, no
  `_publish_box_state`, no size_eval publish for the failed instance;
  `_box_to_record` and the size_eval payload carry the GT-reference
  identity (`version`, `stl_sha256`, path, visual/tier, lid_offset) on
  eval-side topics only.
- `src/luggage_description/test/test_pf_r5a_gt_fail_closed.py` (new):
  10 tests — unknown tier / missing / truncated / non-finite /
  non-binary raise `MeshReferenceError`; six pinned observable values
  equal to the run8-era computation (places=3); identity fields;
  sha256 determinism + content addressing; resolution determinism;
  exact version-string pin (a semantics change without a version bump
  fails the suite).

## Verification

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select luggage_description luggage_gazebo
cd src/luggage_description && python3 -m pytest test/ -q   # 135 passed
cd ../luggage_gazebo && python3 -m pytest test/ -q         #  43 passed
cd ../luggage_perception && python3 -m pytest test/ -q      # 461 passed
# spawner parses and imports cleanly
```

## Online-path audit

New symbols are imported only by the spawner and its tests
(`rg resolve_observable_reference|MeshReferenceError|stl_sha256 src/`):
no perception/planning node imports or consumes the asset hash, GT
reference, GetCurrentBox, or size_eval payload. DetectedLuggage message
and all online behavior unchanged.

## Rerun boundary statement (for PF-R5B)

PF-R5A changes only the failure path and record metadata: the six valid
observable values are pinned by test to the run8-era computation
(places=3), evaluator scoring and the online detection path are
untouched. Per plan Fixed Decision 4, run8 remains the 30-trial
accuracy evidence; PF-R5B must verify this mechanically (six-value
comparison + evaluator source diff against c5921d5).

## Result

- pass: all PF-R5A acceptance points met at commit `24060c7`.

## Pointers

- `docs/plans/platform_free_height_closure.md` (approved @ 28370bc)
- `src/luggage_description/test/test_pf_r5a_gt_fail_closed.py`

## Open

- None. PF-R5B not started (per plan: only after PF-R5A passes).
