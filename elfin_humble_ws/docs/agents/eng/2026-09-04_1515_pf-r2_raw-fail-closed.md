# 2026-09-04 15:15 — PF-R2 raw-only cargo input fails closed

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude-code
- status: done
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R2
- base_revision: 0674f84 (working tree, after PF-R1)
- started_at: 2026-09-04 15:09 local
- completed_at: 2026-09-04 15:15 local

## Summary

Raw-only input (`use_semantic=false`) now fails closed before any
fitting: an unsegmented raw depth cloud is not a luggage observation, so
the 0.86 m pickup platform can never be reported as a valid luggage top,
in any support mode, with or without a configured `platform_z`.

## Changed

- `src/luggage_perception/luggage_perception/top_support_estimator.py`:
  new failure code `DETECT_CARGO_SEGMENTATION_REQUIRED`.
- `src/luggage_perception/luggage_perception/platform_free_pipeline.py`:
  `PlatformFreeDetector.update(..., cargo_segmented=True)` — `False`
  returns `top_valid=false` with that reason and clears the stability
  window before `estimate_top_surface` is ever called.
- `src/luggage_perception/scripts/luggage_detector_node.py`: passes
  `cargo_segmented=self._use_semantic`; removed the E2-era fallback that
  reused the cargo cloud as its own raw support source on the raw path;
  startup `ERROR` log names the exact failure code and the fix
  (`use_semantic:=true`). The node stays up (motion/vacuum workflows
  keep their infrastructure) but every detection fails explicitly.
- `src/luggage_perception/test/test_pf_g3a_raw_fail_closed.py` (new):
  PF-G3A suite.

## Verification

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select luggage_perception
cd src/luggage_perception
python3 -m pytest test/test_pf_g3a_raw_fail_closed.py -q   # 6 passed
python3 -m pytest test/ -q                                  # 387 passed
```

## Requirement

- PF-G3A: hazard documented (unsegmented platform-only scene *would*
  fit 0.86 m as a valid top without the gate — first test); raw-only
  fails in every support mode incl. `configured` with `platform_z`;
  configured platform Z cannot promote raw points to cargo; segmented
  input still reaches the estimator; no FULL_3D / `height_valid=true`
  is ever produced from raw input.
- No legacy raw-only escape hatch was added: no current workflow needs
  it (all eval drivers launch `use_semantic=true`); if one appears it
  must be an explicit non-default mode per the remediation plan.

## Result

- pass: PF-G3A green, perception regression 387 passed.

## Pointers

- `docs/plans/platform_free_height_remediation.md` (PF-R2/PF-G3A)
- `src/luggage_perception/test/test_pf_g3a_raw_fail_closed.py`

## Open

- None.
