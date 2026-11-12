# 2026-11-12 -- Occupancy margin provenance

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

User asks who decided the per-face5cm O inflation. Targeted blame and historical
role/thread audit identify the implementation owner as eng/codex/gpt-6 in the
2026-10-16 placement prior task. Git author identity is the shared account and
must not be used as evidence the user chose the value.

- b108fc11 (2026-10-16 15:30): introduced caller-supplied collision margin in
  private surface scene composition; this alone did not select5cm.
- 87275bef (2026-10-16 16:08): prior node and launch select0.05m by default.
  The dated Codex role note and authoritative CORE thread identify the owner.
- 7b61aab3 (2026-10-16 18:27): volume conversion adds2*margin to each ofXYZ
  dimensions, expanding each face5cm at the default setting.
- 51c2a5e0 (2026-11-11): Codex complete-operation preparation carries the same
  collision_margin_m default0.05m into the new interface.

No explicit user approval of this numerical per-face policy or empirical
calibration rationale was found in the targeted plan, thread and role records.
General user authorization to implement and preserve safety is not evidence
that the user selected5cm. The present implementation is an inherited software
default, not an established measured error bound. Current work retained it;
maintaining that default without checking its rationale was an oversight.
User confirmation of5cm release gap is distinct and cannot approve O inflation.
No settings changed. The preceding margin audit and native ablation remain
valid; next policy review must account for actual measurement/model uncertainty,
voxel quantization and attachment envelope without assuming duplicate terms.

## Pointers

- `87275bef`
- `7b61aab3`
- `51c2a5e0`
- `docs/agents/discuss/2026-10-16_1508_placement-prior-core-g1.md`
- `docs/agents/eng/2026-10-16_prior-prefetch-progress.md`
- `docs/agents/eng/2026-11-11_1538_joint-operation-prepare-entry.md`
- `docs/plans/placement_prior_g1.md`
- `docs/agents/reviews/2026-11-12_1231_current-planning-margin-audit.md`
