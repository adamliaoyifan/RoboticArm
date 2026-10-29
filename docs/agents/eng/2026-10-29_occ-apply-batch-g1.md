# 2026-10-29 -- ApplyBatch lookup and lazy maintenance closeout

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

Approved plan c49b2786 implemented in existing worktree. No new worktree, Gazebo or production promotion. A c4407bc0 removes redundant search via official sensor-model arithmetic and setNodeValue. B186cdf15 batches inner occupancy and pruning before return. Closeout4c8f1862 adds explicit unknown miss/hit coverage and exception contract documentation; measured runtime remains B186cdf15. Shared canonical adapter57a893f5, final runner3cb40447.

## Verification

Frozen A/B and final closeout CTest4/4; closeout Python45/45; B ASan+UBSan profile/keyset exit0; ROS node linked only. 28/28 pairs,924 paired frames including warmup,0 raw/canonical/content mismatches. Four separate no-digest RSS windows complete and do not claim semantic equivalence. Seven reference overshoot replay bundles complete; no rescoring or sample removal. Total campaign measurement2741.5s.

## Result

A/base integrate median ratio0.923–0.969, P95 0.842–1.075. B/A median0.929–1.024, P95 0.735–0.845,max0.768–0.838. B's12 primary Off windows and4 attribution Off windows meet existing references; some medians regress slightly. Stages4/4 satisfy prior one-sided perturbation limits, but one ratio0.751 signals substantial window variation, so stage clocks are not equated to Off percentages. B no-digest CLI RSS246MiB30k,408–411MiB100k, not pure map or Orin measurement. Recommend B as a future formal-acceptance candidate; O5/O6/O7 and product acceptance remain not_evaluated.

Allocation-failure rollback/recovery is not claimed; owning store must be reset/discarded/restored after failed apply. Pre-apply validation and invalid checkpoint rejection remain separately covered.

## Pointers

- docs/plans/OCC_APPLY_BATCH_G1_20261029.md
- docs/agents/discuss/2026-10-29_1711_occ-apply-batch-g1.md
- docs/status/evidence/occ_owner_b1/occ_apply_batch_20261029/run01/FINAL_REPORT.md
- docs/status/evidence/occ_owner_b1/occ_apply_batch_20261029/run01/VERSIONS.json
- docs/status/evidence/occ_owner_b1/occ_apply_batch_20261029/run01/REPLAY.md
