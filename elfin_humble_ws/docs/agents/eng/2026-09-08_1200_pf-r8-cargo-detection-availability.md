# 2026-09-08 -- PF-R8 cargo detection availability closed (A1-A5)

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude
- status: done

## Summary

Implemented and closed PF-R8 at `7b0b41a` (base `f03ccc3`, the PF-R6 gen3
commit). A1: accepted-detection predicate — backproject each cargo bbox
centre onto the pickup-platform plane via live intrinsics + stamped TF and
accept within max(half_extents)+0.15 m of the workspace centre; rejects the
static right-edge pedestal false positive (centre ray ~0.95 m off-centre,
byte-identical bboxes across spawns), accepts an edge-clipped suitcase, fails
open flagged when camera_info/TF is missing. Frozen 440-instance fixture
(319 right-edge negatives = superset of the plan's six recurring bboxes/184
instances, 120 suitcase positives, plus one geometrically-derived
edge-clipped positive with provenance recorded — no real instance exists in
any capture): 0 false accepts, 0 false rejects. A2: `DetectionTemporalGate`
positive sample = accepted detection only; window identity immune to
suitcase/FP alternation; epoch reset and expire-to-empty tested. A3 causal
replay over the 155-frame capture: miss set exactly the plan's 42 FP-only
frames, 0 acausal holds, 0 missed holds, 40 unrecoverable frames reported.
A4: `confidence_threshold` 0.04 -> 0.01 (the two ~10 s whole-trial dropouts
score 0.01-0.03 in the documented diagonal-yaw/lighting states); same-command
measurement: baseline recall 0.874 / miss run 33 -> PF-R8 1.000 / 0 on two
independent runs (489 + 512 settled frames), production annotation agreement
845 matched / 100% where both sides had TF, all 319 FP instances rejected,
detection_frame outcomes only ok/DETECT_NO_CLOUD. Full perception suite 502
passed; teardown residual 0 after every sim run. Implementation and
falsification were offline per the plan; live runs were only the A4
measurement itself.

## Pointers

- `docs/agents/discuss/2026-09-07_2039_pf-r8-cargo-detection-availability.md`
- `docs/status/evidence/platform_free_height/2026-09-08_pfr8_recall/RESULT.md`
- `src/luggage_perception/test/fixtures/pf_r8/acceptance_fixture.json`
- commits `f03ccc3` (base), `7b0b41a` (implementation)

## Open

- none for PF-R8; PF-R10 re-verifies false_measured_height under GT at C1.
