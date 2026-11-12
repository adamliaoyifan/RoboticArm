# 2026-11-12 -- Placement and planner evidence-backed design audit

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: open
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-DESIGN-AUDIT
- base_revision: bcebf44d5c0aa353c66e1de34d09cb2ab1dd6df9
- started_at: 2026-11-12T12:51:35+08:00
- completed_at: n/a

## Summary

Completed read-only audit with31 prioritized findings (13P0,16P1,2P2), a
needed/available/missing-data matrix and staged review plan. Collected103
hashed source/config/evidence snapshots,63 source references,14 named install
checks,34 frozen dependency rechecks and10 existing bag metadata/stat checks.
No production behavior, robot, simulator, controller, map or trajectory changed.
Physical accuracy, release/multibox qualification and live parameter overrides
remain not_evaluated. A finding may be a documented limit or missing coverage,
not necessarily a reproduced product defect.

## Requirement

Planfc42bb89 ST-4-DESIGN-AUDIT generation1; actual evidence-backed review of
ComputePlacement, atlas, candidate generation, joint planning, O, release and
physical validation barriers. No unsolicited correction of behavior.

## Changed

Detailed report, finite offline witnesses, source/data capture and provenance
verification programs and records. Corrected earlier claim that the new
prepare directly executes5cm same-layer spacing: the placed constraint
callback is absent there. The real defect is entry-point policy divergence.

## Verification

- python3 <run_01>/collector.py:90 initial inputs/14 install entries; source
  capture417.24s, below10min; subsequent supplementary review/source provenance
  recorded independently with timestamps, not scored execution.
- PYTHONPATH=$PWD/src/luggage_packing:$PWD/src/luggage_description:$PWD
  python3 <run_01>/probes.py:3/3 finite behavior witnesses reproduce:
  quarter-turn-required default0 vs explicit-yaw8 feasible; nominal contact
  blocked by5cm spacing; top1 atlas rejection vs top2 success.
- python3 <run_01>/supplement.py:34/34 frozen inputs,10/10 metadata hashes;
  full1cm grid4,344,244 cells requires67 pages vs compute source cap64.
- python3 <run_01>/verify.py:31 findings/63 source refs/103 inputs resolve;
  0 provenance errors,0 concurrent input changes,3/3 probes,~1.76MiB retained.
  Current atlas NPZ/YAML combined receipt digest matches. Numeric attachment
  config equals standard installs despite comment-byte differences.
- No67-RPC native paging run, new complete-route native test, raw full-bag
  decode, absolute metrology, performance campaign or physical experiment.

## Result

Audit provenance gate passes; implementation/report commit recorded at closure.
Whole ST-0/ST-3/ST-4 product gates remain open. Prioritize yaw/policy/O config,
paging and deployment wiring before sampling expansion or arbitrary timeout
increases. Review uncertainty/clearance/release with user before fixes.

## Pointers

- docs/reviews/PLACEMENT_PLANNER_DESIGN_AUDIT_20261112.md
- docs/plans/PLACEMENT_PLANNER_DESIGN_AUDIT_20261112.md
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-DESIGN-AUDIT/run_01/manifest.json
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-DESIGN-AUDIT/run_01/verification.json
- docs/agents/discuss/2026-11-12_1251_placement-planner-design-audit.md
