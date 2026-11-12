# 2026-11-12 -- Resolution control changes next-work priority

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

User asks whether2cm would remove the10cm fixture collision. Actual same-input
native control is now available. It still refuses at original sample0 with
unchanged5cm padding; occupied and unknown runs both contribute. Diagnostic
zero-padding removes reported occupied contacts at this sample, but unknowns
still refuse. This narrows what the coarse fixture can establish.

## Decision for discussion

Earlier recommendation treated own-cargo association as the immediate main
integration prerequisite without first measuring a resolution control. That
ordering was broader than the evidence justified. The user-approved association
work remains useful/authorized; do not cancel it or rewrite claimed integration
requirements. First align the benchmark with the actual deployment before
treating this synthetic refusal as a deployment requirement:

1. Review the fixture's sphere centred at suction_contact_frame against actual
   deployed tool collision geometry. The synthetic model has no arm/base
   collision geometry and cannot qualify real clearance.
2. Freeze intended O resolution and sensor observation coverage explicitly.
   Source production default is1cm; this experiment tests2cm, not an implicit
   production change. Reusing the same coarse synthetic raster at finer
   resolution changes the observed/unknown coverage, and that must be measured.
3. Trace5cm collision_margin_m to its declared semantics and actual call-site
   binding. It expands every voxel in all directions; do not confuse it with
   release gap or assume removing it is authorized. Check possible duplicated
   clearances as a hypothesis, never reduce it just to obtain a pass.
4. Reproduce on current deploy_update full robot/tool model with an explicit
   sensor/scene workload. Separate occupied geometry from unknown obstruction
   in evidence. Then decide which minimal integration correction is necessary;
   do not use planned-box carving or require unbounded vision development first.

No successful full-cycle, production margin policy or physical matching
qualification follows. Keep baseline failure visible; comparison matrix and
full original generation2 acceptance remain open. This is a review checkpoint,
not dispatch of a new scored/hardware campaign.

## Pointers

- docs/agents/eng/2026-11-12_1206_release-occupancy-resolution.md
- docs/agents/reviews/2026-11-12_1145_joint-baseline-next-todo.md
- docs/agents/discuss/2026-11-11_2057_joint-operation-full-integration-g2.md
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-INTEGRATION/g2/release_resolution_02/summary.json
