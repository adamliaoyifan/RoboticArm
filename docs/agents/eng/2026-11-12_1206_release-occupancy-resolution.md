# 2026-11-12 -- Native10cm versus2cm occupancy control

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: open
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-O-RESOLUTION
- base_revision: 415506aa
- started_at: 2026-11-12T11:57:56+08:00

## Summary

User correctly challenges the coarse10cm synthetic O fixture. Replayed the
same39 original observations through actual native O at10cm and2cm, then
validated the identical original494-sample timed retreat in private scenes.
2cm with original5cm padding still refuses at sample0. Occupied-cell padding
and finer-grid unknown coverage are independently identified contributors.
No production setting, margin, trajectory, global scene or motion was changed.

## Requirement

Plan67a6e71d, ST-4-O-RESOLUTION generation1. Frozen same39 acquisition requests,
source transforms/order, robot/fine released box and original trajectory. Three
finite native diagnostic variants once each; exact10cm state/contact control
must reproduce. A2cm COLLISION is an experimental outcome, not gate failure.
No reliability/performance or real-wrist/deploy_update clearance claim.

## Changed

Only bounded diagnostic scripts/evidence and plan/role notes. Native O owner,
ObserverSnapshotPort, actual private-scene converter and native certifier are
unchanged. Zero-padding variant is a diagnostic ablation with no runtime
authority. All other geometry and unknowns retained in every native scene.

## Verification

Commands: source /tmp/joint_operation_o_env.sh; python3 <run_02>/replay.py;
python3 <run_02>/analyze_contacts.py. Actual native O/MoveIt, isolated domain184,
same localhost discovery; no Gazebo/hardware/controller. Native run32.19s.

| Resolution/padding | Native verdict | First invalid | O state evidence |
|---|---|---|---|
|10cm/5cm | COLLISION | sample0/494 | original states identical;0 unknown,6 occupied; original4 contact pairs reproduced |
|2cm/5cm | COLLISION | sample0/494 |20668 unknown,40 occupied; returned32 contacts:25 unknown-only runs,7 occupied-only runs |
|2cm/0cm diagnostic | COLLISION | sample0/494 | returned4 contacts, all unknown-only runs; no occupied-only contact reported at this sample |

2cm occupied runs at the box top span z[0.70,0.72]m;5cm padding raises their
top to0.77m. Fixture TCP z0.75m, cup sphere radius0.03m spans down to0.72m.
Native occupied-contact penetration includes0.05m. Correct2cm quantization
therefore reduces the10cm overreach but does not by itself clear this fixture.
The unchanged sparse synthetic rays leave20668/320000 cells unknown at2cm;
unknowns remain obstacles. The no-padding result localizes its sample0 refusal
to unknowns, not proof that the entire unknown-free suffix would pass.
Contact counts are returned bounded diagnostic contacts, not all possible pairs.

All39/39 integrations accepted per resolution; observed_revision39. All194
captured request/reply CDRs rehashed and decoded. Native contact-object bounds
match actual converter run geometry and contributing native state pages.
Mandatory snapshot releases consumed; both O processes and certifier exit0.
run_02~42.94MiB, total retained~61.1MiB within budget. run_01~18.13MiB retains
first harness AttributeError (model SHA was read from Request instead of
Response), exact input/output and clean native teardown; corrected harness
uses retained original Response model SHA and checks native equality.

## Result

Bounded diagnostic gate passes; lifecycle closure pending exact commit. Full
integration remains open.2cm is an appropriate finer control to investigate,
but resolution alone is not sufficient in this preserved fixture. No runtime
padding reduction or unknown erasure follows from this experiment.

## Pointers

- docs/plans/RELEASED_CARGO_ASSOCIATION_20261112.md
- docs/agents/discuss/2026-11-12_1157_release-occupancy-resolution.md
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-INTEGRATION/g2/release_resolution_02/manifest.json
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-INTEGRATION/g2/release_resolution_02/summary.json
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-INTEGRATION/g2/release_resolution_02/contact_source_analysis.json
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-INTEGRATION/g2/release_resolution_01/manifest.json
