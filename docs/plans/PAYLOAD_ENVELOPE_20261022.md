# Diagnostic planning payload envelope — 2026-10-22

User-authorized base: 849cec0d; owner codex/gpt-6, integration worktree
/tmp/elfin-master-pack-baseline-g3. Planning-only repair, no perception or
physical asset change and no new allowed contact with link2.

## Contract

Preserve measured box dimensions and nominal T_tool_box. Diagnostic limits:
yaw +/-5 degrees around suction_contact_frame origin/local Z, box-axis
per-face size margin 0.050 m, tool-axis translation half-range 0.100 m each.
All limits live in an editable YAML file; no coverage probability is claimed.
Roll/pitch are exact and box Z must be parallel/antiparallel to tool Z.
Reject missing measured attachment, invalid configuration or unsupported tilt.
Union all expanded box corners over the continuous yaw interval using endpoint
and stationary extrema, then Minkowski-add translation bounds. Store envelope
center and dimensions in tool coordinates, bind configuration and geometry to
attachment/certificate identity. Same model for slot containment, carrying
edges, corridor clearance, whole-arm validity and final certification; nominal
transform still determines tool targets. Keep existing contact exemptions.

## Acceptance and procedure

1. Focused pytest: endpoint and interior extrema, eccentric grasp, interval
wrap/full circle, exact height under yaw, malformed limits, unsupported tilt,
nominal preservation, config/digest changes and scene-consumer consistency.
Analytic extremum enumeration is the continuous coverage argument; dense
samples only cross-check it. All tests must pass.
2. Replay live_20261022_151711: every vertex of all five physical SDF cuboids
at the captured actual attachment lies in the envelope (1e-9m numerical
tolerance); new model rejects every one of the 15 known physical-collision
knots using native FCL/actual link2 mesh. GT remains offline only.
3. Plan and certify a new route with the frozen envelope. Offline verify
all-arm/physical geometry between knots using conservative interval subdivision
with motion bounds; unresolved intervals are not_evaluated, never pass.
If the initial state/goal is already incompatible with the requested envelope,
preserve the named refusal and do not execute or reduce limits to obtain pass.
4. Only after 1–3 pass, freeze implementation/config commit, build installed
packages and rehearse startup. Run scripts/g3/n3_functional_run.sh with 180s
budget, headless domain7 inch20 seed0, gain0.8, unchanged freshness/settle gates.
Require MAX_BOXES, 3 verified/map-confirmed commits, exit0 and healthy executor.
AND over3 functional units; this is a functional gate, not a reliability estimate.

## Evidence

Use docs/status/evidence/occ_owner_b1/payload_envelope_20261022/. Manifest records
revision/dirty state, commands, config/hash, workload, sample count, results,
artifacts and missing coverage. Preserve replay request, physical geometry and
transforms separately. Offline cases: inputs/results JSON plus FCL contacts,
<=20MiB. Live: existing executor-root-cause runner boundary capture, joint/plugin
streams, selection request/response, lifecycle and failure-window contacts;
<=1GiB/run, <=3GiB total, no deletion until capture health is verified. Before
rerunning a failed stage preserve its inputs/outputs and first failure. Sim
exclusivity/owned teardown follow .cursor/rules/sim-lifecycle.mdc.
