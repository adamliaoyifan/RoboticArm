# Full Gazebo model atlas qualification

Reuses the freshly sampled deploy_update empty-load seed arrays unchanged. All
12,098 states were independently requalified on 2026-11-12 with the exact full
Gazebo URDF/SRDF. Every link/joint and all 34 original frozen inputs match; only
Gazebo/control elements differ. FK maximum errors: 0.000495365m / 0.000208089rad,
zero breaches at <=0.002m / <=0.02rad. All states passed native bounds/collision
checking against the original full static scene whose geometry inputs are
unchanged; actual reader returns QUALIFIED. This is revalidation, not new sampling.

Evidence: docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-GAZEBO-OPERATION/atlas_requalification_01/.
Plan: docs/plans/GAZEBO_ATLAS_REQUALIFICATION_20261112.md at e65cb46e.

The joint_operation Gazebo entry selects atlas_runtime_gazebo_operation.yaml.
The hardware model retains its separate existing receipt. This artifact does
not certify measured payloads, intermediate paths or successful execution; live
refinement, connection and whole-chain collision validation remain mandatory.
Changing spawn joints can change model bytes and must refuse this binding.
