# Deploy-update atlas qualification

Resampled on 2026-11-11 against the current source scene/model after deployment
cleanup and explicit user confirmation of the approximately 1.059 m base-to-ULD
translation. This is an empty-load, suction_contact_frame, container-directed
four-yaw proposal atlas. It does not certify any measured carrying geometry.

Qualification evidence: `docs/status/evidence/moveit_atlas_joint_plan/g1/ST-0/deploy_freeze_01/`.
All 12,098 stored contact/transit states passed independent FK and native
bounds/collision validation in the frozen static scene. Actual artifact paired
replay refined both seeds and checked a 33-state native connection. These gates
are deterministic artifact checks, not execution or statistical reliability.
The all-state gate does not certify unsampled configurations between the states.

The independent prepare entry accepts this artifact through explicit parameters:
`atlas_npz`, `atlas_yaml`, `atlas_qualification_file` and `atlas_scene_tf_config`.
Resolve the first three to the installed files in this directory (receipt:
`operation_atlas_qualification.json`) and the last to the installed description
package's `config/scene_tf_deploy_update.yaml`. The entry also requires its
existing explicit shared URDF/SRDF model and frame_scene_revision. The receipt
binds those exact model bytes and the scene MD5; mismatches return online fallback.
Do not regenerate the receipt at runtime to make a mismatching model pass.
The prepare service still validates exact contact/release targets, actual
attached-load collision scene, endpoint paths and the complete timed chain.
No complete trajectory cache, execution controller, observed OCC or box detector
is supplied by this atlas directory.
