# Full Gazebo model atlas requalification

- status: approved
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-GAZEBO-OPERATION
- generation: 1
- owner: codex/gpt-6
- authorization: User confirmed final baseline must use requalified atlas; diagnose whole-chain first.

## Procedure and acceptance

This is independent empty-load artifact validation, not a full-cycle witness.
Use frozen cycle_03 runtime URDF/SRDF and the current production native certifier,
ROS domain88, no Gazebo, controllers, vacuum or hardware. Compare every link and
joint element with the original qualified model and every original frozen input
hash with current source (34 inputs); all must match. Other URDF elements may
differ and must be enumerated. Only with this proof may the original captured
full static PlanningScene be reused: static geometry/mesh inputs and all collision
links/joints are identical. Preserve original scene CDR and its SHA in provenance.

Copy NPZ without changing any array. Derive a candidate YAML with current URDF
hash, explicit original sampling provenance and requalification scope. Do not
install it or mint a reviewed receipt before validation. The prior is empty-load;
actual measured payload refinement/collision checking remains mandatory.

Run scripts/certify_atlas_seeds.py on the candidate using exact Gazebo URDF:
all12,098 stored contact/transit seeds must pass position <=0.002m and orientation
<=0.02rad, zero breaches, at least one contact witness in each directed yaw bin.
Then run existing ST-0 verify.py procedure adapted only for new frozen paths,
current installed native executable, domain88 and new validation_id. Every state
must pass full-model bounds/collision, exact response identities/model hash and
complete leg count. One run of all states; AND across states, deterministic
artifact qualification, no inferred execution reliability. Missing samples or a
failure gives not_evaluated/failed and no approved receipt. No mutation of source
packages during the run. Save original requests/responses and state verdicts.
After all gates pass, call actual load_operation_atlas and require QUALIFIED.
Keep artifact integration separate from this offline qualification; live use must
record matching model/scene/artifact identities and actual atlas consumption.

## Evidence

ST-4-GAZEBO-OPERATION/atlas_requalification_01. Keep exact candidate files, source
comparison/provenance, FK JSON/JSONL, native CDR request/response pairs and process
logs/result. <=256MiB for this unit. Native process owned PGID and clean teardown.
No artifact replacement on failure. Current revision and dirty-state hashes are
recorded before and after; repeat only after a measured cause is repaired.
