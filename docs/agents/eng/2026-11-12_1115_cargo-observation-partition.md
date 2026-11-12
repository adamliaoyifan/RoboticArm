# 2026-11-12 -- Exact-acquisition cargo observation partition

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: done
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-OBS-JOIN
- base_revision: 7b697e00
- started_at: 2026-11-12T11:04:56+08:00
- completed_at: 2026-11-12T11:14:30+08:00

## Summary

Implemented the first source-evidence prerequisite for the user-authorized released-box representation policy. It binds real exact-acquisition RGB/depth/mask/calibration/TF to immutable point/pixel/frame-label bytes. No physical identity, O mutation, obstacle substitution or whole-cycle success is claimed.

## Requirement

Plan18f35ccb, ST-4-OBS-JOIN generation2 (administrative replacement of symbolic-HEAD gen1; scope unchanged). All finite positive/negative cases must return the required outcome, with lossless IDL transport and source/installed behavior parity. No raw sensor re-pairing, missing-depth completion, latest TF or task-label ownership. Preserve5cm release gap.

## Changed

- cargo_observation_partition.py: ROS-free exact-acquisition geometry and immutable binding; labels remain local to one frame. All sampled returns including background retained; invalid depth excluded without reindexing source pixels.
- cargo_observation_service.py: bounded immutable CDR buffer, typed source hash independent of DDS padding and Python bool/int representation, conflicting acquisition refusal, eviction watermark, final age/conflict check after TF.
- cargo_observation_node.py: opt-in read-only subscriptions and acquisition-time TF/service boundary. No robot/vacuum/O/scene command interfaces. Source defaults are existing preprocessed topics; sampling/cache/age are explicit resource policies, not geometric identity tolerances.
- CargoObservationPartitionV2.msg / GetCargoObservationV2.srv, generated dependency/install declarations; pending F27 inventory.

## Verification

- source /tmp/joint_operation_o_env.sh; colcon --log-base /tmp/cargo_observation_log build --base-paths src/luggage_msgs src/luggage_perception --packages-select luggage_msgs luggage_perception --build-base /tmp/cargo_observation_build --install-base /tmp/cargo_observation_install --cmake-args -DBUILD_TESTING=OFF:2 packages pass.
- source-first pytest -q src/luggage_perception/test/test_cargo_observation_partition.py src/luggage_perception/test/test_cargo_observation_service.py:62 passed,0.32s. Complete typed source inputs/outputs and actual DDS Image/TF/service witness retained.
- pytest -q src/luggage_perception/test/test_ros_message_adapters.py src/luggage_perception/test/test_segmenter_ingest.py src/luggage_perception/test/test_container_depth_observer.py:62 passed,0.37s.
- Installed-only module path and generated IDL witness:4 encoding/endianness roundtrips + actual DDS service=5 passed,0.24s;2 installed module hashes and installed node script match source.
- python3 scripts/check_frame_contracts.py --mode baseline --output <run_03>/frame_inventory:50 cases pass; migration remains NOT ready, F27 pending.
- Additional staged-inventory check:all50 geometric cases pass, but baseline exits1 because pre-existing F14 points to an atlas deleted by unrelated deployment cleanup. Current working inventory includes that cleanup correction and passes; only new F27 is staged here. Preserve staged_frame_gate output; no global clean-inventory or production migration claim.
- Earlier run_01 fixture float-type failure and run_02 source-hash normalization failure preserved before fixes. No physical motion or scored performance run. Functional AND once per named case does not estimate hardware reliability or speedup.

## Result

Pass at cf3192f6acfddd90f8e6716a3e9b3a38e34f8991; bounded OBS-JOIN generation2 closed through agent_complete.sh. Physical cargo association, exhaustive O contributor provenance, safe private-scene substitution, current deploy_update witness and original full integration matrix remain open. New service alone cannot clear an O voxel.

## Pointers

- docs/plans/RELEASED_CARGO_ASSOCIATION_20261112.md
- docs/agents/discuss/2026-11-12_1104_released-cargo-exact-observation-g2.md
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-OBS-JOIN/run_01/
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-OBS-JOIN/run_02/
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-OBS-JOIN/run_03/manifest.json
