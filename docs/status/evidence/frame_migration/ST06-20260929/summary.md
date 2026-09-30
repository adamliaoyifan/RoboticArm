# ST06 -- atlas and concrete tool branches

- outcome: pass
- generation: 2
- base_revision: 20048a5a7004f5f06bb99c568b0d04cfe4bbd796 (ST03-ST05 repair round 2)
- plan_revision: d90332d87ad6da916b397e487d6260cc3b7023fb
- revision: (see manifest.json / revision.txt; final commit closes this stage)
- branch: integration/frame-migration-st06-20260929
- worktree: /tmp/elfin-frame-migration-st06

## Implemented

- Directed 2pi concrete-tool bins (F14 target): builder `~yaw_bins`
  default is `[0, pi/2, pi, 3pi/2]` in container_link; the bin yaw is
  mapped through the scene-config container rotation (constant Z offset
  for a horizontal container) inside `atlas_frame_contracts.
  tool_target_in_base`, the single ROS-free home of the advertised
  target construction (builder + tests + certifier share it). 0 and pi
  are distinct branches; q/-q share a rotation; container tilt is
  rejected by the 2.5D admission before any construction.
- v2 artifact contract: every artifact writes a top-level `contract`
  block (`validate_atlas` fields; `yaw_period_rad 2pi`; scene_revision =
  scene_tf_hash; geometry/payload identity hashes reproduced from the
  artifact's own sections, so a v1 NPZ cannot be relabelled). Schema
  `atlas_version 3.0 / schema_version 3`; per-branch `transit_lift`
  records the exact lift each vertical transit seed was solved at.
- Directed query semantics: `ReachabilityAtlas.query_pi_equivalents`
  (and the QuerySemantics passthrough) consult both pi-equivalent
  branches; hard reject requires BOTH branches confidently unreachable
  interior; unknown stays unknown; `snap_yaw_for_atlas` dispatches v2
  directed (no fold) vs v1 folded behavior; the placement hook queries
  both directed grasps and records `atlas_yaw_bins_queried`; stub/v1
  atlases keep the legacy single folded query byte-identically.
- Strict runtime gate: `load_runtime_atlas` refuses a v1 artifact under
  `hash_policy: strict` (`atlas_semantics_v1_strict` /
  ATLAS_SEMANTICS_MISMATCH, zero seeds) and refuses a contract whose
  identity hashes do not reproduce from the artifact's own sections
  (`atlas_contract_strict`); warn keeps v1 as a declared downgrade.
- F16 seed side: `motion_planner_node._container_tool_yaw` re-expresses
  the world tool yaw about container +Z before the bin snap (a raw
  world yaw names the wrong branch on a rotated scene); the
  `box_size=list(key)` cache-key crash is fixed to `key[0]`. The F16
  row stays pending -- ST08 owns the MoveIt boundary consumer.
- Task grid isolation: `reachable_task_grid` writes and exposes an
  explicit `semantics: v1_pi_folded` marker; no runtime consumer, not
  certified v2 (declared deferral, not an omission).
- FK certification: `scripts/certify_atlas_seeds.py` FKs every stored
  contact and vertical-transit seed against the independently
  reconstructed container target (<=0.002 m / <=0.02 rad), gates
  internal-lift transit seeds on orientation+hull, requires >=1
  certified reachable seed per directed bin, and md5-verifies the
  build-time robot_description capture. A breach is never masked by the
  coverage verdict.

## Build campaign (all four profiles rebuilt)

`campaign/run_campaign.sh` (exclusive stack, gui=false, DOMAIN_ID=7,
stop_sim teardown; v1 baselines restored from git with a 2-bin shape
guard):

- GATE PASS x4 against the preserved v1 artifacts. The mapped bins
  reproduce v1 seed-for-seed (legacy base-yaw-0 branch == container bin
  pi/2; KNOWN_SEED within 2e-3), no unexplained low-layer drops, iz5/6
  connected not regressed, no REACHABLE at z >= 1.70. Empty-load
  deterministic fingerprint 56292d189360e0850a7e12feb128f4a73aa3b5c2bbabfa2df2cddeb7545b6c36.
- FK certification x4 (`campaign/fk/<profile>/`): exact-pose rows max
  2.5e-5 m / 6.3e-6 rad; per directed bin certified reachable seeds
  729/958/995/828 (empty-load) and 369-607 (payload profiles); zero
  breaches; internal transit rows meet orientation+hull.
- Scene identity continuous: scene_tf_hash b40c85ae6eb2eabc47d7da2ffb23b72c
  before and after (deps and contract agree).

## Gates

- Stage suite `test_frame_migration_atlas.py`: 24/24 (64-case matrix,
  contract positives/negatives with exact codes, directed semantics,
  strict refusals with zero seeds, hook both-grasps, seed-side yaw,
  FK gate pass/breach/not_evaluated, task-grid marker, rebuilt-artifact
  coverage). Case JSONL under `cases/`.
- Batteries: planning 653 passed; packing 126 passed + 1 pre-existing
  environment failure (`test_placement_geometry_identity_live` -- live
  atlas URDF identity needs robot_description; reproduced identically at
  pristine 20048a5); perception 1048 passed + 1 pre-existing
  (`eval/test_pf_a1_static_audit`, sim-backend classification;
  reproduced identically at pristine 20048a5); description 187 passed.
  `test_atlas_runtime` pins updated in-chain to the directed naming
  (container bin pi/2 = legacy base yaw 0; seeds unchanged).
- Checker: baseline 50/50 exit 0; migration exit 2 with **13 pending**
  (`migration2/` is the final bundle; `migration/` and
  `migration_preproof/` are the intermediate pre-proof runs, kept).
- Proofs: F14 verified with `runtime_F14.json` (row sources extended
  with `atlas_frame_contracts.py`, `certify_atlas_seeds.py` and the four
  rebuilt artifact pairs so the proof hashes the certified bytes); F07
  proof refreshed on this tree (`runtime_F07.json`) because
  `placement_reachability.py` gained the directed both-grasps hook.

## Notes for review

- The campaign initially compared against a contaminated baseline (the
  second run's preserve step copied an already-rebuilt file); the
  runner now restores v1 from git and refuses non-2-bin baselines. The
  gate rule for iz5/6 changed from "must strictly improve" to "must not
  regress" (`<` instead of `<=`): a semantics-preserving rebuild keeps
  mapped bins identical, which the old rule would have failed.
- `campaign/old/` holds the four preserved v1 pairs (evidence of the
  pre-ST06 state).
