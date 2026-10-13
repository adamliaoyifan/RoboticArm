# ST01 generation 2 -- pinned WIP integration baseline

Parent: FRAME-MIGRATION-20260929. Owner: eng/codex/gpt-6.
Base: c8ce801676db52c9f15925876253ce8b0846c68d, containing the ST00 plan;
ST00 generation 1 passed against this revision plus its recorded shared sources.
User requested completion of ST01. This release resolves the queued source-set
and build-profile decisions; the module implementation plan remains unchanged.

## Source scope

`docs/plans/frame_migration_st01_sources.json` pins exactly 90 WIP files by
SHA-256 (16,314,169 bytes), feature lineage and snapshot owner. All other tracked
files retain base-commit bytes. It explicitly excludes nine removed historical
atlas reports (retain base), generated visualization files and an unrelated audit.
Feature provenance is inferred from code and existing owner notes, not an
assertion that prior feature gates passed. Codex owns this integration snapshot;
prior MOTION-OCC and atlas feature certifications remain open/unchanged.
No cherry-pick of unreviewed shared state, source glob or shared checkout reset.
Copy only manifest entries after hash verification. The 13 untracked inventory
sources are included; their Python helpers, tests, configs and runtime atlas
artifacts form the selected WIP closure. Source changes after dispatch are errors.
Only demonstrated build/import wiring defects may be repaired with separate
commits and preserved failure logs. No coordinate or algorithm migration.

## Environment and procedure

Use `/tmp/elfin-frame-migration-st01`, branch
`integration/frame-migration-st01-20260929`. Ubuntu host, /usr/bin/python3,
ROS 2 Humble `/opt/ros/humble`, CPU only. Start bash without user startup files,
clear inherited overlay variables using `env -i`, retain HOME/PATH/locale only,
source Humble; build from empty build/install/log directories. No primary-workspace
install overlay, simulation, hardware, Docker IKFast generation or atlas resampling.
Precomputed grids are pinned inputs, not newly certified artifacts.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to luggage_description luggage_msgs luggage_perception luggage_packing luggage_planning luggage_gazebo
source install/local_setup.bash
python3 scripts/check_frame_contracts.py --mode baseline --output docs/status/evidence/frame_migration/ST01-20260929/baseline
python3 scripts/check_frame_contracts.py --mode migration --output docs/status/evidence/frame_migration/ST01-20260929/migration
python3 -m unittest discover -s src/luggage_description/test -p test_frame_contracts.py -v
```

Evidence paths above live in the primary workspace; pass absolute output paths
from the isolated checkout. Do not create a satellite mailbox or role-note tree.

## Acceptance

One deterministic run per pinned revision: build exit 0 and package summary with
zero failed/aborted packages; baseline exactly 50/50 pass, exit 0; focused tests
exactly 15/15 pass; migration exit 2, exactly F01--F24 pending. AND across these
checks encodes 100% correctness of this finite deterministic workload, not a
statistical claim about live per-unit reliability. Every interface source tracked,
84 ST00 manifest hashes unchanged unless a justified wiring repair explicitly
records its before/after identity. Working checkout status empty before/after
verification (ignored build products allowed and recorded).
Record every package.xml dependency's workspace source, ROS prefix or installed
system package origin; statically enumerate Python imports in selected Humble
packages and inventory tooling, classify optional/legacy-only dependencies, and
check required imports against the isolated installed environment. Import actual
installed modules and SelectPlacePath binding without creating ROS nodes; record
resolved paths/prefixes and reject primary-workspace leakage. Missing required
origin/import is a failed gate, never implicitly optional. ROS1 reference scripts
and optional hardware/learning adapters must be explicitly identified as outside
this CPU/Humble build profile. A successful build is not a full runtime proof.

## Evidence and failure capture

Primary `docs/status/evidence/frame_migration/ST01-20260929/`: manifest with commit,
environment, commands, times, source hashes and dirty state; pinned-source manifest;
package dependency and Python resolution JSON; full build log and package summary;
baseline/migration checker bundles; unittest log; final summary. Preserve each
failed attempt separately before repair. Build/import failure boundary payload is
the exact command, environment, traceback/compiler logs, package manifests and
pinned source commit; retain entire invocation, no timing-window approximation.
Limit 128 MiB logs/JSON per attempt, 512 MiB total (Git contains sources). Check
capture files exist and have successful serialization; do not erase failed logs.
Replay with this plan's command in a fresh worktree at the recorded revision.
