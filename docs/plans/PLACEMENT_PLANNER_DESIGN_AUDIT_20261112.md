# Placement and planner design audit -- 2026-11-12

- status: ready
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-DESIGN-AUDIT
- generation: 1
- owner: eng/codex/gpt-6
- base_revision: bcebf44d5c0aa353c66e1de34d09cb2ab1dd6df9
- dispatch_ready: yes

## Scope

User requests collection of available design data and evidence plus a detailed
review of current ComputePlacement and planner impediments to the complete
chain and physical validation. Read source, effective configuration artifacts,
existing finite captures and bag audit; run only bounded offline introspection
and focused behavioral probes if supported locally. No behavior fixes, ROS
controller calls, simulation, map writes or new physical data collection.
Preserve shared dirty files. Distinguish source code, installed copies,
historical native observations and unmeasured live deployment parameters.

## Acceptance

A single audit report covers all boundaries: measured detection/top; O and
support; ComputePlacement generation/ranking/retention; atlas seeds; fixed
pick attachment and target coupling; MoveIt connection/certification; release,
empty retreat and runtime adoption; actual launch/install/controller readiness.
For each material finding record ID, priority, evidence class, source path and
line/hash or existing case identity, mechanism, practical effect, remaining
unknown, smallest next verification/correction and user decision if required.
Do not call a configured limit unreasonable without its mechanism and limit
of evidence. Include a complete clearance/support/release inventory in reviewed
paths and separate search, time and dimensionless0.05 constants.

Data matrix covers current model/scene/atlas/O configuration; actual historical
camera/pointcloud/TF/joints data; perception/grasp uncertainty; controller and
release/settling coverage; missing measured accuracy/identity signals. Each
row says available, inspected sample/count, supported conclusion, unavailable
conclusion, exact retained evidence and next acquisition/procedure. Existing
sample counts and test counts remain historical, not new pass claims.

One read-only collector run records exact base revision, complete dirty list,
selected file hashes, source-vs-install parity for named entry points and
configurations, bounded source excerpts for every finding and summaries/hashes
of cited prior manifests. Max retained25MiB; bounded offline run <=10min;
no statistical campaign, reliability or performance verdict. Each finding's
provenance must resolve; all hash targets must exist. Rehash selected current
inputs at end: changed files are marked concurrent_change and excluded from
stable-source claims rather than discarded or silently updated. A valid audit
can report missing or divergent installs and unclosed product gates.

## Evidence capture and verification

| Boundary | T1 | Trigger | T2 | Window/budget | Replay |
|---|---|---|---|---|---|
| source/config/install | path, SHA, revision, dirty state, read time | missing/mismatch/concurrent change | retained selected text and mismatch metadata | one snapshot, <=10MiB | collector.py |
| historical failure/data summaries | exact case, manifest hash, result | missing or unqualified reference | original pointer and missing reason; no destructive rerun | bounded summaries, <=5MiB | inspect JSON pointers |
| offline probes if run | input/config, result, count, time | unexpected result/exception | exact inputs, output, traceback and source hash | one finite matrix, <=5MiB | recorded command |

Capture-health gate is AND over provenance coverage, retained artifact hashes,
size budget and a complete method/limitations section. No sample success-rate
claim is encoded. Physical accuracy, live parameter overrides and complete
whole-cycle reliability remain not_evaluated without corresponding data.

## Deliverables

- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-DESIGN-AUDIT/run_01/
- docs/reviews/PLACEMENT_PLANNER_DESIGN_AUDIT_20261112.md
- docs/agents/eng/2026-11-12_*_placement-planner-design-audit.md
- concise user summary plus full prioritized report and data acquisition matrix.
