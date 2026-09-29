# Frame contract baseline

User-authorized scope (2026-09-29): deliver the interface inventory, executable
strict checks, and offline comparison cases discussed in the frame reviews.
One bounded implementation unit; no delegated agent or simulation required.

## Scope

- A normative staged-v2 contract document and machine-readable interface
  inventory covering perception, map, placement, pick/place, selector, atlas,
  execution, scene/attach, commit, recovery, eval, and legacy boundaries.
- ROS-free strict validators and complete pose transforms under
  `luggage_description`, with no silent frame/default-datum inference.
- An offline checker with explicit baseline and migration-readiness modes.
  Baseline success must never imply existing ROS nodes have adopted v2.
- Hand-calculated positive/negative geometry and semantic cases, runnable with
  plain Python, including original scene-transform helpers where applicable.
- Durable per-case inputs/expected/actual results and source/revision identity.

No changes to existing ROS messages, node behavior, active maps, atlas data,
calibration, simulation, or hardware in this unit. Existing dirty edits are
preserved. Runtime integration remains explicitly pending in the inventory.

## Owner and revision

| Unit | Owner | Dependency | Base revision | Plan revision | Commit evidence |
|---|---|---|---|---|---|
| BASELINE | eng/codex/gpt-6 | none | c07fe95c6fb26c09ef339ee4ebe6f5e574a8048f | exact commit recorded in dispatch | implementation commit recorded at completion |

## Acceptance

Environment: workspace Python 3, CPU, no ROS/Gazebo/IK/hardware. Fixed synthetic
inputs, no random seeds. Run each deterministic case once; all cases must
match their numerical oracle (position/component tolerance 1e-9 m, quaternion
rotation equivalence 1e-9 in sign-invariant component distance) or exact
expected rejection code. This is an AND over deterministic obligations with
100% required per-case correctness, not an estimate of stochastic reliability.
At least 20 cases, with nonzero translation/z, yaw 90/180/non-axis-aligned,
box-center/TCP offsets, stacked surface heights, q/-q, distinct tool branches,
missing/wrong frame, version, datum, time/scene identity and invalid numeric
payloads. No zero-sample pass; empty suites/inventories fail.

Commands from workspace root:

```bash
python3 scripts/check_frame_contracts.py --mode baseline --output docs/status/evidence/frame_contracts/baseline
python3 scripts/check_frame_contracts.py --mode migration --output docs/status/evidence/frame_contracts/migration
PYTHONPATH=src/luggage_description python3 -m unittest discover -s src/luggage_description/test -p test_frame_contracts.py
git diff --check
scripts/check_agent_contract.sh
```

Baseline: exit 0, zero failed cases, all declared source pointers exist, unique
interface IDs and valid contract profiles, every interface linked to offline
cases. Migration: exit 2 with nonempty explicit pending-interface list; this is
the expected refusal of an unmigrated production chain, not a geometry failure.
Focused unittests must pass and test malformed suites/inventories and CLI
failure behavior in addition to geometry. Existing unrelated contract/lint
failures must be enumerated rather than silently fixed or claimed passing.

Revision rule: record HEAD, dirty paths/count and SHA-256 for checker, cases,
inventory, validator/transform source and relevant original scene helpers.
Commit only this unit's files; do not stage other agents' work. Evidence
represents HEAD plus these exact dirty hashes, not clean-HEAD certification.

## Evidence and failure capture

Output directory must be newly created; refuse overwrite. Each run writes
manifest.json, cases.json, summary.json, and the exact suite/inventory snapshots.
All records must be flushed before the checker returns, including on case
failure. Independent expected values are stored separately from actual results.

| Boundary | Always-on trace | Failure trigger | Preserved payload | Window | Budget | Replay |
|---|---|---|---|---|---|---|
| fixture to validator/transform | case ID, operation, duration | unexpected accept/reject/output | complete input, expected, actual/error and source identity | one synchronous case | 4 MiB total/run; no sensor data, no compression | same checker with saved suite/inventory |
| inventory to readiness gate | IDs, paths, migration state | malformed inventory or incomplete migration | exact inventory, issues and pending list | one invocation | included above | same command/mode |

Keep success/failure artifacts under the named evidence root; repeated attempts
use fresh directories. Capture-health requires nonempty results and all listed
artifacts; exceptions must produce nonzero exit. Timing is informational only;
no performance claims. Pre/post rolling windows are inapplicable to these
stateless, fully specified offline calls.
