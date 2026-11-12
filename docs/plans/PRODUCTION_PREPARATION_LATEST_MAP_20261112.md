# 2026-11-12 -- Latest O map recertification before plan publication

- status: approved
- dispatch_ready: yes
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-PREPARATION-LATEST-MAP
- generation: 1
- owner: codex/gpt-6
- base_revision: ca1f8f65e3011ad6f745356eda8d2f1dde7572b2
- authorization: User explicitly selected latest-map full recertification of retained candidates/trajectories.

## Decisions and scope

Default O preparation plans on one pinned complete snapshot. Before returning,
retain the selected original candidate and six timed original legs. If original
O source and baseline scene remain current, preserve the existing final gate.
If O has advanced within the same epoch/config/hull/model/deployment, acquire
latest complete measured volume and a fresh full baseline scene under the same
request deadline. Rebuild all three load-state collision scenes, retaining all
unrelated obstacles/unknown cells and the original frozen box/tool geometry.
Do not run IK, connection search, retiming, change pick/J6/grasp/place, or remove
new obstacles. Native controller-sampled full-chain certification checks the
exact original legs in the refreshed scenes. Rebind only map/snapshot identity
and native certificate/digest; legs and physical geometry remain byte-identical.
Issue/retain the final latest source in the preparation owner registry.

Admission of the original placement must also survive: fixed hull/resolution,
complete fresh candidate volume known-free, and identical observed support patch
under the complete footprint (state/height/confidence, source cells including edge
slivers). This conservative unchanged-support proof avoids inventing support or
rerunning the entire candidate search. A changed patch returns explicit incomplete
support evidence; a later request may recompute placement. No physical no-space
conclusion follows. No unexplained geometric threshold is introduced.

Frozen observation/contact relation and current measured start remain valid.
Compare full baseline semantics and latest source after native recertification;
if either advanced, retry capture/certification of the same legs within the same
deadline, bounded by explicit preparation_refresh_attempts (default2). Exhaustion
returns source instability, never a partly issued plan. Reset/backward revision,
config/hull/model/calibration change, expired data, observation revocation, start
motion, native failure, malformed receipt or deadline all refuse. The existing
observer_max_age_ms=1000 remains unchanged; it is separately awaiting measured
sensor/latency evidence. G explicit compatibility retains its strict final gate
in this O-authorized unit. No default production client switch is claimed.

## Acceptance

System Python/Humble production prefix plus explicit source runner; deterministic
fixtures are an AND over cases, not cycle reliability. One run each, all selected
cases pass with no skipped selected cases. No physical robot or Gazebo.

1. Successful refreshed identity/native receipt binds the new snapshot/map, while
   original legs (all joints, J6 turns, knots, derivatives, efforts, times), poses,
   attachment and event barriers compare exactly equal. Old certificate is not
   accepted as a receipt for new scene/draft. No planner/IK/retimer called.
2. Full three-phase scene fixtures prove a new unrelated obstacle and unknown
   cargo representation survive in native inputs; refused certificate produces no
   plan. Bad receipt candidate/model/phase digest/draft digest/certificate refuses.
3. Source epoch/config/geometry/generation/reset/backward identity refuses;
   support footprint includes positive edge slivers, retains source resolution/
   absolute floor, rejects unknown/occupied candidate volume and changed support.
   Remote support-map change may survive if local patch unchanged; no guessed box.
4. Real core/root callbacks consume refreshed plan/source consistently. Unchanged
   source follows old gate. Changed source invokes complete certification, then
   stable current source check; another update retries at most configured attempts.
   Expired deadline/unstable source/observation revoked/start moved/capture failure
   keeps response inactive, no issued token; lock and leases retain existing rules.
5. Focused tests: new refresh fixtures plus profiling, operation planner, scenes,
   certification, issuance, observer, candidates and common placement consumer tests.
   Explicit native opt-ins excluded from offline matrix, not counted as coverage.
   Freeze exact commands and fixtures before execution; failures preserve replay.
6. Rebuild closure and installed-origin/hash check new refresh modules. All build
   packages/checks pass, owned diff whitespace clean. Native/backend and full-model
   simulation integration remain required before whole-operation baseline closure.

## Evidence

Root docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PREPARATION-LATEST-MAP/run_01/.
Freeze base/dirty/owned input hashes; preserve unrelated runtime changes unstaged.
Every deterministic fixture source is a lossless replay recipe, truth separate
from admitted inputs. No eval artifact mutation under single scored budgets.

| Boundary | Trace | Trigger | Failure payload | Window/budget | Replay |
|---|---|---|---|---|---|
| Pinned -> latest source | full identity, geometry, counts | stale/changed input | exact fixture+new/old source, reasons | one request <=16MiB | named test |
| Original legs -> refreshed certifier | all three scenes, draft/receipt digests | mismatch/collision/timeout | source recipe/CDR fixture, plan equality, trace | one candidate <=32MiB | named test |
| Final source -> issuance | source/certificate/digest/attempts | another update or capture failure | request/response trace, registry observations | one request <=16MiB | callback test |
| Build/install | package/hash/timing | failure | logs/receipt/input hashes | <=32MiB | production build/check |

Total<=128MiB. Capture health requires command, immutable replay inputs, logs,
JUnit and complete manifest before rerun. Fixture timings are not live throughput
or collision success-rate evidence. Teardown not_needed (no external processes).
