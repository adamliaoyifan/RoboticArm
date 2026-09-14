# PF-R7 generation 3 bounded conditional acceptance

Date: 2026-09-14

Parent: `PFH-REMEDIATION-20260904`

Subtask: `PF-R7`

Supersedes: PF-R7 generation 2 acceptance only. PF-R1 through PF-R10 results
remain unchanged.

## Decision and objective

PF-R7 remains the independent G0-G4/G6 integration audit, but it no longer
scores known YOLO proposal misses as failures of platform-free geometry. The
availability defect is already owned by perception and remains visible in a
separate miss ledger. PF-R7 generation 3 answers the narrower release question:

> Given a valid, production-threshold cargo proposal, is the platform-free
> geometry path accurate, fail-closed, temporally correct, performant, bounded,
> and cleanly resettable?

This is a conditional acceptance, not a detector-recall waiver. A campaign
cannot pass without the required number of eligible observations, and a false
positive, wrong geometry, stale join, privileged-truth dependency, resource
growth, or missing evidence can never be relabelled as a detector miss.

PF-R7 generation 3 does not block the already-dispatched perfect-input POS-1
place/map experiment. Only one simulation stack may run at a time.

## Problems being accepted

1. **Conditional geometry correctness:** accepted cargo proposals must produce
   correct top, support and full 3D geometry across carryon, standard and large
   boxes.
2. **Fail-closed safety:** platform, edge strips, stale state, missing TF and
   cross-epoch data must not become valid cargo geometry.
3. **Performance and lifecycle:** the accepted path must meet the PF-R10 C2
   resource bars and leave zero simulator/eval residuals.
4. **Harness liveness:** a missing proposal, broken spawn, controller outage or
   stalled action must terminate with a stable class, complete evidence and a
   bounded reset. No condition may cause an unbounded retry loop.

Detector recall under low-texture simulation is reported but is not a PF-R7
generation 3 pass/fail metric. Real-sensor detector recall remains a separate
perception/hardware acceptance item.

## Decidable attempt classes

Every attempted case has exactly one terminal class.

| Class | Scored? | Effect on three-slot sequence | Required action |
|---|---:|---|---|
| `eligible_pass` | yes | advances the current slot | record T0/T1 and reset case |
| `eligible_fail` | yes | fails and stops the campaign | freeze T2 before teardown; no automatic rerun |
| `known_detector_miss` | no | neither advances nor resets | freeze detector-boundary T2, append miss ledger, substitute case within budget |
| `infrastructure_invalid` | no | neither advances nor resets | freeze available evidence, perform stack reset, substitute within budget |
| `fixture_invalid` | no | neither advances nor resets | record spawn/pose evidence and reset fixture |
| `evidence_invalid` | no | cannot close PF-R7 | repair capture before another attempt with the same failure signature |

### Exact `known_detector_miss` boundary

Classify an attempt as `known_detector_miss` only when all of the following are
proved from the same case and timestamp window:

- the spawned GT box is physically stable, fully in the camera frame and in the
  declared evaluation workspace;
- sensor ingest, TF, camera rate and inference health are valid, `/clock` has
  exactly one publisher, and the accepted production confidence floor remains
  `0.20`;
- no matching production-accepted cargo proposal exists before the 1.4 second
  availability deadline; an eval-only YOLO trace identifies either no matching
  proposal or best matching confidence below `0.20`;
- no online cargo cloud or geometry request was created from that box;
- no accepted edge-strip or other false cargo proposal exists; and
- the detector-boundary dump is complete and replayable.

The eval-only GT and low-confidence trace are classification/reference data
only and must never feed an online node.

The following are always `eligible_fail`, never `known_detector_miss`:

- a valid proposal existed but mask/cloud join, TF, filtering or geometry
  failed;
- an accepted edge/cabinet/platform false positive or fake cargo cloud exists;
- the box is out of frame, flipped, unstable or otherwise fixture-invalid;
- any top/support/size error exceeds a geometry limit;
- false `height_valid=true`, stale/cross-epoch fusion, online GT use, buffer or
  resource failure; or
- required evidence is absent or cannot be joined.

## Workload and repetition

- Test one clean committed revision, `dirty=0`, that descends from the PF-R10
  accepted revision `60dafb7deee50a6f3a76d48076b743bf3e3e1bc8`.
- Headless Fortress, `ROS_DOMAIN_ID=7`, semantic path enabled,
  `support_mode=auto`, `platform_z` omitted, production confidence floors
  unchanged, one `/clock` publisher.
- Use a recorded deterministic XY/yaw seed matrix. Each accepted slot contains
  six eligible cases: two carryon, two standard and two large. Substitutes use
  the next predeclared seed for the same size; seeds are never selected from
  metric results.
- Pass requires three independent accepted slots. Unscored attempts between
  eligible cases do not advance or reset the sequence.
- Per slot: target six eligible cases, at most 12 total attempts, at most six
  total exclusions, and at most three consecutive exclusions for one size.
- Campaign: at most 36 attempts, two full stack resets and 45 wall-clock
  minutes. Hitting a limit yields `inconclusive/perception_availability_blocked`
  or `inconclusive/infrastructure_blocked`, tears down the stack and releases
  the simulator slot. It is not an `eligible_fail`, but PF-R7 cannot close
  without the required eligible sample count.
- One `eligible_fail` stops the campaign after evidence flush. Diagnose/fix it
  before a new generation-3 campaign; do not restart a streak blindly.

## Acceptance criteria

### A. G0-G3 deterministic and fail-closed gates

Run the existing G0-G3 build, interface, synthetic algorithm, stamp/TF/state,
compatibility and fault-injection suites once on the tested revision. All must
pass with zero new failure, error or skip. In particular:

- invalid height never creates collision/place geometry;
- pick Z uses valid top-surface Z plus configured clearance;
- synthetic top/support errors are at most 5 mm, height at most 10 mm and
  width/depth at most 20 mm per axis;
- cross-stamp and cross-epoch fusion count is zero; no latest-TF fallback;
- missing support returns `TOP_ONLY`, and missing cargo cannot turn the
  platform into luggage; and
- all rejection paths retain stable machine-readable reason codes.

### B. G4 conditional live geometry

For every eligible case, define `t_proposal` as the first production-accepted
proposal matching the GT box. Score geometry only at and after that boundary.

- first valid top and first `FULL_3D` are each no later than 1.4 seconds after
  `t_proposal`;
- at least 30 settled observations are collected;
- conditional valid top rate and conditional `FULL_3D` rate are each at least
  0.95;
- top Z P95/max <= 15/25 mm; support Z P95/max <= 15/25 mm;
- height P95/max <= 25/40 mm; XY center P95 <= 30 mm;
- width/depth P95 <= 50 mm per axis;
- false measured-height count, accepted online fake-cargo count, stale/cross-
  epoch fusion count and online GT/spawner geometry reads are all exactly zero;
- all three slots cover exactly two eligible cases per catalog size; and
- the detector-miss ledger reports attempts, misses, best confidence, IoU,
  size, XY/yaw, seed and dump path separately, without entering the G4
  numerator or denominator.

### C. G6 performance and lifecycle

Score only eligible active windows, while still reporting whole-run health.

- accepted semantic geometry output >= 4 Hz;
- every configured bounded buffer peak is <= maxlen; pending-work buffer Q4
  mean is <= 0.5 x maxlen;
- executor lag Q4 mean <= 0.20 seconds and <= 1.25 x Q1 mean;
- workload-adjusted RSS coefficient <= 2 MiB/min for each online perception
  node, with each size represented by at least two eligible occurrences and at
  least ten RSS samples per occurrence;
- detector history-ring occupancy at its configured max is history, not
  pending work, and is reported but not failed solely for being full;
- expected camera/inference rates remain healthy; dual clocks or collapsed
  inference classify the attempt as infrastructure-invalid; and
- `scripts/stop_sim.sh` plus verified orphan-bridge cleanup leaves zero
  relevant launch, Gazebo, bridge, probe and eval processes.

### D. Evidence and reproducibility

- Every campaign records exact commit, dirty count, commands, parameters,
  seeds, environment, class for every attempt, reset events and teardown.
- Every unscored classification contains the evidence proving why it is
  unscored. A label alone is insufficient.
- Every failure bundle has `capture_complete=true` and
  `replay_possible=true`. Otherwise the attempt is `evidence_invalid`.
- Replaying the same boundary payload/config produces the same class and
  reason codes.

PF-R7 generation 3 passes only when A-D pass and all three bounded slots pass.
Gate 5 real-data acceptance remains explicitly inconclusive until the required
independently referenced real dataset is available.

## Reset and watchdog mechanism

The acceptance driver must implement explicit states:

`PRECHECK -> PRIME -> SPAWN -> OBSERVE -> CLASSIFY -> FLUSH -> CASE_RESET`

with `STACK_RESET` and terminal `PASS/FAIL/INCONCLUSIVE` branches.

### Case reset

After every attempt, including a detector miss:

1. freeze/flush the attempt manifest and triggered ring before deleting data;
2. cancel outstanding eval service/action requests and confirm terminal result;
3. increment the task/generation epoch, clear detector/filter/support temporal
   state and reject messages carrying the previous epoch;
4. delete the old box, confirm it is absent, spawn only the next declared case;
5. require zero stale-instance frames during the next scoring window.

Deadlines: 60 seconds for preflight readiness, 1.4 seconds for proposal
availability classification, 8 seconds total observation per attempt, 20
seconds for case reset, and 10 minutes per slot. A no-progress interval of five
wall seconds forces classification and reset; it never waits indefinitely.

### Stack reset

On controller unavailability, spawn service hang, duplicate `/clock`, abnormal
camera/inference rate, crashed node or case-reset timeout:

1. classify the attempt before reset and flush all available evidence;
2. stop the driver, run `scripts/stop_sim.sh`, then inspect and reap only orphan
   bridges/processes proven to belong to this campaign;
3. require no old pidfile, no relevant process, no GPU compute owner from the
   campaign and zero previous `/clock` publisher;
4. relaunch once, require exactly one `/clock`, then run a health preflight.

Two stack resets exhaust the campaign. The driver then emits `inconclusive`,
performs final teardown and exits nonzero. SIGTERM/SIGINT must use the same
flush-and-teardown path, so a directory-only half case cannot be scored.

## Required implementation and tests before simulation

The next PF-R7 TODO is `PF-R7-H1`, a bounded harness change:

1. add the terminal class enum and fail-closed classifier to
   `scripts/platform_free_height_gate4_eval.py` or a small reusable eval module;
2. add an explicit PF-R7 generation-3 CLI/profile with attempt, exclusion,
   reset and wall-time budgets; never change production launch defaults;
3. add the state-machine watchdog, signal-safe evidence flush and case/stack
   reset verification;
4. emit `attempts.jsonl`, `miss_ledger.jsonl`, `reset_ledger.jsonl`, slot
   summaries and one final machine-readable verdict; and
5. add focused tests for every class boundary, especially proposal-miss versus
   accepted-proposal/downstream-failure, exclusion budgets, stale-epoch reset,
   duplicate-clock reset, signal abort and missing-dump fail-closed behavior.

Focused tests must run three times with zero failures/errors/skips or retries.
Affected perception evaluator tests and the PF-R10 G6 classifier/RSS tests must
pass once without changed results. Only then may the bounded live campaign run.

Expected command after PF-R7-H1 implements the profile:

```bash
python3 scripts/pf_r7_bounded_acceptance.py \
  --out docs/status/evidence/platform_free_height/<run>/ \
  --slots 3 --eligible-per-size 2 --max-attempts-per-slot 12 \
  --max-exclusions-per-slot 6 --max-consecutive-size-exclusions 3 \
  --max-stack-resets 2 --campaign-timeout-sec 2700 \
  --ros-domain-id 7
```

The runner owns `scripts/stop_sim.sh` on every terminal path. If another sim is
active, it queues or exits without launching a second stack.

## Dump matrix

Evidence root:
`docs/status/evidence/platform_free_height/<run>/sha_<commit>/`.

| Boundary | T1 always-on trace | Trigger | T2/T3 payload | Window | Format / budget | Inspection or replay |
|---|---|---|---|---|---|---|
| sensor/TF -> YOLO proposal | stamps, rates, TF status, bbox/class/confidence/acceptance reason, queue/drop counters | no matching accepted proposal by 1.4 s, clock/rate fault | RGB, depth, camera info, TF, all raw/eval-only YOLO proposals and separate GT reference | 1 s pre-spawn through 2 s after deadline | MCAP + JSON/PNG, <= 50 MiB/attempt | classifier unit replay plus `attempt_manifest.json` inspection |
| proposal -> mask/cargo cloud | proposal and generation IDs, mask/cloud counts, join/drop reasons | accepted proposal but missing/empty/mismatched mask or cloud | proposal, mask, synchronized depth/cloud, filtered cargo cloud, join buffers | 1 s pre to 2 s post first divergence | MCAP + NPZ/JSON, <= 60 MiB | gate4 boundary replay/classifier tests |
| cargo cloud -> top/support/full geometry | input/output IDs, fit stats, validity/source/reason codes, latency | timeout, invalid geometry or any metric failure | cargo/raw support clouds, TF, candidates/inliers, final detection and separate GT | 1 s pre to 2 s post | NPZ/JSON, <= 80 MiB | existing PCA/RANSAC replay directory and scoring replay |
| geometry -> evaluator verdict | frame/epoch IDs, conditional denominator, errors and class decision | inconsistent class, stale frame, threshold failure | frames JSONL, reference JSON, exact classifier inputs/outputs | full attempt, bounded | JSONL, <= 10 MiB | focused verdict replay |
| reset/lifecycle/resources | state transitions, deadlines, PID/process/clock/GPU identity, buffer/RSS/lag samples | no progress, reset timeout, duplicate clock, crash, resource bar | logs, process snapshots, C2 raw/summary and reset ledger | 5 s pre through teardown health proof | JSONL/text, <= 25 MiB | reset state-machine tests and verdict inspection |

T0/T1 are mandatory for all attempts. T2 is mandatory for every exclusion or
failure. T3 is enabled only after T2 localizes a module and is marked timing-
unscored. Limit retained data to 100 MiB per normal eligible case, 200 MiB per
failed/excluded case, 500 MiB per slot and 1.5 GiB per campaign. Keep every
failure/exclusion and all final passing summaries; compress arrays/MCAP. T1
overhead must remain below 5% wall time and 75 MiB RSS.

## Completion and next development TODO

PF-R7-H1 is the next PF-R7 action and does not require the simulator. Once its
focused tests pass, run exactly one bounded generation-3 campaign. Do not
resume the old unbounded generation-2 streak.

The simulator slot may meanwhile be used by the already-claimed `POS-1`
perfect-geometry place/map task. PF-R7 generation 3 and POS-1 remain separate:
PF-R7 proves conditional perception geometry; POS-1 proves placement and
occupancy updates with perfect 3D input.
