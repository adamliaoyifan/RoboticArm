# ULD batch first training and evaluation chain

- parent: ULD-BATCH-20261019
- owner: codex/gpt-6
- research_base: 4daca9b
- workspace_base: 3cb4ec16
- design: research/step-bpp-topk/docs/topk/ULD_BATCH_POLICY_PIPELINE.md

## Scope and ownership

One owner implements and integrates buffered batch volume-delta RL, trains actual
policies, selects checkpoints on validation, evaluates on held-out batches against
three baselines, and records defects/fixes/results/unresolved limitations. No robot
motion. Preserve unrelated modifications. Reuse authoritative ULD geometry and
the existing parallel PPO lifecycle, introducing a v2 interface rather than silently
changing old checkpoints. All design scheduler/action/reward invariants apply.

| Subtask | Owner | Depends | Scope |
|---|---|---|---|
| CORE | codex/gpt-6 | none | scheduler, batch env, Transformer policy, configuration and focused verification |
| INTEGRATION | codex/gpt-6 | CORE | workload, training/validation/test/baselines/report, fix integration failures |

## Acceptance

CPU deterministic fixtures must all pass (100% expected per fixture, not a streak
or reliability-rate estimate). `tests/packing_rl` plus `tests/topk` must pass.
Add B=0/1/4 scheduling, N=1/3/8 accounting, full-buffer masks, terminal drain,
stale action after unchanged occupancy, hidden-future observation equivalence,
exact return=final utilization (1e-9), placement orientation and invalid action
atomicity. Hand fixtures demonstrate rejecting/defering can improve volume with
exhaustive finite-action enumeration. Two spawn workers preserve independent
schedulers and exact update-boundary CPU resume; injected failures preserve state.

Run a measured CPU rehearsal on actual ULD: report candidate/step/optimizer
latencies, peak RSS and transitions/sec. Freeze the first campaign configuration
before scored evaluation. Default target is 1M transitions per training seed
0,1,2 from the design; initial campaign may use a shorter declared budget if the
measured estimate exceeds two hours total training. In that case use the largest
equal multiple of rollout size within two hours, at least 16,384 transitions/seed;
if the minimum exceeds the budget, optimize measured bottlenecks first. Record
chosen budget before launch and never present it as converged training.

Create disjoint train/val/test workload families. Validation and test each contain
100 fixed batches, 20 each of five documented size/order regimes, with supplied
volume and shortage/overcapacity flags. Candidate/buffer/information limits are
identical across learned and baseline policies. Train three seeds with equal
transition budgets and >=3 validation checkpoints, selecting by mean validation
utilization only. Test each selected policy once per batch; random baseline seeds
0,1,2 and two deterministic baselines. Report per-seed mean/median/P10 utilization,
paired per-case deltas and 10,000-resample paired bootstrap CIs. All cases must
finish with no accounting/geometry violations. Weak/worse learning is a reported
outcome, not a reason to hide cases or change the test set. Improvement is claimed
only if positive lower paired CI against each named baseline for all three seeds.

Outputs include initial/last/best checkpoints; config/revision/dataset hashes;
train/val curves; per-case test results and action traces; report of problems with
observed symptoms, cause evidence, repair, verification and residual limitations.
Fresh-process resume/evaluation must work. No product quality threshold is implied.

## Evidence and failure captures

Primary root: docs/status/evidence/uld_batch_policy/<run>/. 8 GiB total initially;
before exceeding it, reduce trace/checkpoint retention without losing failures.
CPU <=4 workers, <=2 threads each initially. No sim lifecycle work required.

| Boundary | Always on | Failure payload | Window/budget | Replay |
|---|---|---|---|---|
| input/config | revision, hash, seed, split, workload IDs | config/exception/manifest | current case, 2MiB | CLI config |
| scheduler/geometry | decisions, IDs, volumes, masks summary, timing | current world/scheduler/observation/action/RNG | last transition, 32MiB | restore + step |
| worker IPC | PID, operation, serial | last snapshots and command history | last RPC, 64MiB | restore + command replay |
| PPO | update/loss/grad/reward/throughput | rollout/model/optimizer/RNG | one update, 512MiB | checkpoint + update |
| evaluation | every case metrics, decision trace, split | case state/checkpoint/traceback | failing case, 64MiB | evaluate exact case |

Preserve failed artifacts before retry. Startup failures consume no scored cases.
Final scored runs use committed implementation; unrelated dirty files are listed
and hashed. Rehearsals may repeat after diagnosed repairs. Record actual command,
environment, sample counts, timings, artifact inventory and teardown state.
