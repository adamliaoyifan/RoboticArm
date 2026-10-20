# ULD discrete checked luggage workload

- parent: ULD-INCH-20261020
- owner: codex/gpt-6
- research_base: b6ded5e9cbdb624fd7388f1725be5037d4f1ba49

## Scope

Change RL batch default sampling and fixed dataset generation to repository
INCH_SIZE_TIERS inch22/24/26 only. Preserve lying local XYZ as size_wdh_m;
no independent-axis sampling/interpolation. Default weights equal, configurable.
Read source without importing Gazebo dependencies; freeze source hash/selected
values into provenance. No Gazebo catalog edits or long scored retraining in this
iteration; old campaign remains intact and is not represented as luggage results.

## Acceptance

One codex owner implements and verifies all changes. Positive gates are AND over
deterministic cases (100% required, not statistical reliability estimates):
- Source values exactly match inch22/24/26 in suitcase_visual.py; exclude inch20,
  odd sizes and generic tiers. Online generated samples and manifest samples all
  belong to these three complete triples; reject incompatible manifests.
- Equal sampling default, valid custom weights, reject malformed/negative/zero
  weights; deterministic reset/resume sequences including a one-hot weight case.
- Generate 500 train /100 val /100 test, five discrete size/order regimes,
  40/60 items, zero identical complete sequences crossing splits. Repeat generation
  with same seed is byte-identical; save tier IDs, masses and source provenance.
- Legacy continuous profile is explicit and retained for old fixtures.
- Run tests/packing_rl and tests/topk once on CPU; all pass. Run batch YAML
  preflight on actual ULD and one PPO update (4 transitions, 1 CPU worker) on a
  newly generated discrete manifest, plus one fixed test episode evaluation;
  counts resolve, final return equals volume utilization, no invalid geometry.
- Save resolved config, manifests, stdout/stderr and any failure boundary under
  docs/status/evidence/uld_inch_20261020/. Startup failures consume no scored budget;
  smoke is wiring evidence, never a policy-quality claim. No simulator required.
- Commit only owned research changes; preserve existing dirty source size table,
  primary scene/catalog files, unrelated research docs and campaign artifacts.
