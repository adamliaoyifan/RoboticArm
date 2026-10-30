# 2026-10-30 -- OctoMap interface collection closeout audit

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

User asked whether previous OctoMap testing meets closure requirements. Task metadata records done/pass, and the four principal diagnostic findings have useful evidence. However, strict acceptance against the frozen collection plan is not fully demonstrated. The earlier 17:12 note was explicitly report-level; this review inspected artifact structures and collection/report code and found closeout gaps. This does not invalidate all diagnostic observations or change historical product verdicts. No task was reopened or experiment dispatched.

## Findings

1. Required executable replay is defective. COMMANDS.sh appends literal `trace=on/off` as a probe positional argument, while the executable requires argc=11 and reads trace mode from ULD_OCC_INTERFACE_TRACE. The recorded command has argc=12 and returns exit 3 / INVALID_ARGUMENTS before measurement. Verified using the recorded ON argv with output redirected to an unused temporary path and the diagnostic/ROS library paths supplied; no output was written. The initial bare invocation also failed to resolve its diagnostic shared library, confirming the script does not recreate its runtime environment. The shell file also omits trace environment variables, CLI help/tests/report recomputation commands and targets original evidence destinations. This is an actual replay defect, not a missing performance result.
2. Required accounting at each phase deadline is not delivered. INPUT_ACCOUNTING contains final buckets/assigned/inputs/conserved only; account_folder iterates IDs actually in ledger.sent rather than the frozen planned set. Overload final accounting mixes warmup, overload and recovery. No explicit send/drain/recovery cutoff events exist among client event types. Existing timestamps can support partial reconstruction, but exact boundaries and planned-but-unsent proof must not be invented. Current normal session waits for sends/queries, then drains; it does not explicitly enforce the full 60-second boundary. Report needs actual duration/coverage proof rather than sample-count inference.
3. Required resource and identity/capture artifacts are incomplete. The session records process CPU around query chains but has no 1Hz process RSS/CPU timeline. Root MANIFEST has platform/governor/product/tool and two binary hashes, but not the prescribed input/config/build/install inventory. No detailed trigger message bundle is present among inspected session artifacts; query arrays are reduced to metadata/hashes. Required capture/replay completeness and cleanup/limit verification need an explicit inventory and limitations, not just `status=complete`.
4. Coverage/validation output is narrower than its label. TRACE_VALIDATION checks event schema/sequence/time ordering; it is not a complete per-chain content/page/release/identity validator. The coverage document lists historical fields but does not deliver the planned nine-window schedule and health-gap/end-boundary recomputation. QUERY_SEGMENTS contains content hashes and release status, so some missing checks can be added offline. Missing event joins or unobservable timing should be explicitly reported rather than inferred from the trace-schema PASS.

Acquire ON/OFF ratio 1.104 and tiny cached-Surface ratio 5.66 are qualified in the report. They do not alone block diagnostic delivery: the plan permits high-perturbation relationship observations with absolute timing limitations. Optional middleware serialization and read-view/cache byte attribution unknowns likewise do not alone block this collection task. These differ from mandatory replay/accounting/capture omissions above.

## Closure assessment

The scientific observations can be archived as a useful partial investigation. Strict all-deliverables completion should not yet be independently signed off without an evidence supplement or an explicit scope exception. First repair replay metadata and perform offline validation/account reconstruction from existing evidence; identify truly absent raw data separately. User has since cancelled new 1cm tests, so this finding does not authorize rerunning those experiments. Any future necessary measurements belong to the new 2cm scope with a frozen plan.

Product acceptance remains separate: historical O5 not_evaluated, historical O6/O7 failures unchanged; new 2cm O5/O7 not proven by old 1cm diagnostics, resource attribution still open. Existing 2/5cm O6 evidence may be reused when identities and gates match.

## Pointers

- `docs/plans/OCC_O5_O7_INTERFACE_COLLECTION_G1_20261030.md`
- `docs/status/evidence/occ_interface_close_review_20261030/run01/FINDINGS.json`
- `docs/status/evidence/occ_o5_o7_interface_collection_20261030/run01/COMMANDS.sh`
- `docs/status/evidence/occ_o5_o7_interface_collection_20261030/run01/INPUT_ACCOUNTING.json`
- `docs/status/evidence/occ_o5_o7_interface_collection_20261030/run01/MANIFEST.json`
- `/tmp/elfin-occ-octomap-perf-cursor-g1/src/uld_occ_observer/test/benchmark/interface_campaign.py:127`
- `/tmp/elfin-occ-octomap-perf-cursor-g1/src/uld_occ_observer/test/benchmark/interface_report.py:72`
- `/tmp/elfin-occ-octomap-perf-cursor-g1/src/uld_occ_observer/test/benchmark/interface_session.py:370`
