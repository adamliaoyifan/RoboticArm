# OCC query concurrency evidence and replay

The G2 pressure fixture preserves the original mapper main, callback groups,
heartbeat period and lease. It adds passive boundary observations and a load
client; it starts no simulation or motion stack. The registered baseline is
`b33dda919521ef043fa9e30b07dcc79c87f76fce`.

Runtime, placement and concurrency campaigns construct child environments with
`scripts/g3/occ_query_runtime_environment.py`. The helper sources ROS Humble and
the requested isolated install, discarding inherited ROS overlays. The owner,
client, capture wake and teardown wake use that same environment. Launching the
CLI from a shell without ROS setup must not change child dependency resolution.
Setup errors fail before scoring; preserve them separately from product cases.

`occ_query_concurrency_client.py` owns scheduled requests, completion records
and the wire journal. A request keeps its dispatch phase even if it completes
after the scoring window. `observe_backlog` alone updates the maximum age.
A queued integrate retains the phase in which its cloud was published.

`occ_query_wire_journal.py` retains lossless gzip-compressed requests and
responses for the whole pressure window, separately from the ten-second
failure ring. Its JSONL index joins case/sequence/direction, records lengths
and SHA256, and allows explicit unanswered requests. The journal has a 4GiB
compressed limit, including the index, and a 64MiB raw-message limit. The reader
checks every indexed payload and detects unindexed files, corruption, duplicate
or orphan entries. It keeps metadata in memory and decompresses one message
at a time. The campaign's existing 8GiB/run and 32GiB total limits still apply.
Compression is synchronous client instrumentation; its live T1 overhead must
pass the existing gate before formal certification. An offline component cost
measurement does not establish that gate.

The owner captures mapper parameters and actual Store snapshot-return events.
A callback's ending Store identity can belong to a newer concurrent write;
refused legacy volume responses therefore use the snapshot observed *inside*
that callback. Missing or ambiguous joins are `not_evaluated`.

Run replay after sourcing ROS Humble and the isolated candidate install (its
IDL includes both original and new services):

```bash
python3 scripts/g3/occ_query_eval.py replay \
  --bundle <campaign>/b0_repeat0/candidate --out <fresh-evidence-directory>
```

Replay runs in a fresh process pinned to the campaign's frozen baseline
primary modules. `occ_query_read_oracle.py` checks original stats/full-volume
or typed summary/ROI/distance/surface answers from those facts, without loading
the candidate query implementation. `occ_query_identity_oracle.py` separately
recomputes the complete schema-1/2 voxel hash, including private evidence,
active masks, geometry membership, setup bindings and publication provenance.
The fixed pressure workload only integrates after named-box setup; this hash
oracle explicitly refuses a ledger it cannot reconstruct. Geometry rasterization
comes from the registered baseline. Legacy ring evidence may recover only the
unique fixed wiring prefix; it never invents arbitrary missing inputs.

`result.json` reports semantic replay; a verified conservative budget refusal
can be semantically correct while the workload completion gate fails.
`gates.json` independently checks full 120-second load, exact schedule, request
joins/statuses, backlog, P95/max, both heartbeat directions, lease observations
and resource limits. Runtime captures carry monotonic observation timestamps.
Lease readiness and transitions recorded after the client observation window
are teardown evidence; they do not change the pressure verdict. Unknown-time
failures and untimestamped rejection counters remain conservative failures.
The required 10-second pre/5-second post window and native lease stay intact.
Missing samples remain `not_evaluated`; observed failures
remain failures. Replay's exit code describes semantic replay, so inspect
`gates.json` for the pressure verdict. Neither file certifies overall G2.

Whole-write workers keep ordinary timing samples free of extra stage observers.
The diagnostic `occ_query_write_fixture.py --profile` mode additionally records
each owner `apply_delta` boundary and its work/surface counters under
`diagnostic_index_stages`. A later empty refresh can overwrite the owner's
last-work value; use the complete stage list to explain transaction work.
The observer restores the fixture owner's method on success or failure and
does not change primary facts or production code. Its cProfile/stage timings
are diagnostic and cannot certify the write regression gate.

The campaign stops its owned processes before decoding/rebuilding evidence.
Its exit code uses the strict gates. Every replay records verifier revision,
dirty state, helper snapshots/hashes and frozen-source integrity. Existing
failure evidence is read-only; each replay requires a fresh output directory.

Focused checks (no ROS graph):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/luggage_perception/test/test_occ_query_wire_journal.py \
  src/luggage_perception/test/test_occ_query_read_oracle.py \
  src/luggage_perception/test/test_occ_query_concurrency_gates.py \
  src/luggage_perception/test/test_occ_query_eval_cli.py \
  src/luggage_perception/test/test_occ_query_capture_health.py \
  src/luggage_perception/test/test_occ_query_trace_buffer.py
```
