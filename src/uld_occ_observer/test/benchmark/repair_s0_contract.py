"""Offline contract for the 2 cm OctoMap repair acceptance.

This module records the approved gate numbers. It does not change the observer,
and a synthetic pass does not certify a live capture.
"""
from __future__ import annotations

PLAN_REVISION = "c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76"
PRODUCT_BASE = "4c8f1862ec357cb01616f4b04be636227461cf44"
TOOL_BASE = "57d8c0bfe23fee54920c89c06a2b6bdcb97de976"
GENERATOR_VERSION = "o6-load-v1"
SCHEMA_V1 = "occ_octomap_repair_s0_v1"
SCHEMA = "occ_octomap_repair_s0_v2"

# P95 is the ceil(0.95*n)th ascending sample. n=30 selects the 29th.
INTEGRATE_30K = {"p95_ms": 500.0, "max_ms": 1000.0}
INTEGRATE_100K = {"p95_ms": 2000.0, "max_ms": 5000.0}
QUERY = {"p95_ms": 100.0, "max_ms": 250.0}
INIT_LIMIT_S = 30.0
RSS_LIMIT_KIB = 2 * 1024 * 1024
READ_VIEW_CACHE_LIMIT_BYTES = 512 * 1024 * 1024
QUEUE_LIMIT_MS = 1000.0
HEALTH_GAP_S = 1.0
NORMAL_INPUTS = 240
NORMAL_QUERIES = 120
NORMAL_DURATION_S = 120.0
OVERLOAD_INPUTS = 600
RECOVERY_INPUTS = 60
RECOVERY_QUERIES = 30
DRAIN_S = 30.0
WAIT_TIMEOUT_MS = 2000.0
WAIT_MAX_AGE_MS = 2000.0
SCORED_SAMPLES = 30
WARMUP_SAMPLES = 3
O6_GROUPS = 96
O5_GROUPS = 4
O7_WINDOWS = 9
O8_RUNS = 3
O8_HOLD_S = 10.0
O8_HOLDS = 3
# gen7: the saturation check is its own frame pass with per-frame receipts,
# RSS medians come from real timestamped windows, and the duplicate-id replay
# is judged as its own hold kind instead of standing in for occlusion.
O8_SATURATION_FRAMES = 30
O8_RSS_WINDOW_S = 60.0
O8_HEALTH_MIN_SAMPLES = 30
O8_HOLD_KINDS = ("no_input", "occlusion", "duplicate_id")
O9_PAIRS = 30
O9_RATIO = 1.10
O8_RSS_GROWTH = 0.05
CLIENT_EXECUTOR_THREADS = 4
INTEGRATE_THREADS = 1

HISTORICAL_O6_FAIL_ID = "r01_b00_cuboid_c30000_o50_x50"
HISTORICAL_O6_FAIL_P95_MS = 517.297808

QUERY_METRICS = (
    "acquire_ms",
    "roi_ms",
    "surface_first_ms",
    "surface_after_update_ms",
    "surface_repeat_ms",
)

PUBLISH_FIELDS = (
    "input_id", "phase", "publish_end_ns", "callback_enter_ns",
    "disposition", "reject_reason",
)
INTEGRATE_FIELDS = (
    "input_id", "terminal_count", "integrate_ms", "queue_age_ms",
    "observed_revision", "receipt_received",
)
QUERY_FIELDS_V1 = (
    "query_id", "op", "status", "latency_ms", "age_ms", "covers_observation",
    "expected_observation_id", "acquire_ms", "roi_ms", "surface_ms",
)
QUERY_FIELDS = QUERY_FIELDS_V1 + ("epoch", "observed_revision", "content_hash")
# v1 ledgers are judged as stored. Replay does not invent epoch, content, or plan ids.
# A v1 ledger that lacks those fields is not_evaluated. It is not rewritten into v2.
SCHEMA_V3 = "occ_octomap_repair_s0_v3"
# v3 adds the CLOCK_MONOTONIC publish boundary: the planned deadline, the real
# publish return, and the cloud header stamp in its own field (never standing
# in for the boundary), plus per-request query start/end and receipt problems.
# v2/v1 ledgers are judged as stored and never rewritten into v3.
PUBLISH_FIELDS_V3 = PUBLISH_FIELDS + ("publish_planned_ns", "input_stamp_ns", "clock_domain")
INTEGRATE_FIELDS_V3 = INTEGRATE_FIELDS + ("receipt_problems",)
QUERY_FIELDS_V3 = QUERY_FIELDS + ("started_mono_ns", "ended_mono_ns")


def gate_table():
    """Machine-readable copy of the approved thresholds."""
    return {
        "plan_revision": PLAN_REVISION,
        "product_base": PRODUCT_BASE,
        "tool_base": TOOL_BASE,
        "p95_rule": "ascending sample number ceil(0.95*n); n=30 is sample 29",
        "pooled_latency": "not_computed",
        "rows": [
            {"gate": "O5", "workload": "2cm chamfer, boxes 0/20, clouds 30k/100k, outside 0.5, cross 0",
             "repeat": "3 warmup + 30 scored, plus 30 cold first-build samples",
             "pass": "30k integrate P95<=500 max<=1000; 100k P95<=2000 max<=5000; acquire, ROI, first surface, post-update surface, and repeat surface each P95<=100 max<=250"},
            {"gate": "O6", "workload": "2cm and 5cm, boxes 0/5/20, both hulls, 30k/100k, outside 0.5/0.9, cross 0/0.5",
             "repeat": "96 groups, each 3+30 and 30 cold starts",
             "pass": "each group meets the O5 latency gates; init<=30s; RSS<=2GiB; deduped read-view plus cache peak<=512MiB"},
            {"gate": "O7-normal", "workload": "2cm chamfer 30k outside 0.5 cross 0; boxes 0/5/20; 3 windows",
             "repeat": "120s, 240 inputs at 2Hz, 120 query chains at 1Hz",
             "pass": "every input and query completes once; no reject, unknown, or duplicate; latency gates; queue age max<=1s; health gap<1s"},
            {"gate": "O7-overload", "workload": "20 boxes, 10Hz for 60s",
             "repeat": "600 inputs, 1Hz queries, then 30s drain",
             "pass": "all 600 IDs terminal; accepted inputs complete; explicit rejects allowed; at least one queue or reject observed; drain in-flight 0"},
            {"gate": "O7-recovery", "workload": "2Hz for 30s after the overload drain",
             "repeat": "60 inputs, 30 queries",
             "pass": "60 inputs complete; first WaitSnapshot OK within 2s and age<=2s; other 29 immediate queries OK inside the query gate"},
            {"gate": "O8", "workload": "fixed rays, increasing stamps",
             "repeat": "180s x3 with first 60s warmup; 30 saturation frames; no-input and occlusion 10s x3",
             "pass": "saturation probability keys and geometry blocks are 0; node count stable; last two 60s RSS medians grow by at most 5%; held probability and freshness stay put"},
            {"gate": "O9", "workload": "ROI, 30k fusion, first surface",
             "repeat": "30 ON/OFF pairs, even AB and odd BA",
             "pass": "P95 ratio<=1.10; no missing or mismatched pair; an injected fault stays replayable and cannot certify performance"},
        ],
    }


def inventory():
    """Separate measured facts, source mechanisms, and open hypotheses.

    Numbers come from the frozen 2026-10-30 closeout and interface collection.
    They are not a new measurement and they do not change those verdicts.
    """
    return {
        "classification": "fact, mechanism, and hypothesis are separate lists",
        "facts": [
            {"id": "o6-historical-fail",
             "statement": "1 cm group r01_b00_cuboid_c30000_o50_x50 integrate P95 is 517.297808 ms. The other 143 historical groups passed. This id is outside the new 96-group 2/5 cm matrix and stays failed.",
             "source": "docs/status/evidence/occ_o6_o7_20261029/closeout_g2/FINAL_REPORT.md"},
            {"id": "o7-surface-latency",
             "statement": "Nine 1 Hz normal windows failed only surface_ms. Medians were 139-145 ms, 101-110 ms, and 112-124 ms for 0, 5, and 20 boxes. Queue age max was 0.03-0.04 ms.",
             "source": "docs/status/evidence/occ_o6_o7_20261029/closeout_g2/WINDOW_LEDGER.json"},
            {"id": "overload-gap",
             "statement": "Overload sent 600, observed 523 completions, and left 77 ids at sent_no_receipt. A later boundary pass placed 227 inputs between publish return and callback entry, 13 at QUEUE_FULL, and 330 accepted frames waiting across another integrate. No DDS drop event was stored.",
             "source": "docs/status/evidence/occ_o5_o7_interface_collection_20261030/run01/FINAL_REPORT.md"},
            {"id": "recovery-stale",
             "statement": "The immediate GetSnapshot recovery query selected a snapshot about 30.4 s old and returned STALE. A query issued after the integrated receipt selected a snapshot about 0.33 s old and returned OK.",
             "source": "docs/status/evidence/occ_o5_o7_interface_collection_20261030/run01/FINAL_REPORT.md"},
            {"id": "write-loop",
             "statement": "On the scored O6 failure's diagnostic rerun, ApplyBatch was the larger fusion child on 30 of 30 samples, and write_loop_ms was the larger ApplyBatch child on 30 of 30. The rerun did not replace 517.297808 ms.",
             "source": "docs/status/evidence/occ_o5_o7_interface_collection_20261030/run01/FINAL_REPORT.md"},
            {"id": "store-lock-samples",
             "statement": "On the instrumented b20_D1 arm, 2 of 30 surface callbacks waited on the store lock for more than 150 ms. The other 28, and the b00_D1 arm, stayed near the no-write band. Cache-hit surface() itself was about 0.003 ms.",
             "source": "docs/status/evidence/occ_o5_o7_interface_collection_20261030/run01/FINAL_REPORT.md"},
            {"id": "host-threads",
             "statement": "The host reported 32 logical CPUs and powersave. The run was not pinned. Integrate runs on one worker thread. The observer executor is constructed with 6 threads.",
             "source": "docs/status/evidence/occ_o6_o7_20261029/closeout_g2/MANIFEST.json"},
            {"id": "unevaluated",
             "statement": "Formal O5 was not evaluated. Read-view and cache bytes had no ownership attribution, so that resource gate was not evaluated.",
             "source": "docs/status/evidence/occ_o6_o7_20261029/closeout_g2/FINAL_REPORT.md"},
        ],
        "mechanisms": [
            {"id": "get-takes-store-lock",
             "statement": "SnapshotRepository::get holds the repository mutex and calls ObservationStore::diagnostics, which locks the same mutex integrate holds for the whole update. A read of an existing handle waits for that update.",
             "source": "src/uld_occ_observer/src/snapshot_repository.cpp"},
            {"id": "callback-work",
             "statement": "The subscription callback looks up TF with a 0.1 s timeout and calls store diagnostics before enqueue. The subscription is reliable KeepLast depth 2. The worker queue also rejects at depth 2.",
             "source": "src/uld_occ_observer/src/observer_node.cpp"},
            {"id": "health-cache",
             "statement": "publish_health reads the worker's cached diagnostics under the worker mutex. That call itself does not take the store mutex.",
             "source": "src/uld_occ_observer/src/observer_node.cpp"},
            {"id": "single-integrator",
             "statement": "ObservationWorker integrates on one thread. occupancy_grid.cpp has no OpenMP region.",
             "source": "src/uld_occ_observer/src/observation_worker.cpp"},
        ],
        "hypotheses": [
            {"id": "residual-surface",
             "statement": "The two long store-lock samples do not account for the scored 101-145 ms surface medians. The historical response has no lock, build, copy, or serialization split, so that residual is not assigned."},
            {"id": "publish-gap",
             "statement": "The 227 publish-to-callback gaps are not identified as DDS loss. Depth 2 and callback work are a candidate, and this list does not treat that candidate as the measured cause."},
            {"id": "write-loop-split",
             "statement": "If a future 2 cm or 5 cm integrate exceeds its gate, the next split to record is search, probability, setNodeValue, allocation, inner update, and prune. That split was not measured here."},
        ],
    }
