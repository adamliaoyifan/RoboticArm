"""Replay of the 1621 review's audit against the CURRENT judge, using the
contract-GRADE fixtures from test_freespace_g3_offline (the gen2 fixtures
the review mutated were short of the contract and no longer exist).

Expected: the three controls stay green (reasons=[]) and every one of the
eight injected bad cases is rejected with named reasons.
"""
import copy
import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "src/uld_occ_observer/test/benchmark"))
sys.path.insert(0, str(ROOT / "src/uld_occ_observer/test"))
spec = importlib.util.spec_from_file_location(
    "controls", ROOT / "src/uld_occ_observer/test/test_freespace_g3_offline.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
judge = t.judge
contract = t.contract

results = {}


def check(name, factory, judge_fn, mutate=None):
    artifact = copy.deepcopy(factory())
    if mutate:
        mutate(artifact)
    results[name] = judge_fn(artifact)


check("p_control", t.full_p_window, judge.judge_p_window)
check("r_control", t.full_r_artifact, judge.judge_r_stage)
check("l_control", t.full_l_artifact, judge.judge_l_stage)
check("p_all_receipts_same_unrelated_id", t.full_p_window, judge.judge_p_window,
      lambda a: [row.update(input_id="unrelated") for row in a["input_timeline"]])
check("p_negative_query_time", t.full_p_window, judge.judge_p_window,
      lambda a: [r.update(ended_mono_ns=r["planned_mono_ns"] - 1) for r in a["queries"]])
check("r_no_recovery_receipts_or_queries", t.full_r_artifact, judge.judge_r_stage,
      lambda a: a.update(recovery_timeline=[], recovery_queries=[]))
check("r_second_query_error", t.full_r_artifact, judge.judge_r_stage,
      lambda a: a["recovery_queries"].__setitem__(
          1, dict(a["recovery_queries"][1], status="INTERNAL_ERROR")))
check("l_all_but_first_query_error", t.full_l_artifact, judge.judge_l_stage,
      lambda a: a.update(query_records=[a["query_records"][0]]
                         + [{"status": "INTERNAL_ERROR"}] * (len(a["query_records"]) - 1)))
check("l_drop_10pct", t.full_l_artifact, judge.judge_l_stage,
      lambda a: a.update(sent_inputs=contract.L_INPUTS - 1800,
                         input_records=a["input_records"][:-1800],
                         input_timeline=a["input_timeline"][:-1800],
                         query_records=a["query_records"][:contract.L_QUERIES - 1440]))
check("l_all_rss_same_timestamp", t.full_l_artifact, judge.judge_l_stage,
      lambda a: [s.update(t_mono_ns=600 * 10**9) for s in a["rss_series"][1:]])
check("l_missing_10min_bucket", t.full_l_artifact, judge.judge_l_stage,
      lambda a: a.update(rss_series=[
          s for s in a["rss_series"]
          if not t.T0 + 1800 * 10**9 <= s["t_mono_ns"] < t.T0 + 2400 * 10**9]))

print(json.dumps({k: v["reasons"][:3] for k, v in results.items()}, indent=1))
Path(__file__).with_name("RESULTS.json").write_text(json.dumps(results, indent=1))
bad = [name for name, verdict in results.items()
       if name.endswith("_control") and verdict["reasons"]]
escaped = [name for name, verdict in results.items()
           if not name.endswith("_control") and not verdict["reasons"]]
if bad or escaped:
    sys.exit("audit replay failed: bad_controls=%s escaped=%s" % (bad, escaped))
