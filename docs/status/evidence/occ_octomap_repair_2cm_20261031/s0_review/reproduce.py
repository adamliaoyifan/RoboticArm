"""Offline S0 adversarial gates. Exit 1 means a malformed/failing case passed."""
import argparse
import copy
import json
import sys
import subprocess
from pathlib import Path
parser = argparse.ArgumentParser()
parser.add_argument("--worktree", required=True)
parser.add_argument("--output", required=True)
args = parser.parse_args()
sys.path.insert(0, str(Path(args.worktree) / "src/uld_occ_observer/test/benchmark"))
import repair_s0_judge as judge
cases = {}
case = judge.healthy_o7_recovery()
case["inputs"][1].update(disposition="rejected", reject_reason="QUEUE_FULL", integrate_ms=None)
cases["recovery_one_rejected"] = case
case = judge.healthy_o7_normal()
case["queries"][1] = copy.deepcopy(case["queries"][0])
cases["normal_duplicate_query_missing_other"] = case
cases["o6_same_group_96_times"] = {"gate": "o6_campaign", "groups": [
    judge.healthy_latency_group("r02_b00_chamfer_c30000_o50_x00", 30000) for _ in range(96)]}
case = judge.healthy_o7_normal()
case["queries"][0]["covers_observation"] = False
cases["normal_wrong_snapshot"] = case
case = judge.healthy_o7_recovery()
case["queries"][0]["latency_ms"] = float("nan")
cases["recovery_nan_latency"] = case
results = []
for name, case in cases.items():
    try:
        actual = judge.judge_case(case)
        rejected = actual.get("pass") is False and actual.get("status") in ("fail", "not_evaluated")
    except Exception as error:
        actual = {"exception": type(error).__name__, "message": str(error)}
        rejected = False
    results.append({"case": name, "expected": "explicit non-pass without exception",
                    "actual": actual, "correctly_rejected": rejected})
report = {"revision": subprocess.check_output(["git", "-C", args.worktree, "rev-parse", "HEAD"], text=True).strip(),
          "scope": "offline judge audit, not product evaluation", "results": results,
          "false_pass_count": sum(r["actual"].get("pass") is True for r in results)}
Path(args.output).write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
print(json.dumps(report, ensure_ascii=False, indent=2))
sys.exit(0 if all(r["correctly_rejected"] for r in results) else 1)
