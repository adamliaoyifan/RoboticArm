"""Write the S0 manifest, gate table, inventory, and judge rehearsal.

This command does not start the observer and does not score a product run.
"""
from __future__ import annotations

import json
from pathlib import Path

import repair_s0_contract as contract
import repair_s0_judge as judge
import repair_s0_manifest as manifest
import repair_s0_replay as replay


def _copy(case):
    return json.loads(json.dumps(case))


def control_cases(group_ids, o5_ids):
    """Named ledgers the judge must separate. These are not captured runs."""
    healthy = judge.healthy_o7_normal()
    missing = _copy(healthy)
    missing["inputs"].pop()
    duplicate = _copy(healthy)
    duplicate["inputs"][0]["terminal_count"] = 2
    rejected = _copy(healthy)
    for row in rejected["inputs"]:
        row["disposition"] = "rejected"
        row["reject_reason"] = "QUEUE_FULL"
        row["integrate_ms"] = None
    gap = _copy(healthy)
    gap["inputs"][0]["disposition"] = "not_entered"
    gap["inputs"][0]["callback_enter_ns"] = None
    gap["inputs"][0]["terminal_count"] = 0
    gap["inputs"][0]["receipt_received"] = False
    health = _copy(healthy)
    health["health_times_s"] = [0.0, 2.0, contract.NORMAL_DURATION_S]
    windows = [judge.healthy_o7_normal("normal_b%02d_r%d" % (boxes, repeat))
               for boxes in (0, 5, 20) for repeat in range(3)]
    for query in windows[0]["queries"]:
        query["surface_ms"] = 200.0
    single = {"gate": "o7_campaign", "windows": windows}
    timeout = judge.healthy_o7_recovery()
    timeout["queries"][0]["status"] = "TIMEOUT"
    timeout["queries"][0]["latency_ms"] = 2500.0
    wrong = judge.healthy_o7_recovery()
    wrong["queries"][0]["covers_observation"] = False
    resource = judge.healthy_latency_group(group_ids[0], 30000)
    resource["resources"]["attribution_complete"] = False
    unknown = judge.healthy_o7_overload()
    unknown["inputs"][0]["disposition"] = "not_entered"
    unknown["inputs"][0]["callback_enter_ns"] = None
    unknown["inputs"][0]["terminal_count"] = 0
    unknown["inputs"][0]["receipt_received"] = False
    quiet = {"gate": "o7_overload", "in_flight_after_drain": 0,
             "inputs": [judge._input(index, "overload") for index in range(contract.OVERLOAD_INPUTS)]}
    for row in quiet["inputs"]:
        row["queue_age_ms"] = 0.0
    ratio = judge.healthy_o9()
    for pairs in ratio["operations"].values():
        for pair in pairs:
            pair["on_ms"] = 12.0
    injected = judge.healthy_o9()
    injected["operations"]["roi"][0]["injected_fault"] = "dropped-sample"
    groups = [judge.healthy_latency_group(group_id, 30000) for group_id in group_ids]
    groups[0]["integrate_ms"] = [600.0] * contract.SCORED_SAMPLES
    o5_groups = [judge.healthy_latency_group(group_id, 30000 if "c30000" in group_id else 100000)
                 for group_id in o5_ids]
    growth = judge.healthy_o8()
    for run in growth["runs"]:
        run["rss_median_kib"] = [1000, 1000, 1060]
    overload_reject = {"gate": "o7_overload", "in_flight_after_drain": 0,
                       "inputs": [judge._input(index, "overload", "rejected")
                                  for index in range(contract.OVERLOAD_INPUTS)]}
    return [
        ("healthy_normal", healthy, "pass", []),
        ("missing_frame", missing, "fail", ["missing_frame"]),
        ("duplicate_terminal", duplicate, "fail", ["duplicate_terminal"]),
        ("all_rejected", rejected, "fail", ["all_rejected"]),
        ("pre_callback_gap", gap, "fail", ["pre_callback_gap"]),
        ("health_gap", health, "fail", ["health_gap"]),
        ("single_window", single, "fail", ["normal_b00_r0:surface_ms"]),
        ("recovery_timeout", timeout, "fail", ["recovery_timeout"]),
        ("wrong_snapshot", wrong, "fail", ["wrong_snapshot"]),
        ("resource_missing", resource, "fail", ["resource_missing"]),
        ("pre_callback_overload", unknown, "fail", ["pre_callback_gap"]),
        ("overload_not_observed", quiet, "not_evaluated", ["overload_not_observed"]),
        ("perturbation_ratio", ratio, "fail", ["perturbation_ratio"]),
        ("injected_fault", injected, "fail", ["injected_fault"]),
        ("o6_one_group", {"gate": "o6_campaign", "groups": groups}, "fail", ["group:%s" % group_ids[0]]),
        ("o5_campaign", {"gate": "o5_campaign", "groups": o5_groups}, "pass", []),
        ("o8_growth", growth, "fail", ["rss_growth"]),
        ("healthy_recovery", judge.healthy_o7_recovery(), "pass", []),
        ("healthy_o8", judge.healthy_o8(), "pass", []),
        ("healthy_o9", judge.healthy_o9(), "pass", []),
        ("overload_all_rejected", overload_reject, "pass", []),
    ]


def rehearse(fixtures):
    document = manifest.build_manifest(fixtures)
    group_ids = [group["id"] for group in document["o6_groups"]]
    o5_ids = [case["id"] for case in document["o5_cases"]]
    rows = []
    for name, case, status, reasons in control_cases(group_ids, o5_ids):
        verdict = judge.judge_case(case)
        replayed = replay.verify_replay(case)
        if verdict["status"] != status or verdict["reasons"] != reasons or not replayed["replay_possible"]:
            raise RuntimeError("%s judged %s/%s replay %s" % (name, verdict["status"], verdict["reasons"], replayed))
        rows.append({"name": name, "status": verdict["status"], "reasons": verdict["reasons"],
                     "replay_possible": True, "capture_certified": False})
    return document, rows


def _commands(out):
    bench = Path(__file__).resolve()
    test = bench.parents[1] / "test_repair_s0.py"
    return "\n".join([
        "#!/bin/sh",
        "set -eu",
        "python3 -m pytest -q --tb=line %s" % test,
        "python3 %s write --fixtures %s --out %s" % (bench, manifest.DEFAULT_FIXTURES, out),
        "",
    ])


def write_bundle(fixtures, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    document, rows = rehearse(fixtures)
    (out / "MANIFEST.json").write_text(json.dumps({
        "schema": document["schema"],
        "plan_revision": document["plan_revision"],
        "product_base": document["product_base"],
        "tool_base": document["tool_base"],
        "generator_version": document["generator_version"],
        "input_bytes": document["input_bytes"],
        "scoring": document["scoring"],
        "environment": document["environment"],
        "o6_group_count": len(document["o6_groups"]),
        "o6_group_ids": [group["id"] for group in document["o6_groups"]],
        "o5_cases": document["o5_cases"],
        "o7_windows": document["o7_windows"],
        "o8": document["o8"],
        "o9": document["o9"],
        "excluded_historical_failure": document["excluded_historical_failure"],
    }, indent=2) + "\n")
    (out / "O6_GROUPS.json").write_text(json.dumps(document["o6_groups"], indent=2) + "\n")
    (out / "GATE_TABLE.json").write_text(json.dumps(contract.gate_table(), indent=2) + "\n")
    (out / "INVENTORY.json").write_text(json.dumps(contract.inventory(), indent=2) + "\n")
    (out / "DISCRIMINATION.json").write_text(json.dumps({
        "capture_certified": False,
        "note": "Synthetic ledgers prove the judge names each fault. They do not certify a live capture.",
        "cases": rows,
    }, indent=2) + "\n")
    commands = out / "COMMANDS.sh"
    commands.write_text(_commands(out))
    commands.chmod(0o755)
    report = out / "FINAL_REPORT.md"
    report.write_text(_report(document, rows))
    return {"groups": len(document["o6_groups"]), "controls": len(rows), "out": str(out)}


def _report(document, rows):
    lines = [
        "# OctoMap 2 cm S0 合同与判官",
        "",
        "- plan_revision: `%s`" % document["plan_revision"],
        "- product_base: `%s`" % document["product_base"],
        "- tool_base: `%s`" % document["tool_base"],
        "- scoring: not_run",
        "- capture_certified: false",
        "",
        "本阶段冻结 96 组 2/5 cm 身份、O5/O7/O8/O9 清单和逐门判官。点云字节没有生成，观察器没有启动，产品没有修改。",
        "",
        "历史 1 cm 失败 `%s` 的 P95 %.6f ms 留在矩阵外，不改判。" % (
            document["excluded_historical_failure"]["id"], document["excluded_historical_failure"]["p95_ms"]),
        "",
        "判官排练 %d 条。每条重放后原因码与直接判定一致。合成故障不是实机捕获资格。" % len(rows),
        "",
        "| 用例 | 状态 | 原因 |",
        "|---|---|---|",
    ]
    for row in rows:
        lines.append("| %s | %s | %s |" % (row["name"], row["status"], ",".join(row["reasons"]) or "none"))
    lines.extend([
        "",
        "资源字节门只接受调用方已经按所有权去重的计数。缺归属时该门是 not_evaluated，整组不能通过。",
        "",
        "S1 需要 reviews 绑定本次工具提交后才能领取。S0 的合成通过不能代替 S2 的真实捕获资格。",
        "",
    ])
    return "\n".join(lines)


def main(argv):
    import argparse
    parser = argparse.ArgumentParser(description="Write the offline 2 cm S0 contract bundle")
    parser.add_argument("command", choices=("write",))
    parser.add_argument("--fixtures", type=Path, default=manifest.DEFAULT_FIXTURES)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    summary = write_bundle(args.fixtures, args.out)
    print("groups", summary["groups"])
    print("controls", summary["controls"])
    print("out", summary["out"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main(__import__("sys").argv[1:]))
