"""cli — subcommand dispatch for occ_consumer_compat (thin)."""
from __future__ import annotations

import json
import sys
from pathlib import Path

from . import inventory as inventory_mod
from .ledger import EventLedger


def _write_result(out_dir, name, result):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / (name + ".json")).write_text(json.dumps(result, indent=1,
                                                   sort_keys=True))


def cmd_inventory(args):
    result = inventory_mod.run_inventory(args.manifest, args.out)
    _write_result(args.out, "INVENTORY_RESULT", result)
    print("inventory frozen: %d consumers, %d V1 cases, %d V2 cases"
          % (result["consumers"], result["v1_cases"], result["v2_cases"]))
    return 0


def cmd_preflight(args):
    from . import chains
    result = chains.preflight(args.manifest, args.out,
                              backend=getattr(args, 'backend', 'G'))
    _write_result(args.out, "PREFLIGHT_RESULT", result)
    print("preflight: %s (%.1fs real monotonic)"
          % (result.get("verdict"), result.get("elapsed_s", -1)))
    return 0 if result.get("verdict") == "PASS" else 1


def cmd_selftest(args):
    from . import judge as judge_mod
    result = judge_mod.selftest(args.manifest, args.out)
    _write_result(args.out, "SELFTEST_RESULT", result)
    ok = all(v == "REJECTED" for v in result.get("mutations", {}).values()) \
        and all(v == "PASS" for v in result.get("controls", {}).values())
    print("selftest: %d/%d mutations rejected, %d/%d controls passed"
          % (sum(1 for v in result.get("mutations", {}).values()
                 if v == "REJECTED"), len(result.get("mutations", {})),
             sum(1 for v in result.get("controls", {}).values()
                 if v == "PASS"), len(result.get("controls", {}))))
    return 0 if ok else 1


def cmd_freeze(args):
    from . import freeze as freeze_mod
    result = freeze_mod.run_freeze(args.manifest, args.out)
    _write_result(args.out, "FREEZE_RESULT", result)
    ok = not result.get("problems")
    print("freeze: %s" % ("clean" if ok else "PROBLEMS: %s"
                          % result["problems"]))
    return 0 if ok else 1


def cmd_verify(args):
    from . import runner
    result = runner.run_matrices(args.freeze, args.matrix.split(","),
                                 args.out)
    _write_result(args.out, "VERIFY_RESULT", result)
    print("verify: %s" % result.get("verdict"))
    return 0 if result.get("verdict") == "PASS" else 1


def cmd_judge(args):
    from . import judge as judge_mod
    result = judge_mod.judge_archive(args.archive, args.out)
    _write_result(args.out, "VERDICT", result)
    gates = result.get("gates", {})
    print("judge: " + " ".join("%s=%s" % (k, v)
                               for k, v in sorted(gates.items())))
    ok = all(v == "PASS" for v in gates.values())
    return 0 if ok else 1


def cmd_replay(args):
    from . import replaycmd
    result = replaycmd.replay_case(args.archive, args.case, args.out)
    _write_result(args.out, "REPLAY_RESULT", result)
    print("replay %s: %s" % (args.case, result.get("verdict")))
    return 0 if result.get("verdict") == "PASS" else 1
