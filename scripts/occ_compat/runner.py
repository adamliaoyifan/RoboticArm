"""runner — matrix sequencing for verify (V1/V2).

Brings up one real chain per (backend, hull) group, runs the case
dispatch against it with the ledger, and tears down. Every process gets
its own ROS domain; failures keep the run directory (no rerun-and-pick-
green: a FAILED case stays FAILED in the archive until a new freeze and
a new run).
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import yaml

from . import chains, v1_states
from .client import ChainClient
from .inventory import SCENE_SELECTION, sha256_file
from .ledger import EventLedger, mono_ns

CASE_ORDER = ["K04", "K01", "K02", "K03", "K05", "K06", "K07", "K08",
              "K09", "K10"]


def _load_fixtures(manifest, hull):
    """Scene fixture documents for one hull (verified at inventory)."""
    inputs = json.loads(Path(manifest["prep_inputs_json"]).read_text())
    fixtures = {}
    for scene, combo_key in SCENE_SELECTION[hull].items():
        combo = inputs["combos"][combo_key]
        docs = []
        for frame in combo["frames"][:4]:
            path = frame["path"] if isinstance(frame, dict) else frame
            docs.append(yaml.safe_load(open(path).read()))
        fixtures[scene] = docs
    return fixtures


def _bring_up(backend, hull, manifest, root):
    """Spawn one backend chain; returns (stack, client, clock_stop)."""
    import rclpy
    stack = chains.ChainStack(root, "chain-%s-%s" % (backend, hull))
    if backend == "G":
        stack.spawn("mapper", chains.g_mapper_command(manifest))
        stack.spawn("planner", chains.planner_command(
            manifest, backend="G"))
    else:
        observer, bridge = chains.o_chain_commands(manifest)
        stack.spawn("observer", observer)
        stack.spawn("bridge", bridge)
        stack.spawn("planner", chains.planner_command(
            manifest, backend="O"))
    time.sleep(8.0)
    if not all(p.alive() for p in stack.processes):
        raise RuntimeError("chain boot failed: %s" % [
            (p.name, p.tail(2)) for p in stack.processes if not p.alive()])
    client = ChainClient(backend, sim=True)
    ok, pending = client.wait_services(15.0)
    if not ok:
        client.destroy_node()
        raise RuntimeError("services unavailable: %s" % pending)
    fixtures = _load_fixtures(manifest, hull)
    first = fixtures["S-free"][0]
    base = int(first["stamp_ns"])
    # Fixture scene TF: static optical->container for the frame poses.
    client.broadcast_transform(first.get("transform") or {}, base,
                               static=True,
                               frame_id=first.get("optical_frame"))
    stop = client.start_clock(base)
    time.sleep(0.8)
    return stack, client, stop, fixtures


def run_v1_group(backend, hull, manifest, run_root, run_id):
    """One (backend, hull) group of the V1 matrix (E1 first; E2/E3/E4
    land with their drivers — the dispatch already reserves them)."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()
    ledger = EventLedger(Path(run_root) / "events.jsonl", run_id)
    results = {}
    try:
        stack, client, stop, fixtures = _bring_up(
            backend, hull, manifest, Path(run_root) / "chain")
        ctx = v1_states.CaseContext(
            backend, hull, stack, client, ledger, fixtures, stop)
        for case in CASE_ORDER:
            started = mono_ns()
            outcomes = v1_states.run_case(ctx, case)
            results[case] = outcomes
            ledger.emit(
                backend=backend, consumer="-", hull=hull, case_id=case,
                subprobe="", event_kind="case_complete", stage="case",
                state="DONE", duration_ms=(mono_ns() - started) // 1_000_000)
        stop.set()
        client.destroy_node()
        stack.stop_all()
    finally:
        ledger.close()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001 - teardown never masks results
            pass
    return results


def run_matrices(freeze_path, matrices, out_dir):
    """verify entry: sequence the requested matrices (V1 first, then V2)."""
    freeze = json.loads(Path(freeze_path).read_text())
    manifest = freeze["manifest"]
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    run_id = "v1-%d" % int(time.time())
    summary = {"run_id": run_id, "matrices": matrices, "groups": {}}
    if "V1" in matrices:
        for backend in ("G", "O"):
            for hull in ("cuboid", "chamfer"):
                group_root = out / ("V1-%s-%s" % (backend, hull))
                group_root.mkdir(parents=True, exist_ok=True)
                try:
                    results = run_v1_group(
                        backend, hull, manifest, group_root, run_id)
                    summary["groups"]["V1|%s|%s" % (backend, hull)] = \
                        results
                except Exception as error:  # noqa: BLE001 - named failure
                    summary["groups"]["V1|%s|%s" % (backend, hull)] = {
                        "error": str(error)[:200]}
    if "V2" in matrices:
        summary["groups"]["V2"] = {"status": "NOT_EVALUATED",
                                   "reason": "race driver pending"}
    verdict = "NOT_EVALUATED"
    flat = [state for group in summary["groups"].values()
            if isinstance(group, dict)
            for outcomes in group.values() if isinstance(outcomes, dict)
            for state in outcomes.values()]
    if flat:
        verdict = ("PASS" if all(s == "PASS" for s in flat)
                   else "PARTIAL" if any(s == "PASS" for s in flat)
                   else "FAIL")
    summary["verdict"] = verdict
    return summary
