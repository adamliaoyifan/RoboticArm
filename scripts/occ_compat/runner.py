"""runner — matrix sequencing for verify (V1/V2).

Brings up one real chain per (backend, hull) group, runs the case
dispatch against it with the ledger, and tears down. Every process gets
its own ROS domain; failures keep the run directory (no rerun-and-pick-
green: a FAILED case stays FAILED in the archive until a new freeze and
a new run).
"""
from __future__ import annotations

import json
import os
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
    """Scene fixture documents for one hull.

    S-free comes from the frozen seed=20261105 generator payloads (the
    run793 reuse set has no observed known-free region — coverage
    analysis 085e785d); S-occupied/S-unknown reuse the verified 30k set.
    """
    inputs = json.loads(Path(manifest["prep_inputs_json"]).read_text())
    fixtures = {}
    sfree_dir = Path(manifest.get(
        "sfree_dir", "")) / ("sfree_%s" % hull)
    sfree_paths = sorted(sfree_dir.glob("*.yaml"))
    if not sfree_paths:
        raise RuntimeError("S-free payloads missing under %s" % sfree_dir)
    fixtures["S-free"] = [yaml.safe_load(open(p).read())
                          for p in sfree_paths]
    for scene in ("S-occupied", "S-unknown"):
        combo_key = SCENE_SELECTION[hull][scene]
        combo = inputs["combos"][combo_key]
        docs = []
        for frame in combo["frames"][:4]:
            path = frame["path"] if isinstance(frame, dict) else frame
            docs.append(yaml.safe_load(open(path).read()))
        fixtures[scene] = docs
    return fixtures


def _hull_manifest(manifest, hull):
    """Per-hull view of the manifest (scene TF + O fixture geometry)."""
    view = dict(manifest)
    view["scene_tf"] = manifest.get("scene_tf_%s" % hull,
                                    manifest.get("scene_tf", ""))
    view["o_geometry"] = manifest.get("o_geometry_%s" % hull,
                                      manifest.get("o_geometry", ""))
    view["o_config"] = manifest.get("o_config_%s" % hull,
                                    manifest.get("o_config", ""))
    return view


def _bring_up(backend, hull, manifest, root):
    """Spawn one backend chain; returns (stack, client, clock_stop)."""
    import rclpy
    manifest = _hull_manifest(manifest, hull)
    stack = chains.ChainStack(root, "chain-%s-%s" % (backend, hull))
    if backend == "G":
        stack.spawn("mapper", chains.g_mapper_command(
            manifest, scene_revision="scene-g1"))
        stack.spawn("planner", chains.planner_command(
            manifest, backend="G", scene_revision="scene-g1"))
    else:
        observer, bridge = chains.o_chain_commands(
            manifest, scene_revision="scene-g1")
        stack.spawn("observer", observer)
        stack.spawn("bridge", bridge)
        stack.spawn("planner", chains.planner_command(
            manifest, backend="O", scene_revision="scene-g1"))
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
    # Fixture scene TF: the fixture transform IS T_container_from_optical
    # (p_container = R p_opt + t), so the static edge is
    # parent=container_link -> child=optical_frame carrying it verbatim
    # (the same convention as the observer's uld_from_optical).
    client.broadcast_transform(first.get("transform") or {}, base,
                               static=True, frame_id="container_link",
                               child_frame_id=first.get("optical_frame"))
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
        # Static TFs for the E4 plan() lookups (stamped at the base).
        base = int(fixtures["S-free"][0]["stamp_ns"])
        identity_matrix = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        for static_tf in (
                {"child_frame_id": "suction_contact_frame",
                 "translation": [0.0, 0.0, 0.1],
                 "rotation": identity_matrix},
                {"child_frame_id": "container_link",
                 "translation": [0.0, 0.0, 0.0],
                 "rotation": identity_matrix}):
            client.broadcast_transform(
                static_tf, base, static=True, frame_id="container_link"
                if static_tf["child_frame_id"] == "suction_contact_frame"
                else "world")
        for case in CASE_ORDER:
            started = mono_ns()
            # Every case starts with the sim robot stream running (an
            # earlier sweep's E4 window may have stopped it mid-error).
            ctx.fresh_robot()
            outcomes = v1_states.run_case(ctx, case)
            results[case] = outcomes
            # Per-consumer sweep: E2/E3/E4 terminals for this scenario
            # (contract V1 coverage — 4 consumers x 40 scenarios).
            try:
                sweep = v1_states.consumer_sweep(ctx, case, None)
                outcomes.update({"E2": sweep["E2"], "E3": sweep["E3"],
                                 "E4": sweep["E4"]})
            except Exception as error:  # noqa: BLE001 - named failure
                import traceback
                outcomes["sweep"] = "ERROR: %s" % str(error)[:80]
                ledger.emit(
                    backend=backend, consumer="-", hull=hull,
                    case_id=case, subprobe="",
                    event_kind="sweep_error", stage="case",
                    state="ERROR",
                    reason=repr(error)[:120],
                    trace=traceback.format_exc()[-400:])
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


def _run_group_subprocess(matrix, backend, hull, freeze_path, group_root,
                          domain, run_id, timeout_s=5400):
    """One group in its own interpreter/session; returns (results, note).

    A fresh interpreter keeps rclpy lifecycles independent (in-process
    sequencing degraded into InvalidHandle teardowns that leaked whole
    chains onto the shared default domain — formal_run01), a unique
    ROS_DOMAIN_ID fences any leak, PDEATHSIG cleans the chain if the
    child dies, and the session (process group) is verified EMPTY after
    exit so nothing survives into the next group.
    """
    import os
    import signal
    import subprocess
    import sys
    scripts_dir = str(Path(__file__).resolve().parent.parent)
    group_root.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env["PYTHONPATH"] = scripts_dir + ":" + env.get("PYTHONPATH", "")
    cmd = [sys.executable, "-m", "occ_compat.group_main",
           "--matrix", matrix, "--backend", backend, "--hull", hull,
           "--freeze", str(freeze_path), "--out", str(group_root),
           "--domain", str(domain), "--run-id", run_id]
    child = subprocess.Popen(
        cmd, cwd=scripts_dir, env=env, start_new_session=True,
        stdout=open(group_root / "GROUP_DRIVER.log", "w"),
        stderr=subprocess.STDOUT)
    try:
        code = child.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait()
        return None, "group driver timeout after %ss (session killed)" % timeout_s
    note = ""
    try:
        os.killpg(child.pid, 0)
        # the session still has members: kill them and name the leak
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        note = "leaked session members after driver exit (killed)"
    except ProcessLookupError:
        pass
    result_path = group_root / "GROUP_RESULT.json"
    if code != 0 or not result_path.exists():
        tail = ""
        log = group_root / "GROUP_DRIVER.log"
        if log.exists():
            tail = log.read_text().strip().splitlines()[-1][:120] if \
                log.read_text().strip() else ""
        return None, ("driver exit=%s %s" % (code, tail)).strip()[:200]
    return json.loads(result_path.read_text()).get("results", {}), note


def run_matrices(freeze_path, matrices, out_dir):
    """verify entry: sequence the requested matrices (V1 first, then V2).

    Every group runs as an isolated subprocess (see
    _run_group_subprocess) on its own ROS domain."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    run_id = "v1-%d" % int(time.time())
    # FastRTPS rejects domainId > 232 ("Calculated port number is too
    # high"); rotate within the legal range, seeded per run for spread.
    base_domain = os.getpid() % 230 + 1
    summary = {"run_id": run_id, "matrices": matrices, "groups": {},
               "isolation": "one subprocess + unique ROS domain per group"}
    index = 0
    plan = []
    if "V1" in matrices:
        for backend in ("G", "O"):
            for hull in ("cuboid", "chamfer"):
                plan.append(("V1", backend, hull))
    if "V2" in matrices:
        for backend in ("G", "O"):
            plan.append(("V2", backend, "cuboid"))
    for matrix, backend, hull in plan:
        key = "%s|%s|%s" % (matrix, backend, hull)
        group_root = out / ("%s-%s-%s" % (matrix, backend, hull))
        group_root.mkdir(parents=True, exist_ok=True)
        index += 1
        domain = base_domain + index
        if domain > 230:
            domain = domain - 230
        try:
            results, note = _run_group_subprocess(
                matrix, backend, hull, freeze_path, group_root,
                domain,
                "%s-%s-%s" % (matrix.lower(), backend.lower(), hull))
            entry = results if results is not None else {
                "error": note or "no result"}
            if note and results is not None:
                entry = dict(entry)
                entry["isolation_note"] = note
            summary["groups"][key] = entry
        except Exception as error:  # noqa: BLE001 - named failure
            summary["groups"][key] = {"error": str(error)[:200]}
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
