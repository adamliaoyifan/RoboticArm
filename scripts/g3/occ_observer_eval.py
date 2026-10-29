#!/usr/bin/env python3
"""Independent OctoMap observer acceptance driver (O0-O9, S1-S3).

Oracle expected keys come from src/uld_occ_observer/test/independent_oracle.cpp,
not from OcTree::computeRayKeys. The installed observer_eval_native binary is
the SUT runner. Exit 0=pass, 3=fail, 4=not_evaluated.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

PASS, FAIL, NOT_EVAL = 0, 3, 4
ROOT = Path(__file__).resolve().parents[2]


def _die(code: int, msg: str) -> int:
    print(msg, file=sys.stderr)
    return code


def _sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _native(install: Path) -> Path:
    cand = install / "uld_occ_observer" / "lib" / "uld_occ_observer" / "observer_eval_native"
    if cand.exists():
        return cand
    cand = install / "lib" / "uld_occ_observer" / "observer_eval_native"
    if cand.exists():
        return cand
    raise FileNotFoundError("observer_eval_native not in install prefix")


def _cli(install: Path) -> Path:
    for rel in (
        "uld_occ_observer/lib/uld_occ_observer/observer_cli",
        "lib/uld_occ_observer/observer_cli",
    ):
        p = install / rel
        if p.exists():
            return p
    raise FileNotFoundError("observer_cli not in install prefix")


def _run(cmd, **kw):
    return subprocess.run(cmd, check=False, text=True, capture_output=True, **kw)


def _export_geometry(scene: Path, out: Path, cuboid: bool) -> None:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT / "src" / "luggage_description") + os.pathsep + env.get(
        "PYTHONPATH", ""
    )
    exporter = ROOT / "src" / "uld_occ_observer" / "scripts" / "export_geometry.py"
    tmp = out.with_suffix(".full.yaml")
    r = _run([sys.executable, str(exporter), "--scene-config", str(scene), "--out", str(tmp)], env=env)
    if r.returncode != 0:
        raise RuntimeError(r.stderr or r.stdout or "export_geometry failed")
    if not cuboid:
        out.write_text(tmp.read_text())
        tmp.unlink(missing_ok=True)
        return
    env_path = str(ROOT / "src" / "luggage_description")
    if env_path not in sys.path:
        sys.path.insert(0, env_path)
    import yaml
    from luggage_description.container_geometry import normalize_descriptor

    envelope = yaml.safe_load(tmp.read_text())
    descriptor = json.loads(envelope["descriptor_json"])
    descriptor.pop("chamfer", None)
    descriptor.pop("geometry_hash", None)
    geom = normalize_descriptor(descriptor)
    canonical = json.dumps(geom.descriptor(), sort_keys=True, separators=(",", ":"), allow_nan=False)
    artifact = {
        "schema": "uld_geometry_export_v1",
        "descriptor_json": canonical,
        "descriptor_sha256": hashlib.sha256(canonical.encode()).hexdigest(),
    }
    out.write_text(yaml.safe_dump(artifact, sort_keys=True))
    tmp.unlink(missing_ok=True)


def cmd_fixtures(args) -> int:
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    scene = ROOT / "src" / "luggage_description" / "config" / "scene_tf.yaml"
    resolutions = [float(x) for x in args.resolutions.split(",") if x]
    boxes = [int(x) for x in args.boxes.split(",") if x]
    hulls = [x for x in args.hulls.split(",") if x]
    seeds = int(args.seeds)
    scenes = []
    for hull in hulls:
        geom = out / ("geometry_%s.yaml" % hull)
        _export_geometry(scene, geom, cuboid=(hull == "cuboid"))
        for res in resolutions:
            cfg = out / ("config_%s_r%s.yaml" % (hull, str(res).replace(".", "p")))
            cfg.write_text(
                "resolution_m: %s\ndepth_z_min_m: 0.52\ndepth_z_max_m: 6.0\nclock_domain: ros\n"
                % res
            )
            for nbox in boxes:
                sid = "r%s_b%s_%s" % (res, nbox, hull)
                scenes.append(
                    {
                        "id": sid,
                        "resolution": res,
                        "boxes": nbox,
                        "hull": hull,
                        "geometry_file": str(geom.resolve()),
                        "config_file": str(cfg.resolve()),
                        "geometry_sha256": _sha256_file(geom),
                        "config_sha256": _sha256_file(cfg),
                    }
                )
    manifest = {
        "schema": "occ_observer_fixture_manifest_v1",
        "generator_version": "o6-load-v1",
        "seeds": list(range(seeds)),
        "scenes": scenes,
        "raw_input_hash_policy": "optical_xyz_sha256_before_filter",
    }
    (out / "TEST_MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("wrote %s scenes" % len(scenes))
    return PASS if scenes else FAIL


def _need_native(install: Path) -> Path:
    try:
        return _native(Path(install))
    except FileNotFoundError as exc:
        raise SystemExit(_die(FAIL, str(exc)))


def _load_manifest(path: Path) -> dict:
    man = path / "TEST_MANIFEST.json" if path.is_dir() else path
    return json.loads(man.read_text())


def cmd_correctness(args) -> int:
    install = Path(args.install)
    native = _need_native(install)
    fixtures = Path(args.fixtures)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    try:
        manifest = _load_manifest(fixtures)
    except FileNotFoundError:
        return _die(NOT_EVAL, "fixtures manifest missing")
    results = []
    overall = True
    probe = install / "lib" / "uld_occ_observer" / "observer_ray_probe"
    if not probe.exists():
        probe = install / "uld_occ_observer" / "lib" / "uld_occ_observer" / "observer_ray_probe"
    if probe.exists():
        probe_out = out / "native_ray_probe.json"
        _run([str(probe), str(probe_out)])
    for scene in manifest["scenes"]:
        for gate, extra in (
            ("o1-filter", []),
            (
                "o1",
                [
                    "--seeds",
                    str(len(manifest["seeds"])),
                    "--frames",
                    "100",
                    "--boxes",
                    str(scene["boxes"]),
                    "--cloud",
                    "128",
                    "--outside",
                    "0.5",
                    "--cross",
                    "0.25",
                ],
            ),
            (
                "o2",
                [
                    "--seeds",
                    str(len(manifest["seeds"])),
                    "--frames",
                    "1000",
                    "--boxes",
                    str(scene["boxes"]),
                ],
            ),
            ("o3", []),
            ("o4", []),
        ):
            report = out / ("%s_%s.json" % (scene["id"], gate))
            cmd = [
                str(native),
                "--geometry",
                scene["geometry_file"],
                "--config",
                scene["config_file"],
                "--gate",
                gate,
                "--out",
                str(report),
                *extra,
            ]
            t0 = time.monotonic()
            r = _run(cmd)
            elapsed = time.monotonic() - t0
            payload = {}
            if report.exists():
                try:
                    payload = json.loads(report.read_text())
                except json.JSONDecodeError:
                    payload = {"pass": False, "reason": "invalid_report"}
            ok = r.returncode == 0 and payload.get("pass") is True
            overall = overall and ok
            results.append(
                {
                    "scene": scene["id"],
                    "gate": gate,
                    "returncode": r.returncode,
                    "elapsed_s": round(elapsed, 3),
                    "pass": ok,
                    "stderr": (r.stderr or "")[-500:],
                    "report": payload,
                }
            )
            if not ok and gate in ("o1-filter", "o1"):
                (out / "FAILURE_BOUNDARY.json").write_text(
                    json.dumps({"scene": scene, "gate": gate, "stderr": r.stderr}, indent=2)
                )
                overall = False
    (out / "CORRECTNESS.md").write_text(
        "# Correctness\n\n%s\n" % json.dumps(results, indent=2)[:20000]
    )
    (out / "correctness_summary.json").write_text(json.dumps({"pass": overall, "results": results}, indent=2))
    return PASS if overall else FAIL


def cmd_benchmark(args) -> int:
    install = Path(args.install)
    native = _need_native(install)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    fixtures = Path(args.fixtures) if args.fixtures else None
    if fixtures is None or not (fixtures / "TEST_MANIFEST.json").exists():
        return _die(NOT_EVAL, "fixtures required for benchmark")
    manifest = _load_manifest(fixtures)
    profile = args.profile
    rows = []
    overall = True
    bench_dir = ROOT / "src" / "uld_occ_observer" / "test" / "benchmark"
    if str(bench_dir) not in sys.path:
        sys.path.insert(0, str(bench_dir))
    from o6_matrix import select_benchmark_scenes
    selected = select_benchmark_scenes(manifest, profile)
    for item in selected:
        scene = item["scene"]
        cloud = item["cloud"]
        outside = item["outside"]
        cross = item["cross"]
        report = out / ("%s_c%s_o%s_x%s.json" % (scene["id"], cloud, outside, cross))
        cmd = [
            str(native),
            "--geometry",
            scene["geometry_file"],
            "--config",
            scene["config_file"],
            "--gate",
            "bench",
            "--out",
            str(report),
            "--boxes",
            str(scene["boxes"]),
            "--cloud",
            str(cloud),
            "--outside",
            str(outside),
            "--cross",
            str(cross),
            "--warmup",
            str(args.warmup),
            "--repeat",
            str(args.repeat),
        ]
        r = _run(cmd)
        payload = json.loads(report.read_text()) if report.exists() else {"pass": False}
        ok = r.returncode == 0 and payload.get("pass") is True
        overall = overall and ok
        rows.append({
            "scene": scene["id"],
            "hull": scene["hull"],
            "cloud": cloud,
            "outside": outside,
            "cross": cross,
            "pass": ok,
            "report": payload,
        })
    document = {"pass": bool(overall and rows), "rows": rows}
    (out / "performance.json").write_text(json.dumps(document, indent=2) + "\n")
    (out / "PERFORMANCE.md").write_text(
        "# Performance\n\nThe verdict is performance.json (%d rows). This file is not the gate.\n" % len(rows))
    return PASS if document["pass"] else (NOT_EVAL if not rows else FAIL)


def cmd_capture(args) -> int:
    install = Path(args.install)
    native = _need_native(install)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    fixtures = Path(args.fixtures) if args.fixtures else None
    if fixtures is None or not (fixtures / "TEST_MANIFEST.json").exists():
        return _die(NOT_EVAL, "fixtures required")
    scene = next(
        s
        for s in _load_manifest(fixtures)["scenes"]
        if s["resolution"] == 0.01 and s["hull"] == "chamfer" and s["boxes"] == 0
    )
    off = out / "off.json"
    on = out / "on.json"
    common = [
        str(native),
        "--geometry",
        scene["geometry_file"],
        "--config",
        scene["config_file"],
        "--boxes",
        "0",
        "--cloud",
        "30000",
        "--warmup",
        "3",
        "--repeat",
        str(args.repeat),
    ]
    r0 = _run(common + ["--gate", "bench", "--out", str(off)])
    r1 = _run(common + ["--gate", "bench-capture", "--out", str(on)])
    if not off.exists() or not on.exists():
        return FAIL
    a = json.loads(off.read_text())
    b = json.loads(on.read_text())
    try:
        ratio = float(b["p95_integrate_ms"]) / max(float(a["p95_integrate_ms"]), 1e-9)
        n0 = int(float(a.get("n", "0")))
        n1 = int(float(b.get("n", "0")))
    except (KeyError, ValueError):
        return FAIL
    # O9 scores capture overhead, not O5 absolute latency.
    ok = n0 == n1 and n0 > 0 and ratio <= 1.10
    (out / "capture_overhead.json").write_text(
        json.dumps(
            {
                "ratio": ratio,
                "pass": ok,
                "n_off": n0,
                "n_on": n1,
                "off_rc": r0.returncode,
                "on_rc": r1.returncode,
                "latency_pass_off": a.get("pass"),
                "latency_pass_on": b.get("pass"),
            },
            indent=2,
        )
        + "\n"
    )
    return PASS if ok else FAIL


def cmd_o6(args) -> int:
    bench = ROOT / "src" / "uld_occ_observer" / "test" / "benchmark"
    if str(bench) not in sys.path:
        sys.path.insert(0, str(bench))
    from o6_matrix import O6Campaign
    groups = [item for item in args.rehearsal_groups.split(",") if item]
    campaign = O6Campaign(
        args.profile_cli, args.eval_native, args.fixtures, args.out,
        budget_s=args.budget_s, rehearsal=args.rehearsal, rehearsal_groups=groups)
    report = campaign.run()
    verdict = report["verdict"]["verdict"]
    if args.rehearsal:
        return PASS if report["groups"] else FAIL
    if verdict == "pass":
        return PASS
    if verdict == "fail":
        return FAIL
    return NOT_EVAL


def cmd_o7(args) -> int:
    if not args.node or not args.setup or not args.eval_native or not args.fixtures:
        return _die(NOT_EVAL, "O7 needs --node --setup --eval-native --fixtures")
    bench = ROOT / "src" / "uld_occ_observer" / "test" / "benchmark"
    if str(bench) not in sys.path:
        sys.path.insert(0, str(bench))
    from sustained_run import run_campaign
    return run_campaign(args)


def cmd_concurrency(args) -> int:
    marker = Path(args.out).parent / "representative" / "PERFORMANCE.md"
    if args.out:
        Path(args.out).mkdir(parents=True, exist_ok=True)
        (Path(args.out) / "skipped.json").write_text(
            json.dumps(
                {
                    "gate": "O7",
                    "status": "not_evaluated",
                    "reason": "O7 starts only after O5 representative pass; see plan stop rule",
                },
                indent=2,
            )
            + "\n"
        )
    return NOT_EVAL


def cmd_stability(args) -> int:
    install = Path(args.install)
    native = _need_native(install)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    fixtures = Path(args.fixtures) if args.fixtures else None
    if fixtures is None or not (fixtures / "TEST_MANIFEST.json").exists():
        return _die(NOT_EVAL, "fixtures required for O8")
    scene = next(
        s
        for s in _load_manifest(fixtures)["scenes"]
        if s["resolution"] == 0.01 and s["hull"] == "chamfer" and s["boxes"] == 0
    )
    rows = []
    overall = True
    for i in range(args.repeat):
        report = out / ("o8_%02d.json" % i)
        r = _run(
            [
                str(native),
                "--geometry",
                scene["geometry_file"],
                "--config",
                scene["config_file"],
                "--gate",
                "o8",
                "--boxes",
                "0",
                "--out",
                str(report),
            ]
        )
        payload = json.loads(report.read_text()) if report.exists() else {"pass": False}
        ok = r.returncode == 0 and payload.get("pass") is True
        overall = overall and ok
        rows.append({"i": i, "pass": ok, "report": payload, "rc": r.returncode})
    (out / "o8_summary.json").write_text(json.dumps({"pass": overall, "rows": rows}, indent=2))
    return PASS if overall else FAIL


def cmd_simulate(args) -> int:
    script = ROOT / "scripts" / "g3" / "occ_observer_sensor_sim.py"
    cmd = [
        sys.executable,
        str(script),
        "--install",
        args.install,
        "--out",
        args.out,
        "--fixtures",
        args.fixtures or "",
        "--cases",
        args.cases,
        "--repeat",
        str(args.repeat),
        "--duration-wall",
        str(args.duration_wall),
    ]
    env = os.environ.copy()
    try:
        env.update(_source_install(Path(args.install)))
    except Exception as exc:  # noqa: BLE001
        return _die(NOT_EVAL, "simulate install env: %s" % exc)
    r = subprocess.run(cmd, cwd=str(ROOT), env=env, check=False)
    if r.returncode in (PASS, FAIL, NOT_EVAL):
        return r.returncode
    return FAIL


def cmd_unsupported(args) -> int:
    install = Path(args.install)
    cli = _cli(install)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    fixtures = Path(args.fixtures)
    man = _load_manifest(fixtures)
    scene = man["scenes"][0]
    cmds = out / "unsupported_cmds.yaml"
    resp = out / "unsupported_resp.yaml"
    cmds.write_text("- op: add_box\n  expect_status: UNSUPPORTED_OPERATION\n" * 3)
    r = _run(
        [
            str(cli),
            "--geometry",
            scene["geometry_file"],
            "--config",
            scene["config_file"],
            "--commands",
            str(cmds),
            "--out",
            str(resp),
        ]
    )
    payload = resp.read_text() if resp.exists() else ""
    ok = r.returncode == 0 and payload.count("UNSUPPORTED_OPERATION") >= 3
    (out / "unsupported.json").write_text(
        json.dumps({"pass": ok, "rc": r.returncode, "response": payload[-2000:]}, indent=2)
    )
    return PASS if ok else FAIL


def cmd_replay(args) -> int:
    man = Path(args.manifest)
    if not man.exists():
        return _die(NOT_EVAL, "replay manifest missing")
    payload = json.loads(man.read_text())
    if not payload.get("replay_possible"):
        return FAIL
    return PASS


def _source_install(install: Path) -> dict:
    env = os.environ.copy()
    setup = install / "setup.bash"
    if not setup.exists():
        setup = install / "local_setup.bash"
    cmd = "set +u; source /opt/ros/humble/setup.bash; source %s; python3 -c 'import os,json; print(json.dumps(dict(os.environ)))'" % setup
    r = subprocess.run(["bash", "-lc", cmd], check=False, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr)
    env.update(json.loads(r.stdout.splitlines()[-1]))
    return env


def cmd_o0(args) -> int:
    install = Path(args.install)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    node = None
    for rel in (
        "lib/uld_occ_observer/observer_node",
        "uld_occ_observer/lib/uld_occ_observer/observer_node",
    ):
        if (install / rel).exists():
            node = install / rel
            break
    if node is None:
        return _die(FAIL, "observer_node missing")
    ldd = _run(["ldd", str(node)])
    (out / "ldd.txt").write_text(ldd.stdout + ldd.stderr)
    linked = "liboctomap.so.1.9" in ldd.stdout
    if "luggage_planning" in ldd.stdout or "cargo_map" in ldd.stdout:
        return _die(FAIL, "production mapper/planner linked")
    geom = Path(args.geometry) if getattr(args, "geometry", "") else None
    if geom is None:
        return PASS if linked else FAIL
    env = _source_install(install)
    starts = []
    for i in range(3):
        domain = str(170 + i)
        env_i = dict(env)
        env_i["ROS_DOMAIN_ID"] = domain
        t0 = time.monotonic()
        proc = subprocess.Popen(
            [
                "ros2",
                "launch",
                "uld_occ_observer",
                "standalone.launch.py",
                "geometry_file:=%s" % geom,
                "input_mode:=service",
            ],
            cwd=str(ROOT),
            env=env_i,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        status = "TIMEOUT"
        try:
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                call = subprocess.run(
                    [
                        "ros2",
                        "service",
                        "call",
                        "/uld_occ_observer/get_snapshot",
                        "uld_occ_observer/srv/GetSnapshot",
                        "{max_age_ms: 1000, allow_stale: true, expected_epoch: 0, exact_revision: false, expected_revision: 0}",
                    ],
                    env=env_i,
                    capture_output=True,
                    text=True,
                    check=False,
                    timeout=5,
                )
                if "NO_DATA" in (call.stdout + call.stderr):
                    status = "NO_DATA"
                    break
                time.sleep(0.5)
        finally:
            try:
                os.killpg(proc.pid, 15)
            except OSError:
                proc.terminate()
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, 9)
                except OSError:
                    proc.kill()
        elapsed = time.monotonic() - t0
        starts.append({"i": i, "status": status, "elapsed_s": round(elapsed, 3)})
        if elapsed > 30 or status != "NO_DATA":
            (out / "o0_starts.json").write_text(json.dumps({"linked": linked, "starts": starts}, indent=2))
            return FAIL
    (out / "o0_starts.json").write_text(json.dumps({"linked": linked, "starts": starts}, indent=2))
    return PASS if linked else FAIL


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="occ_observer_eval.py")
    sub = p.add_subparsers(dest="cmd")
    fx = sub.add_parser("fixtures")
    fx.add_argument("--resolutions", default="0.01,0.02,0.05")
    fx.add_argument("--boxes", default="0,5,20")
    fx.add_argument("--hulls", default="cuboid,chamfer")
    fx.add_argument("--seeds", default="10")
    fx.add_argument("--out", required=True)
    cr = sub.add_parser("correctness")
    cr.add_argument("--fixtures", required=True)
    cr.add_argument("--install", required=True)
    cr.add_argument("--out", required=True)
    bm = sub.add_parser("benchmark")
    bm.add_argument("--profile", choices=("representative", "full"), default="representative")
    bm.add_argument("--warmup", type=int, default=3)
    bm.add_argument("--repeat", type=int, default=30)
    bm.add_argument("--install", required=True)
    bm.add_argument("--fixtures", default="")
    bm.add_argument("--out", required=True)
    for name in ("concurrency", "o7"):
        s = sub.add_parser(name)
        s.add_argument("--install", default="")
        s.add_argument("--out", required=True)
        s.add_argument("--fixtures", default="")
        s.add_argument("--node", default="")
        s.add_argument("--setup", default="")
        s.add_argument("--eval-native", default="")
        s.add_argument("--rehearsal", action="store_true")
        s.add_argument("--domain-id", type=int, default=17)
    o6 = sub.add_parser("o6")
    o6.add_argument("--out", required=True)
    o6.add_argument("--fixtures", required=True)
    o6.add_argument("--profile-cli", required=True)
    o6.add_argument("--eval-native", required=True)
    o6.add_argument("--budget-s", type=float, default=8 * 3600)
    o6.add_argument("--rehearsal", action="store_true")
    o6.add_argument("--rehearsal-groups", default="")
    for name in ("stability", "simulate"):
        s = sub.add_parser(name)
        s.add_argument("--install", required=True)
        s.add_argument("--out", required=True)
        s.add_argument("--fixtures", default="")
        s.add_argument("--duration-wall", type=int, default=120)
        s.add_argument("--repeat", type=int, default=3)
        s.add_argument("--boxes", default="0,5,20")
        s.add_argument("--cases", default="static,moving-camera,occluded")
    cap = sub.add_parser("capture-overhead")
    cap.add_argument("--repeat", type=int, default=30)
    cap.add_argument("--install", required=True)
    cap.add_argument("--fixtures", default="")
    cap.add_argument("--out", required=True)
    uns = sub.add_parser("unsupported")
    uns.add_argument("--install", required=True)
    uns.add_argument("--fixtures", required=True)
    uns.add_argument("--out", required=True)
    rp = sub.add_parser("replay")
    rp.add_argument("--manifest", required=True)
    rp.add_argument("--install", required=True)
    rp.add_argument("--out", required=True)
    o0 = sub.add_parser("o0")
    o0.add_argument("--install", required=True)
    o0.add_argument("--out", required=True)
    o0.add_argument("--geometry", default="")
    return p


def main() -> int:
    parser = build_parser()
    if len(sys.argv) == 1:
        parser.print_help()
        return FAIL
    try:
        args = parser.parse_args()
    except SystemExit as exc:
        code = exc.code
        if code in (0, None):
            return PASS
        return FAIL
    if not args.cmd:
        parser.print_help()
        return FAIL
    try:
        if args.cmd == "fixtures":
            return cmd_fixtures(args)
        if args.cmd == "correctness":
            return cmd_correctness(args)
        if args.cmd == "benchmark":
            return cmd_benchmark(args)
        if args.cmd == "capture-overhead":
            return cmd_capture(args)
        if args.cmd in ("concurrency", "o7"):
            return cmd_o7(args)
        if args.cmd == "o6":
            return cmd_o6(args)
        if args.cmd == "stability":
            return cmd_stability(args)
        if args.cmd == "simulate":
            return cmd_simulate(args)
        if args.cmd == "replay":
            return cmd_replay(args)
        if args.cmd == "unsupported":
            return cmd_unsupported(args)
        if args.cmd == "o0":
            return cmd_o0(args)
        return FAIL
    except FileNotFoundError as exc:
        return _die(NOT_EVAL, str(exc))
    except SystemExit as exc:
        if isinstance(exc.code, int):
            return exc.code
        return FAIL
    except Exception as exc:  # noqa: BLE001 — CLI boundary
        return _die(FAIL, "%s: %s" % (type(exc).__name__, exc))


if __name__ == "__main__":
    sys.exit(main())
