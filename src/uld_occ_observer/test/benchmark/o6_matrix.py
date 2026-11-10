"""O6 matrix runner.

The scored integrate path is the frozen observer_profile_cli. This module
generates the load, checks it with the independent classifier, and does not
average one group's failure into another.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import yaml

import fusion_benchmark
import fusion_report

RESOLUTIONS = (0.01, 0.02, 0.05)
BOXES = (0, 5, 20)
HULLS = ("chamfer", "cuboid")
CLOUDS = (30000, 100000)
OUTSIDE = (0.5, 0.9)
CROSS = (0.0, 0.5)
WARMUP = 3
SCORED = 30
COLD = 30
RSS_LIMIT_KIB = 2 * 1024 * 1024
INIT_LIMIT_S = 30.0
QUERY_P95 = fusion_report.REFERENCE_GATES["query"]["p95_ms"]
QUERY_MAX = fusion_report.REFERENCE_GATES["query"]["max_ms"]
LOCK_PATH = Path("/tmp/elfin-occ-backend-perf.lock")


def quotas_for(count, outside, cross):
    n_cross = math.floor(count * outside * cross + 1e-9)
    n_miss = math.floor(count * outside * (1.0 - cross) + 1e-9)
    if n_cross + n_miss > count:
        raise ValueError("quota overflow")
    return {"interior": count - n_cross - n_miss, "cross": n_cross, "miss": n_miss}


def group_id(resolution, boxes, hull, cloud, outside, cross):
    res = {0.01: "r01", 0.02: "r02", 0.05: "r05"}[resolution]
    return "%s_b%02d_%s_c%d_o%02d_x%02d" % (
        res, boxes, hull, cloud, int(round(outside * 100)), int(round(cross * 100)))


def official_matrix(scenes):
    """144 groups. Both container hulls are required."""
    by_key = {(scene["resolution"], scene["boxes"], scene["hull"]): scene for scene in scenes}
    groups = []
    for resolution in RESOLUTIONS:
        for boxes in BOXES:
            for hull in HULLS:
                scene = by_key[(resolution, boxes, hull)]
                for cloud in CLOUDS:
                    for outside in OUTSIDE:
                        for cross in CROSS:
                            groups.append({
                                "id": group_id(resolution, boxes, hull, cloud, outside, cross),
                                "resolution": resolution,
                                "boxes": boxes,
                                "hull": hull,
                                "cloud": cloud,
                                "outside": outside,
                                "cross": cross,
                                "geometry_file": scene["geometry_file"],
                                "config_file": scene["config_file"],
                            })
    if len(groups) != 144 or len({group["hull"] for group in groups}) != 2:
        raise ValueError("O6 matrix is not the 144-group two-hull set")
    if len({group["id"] for group in groups}) != 144:
        raise ValueError("O6 group ids are not unique")
    return groups


def select_benchmark_scenes(manifest, profile):
    """Representative stays the small chamfer slice. Full keeps every hull."""
    selected = []
    for scene in manifest["scenes"]:
        if profile == "representative":
            if scene["hull"] != "chamfer" or scene["resolution"] != 0.01 or scene["boxes"] not in (0, 20):
                continue
            loads = ((30000, 0.5, 0.0), (100000, 0.5, 0.0))
        elif profile == "full":
            loads = tuple(
                (cloud, outside, cross)
                for cloud in CLOUDS for outside in OUTSIDE for cross in CROSS)
        else:
            raise ValueError("unknown benchmark profile")
        for cloud, outside, cross in loads:
            selected.append({
                "scene": scene,
                "cloud": cloud,
                "outside": outside,
                "cross": cross,
            })
    return selected


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _versions(profile_cli, eval_native):
    def git(args):
        try:
            return subprocess.check_output(args, cwd=str(Path(__file__).resolve().parents[4]), text=True).strip()
        except subprocess.CalledProcessError:
            return ""
    status = git(["git", "status", "--short"])
    return {
        "tool_head": git(["git", "rev-parse", "HEAD"]),
        "tool_dirty": bool(status),
        "tool_status": status,
        "profile_cli_sha256": _sha256(profile_cli),
        "eval_native_sha256": _sha256(eval_native),
    }


def campaign_and(groups, expected=144):
    """One failed or missing group keeps the campaign from passing. No pooled latency."""
    statuses = [group.get("status") for group in groups]
    if any(status == "fail" for status in statuses):
        verdict = "fail"
    elif len(groups) != expected or any(status != "pass" for status in statuses):
        verdict = "not_evaluated"
    else:
        verdict = "pass"
    return {
        "verdict": verdict,
        "groups": len(groups),
        "expected_groups": expected,
        "pooled_latency": "not_computed",
    }


def _finite_numbers(rows, extract):
    values = []
    for row in rows:
        value = extract(row)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("invalid metric sample")
        values.append(float(value))
    return values


def _gate(name, values, expected, p95_limit, max_limit):
    stats = fusion_report.exact_stats(values, expected)
    return {
        "metric": name,
        "stats": stats,
        "limit_p95_ms": p95_limit,
        "limit_max_ms": max_limit,
        "pass": fusion_report.within_gate(stats, p95_limit, max_limit),
    }


def score_latency(rows, cloud, expected):
    integrate_limit = fusion_report.REFERENCE_GATES["integrate_100k" if cloud >= 100000 else "integrate_30k"]
    gates = [
        _gate("integrate_external_ms", _finite_numbers(rows, lambda row: row["integrate_external_ms"]),
              expected, integrate_limit["p95_ms"], integrate_limit["max_ms"]),
        _gate("acquire_ms", _finite_numbers(rows, lambda row: row["acquire_ms"]),
              expected, QUERY_P95, QUERY_MAX),
        _gate("roi_ms", _finite_numbers(rows, lambda row: row["roi_ms"]),
              expected, QUERY_P95, QUERY_MAX),
        _gate("surface_first_ms", _finite_numbers(rows, lambda row: row["surface_first_call"]["external_ms"]),
              expected, QUERY_P95, QUERY_MAX),
        _gate("surface_second_ms", _finite_numbers(rows, lambda row: row["surface_second_call"]["external_ms"]),
              expected, QUERY_P95, QUERY_MAX),
    ]
    return {"gates": gates, "pass": all(gate["pass"] for gate in gates)}


class O6Campaign:
    def __init__(self, profile_cli, eval_native, fixtures, out, budget_s=8 * 3600,
                 rehearsal=False, rehearsal_groups=(), selected_groups=(), lock_path=LOCK_PATH):
        self.profile_cli = Path(profile_cli)
        self.eval_native = Path(eval_native)
        self.fixtures = Path(fixtures)
        self.out = Path(out)
        self.budget_s = budget_s
        self.rehearsal = rehearsal
        self.rehearsal_groups = tuple(rehearsal_groups)
        self.selected_groups = tuple(selected_groups)
        self.lock_path = Path(lock_path)
        self._lock = None

    def _groups(self):
        manifest = json.loads((self.fixtures / "TEST_MANIFEST.json").read_text())
        if "implementation_revision" in manifest or "plan_revision" in manifest:
            raise ValueError("fixture manifest hardcodes a product or plan SHA")
        groups = official_matrix(manifest["scenes"])
        if self.rehearsal:
            if not self.rehearsal_groups or self.selected_groups:
                raise ValueError("rehearsal needs an explicit group list and is not a scored subset")
            chosen = [group for group in groups if group["id"] in self.rehearsal_groups]
            if len(chosen) != len(self.rehearsal_groups):
                raise ValueError("rehearsal group is not in the 144")
            return chosen
        if self.selected_groups:
            chosen = [group for group in groups if group["id"] in self.selected_groups]
            if len(chosen) != len(set(self.selected_groups)):
                raise ValueError("selected group is not in the 144")
            return chosen
        return groups

    def _lock_acquire(self):
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = open(self.lock_path, "a")
        started = time.monotonic()
        import fcntl
        fcntl.flock(self._lock, fcntl.LOCK_EX)
        return time.monotonic() - started

    def _lock_release(self):
        if self._lock is not None:
            import fcntl
            fcntl.flock(self._lock, fcntl.LOCK_UN)
            self._lock.close()
            self._lock = None

    def run(self):
        groups = self._groups()
        self.out.mkdir(parents=True, exist_ok=True)
        if any(self.out.iterdir()):
            raise ValueError("output directory already contains files")
        wait_s = self._lock_acquire()
        started = time.monotonic()
        results = []
        try:
            for index, group in enumerate(groups):
                elapsed = time.monotonic() - started
                if elapsed > self.budget_s:
                    for pending in groups[index:]:
                        results.append({"id": pending["id"], "status": "not_evaluated",
                                        "reason": "measurement budget exhausted"})
                    break
                results.append(self._run_group(group))
                (self.out / "partial_verdict.json").write_text(json.dumps({
                    "groups": results, "elapsed_s": time.monotonic() - started}, indent=2) + "\n")
        finally:
            self._lock_release()
        expected = len(groups) if (self.rehearsal or self.selected_groups) else 144
        verdict = campaign_and(results, expected=expected)
        if self.rehearsal:
            verdict = {"verdict": "rehearsal", "product_scored": False, "groups": len(results),
                       "pooled_latency": "not_computed"}
        elif self.selected_groups:
            verdict["full_matrix"] = False
        report = {
            "campaign_role": "rehearsal" if self.rehearsal else "scored_subset" if self.selected_groups else "scored",
            "versions": _versions(self.profile_cli, self.eval_native),
            "lock_wait_s": wait_s,
            "measurement_s": time.monotonic() - started,
            "read_view_and_cache_bytes": {
                "status": "not_evaluated",
                "reason": "the frozen interface does not attribute read-view or cache bytes; RSS is not a substitute",
            },
            "verdict": verdict,
            "groups": results,
        }
        (self.out / "O6_VERDICT.json").write_text(json.dumps(report, indent=2) + "\n")
        return report

    def _run_group(self, group):
        folder = self.out / "groups" / group["id"]
        folder.mkdir(parents=True)
        outcome = {"id": group["id"], "status": "fail", "hull": group["hull"], "cloud": group["cloud"]}
        try:
            emitted = self._emit(group, folder / "load")
            self._check_load(group, emitted)
            warm = self._profile(group, folder / "warm", emitted, scored=True)
            cold_rows = []
            cold_init = []
            cold_rss = []
            for frame in emitted["frames"]:
                if frame["warmup"]:
                    continue
                cold = self._profile_one(group, folder / "cold" / ("c%02d" % frame["index"]), frame)
                cold_rows.append(cold["row"])
                cold_init.append(cold["startup_to_first_sample_s"])
                cold_rss.append(cold["peak_rss_kib"])
            scored_rows = [row for row in warm["rows"] if not row.get("warmup")]
            latency = score_latency(scored_rows, group["cloud"], SCORED)
            cold_latency = score_latency(cold_rows, group["cloud"], COLD)
            # Cold integrate is reported and is not the warmed-window gate.
            cold_integrate = cold_latency["gates"][0]
            cold_query = cold_latency["gates"][1:]
            init_ok = (warm["startup_to_first_sample_s"] is not None and
                       warm["startup_to_first_sample_s"] <= INIT_LIMIT_S and
                       all(value is not None and value <= INIT_LIMIT_S for value in cold_init))
            rss_ok = warm["peak_rss_kib"] <= RSS_LIMIT_KIB and all(value <= RSS_LIMIT_KIB for value in cold_rss)
            load_ok = self._frames_conserve(group, warm["rows"], emitted)
            passed = (latency["pass"] and all(gate["pass"] for gate in cold_query) and
                      init_ok and rss_ok and load_ok and warm["status"] == "ok" and
                      len(cold_rows) == COLD)
            outcome.update({
                "status": "pass" if passed else "fail",
                "warm_latency": latency,
                "cold_query": cold_query,
                "cold_integrate": {"gate": "not_applied", "stats": cold_integrate["stats"]},
                "startup_to_first_sample_s": warm["startup_to_first_sample_s"],
                "cold_startup_to_first_sample_s": cold_init,
                "peak_rss_kib": warm["peak_rss_kib"],
                "cold_peak_rss_kib": cold_rss,
                "init_pass": init_ok,
                "rss_pass": rss_ok,
                "load_pass": load_ok,
            })
            if passed:
                for path in (folder / "load" / "frames").glob("*.yaml"):
                    path.unlink()
        except Exception as exc:  # noqa: BLE001 - one group must not kill the matrix
            outcome["status"] = "fail"
            outcome["reason"] = str(exc)
        (folder / "group_verdict.json").write_text(json.dumps(outcome, indent=2) + "\n")
        return outcome

    def _emit(self, group, dest):
        dest.mkdir(parents=True)
        cmd = [str(self.eval_native), "--geometry", group["geometry_file"], "--config", group["config_file"],
               "--gate", "emit-window", "--out", str(dest), "--group", group["id"],
               "--boxes", str(group["boxes"]), "--cloud", str(group["cloud"]),
               "--outside", str(group["outside"]), "--cross", str(group["cross"]),
               "--warmup", str(WARMUP), "--repeat", str(SCORED)]
        (dest.parent / "emit_argv.json").write_text(json.dumps(cmd) + "\n")
        result = subprocess.run(cmd, capture_output=True, text=True, check=False)
        (dest.parent / "emit_stderr.txt").write_text(result.stderr or "")
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or "emit-window failed")
        return json.loads((dest / "load_manifest.json").read_text())

    def _check_load(self, group, emitted):
        expected = quotas_for(group["cloud"], group["outside"], group["cross"])
        if len(emitted["frames"]) != WARMUP + SCORED:
            raise RuntimeError("frame count")
        for frame in emitted["frames"]:
            if frame["invalid"] != 0 or frame["interior"] != expected["interior"] or \
                    frame["cross"] != expected["cross"] or frame["miss"] != expected["miss"]:
                raise RuntimeError("classified load does not match the quota")

    def _frame_path(self, group, frame):
        return self.out / "groups" / group["id"] / "load" / frame["file"]

    def _profile(self, group, dest, emitted, scored):
        del scored
        dest.mkdir(parents=True)
        manifest = {
            "case_id": group["id"],
            "mode": "off",
            "surface": True,
            "state_digest": False,
            "frames": [{
                "path": os.path.relpath(self._frame_path(group, frame), dest),
                "sha256": frame["file_sha256"],
                "warmup": frame["warmup"],
                "expect_status": "INTEGRATED",
            } for frame in emitted["frames"]],
        }
        return self._run_profile(group, dest, manifest, timeout_s=900)

    def _profile_one(self, group, dest, frame):
        dest.mkdir(parents=True)
        manifest = {
            "case_id": group["id"] + "-cold-%02d" % frame["index"],
            "mode": "off",
            "surface": True,
            "state_digest": False,
            "frames": [{
                "path": os.path.relpath(self._frame_path(group, frame), dest),
                "sha256": frame["file_sha256"],
                "warmup": False,
                "expect_status": "INTEGRATED",
            }],
        }
        result = self._run_profile(group, dest, manifest, timeout_s=180)
        if len(result["rows"]) != 1:
            raise RuntimeError("cold process did not record one frame")
        return {"row": result["rows"][0], "startup_to_first_sample_s": result["startup_to_first_sample_s"],
                "peak_rss_kib": result["peak_rss_kib"], "status": result["status"]}

    def _run_profile(self, group, dest, manifest, timeout_s):
        manifest_path = dest / "manifest.yaml"
        samples = dest / "samples.yaml"
        manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False))
        cmd = [str(self.profile_cli), "--geometry", group["geometry_file"], "--config", group["config_file"],
               "--manifest", str(manifest_path), "--out", str(samples)]
        (dest / "argv.json").write_text(json.dumps(cmd) + "\n")
        started = time.monotonic()
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                                start_new_session=True)
        first = None
        timed_out = False
        def stop(sig):
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, sig)
                except ProcessLookupError:
                    pass
        try:
            while proc.poll() is None:
                if first is None and samples.exists() and samples.stat().st_size > 0:
                    first = time.monotonic() - started
                if time.monotonic() - started > timeout_s:
                    timed_out = True
                    stop(signal.SIGTERM)
                    break
                time.sleep(0.05)
            try:
                stdout, stderr = proc.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                stop(signal.SIGKILL)
                stdout, stderr = proc.communicate()
        finally:
            stop(signal.SIGKILL)
        if first is None and samples.exists() and samples.stat().st_size > 0:
            # The process exited before the poll observed the first flush.
            # Whole-process time is an upper bound on startup, not a missing sample.
            first = time.monotonic() - started
        (dest / "stdout.txt").write_text(stdout or "")
        (dest / "stderr.txt").write_text(stderr or "")
        rows = fusion_benchmark.complete_documents(samples.read_text()) if samples.exists() else []
        peak = 0
        for row in rows:
            try:
                peak = max(peak, int(row.get("process_peak_rss_kib") or 0))
            except (TypeError, ValueError):
                peak = 0
        status = "ok"
        if timed_out or proc.returncode != 0 or peak <= 0:
            status = "failed"
        return {"rows": rows, "status": status, "startup_to_first_sample_s": first,
                "peak_rss_kib": peak, "returncode": proc.returncode}

    def _frames_conserve(self, group, rows, emitted):
        if len(rows) != len(emitted["frames"]):
            return False
        for row, frame in zip(rows, emitted["frames"]):
            receipt = row.get("receipt") or {}
            if row.get("status") != "OK" or receipt.get("status") != "INTEGRATED":
                return False
            if int(receipt.get("raw_points", -1)) != group["cloud"]:
                return False
            if int(receipt.get("accepted_points", -1)) != group["cloud"]:
                return False
            if bool(row.get("warmup")) != bool(frame["warmup"]):
                return False
        return True
