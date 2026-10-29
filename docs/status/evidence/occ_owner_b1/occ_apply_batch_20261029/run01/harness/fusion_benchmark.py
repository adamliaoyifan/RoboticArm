#!/usr/bin/env python3
"""Reusable Off/Stages fusion attribution campaign.

Paths come from the constructor or CLI. The runner calls observer_profile_cli
and does not score product acceptance. rehearsal is not a performance sample.
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
import platform
import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from fusion_report import (
    NOT_COLLECTED,
    WindowResult,
    budget_allows,
    campaign_status,
    compare_frames,
    execution_order,
    pair_status,
    perturbation,
    summarize_window,
)

OFFICIAL_CASES = ("o5_b0_c30000", "o5_b0_c100000", "o5_b20_c30000", "o5_b20_c100000")
LOCK_PATH = "/tmp/elfin-occ-backend-perf.lock"
FAILURE_BYTES_LIMIT = 512 * 1024 * 1024
CAMPAIGN_BYTES_LIMIT = 8 * 1024 * 1024 * 1024


@dataclass
class InputManifest:
    campaign: str
    cases: list
    baseline_cli: str
    candidate_cli: str
    geometry: str
    config: str
    files: dict = field(default_factory=dict)


@dataclass
class ProcessResult:
    returncode: int | None
    timed_out: bool
    stdout: str
    stderr: str
    elapsed_s: float


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        for chunk in iter(lambda: source.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_mode(value):
    # YAML 1.1 reads an unquoted off/on scalar as a boolean. The profile schema
    # uses the strings off, stages, and detailed.
    if value is False:
        return "off"
    if value is True:
        return "on"
    return value


def normalize_document(doc):
    if isinstance(doc, dict) and "mode" in doc:
        doc["mode"] = canonical_mode(doc.get("mode"))
    profile = ((doc.get("receipt") or {}).get("profile") if isinstance(doc, dict) else None)
    if isinstance(profile, dict) and "mode" in profile:
        profile["mode"] = canonical_mode(profile.get("mode"))
    return doc


def complete_documents(text):
    docs = []
    for part in text.split("\n---\n"):
        body = part.strip()
        if body.startswith("---"):
            body = body[3:].strip()
        if not body:
            continue
        try:
            doc = yaml.safe_load(body)
        except yaml.YAMLError:
            break
        if isinstance(doc, dict):
            docs.append(normalize_document(doc))
        else:
            break
    return docs


def run_process_group(cmd, timeout_s, log_dir):
    log_dir = Path(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    proc = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, start_new_session=True)
    timed_out = False
    try:
        stdout, stderr = proc.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired as expired:
        timed_out = True
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            rest_out, rest_err = proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            rest_out, rest_err = proc.communicate()
        stdout = (expired.stdout or "") + (rest_out or "")
        stderr = (expired.stderr or "") + (rest_err or "")
    (log_dir / "stdout.txt").write_text(stdout or "")
    (log_dir / "stderr.txt").write_text(stderr or "")
    return ProcessResult(proc.returncode, timed_out, stdout or "", stderr or "",
                         time.monotonic() - started)


def _executable(path):
    candidate = Path(path)
    if not candidate.is_file() or not os.access(candidate, os.X_OK):
        raise ValueError(f"profile CLI is not executable: {path}")
    return str(candidate.resolve())


def _load_case(fixture_root, name, rehearsal):
    folder = Path(fixture_root) / name
    manifest_path = folder / "manifest.yaml"
    if not manifest_path.is_file():
        raise ValueError(f"missing manifest for {name}")
    manifest = yaml.safe_load(manifest_path.read_text())
    frames = manifest.get("frames") or []
    if rehearsal and len(frames) > 8:
        raise ValueError(f"{name} is too large for rehearsal")
    if not rehearsal and (len(frames) != 33 or sum(bool(item.get("warmup")) for item in frames) != 3):
        raise ValueError(f"{name} is not a 3-warmup 30-score window")
    loaded = []
    for item in frames:
        frame_path = (folder / item["path"]).resolve()
        if rehearsal and frame_path.stat().st_size > 2_000_000:
            raise ValueError(f"{name} frame is too large for rehearsal")
        digest = sha256_file(frame_path)
        if digest != item["sha256"]:
            raise ValueError(f"input hash mismatch for {frame_path}")
        loaded.append({
            "path": item["path"],
            "abs_path": str(frame_path),
            "sha256": digest,
            "warmup": bool(item.get("warmup", False)),
            "expect_status": item.get("expect_status", "INTEGRATED"),
        })
    return {
        "name": name,
        "case_id": manifest["case_id"],
        "manifest_sha256": sha256_file(manifest_path),
        "frames": loaded,
    }


def _case_public(case):
    public = {key: value for key, value in case.items() if key != "frames"}
    public["frames"] = len(case["frames"])
    return public


def libraries(binary):
    text = subprocess.check_output(["ldd", binary], text=True, stderr=subprocess.STDOUT)
    found = []
    for line in text.splitlines():
        if "=>" not in line:
            continue
        parts = line.split()
        if len(parts) >= 3 and parts[2].startswith("/"):
            found.append({"name": parts[0], "path": parts[2], "sha256": sha256_file(parts[2])})
    return found


def machine_info():
    cpu = "unknown"
    cpuinfo = Path("/proc/cpuinfo")
    if cpuinfo.is_file():
        for line in cpuinfo.read_text(errors="replace").splitlines():
            if line.startswith("model name"):
                cpu = line.split(":", 1)[1].strip()
                break
    return {"cpu": cpu, "platform": platform.platform(), "python": platform.python_version()}


def _safe(label, function):
    try:
        return function()
    except (OSError, subprocess.CalledProcessError) as exc:
        return {"error": f"{label}: {exc}"}


def repetition_count(campaign, override):
    if override is None:
        return 3 if campaign == "stages" else 1
    if isinstance(override, bool) or not isinstance(override, int) or override < 1:
        raise ValueError("repetitions must be a positive integer")
    if campaign == "memory" and override != 1:
        raise ValueError("memory campaign requires exactly one repetition")
    return override


def content_required(default, force):
    """Omitting the flag keeps the campaign default. Passing it forces content digests."""
    return True if force else bool(default)


def git_info(source_root):
    if not source_root:
        return {}
    def capture(args):
        return subprocess.check_output(args, cwd=source_root, text=True).strip()
    status = capture(["git", "status", "--short"])
    return {"head": capture(["git", "rev-parse", "HEAD"]), "status": status, "dirty": bool(status)}


class FusionBenchmarkSuite:
    def __init__(self, baseline_cli, candidate_cli, fixture_root, geometry, config, out,
                 campaign, timeout_s=900, budget_s=7200, lock_path=LOCK_PATH,
                 lock_wait_s=None, source_root="", repetitions=None, require_content=False):
        self.baseline_cli = str(baseline_cli)
        self.candidate_cli = str(candidate_cli)
        self.fixture_root = Path(fixture_root)
        self.geometry = Path(geometry)
        self.config = Path(config)
        self.out = Path(out)
        self.campaign = campaign
        self.timeout_s = timeout_s
        self.budget_s = budget_s
        self.lock_path = Path(lock_path)
        self.lock_wait_s = lock_wait_s
        self.source_root = source_root
        self.repetitions = repetitions
        self.require_content = bool(require_content)
        self._lock_fh = None
        self._evidence_budget_exceeded = False
        self._incomplete_failure_capture = False

    def validate_inputs(self) -> InputManifest:
        if self.campaign not in {"rehearsal", "baseline", "stages", "memory"}:
            raise ValueError(f"unknown campaign {self.campaign}")
        repetition_count(self.campaign, self.repetitions)
        candidate = _executable(self.candidate_cli)
        baseline = candidate if self.campaign == "memory" else _executable(self.baseline_cli)
        if self.campaign == "memory" and self.require_content:
            raise ValueError("memory campaign cannot require content digests")
        if not self.geometry.is_file() or not self.config.is_file():
            raise ValueError("geometry or config path is missing")
        if not self.fixture_root.is_dir():
            raise ValueError("fixture root is missing")
        if self.out.exists() and any(self.out.rglob("*")):
            raise ValueError("output directory already contains measurement files")
        if self.campaign == "rehearsal":
            names = sorted(path.name for path in self.fixture_root.iterdir()
                           if (path / "manifest.yaml").is_file())
            if not names:
                raise ValueError("rehearsal fixture has no cases")
        else:
            names = list(OFFICIAL_CASES)
        cases = [_load_case(self.fixture_root, name, self.campaign == "rehearsal") for name in names]
        if self.campaign == "memory" and any(
                [frame["warmup"] for frame in case["frames"]] != [True] * 3 + [False] * 30
                for case in cases):
            raise ValueError("memory requires three leading warmups followed by thirty scored frames")
        return InputManifest(
            self.campaign, cases, baseline, candidate, str(self.geometry.resolve()),
            str(self.config.resolve()), {
                "geometry_sha256": sha256_file(self.geometry),
                "config_sha256": sha256_file(self.config),
                "baseline_sha256": sha256_file(baseline),
                "candidate_sha256": sha256_file(candidate),
                "runner_sha256": sha256_file(Path(__file__).resolve()),
                "report_sha256": sha256_file(Path(__file__).resolve().with_name("fusion_report.py")),
            })

    def _write_manifest(self, case, mode, dest):
        dest.mkdir(parents=True, exist_ok=False)
        frames = []
        for item in case["frames"]:
            frames.append({
                "path": os.path.relpath(item["abs_path"], dest),
                "sha256": item["sha256"],
                "warmup": item["warmup"],
                "expect_status": item["expect_status"],
            })
        document = {
            "case_id": case["case_id"],
            "mode": mode,
            "surface": True,
            "state_digest": self.campaign != "memory",
            "frames": frames,
        }
        path = dest / "manifest.yaml"
        path.write_text(yaml.safe_dump(document, sort_keys=False))
        return path

    def run_window(self, binary, mode, case, repetition) -> WindowResult:
        tag = hashlib.sha256(str(Path(binary).resolve()).encode()).hexdigest()[:8]
        dest = self.out / "windows" / f"{case['name']}_r{repetition}_{mode}_{tag}"
        samples = dest / "samples.yaml"
        if samples.exists() or dest.exists():
            return WindowResult(case["name"], repetition, mode, status="failed", binary=binary,
                                error="output already exists")
        manifest = self._write_manifest(case, mode, dest)
        argv = [str(Path(binary).resolve()), "--geometry", str(self.geometry.resolve()),
                "--config", str(self.config.resolve()), "--manifest", str(manifest.resolve()),
                "--out", str(samples.resolve())]
        (dest / "argv.json").write_text(json.dumps(argv))
        process = run_process_group(argv, self.timeout_s, dest)
        text = samples.read_text() if samples.exists() else ""
        rows = complete_documents(text)
        last = rows[-1]["index"] if rows and "index" in rows[-1] else None
        status = "ok"
        error = ""
        if process.timed_out:
            status = "timeout"
            error = "window process group timed out"
        elif process.returncode != 0:
            status = "failed"
            error = process.stderr.strip().splitlines()[-1] if process.stderr.strip() else f"exit {process.returncode}"
        elif len(rows) != len(case["frames"]):
            status = "failed"
            error = f"expected {len(case['frames'])} frames, parsed {len(rows)}"
        returncode = process.returncode if process.returncode is not None else -1
        return WindowResult(
            case["name"], repetition, mode, rows, status, False, returncode,
            str(samples), str(dest / "stdout.txt"), str(dest / "stderr.txt"), last,
            process.timed_out, binary, argv, error)

    def compare_frames(self, left, right):
        return compare_frames(left, right)

    def summarize_window(self, window, expect_scored=30):
        return summarize_window(window, expect_scored)

    def _acquire_lock(self):
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock_fh = open(self.lock_path, "a")
        started = time.monotonic()
        try:
            fcntl.flock(self._lock_fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return {"state": "acquired", "acquired": True, "wait_s": 0.0}
        except BlockingIOError:
            if self.lock_wait_s == 0:
                return {"state": "LOCK_BUSY", "acquired": False, "wait_s": 0.0}
            flags = fcntl.LOCK_EX
            # Queue without consuming a sample. The wait is outside the measurement budget.
            fcntl.flock(self._lock_fh, flags)
            return {"state": "LOCK_BUSY", "acquired": True, "wait_s": time.monotonic() - started}

    def _release_lock(self):
        if self._lock_fh is not None:
            fcntl.flock(self._lock_fh, fcntl.LOCK_UN)
            self._lock_fh.close()
            self._lock_fh = None

    def _sides(self):
        required = content_required(self.campaign == "stages", self.require_content)
        if self.campaign == "baseline":
            return (
                {"role": "A", "label": "baseline_off", "binary": self.baseline_cli, "mode": "off",
                 "require_content": required},
                {"role": "B", "label": "candidate_off", "binary": self.candidate_cli, "mode": "off",
                 "require_content": required},
            )
        return (
            {"role": "A", "label": "candidate_off", "binary": self.candidate_cli, "mode": "off",
             "require_content": required},
            {"role": "B", "label": "candidate_stages", "binary": self.candidate_cli, "mode": "stages",
             "require_content": required},
        )

    def run(self):
        try:
            manifest = self.validate_inputs()
        except ValueError as exc:
            return {"status": "validation_failed", "error": str(exc), "product_acceptance": "not_evaluated",
                    "pairs": []}
        self.baseline_cli = manifest.baseline_cli
        self.candidate_cli = manifest.candidate_cli
        lock = self._acquire_lock()
        if not lock["acquired"]:
            return {"status": "lock_busy", "lock": lock, "product_acceptance": "not_evaluated",
                    "pairs": [], "error": "LOCK_BUSY"}
        try:
            self.out.mkdir(parents=True, exist_ok=True)
            payload = {
                "campaign": manifest.campaign,
                "cases": [_case_public(case) for case in manifest.cases],
                "files": manifest.files,
                "geometry": manifest.geometry,
                "config": manifest.config,
                "performance_sample": manifest.campaign != "rehearsal",
            }
            (self.out / "INPUT_MANIFEST.json").write_text(json.dumps(payload, indent=2, sort_keys=True))
            return self._measure(manifest, lock)
        finally:
            self._release_lock()

    def _measure(self, manifest, lock):
        if self.campaign == "memory":
            return self._measure_memory(manifest, lock)
        measurement_started = time.monotonic()
        started_any = False
        reps = repetition_count(self.campaign, self.repetitions)
        expect_scored = None if self.campaign == "rehearsal" else 30
        sides = {side["role"]: side for side in self._sides()}
        pairs = []
        for case_index, case in enumerate(manifest.cases):
            for repetition in range(reps):
                order = execution_order(case_index, repetition)
                results = {}
                for role in order:
                    if self._retained_bytes(self.out) >= CAMPAIGN_BYTES_LIMIT:
                        self._evidence_budget_exceeded = True
                    if self._evidence_budget_exceeded or not budget_allows(started_any, time.monotonic() - measurement_started, self.budget_s):
                        results[role] = WindowResult(
                            case["name"], repetition, sides[role]["mode"], status="not_evaluated",
                            error="measurement budget exhausted before this window")
                        continue
                    started_any = True
                    window = self.run_window(sides[role]["binary"], sides[role]["mode"], case, repetition)
                    window.require_content = sides[role]["require_content"]
                    results[role] = window
                left = results["A"]
                right = results["B"]
                try:
                    comparison = self.compare_frames(left, right)
                    left_summary = self.summarize_window(left, expect_scored)
                    right_summary = self.summarize_window(right, expect_scored)
                except ValueError as exc:
                    comparison = {"status": "not_comparable", "reason": str(exc)}
                    left_summary = {"status": "failed", "error": str(exc), "case": case["name"],
                                    "repetition": repetition, "mode": left.mode}
                    right_summary = {"status": "failed", "error": str(exc), "case": case["name"],
                                     "repetition": repetition, "mode": right.mode}
                self._write_window(left, left_summary)
                self._write_window(right, right_summary)
                status = pair_status(comparison, left_summary, right_summary)
                if left.status == "not_evaluated" or right.status == "not_evaluated":
                    status = "not_evaluated"
                pair = {
                    "case": case["name"],
                    "repetition": repetition,
                    "order": list(order),
                    "status": status,
                    "comparison": comparison,
                    "A": self._public_summary(left, left_summary),
                    "B": self._public_summary(right, right_summary),
                }
                if self.campaign == "stages":
                    pair["perturbation"] = perturbation(left_summary, right_summary)
                if status not in {"complete", "not_a_performance_sample", "not_evaluated"}:
                    pair["failure_replay"] = self._write_failure_replay(
                        case, repetition, {"A": left, "B": right}, comparison, manifest)
                elif status == "complete" and any(
                        check.get("within_reference") is False
                        for summary in (left_summary, right_summary)
                        for check in summary.get("reference_gate", {}).get("checks", [])):
                    pair["failure_replay"] = self._write_failure_replay(
                        case, repetition, {"A": left, "B": right}, comparison, manifest,
                        trigger="latency_reference", summaries={"A": left_summary, "B": right_summary})
                pairs.append(pair)
                if self._evidence_budget_exceeded or self._incomplete_failure_capture:
                    break
            if self._evidence_budget_exceeded or self._incomplete_failure_capture:
                break
        report = {
            "campaign": self.campaign,
            "status": ("failed" if self._evidence_budget_exceeded or self._incomplete_failure_capture
                       else campaign_status([pair["status"] for pair in pairs])),
            "incomplete_failure_capture": self._incomplete_failure_capture,
            "product_acceptance": "not_evaluated",
            "performance_sample": self.campaign != "rehearsal",
            "lock": lock,
            "measurement_wall_s": time.monotonic() - measurement_started,
            "budget_s": self.budget_s,
            "window_timeout_s": self.timeout_s,
            "repetitions": repetition_count(self.campaign, self.repetitions),
            "require_content": content_required(self.campaign == "stages", self.require_content),
            "files": manifest.files,
            "machine": _safe("machine", machine_info),
            "libraries": _safe("libraries", lambda: libraries(self.candidate_cli)),
            "git": _safe("git", lambda: git_info(self.source_root)),
            "pairs": pairs,
            "attribution": self._attribution(pairs),
            "notes": {
                "pooled_mean": "not_computed",
                "reference_gates": "listed per window and do not decide this campaign",
            },
        }
        (self.out / "campaign_report.json").write_text(json.dumps(report, indent=2, sort_keys=True))
        (self.out / "REPORT.md").write_text(render_report(report))
        return report

    @staticmethod
    def _retained_bytes(folder):
        return sum(path.stat().st_size for path in Path(folder).rglob("*") if path.is_file())

    def _write_failure_replay(self, case, repetition, windows, comparison, manifest,
                              trigger="semantic_or_execution_failure", summaries=None):
        """Reference immutable inputs; replay starts from an empty store, not a checkpoint."""
        indices = []
        if isinstance(comparison.get("frame"), int):
            indices.append(comparison["frame"])
        for window in windows.values():
            indices.extend(row.get("index", index) for index, row in enumerate(window.rows)
                           if row.get("status") != "OK")
            if window.status != "ok" and len(window.rows) < len(case["frames"]):
                indices.append(len(window.rows))
        first = min(indices) if indices else None
        prefix = case["frames"][:first + 1] if first is not None else case["frames"]
        timing = {}
        if trigger == "latency_reference":
            # Percentile failures require all 30 samples, not only the first slow frame.
            prefix = case["frames"]
            for role, summary in (summaries or {}).items():
                timing[role] = {"reference_gate": summary.get("reference_gate"),
                                "frames": summary.get("frames_extracted", [])}
            slow_indices = [frame["index"] for evidence in timing.values()
                            for check in evidence["reference_gate"].get("checks", [])
                            if check.get("within_reference") is False
                            for frame in evidence["frames"] if not frame["warmup"] and
                            (frame[check["metric"]] > check["limit_max_ms"] or
                             (check["p95_ms"] > check["limit_p95_ms"] and
                              frame[check["metric"]] > check["limit_p95_ms"]))]
            first = min(slow_indices) if slow_indices else None
        artifacts = {}
        failure_bytes = 0
        for role, window in windows.items():
            paths = []
            if window.sample_path:
                folder = Path(window.sample_path).parent
                paths = sorted(path for path in folder.rglob("*") if path.is_file())
            failure_bytes += sum(path.stat().st_size for path in paths)
            artifacts[role] = {
                "binary": window.binary, "argv": window.argv,
                "status": window.status, "last_complete_index": window.last_complete_index,
                "replay_manifest": {"case_id": case["case_id"], "mode": window.mode,
                                    "surface": True, "state_digest": self.campaign != "memory",
                                    "frames": [{"path": item["abs_path"], "sha256": item["sha256"],
                                                "warmup": item["warmup"], "expect_status": item["expect_status"]}
                                               for item in prefix]},
                "outputs": [{"path": str(path.resolve()), "bytes": path.stat().st_size,
                             "sha256": sha256_file(path)} for path in paths],
            }
        document = {
            "case": case["name"], "repetition": repetition, "comparison": comparison,
            "trigger": trigger, "timing_evidence": timing,
            "first_failing_index": first, "prefix_includes_failure": first is not None,
            "replay_method": "fresh store; replay ordered prefix with recorded binary/config/geometry",
            "frames": [{"index": index, "path": item["abs_path"], "sha256": item["sha256"],
                        "warmup": item["warmup"], "expect_status": item["expect_status"]}
                       for index, item in enumerate(prefix)],
            "geometry": manifest.geometry, "config": manifest.config, "hashes": manifest.files,
            "windows": artifacts, "capture_complete": False,
            "missing_artifacts": ["pre_checkpoint", "post_checkpoint"],
            "replay_possible": all(Path(item["abs_path"]).is_file() and
                                   sha256_file(item["abs_path"]) == item["sha256"] for item in prefix),
            "budget": {"failure_limit_bytes": FAILURE_BYTES_LIMIT,
                       "campaign_limit_bytes": CAMPAIGN_BYTES_LIMIT,
                       "failure_output_bytes": failure_bytes,
                       "campaign_retained_bytes_before_manifest": self._retained_bytes(self.out),
                       "external_inputs": "referenced with SHA256; not copied or charged as new retained bytes"},
        }
        if trigger == "latency_reference":
            timing_complete = set(timing) == set(windows) and all(
                len(evidence["frames"]) == len(case["frames"]) and
                sum(not frame["warmup"] for frame in evidence["frames"]) == 30
                for evidence in timing.values())
            document["capture_complete"] = document["replay_possible"] and timing_complete
            document["missing_artifacts"] = ([] if document["capture_complete"] else
                                             ["full_inputs_or_timing"])
        self._incomplete_failure_capture |= not document["capture_complete"]
        dest = self.out / "failures"
        dest.mkdir(exist_ok=True)
        path = dest / f"{case['name']}_r{repetition}.json"
        # Include the manifest itself in both limits; never truncate replay evidence.
        encoded = json.dumps(document, indent=2, sort_keys=True)
        exceeded = (failure_bytes + len(encoded.encode()) + 1024 > FAILURE_BYTES_LIMIT or
                    self._retained_bytes(self.out) + len(encoded.encode()) + 1024 > CAMPAIGN_BYTES_LIMIT)
        document["budget"]["status"] = "exceeded" if exceeded else "within_limits"
        self._evidence_budget_exceeded |= exceeded
        with path.open("x") as sink:
            json.dump(document, sink, indent=2, sort_keys=True)
        return {"path": str(path.resolve()), "capture_complete": document["capture_complete"],
                "trigger": trigger,
                "replay_possible": document["replay_possible"], "budget": document["budget"]["status"]}

    def _measure_memory(self, manifest, lock):
        started = time.monotonic()
        windows = []
        for case in manifest.cases:
            if self._retained_bytes(self.out) >= CAMPAIGN_BYTES_LIMIT:
                self._evidence_budget_exceeded = True
            if self._evidence_budget_exceeded or not budget_allows(bool(windows), time.monotonic() - started, self.budget_s):
                window = WindowResult(case["name"], 0, "off", status="not_evaluated",
                                      error="measurement or evidence budget exhausted")
            else:
                window = self.run_window(self.candidate_cli, "off", case, 0)
                if window.status == "ok" and any(row.get("status") != "OK" for row in window.rows):
                    window.status = "failed"
                    window.error = "memory window contains a failed frame"
            try:
                summary = self.summarize_window(window, 30)
            except ValueError as exc:
                summary = {"status": "failed", "error": str(exc)}
            self._write_window(window, summary)
            saved = self._public_summary(window, summary)
            saved["case"] = case["name"]
            saved["semantic_equivalence"] = "not_evaluated"
            saved["process_peak_rss_kib"] = max(
                (row["process_peak_rss_kib"] for row in window.rows if "process_peak_rss_kib" in row),
                default=NOT_COLLECTED)
            if summary["status"] not in {"complete", "not_evaluated"}:
                saved["failure_replay"] = self._write_failure_replay(
                    case, 0, {"candidate": window}, {"status": "not_evaluated",
                    "reason": "memory campaign performs no semantic comparison"}, manifest)
            windows.append(saved)
        if self._retained_bytes(self.out) >= CAMPAIGN_BYTES_LIMIT:
            self._evidence_budget_exceeded = True
        report = {
            "campaign": "memory", "status": ("failed" if self._evidence_budget_exceeded else
                                               campaign_status([item["status"] for item in windows])),
            "evidence_budget": "exceeded" if self._evidence_budget_exceeded else "within_limits",
            "product_acceptance": "not_evaluated", "semantic_equivalence": "not_evaluated",
            "performance_sample": True, "lock": lock, "measurement_wall_s": time.monotonic() - started,
            "budget_s": self.budget_s, "window_timeout_s": self.timeout_s, "repetitions": 1,
            "require_content": False, "state_digest": False, "files": manifest.files,
            "machine": _safe("machine", machine_info),
            "libraries": _safe("libraries", lambda: libraries(self.candidate_cli)),
            "git": _safe("git", lambda: git_info(self.source_root)),
            "pairs": [], "windows": windows, "attribution": {"status": "not_evaluated"},
            "notes": {"memory": "candidate Off process peak RSS without state/content digest allocations"},
        }
        (self.out / "campaign_report.json").write_text(json.dumps(report, indent=2, sort_keys=True))
        (self.out / "REPORT.md").write_text(render_report(report))
        return report

    def _write_window(self, window, summary):
        if not window.sample_path:
            return
        dest = Path(window.sample_path).parent
        frames = summary.get("frames_extracted") or []
        with (dest / "frames.jsonl").open("w") as sink:
            for frame in frames:
                sink.write(json.dumps(frame, sort_keys=True) + "\n")
        saved = {key: value for key, value in summary.items() if key != "frames_extracted"}
        (dest / "summary.json").write_text(json.dumps(saved, indent=2, sort_keys=True))

    def _public_summary(self, window, summary):
        saved = {key: value for key, value in summary.items() if key != "frames_extracted"}
        saved["argv"] = window.argv
        saved["sample_path"] = window.sample_path
        saved["returncode"] = window.returncode
        saved["last_complete_index"] = window.last_complete_index
        return saved

    def _attribution(self, pairs):
        if self.campaign != "stages":
            return {"status": "not_evaluated", "reason": "stages matrix not run"}
        uses = [pair.get("perturbation", {}).get("attribution_use") for pair in pairs]
        dominants = [pair["B"].get("dominant_phase") for pair in pairs if pair["status"] == "complete"]
        if not dominants or any(use != "low_perturbation" for use in uses):
            return {"status": "not_determined", "dominants": dominants, "perturbation": uses,
                    "reason": "missing complete low-perturbation windows or stages disagree"}
        if any(item in {NOT_COLLECTED, "not_separated"} for item in dominants):
            return {"status": "not_determined", "dominants": dominants}
        if len(set(dominants)) != 1:
            return {"status": "not_determined", "dominants": dominants, "reason": "phase ranking differs across windows"}
        return {"status": "consistent", "dominant_phase": dominants[0], "windows": len(dominants)}


def render_report(report):
    lines = [
        f"# Fusion breakdown {report['campaign']}",
        "",
        f"- status: {report['status']}",
        "- product_acceptance: not_evaluated",
        f"- performance_sample: {str(report['performance_sample']).lower()}",
        f"- measurement_wall_s: {report['measurement_wall_s']:.3f}",
        f"- budget_s: {report['budget_s']}",
        "",
        "Reference latency gates are listed on each complete window and do not decide this campaign.",
        "fusion_remainder_ms is unclassified Fusion time, not allocation.",
        "mutation byte products are not RSS.",
        "",
        "| case | rep | status | content_compared | perturbation | dominant |",
        "|---|---:|---|---|---|---|",
    ]
    if report["campaign"] == "memory":
        lines.extend(["Semantic equivalence: not_evaluated (no paired comparison).", ""])
        for window in report["windows"]:
            lines.append(f"- {window['case']}: {window['status']}; samples: {window['sample_path']}")
    for pair in report["pairs"]:
        compared = pair["comparison"].get("content_compared")
        perturb = (pair.get("perturbation") or {}).get("status", "")
        dominant = pair["B"].get("dominant_phase", "")
        lines.append(f"| {pair['case']} | {pair['repetition']} | {pair['status']} | {compared} | {perturb} | {dominant} |")
    lines.extend(["", f"Attribution: `{json.dumps(report['attribution'], sort_keys=True)}`", ""])
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run one fusion breakdown campaign.")
    parser.add_argument("--baseline-cli", default="", help="required for paired campaigns")
    parser.add_argument("--candidate-cli", required=True)
    parser.add_argument("--fixture-root", required=True)
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--campaign", required=True, choices=("rehearsal", "baseline", "stages", "memory"))
    parser.add_argument("--timeout-s", type=float, default=900)
    parser.add_argument("--budget-s", type=float, default=7200)
    parser.add_argument("--source-root", default="")
    parser.add_argument("--lock-path", default=LOCK_PATH)
    parser.add_argument("--repetitions", type=int, default=None)
    parser.add_argument("--require-content", action="store_true")
    args = parser.parse_args(argv)
    suite = FusionBenchmarkSuite(
        args.baseline_cli, args.candidate_cli, args.fixture_root, args.geometry, args.config,
        args.out, args.campaign, args.timeout_s, args.budget_s, args.lock_path,
        source_root=args.source_root, repetitions=args.repetitions,
        require_content=args.require_content)
    report = suite.run()
    print(f"campaign={args.campaign} status={report['status']} product_acceptance=not_evaluated")
    if report["status"] in {"complete", "not_a_performance_sample"}:
        return 0
    if report["status"] in {"validation_failed", "lock_busy"}:
        return 2
    return 3


if __name__ == "__main__":
    raise SystemExit(main())
