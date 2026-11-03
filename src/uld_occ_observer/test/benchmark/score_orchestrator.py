#!/usr/bin/env python3
"""The unified formal-score orchestrator (S3).

One serial formal measurement under ONE hold of the backend perf lock, with
ONE 8-hour budget that covers every formal stage (P/H -> R -> L -> O8 -> O5 ->
O6) plus the final judge pass. Lock waiting is not budget time; the clock
starts after the lock is held.

What this adds over the per-stage COMMANDS entries:

- run_manifest.json: run_id, freeze binding, per-stage records (argv, env
  markers, start/end, exit code, duration). A stage the orchestrator never
  started (budget/dependency) is recorded ``skipped`` with its reason; a
  stage that ran and exited non-zero is a real result, never rewritten.
- input_manifest.json: the exact formal input workload -- every frame file
  by SHA and box tag, the per-stage stamp mapping rule, and the counts. The
  six rehearsal frames alone never claimed to freeze the formal workload;
  this manifest plus the frozen frame files do.
- ACTUAL_COMMANDS.log: every executed command with timestamps and exit codes.
- Stop rules: P failure stops R/L inside the campaign runner (its own
  scheduler); O5 failure stops O6 here; every other stage is independent and
  continues while budget allows. A stage whose conservative estimate does
  not fit the remaining budget is NOT started (recorded not_evaluated) --
  the 120-minute L soak is never cut short to make room for a later stage.

The judge step recomputes the FIXED 20-row verdict from the artifact files
and writes verdict.json in the run root. The judge's exit code reflects
failure: 0 only when no row fails.
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import freespace_contract as fs_contract  # noqa: E402

LOCK_PATH = Path("/tmp/elfin-occ-backend-perf.lock")

# The stage executor gates read VALUES, not just presence: the live adapter
# marker must say "live", the S3/official markers must say "1".
STAGE_ENV = {"ULD_OCC_S2_ADAPTER": "live", "ULD_OCC_S3_OPEN": "1",
             "ULD_OCC_S2_CONFIRM_OFFICIAL": "1"}

# Conservative whole-stage estimates (minutes) for the pre-start budget gate.
# Formal only; the bounded rehearsal runs inside its own small budget.
FORMAL_ESTIMATE_MIN = {
    # P 9x120 s + R ~2.5 min + L 120 min + node launches, readiness, drain.
    "campaign": 165.0,
    # 3 x 180 s runs + 30 saturation frames + 9 holds x >=10 s + launches.
    "o8": 25.0,
    # 4 groups x (cumulative 34 samples + 30 cold processes + emit).
    "o5": 50.0,
    # o6 is estimated from the measured o5 per-group rate x 96 x margin.
    "o6_margin": 1.3,
    "judge": 10.0,
}


def _iso():
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")


def _sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def _write_json(path, payload):
    Path(path).write_text(json.dumps(payload, indent=1, sort_keys=True) + "\n")


def _env_snapshot():
    """Cheap environment record: load, governor, core count."""
    snapshot = {"utc": _iso()}
    try:
        snapshot["loadavg"] = open("/proc/loadavg").read().split()[:3]
    except OSError:
        snapshot["loadavg"] = None
    try:
        governors = {}
        for cpu in sorted(os.listdir("/sys/devices/system/cpu")):
            if not re.fullmatch(r"cpu\d+", cpu):
                continue
            path = Path("/sys/devices/system/cpu") / cpu / "cpufreq" / "scaling_governor"
            if path.is_file():
                governors[cpu] = path.read_text().strip()
        snapshot["governors"] = governors or None
    except OSError:
        snapshot["governors"] = None
    snapshot["nproc"] = os.cpu_count()
    return snapshot


def verify_product_shas(freeze_path):
    """Byte-verify the FROZEN product binaries still on disk.

    Returns (freeze_document, problems). The orchestrator refuses to score on
    any mismatch: a rebuilt or edited product is a new candidate, not this
    one.
    """
    freeze = _load_json(freeze_path)
    if not isinstance(freeze, dict):
        return None, ["freeze_unreadable:%s" % freeze_path]
    problems = []
    checked = 0
    binaries = freeze.get("binaries")
    if not isinstance(binaries, dict):
        return freeze, ["freeze_binaries_group_missing"]
    for name, entry in sorted(binaries.items()):
        path = entry.get("path") if isinstance(entry, dict) else None
        sha = entry.get("sha256") if isinstance(entry, dict) else None
        if not path or not sha:
            problems.append("freeze_binary_entry_%s" % name)
            continue
        if not Path(path).is_file():
            problems.append("freeze_binary_missing_%s" % name)
            continue
        checked += 1
        if _sha256_file(path) != sha:
            problems.append("freeze_binary_mismatch_%s" % name)
    return {"sha256": _sha256_file(freeze_path), "binaries_checked": checked,
            "head": freeze.get("head") if isinstance(freeze.get("head"), str)
            else (freeze.get("git_head") or None)}, problems


def verify_s3_freeze(s3_freeze_path):
    """Byte-verify the S3 freeze: the scoring-side tools, the input frames,
    and the scoring COMMANDS itself. The gen4 PRODUCT groups are verified
    separately against the gen4 freeze (they must be unchanged)."""
    freeze = _load_json(s3_freeze_path)
    if not isinstance(freeze, dict):
        return None, ["s3_freeze_unreadable:%s" % s3_freeze_path]
    problems = []
    checked = 0
    for group in ("tools", "inputs", "commands"):
        entries = freeze.get(group)
        if not isinstance(entries, dict) or not entries:
            problems.append("s3_freeze_group_empty_%s" % group)
            continue
        for name, entry in entries.items():
            path = entry.get("path") if isinstance(entry, dict) else None
            sha = entry.get("sha256") if isinstance(entry, dict) else None
            if not path or not sha or not Path(path).is_file() \
                    or _sha256_file(path) != sha:
                problems.append("s3_freeze_mismatch_%s_%s" % (group, name))
            else:
                checked += 1
    return {"sha256": _sha256_file(s3_freeze_path),
            "entries_matched": checked, "head": freeze.get("head")}, problems


def build_input_manifest(frames_dir, formal):
    """Bind the formal input workload: frames by SHA + the stamp rules.

    The formal inputs are the frozen frame files sent cyclically at the
    contract rates; the observation ids are the stamp stream. This manifest
    is the verifiable lossless generation rule the acceptance contract asks
    for -- the six rehearsal files alone never described the formal load.
    """
    import yaml
    frames = []
    for path in sorted(Path(frames_dir).glob("b*.yaml")):
        tag = re.match(r"b(\d+)_", path.name)
        frames.append({
            "name": path.name,
            "box_tag": int(tag.group(1)) if tag else None,
            "sha256": _sha256_file(path),
            "points": len(yaml.safe_load(path.read_text()).get("points") or [])
            if path.stat().st_size < (8 << 20) else "large",
        })
    if not frames:
        raise SystemExit("no frame files under %s" % frames_dir)
    manifest = {
        "formal": bool(formal),
        "generator": "frozen frame files sent cyclically; stamps are the "
                     "identity stream (send_inputs), observation_id = "
                     "str(base_stamp + index * period_ns)",
        "frames": frames,
        "stamp_rules": {
            "p_window": {
                "inputs": fs_contract.P_WINDOW_INPUTS if formal else "bounded",
                "hz": fs_contract.UPDATE_HZ,
                "stamp_step_ns": int(1e9 / fs_contract.UPDATE_HZ),
            },
            "r_overload": {
                "inputs": fs_contract.R_OVERLOAD_INPUTS if formal else "bounded",
                "hz": 10.0,
                "stamp_step_ns": int(1e8),
            },
            "r_recovery": {
                "inputs": fs_contract.R_RECOVERY_INPUTS if formal else "bounded",
                "hz": fs_contract.UPDATE_HZ,
                "stamp_step_ns": int(1e9 / fs_contract.UPDATE_HZ),
            },
            "l": {
                "inputs": "continuous at update hz for the soak duration",
                "hz": fs_contract.UPDATE_HZ,
                "stamp_step_ns": int(1e9 / fs_contract.UPDATE_HZ),
            },
        },
        "p_windows_per_box": fs_contract.P_WINDOWS_PER_BOX if formal else 1,
        "query_box_models_m": [
            [0.30, 0.20, 0.20],
            [0.50, 0.30, 0.30],
        ],
        "query_yaws_deg": list(fs_contract.LEGAL_QUERY_YAWS)
        if hasattr(fs_contract, "LEGAL_QUERY_YAWS") else [0, 90],
    }
    return manifest


class Orchestrator:
    def __init__(self, args):
        self.args = args
        self.root = Path(args.root)
        self.budget_s = args.budget_minutes * 60.0
        self.manifest = {
            "run_id": args.run_id,
            "formal": not args.rehearsal,
            "budget_minutes": args.budget_minutes,
            "lock_path": str(LOCK_PATH),
            "worktree": str(Path(args.worktree).resolve()),
            "qualification_root": self._relative_qualification(),
            "stage_order": ["campaign", "o8", "o5", "o6", "judge"],
            "stages": {},
            "created_at": _iso(),
            "argv": list(sys.argv),
        }
        self.lock_file = None
        self.lock_wait_s = None
        self.started_mono = None
        self.actual_log = self.root / "ACTUAL_COMMANDS.log"

    def _relative_qualification(self):
        # Both roots live under docs/status/evidence/occ_freespace_final_*/
        # -- a relative pointer survives the copy to the main workspace.
        target = Path(self.args.qualification_root)
        try:
            return str(target.resolve().relative_to(self.root.resolve().parent))
        except ValueError:
            return str(target)

    # -- bookkeeping ---------------------------------------------------------

    def save_manifest(self):
        _write_json(self.root / "run_manifest.json", self.manifest)

    def log_actual(self, text):
        with open(self.actual_log, "a") as handle:
            handle.write(text.rstrip("\n") + "\n")

    def elapsed_min(self):
        return (time.monotonic() - self.started_mono) / 60.0 if self.started_mono \
            else 0.0

    def remaining_min(self):
        return self.args.budget_minutes - self.elapsed_min()

    # -- execution ------------------------------------------------------------

    def acquire_lock(self):
        LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
        self.lock_file = open(LOCK_PATH, "a")
        began = time.monotonic()
        fcntl.flock(self.lock_file, fcntl.LOCK_EX)
        self.lock_wait_s = time.monotonic() - began
        # The budget clock starts HERE: lock waiting is not measurement time.
        self.started_mono = time.monotonic()
        # setdefault: a resume keeps the FIRST launch's lock record and only
        # notes its own wait separately.
        self.manifest.setdefault("lock_wait_s", round(self.lock_wait_s, 3))
        self.manifest.setdefault("measurement_started_at", _iso())
        self.manifest.setdefault("resumes", []).append({
            "at": _iso(), "stage": getattr(self, "_resume_stage_marker", None),
            "lock_wait_s": round(self.lock_wait_s, 3)})

    def release_lock(self):
        if self.lock_file is not None:
            fcntl.flock(self.lock_file, fcntl.LOCK_UN)
            self.lock_file.close()
            self.lock_file = None

    def stage_state(self, name):
        return self.manifest["stages"].setdefault(name, {})

    def run_stage(self, name, argv, env_overrides=None, estimate_min=None,
                  timeout_s=None):
        """Execute one stage as a process group; record everything."""
        state = self.stage_state(name)
        if estimate_min is not None and not self.args.rehearsal:
            if self.remaining_min() < estimate_min:
                state.update({
                    "status": "skipped", "not_evaluated_reason":
                        "budget_insufficient_estimate_%.0fmin_remaining_%.0fmin"
                        % (estimate_min, max(0.0, self.remaining_min())),
                    "argv": [str(item) for item in argv],
                    "decided_at": _iso(),
                })
                self.save_manifest()
                self.log_actual("SKIP %s (budget): %s" % (name, state["not_evaluated_reason"]))
                return None
        environment = dict(os.environ)
        environment.update(env_overrides or {})
        state.update({
            "status": "started", "argv": [str(item) for item in argv],
            "env_overrides": dict(env_overrides or {}), "started_at": _iso(),
            "env_before": _env_snapshot(),
            "budget_remaining_min_at_start": round(self.remaining_min(), 2),
        })
        self.save_manifest()
        self.log_actual("START %s :: %s" % (_iso(), " ".join(state["argv"])))
        log_path = self.root / "logs" / ("%s.log" % name)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        began = time.monotonic()
        exit_code = None
        timed_out = False
        with open(log_path, "w") as log:
            process = subprocess.Popen(
                [str(item) for item in argv], stdout=log,
                stderr=subprocess.STDOUT, env=environment,
                start_new_session=True, cwd=str(self.args.worktree))
            hard_limit = timeout_s if timeout_s is not None \
                else max(60.0, self.remaining_min() * 60 + 900)
            try:
                exit_code = process.wait(timeout=hard_limit)
            except subprocess.TimeoutExpired:
                timed_out = True
                for sig in (2, 15, 9):
                    try:
                        os.killpg(process.pid, sig)
                    except ProcessLookupError:
                        break
                    try:
                        process.wait(timeout=10)
                        break
                    except subprocess.TimeoutExpired:
                        continue
                exit_code = process.poll()
                if exit_code is None:
                    exit_code = -9
        duration_s = time.monotonic() - began
        state.update({
            "status": "completed" if not timed_out else "timeout_killed",
            "ended_at": _iso(), "exit_code": exit_code,
            "duration_s": round(duration_s, 1),
            "env_after": _env_snapshot(),
            "log": str(log_path.relative_to(self.root)),
        })
        self.save_manifest()
        self.log_actual("END %s exit=%s dur=%.1fs :: %s"
                        % (_iso(), exit_code, duration_s, name))
        return exit_code

    def skip_stage(self, name, reason):
        self.stage_state(name).update({
            "status": "skipped", "not_evaluated_reason": reason,
            "decided_at": _iso(),
        })
        self.save_manifest()
        self.log_actual("SKIP %s: %s" % (name, reason))

    # -- stage builders --------------------------------------------------------

    def campaign_argv(self):
        args = self.args
        return [
            sys.executable, Path(args.worktree) / CAMPAIGN_REL,
            "--node", args.node, "--eval-native", args.eval_native,
            "--frames-glob", str(Path(args.frames_dir) / "b*.yaml"),
            "--out", str(self.root / "campaign"), "--stages", "p,r,l",
            "--budget-minutes", "%.1f" % max(1.0, self.remaining_min()),
        ] + ([] if self.args.rehearsal else ["--formal"])

    def executor_argv(self, stage):
        args = self.args
        extra = []
        if self.args.rehearsal:
            extra = ["--frame-limit", "2"]
            if stage == "o8":
                extra = ["--input-limit", "2"]
        # --out is the RUN ROOT: the executor prepends the stage name itself
        # (out/<stage>/...), which is the layout the formal rows judge.
        return [
            sys.executable, Path(args.worktree) / EXECUTOR_REL,
            "--mode", "stage", "--stage", stage,
            "--cli", args.cli, "--eval-native", args.eval_native,
            "--node", args.node, "--geometry", args.geometry,
            "--config", args.config, "--out", str(self.root),
            "--budget-minutes", "%.1f" % max(1.0, self.remaining_min()),
        ] + extra

    def judge_argv(self):
        return [sys.executable, Path(self.args.worktree) / VERDICT_REL,
                str(self.root), "write"]

    # -- top level --------------------------------------------------------------

    def prepare(self):
        self.root.mkdir(parents=True, exist_ok=True)
        if any(self.root.iterdir()):
            raise SystemExit("refusing to run inside a non-empty %s" % self.root)
        freeze_report, problems = verify_product_shas(self.args.freeze_json)
        s3_report, s3_problems = None, []
        if self.args.s3_freeze:
            s3_report, s3_problems = verify_s3_freeze(self.args.s3_freeze)
        if problems or freeze_report is None or s3_problems:
            _write_json(self.root / "freeze_gate_refused.json",
                        {"problems": problems + s3_problems})
            raise SystemExit("freeze gate refused: %s" % (problems + s3_problems))
        self.manifest["freeze"] = freeze_report
        if s3_report is not None:
            s3_path = Path(self.args.s3_freeze)
            try:
                relative = s3_path.resolve().relative_to(self.root.resolve().parent)
            except ValueError:
                relative = str(s3_path.resolve())
            self.manifest["s3_freeze"] = {"path": str(relative),
                                          "sha256": s3_report["sha256"]}
        input_manifest = build_input_manifest(self.args.frames_dir,
                                              formal=not self.args.rehearsal)
        _write_json(self.root / "input_manifest.json", input_manifest)
        self.manifest["input_manifest_sha256"] = _sha256_file(
            self.root / "input_manifest.json")
        self.save_manifest()
        self.log_actual("ORCHESTRATOR %s" % " ".join(sys.argv))

    def o6_decision(self):
        """Pure decision: run O6 with an estimate, or skip with a reason."""
        if self.args.rehearsal:
            return None, "rehearsal_wiring_o5_only"
        o5_state = self.manifest["stages"].get("o5") or {}
        o5_exit = o5_state.get("exit_code")
        if o5_exit != 0:
            return None, "o5_dependency_failed_exit_%s" % o5_exit
        per_group_min = float(o5_state.get("duration_s") or 0.0) / 60.0 / 4.0
        return per_group_min * 96.0 * FORMAL_ESTIMATE_MIN["o6_margin"], None

    def resume(self, stage):
        """Resume one contract-order stage into an EXISTING run root.

        Allowed by the delivery contract: the formal path may resume stage by
        stage but never repeats an already-scored unit. The resume is recorded
        under its own manifest key; the original stage records stay untouched
        history. The lock is re-held (wait excluded) and the budget continues
        from the run's recorded measurement time -- wall-clock gaps between
        resumes are not measurement time.
        """
        if not (self.root / "run_manifest.json").is_file():
            raise SystemExit("resume needs an existing run root")
        if stage not in ("l", "o6"):
            raise SystemExit("resume supports l and o6 only")
        # Same candidate discipline as the first launch: the product must
        # still be byte-identical to the bound freeze.
        _report, problems = verify_product_shas(self.args.freeze_json)
        s3_problems = []
        if self.args.s3_freeze:
            _s3, s3_problems = verify_s3_freeze(self.args.s3_freeze)
        if problems or s3_problems:
            raise SystemExit("resume freeze gate refused: %s"
                             % (problems + s3_problems))
        # Refuse BEFORE taking the lock when the unit is already scored or
        # the dependency is not satisfied.
        if stage == "l" and (self.root / "campaign" / "l.json").is_file():
            raise SystemExit("REFUSING: l.json already scored "
                             "(repeating a scored unit is forbidden)")
        if stage == "o6":
            import freespace_verdict
            status = freespace_verdict._row_formal(self.root, "o5")[0]
            if status != "pass":
                raise SystemExit("REFUSING: O5 row is %s, not pass -- the "
                                 "O6 dependency is not satisfied" % status)
            if any((self.root / "o6").glob("*.json")):
                raise SystemExit("REFUSING: o6 artifacts already scored")
        # Merge the EXISTING manifest: a resume never rewrites the run's
        # history (stage records, measurement time, freeze binding, input
        # manifest SHA stay exactly as the first launch recorded them).
        existing = _load_json(self.root / "run_manifest.json") or {}
        if existing.get("run_id") != self.manifest["run_id"]:
            raise SystemExit("REFUSING: existing run root belongs to run_id "
                             "%s, not %s" % (existing.get("run_id"),
                                             self.manifest["run_id"]))
        self.manifest.update({
            key: value for key, value in existing.items()
            if key not in ("stages", "argv")})
        self.manifest["stages"] = dict(existing.get("stages") or {})
        self._resume_stage_marker = stage
        self.acquire_lock()
        try:
            prior_s = float(self.manifest.get("measurement_s") or 0.0)
            self.manifest["resumed_at"] = _iso()
            self.manifest["resume_stage"] = stage
            self.manifest["prior_measurement_s"] = prior_s
            self.manifest["resume_argv"] = list(sys.argv)
            self.save_manifest()

            def remaining_min():
                return self.args.budget_minutes - prior_s / 60.0 \
                    - (time.monotonic() - self.started_mono) / 60.0

            if stage == "l":
                summary = self.root / "campaign" / "summary.json"
                if summary.is_file():
                    backup = self.root / "campaign" / "summary_pre_resume.json"
                    backup.write_bytes(summary.read_bytes())
                argv = [
                    sys.executable, Path(self.args.worktree) / CAMPAIGN_REL,
                    "--node", self.args.node, "--eval-native", self.args.eval_native,
                    "--frames-glob",
                    str(Path(self.args.frames_dir) / "b*.yaml"),
                    "--out", str(self.root / "campaign"), "--stages", "l",
                    "--formal", "--budget-minutes", "%.1f" % max(1.0, remaining_min()),
                ]
                self.run_stage("campaign_resume_l", argv,
                               estimate_min=None, timeout_s=150 * 60)
            else:  # o6
                o5_state = self.manifest["stages"].get("o5") or {}
                per_group_min = float(o5_state.get("duration_s") or 0.0) / 60.0 / 4.0
                estimate = per_group_min * 96.0 * FORMAL_ESTIMATE_MIN["o6_margin"]
                if remaining_min() < estimate:
                    self.skip_stage("o6_resume", "budget_insufficient_estimate_"
                                    "%.0fmin_remaining_%.0fmin"
                                    % (estimate, max(0.0, remaining_min())))
                    self.finish_resume()
                    return
                argv = self.executor_argv("o6") + [
                    "--o5-evidence", str(self.root / "o5" / "verdict.json")]
                argv[argv.index("--budget-minutes") + 1] = "%.1f" % max(
                    1.0, remaining_min())
                self.run_stage("o6_resume", argv, estimate_min=None)
            self.finish_resume()
        finally:
            self.release_lock()

    def finish_resume(self):
        self.run_stage("judge", self.judge_argv(),
                       estimate_min=FORMAL_ESTIMATE_MIN["judge"])
        self.manifest["completed_at"] = _iso()
        self.manifest["measurement_s"] = round(
            float(self.manifest.get("prior_measurement_s") or 0.0)
            + (time.monotonic() - self.started_mono), 1)
        self.save_manifest()

    def run(self):
        self.acquire_lock()
        try:
            # 1) The P/H -> R -> L chain. The runner's own scheduler stops R/L
            #    after a failed P and enforces the passed budget between
            #    stages; the 120-minute soak is never cut.
            self.run_stage("campaign", self.campaign_argv(),
                           estimate_min=FORMAL_ESTIMATE_MIN["campaign"],
                           env_overrides={} if self.args.rehearsal
                           else {"OCC_FS_SCORE_OPEN": "1"})
            # 2) O8 is independent of P/R/L outcomes (safe stage).
            self.run_stage("o8", self.executor_argv("o8"),
                           env_overrides=STAGE_ENV, 
                           estimate_min=FORMAL_ESTIMATE_MIN["o8"])
            # 3) O5 (safe stage).
            self.run_stage("o5", self.executor_argv("o5"),
                           env_overrides=STAGE_ENV,
                           estimate_min=FORMAL_ESTIMATE_MIN["o5"])
            # 4) O6 depends on O5: a failed O5 stops O6 here. The O6 estimate
            #    is the measured O5 per-group rate x 96 groups x margin.
            estimate, skip_reason = self.o6_decision()
            if skip_reason is not None:
                self.skip_stage("o6", skip_reason)
            else:
                self.run_stage("o6", self.executor_argv("o6"),
                               env_overrides=STAGE_ENV,
                               estimate_min=estimate)
            # 5) Judge: recompute the FIXED 20-row verdict from the files and
            #    write verdict.json in this run root. Its exit code reflects
            #    failing rows -- never laundered.
            self.run_stage("judge", self.judge_argv(),
                           estimate_min=FORMAL_ESTIMATE_MIN["judge"])
        finally:
            self.manifest["completed_at"] = _iso()
            self.manifest["measurement_s"] = round(
                time.monotonic() - self.started_mono, 1)
            self.save_manifest()
            self.release_lock()
        judge_state = self.manifest["stages"].get("judge") or {}
        return 0 if judge_state.get("exit_code") == 0 else 1


CAMPAIGN_REL = "src/uld_occ_observer/test/benchmark/freespace_campaign.py"
EXECUTOR_REL = "src/uld_occ_observer/test/benchmark/campaign_executor.py"
VERDICT_REL = "src/uld_occ_observer/test/benchmark/freespace_verdict.py"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True,
                        help="the scoring run root (must be empty or new)")
    parser.add_argument("--worktree", required=True)
    parser.add_argument("--qualification-root", required=True,
                        help="the gen4 qualification evidence root "
                             "(f_matrix/, rehearsal/, o9/, FREEZE.json)")
    parser.add_argument("--freeze-json", required=True,
                        help="FREEZE.json of the bound candidate")
    parser.add_argument("--s3-freeze", default=None,
                        help="FREEZE_S3.json binding the S3 scoring tools, "
                             "inputs and commands (formal runs require it)")
    parser.add_argument("--node", required=True)
    parser.add_argument("--cli", required=True)
    parser.add_argument("--eval-native", required=True)
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--frames-dir", required=True,
                        help="directory with the frozen b*.yaml frame files")
    parser.add_argument("--budget-minutes", type=float, default=480.0)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--rehearsal", action="store_true",
                        help="bounded same-source wiring run: tiny counts, "
                             "never a formal sample, judges must NOT certify")
    parser.add_argument("--resume-stage", choices=("l", "o6"), default=None,
                        help="resume one contract-order stage into an "
                             "existing run root (never repeats a scored unit)")
    args = parser.parse_args(argv)
    if not args.run_id:
        args.run_id = ("rehearsal_%s" % time.strftime("%Y%m%d_%H%M%S")
                       if args.rehearsal else "run01")
    orchestrator = Orchestrator(args)
    if args.resume_stage:
        return orchestrator.resume(args.resume_stage)
    orchestrator.prepare()
    return orchestrator.run()


if __name__ == "__main__":
    raise SystemExit(main())
