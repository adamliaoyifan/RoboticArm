#!/usr/bin/env python3
"""PF-R7 generation-3 bounded campaign harness. Eval-only, ROS-free.

State machine:
``PRECHECK -> PRIME -> SPAWN -> OBSERVE -> CLASSIFY -> FLUSH -> CASE_RESET``
with ``STACK_RESET`` and terminal ``PASS`` / ``FAIL`` / ``INCONCLUSIVE``.

A live ROS backend is injected by ``scripts/pf_r7_bounded_acceptance.py``.
Unit tests use ``ScriptedBackend``. This module never launches a second
Gazebo world; occupancy is checked before a live spawn.
"""

from __future__ import division

import json
import os
import signal
import subprocess
import time
from collections import defaultdict

from luggage_perception.eval.pf_r7_classifier import (
    CLASS_ELIGIBLE_FAIL,
    CLASS_ELIGIBLE_PASS,
    CLASS_INFRA,
    CLASS_KNOWN_MISS,
    SCAN_EVIDENCE,
    SCAN_PROPOSAL_AVAILABLE,
    classify_attempt,
    is_exclusion,
    scan_availability_class,
    stops_campaign,
)
from luggage_perception.eval.sim_texture import CATALOG_SIZES

SIZES = CATALOG_SIZES

STATE_PRECHECK = "PRECHECK"
STATE_PRIME = "PRIME"
STATE_SPAWN = "SPAWN"
STATE_OBSERVE = "OBSERVE"
STATE_CLASSIFY = "CLASSIFY"
STATE_FLUSH = "FLUSH"
STATE_CASE_RESET = "CASE_RESET"
STATE_STACK_RESET = "STACK_RESET"
STATE_PASS = "PASS"
STATE_FAIL = "FAIL"
STATE_INCONCLUSIVE = "INCONCLUSIVE"

DEFAULT_DEADLINES = {
    "precheck_sec": 60.0,
    "proposal_sec": 1.4,
    "observe_sec": 8.0,
    "case_reset_sec": 20.0,
    "slot_sec": 600.0,
    "no_progress_sec": 5.0,
    "campaign_sec": 2700.0,
}

DEFAULT_BUDGETS = {
    "slots": 3,
    "eligible_per_size": 2,
    "max_attempts_per_slot": 12,
    "max_exclusions_per_slot": 6,
    "max_consecutive_size_exclusions": 3,
    "max_stack_resets": 2,
    "max_attempts_campaign": 36,
}


def default_seed_matrix():
    """Predeclared XY/yaw seeds. Substitutes take the next same-size seed."""
    offsets = (
        (0.00, 0.00), (0.05, 0.00), (-0.05, 0.00), (0.00, 0.05),
        (0.00, -0.05), (0.04, 0.04), (-0.04, 0.04), (0.04, -0.04),
        (-0.04, -0.04), (0.08, 0.00), (-0.08, 0.00), (0.00, 0.08),
        (0.06, 0.02), (-0.06, -0.02), (0.02, 0.06), (-0.02, -0.06),
    )
    yaws = (
        0.00, 0.20, -0.20, 0.40, -0.40, 0.10, -0.10, 0.30,
        -0.30, 0.15, -0.15, 0.05, -0.05, 0.25, -0.25, 0.00,
    )
    matrix = {}
    for size in SIZES:
        seeds = []
        for index, (xy, yaw) in enumerate(zip(offsets, yaws)):
            seeds.append({
                "seed_id": "%s_%02d" % (size, index),
                "size": size,
                "xy": [float(xy[0]), float(xy[1])],
                "yaw": float(yaw),
            })
        matrix[size] = seeds
    return matrix


G4_IMPORTED_STANDARD = (
    ("standard_00", SCAN_PROPOSAL_AVAILABLE),
    ("standard_01", SCAN_PROPOSAL_AVAILABLE),
    ("standard_02", SCAN_PROPOSAL_AVAILABLE),
    ("standard_03", "known_detector_miss_valid"),
    ("standard_04", "known_detector_miss_valid"),
    ("standard_05", "known_detector_miss_valid"),
)


def g4_imported_standard_rows():
    """Import G4 standard_00-05 classifications; do not rescan them."""
    by_id = {
        seed["seed_id"]: seed
        for seed in default_seed_matrix()["standard"]
    }
    rows = []
    for seed_id, scan_class in G4_IMPORTED_STANDARD:
        seed = by_id[seed_id]
        rows.append({
            "seed_id": seed_id,
            "size": "standard",
            "xy": list(seed["xy"]),
            "yaw": float(seed["yaw"]),
            "scan_class": scan_class,
            "imported": True,
            "source": "g4_c086a39",
        })
    return rows


def load_seed_matrix_json(path, base=None):
    """Replace named sizes from JSON; unspecified sizes keep the G4 matrix."""
    payload = json.loads(open(path, "r", encoding="utf-8").read())
    return merge_seed_matrix(payload, base=base)


def merge_seed_matrix(override, base=None):
    matrix = default_seed_matrix() if base is None else {
        size: [dict(seed) for seed in seeds]
        for size, seeds in base.items()
    }
    if not override:
        return matrix
    for size in SIZES:
        if size not in override:
            continue
        matrix[size] = [dict(seed) for seed in override[size]]
    return matrix


def live_seed_matrix_from_available(available_standard):
    matrix = default_seed_matrix()
    matrix["standard"] = [{
        "seed_id": row["seed_id"],
        "size": "standard",
        "xy": list(row["xy"]),
        "yaw": float(row["yaw"]),
    } for row in available_standard]
    return matrix


def slot_size_plan(eligible_per_size=2):
    plan = []
    for _ in range(int(eligible_per_size)):
        plan.extend(SIZES)
    return tuple(plan)


def atomic_write_text(path, text):
    path = os.path.abspath(path)
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write(text)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)
    return path


def atomic_write_json(path, payload):
    text = json.dumps(payload, indent=2, sort_keys=True, default=str)
    if not text.endswith("\n"):
        text += "\n"
    return atomic_write_text(path, text)


def append_jsonl(path, payload):
    path = os.path.abspath(path)
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    line = json.dumps(payload, sort_keys=True, default=str) + "\n"
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(line)
        handle.flush()
        os.fsync(handle.fileno())
    return path


def gazebo_launch_running():
    try:
        proc = subprocess.run(
            ["pgrep", "-af",
             "/usr/bin/python3 /opt/ros/humble/bin/ros2 launch luggage_gazebo"],
            capture_output=True, text=True, check=False)
        lines = [line for line in (proc.stdout or "").splitlines()
                 if "launch luggage_gazebo" in line
                 and "pgrep" not in line]
        return bool(lines)
    except OSError:
        return False


def pidfile_live(pidfile):
    try:
        if not os.path.isfile(pidfile):
            return False
        text = open(pidfile, "r", encoding="utf-8").read().strip()
        if not text:
            return False
        pid = int(text.split()[0])
        os.kill(pid, 0)
        return True
    except (OSError, ValueError):
        return False


def sim_slot_busy(pidfile="/tmp/elfin_humble_sim.pid"):
    """True when a luggage_gazebo launch or live pidfile already owns the GPU."""
    return gazebo_launch_running() or pidfile_live(pidfile)


class FakeClock(object):
    def __init__(self, start=0.0):
        self.t = float(start)

    def monotonic(self):
        return self.t

    def sleep(self, dt):
        self.t += float(dt)


class WallClock(object):
    def monotonic(self):
        return time.monotonic()

    def sleep(self, dt):
        time.sleep(max(0.0, float(dt)))


class ScriptedBackend(object):
    """Deterministic observe/reset backend for harness tests."""

    def __init__(self, records_by_size=None, record_list=None,
                 hang_observe=False, snapshot=None, flush_writes=True,
                 stale_after_reset=0, stack_reset_ok=True,
                 occupied=False, case_reset_hang=False):
        self.records_by_size = {
            size: list(items)
            for size, items in (records_by_size or {}).items()
        }
        self.record_list = list(record_list or [])
        self.hang_observe = bool(hang_observe)
        self.snapshot_record = snapshot
        self.flush_writes = bool(flush_writes)
        self.stale_after_reset = int(stale_after_reset)
        self.stack_reset_ok = bool(stack_reset_ok)
        self.occupied = bool(occupied)
        self.case_reset_hang = bool(case_reset_hang)
        self.epoch = 0
        self.stop_sim_calls = 0
        self.stack_reset_calls = 0
        self.case_reset_calls = 0
        self.teardown_calls = 0
        self.deleted = []
        self.spawned = []
        self.cancelled = []
        self.flushed = []
        self.states = []

    def sim_busy(self):
        return self.occupied

    def precheck(self, case):
        return {"ok": True, "n_clock_publishers": 1}

    def prime(self, case):
        return {"ok": True, "epoch": self.epoch}

    def spawn(self, case):
        self.spawned.append(dict(case))
        return {"ok": True, "case": dict(case)}

    def poll_observe(self, case):
        if self.hang_observe:
            return None
        record = self._next_record(case)
        return {"record": record, "progress": True}

    def snapshot(self, case):
        if self.snapshot_record is not None:
            payload = dict(self.snapshot_record)
            payload.setdefault("size", case.get("size"))
            payload.setdefault("seed_id", case.get("seed_id"))
            return payload
        if self.record_list or any(self.records_by_size.values()):
            return self._next_record(case)
        return None

    def cancel_requests(self, case):
        self.cancelled.append(dict(case))
        return {"ok": True}

    def flush(self, attempt_dir, record, classified):
        self.flushed.append({
            "attempt_dir": attempt_dir,
            "class": (classified or {}).get("attempt_class"),
        })
        if not self.flush_writes:
            return {
                "capture_complete": False,
                "replay_possible": False,
                "missing": ["flush_skipped"],
            }
        os.makedirs(attempt_dir, exist_ok=True)
        health = dict(record.get("dump_health") or {
            "capture_complete": True,
            "replay_possible": True,
            "missing": [],
        })
        return health

    def case_reset(self, case):
        if self.case_reset_hang:
            return {"ok": False, "timeout": True, "epoch": self.epoch}
        self.case_reset_calls += 1
        self.deleted.append(case.get("seed_id"))
        self.epoch += 1
        return {
            "ok": True,
            "epoch": self.epoch,
            "box_absent": True,
            "stale_frames": self.stale_after_reset,
        }

    def stack_reset(self):
        self.stack_reset_calls += 1
        self.stop_sim_calls += 1
        self.epoch += 1
        if not self.stack_reset_ok:
            return {"ok": False, "n_clock_publishers": 0, "residuals": 1}
        return {
            "ok": True,
            "n_clock_publishers": 1,
            "residuals": 0,
            "pidfile": False,
        }

    def teardown(self):
        self.teardown_calls += 1
        self.stop_sim_calls += 1
        return {"residuals": 0, "stop_sim": True}

    def _next_record(self, case):
        size = case.get("size")
        queue = self.records_by_size.get(size) or []
        if queue:
            record = dict(queue.pop(0))
        elif self.record_list:
            record = dict(self.record_list.pop(0))
        else:
            raise RuntimeError("scripted backend exhausted for size %s" % size)
        record.setdefault("size", size)
        record.setdefault("seed_id", case.get("seed_id"))
        return record


class CampaignDriver(object):
    def __init__(self, out_dir, backend, budgets=None, deadlines=None,
                 seed_matrix=None, clock=None, poll_dt=0.05,
                 git_commit="", stop_sim_fn=None, install_signals=False):
        self.out_dir = os.path.abspath(out_dir)
        os.makedirs(self.out_dir, exist_ok=True)
        self.backend = backend
        self.budgets = dict(DEFAULT_BUDGETS)
        self.budgets.update(budgets or {})
        self.deadlines = dict(DEFAULT_DEADLINES)
        self.deadlines.update(deadlines or {})
        self.seed_matrix = seed_matrix or default_seed_matrix()
        self.clock = clock or WallClock()
        self.poll_dt = float(poll_dt)
        self.git_commit = git_commit or ""
        self.stop_sim_fn = stop_sim_fn
        self.state = STATE_PRECHECK
        self.abort = False
        self.stack_resets = 0
        self.campaign_attempts = 0
        self.seed_index = {size: 0 for size in SIZES}
        self.attempts_path = os.path.join(self.out_dir, "attempts.jsonl")
        self.miss_path = os.path.join(self.out_dir, "miss_ledger.jsonl")
        self.reset_path = os.path.join(self.out_dir, "reset_ledger.jsonl")
        self.verdict_path = os.path.join(self.out_dir, "verdict.json")
        self.state_log = []
        if install_signals:
            self._install_signals()

    def request_abort(self, *_args):
        self.abort = True

    def _install_signals(self):
        signal.signal(signal.SIGINT, self.request_abort)
        signal.signal(signal.SIGTERM, self.request_abort)

    def _transition(self, state):
        self.state = state
        self.state_log.append({
            "state": state,
            "t": self.clock.monotonic(),
        })

    def _next_seed(self, size):
        seeds = self.seed_matrix.get(size) or []
        index = self.seed_index[size]
        if index >= len(seeds):
            return None
        self.seed_index[size] = index + 1
        return dict(seeds[index])

    def run(self):
        t_campaign = self.clock.monotonic()
        atomic_write_json(os.path.join(self.out_dir, "campaign_config.json"), {
            "budgets": self.budgets,
            "deadlines": self.deadlines,
            "git_commit": self.git_commit,
            "seed_matrix": self.seed_matrix,
        })
        if getattr(self.backend, "sim_busy", lambda: False)():
            verdict = self._finish(
                STATE_INCONCLUSIVE,
                "inconclusive/sim_slot_busy",
                teardown=False)
            return verdict

        slots_passed = []
        reason = None
        outcome_state = STATE_PASS
        for slot in range(1, int(self.budgets["slots"]) + 1):
            result = self._run_slot(slot, t_campaign)
            if result.get("outcome") == CLASS_ELIGIBLE_FAIL or result.get(
                    "state") == STATE_FAIL:
                outcome_state = STATE_FAIL
                reason = result.get("reason") or "eligible_fail"
                break
            if result.get("state") == STATE_INCONCLUSIVE:
                outcome_state = STATE_INCONCLUSIVE
                reason = result.get("reason")
                break
            if not result.get("slot_pass"):
                outcome_state = STATE_INCONCLUSIVE
                reason = result.get("reason") or "slot_incomplete"
                break
            slots_passed.append(slot)
        else:
            reason = "slots_passed"

        if outcome_state == STATE_PASS and len(slots_passed) < int(
                self.budgets["slots"]):
            outcome_state = STATE_INCONCLUSIVE
            reason = reason or "slots_incomplete"
        return self._finish(outcome_state, reason)

    def run_standard_scan(self, imported=None, scan_start="standard_06",
                          stop_available=6, max_scan=24):
        """Detector-availability scan. Does not score the G4 geometry window."""
        os.makedirs(self.out_dir, exist_ok=True)
        imported_rows = [dict(row) for row in (imported or [])]
        atomic_write_json(
            os.path.join(self.out_dir, "imported_g4.json"),
            {"seeds": imported_rows})
        atomic_write_json(os.path.join(self.out_dir, "scan_config.json"), {
            "scan_start": scan_start,
            "stop_available": int(stop_available),
            "max_scan": int(max_scan),
            "git_commit": self.git_commit,
        })
        if getattr(self.backend, "sim_busy", lambda: False)():
            verdict = {
                "outcome": "inconclusive",
                "reason": "inconclusive/sim_slot_busy",
                "n_available": 0,
                "n_scanned": 0,
            }
            atomic_write_json(os.path.join(self.out_dir, "scan_verdict.json"),
                              verdict)
            return verdict

        rows = list(imported_rows)
        available = [
            row for row in rows
            if row.get("scan_class") == SCAN_PROPOSAL_AVAILABLE
        ]
        counted = len(rows)
        standard_seeds = list(
            (self.seed_matrix.get("standard")
             or default_seed_matrix()["standard"]))
        start_index = 0
        for index, seed in enumerate(standard_seeds):
            if seed.get("seed_id") == scan_start:
                start_index = index
                break
        else:
            if scan_start:
                start_index = len(standard_seeds)

        defect = None
        scanned = []
        slot_state = {
            "attempts": 0,
            "exclusions": 0,
            "eligible": {size: 0 for size in SIZES},
            "consecutive_excl": {size: 0 for size in SIZES},
        }
        for seed in standard_seeds[start_index:]:
            if len(available) >= int(stop_available):
                break
            if counted >= int(max_scan):
                break
            case = dict(seed)
            case["slot"] = 0
            case["attempt"] = counted + 1
            attempt = self._run_attempt(case, slot_state)
            record = attempt.get("record") or {}
            classified = attempt.get("classified") or classify_attempt(record)
            scan_class, classified = scan_availability_class(
                record, classified)
            row = {
                "seed_id": case.get("seed_id"),
                "size": "standard",
                "xy": list(case.get("xy") or []),
                "yaw": case.get("yaw"),
                "scan_class": scan_class,
                "imported": False,
                "attempt": self.campaign_attempts,
                "attempt_class": classified.get("attempt_class"),
                "reasons": classified.get("reasons") or [],
                "dump": attempt.get("attempt_dir"),
            }
            rows.append(row)
            scanned.append(row)
            counted += 1
            if scan_class == SCAN_PROPOSAL_AVAILABLE:
                available.append(row)
            if scan_class == SCAN_EVIDENCE:
                defect = row
                break
            if attempt.get("needs_stack_reset"):
                reset = self._stack_reset(case, classified.get("attempt_class"))
                if not reset.get("ok"):
                    defect = dict(row, scan_class=SCAN_EVIDENCE,
                                  reasons=["stack_reset_failed"])
                    row["scan_class"] = SCAN_EVIDENCE
                    break

        live_matrix = live_seed_matrix_from_available(
            available[:int(stop_available)])
        atomic_write_json(
            os.path.join(self.out_dir, "available_standard_seeds.json"),
            {"seeds": available[:int(stop_available)]})
        atomic_write_json(
            os.path.join(self.out_dir, "live_seed_matrix.json"), live_matrix)
        atomic_write_json(
            os.path.join(self.out_dir, "scan_ledger.json"), {"seeds": rows})
        n_available = len(available)
        if defect is not None:
            reason = "evidence_invalid"
            outcome = "fail"
        elif n_available >= int(stop_available):
            reason = "six_proposal_available"
            outcome = "pass"
        else:
            reason = "inconclusive/perception_availability_blocked"
            outcome = "inconclusive"
        verdict = {
            "outcome": outcome,
            "reason": reason,
            "n_available": n_available,
            "n_imported": len(imported_rows),
            "n_scanned": len(scanned),
            "n_counted": counted,
            "stop_available": int(stop_available),
            "max_scan": int(max_scan),
            "git_commit": self.git_commit,
            "defect_seed": None if defect is None else defect.get("seed_id"),
            "available_seed_ids": [
                row["seed_id"] for row in available[:int(stop_available)]
            ],
        }
        residuals = 0
        result = self.backend.teardown() or {}
        residuals = int(result.get("residuals") or 0)
        if self.stop_sim_fn:
            self.stop_sim_fn()
        verdict["residuals"] = residuals
        atomic_write_json(
            os.path.join(self.out_dir, "scan_verdict.json"), verdict)
        return verdict

    def _budget_reason(self, slot_state, size, t_campaign, t_slot):
        now = self.clock.monotonic()
        if now - t_campaign >= float(self.deadlines["campaign_sec"]):
            return "inconclusive/time_budget"
        if now - t_slot >= float(self.deadlines["slot_sec"]):
            return "inconclusive/time_budget"
        if self.campaign_attempts >= int(self.budgets["max_attempts_campaign"]):
            return "inconclusive/perception_availability_blocked"
        if slot_state["attempts"] >= int(self.budgets["max_attempts_per_slot"]):
            return "inconclusive/perception_availability_blocked"
        if slot_state["exclusions"] >= int(
                self.budgets["max_exclusions_per_slot"]):
            return "inconclusive/perception_availability_blocked"
        if slot_state["consecutive_excl"][size] >= int(
                self.budgets["max_consecutive_size_exclusions"]):
            return "inconclusive/perception_availability_blocked"
        if self.stack_resets > int(self.budgets["max_stack_resets"]):
            return "inconclusive/infrastructure_blocked"
        return None

    def _run_slot(self, slot, t_campaign):
        t_slot = self.clock.monotonic()
        plan = slot_size_plan(self.budgets["eligible_per_size"])
        plan_index = 0
        slot_state = {
            "slot": slot,
            "attempts": 0,
            "exclusions": 0,
            "eligible": {size: 0 for size in SIZES},
            "consecutive_excl": {size: 0 for size in SIZES},
            "eligible_records": [],
        }
        while plan_index < len(plan):
            if self.abort:
                return {
                    "state": STATE_INCONCLUSIVE,
                    "reason": "inconclusive/signal_abort",
                    "slot_pass": False,
                }
            size = plan[plan_index]
            budget = self._budget_reason(slot_state, size, t_campaign, t_slot)
            if budget:
                return {
                    "state": STATE_INCONCLUSIVE,
                    "reason": budget,
                    "slot_pass": False,
                }
            case = self._next_seed(size)
            if case is None:
                return {
                    "state": STATE_INCONCLUSIVE,
                    "reason": "inconclusive/seed_exhausted",
                    "slot_pass": False,
                }
            case["slot"] = slot
            case["attempt"] = self.campaign_attempts + 1
            attempt = self._run_attempt(case, slot_state)
            classified = attempt["classified"]
            attempt_class = classified["attempt_class"]
            if self.abort:
                return {
                    "state": STATE_INCONCLUSIVE,
                    "reason": "inconclusive/signal_abort",
                    "slot_pass": False,
                }
            if attempt.get("needs_stack_reset"):
                if self.stack_resets >= int(self.budgets["max_stack_resets"]):
                    return {
                        "state": STATE_INCONCLUSIVE,
                        "reason": "inconclusive/infrastructure_blocked",
                        "slot_pass": False,
                    }
                reset = self._stack_reset(case, attempt_class)
                if not reset.get("ok"):
                    return {
                        "state": STATE_INCONCLUSIVE,
                        "reason": "inconclusive/infrastructure_blocked",
                        "slot_pass": False,
                    }
            if stops_campaign(attempt_class):
                return {
                    "state": STATE_FAIL,
                    "reason": "eligible_fail",
                    "outcome": CLASS_ELIGIBLE_FAIL,
                    "slot_pass": False,
                    "classified": classified,
                }
            if attempt_class == CLASS_ELIGIBLE_PASS:
                slot_state["eligible"][size] += 1
                slot_state["consecutive_excl"][size] = 0
                slot_state["eligible_records"].append(attempt)
                plan_index += 1
            elif is_exclusion(attempt_class):
                slot_state["exclusions"] += 1
                slot_state["consecutive_excl"][size] += 1
            if slot_state["consecutive_excl"][size] >= int(
                    self.budgets["max_consecutive_size_exclusions"]):
                return {
                    "state": STATE_INCONCLUSIVE,
                    "reason": "inconclusive/perception_availability_blocked",
                    "slot_pass": False,
                }
        summary = {
            "slot": slot,
            "slot_pass": all(
                slot_state["eligible"][size]
                >= int(self.budgets["eligible_per_size"])
                for size in SIZES),
            "eligible": dict(slot_state["eligible"]),
            "attempts": slot_state["attempts"],
            "exclusions": slot_state["exclusions"],
            "state": STATE_PASS,
        }
        atomic_write_json(
            os.path.join(self.out_dir, "slot%d" % slot, "summary.json"),
            summary)
        return summary

    def _run_attempt(self, case, slot_state):
        self.campaign_attempts += 1
        slot_state["attempts"] += 1
        t0 = self.clock.monotonic()
        needs_stack_reset = False
        record = None
        classified = None
        attempt_dir = os.path.join(
            self.out_dir, "slot%d" % case["slot"],
            "attempt%02d" % self.campaign_attempts)
        os.makedirs(attempt_dir, exist_ok=True)

        self._transition(STATE_PRECHECK)
        pre = self._wait_step(
            lambda: self.backend.precheck(case),
            self.deadlines["precheck_sec"], "precheck_timeout")
        if not pre.get("ok") and not self.abort:
            record = {
                "infrastructure_invalid": True,
                "controller_unreachable": (
                    pre.get("reason") == "precheck_timeout"),
                "dump_health": {
                    "capture_complete": False,
                    "replay_possible": False,
                    "missing": [pre.get("reason") or "precheck"],
                },
            }
            needs_stack_reset = True
        if self.abort:
            classified = self._abort_class()
            record = record or {"abort": True}
            return self._close_attempt(
                case, record, classified, attempt_dir, t0, True)

        if record is None:
            self._transition(STATE_PRIME)
            self.backend.prime(case)

            self._transition(STATE_SPAWN)
            spawn = self.backend.spawn(case)
            if not spawn.get("ok"):
                record = {
                    "infrastructure_invalid": True,
                    "spawn_hang": True,
                    "dump_health": {"capture_complete": False,
                                    "replay_possible": False,
                                    "missing": ["spawn"]},
                }
                needs_stack_reset = True

        self._transition(STATE_OBSERVE)
        if record is None:
            record, _observe_infra = self._observe(case)

        if self.abort:
            classified = self._abort_class()
            self._flush(attempt_dir, record or {"abort": True}, classified)
            return self._close_attempt(
                case, record, classified, attempt_dir, t0, True)

        self._transition(STATE_CLASSIFY)
        record = record or {}
        record.setdefault("size", case.get("size"))
        record.setdefault("seed_id", case.get("seed_id"))
        classified = classify_attempt(record)

        self._transition(STATE_FLUSH)
        health = self._flush(attempt_dir, record, classified)
        record["dump_health"] = health
        if not self.abort:
            classified = classify_attempt(record)

        if classified.get("attempt_class") == CLASS_INFRA:
            infra_reasons = classified.get("reasons") or []
            if any(item.startswith("clock") or "controller" in item
                   or "spawn_hang" in item or "duplicate_clock" in item
                   or "node_crashed" in item or "rate" in item
                   for item in infra_reasons):
                needs_stack_reset = True

        self._transition(STATE_CASE_RESET)
        reset = self._case_reset(case)
        if not reset.get("ok") or reset.get("timeout"):
            needs_stack_reset = True
            classified = classify_attempt(dict(record, case_reset_timeout=True))
        if int(reset.get("stale_frames") or 0) > 0:
            needs_stack_reset = True

        row = self._attempt_row(
            case, classified, record, attempt_dir, t0)
        append_jsonl(self.attempts_path, row)
        if classified.get("attempt_class") == CLASS_KNOWN_MISS:
            append_jsonl(self.miss_path, {
                "attempt": self.campaign_attempts,
                "slot": case["slot"],
                "size": case["size"],
                "seed_id": case["seed_id"],
                "xy": case.get("xy"),
                "yaw": case.get("yaw"),
                "best_confidence": classified.get("best_confidence"),
                "iou": classified.get("best_iou"),
                "dump": attempt_dir,
                "reasons": classified.get("reasons") or [],
            })
        return {
            "classified": classified,
            "record": record,
            "attempt_dir": attempt_dir,
            "needs_stack_reset": needs_stack_reset,
            "row": row,
        }

    def _observe(self, case):
        t0 = self.clock.monotonic()
        last_progress = t0
        while True:
            if self.abort:
                snap = self.backend.snapshot(case) or {"abort": True}
                return snap, True
            now = self.clock.monotonic()
            if now - t0 >= float(self.deadlines["observe_sec"]):
                snap = self.backend.snapshot(case) or {
                    "infrastructure_invalid": True,
                    "dump_health": {
                        "capture_complete": False,
                        "replay_possible": False,
                        "missing": ["observe_timeout"],
                    },
                }
                return snap, False
            if now - last_progress >= float(self.deadlines["no_progress_sec"]):
                snap = self.backend.snapshot(case) or {
                    "infrastructure_invalid": True,
                    "dump_health": {
                        "capture_complete": False,
                        "replay_possible": False,
                        "missing": ["no_progress"],
                    },
                }
                return snap, False
            event = self.backend.poll_observe(case)
            if event is None:
                self.clock.sleep(self.poll_dt)
                continue
            if event.get("progress"):
                last_progress = self.clock.monotonic()
            if event.get("record") is not None:
                return event["record"], False
            self.clock.sleep(self.poll_dt)

    def _wait_step(self, fn, timeout, reason):
        t0 = self.clock.monotonic()
        while True:
            if self.abort:
                return {"ok": False, "reason": "signal_abort"}
            result = fn()
            if result and result.get("ok"):
                return result
            if self.clock.monotonic() - t0 >= float(timeout):
                return {"ok": False, "reason": reason}
            self.clock.sleep(self.poll_dt)

    def _flush(self, attempt_dir, record, classified):
        os.makedirs(attempt_dir, exist_ok=True)
        manifest = {
            "git_commit": self.git_commit,
            "state": self.state,
            "record_keys": sorted(str(k) for k in (record or {})),
            "classified": classified,
            "capture_complete": bool(
                ((record or {}).get("dump_health") or {}).get(
                    "capture_complete")),
            "replay_possible": bool(
                ((record or {}).get("dump_health") or {}).get(
                    "replay_possible")),
        }
        atomic_write_json(
            os.path.join(attempt_dir, "attempt_manifest.json"), manifest)
        health = self.backend.flush(attempt_dir, record or {}, classified)
        health = health or {}
        atomic_write_json(os.path.join(attempt_dir, "dump_health.json"), health)
        return health

    def _case_reset(self, case):
        t0 = self.clock.monotonic()
        self.backend.cancel_requests(case)
        reset = self.backend.case_reset(case)
        dt = self.clock.monotonic() - t0
        timeout = dt > float(self.deadlines["case_reset_sec"]) or reset.get(
            "timeout")
        payload = dict(reset or {})
        payload["kind"] = "case"
        payload["attempt"] = self.campaign_attempts
        payload["size"] = case.get("size")
        payload["timeout"] = bool(timeout)
        payload["dt_sec"] = dt
        append_jsonl(self.reset_path, payload)
        return payload

    def _stack_reset(self, case, attempt_class):
        self._transition(STATE_STACK_RESET)
        self.stack_resets += 1
        if self.stop_sim_fn:
            self.stop_sim_fn()
        reset = self.backend.stack_reset()
        payload = dict(reset or {})
        payload["kind"] = "stack"
        payload["attempt"] = self.campaign_attempts
        payload["attempt_class"] = attempt_class
        payload["stack_resets"] = self.stack_resets
        append_jsonl(self.reset_path, payload)
        return payload

    def _abort_class(self):
        return {
            "attempt_class": CLASS_INFRA,
            "reasons": ["signal_abort"],
            "scored": False,
            "advances_slot": False,
            "stops_campaign": False,
            "exclusion": True,
            "counts_as_identity_slot": False,
        }

    def _close_attempt(self, case, record, classified, attempt_dir, t0,
                       needs_stack_reset):
        self._transition(STATE_FLUSH)
        self._flush(attempt_dir, record or {}, classified)
        row = self._attempt_row(case, classified, record, attempt_dir, t0)
        append_jsonl(self.attempts_path, row)
        return {
            "classified": classified,
            "record": record,
            "attempt_dir": attempt_dir,
            "needs_stack_reset": needs_stack_reset,
            "row": row,
        }

    def _attempt_row(self, case, classified, record, attempt_dir, t0):
        return {
            "attempt": self.campaign_attempts,
            "slot": case.get("slot"),
            "size": case.get("size"),
            "seed_id": case.get("seed_id"),
            "xy": case.get("xy"),
            "yaw": case.get("yaw"),
            "attempt_class": classified.get("attempt_class"),
            "reasons": classified.get("reasons") or [],
            "scored": classified.get("scored"),
            "advances_slot": classified.get("advances_slot"),
            "stops_campaign": classified.get("stops_campaign"),
            "exclusion": classified.get("exclusion"),
            "dump": attempt_dir,
            "dt_sec": self.clock.monotonic() - t0,
            "git_commit": self.git_commit,
        }

    def _finish(self, state, reason, teardown=True):
        self._transition(state)
        residuals = 0
        if teardown:
            result = self.backend.teardown() or {}
            residuals = int(result.get("residuals") or 0)
            if self.stop_sim_fn:
                self.stop_sim_fn()
        outcome = {
            STATE_PASS: "pass",
            STATE_FAIL: "fail",
            STATE_INCONCLUSIVE: "inconclusive",
        }.get(state, "inconclusive")
        verdict = {
            "outcome": outcome,
            "reason": reason,
            "state": state,
            "n_attempts": self.campaign_attempts,
            "stack_resets": self.stack_resets,
            "residuals": residuals,
            "git_commit": self.git_commit,
            "abort": self.abort,
        }
        atomic_write_json(self.verdict_path, verdict)
        return verdict
