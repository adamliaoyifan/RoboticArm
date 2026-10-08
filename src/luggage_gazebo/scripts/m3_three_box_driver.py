#!/usr/bin/env python3
"""ST09-I S5 M3: three-box packing on the deployment scene (Gazebo).

Assembly on top of the MOTION-OCC production chain — no new planning,
certification, or gating policy. What this subclass adds:

* ``FinalizeCurrentBox`` after the placed box's integration succeeded,
  the map revision strictly grew, and return/recovery completed — so
  the PHYSICAL box survives into the next cycle (refusal stops the
  session; the next trial's ensure_clean would delete it otherwise).
* ``m3_evidence.json`` / ``m3_frozen.json`` in the unified N-box
  acceptance form: one case per box with the 11 EVENT_ORDER stages,
  RAW acquisition stamps (never rewritten) + receiver monotonic stamps
  + strictly increasing ``event_seq``, per-event ``input_map_revision``
  (and ``output_map_revision`` on the commit stages — events are never
  forced onto one disguised revision), unique ``<catalog>@<ordinal>``
  box ids, and revision_handoff rows joined to the committed chain.
  Rewritten ATOMICALLY after every box so a mid-run death stays
  judgeable.
* Dense Gazebo ground truth around the release window: a streaming
  ``ign topic -e pose/info`` subprocess drained by a reader thread
  into a bounded JSONL, started BEFORE vacuum-off with a positive-sample
  precondition (no post-hoc fabrication), segmented pre-release /
  respawn gap / post-release, with central-difference velocities over
  0.15 s windows and a stream-vs-poll cross-check. GT feeds
  ``gt_reference`` ONLY — the measured map always comes from the real
  sensor chain.

S5 scope: non-scored, complete/judgeable/replayable evidence. The
frozen hashes block is labeled ``recorded_inputs``; formal freezing is
S6.
"""

from __future__ import division

import argparse
import collections
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from motion_occ_eval_driver import (  # noqa: E402
    MotionOccDriver,
    parse_args as parse_occ_args,
    _atomic_json,
)
from place_smoke_driver import _ign_model_bin, _xyz3  # noqa: E402
from luggage_gazebo.place_metrics import (  # noqa: E402
    PlaceTrial,
    place_ok,
    summarize,
    trial_to_dict,
)
from luggage_msgs.srv import (  # noqa: E402
    FinalizeCurrentBox,
    GetCargoMapStats,
)
from luggage_gazebo.integration_acceptance import (  # noqa: E402
    COMMIT_STAGES,
    EVENT_ORDER,
    extract_acceptance,
)

DEFAULT_OUT = os.path.normpath(os.path.join(
    _SCRIPTS, "..", "..", "..", "docs", "status", "evidence",
    "frame_migration", "ST09-parallel-I-s5", "m3", "latest"))
DEFAULT_SEQUENCE = "carryon,standard,carryon"

#: Parent rehearsal-mark name -> M3 event stage.
_MARK_STAGES = {
    "observation": "observation",
    "placement_response": "slot",
    "integration": "measured_integration",
}

#: release-window GT constants (declared, recorded, not tuned silently).
GT_FRESH_S = 0.5             # newest sample must be this recent at detach
GT_PRE_ROLL_S = GT_FRESH_S   # kept name: freshness, not a late subscribe
GT_DURATION_S = 12.0         # analysis window around release, not process life
GT_SUBSCRIBE_S = 600.0       # ign echo cap; stop() ends it at settle
GT_POSITIVE_TIMEOUT_S = 2.0  # path check once the model id is known
GT_SPEED_WINDOW_S = 0.15     # central-difference window for velocities
GT_MIN_DT_S = 1e-3           # skip numerically degenerate sample gaps
GT_GAP_S = 0.10              # missing interval that marks the respawn gap
GT_POLL_TOL_M = 0.005        # stream vs poll cross-check bound

GT_GAPS = (
    "process_exited",
    "no_messages",
    "model_not_found",
    "parse_failed",
    "stale_sample",
)

_GZ_NAME = re.compile(r'name:\s*"([^"]+)"')
_GZ_FIELD = re.compile(
    r"(x|y|z|w):\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)")


def classify_gt_gap(record):
    """One gap name for an empty or unusable pre-release stream.

    Raw counters stay on the record. A dead process with no stdout is
    ``process_exited``. Silence from a live process is ``no_messages``.
    Parsed poses that never name the target are ``model_not_found``.
    A named target that does not become a sample is ``parse_failed``.
    ``stale_sample`` is applied by the release gate, not here.
    """
    if int(record.get("sample_count") or 0) > 0:
        return ""
    lines = int(record.get("stdout_lines") or 0)
    blocks = int(record.get("pose_blocks") or 0)
    names = list(record.get("seen_names") or [])
    model = str(record.get("model") or "")
    parses = int(record.get("parse_failures") or 0)
    exited = record.get("ign_returncode") is not None
    if exited and lines == 0 and blocks == 0:
        return "process_exited"
    if lines == 0 and blocks == 0:
        return "no_messages"
    if model and model in names:
        return "parse_failed"
    if parses > 0 and not names:
        return "parse_failed"
    if blocks > 0 or names:
        return "model_not_found"
    if exited:
        return "process_exited"
    return "no_messages"


class GtReleaseStream(object):
    """Streaming `ign topic -e pose/info` reader for one release window.

    A dedicated reader thread drains the subprocess pipe continuously
    (an undrained pipe fills and blocks ign), writes each target-model
    sample to a JSONL immediately, and keeps a bounded in-memory deque.
    Samples carry the message sim stamp AND the receiver monotonic
    stamp; a chunk without a header is flagged ``stamp_source:
    "receipt"``.
    """

    def __init__(self, model, jsonl_path, world="airport_loading",
                 duration_s=GT_SUBSCRIBE_S, max_samples=8000):
        self.model = str(model)
        self.path = str(jsonl_path)
        self._samples = collections.deque(maxlen=int(max_samples))
        self._lock = threading.Lock()
        self._pose_blocks = 0
        self._stdout_lines = 0
        self._parse_failures = 0
        self._unparsed_text = ""
        self._seen_names = []
        self._stderr_tail = ""
        self._file = open(self.path, "w", encoding="utf-8")
        self._proc = subprocess.Popen(
            [_ign_model_bin(), "topic", "-e", "-t",
             "/world/%s/pose/info" % world, "-d", str(int(duration_s))],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1)
        self._alive = True
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._err_thread = threading.Thread(
            target=self._read_stderr, daemon=True)
        self._thread.start()
        self._err_thread.start()

    # -- reader ---------------------------------------------------------

    def _read(self):
        depth = 0
        block = []
        stamp_ns = None
        sec = None
        for line in self._proc.stdout:
            if not self._alive:
                break
            with self._lock:
                self._stdout_lines += 1
            text = line.strip()
            if depth == 0:
                if text.startswith("pose"):
                    depth = text.count("{")
                    block = [text]
                    continue
                if text.startswith("sec:"):
                    try:
                        sec = int(text.split(":", 1)[1])
                    except ValueError:
                        sec = None
                elif text.startswith("nsec:") and sec is not None:
                    try:
                        stamp_ns = sec * 1_000_000_000 + int(
                            text.split(":", 1)[1])
                    except ValueError:
                        pass
                continue
            block.append(text)
            depth += text.count("{") - text.count("}")
            if depth <= 0:
                self._accept("\n".join(block), stamp_ns)
                depth = 0
                block = []

    def _read_stderr(self):
        chunks = []
        stream = self._proc.stderr
        if stream is None:
            return
        for line in stream:
            chunks.append(line)
            if sum(len(item) for item in chunks) > 4000:
                chunks = chunks[-20:]
        self._stderr_tail = "".join(chunks)[-2000:]

    def _accept(self, text, stamp_ns):
        named = _GZ_NAME.search(text)
        name = named.group(1) if named is not None else ""
        with self._lock:
            self._pose_blocks += 1
            if name and name not in self._seen_names and len(self._seen_names) < 16:
                self._seen_names.append(name)
        if not name or name != self.model:
            return
        pos_m = re.search(r"position\s*\{([^}]*)\}", text)
        ori_m = re.search(r"orientation\s*\{([^}]*)\}", text)
        # Ignition omits zero components. A missing axis is 0, not a
        # parse failure. Failure is a named pose with no position or
        # orientation group at all.
        if pos_m is None or ori_m is None:
            with self._lock:
                self._parse_failures += 1
                if not self._unparsed_text:
                    self._unparsed_text = text[:1500]
            return
        pos = dict(_GZ_FIELD.findall(pos_m.group(1)))
        ori = dict(_GZ_FIELD.findall(ori_m.group(1)))
        sample = {
            "sim_stamp_ns": stamp_ns,
            "recv_ns": time.monotonic_ns(),
            "stamp_source": "message_header" if stamp_ns is not None
            else "receipt",
            "xyz": [float(pos.get("x", 0.0)), float(pos.get("y", 0.0)),
                    float(pos.get("z", 0.0))],
            "quat_xyzw": [float(ori.get("x", 0.0)),
                          float(ori.get("y", 0.0)),
                          float(ori.get("z", 0.0)),
                          float(ori.get("w", 1.0))],
        }
        with self._lock:
            self._samples.append(sample)
        try:
            self._file.write(json.dumps(sample, sort_keys=True) + "\n")
        except (OSError, ValueError):
            pass

    # -- control --------------------------------------------------------

    def positive(self, timeout_s=GT_POSITIVE_TIMEOUT_S):
        """Wait until the target model has been parsed at least once.

        This confirms the subscription. It does not accept that sample
        as the pre-release observation.
        """
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            with self._lock:
                if self._samples:
                    return True
            time.sleep(0.02)
        with self._lock:
            return bool(self._samples)

    def latest_age_s(self):
        now = time.monotonic_ns()
        with self._lock:
            if not self._samples:
                return None
            latest = int(self._samples[-1]["recv_ns"])
        return (now - latest) / 1e9

    def release_gate(self, fresh_s=GT_FRESH_S):
        """A pre-release sample must be newer than ``fresh_s``.

        An earlier sample that only proved the path is not a substitute.
        Returns ``(ok, record)``. The record's ``gap`` is empty on success.
        """
        age = self.latest_age_s()
        record = self.failure_record()
        record["latest_age_s"] = age
        if age is not None and age <= float(fresh_s):
            record["positive"] = True
            record["gap"] = ""
            return True, record
        if age is not None:
            record["gap"] = "stale_sample"
            record["positive"] = False
        return False, record

    def failure_record(self):
        """Why a pre-release sample never arrived. Does not invent poses."""
        returncode = None
        proc = getattr(self, "_proc", None)
        if proc is not None and proc.poll() is not None:
            returncode = proc.returncode
        with self._lock:
            blocks = int(getattr(self, "_pose_blocks", 0))
            names = list(getattr(self, "_seen_names", []))
            lines = int(getattr(self, "_stdout_lines", 0))
            parses = int(getattr(self, "_parse_failures", 0))
            samples = len(self._samples)
            unparsed = str(getattr(self, "_unparsed_text", "") or "")
        record = {
            "model": self.model,
            "positive": samples > 0,
            "jsonl": self.path,
            "pose_blocks": blocks,
            "stdout_lines": lines,
            "parse_failures": parses,
            "sample_count": samples,
            "seen_names": names,
            "ign_returncode": returncode,
            "stderr_tail": str(getattr(self, "_stderr_tail", "") or ""),
            "unparsed_text": unparsed,
        }
        record["gap"] = classify_gt_gap(record)
        return record

    def stop(self):
        if not self._alive:
            return
        self._alive = False
        try:
            self._proc.terminate()
            self._proc.wait(timeout=3.0)
        except (OSError, subprocess.TimeoutExpired):
            try:
                self._proc.kill()
            except OSError:
                pass
        self._thread.join(timeout=2.0)
        err_thread = getattr(self, "_err_thread", None)
        if err_thread is not None:
            err_thread.join(timeout=2.0)
        try:
            self._file.close()
        except OSError:
            pass

    # -- analysis -------------------------------------------------------

    def _rows(self):
        with self._lock:
            return list(self._samples)

    def analysis(self, t_release_recv_ns, poll_xyz=None):
        """Release-window summary with honest central-difference speeds.

        ``t_release_recv_ns`` is the vacuum-off instant (receiver
        monotonic). Velocities never smooth across the release instant
        or the respawn gap; the delete->create gap is detected as a
        missing interval and marked on the samples that follow it.
        """
        rows = self._rows()
        result = {
            "model": self.model,
            "jsonl": self.path,
            "n_samples": len(rows),
            "release_recv_ns": t_release_recv_ns,
            "respawn_gap": False,
            "pre_release_speed_m_s": None,
            "post_release_peak_speed_m_s": None,
            "post_release_peak_at_s": None,
            "settled_speed_m_s": None,
            "stream_vs_poll_max_delta_m": None,
            "median_dt_s": None,
            "sim_stamp_valid_ratio": 0.0,
        }
        if not rows:
            return result
        result["sim_stamp_valid_ratio"] = round(sum(
            1 for row in rows
            if row.get("sim_stamp_ns") is not None) / float(len(rows)), 4)
        times = [row["recv_ns"] for row in rows]
        dts = [(times[i + 1] - times[i]) / 1e9 for i in range(len(times) - 1)]
        if dts:
            dts_sorted = sorted(dts)
            result["median_dt_s"] = round(
                dts_sorted[len(dts_sorted) // 2], 6)
        # respawn gap: the largest missing interval inside the release
        # window marks the delete->create boundary (pose/info drops the
        # model between remove and create).
        gap_index = None
        if t_release_recv_ns is not None:
            window = [i for i in range(len(times) - 1)
                      if abs(times[i] - t_release_recv_ns) < 3_000_000_000]
            for i in window:
                if dts[i] >= GT_GAP_S:
                    gap_index = i
                    break
        result["respawn_gap"] = gap_index is not None
        speeds = self._speeds(rows)

        def _in(span):
            return [value for index, value in enumerate(speeds)
                    if index in span]

        if t_release_recv_ns is not None:
            pre = [i for i in range(len(rows))
                   if times[i] <= t_release_recv_ns]
            if pre:
                result["pre_release_speed_m_s"] = _max_safe(
                    _in(range(max(0, pre[0]), pre[-1] + 1)))
            post = [i for i in range(len(rows))
                    if t_release_recv_ns <= times[i]
                    <= t_release_recv_ns + 600_000_000
                    and (gap_index is None or i > gap_index + 1)]
            values = _in(range(post[0], post[-1] + 1)) if post else []
            peak = _max_safe(values)
            result["post_release_peak_speed_m_s"] = peak
            if peak is not None:
                for index, value in enumerate(speeds):
                    if value == peak:
                        result["post_release_peak_at_s"] = round(
                            (times[min(index, len(times) - 1)]
                             - t_release_recv_ns) / 1e9, 4)
                        break
        tail = range(max(0, len(rows) - 1), len(rows))
        result["settled_speed_m_s"] = _max_safe(_in(tail))
        if poll_xyz:
            deltas = []
            for pose in poll_xyz:
                if not pose or len(pose) < 3:
                    continue
                best = min(
                    math.sqrt(sum(
                        (row["xyz"][axis] - float(pose[axis])) ** 2
                        for axis in range(3)))
                    for row in rows)
                deltas.append(best)
            if deltas:
                result["stream_vs_poll_max_delta_m"] = round(max(deltas), 6)
        return result

    def _speeds(self, rows):
        """Central-difference speeds over ~GT_SPEED_WINDOW_S windows."""
        times = [row["recv_ns"] for row in rows]
        speeds = [None] * len(rows)
        for index in range(len(rows)):
            lo = index
            while lo > 0 and (times[index] - times[lo - 1]) / 1e9 \
                    < GT_SPEED_WINDOW_S / 2.0:
                lo -= 1
            hi = index
            while hi < len(rows) - 1 and (
                    times[hi + 1] - times[index]) / 1e9 \
                    < GT_SPEED_WINDOW_S / 2.0:
                hi += 1
            if lo == hi:
                continue
            dt = (times[hi] - times[lo]) / 1e9
            if dt < GT_MIN_DT_S:
                continue
            speeds[index] = math.sqrt(sum(
                (rows[hi]["xyz"][axis] - rows[lo]["xyz"][axis]) ** 2
                for axis in range(3))) / dt
        return speeds


def _max_safe(values):
    finite = [float(v) for v in values if v is not None]
    return round(max(finite), 6) if finite else None


class M3ThreeBoxDriver(MotionOccDriver):
    """Three-box deployment-scene M3 driver (ST09-I S5)."""

    def __init__(self, args):
        super().__init__(args)
        self._m3_sequence = [
            str(part).strip() for part in str(
                getattr(args, "m3_sequence", "") or DEFAULT_SEQUENCE
            ).split(",") if str(part).strip()]
        self._m3_cases = []
        self._m3_handoffs = []
        self._m3_notes = []
        self._m3_seq = 0
        self._m3_hashes = None
        self._m3_gt = None
        self._m3_gt_stream = None
        self._m3_gt_release_ns = None
        self._m3_stop_next = False
        self._finalize = self.create_client(
            FinalizeCurrentBox, "/pickup_box_spawner/finalize_current_box",
            callback_group=self._group)
        self._m3_stats = self.create_client(
            GetCargoMapStats, "/cargo_map/get_stats",
            callback_group=self._group)

    # -- stage notes (raw acquisition + receiver monotonic) --------------

    def _m3_note(self, stage):
        if any(note["stage"] == stage for note in self._m3_notes):
            return
        self._m3_notes.append({
            "stage": str(stage),
            "t_ros": float(self.ros_now_sec()),
            "t_wall": time.time(),
            "t_mono": time.monotonic_ns(),
            "request_id": str(getattr(self, "_m3_request_id", "") or ""),
            "source": "m3_note",
        })

    def _m3_note_place_attach(self):
        """Stamp attach when place motion is submitted, not at pick vacuum-on.

        EVENT_ORDER puts attach after execution. The pick-time vacuum
        command is before observation, so it must not become this stamp.
        """
        state = dict(getattr(self, "_vacuum_state", None) or {})
        if state.get("attached") and not state.get("fail_reason"):
            self._m3_note("attach")

    def _m3_note_route_boundaries(self):
        """Slot, selected path, then probe, in that clock order."""
        self._m3_note("slot")
        self._m3_note("selected_path")
        self._m3_note("probe")

    def _m3_note_return_if_place_finished(self, ok):
        """The pick-start home is not this trial's return_complete."""
        if ok and str(getattr(self, "_place_state", "") or "") in (
                "VERIFIED", "HOME"):
            self._m3_note("return_complete")

    def _rehearsal_mark(self, name):
        super()._rehearsal_mark(name)
        stage = _MARK_STAGES.get(str(name))
        if stage is not None:
            self._m3_note(stage)

    def vacuum_command(self, enable, pick_msg=None):
        enable = bool(enable)
        if (not enable) and getattr(self._args, "use_vacuum", False) \
                and not self._args.plan_only and self._box_model:
            # The subscription is already open. Detach still needs a
            # sample received inside GT_FRESH_S. An older path-check
            # sample is not that sample, and vacuum-off is not sent.
            ok, reason = self._m3_require_fresh_gt()
            if not ok:
                return False, reason
            self._m3_gt_release_ns = time.monotonic_ns()
            self._m3_note("release")
        return super().vacuum_command(enable, pick_msg=pick_msg)

    def _gt_jsonl_path(self):
        return os.path.join(
            self._args.dump_dir, "box_%02d_release_gt.jsonl"
            % int(getattr(self, "_m3_trial_index", 0) or 0))

    def _stop_gt_stream(self):
        stream = self._m3_gt_stream
        self._m3_gt_stream = None
        if stream is not None:
            stream.stop()

    def _write_gt_gap(self, stream, record=None):
        if record is None:
            record = stream.failure_record()
        else:
            record = dict(record)
        record["model"] = str(self._box_model or record.get("model") or "")
        if not record.get("gap"):
            record["gap"] = classify_gt_gap(record)
        stream.stop()
        trial_index = getattr(self, "_m3_trial_index", 0)
        _atomic_json(os.path.join(
            self._args.dump_dir, "box_%02d_release_gt_empty.json"
            % int(trial_index or 0)), record)
        if self._m3_gt_stream is stream:
            self._m3_gt_stream = None
        return record

    def _m3_open_gt_stream(self):
        """Subscribe as soon as the model id exists. Does not mean fresh."""
        if self._m3_gt_stream is not None:
            return self._m3_gt_stream
        stream = GtReleaseStream(self._box_model, self._gt_jsonl_path())
        self._m3_gt_stream = stream
        return stream

    def _m3_confirm_gt_path(self, trial):
        """Prove subscribe/parse/write before any place motion."""
        self._stop_gt_stream()
        model = str(getattr(self, "_box_model", "") or "")
        if not model:
            trial.fail_code = "GT_STREAM_NO_PRE_SAMPLE"
            trial.extras["gt_stream"] = {
                "gap": "no_messages", "model": "", "sample_count": 0}
            return False
        stream = self._m3_open_gt_stream()
        if stream.positive():
            trial.extras["gt_stream"] = {
                "gap": "",
                "model": model,
                "path_ready": True,
                "sample_count": stream.failure_record()["sample_count"],
            }
            return True
        record = self._write_gt_gap(stream)
        trial.fail_code = "GT_STREAM_NO_PRE_SAMPLE"
        trial.extras["gt_stream"] = record
        return False

    def _m3_require_fresh_gt(self):
        stream = self._m3_gt_stream
        if stream is None:
            stream = self._m3_open_gt_stream()
            if not stream.positive():
                record = self._write_gt_gap(stream)
                return False, "GT_STREAM_NO_PRE_SAMPLE:%s" % record["gap"]
        ok, record = stream.release_gate()
        if ok:
            return True, ""
        gap = str(record.get("gap") or "no_messages")
        self._write_gt_gap(stream, record)
        if gap == "stale_sample":
            return False, "GT_STREAM_STALE"
        return False, "GT_STREAM_NO_PRE_SAMPLE:%s" % gap

    def run_place_from_carry(self, pick_msg, trial, slot, slot_meta):
        if (getattr(self._args, "use_vacuum", False)
                and not self._args.plan_only):
            if not self._m3_confirm_gt_path(trial):
                trial.place_state = self._place_state or "CARRY_READY"
                self._dump_place(trial)
                return trial
        return super().run_place_from_carry(
            pick_msg, trial, slot, slot_meta)

    def _poses_after_settle(self):
        result = super()._poses_after_settle()
        stream = getattr(self, "_m3_gt_stream", None)
        if stream is not None:
            self._m3_gt_stream = None
            # The settle poll's own pose readings are the cross-check
            # reference for the stream (the verify extras are not yet
            # populated at this point in the parent flow).
            poll = _xyz3(result[1] or result[0] if result else None)
            self._m3_gt = stream.analysis(self._m3_gt_release_ns,
                                          [poll] if poll else None)
            self._m3_gt["release_note"] = "GT feeds gt_reference only"
            stream.stop()
        return result

    def _place_slot_for(self, pick_msg, box_msg, trial):
        result = super()._place_slot_for(pick_msg, box_msg, trial)
        if result is not None and result[0] is not None:
            self._m3_note("selected_path")
        return result

    def _verify_selected_route(self, response, request_id, stats):
        self._m3_note_route_boundaries()
        result = super()._verify_selected_route(response, request_id, stats)
        if stats.get("map_source") is not None:
            self._m3_selection_map = {
                "map_source": stats.get("map_source"),
                "map_revision": stats.get("admitted_map_revision"),
            }
        return result

    def _execute_segment(self, segment, trial):
        if any(note["stage"] == "selected_path"
               for note in self._m3_notes):
            self._m3_note("execution")
            if str(getattr(segment, "name", "") or "") in (
                    "transit", "traverse", "insert", "descend"):
                self._m3_note_place_attach()
        return super()._execute_segment(segment, trial)

    def _capture_success_debug(self, trial, slot, slot_meta, pose):
        self._m3_note("measured_verification")
        return super()._capture_success_debug(trial, slot, slot_meta, pose)

    def _accept_view(self, trial, final, traces):
        result = super()._accept_view(trial, final, traces)
        self._m3_note("map_update")
        return result

    def _home_arm(self):
        result = super()._home_arm()
        self._m3_note_return_if_place_finished(bool(result and result[0]))
        return result

    # -- per-trial close: GT merge, sequence, finalize, events -----------

    def run_trial(self, index, slot=None, slot_meta=None,
                  already_spawned=False, keep_placed=False):
        if self._m3_stop_next:
            trial = PlaceTrial(index=int(index))
            trial.fail_code = "M3_SESSION_STOPPED"
            return trial
        self._m3_notes = []
        self._m3_selection_map = None
        self._m3_trial_index = index
        self._m3_request_id = "m3-box-%02d" % (int(index) + 1)
        try:
            trial = super().run_trial(
                index, slot=slot, slot_meta=slot_meta,
                already_spawned=already_spawned, keep_placed=keep_placed)
            try:
                self._m3_close_trial(trial)
            except Exception as error:  # noqa: BLE001 - evidence boundary
                import traceback

                trial.extras["m3_close_error"] = "%r %s" % (
                    error, traceback.format_exc(limit=3).replace("\n", " | "))
            self._m3_rewrite_evidence()
            return trial
        finally:
            self._stop_gt_stream()

    def _m3_close_trial(self, trial):
        if self._m3_gt is not None:
            trial.extras["release_gt"] = self._m3_gt
            self._m3_gt = None
        ordinal = int(trial.index) + 1
        catalog = str(trial.catalog_id or "")
        expected = (self._m3_sequence[ordinal - 1]
                    if ordinal <= len(self._m3_sequence) else None)
        if expected is not None and catalog and catalog != expected:
            trial.fail_code = trial.fail_code or "M3_SEQUENCE_DRIFT"
        integrate = trial.extras.get("integrate") or {}
        input_revision = integrate.get("expected_map_revision")
        output_revision = integrate.get("resulting_map_revision")
        return_ok = (str(trial.place_state or "") == "HOME"
                     and place_ok(trial))
        finalize_ok = None
        if (place_ok(trial)
                and integrate.get("success") is True
                and type(input_revision) is int
                and type(output_revision) is int
                and output_revision > input_revision
                and return_ok):
            response = self.call_srv(
                self._finalize, FinalizeCurrentBox.Request(), timeout=15.0)
            finalize_ok = bool(response is not None
                               and getattr(response, "success", False))
            trial.extras["finalize"] = {
                "ok": finalize_ok,
                "message": (getattr(response, "message", "")
                            if response is not None else "timeout"),
            }
            if not finalize_ok:
                # Without finalize the next ensure_clean DELETES the
                # placed box — stop the session instead.
                trial.fail_code = "FINALIZE_REFUSED"
                self._m3_stop_next = True
        self._m3_emit_case(trial, ordinal, catalog, integrate,
                           input_revision, output_revision, finalize_ok,
                           return_ok)
        if finalize_ok and type(output_revision) is int:
            self._m3_record_handoff(ordinal, output_revision)

    def _m3_record_handoff(self, ordinal, output_revision):
        """A handoff exists only when another box will consume this revision.

        The last box of a 1-box or 3-box session has no successor. Emitting
        one anyway fails the judge (1-box expects 0; N-box expects N-1).
        """
        if int(ordinal) >= len(self._m3_sequence):
            return
        self._m3_handoffs.append({
            "box_id": self._m3_box_id(int(ordinal) + 1, None),
            "request_id": "m3-handoff-%02d" % int(ordinal),
            "from": int(ordinal),
            "to": int(ordinal) + 1,
            "from_revision": output_revision,
            "to_revision": output_revision,
        })

    def _m3_box_id(self, ordinal, catalog):
        name = catalog or (
            self._m3_sequence[ordinal - 1]
            if ordinal <= len(self._m3_sequence) else "box")
        return "%s@%02d" % (name, ordinal)

    def _m3_emit_case(self, trial, ordinal, catalog, integrate,
                      input_revision, output_revision, finalize_ok,
                      return_ok):
        hashes = self._m3_hashes or {}
        note_ids = {
            str(note.get("request_id") or "")
            for note in self._m3_notes if note.get("request_id")}
        request_id = (note_ids.pop() if len(note_ids) == 1 else str(
            trial.extras.get("rehearsal_request_id")
            or getattr(self, "_m3_request_id", "")
            or "m3-box-%02d" % ordinal))
        success_all = place_ok(trial)
        notes = {note["stage"]: note for note in self._m3_notes}
        events = []
        for stage in EVENT_ORDER:
            note = notes.get(stage)
            row = {
                "stage": stage,
                "request_id": request_id,
                "success": bool(success_all),
                "scene_revision": hashes.get("scene", ""),
                "geometry_hash": hashes.get("hull", ""),
                "input_map_revision": input_revision,
                "stamp_source": "measured_note" if note else "missing",
            }
            if note is not None:
                row["acquisition_stamp_ns"] = int(
                    round(float(note["t_ros"]) * 1e9))
                row["receiver_stamp_ns"] = int(note["t_mono"])
            if stage in COMMIT_STAGES:
                row["output_map_revision"] = output_revision
            self._m3_seq += 1
            row["event_seq"] = self._m3_seq
            events.append(row)
        verify = trial.extras.get("verify") or {}
        gt = dict(trial.extras.get("release_gt") or {})
        case = {
            "box_id": self._m3_box_id(ordinal, catalog),
            "request_id": request_id,
            "cycle_id": "m3-cycle-%02d" % ordinal,
            "causal_id": "m3-causal-%02d" % ordinal,
            "catalog_id": catalog,
            "events": events,
            "measured": {
                "integration_success": integrate.get("success") is True,
                "return_success": bool(return_ok),
                "payload_retained": bool(finalize_ok),
                "input_map_revision": input_revision,
                "output_map_revision": output_revision,
                "sensor_occupied_cells": integrate.get("n_sensor_occupied"),
                "floor_through_candidates": integrate.get("floor_through"),
                "floor_through_rule": integrate.get("floor_through_rule"),
                "floor_through_max_overlap_m": integrate.get(
                    "floor_through_max_overlap_m"),
                "floor_through_slivers": integrate.get("floor_through_slivers"),
                "floor_through_full_cell_m": integrate.get(
                    "floor_through_full_cell_m"),
                "selection_map": getattr(self, "_m3_selection_map", None),
                "retreat_start": trial.extras.get("retreat_start"),
                **self._m3_map_confirmation(integrate, trial),
            },
            "gt_reference": self._m3_gt_reference(verify, integrate, gt),
            "release_gt": gt,
            "fail_code": trial.fail_code or "",
            "state_timeline": [dict(row) for row in
                               getattr(self, "_timeline", None) or []],
        }
        self._m3_cases.append(case)

    def _m3_map_confirmation(self, integrate, trial):
        """Mapper confirmation evidence when the box was committed.

        Empty without ``--commit-placed-box``: the box is then measured by the
        integration alone and acceptance keeps the sensor-cell rules.
        """
        if not self._commit_mode:
            return {}
        settle = dict(integrate.get("last_settle") or {})
        cleared = list(settle.get("cleared") or [])
        return {"map_confirmation": {
            "mode": "commit_confirmed",
            "confirmed": bool(
                integrate.get("occupancy_confirmed") and cleared),
            "cleared": cleared,
            "evidence_points": dict(settle.get("evidence_points") or {}),
            "min_evidence_points": settle.get("min_evidence_points"),
            "pending_changes": list(integrate.get("pending_changes") or []),
            "cargo_map_commit": trial.extras.get("cargo_map_commit"),
            "commit_revisions": dict(
                trial.extras.get("cargo_map_commit_revisions") or {}),
        }}

    def _m3_gt_reference(self, verify, integrate, gt):
        """gt_reference ONLY from measured artifacts (verify + sensors).

        GT stream numbers ride next to it, never inside the measured
        map fields.
        """
        reference = {}
        if verify:
            if verify.get("err_xy") is not None:
                reference["xy_error_m"] = float(verify["err_xy"])
            if verify.get("err_z") is not None:
                reference["z_error_m"] = float(verify["err_z"])
            if verify.get("drift") is not None:
                reference["drift_m"] = float(verify["drift"])
            if verify.get("roll") is not None:
                reference["roll_deg"] = math.degrees(
                    float(verify["roll"]))
            if verify.get("pitch") is not None:
                reference["pitch_deg"] = math.degrees(
                    float(verify["pitch"]))
            if verify.get("inside_inner_box") is not None:
                reference["center_inside_hull"] = bool(
                    verify["inside_inner_box"])
        if integrate.get("total_cells") is not None:
            reference["footprint_reference_cells"] = int(
                integrate["total_cells"])
        if integrate.get("hit_cells") is not None:
            reference["footprint_sensor_cells"] = int(
                integrate["hit_cells"])
        if gt:
            reference["stream"] = {
                key: gt.get(key) for key in (
                    "n_samples", "median_dt_s", "respawn_gap",
                    "pre_release_speed_m_s", "post_release_peak_speed_m_s",
                    "settled_speed_m_s", "stream_vs_poll_max_delta_m")}
        return reference

    # -- atomic evidence -------------------------------------------------

    def set_frozen_hashes(self, hashes):
        self._m3_hashes = dict(hashes)

    def m3_evidence(self, complete=False, session_deadline=False):
        return {
            "hashes": dict(self._m3_hashes or {}),
            "complete": bool(complete),
            "session_deadline": bool(session_deadline),
            "cases": self._m3_cases,
            "handoffs": self._m3_handoffs,
        }

    def _m3_rewrite_evidence(self, complete=False, session_deadline=False):
        if not self._m3_hashes:
            return
        _atomic_json(
            os.path.join(self._args.out, "m3_evidence.json"),
            self.m3_evidence(complete=complete,
                             session_deadline=session_deadline))

    def _stats_now(self):
        response = self.call_srv(
            self._m3_stats, GetCargoMapStats.Request(), timeout=5.0)
        if response is None or not response.success:
            return None
        return {
            "map_revision": int(response.map_revision),
            "last_acquisition_stamp_ns": int(
                response.last_acquisition_stamp_ns),
            "geometry_hash": str(response.geometry_hash),
            "map_source": str(response.map_source),
        }


def place_metrics_trial(index):
    return PlaceTrial(index=int(index))


def _frozen_groups(repo):
    """Recorded-input file groups for the frozen manifest (S5).

    ``repo`` is the SOURCE tree root (M3_REPO from the runner, or the
    source derivation when the driver itself runs from source). The
    driver executable may live in the install overlay — its own path is
    never a valid repo root.
    """
    repo = os.environ.get("M3_REPO", "") or repo
    scene = os.environ.get("M3_SCENE_TF", "")
    atlas_dir = os.environ.get("M3_ATLAS_DIR", "")
    scene = scene or os.path.join(
        repo, "src", "luggage_description", "config", "scene_tf_deploy.yaml")
    atlas_dir = atlas_dir or os.path.join(
        repo, "src", "luggage_planning", "data",
        "reachability_atlas_deploy_sim")
    atlas_files = sorted(
        os.path.join(atlas_dir, name) for name in os.listdir(atlas_dir)
        if name.endswith((".npz", ".yaml"))) if os.path.isdir(atlas_dir) else []
    container_model = os.path.join(
        repo, "src", "luggage_gazebo", "models", "airport_container_real",
        "model.sdf")
    return {
        "runner": [os.path.abspath(__file__)],
        "extractor": [os.path.join(
            repo, "scripts", "frame_migration_st09_extract.py")],
        "resolved_profile": [os.path.join(
            repo, "st09_contracts", "resolved_planning_contract.json")],
        "scene": [scene],
        "model": [container_model],
        "atlas": atlas_files,
        "urdf": ([os.path.join(atlas_dir, "robot_description.xml")]
                 if os.path.isfile(os.path.join(
                     atlas_dir, "robot_description.xml")) else []),
        "srdf": [os.path.join(
            repo, "src", "luggage_description", "config",
            "S20_with_camera.srdf")],
        "messages": [os.path.join(
            repo, "src", "luggage_msgs", "msg", "MotionSegment.msg"),
            os.path.join(repo, "src", "luggage_msgs", "srv",
                         "SelectPlacePathV2.srv"),
            os.path.join(repo, "src", "luggage_msgs", "srv",
                         "VacuumCommandV2.srv"),
            os.path.join(repo, "src", "luggage_msgs", "srv",
                         "BuildMotionSequenceV2.srv")],
        "hull": [container_model],
    }


def parse_args(argv=None):
    argv = list(argv or [])
    local = argparse.ArgumentParser(add_help=False)
    local.add_argument("--m3-sequence", default=DEFAULT_SEQUENCE)
    # Commit each released box to the mapper ledger, then confirm it by
    # evidence (pairs with the mapper's require_confirmed_map).
    local.add_argument("--commit-placed-box", action="store_true")
    local_ns, remaining = local.parse_known_args(argv)
    if "--n" not in remaining:
        remaining.extend(["--n", "3"])
    args = parse_occ_args(remaining)
    args.m3_sequence = str(local_ns.m3_sequence or DEFAULT_SEQUENCE)
    args.commit_placed_box = bool(local_ns.commit_placed_box)
    return args


def main(argv=None):
    argv = list(argv or sys.argv[1:])
    if "--out" not in argv:
        argv.extend(["--out", DEFAULT_OUT])
    args = parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    args.dump_dir = args.dump_dir or os.path.join(args.out, "dumps")
    os.makedirs(args.dump_dir, exist_ok=True)
    repo = os.path.normpath(os.path.join(_SCRIPTS, "..", "..", ".."))
    from luggage_gazebo.rehearsal_evidence import frozen_document, hash_inputs
    groups = _frozen_groups(repo)
    try:
        hashes = hash_inputs(groups)
    except (ValueError, OSError) as error:
        print("frozen inputs incomplete: %s" % error, file=sys.stderr)
        return 2
    replay = ["bash", "scripts/st09_i/s5_m3_run.sh",
              "--out", args.out, "--n", str(args.n)]
    frozen = frozen_document(
        hashes,
        release_sample_interval_s=0.05,
        drift_sample_interval_s=float(getattr(args, "drift_wait", 1.0)
                                      or 1.0),
        replay_command=replay)
    frozen.update({
        "box_count": int(args.n),
        "sequence_ids": [part for part in str(
            args.m3_sequence).split(",") if part],
        "hashes_status": "recorded_inputs (S5; formal freeze happens at S6)",
        "hash_groups": {key: [os.path.relpath(path, repo)
                              for path in paths]
                        for key, paths in groups.items()},
    })
    jsonl = os.path.join(args.out, "trials.jsonl")
    with open(jsonl, "w", encoding="utf-8"):
        pass
    rclpy_guard = []
    try:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        rclpy.init()
        rclpy_guard.append(rclpy)
    except Exception as error:  # noqa: BLE001
        print("rclpy unavailable: %r" % error, file=sys.stderr)
        return 2
    driver = M3ThreeBoxDriver(args)
    driver.set_frozen_hashes(hashes)
    _atomic_json(os.path.join(args.out, "m3_frozen.json"), frozen)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(driver)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    records = []
    stop = {"flag": False}

    def _on_sigint(_signum, _frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _on_sigint)
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    verdict = {"status": "not_evaluated", "missing": ["session_not_run"],
               "failures": []}
    try:
        if not args.skip_graph_check:
            err = driver.wait_graph(timeout=180.0)
            if err:
                print("graph: %s" % err)
                _atomic_json(os.path.join(args.out, "startup_failure.json"),
                             {"reason": err, "cases_attempted": 0})
                return 2
        print("graph ok; M3 n=%d sequence=%s" % (
            args.n, args.m3_sequence), flush=True)
        time.sleep(8.0)
        for index in range(args.n):
            if stop["flag"] or not rclpy_guard[0].ok():
                break
            if getattr(driver, "_m3_stop_next", False):
                break
            rec = driver.run_trial(index)
            records.append(rec)
            with open(jsonl, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(trial_to_dict(rec),
                                        sort_keys=True, default=str) + "\n")
            print(json.dumps({
                "index": rec.index, "fail_code": rec.fail_code,
                "finalize": rec.extras.get("finalize"),
            }, sort_keys=True, default=str), flush=True)
            if rec.fail_code and args.on_place_fail == "stop":
                if rec.fail_code not in ("COVERAGE_LOW",):
                    break
        # complete = the session ran its course (every planned trial
        # attempted, no deadline/stop). A FAILED box still emits its
        # case with success False — the judge must SEE that as fail,
        # not hide it behind not_evaluated.
        complete = (len(driver._m3_cases) == int(args.n)
                    and not stop["flag"])
        driver._m3_rewrite_evidence(
            complete=complete, session_deadline=bool(stop["flag"]))
        evidence = driver.m3_evidence(complete=complete,
                                      session_deadline=bool(stop["flag"]))
        verdict = extract_acceptance(evidence, frozen)
        if bool(stop["flag"]):
            verdict = dict(verdict)
            verdict["status"] = "not_evaluated"
            verdict["missing"] = list(verdict.get("missing") or []) + [
                "SESSION_DEADLINE"]
        _atomic_json(os.path.join(args.out, "m3_verdict.json"), verdict)
        summary = summarize(records)
        summary.update({
            "sequence_ids": frozen["sequence_ids"],
            "n_cases": len(driver._m3_cases),
            "complete": complete,
            "finalize": [case.get("fail_code")
                         for case in driver._m3_cases],
        })
        with open(os.path.join(args.out, "summary.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True,
                      default=str)
            handle.write("\n")
        print(json.dumps(verdict, sort_keys=True, default=str), flush=True)
        if complete:
            return 0 if verdict.get("status") == "pass" else 1
        return 1
    finally:
        try:
            executor.remove_node(driver)
            driver.destroy_node()
            rclpy_guard[0].shutdown()
        except Exception:  # noqa: BLE001 - teardown boundary
            pass


if __name__ == "__main__":
    sys.exit(main())
