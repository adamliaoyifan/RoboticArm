"""Opt-in post-verdict evidence window before failed carrying-box cleanup.

No goals, pose changes, vacuum commands or gate changes. The handoff verdict
is already final when this runs. The window is bounded by both clocks.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def retain_failure(driver, record, *, root=None, now=time.monotonic,
                   sleep=time.sleep, recorder_factory=subprocess.Popen):
    root = root if root is not None else os.environ.get(
        "ELFIN_HANDOFF_FAILURE_CAPTURE_DIR", "").strip()
    if not root:
        return None
    directory = Path(root) / ("handoff_%d" % int(now()*1e6))
    directory.mkdir(parents=True)
    started = now()
    start_sim = float(driver.ros_now_sec())
    rows = []
    topic = os.environ.get("ELFIN_BOX_CONTACT_TOPIC", "").strip()
    recorder = None
    log = (directory / "recorder.log").open("w")
    result = dict(verdict=record["code"], gate_deadline_mono=record["deadline_mono"],
                  started_mono=started, started_sim=start_sim,
                  sim_limit_s=5.0, wall_limit_s=30.0, contact_topic=topic,
                  changes_to_motion_or_verdict=False)
    (directory / "manifest.json").write_text(json.dumps(result, indent=2)+"\n")
    try:
        if topic:
            recorder = recorder_factory([
                sys.executable, "-m", "luggage_gazebo.release_capture",
                "--output", str(directory / "transport"),
                "--contact-topic", topic], stdout=log, stderr=subprocess.STDOUT)
        while True:
            sim = float(driver.ros_now_sec())
            rows.append(dict(mono=now(), sim=sim,
                             joints=getattr(driver, "_last_joint_state", None),
                             vacuum=getattr(driver, "_vacuum_state", None)))
            if sim-start_sim >= 5.0:
                result["end_reason"] = "sim_window_complete"
                break
            if now()-started >= 30.0:
                result["end_reason"] = "wall_limit"
                break
            lifecycle = getattr(driver, "_executor_lifecycle", None)
            if lifecycle is not None and lifecycle.failure is not None:
                result["end_reason"] = "executor_failed"
                break
            sleep(0.05)
    except Exception as exc:
        result["end_reason"] = "capture_error"
        result["capture_error"] = "%s: %s" % (type(exc).__name__, exc)
    finally:
        if recorder is not None:
            if recorder.poll() is None:
                recorder.terminate()
            try:
                recorder.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                recorder.kill()
                recorder.wait(timeout=2.0)
                result["recorder_forced_stop"] = True
        log.close()
        with (directory / "joint_vacuum_window.jsonl").open("w") as handle:
            for row in rows:
                handle.write(json.dumps(row)+"\n")
        result.update(ended_mono=now(), ended_sim=float(driver.ros_now_sec()),
                      sample_count=len(rows),
                      contact_absence_proven=False)
        (directory / "manifest.json").write_text(json.dumps(result, indent=2)+"\n")
    return str(directory)
