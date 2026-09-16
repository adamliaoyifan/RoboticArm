"""Stage sequencing and launch-log predicates for the sim_world startup chain.

ROS-free by contract: the node layer lives in
``scripts/sim_startup_rehearsal.py`` and supplies the per-stage callables.

The sequencing rule here exists because of the PF-R7 generation 9 defect: two
probes ran on two clocks, so a spawn deadline started from an eval-side
parameter read while the launch action that owned the edge was still blocked
in its own call, and expired before that action could exit. Each stage's
deadline is therefore measured from the moment its predecessor completed, and
one caller owns every stage.
"""
from __future__ import annotations

import re
import time
from dataclasses import dataclass, field

#: Ordered startup stages. A stage may only be entered after its predecessor
#: reported ``ok``; the deadline for a stage is measured from the monotonic
#: timestamp at which its predecessor completed, never from an outside probe.
STAGES = (
    "robot_description",
    "robot_spawn",
    "plugin_urdf",
    "controller_manager",
    "controllers_active",
    "joint_states",
)

#: Failure reason recorded when a stage does not complete before its deadline.
STAGE_TIMEOUT_REASON = {
    "robot_description": "robot_state_publisher_get_parameters_timeout",
    "robot_spawn": "robot_spawn_missing",
    "plugin_urdf": "plugin_urdf_not_received",
    "controller_manager": "controller_manager_service_absent",
    "controllers_active": "controllers_not_active",
    "joint_states": "joint_states_absent_after_controller_startup",
}

#: Default per-stage deadline in seconds, measured from the previous stage.
DEFAULT_STAGE_DEADLINE = {
    "robot_description": 30.0,
    "robot_spawn": 20.0,
    "plugin_urdf": 20.0,
    "controller_manager": 40.0,
    "controllers_active": 40.0,
    "joint_states": 20.0,
}

#: Emitted by ``sim_world.launch.py`` immediately before the S20 create action.
SPAWN_MARKER = "spawn_robot: create S20"

#: Emitted by the ``gz_ros2_control`` plugin once it holds the model URDF.
PLUGIN_URDF_MARKER = "Received URDF from param server"

#: Fallback for a launch file that predates ``SPAWN_MARKER``: the robot create
#: is the fourth ``create`` process (pedestal, platform, container, then S20).
_CREATE_STARTED = re.compile(r"\[create-(\d+)\]: process started")

SCENE_CREATE_COUNT = 3

REQUIRED_CONTROLLERS = ("joint_state_broadcaster", "elfin_arm_controller")

EXPECTED_JOINT_COUNT = 6


def robot_spawn_started(launch_log: str) -> bool:
    """True once the launch actually scheduled the S20 ``create`` action.

    Prefers the explicit marker. Falls back to counting ``create`` processes so
    a stock launch file without the marker is still measurable.
    """
    if SPAWN_MARKER in launch_log:
        return True
    started = {int(n) for n in _CREATE_STARTED.findall(launch_log)}
    return len(started) > SCENE_CREATE_COUNT


def plugin_received_urdf(launch_log: str) -> bool:
    return PLUGIN_URDF_MARKER in launch_log


def marker_order_ok(launch_log: str) -> bool:
    """The spawn marker must precede the plugin URDF receipt.

    A plugin receipt with no preceding spawn marker means the marker was
    emitted by teardown rather than by the startup edge.
    """
    spawn = launch_log.find(SPAWN_MARKER)
    plugin = launch_log.find(PLUGIN_URDF_MARKER)
    if plugin < 0:
        return True
    if spawn < 0:
        return False
    return spawn < plugin


def spawn_marker_count(launch_log: str) -> int:
    return launch_log.count(SPAWN_MARKER)


def wait_for_service(available, timeout_sec, poll_sec=0.25,
                     clock=time.monotonic, sleep=time.sleep):
    """Poll ``available`` until it is true or ``timeout_sec`` elapses.

    Used by the controller-manager watchdog. ``available`` must be a cheap
    check against an already-constructed node; building one per poll is what
    made the generation-9 helper unable to answer in time.
    """
    deadline = clock() + float(timeout_sec)
    while clock() < deadline:
        if available():
            return True
        sleep(poll_sec)
    return bool(available())


@dataclass
class StageRecord:
    stage: str
    ok: bool
    t_enter: float
    t_exit: float
    detail: dict = field(default_factory=dict)

    @property
    def duration_sec(self) -> float:
        return self.t_exit - self.t_enter

    def as_dict(self) -> dict:
        return {
            "stage": self.stage,
            "ok": self.ok,
            "t_enter_sec": round(self.t_enter, 4),
            "t_exit_sec": round(self.t_exit, 4),
            "duration_sec": round(self.duration_sec, 4),
            "detail": self.detail,
        }


@dataclass
class ProbeResult:
    ok: bool
    reason: str
    stages: list = field(default_factory=list)
    elapsed_sec: float = 0.0

    def as_dict(self) -> dict:
        return {
            "ok": self.ok,
            "reason": self.reason,
            "elapsed_sec": round(self.elapsed_sec, 4),
            "stages": [s.as_dict() for s in self.stages],
            "stage_durations_sec": {
                s.stage: round(s.duration_sec, 4) for s in self.stages
            },
            "last_stage": self.stages[-1].stage if self.stages else None,
        }


def run_stages(checks, deadlines, clock=time.monotonic, poll_sec=0.2,
               global_deadline_sec=None, abort=None):
    """Drive ``STAGES`` in order against ``checks`` under one clock.

    ``checks`` maps a stage name to a callable returning either ``True`` /
    ``False`` or ``(ok, detail_dict)``. Each stage's deadline is measured from
    the moment its predecessor completed, so a slow-but-successful earlier
    stage can never make a later stage look late.

    ``abort`` is an optional callable returning a reason string to stop early
    (used for a dead launch process).
    """
    t_start = clock()
    stages = []
    t_enter = t_start
    for stage in STAGES:
        check = checks[stage]
        deadline = t_enter + float(deadlines[stage])
        detail = {}
        ok = False
        while True:
            if abort is not None:
                reason = abort()
                if reason:
                    stages.append(StageRecord(stage, False, t_enter - t_start,
                                              clock() - t_start, detail))
                    return ProbeResult(False, reason, stages, clock() - t_start)
            outcome = check()
            if isinstance(outcome, tuple):
                ok, detail = outcome[0], (outcome[1] or {})
            else:
                ok, detail = bool(outcome), {}
            now = clock()
            if ok:
                stages.append(StageRecord(stage, True, t_enter - t_start,
                                          now - t_start, detail))
                t_enter = now
                break
            if global_deadline_sec is not None and \
                    now - t_start >= float(global_deadline_sec):
                stages.append(StageRecord(stage, False, t_enter - t_start,
                                          now - t_start, detail))
                return ProbeResult(False, "startup_global_timeout", stages,
                                   now - t_start)
            if now >= deadline:
                stages.append(StageRecord(stage, False, t_enter - t_start,
                                          now - t_start, detail))
                return ProbeResult(False, STAGE_TIMEOUT_REASON[stage], stages,
                                   now - t_start)
            time.sleep(poll_sec)
    return ProbeResult(True, "ok", stages, clock() - t_start)
