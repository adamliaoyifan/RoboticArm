#!/usr/bin/env python3
"""Step-through front end for the active-loading pipeline.

This panel used to call the pipeline services itself, which made it a second
implementation of the orchestrator state machine. The two drifted: the panel
kept falling back to spawner ground truth on a detection failure, opened the
vacuum on the wrong segment, never re-planned the placement with the payload
attached, and committed a placement without the verify / margin / overlap
chain -- including calling ``clear_current_box`` where the orchestrator calls
``finalize_current_box``, which is exactly the bug that invalidated the E16 and
E17 multi-box runs.

So the panel no longer executes anything. The orchestrator runs in
``run_mode:=manual``, pauses before each state, and this panel decides when it
may proceed. Every gate, log record and acceptance event therefore comes from
the same code path an automated run uses, and a manual run produces an
``events.jsonl`` the bag harness can judge.

The Probe tab is the deliberate exception: it calls services directly for
ad-hoc debugging, and any use of it permanently marks the session so its events
file can never be presented as evidence.
"""

from __future__ import division

import json
import os
import queue
import subprocess
import sys
import threading

import rospy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from std_srvs.srv import Empty, Trigger

from luggage_msgs.msg import LoadTaskStatus
from luggage_msgs.srv import (
    ClearCurrentBox,
    GetCargoMapStats,
    GoToRobotPose,
    InspectContainer,
    OrchestratorStep,
    ResetCargoMap,
    VacuumCommand,
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
from size_uncertainty import (  # noqa: E402
    aabb_overlap_volume,
    box_aabb,
    inflate_size,
    inflated_center_z,
)

# Imported rather than copied so the step targets cannot drift from the states
# the orchestrator actually runs.
try:
    from orchestrator_node import Orchestrator
    PIPELINE_STATES = tuple(Orchestrator.STATES)
except Exception:  # pragma: no cover - GUI must still open without the node
    PIPELINE_STATES = (
        "Idle", "ResetObserve", "InitialExploreCargo",
        "ReturnObserveAfterInitialExplore", "SyncScene", "SpawnCurrentBox",
        "DetectPickupBox", "ReturnObserveBeforeDetect", "ExploreCargo",
        "InspectContainer", "ComputePlacement", "Detect", "PlanPick",
        "ExecPick", "PlanPlace", "ExecPlace", "UpdateOccupancy",
    )

STEP_SERVICE = "/orchestrator/step"
STATUS_TOPIC = "/orchestrator/status"
POLL_PERIOD_SEC = 0.5
EVENTS_TAIL_LINES = 40


def _fmt(value, digits=3):
    try:
        return ("%%.%df" % digits) % float(value)
    except (TypeError, ValueError):
        return str(value)


class ManualLoadingWindow(QMainWindow):
    def __init__(self):
        super(ManualLoadingWindow, self).__init__()
        self.setWindowTitle("Active Loading — step control")
        self._messages = queue.Queue()
        self._snapshot = {}
        self._last_status = None
        self._events_path = ""
        self._probe_calls = 0
        self._services = {}

        self._build_ui()

        rospy.Subscriber(
            STATUS_TOPIC, LoadTaskStatus, self._on_status, queue_size=10)

        self._poll_stop = threading.Event()
        self._poll_thread = threading.Thread(target=self._poll_loop)
        self._poll_thread.daemon = True
        self._poll_thread.start()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._drain)
        self._timer.start(200)

    # ------------------------------------------------------------------ UI

    def _build_ui(self):
        tabs = QTabWidget()
        tabs.addTab(self._build_pipeline_tab(), "Pipeline")
        tabs.addTab(self._build_probe_tab(), "Probe (non-evidence)")
        self.setCentralWidget(tabs)

    def _build_pipeline_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(self._build_banner())
        layout.addWidget(self._build_step_controls())
        layout.addWidget(self._build_breakpoints())
        layout.addWidget(self._build_observation(), stretch=1)
        layout.addWidget(self._build_evidence())
        return page

    def _build_banner(self):
        box = QGroupBox("Orchestrator")
        grid = QGridLayout(box)
        self.banner = {}
        fields = [
            ("run_mode", "run mode"),
            ("paused", "paused before"),
            ("placed", "placed"),
            ("state", "last status"),
            ("dry_run", "dry_run_motion"),
            ("strict", "strict_perception"),
            ("events", "events file"),
            ("taint", "session"),
        ]
        for index, (key, label) in enumerate(fields):
            value = QLabel("—")
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            grid.addWidget(QLabel("%s:" % label), index // 2, (index % 2) * 2)
            grid.addWidget(value, index // 2, (index % 2) * 2 + 1)
            self.banner[key] = value
        return box

    def _build_step_controls(self):
        box = QGroupBox("Step control")
        row = QHBoxLayout(box)
        self.step_buttons = []
        for label, handler in (
            ("Step", self.cmd_step),
            ("Run", self.cmd_run),
            ("Run to next box", self.cmd_run_next_box),
            ("Pause", self.cmd_pause),
            ("Abort", self.cmd_abort),
        ):
            button = QPushButton(label)
            button.clicked.connect(handler)
            row.addWidget(button)
            self.step_buttons.append(button)

        self.run_to_state = QComboBox()
        self.run_to_state.addItems(
            [name for name in PIPELINE_STATES if name != "Idle"])
        row.addWidget(self.run_to_state)
        run_to = QPushButton("Run to")
        run_to.clicked.connect(self.cmd_run_to)
        row.addWidget(run_to)
        self.step_buttons.append(run_to)
        return box

    def _build_breakpoints(self):
        box = QGroupBox("Breakpoints (pause before these states)")
        layout = QVBoxLayout(box)
        self.breakpoint_list = QListWidget()
        self.breakpoint_list.setMaximumHeight(110)
        self.breakpoint_list.setFlow(QListWidget.LeftToRight)
        self.breakpoint_list.setWrapping(True)
        self.breakpoint_list.setResizeMode(QListWidget.Adjust)
        for name in PIPELINE_STATES:
            if name == "Idle":
                continue
            item = QListWidgetItem(name)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Unchecked)
            self.breakpoint_list.addItem(item)
        layout.addWidget(self.breakpoint_list)
        apply_button = QPushButton("Apply breakpoints")
        apply_button.clicked.connect(self.cmd_apply_breakpoints)
        layout.addWidget(apply_button)
        return box

    def _build_observation(self):
        tabs = QTabWidget()

        self.box_view = QTextEdit()
        self.box_view.setReadOnly(True)
        tabs.addTab(self.box_view, "Box / size")

        self.candidate_table = QTableWidget(0, 9)
        self.candidate_table.setHorizontalHeaderLabels([
            "id", "score", "peak", "container_x", "container_y",
            "atlas", "floor", "feasible", "reason",
        ])
        tabs.addTab(self.candidate_table, "Candidates")

        self.commit_view = QTextEdit()
        self.commit_view.setReadOnly(True)
        tabs.addTab(self.commit_view, "Commit preview")

        self.verify_view = QTextEdit()
        self.verify_view.setReadOnly(True)
        tabs.addTab(self.verify_view, "Verify / drift")

        self.events_view = QTextEdit()
        self.events_view.setReadOnly(True)
        tabs.addTab(self.events_view, "Events")

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        tabs.addTab(self.log_view, "Status log")
        return tabs

    def _build_evidence(self):
        box = QGroupBox("Evidence")
        row = QHBoxLayout(box)
        self.harness_boxes = QComboBox()
        self.harness_boxes.addItems([str(n) for n in range(1, 11)])
        self.harness_boxes.setCurrentText("3")
        row.addWidget(QLabel("expected boxes:"))
        row.addWidget(self.harness_boxes)
        button = QPushButton("Run bag harness on events file")
        button.clicked.connect(self.cmd_run_harness)
        row.addWidget(button, stretch=1)
        return box

    def _build_probe_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        warning = QLabel(
            "These calls bypass the orchestrator. Using anything here marks "
            "the session permanently, and the bag harness will refuse its "
            "events file (untainted_session gate)."
        )
        warning.setWordWrap(True)
        warning.setStyleSheet(
            "background:#8b0000; color:white; padding:6px; font-weight:bold;")
        layout.addWidget(warning)

        grid = QGridLayout()
        probes = [
            ("Go to observe", lambda: self._probe_pose("observe")),
            ("Go to pickup_observe",
             lambda: self._probe_pose("pickup_observe")),
            ("Clear octomap", self._probe_clear_octomap),
            ("Sync dynamic scene", self._probe_sync_dynamic),
            ("Reset cargo map", self._probe_reset_map),
            ("Cargo map stats", self._probe_map_stats),
            ("Inspect container", self._probe_inspect),
            ("Vacuum ON", lambda: self._probe_vacuum(True)),
            ("Vacuum OFF", lambda: self._probe_vacuum(False)),
            ("Clear current box", self._probe_clear_box),
        ]
        for index, (label, handler) in enumerate(probes):
            button = QPushButton(label)
            button.clicked.connect(
                lambda _checked=False, h=handler, n=label: self._run_probe(n, h))
            grid.addWidget(button, index // 2, index % 2)
        layout.addLayout(grid)

        self.probe_log = QTextEdit()
        self.probe_log.setReadOnly(True)
        layout.addWidget(self.probe_log, stretch=1)
        return page

    # -------------------------------------------------------------- plumbing

    def _service(self, name, srv_type, timeout=3.0):
        if name not in self._services:
            rospy.wait_for_service(name, timeout=timeout)
            self._services[name] = rospy.ServiceProxy(name, srv_type)
        return self._services[name]

    def _on_status(self, msg):
        self._messages.put(("status", msg))

    def _call_step(self, command, target_state="", breakpoints=None,
                   clear_breakpoints=False, reason=""):
        proxy = self._service(STEP_SERVICE, OrchestratorStep)
        return proxy(
            command, target_state, list(breakpoints or []),
            bool(clear_breakpoints), reason)

    def _send(self, command, **kwargs):
        try:
            response = self._call_step(command, **kwargs)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            self._messages.put(("log", "%s failed: %s" % (command, exc)))
            return None
        self._messages.put((
            "log", "%s: %s" % (command, response.message)))
        self._messages.put(("snapshot", self._snapshot_from(response)))
        return response

    @staticmethod
    def _snapshot_from(response):
        return {
            "run_mode": response.run_mode,
            "paused": response.paused,
            "paused_state": response.paused_state,
            "placed_count": response.placed_count,
            "probe_touched": response.probe_touched,
            "breakpoints": list(response.active_breakpoints),
        }

    def _poll_loop(self):
        """Background poller so a paused orchestrator never blocks the UI."""
        while not self._poll_stop.is_set() and not rospy.is_shutdown():
            try:
                response = self._call_step("status")
                self._messages.put(
                    ("snapshot", self._snapshot_from(response)))
            except (rospy.ROSException, rospy.ServiceException):
                self._messages.put(("snapshot", None))
            try:
                self._messages.put(("params", self._read_params()))
            except Exception as exc:  # params are best effort
                rospy.logdebug("param poll failed: %s", exc)
            self._poll_stop.wait(POLL_PERIOD_SEC)

    def _read_params(self):
        return {
            "best": rospy.get_param("/luggage/placement/best", {}) or {},
            "candidates": rospy.get_param(
                "/luggage/placement/candidates", []) or [],
            "placed": rospy.get_param(
                "/luggage/container_inspection/placed_boxes", []) or [],
            "size_eval": rospy.get_param(
                "/luggage/perception/size_eval/latest", {}) or {},
            "size_drift": rospy.get_param(
                "/luggage/verification/size_drift", {}) or {},
            "current_box": rospy.get_param("/luggage/current_box", {}) or {},
            "events_path": rospy.get_param("/orchestrator/events_path", ""),
            "dry_run": rospy.get_param("/orchestrator/dry_run_motion", None),
            "strict": rospy.get_param(
                "/orchestrator/strict_perception", None),
            "xy_margin": float(rospy.get_param(
                "/orchestrator/size_uncertainty/xy_margin", 0.02)),
            "z_margin": float(rospy.get_param(
                "/orchestrator/size_uncertainty/z_margin", 0.01)),
        }

    def _drain(self):
        while True:
            try:
                kind, payload = self._messages.get_nowait()
            except queue.Empty:
                return
            if kind == "status":
                self._apply_status(payload)
            elif kind == "snapshot":
                self._apply_snapshot(payload)
            elif kind == "params":
                self._apply_params(payload)
            elif kind == "probe":
                self.probe_log.append(payload)
            else:
                self.log_view.append(payload)

    def _apply_status(self, msg):
        self._last_status = msg
        self.banner["state"].setText("[%s] %s" % (msg.state, msg.message))
        self.banner["placed"].setText(str(msg.placed_count))
        self.log_view.append("[%s] %s" % (msg.state, msg.message))

    def _apply_snapshot(self, snapshot):
        if snapshot is None:
            self.banner["run_mode"].setText("orchestrator unavailable")
            self.banner["paused"].setText("—")
            return
        self._snapshot = snapshot
        mode = snapshot["run_mode"]
        self.banner["run_mode"].setText(
            mode if mode == "manual"
            else "%s (stepping disabled)" % mode)
        self.banner["paused"].setText(
            snapshot["paused_state"] if snapshot["paused"]
            else "running (%s)" % (snapshot["paused_state"] or "—"))
        self.banner["taint"].setText(
            "TAINTED — not evidence" if snapshot["probe_touched"]
            else "clean")
        for index in range(self.breakpoint_list.count()):
            item = self.breakpoint_list.item(index)
            checked = item.text() in snapshot["breakpoints"]
            item.setCheckState(Qt.Checked if checked else Qt.Unchecked)

    def _apply_params(self, params):
        self._events_path = params["events_path"]
        self.banner["events"].setText(params["events_path"] or "not recording")
        for key, label in (("dry_run", "dry_run"), ("strict", "strict")):
            value = params[key]
            self.banner[label].setText(
                "unknown" if value is None else str(bool(value)))
        self._render_box(params)
        self._render_candidates(params["candidates"], params["best"])
        self._render_commit_preview(params)
        self._render_verify(params)
        self._render_events(params["events_path"])

    # ------------------------------------------------------------- rendering

    def _render_box(self, params):
        evaluation = params["size_eval"]
        current = params["current_box"]
        lines = []
        if current:
            lines.append("current box: %s" % json.dumps(current, sort_keys=True))
        if evaluation:
            detected = evaluation.get("detected") or []
            spawned = evaluation.get("spawned") or []
            errors = evaluation.get("errors") or []
            lines.append("detected  w/d/h: %s" % ", ".join(
                _fmt(v) for v in detected))
            lines.append("spawned   w/d/h: %s" % ", ".join(
                _fmt(v) for v in spawned))
            lines.append("error     w/d/h: %s" % ", ".join(
                _fmt(v) for v in errors))
            lines.append("")
            lines.append(
                "The spawned row is an evaluation oracle only; perception owns "
                "the size and is gated on physical plausibility, not on this "
                "comparison.")
        if not lines:
            lines.append("no box detected yet")
        self.box_view.setPlainText("\n".join(lines))

    def _render_candidates(self, candidates, best):
        best_id = str(best.get("candidate_id", "")) if best else ""
        self.candidate_table.setRowCount(len(candidates))
        for row, candidate in enumerate(candidates):
            local = candidate.get("center_local", [0.0, 0.0, 0.0])
            peak = float(candidate.get("peak", 0.0))
            values = [
                str(candidate.get("candidate_id", "")),
                _fmt(candidate.get("score", 0.0), 3),
                _fmt(peak, 3),
                _fmt(local[0]),
                _fmt(local[1]),
                str(candidate.get("atlas_status", "")),
                "yes" if peak <= 1e-3 else "no",
                "yes" if candidate.get("feasible") else "no",
                str(candidate.get("reason", "")),
            ]
            is_best = bool(values[0]) and values[0] == best_id
            for column, text in enumerate(values):
                item = QTableWidgetItem(text)
                if is_best:
                    item.setBackground(QColor(32, 96, 32))
                self.candidate_table.setItem(row, column, item)
        self.candidate_table.resizeColumnsToContents()

    def _render_commit_preview(self, params):
        best = params["best"]
        if not best or not best.get("feasible"):
            self.commit_view.setPlainText("no feasible candidate selected")
            return
        size = [float(v) for v in best.get("size", [0.0, 0.0, 0.0])]
        center = [float(v) for v in best.get("center_base", [0.0, 0.0, 0.0])]
        inflated = inflate_size(size, params["xy_margin"], params["z_margin"])
        center_z = inflated_center_z(center[2], params["z_margin"])
        candidate_aabb = box_aabb(
            (center[0], center[1], center_z), inflated)

        lines = [
            "measured size    : %s" % ", ".join(_fmt(v) for v in size),
            "committed size   : %s  (xy margin %s, z margin %s)" % (
                ", ".join(_fmt(v) for v in inflated),
                _fmt(params["xy_margin"]), _fmt(params["z_margin"])),
            "committed center : %s" % ", ".join(
                _fmt(v) for v in (center[0], center[1], center_z)),
            "",
            "overlap against %d committed boxes:" % len(params["placed"]),
        ]
        worst = 0.0
        for index, placed in enumerate(params["placed"]):
            position = placed.get("place_pose", {}).get("position", {})
            other = box_aabb(
                (position.get("x", 0.0), position.get("y", 0.0),
                 position.get("z", 0.0)),
                (placed.get("width", 0.0), placed.get("depth", 0.0),
                 placed.get("height", 0.0)))
            volume = aabb_overlap_volume(candidate_aabb, other)
            worst = max(worst, volume)
            lines.append("  placed_%d: %.6f m3" % (index, volume))
        if not params["placed"]:
            lines.append("  (none)")
        lines.append("")
        lines.append(
            "worst overlap %.6f m3 — the commit gate rejects anything above "
            "the tolerance." % worst)
        self.commit_view.setPlainText("\n".join(lines))

    def _render_verify(self, params):
        drift = params["size_drift"]
        if not drift:
            self.verify_view.setPlainText("no post-place measurement yet")
            return
        self.verify_view.setPlainText(json.dumps(drift, indent=2, sort_keys=True))

    def _render_events(self, path):
        if not path or not os.path.isfile(path):
            self.events_view.setPlainText(
                "no events file (start the orchestrator with events_path)")
            return
        try:
            with open(path, "r") as stream:
                lines = stream.read().strip().splitlines()
        except IOError as exc:
            self.events_view.setPlainText("cannot read events: %s" % exc)
            return
        self.events_view.setPlainText(
            "\n".join(lines[-EVENTS_TAIL_LINES:]))

    # -------------------------------------------------------------- commands

    def cmd_step(self):
        self._send("step")

    def cmd_run(self):
        self._send("run")

    def cmd_run_to(self):
        self._send("run_to", target_state=self.run_to_state.currentText())

    def cmd_run_next_box(self):
        self._send("run_to", target_state="SpawnCurrentBox")

    def cmd_pause(self):
        self._send("pause")

    def cmd_abort(self):
        reply = QMessageBox.question(
            self, "Abort run",
            "End the run at the next state boundary?",
            QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self._send("abort")

    def cmd_apply_breakpoints(self):
        selected = []
        for index in range(self.breakpoint_list.count()):
            item = self.breakpoint_list.item(index)
            if item.checkState() == Qt.Checked:
                selected.append(item.text())
        self._send(
            "status", breakpoints=selected, clear_breakpoints=not selected)

    def cmd_run_harness(self):
        if not self._events_path or not os.path.isfile(self._events_path):
            QMessageBox.warning(
                self, "No events file",
                "The orchestrator is not recording events. Start it with "
                "events_path:=<file>.")
            return
        command = [
            sys.executable,
            os.path.join(SCRIPT_DIR, "active_loading_bag_harness.py"),
            self._events_path,
            "--expected-boxes", self.harness_boxes.currentText(),
        ]
        try:
            completed = subprocess.run(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                check=False)
            output = completed.stdout.decode("utf-8", "replace")
        except OSError as exc:
            output = "harness failed to start: %s" % exc
        self.log_view.append(output)
        QMessageBox.information(self, "Bag harness", output[-4000:])

    # ----------------------------------------------------------------- probe

    def _run_probe(self, name, handler):
        """Every probe taints the session before it touches anything."""
        try:
            self._call_step("taint", reason="probe: %s" % name)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            # No orchestrator to taint means no events file to protect, but the
            # user still needs to know the marking did not happen.
            self._messages.put((
                "probe", "WARNING could not mark session tainted: %s" % exc))
        self._probe_calls += 1
        thread = threading.Thread(
            target=self._probe_worker, args=(name, handler))
        thread.daemon = True
        thread.start()

    def _probe_worker(self, name, handler):
        try:
            message = handler()
        except Exception as exc:
            message = "failed: %s" % exc
        self._messages.put(("probe", "%s -> %s" % (name, message)))

    def _probe_pose(self, pose_name):
        response = self._service(
            "/motion_planner/go_to_robot_pose", GoToRobotPose,
            timeout=30.0)(pose_name)
        return response.message

    def _probe_clear_octomap(self):
        self._service("/clear_octomap", Empty)()
        return "octomap cleared"

    def _probe_sync_dynamic(self):
        return self._service(
            "/dynamic_scene_manager/sync_dynamic_scene", Trigger)().message

    def _probe_reset_map(self):
        return self._service(
            "/cargo_volume_mapper/reset_cargo_map", ResetCargoMap)().message

    def _probe_map_stats(self):
        stats = self._service(
            "/cargo_volume_mapper/get_cargo_map_stats", GetCargoMapStats)()
        return "unknown=%.1f%% occupied=%.1f%% frontier=%d" % (
            stats.unknown_ratio * 100.0, stats.occupancy_ratio * 100.0,
            stats.frontier_count)

    def _probe_inspect(self):
        return self._service(
            "/container_inspector/inspect_container",
            InspectContainer)("fused").message

    def _probe_vacuum(self, enable):
        return self._service(
            "/vacuum_simulator/vacuum_command", VacuumCommand)(enable).message

    def _probe_clear_box(self):
        return self._service(
            "/pickup_box_spawner/clear_current_box", ClearCurrentBox)().message

    def closeEvent(self, event):
        self._poll_stop.set()
        super(ManualLoadingWindow, self).closeEvent(event)


def main():
    rospy.init_node("manual_loading_gui", anonymous=True)
    app = QApplication([])
    window = ManualLoadingWindow()
    window.resize(1000, 860)
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
