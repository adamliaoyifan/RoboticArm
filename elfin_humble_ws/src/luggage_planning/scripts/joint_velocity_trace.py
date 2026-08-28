#!/usr/bin/env python3
"""Record joint velocity traces to answer the release-settle question.

The release-settle gate reports one number -- the peak over a 3 s window -- and
that number cannot distinguish "the arm is still damping out" from "the sampled
velocity signal never gets below the tolerance in the first place". This tool
records the trace itself so the two can be told apart.

Deliberately minimal dependencies: it talks to the controller action and
``/joint_states`` directly, so experiments A and B need only

    roslaunch luggage_gazebo sim_world.launch

with no MoveIt, no orchestrator and no perception.

Usage::

    # Experiment A: idle noise floor where the arm is parked
    rosrun luggage_planning joint_velocity_trace.py --duration 60 \
        --label idle_observe --output /tmp/idle.json

    # Experiment B: same, at several joint configurations
    rosrun luggage_planning joint_velocity_trace.py --duration 20 \
        --pose near=0,-0.6,-1.2,0,0.6,0 \
        --pose far=0.6,-0.2,-1.6,0,1.0,0 \
        --output /tmp/poses.json

    # Experiment C: the window right after a motion ends
    rosrun luggage_planning joint_velocity_trace.py --duration 5 \
        --pose far=0.6,-0.2,-1.6,0,1.0,0 --settle-delay 0 \
        --controller-state --output /tmp/post_motion.json
"""

from __future__ import division

import argparse
import json
import os
import sys
import time

import actionlib
import rospy
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    JointTrajectoryControllerState,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
from settle_criterion import (  # noqa: E402
    DISPLACEMENT,
    VELOCITY,
    format_diagnostics,
    position_excursions,
    settle_decision,
    trace_statistics,
)

ARM_JOINT_NAMES = [
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
]

DEFAULT_ACTION = "/S20/elfin_arm_controller/follow_joint_trajectory"
DEFAULT_STATE_TOPIC = "/S20/elfin_arm_controller/state"


class Recorder(object):
    def __init__(self, joint_names, record_controller_state=False):
        self._joint_names = list(joint_names)
        self._samples = []
        self._controller_samples = []
        self._recording = False
        self._t0 = None
        rospy.Subscriber(
            "/joint_states", JointState, self._on_joint_state, queue_size=200)
        if record_controller_state:
            rospy.Subscriber(
                DEFAULT_STATE_TOPIC, JointTrajectoryControllerState,
                self._on_controller_state, queue_size=200)

    def _on_joint_state(self, msg):
        if not self._recording:
            return
        if not msg.velocity:
            return
        velocities = {}
        positions = {}
        for index, name in enumerate(msg.name):
            if name not in self._joint_names:
                continue
            velocities[name] = float(msg.velocity[index])
            if index < len(msg.position):
                positions[name] = float(msg.position[index])
        if not velocities:
            return
        # Positions matter as much as velocities here: under Gazebo's kinematic
        # position mode the reported velocity carries a static bias, and the
        # only way to see that is to check whether anything actually moved.
        self._samples.append(
            (msg.header.stamp.to_sec(), velocities, positions))

    def _on_controller_state(self, msg):
        if not self._recording:
            return
        self._controller_samples.append({
            "t": msg.header.stamp.to_sec(),
            "joints": list(msg.joint_names),
            "desired": list(msg.desired.positions),
            "actual": list(msg.actual.positions),
            "error": list(msg.error.positions),
            "actual_velocity": list(msg.actual.velocities),
        })

    def start(self):
        self._samples = []
        self._controller_samples = []
        self._t0 = rospy.Time.now().to_sec()
        self._recording = True

    def stop(self):
        self._recording = False
        return list(self._samples), list(self._controller_samples)


def send_joint_trajectory(values, duration, action_ns=DEFAULT_ACTION):
    client = actionlib.SimpleActionClient(
        action_ns, FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(10.0)):
        raise RuntimeError("controller action unreachable: %s" % action_ns)
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = list(ARM_JOINT_NAMES)
    point = JointTrajectoryPoint()
    point.positions = list(values)
    point.velocities = [0.0] * len(values)
    point.time_from_start = rospy.Duration(duration)
    goal.trajectory.points = [point]
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(duration + 10.0))
    return client.get_state()


def parse_pose(text):
    """``label=v1,v2,v3,v4,v5,v6`` -> (label, [floats])."""
    if "=" in text:
        label, raw = text.split("=", 1)
    else:
        label, raw = "pose", text
    values = [float(v) for v in raw.split(",") if v.strip()]
    if len(values) != len(ARM_JOINT_NAMES):
        raise argparse.ArgumentTypeError(
            "pose '%s' needs %d joint values, got %d"
            % (label, len(ARM_JOINT_NAMES), len(values)))
    return label.strip(), values


def record_once(recorder, label, duration, vel_tol, hold_time, samples=None):
    if samples is None:
        recorder.start()
        deadline = time.time() + duration
        while time.time() < deadline and not rospy.is_shutdown():
            time.sleep(0.05)
        samples, controller_samples = recorder.stop()
    else:
        controller_samples = []

    statistics = trace_statistics(samples)
    excursions = position_excursions(samples)
    # Replay the production rule over the recorded window with BOTH criteria,
    # so one recording shows what each of them would have decided.
    verdicts = {}
    for criterion in (DISPLACEMENT, VELOCITY):
        settled, elapsed, diagnostics = settle_decision(
            samples, vel_tol=vel_tol, hold_time=hold_time, timeout=duration,
            criterion=criterion)
        verdicts[criterion] = {
            "settled": settled,
            "elapsed_sec": elapsed,
            "diagnostics": diagnostics,
        }
        rospy.loginfo(
            "[%s/%s] settled=%s elapsed=%.2fs %s",
            label, criterion, settled, elapsed,
            format_diagnostics(diagnostics))
    return {
        "label": label,
        "duration_sec": duration,
        "vel_tol": vel_tol,
        "hold_time": hold_time,
        "verdicts": verdicts,
        "statistics": statistics,
        "position_excursion": excursions,
        "max_position_excursion": max(excursions.values()) if excursions else 0.0,
        "samples": [
            [
                round(sample[0], 4),
                [round(sample[1].get(name, 0.0), 8)
                 for name in ARM_JOINT_NAMES],
                [round(sample[2].get(name, 0.0), 8)
                 for name in ARM_JOINT_NAMES] if len(sample) > 2 else [],
            ]
            for sample in samples
        ],
        "controller_state": controller_samples,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Record joint velocity traces for settle diagnosis.")
    parser.add_argument("--duration", type=float, default=60.0,
                        help="seconds to record per pose")
    parser.add_argument("--label", default="idle",
                        help="label when no --pose is given")
    parser.add_argument("--pose", action="append", default=[],
                        type=parse_pose, metavar="LABEL=J1,..,J6",
                        help="move here first, then record; repeatable")
    parser.add_argument("--move-duration", type=float, default=4.0,
                        help="trajectory duration when moving to a pose")
    parser.add_argument("--settle-delay", type=float, default=1.0,
                        help="pause between arriving and recording; 0 "
                             "reproduces the post-motion window the release "
                             "gate actually sees")
    parser.add_argument("--vel-tol", type=float, default=0.03)
    parser.add_argument("--hold-time", type=float, default=0.25)
    parser.add_argument("--controller-state", action="store_true",
                        help="also record desired/actual/error from the "
                             "controller, to tell tracking error from "
                             "measurement noise")
    parser.add_argument("--record-motion", action="store_true",
                        help="record while the arm is moving to the pose; "
                             "produces the genuinely-moving trace that any "
                             "settle rule must reject")
    parser.add_argument("--repeat", type=int, default=1,
                        help="cycle through the poses this many times; with "
                             "--settle-delay 0 this is the repeatability "
                             "check for the release-settle gate")
    parser.add_argument("--output", default="",
                        help="write the full trace JSON here")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("joint_velocity_trace", anonymous=True)
    recorder = Recorder(
        ARM_JOINT_NAMES, record_controller_state=args.controller_state)
    # Give the subscriber time to connect before the first recording.
    rospy.sleep(1.0)

    recordings = []
    if args.pose:
        cycle = [
            ("%s_%02d" % (label, index), values)
            for index in range(max(1, args.repeat))
            for label, values in args.pose
        ] if args.repeat > 1 else list(args.pose)
        for label, values in cycle:
            rospy.loginfo("moving to %s: %s", label, values)
            if args.record_motion:
                recorder.start()
                send_joint_trajectory(values, args.move_duration)
                samples, _ = recorder.stop()
                recordings.append(record_once(
                    recorder, label, args.move_duration, args.vel_tol,
                    args.hold_time, samples=samples))
                continue
            send_joint_trajectory(values, args.move_duration)
            if args.settle_delay > 0:
                rospy.sleep(args.settle_delay)
            recordings.append(record_once(
                recorder, label, args.duration, args.vel_tol, args.hold_time))
    else:
        recordings.append(record_once(
            recorder, args.label, args.duration, args.vel_tol, args.hold_time))

    summary = {}
    for criterion in (DISPLACEMENT, VELOCITY):
        passed = [
            r for r in recordings if r["verdicts"][criterion]["settled"]]
        elapsed = sorted(
            r["verdicts"][criterion]["elapsed_sec"] for r in passed)
        summary[criterion] = {
            "pass_count": len(passed),
            "total": len(recordings),
            "pass_rate": len(passed) / len(recordings) if recordings else 0.0,
            "elapsed_p95": (
                elapsed[int(0.95 * (len(elapsed) - 1))] if elapsed else None),
            "elapsed_max": elapsed[-1] if elapsed else None,
        }

    result = {
        "schema_version": 1,
        "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "joint_names": ARM_JOINT_NAMES,
        "vel_tol": args.vel_tol,
        "hold_time": args.hold_time,
        "settle_delay_sec": args.settle_delay,
        "repeat": args.repeat,
        "summary": summary,
        "recordings": recordings,
    }
    if args.output:
        directory = os.path.dirname(os.path.abspath(args.output))
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        with open(args.output, "w") as stream:
            json.dump(result, stream, indent=2, sort_keys=True)
            stream.write("\n")
        rospy.loginfo("wrote %s", args.output)

    # Compact summary on stdout; the JSON keeps the full trace. The two
    # verdict columns side by side are the point: where they disagree, the
    # velocity signal is claiming motion that the positions do not show.
    print("%-16s %9s %9s %12s  %-9s %-9s"
          % ("label", "vel_p50", "vel_p100", "excursion",
             "displ?", "vel?"))
    for recording in recordings:
        stats = recording["statistics"]["max_over_joints"]
        print(
            "%-16s %9.5f %9.5f %12.8f  %-9s %-9s"
            % (
                recording["label"],
                stats["p50"], stats["p100"],
                recording["max_position_excursion"],
                recording["verdicts"][DISPLACEMENT]["settled"],
                recording["verdicts"][VELOCITY]["settled"],
            ))
    for criterion, row in sorted(summary.items()):
        print(
            "%-14s pass %d/%d (%.0f%%)  elapsed p95=%s max=%s"
            % (
                criterion, row["pass_count"], row["total"],
                100.0 * row["pass_rate"],
                "n/a" if row["elapsed_p95"] is None
                else "%.2fs" % row["elapsed_p95"],
                "n/a" if row["elapsed_max"] is None
                else "%.2fs" % row["elapsed_max"],
            ))


if __name__ == "__main__":
    main()
