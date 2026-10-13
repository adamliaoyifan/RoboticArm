#!/usr/bin/env python3
"""Refuse to enter motion unless the reference-capture inputs are really live.

Checking that a topic exists, or that it has a publisher, is not enough (the
``/S20/...`` controller_state name had neither a publisher nor data and would
have passed a name check). This probe listens to BOTH streams at once with one
persistent node and verifies the content:

* ``controller_state``: continuous strictly-advancing stamps at a plausible
  rate, the six canonical joint names in every message, finite six-element
  ``reference``, ``feedback`` and ``error`` positions in every message;
* ``/joint_states``: continuous strictly-advancing stamps, all six joints,
  finite positions;
* both streams overlap in time and both were still arriving at the end of the
  window.

Exit 0 when every check passes, 1 otherwise, 2 on usage error. One JSON object
on stdout. The runner must not start the driver on a non-zero exit.

  capture_preflight.py [--window-sec 3.0] [--timeout-sec 30]
"""
import argparse
import json
import math
import sys
import time

JOINTS = tuple("elfin_joint%d" % i for i in range(1, 7))
CONTROLLER_STATE_TOPIC = "/elfin_arm_controller/controller_state"
JOINT_STATES_TOPIC = "/joint_states"

MIN_MESSAGES = 100
MIN_RATE_HZ = 40.0
MIN_OVERLAP_SEC = 1.5
MAX_FINAL_AGE_SEC = 0.5


def _finite_six(values):
    try:
        values = [float(v) for v in values]
    except (TypeError, ValueError):
        return False
    return len(values) == 6 and all(math.isfinite(v) for v in values)


def _stamp_stats(stamps):
    """``(count, span, rate, non_advancing)`` for a stamp list."""
    count = len(stamps)
    non_advancing = sum(
        1 for a, b in zip(stamps, stamps[1:]) if b <= a)
    span = (stamps[-1] - stamps[0]) if count >= 2 else 0.0
    rate = ((count - 1) / span) if span > 0.0 else 0.0
    return count, span, rate, non_advancing


def evaluate_capture(controller_rows, joint_rows, end_mono):
    """Pure verdict. Rows are dicts; see the module tests for the shape.

    controller row: stamp, rx_mono, names, reference, feedback, error
    joint row:      stamp, rx_mono, names, position
    ``end_mono`` is the monotonic time at the end of the listening window.
    """
    checks = []

    def check(name, ok, detail):
        checks.append({"check": name, "ok": bool(ok), "detail": detail})

    c_stamps = [row["stamp"] for row in controller_rows]
    j_stamps = [row["stamp"] for row in joint_rows]
    c_count, c_span, c_rate, c_bad = _stamp_stats(c_stamps)
    j_count, j_span, j_rate, j_bad = _stamp_stats(j_stamps)
    metrics = {
        "controller_state": {
            "messages": c_count, "span_sec": round(c_span, 3),
            "rate_hz": round(c_rate, 2), "non_advancing_stamps": c_bad},
        "joint_states": {
            "messages": j_count, "span_sec": round(j_span, 3),
            "rate_hz": round(j_rate, 2), "non_advancing_stamps": j_bad},
    }
    check("controller_state_messages", c_count >= MIN_MESSAGES,
          "%d messages (need >= %d)" % (c_count, MIN_MESSAGES))
    check("controller_state_rate", c_rate >= MIN_RATE_HZ,
          "%.1f Hz (need >= %.0f)" % (c_rate, MIN_RATE_HZ))
    check("controller_state_stamps_advance", c_count > 0 and c_bad == 0,
          "%d non-advancing stamps" % c_bad)
    bad_names = [i for i, row in enumerate(controller_rows)
                 if tuple(row["names"] or ()) != JOINTS
                 and set(row["names"] or ()) != set(JOINTS)]
    check("controller_state_joint_names", c_count > 0 and not bad_names,
          "%d messages without the six canonical joint names"
          % len(bad_names))
    for field in ("reference", "feedback", "error"):
        bad = [i for i, row in enumerate(controller_rows)
               if not _finite_six(row.get(field) or [])]
        check("controller_state_%s_present" % field,
              c_count > 0 and not bad,
              "%d messages without six finite %s positions"
              % (len(bad), field))
    check("joint_states_messages", j_count >= MIN_MESSAGES,
          "%d messages (need >= %d)" % (j_count, MIN_MESSAGES))
    check("joint_states_rate", j_rate >= MIN_RATE_HZ,
          "%.1f Hz (need >= %.0f)" % (j_rate, MIN_RATE_HZ))
    check("joint_states_stamps_advance", j_count > 0 and j_bad == 0,
          "%d non-advancing stamps" % j_bad)
    bad_joint_rows = [
        i for i, row in enumerate(joint_rows)
        if not set(JOINTS).issubset(set(row["names"] or ()))
        or not all(math.isfinite(float(v)) for v in row["position"])]
    check("joint_states_names_finite", j_count > 0 and not bad_joint_rows,
          "%d messages missing a joint or with non-finite position"
          % len(bad_joint_rows))

    if c_count and j_count:
        overlap = (min(c_stamps[-1], j_stamps[-1])
                   - max(c_stamps[0], j_stamps[0]))
    else:
        overlap = 0.0
    check("streams_overlap", overlap >= MIN_OVERLAP_SEC,
          "%.2f s of simultaneous sim time (need >= %.1f)"
          % (overlap, MIN_OVERLAP_SEC))
    metrics["overlap_sec"] = round(overlap, 3)
    for label, rows in (("controller_state", controller_rows),
                        ("joint_states", joint_rows)):
        age = (end_mono - rows[-1]["rx_mono"]) if rows else float("inf")
        check("%s_still_arriving" % label, age <= MAX_FINAL_AGE_SEC,
              "newest message %.2f s before the window ended" % age)
    # Informational: idle arm, so measured and controller feedback agree.
    deltas = []
    if controller_rows and joint_rows:
        by_stamp = sorted((row["stamp"], row) for row in joint_rows)
        row = controller_rows[-1]
        nearest = min(by_stamp, key=lambda item: abs(item[0] - row["stamp"]))
        index = {n: v for n, v in zip(nearest[1]["names"],
                                      nearest[1]["position"])}
        if all(n in index for n in JOINTS) and _finite_six(
                row.get("feedback") or []):
            names = list(row["names"])
            fb = dict(zip(names, row["feedback"]))
            deltas = [abs(fb[n] - index[n]) for n in JOINTS if n in fb]
    metrics["feedback_vs_joint_states_max_delta_rad"] = (
        None if not deltas else round(max(deltas), 6))
    failed = [item["check"] for item in checks if not item["ok"]]
    return {
        "ok": not failed,
        "failed": failed,
        "checks": checks,
        "metrics": metrics,
    }


def _probe(window_sec, timeout_sec):
    import rclpy
    from control_msgs.msg import JointTrajectoryControllerState
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = Node("capture_preflight")
    controller_rows = []
    joint_rows = []

    def on_state(msg):
        controller_rows.append({
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "rx_mono": time.monotonic(),
            "names": list(msg.joint_names),
            "reference": list(msg.reference.positions),
            "feedback": list(msg.feedback.positions),
            "error": list(msg.error.positions),
        })

    def on_joints(msg):
        joint_rows.append({
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "rx_mono": time.monotonic(),
            "names": list(msg.name),
            "position": list(msg.position),
        })

    qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(
        JointTrajectoryControllerState, CONTROLLER_STATE_TOPIC, on_state, qos)
    node.create_subscription(JointState, JOINT_STATES_TOPIC, on_joints, qos)
    # Wait for the first message on both streams, then listen for the window.
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and not (
            controller_rows and joint_rows):
        rclpy.spin_once(node, timeout_sec=0.1)
    window_start = time.monotonic()
    controller_rows.clear()
    joint_rows.clear()
    while time.monotonic() - window_start < window_sec:
        rclpy.spin_once(node, timeout_sec=0.05)
    end = time.monotonic()
    node.destroy_node()
    rclpy.shutdown()
    return controller_rows, joint_rows, end


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window-sec", type=float, default=3.0)
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    args = parser.parse_args(argv)
    try:
        controller_rows, joint_rows, end = _probe(
            args.window_sec, args.timeout_sec)
    except Exception as exc:  # noqa: BLE001 - report, never crash silently
        print(json.dumps({"ok": False, "failed": ["probe_error"],
                          "error": "%s: %s" % (type(exc).__name__, exc)}))
        return 1
    verdict = evaluate_capture(controller_rows, joint_rows, end)
    verdict["topics"] = {
        "controller_state": CONTROLLER_STATE_TOPIC,
        "joint_states": JOINT_STATES_TOPIC,
    }
    verdict["window_sec"] = args.window_sec
    print(json.dumps(verdict, sort_keys=True))
    return 0 if verdict["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
