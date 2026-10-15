#!/usr/bin/env python3
"""Step-2 edge-level diagnostic (review 2026-10-15_1330 protocol).

Replays ONE online-failed PORTAL_TRANSIT edge (portal_01 of score_run02):
the exact captured start_q, goal constraints, attached b-payload, world
scene and ACM are applied to an isolated MoveIt stack, then the same
MoveGroup action (/move_action, RRTConnect, 2.0 s) is called N times.
Per attempt: the RAW moveit error code, planning time, waypoint count,
and a production-style validation of the returned path against the same
frozen scene (ValidateRouteV2) marking the first invalid sample.

Diagnostic sample only: no reliability signature, no retry policy.
usage: portal_edge_replay.py <out_dir> [portal_index=1] [n=10]
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, "/tmp/elfin-master-pack-baseline-g3/scripts/g2")
import g2_common as common  # noqa: E402

RUN = ("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
       "master_pack_recovery/2026-10-14_g3/score_run02")


def main():
    out_dir = sys.argv[1]
    portal_index = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 10
    os.makedirs(out_dir, exist_ok=True)

    import rclpy
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message
    from moveit_msgs.msg import PlanningScene
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint, RobotState
    from luggage_msgs.srv import ValidateRouteV2
    from luggage_planning.planning_scene_client import PlanningSceneClient

    rec = json.load(open(os.path.join(
        RUN, "portal_capture", "portal_%02d.json" % portal_index)))
    scene = deserialize_message(open(os.path.join(
        RUN, "portal_capture", "portal_%02d.scene.bin" % portal_index),
        "rb").read(), PlanningScene)

    stack = common.MoveItStack(out_dir, "portal_edge")
    stack.start()
    rclpy.init()
    node = Node("g3_portal_edge_replay")
    stop, thread = common.score._spin(node)  # noqa: SLF001
    report = {"portal": portal_index, "n": n, "attempts": []}
    try:
        scene_client = PlanningSceneClient(node)
        scene_client.wait_ready(timeout_sec=30.0)
        applied = scene_client.apply_diff(scene)
        report["scene_applied"] = bool(getattr(applied, "success", applied))

        action = node.create_client(
            MoveGroup, "/move_action") if False else None
        from rclpy.action import ActionClient
        action = ActionClient(node, MoveGroup, "/move_action")
        if not action.wait_for_server(timeout_sec=60.0):
            raise SystemExit("move_action unavailable")
        validate = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not validate.wait_for_service(timeout_sec=60.0):
            raise SystemExit("validate_route_v2 unavailable")

        goal = MoveGroup.Goal()
        goal.request.group_name = "elfin_arm"
        goal.request.planner_id = "RRTConnect"
        goal.request.allowed_planning_time = 2.0
        goal.request.num_planning_attempts = 1
        start = RobotState()
        start.joint_state.name = [
            "elfin_joint%d" % (i + 1) for i in range(6)]
        start.joint_state.position = list(rec["start_q"])
        # Fidelity fix: carry the attached b-payload (and its touch links)
        # in the request start state, exactly as the captured online scene
        # had it — MoveGroup replaces the full start state, so an empty
        # attachment list plans payload-less and invalidates the A/B.
        start.attached_collision_objects = list(
            scene.robot_state.attached_collision_objects)
        start.is_diff = True
        goal.request.start_state = start
        constraints = Constraints()
        constraints.name = "portal_edge_replay"
        for spec in rec["goal_constraints"]:
            jc = JointConstraint()
            jc.joint_name = spec["joint_name"]
            jc.position = float(spec["position"])
            jc.tolerance_above = float(spec["tolerance_above"])
            jc.tolerance_below = float(spec["tolerance_below"])
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)

        for attempt in range(n):
            entry = {"attempt": attempt}
            t0 = time.perf_counter()
            future = action.send_goal_async(goal)
            end = time.time() + 30.0
            while not future.done() and time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            handle = future.result()
            entry["accepted"] = bool(handle.accepted)
            result_future = handle.get_result_async()
            end = time.time() + 30.0
            while not result_future.done() and time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            result = result_future.result().result
            entry["wall_s"] = round(time.perf_counter() - t0, 3)
            entry["error_code"] = int(result.error_code.val)
            traj = result.planned_trajectory.joint_trajectory
            entry["n_waypoints"] = len(traj.points)
            # production-style validation of the returned path
            if traj.points:
                from luggage_msgs.msg import RouteLegV2
                leg = RouteLegV2()
                leg.edge_id = "replay_%02d" % attempt
                leg.edge_class = "PORTAL_TRANSIT"
                leg.carrying = True
                leg.joint_names = list(traj.joint_names)
                times = [0.02 * (i + 1) for i in range(len(traj.points))]
                leg.n_samples = len(traj.points)
                leg.sample_times_ns = [float(t) for t in times]
                leg.sample_positions_flat = [
                    float(v) for p in traj.points for v in p.positions]
                vreq = ValidateRouteV2.Request()
                vreq.request_id = "edge-replay-%02d" % attempt
                vreq.load_state_scenes = [scene]
                vreq.max_sample_dt_ns = 0.0
                vreq.budget_s = 5.0
                vreq.legs.append(leg)
                vfuture = validate.call_async(vreq)
                end = time.time() + 30.0
                while not vfuture.done() and time.time() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
                vresp = vfuture.result()
                entry["returned_path_all_valid"] = bool(vresp.all_valid)
                bad = [l for l in vresp.leg_results if not l.valid]
                if bad:
                    entry["first_invalid_sample"] = int(
                        bad[0].first_invalid_sample)
                    entry["invalid_reason"] = str(bad[0].reason)
                    entry["contacts"] = [
                        [str(c.body_1), str(c.body_2), round(float(c.depth), 5)]
                        for c in getattr(bad[0], "contacts", [])][:4]
            report["attempts"].append(entry)
            print(json.dumps(entry))
        with open(os.path.join(out_dir, "edge_replay.json"), "w") as h:
            json.dump(report, h, indent=1)
        ok = sum(1 for a in report["attempts"]
                 if a["error_code"] == 1
                 and a.get("returned_path_all_valid", False))
        print("SUMMARY success=%d/%d" % (ok, n))
        return 0
    finally:
        stop.set()
        thread.join(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown()
        stack.stop()


if __name__ == "__main__":
    sys.exit(main())
