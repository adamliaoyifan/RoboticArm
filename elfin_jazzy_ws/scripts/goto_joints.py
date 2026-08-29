#!/usr/bin/env python3
"""Send an FJT goal with explicit joint values (usage: goto_joints.py v1..v6 [duration])."""
import sys, time, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from action_msgs.msg import GoalStatus

JOINTS = ["elfin_joint1","elfin_joint2","elfin_joint3","elfin_joint4","elfin_joint5","elfin_joint6"]
values = [float(v) for v in sys.argv[1:7]]
duration = float(sys.argv[7]) if len(sys.argv) > 7 else 6.0

rclpy.init()
node = Node("goto_joints")
from rclpy.action import ActionClient
client = ActionClient(node, FollowJointTrajectory, "/elfin_arm_controller/follow_joint_trajectory")
executor = SingleThreadedExecutor(); executor.add_node(node)
t = threading.Thread(target=executor.spin, daemon=True); t.start()
if not client.wait_for_server(timeout_sec=20.0):
    print("server unavailable"); sys.exit(1)
goal = FollowJointTrajectory.Goal()
traj = JointTrajectory(); traj.joint_names = JOINTS
p = JointTrajectoryPoint(); p.positions = values; p.velocities = [0.0]*6
sec = int(duration); p.time_from_start = Duration(sec=sec, nanosec=int((duration-sec)*1e9))
traj.points = [p]; goal.trajectory = traj
goal.goal_time_tolerance = Duration(sec=2, nanosec=0)
fut = client.send_goal_async(goal)
ev = threading.Event(); fut.add_done_callback(lambda f: ev.set()); ev.wait(20.0)
handle = fut.result()
if not handle.accepted:
    print("goal rejected"); sys.exit(1)
rf = handle.get_result_async(); ev2 = threading.Event(); rf.add_done_callback(lambda f: ev2.set()); ev2.wait(duration+60.0)
res = rf.result()
print("status=%s error_code=%s" % (res.status, res.result.error_code))
sys.exit(0 if res.status == GoalStatus.STATUS_SUCCEEDED and res.result.error_code == 0 else 1)
