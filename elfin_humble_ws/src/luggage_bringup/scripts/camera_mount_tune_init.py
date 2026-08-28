#!/usr/bin/env python3
"""Set arm to observe pose (+ mount joints) after Gazebo model and controllers are ready."""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import actionlib
import rospy
import rospkg
import yaml
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelConfiguration
from mount_config_utils import mount_dict_to_tune_joints
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
if DESC_SCRIPTS not in sys.path:
    sys.path.insert(0, DESC_SCRIPTS)

from joint_angle_utils import (  # noqa: E402
    WRAP_EQUIVALENT_JOINTS,
    format_rewrites,
    max_joint_error,
    normalize_joint_targets,
)
from gazebo_urdf_utils import GAZEBO_MODEL_URDF_PARAM  # noqa: E402

ARM_JOINT_NAMES = [
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
]

MOUNT_JOINT_NAMES = [
    "cam_mount_tx",
    "cam_mount_ty",
    "cam_mount_tz",
    "cam_mount_rx",
    "cam_mount_ry",
    "cam_mount_rz",
]

ARM_CONTROLLER_ACTION_DEFAULT = "/S20/elfin_arm_controller/follow_joint_trajectory"
ARM_CONTROLLER_NAME = "elfin_arm_controller"


def arm_controller_action():
    return rospy.get_param("~arm_controller_action", ARM_CONTROLLER_ACTION_DEFAULT)

DEFAULT_MOUNT_JOINTS = [
    -0.017202,
    0.129806,
    0.101650,
    -1.57079632679,
    -1.57079632679,
    1.57079632679,
]


def _pkg_config():
    return os.path.join(rospkg.RosPack().get_path("luggage_description"), "config")


def load_observe_arm_pose():
    path = rospy.get_param(
        "~robot_poses_config",
        os.path.join(_pkg_config(), "robot_poses.yaml.example"),
    )
    pose_name = rospy.get_param("~observe_pose_name", "observe")
    with open(path, "r") as handle:
        config = yaml.safe_load(handle)
    pose = config["poses"][pose_name]
    joints = list(pose["joints"])
    values = [float(v) for v in pose["values"]]
    if len(joints) != len(values):
        raise ValueError("observe pose joints/values length mismatch")
    tolerance = float(rospy.get_param("~observe_tolerance", pose.get("tolerance", 0.02)))
    return joints, values, tolerance


def load_mount_pose():
    path = rospy.get_param(
        "~mount_config",
        os.path.join(_pkg_config(), "realsense_d435_mount.yaml.example"),
    )
    with open(path, "r") as handle:
        mount = yaml.safe_load(handle).get("mount", {})
    try:
        return mount_dict_to_tune_joints(mount)
    except (KeyError, TypeError, ValueError):
        return list(DEFAULT_MOUNT_JOINTS)


def wait_for_gazebo_model(model_name, timeout_sec):
    deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
    rate = rospy.Rate(5)
    while rospy.Time.now() < deadline and not rospy.is_shutdown():
        try:
            msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=2.0)
            if model_name in msg.name:
                rospy.loginfo("Gazebo model %s is spawned", model_name)
                return True
        except rospy.ROSException:
            pass
        rate.sleep()
    return False


def wait_for_arm_controller(timeout_sec):
    action = arm_controller_action()
    client = actionlib.SimpleActionClient(action, FollowJointTrajectoryAction)
    if client.wait_for_server(rospy.Duration(timeout_sec)):
        rospy.loginfo("Arm controller action server ready at %s", action)
        return client
    return None


def wait_for_joint_positions(joint_names, timeout_sec):
    deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
    missing = list(joint_names)
    while rospy.Time.now() < deadline and not rospy.is_shutdown():
        try:
            msg = rospy.wait_for_message("/joint_states", JointState, timeout=1.0)
        except rospy.ROSException:
            continue
        positions = dict(zip(msg.name, msg.position))
        missing = [name for name in joint_names if name not in positions]
        if not missing:
            return [positions[name] for name in joint_names], ""
    return None, "missing joints in /joint_states: %s" % ", ".join(missing)


def max_joint_error_for_pose(joint_names, current_positions, target_positions):
    """Wrap-aware max joint error (delegates to joint_angle_utils)."""
    return max_joint_error(
        joint_names,
        current_positions,
        target_positions,
        wrap_joints=WRAP_EQUIVALENT_JOINTS,
    )


def _controller_manager_prefix():
    ns = str(rospy.get_param("~controller_manager_ns", "S20")).strip("/")
    return "/%s/controller_manager" % ns


def _switch_arm_controller(start):
    """Start or stop elfin_arm_controller; keep joint_state_controller running."""
    cm_prefix = _controller_manager_prefix()
    switch_srv = "%s/switch_controller" % cm_prefix
    try:
        rospy.wait_for_service(switch_srv, timeout=10.0)
    except rospy.ROSException as exc:
        rospy.logerr("controller_manager unavailable at %s: %s", cm_prefix, exc)
        return False
    switch_controller = rospy.ServiceProxy(switch_srv, SwitchController)
    if start:
        resp = switch_controller(
            start_controllers=[ARM_CONTROLLER_NAME],
            stop_controllers=[],
            strictness=SwitchControllerRequest.BEST_EFFORT,
            start_asap=False,
            timeout=5.0,
        )
    else:
        resp = switch_controller(
            start_controllers=[],
            stop_controllers=[ARM_CONTROLLER_NAME],
            strictness=SwitchControllerRequest.BEST_EFFORT,
            start_asap=False,
            timeout=5.0,
        )
    return bool(resp.ok)


def _gazebo_set_model_configuration_service():
    srv_name = rospy.get_param(
        "~gazebo_set_model_configuration_service", "/gazebo/set_model_configuration"
    )
    rospy.wait_for_service(srv_name, timeout=10.0)
    return rospy.ServiceProxy(srv_name, SetModelConfiguration)


def _call_empty_service(param_name, default_service, timeout=10.0):
    srv_name = rospy.get_param(param_name, default_service)
    rospy.wait_for_service(srv_name, timeout=timeout)
    srv = rospy.ServiceProxy(srv_name, Empty)
    srv()


def _pause_physics(timeout=10.0):
    _call_empty_service(
        "~gazebo_pause_physics_service", "/gazebo/pause_physics", timeout=timeout
    )


def _unpause_physics(timeout=10.0):
    _call_empty_service(
        "~gazebo_unpause_physics_service", "/gazebo/unpause_physics", timeout=timeout
    )


def _verify_observe_at_goal(arm_joints, arm_values, tolerance, timeout_sec=5.0):
    current_values, msg = wait_for_joint_positions(arm_joints, timeout_sec)
    if current_values is None:
        return False, msg
    err = max_joint_error_for_pose(arm_joints, current_values, arm_values)
    if err > tolerance:
        parts = []
        for name, cur, target in zip(arm_joints, current_values, arm_values):
            joint_err = max_joint_error_for_pose([name], [cur], [target])
            parts.append("%s cur=%.4f target=%.4f err=%.4f" % (
                name, cur, target, joint_err
            ))
        return False, "observe verify failed: max error %.4f > %.4f (%s)" % (
            err, tolerance, "; ".join(parts)
        )
    return True, "observe verified (max error %.4f)" % err


def _log_joint_pose_error(arm_joints, arm_values, label):
    current, msg = wait_for_joint_positions(arm_joints, 2.0)
    if current is None:
        rospy.logwarn("%s: cannot read /joint_states: %s", label, msg)
        return None, None
    err = max_joint_error_for_pose(arm_joints, current, arm_values)
    rospy.loginfo("%s: wrap-aware max error %.4f", label, err)
    return current, err


def hard_recover_observe_pose(
    model_name,
    arm_joints,
    arm_values,
    observe_tolerance,
    controller_timeout,
):
    """Stop arm controller, Gazebo snap, restart, trajectory to observe.

    switch_controller only completes once the controller_manager update loop
    runs, which is driven by Gazebo stepping — so it must NOT be called while
    physics is paused (the service call deadlocks under a paused world). The
    set_model_configuration snap, by contrast, works fine while paused.
    Sequence: stop (running), pause, snap, unpause, start controller,
    trajectory from actual joint state.
    """
    hold_duration = float(rospy.get_param("~hard_recover_hold_duration", 2.0))
    motion_duration = float(
        rospy.get_param(
            "~hard_recover_motion_duration",
            rospy.get_param("~observe_motion_duration", 6.0),
        )
    )
    max_attempts = 2
    set_model_config = _gazebo_set_model_configuration_service()

    for attempt in range(1, max_attempts + 1):
        controller_stopped = False
        controller_restarted = False
        physics_paused = False
        snap_ok = False
        try:
            if not _switch_arm_controller(start=False):
                return False, "hard recover: failed to stop %s" % ARM_CONTROLLER_NAME
            controller_stopped = True
            rospy.loginfo("hard recover: stopped arm controller (attempt %d)", attempt)

            _pause_physics()
            physics_paused = True
            rospy.loginfo("hard recover: paused Gazebo physics (attempt %d)", attempt)

            ok, status = set_gazebo_joints(
                set_model_config, model_name, arm_joints, arm_values
            )
            snap_ok = ok
            if not ok:
                return False, "hard recover: gazebo snap failed: %s" % status
            rospy.loginfo("hard recover: gazebo snap ok")

            # Unpause immediately after the snap, before anything that reads
            # /joint_states or starts a controller. Both depend on a stepping
            # world: wait_for_joint_positions loops on a sim-time deadline that
            # never elapses under a frozen /clock, and switch_controller is only
            # applied by the Gazebo-driven controller_manager update loop.
            # Leaving physics paused here hangs hard recover, freezes /clock,
            # and robot_state_publisher emits no arm TF (empty TF tree).
            _unpause_physics()
            physics_paused = False
            rospy.loginfo("hard recover: unpaused Gazebo physics")

            _log_joint_pose_error(
                arm_joints, arm_values, "hard recover: post-snap"
            )

            if _switch_arm_controller(start=True):
                controller_restarted = True
                controller_stopped = False
                rospy.loginfo(
                    "hard recover: started arm controller after unpause (attempt %d)",
                    attempt,
                )
        except rospy.ROSException as exc:
            return False, "hard recover: Gazebo service failed: %s" % exc
        finally:
            if physics_paused:
                try:
                    _unpause_physics()
                    rospy.loginfo("hard recover: unpaused Gazebo physics (cleanup)")
                except rospy.ROSException as exc:
                    if controller_stopped:
                        _switch_arm_controller(start=True)
                    return False, "hard recover: failed to unpause physics: %s" % exc

        if not snap_ok:
            if controller_stopped:
                _switch_arm_controller(start=True)
            return False, "hard recover: gazebo snap failed"

        if not controller_restarted:
            if not _switch_arm_controller(start=True):
                return False, "hard recover: failed to start %s" % ARM_CONTROLLER_NAME
            rospy.loginfo("hard recover: started arm controller")

        rospy.sleep(0.2)

        client = wait_for_arm_controller(controller_timeout)
        if client is None:
            return False, "hard recover: action server unavailable after restart"

        current_values, pre_err = _log_joint_pose_error(
            arm_joints, arm_values, "hard recover: pre-trajectory"
        )
        if current_values is None:
            return False, "hard recover: no /joint_states after controller restart"

        hold_targets = _observe_targets_with_wrap(
            arm_joints, current_values, arm_values
        )
        move_duration = (
            hold_duration
            if pre_err <= observe_tolerance
            else max(hold_duration, motion_duration)
        )
        if pre_err > observe_tolerance:
            rospy.loginfo(
                "hard recover: driving to observe (error %.4f) over %.1fs",
                pre_err,
                move_duration,
            )

        ok, traj_msg = move_arm_via_controller(
            client,
            arm_joints,
            hold_targets,
            current_positions=current_values,
            duration_sec=move_duration,
        )
        if not ok:
            rospy.logwarn(
                "hard recover: hold trajectory failed (attempt %d): %s",
                attempt,
                traj_msg,
            )
            continue
        rospy.loginfo("hard recover: hold trajectory succeeded")

        verify_ok, verify_msg = _verify_observe_at_goal(
            arm_joints, arm_values, observe_tolerance
        )
        if verify_ok:
            return True, verify_msg
        rospy.logwarn("hard recover: %s (attempt %d)", verify_msg, attempt)

    return False, "hard recover failed after %d attempts" % max_attempts


def move_arm_via_controller(
    client,
    joint_names,
    joint_positions,
    current_positions=None,
    duration_sec=4.0,
):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = list(joint_names)
    points = []
    if current_positions is not None:
        start = JointTrajectoryPoint()
        start.positions = list(current_positions)
        start.time_from_start = rospy.Duration(0.0)
        points.append(start)
    point = JointTrajectoryPoint()
    point.positions = list(joint_positions)
    point.time_from_start = rospy.Duration(duration_sec)
    points.append(point)
    goal.trajectory.points = points
    client.send_goal(goal)
    finished = client.wait_for_result(rospy.Duration(duration_sec + 10.0))
    if not finished:
        client.cancel_goal()
        return False, "arm trajectory timed out"
    state = client.get_state()
    if state != actionlib.GoalStatus.SUCCEEDED:
        return False, "arm trajectory state=%s" % state
    return True, "ok"


def _observe_targets_with_wrap(arm_joints, current_values, arm_values):
    """Normalize observe targets onto the wrap branch closest to current.

    Returns the target list to send to the controller (after wrap snapping).
    Logs any rewrites so the spawn/corrective branch mismatch is visible.
    """
    normalized_values, rewrites = normalize_joint_targets(
        arm_joints, current_values, arm_values,
        wrap_joints=WRAP_EQUIVALENT_JOINTS,
    )
    if rewrites:
        rospy.logwarn(
            "observe corrective: snapping wrap joints: %s",
            format_rewrites(rewrites),
        )
    return normalized_values


def set_gazebo_joints(set_model_config, model_name, joint_names, joint_positions):
    urdf_param_name = (
        GAZEBO_MODEL_URDF_PARAM
        if rospy.has_param(GAZEBO_MODEL_URDF_PARAM)
        else "robot_description"
    )
    resp = set_model_config(
        model_name=model_name,
        urdf_param_name=urdf_param_name,
        joint_names=joint_names,
        joint_positions=joint_positions,
    )
    return resp.success, resp.status_message


def publish_mount_joint_states(positions):
    pub = rospy.Publisher("/cam_mount_tune/joint_states", JointState, queue_size=1, latch=True)
    rospy.sleep(0.2)
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = list(MOUNT_JOINT_NAMES)
    msg.position = list(positions)
    pub.publish(msg)


def apply_observe_pose(
    model_name="S20",
    apply_mount=True,
    gazebo_timeout=120.0,
    controller_timeout=120.0,
):
    """Wait for sim, move arm via ros_control, set mount joints in Gazebo."""
    if not wait_for_gazebo_model(model_name, gazebo_timeout):
        return False, "Gazebo model %s not found" % model_name

    arm_joints, arm_values, observe_tolerance = load_observe_arm_pose()
    mount_values = load_mount_pose() if apply_mount else None

    client = wait_for_arm_controller(controller_timeout)
    if client is None:
        return False, "elfin_arm_controller action server not available"

    if rospy.get_param("~skip_arm_trajectory", False):
        rospy.loginfo("Skipping arm observe trajectory; startup spawn already set arm joints")
        skip_drift_tolerance = float(rospy.get_param("~skip_drift_tolerance", 0.15))
        always_hard_recover = bool(
            rospy.get_param("~always_hard_recover_on_start", True)
        )
        joint_state_timeout = float(rospy.get_param("~joint_state_wait_timeout", 10.0))
        current_values, msg = wait_for_joint_positions(arm_joints, joint_state_timeout)
        if current_values is None:
            return False, msg

        err = max_joint_error_for_pose(arm_joints, current_values, arm_values)
        need_recover = always_hard_recover or err > skip_drift_tolerance
        if need_recover:
            if err > skip_drift_tolerance:
                rospy.logwarn(
                    "Arm drifted from observe pose (wrap-aware max error %.4f > %.4f) "
                    "— running hard recover (stop/snap/hold)",
                    err,
                    skip_drift_tolerance,
                )
            else:
                rospy.loginfo(
                    "always_hard_recover_on_start: syncing controller to observe "
                    "(wrap-aware max error %.4f)",
                    err,
                )
            ok, recover_msg = hard_recover_observe_pose(
                model_name,
                arm_joints,
                arm_values,
                observe_tolerance,
                controller_timeout,
            )
            if not ok:
                return False, recover_msg
            rospy.loginfo("%s", recover_msg)
        else:
            rospy.loginfo(
                "Arm at observe pose (wrap-aware max error %.4f <= %.4f); "
                "no correction needed",
                err,
                skip_drift_tolerance,
            )
    else:
        settle_sec = float(rospy.get_param("~post_controller_settle", 1.0))
        if settle_sec > 0.0:
            rospy.sleep(rospy.Duration(settle_sec))

        joint_state_timeout = float(rospy.get_param("~joint_state_wait_timeout", 10.0))
        current_values, msg = wait_for_joint_positions(arm_joints, joint_state_timeout)
        if current_values is None:
            return False, msg

        err = max_joint_error_for_pose(arm_joints, current_values, arm_values)
        if err <= observe_tolerance:
            rospy.loginfo(
                "Arm already at observe pose (max error %.4f <= %.4f); skipping trajectory",
                err,
                observe_tolerance,
            )
        else:
            # Snap wrap joints (J1/J4/J5/J6) onto the 2π branch closest to the
            # current joint positions. Without this, a startup mismatch between
            # the spawned branch and the YAML-configured branch makes the
            # controller drive the joint a full turn.
            normalized_values, rewrites = normalize_joint_targets(
                arm_joints,
                current_values,
                arm_values,
                wrap_joints=WRAP_EQUIVALENT_JOINTS,
            )
            if rewrites:
                rospy.logwarn(
                    "observe trajectory: snapping wrap joints to current branch: %s",
                    format_rewrites(rewrites),
                )
            duration_sec = float(rospy.get_param("~observe_motion_duration", 4.0))
            ok, msg = move_arm_via_controller(
                client,
                arm_joints,
                normalized_values,
                current_positions=current_values,
                duration_sec=duration_sec,
            )
            if not ok:
                return False, "arm move failed: %s" % msg

    verify_ok, verify_msg = _verify_observe_at_goal(
        arm_joints, arm_values, observe_tolerance
    )
    if not verify_ok:
        return False, verify_msg
    rospy.loginfo("%s", verify_msg)

    if not apply_mount:
        return True, "arm at observe pose"

    srv_name = rospy.get_param(
        "~gazebo_set_model_configuration_service", "/gazebo/set_model_configuration"
    )
    try:
        rospy.wait_for_service(srv_name, timeout=10.0)
        set_model_config = rospy.ServiceProxy(srv_name, SetModelConfiguration)
    except rospy.ROSException:
        return False, "SetModelConfiguration unavailable for mount joints"

    ok, status = set_gazebo_joints(
        set_model_config, model_name, MOUNT_JOINT_NAMES, mount_values
    )
    if not ok:
        return False, "mount SetModelConfiguration rejected: %s" % status

    publish_mount_joint_states(mount_values)
    return True, "arm at observe pose with mount joints applied"


def _ensure_gazebo_unpaused_on_shutdown():
    """Best-effort unpause so /clock and joint_states resume after a failed init."""
    def _handler():
        try:
            timeout = float(rospy.get_param("~gazebo_service_timeout", 5.0))
            _unpause_physics(timeout)
            rospy.loginfo("shutdown: ensured Gazebo physics unpaused")
        except Exception as exc:
            rospy.logwarn("shutdown: could not unpause Gazebo physics: %s", exc)

    rospy.on_shutdown(_handler)


def main():
    try:
        rospy.init_node("camera_mount_tune_init")
        _ensure_gazebo_unpaused_on_shutdown()
        if not rospy.get_param("~set_observe_on_start", True):
            rospy.loginfo("set_observe_on_start=false — skipping observe pose")
            return

        model_name = rospy.get_param("~gazebo_model_name", "S20")
        apply_mount = bool(rospy.get_param("~apply_mount_on_start", True))
        gazebo_timeout = float(rospy.get_param("~gazebo_wait_timeout", 120.0))
        controller_timeout = float(rospy.get_param("~controller_wait_timeout", 120.0))

        rospy.loginfo(
            "Waiting for Gazebo model + arm controller before moving to observe ..."
        )
        ok, message = apply_observe_pose(
            model_name=model_name,
            apply_mount=apply_mount,
            gazebo_timeout=gazebo_timeout,
            controller_timeout=controller_timeout,
        )
        if ok:
            rospy.loginfo("%s", message)
        else:
            rospy.logerr("%s", message)
            sys.exit(1)
    except Exception as exc:
        try:
            rospy.logerr("camera_mount_tune_init failed: %s", exc)
        except Exception:
            print("camera_mount_tune_init failed:", exc, file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
