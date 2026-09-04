#!/usr/bin/env python3
"""ROS node: MoveIt reachability/collision filter for placement candidates.

Reads the candidate list published by ``placement_planner`` on
``/luggage/placement/candidates``, evaluates each candidate's place pose with
MoveIt ``/compute_ik`` (collision-aware), annotates a ``reachability_score`` and
optionally demotes unreachable candidates to infeasible, then writes the list
back and re-publishes the RViz markers.

This is a pure verifier: it never invents poses, only checks feasibility, so the
failure reason is explainable (``ik_unreachable``).
"""

from __future__ import division

import math
import os
import sys
import time

import rospy
import rospkg
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import SetBool

PACK_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_packing"), "scripts")
if PACK_SCRIPTS not in sys.path:
    sys.path.insert(0, PACK_SCRIPTS)

from placement_markers import build_candidate_markers  # noqa: E402


class PlacementMotionFilterNode:
    def __init__(self):
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")
        self._group = rospy.get_param("~move_group", "elfin_arm")
        # Must match execution semantics: place targets are suction contact
        # poses, not link6 poses.
        self._ik_link = rospy.get_param("~ik_link", "suction_contact_frame")
        self._ik_service = rospy.get_param("~ik_service", "/compute_ik")
        self._ik_timeout = float(rospy.get_param("~ik_timeout", 0.2))
        self._ik_attempts = int(rospy.get_param("~ik_attempts", 3))
        self._avoid_collisions = bool(rospy.get_param("~avoid_collisions", True))
        # Contact frame is at the suitcase top for the final placement; transit
        # is checked above it before insertion.
        self._approach_offset = float(rospy.get_param("~approach_offset", 0.0))
        self._transit_clearance = float(
            rospy.get_param("~transit_clearance", 0.30))
        self._stack_transit_clearance = float(
            rospy.get_param("~stack_transit_clearance", 0.15))
        # When true, unreachable candidates are demoted to infeasible.
        self._hard_filter = bool(rospy.get_param("~hard_filter", True))
        self._candidates_param = rospy.get_param(
            "~candidates_param", "/luggage/placement/candidates"
        )
        self._wait_timeout = float(rospy.get_param("~ik_wait_timeout", 15.0))
        self._deadline_sec = float(rospy.get_param("~deadline_sec", 2.0))
        self._max_backups = int(rospy.get_param("~max_backups", 8))

        self._markers_pub = rospy.Publisher(
            "/luggage/placement/candidate_markers", MarkerArray, queue_size=1, latch=True
        )

        self._seed_wait_timeout = float(rospy.get_param(
            "~seed_state_wait_timeout", 1.0))
        self._seed_state = None
        self._ik = None
        self._set_support_touch = None
        try:
            rospy.wait_for_service(self._ik_service, timeout=self._wait_timeout)
            self._ik = rospy.ServiceProxy(self._ik_service, GetPositionIK)
        except rospy.ROSException:
            rospy.logwarn(
                "compute_ik service %s unavailable; motion filter will pass-through",
                self._ik_service,
            )

        rospy.Service("~filter_placements", Trigger, self.handle_filter)

    @staticmethod
    def _tool_down_quaternion(yaw):
        """Quaternion with tool z-axis pointing down (-Z) at the requested yaw."""
        c = math.cos(yaw * 0.5)
        s = math.sin(yaw * 0.5)
        return Quaternion(x=c, y=s, z=0.0, w=0.0)

    def _place_pose(self, cand, extra_z=0.0):
        center = cand["center_base"]
        top_z = (
            center[2] + cand["size"][2] * 0.5
            + self._approach_offset + float(extra_z)
        )
        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.pose = Pose(
            position=Point(x=center[0], y=center[1], z=top_z),
            orientation=self._tool_down_quaternion(cand.get("yaw", 0.0)),
        )
        return pose

    def _current_robot_state(self):
        """Seed state from the live joint states, refreshed once per filter pass.

        Passing a default-constructed RobotState makes MoveIt log
        "Found empty JointState message" at ERROR for every single IK call --
        55 of them in one loading run, which buries real errors. Seeding with
        the actual arm state removes the noise and gives IK a start state near
        the robot instead of the model default.
        """
        if self._seed_state is not None:
            return self._seed_state
        try:
            joint_state = rospy.wait_for_message(
                "/joint_states", JointState, timeout=self._seed_wait_timeout)
        except rospy.ROSException:
            rospy.logwarn_throttle(
                30.0, "no /joint_states for IK seeding; using model default")
            return RobotState(joint_state=JointState(
                name=[], position=[]))
        self._seed_state = RobotState(joint_state=joint_state)
        return self._seed_state

    def _solve_ik(self, pose, seed_state=None):
        if self._ik is None:
            return None
        req = PositionIKRequest()
        req.group_name = self._group
        req.ik_link_name = self._ik_link
        req.pose_stamped = pose
        req.avoid_collisions = self._avoid_collisions
        req.timeout = rospy.Duration(self._ik_timeout)
        req.robot_state = seed_state or self._current_robot_state()
        for _ in range(max(1, self._ik_attempts)):
            try:
                resp = self._ik(req)
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(5.0, "compute_ik failed: %s", exc)
                return False
            if resp.error_code.val == resp.error_code.SUCCESS:
                # Preserve the full RobotState (including attached bodies when
                # supplied by MoveIt) as the seed for the next chain stage.
                return resp.solution
        return False

    def _check_ik(self, cand):
        """Check transit then final contact pose on one continuous IK branch."""
        clearance = (
            self._stack_transit_clearance
            if float(cand.get("peak", 0.0)) > 0.01
            else self._transit_clearance)
        transit = self._place_pose(cand, extra_z=clearance)
        place = self._place_pose(cand)
        transit_state = self._solve_ik(transit)
        if transit_state is None:
            return None, "ik_unavailable"
        if transit_state is False:
            return False, "transit_ik_unreachable"
        place_state = self._solve_ik(place, seed_state=transit_state)
        if place_state is False:
            return False, "place_ik_unreachable"
        return True, "ok"

    def _filter(self):
        candidates = rospy.get_param(self._candidates_param, [])
        if not candidates:
            rospy.set_param("/luggage/workspace/placement_summary", {
                "candidate_count": 0,
                "checked": 0,
                "reachable": 0,
                "payload_attached": bool(rospy.get_param(
                    "/luggage/vacuum/attached", False)),
                "failure_reasons": {"no_candidates": 1},
            })
            return candidates, "no candidates to filter"

        reachable = 0
        checked = 0
        failure_reasons = {}
        # Re-read the arm state once per pass; candidates within a pass share it.
        self._seed_state = None
        started = time.monotonic()
        current_revision = int(rospy.get_param(
            "/luggage/cargo_map/stats/map_revision", -1))
        no_good = set(rospy.get_param(
            "/luggage/placement/no_good_ids", []))
        for cand in candidates:
            if not cand.get("feasible", False):
                cand["reachability_score"] = -1.0
                continue
            candidate_id = str(cand.get("candidate_id", ""))
            if candidate_id and candidate_id in no_good:
                cand["feasible"] = False
                cand["reachability_score"] = 0.0
                cand["reachability_reason"] = "no_good"
                cand["reason"] = "no_good"
                failure_reasons["no_good"] = (
                    failure_reasons.get("no_good", 0) + 1)
                continue
            candidate_revision = int(cand.get("map_revision", -1))
            if (
                    candidate_revision >= 0
                    and current_revision >= 0
                    and candidate_revision != current_revision):
                cand["feasible"] = False
                cand["reachability_score"] = 0.0
                cand["reachability_reason"] = "stale_map_revision"
                cand["reason"] = "stale_map_revision"
                failure_reasons["stale_map_revision"] = (
                    failure_reasons.get("stale_map_revision", 0) + 1)
                continue
            if time.monotonic() - started > self._deadline_sec:
                cand["reachability_score"] = -1.0
                cand["reachability_reason"] = "deadline_exhausted"
                failure_reasons["deadline_exhausted"] = (
                    failure_reasons.get("deadline_exhausted", 0) + 1)
                continue
            result, reason = self._check_ik(cand)
            if result is None:
                cand["reachability_score"] = -1.0
                cand["reachability_reason"] = reason
                continue
            checked += 1
            cand["reachability_score"] = 1.0 if result else 0.0
            cand["reachability_reason"] = reason
            if result:
                reachable += 1
            elif self._hard_filter:
                cand["feasible"] = False
                cand["reason"] = reason
            if not result:
                failure_reasons[reason] = failure_reasons.get(reason, 0) + 1

        # Re-sort feasible-first, reachable preferred, then by score.
        candidates.sort(
            key=lambda c: (
                not c.get("feasible", False),
                -(c.get("reachability_score", -1.0)),
                -c.get("score", 0.0),
            )
        )
        rospy.set_param(self._candidates_param, candidates)
        backups = [
            cand for cand in candidates
            if cand.get("feasible", False)
            and cand.get("reachability_score", -1.0) > 0.0
        ][:self._max_backups]
        rospy.set_param("/luggage/placement/backups", backups)
        best = backups[0] if backups else {}
        rospy.set_param("/luggage/placement/best", best)
        rospy.set_param("/luggage/workspace/placement_summary", {
            "candidate_count": len(candidates),
            "checked": checked,
            "reachable": reachable,
            "reachable_ratio": (
                float(reachable) / float(checked) if checked else 0.0),
            "payload_attached": bool(rospy.get_param(
                "/luggage/vacuum/attached", False)),
            "ik_link": self._ik_link,
            "failure_reasons": failure_reasons,
            "map_revision": current_revision,
            "elapsed_sec": round(time.monotonic() - started, 4),
            "backup_count": len(backups),
        })
        self._markers_pub.publish(
            build_candidate_markers(candidates, self._base_frame, rospy.Time.now())
        )
        if self._ik is None:
            return candidates, "ik unavailable; %d candidates passed through" % len(candidates)
        return candidates, "checked %d, reachable %d" % (checked, reachable)

    def handle_filter(self, _req):
        try:
            if self._set_support_touch is None:
                rospy.wait_for_service(
                    "/scene_manager/set_place_support_touch", timeout=1.0)
                self._set_support_touch = rospy.ServiceProxy(
                    "/scene_manager/set_place_support_touch", SetBool)
            self._set_support_touch(True)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn(
                "placement filter support-touch ACM unavailable: %s", exc)
        try:
            candidates, message = self._filter()
        finally:
            if self._set_support_touch is not None:
                try:
                    self._set_support_touch(False)
                except rospy.ServiceException:
                    pass
        return TriggerResponse(success=bool(candidates), message=message)



# Log level must be chosen before init_node, so it cannot come from a private
# param; log_level_utils reads the LUGGAGE_LOG_LEVEL environment variable.
import os as _os
import sys as _sys
import rospkg as _rospkg
_DESC = _os.path.join(
    _rospkg.RosPack().get_path("luggage_description"), "scripts")
if _DESC not in _sys.path:
    _sys.path.insert(0, _DESC)
from log_level_utils import resolve_log_level  # noqa: E402

def main():
    rospy.init_node("placement_motion_filter", log_level=resolve_log_level())
    PlacementMotionFilterNode()
    rospy.loginfo("placement_motion_filter ready")
    rospy.spin()


if __name__ == "__main__":
    main()
