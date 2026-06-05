#!/usr/bin/env python3
"""ROS node: plan next Cargo exploration view (fixed_scan or NBV)."""

from __future__ import division

import os
import sys

import rospy
import rospkg

from luggage_msgs.srv import (
    GetCargoMapStats,
    PlanNextCargoView,
    PlanNextCargoViewResponse,
)

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
PLAN_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_planning"), "scripts")
for path in (DESC_SCRIPTS, PLAN_SCRIPTS):
    if path not in sys.path:
        sys.path.insert(0, path)

from container_config_utils import (  # noqa: E402
    exploration_joint_names,
    fixed_scan_poses,
    initial_scan_poses,
    load_exploration_config,
    nbv_candidate_poses,
    nbv_weights,
    default_exploration_path,
)
from cargo_nbv_planner import CargoNBVPlanner  # noqa: E402


class CargoExplorationPlannerNode:
    def __init__(self):
        self._config = load_exploration_config(
            rospy.get_param("~exploration_config", default_exploration_path())
        )
        self._joint_names = exploration_joint_names(self._config)
        self._fixed_poses = fixed_scan_poses(self._config)
        self._initial_poses = initial_scan_poses(self._config)
        self._unknown_threshold = float(
            rospy.get_param(
                "~unknown_threshold",
                self._config.get("unknown_threshold", 0.15),
            )
        )
        self._max_views = int(
            rospy.get_param("~max_explore_views", self._config.get("max_views", 8))
        )
        self._nbv = CargoNBVPlanner(
            nbv_candidate_poses(self._config),
            self._joint_names,
            nbv_weights(self._config),
            self._unknown_threshold,
            self._max_views,
        )
        mapper_ns = rospy.get_param("~mapper_ns", "cargo_volume_mapper")
        rospy.wait_for_service("/%s/get_cargo_map_stats" % mapper_ns)
        self._get_stats = rospy.ServiceProxy(
            "/%s/get_cargo_map_stats" % mapper_ns, GetCargoMapStats
        )
        rospy.Service("~plan_next_cargo_view", PlanNextCargoView, self.handle_plan)

    def handle_plan(self, req):
        mode = (req.mode or "fixed_scan").strip().lower()
        views_used = int(req.views_used) if req.views_used >= 0 else 0
        current = list(req.current_joint_values) if req.current_joint_values else []

        try:
            stats_resp = self._get_stats()
        except rospy.ServiceException as exc:
            return PlanNextCargoViewResponse(
                success=False,
                done=True,
                message="get_cargo_map_stats failed: %s" % exc,
                joint_values=[],
                joint_names=self._joint_names,
                view_index=-1,
            )

        if not stats_resp.success:
            return PlanNextCargoViewResponse(
                success=False,
                done=True,
                message=stats_resp.message,
                joint_values=[],
                joint_names=self._joint_names,
                view_index=-1,
            )

        stats = {
            "unknown_ratio": stats_resp.unknown_ratio,
            "frontier_count": stats_resp.frontier_count,
        }
        frontier_points = rospy.get_param("/luggage/cargo_map/frontier_points", [])

        if views_used == 0:
            self._nbv.reset()

        if mode == "nbv":
            result = self._nbv.plan_next(stats, frontier_points, current, views_used)
        elif mode == "initial_fixed_scan":
            result = self._plan_pose_sequence(
                self._initial_poses,
                stats,
                views_used,
                apply_unknown_threshold=False,
                label="initial scan",
            )
        else:
            result = self._plan_fixed_scan(stats, views_used)

        return PlanNextCargoViewResponse(
            success=not result["done"] or bool(result.get("message")),
            done=result["done"],
            message=result["message"],
            joint_values=result["joint_values"],
            joint_names=result["joint_names"],
            view_index=result["view_index"],
        )

    def _plan_fixed_scan(self, stats, views_used):
        return self._plan_pose_sequence(
            self._fixed_poses,
            stats,
            views_used,
            apply_unknown_threshold=True,
            label="fixed scan",
        )

    def _plan_pose_sequence(self, poses, stats, views_used, apply_unknown_threshold, label):
        if views_used >= len(poses) or views_used >= self._max_views:
            return {
                "done": True,
                "joint_values": [],
                "joint_names": self._joint_names,
                "view_index": -1,
                "message": "%s poses exhausted" % label,
            }
        if apply_unknown_threshold and stats["unknown_ratio"] <= self._unknown_threshold and views_used > 0:
            return {
                "done": True,
                "joint_values": [],
                "joint_names": self._joint_names,
                "view_index": -1,
                "message": "unknown below threshold",
            }

        pose = poses[views_used]
        return {
            "done": False,
            "joint_values": pose["values"],
            "joint_names": self._joint_names,
            "view_index": views_used,
            "message": "%s %s" % (label, pose.get("name", views_used)),
        }


def main():
    rospy.init_node("cargo_exploration_planner")
    CargoExplorationPlannerNode()
    rospy.loginfo("cargo_exploration_planner ready")
    rospy.spin()


if __name__ == "__main__":
    main()
