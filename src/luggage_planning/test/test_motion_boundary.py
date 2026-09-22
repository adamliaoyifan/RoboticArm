#!/usr/bin/env python3
"""T2 motion-boundary serialization (no live MoveIt)."""

from __future__ import division

import json
import os
import tempfile
import unittest

import pytest

from luggage_planning.motion_boundary import (
    load_latest_boundary,
    replay_manifest,
    robot_traj_to_dict,
    write_boundary_dump,
)


class _Stamp(object):
    def __init__(self, sec, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


class _Point(object):
    def __init__(self, t, q):
        self.time_from_start = _Stamp(int(t), int((t - int(t)) * 1e9))
        self.positions = list(q)


class _JointTraj(object):
    def __init__(self, names, points):
        self.joint_names = list(names)
        self.points = list(points)


class _RobotTraj(object):
    def __init__(self, names, points):
        self.joint_trajectory = _JointTraj(names, points)


class TestMotionBoundary(unittest.TestCase):
    def test_robot_traj_to_dict_and_roundtrip_file(self):
        traj = _RobotTraj(
            ["elfin_joint1", "elfin_joint2"],
            [_Point(0.0, [0.1, 0.2]), _Point(1.5, [0.3, 0.4])])
        payload = robot_traj_to_dict(traj)
        self.assertEqual(payload["n_points"], 2)
        self.assertEqual(payload["points"][-1]["q"], [0.3, 0.4])
        record = {"name": "place_exit", "trajectory": payload, "success": False}
        root = tempfile.mkdtemp()
        path = write_boundary_dump(root, record)
        self.assertTrue(os.path.isfile(path))
        latest = load_latest_boundary(root)
        self.assertEqual(latest["name"], "place_exit")
        self.assertEqual(latest["trajectory"]["n_points"], 2)

    def test_joint_traj_without_wrapper(self):
        payload = robot_traj_to_dict(_JointTraj(["a"], [_Point(0.0, [1.0])]))
        self.assertEqual(payload["points"][0]["q"], [1.0])

    def test_replay_manifest_missing_traj(self):
        manifest = replay_manifest(
            "trial_00", "GOTO_FAILED", "place_exit",
            {"planning_scene": "replay/planning_scene.json"},
            ["trajectory"])
        self.assertFalse(manifest["replay_possible"])
        self.assertEqual(manifest["missing"], ["trajectory"])


class TestSceneSummary(unittest.TestCase):
    def test_summarize_box_omits_mesh_vertices(self):
        pytest.importorskip("moveit_msgs")
        from luggage_planning.planning_scene_client import (
            build_add_scene,
            summarize_collision_object,
            summarize_planning_scene,
        )
        scene = build_add_scene(
            "placed_0_0_0", [1.03, -0.735, 0.65],
            [0.0, 0.0, 0.0, 1.0], [0.50, 0.35, 0.25])
        obj = scene.world.collision_objects[0]
        row = summarize_collision_object(obj)
        self.assertEqual(row["id"], "placed_0_0_0")
        self.assertEqual(row["type"], "box")
        self.assertEqual(row["size"][0], 0.50)
        summary = summarize_planning_scene(scene)
        self.assertEqual(summary["world_object_ids"], ["placed_0_0_0"])
        self.assertEqual(summary["world_objects"][0]["xyz"][0], 1.03)


if __name__ == "__main__":
    unittest.main()
