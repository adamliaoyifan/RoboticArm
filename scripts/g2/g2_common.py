"""Shared G2 harness pieces: isolated MoveIt lifecycle, frozen inputs, scene reset.

Isolation: ROS_DOMAIN_ID=42, ROS_LOCALHOST_ONLY=1, no Gazebo, no controllers,
no /clock. The MoveIt stack is the production isolated launch
(`luggage_planning isolated_moveit.launch.py`); its occupancy is appended to
``moveit_usage.jsonl`` so the 60 minute plan cap is measurable.
"""

import hashlib
import json
import os
import sys
import time

ROOT = os.environ.get("G2_ROOT", "/tmp/elfin-master-pack-baseline-g2")
EVID = os.environ["G2_EVID"]
CAPTURE = os.environ["G1_CAPTURE"]
for _path in (
        os.path.join(ROOT, "scripts", "st09_i"), ROOT,
        os.path.join(ROOT, "src", "luggage_planning"),
        os.path.join(ROOT, "src", "luggage_description"),
        os.path.join(ROOT, "src", "luggage_perception")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import corridor_lattice_score as score  # noqa: E402
import paired_box2_scene_replay as paired  # noqa: E402

# Production configuration of the G1 live run (see G1 run.sh).
SCENE_TF = os.path.join(
    ROOT, "src/luggage_description/config/scene_tf_deploy_update.yaml")
ATLAS_CFG = os.path.join(
    ROOT, "src/luggage_planning/config/atlas_runtime_deploy_update.yaml")
paired.SCENE_TF = SCENE_TF
paired.ATLAS_CFG = ATLAS_CFG
# The live G1 stack started from the sim_deploy_update initial joints. The
# atlas hash (urdf md5 06c4187e...) is bound to exactly these values, so the
# harness MoveIt must expand the same URDF or the atlas never loads.
GZ_LAUNCH = dict(paired.GZ_LAUNCH)
GZ_LAUNCH["xacro_extra"] = (
    "initial_joint1:=1.8806,initial_joint2:=-1.7736,"
    "initial_joint3:=-1.0491,initial_joint4:=4.4439,"
    "initial_joint5:=1.7721,initial_joint6:=1.8473")
REQUEST_BIN = os.path.join(CAPTURE, "select_capture/select_place_path_v2_00.bin")
SURFACE_JSON = os.path.join(
    CAPTURE, "dumps/box_00_PLANNING_BUDGET_EXHAUSTED/replay/surface_2d.json")


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def dump(directory, name, document):
    os.makedirs(directory, exist_ok=True)
    with open(os.path.join(directory, name), "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=1, sort_keys=True, default=str)
        handle.write("\n")


def load_request():
    from rclpy.serialization import deserialize_message
    from luggage_msgs.srv import SelectPlacePathV2

    with open(REQUEST_BIN, "rb") as handle:
        return deserialize_message(handle.read(), SelectPlacePathV2.Request)


def load_portal(index):
    from rclpy.serialization import deserialize_message
    from moveit_msgs.msg import PlanningScene

    record = json.load(open(
        os.path.join(CAPTURE, "portal_capture/portal_%02d.json" % index)))
    with open(os.path.join(CAPTURE, "portal_capture", record["scene_bin"]), "rb") as handle:
        scene = deserialize_message(handle.read(), PlanningScene)
    return record, scene


class MoveItStack(object):
    """One isolated MoveIt launch with timed usage accounting."""

    def __init__(self, out_dir, label):
        self.out_dir = out_dir
        self.label = label
        self.proc = None
        self.started = None

    def start(self):
        from luggage_perception.eval.isolated_domain import isolated_replay_env

        residuals = score._residuals()  # noqa: SLF001
        if residuals:
            raise RuntimeError("residual sim/moveit processes: %r" % residuals)
        env = isolated_replay_env(score.DOMAIN)
        os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        os.makedirs(self.out_dir, exist_ok=True)
        self.started = time.time()
        self.proc = score._start_moveit(  # noqa: SLF001
            env, SCENE_TF, os.path.join(self.out_dir, "moveit_%s.log" % self.label),
            GZ_LAUNCH)
        return self.proc

    def stop(self):
        if self.proc is None:
            return
        score._stop_moveit(self.proc)  # noqa: SLF001
        ended = time.time()
        with open(os.path.join(EVID, "moveit_usage.jsonl"), "a", encoding="utf-8") as handle:
            handle.write(json.dumps({
                "label": self.label, "start": self.started, "end": ended,
                "seconds": round(ended - self.started, 1),
                "residuals_after": score._residuals()}) + "\n")  # noqa: SLF001
        self.proc = None
