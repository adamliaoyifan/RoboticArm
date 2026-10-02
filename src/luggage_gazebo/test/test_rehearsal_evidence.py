"""Offline rehearsal evidence: join order, thresholds, and hash invalidation."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from luggage_gazebo.integration_acceptance import (
    EVENT_ORDER, FROZEN_HASHES, capture_window, extract_acceptance,
)
from luggage_gazebo.rehearsal_evidence import (
    assert_live_identity, bind_evidence, build_evidence, case_from_artifacts,
    fault_window_ok, frozen_document, hash_inputs, latency_samples,
    budget_allows, median_degradation, throughput_degradation,
    topic_counts_match,
)


HULL = "b" * 64
SCENE_MD5 = "c" * 32


def _timeline(t0):
    states = (
        "CARRY_READY", "SLOT_RESOLVED", "SEQUENCE_BUILT",
        "TRANSIT", "DESCEND", "RELEASED", "VERIFIED", "HOME",
    )
    return [{"to": name, "t_ros": t0 + index * 0.5} for index, name in enumerate(states)]


def healthy_trial(box, index, revision_in, revision_out):
    t0 = 1000.0 + index * 100.0
    return {
        "index": index,
        "catalog_id": box,
        "place_state": "HOME",
        "fail_code": "",
        "err_xy": 0.01,
        "err_z": -0.01,
        "drift": 0.005,
        "roll": 0.01,
        "pitch": -0.01,
        "inside_inner_box": True,
        "vac_attach": True,
        "lost_payload": False,
        "extras": {
            "rehearsal_request_id": "req-%s" % box,
            "rehearsal_marks": [
                {"name": "observation", "t_ros": t0},
                {"name": "placement_request", "t_ros": t0 + 0.2},
                {"name": "placement_response", "t_ros": t0 + 0.45},
                {"name": "integration", "t_ros": t0 + 3.2},
            ],
            "state_timeline": _timeline(t0),
            "vacuum_event": {"event": "attach", "ok": True},
            "selector": {
                "winner_method": "grid",
                "probe": {"mode": "on", "probed": 2, "unavailable": 0},
            },
            "integrate": {
                "success": True,
                "expected_map_revision": revision_in,
                "resulting_map_revision": revision_out,
                "n_sensor_occupied": 3,
                "floor_through": 0,
                "hit_cells": 5,
                "total_cells": 10,
                "geometry_hash": HULL,
            },
            "live_scene_revision": SCENE_MD5,
            "live_geometry_hash": HULL,
        },
    }


def paired_evidence():
    cases = [
        case_from_artifacts(healthy_trial("carryon", 0, 0, 1)),
        case_from_artifacts(healthy_trial("standard", 1, 1, 2)),
    ]
    return build_evidence(cases)


def write_inputs(directory):
    groups = {}
    for key in FROZEN_HASHES:
        if key == "atlas":
            paths = []
            for name in ("atlas.npz", "atlas.yaml"):
                path = directory / name
                path.write_bytes(b"atlas-" + name.encode() + b"\n")
                paths.append(path)
            groups[key] = paths
            continue
        if key == "hull":
            path = directory / "hull.json"
            path.write_bytes(HULL.encode())
        else:
            path = directory / (key + ".bin")
            path.write_bytes(("input-" + key).encode())
        groups[key] = [path]
    return groups


class RehearsalEvidence(unittest.TestCase):
    def test_joined_cases_pass_extractor(self):
        evidence = paired_evidence()
        self.assertTrue(evidence["complete"])
        self.assertFalse(evidence["scoring_writes"])
        self.assertEqual(evidence["namespace"], "rehearsal")
        with tempfile.TemporaryDirectory() as directory:
            groups = write_inputs(Path(directory))
            # Hull digest must equal the live geometry hash the trial recorded.
            hull = Path(directory) / "hull.json"
            hull.write_bytes(b"canonical-hull")
            groups["hull"] = [hull]
            digest = hashlib.sha256(b"canonical-hull").hexdigest()
            for case in evidence["cases"]:
                case["live_geometry_hash"] = digest
            hashes = hash_inputs(groups)
            self.assertEqual(set(hashes), set(FROZEN_HASHES))
            assert_live_identity(evidence, SCENE_MD5, digest)
            bound = bind_evidence(evidence, hashes)
            frozen = frozen_document(
                hashes, 0.35, 1.0, ["scripts/frame_migration_st09_replay.sh", "--domain", "42"],
                capture_window=capture_window(
                    bound["latency"]["upstream_seconds"],
                    bound["latency"]["downstream_seconds"]))
            result = extract_acceptance(bound, frozen)
            self.assertEqual(result["status"], "pass", result)
            for case in bound["cases"]:
                stamps = [event["stamp_ns"] for event in case["events"]]
                self.assertEqual(stamps, sorted(stamps))
                self.assertEqual(len(set(stamps)), len(EVENT_ORDER))
                self.assertTrue(all(event["request_id"] == case["request_id"] for event in case["events"]))

    def test_missing_stale_threshold_and_hash_change(self):
        evidence = paired_evidence()
        with tempfile.TemporaryDirectory() as directory:
            groups = write_inputs(Path(directory))
            hashes = hash_inputs(groups)
            for case in evidence["cases"]:
                case["live_geometry_hash"] = hashes["hull"]
            bound = bind_evidence(evidence, hashes)
            frozen = frozen_document(hashes, 0.35, 1.0, ["replay"])
            self.assertEqual(extract_acceptance(bound, frozen)["status"], "pass")

            truncated = copy.deepcopy(bound)
            truncated["cases"][0]["events"].pop()
            truncated["complete"] = False
            self.assertEqual(extract_acceptance(truncated, frozen)["status"], "not_evaluated")

            stale = copy.deepcopy(bound)
            stale["cases"][1]["measured"]["input_map_revision"] = 0
            self.assertIn("STALE_MAP_REVISION",
                          " ".join(extract_acceptance(stale, frozen)["failures"]))

            tipped = copy.deepcopy(bound)
            tipped["cases"][0]["gt_reference"]["roll_deg"] = 2.1
            self.assertIn("GT_ROLL_DEG",
                          " ".join(extract_acceptance(tipped, frozen)["failures"]))

            scene = groups["scene"][0]
            scene.write_bytes(scene.read_bytes() + b"x")
            changed = hash_inputs(groups)
            self.assertNotEqual(changed["scene"], hashes["scene"])
            changed_frozen = frozen_document(changed, 0.35, 1.0, ["replay"])
            result = extract_acceptance(bound, changed_frozen)
            self.assertIn("FROZEN_INPUT_CHANGED", result["failures"])
            self.assertEqual(result["status"], "fail")
            with self.assertRaises(ValueError):
                assert_live_identity(evidence, "d" * 32, hashes["hull"])

    def test_latency_window_and_fault_budget(self):
        evidence = paired_evidence()
        samples = latency_samples(evidence["cases"])
        self.assertEqual(len(samples["upstream_seconds"]), 2)
        self.assertEqual(len(samples["downstream_seconds"]), 2)
        window = capture_window(samples["upstream_seconds"], samples["downstream_seconds"])
        self.assertGreater(window["pre_seconds"], 0)
        self.assertGreater(window["post_seconds"], 0)
        before = {"segments": 0, "commits": 0, "attached": False}
        after = dict(before)
        self.assertTrue(fault_window_ok(
            "pre_pick", "MISSING_PEER", before, after, after, True))
        self.assertFalse(fault_window_ok(
            "pre_pick", "REHEARSAL_FAULT_NOT_REFUSED", before, after, after, True))
        carrying = {"segments": 0, "commits": 0, "attached": True}
        moved = {"segments": 1, "commits": 0, "attached": True}
        self.assertTrue(fault_window_ok(
            "pre_place", "MISSING_PEER", carrying, carrying, carrying, True))
        self.assertFalse(fault_window_ok(
            "pre_place", "MISSING_PEER", carrying, moved, carrying, True))
        self.assertFalse(fault_window_ok(
            "pre_place", "MISSING_PEER", carrying, carrying, carrying, False))

    def test_replay_throughput_and_topic_joins(self):
        self.assertAlmostEqual(throughput_degradation(10.0, 9.5), 0.05)
        self.assertLessEqual(median_degradation([0.2, 0.04, 0.08]), 0.10)
        self.assertEqual(topic_counts_match({"/clock": 3, "/tf": 1}, {"/clock": 3, "/tf": 1}), [])
        self.assertEqual(
            topic_counts_match({"/clock": 3}, {"/clock": 2})[0]["topic"], "/clock")
        with self.assertRaises(ValueError):
            throughput_degradation(0, 1)
        self.assertTrue(budget_allows(0, 0))
        self.assertFalse(budget_allows(1024 ** 3, 0))
        self.assertFalse(budget_allows(0, 4 * 1024 ** 3))


class FreezeCli(unittest.TestCase):
    def test_scene_byte_invalidates_live_identity(self):
        import os
        import subprocess
        import yaml
        from luggage_description.container_geometry import (
            descriptor_from_scene_config, geometry_hash)
        from luggage_description.runtime_identity import scene_revision_from_path

        repo = Path(__file__).resolve().parents[3]
        script = repo / "scripts" / "frame_migration_st09_freeze.py"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scene = root / "scene.yaml"
            scene.write_text(
                "container:\n  inner:\n    length: 1.2\n    width: 0.8\n"
                "    floor_z: 0.0\n    ceiling_z: 1.4\n", encoding="utf-8")
            config = yaml.safe_load(scene.read_text(encoding="utf-8"))
            hull = geometry_hash(
                descriptor_from_scene_config(config).descriptor(include_hash=False))
            evidence = paired_evidence()
            for case in evidence["cases"]:
                case["live_scene_revision"] = scene_revision_from_path(scene)
                case["live_geometry_hash"] = hull
            evidence_path = root / "evidence.json"
            evidence_path.write_text(json.dumps(evidence), encoding="utf-8")
            files = {}
            for name in ("runner", "profile", "model", "urdf", "srdf", "msg"):
                path = root / name
                path.write_bytes(name.encode())
                files[name] = path
            atlas = root / "atlas.npz"
            atlas.write_bytes(b"npz")
            meta = root / "atlas.yaml"
            meta.write_bytes(b"meta")
            env = os.environ.copy()
            env["PYTHONPATH"] = os.pathsep.join([
                str(repo / "src" / "luggage_gazebo"),
                str(repo / "src" / "luggage_description"),
                env.get("PYTHONPATH", ""),
            ])
            base = [
                "python3", str(script),
                "--evidence", str(evidence_path),
                "--scene", str(scene),
                "--runner", str(files["runner"]),
                "--extractor", str(script),
                "--extractor", str(files["runner"]),
                "--resolved-profile", str(files["profile"]),
                "--model", str(files["model"]),
                "--atlas", str(atlas), "--atlas", str(meta),
                "--urdf", str(files["urdf"]),
                "--srdf", str(files["srdf"]),
                "--messages", str(files["msg"]),
                "--replay-command", "scripts/frame_migration_st09_replay.sh",
                "--replay-command", "case.json",
                "--require-window",
            ]
            good = subprocess.run(
                base + ["--output", str(root / "frozen.json"),
                        "--bound-output", str(root / "bound.json")],
                capture_output=True, text=True, env=env, check=False)
            self.assertEqual(good.returncode, 0, good.stderr)
            scene.write_bytes(scene.read_bytes() + b"\n")
            bad = subprocess.run(
                base + ["--output", str(root / "frozen2.json"),
                        "--bound-output", str(root / "bound2.json")],
                capture_output=True, text=True, env=env, check=False)
            self.assertNotEqual(bad.returncode, 0)
            self.assertIn("live scene revision", bad.stderr)


if __name__ == "__main__":
    unittest.main()
