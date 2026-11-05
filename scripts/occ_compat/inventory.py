"""inventory — freeze the G1 consumer/matrix/policy/fixture inventory.

Outputs under --out (PREP):
  CONSUMERS.csv        frozen consumer entries (E1-E4) with entrypoints
  INTERFACE_MATRIX.json G/O topic/service/identity/error-code matrix
  POLICY.json          actual policy values (code-read, not defaults docs)
  FIXTURES.json        selected scene fixtures + per-frame verification
  ROBOT_FIXTURES.json  frozen joint/TF fixture records + hashes
  WIRE_SCHEMA.md       validity JSON + srv texts (the G1 wire contract)
  EXPECTED.json        V1/V2 case skeleton (one terminal per case/subprobe)
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

#: Frozen consumer inventory (I4): every active planning-side consumer of
#: the map, with its real entrypoint. Exclusions: motion_planner_node's
#: own occupancy subscription is out of this deployment's graph (not
#: started); placement_prior_requests shares from_surface_2d via E3's
#: entry; pure tests are not consumers.
CONSUMERS = [
    {
        "id": "E1", "name": "placement_planner_v1",
        "entrypoint": "src/luggage_packing/scripts/placement_planner_node.py"
                      "::_handle (/placement_planner/compute_placement)",
        "map_inputs": ["/luggage/cargo_map/surface_2d (v1 JSON)"],
        "validity_input": "validity_topic param",
        "identity_binding": "epoch+generation via AdmissionGate",
    },
    {
        "id": "E2", "name": "placement_planner_v2",
        "entrypoint": "src/luggage_packing/scripts/placement_planner_node.py"
                      "::_handle_v2 (/placement_planner/compute_placement_v2)",
        "map_inputs": ["/luggage/cargo_map/surface_v2 (typed)",
                       "/luggage/cargo_map/committed (ledger)"],
        "validity_input": "validity_topic param",
        "identity_binding": "epoch+generation + strict revision + "
                            "ledger subset",
    },
    {
        "id": "E3", "name": "occupancy_place_paths",
        "entrypoint": "src/luggage_planning/luggage_planning/"
                      "occupancy_place_paths.py::OccupancySnapshot."
                      "from_surface_2d (driven via plan_attached_box)",
        "map_inputs": ["admitted surface (v1 dict / v2 record view)"],
        "validity_input": "upstream gate (E1/E2 admission)",
        "identity_binding": "expected_hash + expected_revision pinned",
    },
    {
        "id": "E4", "name": "hardware_place_planning",
        "entrypoint": "src/luggage_planning/luggage_planning/"
                      "hardware_place_planning.py::HardwarePlacePlanner.plan",
        "map_inputs": ["/luggage/cargo_map/surface_v2 (typed)",
                       "/luggage/cargo_map/surface_2d (geometry)",
                       "/joint_states"],
        "validity_input": "own subscription (validity_topic ctor arg)",
        "identity_binding": "entry+commit AdmissionToken double check",
    },
]

#: G/O interface matrix per boundary (wire facts verified in code).
INTERFACE_MATRIX = {
    "validity": {
        "O": {"topic": "/uld_occ_observer/validity",
              "type": "std_msgs/String (JSON)",
              "epoch_type": "uint64 (JSON number)",
              "qos": "reliable transient_local depth 64"},
        "G": {"topic": "/luggage/cargo_map/validity",
              "type": "std_msgs/String (JSON)",
              "epoch_type": "UUID string (verbatim)",
              "qos": "reliable transient_local depth 64"},
        "common_fields": ["state", "epoch", "generation", "reason",
                          "reset_id", "transition_mono_ns", "map_revision",
                          "observed_revision",
                          "last_integrated_stamp_ns", "geometry_hash",
                          "config_digest", "clock_domain"],
        "g_only_fields": ["backend", "transition_stamp_ns"],
        "state_words": ["VALID", "INVALID", "BUILDING"],
    },
    "map_v1": {
        "O": {"publisher": "occ_surface_bridge (--topic remap)",
              "identity": "epoch int, validity_generation present"},
        "G": {"publisher": "cargo_volume_mapper_node",
              "identity": "epoch uuid string, validity_generation present"},
        "topic": "/luggage/cargo_map/surface_2d",
    },
    "map_v2": {
        "O": {"publisher": "occ_surface_bridge --publish-v2 "
                           "(C1-worktree adapter; O C++ untouched)",
              "identity": "epoch string, validity_generation"},
        "G": {"publisher": "cargo_volume_mapper_node "
                           "(frame_scene_revision set)",
              "identity": "epoch string, validity_generation"},
        "topic": "/luggage/cargo_map/surface_v2",
        "type": "luggage_msgs/SurfaceContractV2",
    },
    "reset": {
        "O": {"service": "/uld_occ_observer/reset_observation",
              "request": "expected_epoch uint64, reset_id, reason",
              "statuses": ["RESET_ID_REQUIRED", "idempotent replay",
                           "EPOCH_MISMATCH"]},
        "G": {"service": "/cargo_map/reset_v2 (luggage_msgs/"
                         "ResetCargoMapV2)",
              "request": "reset_id, expected_epoch (string), reason",
              "statuses": ["RESET_ID_REQUIRED", "ALREADY_PROCESSED",
                           "OPERATION_ID_CONFLICT", "EPOCH_MISMATCH"]},
        "legacy": {"G": "/cargo_map/reset unchanged premise-less"},
    },
    "status_query": {
        "O": {"service": "/uld_occ_observer/get_validity",
              "type": "uld_occ_observer/srv/GetValidity"},
        "G": {"service": "/cargo_map/get_validity "
                         "(luggage_msgs/srv/GetCargoValidity)",
              "fields": "same names; G adds backend + "
                        "transition_stamp_ns"},
    },
    "error_codes_common": [
        "VALIDITY_GATE_CLOSED", "VALIDITY_GATE_UNBOUND",
        "VALIDITY_COMMIT_REJECTED", "map_epoch_missing",
        "map_epoch_mismatch", "map_generation_missing",
        "map_generation_mismatch", "map_backend_mismatch",
        "STALE_MAP_REVISION", "CARGO_MAP_GEOMETRY_MISMATCH",
    ],
    "g_store_rejections": [
        "EPOCH_MISMATCH_UNDER_RACE", "STAMP_ALREADY_INTEGRATED",
        "OPERATION_ID_CONFLICT", "EPOCH_MISMATCH",
    ],
}

#: Policy frozen from the C1/G code at the G1 commits (not doc defaults).
POLICY = {
    "robot_state_max_age_ms": 500,
    "validity_gate_enforce": True,
    "g_validity_max_age_ms": 1000,
    "o_validity_max_age_ms": 1000,
    "g_freshness_window_sec": 0.20,
    "hardware_max_state_age_s": 1.0,
    "map_generation_binding": "unconditional (missing/0 rejected, both "
                              "backends)",
    "request_deadline_s": 30,
    "probe_boxes_m": [[0.30, 0.20, 0.20], [0.50, 0.30, 0.25]],
    "float_tolerance_m": 1e-9,
}

#: Scene fixtures: run793 generator groups (seed 20261104 lineage) reused
#: per contract §4; verification below records actual per-frame facts.
SCENE_SELECTION = {
    "cuboid": {
        "S-free": "cuboid_b00_c30000",
        "S-occupied": "cuboid_b20_c30000",
        "S-unknown": "cuboid_b05_c30000",
    },
    "chamfer": {
        "S-free": "chamfer_b00_c30000",
        "S-occupied": "chamfer_b20_c30000",
        "S-unknown": "chamfer_b05_c30000",
    },
}
#: S-free: b00 (no boxes) leaves open floor; S-occupied: b20 provides
#: dense committed obstacles for conflict candidates; S-unknown: b05 has
#: partially covered regions (per-frame verification below; if the verify
#: phase shows no unknown-interior candidates, the seed=20261105
#: generator replaces this entry BEFORE freeze).

V1_CASES = ["K%02d" % i for i in range(1, 11)]
V2_CASES = ["R%02d" % i for i in range(1, 13)]


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _verify_frames(combo, max_frames=3):
    """Per-frame verification: point count, TF presence, stamps.

    Reads the frozen generator YAML frames directly (offline; no ROS).
    """
    import yaml
    verified = []
    frames = combo.get("frames", [])[:max_frames]
    for frame in frames:
        path = frame.get("path") if isinstance(frame, dict) else frame
        if not path or not Path(path).exists():
            verified.append({"path": str(path), "error": "MISSING"})
            continue
        document = yaml.safe_load(open(path).read())
        points = document.get("points_m") or []
        transform = document.get("transform") or document.get(
            "sensor_to_container") or {}
        verified.append({
            "path": str(path),
            "sha256": sha256_file(path),
            "n_points": len(points),
            "has_transform": bool(transform),
            "stamp_ns": int(document.get("stamp_ns", 0) or 0),
            "observation_id": str(document.get("observation_id", "")),
        })
    return verified


def _expected_skeleton():
    """One terminal per case/subprobe, generated from the frozen matrix —
    never derived from observed outcomes."""
    expected = {"schema": "expected-v1", "V1": {}, "V2": {}}
    for backend in ("G", "O"):
        for hull in ("cuboid", "chamfer"):
            for consumer in ("E1", "E2", "E3", "E4"):
                for case in V1_CASES:
                    key = "V1|%s|%s|%s|%s" % (backend, hull, consumer, case)
                    subprobes = ["main"]
                    if case == "K05":
                        subprobes = ["stale", "revision-mismatch", "fresh"]
                    if case == "K08":
                        subprobes = ["unit-lie", "legal-conversion"]
                    if case == "K10":
                        subprobes = ["missing-robot", "old-robot",
                                     "new-robot"]
                    expected["V1"][key] = {
                        "subprobes": subprobes,
                        "terminal": "PASS",
                    }
            for case in V2_CASES:
                key = "V2|%s|%s" % (backend, case)
                expected["V2"][key] = {"terminal": "PASS"}
    return expected


def run_inventory(manifest_path, out_dir):
    manifest = json.loads(Path(manifest_path).read_text())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    def _cell(value):
        text = value if isinstance(value, str) else ";".join(value)
        return '"%s"' % text.replace('"', "'").replace(",", ";")

    (out / "CONSUMERS.csv").write_text(
        "id,name,entrypoint,map_inputs,validity_input,identity_binding\n"
        + "\n".join(
            ",".join(_cell(c[k]) for k in
                     ("id", "name", "entrypoint", "map_inputs",
                      "validity_input", "identity_binding"))
            for c in CONSUMERS) + "\n")
    (out / "INTERFACE_MATRIX.json").write_text(
        json.dumps(INTERFACE_MATRIX, indent=1, sort_keys=True))
    (out / "POLICY.json").write_text(
        json.dumps(POLICY, indent=1, sort_keys=True))

    inputs = json.loads(Path(manifest["prep_inputs_json"]).read_text())
    fixtures = {"selection": SCENE_SELECTION, "combos": {}}
    for hull, scenes in SCENE_SELECTION.items():
        for scene, combo_key in scenes.items():
            combo = inputs["combos"][combo_key]
            fixtures["combos"]["%s|%s" % (hull, scene)] = {
                "combo": combo_key,
                "hull": combo["hull"],
                "boxes": combo["boxes"],
                "requested_point_count": combo["requested_point_count"],
                "generator_group": combo.get("generator_group"),
                "geometry": combo["geometry"],
                "config_2cm": combo["config_2cm"],
                "frames_verified": _verify_frames(combo),
                "source": "run793 generator lineage (seed 20261104), "
                          "reused per contract §4 after verification",
            }
    (out / "FIXTURES.json").write_text(
        json.dumps(fixtures, indent=1, sort_keys=True))

    (out / "ROBOT_FIXTURES.json").write_text(json.dumps({
        "joint_states": {
            "topic": "/joint_states",
            "rule": "frozen sequence, header.stamp from the fixture "
                    "scenario clock; zero-stamp samples exist on purpose "
                    "in K10/old-robot subprobes",
            "names": ["elfin_joint%d" % i for i in range(1, 7)],
            "provenance": "explicit test input via the real subscription",
        },
        "tf": {
            "frames": ["container_link", "suction_contact_frame"],
            "provenance": "scene_tf yamls from the run793 prep lineage "
                          "(hash-pinned in FIXTURES)",
        },
    }, indent=1))

    (out / "WIRE_SCHEMA.md").write_text(
        "# G1 wire schema (frozen)\n\n"
        "## validity JSON (both backends; epoch type differs)\n```json\n"
        + json.dumps({k: ("<uint64|uuid-string>" if k == "epoch" else
                          ("<int>" if k in ("generation", "transition_mono_ns",
                                            "map_revision",
                                            "observed_revision",
                                            "last_integrated_stamp_ns")
                           else "<string>"))
                      for k in INTERFACE_MATRIX["validity"][
                          "common_fields"]}, indent=1)
        + "\n```\nG adds: backend, transition_stamp_ns.\n\n"
        "## srv texts\nSee G worktree: luggage_msgs/srv/"
        "ResetCargoMapV2.srv, GetCargoValidity.srv (O's GetValidity is "
        "the field-name-compatible control).\n")

    expected = _expected_skeleton()
    (out / "EXPECTED.json").write_text(json.dumps(expected, indent=1))
    return {"out": str(out),
            "consumers": len(CONSUMERS),
            "v1_cases": len(expected["V1"]),
            "v2_cases": len(expected["V2"])}
