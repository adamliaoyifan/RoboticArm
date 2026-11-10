"""Independent baseline-primary replay of a retained candidate pressure case.

No candidate query implementation is imported. Coverage is explicitly limited
by the retained wire window; missing responses cannot pass a campaign gate.
"""

import argparse
import hashlib
import itertools
import json
from pathlib import Path
import sys
import shutil
import time


def load(path):
    return json.loads(path.read_text())


def save(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def replay(args):
    case = Path(args.case).resolve()
    campaign = case.parent.parent
    baseline = campaign / "baseline_source"
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(__file__, out / "source_snapshot.py")
    manifest = dict(
        kind="concurrency_replay",
        phase="initializing",
        started_ns=time.time_ns(),
        case=str(case),
        verifier_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        overall_g2="not_evaluated",
    )
    save(out / "manifest.json", manifest)
    try:
        helpers = Path(__file__).resolve().parents[2] / "src/luggage_perception/test"
        manifest["verifier_helpers"] = {}
        for name in (
            "occ_query_read_oracle.py",
            "occ_query_identity_oracle.py",
            "occ_query_wire_journal.py",
            "occ_query_concurrency_gates.py",
            "occ_query_legacy_read_join.py",
        ):
            shutil.copy2(helpers / name, out / name)
            manifest["verifier_helpers"][name] = hashlib.sha256(
                (helpers / name).read_bytes()
            ).hexdigest()
        from occ_query_eval import source_hashes, git

        manifest["verifier_revision"] = git("rev-parse", "HEAD")
        manifest["verifier_dirty"] = git("status", "--porcelain").splitlines()
        source_campaign = load(campaign / "manifest.json")
        manifest["source_campaign"] = {
            key: source_campaign[key]
            for key in (
                "kind",
                "revision",
                "baseline_revision",
                "dirty",
                "formal_matrix",
                "rates",
                "heartbeat_period_s",
                "lease_s",
                "points",
                "resolution",
            )
            if key in source_campaign
        }
        expected = load(campaign / "source_hashes.json")["baseline"]
        manifest["baseline_source_matches_capture"] = source_hashes(baseline) == expected
        save(out / "manifest.json", manifest)
        if not manifest["baseline_source_matches_capture"]:
            raise RuntimeError("FROZEN_BASELINE_SOURCE_CHANGED")
        return replay_case(case, campaign, baseline, out, manifest)
    except Exception as error:
        manifest.update(
            phase="failed",
            error=repr(error),
            ended_ns=time.time_ns(),
            capture_complete=False,
            replay_possible=False,
        )
        save(out / "manifest.json", manifest)
        raise


def replay_case(case, campaign, baseline, out, manifest):
    sys.path[:0] = [
        str(baseline / "src/luggage_perception"),
        str(baseline / "src/luggage_description"),
        str(Path(__file__).resolve().parents[2] / "src/luggage_perception/test"),
    ]
    import numpy as np
    from rclpy.serialization import deserialize_message
    from luggage_msgs import srv
    from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
    from luggage_perception.occupancy_map_store import (
        OccupancyMapStore,
        MapUpdate,
        UpdateKind,
        SnapshotIdentity,
    )
    from luggage_description.container_geometry import contains_point
    from luggage_description.scene_tf_config_utils import load_scene_tf_config, static_transforms
    from occ_query_wire_journal import read_journal, read_payload

    loaded = Path(sys.modules[CargoVolumeMapper.__module__].__file__).resolve()
    if not loaded.is_relative_to(baseline):
        raise RuntimeError("MIXED_PRIMARY_SOURCE")
    ready = load(case / "owner/ready.json")
    baseline_adapter = load(case / "client/result.json")["baseline_adapter"]
    boundary = load(case / "owner/boundary.json")
    config = boundary["surface"]
    wire = case / "client/failure_wire"
    journal_root = case / "client/wire_journal"
    journal = read_journal(journal_root) if journal_root.exists() else None

    def read_wire(kind, sequence, direction):
        if journal is not None:
            record = journal.get((kind, sequence), {}).get(direction)
            return read_payload(journal_root, record) if record else None
        path = wire / f"{kind}-{sequence}.{direction}.cdr"
        return path.read_bytes() if path.exists() else None

    rows = [json.loads(line) for line in (case / "client/requests.jsonl").read_text().splitlines()]
    accepted = [r for r in rows if r["case"] == "integrate" and r["status"] == "OK"]
    acknowledgments = load(case / "owner/cloud_ack.json")["stamps"]
    if journal is None and len(accepted) != len(acknowledgments):
        raise RuntimeError("AMBIGUOUS_ACCEPTED_CLOUDS")
    scene = load_scene_tf_config(str(baseline / "src/luggage_description/config/scene_tf.yaml"))
    pose = next(
        (
            dict(
                translation=e.get("translation", [0.0, 0.0, 0.0]),
                rotation_rpy=e.get("rotation_rpy", [0.0, 0.0, 0.0]),
            )
            for e in static_transforms(scene)
            if e.get("child") == "container_link"
        ),
        dict(translation=[0.0, 0.0, 0.0], rotation_rpy=[0.0, 0.0, 0.0]),
    )
    measured = ready.get("mapper_config", config)
    mapper = CargoVolumeMapper(
        measured["inner_size"],
        measured["center_base"],
        measured["yaw"],
        measured["resolution"],
        geometry_descriptor=measured["geometry_descriptor"],
        max_raycast_points=measured.get("max_raycast_points", 2000),
    )
    store = OccupancyMapStore(
        mapper,
        epoch=ready["epoch"],
        clock_ns=lambda: 1_000_000_000,
        require_confirmation=measured.get("require_confirmation", False),
    )
    geometry = {}
    for index, (z, y, x) in enumerate(itertools.product(range(4), (-0.48, 0.18), (-0.5, 0.0, 0.5))):
        if index >= ready["boxes"]:
            break
        box = ((x, y, mapper._hull.floor_z + 0.11 + 0.24 * z), (0.35, 0.53, 0.22), 0.0)
        result = store.apply_update(
            MapUpdate(
                operation_id="setup-" + str(index),
                kind=UpdateKind.ADD_BOX_V1,
                box=box,
                box_id="load-box-" + str(index),
            )
        )
        if not result.accepted:
            raise RuntimeError("SETUP_REJECTED")
        geometry["load-box-" + str(index)] = frozenset(mapper._iter_placed_box_voxels(*box))
    assert tuple(store.identity) == (
        ready["epoch"],
        ready["map_revision"],
        ready["content_digest"],
    ), "INITIAL_IDENTITY"
    observations = []
    pending = {}
    kinds = dict(
        summary=srv.GetCargoMapSummaryV2,
        roi=srv.QueryCargoCellsV2,
        distance=srv.QueryCargoDistanceV2,
        surface=srv.GetCargoSurfacePatchV2,
    )
    if baseline_adapter:
        kinds = dict(
            summary=srv.GetCargoMapStats,
            roi=srv.GetCargoVolumeV2,
            distance=srv.GetCargoVolumeV2,
            surface=srv.GetCargoVolumeV2,
        )
    legacy_revisions = {}
    legacy_join = None
    if baseline_adapter:
        from occ_query_legacy_read_join import resolve_legacy_volume_revisions

        descriptors = []
        for row in rows:
            if row["case"] not in ("roi", "distance", "surface"):
                continue
            payload = read_wire(row["case"], row["sequence"], "request")
            if payload is None:
                continue
            request = deserialize_message(payload, srv.GetCargoVolumeV2.Request)
            descriptors.append(dict(
                name=row["case"] + "-" + str(row["sequence"]),
                begin=row["begin"], ended=row["ended"],
                premise={key: getattr(request, key) for key in (
                    "scene_revision", "geometry_hash", "expected_map_revision",
                    "expected_epoch", "expected_content_digest",
                )},
            ))
        traces = [json.loads(line) for line in (case / "owner/trace.jsonl").read_text().splitlines()]
        legacy_revisions, legacy_join = resolve_legacy_volume_revisions(descriptors, traces)
    for row in rows:
        if row["case"] not in kinds:
            continue
        name = row["case"] + "-" + str(row["sequence"])
        kind = kinds[row["case"]]
        request_wire = read_wire(row["case"], row["sequence"], "request")
        response_wire = read_wire(row["case"], row["sequence"], "response")
        if request_wire is None or response_wire is None:
            observations.append(
                dict(name=name, outcome="not_evaluated", reason="MISSING_RETAINED_WIRE")
            )
            continue
        request = deserialize_message(request_wire, kind.Request)
        response = deserialize_message(response_wire, kind.Response)
        actual_status = (
            ("OK" if response.success else response.reason)
            if baseline_adapter
            else response.meta.status
        )
        if actual_status != row["status"]:
            observations.append(dict(name=name, outcome="fail", reason="WIRE_STATUS_JOIN"))
            continue
        if baseline_adapter:
            # Refused old-volume responses carry no response identity. Resolve
            # their observed revision using passive owner callback boundaries;
            # guessing the request revision would hide stale-premise faults.
            if response.success:
                revision = response.map_revision
            else:
                revision = legacy_revisions.get(name)
                if revision is None:
                    observations.append(
                        dict(
                            name=name,
                            outcome="not_evaluated",
                            reason="REFUSED_RESPONSE_REVISION_UNJOINABLE",
                        )
                    )
                    continue
        else:
            if response.meta.status not in ("OK", "QUERY_BUDGET_EXHAUSTED"):
                observations.append(
                    dict(
                        name=name,
                        outcome="not_evaluated",
                        reason="REFUSAL_NOT_REPLAYED",
                        status=response.meta.status,
                    )
                )
                continue
            revision = response.meta.identity.revision
        pending.setdefault(revision, []).append((name, row["case"], row["sequence"], request))
        del response

    from occ_query_read_oracle import QueryReadOracle

    def check_current():
        oracle = QueryReadOracle(mapper, store, geometry, contains_point)
        for name, kind, sequence, request in pending.pop(store.identity.revision, []):
            response = deserialize_message(
                read_wire(kind, sequence, "response"), kinds[kind].Response
            )
            result = dict(name=name, revision=store.identity.revision, outcome="pass")
            try:
                if baseline_adapter:
                    oracle.verify_legacy(kind, request, response, ready["identity"])
                else:
                    oracle.verify(kind, request, response)
            except Exception as error:
                result.update(outcome="fail", error=repr(error))
            observations.append(result)
            print(json.dumps(result), flush=True)

    check_current()
    cloud = np.load(case / "client/cloud.npy").astype(np.float64)
    prefixes = []
    for position, row in enumerate(accepted):
        payload = read_wire("integrate", row["sequence"], "request")
        if payload is not None:
            request = deserialize_message(payload, srv.IntegrateCargoView.Request)
            stamp = [
                request.source_acquisition_stamp.sec,
                request.source_acquisition_stamp.nanosec,
            ]
            if journal is None:
                assert stamp == acknowledgments[position], "CLOUD_STAMP_JOIN"
            premise = SnapshotIdentity(
                request.expected_epoch,
                request.expected_map_revision,
                request.expected_content_digest,
            )
            operation_id = request.operation_id
        else:
            assert (
                row["phase"] == "wiring"
                and row["sequence"] == -1
                and store.identity.revision == ready["map_revision"]
            ), "MISSING_NON_WIRING_INPUT"
            premise = store.identity
            operation_id = "pressure--1"
            stamp = acknowledgments[position]
            prefixes.append(
                dict(
                    sequence=-1,
                    reason="fixed wiring prefix replayed from frozen fixture + unique cloud acknowledgment",
                    stamp=stamp,
                )
            )
        result = store.apply_update(
            MapUpdate(
                operation_id=operation_id,
                kind=UpdateKind.INTEGRATE_VIEW,
                expected_snapshot=premise,
                geometry_hash=mapper.geometry_hash,
                points=cloud,
                origin=(0.0, 0.0, 0.0),
                minimum_z=measured.get("minimum_z", mapper._hull.floor_z + 0.05),
                stamp_key=tuple(stamp),
                acquisition_stamp_ns=stamp[0] * 1_000_000_000 + stamp[1],
                settled=True,
                schema_version=1,
                pose_at_confirm=measured.get("pose_at_confirm", pose),
            )
        )
        assert result.accepted, "ACCEPTED_WRITE_REPLAY_REFUSED"
        response_wire = read_wire("integrate", row["sequence"], "response")
        if response_wire is not None:
            response = deserialize_message(response_wire, srv.IntegrateCargoView.Response)
            assert (
                response.success
                and (response.epoch, response.resulting_map_revision, response.content_digest)
                == tuple(store.identity)
            ), "WRITE_RESPONSE_JOIN"
        check_current()
    recorded = np.load(case / "owner/boundary_map.npz")
    arrays = dict(
        states=np.frombuffer(mapper._grid, "u1"),
        active=np.frombuffer(mapper._active, "u1"),
        source=np.frombuffer(mapper._source, "u1"),
        labels=np.frombuffer(mapper._labels, "<i8"),
        instances=np.frombuffer(mapper._instance_ids, "<i8"),
        logodds=np.asarray(mapper._occupancy.values),
        locked=np.asarray(mapper._occupancy.geometry_locked),
        shadow=np.frombuffer(mapper._sensor_values, "<f8"),
        shadow_seen=np.frombuffer(mapper._sensor_seen, "u1"),
        shadow_labels=np.frombuffer(mapper._sensor_labels, "<i8"),
        shadow_instances=np.frombuffer(mapper._sensor_instance_ids, "<i8"),
    )
    equal = {key: value.tobytes() == recorded[key].tobytes() for key, value in arrays.items()}
    unmatched = [name for entries in pending.values() for name, *_ in entries]
    identity_equal = list(store.identity) == [
        config["epoch"],
        config["map_revision"],
        config["content_digest"],
    ]
    completed_keys = {(row["case"], row["sequence"]) for row in rows}
    sent = [
        json.loads(line) for line in (case / "client/requests_sent.jsonl").read_text().splitlines()
    ]
    missing_completions = [
        dict(case=row["case"], sequence=row["sequence"])
        for row in sent
        if (row["case"], row["sequence"]) not in completed_keys
    ]
    journal_joins = None
    if journal is not None:
        sent_keys = {(row["case"], row["sequence"]) for row in sent}
        response_keys = {key for key, value in journal.items() if "response" in value}
        journal_joins = set(journal) == sent_keys and response_keys == completed_keys
        for row in sent:
            record = journal.get((row["case"], row["sequence"]), {}).get("request", {})
            journal_joins = journal_joins and record.get("sha256") == row["request_sha256"]
    from occ_query_eval import source_hashes

    frozen_unchanged = source_hashes(baseline) == load(campaign / "source_hashes.json")["baseline"]
    helpers = Path(__file__).resolve().parents[2] / "src/luggage_perception/test"
    verifier_unchanged = all(
        hashlib.sha256((helpers / name).read_bytes()).hexdigest() == digest
        for name, digest in manifest["verifier_helpers"].items()
    ) and (hashlib.sha256(Path(__file__).read_bytes()).hexdigest() == manifest["verifier_sha256"])
    manifest["verifier_unchanged"] = verifier_unchanged
    result = dict(
        baseline_adapter=baseline_adapter,
        legacy_callback_join=legacy_join,
        voxel_digest_coverage="full independent primary hash, including private evidence, memberships, binding and provenance",
        frozen_source_unchanged=frozen_unchanged,
        verifier_unchanged=verifier_unchanged,
        journal_joins=journal_joins,
        kind="retained_concurrency_primary_replay",
        baseline_source=str(baseline),
        loaded_primary=str(loaded),
        capture_scope=(
            "all sent/completed wires"
            if journal is not None
            else "retained failure ring; only the fixed wiring prefix may be reconstructed"
        ),
        rows=observations,
        primary_arrays_equal=equal,
        final_identity_equal=identity_equal,
        missing_completions=missing_completions,
        wire_storage=(
            "whole-window compressed journal" if journal is not None else "legacy failure ring"
        ),
        prefix_reconstruction=prefixes,
        unmatched_revision_responses=unmatched,
        final_identity=list(store.identity),
        expected_final_identity=[config["epoch"], config["map_revision"], config["content_digest"]],
        all_captured_queries_pass=bool(observations)
        and all(r["outcome"] == "pass" for r in observations),
        historical_coverage=(
            "not_evaluated"
            if any(r["outcome"] == "not_evaluated" for r in observations)
            else "all logged completed reads"
        ),
        outcome=(
            "pass"
            if frozen_unchanged
            and verifier_unchanged
            and journal_joins is not False
            and all(equal.values())
            and identity_equal
            and observations
            and not missing_completions
            and not unmatched
            and all(r["outcome"] == "pass" for r in observations)
            else "fail_or_incomplete"
        ),
        budget_exhausted_responses=sum(r["status"] == "QUERY_BUDGET_EXHAUSTED" for r in rows),
        request_completion_gate=(
            "fail" if any(r["status"] != "OK" for r in rows) else "not_evaluated"
        ),
        timing_gate="not_evaluated",
        overall_g2="not_evaluated",
    )
    from occ_query_concurrency_gates import evaluate

    jsonl = lambda path: [json.loads(line) for line in path.read_text().splitlines()]
    strict = evaluate(
        load(case / "client/result.json"),
        rows,
        sent,
        load(case / "client/unsent_writes.json"),
        jsonl(case / "client/heartbeat.jsonl"),
        jsonl(case / "owner/trace.jsonl"),
        load(case / "owner/runtime.json"),
        result,
        owner_manifest=load(case / "owner/manifest.json"),
        resources=(
            load(case / "owner/query_resources.json")
            if (case / "owner/query_resources.json").exists()
            else {}
        ),
    )
    save(out / "gates.json", strict)
    result["timing_gate"] = strict["outcome"]
    result["strict_gate_path"] = str(out / "gates.json")
    save(out / "result.json", result)
    manifest.update(
        phase="complete",
        outcome=result["outcome"],
        ended_ns=time.time_ns(),
        capture_complete=journal_joins is not False
        and not unmatched
        and not any(r["outcome"] == "not_evaluated" for r in observations),
        replay_possible=True,
    )
    save(out / "manifest.json", manifest)
    print(json.dumps({k: v for k, v in result.items() if k != "rows"}, indent=2), flush=True)
    return int(result["outcome"] != "pass")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--case", required=True)
    p.add_argument("--out", required=True)
    raise SystemExit(replay(p.parse_args()))
