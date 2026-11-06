"""judge — A0-A5 recomputation from the archive + 12 mutation families.

The judge NEVER trusts summary self-reports: every case terminal is
re-derived from the underlying request/evidence rows in events.jsonl,
and any disagreement with the declared terminal is an error. The
self-test (A1) copies a REAL healthy rehearsal archive, applies each
mutation family structurally, and must reject every mutant while the
untouched control passes — the run793 lesson (checkers existed, the
formal path was never fed them) is baked in: mutations go through the
SAME judge entry as the formal verdict.
"""
from __future__ import annotations

import copy
import json
from pathlib import Path

from .ledger import load_events


def _strict_loads(path):
    """Duplicate-key-rejecting JSON load for every judge input (F10)."""
    def _no_dupes(pairs):
        out = {}
        for key, value in pairs:
            if key in out:
                raise ValueError("duplicate JSON key %r in %s"
                                 % (key, path))
            out[key] = value
        return out
    return json.loads(Path(path).read_text(),
                      object_pairs_hook=_no_dupes)

#: Mutation families (contract V3): each is (name, apply(events) ->
# mutated_events, rule) — the rule is what the judge must fire.
MUTATIONS = (
    ("m01_sha_or_payload_wrong", "input_hash_mismatch"),
    ("m02_expected_id_missing_or_dup", "expected_id_violation"),
    # m03/m11's forged rows are caught on the POSITIVE-leg path, whose
    # frozen recompute rule is evidence_missing (a PASS terminal whose
    # success evidence was destroyed) — labels corrected so the F06
    # rule-hit check verifies the rule that ACTUALLY fires.
    ("m03_summary_pass_but_events_reject", "evidence_missing"),
    ("m04_old_token_authorized_after_reset", "terminal_event_disagreement"),
    ("m05_recovery_without_new_robot", "evidence_missing"),
    ("m06_valid_without_new_epoch_observation", "evidence_missing"),
    ("m07_epoch_truncated_or_cross_backend", "identity_violation"),
    ("m08_unknown_interior_marked_legal", "oracle_violation"),
    ("m09_counts_only_candidates", "payload_stripped"),
    ("m10_past_deadline_marked_ok", "deadline_violation"),
    ("m11_all_positives_no_solution", "evidence_missing"),
    ("m12_replay_payload_missing", "payload_stripped"),
)


def _placements(events):
    return [e for e in events if e["event_kind"] == "placement_request"]


def _terminals(events):
    return [e for e in events if e["event_kind"] == "case_terminal"]


def _integrates(events):
    return [e for e in events if e["event_kind"] == "integrate"]


#: Positive/negative expectation per (case, subprobe): derived from the
#: frozen EXPECTED matrix — a PASS terminal must be backed by matching
#: evidence rows (refusals for negative legs, successes for positive).
NEGATIVE_LEGS = {
    ("K04", "main"), ("K05", "stale"), ("K05", "revision-mismatch"),
    ("K10", "missing-robot"), ("K10", "old-robot"),
    ("R04", "main"), ("R05", "main"), ("R06", "main"),
}
#: Per-consumer composite ids from the sweep (case/CONSUMER): refusal
#: scenarios must refuse through E2/E4 too; E3 refuses via its own
#: boundary (the e3_snapshot_read row carries REFUSED).
for _case in ("K04",):
    NEGATIVE_LEGS.add((_case + "/E2", "main"))
    NEGATIVE_LEGS.add((_case + "/E4", "main"))
POSITIVE_LEGS = {
    ("K01", "main"), ("K05", "fresh"), ("K10", "new-robot"),
    ("K10", "pre-reset-baseline"), ("K10", "pre"),
    ("R02", "main"), ("R03", "main"), ("R07", "main"), ("R08", "main"),
    ("R09", "main"), ("R10", "main"), ("R11", "main"), ("R12", "main"),
}
for _case in ("K01", "K02", "K03", "K05", "K06", "K07", "K08", "K09",
              "K10"):
    POSITIVE_LEGS.add((_case + "/E2", "main"))
    POSITIVE_LEGS.add((_case + "/E4", "main"))
REFUSAL_TOKENS = ("VALIDITY_GATE", "VALIDITY_COMMIT", "STALE_MAP",
                  "robot_state", "BIN_FULL")
SUCCESS_TOKEN = "slot id="

#: Frozen per-case REQUIRED subprobe terminal sets (reviews F03): an ID
#: hits only when EVERY required leg carries exactly one legal terminal;
#: any FAIL dominates; unknown/missing/duplicate legs are named, never
#: erased by another leg's PASS. `page-read` is an O-only diagnostic —
#: listed as optional: a FAIL in it fails, its absence on G is fine.
REQUIRED_SUBPROBES = {
    "K01": ("main",), "K02": ("main",), "K03": ("main",),
    "K04": ("main",), "K05": ("fresh", "stale", "revision-mismatch"),
    "K06": ("main",), "K07": ("main",),
    "K08": ("unit-lie", "legal-conversion"),
    "K09": ("old-surface-replay", "old-revision-request"),
    "K10": ("missing-robot", "old-robot", "new-robot",
            "pre-reset-baseline"),
    "R01": ("main",), "R02": ("main",),
    "R03": ("handle-acquired", "old-handle-refused",
            "old-identity-refused", "old-payload-unauthorized",
            "old-revision-request-refused", "new-epoch-healthy", "main"),
    "R04": ("main",), "R05": ("main",), "R06": ("main",),
    "R07": ("main",), "R08": ("main",), "R09": ("main",),
    "R10": ("main",), "R11": ("main",), "R12": ("main",),
}
OPTIONAL_SUBPROBES = {"R03": ("page-read",)}  # backend-specific legs
for _case in ("K01", "K02", "K03", "K04", "K05", "K06", "K07", "K08",
              "K09", "K10"):
    for _e in (2, 3, 4):  # per-consumer composites are their own IDs
        REQUIRED_SUBPROBES["%s/E%d" % (_case, _e)] = ("main",)

#: Frozen per-case refusal vocabularies (reviews F07): a negative leg's
#: refusal must come from the case's OWN reason class — R04/R05/R06 are
#: mid-computation reset races (commit/gate/stale revision), so BIN_FULL
#: or robot staleness cannot substantiate them.
ALLOWED_REFUSALS = {
    "R04": ("VALIDITY_COMMIT_REJECTED", "VALIDITY_GATE_CLOSED",
            "STALE_MAP_REVISION"),
    "R05": ("VALIDITY_COMMIT_REJECTED", "VALIDITY_GATE_CLOSED",
            "STALE_MAP_REVISION"),
    "R06": ("VALIDITY_COMMIT_REJECTED", "VALIDITY_GATE_CLOSED",
            "STALE_MAP_REVISION"),
}

#: Frozen K01 oracle contract: exact leg set + coordinate-bearing sample.
K01_ORACLE_LEGS = frozenset(
    ("clearance", "hull", "occupied_overlap", "support",
     "unknown_interior"))
K01_SAMPLE_FIELDS = ("cells_x", "cells_y", "center_container")


def _rows(events, kind, case_id, subprobe):
    return [e for e in events if e["event_kind"] == kind
            and e.get("case_id") == case_id
            and e.get("subprobe", "") == subprobe]


def recompute_case(events, case_id, subprobe=""):
    """Re-derive one case terminal from evidence rows."""
    terminals = [e for e in _terminals(events)
                 if e["case_id"] == case_id
                 and e.get("subprobe", "") == subprobe]
    if not terminals:
        return None, "no_terminal"
    if len(terminals) > 1:
        return None, "duplicate_terminal"
    declared = terminals[0]
    base_case = case_id.split("/")[0]
    placements = _rows(events, "placement_request", case_id, subprobe)
    key = (case_id, subprobe)
    if declared["state"] == "PASS":
        if key in NEGATIVE_LEGS:
            allowed = ALLOWED_REFUSALS.get(base_case)
            rows = placements + [e for e in events
                                 if e.get("case_id") == case_id
                                 and e["event_kind"] == "e4_plan_entry"]
            if allowed is not None:
                # F07: a restricted-vocabulary case must show its OWN
                # reason class in the message — the generic REFUSED
                # state form cannot substitute for it.
                if not any(any(tok in str(p.get("message", "") or "")
                               for tok in allowed)
                           for p in rows):
                    return None, "terminal_event_disagreement"
            elif not any(any(tok in str(p.get("message", "") or "")
                             for tok in REFUSAL_TOKENS)
                         or str(p.get("state", "")).startswith("REFUSED")
                         for p in rows):
                return None, "terminal_event_disagreement"
        elif key in POSITIVE_LEGS:
            rows = placements + [e for e in events
                                 if e.get("case_id") == case_id
                                 and e["event_kind"] in (
                                     "probe_detail", "e4_plan_entry",
                                     "e3_snapshot_read")]
            # E4 positive: BOUNDARY_REACHED is the success vocabulary.
            if not any(
                    SUCCESS_TOKEN in str(r.get("message", "") or "")
                    or str(r.get("state", "")) == "BOUNDARY_REACHED"
                    or str(r.get("reason", "")).startswith(
                        "identity matched")
                    for r in rows):
                return None, "evidence_missing"
        else:
            evidence = [e for e in events
                        if e.get("case_id") == case_id
                        and e["event_kind"] in (
                            "placement_request", "integrate",
                            "e3_window_stats", "datum_probe",
                            "clock_probe", "probe_detail",
                            "e3_snapshot_read", "e4_plan_entry",
                            "e4_republish")]
            if not placements and not evidence:
                return None, "evidence_missing"
            # F07 fact-scope: a PASS leg must be backed by REAL rows —
            # either a row scoped to this leg, or a case-scoped FACT
            # row (integrate/placement/snapshot/e4/datum probe). A lone
            # non-fact probe row (e.g. state BROKEN) cannot substantiate
            # any leg (0031 counterexample: bare leg labels + one
            # broken probe).
            fact_kinds = ("integrate", "placement_request",
                          "e3_snapshot_read", "e4_plan_entry",
                          "e4_republish", "datum_probe",
                          "e3_window_stats")
            bad_states = ("BROKEN", "FAIL")
            backing = [
                e for e in evidence
                if (e.get("subprobe", "") == subprobe
                    or e["event_kind"] in fact_kinds)
                and str(e.get("state", "OK")) not in bad_states]
            if not backing:
                return None, "evidence_missing"
        for event in placements:
            deadline = event.get("deadline_ns")
            if deadline and event.get("mono_ns", 0) > int(deadline):
                return None, "deadline_violation"
        # payload discipline: every request row carries its payload.
        for event in placements:
            if "message" not in event:
                return None, "payload_stripped"
        for event in _integrates(events):
            if event.get("case_id") == case_id and "result_repr" not in event:
                return None, "payload_stripped"
        # identity sanity: no null/truncated epoch may appear anywhere
        # in the archive's recorded messages.
        for event in events:
            identity = str(event.get("message", "") or "")
            if "epoch=None" in identity or "epoch=truncated" in identity:
                return None, "identity_violation"
        # K01 evidence: BOTH outcomes carry the oracle verdict; a PASS
        # must show every leg PASS and a nonempty candidate sample.
        if case_id == "K01":
            oracle_rows = [e for e in events
                           if e.get("case_id") == "K01"
                           and "oracle_legs" in e]
            if not oracle_rows:
                return None, "payload_stripped"
            legs = oracle_rows[-1].get("oracle_legs") or {}
            if declared["state"] == "PASS" and (
                    not legs or any(v != "PASS" for v in legs.values())):
                return None, "oracle_violation"
            # Frozen leg vocabulary + coordinate-bearing candidate
            # sample: invented leg names or a counts-only sample are
            # oracle violations (reviews F07).
            if set(legs) != set(K01_ORACLE_LEGS):
                return None, "oracle_violation"
            sample = oracle_rows[-1].get("candidate_sample") or {}
            if not all(f in sample for f in K01_SAMPLE_FIELDS):
                return None, "payload_stripped"
            if not all(sample.get(f) not in (None, [], {})
                       for f in K01_SAMPLE_FIELDS):
                return None, "payload_stripped"
            claim = float((oracle_rows[-1].get("candidate_sample")
                           or {}).get("confidence_ratio", 0.0) or 0.0)
            classes = oracle_rows[-1].get("classes") or [
                c for c in (declared.get("classes") or [])]
            if claim >= 0.5 and classes and "known_free" not in classes:
                return None, "oracle_violation"
            if claim < 0.5 and classes and "floor_prior" not in classes:
                return None, "oracle_violation"
            if declared["state"] == "PASS" and not (
                    oracle_rows[-1].get("candidate_sample")):
                return None, "payload_stripped"
        # The recovery ladder's new-robot leg needs its post-reset
        # observation evidence (the K10b integrates).
        if key == ("K10", "new-robot"):
            if not any(e.get("case_id") == "K10b"
                       and e.get("state") == "OK"
                       for e in _integrates(events)):
                return None, "evidence_missing"
    return declared["state"], None


def _leg_completeness_problems(events):
    """F07/F03: a declared case must carry its FROZEN leg set — an
    archive with only a bare PASS main terminal (no legs) is
    evidence_missing, whatever the terminal claims."""
    problems = []
    by_case = {}
    for t in _terminals(events):
        by_case.setdefault(t["case_id"], []).append(t)
    for case_id, rows in by_case.items():
        if case_id not in REQUIRED_SUBPROBES:
            continue  # unknown ids are the known_ids check's business
        if not any(t["state"] == "PASS" for t in rows):
            continue  # nothing claimed: a named NE archive stays legal
        ok, detail = _evaluate_case(case_id, rows)
        if not ok and detail != "FAIL":
            problems.append("%s: evidence_missing (%s)"
                            % (case_id, detail))
    return problems


def judge_events(events, known_ids=None):
    """Full verdict over one archive's events: recompute every declared
    terminal; any violation rejects the whole archive. ``known_ids`` (a
    set) cross-checks every integrate id against the frozen fixture
    manifest — an unknown id is an input-hash/payload violation."""
    problems = []
    if known_ids is not None:
        for event in _integrates(events):
            if event.get("case_id") == "K07":
                continue   # the deliberately-foreign probe row (K07)
            base = str(event.get("request_id", "")).split("#")[0]
            if base and base not in known_ids:
                problems.append(
                    "input_hash_mismatch: unknown observation id %r"
                    % base)
                break
    seen = {}
    for terminal in _terminals(events):
        key = (terminal["case_id"], terminal.get("subprobe", ""))
        seen.setdefault(key, 0)
        seen[key] += 1
        state, rule = recompute_case(events, terminal["case_id"],
                                     terminal.get("subprobe", ""))
        if rule and rule != "no_terminal":
            problems.append("%s/%s: %s" % (key[0], key[1], rule))
    for key, count in seen.items():
        if count > 1:
            problems.append("%s/%s: duplicate_terminal" % key)
    integrate_ids = [e.get("request_id") for e in _integrates(events)
                     if e.get("request_id")]
    if len(integrate_ids) != len(set(integrate_ids)):
        problems.append("expected_id_violation: duplicate integrate id")
    problems.extend(_leg_completeness_problems(events))
    return problems


# ---- mutation appliers (structural, per family) ----------------------
def _mutate(events, name):
    mutated = copy.deepcopy(events)
    def _placement_for(case_id, subprobe="", need_success=None):
        for event in mutated:
            if (event["event_kind"] == "placement_request"
                    and event["case_id"] == case_id
                    and (not subprobe or
                         event.get("subprobe", "") == subprobe)):
                msg = str(event.get("message", "") or "")
                if need_success is True and SUCCESS_TOKEN not in msg:
                    continue
                if need_success is False and SUCCESS_TOKEN in msg:
                    continue
                return event
        return None

    if name == "m01_sha_or_payload_wrong":
        # an integrate row whose observation id no longer matches the
        # frozen fixture manifest (unknown expected id)
        for event in _integrates(mutated):
            rid = str(event.get("request_id", ""))
            base, sep, suffix = rid.partition("#")
            event["request_id"] = base + "X" + sep + suffix
            break
    elif name == "m02_expected_id_missing_or_dup":
        integrates = [e for e in _integrates(mutated)
                      if e.get("request_id")]
        if integrates:
            mutated.append(copy.deepcopy(integrates[0]))
    elif name == "m03_summary_pass_but_events_reject":
        event = _placement_for("K01", "main", need_success=True) \
            or _placement_for("R02", "main", need_success=True)
        if event:
            event["message"] = "VALIDITY_GATE_CLOSED: forged"
    elif name == "m04_old_token_authorized_after_reset":
        event = _placement_for("R05", "main", need_success=False)
        if event:
            event["message"] = "slot id=forged"
    elif name == "m05_recovery_without_new_robot":
        event = _placement_for("K10", "new-robot", need_success=True)
        if event:
            event["message"] = "robot_state_missing (forged)"
    elif name == "m06_valid_without_new_epoch_observation":
        # the recovery evidence disappears: no post-reset observation
        # backs the K10 ladder
        for event in _integrates(mutated):
            if "K10b" in str(event.get("request_id", "")):
                event["state"] = "ERROR"
    elif name == "m07_epoch_truncated_or_cross_backend":
        for event in mutated:
            message = str(event.get("message", "") or "")
            if "epoch=" in message:
                event["message"] = message.replace(
                    "epoch=", "epoch=None truncated ", 1)
                break
    elif name == "m08_unknown_interior_marked_legal":
        for event in mutated:
            if (event.get("case_id") == "K01"
                    and "candidate_sample" in event
                    and event.get("candidate_sample")):
                event["candidate_sample"] = dict(
                    event["candidate_sample"], confidence_ratio=1.0)
                event["oracle_legs"] = {
                    k: "PASS" for k in (event.get("oracle_legs") or {})}
                break
    elif name == "m09_counts_only_candidates":
        for event in mutated:
            event.pop("candidate_sample", None)
            event.pop("claims", None)
            event.pop("oracle_legs", None)
    elif name == "m10_past_deadline_marked_ok":
        event = _placement_for("K01", "main") or _placement_for("R02")
        if event:
            event["deadline_ns"] = 1
    elif name == "m11_all_positives_no_solution":
        for event in mutated:
            if (event["event_kind"] == "placement_request"
                    and SUCCESS_TOKEN in str(event.get("message", ""))):
                event["message"] = "BIN_FULL forged"
                event["reason"] = "BIN_FULL"
    elif name == "m12_replay_payload_missing":
        for event in mutated:
            event.pop("result_repr", None)
            event.pop("message", None)
            event.pop("oracle_legs", None)
    return mutated


def _mini_a0_fixture(base):
    """Build a REAL miniature freeze-v2 (store, manifests, probes) that
    passes structural _freeze_gate and independent a0_verify, so the
    w09–w12 mutations exercise the actual payload-verification path
    rather than a mock."""
    import hashlib as _hashlib

    def _sha(data):
        return _hashlib.sha256(data).hexdigest()

    base = Path(base)
    base.mkdir(parents=True, exist_ok=True)
    payloads = {
        # side-annotated absolute artifact path -> bytes
        "install/g/luggage_perception/lib/luggage_perception/mod.py":
            b"# g module\n",
        "install/g/luggage_perception/share/luggage_perception/cfg.yaml":
            b"resolution: 0.02\n",
        "install/g/luggage_perception/lib/luggage_perception/"
        "cargo_volume_mapper_node.py": b"#!/usr/bin/env python3\n",
        "install/c1/luggage_packing/lib/luggage_packing/"
        "placement_planner_node.py": b"#!/usr/bin/env python3\n",
        "install/o/uld_occ_observer/lib/uld_occ_observer/observer_node":
            b"\x7fELF-fake\n",
    }
    entries = {"G": {}, "C1": {}, "O_CONTROL": {}}
    store = base / "store"
    for rel, data in payloads.items():
        path = base / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        digest = _sha(data)
        shard = store / digest[:2] / digest
        shard.parent.mkdir(parents=True, exist_ok=True)
        shard.write_bytes(data)
        side, pkg_rest = rel.split("/", 2)[1], rel.split("/", 2)[2]
        side = {"g": "G", "c1": "C1", "o": "O_CONTROL"}[side]
        entries[side][pkg_rest] = {"sha256": digest,
                                   "size": len(data), "role": "module"}
    doc = {
        "schema": "freeze-v2",
        "source_map": {
            "G": {"commit": "mini", "packages": {
                "luggage_perception": {"mod.py": _sha(b"# g module\n")}}},
            "C1": {"commit": "mini", "packages": {
                "luggage_packing": {"scripts/planner.py":
                                    _sha(b"# c1\n")}}},
            "T1": {"commit": "mini", "scripts": {
                "occ_compat/judge.py": _sha(b"# t1\n")}},
            "O_CONTROL": {"commit": "mini"},
        },
        "install_manifest": entries,
        "store": {"kind": "content-addressed-sha256", "shards": 0},
        "dirty": {"G": [], "C1": [], "O": []},
        "problems": [],
        "manifest": {
            "g_install": str(base / "install/g"),
            "c1_install": str(base / "install/c1"),
            "o_install": str(base / "install/o"),
            "g_root": str(base / "rootg"),
            "c1_root": str(base / "rootc1"),
            "sfree_dir": "mini",
        },
    }
    # mirror the source/T1 hashes into the store so declared==stored
    for data in (b"# g module\n", b"# c1\n", b"# t1\n"):
        digest = _sha(data)
        shard = store / digest[:2] / digest
        shard.parent.mkdir(parents=True, exist_ok=True)
        shard.write_bytes(data)
    (base / "FREEZE.json").write_text(json.dumps(doc))
    probes = {"schema": "module-probes-v1", "probes": [
        {"kind": "module", "module": "luggage_perception.mod",
         "path": str(base / "install/g/luggage_perception/lib/"
                    "luggage_perception/mod.py"),
         "sha256": _sha(b"# g module\n")},
        {"kind": "executable", "name": "observer",
         "path": str(base / "install/o/uld_occ_observer/lib/"
                    "uld_occ_observer/observer_node"),
         "sha256": _sha(b"\x7fELF-fake\n")},
    ]}
    # probes must reference real on-disk paths: materialize the first
    # probe's path exactly as declared
    (base / "PROBES.json").write_text(json.dumps(probes))
    return base


def _mutate_store_byte(fixture):
    shard = next(iter(sorted((fixture / "store").glob("*/*"))))
    data = bytearray(shard.read_bytes())
    data[0] ^= 0xFF
    shard.write_bytes(bytes(data))


def _mutate_store_delete(fixture):
    shard = next(iter(sorted((fixture / "store").glob("*/*"))))
    shard.unlink()


def _mutate_manifest_crosswire(fixture):
    """Same-name package replacement: point one manifest entry at
    ANOTHER payload's hash — the store stays consistent, the runtime
    probe exposes the cross-wiring."""
    doc = json.loads((fixture / "FREEZE.json").read_text())
    sides = doc["install_manifest"]
    # swap the PROBE-BOUND module's hash so runtime identity exposes the
    # cross-wiring (store itself stays name==hash consistent)
    target = next(k for k in sorted(sides["G"]) if k.endswith("mod.py"))
    donor = sorted(sides["C1"])[0]
    sides["G"][target]["sha256"] = sides["C1"][donor]["sha256"]
    (fixture / "FREEZE.json").write_text(json.dumps(doc))


def _mutate_probe_undeclared(fixture):
    """A module the run loaded that no manifest entry declares."""
    rogue = fixture / "rogue_installer.py"
    rogue.write_bytes(b"# never frozen\n")
    probes = json.loads((fixture / "PROBES.json").read_text())
    probes["probes"].append({
        "kind": "module", "module": "rogue_installer",
        "path": str(rogue), "sha256": _sha256_str(b"# never frozen\n")})
    (fixture / "PROBES.json").write_text(json.dumps(probes))


def _sha256_str(data):
    import hashlib
    return hashlib.sha256(data).hexdigest()


def selftest(manifest_path, out_dir):
    """A1: mutations on a REAL rehearsal archive through the SAME judge
    entry as the formal verdict. Expects the latest V1 rehearsal ledger
    under the evidence root (or the manifest's rehearsal_ledger)."""
    manifest = json.loads(Path(manifest_path).read_text())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    ledger_path = manifest.get("rehearsal_ledger")
    if not ledger_path or not Path(ledger_path).exists():
        # fall back: the newest FULL V1 group ledger under the evidence
        # root — the mutation families target the K-case sweep, so a
        # targeted R-only archive cannot exercise them (the g2 targeted
        # runs made "newest events.jsonl" the wrong corpus).
        root = Path(manifest.get(
            "evidence_root",
            "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
            "occ_grid_consumer_compat_20261105"))
        candidates = []
        for path in root.glob("**/events.jsonl"):
            try:
                text = path.read_text()
            except OSError:
                continue
            if '"case_id": "K01"' in text and '"case_id": "K10"' in text:
                candidates.append(path)
        candidates.sort(key=lambda p: p.stat().st_mtime)
        if not candidates:
            return {"error": "no full V1 group ledger found"}
        ledger_path = str(candidates[-1])
    known_ids = set()
    for manifest_json in Path(manifest.get("sfree_dir", ".")).glob(
            "*/MANIFEST.json"):
        for frame in json.loads(
                manifest_json.read_text()).get("frames", []):
            known_ids.add(Path(frame.get("path", "")).stem)
    archives = []
    primary = Path(ledger_path)
    archives.append(primary)
    sibling = primary.parent.parent / "v2-g-cuboid" / "events.jsonl"
    if not sibling.exists() or "R05" not in sibling.read_text():
        # the sibling must be a FULL V2 group (a targeted --cases smoke
        # cannot exercise the R-case mutation families); fall back to
        # the newest full V2 corpus under the evidence root.
        root = Path(manifest.get(
            "evidence_root",
            "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
            "occ_grid_consumer_compat_20261105"))
        cands = []
        for path in root.glob("**/V2-G-cuboid/events.jsonl"):
            try:
                text = path.read_text()
            except OSError:
                continue
            if '"case_id": "R05"' in text and '"case_id": "R12"' in text:
                cands.append(path)
        cands.sort(key=lambda p_: p_.stat().st_mtime)
        sibling = cands[-1] if cands else sibling
    if sibling.exists():
        archives.append(sibling)
    result = {"archives": [str(a) for a in archives],
              "known_ids": len(known_ids), "mutations": {}}
    controls_ok = True
    for archive in archives:
        events = load_events(archive)
        problems = judge_events(events, known_ids=known_ids)
        if problems:
            controls_ok = False
            result.setdefault("control_problems", []).extend(problems[:3])
    result["control"] = "PASS" if controls_ok else "FAIL"
    result["mutations_rule_hit"] = {}
    for name, expected_rule in MUTATIONS:
        rejected = False
        rule_hit = False
        for archive in archives:
            events = load_events(archive)
            problems = judge_events(_mutate(events, name),
                                    known_ids=known_ids)
            if problems:
                rejected = True
                if any(expected_rule in str(p) for p in problems):
                    rule_hit = True
                break
        result["mutations"][name] = "REJECTED" if rejected else (
            "FALSE_PASS" if controls_ok else "REJECTED")
        result["mutations_rule_hit"][name] = bool(
            rejected and rule_hit)
    # Tool/tree binding: the selftest records the worktree commit it ran
    # under; the formal A1 check compares it to the freeze's C1 commit.
    import subprocess
    try:
        root = Path(__file__).resolve().parents[2]
        result["tree_commit"] = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            capture_output=True, text=True, timeout=30).stdout.strip()
    except Exception as error:  # noqa: BLE001 - binding is required
        result["tree_commit"] = ""
        result["tree_commit_error"] = str(error)[:80]
    # Gate-wiring families (reviews 2026-11-06_1548): each feeds the
    # formal gate functions a KNOWN-BAD input and must NOT pass. These
    # verify the wiring this judge was rebuilt for — empty archives,
    # missing freeze fields, missing mutation ids, named required cases
    # and missing failure payloads can no longer slip through.
    wiring = {}
    empty_dir = out / "w_empty_archive"
    empty_dir.mkdir(parents=True, exist_ok=True)
    wiring["w01_empty_archive_not_all_pass"] = "REJECTED" if any(
        v != "PASS" for v in judge_archive(
            empty_dir, out / "w_empty_verdict").get("gates", {}).values()
    ) or not judge_archive(
        empty_dir, out / "w_empty_verdict").get("gates") else "FALSE_PASS"
    wiring["w02_missing_freeze_keys"] = "REJECTED" if _freeze_gate(
        {}).startswith("FAIL") else "FALSE_PASS"
    partial = {"mutations": dict(result["mutations"]),
               "control": "PASS", "archives": result["archives"]}
    partial["mutations"].pop("m07_epoch_truncated_or_cross_backend")
    wiring["w03_missing_mutation_id"] = "REJECTED" if _a1_gate(
        partial, None).startswith("FAIL") else "FALSE_PASS"
    terminals = [dict(case_id=i, state="NOT_EVALUATED")
                 for i in V2_REQUIRED]
    named = _scope_gate([("V2-G-cuboid", terminals, [])],
                        V2_REQUIRED, EXPECTED_V2G_GROUPS)
    wiring["w04_named_required_blocks"] = (
        "REJECTED" if named.startswith("NOT_EVALUATED")
        or named.startswith("FAIL") else "FALSE_PASS")
    # reviews 2026-11-06_1620: an empty scope and an unknown terminal
    # state must never pass — both reproduced by the reviewer offline.
    wiring["w07_missing_required_group"] = "REJECTED" if _scope_gate(
        [], V2_REQUIRED, EXPECTED_V2G_GROUPS).startswith(
            "NOT_EVALUATED") else "FALSE_PASS"
    weird = [dict(case_id=i, state="WEIRD_STATE") for i in V2_REQUIRED]
    wiring["w08_unknown_terminal_state"] = "REJECTED" if _scope_gate(
        [("V2-G-cuboid", weird, [])], V2_REQUIRED,
        EXPECTED_V2G_GROUPS).startswith("NOT_EVALUATED") else "FALSE_PASS"
    wiring["w05_missing_failure_payload"] = "REJECTED" if _replay_gate(
        out / "w_missing_replay").startswith("FAIL") else "FALSE_PASS"
    wiring["w06_missing_regression"] = "REJECTED" if _regression_gate(
        out / "w_missing_regression").startswith("FAIL") else "FALSE_PASS"
    # G2 §3.1 A0 payload wiring: a REAL miniature freeze (built with the
    # same content-addressed layout) feeds the independent verifier; each
    # mutation must reject, the healthy control must reach A0 PASS.
    import shutil
    from .a0_verify import verify as _a0v
    base = _mini_a0_fixture(out / "w_a0_base")
    control = _a0v(base / "FREEZE.json", [base / "PROBES.json"])
    import hashlib as _hl
    # The healthy control must ALSO carry the F05 run/probe binding:
    # one bound group whose ledger/probes digests the report names.
    ctrl_group = base / "ctrl_group"
    ctrl_group.mkdir(parents=True, exist_ok=True)
    (ctrl_group / "events.jsonl").write_text(
        json.dumps({"event_kind": "case_terminal", "case_id": "R03",
                    "subprobe": "main", "state": "PASS"}) + "\n")
    ctrl_binding = {
        "ctrl_group": {
            "ledger_sha256": _hl.sha256(
                (ctrl_group / "events.jsonl").read_bytes()).hexdigest(),
            "probes_sha256": _hl.sha256(
                (base / "PROBES.json").read_bytes()).hexdigest()}}
    control["per_group"] = [
        {"group": "ctrl_group",
         "ledger_sha256": ctrl_binding["ctrl_group"]["ledger_sha256"],
         "probes_sha256": ctrl_binding["ctrl_group"]["probes_sha256"]}]
    wiring["w09_base_control_passes"] = (
        "PASS" if control.get("verdict") == "PASS"
        and _freeze_gate(
            json.loads((base / "FREEZE.json").read_text()), control,
            freeze_sha256=_hl.sha256(
                (base / "FREEZE.json").read_bytes()).hexdigest(),
            archive_binding=ctrl_binding,
        ) == "PASS" else "FALSE_PASS")
    mutations_a0 = {
        "w09_single_byte_corruption": _mutate_store_byte,
        "w10_missing_artifact": _mutate_store_delete,
        "w11_same_name_package_replacement": _mutate_manifest_crosswire,
        "w12_undeclared_loaded_path": _mutate_probe_undeclared,
    }
    for name, mutate in mutations_a0.items():
        target = out / name
        shutil.copytree(base, target)
        mutate(target)
        result_a0 = _a0v(target / "FREEZE.json", [target / "PROBES.json"])
        wiring[name] = "REJECTED" if (
            result_a0.get("verdict") == "REJECTED"
            and _freeze_gate(json.loads(
                (target / "FREEZE.json").read_text()),
                result_a0).startswith("FAIL")) else "FALSE_PASS"
    # F06 formal-path parity: an unknown observation id must be rejected
    # by judge_archive ITSELF (the formal entry derives known_ids from
    # the freeze's fixture manifest), not only by the selftest helper.
    w13_dir = out / "w13_foreign_input"
    (w13_dir / "V1-G-cuboid").mkdir(parents=True, exist_ok=True)
    (w13_dir / "V1-G-cuboid" / "events.jsonl").write_text(json.dumps({
        "event_kind": "integrate", "case_id": "K99",
        "request_id": "foreign#0", "result_repr": "OK"}) + "\n")
    sfree = out / "w13_sfree" / "sfree_x"
    sfree.mkdir(parents=True, exist_ok=True)
    (sfree / "MANIFEST.json").write_text(json.dumps(
        {"frames": [{"path": "expected_frame.bin", "sha256": "0" * 64}]}))
    w13_freeze = out / "w13_FREEZE.json"
    w13_freeze.write_text(json.dumps({"schema": "freeze-v2",
                                      "manifest": {"sfree_dir":
                                                   str(sfree.parent)}}))
    w13_verdict = judge_archive(
        w13_dir, out / "w13_verdict", freeze_path=str(w13_freeze))
    wiring["w13_unknown_observation_id_formal"] = (
        "REJECTED" if any(
            v != "PASS" for v in w13_verdict.get("gates", {}).values())
        or w13_verdict.get("gates", {}).get("A5", "").startswith("FAIL")
        else "FALSE_PASS")
    result["gate_wiring"] = wiring
    (out / "SELFTEST.json").write_text(json.dumps(result, indent=1))
    return result


#: Contract c7039905 §5 — the per-ID required sets (frozen EXPECTED).
#: Quantity floors never substitute: a gate only passes when every
#: required case ID is present and PASS in its scope.
V1_REQUIRED = tuple(
    ["K%02d" % i for i in range(1, 11)]
    + ["K%02d/E%d" % (i, e) for i in range(1, 11) for e in (2, 3, 4)])
V2_REQUIRED = tuple("R%02d" % i for i in range(1, 13))
MUTATION_IDS = (
    "m01_sha_or_payload_wrong", "m02_expected_id_missing_or_dup",
    "m03_summary_pass_but_events_reject",
    "m04_old_token_authorized_after_reset",
    "m05_recovery_without_new_robot",
    "m06_valid_without_new_epoch_observation",
    "m07_epoch_truncated_or_cross_backend",
    "m08_unknown_interior_marked_legal", "m09_counts_only_candidates",
    "m10_past_deadline_marked_ok", "m11_all_positives_no_solution",
    "m12_replay_payload_missing")


def _dirty_scripts():
    import subprocess
    try:
        root = Path(__file__).resolve().parents[2]
        out = subprocess.run(
            ["git", "-C", str(root), "status", "--porcelain", "--",
             "scripts"],
            capture_output=True, text=True, timeout=30).stdout
        return [line.strip() for line in out.splitlines() if line.strip()]
    except Exception:  # noqa: BLE001 - stamping is required
        return ["status-failed"]


def _freeze_gate(freeze, a0_verify=None, freeze_sha256=None,
                 archive_binding=None):
    """A0 版本/输入/构建: structural payload verification of the freeze
    document, upgraded by the INDEPENDENT a0_verify result (G2 §3.1).
    Missing keys FAIL (an absent field must never default to an
    empty-and-therefore-clean reading); the two C1 sides must share one
    commit and every side ships non-empty source and install maps.
    Structural pass alone is NOT_EVALUATED — payload bytes must be
    verified by a0_verify (store recomputation + runtime probe closure),
    whose verdict maps: PASS→PASS, REJECTED→FAIL, absent/v1→NE."""
    if freeze is None:
        return "FAIL: no freeze document provided"
    missing = []
    source = freeze.get("source_map") or {}
    for side in ("G", "C1", "T1", "O_CONTROL"):
        if not (source.get(side) or {}).get("commit"):
            missing.append("source_map.%s.commit" % side)
    for side in ("G", "C1"):
        if not (source.get(side) or {}).get("packages"):
            missing.append("source_map.%s.packages" % side)
    installs = freeze.get("install_manifest") or {}
    for side in ("G", "C1", "O_CONTROL"):
        if not installs.get(side):
            missing.append("install_manifest.%s" % side)
    dirty = freeze.get("dirty")
    if not isinstance(dirty, dict) or "G" not in dirty or "C1" not in dirty:
        missing.append("dirty.G/C1 presence")
    else:
        if dirty.get("G"):
            missing.append("dirty.G nonempty")
        if dirty.get("C1"):
            missing.append("dirty.C1 nonempty")
    if freeze.get("problems") is None:
        missing.append("problems presence")
    elif freeze["problems"]:
        missing.append("problems nonempty")
    if (source.get("C1") or {}).get("commit") != \
            (source.get("T1") or {}).get("commit"):
        missing.append("C1/T1 commit divergence")
    if not (freeze.get("manifest") or {}).get("sfree_dir"):
        missing.append("manifest.sfree_dir")
    if missing:
        return "FAIL: %s" % missing[:4]
    # Structural checks pass. The contract's A0 also demands the
    # referenced source/install/input BYTES be hash-verified against the
    # manifest expectations — an independent judge cannot take the freeze
    # generator's own declarations for that, so the verdict comes from
    # the separate a0_verify module (store recomputation + runtime probe
    # closure), never from this structural pass alone.
    if a0_verify is None:
        return ("NOT_EVALUATED: structural checks pass; no independent "
                "A0_VERIFY payload result provided")
    verdict = a0_verify.get("verdict")
    if verdict == "PASS":
        # Binding (reviews F05): a PASS report about a DIFFERENT freeze
        # must not clear this gate. The report must carry the digest of
        # the freeze it verified and it must equal the judged freeze's.
        reported = a0_verify.get("freeze_sha256")
        if not reported or not freeze_sha256:
            return ("FAIL: a0 report lacks freeze binding "
                    "(freeze_sha256 absent)")
        if reported != freeze_sha256:
            return "FAIL: a0 report bound to a different freeze"
        # Run/probe binding (reviews 0031 F05): the report must name
        # THIS archive's groups with per-group ledger digests and probe
        # file digests — a report about another run under the same
        # freeze cannot be reused.
        if not archive_binding:
            return ("FAIL: a0 report cannot be bound to this archive "
                    "(no archive binding available)")
        per_group = a0_verify.get("per_group") or {}
        reported_groups = {entry.get("group") for entry in per_group
                           if isinstance(entry, dict)}
        if reported_groups != set(archive_binding):
            return ("FAIL: a0 report groups %s != judged archive %s"
                    % (sorted(reported_groups)[:3],
                       sorted(archive_binding)[:3]))
        for entry in per_group:
            if not isinstance(entry, dict):
                continue
            want = archive_binding.get(entry.get("group"), {})
            if entry.get("ledger_sha256") != want.get("ledger_sha256") \
                    or entry.get("probes_sha256") != want.get(
                        "probes_sha256"):
                return ("FAIL: a0 report %s binding does not match the "
                        "judged ledger/probes" % entry.get("group"))
        gaps = a0_verify.get("closure_gaps") or []
        if gaps:
            return ("NOT_EVALUATED: payload store/probes closed; "
                    "named gaps: %s (see supplement matrix)" % gaps[:2])
        return "PASS"
    if verdict == "REJECTED":
        return "FAIL: %s" % (a0_verify.get("problems") or
                             ["payload verification rejected"])[:3]
    return "NOT_EVALUATED: %s" % (a0_verify.get("problems")
                                  or ["payload verification not "
                                      "evaluated"])[:3]


#: Every wiring family the selftest MUST emit (reviews F06): removing
#: all negatives and keeping only the healthy control may not pass.
REQUIRED_WIRING = (
    "w01_empty_archive_not_all_pass", "w02_missing_freeze_keys",
    "w03_missing_mutation_id", "w04_named_required_blocks",
    "w05_missing_failure_payload", "w06_missing_regression",
    "w07_missing_required_group", "w08_unknown_terminal_state",
    "w09_base_control_passes",
    "w09_single_byte_corruption", "w10_missing_artifact",
    "w11_same_name_package_replacement", "w12_undeclared_loaded_path",
    "w13_unknown_observation_id_formal",
)


def _a1_gate(selftest, freeze):
    """A1 判官接线: exactly the twelve canonical mutation families, all
    REJECTED; a healthy control judged PASS; the gate-wiring families
    all rejected; and the selftest bound to the freeze's C1 commit."""
    if selftest is None:
        return "FAIL: no selftest result provided"
    missing = []
    if selftest.get("control") != "PASS":
        missing.append("control=%s" % selftest.get("control"))
    mutations = selftest.get("mutations") or {}
    got, need = set(mutations), set(MUTATION_IDS)
    if got != need:
        missing.append("mutation ids +%s/-%s"
                       % (sorted(got - need)[:3], sorted(need - got)[:3]))
    bad = [k for k, v in mutations.items() if v != "REJECTED"]
    if bad:
        missing.append("not rejected: %s" % bad[:3])
    if not selftest.get("archives"):
        missing.append("no healthy archive judged")
    wiring = selftest.get("gate_wiring") or {}
    # The FULL negative family set is required (reviews F06): deleting
    # every negative while keeping the healthy control must fail.
    absent = [w for w in REQUIRED_WIRING if w not in wiring]
    if absent:
        missing.append("wiring families absent: %s" % absent[:4])
    bad_wiring = [k for k, v in wiring.items()
                  if k != "w09_base_control_passes" and v != "REJECTED"]
    if bad_wiring:
        missing.append("gate wiring not rejected: %s" % bad_wiring[:3])
    if wiring.get("w09_base_control_passes") != "PASS":
        missing.append("A0 wiring control not PASS: %s"
                       % wiring.get("w09_base_control_passes"))
    # Each mutation family must fire its OWN frozen rule, not just any
    # problem (reviews F06).
    rule_hits = selftest.get("mutations_rule_hit") or {}
    if set(rule_hits) != set(MUTATION_IDS) or \
            not all(rule_hits.values()):
        missed = [k for k, v in rule_hits.items() if not v]
        missing.append("mutation rule hits incomplete: %s"
                       % (missed[:4] or "absent"))
    got_commit = selftest.get("tree_commit")
    if not got_commit:
        missing.append("selftest lacks tree_commit binding")
    else:
        # The selftest must have run in the tree that is JUDGING (the
        # G2 generation splits historical collection T from the judge T,
        # so an old archive re-judged by a newer tool binds to the judge
        # tree, not the collection freeze). A freeze-v2 run additionally
        # requires judge tree == freeze C1 commit (one generation).
        if got_commit != _JUDGE_TREE:
            missing.append("tree binding %s != judge tree %s"
                           % (got_commit[:8], _JUDGE_TREE[:8]))
        # NOTE: collection provenance is A0's job — a freeze-v2 run's
        # actually-loaded bytes are proven against its OWN freeze by the
        # runtime probe closure, which is exactly why later driver fixes
        # (for later runs) cannot taint an earlier run's provenance.
        # A1 binds the JUDGE side only: the selftest must have run in
        # the tree that is judging (checked above). Demanding
        # freeze-C1 == judge-tree would outlaw the offline re-judging of
        # existing archives that reviews 2026-11-06_2332 explicitly
        # requires, without proving anything A0 has not proven better.
    return "PASS" if not missing else "FAIL: %s" % missing[:4]


def _judge_tree():
    import subprocess
    try:
        root = Path(__file__).resolve().parents[2]
        return subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            capture_output=True, text=True, timeout=30).stdout.strip()
    except Exception:  # noqa: BLE001 - binding is required
        return ""


_JUDGE_TREE = _judge_tree()


#: The frozen expected group sets: gates are checked against THIS
#: universe, not against whatever directories happen to be present —
#: a missing required group or an unknown terminal state can never
#: silently pass (reviews 2026-11-06_1620 counterexamples 2).
EXPECTED_V1_GROUPS = ("V1-G-cuboid", "V1-G-chamfer",
                      "V1-O-cuboid", "V1-O-chamfer")
EXPECTED_V1_O_GROUPS = ("V1-O-cuboid", "V1-O-chamfer")
EXPECTED_V2G_GROUPS = ("V2-G-cuboid",)
EXPECTED_V2O_GROUPS = ("V2-O-cuboid",)


def _case_leg_states(rows):
    """Per required leg of one case: (state, count) with FAIL
    dominance and explicit unknown/missing naming — a PASS on one leg
    can never erase another leg's FAIL/absence (reviews F03)."""
    by_leg = {}
    for t in rows:
        by_leg.setdefault(t.get("subprobe", ""), []).append(t["state"])
    return by_leg


def _evaluate_case(case_id, rows):
    """(ok, detail) for one required ID against its FROZEN leg set."""
    legs = REQUIRED_SUBPROBES.get(case_id, ("main",))
    optional = OPTIONAL_SUBPROBES.get(case_id, ())
    by_leg = _case_leg_states(rows)
    if any(state == "FAIL" for states in by_leg.values()
           for state in states):
        return False, "FAIL"
    for leg in legs:
        states = by_leg.get(leg) or []
        if not states:
            return False, "missing-leg:%s" % leg
        if len(states) > 1:
            return False, "dup-leg:%s x%d" % (leg, len(states))
        if states[0] != "PASS":
            return False, "leg:%s=%s" % (leg, states[0])
    for leg in optional:
        states = by_leg.get(leg) or []
        if any(st == "FAIL" for st in states):
            return False, "FAIL(optional %s)" % leg
    unknown = [leg for leg in by_leg if leg not in legs
               and leg not in optional]
    if unknown:
        return False, "unknown-leg:%s" % unknown[:2]
    return True, "PASS"


def _scope_gate(groups, required, expected_groups):
    """Contract per-ID rule for one gate scope. ``groups`` is a list of
    (group_name, terminals, problems); ``expected_groups`` is the FROZEN
    group universe for the scope. Per reviews F03 every required ID must
    satisfy its FROZEN leg set (each leg exactly one PASS terminal);
    FAIL dominates; unknown/missing legs and unknown states are named —
    an empty scope can never pass."""
    failed, not_hit, missing_groups = [], [], []
    present = {name for name, _, _ in groups}
    for expected in expected_groups:
        if expected not in present:
            missing_groups.append(expected)
    for name, terminals, problems in groups:
        by_id = {}
        for t in terminals:
            by_id.setdefault(t["case_id"], []).append(t)
        for i in required:
            ok, detail = _evaluate_case(i, by_id.get(i) or [])
            if detail == "FAIL":
                failed.append("%s/%s" % (name, i))
            elif not ok:
                not_hit.append("%s/%s(%s)" % (name, i, detail))
        if problems:
            failed.append("%s: %s" % (name, problems[0]))
    if failed:
        return "FAIL: %s" % failed[:4]
    if missing_groups:
        return "NOT_EVALUATED: missing groups %s" % missing_groups
    if not_hit:
        return "NOT_EVALUATED: %s" % not_hit[:6]
    return "PASS"


#: Frozen required regression slice (reviews F08): suite name ->
#: (collected, junit sha256) from the G1-frozen TEST_SELECTION v2. A
#: mutated selection (dropped suite, changed scope, wrong count, wrong
#: digest) cannot pass.
REQUIRED_REGRESSION = {
    "G-CORE": (151, "72be73926b0f043ad6f87d69a5724627f56ec4ba8dcc10810567f29fa77bc4f7"),
    "C1-PACKING": (102, "a05fb002577e7ffb1465ed0016f33722cdaa8894263adeaf939ef0940740e26c"),
    "C1-PLANNING": (52, "8cd2f0407ab4eac95e1a94114c6fabb68389799c719b766c92cd5ea64fa14917"),
    "O-BRIDGE-LIVE": (5, "27a564e437f5c30fe94d22fb862c75ecf79a29b80225e8ddcfdd6424d051f3f9"),
}


def _regression_gate(evidence_root):
    """A5 regression slice: re-verify the contract-required suites'
    junit payloads directly (existence, nonzero cases, zero
    failure/error/skipped) — never the recorded summary strings."""
    if evidence_root is None:
        return "FAIL: no evidence root provided"
    import xml.etree.ElementTree as ET
    root = Path(evidence_root)
    selection = root / "regression/TEST_SELECTION.json"
    if not selection.is_file():
        return "FAIL: missing TEST_SELECTION.json"
    doc = _strict_loads(selection)
    problems, checked = [], 0
    import hashlib as _hl
    for name, (want_count, frozen_sha) in REQUIRED_REGRESSION.items():
        suite = (doc.get("suites") or {}).get(name)
        if suite is None:
            problems.append("%s: required suite absent" % name)
            continue
        if not str(suite.get("scope", "")).startswith("contract-required"):
            problems.append("%s: scope downgraded to %r"
                            % (name, suite.get("scope")))
            continue
        junit = selection.parent / suite["evidence"]["junit"]
        if not junit.is_file():
            problems.append("%s: junit missing" % name)
            continue
        # F08: the expectation is the INDEPENDENTLY frozen digest in
        # this file — never a digest read from the selection document
        # under test (a mutated selection that rewrites its own digest
        # must still fail).
        actual_sha = _hl.sha256(junit.read_bytes()).hexdigest()
        recorded = suite["evidence"].get("junit_sha256")
        if recorded != frozen_sha:
            problems.append("%s: selection digest != independently "
                            "frozen digest" % name)
        if actual_sha != frozen_sha:
            problems.append("%s: junit bytes != independently frozen "
                            "digest" % name)
        cases = list(ET.parse(junit).getroot().iter("testcase"))
        if len(cases) != want_count:
            problems.append("%s: %d cases != frozen %d"
                            % (name, len(cases), want_count))
        bad = [c for c in cases if c.find("failure") is not None
               or c.find("error") is not None
               or c.find("skipped") is not None]
        if bad:
            problems.append("%s: %d failure/error/skipped"
                            % (name, len(bad)))
        checked += 1
    if checked != len(REQUIRED_REGRESSION):
        problems.append("checked %d of %d frozen required suites"
                        % (checked, len(REQUIRED_REGRESSION)))
    if checked == 0:
        problems.append("no contract-required suites in selection")
    return "PASS" if not problems else "FAIL: %s" % problems[:4]


def _replay_gate(evidence_root):
    """A5 failure-payload slice: the injected-failure replay is PASS with
    unchanged inputs AND its ledger carries the actual refusal category
    (payload check, not the summary verdict)."""
    if evidence_root is None:
        return "FAIL: no evidence root provided"
    d = Path(evidence_root) / "replay/k04main"
    if not (d / "REPLAY_RESULT.json").is_file():
        return "FAIL: missing failure-replay evidence (replay/k04main)"
    result = _strict_loads(d / "REPLAY_RESULT.json")
    if (result.get("verdict") != "PASS"
            or result.get("inputs_unchanged") is not True
            or result.get("same_category") is not True):
        return "FAIL: replay verdict/inputs/category %s/%s/%s" % (
            result.get("verdict"), result.get("inputs_unchanged"),
            result.get("same_category"))
    # The replay must be CASE-JOINED with LEGAL terminal states (F09):
    # one case whose terminal AND outcome are PASS, tied to a refusal
    # in a consumer-path row; an unrelated or FAIL-terminal replay is a
    # failure, not evidence.
    events = load_events(d)
    by_case = {}
    for e in events:
        by_case.setdefault(e.get("case_id", ""), []).append(e)
    joined = [
        c for c, rows in by_case.items()
        if c
        and any(r["event_kind"] == "case_terminal"
                and r.get("state") == "PASS" for r in rows)
        and any(r["event_kind"] == "replay_outcome"
                and r.get("state") == "PASS"
                and str(r.get("reason", "")) == "PASS" for r in rows)
        and any(r["event_kind"] == "placement_request"
                and "VALIDITY_GATE_CLOSED" in str(r.get("reason", ""))
                for r in rows)
    ]
    if not joined:
        return ("FAIL: replay ledger lacks a case-joined "
                "PASS terminal+outcome+refusal payload")
    original = result.get("original_case")
    if original and joined and original not in joined:
        return "FAIL: replay original_case %s not the joined case %s" % (
            original, joined[:2])
    if not original:
        return "FAIL: replay records no original-case correspondence"
    # Byte-level input equality: per-input digests must be RECORDED and
    # match the files on disk (F09). Missing digests is NOT_EVALUATED —
    # a named limitation comment can never substitute the evidence.
    digests = result.get("input_digests") or {}
    if not digests:
        return ("NOT_EVALUATED: replay artifact predates per-input "
                "digest recording — see supplement matrix")
    import hashlib as _hl
    for path_text, want in sorted(digests.items()):
        path = Path(path_text)
        if not path.is_absolute():
            path = d / path
        if not path.is_file():
            return ("NOT_EVALUATED: replay input file absent: %s"
                    % path_text[:80])
        if _hl.sha256(path.read_bytes()).hexdigest() != want:
            return "FAIL: replay input bytes != recorded digest (%s)" \
                % path_text[:60]
    return "PASS"


def _fail_evidence_gate(groups):
    """A5 completeness slice: every FAIL terminal must be corroborated by
    a consumer-path event for its case (placement_request / stage /
    hook). Integrates alone cannot substantiate a consumer-bound failure
    — formal_run02's R05-O (a no-reply FAIL with three integrates and
    zero consumer events) is INCOMPLETE_EVIDENCE, not a clean fail."""
    issues = _fail_evidence_issues(groups)
    if issues:
        first = issues[0]
        return ("INCOMPLETE_EVIDENCE: %s/%s FAIL lacks consumer-path "
                "events (+%d more)" % (first["group"], first["case"],
                                       len(issues) - 1))
    return "PASS"


def _fail_evidence_issues(groups):
    """Structured form: one issue per FAIL terminal lacking consumer-
    path evidence (scope/rule/group/case — reviews 2026-11-06_2332)."""
    issues = []
    for name, events, _ in groups:
        for t in _terminals(events):
            if t["state"] != "FAIL":
                continue
            rows = [e for e in events
                    if e.get("case_id") == t["case_id"]
                    and e.get("event_kind") not in (
                        "case_terminal", "case_complete", "input")]
            if not any(e.get("event_kind") in (
                    "placement_request", "stage_marker", "hook",
                    "replay_outcome") for e in rows):
                issues.append({
                    "scope": "case-local",
                    "rule": "fail_evidence",
                    "group": name,
                    "case": t["case_id"],
                    "detail": "FAIL terminal lacks consumer-path events",
                })
    return issues


def judge_archive(archive, out_dir, selftest_path=None, freeze_path=None,
                  evidence_root=None, a0_verify_path=None):
    """Formal A0-A5 verdict per contract c7039905 §5 (recomputed, never
    self-reported).

    Every group ledger is judged on its own (integrity rules are per
    (run, backend, hull, case, subprobe)). Gates follow the contract's
    own names and scopes: A2 = G's twelve V2 cases all hit; A3 = every
    required consumer's V1 forty-case set on BOTH backends and hulls;
    A4 = the fixed-O side (V1 O groups plus V2-O's twelve); A5 = ledger
    integrity plus required regression payloads plus the failure replay
    plus FAIL-evidence completeness. A required case that is missing or
    NOT_EVALUATED keeps its gate NOT_EVALUATED — named IDs, never a
    quantity floor."""
    archive = Path(archive)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    gates = {}
    groups = []          # (name, events, problems)
    known_ids = set()
    freeze_doc = json.loads(Path(freeze_path).read_text()) if freeze_path \
        else None
    fixture_manifest_found = False
    if freeze_doc:
        # Parity with the selftest input (reviews F06): the formal path
        # must flag unknown observation ids exactly like the selftest
        # does, using the freeze's own fixture manifest. An EMPTY or
        # absent manifest never disables the check (known_ids or None
        # was a fail-open hole) — it becomes a named group problem.
        for manifest_json in Path(freeze_doc.get("manifest", {})
                                  .get("sfree_dir", ".")).glob(
                                      "*/MANIFEST.json"):
            fixture_manifest_found = True
            for frame in json.loads(
                    manifest_json.read_text()).get("frames", []):
                known_ids.add(Path(frame.get("path", "")).stem)
    for ledger in sorted(archive.glob("**/events.jsonl")):
        group_events = load_events(ledger)
        group_problems = judge_events(group_events, known_ids=known_ids)
        if freeze_doc and not fixture_manifest_found:
            group_problems.append(
                "fixture manifest absent under the freeze's sfree_dir — "
                "input identity cannot be verified")
        groups.append((ledger.parent.name, group_events, group_problems))
    if not groups:
        gates["A5"] = "FAIL: no group ledgers in the archive"

    v1_groups = [(n, _terminals(e), p) for n, e, p in groups
                 if n.startswith("V1-")]
    v2_groups = [(n, _terminals(e), p) for n, e, p in groups
                 if n.startswith("V2-")]
    gates["A2"] = _scope_gate(
        [g for g in v2_groups if "-G-" in g[0]], V2_REQUIRED,
        EXPECTED_V2G_GROUPS)
    gates["A3"] = _scope_gate(v1_groups, V1_REQUIRED, EXPECTED_V1_GROUPS)
    # A4 固定O回归: the O half of V1 (both hulls, the 40-case set each)
    # plus V2-O's twelve — both scopes per-ID against the frozen group
    # universe, FAIL dominating, anything not exactly PASS stays NE.
    o_v1 = [g for g in v1_groups if "-O-" in g[0]]
    o_v2 = [g for g in v2_groups if "-O-" in g[0]]
    a4_v1 = _scope_gate(o_v1, V1_REQUIRED, EXPECTED_V1_O_GROUPS)
    a4_v2 = _scope_gate(o_v2, V2_REQUIRED, EXPECTED_V2O_GROUPS)
    if a4_v1.startswith("FAIL") or a4_v2.startswith("FAIL"):
        gates["A4"] = "FAIL: %s" % "; ".join(
            g for g in (a4_v1, a4_v2) if g.startswith("FAIL"))
    elif a4_v1 != "PASS" or a4_v2 != "PASS":
        gates["A4"] = "NOT_EVALUATED: %s" % "; ".join(
            g for g in (a4_v1, a4_v2) if g != "PASS")
    else:
        gates["A4"] = "PASS"

    try:
        freeze = _strict_loads(freeze_path) if freeze_path else None
        selftest = _strict_loads(selftest_path) \
            if selftest_path else None
    except (OSError, ValueError) as error:
        print("judge: invalid input document: %s (exit 2)" % error)
        raise SystemExit(2)
    # Content binding (reviews F02): the verdict records the DIGEST of
    # every input it actually read — a merger can then prove the
    # ledgers it evaluates are the ones this verdict judged.
    import hashlib as _hashlib

    def _file_sha(path):
        return _hashlib.sha256(Path(path).read_bytes()).hexdigest()

    inputs_binding = {
        "groups": {
            ledger.parent.name: {"ledger_sha256": _file_sha(ledger)}
            for ledger in sorted(archive.glob("**/events.jsonl"))},
        "archive": str(archive.resolve()),
    }
    for role, path in (("freeze", freeze_path),
                       ("selftest", selftest_path),
                       ("a0_verify", a0_verify_path)):
        if path:
            inputs_binding["%s_sha256" % role] = _file_sha(path)
    if freeze_doc:
        inputs_binding["freeze_commits"] = {
            side: (freeze_doc.get("source_map", {})
                   .get(side, {}).get("commit"))
            for side in ("G", "C1", "T1", "O_CONTROL")}
        # PRODUCT source digest: the G/C1/O package file maps, NOT the
        # worktree commits — the C1 worktree hosts the tool too, so its
        # commit advances with T1 while the product bytes stay frozen.
        packages = {
            side: (freeze_doc.get("source_map", {}).get(side, {})
                   .get("packages", {}))
            for side in ("G", "C1")}
        # O has no source packages in the freeze (read-only control);
        # its frozen IDENTITY is the install closure (F02 0031: the
        # digest previously read a nonexistent O_CONTROL.packages and
        # contributed nothing).
        packages["O_CONTROL"] = freeze_doc.get(
            "install_manifest", {}).get("O_CONTROL", {})
        inputs_binding["product_source_sha256"] = _hashlib.sha256(
            json.dumps(packages, sort_keys=True).encode()).hexdigest()
    judge_identity = {
        "tree_commit": _JUDGE_TREE,
        "dirty_scripts": _dirty_scripts(),
        "judge_sha256": _file_sha(Path(__file__)),
    }
    terminals = [t for _, evs, _ in groups for t in _terminals(evs)]
    v2n = sorted(t["case_id"] + "/" + t.get("backend", "")
                 for t in terminals
                 if t["case_id"].startswith("R")
                 and t["state"] == "NOT_EVALUATED")
    v2f = [t["case_id"] + "/" + t.get("backend", "") for t in terminals
           if t["case_id"].startswith("R") and t["state"] == "FAIL"]
    a0v = _strict_loads(a0_verify_path) if a0_verify_path else None
    import hashlib as _h2
    archive_dir_for_binding = Path(inputs_binding["archive"])
    archive_binding = {
        group: {
            "ledger_sha256": meta["ledger_sha256"],
            "probes_sha256": _h2.sha256(
                (archive_dir_for_binding / group /
                 "MODULE_PROBES.json").read_bytes()).hexdigest()
            if (archive_dir_for_binding / group /
                "MODULE_PROBES.json").is_file() else None}
        for group, meta in inputs_binding["groups"].items()}
    gates["A0"] = _freeze_gate(
        freeze, a0v,
        freeze_sha256=inputs_binding.get("freeze_sha256"),
        archive_binding=archive_binding)
    if a0v is not None:
        gates["A0_verify_detail"] = "%s: probes=%s problems=%s" % (
            a0v.get("verdict"), (a0v.get("checks") or {}).get("probes"),
            (a0v.get("problems") or [])[:2])
    gates["A1"] = _a1_gate(selftest, freeze)

    if "A5" not in gates:
        # EVERY sub-check runs and is reported (reviews 2026-11-06_2332:
        # an if/elif first-defect-wins string let a local K03/E4 gap
        # hide a simultaneous GLOBAL regression/replay failure — the
        # merger then excused the local one and the global hid behind
        # it). The headline string lists ALL failing sub-checks; the
        # structured detail carries scope/rule/group/case per issue.
        all_problems = [p for _, _, probs in groups for p in probs]
        fail_issues = _fail_evidence_issues(groups)
        reg = _regression_gate(evidence_root)
        rep = _replay_gate(evidence_root)
        subchecks = {
            "fail_evidence": {
                "scope": "case-local" if fail_issues else "clean",
                "state": ("INCOMPLETE_EVIDENCE" if fail_issues
                          else "PASS"),
                "issues": fail_issues},
            "ledger": {
                "scope": "global", "state": ("FAIL" if all_problems
                                             else "PASS"),
                "issues": [{"scope": "global", "rule": "ledger",
                            "detail": p} for p in all_problems]},
            "regression": {
                "scope": "global",
                "state": str(reg).split(":", 1)[0].strip(),
                "detail": str(reg)[:200], "issues": []},
            "replay": {
                "scope": "global",
                "state": str(rep).split(":", 1)[0].strip(),
                "detail": str(rep)[:200], "issues": []},
        }
        failing = [(name, sc) for name, sc in subchecks.items()
                   if sc["state"] != "PASS"]
        if failing:
            gates["A5"] = "; ".join(
                "%s=%s" % (name, sc["state"][:70])
                for name, sc in failing)
        else:
            gates["A5"] = "PASS"
        a5_subchecks = subchecks

    try:
        a5_subchecks
    except NameError:
        a5_subchecks = None
    detail = ({"A5": a5_subchecks} if a5_subchecks else {})
    verdict = {"archive": str(archive), "gates": gates,
               "gates_detail": detail,
               "inputs": inputs_binding,
               "judge_identity": judge_identity,
               "named_not_evaluated": v2n,
               "v2_failed": v2f,
               "terminals": {
                   "V1_pass": sum(1 for t in terminals
                                  if t["case_id"].startswith("K")
                                  and t["state"] == "PASS"),
                   "V1_total": sum(1 for t in terminals
                                   if t["case_id"].startswith("K")),
                   "V2_pass": sum(1 for t in terminals
                                  if t["case_id"].startswith("R")
                                  and t["state"] == "PASS"),
                   "V2_total": sum(1 for t in terminals
                                   if t["case_id"].startswith("R"))}}
    (out / "VERDICT.json").write_text(json.dumps(verdict, indent=1))
    return verdict
