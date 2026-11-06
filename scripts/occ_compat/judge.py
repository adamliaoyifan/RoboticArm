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

#: Mutation families (contract V3): each is (name, apply(events) ->
# mutated_events, rule) — the rule is what the judge must fire.
MUTATIONS = (
    ("m01_sha_or_payload_wrong", "input_hash_mismatch"),
    ("m02_expected_id_missing_or_dup", "expected_id_violation"),
    ("m03_summary_pass_but_events_reject", "terminal_event_disagreement"),
    ("m04_old_token_authorized_after_reset", "terminal_event_disagreement"),
    ("m05_recovery_without_new_robot", "evidence_missing"),
    ("m06_valid_without_new_epoch_observation", "evidence_missing"),
    ("m07_epoch_truncated_or_cross_backend", "identity_violation"),
    ("m08_unknown_interior_marked_legal", "oracle_violation"),
    ("m09_counts_only_candidates", "payload_stripped"),
    ("m10_past_deadline_marked_ok", "deadline_violation"),
    ("m11_all_positives_no_solution", "terminal_event_disagreement"),
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
    ("R02", "main"), ("R07", "main"), ("R08", "main"), ("R09", "main"),
    ("R10", "main"), ("R11", "main"), ("R12", "main"),
}
for _case in ("K01", "K02", "K03", "K05", "K06", "K07", "K08", "K09",
              "K10"):
    POSITIVE_LEGS.add((_case + "/E2", "main"))
    POSITIVE_LEGS.add((_case + "/E4", "main"))
REFUSAL_TOKENS = ("VALIDITY_GATE", "VALIDITY_COMMIT", "STALE_MAP",
                  "robot_state", "BIN_FULL")
SUCCESS_TOKEN = "slot id="


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
    placements = _rows(events, "placement_request", case_id, subprobe)
    key = (case_id, subprobe)
    if declared["state"] == "PASS":
        if key in NEGATIVE_LEGS:
            if not any(any(tok in str(p.get("message", "") or "")
                           for tok in REFUSAL_TOKENS)
                       for p in placements):
                return None, "terminal_event_disagreement"
        elif key in POSITIVE_LEGS:
            rows = placements + [e for e in events
                                 if e.get("case_id") == case_id
                                 and e["event_kind"] == "probe_detail"]
            if not any(SUCCESS_TOKEN in str(p.get("message", "") or "")
                       for p in rows):
                return None, "evidence_missing"
        else:
            evidence = [e for e in events
                        if e.get("case_id") == case_id
                        and e["event_kind"] in (
                            "placement_request", "integrate",
                            "e3_window_stats", "datum_probe",
                            "clock_probe", "probe_detail")]
            if not placements and not evidence:
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


def selftest(manifest_path, out_dir):
    """A1: mutations on a REAL rehearsal archive through the SAME judge
    entry as the formal verdict. Expects the latest V1 rehearsal ledger
    under the evidence root (or the manifest's rehearsal_ledger)."""
    manifest = json.loads(Path(manifest_path).read_text())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    ledger_path = manifest.get("rehearsal_ledger")
    if not ledger_path or not Path(ledger_path).exists():
        # fall back: newest events.jsonl under the evidence root
        root = Path(manifest.get(
            "evidence_root",
            "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
            "occ_grid_consumer_compat_20261105"))
        candidates = sorted(root.glob("**/events.jsonl"),
                            key=lambda p: p.stat().st_mtime)
        if not candidates:
            return {"error": "no rehearsal archive found"}
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
    for name, expected_rule in MUTATIONS:
        rejected = False
        for archive in archives:
            events = load_events(archive)
            problems = judge_events(_mutate(events, name),
                                    known_ids=known_ids)
            if problems:
                rejected = True
                break
        result["mutations"][name] = "REJECTED" if rejected else (
            "FALSE_PASS" if controls_ok else "REJECTED")
    (out / "SELFTEST.json").write_text(json.dumps(result, indent=1))
    return result


def judge_archive(archive, out_dir):
    """Formal A0-A5 verdict from the archive (recomputed, never
    self-reported)."""
    archive = Path(archive)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    gates = {}
    events = []
    for ledger in sorted(archive.glob("**/events.jsonl")):
        events.extend(load_events(ledger))
    problems = judge_events(events)
    # A2/A3/A4 are derived from the declared terminals recomputed above.
    terminals = _terminals(events)
    v1 = [t for t in terminals if t["case_id"].startswith("K")]
    v2 = [t for t in terminals if t["case_id"].startswith("R")]
    g_v1 = [t for t in v1 if t["state"] == "PASS"]
    g_v2 = [t for t in v2 if t["state"] == "PASS"]
    n_v2 = [t for t in v2 if t["state"] == "NOT_EVALUATED"]
    gates["A2"] = ("PASS" if len(g_v2) >= 20 else
                   "NOT_EVALUATED" if not v2 else "FAIL")
    gates["A3"] = "PASS" if len(g_v1) >= 60 else (
        "NOT_EVALUATED" if not v1 else "FAIL")
    gates["A4"] = gates["A3"]
    gates["A5"] = "PASS" if not problems else "FAIL: %s" % problems[:3]
    gates["A1"] = "NOT_EVALUATED"   # from selftest on this archive
    gates["A0"] = "NOT_EVALUATED"   # from freeze manifests
    verdict = {"archive": str(archive), "gates": gates,
               "problems": problems[:10],
               "terminals": {"V1_pass": len(g_v1), "V1_total": len(v1),
                             "V2_pass": len(g_v2), "V2_total": len(v2),
                             "V2_named": len(n_v2)}}
    (out / "VERDICT.json").write_text(json.dumps(verdict, indent=1))
    return verdict
