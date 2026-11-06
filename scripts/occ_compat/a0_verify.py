"""a0_verify — independent A0 payload verification (G2 P1, §3.1/§3.2).

Separate from the freeze generator ON PURPOSE: this module never trusts
FREEZE.json's own declarations beyond using them as the *expected* set.
It recomputes every hash from the content-addressed store, proves the
store is internally consistent (shard name == sha256 of its bytes),
proves every manifest entry's bytes exist, and proves the runtime
identity probes (modules imported and executables launched by the
collected run) bind to manifest entries with matching hashes.

Old freeze-v1 documents have no store and no executable coverage, so
they can only reach NOT_EVALUATED — history is never upgraded by
re-hashing the present.
"""
import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from occ_compat.inventory import sha256_file  # noqa: E402

SCHEMA = "a0-verify-v1"


def _iter_shards(store_dir):
    for shard in sorted(store_dir.glob("*/*")):
        if shard.is_file():
            yield shard


def _probe_entries(probe_paths, problems):
    """Load MODULE_PROBES/IMPORT_PROBES json files into one probe list.
    A missing probe file is a NAMED problem, never a silent skip
    (reviews F04)."""
    probes = []
    for path in probe_paths:
        path = Path(path)
        if not path.exists():
            problems.append("probe file missing: %s" % path)
            continue
        doc = _strict_loads(path)
        rows = doc if isinstance(doc, list) else (
            doc.get("probes") or doc.get("modules") or [])
        for row in rows:
            if isinstance(row, dict) and row.get("path"):
                probes.append(row)
    return probes


def _strict_loads(path):
    def _no_dupes(pairs):
        out = {}
        for key, value in pairs:
            if key in out:
                raise ValueError("duplicate JSON key %r" % key)
            out[key] = value
        return out
    return json.loads(Path(path).read_text(),
                      object_pairs_hook=_no_dupes)


def verify(freeze_path, probe_paths, ledger_paths=()):
    freeze = _strict_loads(freeze_path)
    freeze_path = Path(freeze_path)
    freeze_dir = freeze_path.resolve().parent
    import hashlib
    result = {
        "schema": SCHEMA,
        "freeze": str(freeze_path),
        "freeze_sha256": hashlib.sha256(
            freeze_path.read_bytes()).hexdigest(),
        "freeze_schema": freeze.get("schema"),
        "probes_sources": [str(p) for p in probe_paths],
        "problems": [],
        "checks": {},
    }

    if freeze.get("schema") != "freeze-v2":
        result["verdict"] = "NOT_EVALUATED"
        result["problems"].append(
            "freeze schema %r lacks the content-addressed store and "
            "executable coverage required for payload verification; "
            "historical runs stay not-evaluated (G2 §3.2: re-hashing the "
            "present never proves what a past run loaded)"
            % freeze.get("schema"))
        return result
    if freeze.get("problems"):
        result["verdict"] = "REJECTED"
        result["problems"].append(
            "freeze records unresolved problems: %s" % freeze["problems"][:3])
        return result

    store_dir = freeze_dir / "store"
    if not store_dir.is_dir():
        result["verdict"] = "REJECTED"
        result["problems"].append("content-addressed store missing")
        return result

    # 1. store self-consistency: shard name must equal sha256(bytes)
    shard_hashes = {}
    corrupt = []
    for shard in _iter_shards(store_dir):
        digest = sha256_file(shard)
        if digest != shard.name:
            corrupt.append(shard.name)
        shard_hashes[digest] = shard
    result["checks"]["store_shards"] = len(shard_hashes)
    result["checks"]["store_corrupt"] = len(corrupt)
    if corrupt:
        result["problems"].append(
            "%d store shard(s) fail name==sha256 (first: %s)"
            % (len(corrupt), corrupt[0]))

    # 2. every declared entry's bytes exist in the store
    installs = freeze.get("install_manifest") or {}
    missing = []
    referenced = set()
    for side, entries in installs.items():
        for key, entry in entries.items():
            digest = entry.get("sha256") if isinstance(entry, dict) else entry
            referenced.add(digest)
            if digest not in shard_hashes:
                missing.append("%s/%s" % (side, key))
    source = freeze.get("source_map") or {}
    for side in ("G", "C1"):
        for package, files in (source.get(side) or {}).get(
                "packages", {}).items():
            for rel, digest in files.items():
                referenced.add(digest)
                if digest not in shard_hashes:
                    missing.append("%s/%s/%s" % (side, package, rel))
    for name, digest in (source.get("T1") or {}).get("scripts", {}).items():
        referenced.add(digest)
        if digest not in shard_hashes:
            missing.append("T1/%s" % name)
    result["checks"]["declared_missing"] = len(missing)
    if missing:
        result["problems"].append(
            "%d declared payload(s) missing from store (first: %s)"
            % (len(missing), missing[0]))

    # 3. no undeclared content rides along in the store
    extra = [h for h in shard_hashes if h not in referenced]
    result["checks"]["store_unreferenced"] = len(extra)
    if extra:
        result["problems"].append(
            "%d unreferenced store shard(s) (first: %s)"
            % (len(extra), extra[0]))

    # 4. runtime identity closure: every probed module/executable binds
    #    to a manifest entry with a matching hash — install entries for
    #    sourced-overlay processes, source_map entries for the nodes the
    #    chains launch straight from the C1/G source trees. No
    #    undeclared loaded path, no cross-wired package, no post-freeze
    #    byte drift.
    probes = _probe_entries(probe_paths, result["problems"])
    # Coverage floor (reviews F04): zero probes or a missing probe file
    # can never PASS, and a real chain run always loads BOTH imported
    # modules AND spawned executables.
    if not probe_paths:
        result["problems"].append("no probe files provided at all")
    kinds = {p.get("kind") for p in probes}
    if "module" not in kinds:
        result["problems"].append(
            "no module probes — runtime import identity unproven")
    if "executable" not in kinds:
        result["problems"].append(
            "no executable probes — spawned entry identity unproven")
    fm = freeze.get("manifest") or {}

    def _bind(path):
        """Return the manifest digest for an absolute artifact path, or
        (None, reason) when the path binds to nothing declared."""
        for side, key in (("G", "g_install"), ("C1", "c1_install"),
                          ("O_CONTROL", "o_install")):
            if not fm.get(key):
                continue
            prefix = str(Path(fm[key]).resolve())
            if path.startswith(prefix + os.sep):
                entry_key = "%s/%s" % (side, path[len(prefix) + 1:])
                digest = flat_installs.get(entry_key)
                return digest, entry_key
        for side, key in (("G", "g_root"), ("C1", "c1_root")):
            if not fm.get(key):
                continue
            root = str(Path(fm[key]).resolve())
            if not path.startswith(root + os.sep + "src" + os.sep):
                continue
            rel = path[len(root) + 5:]  # drop "<root>/src/"
            parts = rel.split(os.sep, 1)
            if len(parts) != 2:
                return None, "malformed-source-path"
            package_dir, rest = parts
            for alias in (package_dir, package_dir + "_scripts"):
                files = (source.get(side) or {}).get(
                    "packages", {}).get(alias)
                if isinstance(files, dict) and rest in files:
                    return files[rest], "%s/src/%s/%s" % (
                        side, package_dir, rest)
        # T1's own scripts tree: the client interpreter loads occ_compat
        # modules straight from <c1_root>/scripts — bound against the
        # T1 section of the source map.
        c1_root = str(Path(fm.get("c1_root", ".")).resolve())
        if path.startswith(c1_root + os.sep + "scripts" + os.sep):
            rel = path[len(c1_root) + len("/scripts/"):]
            scripts = (source.get("T1") or {}).get("scripts", {})
            if rel in scripts:
                return scripts[rel], "T1/scripts/%s" % rel
            return None, "t1-script-not-declared:%s" % rel
        return None, "no-declared-prefix"

    flat_installs = {}
    for side, entries in installs.items():
        for key, entry in entries.items():
            digest = entry.get("sha256") if isinstance(entry, dict) else entry
            flat_installs["%s/%s" % (side, key)] = digest
    unmatched, mismatched = [], []
    for probe in probes:
        path = str(Path(probe["path"]).resolve())
        digest, key = _bind(path)
        if digest is None:
            unmatched.append("%s(%s)" % (key, probe["path"]))
            continue
        if digest != probe.get("sha256"):
            mismatched.append(key)
    result["checks"]["probes"] = len(probes)
    result["checks"]["probes_unmatched"] = len(unmatched)
    result["checks"]["probes_hash_mismatch"] = len(mismatched)
    if unmatched:
        result["problems"].append(
            "%d probe(s) bind to no manifest entry (first: %s) — "
            "undeclared loaded path" % (len(unmatched), unmatched[0]))
    if mismatched:
        result["problems"].append(
            "%d probe(s) hash-mismatch the manifest (first: %s) — "
            "cross-wired or post-freeze drift" % (len(mismatched),
                                                  mismatched[0]))
    if probes and len(probes) == len(unmatched):
        result["problems"].append("every probe unmatched — probe/manifest "
                                  "prefix binding is broken")

    # Input closure (reviews F04): every observation id the run actually
    # integrated must belong to the frozen fixture frame set — verified
    # against the manifest's OWN recorded per-frame digests (the frame
    # files' current bytes must still match, so the closure is honest
    # about the bytes that exist, never a post-hoc hash).
    closure_gaps = []
    if ledger_paths:
        known = {}
        frames_root = Path(fm.get("sfree_dir", "."))
        for manifest_json in sorted(frames_root.glob("*/MANIFEST.json")):
            for frame in json.loads(
                    manifest_json.read_text()).get("frames", []):
                stem = Path(str(frame.get("path", ""))).stem
                known[stem] = frame
        unknown_ids, bad_bytes = [], []
        for ledger_path in ledger_paths:
            for line in Path(ledger_path).read_text().splitlines():
                if not line.strip():
                    continue
                row = json.loads(line)
                if row.get("event_kind") != "integrate":
                    continue
                if row.get("case_id") == "K07":
                    # K07's foreign observation is part of the frozen
                    # case contract (the unit-lie probe) — the judge's
                    # known_ids check carries the same exemption.
                    continue
                stem = str(row.get("request_id", "")).split("#")[0]
                frame = known.get(stem)
                if frame is None:
                    unknown_ids.append(stem)
                    continue
                # F04 deep closure: the frame FILE must exist and its
                # CURRENT bytes must match the historically recorded
                # digest — a matched id with a missing file or a
                # deliberately-invalid digest is rejected.
                frame_path = Path(str(frame.get("path", "")))
                if not frame_path.is_file():
                    bad_bytes.append("%s (frame file missing)" % stem)
                    continue
                digest = sha256_file(frame_path)
                if digest != frame.get("sha256"):
                    bad_bytes.append("%s (bytes != recorded digest)"
                                     % stem)
        result["checks"]["ledger_inputs_unmatched"] = len(unknown_ids)
        result["checks"]["ledger_inputs_bad_bytes"] = len(bad_bytes)
        if unknown_ids:
            result["problems"].append(
                "%d integrated observation id(s) outside the frozen "
                "fixture set (first: %s)"
                % (len(unknown_ids), unknown_ids[0]))
        if bad_bytes:
            result["problems"].append(
                "%d integrated input(s) fail byte verification (first: "
                "%s)" % (len(bad_bytes), bad_bytes[0]))

    # TF / external config closure (F04): these inputs were never
    # digested at freeze time, so no historical expectation exists to
    # verify against. Named as closure gaps — never silently accepted,
    # never "verified" by hashing today's bytes.
    for role, key in (("scene_tf", "scene_tf"), ("o_config", "o_config"),
                      ("o_geometry", "o_geometry"),
                      ("prep_inputs", "prep_inputs_json")):
        path = fm.get(key)
        if not path:
            continue
        if not Path(path).exists():
            result["problems"].append(
                "%s input path absent: %s" % (role, str(path)[:80]))
        else:
            closure_gaps.append(
                "%s: no historical digest recorded at freeze time" % role)

    result["closure_gaps"] = closure_gaps
    if closure_gaps:
        # The proven aspects stand; the unprovable ones keep the verdict
        # from claiming full input closure.
        result["verdict"] = "NOT_EVALUATED"
        result["problems"] = result["problems"]  # unchanged
        return result
    result["verdict"] = "REJECTED" if result["problems"] else "PASS"
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--freeze", required=True)
    parser.add_argument("--probes", nargs="*", default=[],
                        help="MODULE_PROBES/IMPORT_PROBES json files")
    parser.add_argument("--ledgers", nargs="*", default=[],
                        help="run events.jsonl files for input closure")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    result = verify(args.freeze, args.probes, args.ledgers)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=1))
    print("a0_verify: %s (%s)" % (result["verdict"],
                                  result["problems"][:2] or "clean"))
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
