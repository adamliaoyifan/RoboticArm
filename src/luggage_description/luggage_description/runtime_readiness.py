"""Consumer-owned readiness: current peer leases and immutable process identity.

Call require_ready immediately before side effects. A heartbeat only refreshes
an unchanged incarnation. A restart latches the gate closed until explicit
re-admission; it must never silently resume a carrying sequence.
"""
from copy import deepcopy
import time
import math
from pathlib import Path

from .runtime_profile import IDENTITY_FIELDS, canonical_hash, reject, validate_identity


class ReadinessGate:
    def __init__(self, expected_identity, peers, install_root, lease_seconds=2.0,
                 clock=time.monotonic):
        self.identity = validate_identity(expected_identity)
        if (not isinstance(peers, (list, tuple)) or not peers
                or any(not isinstance(peer, str) or not peer.strip() for peer in peers)
                or len(peers) != len(set(peers))):
            reject("INVALID_PEER_SET", "peers", "unique nonempty peer set", peers)
        if (type(lease_seconds) not in (int, float) or not math.isfinite(lease_seconds)
                or lease_seconds <= 0):
            reject("INVALID_LEASE", "lease_seconds", "positive", lease_seconds)
        self.peers = tuple(peers)
        self.install_root = Path(install_root).resolve()
        self.lease_seconds = lease_seconds
        self.clock = clock
        self.reports = {}
        self.revoked = None
        self.transitions = []
        self._state = None

    def _transition(self, reason):
        if reason != self._state:
            self.transitions.append({"at_monotonic": self.clock(), "reason": reason})
            self._state = reason
        return reason

    def observe(self, report):
        if not isinstance(report, dict):
            self.revoked = "INVALID_PROCESS_REPORT"
            reject(self.revoked, "report", "mapping", type(report).__name__)
        peer = report.get("peer")
        if peer not in self.peers:
            reject("UNKNOWN_PEER", "peer", self.peers, peer)
        try:
            identity = validate_identity(report.get("identity"))
            for key in ("contract_version", *IDENTITY_FIELDS):
                if identity[key] != self.identity[key]:
                    code = "STALE_ATLAS" if key.startswith("atlas_") else "PEER_IDENTITY_MISMATCH"
                    reject(code, key, self.identity[key], identity[key])
            for key in ("incarnation", "resolved_config", "artifacts", "package_prefixes"):
                if not report.get(key):
                    reject("MISSING_PROCESS_PROVENANCE", key, "nonempty", report.get(key))
            for key, expected_type in (("incarnation", str), ("resolved_config", dict),
                                       ("artifacts", list), ("package_prefixes", dict)):
                if not isinstance(report[key], expected_type):
                    reject("INVALID_PROCESS_REPORT", key, expected_type.__name__, type(report[key]).__name__)
            kinds = set()
            for artifact in report["artifacts"]:
                if not isinstance(artifact, dict) or not isinstance(artifact.get("path"), str):
                    reject("INVALID_PROCESS_REPORT", "artifact", "mapping with path", artifact)
                path = Path(artifact["path"]).resolve()
                if not path.is_relative_to(self.install_root):
                    reject("WRONG_OVERLAY", "path", str(self.install_root), str(path))
                digest = artifact.get("sha256", "")
                if not isinstance(digest, str) or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
                    reject("MISSING_PROCESS_PROVENANCE", "sha256", "SHA256", digest)
                kinds.add(artifact.get("kind"))
            if not {"module", "generated_message", "type_support"}.issubset(kinds):
                reject("MISSING_PROCESS_PROVENANCE", "artifact kinds", "module/message/type_support", sorted(kinds))
            for prefix in report["package_prefixes"].values():
                if not Path(prefix).resolve().is_relative_to(self.install_root):
                    reject("WRONG_OVERLAY", "package_prefix", str(self.install_root), prefix)
            prior = self.reports.get(peer)
            fingerprint = canonical_hash(report)
            if prior and prior["fingerprint"] != fingerprint:
                code = ("PEER_RESTART" if prior["report"]["incarnation"] != report["incarnation"]
                        else "PEER_IDENTITY_CHANGED")
                reject(code, peer, prior["fingerprint"], fingerprint)
            self.reports[peer] = {"report": deepcopy(report), "fingerprint": fingerprint,
                                  "received": self.clock()}
        except Exception as error:
            self.revoked = getattr(error, "code", "INVALID_PROCESS_REPORT")
            self._transition(self.revoked)
            raise

    def reason(self, providers=None):
        if self.revoked:
            return self._transition(self.revoked)
        now = self.clock()
        for peer in self.peers:
            if peer not in self.reports:
                return self._transition("MISSING_PEER")
            if now - self.reports[peer]["received"] > self.lease_seconds:
                # An expired admitted peer cannot re-enable an interrupted request.
                self.revoked = "PEER_LEASE_EXPIRED"
                return self._transition(self.revoked)
        if providers is None:
            return self._transition("MISSING_ENDPOINT_GRAPH")
        for endpoint, expected_type in self.identity["endpoints"].items():
            actual = providers.get(endpoint, [])
            if len(actual) > 1:
                self.revoked = "DUPLICATE_PROVIDER"
                return self._transition(self.revoked)
            if not actual:
                return self._transition("MISSING_ENDPOINT")
            if actual[0] != expected_type:
                self.revoked = "ENDPOINT_TYPE_MISMATCH"
                return self._transition(self.revoked)
        return self._transition("READY")

    def require_ready(self, providers, wire_version=2):
        if wire_version != 2:
            reject("LEGACY_ENDPOINT_DISABLED", "wire_version", 2, wire_version)
        reason = self.reason(providers)
        if reason != "READY":
            reject(reason, "readiness", "READY", reason)
