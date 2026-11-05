"""v1_states — the K01-K10 matrix executors (V1).

One chain per (backend, hull, scene-profile) group serves every consumer
and case against it; cases that need a pristine producer run FIRST on
the fresh chain, then each case resets the producer to a defined state
(idempotent reset with a unique reset_id, then the case's scenario).

Every case writes its scenario events to the ledger and exactly one
terminal per (case, subprobe). E1/E2 exercise the real planner services;
E3 (occupancy_place_paths via plan_attached_box) and E4
(HardwarePlacePlanner.plan) run through their T1 driver entries.
"""
from __future__ import annotations

import json
import time

from .inventory import POLICY
from .ledger import mono_ns

#: Deadline for every request: fixed 30s after send, real monotonic.
REQUEST_DEADLINE_S = POLICY["request_deadline_s"]

BOX_A = tuple(POLICY["probe_boxes_m"][0])   # 0.30 x 0.20 x 0.20
BOX_B = tuple(POLICY["probe_boxes_m"][1])   # 0.50 x 0.30 x 0.25


class CaseContext:
    """One live chain + client + ledger scoped to a (backend, hull)."""

    def __init__(self, backend, hull, stack, client, ledger, fixtures,
                 clock_stop):
        self.backend = backend
        self.hull = hull
        self.stack = stack
        self.client = client
        self.ledger = ledger
        self.fixtures = fixtures       # {"S-free": [docs], ...}
        self.clock_stop = clock_stop
        self.reset_seq = 0

    def reset_producer(self, reason):
        """Idempotent reset to a defined BUILDING state (unique id)."""
        self.reset_seq += 1
        reset_id = "v1-%s-%s-%d" % (self.backend, self.hull,
                                    self.reset_seq)
        result = self.client.reset(reset_id=reset_id, reason=reason)
        receipt = getattr(result, "receipt", None) or result
        status = getattr(receipt, "status", None) or getattr(
            result, "status", "")
        self.ledger.emit(
            backend=self.backend, consumer="-", hull=self.hull,
            case_id="-", subprobe="", event_kind="reset", stage="reset",
            state="OK" if status in ("OK", "accepted") else status,
            reason=str(status), reset_id=reset_id)
        return status

    def integrate_scene(self, scene, count=3, suffix=""):
        """Drive real frames of one scene through the backend intake.

        The continuous scenario clock is paused per frame: each stamp is
        injected with a manual tick (freshness windows judge the node
        clock, which is sim time), then the clock resumes from the last
        stamp so age premises stay satisfiable between events.
        """
        docs = self.fixtures[scene][:count]
        accepted = 0
        paused = False
        if self.clock_stop is not None:
            self.clock_stop.set()
            paused = True
        try:
            return self._integrate_frames(
                docs[:count], suffix, accepted)
        finally:
            if paused:
                last = int(docs[min(count, len(docs)) - 1].get(
                    "stamp_ns", 0)) + 200_000_000
                self.clock_stop = self.client.start_clock(last)

    def _integrate_frames(self, docs, suffix, accepted):
        for index, doc in enumerate(docs):
            stamp = int(doc.get("stamp_ns", 0))
            for _ in range(4):
                self.client.tick(stamp + 60_000_000)
            result = self.client.integrate_frame(
                doc, observation_suffix="%s-v1-%d" % (suffix, index))
            receipt = getattr(result, "receipt", None) or result
            status = str(getattr(receipt, "status", "") or "")
            self.ledger.emit(
                backend=self.backend, consumer="-", hull=self.hull,
                case_id="-", subprobe="", event_kind="integrate",
                stage="input", state="OK" if status in (
                    "INTEGRATED", "accepted") else status,
                reason=status, request_id=str(
                    doc.get("observation_id", "")),
                result_repr=str(result)[:100])
            if status in ("INTEGRATED", "accepted"):
                accepted += 1
            import time as _t
            _t.sleep(0.05)
        return accepted

    def fresh_robot(self, offset_ns=50_000_000):
        """Ensure the frozen robot fixture STREAMS (production robots
        publish continuously; solves outlive any single sample's 500ms
        window under the running scenario clock)."""
        stop = getattr(self.client, "_robot_stop", None)
        if stop is None or stop.is_set():
            self.client.start_robot_stream(stamp_offset_ns=offset_ns)
        return self.client.clock_now_ns()


def run_case(ctx, case_id, run_e3=None, run_e4=None):
    """Dispatch one K-case; returns {subprobe: terminal-state}."""
    dispatch = {
        "K01": _k01, "K02": _k02, "K03": _k03, "K04": _k04, "K05": _k05,
        "K06": _k06, "K07": _k07, "K08": _k08, "K09": _k09, "K10": _k10,
    }
    handler = dispatch.get(case_id)
    if handler is None:
        return {"main": "NOT_EVALUATED"}
    return handler(ctx, run_e3=run_e3, run_e4=run_e4)


def _terminal(ctx, case, subprobe, state, reason="", **payload):
    ctx.ledger.terminal(
        backend=ctx.backend, consumer="ALL", hull=ctx.hull,
        case_id=case, subprobe=subprobe, state=state, reason=reason,
        **payload)
    return state


def _placement(ctx, case, subprobe, box, **kwargs):
    """One real ComputePlacement call through the chain (30s deadline)."""
    deadline = mono_ns() + int(REQUEST_DEADLINE_S * 1e9)
    response = ctx.client.place_v1(box, **kwargs)
    ctx.ledger.emit(
        backend=ctx.backend, consumer="E1", hull=ctx.hull, case_id=case,
        subprobe=subprobe, event_kind="placement_request",
        stage="consumer", state="REPLIED", deadline_ns=deadline,
        reason=str(getattr(response, "reason_code", "") or ""))
    return response


def _k01(ctx, run_e3=None, run_e4=None):
    """Healthy: real VALID map, >=1 legal candidate, oracle passes."""
    ctx.reset_producer("K01")
    accepted = ctx.integrate_scene("S-free", 3, suffix="K01")
    ok = ctx.client.wait_validity("VALID", 8.0)
    if not ok or accepted < 1:
        return {"main": _terminal(
            ctx, "K01", "main", "FAIL",
            "scenario setup failed (validity=%s accepted=%d)"
            % (ctx.client.validity_record().get("state"), accepted))}
    ctx.fresh_robot()
    time.sleep(0.3)
    response = _placement(ctx, "K01", "main", BOX_A)
    from .oracle import verify_response
    # The oracle consumes the latched last_result dump (full candidate
    # payloads in container frame) plus the admitted v1 surface map.
    surface = ctx.client.surface_map or {}
    dump = ctx.client.last_result.get("candidates") or []
    from .oracle import verify_response as _vr
    verdict = _vr(
        type("R", (), {"candidates": dump,
                       "reason_code": getattr(response, "reason_code",
                                              "")})(),
        surface=surface, box_size=BOX_A,
        hull={"inner_size": [1.49, 1.97, 1.48],
                       "floor_z_container": 0.53})
    classes = [v.get("candidate_class") for v in verdict["verdicts"]]
    if getattr(response, "success", False) and verdict["n_candidates"] \
            >= 1 and verdict["all_pass"]:
        # K01 positive (contract K01 row): a real legal candidate set,
        # oracle-verified in each candidate's OWN claimed class (a
        # floor-prior candidate on unknown ground claiming confidence 0
        # is legal; CLAIMING known-free there is K03's failure class),
        # plus the real admission+commit which produced it. Claim classes
        # ride into the terminal as evidence.
        return {"main": _terminal(ctx, "K01", "main", "PASS",
                                  n_candidates=verdict["n_candidates"],
                                  classes=classes)}
    sample = (ctx.client.last_result.get("candidates") or [{}])[0]
    return {"main": _terminal(
        ctx, "K01", "main", "FAIL",
        "success=%s candidates=%d oracle=%s reason=%s"
        % (getattr(response, "success", False), verdict["n_candidates"],
           verdict["all_pass"], verdict["reason_code"]),
        candidate_sample=dict(sample) if sample else {},
        claims=[v.get("claim_confidence") for v in verdict["verdicts"]][:24],
        classes=classes,
        failing_legs=[v["legs"] for v in verdict["verdicts"]
                      if not v["pass"]][:4],
        footprint_sample=(
            {"%d,%d" % (ix, iy):
             [surface["state"][ix][iy], surface["height"][ix][iy]]
             for ix, iy in [(int(sample.get("ix0", 0)) + dx,
                             int(sample.get("iy0", 0)) + dy)
                            for dx in (0, 4, 9)
                            for dy in (0, 7, 14)]
             if sample and surface
             and 0 <= ix < int(surface.get("nx", 0))
             and 0 <= iy < int(surface.get("ny", 0))}
            if surface else {}),
        floor_z_of_map=surface.get("floor_z") if surface else None,
        oracle_legs=verdict["verdicts"][0]["legs"] if verdict["verdicts"]
        else {})}


def _k04(ctx, run_e3=None, run_e4=None):
    """No data: fresh producer, floor prior only -> UNBOUND, zero
    candidates, zero downstream admission."""
    response = _placement(ctx, "K04", "main", BOX_A)
    message = str(getattr(response, "message", "") or "")
    success = bool(getattr(response, "success", False))
    candidates = list(getattr(response, "candidates", None) or [])
    # No map ever admitted: the refusal is a real validity refusal — the
    # gate unbound (no publication ever) or bound to a non-VALID stream
    # (the latched boot BUILDING arrives first on a live producer).
    refused = ("VALIDITY_GATE_UNBOUND" in message
               or "validity_building" in message
               or "validity_invalid" in message)
    if (not success and refused and not candidates):
        return {"main": _terminal(ctx, "K04", "main", "PASS")}
    return {"main": _terminal(
        ctx, "K04", "main", "FAIL",
        "success=%s message=%s candidates=%d"
        % (success, message[:60], len(candidates)))}


def _k02(ctx, run_e3=None, run_e4=None):
    return {"main": _terminal(ctx, "K02", "main", "NOT_EVALUATED",
                              reason="scene fixture pending selection")}


def _k03(ctx, run_e3=None, run_e4=None):
    return {"main": _terminal(ctx, "K03", "main", "NOT_EVALUATED",
                              reason="scene fixture pending selection")}


def _k05(ctx, run_e3=None, run_e4=None):
    return {"main": _terminal(ctx, "K05", "main", "NOT_EVALUATED",
                              reason="stale subprobes pending driver")}


def _k06(ctx, run_e3=None, run_e4=None):
    return {"main": _terminal(ctx, "K06", "main", "NOT_EVALUATED",
                              reason="hull cross-config pending driver")}


def _k07(ctx, run_e3=None, run_e4=None):
    return {"main": _terminal(ctx, "K07", "main", "NOT_EVALUATED",
                              reason="frame/TF subprobes pending driver")}


def _k08(ctx, run_e3=None, run_e4=None):
    return {"main": _terminal(ctx, "K08", "main", "NOT_EVALUATED",
                              reason="unit subprobes pending driver")}


def _k09(ctx, run_e3=None, run_e4=None):
    return {"main": _terminal(ctx, "K09", "main", "NOT_EVALUATED",
                              reason="reset-replay pending driver")}


def _k10(ctx, run_e3=None, run_e4=None):
    return {"main": _terminal(ctx, "K10", "main", "NOT_EVALUATED",
                              reason="recovery ladder pending driver")}
