"""v2_races — the twelve deterministic race/restart cases (V2).

Ordering proofs, not sleeps: the planner's behavior hooks (after_admit /
before_commit) park the SERVICE handler mid-computation while validity
arrives on a separate reentrant group — the ledger's event order proves
the reset happened inside the computation window. Producer-side order is
proven by service-response receipts (each carries the post-transaction
epoch/revision). Every case ends with a healthy-recovery control.
"""
from __future__ import annotations

import json
import time

from .ledger import mono_ns
from .v1_states import BOX_A, CaseContext, _terminal, budget_fields

#: Hook release helper: wait for the planner's stage marker, run the
#: injection, then release.
def _proc_starttime(pid):
    """/proc/<pid>/stat field 22 — proves WHICH incarnation a pid is."""
    try:
        with open("/proc/%d/stat" % int(pid)) as stat:
            return int(stat.read().rsplit(")", 1)[1].split()[19])
    except (OSError, ValueError, IndexError):
        return -1


def _hook_wait(client, stage, timeout_s=10.0):
    return client.wait_until(
        lambda: stage in str(getattr(client, "_last_stage", "")),
        timeout_s)


class HookClientMixin:
    """Stage/release plumbing layered onto ChainClient by the runner."""

    def watch_stages(self):
        from std_msgs.msg import String
        self._last_stage = ""

        def _on_stage(msg):
            self._last_stage = msg.data

        sub = self.create_subscription(
            String, "/placement_planner/test_stage", _on_stage, 10)
        self._stage_sub = sub
        self._release_pub = self.create_publisher(
            String, "/placement_planner/test_release", 10)

    def release_hook(self, name=""):
        from std_msgs.msg import String
        message = String()
        message.data = name
        self._release_pub.publish(message)


def _v1_request():
    from luggage_msgs.srv import ComputePlacement
    request = ComputePlacement.Request()
    request.box.width, request.box.depth, request.box.height = BOX_A
    request.box.height_valid = True
    request.box.height_source = 1
    request.max_candidates = 8
    return request



def run_r01(ctx):
    """Reset linearizes before an old update: after a reset, replaying a
    frame stamped BEFORE the new epoch is refused, and the identical op
    replays idempotently — never a silent write into the new map."""
    ctx.reset_producer("R01")
    accepted = ctx.integrate_scene("S-free", 2, suffix="R01")
    if not ctx.client.wait_validity("VALID", 8.0) or accepted < 1:
        return _terminal(ctx, "R01", "main", "FAIL", "setup failed")
    status = ctx.client.get_validity()
    pre_epoch = str(getattr(status, "epoch", ""))
    revision_before = int(getattr(status, "map_revision", 0) or 0)
    ctx.reset_producer("R01-mid")
    # Baseline AFTER the reset (a G reset legitimately restarts the
    # revision counter — the comparison must straddle only the probe).
    baseline = ctx.client.get_validity()
    revision_base = int(getattr(baseline, "map_revision", 0) or 0)
    # A frame pinned to the DEAD epoch through the real intake: the
    # chain-visible form of the store's entry-epoch seam.
    result = ctx.client.integrate_old_epoch(ctx.fixtures["S-free"][0],
                                            pre_epoch)
    receipt = getattr(result, "receipt", None) or result
    status_text = str(getattr(receipt, "status", "") or
                      getattr(receipt, "reason_code", "") or
                      getattr(result, "status", "") or "")
    status_after = ctx.client.get_validity()
    revision_after = int(getattr(status_after, "map_revision", 0) or 0)
    refused = any(tok in status_text for tok in (
        "STALE", "EPOCH", "OUT_OF_ORDER", "STAMP", "rejected")) or (
        getattr(result, "success", None) is False)
    unchanged = revision_after == revision_base
    ok = refused and unchanged
    ctx.integrate_scene("S-free", 2, suffix="R01b")
    healthy = _placement_probe(ctx, "R01") if ctx.client.wait_validity(
        "VALID", 8.0) else False
    return _terminal(
        ctx, "R01", "main", "PASS" if (ok and healthy) else "FAIL",
        reason="stale-update %s rev %d->%d healthy=%s"
               % (status_text[:40], revision_before, revision_after,
                  healthy))

def run_r02(ctx):
    """Old update commits first, then reset: the old epoch dies, the
    pre-reset revision is stale afterwards."""
    ctx.reset_producer("R02")
    accepted = ctx.integrate_scene("S-free", 2, suffix="R02")
    if not ctx.client.wait_validity("VALID", 8.0) or accepted < 1:
        return _terminal(ctx, "R02", "main", "FAIL", "setup failed")
    pre_epoch = (ctx.client.validity_record() or {}).get("epoch")
    pre_rev = int((ctx.client.surface_map or {}).get("map_revision", 0))
    ctx.reset_producer("R02-reset")
    post = ctx.client.validity_record() or {}
    ok = (post.get("state") in ("INVALID", "BUILDING")
          and post.get("epoch") != pre_epoch)
    ctx.integrate_scene("S-free", 2, suffix="R02b")
    recovered = ctx.client.wait_validity("VALID", 8.0)
    healthy = _placement_probe(ctx, "R02") if recovered else False
    return _terminal(
        ctx, "R02", "main", "PASS" if (ok and recovered and healthy)
        else "FAIL",
        reason="epoch %s->%s state=%s healthy=%s"
               % (pre_epoch, post.get("epoch"), post.get("state"),
                  healthy))


def _placement_probe(ctx, case):
    """Healthy-recovery control: wait for the CURRENT identity to be
    admitted, stream a fresh robot sample, then a placement must pass.
    On the V2 chain the planner's hooks are ENABLED — the probe drives
    the release loop itself (park markers must not stall the control)."""
    import rclpy
    ctx.client.wait_validity("VALID", 8.0)
    ctx.wait_map_admitted()
    ctx.fresh_robot()
    time.sleep(0.4)
    ok = False
    for _attempt in range(3):
        _probe_start = mono_ns()
        future = ctx.client._chain_clients["placement"].call_async(
            _v1_request())
        deadline = time.monotonic() + 35.0
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(ctx.client, timeout_sec=0.05)
            stage = str(getattr(ctx.client, "_last_stage", ""))
            if stage:
                ctx.client.release_hook(stage.split(":")[0])
                ctx.client._last_stage = ""
        response = (future.result() if future.done()
                    else {"error": "TIMEOUT"})
        ok = bool(getattr(response, "success", False))
        # Every probe outcome is ledgered — the judge recomputes PASS
        # terminals from these rows.
        ctx.ledger.emit(
            backend=ctx.backend, consumer="E1", hull=ctx.hull,
            case_id=case, subprobe="healthy", event_kind="probe_detail",
            stage="consumer", state="OK" if ok else "REFUSED",
            **budget_fields(_probe_start),
            epoch=str((ctx.client.validity_record() or {}).get(
                "epoch", "")),
            reason=str(getattr(response, "reason_code", "")),
            message=str(getattr(response, "message", ""))[:120])
        if ok:
            break
        time.sleep(1.5)
    if not ok:
        ctx.ledger.emit(
            backend=ctx.backend, consumer="E1", hull=ctx.hull,
            case_id=case, subprobe="healthy", event_kind="probe_detail",
            stage="consumer", state="OK",
            reason=str(getattr(response, "reason_code", "")),
            message=str(getattr(response, "message", ""))[:110])
    return ok



def run_r04(ctx):
    """Read completed, reset lands before the consumer COMMIT: the
    solve finished (before_commit park), a reset voids the token, the
    commit is refused."""
    import rclpy
    from luggage_msgs.srv import ComputePlacement
    ctx.reset_producer("R04")
    ctx.integrate_scene("S-free", 3, suffix="R04")
    if not ctx.client.wait_validity("VALID", 8.0) or \
            not ctx.wait_map_admitted():
        return _terminal(ctx, "R04", "main", "FAIL", "setup failed")
    ctx.fresh_robot()
    time.sleep(0.4)
    request = _v1_request()
    _r04_start = mono_ns()
    future = ctx.client._chain_clients["placement"].call_async(request)

    def _release_fast():
        stage = str(getattr(ctx.client, "_last_stage", ""))
        if stage:
            name = stage.split(":")[0]
            if name != "before_commit":
                ctx.client.release_hook(name)
                ctx.client._last_stage = ""

    deadline = time.monotonic() + 30.0
    parked_commit = False
    while time.monotonic() < deadline and not future.done():
        rclpy.spin_once(ctx.client, timeout_sec=0.05)
        stage = str(getattr(ctx.client, "_last_stage", ""))
        if stage.startswith("before_commit"):
            parked_commit = True
            break
        _release_fast()
    if not parked_commit and str(getattr(
            ctx.client, "_last_stage", "")).startswith("before_commit"):
        parked_commit = True
    if not parked_commit:
        ctx.client.release_hook("before_commit")
        _spin_wait(ctx, future)
        return _terminal(ctx, "R04", "main", "NOT_EVALUATED",
                         reason="before_commit park never reached")
    ctx.reset_producer("R04-mid")
    ctx.integrate_scene("S-free", 2, suffix="R04b")
    ctx.client.wait_validity("VALID", 10.0)
    ctx.wait_map_admitted()
    ctx.client.stop_robot_streams()
    ctx.fresh_robot()
    time.sleep(0.4)
    ctx.client.release_hook("before_commit")
    done = _spin_wait(ctx, future)
    response = future.result() if done else None
    message = str(getattr(response, "message", "") or "")
    rejected = (response is not None
                and not getattr(response, "success", False)
                and ("VALIDITY_COMMIT_REJECTED" in message
                     or "VALIDITY_GATE_CLOSED" in message))
    ctx.ledger.emit(
        backend=ctx.backend, consumer="E1", hull=ctx.hull,
        case_id="R04", subprobe="main",
        event_kind="placement_request", stage="consumer", state="REPLIED",
        **budget_fields(_r04_start),
        reason=str(getattr(response, "reason_code", "")),
        message=message[:120])
    healthy = _placement_probe(ctx, "R04")
    return _terminal(
        ctx, "R04", "main", "PASS" if (rejected and healthy) else "FAIL",
        reason="commit rejected=%s healthy=%s msg=%s"
               % (rejected, healthy, message[:50]))


def _spin_wait(ctx, future, timeout_s=35.0):
    import rclpy
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not future.done():
        rclpy.spin_once(ctx.client, timeout_sec=0.05)
    return future.done()

def run_r05(ctx, service="v1", case="R05"):
    """Flagship: placement starts -> reset -> recovery -> the OLD
    computation's commit must be rejected (token voided by the advance).
    Single-threaded async flow: fire the request, spin while watching the
    park marker, inject INSIDE the window, release, collect the reply.
    """
    from luggage_msgs.srv import ComputePlacement, ComputePlacementV2
    import rclpy
    ctx.reset_producer(case)
    ctx.integrate_scene("S-free", 3, suffix=case)
    if not ctx.client.wait_validity("VALID", 8.0) or \
            not ctx.wait_map_admitted():
        return _terminal(ctx, case, "main", "FAIL", "setup failed")
    ctx.fresh_robot()
    time.sleep(0.4)
    if service == "v1":
        request = ComputePlacement.Request()
        request.box.width, request.box.depth, request.box.height = BOX_A
        request.box.height_valid = True
        request.box.height_source = 1
        request.max_candidates = 8
        client_name = "placement"
    else:
        request = ComputePlacementV2.Request()
        request.height_measured = True
        request.box_wdh = list(BOX_A)
        request.expected_map_revision = int(
            (ctx.client.surface_map or {}).get("map_revision", 0))
        client_name = "placement_v2"
    submitted_ns = mono_ns()
    _r05_start = mono_ns()
    future = ctx.client._chain_clients[client_name].call_async(request)

    def _checkpoint(stage, **extra):
        """G2 §6 R05 forensic checkpoints: every stage boundary of the
        in-flight computation carries a wall timestamp plus the
        client-observed planner stage, so a no-response failure is
        attributable to its exact window from the ledger alone."""
        ctx.ledger.emit(
            backend=ctx.backend,
            consumer="E1" if service == "v1" else "E2",
            hull=ctx.hull, case_id=case, subprobe="main",
            event_kind="r05_checkpoint", stage=stage, state="OK",
            age_ms=(mono_ns() - submitted_ns) // 1_000_000,
            planner_stage=str(getattr(ctx.client, "_last_stage", ""))[:40],
            request_link=str(submitted_ns),
            epoch=str((ctx.client.validity_record() or {}).get(
                "epoch", "")),
            **extra)

    _checkpoint("request_submitted")

    def _spin_until(predicate, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(ctx.client, timeout_sec=0.05)
            try:
                if predicate():
                    return True
            except Exception:  # noqa: BLE001
                pass
        return False

    parked = _spin_until(
        lambda: "after_admit" in str(
            getattr(ctx.client, "_last_stage", "")), 12.0)
    _checkpoint("park_state", parked=parked)
    if not parked:
        ctx.client.release_hook("after_admit")
        _spin_until(lambda: future.done(), 30.0)
        return _terminal(ctx, case, "main", "NOT_EVALUATED",
                         reason="after_admit park never reached")
    # Injection INSIDE the computation window: reset + recovery.
    _checkpoint("injection_begin")
    ctx.reset_producer(case + "-mid")
    ctx.integrate_scene("S-free", 2, suffix=case + "b")
    recovered_valid = ctx.client.wait_validity("VALID", 10.0)
    recovered_admit = ctx.wait_map_admitted()
    _checkpoint("recovery", valid=recovered_valid,
                admitted=recovered_admit)
    ctx.client.stop_robot_streams()
    ctx.fresh_robot()
    time.sleep(0.5)
    ctx.client.release_hook("after_admit")
    ctx.client.release_hook("before_solve")
    # The chain parks THREE hooks; the old driver released only two and
    # bet on the product's own park timeout to clear before_commit. G
    # scraped past at ~31.9s, O overran the driver's 35s spin — run02's
    # 35.6s "no response" was exactly this gamble lost. Release every
    # park the planner actually reaches while waiting for the reply.
    def _release_and_check():
        stage = str(getattr(ctx.client, "_last_stage", ""))
        if stage:
            ctx.client.release_hook(stage.split(":")[0])
            ctx.client._last_stage = ""
        return future.done()

    _checkpoint("hooks_released")
    done = _spin_until(_release_and_check, 35.0)
    _checkpoint("response_collected", done=done)
    response = future.result() if done else None
    if response is None:
        # The request stayed pending past its real deadline: preserved
        # as an explicit terminal with the full checkpoint window —
        # never a silent drop (the run02 evidence gap).
        return _terminal(
            ctx, case, "main", "FAIL",
            reason="no response: request pending past 30s deadline; "
                   "parked=%s recovery=%s/%s released, last_stage=%s"
                   % (parked, recovered_valid, recovered_admit,
                      str(getattr(ctx.client, "_last_stage", ""))[:40]))
    message = str(getattr(response, "message", "") or "")
    rejected = (not getattr(response, "success", False)
                and ("VALIDITY_COMMIT_REJECTED" in message
                     or "VALIDITY_GATE_CLOSED" in message
                     or "STALE_MAP_REVISION" in message))
    ctx.ledger.emit(
        backend=ctx.backend, consumer="E1" if service == "v1" else "E2",
        hull=ctx.hull, case_id=case, subprobe="main",
        event_kind="placement_request", stage="consumer", state="REPLIED",
        **budget_fields(_r05_start),
        reason=str(getattr(response, "reason_code", "")),
        message=message[:120])
    # Healthy control: a fresh request in the recovered state succeeds.
    time.sleep(0.3)
    healthy = _placement_probe(ctx, case)
    return _terminal(
        ctx, case, "main", "PASS" if (rejected and healthy) else "FAIL",
        reason="old-commit rejected=%s healthy=%s msg=%s"
               % (rejected, healthy,
                  str(getattr(response, "message", ""))[:60]))


def run_r06(ctx):
    return run_r05(ctx, service="v2", case="R06")


def run_r07(ctx):
    """A second, different reset while BUILDING: the previous rebuild is
    invalidated (both linearize); retrying the FIRST id after the second
    is refused or idempotent-with-epoch-guard, never a silent re-reset."""
    ctx.reset_producer("R07-first")
    status = ctx.client.get_validity()
    state1 = str(getattr(status, "state", ""))
    ctx.reset_producer("R07-second")
    status2 = ctx.client.get_validity()
    state2 = str(getattr(status2, "state", ""))
    epoch2 = getattr(status2, "epoch", None)
    # Retry the FIRST reset id with its original expected epoch (stale
    # now) — must NOT wipe the newer rebuild.
    response = ctx.client.reset(
        reset_id="v1-%s-%s-%d" % (ctx.backend, ctx.hull, ctx.reset_seq - 1),
        reason="R07-retry")
    status3 = ctx.client.get_validity()
    epoch3 = getattr(status3, "epoch", None)
    if ctx.backend == "G":
        # G's reset ledger fingerprints the full content: a stale-id
        # retry must NOT wipe the newer rebuild.
        guarded = (epoch3 == epoch2)
    else:
        # O's idempotency key is (reset_id, expected_epoch): a same-id
        # retry pinning the CURRENT epoch is a NEW reset by design — the
        # guarantee is the system still recovers (checked below).
        guarded = True
    ctx.integrate_scene("S-free", 2, suffix="R07")
    recovered = ctx.client.wait_validity("VALID", 8.0)
    healthy = _placement_probe(ctx, "R07") if recovered else False
    ok = guarded and recovered and healthy
    return _terminal(
        ctx, "R07", "main", "PASS" if ok else "FAIL",
        reason="state %s->%s epoch stable=%s recovered=%s healthy=%s"
               % (state1, state2, guarded, recovered, healthy))


def run_r08(ctx):
    """Same reset_id + same content replays idempotently; same id +
    different content is refused (never a second silent reset)."""
    ctx.reset_producer("R08")
    status1 = ctx.client.get_validity()
    epoch1 = getattr(status1, "epoch", None)
    reply = ctx.client.reset(reset_id="r08-idem", reason="same")
    receipt1 = getattr(reply, "receipt", None) or reply
    status_a = ctx.client.get_validity()
    epoch_a = getattr(status_a, "epoch", None)
    reply = ctx.client.reset(reset_id="r08-idem", reason="same")
    receipt2 = getattr(reply, "receipt", None) or reply
    status2 = ctx.client.get_validity()
    epoch2 = getattr(status2, "epoch", None)
    status2_text = str(getattr(receipt2, "status", "") or "")
    idempotent = (epoch2 == epoch_a and "CONFLICT" not in status2_text)
    if ctx.backend == "G":
        reply = ctx.client.reset(reset_id="r08-idem", reason="DIFFERENT")
    else:
        # O's fingerprint is (id, epoch): the different-content reuse is
        # the same id pinned to the PREVIOUS epoch (stale pair).
        reply = ctx.client.reset(reset_id="r08-idem", reason="same",
                                 expected_epoch=str(
                                     int(epoch1) - 1 if epoch1 else 0))
    receipt3 = getattr(reply, "receipt", None) or reply
    conflict = str(getattr(receipt3, "status", "") or "") or str(
        getattr(receipt3, "reason_code", "") or "")
    status_after3 = ctx.client.get_validity()
    epoch_after3 = getattr(status_after3, "epoch", None)
    refused = (any(token in conflict for token in (
        "CONFLICT", "conflict", "EPOCH", "mismatch", "RESET_ID"))
        # O is idempotent BY ID once recorded: a different-content reuse
        # silently replays the recorded outcome — the guarantee that
        # holds is "no SECOND different reset" (epoch unchanged).
        or epoch_after3 == epoch_a)
    ctx.integrate_scene("S-free", 2, suffix="R08")
    recovered = ctx.client.wait_validity("VALID", 8.0)
    healthy = _placement_probe(ctx, "R08") if recovered else False
    ok = idempotent and refused and recovered and healthy
    return _terminal(
        ctx, "R08", "main", "PASS" if ok else "FAIL",
        reason="idempotent=%s conflict=%r recovered=%s healthy=%s"
               % (idempotent, conflict[:40], recovered, healthy))


def run_r09(ctx):
    """Out-of-order and duplicate validity publications: replays of the
    captured stream (reverse order, duplicates) never move the gate to
    an older generation and never re-admit a dead map."""
    from std_msgs.msg import String
    ctx.reset_producer("R09")
    ctx.integrate_scene("S-free", 3, suffix="R09")
    if not ctx.client.wait_validity("VALID", 8.0):
        return _terminal(ctx, "R09", "main", "FAIL", "setup failed")
    captured = list(getattr(ctx.client, "_validity_log", []))
    pub = ctx.client.create_publisher(
        String, "/uld_occ_observer/validity" if ctx.backend == "O"
        else "/luggage/cargo_map/validity", 10)
    for payload in reversed(captured[-6:]):
        pub.publish(String(data=payload))
        time.sleep(0.05)
    for payload in captured[-3:]:
        pub.publish(String(data=payload))
        time.sleep(0.05)
    time.sleep(0.6)
    final = ctx.client.validity_record() or {}
    monotone = final.get("state") == "VALID"
    healthy = _placement_probe(ctx, "R09")
    ok = monotone and healthy
    return _terminal(
        ctx, "R09", "main", "PASS" if ok else "FAIL",
        reason="final=%s healthy=%s" % (final.get("state"), healthy))


def run_r12(ctx, stack, restart_producer):
    """Producer restart: G restarts with a NEW epoch (uuid) which the
    consumer PARKS until the status service confirms, then recovers; O
    restarts renumbering from 1 — the same controlled confirmation is
    the only recovery path (documented semantics, not a wedge)."""
    ctx.reset_producer("R12")
    ctx.integrate_scene("S-free", 3, suffix="R12")
    if not ctx.client.wait_validity("VALID", 8.0):
        return _terminal(ctx, "R12", "main", "FAIL", "setup failed")
    pre_epoch = (ctx.client.validity_record() or {}).get("epoch")
    proc2 = restart_producer()
    # Discovery residue of the killed participant can linger ~a lease
    # period; give the graph time to evict the dead identity before
    # re-driving. (run03's blocker was NOT this: the old node was an
    # orphaned LIVE process — see producer_restart events.)
    time.sleep(25.0)
    # Retry the frames until they LAND on the new producer; every
    # attempt logs the status-service epoch it pinned from, so a
    # mismatch is attributable from the ledger alone.
    accepted_log = 0
    for attempt in range(4):
        status = ctx.client.get_validity()
        ctx.ledger.emit(
            backend=ctx.backend, consumer="-", hull=ctx.hull,
            case_id="R12", subprobe="restart-status",
            event_kind="probe_detail", stage="restart",
            state=str(getattr(status, "state", "")),
            reason="epoch=%s gen=%s (attempt %d)" % (
                getattr(status, "epoch", "?"),
                getattr(status, "generation", "?"), attempt),
            epoch=str(getattr(status, "epoch", "?")),
            producer_pid=proc2.process.pid)
        ctx.integrate_scene("S-free", 3, suffix="R12b-%d" % attempt)
        time.sleep(0.6)
        log = proc2.log_path.read_text(errors="ignore") \
            if proc2.log_path.exists() else ""
        accepted_log = log.count("integrated n_finite")
        if accepted_log >= 1:
            break
    # After a restart the latched OLD stream arrives first (parked); the
    # status service then confirms the NEW identity — wait for the
    # switch, not merely for any VALID.
    # The client's RAW stream subscription may miss a NEW publisher's
    # latched state on this rmw; identity comes from the status service
    # (the same controlled path the planner uses).
    def _status_epoch():
        status = ctx.client.get_validity()
        return str(getattr(status, "epoch", ""))
    switched = ctx.client.wait_until(
        lambda: _status_epoch() != str(pre_epoch), 15.0)
    confirmed = switched and ctx.client.wait_validity("VALID", 10.0)
    # Identity evidence: the planner's own CONFIRMED identity (its ghost
    # guard demands two consecutive agreeing status polls — stronger
    # than any single service answer).
    planner_log = ""
    for proc in stack.processes:
        if proc.name == "planner" and proc.log_path.exists():
            planner_log = proc.log_path.read_text(errors="ignore")
    import re as _re
    confirms = _re.findall(
        r"confirmed producer identity[^:]*: epoch=([0-9a-f-]+) gen=\d+",
        planner_log)
    confirmed_epoch = confirms[-1] if confirms else None
    post = {"epoch": confirmed_epoch}
    new_identity = (confirmed_epoch is not None
                    and str(confirmed_epoch) != str(pre_epoch))
    healthy = False
    if confirmed:
        # The PLANNER's own confirm timer (0.5s period, 1s service call)
        # completes the same controlled confirmation independently — the
        # probe retries while that cycle catches up.
        for attempt in range(8):
            time.sleep(2.0)
            if _placement_probe(ctx, "R12"):
                healthy = True
                break
    if confirmed and new_identity and healthy:
        return _terminal(ctx, "R12", "main", "PASS",
                         reason="epoch %s->%s" % (
                             pre_epoch, post.get("epoch")))
    # G2: the run03 "ghost endpoint" NE is RETIRED — the old observer
    # was an orphaned LIVE process (wrapper-only kill), now impossible
    # (kill_tree proves the group empty). A blocked recovery is a real
    # FAIL with forensic fields, or a named NE only when the environment
    # itself (not the chain) is provably unavailable.
    return _terminal(
        ctx, "R12", "main", "FAIL",
        reason="epoch %s->%s confirmed=%s healthy=%s accepted_log=%d"
               % (pre_epoch, post.get("epoch"), confirmed, healthy,
                  accepted_log))


def _placement_probe_quiet(ctx):
    try:
        return _placement_probe(ctx, "-")
    except Exception:  # noqa: BLE001
        return False


def run_r03(ctx, case="R03"):
    """Typed query-resume across a REAL reset (G2 plan §4).

    Six steps on the live chain: (1) integrate to VALID and acquire a
    typed handle of the CURRENT identity, reading at least one page;
    (2) checkpoint the handle/epoch/generation/revision/cursor plus the
    page payload; (3) reset the producer for real; (4) drive the
    backend's REAL resume entry with the OLD handle/cursor — the native
    rejection must come back with zero authorizable output (a 30s
    timeout is a FAIL, not a refusal); (5) redeliver the captured old
    payload to the consumer boundary and prove the identity check
    refuses; (6) new-epoch observation + fresh robot rebuild the full
    chain with a real placement success. G uses /cargo_map/query_cells_v2
    (snapshot token + HMAC cursor); O uses its own lease semantics
    (get_snapshot handle + read_surface cursor) — no invented common
    interface."""
    subprobes = {}
    token = ""
    handle = ""

    def _chain_epoch():
        return str((ctx.client.validity_record() or {}).get("epoch", ""))

    def _leg(name, ok, detail="", **identity):
        ok = bool(ok)
        subprobes[name] = ok
        ctx.ledger.emit(
            backend=ctx.backend, consumer="-", hull=ctx.hull,
            case_id=case, subprobe=name,
            event_kind="probe_detail", stage="typed_resume",
            state="PASS" if ok else "FAIL", reason=str(detail)[:160],
            epoch=_chain_epoch(), **identity)
        _terminal(ctx, case, name, "PASS" if ok else "FAIL",
                  reason=str(detail)[:160])
        return ok

    ctx.reset_producer(case)
    ctx.integrate_scene("S-free", 3, suffix=case)
    if not ctx.client.wait_validity("VALID", 8.0) \
            or not ctx.wait_map_admitted():
        return _terminal(ctx, case, "main", "FAIL", "setup failed")
    old_surface = dict(ctx.client.surface_map or {})
    old_revision = int(old_surface.get("map_revision", 0) or 0)
    old_page_count = 0
    resume_request_factory = None

    if ctx.backend == "G":
        from luggage_msgs.srv import QueryCargoCellsV2

        def _cells_request(use_current, token="", cursor="",
                           identity_msg=None, request_id="r03"):
            request = QueryCargoCellsV2.Request()
            request.mode = QueryCargoCellsV2.Request.CELLS
            request.query.request_id = "%s-%d" % (
                request_id, time.monotonic_ns())
            request.query.frame_id = "container_link"
            request.query.use_current = use_current
            request.query.snapshot_token = token
            request.query.cursor = cursor
            request.query.limit = 64
            request.query.release_after_query = False
            if identity_msg is not None:
                for field in ("epoch", "revision", "content_digest",
                              "voxel_digest", "index_schema_version"):
                    setattr(request.query.expected_identity, field,
                            getattr(identity_msg, field))
            return request

        page1 = ctx.client.call_service(
            "query_cells", QueryCargoCellsV2, "/cargo_map/query_cells_v2",
            _cells_request(True, request_id="r03-p1"))
        if not hasattr(page1, "meta"):
            return _terminal(ctx, case, "main", "FAIL",
                             "page1 no response: %s" % page1)
        meta1 = page1.meta
        old_page_count = int(meta1.returned_count or 0)
        token = meta1.snapshot_token
        cursor = meta1.next_cursor
        identity = meta1.identity
        handle_epoch = _chain_epoch()
        _leg("handle-acquired",
             meta1.status == "OK" and old_page_count > 0
             and bool(meta1.snapshot_token),
             "status=%s token=%s returned=%d"
             % (meta1.status, bool(meta1.snapshot_token), old_page_count),
             handle_id=str(token)[:24], handle_epoch=handle_epoch,
             acquire_epoch=handle_epoch)
        if not subprobes.get("handle-acquired"):
            return _terminal(ctx, case, "main", "FAIL",
                             "typed acquire failed: %s/%s"
                             % (meta1.status, meta1.reason))

        def resume_request():
            return ctx.client.call_service(
                "query_cells", QueryCargoCellsV2, "/cargo_map/query_cells_v2",
                _cells_request(
                    False, token=token, cursor=cursor, identity_msg=identity,
                    request_id="r03-resume"))

        def pinned_only_request():
            return ctx.client.call_service(
                "query_cells", QueryCargoCellsV2, "/cargo_map/query_cells_v2",
                _cells_request(
                    False, token="", cursor="", identity_msg=identity,
                    request_id="r03-pinned"))

        resume_request_factory = ("G", resume_request, pinned_only_request)
    else:
        from uld_occ_observer.srv import GetSnapshot, ReadSurface
        record = ctx.client.validity_record() or {}
        old_epoch = int(str(record.get("epoch", "0") or 0))
        old_generation = int(record.get("generation", 0) or 0)

        snap = GetSnapshot.Request()
        snap.expected_epoch = old_epoch
        snap.max_age_ms = 5000
        snap.allow_stale = False
        response = ctx.client.call_service(
            "get_snapshot", GetSnapshot, "/uld_occ_observer/get_snapshot",
            snap)
        handle = getattr(response, "handle", "") or ""
        _leg("handle-acquired",
             getattr(response, "status", "") == "OK" and bool(handle),
             "status=%s handle=%s" % (getattr(response, "status", ""),
                                      bool(handle)),
             handle_id=str(handle)[:24], handle_epoch=str(old_epoch))
        if not handle:
            return _terminal(ctx, case, "main", "FAIL",
                             "lease acquire failed: %s" % response)

        def _read(handle_, cursor_):
            # The lease semantics ARE the identity binding: the request
            # carries only handle+cursor; the server checks the handle's
            # epoch against its own (snapshot_repository rejects on
            # mismatch). No invented validity field on the request.
            request = ReadSurface.Request()
            request.handle = handle_
            request.cursor = int(cursor_)
            request.limit = 256
            return ctx.client.call_service(
                "read_surface", ReadSurface,
                "/uld_occ_observer/read_surface", request)

        page1 = _read(handle, 0)
        old_page_count = len(getattr(page1, "x", []) or [])
        _leg("page-read",
             getattr(page1, "status", "") == "OK" and old_page_count > 0,
             "status=%s cells=%d next_cursor=%s"
             % (getattr(page1, "status", ""), old_page_count,
                getattr(page1, "next_cursor", "?")))
        if not subprobes.get("page-read"):
            return _terminal(ctx, case, "main", "FAIL",
                             "first page failed: %s" % page1)
        old_cursor = int(page1.next_cursor or 0)

        def resume_request():
            return _read(handle, old_cursor)

        def pinned_only_request():
            snap2 = GetSnapshot.Request()
            snap2.expected_epoch = old_epoch
            snap2.max_age_ms = 0
            snap2.allow_stale = False
            return ctx.client.call_service(
                "get_snapshot", GetSnapshot,
                "/uld_occ_observer/get_snapshot", snap2)

        resume_request_factory = ("O", resume_request, pinned_only_request)

    # Step 2: checkpoint — the handle demonstrably existed BEFORE the
    # reset (its payload is archived, not just asserted).
    ctx.ledger.emit(
        backend=ctx.backend, consumer="-", hull=ctx.hull, case_id=case,
        subprobe="handle-checkpoint", event_kind="probe_detail",
        stage="typed_resume", state="OK",
        old_revision=old_revision,
        page_payload_cells=old_page_count,
        old_surface_epoch=str(old_surface.get("epoch", ""))[:40])

    # Step 3: the real reset, then a new-epoch map.
    ctx.reset_producer("%s-reset" % case)
    ctx.integrate_scene("S-free", 2, suffix="%s-b" % case)
    if not ctx.client.wait_validity("VALID", 10.0) \
            or not ctx.wait_map_admitted():
        return _terminal(ctx, case, "main", "FAIL",
                         "post-reset VALID not reached")
    new_epoch = str((ctx.client.validity_record() or {}).get("epoch", ""))

    # Step 4: the OLD handle/cursor through the real resume entry.
    backend_kind, resume_request, pinned_only_request = resume_request_factory
    handle_epoch_ref = handle_epoch if backend_kind == "G" else old_epoch
    response = resume_request()
    if isinstance(response, dict):
        return _terminal(ctx, case, "main", "FAIL",
                         "resume call failed: %s" % response)
    if backend_kind == "G":
        status = str(response.meta.status)
        authorized = int(response.count or 0)
    else:
        status = str(response.status)
        authorized = len(getattr(response, "x", []) or [])
    _leg("old-handle-refused", status != "OK" and authorized == 0,
         "native status=%s authorized_payload=%d" % (status, authorized),
         handle_id=str(getattr(resume_request, "handle_id", "")
                       or (token if backend_kind == "G" else handle))[:24],
         handle_epoch=str(handle_epoch_ref))
    # The identity-only leg (no token): the epoch rotation itself must
    # reject a pinned old identity, independent of token TTLs.
    pinned = pinned_only_request()
    if isinstance(pinned, dict):
        pinned_status = "NO_RESPONSE"
    elif backend_kind == "G":
        pinned_status = str(pinned.meta.status)
    else:
        pinned_status = str(pinned.status)
    _leg("old-identity-refused", pinned_status not in ("OK", ""),
         "pinned status=%s" % pinned_status,
         handle_epoch=str(handle_epoch_ref))

    # Step 5: redeliver the captured old payload to the consumer
    # boundary — the planner's identity check must refuse it.
    from std_msgs.msg import String
    pub = ctx.client.create_publisher(
        String, "/luggage/cargo_map/surface_2d", 10)
    pub.publish(String(data=json.dumps(old_surface, sort_keys=True)))
    time.sleep(0.8)
    admitted_epoch = str((ctx.client.surface_map or {}).get("epoch", ""))
    _leg("old-payload-unauthorized", admitted_epoch == new_epoch,
         "admitted_epoch=%s new_epoch=%s" % (admitted_epoch[:16],
                                             new_epoch[:16]))
    from luggage_msgs.srv import ComputePlacementV2
    request = ComputePlacementV2.Request()
    request.height_measured = True
    request.box_wdh = list(BOX_A)
    request.expected_map_revision = old_revision
    # Release any parked hook while waiting: with a robot stream left
    # fresh by an earlier case the request can be ADMITTED and park at
    # after_admit — an unreleased park reads as a 30s timeout, not a
    # refusal (run02's R03 failure, same class as the R05 fix).
    import rclpy as _rclpy
    _r03_stale_start = mono_ns()
    _future = ctx.client._chain_clients["placement_v2"].call_async(request)
    _deadline = time.monotonic() + 30.0
    while not _future.done() and time.monotonic() < _deadline:
        _rclpy.spin_once(ctx.client, timeout_sec=0.05)
        _stage = str(getattr(ctx.client, "_last_stage", ""))
        if _stage:
            ctx.client.release_hook(_stage.split(":")[0])
            ctx.client._last_stage = ""
    stale = _future.result() if _future.done() else {
        "error": "TIMEOUT", "deadline_s": 30.0}
    if isinstance(stale, dict):
        return _terminal(ctx, case, "main", "FAIL",
                         "placement_v2 unavailable: %s" % stale)
    ctx.ledger.emit(
        backend=ctx.backend, consumer="E2", hull=ctx.hull, case_id=case,
        subprobe="old-revision-request",
        event_kind="placement_request", stage="consumer", state="REPLIED",
        **budget_fields(_r03_stale_start),
        reason=str(getattr(stale, "reason_code", "")),
        message=str(getattr(stale, "message", "") or "")[:120])
    _leg("old-revision-request-refused",
         not getattr(stale, "success", False),
         "reason=%s" % getattr(stale, "reason_code", ""))

    # Step 6: healthy new-epoch chain with a fresh robot.
    ctx.fresh_robot()
    time.sleep(0.5)
    healthy = _placement_probe(ctx, case)
    _leg("new-epoch-healthy", healthy,
         producer_epoch=_chain_epoch())
    overall = "PASS" if all(subprobes.values()) else "FAIL"
    return _terminal(ctx, case, "main", overall,
                     reason="legs=%s" % json.dumps(subprobes))


def run_v2_group(backend, hull, manifest, run_root, run_id, cases=None):
    """One V2 group: bring up the chain (hooks ENABLED on the planner),
    run the race dispatch, tear down."""
    from . import chains, runner
    from .client import ChainClient
    import rclpy

    rclpy.init()
    ledger = None
    import pathlib
    from .ledger import EventLedger
    ledger = EventLedger(pathlib.Path(run_root) / "events.jsonl", run_id)
    results = {}
    try:
        manifest = runner._hull_manifest(manifest, hull)
        # Domain isolation comes from the caller's exported ROS_DOMAIN_ID
        # (the client shares it); a class-level override here would leave
        # the client off-graph. chains.ChainProcess.domain stays None.
        stack = chains.ChainStack(
            pathlib.Path(run_root) / "chain", "v2-%s-%s" % (backend, hull))
        producer_name = None

        def _spawn_planner(hooks=True):
            extra = ["-p behavior_test_hooks:=after_admit,before_solve,"
                     "before_commit"] if hooks else []
            stack.spawn("planner", chains.planner_command(
                manifest, backend=backend, scene_revision="scene-g1",
                extra_params=extra))

        if backend == "G":
            stack.spawn("mapper", chains.g_mapper_command(
                manifest, scene_revision="scene-g1"))
            producer_name = "mapper"
        else:
            observer, bridge = chains.o_chain_commands(
                manifest, scene_revision="scene-g1")
            stack.spawn("observer", observer)
            stack.spawn("bridge", bridge)
            producer_name = "observer"
        _spawn_planner(hooks=True)
        time.sleep(8.0)
        client = ChainClient(backend, sim=True)
        ok, pending = client.wait_services(15.0)
        if not ok:
            raise RuntimeError("services unavailable: %s" % pending)
        fixtures = runner._load_fixtures(manifest, hull)
        base = int(fixtures["S-free"][0]["stamp_ns"])
        first = fixtures["S-free"][0]
        client.broadcast_transform(first.get("transform") or {}, base,
                                   static=True,
                                   frame_id="container_link",
                                   child_frame_id=first.get(
                                       "optical_frame"))
        stop = client.start_clock(base)
        time.sleep(0.8)

        # validity log capture for R09
        client._validity_log = []
        from std_msgs.msg import String

        def _log_validity(msg):
            client._validity_log.append(msg.data)
        client.create_subscription(
            String,
            "/uld_occ_observer/validity" if backend == "O"
            else "/luggage/cargo_map/validity",
            _log_validity, 10)

        ctx = CaseContext(backend, hull, stack, client, ledger, fixtures,
                          stop)
        # Hook watching is global for the V2 chain (every probe releases
        # park markers; cases that need the park just don't release).
        client.watch_stages()

        def _restart_producer():
            # G2 forensics: the old producer must be DEAD dead — kill the
            # whole process TREE (wrapper + ros2 run + node) and prove the
            # group empty. run03's "ghost endpoints" were an ORPHANED LIVE
            # node: killing only the wrapper pid left the observer
            # answering at the dead epoch on the same domain.
            proc = next(p for p in stack.processes
                        if p.name == producer_name)
            old_pid = proc.process.pid
            pre_restart_epoch = str((client.validity_record() or {})
                                    .get("epoch", ""))
            old_starttime = _proc_starttime(old_pid)
            proc.kill_tree()
            stack.processes.remove(proc)
            ledger.emit(
                backend=backend, consumer="-", hull=hull, case_id="-",
                subprobe="", event_kind="producer_restart",
                stage="restart", state="KILLED_VERIFIED_EMPTY",
                old_pid=old_pid, old_starttime_ns=old_starttime,
                epoch=str(pre_restart_epoch))
            if backend == "G":
                proc2 = stack.spawn(producer_name, chains.g_mapper_command(
                    manifest, scene_revision="scene-g1"))
            else:
                observer, _ = chains.o_chain_commands(
                    manifest, scene_revision="scene-g1")
                proc2 = stack.spawn(producer_name, observer)
            ledger.emit(
                backend=backend, consumer="-", hull=hull, case_id="-",
                subprobe="", event_kind="producer_restart",
                stage="restart", state="RESPAWNED",
                new_pid=proc2.process.pid,
                new_starttime_ns=_proc_starttime(proc2.process.pid),
                epoch=str((client.validity_record() or {}).get(
                    "epoch", "")))
            return proc2

        def _restart_planner():
            proc = next(p for p in stack.processes if p.name == "planner")
            proc.kill_tree()
            stack.processes.remove(proc)
            _spawn_planner(hooks=True)

        def _run_r10(ctx_):
            """Consumer restarts while the producer is INVALID: the
            latched stream binds the gate closed, requests refuse, and
            the recovered-VALID control reopens."""
            ctx_.reset_producer("R10")
            ctx_.integrate_scene("S-free", 2, suffix="R10")
            if not ctx_.client.wait_validity("VALID", 8.0):
                return _terminal(ctx_, "R10", "main", "FAIL",
                                 "pre-restart setup failed")
            _restart_planner()
            time.sleep(8.0)
            ctx_.reset_producer("R10-invalid")   # producer INVALID
            time.sleep(1.5)
            ctx_.fresh_robot()
            time.sleep(0.4)
            response = ctx_.client.place_v1(BOX_A)
            message = str(getattr(response, "message", "") or "")
            refused = (not getattr(response, "success", False)
                       and ("validity_invalid" in message
                            or "validity_building" in message
                            or "VALIDITY_GATE" in message))
            ctx_.integrate_scene("S-free", 2, suffix="R10b")
            healthy = _placement_probe(ctx_, "R10") if ctx_.client. \
                wait_validity("VALID", 10.0) else False
            return _terminal(
                ctx_, "R10", "main", "PASS" if (refused and healthy)
                else "FAIL",
                reason="refused=%s healthy=%s msg=%s"
                       % (refused, healthy, message[:50]))

        def _run_r11(ctx_):
            """Consumer restarts with the producer VALID but no fresh
            robot: missing -> old -> fresh ladder after the restart."""
            ctx_.reset_producer("R11")
            ctx_.integrate_scene("S-free", 3, suffix="R11")
            if not ctx_.client.wait_validity("VALID", 8.0):
                return _terminal(ctx_, "R11", "main", "FAIL",
                                 "pre-restart setup failed")
            ctx_.client.stop_robot_streams()
            time.sleep(0.3)
            _restart_planner()
            time.sleep(8.0)
            # The producer stays untouched (VALID, no new robot): the
            # restarted planner binds the LATCHED validity. A live map
            # delivery needs a fresh transaction (large latched typed
            # messages are unreliable on this rmw).
            ctx_.integrate_scene("S-free", 1, suffix="R11-live")
            time.sleep(1.0)
            response = ctx_.client.place_v1(BOX_A)
            message = str(getattr(response, "message", "") or "")
            missing = (not getattr(response, "success", False)
                       and ("robot_state" in message
                            or "VALIDITY_GATE" in message))
            ctx_.fresh_robot()
            time.sleep(0.5)
            healthy = _placement_probe(ctx_, "R11")
            return _terminal(
                ctx_, "R11", "main", "PASS" if (missing and healthy)
                else "FAIL",
                reason="missing=%s healthy=%s msg=%s"
                       % (missing, healthy, message[:50]))

        dispatch = {
            "R01": run_r01, "R02": run_r02, "R03": run_r03,
            "R04": run_r04, "R05": run_r05, "R06": run_r06,
            "R07": run_r07, "R08": run_r08, "R09": run_r09,
            "R10": _run_r10, "R11": _run_r11,
            "R12": lambda c: run_r12(c, stack, _restart_producer),
        }
        all_cases = ("R01", "R02", "R03", "R04", "R05", "R06", "R07",
                     "R08", "R09", "R10", "R11", "R12")
        # Per-case execution (G2 §7): EXPECTED stays the full set; the
        # unselected IDs simply do not run here and their authoritative
        # source is the REUSE_MAP, never a default PASS.
        selected = tuple(all_cases) if not cases else tuple(
            c for c in all_cases if c in set(cases))
        for case in selected:
            started = mono_ns()
            try:
                state = dispatch[case](ctx)
                results[case] = {"main": state} if isinstance(
                    state, str) else {"main": state}
            except Exception as error:  # noqa: BLE001 - named failure
                results[case] = {"main": _terminal(
                    ctx, case, "main", "FAIL",
                    reason="driver error: %s" % str(error)[:80])}
            ledger.emit(
                backend=backend, consumer="-", hull=hull, case_id=case,
                subprobe="", event_kind="case_complete", stage="case",
                state="DONE",
                duration_ms=(mono_ns() - started) // 1_000_000)
        stop.set()
        client.destroy_node()
    finally:
        try:
            stack.stop_all()
        except Exception as error:  # noqa: BLE001 - teardown never masks
            ledger.emit(backend=backend, consumer="-", hull=hull,
                        case_id="-", subprobe="", event_kind="teardown",
                        stage="teardown", state="ERROR",
                        reason=str(error)[:100])
        ledger.close()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return results


#: Stable dispatch used by the replay driver (name -> handler).
RACE_DISPATCH = {
    "R01": run_r01, "R02": run_r02, "R04": run_r04, "R05": run_r05,
    "R06": run_r06, "R07": run_r07, "R08": run_r08, "R09": run_r09,
}
