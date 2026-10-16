"""Summarize one unscored prior cycle; never certify an F3 campaign."""


def summarize_prior_cycle(trial):
    extras = trial.get("extras") or {}
    selector = extras.get("selector") or {}
    probe = selector.get("probe") or {}
    prior = extras.get("placement_prior") or {}
    integration = extras.get("integrate") or {}
    submitted = extras.get("submitted_legs") or []
    planned = trial.get("segments_planned")
    before = integration.get("expected_map_revision")
    after = integration.get("resulting_map_revision")
    admitted = probe.get("admitted_map_revision")
    validated = probe.get("validated_map_revision")
    checks = {
        "prior_adopted": prior.get("adopted") is True and bool(prior.get("candidate_id")),
        "measured_pick_authorized": (extras.get("authorized_pick") or {}).get("authorized") is True,
        "route_verified": (probe.get("mode") == "verify" and probe.get("verified") == 1
                           and probe.get("reason") == "OK" and probe.get("unavailable") == 0
                           and bool(probe.get("certification_id"))
                           and type(admitted) is int and admitted == validated
                           and selector.get("winner_method") == "route-graph"),
        "all_submitted_legs_succeeded": (type(planned) is int and planned > 0
            and trial.get("segments_succeeded") == planned and len(submitted) == planned
            and all(leg.get("ok") is True for leg in submitted)),
        "placement_returned_without_failure": trial.get("fail_code") == "" and trial.get("place_state") == "HOME",
        "payload_retained": trial.get("vac_attach") is True and trial.get("lost_payload") is False,
        "placement_inside_and_settled": (trial.get("inside_inner_box") is True
            and (extras.get("verify") or {}).get("settled") is True),
        "measured_map_confirmed": (extras.get("map_confirmation") or {}).get("confirmed") is True,
        "measured_height_confirmed": integration.get("occupancy_height_ok") is True,
        "map_revision_advanced": (integration.get("success") is True
            and type(before) is int and type(after) is int and after > before),
    }
    return {
        "scope": "single unscored prior motion and measured occupancy cycle",
        "scored": False,
        "functional_cycle_passed": all(checks.values()),
        "checks": checks,
        "f3_campaign": "not_evaluated",
        "performance": "not_evaluated",
        "limitations": ["No repeated-case reliability verdict",
                        "No independent zero-contact campaign verdict",
                        "No hardware plan-only acceptance verdict"],
        "candidate_id": prior.get("candidate_id"),
        "certificate_id": probe.get("certification_id"),
        "diagnostic_gt": {key: trial.get(key) for key in ("err_xy", "err_z", "roll", "pitch", "drift")},
        "map": {key: integration.get(key) for key in ("expected_map_revision", "resulting_map_revision",
                  "footprint_hits", "occupancy_height_m", "coverage")},
    }
