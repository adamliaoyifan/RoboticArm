"""Preparation facts for recovery decisions; no inferred observation deficit.

A false support cell can mean missing depth, off-face area, masking or a real
surface defect. Its cause is not present in the binary domain. Neither contact
exhaustion nor unsuccessful native search proves that another view will help.
This receipt reports reached boundaries, not a recovery or motion permission.
"""
from collections import Counter


class PreparationSearchEvidence:
    """One request's bounded counters, alongside the original detailed trace."""

    def __init__(self, domain):
        self.domain_sha256 = domain.digest()
        self.supported_cells = sum(domain.support)
        self.total_cells = len(domain.support)
        self.furthest_stage = 'NOT_STARTED'
        self.proposals_checked = 0
        self.proposals_accepted = 0
        self.centres_checked = 0
        self.quality_batches_complete = 0
        self.quality_batches_incomplete = 0
        self.quality_contacts_accepted = 0
        self.quality_contacts_refused = 0
        self.native_candidates_yielded = 0
        self.proposal_reasons = Counter()
        self.quality_batch_reasons = Counter()
        self.quality_contact_reasons = Counter()

    def proposal(self, proposal):
        if self.furthest_stage == 'NOT_STARTED':
            self.furthest_stage = 'CONTACT_PROPOSAL'
        self.proposals_checked += 1
        self.centres_checked += proposal.checked_centres
        self.proposals_accepted += proposal.reason == 'OBSERVED_FOOTPRINT_PROPOSAL'
        self.proposal_reasons[proposal.reason] += 1

    def quality(self, evidence):
        if self.furthest_stage != 'NATIVE_CANDIDATE':
            self.furthest_stage = 'DIRECTED_QUALITY'
        self.quality_batch_reasons[evidence.reason] += 1
        if evidence.complete:
            self.quality_batches_complete += 1
            self.quality_contacts_accepted += len(evidence.contacts)
            for _, accepted, reason, _ in evidence.verdicts:
                self.quality_contacts_refused += not accepted
                self.quality_contact_reasons[reason] += 1
        else:
            # An incomplete/invalid receipt cannot establish per-contact facts.
            self.quality_batches_incomplete += 1

    def candidate_yielded(self):
        self.furthest_stage = 'NATIVE_CANDIDATE'
        self.native_candidates_yielded += 1

    def record(self, terminal_reason):
        return dict(schema='preparation_search_evidence_v1',
            domain_sha256=self.domain_sha256,
            supported_cells=self.supported_cells,
            unproven_cells=self.total_cells-self.supported_cells,
            furthest_stage=self.furthest_stage,
            proposals_checked=self.proposals_checked,
            proposals_accepted=self.proposals_accepted,
            centres_checked=self.centres_checked,
            quality_batches_complete=self.quality_batches_complete,
            quality_batches_incomplete=self.quality_batches_incomplete,
            quality_contacts_accepted=self.quality_contacts_accepted,
            quality_contacts_refused=self.quality_contacts_refused,
            native_candidates_yielded=self.native_candidates_yielded,
            proposal_reasons=dict(sorted(self.proposal_reasons.items())),
            quality_batch_reasons=dict(sorted(self.quality_batch_reasons.items())),
            quality_contact_reasons=dict(sorted(self.quality_contact_reasons.items())),
            terminal_reason=terminal_reason,
            observation_deficiency='NOT_ESTABLISHED')
