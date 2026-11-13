"""Durable per-pick recovery budgets; retries never renew time or motion units.

Wall time persists across restart. Clock rollback refuses instead of granting
extra time. Entries are explicitly retained: capacity pressure cannot silently
erase a box's consumed budget. The orchestration task owns the stable task key.
"""
import copy
import json
import math
import os
from pathlib import Path
import threading
import time


class PickupViewBudgetLedger:
    def __init__(self, path, *, capacity=128, max_acquisitions_per_task=128,
                 max_terminal_receipt_bytes=8*1024*1024, clock=time.time):
        if (not path or type(capacity) is not int or capacity < 1
                or type(max_acquisitions_per_task) is not int or max_acquisitions_per_task < 1
                or type(max_terminal_receipt_bytes) is not int or max_terminal_receipt_bytes < 1):
            raise ValueError('EXPLICIT_VIEW_BUDGET_STORAGE_REQUIRED')
        self.path, self.capacity, self.clock = Path(path), capacity, clock
        self.max_acquisitions_per_task = max_acquisitions_per_task
        self.max_terminal_receipt_bytes = max_terminal_receipt_bytes
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.rows = json.loads(self.path.read_text()) if self.path.exists() else {}
        if not isinstance(self.rows, dict) or len(self.rows) > capacity:
            raise ValueError('INVALID_VIEW_BUDGET_LEDGER')
        self.lock = threading.RLock()

    @staticmethod
    def _acquisitions(row):
        # Old durable rows keep their original opening/count/time; no migration
        # writes are necessary until a genuinely verified new acquisition arrives.
        return row.get('acquisitions', {row['domain_sha256']: 'initial'})

    def adopt_acquisition(self, task_id, association):
        """Caller owns matching; adopt its exact typed proof without a new budget."""
        from luggage_perception.stationary_pickup_association import StationaryPickupAssociation
        if (type(association) is not StationaryPickupAssociation
                or association.accepted is not True
                or association.reason != 'CONDITIONAL_UNIQUE_STATIONARY_MATCH'):
            raise ValueError('VERIFIED_STATIONARY_ASSOCIATION_REQUIRED')
        with self.lock:
            row = self.rows[task_id]
            now = self._now(row)
            if now-row['opened_wall_s'] >= row['total_budget_s']:
                raise ValueError('VIEW_RECOVERY_TIME_LIMIT')
            acquisitions = dict(self._acquisitions(row))
            before, after = association.before_domain_sha256, association.after_domain_sha256
            if before not in acquisitions:
                raise ValueError('VIEW_ALIAS_PARENT_NOT_AUTHORIZED')
            if any(after in self._acquisitions(other) for key, other in self.rows.items() if key != task_id):
                raise ValueError('VIEW_ACQUISITION_TASK_RENAME_REFUSED')
            proof = association.digest()
            if after in acquisitions:
                if acquisitions[after] != proof:
                    raise ValueError('VIEW_ALIAS_PROOF_REBIND_REFUSED')
                return  # Identical retry does not update any persisted clock.
            if len(acquisitions) >= self.max_acquisitions_per_task:
                raise ValueError('VIEW_RECOVERY_ACQUISITION_CAPACITY')
            acquisitions[after] = proof
            updated = dict(row, acquisitions=acquisitions, last_wall_s=now)
            self.rows[task_id] = updated
            try:
                self._save()
            except Exception:
                # A retry must not treat an unpersisted alias as committed.
                self.rows[task_id] = row
                raise

    def record_settled_view(self, task_id, *, plan_cdr, route_digest, domain_sha256, settled_stamp_ns):
        """Runtime-only: retain an actual successful empty Action's terminal view.

        This records an event, including after the last motion unit. It never
        grants another motion, renews time, or proves a newly observed box.
        """
        if (type(plan_cdr) is not bytes or not 0 < len(plan_cdr) <= self.max_terminal_receipt_bytes
                or any(not isinstance(v, str) or len(v) != 64
                       or any(c not in '0123456789abcdef' for c in v)
                       for v in (route_digest, domain_sha256))
                or type(settled_stamp_ns) is not int or not 0 < settled_stamp_ns < 2**64):
            raise ValueError('EXACT_BOUNDED_SETTLED_VIEW_REQUIRED')
        with self.lock:
            row = self.rows[task_id]
            now = self._now(row)
            if row['dispatched'] < 1 or domain_sha256 not in self._acquisitions(row):
                raise ValueError('CHARGED_AUTHORIZED_VIEW_REQUIRED')
            receipt = dict(plan_cdr_hex=plan_cdr.hex(), route_digest=route_digest,
                domain_sha256=domain_sha256, settled_stamp_ns=settled_stamp_ns,
                dispatched=row['dispatched'])
            previous = row.get('settled_view')
            if previous == receipt:
                return  # No clock rewrite on identical retry.
            if previous and (previous['dispatched'] >= receipt['dispatched']
                             or previous['settled_stamp_ns'] >= settled_stamp_ns):
                raise ValueError('SETTLED_VIEW_RECEIPT_REBIND_REFUSED')
            self.rows[task_id] = dict(row, settled_view=receipt, last_wall_s=now)
            try:
                self._save()
            except Exception:
                self.rows[task_id] = row
                raise

    def settled_view(self, *, route_digest, domain_sha256, settled_stamp_ns):
        """Return task and retained CDR for semantic comparison at the ROS port."""
        with self.lock:
            found = [task for task, row in self.rows.items() if row.get('settled_view') and
                all(row['settled_view'][key] == value for key, value in dict(
                    route_digest=route_digest,
                    domain_sha256=domain_sha256, settled_stamp_ns=settled_stamp_ns).items())]
            if len(found) != 1:
                raise ValueError('EXACT_RUNTIME_SETTLED_VIEW_REQUIRED')
            task = found[0]
            return task, bytes.fromhex(self.rows[task]['settled_view']['plan_cdr_hex'])

    def continuation_remaining(self, task_id):
        """Original wall-time budget, independent of exhausted motion count."""
        with self.lock:
            row = self.rows[task_id]
            remaining = row['total_budget_s']-(self._now(row)-row['opened_wall_s'])
            if remaining <= 0:
                raise ValueError('VIEW_RECOVERY_TIME_LIMIT')
            return remaining

    def _save(self):
        raw = json.dumps(self.rows, sort_keys=True, allow_nan=False)
        pending = self.path.with_name(self.path.name + '.pending')
        with pending.open('w') as stream:
            stream.write(raw); stream.flush(); os.fsync(stream.fileno())
        os.replace(pending, self.path)
        descriptor = os.open(str(self.path.parent), os.O_DIRECTORY)
        try: os.fsync(descriptor)
        finally: os.close(descriptor)

    def _now(self, row=None):
        now = self.clock()
        if not math.isfinite(now) or (row is not None and now < row['last_wall_s']):
            raise ValueError('VIEW_RECOVERY_CLOCK_ROLLBACK_OR_INVALID')
        return now

    def open(self, task_id, domain_sha256, *, max_motions, total_budget_s):
        if (not isinstance(task_id, str) or not task_id.strip()
                or len(domain_sha256) != 64 or any(c not in '0123456789abcdef' for c in domain_sha256)
                or type(max_motions) is not int or max_motions < 0
                or not math.isfinite(total_budget_s) or total_budget_s <= 0):
            raise ValueError('EXPLICIT_VIEW_RECOVERY_LIMITS_REQUIRED')
        with self.lock:
            row = self.rows.get(task_id)
            now = self._now(row)
            if row is None:
                if any(domain_sha256 in self._acquisitions(item) for item in self.rows.values()):
                    raise ValueError('VIEW_ACQUISITION_TASK_RENAME_REFUSED')
                if len(self.rows) >= self.capacity:
                    raise ValueError('VIEW_RECOVERY_LEDGER_CAPACITY')
                row = dict(domain_sha256=domain_sha256, max_motions=max_motions,
                    total_budget_s=total_budget_s, opened_wall_s=now, last_wall_s=now, dispatched=0)
                self.rows[task_id] = row
            elif (domain_sha256 not in self._acquisitions(row)
                    or (row['max_motions'], row['total_budget_s']) != (max_motions, total_budget_s)):
                raise ValueError('VIEW_RECOVERY_LIMITS_OR_ACQUISITION_REBIND_REFUSED')
            row['last_wall_s'] = now
            self._save()
            self.remaining(task_id)
            return copy.deepcopy(row)

    def remaining(self, task_id):
        with self.lock:
            row = self.rows[task_id]
            now = self._now(row)
            remaining = row['total_budget_s'] - (now - row['opened_wall_s'])
            if row['dispatched'] >= row['max_motions']:
                raise ValueError('VIEW_RECOVERY_MOTION_LIMIT')
            if remaining <= 0:
                raise ValueError('VIEW_RECOVERY_TIME_LIMIT')
            return remaining

    def dispatch(self, task_id):
        """Persist before transport attempt. Never refund failed/cancelled sends."""
        with self.lock:
            self.remaining(task_id)
            row = self.rows[task_id]
            row['last_wall_s'] = self._now(row)
            row['dispatched'] += 1
            self._save()

    def status(self, task_id):
        with self.lock:
            row = self.rows[task_id]
            return row['dispatched'], max(0., self._now(row) - row['opened_wall_s'])
