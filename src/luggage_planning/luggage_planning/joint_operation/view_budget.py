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
    def __init__(self, path, *, capacity=128, clock=time.time):
        if not path or type(capacity) is not int or capacity < 1:
            raise ValueError('EXPLICIT_VIEW_BUDGET_STORAGE_REQUIRED')
        self.path, self.capacity, self.clock = Path(path), capacity, clock
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.rows = json.loads(self.path.read_text()) if self.path.exists() else {}
        if not isinstance(self.rows, dict) or len(self.rows) > capacity:
            raise ValueError('INVALID_VIEW_BUDGET_LEDGER')
        self.lock = threading.RLock()

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
                if any(item['domain_sha256'] == domain_sha256 for item in self.rows.values()):
                    raise ValueError('VIEW_ACQUISITION_TASK_RENAME_REFUSED')
                if len(self.rows) >= self.capacity:
                    raise ValueError('VIEW_RECOVERY_LEDGER_CAPACITY')
                row = dict(domain_sha256=domain_sha256, max_motions=max_motions,
                    total_budget_s=total_budget_s, opened_wall_s=now, last_wall_s=now, dispatched=0)
                self.rows[task_id] = row
            elif (row['domain_sha256'], row['max_motions'], row['total_budget_s']) != (
                    domain_sha256, max_motions, total_budget_s):
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
