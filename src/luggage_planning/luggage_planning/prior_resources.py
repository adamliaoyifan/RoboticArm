"""Read-only Linux process-tree resource sampling for bounded prior experiments.

CPU is the sum of observed per-process user/system deltas, not cumulative
children CPU (which would double count). RSS is a sampled aggregate, not the
sum of independent lifetime peaks. Very short-lived children can be missed;
the report preserves that limitation and root continuity explicitly.
"""
from dataclasses import dataclass
import os
from pathlib import Path
import threading
import time


@dataclass(frozen=True)
class Process:
    pid: int
    parent: int
    start_ticks: int
    cpu_ticks: int
    rss_bytes: int

    @property
    def identity(self):
        return self.pid, self.start_ticks


def read_processes(proc_root=Path('/proc')):
    page_size = os.sysconf('SC_PAGE_SIZE')
    rows, errors = {}, []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            # comm may contain spaces and parentheses; numeric fields follow
            # its LAST closing parenthesis. Indices below start at field 3.
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            row = Process(int(entry.name), int(fields[1]), int(fields[19]),
                          int(fields[11]) + int(fields[12]), max(0, int(fields[21])) * page_size)
            rows[row.pid] = row
        except FileNotFoundError:
            continue  # process disappeared between directory read and stat
        except (PermissionError, OSError, IndexError, ValueError) as exc:
            errors.append(dict(pid=int(entry.name), error=type(exc).__name__))
    return rows, errors


class ProcessTreeLedger:
    """Accumulate one measured window, with PID reuse and reparenting handled."""
    def __init__(self, rows, planner_roots, prior_roots, *, ticks_per_second=None):
        self.ticks_per_second = ticks_per_second or os.sysconf('SC_CLK_TCK')
        roots = list(planner_roots) + list(prior_roots)
        if not planner_roots or not prior_roots or len(set(roots)) != len(roots):
            raise ValueError('distinct planner and prior root PIDs required')
        if any(pid not in rows for pid in roots):
            raise ValueError('all measured roots must be live at window start')
        self.roots = {rows[pid].identity: ('prior' if pid in prior_roots else 'planner') for pid in roots}
        self.owned = dict(self.roots)
        self.baselines = {}
        self.maximums = {}
        self.samples = []
        self.discontinuities = []
        self.lost_processes = []
        self.add(rows, elapsed_s=0., initial=True)

    def add(self, rows, *, elapsed_s, initial=False):
        for identity in self.owned:
            current = rows.get(identity[0])
            if current is None or current.identity != identity:
                event = dict(process=list(identity), elapsed_s=elapsed_s)
                if not any(e['process'] == event['process'] for e in self.lost_processes):
                    self.lost_processes.append(event)
        for identity in self.roots:
            current = rows.get(identity[0])
            if current is None or current.identity != identity:
                event = dict(root=list(identity), elapsed_s=elapsed_s)
                if not any(e['root'] == event['root'] for e in self.discontinuities):
                    self.discontinuities.append(event)
        active = {pid: self.owned[p.identity] for pid, p in rows.items() if p.identity in self.owned}
        # Existing descendant identity remains owned if subsequently reparented.
        changed = True
        while changed:
            changed = False
            for pid, row in rows.items():
                if pid not in active and row.parent in active:
                    active[pid] = active[row.parent]
                    self.owned[row.identity] = active[pid]
                    changed = True
        rss = dict(planner=0, prior=0)
        for pid, group in active.items():
            row = rows[pid]
            self.baselines.setdefault(row.identity, row.cpu_ticks if initial else 0)
            self.maximums[row.identity] = max(self.maximums.get(row.identity, 0), row.cpu_ticks)
            rss[group] += row.rss_bytes
        cpu = dict(planner=0., prior=0.)
        for identity, maximum in self.maximums.items():
            cpu[self.owned[identity]] += max(0, maximum - self.baselines[identity]) / self.ticks_per_second
        self.samples.append(dict(elapsed_s=elapsed_s, live_processes=len(active),
                                 rss_bytes=rss, cpu_s=cpu))

    def report(self):
        last = self.samples[-1]
        return dict(
            resource_scope='planner_and_prior_process_trees',
            measurement_kind='sampled_proc_stat',
            root_continuity=not self.discontinuities,
            discontinuities=self.discontinuities,
            lost_processes=self.lost_processes,
            cpu_s=sum(last['cpu_s'].values()), prior_cpu_s=last['cpu_s']['prior'],
            peak_rss_bytes=max(sum(s['rss_bytes'].values()) for s in self.samples),
            prior_peak_rss_bytes=max(s['rss_bytes']['prior'] for s in self.samples),
            observed_processes=[dict(pid=i[0], start_ticks=i[1], group=g)
                                for i, g in sorted(self.owned.items())],
            samples=self.samples,
            limitations=['CPU excludes work after an observed process last sample and before exit',
                         'Processes born and exited between samples may be missed',
                         'Aggregate RSS is sampled and shared pages can be counted by multiple processes'])


class ResourceWindow:
    """Sample only a specified planning/preparation window, excluding startup."""
    def __init__(self, planner_roots, prior_roots, *, interval_s=.05):
        if not .01 <= interval_s <= 1.:
            raise ValueError('sample interval must be between 10 ms and 1 second')
        self.planner_roots, self.prior_roots = planner_roots, prior_roots
        self.interval_s = interval_s
        self.stop_event = threading.Event()
        self.thread = None
        self.errors = []

    def __enter__(self):
        if self.thread is not None:
            raise RuntimeError('resource window cannot be reused')
        rows, errors = read_processes()
        self.errors.extend(errors)
        self.ledger = ProcessTreeLedger(rows, self.planner_roots, self.prior_roots)
        self.started = time.monotonic()
        self.thread = threading.Thread(target=self._sample, daemon=True)
        self.thread.start()
        return self

    def _sample(self):
        try:
            while not self.stop_event.wait(self.interval_s):
                rows, errors = read_processes()
                self.errors.extend(errors)
                self.ledger.add(rows, elapsed_s=time.monotonic() - self.started)
        except Exception as exc:
            self.errors.append(dict(error=repr(exc)))

    def __exit__(self, *exc):
        self.stop_event.set()
        self.thread.join()
        rows, errors = read_processes()
        self.errors.extend(errors)
        self.ledger.add(rows, elapsed_s=time.monotonic() - self.started)
        self.result = self.ledger.report()
        self.result.update(interval_s=self.interval_s, read_errors=self.errors,
                           wall_s=time.monotonic() - self.started)
        self.result['measurement_complete'] = (self.result['root_continuity']
            and not self.result['lost_processes'] and not self.errors)
