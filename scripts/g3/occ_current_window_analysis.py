#!/usr/bin/env python3
"""G4-A gen3 CURRENT window decomposition (review_three_issues #2).

PURE offline analysis over an existing representative events JSONL:
- every steady reject is classified as LAG_WINDOW (a commit is pending
  propagation: last commit newer than last READY) or RESYNC_WINDOW (a
  resync episode was open: resync event newer than last READY);
- cause-specific window counts, longest continuous un-queryable
  streak (count and wall time), first-success delay per episode;
- SEQ_GAP verification: which published ledger seqs the receiver
  never READ (skipped pointer) vs read-but-rejected;
- facade-vs-typed same-cycle comparison (scheduling hypothesis: typed
  queries happen later in the same cycle against a newer writer).

No product claims: this only explains the existing trace.
"""

import argparse
import json


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('events_jsonl')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    with open(args.events_jsonl) as handle:
        records = [json.loads(line) for line in handle if line.strip()]

    commits = [r for r in records if r.get('kind') == 'commit'
               and r.get('accepted')]
    readies = [r for r in records if r.get('event') == 'ready']
    resyncs = [r for r in records if r.get('event') == 'resync']
    received = [r for r in records if r.get('event') == 'received']
    ledgers = [r for r in records if r.get('kind') == 'ledger']
    queries = [r for r in records if r.get('kind') == 'query']

    # --- classify steady rejects ------------------------------------
    def classify(query):
        if query.get('startup') or query.get('status') == 'ok':
            return None
        t = query.get('t_ns', 0)
        last_ready = max((r['t_ns'] for r in readies
                          if r['t_ns'] <= t), default=0)
        last_resync = max((r['t_ns'] for r in resyncs
                           if r['t_ns'] <= t), default=0)
        last_commit = max((c['t_ns'] for c in commits
                           if c['t_ns'] <= t), default=0)
        if last_resync > last_ready:
            return 'RESYNC_WINDOW'
        if last_commit > last_ready:
            return 'LAG_WINDOW'
        return 'OTHER'

    per_path = {}
    for path in ('facade', 'typed'):
        sel = [q for q in queries if q.get('path') == path
               and not q.get('startup')]
        reasons = {}
        streak_len, streak_start, worst_streak, episodes = 0, None, 0, []
        first_reject_in_episode = None
        for q in sel:
            if q['status'] == 'ok':
                if first_reject_in_episode is not None:
                    episodes.append({
                        'start_ns': first_reject_in_episode,
                        'end_ns': q.get('t_ns', 0),
                        'rejects': streak_len,
                        'first_success_delay_ms': round(
                            (q.get('t_ns', 0)
                             - first_reject_in_episode) / 1e6, 1)})
                streak_len, first_reject_in_episode = 0, None
                continue
            klass = classify(q)
            reasons[klass] = reasons.get(klass, 0) + 1
            if first_reject_in_episode is None:
                first_reject_in_episode = q.get('t_ns', 0)
            streak_len += 1
            worst_streak = max(worst_streak, streak_len)
        worst_wall = max((e['first_success_delay_ms'] for e in episodes),
                         default=0.0)
        per_path[path] = {
            'total': len(sel),
            'ok': sum(1 for q in sel if q['status'] == 'ok'),
            'reject_classification': reasons,
            'longest_reject_streak': worst_streak,
            'episodes': len(episodes),
            'worst_episode_ms': worst_wall,
            'worst_episode_detail': max(
                episodes, key=lambda e: e['first_success_delay_ms'],
                default=None)}

    # --- SEQ_GAP verification ----------------------------------------
    published_seqs = sorted({r['published_seq'] for r in ledgers
                             if r.get('published_seq')})
    received_seqs = {r['seq'] for r in received}
    ready_seqs = {r['seq'] for r in readies}
    skipped = [s for s in published_seqs if s not in received_seqs]
    resync_reasons = {}
    for r in resyncs:
        resync_reasons[r.get('reason')] = \
            resync_reasons.get(r.get('reason'), 0) + 1

    # --- facade vs typed same-cycle scheduling check -----------------
    by_cycle = {}
    for q in queries:
        if q.get('startup'):
            continue
        by_cycle.setdefault(str(q.get('request_id', ''))[1:] \
                            if q.get('request_id') else '', {})[
            q.get('path')] = q.get('status')
    facade_ok_typed_reject = sum(
        1 for c in by_cycle.values()
        if c.get('facade') == 'ok' and c.get('typed') != 'ok')
    both_reject = sum(1 for c in by_cycle.values()
                      if c.get('facade') != 'ok' and c.get('typed') != 'ok')

    report = {
        'scope': 'offline decomposition of an existing trace',
        'commits': len(commits), 'readies': len(readies),
        'resync_events': len(resyncs),
        'resync_reasons': resync_reasons,
        'per_path': per_path,
        'seq_gap_check': {
            'published_generations': len(published_seqs),
            'receiver_read': len(received_seqs & set(published_seqs)),
            'skipped_pointer_seqs': skipped[:20],
            'skipped_count': len(skipped),
            'skipped_became_ready': sum(
                1 for s in skipped if s in ready_seqs)},
        'same_cycle': {
            'cycles': len(by_cycle),
            'facade_ok_typed_rejected': facade_ok_typed_reject,
            'both_rejected': both_reject},
    }
    with open(args.out, 'w') as handle:
        json.dump(report, handle, indent=1)
    print(json.dumps(report, indent=1))


if __name__ == '__main__':
    main()
