"""G4-A gen3 M3/M4: isolation CLI dispatcher wiring BEHAVES (plan
section 8: 'verify the actual wiring -- params must really control
fixtures/samples/outputs'; codex note: cwd/param-forwarding issues).
Each test runs the real subprocess and checks its EFFECT, never
--help output.
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
CLI = os.path.join(REPO, 'scripts', 'g3', 'occ_query_eval_g4.py')


def run_cli(argv, timeout=60):
    return subprocess.run([sys.executable, CLI] + argv,
                          capture_output=True, text=True, timeout=timeout)


class TestJudgeStage(unittest.TestCase):

    def _events(self, path, n_queries=40, n_commits=35, drop_last=False,
                p95_latencies=None):
        lines = []
        t = 0
        for i in range(n_commits):
            t += 10_000_000
            lines.append({'kind': 'commit', 'op_id': 'c%d' % i,
                          'ledger_id': i,
                          'identity': ['e', i, 'a', 'b', i],
                          'epoch': 'e', 'accepted': True,
                          't_ns': t, 'write_ms': 5.0})
            lines.append({'kind': 'ledger', 'ledger_id': i,
                          'identity': ['e', i, 'a', 'b', i],
                          'commit_ns': t, 'status': 'exact_visible',
                          'published_seq': i + 1,
                          'observed_ns': t + 5e6})
            lines.append({'event': 'ready', 'seq': i + 1,
                          'target_identity': ['e', i, 'a', 'b', i],
                          't_ns': t + 6_000_000})
        if drop_last:
            lines = [l for l in lines if not (
                l.get('event') == 'ready' and l['seq'] == n_commits)]
        for i in range(n_queries):
            lines.append({'kind': 'query', 'request_id': 'q%d' % i,
                          'expected': 'CURRENT', 'returned': None,
                          'status': 'ok',
                          'total_ms': (p95_latencies[i]
                                       if p95_latencies else 8.0),
                          'state_at_request': 'READY', 'startup': False})
        with open(path, 'w') as handle:
            for line in lines:
                handle.write(json.dumps(line) + '\n')

    def test_pass_stream_exits_0_and_writes_summary(self):
        with tempfile.TemporaryDirectory() as tmp:
            events = os.path.join(tmp, 'e.jsonl')
            summary = os.path.join(tmp, 's.json')
            self._events(events)
            result = run_cli(['judge', events, '--summary', summary])
            self.assertEqual(result.returncode, 0, result.stderr[-400:])
            verdict = json.load(open(summary))
            self.assertEqual(verdict['exit'], 0)

    def test_bound_param_actually_controls_verdict(self):
        """--vis-p95-ms must really be forwarded: a stream with 1.5 s
        visibility passes with a 2000 ms bound, fails at 1000 ms."""
        with tempfile.TemporaryDirectory() as tmp:
            events = os.path.join(tmp, 'e.jsonl')
            self._events(events, n_commits=35)
            slow = os.path.join(tmp, 'slow.jsonl')
            with open(events) as src, open(slow, 'w') as dst:
                for line in src:
                    record = json.loads(line)
                    if record.get('event') == 'ready':
                        record['t_ns'] += 1_500_000_000
                    dst.write(json.dumps(record) + '\n')
            lax = run_cli(['judge', slow, '--vis-p95-ms', '2000',
                           '--vis-max-ms', '2000'])
            strict = run_cli(['judge', slow, '--vis-p95-ms', '1000'])
            self.assertEqual(lax.returncode, 0, lax.stdout[-300:])
            self.assertEqual(strict.returncode, 3, strict.stdout[-300:])

    def test_dropped_generation_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            events = os.path.join(tmp, 'e.jsonl')
            self._events(events, drop_last=True)
            result = run_cli(['judge', events])
            self.assertEqual(result.returncode, 3)


class TestOutputRefusal(unittest.TestCase):

    def test_gate_scripts_refuse_existing_out(self):
        """Both v3 gate scripts must refuse to overwrite an existing
        --out (plan section 8: 'old directories refuse writes')."""
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, 'faults.json')
            open(out, 'w').write('sentinel')
            script = os.path.join(REPO, 'scripts', 'g3',
                                  'occ_fault_gates_v3.py')
            result = subprocess.run(
                [sys.executable, script, '--gates', '', '--out', out],
                capture_output=True, text=True, timeout=30)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(open(out).read(), 'sentinel')
            rep = os.path.join(REPO, 'scripts', 'g3',
                               'occ_concurrent_representative_v3.py')
            events = out + '.x.events.jsonl'
            open(events, 'w').write('sentinel')
            result = subprocess.run(
                [sys.executable, rep, '--duration-s', '1',
                 '--out', out + '.x'], capture_output=True, text=True,
                timeout=30)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(open(events).read(), 'sentinel')


if __name__ == '__main__':
    unittest.main()
