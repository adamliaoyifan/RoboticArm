"""Publication protocol failures must not be confused with actual tracking."""
from dataclasses import replace
from pathlib import Path
import json
import os
import sys
import tempfile
import unittest

from carriage_commands import CarriageCommandPublisher
from execution_policy import ExecutionPolicy
from release_fixture import ReleaseFixturePolicy


class CarriageCommandContracts(unittest.TestCase):
    def peer(self, directory, body):
        path = directory / 'protocol_peer'
        path.write_text('#!' + sys.executable + '\n' + body)
        path.chmod(0o755)
        return path

    def publisher(self, directory, body):
        return CarriageCommandPublisher(self.peer(directory, body), ('/x', '/z'), os.environ,
            directory, ReleaseFixturePolicy.load(), replace(ExecutionPolicy.load(),
                condition_wait_budget_s=.5, observation_max_age_s=.1))

    def test_command_values_and_sequence_are_published_without_tracking_claim(self):
        body = '''import sys,json
print(json.dumps(dict(ready=True)),flush=True)
for line in sys.stdin:
 seq,x,z=line.split()
 print(json.dumps(dict(sequence=int(seq),published=True)),flush=True)
'''
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary); publisher = self.publisher(directory, body)
            try:
                publisher.start()
                result = publisher.publish((.0123, -.0456), 2.)
                self.assertEqual(result['positions_xz_m'], (.0123, -.0456))
                self.assertFalse(result['tracking_confirmed'])
                self.assertEqual(publisher.publish((.01, -.04), 3.)['sequence'], 2)
                for positions in ((float('nan'), 0.), (.6, 0.), (False, 0.), (0.,)):
                    with self.assertRaises(ValueError):
                        publisher.publish(positions, 3.)
            finally:
                publisher.close()
            requests = [json.loads(line)['request'] for line in (directory / 'joint_commands.jsonl').read_text().splitlines()
                        if 'request' in json.loads(line)]
            self.assertEqual(len(requests), 2)

    def test_wrong_sequence_or_missing_newline_never_acknowledges_command(self):
        for response, expected in [('print("{\\"sequence\\":99,\\"published\\":true}",flush=True)',
                                     'ACK_MISMATCH'),
                                    ('sys.stdout.write("{");sys.stdout.flush();time.sleep(.3)',
                                     'ACK_TIMEOUT')]:
            body = 'import sys,time\nprint("{\\"ready\\":true}",flush=True)\nsys.stdin.readline()\n' + response + '\n'
            with self.subTest(response=response), tempfile.TemporaryDirectory() as temporary:
                publisher = self.publisher(Path(temporary), body)
                try:
                    publisher.start()
                    with self.assertRaisesRegex((RuntimeError, TimeoutError), expected):
                        publisher.publish((0., 0.), 1.)
                finally:
                    publisher.close()

    def test_disconnected_peer_is_not_ready(self):
        with tempfile.TemporaryDirectory() as temporary:
            publisher = self.publisher(Path(temporary), 'print("{\\"ready\\":false}",flush=True)\n')
            try:
                with self.assertRaisesRegex(RuntimeError, 'NOT_READY'):
                    publisher.start()
            finally:
                publisher.close()


if __name__ == '__main__':
    unittest.main()
