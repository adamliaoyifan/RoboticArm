"""Public research commands must load before any simulation can be launched."""
from pathlib import Path
import subprocess
import sys
import unittest


class CommandEntryContracts(unittest.TestCase):
    def test_public_commands_load_and_offer_help_without_running_work(self):
        directory = Path(__file__).resolve().parents[1]
        for name in ('scene.py', 'check_scene.py', 'run.py', 'feasibility.py',
                     'motion_feasibility.py', 'physical_probe.py', 'wiring_rehearsal.py'):
            with self.subTest(command=name):
                result = subprocess.run([sys.executable, str(directory / name), '--help'],
                                        capture_output=True, text=True, timeout=20)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn('usage:', result.stdout)
                self.assertIn('--help', result.stdout)


if __name__ == '__main__':
    unittest.main()
