#!/usr/bin/env python3
import subprocess
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[3]
eval_py = root / "scripts" / "g3" / "occ_observer_eval.py"
r = subprocess.run([sys.executable, str(eval_py), "--help"], check=False)
sys.exit(0 if r.returncode == 0 else 3)
