"""Joint command boundary capture (plan: docs/plans/joint_command_boundary_capture.md).

Sim chain: JTC-published state + plugin-received command-interface values ->
gz plugin applied writes -> physical feedback. Real machine: backend interface
reserved, not implemented (explicit refusal, no silent fallback).
"""

__version__ = "0.1.0"
