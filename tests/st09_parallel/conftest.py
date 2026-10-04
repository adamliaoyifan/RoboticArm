"""Shared pytest path setup for the ST09 parallel task directories.

Adds the workspace root (repo root) to sys.path so the ROS-free
``st09_contracts`` package resolves regardless of the install state.

``tests/st09_parallel/B`` (``mock_graph``) and ``tests/st09_parallel/I``
(``whole_chain``) each call ``rclpy.init()`` on their own ROS domain.
One interpreter cannot host both: the second init is the nine
``whole_chain`` errors seen when both directories are collected together.
The documented gate is ``scripts/st09_i/run_bi_route_tests.sh``. A combined
``pytest`` invocation that still collects both live fixtures is re-run as
those two processes instead of sharing a context.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

_CHILD_ENV = "ST09_ROUTE_TEST_CHILD"
_OPT_WITH_ARG = {
    "-k", "-m", "-p", "--tb", "-o", "-c", "--confcutdir", "--basetemp",
    "--junitxml", "--html", "--maxfail", "--timeout", "--durations",
    "-n", "--dist", "--rootdir", "--override-ini", "-W", "--ignore",
    "--deselect", "--log-level", "--log-cli-level",
}


def partition_pytest_args(args):
    """Split a pytest argv into option tokens and path tokens.

    Flags that take a separate value stay in the option list with that
    value. ``--name=value`` stays one token. Everything else is a path.
    """
    options = []
    paths = []
    index = 0
    while index < len(args):
        token = args[index]
        if token == "--":
            paths.extend(args[index + 1:])
            break
        if token.startswith("-"):
            options.append(token)
            name = token.split("=", 1)[0]
            if name in _OPT_WITH_ARG and "=" not in token:
                index += 1
                if index < len(args):
                    options.append(args[index])
            index += 1
            continue
        paths.append(token)
        index += 1
    return options, paths


def _live_ros_owners(items):
    owners = set()
    for item in items:
        names = getattr(item, "fixturenames", ()) or ()
        if "mock_graph" in names:
            owners.add("B")
        if "whole_chain" in names:
            owners.add("I")
    return owners


@pytest.hookimpl(trylast=True)
def pytest_collection_modifyitems(config, items):
    """Re-run B and I in separate processes when both live fixtures appear."""
    if os.environ.get(_CHILD_ENV):
        return
    if getattr(config.option, "collectonly", False):
        return
    if _live_ros_owners(items) != {"B", "I"}:
        return
    options, _paths = partition_pytest_args(
        list(config.invocation_params.args))
    script = ROOT / "scripts" / "st09_i" / "run_bi_route_tests.sh"
    print(
        "ST09: mock_graph and whole_chain need separate ROS contexts; "
        "running %s" % script,
        flush=True)
    code = subprocess.call([str(script), *options], cwd=str(ROOT))
    pytest.exit(
        "B and I ROS suites ran in separate processes (exit %s)" % code,
        returncode=int(code))
