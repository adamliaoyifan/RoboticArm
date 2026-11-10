"""Behavior controls for the gen6 evidence-root parameterization (review ruling #2).

Covers: absolute-root enforcement, the campaign-visible common.EVIDENCE view,
budget checks following the effective root, CLI rejection of outs outside the
effective root, and the G2 default root gaining nothing when a new root is in
effect. No scored campaign runs here.
"""
import subprocess
import sys
from pathlib import Path

import pytest

import occ_query_eval as runner

G2_DEFAULT_SUBPATH = 'docs/status/evidence/occ_owner_b1/occ_query_index_g2_20261025'


@pytest.fixture
def restore_root():
    yield
    runner.EVIDENCE = runner.EVIDENCE_DEFAULT


def test_default_root_is_g2_and_untouched_module_state(restore_root):
    runner.EVIDENCE = runner.EVIDENCE_DEFAULT
    assert runner.EVIDENCE == runner.ROOT/G2_DEFAULT_SUBPATH


def test_apply_requires_absolute_path(restore_root):
    with pytest.raises(RuntimeError):
        runner.apply_evidence_root('relative/root')


def test_apply_repoints_the_campaign_view(restore_root, tmp_path):
    new = runner.apply_evidence_root(str(tmp_path/'g6root'))
    assert new == (tmp_path/'g6root').resolve()
    assert runner.EVIDENCE == new
    assert runner.EVIDENCE != runner.EVIDENCE_DEFAULT


def test_budget_follows_effective_root(restore_root, tmp_path, monkeypatch):
    root = tmp_path/'g6root'
    out = root/'run01'
    out.mkdir(parents=True)
    (out/'blob').write_bytes(b'x'*1024)
    runner.apply_evidence_root(str(root))
    runner.check_evidence_budget(out)
    monkeypatch.setattr(runner, 'RUN_BYTES', 512)
    with pytest.raises(RuntimeError):
        runner.check_evidence_budget(out)


def _cli(*argv, timeout=120):
    return subprocess.run(
        [sys.executable, str(runner.ROOT/'scripts/g3/occ_query_eval.py'), *argv],
        cwd='/tmp', capture_output=True, text=True, timeout=timeout)


def _g2_snapshot():
    root = runner.ROOT/G2_DEFAULT_SUBPATH
    if not root.exists():
        return []
    return sorted(str(p.relative_to(root)) for p in root.rglob('*'))


def test_cli_rejects_outside_root_and_g2_gains_nothing(tmp_path):
    before = _g2_snapshot()
    outside = tmp_path/'escape'
    result = _cli('benchmark', '--mode', 'all',
                  '--evidence-root', str(tmp_path/'g6root'),
                  '--out', str(outside))
    assert result.returncode != 0
    assert 'evidence outside task root' in result.stderr
    assert not outside.exists()
    assert _g2_snapshot() == before


def test_cli_without_override_still_pins_g2(tmp_path):
    # No override: the effective root stays the G2 default (unit-tested above),
    # so an out beside it is rejected by the same containment check.
    outside = tmp_path/'escape'
    result = _cli('benchmark', '--mode', 'all', '--out', str(outside))
    assert result.returncode != 0
    assert 'evidence outside task root' in result.stderr
    assert not outside.exists()
    assert runner.EVIDENCE_DEFAULT == runner.ROOT/G2_DEFAULT_SUBPATH
