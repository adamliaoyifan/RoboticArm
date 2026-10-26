"""Behavioral checks for statistic ranks and impossible profile records."""
import importlib.util
from pathlib import Path
import pytest

path = Path(__file__).resolve().parents[3]/'scripts/g3/occ_observer_profile_report.py'
spec = importlib.util.spec_from_file_location('profile_report',path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_nearest_rank_keeps_29th_sample():
    values=[1]*28+[200,300]
    assert module.stats(values)['p95']==200
    assert module.stats(values)['p99']==300


def test_negative_duration_rejected():
    row={'schema':'observer_profile_v1','status':'OK','mode':'stages',
         'receipt':{'profile':{'mode':'stages','enabled':True,'stage_ms':{'x':-1}}}}
    with pytest.raises(ValueError,match='duration'):
        module.validate(row)


def test_failed_sample_never_becomes_pass():
    with pytest.raises(ValueError,match='failed'):
        module.validate({'schema':'observer_profile_v1','status':'INPUT_HASH_MISMATCH'})


def test_nonfinite_aggregate_rejected():
    with pytest.raises(ValueError,match='invalid metric'):
        module.stats([1,float('nan')])
