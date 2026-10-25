"""Invalid workload requests are rejected before creating an evidence run."""
from pathlib import Path
import subprocess
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[3]/'scripts/g3/occ_query_eval.py'


@pytest.mark.parametrize('arguments', [
    ['--seeds', '0'], ['--seed-start', '-1'], ['--transactions', '0'],
    ['--distance-points', '-1'], ['--resolutions', '.03'],
    ['--resolutions', '.01,.01'], ['--boxes', '2'], ['--shapes', 'unknown'],
])
def test_invalid_matrix_creates_no_artifacts(tmp_path, arguments):
    out = tmp_path/'must_not_exist'
    result = subprocess.run([sys.executable, str(SCRIPT), 'oracle', '--out', str(out),
                             *arguments], capture_output=True, text=True)
    assert result.returncode == 2
    assert not out.exists()


@pytest.mark.parametrize('command', ['oracle', 'benchmark', 'concurrency', 'planning', 'replay'])
def test_help_needs_no_ros_graph_or_product_import(command):
    result = subprocess.run([sys.executable, str(SCRIPT), command, '--help'],
                            capture_output=True, text=True)
    assert result.returncode == 0


@pytest.mark.parametrize('arguments', [
    ['--domain', '0'], ['--samples', '0'], ['--warmup', '-1'],
    ['--boxes', '2'], ['--resolutions', '.03'],
])
def test_invalid_runtime_matrix_creates_no_artifacts(tmp_path, arguments):
    out = tmp_path/'must_not_exist'
    result = subprocess.run([sys.executable, str(SCRIPT), 'benchmark', '--out', str(out),
        '--install', str(tmp_path/'install'), *arguments], capture_output=True, text=True)
    assert result.returncode == 2
    assert not out.exists()


def test_timing_summary_requires_samples_and_uses_nearest_rank():
    import importlib.util
    path = Path(__file__).with_name('occ_query_runtime_benchmark.py')
    spec = importlib.util.spec_from_file_location('g2_timing_summary', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.summarize([1.]*29, (50, 200))['outcome'] == 'not_evaluated'
    result = module.summarize(list(range(1, 31)), (29, 30))
    assert result['p95_ms'] == 29 and result['outcome'] == 'pass'
    assert module.summarize(list(range(1, 31)), (28, 30))['outcome'] == 'fail'


@pytest.mark.parametrize('command,arguments',[
    ('concurrency',['--duration-wall','0']),('concurrency',['--repeat','0']),
    ('concurrency',['--domain','1']),('concurrency',['--boxes','1']),
    ('planning',['--paths','other']),('planning',['--boxes','1']),
    ('planning',['--samples','0']),('planning',['--resolutions','.03']),
])
def test_additional_invalid_workloads_create_no_artifacts(tmp_path,command,arguments):
    out=tmp_path/'must_not_exist'
    result=subprocess.run([sys.executable,str(SCRIPT),command,'--out',str(out),*arguments],
        capture_output=True,text=True)
    assert result.returncode==2
    assert not out.exists()


def test_trace_pair_inputs_are_adjacent_balanced_and_frozen():
    import importlib.util
    path=Path(__file__).with_name('occ_query_runtime_benchmark.py')
    spec=importlib.util.spec_from_file_location('trace_pair_workload',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    sequence=module.paired_sequence(3,30,module.TRACE_PAIR_SEED)
    assert sequence==module.paired_sequence(3,30,module.TRACE_PAIR_SEED)
    assert len(sequence)==68
    measured_first=[]
    for first,second in zip(sequence[::2],sequence[1::2]):
        assert first[0]==second[0]
        assert {first[1],second[1]}=={'trace_on','trace_off'}
        if first[0]>=3:measured_first.append(first[1])
    assert measured_first.count('trace_on')==measured_first.count('trace_off')==15
    assert sequence!=module.paired_sequence(3,30,module.TRACE_PAIR_SEED+1)
