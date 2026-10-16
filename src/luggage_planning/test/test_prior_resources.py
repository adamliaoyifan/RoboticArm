import subprocess
import sys
from luggage_planning.prior_resources import Process, ProcessTreeLedger, ResourceWindow


def p(pid, parent=0, start=1, cpu=0, rss=100):
    return Process(pid, parent, start, cpu, rss)


def test_descendants_reparenting_reuse_and_unrelated_process_exclusion():
    ledger = ProcessTreeLedger({1:p(1,cpu=100),2:p(2,cpu=50),3:p(3,1,cpu=20),9:p(9,rss=99999)},
                               [1], [2], ticks_per_second=100)
    ledger.add({1:p(1,cpu=110),2:p(2,cpu=70),3:p(3,0,cpu=30),4:p(4,2,cpu=5)},elapsed_s=1)
    result = ledger.report()
    assert result['cpu_s'] == .45 and result['prior_cpu_s'] == .25
    assert result['peak_rss_bytes'] == 400
    assert len(result['observed_processes']) == 4
    # Reused descendant PID is no longer owned when unrelated to live roots.
    ledger.add({1:p(1,cpu=110),2:p(2,cpu=70),3:p(3,0,start=2,cpu=999,rss=99999)},elapsed_s=2)
    assert ledger.report()['cpu_s'] == .45
    assert ledger.report()['lost_processes']
    # A replaced root cannot silently continue the original measurement.
    ledger.add({1:p(1,start=2,cpu=999),2:p(2,cpu=70)},elapsed_s=3)
    assert not ledger.report()['root_continuity']


def test_aggregate_peak_is_not_sum_of_independent_lifetime_peaks():
    ledger = ProcessTreeLedger({1:p(1,rss=1000),2:p(2,rss=100)},[1],[2])
    ledger.add({1:p(1,rss=100),2:p(2,rss=1000)},elapsed_s=1)
    assert ledger.report()['peak_rss_bytes'] == 1100


def test_live_resource_window_measures_work_and_does_not_terminate_roots():
    # Roots remain alive until AFTER the final sample, so completed CPU work
    # and held memory are measurable without relying on exit timing.
    program = '''import sys,time
print('ready',flush=True)
sys.stdin.readline()
memory=bytearray(16*1024*1024)
until=time.process_time()+.12
while time.process_time()<until: pass
print('done',flush=True)
sys.stdin.readline()
'''
    children = [subprocess.Popen([sys.executable,'-c',program],stdin=subprocess.PIPE,
                                 stdout=subprocess.PIPE,text=True) for _ in range(2)]
    try:
        for child in children:
            assert child.stdout.readline().strip() == 'ready'
        with ResourceWindow([children[0].pid],[children[1].pid],interval_s=.01) as window:
            for child in children:
                child.stdin.write('go\n');child.stdin.flush()
            for child in children:
                assert child.stdout.readline().strip() == 'done'
        assert all(child.poll() is None for child in children)
        assert window.result['measurement_complete']
        assert window.result['cpu_s'] >= .2
        assert window.result['prior_cpu_s'] >= .1
        assert window.result['peak_rss_bytes'] >= 32*1024*1024
        assert len(window.result['samples']) >= 2
    finally:
        for child in children:
            if child.poll() is None:
                child.stdin.write('stop\n');child.stdin.flush()
            child.wait(timeout=5)
