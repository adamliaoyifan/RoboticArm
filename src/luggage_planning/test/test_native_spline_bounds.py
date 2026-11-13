"""Independent C++ polynomial-root isolation vs NumPy extrema and closed forms."""
import json
import math
import os
from pathlib import Path
import subprocess

import numpy as np
import pytest
from luggage_planning.motion.spline_timing_bounds import derivative_bounds


def test_cpp_continuous_bounds_match_closed_forms_and_independent_numpy_roots(tmp_path):
    root=Path(__file__).resolve().parents[3]
    evidence=os.environ.get('PICKUP_VIEW_MOTION_EVIDENCE')
    directory=Path(evidence)/'cpp_spline_bounds' if evidence else tmp_path
    directory.mkdir(parents=True,exist_ok=not bool(evidence))
    harness=directory/'bounds.cpp'
    harness.write_text('''#include <iostream>
#include <iomanip>
#include "luggage_planning/trajectory_derivative_bounds.hpp"
int main() {
  double dt,q0,q1,v0,v1,a0,a1;int order;
  while (std::cin>>dt>>q0>>q1>>v0>>v1>>a0>>a1>>order) {
    auto b=luggage_planning::trajectory_bounds::interval(dt,q0,q1,v0,v1,a0,a1,order);
    std::cout<<std::setprecision(17)<<b.first<<" "<<b.second<<"\\n";
  }
}
''')
    executable=directory/'bounds'
    command=['/usr/bin/c++','-std=c++17','-Wall','-Wextra','-I',str(root/'src/luggage_planning/include'),str(harness),'-o',str(executable)]
    compiled=subprocess.run(command,capture_output=True,text=True)
    (directory/'compile.json').write_text(json.dumps(dict(command=command,returncode=compiled.returncode,stdout=compiled.stdout,stderr=compiled.stderr)))
    assert compiled.returncode==0,compiled.stderr
    rows=[];expected=[]
    for order in (1,3,5):
        dt=.4;delta=.07
        rows.append((dt,0.,delta,0.,0.,0.,0.,order))
        expected.append((delta/dt,0.) if order==1 else
            (1.5*delta/dt,6*delta/dt**2) if order==3 else
            (15*delta/(8*dt),10*delta/(math.sqrt(3)*dt**2)))
    rng=np.random.default_rng(20261113)
    for _ in range(128):
        dt=10**rng.uniform(-2.,1.);q0,q1=rng.uniform(-3.,3.,2)
        v0,v1=rng.uniform(-.8,.8,2);a0,a1=rng.uniform(-2.,2.,2)
        rows.append((dt,q0,q1,v0,v1,a0,a1,5))
        expected.append(derivative_bounds([[q0],[q1]],[0.,dt],[[v0],[v1]],[[a0],[a1]]))
    raw=''.join(' '.join(str(v) for v in row)+'\n' for row in rows)
    (directory/'inputs.txt').write_text(raw)
    (directory/'expected.json').write_text(json.dumps(expected))
    run=subprocess.run([str(executable)],input=raw,text=True,capture_output=True)
    (directory/'actual.txt').write_text(run.stdout);(directory/'stderr.txt').write_text(run.stderr)
    assert run.returncode==0,run.stderr
    actual=np.loadtxt(run.stdout.splitlines())
    assert actual.shape==(131,2)
    assert actual==pytest.approx(np.asarray(expected),rel=2e-10,abs=2e-10)
