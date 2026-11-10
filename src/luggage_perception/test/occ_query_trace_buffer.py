"""Bounded snapshot T1 storage without allocating containers per request.

Materialize dictionaries only after scoring. Retaining a new event container
per call changed GC allocation state and can perturb full-service timings.
"""
from array import array


class SnapshotTraceBuffer:
    def __init__(self,capacity=100_000):
        self.capacity=capacity
        self.started=array('Q',[0])*capacity
        self.elapsed=array('Q',[0])*capacity
        self.request_ids=[None]*capacity
        self.size=0
        self.dropped=0

    def record(self,request_id,started_ns,elapsed_ns):
        slot=self.size
        if slot==self.capacity:
            self.dropped+=1
            return
        self.started[slot]=started_ns
        self.elapsed[slot]=elapsed_ns
        self.request_ids[slot]=request_id
        self.size=slot+1

    def rows(self):
        for index in range(self.size):
            yield dict(stage='snapshot',request_id=self.request_ids[index],
                started_ns=self.started[index],elapsed_ns=self.elapsed[index])
