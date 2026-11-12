"""Retention spreads bounded native work across existing feasible proposals."""
import math
from luggage_packing.placement_solver import retain_directed_candidates


def test_each_heading_keeps_packing_preference_then_interior_then_shallow():
    rows=[]
    for heading in range(4):
        for name,x in [('deep',.45),('adjacent',.43),('interior',.01),('shallow',-.45)]:
            rows.append(dict(candidate_id=f'{heading}-{name}',heading_theta=heading*math.pi/2,
                center_container=[x,.01,.65],support_layer_z=0.,depth_m=x+.745))
    original=list(rows)
    limited=retain_directed_candidates(rows,8)
    assert [r['candidate_id'] for r in limited]==[f'{h}-deep' for h in range(4)]+[f'{h}-interior' for h in range(4)]
    extended=retain_directed_candidates(rows,12)
    assert [r['candidate_id'] for r in extended[8:]]==[f'{h}-shallow' for h in range(4)]
    assert rows==original and all(any(row is source for source in rows) for row in extended)


def test_legacy_rows_and_small_budget_keep_preferred_order_without_inventing_heading():
    rows=[dict(candidate_id=str(i),heading_theta=i*math.pi/2,depth_m=1.) for i in range(4)]
    assert retain_directed_candidates(rows,2)==rows[:2]
    assert retain_directed_candidates(rows,20)==rows
