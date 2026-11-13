"""One frozen geometry owns grid parity and the shared hard placement gates."""
from types import SimpleNamespace
import pytest
from test_joint_operation_observer import configuration,snapshot
from luggage_planning.attachment_geometry_policy import load_attachment_geometry_policy
from luggage_planning.joint_operation.prepare_node import JointOperationPrepareNode
from luggage_planning.placement_prior.requests import admit_placement_targets
from luggage_planning.placement_prior.task_slots import observed_placement_slots


def test_frozen_dimensions_regenerate_slots_instead_of_reusing_old_grid_parity(tmp_path):
    cfg,_,_=configuration(tmp_path,size=(.8,.8,1.),floor=.53,resolution=.02)
    captured=snapshot(cfg,[1]*__import__('math').prod(cfg.dimensions_xyz))
    limits=load_attachment_geometry_policy()
    old=(.20,.13,.10);fresh=(.21,.11,.12)
    old_slots,_=observed_placement_slots(captured.surface,captured.volume,old,'floor',
        limits=limits,margin=.02,vertical_margin=0.)
    assert old_slots and not all(admit_placement_targets(captured.surface,fresh,old_slots,volume=captured.volume))
    records=[]
    owner=SimpleNamespace(limits=limits,margin=.02,vertical_margin=0.,_record=lambda *args:records.append(args))
    request=SimpleNamespace(placement_task_mode='floor',slots=[],detection=SimpleNamespace(
        width=fresh[0],depth=fresh[1],height=fresh[2]))
    selected=JointOperationPrepareNode._placements(owner,request,captured.surface,captured.volume)
    assert selected and all(admit_placement_targets(captured.surface,fresh,
        tuple(p.container_from_box for p in selected),volume=captured.volume))
    assert records and all(p.container_from_box.translation[2]-fresh[2]/2==pytest.approx(.53) for p in selected)


@pytest.mark.parametrize('mode,slots',[('unknown',[]),('floor',[object()])])
def test_ambiguous_intent_is_refused(mode,slots):
    request=SimpleNamespace(placement_task_mode=mode,slots=slots,
        detection=SimpleNamespace(width=.2,depth=.1,height=.1))
    with pytest.raises(ValueError,match='AMBIGUOUS_OR_UNKNOWN_PLACEMENT_TASK'):
        JointOperationPrepareNode._placements(SimpleNamespace(),request,None,None)
