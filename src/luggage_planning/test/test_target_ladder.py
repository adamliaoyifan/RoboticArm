from types import SimpleNamespace
from luggage_planning.target_ladder import prepare_target_ladder


def targets():
    return [SimpleNamespace(position=(1.,2.,3.),quaternion_xyzw=(1.,0.,0.,0.)),
            SimpleNamespace(position=(1.1,2.,3.),quaternion_xyzw=(0.,1.,0.,0.))]


def test_missing_approach_tries_allowed_alternative_without_deleting_targets():
    original=targets();calls=[]
    def prepare(target):
        calls.append(target)
        return {'portal','slot_upper'} if target is original[0] else {'portal','slot_upper','slot_approach'}
    result,ordered,attempts=prepare_target_ladder(original,prepare,lambda x:x,
        required={'portal','slot_upper','slot_approach'},expired=lambda:False)
    assert calls==original
    assert ordered==(original[1],original[0])
    assert attempts[0]['missing_regions']==['slot_approach']
    assert attempts[1]['missing_regions']==[]


def test_complete_first_ladder_does_not_spend_alternative_preparation():
    original=targets();calls=[]
    result,ordered,attempts=prepare_target_ladder(original,
        lambda target:(calls.append(target) or {'slot_approach'}),lambda x:x,
        required={'slot_approach'},expired=lambda:False)
    assert calls==[original[0]] and ordered==tuple(original)


def test_deadline_stops_alternative_and_preserves_original_refusal():
    original=targets();calls=[]
    result,ordered,attempts=prepare_target_ladder(original,
        lambda target:(calls.append(target) or set()),lambda x:x,
        required={'slot_approach'},expired=lambda:bool(calls))
    assert calls==[original[0]] and ordered==tuple(original)
    assert result==set()


def test_all_missing_returns_first_ladder_with_all_rejections():
    original=targets()
    result,ordered,attempts=prepare_target_ladder(original,
        lambda target: {'portal'} if target is original[0] else {'slot_upper'},lambda x:x,
        required={'slot_approach'},expired=lambda:False)
    assert result=={'portal'} and ordered==tuple(original)
    assert len(attempts)==2
