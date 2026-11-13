"""Continuous target validity survives search-lattice parity changes."""
import math
import pytest
from test_placement_query import observed_map
from luggage_packing.selected_pose_admission import SelectedPoseAdmission
from luggage_packing.placement_volume_query import KnownFreeVolume


@pytest.mark.parametrize('yaw', [0., math.pi/2, math.pi, -math.pi/2])
@pytest.mark.parametrize('stack', [False, True])
def test_dimension_parity_change_keeps_actual_pose(yaw, stack):
    surface, evidence = observed_map()
    base = .1 if stack else 0.
    if stack:
        surface['height'] = [[base]*surface['ny'] for _ in range(surface['nx'])]
        # Full known-free XYZ above a measured support face.
    target = (.007, -.003, .53+base+.05)
    for size in ((.12,.099,.1),(.12,.101108,.1)):
        gate = SelectedPoseAdmission(surface, size, volume_verifier=KnownFreeVolume(evidence))
        assert gate(gate.candidate(target,yaw)) is None


@pytest.mark.parametrize('defect', ['wall','unknown','occupied','support'])
def test_small_change_is_not_a_blanket_deadband(defect):
    surface, evidence = observed_map()
    size, target = (.12,.10,.1), (0.,0.,.58)
    if defect == 'wall': target = (.18,0.,.58)
    if defect == 'unknown': evidence['states'] = [0]*len(evidence['states'])
    if defect == 'occupied': evidence['states'] = [2]*len(evidence['states'])
    if defect == 'support': target = (0.,0.,.68)
    gate = SelectedPoseAdmission(surface,size,volume_verifier=KnownFreeVolume(evidence))
    assert gate(gate.candidate(target,0.)) is not None


@pytest.mark.parametrize('defect',['wall','occupied','unknown'])
def test_two_mm_growth_crossing_real_boundary_refuses(defect):
    surface,evidence=observed_map()
    target=(.14,0.,.58) if defect=='wall' else (0.,0.,.58)
    if defect!='wall':
        # The old +X face is exactly at x=.06, a native cell boundary.
        ix=13;iy=10
        if defect=='occupied':surface['height'][ix][iy]=.1
        else:
            for iz in range(evidence['nz']):
                evidence['states'][(iz*surface['ny']+iy)*surface['nx']+ix]=0
    for size,expected in (((.12,.10,.1),True),((.124,.10,.1),False)):
        gate=SelectedPoseAdmission(surface,size,volume_verifier=KnownFreeVolume(evidence),params={'hull_margin':0.})
        assert (gate(gate.candidate(target,0.)) is None)==expected
