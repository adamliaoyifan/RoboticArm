from types import SimpleNamespace
import numpy as np
from luggage_planning.placement_prior.ranking import rank_payload_clearance


def test_clear_payload_precedes_deep_unknown_without_dropping_candidates():
    states = np.ones((4, 4, 8), dtype=np.uint8)
    states[:, :, 6:] = 0
    volume = SimpleNamespace(dimensions_xyz=[8,4,4], states=states.ravel(),
        active=np.ones(states.size,dtype=bool), origin_container=[0,0,0], resolution_m=.1)
    def candidate(x, identity):
        return dict(center_container=[x,.2,.2],heading_theta=0.,candidate_id=identity)
    rows = [candidate(.7,'deep'),candidate(.3,'clear-a'),candidate(.4,'clear-b')]
    original = states.copy()
    ranked = rank_payload_clearance(rows,[.1,.1,.1],volume,.05)
    assert [r['candidate_id'] for r in ranked] == ['clear-a','clear-b','deep']
    assert rows[0]['candidate_id']=='deep'
    assert np.array_equal(states, original)


def test_rotated_payload_and_margin_are_included():
    states=np.ones((3,6,6),dtype=np.uint8);states[:,4,:]=2
    volume=SimpleNamespace(dimensions_xyz=[6,6,3],states=states.ravel(),
        active=np.ones(states.size,dtype=bool),origin_container=[0,0,0],resolution_m=.1)
    wide=dict(center_container=[.3,.3,.15],heading_theta=0.,candidate_id='wide')
    tall=dict(wide,heading_theta=np.pi/2,candidate_id='rotated')
    assert rank_payload_clearance([tall,wide],[.3,.1,.1],volume,0.)==[wide,tall]


def test_existing_attachment_envelope_ranks_interior_before_wall_without_veto():
    from luggage_description.container_geometry import cuboid_from_inner_size
    from st09_contracts.attachment_geometry import freeze_geometry
    from st09_contracts.payload_envelope import EnvelopeLimits
    hull=cuboid_from_inner_size((1.,1.,1.))
    volume=SimpleNamespace(dimensions_xyz=[10,10,10],states=bytes([1])*1000,
        active=[True]*1000,origin_container=[-.5,-.5,0.],resolution_m=.1)
    wall=dict(center_container=[.39,0.,.05],heading_theta=0.,candidate_id='wall')
    interior=dict(wall,center_container=[.25,0.,.05],candidate_id='interior')
    geometry=freeze_geometry((.2,.1,.1),(0.,0.,.05),(1.,0.,0.,0.),
        EnvelopeLimits(5.,(.1,.1,.1),(0.,0.),.005))
    rows=rank_payload_clearance([wall,interior],[.2,.1,.1],volume,.02,
        hull=hull,geometry=geometry,release_gap_m=.05,vertical_margin=0.)
    assert rows==[interior,wall]
