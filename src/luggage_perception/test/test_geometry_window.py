import numpy as np
import pytest
from luggage_perception.geometry_window import StationaryGeometryWindow


def test_small_drift_bound_and_original_timestamps():
    window=StationaryGeometryWindow()
    a=window.observe(('box',1),10,(0.,0.,.1),0.,(.55,.40,.20))
    b=window.observe(('box',1),20,(0.,0.,.1),0.,(.55,.402,.20))
    assert b.dimensions == pytest.approx((.55,.402,.20))
    assert b.stamps_ns == (10,20)
    assert b.median_dimensions == pytest.approx((.55,.401,.20))
    c=window.observe(('box',1),30,(0.,0.,.1),0.,(.55,.399,.20))
    assert c.dimensions[1] == pytest.approx(.402)


def test_stationary_identity_time_window_and_motion_reset():
    window=StationaryGeometryWindow(capacity=3,max_span_ns=100)
    for t in (10,20,30,40): result=window.observe('box',t,(0.,0.,0.),0.,(1.,1.,1.))
    assert result.stamps_ns == (20,30,40)
    assert window.observe('new',50,(0.,0.,0.),0.,(.5,.5,.5)).stamps_ns == (50,)
    with pytest.raises(ValueError,match='STATIONARY'):
        window.observe('new',60,(0.,0.,0.),0.,(.5,.5,.5),stationary=False)
    assert not window.rows


def test_partial_or_invalid_dimensions_never_shrink_bound():
    window=StationaryGeometryWindow()
    window.observe('a',1,(0.,0.,0.),0.,(1.,1.,1.))
    with pytest.raises(ValueError):window.observe('a',2,(0.,0.,0.),0.,(1.,1.,0.))
    assert window.observe('a',3,(0.,0.,0.),0.,(.9,.9,.9)).dimensions == pytest.approx((1.,1.,.95))


def test_retains_extent_after_statistical_window_expires_and_with_rotated_box():
    window=StationaryGeometryWindow(capacity=2,max_span_ns=100)
    first=window.observe('a',1,(.3,-.2,.1),.7,(.55,.40,.2))
    assert first.dimensions == pytest.approx((.55,.40,.2))
    later=window.observe('a',1000,(.3,-.2,.1),.7,(.50,.35,.18))
    assert later.stamps_ns == (1000,)
    assert later.dimensions == pytest.approx((.55,.40,.19))


def test_late_or_conflicting_frame_cannot_reset_and_shrink_extent():
    window=StationaryGeometryWindow()
    window.observe('a',20,(0.,0.,0.),0.,(1.,1.,1.))
    with pytest.raises(ValueError,match='OUT_OF_ORDER'):
        window.observe('a',10,(0.,0.,0.),0.,(.5,.5,.5))
    with pytest.raises(ValueError,match='CONFLICTING'):
        window.observe('a',20,(0.,0.,0.),0.,(.5,.5,.5))
    result=window.observe('a',30,(0.,0.,0.),0.,(.5,.5,.5))
    assert result.dimensions == pytest.approx((1.,1.,.75))
    assert result.center[2]+result.dimensions[2]/2 == pytest.approx(.25)


def test_temporal_top_noise_preserves_current_contact_plane():
    from st09_contracts.attachment_geometry import freeze_geometry
    from st09_contracts.payload_envelope import EnvelopeLimits
    window=StationaryGeometryWindow()
    window.observe('a',10,(0.,0.,.1005),0.,(.5,.4,.2))
    result=window.observe('a',20,(0.,0.,.1),0.,(.5,.4,.2))
    assert result.center[2]+result.dimensions[2]/2 == pytest.approx(.2)
    # A parallel measured contact must not drift inside the nominal box.
    geometry=freeze_geometry(result.dimensions,(0.,0.,result.dimensions[2]/2),
        (1.,0.,0.,0.),EnvelopeLimits(0.,(0.,0.,0.),(0.,0.),0.))
    assert geometry.measured_dimensions_wdh == result.dimensions
