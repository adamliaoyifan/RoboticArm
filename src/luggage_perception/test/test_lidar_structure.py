import numpy as np
import pytest
from luggage_perception.lidar_structure import calibrate_structure, StructureAssociation


def fixture():
    xx,yy=np.meshgrid(np.linspace(-.4,.4,17),np.linspace(-.4,.4,17))
    points=np.column_stack((xx.ravel(),yy.ravel(),np.full(xx.size,.53003)))
    origins=np.tile((0.,0.,1.3),(len(points),1))
    record=calibrate_structure(points,normal=(0.,0.,1.),offset_m=-.53,
        geometry_hash='hull',sensor_frame='lidar',sensor_mount_id='mount',source_id='empty_observed',origins=origins)
    return points,origins,record


def test_rounding_case_labels_preserve_original_points_and_reject_obstacle():
    points,origins,record=fixture()
    association=StructureAssociation(record,geometry_hash='hull',sensor_frame='lidar',sensor_mount_id='mount')
    original=points.copy()
    assert association.classify(points,origins[0]).sum()==len(points)
    assert np.array_equal(original,points)
    elevated=points.copy();elevated[:,2]+=.002
    assert not association.classify(elevated,origins[0]).any()
    # Uncovered portions of the same mathematical plane are not calibrated.
    uncovered=points.copy();uncovered[:,0]+=2.
    assert not association.classify(uncovered,origins[0]).any()


@pytest.mark.parametrize('field,value',[('geometry_hash','other'),('sensor_frame','other'),('sensor_mount_id','other')])
def test_wrong_binding_refuses_calibration(field,value):
    _,_,record=fixture()
    kwargs=dict(geometry_hash='hull',sensor_frame='lidar',sensor_mount_id='mount');kwargs[field]=value
    with pytest.raises(ValueError):StructureAssociation(record,**kwargs)


def test_tampered_calibration_refused():
    _,_,record=fixture();record['residual_max_m']=.1
    with pytest.raises(ValueError):StructureAssociation(record,geometry_hash='hull',sensor_frame='lidar',sensor_mount_id='mount')
