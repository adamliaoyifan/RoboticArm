import gzip
import json

from luggage_gazebo.acquisition_capture import AcquisitionCapture


def test_selected_acquisition_keeps_original_bytes_and_history(tmp_path):
    capture=AcquisitionCapture()
    capture.add('/depth','Image',10,b'previous')
    capture.add('/depth','Image',20,b'exact-depth')
    capture.add('/mask','Image',20,b'exact-mask')
    result=capture.write(tmp_path/'case',20,required_topics=('/depth','/mask'))
    assert result['selected_acquisition_complete']
    assert len(result['records'])==3
    row=result['records'][1]
    assert row['acquisition_stamp_ns']==20
    assert gzip.decompress((tmp_path/'case'/row['file']).read_bytes())==b'exact-depth'
    assert json.loads((tmp_path/'case/manifest.json').read_text())==result


def test_later_image_never_substitutes_missing_selected_frame(tmp_path):
    capture=AcquisitionCapture()
    capture.add('/depth','Image',21,b'later')
    capture.add('/mask','Image',20,b'mask')
    result=capture.write(tmp_path/'case',20,required_topics=('/depth','/mask'))
    assert not result['selected_acquisition_complete']
    assert result['missing_at_acquisition']==['/depth']
    assert all(row['topic']!='/depth' for row in result['records'])


def test_memory_eviction_reports_missing_instead_of_inventing_frame(tmp_path):
    capture=AcquisitionCapture(memory_limit=5)
    capture.add('/depth','Image',20,b'1234')
    capture.add('/mask','Image',20,b'5678')
    result=capture.write(tmp_path/'case',20,required_topics=('/depth','/mask'))
    assert result['missing_at_acquisition']==['/depth']
    assert capture.bytes<=5
