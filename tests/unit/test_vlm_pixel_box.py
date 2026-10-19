"""Image-coordinate adapters preserve sensor units and explicit absence."""
import json
from unittest.mock import patch
import pytest
from PIL import Image
from python.perception.vlm import decode_pixel_box,ground_description,VlmInferenceError


def test_pixel_boxes_scale_each_source_dimension_independently():
    assert decode_pixel_box('{"bbox":[8,4,72,36]}',80,40)==([8,4,72,36],(100,100,900,900))
    assert decode_pixel_box('{"bbox":null}',80,40)==(None,None)


@pytest.mark.parametrize('payload',[
    '{}','{"bbox":[0,0,90,30]}','{"bbox":[8,4,2,9]}',
    '{"bbox":[false,0,10,10]}','{"bbox":[0,0,NaN,10]}',
    '{"bbox":[0,0,1]}','{"bbox":null,"found":false}',
])
def test_bad_boxes_never_become_clipped_successes_or_absence(payload):
    with pytest.raises(VlmInferenceError):
        decode_pixel_box(payload,80,40)


def observe(tmp_path,answers):
    image=tmp_path/'image.png';Image.new('RGB',(80,40)).save(image)
    binary=tmp_path/'llama-server';binary.touch()
    calls=[];responses=iter(answers)
    class Server:
        def __init__(self,*args):pass
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def complete(self,image,question,prefill,n_predict,**kwargs):
            calls.append((question,prefill,kwargs));return next(responses)
    artifact={'family':'other-compatible-model','grounding_format':'json_pixel_xyxy',
              'path':'model','mmproj_path':'projector'}
    with patch('python.perception.vlm.load_vlm',return_value=artifact),patch('python.perception.vlm._GroundingServer',Server):
        result=ground_description(image,'visible structure',tmp_path/'manifest',binary=binary)
    return result,calls


def test_explicit_format_uses_measured_image_size_and_constrained_json(tmp_path):
    result,calls=observe(tmp_path,['Yes','{"bbox":[8,4,72,36]}'])
    assert result['present'] is True and result['grounding_text']=='(100,100),(900,900)'
    assert result['pixel_box_xyxy']==[8,4,72,36]
    assert result['grounding_format']=='json_pixel_xyxy'
    assert len(calls)==2 and '80 wide, 40 high' in calls[1][0]
    assert 'grammar' in calls[1][2] and calls[1][1]==''


def test_negative_presence_never_requests_a_box(tmp_path):
    result,calls=observe(tmp_path,['No'])
    assert result['present'] is False and result['grounding_text'] is None
    assert len(calls)==1


def test_conflicting_presence_and_null_box_remain_unknown(tmp_path):
    with pytest.raises(VlmInferenceError) as error:
        observe(tmp_path,['Yes','{"bbox":null}'])
    assert error.value.code=='inconsistent_presence'
