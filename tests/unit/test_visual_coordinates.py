"""Model pixels and sensor pixels retain an explicit reversible transform."""
import base64,io
import pytest
from PIL import Image
from python.perception.visual_coordinates import prepare_grounding_image,source_pixel_box,grounding_image_policy
from python.perception.vlm import decode_pixel_box,_GroundingServer

POLICY={'grounding_image':{'alignment':28,'minimum_tokens':1024,'maximum_tokens':1536}}

@pytest.mark.parametrize('size',[(848,480),(1920,1080),(1280,720),(640,480)])
def test_prepared_dimensions_match_the_declared_backend_limits(tmp_path,size):
    p=tmp_path/'sensor.png';Image.new('RGB',size).save(p)
    encoded,t=prepare_grounding_image(p,POLICY)
    w,h=t['model_width'],t['model_height']
    assert w%28==h%28==0
    assert 1024<=w*h/(28*28)<=1536
    assert Image.open(io.BytesIO(base64.b64decode(encoded))).size==(w,h)
    # Normalized coordinates are invariant under the resize, not reinterpreted
    # as original pixels or silently clamped at the camera edge.
    box=[w*.1,h*.2,w*.8,h*.9]
    assert decode_pixel_box(__import__('json').dumps({'bbox':box}),w,h)[1]==(100,200,800,900)
    assert source_pixel_box(box,t)==pytest.approx([size[0]*.1,size[1]*.2,size[0]*.8,size[1]*.9])
    encoded2,t2=prepare_grounding_image(p,POLICY)
    assert (encoded,t)==(encoded2,t2)


def test_backend_receives_matching_limits_and_enough_context():
    server=_GroundingServer('not-launched',POLICY|{'path':'model','mmproj_path':'projector'},0,100)
    command=server.command
    assert command[command.index('--image-min-tokens')+1]=='1024'
    assert command[command.index('--image-max-tokens')+1]=='1536'
    assert int(command[command.index('-c')+1])>=1536+512


def test_legacy_input_is_not_resized_without_a_declared_transform(tmp_path):
    p=tmp_path/'sensor.png';Image.new('RGB',(80,40)).save(p)
    data,transform=prepare_grounding_image(p,{})
    assert transform is None and base64.b64decode(data)==p.read_bytes()


@pytest.mark.parametrize('policy',[{}, {'alignment':0,'minimum_tokens':1,'maximum_tokens':2},
 {'alignment':28,'minimum_tokens':1536,'maximum_tokens':1024},
 {'alignment':True,'minimum_tokens':1024,'maximum_tokens':1536}])
def test_invalid_resolution_policies_do_not_guess_a_transform(policy):
    with pytest.raises(ValueError):grounding_image_policy({'grounding_image':policy})
