"""Bounded thinking is generic manifest state, never a model-name branch."""
import dataclasses
import json
from unittest.mock import Mock, patch

import pytest
from test_llama_runtime import descriptor

from python.dcm.contract import RuntimeDescriptor
from python.dcm.edge_provisioning import flight_artifact
from python.dcm.llama_runtime import LlamaCppRuntime


def command(config, extra=()):
    runtime=LlamaCppRuntime(config,binary=__file__,verify_artifact=False,extra_args=extra)
    with patch('python.dcm.llama_runtime.subprocess.Popen',return_value=Mock()) as spawn, \
            patch.object(runtime,'_start_stderr_drain'),patch.object(runtime,'_await_ready'),patch.object(runtime,'_warm'):
        runtime.start()
    return spawn.call_args.args[0],runtime.descriptor.as_record()


@pytest.mark.parametrize('budget',[None,0,1024,4096])
def test_optional_budget_is_emitted_and_recorded_once(budget):
    args,record=command(dataclasses.replace(descriptor(),reasoning_budget=budget))
    assert args.count('--reasoning-budget') == (0 if budget is None else 1)
    if budget is not None:assert args[args.index('--reasoning-budget')+1]==str(budget)
    assert record['reasoning_budget']==budget
    assert args[args.index('-c')+1]=='8192'


@pytest.mark.parametrize('bad',[-1,True,1.5,'1024',[],{}])
def test_descriptor_rejects_invalid_budget(bad):
    with pytest.raises(ValueError,match='reasoning_budget'):
        dataclasses.replace(descriptor(),reasoning_budget=bad)


@pytest.mark.parametrize('extra,expected',[
    (['--reasoning-budget','1024'],1024),
    (['--reasoning-budget=0'],0),
    (['--reasoning-budget','1024','--reasoning-budget=256'],256),
])
def test_explicit_extra_args_override_normalize_and_update_provenance(extra,expected):
    args,record=command(dataclasses.replace(descriptor(),reasoning_budget=512),extra)
    assert args.count('--reasoning-budget')==1
    assert not any(arg.startswith('--reasoning-budget=') for arg in args)
    assert args[args.index('--reasoning-budget')+1]==str(expected)
    assert record['reasoning_budget']==expected


@pytest.mark.parametrize('extra',[
    ['--reasoning-budget'],['--reasoning-budget','-1'],['--reasoning-budget=bad'],
    ['--reasoning-budget','1.5'],['--reasoning-budget','--other-flag'],
])
def test_malformed_extra_flags_fail_before_start(extra):
    with pytest.raises(ValueError,match='reasoning-budget'):
        LlamaCppRuntime(descriptor(),extra_args=extra)


def test_manifest_precedence_and_provisioning(tmp_path):
    artifact={'role':'primary','path':'/unused.gguf','family':'any','repository':'publisher/model',
              'revision':'publisher-sha','sha256':'0'*64,'quantization':'any'}
    manifest={'llama_cpp_revision':'runtime-sha','reasoning_budget':1024,'artifacts':[artifact]}
    path=tmp_path/'manifest.json'
    def load(**overrides):
        path.write_text(json.dumps(manifest))
        return RuntimeDescriptor.from_manifest(path,**overrides)
    assert load().reasoning_budget==1024
    assert flight_artifact(manifest,verify=False)['reasoning_budget']==1024
    artifact['reasoning_budget']=0
    assert load().reasoning_budget==0
    assert flight_artifact(manifest,verify=False)['reasoning_budget']==0
    assert load(reasoning_budget=512).reasoning_budget==512
    artifact['reasoning_budget']=None
    assert load().reasoning_budget is None
    artifact['reasoning_budget']=-1
    with pytest.raises(ValueError,match='reasoning_budget'):load()


def budget_runtime(budget=1024, deadline=1000):
    runtime=LlamaCppRuntime(dataclasses.replace(descriptor(),reasoning_budget=budget,deadline_ms=deadline),
                           verify_artifact=False)
    runtime.process=Mock()
    return runtime


def responder(count, requests):
    def respond(request, timeout):
        body=json.loads(request.data)
        requests.append((request.full_url,body))
        payload=({'input_tokens':count} if request.full_url.endswith('/input_tokens') else
                 {'choices':[{'message':{'content':'{}'},'finish_reason':'stop'}]})
        response=Mock()
        response.__enter__=Mock(return_value=response)
        response.__exit__=Mock(return_value=False)
        response.read.return_value=json.dumps(payload).encode()
        return response
    return respond


@pytest.mark.parametrize('base,count,effective',[(2048,4800,1024),(1024,4800,1024),(2048,5900,212)])
def test_reply_allowance_preserved_and_reasoning_fits_actual_context(base,count,effective):
    runtime=budget_runtime();requests=[]
    with patch('python.dcm.llama_runtime.urllib.request.urlopen',side_effect=responder(count,requests)):
        assert runtime._chat({'system':'system','user':'mission'},max_tokens=base,response_format={'type':'json_object'})=='{}'
    assert len(requests)==2
    assert requests[0][1]['messages']==requests[1][1]['messages']
    assert requests[0][1]['response_format']==requests[1][1]['response_format']
    assert requests[1][1]['max_tokens']==base+effective
    assert requests[1][1]['reasoning_budget_tokens']==effective
    assert runtime.last_stats['reply_tokens_reserved']==base
    assert runtime.last_stats['reasoning_budget_effective']==effective
    assert runtime.descriptor.context_length==8192


def test_context_too_small_does_not_submit_inference_or_truncate():
    runtime=budget_runtime();requests=[]
    with patch('python.dcm.llama_runtime.urllib.request.urlopen',side_effect=responder(7000,requests)), pytest.raises(RuntimeError,match='no prompt was truncated'):
        runtime._chat({'system':'system','user':'mission'},max_tokens=2048)
    assert len(requests)==1


@pytest.mark.parametrize('count',[None,True,-1,'100'])
def test_invalid_count_never_sends_inference(count):
    runtime=budget_runtime();requests=[]
    with patch('python.dcm.llama_runtime.urllib.request.urlopen',side_effect=responder(count,requests)), pytest.raises(RuntimeError,match='invalid prompt token count'):
        runtime._chat({'system':'system','user':'mission'},max_tokens=2048)
    assert len(requests)==1


def test_no_budget_preserves_existing_single_request():
    runtime=budget_runtime(None);requests=[]
    with patch('python.dcm.llama_runtime.urllib.request.urlopen',side_effect=responder(5000,requests)):
        runtime._chat({'system':'system','user':'mission'},max_tokens=2048)
    assert len(requests)==1
    assert requests[0][1]['max_tokens']==2048
    assert 'reasoning_budget_tokens' not in requests[0][1]


def test_token_counting_is_inside_hard_deadline():
    import time

    from python.dcm.contract import DeadlineExceeded
    runtime=budget_runtime(deadline=30)
    def slow(*args,**kwargs):
        time.sleep(.15)
        raise TimeoutError('delayed count')
    with patch('python.dcm.llama_runtime.urllib.request.urlopen',side_effect=slow),patch.object(runtime,'stop') as restart, pytest.raises(DeadlineExceeded):
        runtime._chat({'system':'system','user':'mission'},max_tokens=2048)
    restart.assert_called_once()


def test_late_token_count_cannot_start_inference_after_deadline():
    import time

    from python.dcm.contract import DeadlineExceeded
    runtime=budget_runtime(deadline=20);requests=[]
    regular=responder(4000,requests)
    def slow(request,timeout):
        time.sleep(.08)
        return regular(request,timeout)
    with patch('python.dcm.llama_runtime.urllib.request.urlopen',side_effect=slow),patch.object(runtime,'restart'), pytest.raises(DeadlineExceeded):
        runtime._chat({'system':'system','user':'mission'},max_tokens=2048)
    time.sleep(.12)
    assert len(requests)==1
    assert requests[0][0].endswith('/input_tokens')
