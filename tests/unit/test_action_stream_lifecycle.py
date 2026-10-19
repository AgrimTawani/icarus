"""Action stream loss must not become an unobserved active flight command."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from test_airsim_vision_mission import _module

pb = _module().action_pb2


def fixture(stream, statuses):
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.session_id = 'session'
    client.episode = SimpleNamespace(record=Mock())
    client.context = lambda name: pb.CommandContext(session_id='session')
    client.action_api = SimpleNamespace(
        WatchActionStatus=Mock(side_effect=stream),
        GetActionStatus=Mock(side_effect=statuses),
        CancelAction=Mock(return_value=pb.ActionReceipt(action_id='original', disposition=pb.ACTION_STATE_CANCELLED)))
    receipt = pb.ActionReceipt(action_id='original', disposition=pb.ACTION_STATE_EXECUTING)
    return client,receipt


def status(state):
    return pb.ActionStatus(action_id='original',type=pb.ACTION_TYPE_RETURN_HOME,state=state,message='state')


def test_execution_budget_has_mode_and_delivery_grace():
    client,receipt=fixture(lambda *a,**k:iter([status(pb.ACTION_STATE_TIMED_OUT)]),[])
    with pytest.raises(RuntimeError,match='Action failed'):
        client.wait_action(receipt,120)
    assert client.action_api.WatchActionStatus.call_args.kwargs['timeout']==135
    client.action_api.CancelAction.assert_not_called()
    assert client._unsettled_action is None


def test_lost_stream_reconciles_completed_command_without_cancelling():
    client,receipt=fixture(lambda *a,**k:iter([]),[status(pb.ACTION_STATE_SUCCEEDED)])
    assert client.wait_action(receipt,120).state==pb.ACTION_STATE_SUCCEEDED
    client.action_api.CancelAction.assert_not_called()


def test_dropped_active_stream_cancel_and_confirm_original_before_failure():
    def broken(*a,**k):
        yield status(pb.ACTION_STATE_EXECUTING)
        raise RuntimeError('dropped transport')
    client,receipt=fixture(broken,[status(pb.ACTION_STATE_EXECUTING),status(pb.ACTION_STATE_CANCELLED)])
    ticks=[]
    with pytest.raises(RuntimeError,match='dropped transport'):
        client.wait_action(receipt,120,on_status=ticks.append)
    assert client.action_api.CancelAction.call_args.args[0].action_id=='original'
    assert client.action_api.GetActionStatus.call_count==2
    assert ticks[-1].state==pb.ACTION_STATE_CANCELLED
    assert client._unsettled_action is None


def test_unavailable_reconciliation_keeps_original_blocking_new_dispatch():
    client,receipt=fixture(lambda *a,**k:iter([]),[RuntimeError('offline'),RuntimeError('offline')])
    with pytest.raises(RuntimeError,match='not confirmed settled'):
        client.wait_action(receipt,120)
    assert client._unsettled_action is receipt
    with pytest.raises(RuntimeError,match='no replacement flight command'):
        client.ensure_action_settled()
    client.action_api.CancelAction.assert_not_called()


def test_fatal_observer_error_still_settles_action():
    client,receipt=fixture(lambda *a,**k:iter([status(pb.ACTION_STATE_EXECUTING)]),
                           [status(pb.ACTION_STATE_EXECUTING),status(pb.ACTION_STATE_CANCELLED)])
    def observer(_):raise ValueError('observer failed')
    with pytest.raises(RuntimeError,match='observer failed'):
        client.wait_action(receipt,10,on_status=observer,observer_errors_fatal=True)
    client.action_api.CancelAction.assert_called_once()
    assert client._unsettled_action is None


def test_replacement_dispatch_blocked_when_original_cannot_be_reconciled():
    from python.dcm.agent_tools import ToolExecutor
    client,receipt=fixture(lambda *a,**k:iter([]),[RuntimeError('offline')])
    client._unsettled_action=receipt
    client.action_api.ReturnHome=Mock()
    result=ToolExecutor(client).execute('return_home',{})
    assert result['status']=='FAILED' and 'not confirmed settled' in result['message']
    client.action_api.ReturnHome.assert_not_called()


def test_close_does_not_claim_release_for_unresolved_action(capsys):
    import threading
    client,receipt=fixture(lambda *a,**k:iter([]),[RuntimeError('offline')])
    client._unsettled_action=receipt
    client.stop_renewal=threading.Event()
    client.renewal_thread=None
    client.lease_id='lease'
    client.episode.seal=Mock()
    client.episode_outcome='failed'
    client.episode_score={}
    client.authority_api=SimpleNamespace(ReleaseControl=Mock())
    client.session_api=SimpleNamespace(CloseSession=Mock())
    client.channel=SimpleNamespace(close=Mock())
    client.close()
    assert client.stop_renewal.is_set()
    client.authority_api.ReleaseControl.assert_not_called()
    client.session_api.CloseSession.assert_not_called()
    assert client.episode_outcome=='unresolved_action'
    assert 'lease-expiry failsafe' in capsys.readouterr().out
