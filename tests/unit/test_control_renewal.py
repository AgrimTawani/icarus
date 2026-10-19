"""Transient transport failures must not silently disable a valid heartbeat."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

import grpc

from test_localize_target import _module


class Unavailable(grpc.RpcError):
    def code(self):
        return grpc.StatusCode.UNAVAILABLE


def client_with_replies(replies, expires):
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.session_id = 'owner'
    client.lease_id = 'lease'
    client.lease_expires_at_unix_ms = expires
    client.stop_renewal = Mock()
    client.stop_renewal.wait.side_effect = [False, False, True]
    client.authority_api = SimpleNamespace(RenewControl=Mock(side_effect=replies))
    return client


def test_transient_failure_retries_within_confirmed_lease():
    reply = SimpleNamespace(granted=True, authority=SimpleNamespace(expires_at_unix_ms=20000))
    client = client_with_replies([Unavailable(), reply], 10000)
    with patch('time.time', return_value=1):
        client._renew()
    assert client.authority_api.RenewControl.call_count == 2
    client.stop_renewal.set.assert_not_called()
    assert client.renewal_error is None
    assert client.lease_expires_at_unix_ms == 20000
    assert client.stop_renewal.wait.call_args_list[1].args == (0.25,)


def test_denial_stops_heartbeat_without_reacquiring_control():
    client = client_with_replies([SimpleNamespace(granted=False, message='preempted')], 10000)
    client.stop_renewal.wait.side_effect = [False, True]
    client._renew()
    client.stop_renewal.set.assert_called_once()
    assert client.renewal_error == 'preempted'


def test_transport_failure_past_expiry_does_not_retry():
    client = client_with_replies([Unavailable()], 1000)
    client.stop_renewal.wait.side_effect = [False, True]
    with patch('time.time', return_value=2):
        client._renew()
    client.stop_renewal.set.assert_called_once()
    assert client.authority_api.RenewControl.call_count == 1
