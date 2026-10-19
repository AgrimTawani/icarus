"""Completed server refusals are not inference deadlines or mission refusals."""
import io
import json
import urllib.error
from unittest.mock import Mock, patch

import pytest
from test_llama_runtime import descriptor

from python.dcm.contract import DeadlineExceeded
from python.dcm.llama_runtime import LlamaCppRuntime


def runtime():
    result = LlamaCppRuntime(descriptor(), binary=__file__, verify_artifact=False)
    result.process = object()
    result.restart = Mock()
    result.stop = Mock()
    return result


@pytest.mark.parametrize('message', ['request exceeds context size', 'invalid grammar definition'])
def test_http400_preserves_bounded_server_reason_without_restart(message):
    model = runtime()
    error = urllib.error.HTTPError('http://localhost', 400, 'Bad Request', {},
                                   io.BytesIO(json.dumps({'error': {'message': message}}).encode()))
    with patch('urllib.request.urlopen', side_effect=error), pytest.raises(RuntimeError) as raised:
        model._chat({'system': 'system', 'user': 'request'})
    assert 'HTTP 400' in str(raised.value)
    assert message in str(raised.value)
    assert not isinstance(raised.value, DeadlineExceeded)
    model.restart.assert_not_called()


def test_connection_refused_is_not_a_deadline():
    model = runtime()
    with patch('urllib.request.urlopen', side_effect=urllib.error.URLError(ConnectionRefusedError('connection refused'))), pytest.raises(RuntimeError, match='connection failed') as raised:
        model._chat({'system': 'system', 'user': 'request'})
    assert not isinstance(raised.value, DeadlineExceeded)
    model.restart.assert_not_called()


@pytest.mark.parametrize('failure', [TimeoutError('timed out'), urllib.error.URLError(TimeoutError('timed out'))])
def test_actual_socket_timeout_stops_owned_runtime(failure):
    model = runtime()
    with patch('urllib.request.urlopen', side_effect=failure), pytest.raises(DeadlineExceeded):
        model._chat({'system': 'system', 'user': 'request'})
    model.stop.assert_called_once_with()


def test_large_http_error_is_bounded_and_not_echoed_whole():
    model = runtime()
    error = urllib.error.HTTPError('http://localhost', 500, 'Server Error', {}, io.BytesIO(b'x' * 10000))
    with patch('urllib.request.urlopen', side_effect=error), pytest.raises(RuntimeError) as raised:
        model._chat({'system': 'system', 'user': 'request'})
    assert len(str(raised.value)) < 600
    model.restart.assert_not_called()


def test_truncated_output_is_transport_failure_with_usage_not_fake_semantics():
    model = runtime()
    response = Mock()
    response.__enter__ = Mock(return_value=response)
    response.__exit__ = Mock(return_value=False)
    response.read.return_value = json.dumps({'choices': [{'finish_reason': 'length', 'message': {'content': '{"objectives":['}}],
                                            'usage': {'prompt_tokens': 3000, 'completion_tokens': 2048}}).encode()
    with patch('urllib.request.urlopen', return_value=response), pytest.raises(RuntimeError, match='truncated at the token limit') as raised:
        model._chat({'system': 'system', 'user': 'request'}, max_tokens=2048)
    assert 'prompt tokens=3000' in str(raised.value)
    assert model.last_stats['tokens_predicted'] == 2048
    model.restart.assert_not_called()


def test_native_output_parse_error_is_repairable_but_unrelated_http500_is_not():
    from python.dcm.llama_runtime import IncompleteModelResponse

    model = runtime()
    for message, repairable in [
        ('The model produced output that does not match the expected peg-native format.', True),
        ('GPU memory allocation failed', False),
    ]:
        error = urllib.error.HTTPError('http://localhost', 500, 'Server Error', {},
                                      io.BytesIO(json.dumps({'error': {'message': message}}).encode()))
        with patch('urllib.request.urlopen', side_effect=error), pytest.raises(RuntimeError) as raised:
            model.decide([{'role': 'user', 'content': 'land'}], [])
        assert isinstance(raised.value, IncompleteModelResponse) is repairable
        model.restart.assert_not_called()
