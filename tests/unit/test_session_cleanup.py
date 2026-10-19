"""Cleanup failures must not skip the session's remaining owned resources."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from python.dcm import fly_cli


@pytest.mark.parametrize('raises', [False, True])
def test_failed_exit_recovery_cannot_seal_a_successful_episode(raises):
    client, runtime = Mock(episode=None), Mock()
    result = {'mission': 'test', 'status': 'completed', 'message': 'done',
              'decisions': 1, 'executed': []}
    recovery = Mock(side_effect=RuntimeError('recovery transport failed')) if raises else Mock(
        return_value={'status': 'FAILED', 'message': 'landing not confirmed'})
    with (
        patch('sys.argv', ['dcm-fly', '--mission', 'test']),
        patch.object(fly_cli, '_load_mission_client', return_value=lambda *a: client),
        patch.object(fly_cli, 'build_runtime', return_value=(runtime, SimpleNamespace(model_id='test'))),
        patch.object(fly_cli, 'fly_mission', return_value=result),
        patch.object(fly_cli, 'safe_exit_recovery', recovery),
        patch.object(fly_cli, 'OperatorConsole'),
    ):
        if raises:
            with pytest.raises(RuntimeError, match='recovery transport failed'):
                fly_cli.main()
        else:
            assert fly_cli.main() == 2
    assert client.episode_outcome == 'incomplete'
    assert client.episode_score['success'] is False
    assert client.episode_score['error']
    runtime.stop.assert_called_once()
    client.close.assert_called_once()


@pytest.mark.parametrize('failure', [KeyboardInterrupt(), RuntimeError('startup failed')])
def test_interrupted_or_failed_first_mission_is_never_sealed_completed(failure):
    client, runtime = Mock(episode=None), Mock()
    runtime.start.side_effect = failure
    descriptor = SimpleNamespace(model_id='test')
    with (
        patch('sys.argv', ['dcm-fly', '--mission', 'test']),
        patch.object(fly_cli, '_load_mission_client', return_value=lambda *a: client),
        patch.object(fly_cli, 'build_runtime', return_value=(runtime, descriptor)),
        patch.object(fly_cli, 'safe_exit_recovery', return_value={'status': 'SUCCEEDED'}),
        patch.object(fly_cli, 'OperatorConsole'),
    ):
        if isinstance(failure, KeyboardInterrupt):
            assert fly_cli.main() == 130
        else:
            with pytest.raises(RuntimeError, match='startup failed'):
                fly_cli.main()
    assert client.episode_outcome == 'incomplete'
    assert client.episode_score['success'] is False
    assert client.episode_score['error']
    client.close.assert_called_once()


def test_console_model_cleanup_failure_still_closes_client():
    client = Mock(episode=None)
    runtime = Mock()
    runtime.stop.side_effect = RuntimeError("model failure")
    result = {
        "mission": "test",
        "status": "completed",
        "message": "done",
        "decisions": 1,
        "executed": [],
    }
    descriptor = SimpleNamespace(model_id="test")
    with (
        patch("sys.argv", ["dcm-fly", "--mission", "test"]),
        patch.object(fly_cli, "_load_mission_client", return_value=lambda *a: client),
        patch.object(fly_cli, "build_runtime", return_value=(runtime, descriptor)),
        patch.object(fly_cli, "fly_mission", return_value=result),
        patch.object(
            fly_cli, "safe_exit_recovery", return_value={"status": "SUCCEEDED"}
        ),
        patch.object(fly_cli, "OperatorConsole"),
        pytest.raises(RuntimeError, match="model failure"),
    ):
        fly_cli.main()
    client.close.assert_called_once()


@pytest.mark.parametrize("settled", [True, False])
def test_detector_and_episode_failure_still_stop_renewal_close_channel_and_respect_unsettled_action(
    settled,
):
    from test_localize_target import _module

    Client = _module().MissionClient
    client = Client.__new__(Client)
    client.ensure_action_settled = Mock(
        side_effect=None if settled else RuntimeError("still executing")
    )
    client.edge_detector_runtime = SimpleNamespace(
        close=Mock(side_effect=RuntimeError("detector failure"))
    )
    client.stop_renewal = Mock()
    client.renewal_thread = Mock()
    client.episode = SimpleNamespace(seal=Mock(side_effect=RuntimeError("disk full")))
    client.episode_outcome = "passed"
    client.episode_score = {}
    client.lease_id = "lease"
    client.session_id = "session"
    client.authority_api = Mock()
    client.session_api = Mock()
    client.channel = Mock()
    with pytest.raises(RuntimeError, match="detector failure.*disk full"):
        client.close()
    client.stop_renewal.set.assert_called_once()
    client.channel.close.assert_called_once()
    assert client.authority_api.ReleaseControl.called is settled
    assert client.session_api.CloseSession.called is settled
