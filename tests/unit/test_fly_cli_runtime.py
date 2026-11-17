"""dcm-fly picks its mission runtime: hosted model by default, local llama on request."""
import os
from unittest.mock import Mock, patch

import pytest

from python.dcm import fly_cli
from python.model_runtime.cloud import CloudRuntime

ENVIRONMENT = {
    "ICARUS_LLM_BASE_URL": "https://api.sarvam.ai/v2",
    "ICARUS_LLM_API_KEY": "sk-secret",
    "ICARUS_LLM_MODEL": "gemma4",
}


@pytest.fixture
def cloud_environment(monkeypatch):
    for name in list(ENVIRONMENT) + [n for n in os.environ if n.startswith("ICARUS_VLM_")]:
        monkeypatch.delenv(name, raising=False)
    for name, value in ENVIRONMENT.items():
        monkeypatch.setenv(name, value)


def test_default_runtime_is_cloud():
    assert fly_cli.parse_args([]).runtime == "cloud"


@pytest.mark.parametrize("name,expected", [("local", "local"), ("cloud", "cloud"), ("llama", "local")])
def test_runtime_choices_and_llama_alias(name, expected):
    assert fly_cli.parse_args(["--runtime", name]).runtime == expected


def test_runtime_is_documented_in_help(capsys):
    with pytest.raises(SystemExit):
        fly_cli.parse_args(["--help"])
    assert "--runtime" in capsys.readouterr().out


def test_local_runtime_is_unchanged(monkeypatch):
    for name in ENVIRONMENT:
        monkeypatch.delenv(name, raising=False)
    runtime, descriptor = fly_cli.build_runtime(fly_cli.parse_args(["--runtime", "local"]))
    assert type(runtime).__name__ == "LlamaCppRuntime"
    assert descriptor is runtime.descriptor
    assert descriptor.model_id
    assert descriptor.contract_version == "direct-agent-v1"


def test_local_runtime_applies_timeout_and_context_overrides():
    args = fly_cli.parse_args(["--runtime", "local", "--timeout-ms", "12000", "--context-length", "4096"])
    _, descriptor = fly_cli.build_runtime(args)
    assert (descriptor.deadline_ms, descriptor.context_length) == (12000, 4096)


def test_cloud_runtime_uses_settings_and_overrides(cloud_environment):
    args = fly_cli.parse_args(["--timeout-ms", "9000", "--context-length", "8192"])
    runtime, record = fly_cli.build_runtime(args)
    assert isinstance(runtime, CloudRuntime)
    assert record == runtime.settings.as_record()
    assert (record["deadline_ms"], record["context_length"]) == (9000, 8192)
    assert record["provider_host"] == "api.sarvam.ai"


def test_cloud_without_key_exits_naming_variable(cloud_environment, monkeypatch, capsys):
    monkeypatch.delenv("ICARUS_LLM_API_KEY")
    with pytest.raises(SystemExit) as stop:
        fly_cli.build_runtime(fly_cli.parse_args([]))
    assert stop.value.code not in (0, None)
    assert "ICARUS_LLM_API_KEY" in capsys.readouterr().err


def _main(argv, runtime, record, client):
    with (
        patch("sys.argv", ["dcm-fly", *argv]),
        patch.object(fly_cli, "_load_mission_client", return_value=client),
        patch.object(fly_cli, "build_runtime", return_value=(runtime, record)),
        patch.object(fly_cli, "prepare_runtime"),
        patch.object(fly_cli, "fly_mission", return_value={"status": "completed", "message": "done",
                                                            "decisions": 1, "executed": [], "mission": "m"}),
        patch.object(fly_cli, "safe_exit_recovery", return_value={"status": "SUCCEEDED"}),
        patch.object(fly_cli, "OperatorConsole"),
    ):
        return fly_cli.main()


def test_cloud_banner_shows_host_and_model_not_key(cloud_environment, capsys):
    _, record = fly_cli.build_runtime(fly_cli.parse_args([]))
    runtime, client = Mock(), Mock(episode=None)
    assert _main(["--mission", "m"], runtime, record, Mock(return_value=client)) == 0
    shown = capsys.readouterr()
    assert "api.sarvam.ai" in shown.out and "gemma4" in shown.out
    assert "sk-secret" not in shown.out + shown.err
    assert "direct-agent-v1" in shown.out


@pytest.mark.parametrize("failure", [
    RuntimeError("cannot reach cloud provider api.sarvam.ai (timed out); use --runtime local to fly offline"),
    RuntimeError("cloud provider api.sarvam.ai rejected the API key (HTTP 401); use --runtime local to fly offline"),
])
def test_unreachable_provider_exits_before_touching_the_drone(cloud_environment, capsys, failure):
    _, record = fly_cli.build_runtime(fly_cli.parse_args([]))
    runtime, client = Mock(), Mock(episode=None)
    runtime.start.side_effect = failure
    make_client = Mock(return_value=client)
    with pytest.raises(SystemExit) as stop:
        _main(["--mission", "m"], runtime, record, make_client)
    assert stop.value.code not in (0, None)
    error = capsys.readouterr().err
    assert str(failure) in error and "--runtime local" in error and "sk-secret" not in error
    make_client.assert_not_called()
    client.connect.assert_not_called()
    client.acquire.assert_not_called()


def test_unreachable_provider_is_caught_through_the_real_runtime(cloud_environment, capsys):
    """Patch the network call, not the runtime, so the stderr text is the real one."""
    import urllib.error

    argv = ["--mission", "m"]
    make_client = Mock()
    with (
        patch("sys.argv", ["dcm-fly", *argv]),
        patch.object(fly_cli, "_load_mission_client", return_value=make_client),
        patch("python.model_runtime.cloud.urllib.request.urlopen",
              side_effect=urllib.error.URLError("connection refused")),
        pytest.raises(SystemExit) as stop,
    ):
        fly_cli.main()
    assert stop.value.code not in (0, None)
    error = capsys.readouterr().err
    assert "api.sarvam.ai" in error and "--runtime local" in error and "sk-secret" not in error
    make_client.assert_not_called()


def test_main_without_key_exits_before_the_drone(cloud_environment, monkeypatch, capsys):
    monkeypatch.delenv("ICARUS_LLM_API_KEY")
    make_client = Mock()
    with (
        patch("sys.argv", ["dcm-fly", "--mission", "m"]),
        patch.object(fly_cli, "_load_mission_client", return_value=make_client),
        pytest.raises(SystemExit),
    ):
        fly_cli.main()
    assert "ICARUS_LLM_API_KEY" in capsys.readouterr().err
    make_client.assert_not_called()


def test_local_flight_does_not_start_the_runtime_before_connecting(capsys):
    runtime, client = Mock(), Mock(episode=None)
    descriptor = Mock(model_id="local-model")
    assert _main(["--runtime", "local", "--mission", "m"], runtime, descriptor, Mock(return_value=client)) == 0
    assert "Model: local-model; architecture: direct-agent-v1; mode: autonomous" in capsys.readouterr().out
    runtime.start.assert_not_called()  # prepare_runtime owns the local start, after the camera detector


def test_operator_evaluation_defaults_to_cloud_and_needs_the_key_before_any_drone(tmp_path):
    import subprocess
    import sys
    from pathlib import Path

    root = Path(__file__).resolve().parents[2]
    environment = {k: v for k, v in os.environ.items() if not k.startswith("ICARUS_")}
    done = subprocess.run(
        [sys.executable, str(root / "scripts/autonomy/evaluate_operator_missions.py"), "--output", str(tmp_path)],
        capture_output=True, text=True, env=environment, timeout=60)
    assert done.returncode != 0
    assert "ICARUS_LLM_BASE_URL" in done.stderr
