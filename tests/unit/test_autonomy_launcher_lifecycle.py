"""The compatibility edge launch owns one API, without another GPU observer."""
import json
import runpy
import signal
import subprocess
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

ROOT = Path(__file__).resolve().parents[2]


def test_simulator_exit_stops_owned_api_and_edge_flag_does_not_duplicate_detector():
    session = {'status': 'ready', 'launcher_pid': 123, 'partition': 'unit-test',
               'scenario': 'edge_people_building', 'gz_ip': '127.0.0.1'}
    session_path = ROOT / 'logs/simulation/active_session.json'
    read_text, is_file = Path.read_text, Path.is_file
    server = Mock(pid=456)
    server.wait.side_effect = [subprocess.TimeoutExpired('server', 1), 0]
    probe = Mock()
    probe.__enter__ = Mock(return_value=Mock(connect_ex=Mock(return_value=1)))
    probe.__exit__ = Mock(return_value=False)
    def read(path, *args, **kwargs):
        return json.dumps(session) if path == session_path else read_text(path, *args, **kwargs)
    def exists(path):
        if path == session_path or path == ROOT / 'build/phase8/icarus-drone-api':
            return True
        return is_file(path)
    with (
        patch('sys.argv', ['start-autonomy', '--profile', 'edge-vision']),
        patch.object(Path, 'read_text', read), patch.object(Path, 'is_file', exists),
        patch('socket.socket', return_value=probe), patch('subprocess.run'),
        patch('subprocess.Popen', return_value=server) as spawn,
        patch('os.kill', side_effect=[None, ProcessLookupError()]),
        patch('os.killpg') as stop, patch('os.chdir'),
        patch.dict('os.environ'), pytest.raises(SystemExit) as result,
    ):
        runpy.run_path(str(ROOT / 'scripts/start-autonomy'), run_name='__main__')
    assert result.value.code == 0
    assert spawn.call_count == 1
    assert spawn.call_args.args[0][0].endswith('icarus-drone-api')
    stop.assert_called_once_with(456, signal.SIGINT)
