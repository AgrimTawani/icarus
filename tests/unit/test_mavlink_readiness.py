"""Navigation startup waits for fused readiness, not merely raw GPS packets."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

SPEC = importlib.util.spec_from_file_location('mavlink_readiness', Path(__file__).resolve().parents[2] / 'scripts/simulation/check_mavlink_ready.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def message(kind, **fields):
    return SimpleNamespace(get_type=lambda: kind, **fields)


def ready_state(flags=49):
    result = MODULE.NavigationReadiness()
    for kind, fields in [('HEARTBEAT', {}), ('GPS_RAW_INT', {'fix_type': 3}), ('GLOBAL_POSITION_INT', {}), ('EKF_STATUS_REPORT', {'flags': flags})]:
        result.observe(message(kind, **fields), 10)
    return result


def test_raw_gps_and_position_do_not_establish_ekf_readiness():
    result = ready_state(flags=33)
    assert not result.status(10)['ready']
    result.observe(message('GPS_RAW_INT', fix_type=6), 11)
    assert not result.status(11)['ready']


def test_initialized_actual_fused_flags_required():
    assert ready_state(49).status(10)['ready']
    assert ready_state(41).status(10)['ready']  # relative fused navigation
    for flags in (0, 1, 17, 48, 33 | 512, 49 | 1024):
        assert not ready_state(flags).status(10)['ready']


def test_stale_ekf_cannot_be_refreshed_by_other_telemetry():
    result = ready_state()
    for kind, fields in [('HEARTBEAT', {}), ('GPS_RAW_INT', {'fix_type': 3}), ('GLOBAL_POSITION_INT', {})]:
        result.observe(message(kind, **fields), 14)
    assert not result.status(14)['ready']
    result.observe(message('EKF_STATUS_REPORT', flags=49), 14)
    assert result.status(14)['ready']


def test_lost_fix_or_ekf_health_revokes_readiness():
    result = ready_state()
    result.observe(message('GPS_RAW_INT', fix_type=1), 11)
    assert not result.status(11)['ready']
    result.observe(message('GPS_RAW_INT', fix_type=3), 11)
    result.observe(message('EKF_STATUS_REPORT', flags=33), 11)
    assert not result.status(11)['ready']


def test_recorded_boot_transition_requires_actual_gps_aiding():
    result = ready_state(39)  # observed disarmed optical-flow initialization
    assert not result.status(10)['ready']
    result.observe(message('GLOBAL_POSITION_INT', time_boot_ms=51000), 11)
    result.observe(message('EKF_STATUS_REPORT', flags=831), 11)
    assert result.status(11)['ready']
    assert result.status(11)['autopilot_boot_ms'] == 51000
