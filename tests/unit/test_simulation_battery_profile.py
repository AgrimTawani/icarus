import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts/simulation' / (name + '.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


@pytest.mark.parametrize('capacity,voltage', [(10,25.2),(4.5,16.8),(20,50.4)])
def test_profile_uses_physical_pack_units(tmp_path, capacity, voltage):
    params = module('battery_profile').write_battery_parameters(
        tmp_path / 'pack.parm', {'capacity_ah':capacity,'voltage':voltage})
    assert params['BATT_CAPACITY'] == capacity * 1000
    assert params['SIM_BATT_VOLTAGE'] == voltage
    assert params['BATT_MONITOR'] == 4
    assert 'BATT_CAPACITY' in (tmp_path / 'pack.parm').read_text()


@pytest.mark.parametrize('capacity,voltage', [(0,25.2),(-1,25.2),(10,float('nan')),(float('inf'),25.2)])
def test_invalid_pack_fails_before_launch(capacity, voltage):
    with pytest.raises(ValueError):
        module('battery_profile').battery_parameters({'capacity_ah':capacity,'voltage':voltage})


def test_generated_battery_topic_uses_model_name():
    model = ET.fromstring('<model name="other_aircraft"><link name="base_link"/><plugin name="ArduPilotPlugin"/></model>')
    world = ET.Element('world')
    module('sensor_profiles').configure(model, world)
    topic = model.find("plugin[@name='ArduPilotPlugin']/battery_state_topic").text
    assert topic == '/model/other_aircraft/battery/flight_battery/state'
    battery = model.find("plugin[@name='gz::sim::systems::LinearBatteryPlugin']")
    params = module('battery_profile').battery_parameters({
        'capacity_ah':float(battery.find('capacity').text),
        'voltage':float(battery.find('voltage').text)})
    assert params['BATT_CAPACITY'] == float(battery.find('capacity').text) * 1000


def test_motor_reference_uses_mass_coefficients_and_motor_count():
    import math
    model = ET.fromstring('<model name="aircraft"><link name="body"><inertial><mass>8</mass></inertial></link></model>')
    for i in range(6):
        p = ET.SubElement(model,'plugin',name='gz::sim::systems::MulticopterMotorModel')
        for key,value in {'jointName':f'rotor{i}','motorConstant':.00002,'rotorVelocitySlowdownSim':7}.items():
            ET.SubElement(p,key).text=str(value)
    assert module('battery_profile').configure_motor_load(model, {'idle_load_w':30,'load_w':900})
    plugin=model.find("plugin[@name='icarus::BatteryLoad']")
    assert float(plugin.findtext('reference_speed_rad_s')) == pytest.approx(math.sqrt(8*9.80665/(6*.00002)))
    assert len(plugin.findall('rotor')) == 6
    assert {r.findtext('slowdown') for r in plugin.findall('rotor')} == {'7'}
    module('battery_profile').configure_motor_load(model, {'idle_load_w':30,'load_w':950})
    assert len(model.findall("plugin[@name='icarus::BatteryLoad']")) == 1


def test_motor_calibration_rejects_negative_idle_or_hover_below_idle():
    model=ET.fromstring('<model><link><inertial><mass>1</mass></inertial></link><plugin name="gz::sim::systems::MulticopterMotorModel"><motorConstant>0.001</motorConstant><jointName>rotor</jointName></plugin></model>')
    for idle,hover in [(-1,10),(50,40),(0,float('nan'))]:
        with pytest.raises(ValueError):
            module('battery_profile').configure_motor_load(model, {'idle_load_w':idle,'load_w':hover})


def test_cpp_rotor_power_model(tmp_path):
    import subprocess
    binary=tmp_path/'battery-load-tests'
    subprocess.run(['c++','-std=c++17',str(ROOT/'tests/unit/cpp/battery_load_model_tests.cpp'),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)


def test_partial_soc_requires_ack_then_verified_telemetry_and_survives_restart(tmp_path):
    cls=module('battery_profile').BatteryStartupSync
    marker=tmp_path/'battery_initialization.json'; sync=cls(marker)
    sync.observe(3.5,10,22.4,2,1,10)
    assert sync.request(10,False,True) == 35
    assert not sync.verify(35,10.1)
    sync.acknowledge(True,10.2)
    assert not sync.verify(100,10.3)
    assert sync.verify(35,10.4)
    assert sync.request(11,False,True) is None
    restarted=cls(marker)
    assert restarted.initialized
    assert restarted.request(15,True,True) is None
    assert restarted.record['physical_recharge'] is False


def test_soc_initialization_never_arms_resets_stale_or_rejects_missing_ack(tmp_path):
    cls=module('battery_profile').BatteryStartupSync
    sync=cls(tmp_path/'soc.json');sync.observe(0,10,0,0,0,0)
    assert sync.request(3,False,True) is None
    with pytest.raises(RuntimeError,match='disarmed'):
        sync.request(0,True,True)
    assert sync.request(0,False,True) == 0
    with pytest.raises(RuntimeError,match='rejected'):
        sync.acknowledge(False,.2)
    with pytest.raises(ValueError):sync.observe(11,10,25,0,0,0)


def test_soc_initialization_retry_is_bounded_and_not_periodic(tmp_path):
    sync=module('battery_profile').BatteryStartupSync(tmp_path/'soc.json')
    for i in range(10):
        sync.observe(10,10,25.2,0,i,i)
        assert sync.request(i,False,True) == 100
        assert sync.request(i+.1,False,True) is None
    sync.observe(10,10,25.2,0,10,10)
    with pytest.raises(RuntimeError,match='acknowledged'):sync.request(10,False,True)
