"""One battery specification for physical Gazebo power and SITL telemetry."""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def battery_parameters(spec):
    """Configure the JSON backend's analog battery monitor from physical units.

    Gazebo is the voltage/current authority. ArduPilot integrates that current
    against the same capacity; it must not use an unrelated default 3S pack.
    """
    capacity = float(spec['capacity_ah'])
    voltage = float(spec['voltage'])
    if not all(math.isfinite(v) and v > 0 for v in (capacity, voltage)):
        raise ValueError('battery capacity and voltage must be finite and positive')
    return {'BATT_MONITOR': 4, 'BATT_CAPACITY': capacity * 1000,
            'SIM_BATT_VOLTAGE': voltage}


def write_battery_parameters(path, spec=None):
    if spec is None:
        spec = json.loads((ROOT / 'config/simulation/sensor_profiles.json').read_text())['battery']
    params = battery_parameters(spec)
    Path(path).write_text('# Simulation only: Gazebo authoritative voltage/current.\n' +
                         ''.join(f'{name} {value:g}\n' for name, value in params.items()))
    return params


def configure_motor_load(model, spec):
    """Derive hover reference from this model's mass and motor constants.

    Electrical hover and idle watts are explicit provisional calibration values,
    not an endurance claim. Gazebo remains the sole charge integrator.
    """
    import xml.etree.ElementTree as ET
    motors = model.findall("plugin[@name='gz::sim::systems::MulticopterMotorModel']")
    if not motors:
        return False
    mass = sum(float(link.findtext('inertial/mass', '0')) for link in model.findall('link'))
    coefficients = [float(m.findtext('motorConstant')) for m in motors]
    idle, hover = float(spec['idle_load_w']), float(spec['load_w'])
    if not math.isfinite(mass) or mass <= 0 or any(not math.isfinite(k) or k <= 0 for k in coefficients):
        raise ValueError('invalid aircraft mass or motor thrust coefficient')
    if not math.isfinite(idle) or not math.isfinite(hover) or not 0 <= idle <= hover:
        raise ValueError('battery idle/hover load calibration is invalid')
    reference = math.sqrt(mass * 9.80665 / sum(coefficients))
    for old in model.findall("plugin[@name='icarus::BatteryLoad']"):
        model.remove(old)
    plugin = ET.SubElement(model, 'plugin', filename='IcarusBatteryLoad', name='icarus::BatteryLoad')
    for name, value in {'battery_name':'flight_battery', 'idle_power_w':idle,
                        'hover_power_w':hover, 'reference_speed_rad_s':reference}.items():
        ET.SubElement(plugin, name).text = str(value)
    for motor in motors:
        rotor = ET.SubElement(plugin, 'rotor')
        ET.SubElement(rotor, 'joint').text = motor.findtext('jointName')
        ET.SubElement(rotor, 'slowdown').text = motor.findtext('rotorVelocitySlowdownSim', '1')
    return True


class BatteryStartupSync:
    """Align AP's coulomb counter once per new simulation session, disarmed.

    This does not recharge Gazebo. The marker belongs to the session directory,
    so restarting the API or this bridge never reinitializes a running battery.
    """
    def __init__(self, marker):
        self.marker = Path(marker)
        self.record = json.loads(self.marker.read_text()) if self.marker.exists() else None
        self.initialized = bool(self.record and self.record.get('verified'))
        self.sample = None
        self.sent = 0
        self.next_send = 0.0
        self.acknowledged_at = None

    def observe(self, charge, capacity, voltage, current, stamp, received):
        values = (charge, capacity, voltage, current, stamp, received)
        if not all(math.isfinite(v) for v in values) or capacity <= 0 or not 0 <= charge <= capacity or voltage < 0:
            raise ValueError('invalid authoritative battery startup sample')
        self.sample = {"charge_ah": charge, "capacity_ah": capacity, "voltage_v": voltage,
                           "current_a": current, "simulation_s": stamp, "received_monotonic_s": received,
                           "percent": charge / capacity * 100}

    def request(self, now, armed, heartbeat_fresh):
        if self.initialized:
            return None
        if armed:
            raise RuntimeError('battery counter initialization requires a disarmed simulator')
        if not heartbeat_fresh or not self.sample or not 0 <= now-self.sample['received_monotonic_s'] <= 2:
            return None
        if self.acknowledged_at is not None or now < self.next_send:
            return None
        if self.sent >= 10:
            raise RuntimeError('battery counter initialization was not acknowledged')
        self.sent += 1
        self.next_send = now + 1
        self.record = dict(self.sample, sent_monotonic_s=now, attempt=self.sent,
                           source='Gazebo flight_battery charge/capacity',
                           command='MAV_CMD_BATTERY_RESET', physical_recharge=False,
                           disarmed_at_initialization=True, verified=False)
        return self.sample['percent']

    def acknowledge(self, accepted, now):
        if self.initialized or not self.sent:
            return
        if not accepted:
            raise RuntimeError('autopilot rejected battery counter initialization')
        self.acknowledged_at = now
        self.record['acknowledged_monotonic_s'] = now

    def verify(self, remaining, received):
        if self.initialized or self.acknowledged_at is None or received < self.acknowledged_at:
            return self.initialized
        if not self.sample or not 0 <= received-self.sample['received_monotonic_s'] <= 2:
            return False
        if not math.isfinite(remaining) or not 0 <= remaining <= 100:
            return False
        # MAVLink SYS_STATUS percentage is an integer, and samples are asynchronous.
        if abs(remaining-self.sample['percent']) > 1.1:
            return False
        self.record.update(verified=True, telemetry_percent=remaining,
                           verified_monotonic_s=received, measured_percent=self.sample['percent'])
        temporary = self.marker.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.record, indent=2)+'\n')
        temporary.replace(self.marker)
        self.initialized = True
        return True
