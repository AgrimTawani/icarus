#include "vehicles/multirotor/firmwares/arducopter/ArduCopterParams.hpp"

#include <cmath>
#include <iostream>
#include <memory>

using msr::airlib::AirSimSettings;
using msr::airlib::ArduCopterParams;
using msr::airlib::SensorFactory;

namespace
{
bool near(float actual, float expected, float tolerance = 1.0e-6f)
{
    return std::fabs(actual - expected) <= tolerance;
}
}

int main()
{
    AirSimSettings::MavLinkVehicleSetting setting;
    setting.vehicle_type = AirSimSettings::kVehicleTypeArduCopter;
    setting.connection_info.model = "Icarus7";
    auto sensors = std::make_shared<SensorFactory>();
    ArduCopterParams vehicle(setting, sensors);
    vehicle.initialize(&setting);
    const auto& params = vehicle.getParams();

    if (params.rotor_count != 4 || !near(params.mass, 4.343f)
        || !near(params.inertia(0, 0), 0.0298044346f)
        || !near(params.inertia(2, 2), 0.0415397287f)
        || !near(params.rotor_params.max_rpm, 15606.0f, 0.01f)
        || !near(params.rotor_poses.at(0).position.x(), 0.1384983492f)
        || !near(params.rotor_poses.at(0).position.y(), 0.1733868713f)
        || !near(params.rotor_poses.at(0).position.z(), -0.0590099010f)) {
        std::cerr << "Icarus7 AirLib parameters do not match the frozen contract\n";
        return 1;
    }
    std::cout << "PASS: AirLib instantiated Icarus7 mass, inertia, propulsion and rotor geometry\n";
    return 0;
}
