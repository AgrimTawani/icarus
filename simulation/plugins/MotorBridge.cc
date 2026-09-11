// Gazebo-only actuator adapter. ArduPilot sends normalized per-channel Double
// commands; MulticopterMotorModel consumes a physical-speed Actuators vector.
#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actuators.hh>
#include <gz/transport/Node.hh>

namespace icarus {
class MotorBridge final : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    model = entity;
    const auto config = sdf->Clone();
    const auto prefix = config->Get<std::string>("topic_prefix");
    coefficient = config->Get<double>("thrust_coefficient");
    if (!(coefficient > 0)) throw std::runtime_error("Invalid thrust coefficient");
    // Versioned provisional throttle/thrust samples from the frozen BOM.
    // Includes origin; no extrapolation above the 70% operational ceiling.
    for (std::size_t i = 0; i < 4; ++i) {
      std::function<void(const gz::msgs::Double &)> cb =
          [this,i](const gz::msgs::Double &msg) {
            std::lock_guard<std::mutex> lock(mutex);
            commands[i] = std::isfinite(msg.data()) ? std::clamp(msg.data(),0.0,0.7) : 0.0;
            received[i] = true;
          };
      if (!node.Subscribe(prefix + "/" + std::to_string(i), cb))
        throw std::runtime_error("Cannot subscribe to motor channel");
    }
    ecm.CreateComponent(model, gz::sim::components::Actuators(gz::msgs::Actuators{}));
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    std::lock_guard<std::mutex> lock(mutex);
    const double dt = std::chrono::duration<double>(info.dt).count();
    gz::msgs::Actuators output;
    for (std::size_t i = 0; i < 4; ++i) {
      age[i] = received[i] ? 0.0 : age[i] + dt;
      received[i] = false;
      const double command = age[i] <= 0.25 ? commands[i] : 0.0;
      constexpr std::array<double,5> throttle{0.0,0.4,0.5,0.6,0.7};
      constexpr std::array<double,5> kgf{0.0,0.761,1.336,1.871,2.451};
      double thrust = 0;
      for (std::size_t j = 1; j < throttle.size(); ++j) {
        if (command <= throttle[j]) {
          const double t = (command-throttle[j-1])/(throttle[j]-throttle[j-1]);
          thrust = (kgf[j-1]+t*(kgf[j]-kgf[j-1]))*9.80665;
          break;
        }
      }
      output.add_velocity(std::sqrt(thrust/coefficient));
    }
    ecm.Component<gz::sim::components::Actuators>(model)->Data() = output;
  }
 private:
  gz::sim::Entity model{gz::sim::kNullEntity};
  double coefficient{};
  gz::transport::Node node;
  std::mutex mutex;
  std::array<double,4> commands{}, age{};
  std::array<bool,4> received{};
};
}
GZ_ADD_PLUGIN(icarus::MotorBridge, gz::sim::System,
              icarus::MotorBridge::ISystemConfigure, icarus::MotorBridge::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(icarus::MotorBridge, "icarus::MotorBridge")
