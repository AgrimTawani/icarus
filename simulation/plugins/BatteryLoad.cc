// Gazebo-only power consumer: one physical battery, actual rotor feedback.
#include "BatteryLoadModel.hh"
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/BatteryPowerLoad.hh>
#include <gz/sim/components/BatterySoC.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>

namespace icarus {
class BatteryLoad final : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    model = entity;
    const auto config = sdf->Clone();
    batteryName = config->Get<std::string>("battery_name");
    idle = config->Get<double>("idle_power_w");
    hover = config->Get<double>("hover_power_w");
    reference = config->Get<double>("reference_speed_rad_s");
    RotorBatteryPower(idle, hover, reference, {}); // validate calibration
    auto rotor = config->GetElement("rotor");
    while (rotor) {
      const double slowdown = rotor->Get<double>("slowdown");
      if (!std::isfinite(slowdown) || slowdown <= 0)
        throw std::invalid_argument("invalid rotor simulation slowdown");
      names.push_back(rotor->Get<std::string>("joint"));
      slowdowns.push_back(slowdown);
      rotor = rotor->GetNextElement("rotor");
    }
    if (names.empty()) throw std::invalid_argument("battery load requires rotors");
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    if (battery == gz::sim::kNullEntity) {
      battery = ecm.EntityByComponents(gz::sim::components::Name(batteryName),
                                       gz::sim::components::ParentEntity(model));
      if (battery == gz::sim::kNullEntity) return;
    }
    std::vector<double> speeds;
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto entity = gz::sim::Model(model).JointByName(ecm, names[i]);
      if (entity == gz::sim::kNullEntity) { speeds.clear(); break; }
      gz::sim::Joint joint(entity);
      joint.EnableVelocityCheck(ecm);
      const auto velocity = joint.Velocity(ecm);
      if (!velocity || velocity->empty()) { speeds.clear(); break; }
      speeds.push_back(velocity->front() * slowdowns[i]);
    }
    const double power = RotorBatteryPower(idle, hover, reference, speeds);
    auto component = ecm.Component<gz::sim::components::BatteryPowerLoad>(model);
    if (component) component->Data() = {battery, power};
    else ecm.CreateComponent(model, gz::sim::components::BatteryPowerLoad({battery, power}));
  }
 private:
  gz::sim::Entity model{gz::sim::kNullEntity}, battery{gz::sim::kNullEntity};
  std::string batteryName;
  double idle{}, hover{}, reference{};
  std::vector<std::string> names;
  std::vector<double> slowdowns;
};
}
GZ_ADD_PLUGIN(icarus::BatteryLoad, gz::sim::System,
              icarus::BatteryLoad::ISystemConfigure, icarus::BatteryLoad::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(icarus::BatteryLoad, "icarus::BatteryLoad")
