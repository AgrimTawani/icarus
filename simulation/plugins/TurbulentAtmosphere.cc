// Model-level low-altitude atmosphere: stochastic turbulence, shear, wakes,
// aerodynamic force and rotational buffeting. Seeded for replayability.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace icarus {
class TurbulentAtmosphere final : public gz::sim::System,
                                  public gz::sim::ISystemConfigure,
                                  public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    const gz::sim::Model model(entity);
    link = gz::sim::Link(model.CanonicalLink(ecm));
    if (!link.Valid(ecm)) throw std::runtime_error("Atmosphere requires a canonical link");
    link.EnableVelocityChecks(ecm);
    const auto config = sdf->Clone();
    seed = config->Get<std::uint64_t>("seed", 1).first;
    generator.seed(seed);
    meanSpeed = config->Get<double>("mean_speed", 0.0).first;
    const double directionDeg = config->Get<double>("direction_deg", 0.0).first;
    const double direction = directionDeg * std::acos(-1.0) / 180.0;
    meanDirection.Set(std::cos(direction), std::sin(direction), 0);
    intensity = config->Get<double>("turbulence_intensity", 0.20).first;
    riseTime = std::max(0.1, config->Get<double>("rise_time", 3.0).first);
    referenceHeight = std::max(0.1, config->Get<double>("reference_height", 3.0).first);
    shearExponent = config->Get<double>("shear_exponent", 0.22).first;
    airDensity = config->Get<double>("air_density", 1.16).first;
    areas = config->Get<gz::math::Vector3d>(
        "projected_area", gz::math::Vector3d(0.12, 0.12, 0.20)).first;
    drag = config->Get<gz::math::Vector3d>(
        "drag_coefficient", gz::math::Vector3d(1.05, 1.05, 1.20)).first;
    rotationalDrag = config->Get<gz::math::Vector3d>(
        "rotational_drag", gz::math::Vector3d(0.025, 0.025, 0.035)).first;
    centerOfPressure = config->Get<gz::math::Vector3d>(
        "center_of_pressure", gz::math::Vector3d(0, 0, -0.025)).first;
    buffetingMoment = config->Get<double>("buffeting_moment", 0.055).first;
    if (meanSpeed < 0 || intensity < 0 || airDensity <= 0 ||
        areas.Min() <= 0 || drag.Min() <= 0) {
      throw std::runtime_error("Invalid atmosphere coefficients");
    }
    if (config->HasElement("wake")) {
      auto item = config->GetElement("wake");
      while (item) {
        wakes.push_back({
            item->Get<gz::math::Vector3d>("center"),
            item->Get<gz::math::Vector3d>("size")});
        item = item->GetNextElement("wake");
      }
    }
    publisher = node.Advertise<gz::msgs::Vector3d>("/icarus/environment/wind");
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || info.dt <= std::chrono::steady_clock::duration::zero()) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    accumulator += dt;
    constexpr double updateStep = 0.02;
    while (accumulator >= updateStep) {
      UpdateStochasticState(updateStep);
      accumulator -= updateStep;
    }
    const auto pose = link.WorldPose(ecm);
    const auto velocity = link.WorldLinearVelocity(ecm);
    const auto angularVelocity = link.WorldAngularVelocity(ecm);
    if (!pose || !velocity || !angularVelocity) return;
    const double elapsed = std::chrono::duration<double>(info.simTime).count();
    const double ramp = 1.0 - std::exp(-elapsed / riseTime);
    const double height = std::max(0.20, pose->Pos().Z());
    const double shear = std::pow(height / referenceHeight, shearExponent);
    double wakeScale = 0;
    double wakeDeficit = 0;
    const gz::math::Vector3d lateral(-meanDirection.Y(), meanDirection.X(), 0);
    for (const auto &wake : wakes) {
      const gz::math::Vector3d relative = pose->Pos() - wake.center;
      const double downstream = relative.Dot(meanDirection);
      const double width = std::max(wake.size.X(), wake.size.Y());
      const double spread = 0.5 * width + 0.18 * std::max(0.0, downstream);
      const double crosswind = std::abs(relative.Dot(lateral));
      if (downstream > 0 && downstream < 8 * width && crosswind < spread &&
          pose->Pos().Z() < wake.center.Z() + 0.7 * wake.size.Z()) {
        const double local = std::exp(-downstream / (3 * width)) *
            (1.0 - crosswind / spread);
        wakeScale = std::max(wakeScale, local);
        wakeDeficit = std::max(wakeDeficit, 0.55 * local);
      }
    }
    const gz::math::Vector3d stochastic = turbulence * (1.0 + 2.2 * wakeScale);
    const gz::math::Vector3d wind =
        meanDirection * (meanSpeed * ramp * shear * (1.0 - wakeDeficit) + gust) +
        stochastic;
    const gz::math::Vector3d relativeWorld = wind - *velocity;
    const gz::math::Vector3d relativeBody =
        pose->Rot().Inverse().RotateVector(relativeWorld);
    gz::math::Vector3d forceBody;
    for (int axis = 0; axis < 3; ++axis) {
      const double value = relativeBody[axis];
      forceBody[axis] = 0.5 * airDensity * areas[axis] * drag[axis] *
          value * std::abs(value);
    }
    const gz::math::Vector3d angularBody =
        pose->Rot().Inverse().RotateVector(*angularVelocity);
    gz::math::Vector3d torqueBody = centerOfPressure.Cross(forceBody);
    for (int axis = 0; axis < 3; ++axis) {
      torqueBody[axis] -= rotationalDrag[axis] * angularBody[axis] *
          std::abs(angularBody[axis]);
    }
    torqueBody += buffetingMoment * intensity * std::max(meanSpeed, 0.5) *
        gz::math::Vector3d(stochastic.Y(), -stochastic.X(), 0.35 * stochastic.Z());
    link.AddWorldWrench(
        ecm,
        pose->Rot().RotateVector(forceBody),
        pose->Rot().RotateVector(torqueBody));
    publishAccumulator += dt;
    if (publishAccumulator >= 0.05) {
      gz::msgs::Vector3d message;
      message.set_x(wind.X());
      message.set_y(wind.Y());
      message.set_z(wind.Z());
      const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
          info.simTime).count();
      message.mutable_header()->mutable_stamp()->set_sec(nanoseconds / 1000000000LL);
      message.mutable_header()->mutable_stamp()->set_nsec(nanoseconds % 1000000000LL);
      publisher.Publish(message);
      publishAccumulator = 0;
    }
  }

 private:
  struct Wake {
    gz::math::Vector3d center;
    gz::math::Vector3d size;
  };

  void UpdateStochasticState(double dt) {
    const gz::math::Vector3d tau(2.4, 1.8, 0.9);
    const gz::math::Vector3d sigma(
        intensity * meanSpeed,
        0.80 * intensity * meanSpeed,
        0.55 * intensity * meanSpeed);
    for (int axis = 0; axis < 3; ++axis) {
      const double decay = std::exp(-dt / tau[axis]);
      turbulence[axis] = decay * turbulence[axis] +
          sigma[axis] * std::sqrt(1.0 - decay * decay) * normal(generator);
    }
    constexpr double gustTau = 9.0;
    const double decay = std::exp(-dt / gustTau);
    gust = decay * gust + 0.45 * intensity * meanSpeed *
        std::sqrt(1.0 - decay * decay) * normal(generator);
  }

  gz::sim::Link link{gz::sim::kNullEntity};
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  std::uint64_t seed{1};
  std::mt19937_64 generator;
  std::normal_distribution<double> normal{0.0, 1.0};
  double meanSpeed{0};
  double intensity{0.2};
  double riseTime{3};
  double referenceHeight{3};
  double shearExponent{0.22};
  double airDensity{1.16};
  double buffetingMoment{0.055};
  double gust{0};
  double accumulator{0};
  double publishAccumulator{0};
  gz::math::Vector3d meanDirection{1, 0, 0};
  gz::math::Vector3d turbulence{0, 0, 0};
  gz::math::Vector3d areas{0.12, 0.12, 0.20};
  gz::math::Vector3d drag{1.05, 1.05, 1.20};
  gz::math::Vector3d rotationalDrag{0.025, 0.025, 0.035};
  gz::math::Vector3d centerOfPressure{0, 0, -0.025};
  std::vector<Wake> wakes;
};
}

GZ_ADD_PLUGIN(icarus::TurbulentAtmosphere, gz::sim::System,
              icarus::TurbulentAtmosphere::ISystemConfigure,
              icarus::TurbulentAtmosphere::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(icarus::TurbulentAtmosphere, "icarus::TurbulentAtmosphere")
