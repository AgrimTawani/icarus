#pragma once
#include <cmath>
#include <stdexcept>
#include <vector>

namespace icarus {
// Provisional fixed-pitch rotor model: shaft power scales with angular speed^3.
// Missing RPM conservatively uses hover draw; it never means free energy.
inline double RotorBatteryPower(double idle, double hover, double reference,
                                const std::vector<double> &physicalRpm) {
  if (!std::isfinite(idle) || idle < 0 || !std::isfinite(hover) || hover < idle ||
      !std::isfinite(reference) || reference <= 0)
    throw std::invalid_argument("invalid battery load calibration");
  if (physicalRpm.empty()) return hover;
  double ratio = 0;
  for (double speed : physicalRpm) {
    if (!std::isfinite(speed)) return hover;
    const double r = std::abs(speed) / reference;
    ratio += r*r*r;
  }
  const double result = idle + (hover-idle)*ratio/physicalRpm.size();
  return std::isfinite(result) ? result : hover;
}
}
